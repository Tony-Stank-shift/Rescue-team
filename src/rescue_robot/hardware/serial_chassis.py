"""
serial_chassis.py —— 上位机 ↔ 下位机(STM32) 串口底盘驱动

上位机（电脑/RDK）通过 TTL 串口与下位机（STM32F407 底盘板）通信。
协议依据 chassis_serial_protocol.md（v1）。

串口约定：
  - 波特率 115200，8 数据位，无校验，1 停止位，无流控，ASCII，逗号分隔，\\r\\n 结尾。
  - 上行（上位机 → 下位机）：
      · PING                      → 连通测试，下位机回复 PONG
      · START                     → 启动，下位机清零局部里程计，回复 ACK,START
      · VEL,v_mm_s,w_mrad_s       → 速度指令（v 整数 mm/s，w 整数 mrad/s）
      · STOP                      → 普通停车（清 PI 积分，不清里程计），回复 ACK,STOP
      · ESTOP                     → 紧急停车锁定，回复 ACK,ESTOP
  - 下行（下位机 → 上位机）：
      · ODOM,x_m,y_m,theta_rad,encL,encR,vL_m_s,vR_m_s   —— 轮式里程计（20Hz，严格 8 字段）
      · EVENT,xxx / ERR,xxx / ACK,xxx / TEL,xxx          —— 只记录，不得误解析为位姿

坐标系（下位机局部）：x 前、y 左、theta 逆时针为正，启动朝向 0。
里程计、地图坐标转换、出发区全局偏移由上位机负责（见 chassis_interface.py）。

设备文件：
  - 电脑调试（USB-TTL）：/dev/ttyUSB0
  - RDK 部署：/dev/ttyS0（待确认具体 UART）
"""

import logging
import time
from typing import Optional, Tuple

from .chassis_interface import ChassisInterface

logger = logging.getLogger("serial_chassis")


class SerialChassis:
    """
    串口底盘驱动。

    用法（autonomous loop 每帧）：
      chassis = SerialChassis(port='/dev/ttyUSB0')
      chassis.open()
      chassis.start_match()                        # PING → START
      ...
      chassis.send_velocity(v_mm_s, w_rad_s)       # 下发速度
      pose = chassis.read_pose()                   # 读位姿（上层 mm 坐标），无数据返回 None
    """

    def __init__(self,
                 port: str = '/dev/ttyUSB0',
                 baudrate: int = 115200,
                 timeout: float = 0.02,
                 start_x_mm: float = 150.0,
                 start_y_mm: float = 150.0,
                 start_theta_rad: float = 1.5707963267948966):
        self._port = port
        self._baudrate = baudrate
        self._timeout = timeout
        self._ser = None
        self._chassis = ChassisInterface(start_x_mm, start_y_mm, start_theta_rad)

        # 统计
        self._bytes_tx = 0
        self._frames_rx = 0

    # ---- 生命周期 ----

    def open(self) -> bool:
        """打开串口。pyserial 延迟导入，避免无串口环境启动失败。"""
        try:
            import serial
        except ImportError:
            logger.error("pyserial 未安装，串口不可用：pip install pyserial")
            return False
        try:
            self._ser = serial.Serial(self._port, self._baudrate, timeout=self._timeout)
        except Exception as e:
            logger.error(f"打开串口失败 ({self._port} @ {self._baudrate}): {e}")
            return False
        logger.info(f"串口已打开: {self._port} @ {self._baudrate}")
        return True

    def close(self) -> None:
        if self._ser is not None:
            try:
                self._ser.close()
            except Exception:
                pass
            self._ser = None
            logger.info("串口已关闭")

    @property
    def is_open(self) -> bool:
        return self._ser is not None and self._ser.is_open

    def set_start_pose(self, x_mm: float, y_mm: float, theta_rad: float) -> None:
        """设置初始位姿（出发区对齐）。"""
        self._chassis.set_start_pose(x_mm, y_mm, theta_rad)

    # ---- 发送（上行） ----

    def send_ping(self) -> bool:
        """连通测试：发 PING，下位机回复 PONG。"""
        return self._send("PING")

    def send_start(self) -> bool:
        """启动命令：下位机收到后清零局部里程计，回复 ACK,START。"""
        return self._send("START")

    def send_velocity(self, v_mm_s: float, w_rad_s: float) -> bool:
        """发送速度指令 VEL,v_mm_s,w_mrad_s。"""
        cmd = ChassisInterface.velocity_to_command(v_mm_s, w_rad_s)
        return self._send(cmd)

    def send_stop(self) -> bool:
        """普通停车（清 PI 积分，不清里程计），回复 ACK,STOP。"""
        return self._send("STOP")

    def send_estop(self) -> bool:
        """紧急停车并锁定，回复 ACK,ESTOP。"""
        return self._send("ESTOP")

    def _send(self, data: str) -> bool:
        if not self.is_open:
            logger.warning("串口未打开，无法发送")
            return False
        payload = (data + "\r\n").encode('ascii')
        self._ser.write(payload)
        self._bytes_tx += len(payload)
        return True

    # ---- 启动流程 ----

    def start_match(self, timeout: float = 0.5) -> bool:
        """
        启动顺序（协议第 8 节）：PING → 确认 PONG；START → 确认 ACK,START。

        Args:
            timeout: 等待每个 ACK 的超时秒数。

        Returns:
            连接 + 启动都成功返回 True，否则 False。
        """
        if not self.is_open:
            logger.warning("串口未打开，无法启动")
            return False
        self.send_ping()
        if not self.wait_for("PONG", timeout):
            logger.warning("PING 未收到 PONG，连通性检查失败")
            return False
        self.send_start()
        if not self.wait_for("ACK,START", timeout):
            logger.warning("START 未收到 ACK,START")
            return False
        logger.info("底盘启动完成（PONG + ACK,START）")
        return True

    # ---- 接收（下行） ----

    def _read_line(self) -> Optional[str]:
        """读一行原始文本（strip）；无数据/未打开返回 None。"""
        if not self.is_open:
            return None
        try:
            line = self._ser.readline()
        except Exception as e:
            logger.warning(f"串口读取异常: {e}")
            return None
        if not line:
            return None
        text = line.decode('ascii', errors='ignore').strip()
        return text if text else None

    def read_frame(self) -> Optional[dict]:
        """读一行并仅解析合法的 ODOM 帧；其他行（ACK/EVENT/ERR/TEL 等）返回 None。"""
        text = self._read_line()
        if not text:
            return None
        return self.parse_frame(text)

    def read_pose(self) -> Optional[Tuple[float, float, float]]:
        """
        读一帧并转换为上层位姿 (x_mm, y_mm, theta_rad)；无数据返回 None。

        theta 已转换到上层约定（从 +X 逆时针，前方 +Y = pi/2）。
        """
        frame = self.read_frame()
        if frame is None:
            return None
        self._chassis.update_raw(
            (frame.get('encL', 0), frame.get('encR', 0)),
            (frame.get('vL', 0.0), frame.get('vR', 0.0)),
        )
        return self._chassis.odom_to_upper(
            frame['x_m'], frame['y_m'], frame['theta_rad'],
        )

    def wait_for(self, prefix: str, timeout: float = 0.5) -> Optional[str]:
        """
        循环读行直到出现以 prefix 开头的行（大小写不敏感），返回该行；超时返回 None。

        用于确认 PONG / ACK,START 等回复。
        """
        deadline = time.time() + timeout
        while time.time() < deadline:
            text = self._read_line()
            if text and text.upper().startswith(prefix.upper()):
                return text
        return None

    def parse_frame(self, text: str) -> Optional[dict]:
        """
        严格解析 ODOM 里程计帧。

        仅接受：前缀严格等于 ODOM、恰好 8 个字段、数值全合法。
        任何其他行（ACK/EVENT/ERR/TEL/乱码/简化帧）一律返回 None。
        """
        parts = [p for p in text.replace(' ', '').split(',') if p]
        if len(parts) != 8:
            return None
        if parts[0].upper() != 'ODOM':
            return None
        try:
            x_f = float(parts[1])
            y_f = float(parts[2])
            theta_f = float(parts[3])
            encL = int(parts[4])
            encR = int(parts[5])
            vL = float(parts[6])
            vR = float(parts[7])
        except ValueError:
            return None

        frame = {
            'x_m': x_f, 'y_m': y_f, 'theta_rad': theta_f,
            'encL': encL, 'encR': encR, 'vL': vL, 'vR': vR,
        }
        self._frames_rx += 1
        return frame

    # ---- 查询 ----

    def get_stats(self) -> dict:
        return {
            "port": self._port,
            "baudrate": self._baudrate,
            "is_open": self.is_open,
            "bytes_tx": self._bytes_tx,
            "frames_rx": self._frames_rx,
            "encoder_counts": self._chassis.encoder_counts,
            "wheel_speeds": self._chassis.wheel_speeds,
        }


# ============================================================
# 独立测试（解析逻辑，不依赖真实串口）
# ============================================================

if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO, format="%(message)s")

    print("=" * 60)
    print("  串口底盘驱动 — 帧解析测试（严格 ODOM）")
    print("=" * 60)

    sc = SerialChassis(port='/dev/ttyUSB0')

    # --- 测试 1：合法 8 字段 ODOM 帧 ---
    frame = sc.parse_frame("ODOM,1.000,0.000,0.000,442,440,0.50,0.48")
    print(f"\n合法帧: {frame}")
    assert frame and abs(frame['x_m'] - 1.0) < 1e-9
    assert frame['encL'] == 442 and frame['encR'] == 440
    assert abs(frame['vL'] - 0.5) < 1e-9 and abs(frame['vR'] - 0.48) < 1e-9
    print("  ✅ 通过")

    # --- 测试 2：简化帧（非 8 字段）应拒绝 ---
    assert sc.parse_frame("0.0,1.0,1.5708") is None
    assert sc.parse_frame("ODOM,1.0,0.0,0.0") is None  # 4 字段
    print("简化帧/字段不足: 均返回 None ✅")

    # --- 测试 3：非 ODOM 前缀应拒绝 ---
    for bad in ["ACK,START", "EVENT,WATCHDOG_STOP", "TEL,12,34", "ERR,FORMAT", "PONG", "garbage", ""]:
        assert sc.parse_frame(bad) is None, f"应拒绝: {bad}"
    print("ACK/EVENT/TEL/ERR/乱码: 均返回 None ✅")

    # --- 测试 4：数值非法应拒绝 ---
    assert sc.parse_frame("ODOM,abc,0,0,1,2,3,4") is None
    assert sc.parse_frame("ODOM,1,0,0,xx,2,3,4") is None
    print("数值非法: 返回 None ✅")

    # --- 测试 5：带空格/多余分隔应清理 ---
    frame = sc.parse_frame("ODOM, 0.5, -0.5, 3.14, 100, 102, 0.3, 0.3")
    assert frame and abs(frame['x_m'] - 0.5) < 1e-9
    print("带空格 ODOM 帧: 解析成功 ✅")

    # --- 测试 6：read_pose 坐标转换 ---
    frame = sc.parse_frame("ODOM,1.0,0.0,0.0,1000,1000,0.4,0.4")  # 向前 1m
    x, y, theta = sc._chassis.odom_to_upper(frame['x_m'], frame['y_m'], frame['theta_rad'])
    print(f"\n向前 1m → 上层 ({x:.0f}, {y:.0f}), theta={__import__('math').degrees(theta):.0f}°")
    assert abs(x - 150) < 1 and abs(y - 1150) < 1
    print("  ✅ 通过")

    # --- 测试 7：命令格式化 + start_match（未打开串口应失败）---
    print(f"\n速度 (500mm/s,1.5rad/s) → {ChassisInterface.velocity_to_command(500.0, 1.5)}")
    assert ChassisInterface.velocity_to_command(500.0, 1.5) == "VEL,500,1500"
    assert ChassisInterface.start_command() == "START"
    assert sc.start_match() is False  # 未打开串口，启动应失败
    print("命令格式化 + 未打开串口启动失败 ✅")

    print(f"\n{'=' * 60}")
    print("  串口底盘驱动 — 全部测试通过 ✅")
    print(f"{'=' * 60}")
