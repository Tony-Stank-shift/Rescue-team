"""
serial_chassis.py —— 上位机 ↔ 下位机(STM32) 串口底盘驱动

上位机（电脑/RDK）通过 TTL 串口与下位机（STM32F407 底盘板）通信。

串口约定：
  - 波特率：115200
  - 上行（上位机 → 下位机）：每行一个命令，\\r\\n 结尾
      · VEL,v_mm_s,w_mrad_s   —— 速度指令（v 整数 mm/s，w 整数 mrad/s）
      · START                 —— 比赛启动（下位机清零轮式里程计）
  - 下行（下位机 → 上位机）：每行一帧，\\r\\n 结尾
      · ODOM,x_m,y_m,theta_rad,encL,encR,vL_m_s,vR_m_s
        或简化  x_m,y_m,theta_rad
        （帧格式待与下位机同学最终确认）

设备文件：
  - 电脑调试（USB-TTL）：/dev/ttyUSB0
  - RDK 部署：/dev/ttyS0（待确认具体 UART）
"""

import logging
from typing import Optional, Tuple

from .chassis_interface import ChassisInterface

logger = logging.getLogger("serial_chassis")


class SerialChassis:
    """
    串口底盘驱动。

    用法（autonomous loop 每帧）：
      chassis = SerialChassis(port='/dev/ttyUSB0')
      chassis.open()
      chassis.send_start()
      ...
      chassis.send_velocity(v_mm_s, w_rad_s)   # 下发速度
      pose = chassis.read_pose()               # 读位姿（上层 mm 坐标），无数据返回 None
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

    def send_start(self) -> bool:
        """发送启动命令，下位机收到后清零里程计。"""
        return self._send("START")

    def send_velocity(self, v_mm_s: float, w_rad_s: float) -> bool:
        """发送速度指令 VEL,v_mm_s,w_mrad_s。"""
        cmd = ChassisInterface.velocity_to_command(v_mm_s, w_rad_s)
        return self._send(cmd)

    def _send(self, data: str) -> bool:
        if not self.is_open:
            logger.warning("串口未打开，无法发送")
            return False
        payload = (data + "\r\n").encode('ascii')
        self._ser.write(payload)
        self._bytes_tx += len(payload)
        return True

    # ---- 接收（下行） ----

    def read_frame(self) -> Optional[dict]:
        """读一行下行帧，返回 dict；无数据/无效返回 None。"""
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
        if not text:
            return None
        return self.parse_frame(text)

    def parse_frame(self, text: str) -> Optional[dict]:
        """
        解析下行帧。

        支持格式：
          ODOM,x_m,y_m,theta_rad,encL,encR,vL,vR
          x_m,y_m,theta_rad            （简化）
          ODOM,x_m,y_m,theta_rad       （无编码器/轮速）
        """
        parts = [p for p in text.replace(' ', '').split(',') if p]
        if len(parts) < 3:
            return None

        idx = 0
        # 可选前缀（ODOM / POS 等）
        if not _is_number(parts[0]):
            idx = 1

        try:
            x_f = float(parts[idx])
            y_f = float(parts[idx + 1])
            theta_f = float(parts[idx + 2])
        except (IndexError, ValueError):
            return None

        frame = {'x_m': x_f, 'y_m': y_f, 'theta_rad': theta_f}

        # 可选：编码器计数、左右轮速
        try:
            frame['encL'] = int(parts[idx + 3])
            frame['encR'] = int(parts[idx + 4])
        except (IndexError, ValueError):
            pass
        try:
            frame['vL'] = float(parts[idx + 5])
            frame['vR'] = float(parts[idx + 6])
        except (IndexError, ValueError):
            pass

        self._frames_rx += 1
        return frame

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


def _is_number(s: str) -> bool:
    try:
        float(s)
        return True
    except ValueError:
        return False


# ============================================================
# 独立测试（帧解析，不依赖真实串口）
# ============================================================

if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO, format="%(message)s")

    print("=" * 60)
    print("  串口底盘驱动 — 帧解析测试")
    print("=" * 60)

    sc = SerialChassis(port='/dev/ttyUSB0')

    # --- 测试 1：完整 ODOM 帧 ---
    frame = sc.parse_frame("ODOM,1.000,0.000,0.000,442,440,0.50,0.48")
    print(f"\n完整帧: {frame}")
    assert frame and abs(frame['x_m'] - 1.0) < 1e-9
    assert frame['encL'] == 442 and frame['encR'] == 440
    assert abs(frame['vL'] - 0.5) < 1e-9
    print("  ✅ 通过")

    # --- 测试 2：简化帧（无编码器/轮速）---
    frame = sc.parse_frame("0.0,1.0,1.5708")
    print(f"简化帧: {frame}")
    assert frame and abs(frame['y_m'] - 1.0) < 1e-9
    assert 'encL' not in frame
    print("  ✅ 通过")

    # --- 测试 3：带空格/多余逗号 ---
    frame = sc.parse_frame("ODOM, 0.5, -0.5, 3.14, 100, 102")
    print(f"带空格帧: {frame}")
    assert frame and abs(frame['x_m'] - 0.5) < 1e-9
    print("  ✅ 通过")

    # --- 测试 4：无效帧 ---
    assert sc.parse_frame("garbage") is None
    assert sc.parse_frame("") is None
    assert sc.parse_frame("ODOM,abc,def") is None
    print(f"\n无效帧: 均返回 None")
    print("  ✅ 通过")

    # --- 测试 5：read_pose 坐标转换 ---
    # 从 read_pose 角度验证（用 parse_frame + odom_to_upper 的组合语义）
    frame = sc.parse_frame("ODOM,1.0,0.0,0.0")  # 向前 1m
    x, y, theta = sc._chassis.odom_to_upper(frame['x_m'], frame['y_m'], frame['theta_rad'])
    print(f"\n向前 1m → 上层 ({x:.0f}, {y:.0f}), theta={__import__('math').degrees(theta):.0f}°")
    assert abs(x - 150) < 1 and abs(y - 1150) < 1
    print("  ✅ 通过")

    # --- 测试 6：命令格式化（复用 ChassisInterface）---
    print(f"\n速度 (500mm/s, 1.5rad/s) → {ChassisInterface.velocity_to_command(500.0, 1.5)}")
    print(f"启动命令 → {ChassisInterface.start_command()}")
    print("  ✅ 通过")

    print(f"\n{'=' * 60}")
    print("  串口底盘驱动 — 全部测试通过 ✅")
    print(f"{'=' * 60}")
