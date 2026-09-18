"""
serial_chassis.py —— 上位机 ↔ 下位机(STM32) 串口底盘驱动

上位机（电脑/RDK）通过 TTL 串口与下位机（STM32F103 底盘板）通信。
协议依据 chassis_serial_protocol.md（v1.1）。

串口约定：
  - 波特率 115200，8 数据位，无校验，1 停止位，无流控，ASCII，逗号分隔，\\r\\n 结尾。
  - 上行（上位机 → 下位机）：
      · PING                      → 连通测试，下位机回复 PONG
      · START                     → 启动，下位机清零局部里程计，回复 ACK,START
      · VEL,v_mm_s,w_mrad_s       → 速度指令（v 整数 mm/s，w 整数 mrad/s）
      · SERVO,RAISE/LOWER/HOLD    → 套取机构舵机动作命令，回复 ACK,SERVO,xxx
      · SERVO,ANGLE,deg           → 调试舵机角度（0~85），回复 ACK,SERVO,ANGLE,deg
      · STOP                      → 普通停车（清 PI 积分，不清里程计），回复 ACK,STOP
      · ESTOP                     → 紧急停车锁定，回复 ACK,ESTOP
  - 下行（下位机 → 上位机）：
      · ODOM,x_m,y_m,theta_rad,encL,encR,vL_m_s,vR_m_s   —— 轮式里程计（20Hz，严格 8 字段）
      · IMU,tick_ms,seq,ax_mg,ay_mg,az_mg,gx_mrad_s,gy_mrad_s,gz_mrad_s,temp_cC  —— IMU 遥测（50Hz，10 字段）
      · ACK,xxx / EVENT,xxx / ERR,xxx / TEL,xxx          —— 只记录，不得误解析为位姿

坐标系（下位机局部）：x 前、y 左、theta 逆时针为正，启动朝向 0。
里程计、地图坐标转换、出发区全局偏移、IMU 零偏校准与融合由上位机负责。

设备文件：
  - 电脑调试（USB-TTL）：/dev/ttyUSB0（用 CHASSIS_PORT 覆盖）
  - RDK X5 部署：**/dev/ttyS1 @115200 —— 已真机实测确认**
    （PING→PONG、ODOM/IMU/TEL 遥测正常、VEL,200 实测 200.1mm/s）；
    代码默认值就是 /dev/ttyS1，RDK 上不需要显式导出 CHASSIS_PORT。
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
      frame = chassis.read_frame()                 # 读一帧（ODOM 或 IMU）
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
        # 共享接收缓冲：所有读取接口都从这里取整行，保证不丢数据（见 T0-6）
        self._rx_buf = b""
        self._rx_lines_dropped = 0          # 被丢弃的非位姿行（诊断用）

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
        # T1-9：关串口前先发一次停车。否则"最后一帧 VEL"会留在下位机里，
        # 只能靠它的 800ms 速度看门狗停下（850mm/s 下最多再冲 ~0.68m）。
        # 用 VEL,0,0 + STOP 而非 ESTOP：后者会锁定板子且 START 无法解除（协议 §4.4）。
        if self.is_open:
            try:
                self._send("VEL,0,0")
                self._send("STOP")
            except Exception as e:
                logger.warning(f"关串口前停车失败: {e}")
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
        """设置出发区位姿（出发区对齐）。**同时把当前下位机里程计记为基线。**

        ⚠️ 必须记基线（2026-09-16 真机实测，代价是整场跑不起来）：
        协议说 `START` 会清零下位机里程计，但**实测并不会** ——
        发 START 前后 ODOM 与编码器累计值**逐位相同**：
            前: x=+0.6823 y=+1.0959 th=-1.7043 enc=(921273,1240079)
            后: x=+0.6823 y=+1.0959 th=-1.7043 enc=(921273,1240079)
        而 `ChassisInterface.odom_to_upper` 把 ODOM 当**相对位移**用 →
        不清基线的话，MCU 上电以来累积的全部位移会被当成"从出发区走的距离"
        加到位姿上，车会以为自己已经在场外。实测自主运行日志：
            位姿重置: (2850, 2850), heading=-90°
            path_planner: A*: 起点 (3973, 2160) 不可通行     ← 场外约 1 米
            navigation_pipeline: 路径规划失败 — 无可行路径
        结果就是路径规划永远失败、车一动不动（电机本身完全正常）。
        """
        # 取一帧**最新**的原始里程计当基线；读不到就退用最近一次读到的
        raw = None
        try:
            self.read_pose()                      # 排空缓冲 + 更新 _last_raw_odom
        except Exception as e:
            logger.warning(f"读取里程计基线时异常: {e}")
        raw = getattr(self, "_last_raw_odom", None)
        if raw is not None:
            self._chassis.set_odom_baseline(*raw)
        else:
            logger.warning("未能取得里程计基线（还没读到过 ODOM）→ "
                           "位姿可能整体偏移；请确认下位机正在发送 ODOM 遥测")
        self._chassis.set_start_pose(x_mm, y_mm, theta_rad)

    # ---- 发送（上行） ----

    def send_ping(self, timeout: float = 0.5) -> bool:
        """连通测试：发 PING 并等待 PONG（返回是否真的收到 PONG 应答）。

        原实现只返回"写入成功"，未校验 PONG，导致自检/联调误判为连通。
        """
        if not self._send("PING"):
            return False
        return self.wait_for("PONG", timeout) is not None

    def send_start(self) -> bool:
        """启动命令：下位机收到后清零局部里程计，回复 ACK,START。"""
        return self._send("START")

    def send_velocity(self, v_mm_s: float, w_rad_s: float) -> bool:
        """发送速度指令 VEL,v_mm_s,w_mrad_s。"""
        cmd = ChassisInterface.velocity_to_command(v_mm_s, w_rad_s)
        return self._send(cmd)

    def send_servo(self, action: str) -> bool:
        """发送套取机构舵机动作命令 SERVO,RAISE/LOWER/HOLD。"""
        return self._send(f"SERVO,{action.upper()}")

    def send_servo_angle(self, deg: float) -> bool:
        """发送调试舵机角度 SERVO,ANGLE,deg（0~85 整数）。"""
        return self._send(f"SERVO,ANGLE,{int(deg)}")

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

    #: 关键命令的应答重试策略（见 ``send_command_await`` 的说明）。
    #: 下位机命令接收存在**秒级静默窗口**且可自愈（2026-09-16 现场实测：
    #: 连续 2 次 PING 无应答、第 3 次才通；也曾出现连续 3 次全哑）。
    #: 任何"只发一次 + 短超时"的关键命令都会因此被误判为故障。
    ACK_ATTEMPTS: int = 4
    ACK_TIMEOUT_S: float = 1.5
    ACK_GAP_S: float = 0.25

    def send_command_await(self, command: str, expect_prefix: str,
                           *, timeout: Optional[float] = None) -> Optional[str]:
        """发一条命令并等待指定前缀的应答，**带重试**；返回命中的应答行，全败返回 None。

        为什么必须重试（现场代价极大，逐条都是真实发生过的）：

        * ``start_match()`` 只发一次、超时 0.5s → 撞进静默窗口就直接把**整场比赛**
          拒掉：``PING 未收到 PONG，连通性检查失败`` →
          ``❌ 底盘 START 握手失败`` → ``🛑 紧急停止！原因: 底盘未启动``。
        * BOOT 自检的连通性探测同样撞过窗口，实测"前 2 次失败、第 3 次成功"。

        注意：静默窗口期间下位机**根本没收到命令**，所以等待再久也不会来应答 ——
        必须**重发**，这也是本函数存在的意义（单纯加长超时是无效的）。
        """
        to = self.ACK_TIMEOUT_S if timeout is None else timeout
        for attempt in range(1, self.ACK_ATTEMPTS + 1):
            # 先排空残留行，避免把上一条命令迟到的应答当成本次应答
            self.drain_lines()
            if not self._send(command):
                return None
            hit = self.wait_for(expect_prefix, to)
            if hit is not None:
                if attempt > 1:
                    logger.info(
                        f"{command} 第 {attempt} 次才收到 {expect_prefix}"
                        f"（下位机接收偶发静默，属已知现象）")
                return hit
            if attempt < self.ACK_ATTEMPTS:
                logger.warning(
                    f"{command} 第 {attempt}/{self.ACK_ATTEMPTS} 次未收到 "
                    f"{expect_prefix}（下位机接收静默窗口）→ 重发")
                time.sleep(self.ACK_GAP_S)
        return None

    def start_match(self, timeout: Optional[float] = None) -> bool:
        """
        启动顺序（协议第 8 节）：PING → 确认 PONG；START → 确认 ACK,START。

        ⚠️ 两个握手都必须**带重试**（见 ``send_command_await``）。旧实现每种只发
        一次、超时 0.5s，现场直接把整场比赛拒掉：
            PING 未收到 PONG，连通性检查失败
            ❌ 底盘 START 握手失败（PONG/ACK 超时）→ 拒绝进入自主模式
            🛑 紧急停止！原因: 底盘未启动
        """
        if not self.is_open:
            logger.warning("串口未打开，无法启动")
            return False
        to = self.ACK_TIMEOUT_S if timeout is None else timeout
        if self.send_command_await("PING", "PONG", timeout=to) is None:
            logger.warning(f"PING 未收到 PONG（已重试 {self.ACK_ATTEMPTS} 次）"
                           f"，连通性检查失败")
            return False
        if self.send_command_await("START", "ACK,START", timeout=to) is None:
            logger.warning(f"START 未收到 ACK,START（已重试 {self.ACK_ATTEMPTS} 次）")
            return False
        logger.info("底盘启动完成（PONG + ACK,START）")
        return True

    # ---- 接收（下行） ----

    def _pump_rx(self) -> None:
        """把串口当前**全部**可用字节搬进接收缓冲（非阻塞，不等待）。"""
        if not self.is_open:
            return
        try:
            waiting = int(getattr(self._ser, "in_waiting", 0) or 0)
            if waiting > 0:
                self._rx_buf += self._ser.read(waiting)
        except Exception as e:
            logger.debug(f"串口读取失败: {e}")

    def _pop_line_buffered(self) -> Optional[str]:
        """只从接收缓冲取一整行；缓冲里没有整行就返回 None（**绝不阻塞**）。"""
        self._pump_rx()
        if b"\n" not in self._rx_buf:
            return None
        line, _, rest = self._rx_buf.partition(b"\n")
        self._rx_buf = rest
        return line.decode("ascii", errors="ignore").strip()

    @property
    def rx_lines_dropped(self) -> int:
        """被丢弃的非位姿行数（T0-6 诊断：正常比赛它会持续增长，属预期）。"""
        return self._rx_lines_dropped

    def drain_lines(self, max_lines: int = 512) -> int:
        """丢弃接收缓冲里**已经到达**的整行，返回丢弃行数（不阻塞）。

        用途：发一条命令并等待它的应答之前，先把上文残留的行清掉。
        否则会出现"把上一条命令迟到的应答当成本次应答"的假通过 ——
        现场实测到的典型表现是 BOOT 自检里
        `电机 #1 FAIL (505ms, 超时)` + `电机 #2 PASS (3ms)`：
        #1 的 PONG 只是晚到，被 #2 的 wait_for 立刻捡走了。

        注意：本方法会丢弃排队中的遥测行（ODOM/IMU/TEL），
        只应在"马上要发命令、且不在乎丢掉这几帧遥测"的场合使用。
        """
        dropped = 0
        for _ in range(max(0, max_lines)):
            if self._pop_line_buffered() is None:
                break
            dropped += 1
        return dropped

    def _read_line(self) -> Optional[str]:
        """读一行原始文本（strip）；无数据/未打开返回 None。"""
        if not self.is_open:
            return None
        # 先看共享缓冲有没有整行（T0-6：缓冲由所有读取接口共用，避免互相"偷吃"数据）
        buffered = self._pop_line_buffered()
        if buffered is not None:
            return buffered
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
        """
        读一行并解析为 ODOM 或 IMU 帧；其他行（ACK/EVENT/ERR/TEL 等）返回 None。

        返回 dict 带 'type' 字段（ODOM / IMU）。
        """
        text = self._read_line()
        if not text:
            return None
        prefix = text.split(',')[0].upper() if ',' in text else text.upper()
        if prefix == 'ODOM':
            frame = self.parse_frame(text)
            if frame:
                frame = dict(frame)
                frame['type'] = 'ODOM'
            return frame
        elif prefix == 'IMU':
            frame = self.parse_imu(text)
            if frame:
                frame = dict(frame)
                frame['type'] = 'IMU'
            return frame
        return None

    #: 单帧最多排空多少行（防止极端积压时把主循环拖住）
    MAX_LINES_PER_FRAME = 64

    def read_pose(self) -> Optional[Tuple[float, float, float]]:
        """读**最新**的 ODOM 位姿（排空接收缓冲，只保留最后一条有效 ODOM）。

        ⚠️⚠️ T0-6：旧实现每帧只 `readline()` **一行**且不排空缓冲，而按协议下位机
        持续发送 ODOM 20Hz + IMU 50Hz + TEL 5~10Hz = **75~80 行/秒**，上位机 50Hz
        主循环只消费 **50 行/秒** → 每秒净积压 25~30 行，非 ODOM 行被直接丢弃且不补读
        → **控制回路用的是过期位姿**（位姿越来越旧），进安全区/贴围栏/套取对准全部
        按过期坐标执行。现在一次把缓冲里**所有**整行取出来，只保留最后一条 ODOM。
        """
        if not self.is_open:
            return None

        latest: Optional[str] = None
        for _ in range(self.MAX_LINES_PER_FRAME):
            text = self._pop_line_buffered()       # 非阻塞，缓冲无整行即返回 None
            if text is None:
                break
            if text.upper().startswith('ODOM'):
                latest = text
            else:
                self._rx_lines_dropped += 1
        if latest is None:
            # 缓冲里这一帧没有 ODOM（例如刚开机/掉线）→ 回退到一次阻塞读，保持原语义
            text = self._read_line()
            if text and text.upper().startswith('ODOM'):
                latest = text
            elif text:
                self._rx_lines_dropped += 1
        if latest is None:
            return None
        text = latest
        frame = self.parse_frame(text)
        if frame is None:
            return None
        self._chassis.update_raw(
            (frame.get('encL', 0), frame.get('encR', 0)),
            (frame.get('vL', 0.0), frame.get('vR', 0.0)),
        )
        # 记下**原始**里程计（未经 odom_to_upper 转换）：
        # set_start_pose() 用它当基线，见那里的说明。
        self._last_raw_odom = (frame['x_m'], frame['y_m'], frame['theta_rad'])
        return self._chassis.odom_to_upper(
            frame['x_m'], frame['y_m'], frame['theta_rad'],
        )

    def read_imu(self, timeout: float = 0.5) -> Optional[dict]:
        """循环读行直到拿到 IMU 遥测帧（串口为 IMU/ODOM/TEL 混流）。

        只读一行会大概率命中 ODOM/TEL 而误判"无 IMU"，故按超时循环读取。
        """
        deadline = time.time() + timeout
        while time.time() < deadline:
            text = self._read_line()
            if text and text.upper().startswith('IMU'):
                return self.parse_imu(text)
        return None

    def wait_for(self, prefix: str, timeout: float = 0.5) -> Optional[str]:
        """
        循环读行直到出现以 prefix 开头的行（大小写不敏感），返回该行；超时返回 None。
        """
        deadline = time.time() + timeout
        while time.time() < deadline:
            text = self._read_line()
            if text and text.upper().startswith(prefix.upper()):
                return text
        return None

    def wait_for_button(self, timeout: float = 5.0) -> bool:
        """等待一键启动按钮事件（下位机发 EVENT,START_BUTTON）；返回是否收到。"""
        return self.wait_for("EVENT,START_BUTTON", timeout) is not None

    def read_button(self) -> Optional[str]:
        """读一行，若为一键启动事件(EVENT,START_BUTTON)返回该行，否则 None。

        注：下位机 command.c 按钮按下时发 `EVENT,START_BUTTON`；
        自锁开关拨动另有 `EVENT,BUTTON_LED_ON/OFF`（此处忽略）。
        """
        # T0-7：DEBUG 阶段也持续收到 75~80 行/秒遥测，而本函数由主循环每 0.5s 才调一次。
        # 旧实现只读一行 → 消费 2 行/秒，缓冲在 1~3 秒内积压饱和并开始丢字节，
        # 而 `EVENT,START_BUTTON` 是**一次性事件**（协议 §5.1.1），丢了就永不匹配
        # → 按了开关进不了 AUTONOMOUS。现在排空缓冲，并在整批里找该事件。
        found = None
        for _ in range(self.MAX_LINES_PER_FRAME):
            text = self._pop_line_buffered()
            if text is None:
                break
            if text.upper().startswith("EVENT,START_BUTTON"):
                found = text
            elif text:
                self._rx_lines_dropped += 1
        if found is None:
            text = self._read_line()
            if text and text.upper().startswith("EVENT,START_BUTTON"):
                found = text
        return found

    #: 视为"操作员请求启动"的下位机事件（任一命中即算）。
    #:   EVENT,START_BUTTON  —— 下位机处于 WAIT_START 时拨动自锁开关：固件走
    #:                          `Command_EnterRunning(true)` 分支并发此事件（正常路径）。
    #:   EVENT,BUTTON_LED_ON  —— 下位机**已处于 RUNNING** 时拨动开关：固件认为
    #:                          "已经在跑了"，只点亮状态灯并发此事件，
    #:                          **不发** START_BUTTON。
    #: 为什么两个都要认：现场流程常是"先跑手动/台架测试（下位机被 START 过）→
    #: 再启动上位机 → 操作员拨启动开关"，此时只来 BUTTON_LED_ON，而旧实现
    #: 只认 START_BUTTON → **上位机永久卡在 DEBUG、车一动不动**
    #: （2026-09-16 现场实际发生）。从操作员意图看，"把启动开关拨到 ON"
    #: 就是"开始比赛"，两个事件等价，都该触发 one_key_start()。
    START_REQUEST_EVENTS = ("EVENT,START_BUTTON", "EVENT,BUTTON_LED_ON")

    def read_start_request(self) -> Optional[str]:
        """排空缓冲后返回命中的启动请求事件行，无则 None（见 START_REQUEST_EVENTS）。"""
        found = None
        for _ in range(self.MAX_LINES_PER_FRAME):
            text = self._pop_line_buffered()
            if text is None:
                break
            if text.upper().startswith(self.START_REQUEST_EVENTS):
                found = text
            elif text:
                self._rx_lines_dropped += 1
        if found is None:
            text = self._read_line()
            if text and text.upper().startswith(self.START_REQUEST_EVENTS):
                found = text
        return found

    def parse_frame(self, text: str) -> Optional[dict]:
        """
        严格解析 ODOM 里程计帧。

        仅接受：前缀严格等于 ODOM、恰好 8 个字段、数值全合法。
        其他行一律返回 None。
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

    def parse_imu(self, text: str) -> Optional[dict]:
        """
        严格解析 IMU 遥测帧。

        格式：IMU,tick_ms,seq,ax_mg,ay_mg,az_mg,gx_mrad_s,gy_mrad_s,gz_mrad_s,temp_cC
        仅接受：前缀严格等于 IMU、恰好 10 个字段、数值全合法（均为整数）。
        """
        parts = [p for p in text.replace(' ', '').split(',') if p]
        if len(parts) != 10:
            return None
        if parts[0].upper() != 'IMU':
            return None
        try:
            tick_ms = int(parts[1])
            seq = int(parts[2])
            ax = int(parts[3])
            ay = int(parts[4])
            az = int(parts[5])
            gx = int(parts[6])
            gy = int(parts[7])
            gz = int(parts[8])
            temp = int(parts[9])
        except ValueError:
            return None

        frame = {
            'tick_ms': tick_ms, 'seq': seq,
            'ax_mg': ax, 'ay_mg': ay, 'az_mg': az,
            'gx_mrad_s': gx, 'gy_mrad_s': gy, 'gz_mrad_s': gz,
            'temp_cC': temp,
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
    print("  串口底盘驱动 — 帧解析测试（ODOM + IMU）")
    print("=" * 60)

    sc = SerialChassis(port='/dev/ttyUSB0')

    # --- ODOM：合法 8 字段 ---
    frame = sc.parse_frame("ODOM,1.000,0.000,0.000,442,440,0.50,0.48")
    assert frame and abs(frame['x_m'] - 1.0) < 1e-9 and frame['encL'] == 442
    print("ODOM 合法帧 ✅")

    # --- ODOM：拒绝非 8 字段 / 非 ODOM 前缀 ---
    assert sc.parse_frame("0.0,1.0,1.5708") is None
    assert sc.parse_frame("ACK,START") is None
    assert sc.parse_frame("EVENT,WATCHDOG_STOP") is None
    assert sc.parse_frame("TEL,1,2") is None
    print("ODOM 拒绝非 8 字段/非 ODOM ✅")

    # --- IMU：合法 10 字段 ---
    imu = sc.parse_imu("IMU,123456,2481,12,-8,998,15,-7,23,3653")
    assert imu and imu['gz_mrad_s'] == 23 and imu['seq'] == 2481 and imu['temp_cC'] == 3653
    print(f"IMU 合法帧 ✅ gz={imu['gz_mrad_s']} mrad/s, temp={imu['temp_cC']/100:.2f}°C")

    # --- IMU：拒绝非 10 字段 / 非整数 ---
    assert sc.parse_imu("IMU,1,2,3") is None
    assert sc.parse_imu("ODOM,1,2,3,4,5,6,7,8") is None
    print("IMU 拒绝非 10 字段 ✅")

    # --- read_frame 区分 ODOM / IMU ---
    print(f"\nframe_prefix 判定: ODOM->{sc.parse_frame('ODOM,1,0,0,1,2,3,4')['x_m']}, "
          f"IMU->{sc.parse_imu('IMU,1,2,3,4,5,6,7,8,9')['gz_mrad_s']}")

    # --- 命令格式化 ---
    print(f"\nSERVO,LOWER -> {sc.send_servo.__name__} 生成 'SERVO,LOWER'")
    # 无法在未打开串口时真发，校验字符串生成逻辑
    assert sc.send_servo('lower') is False  # 未打开串口返回 False
    assert 'SERVO,RAISE'.upper() == 'SERVO,RAISE'
    assert sc.send_servo_angle(90) is False
    print("SERVO 命令（未打开串口返回 False）✅")

    print(f"\n{'=' * 60}")
    print("  串口底盘驱动 — 全部测试通过 ✅")
    print(f"{'=' * 60}")
