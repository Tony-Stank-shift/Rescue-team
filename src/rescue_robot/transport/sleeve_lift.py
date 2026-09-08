"""
sleeve_lift.py —— 升降套取机构控制

机器人使用「从上往下套取」的方式转运目标：
  导航到目标上方 → 下降套住目标 → 保持套住运送 → 升起到安全区释放。

机械结构：单自由度升降（转轴上下，丝杆驱动），无独立旋转自由度。
  - 下降（lower） = 套住目标
  - 升起（raise_up） = 释放目标
  - 保持（hold） = 套住目标运送中

硬件抽象：
  AbstractSleeveLift  — 抽象基类
  MockSleeveLift      — 模拟套取机构（开发测试）
  ScrewSleeveLift     — 丝杆升降（真实硬件，电机 + 上下限位，参数待定）
"""

import logging
import time
from dataclasses import dataclass, field
from enum import Enum, auto
from typing import Optional, Set

logger = logging.getLogger("sleeve_lift")


# ============================================================
# 套取机构动作与状态
# ============================================================

class SleeveAction(Enum):
    """套取机构动作（单自由度升降）"""
    RAISED = auto()     # 升起（高位，释放状态）
    LOWERED = auto()    # 下降（低位，套住目标）
    HOLD = auto()       # 保持（套住目标运送中）


@dataclass
class SleeveState:
    """套取机构当前状态"""
    action: SleeveAction = SleeveAction.RAISED
    position_mm: float = 0.0             # 当前升降位置（0 = 最高位）
    stroke_mm: float = 150.0             # 最大升降行程（待定）
    holding_count: int = 0               # 当前套住目标数
    holding_ids: Set[int] = field(default_factory=set)  # 套住的目标 ID
    timestamp: float = 0.0


# ============================================================
# 抽象套取机构
# ============================================================

class AbstractSleeveLift:
    """套取机构抽象基类"""

    def lower(self) -> bool:
        """下降套住目标"""
        raise NotImplementedError

    def raise_up(self) -> bool:
        """升起释放目标"""
        raise NotImplementedError

    def hold(self) -> bool:
        """保持当前高度（运送中）"""
        raise NotImplementedError

    def is_holding(self) -> bool:
        """是否正在套住目标"""
        raise NotImplementedError

    @property
    def state(self) -> SleeveState:
        raise NotImplementedError


# ============================================================
# Mock 套取机构
# ============================================================

class MockSleeveLift(AbstractSleeveLift):
    """
    模拟套取机构：基于位置判定下降是否套住目标。

    判定逻辑：
    - 下降后目标在套取范围内 → 套取成功
    - 否则 → 套取失败
    """

    LOWER_TIME_S = 0.5     # 下降耗时
    RAISE_TIME_S = 0.3     # 升起耗时

    def __init__(self, stroke_mm: float = 150.0):
        self._state = SleeveState(
            action=SleeveAction.RAISED,
            stroke_mm=stroke_mm,
            timestamp=time.time(),
        )
        self._target_positions: dict = {}  # id → (x, y) 用于模拟判定
        logger.info(f"MockSleeveLift 初始化: stroke={stroke_mm}mm")

    @property
    def state(self) -> SleeveState:
        return self._state

    def lower(self, target_positions: Optional[dict] = None) -> bool:
        """
        下降套取目标。

        Args:
            target_positions: {target_id: (x_mm, y_mm)} 目标位置字典
        Returns:
            True=套取成功, False=下方无目标
        """
        if target_positions is not None:
            self._target_positions = target_positions

        time.sleep(self.LOWER_TIME_S)
        self._state.action = SleeveAction.LOWERED
        self._state.position_mm = self._state.stroke_mm
        self._state.timestamp = time.time()

        # 模拟判定：套住范围内所有目标
        captured = set(self._target_positions.keys())

        if captured:
            self._state.holding_ids = captured
            self._state.holding_count = len(captured)
            self._state.action = SleeveAction.HOLD
            logger.info(f"下降套取成功: {len(captured)} 个目标, IDs={captured}")
            return True
        else:
            self._state.holding_ids.clear()
            self._state.holding_count = 0
            logger.debug("下降套取失败：下方无目标")
            return False

    def lower_with_retry(self, target_positions=None, max_retries=3) -> bool:
        """带重试的下降套取"""
        for attempt in range(1, max_retries + 1):
            self.raise_up()
            if self.lower(target_positions):
                return True
            logger.warning("套取重试 %d/%d", attempt, max_retries)
            time.sleep(0.3)
        logger.error("套取失败（%d 次重试后）", max_retries)
        return False

    def raise_up(self) -> bool:
        """升起释放所有套住目标"""
        if self._state.action == SleeveAction.RAISED:
            return True

        time.sleep(self.RAISE_TIME_S)
        released = self._state.holding_ids.copy()
        self._state.action = SleeveAction.RAISED
        self._state.position_mm = 0.0
        self._state.holding_ids.clear()
        self._state.holding_count = 0
        self._state.timestamp = time.time()
        logger.info(f"升起释放目标: IDs={released}")
        return True

    def hold(self) -> bool:
        self._state.action = SleeveAction.HOLD
        return True

    def is_holding(self) -> bool:
        return self._state.holding_count > 0


# ============================================================
# 丝杆升降套取机构 —— 真实硬件
# ============================================================

class ScrewSleeveLift(AbstractSleeveLift):
    """
    丝杆升降的真实套取机构。

    单自由度：转轴（丝杆）旋转驱动上下移动，配上下限位开关。
    ⚠️ 电机型号 / 驱动方式 / 引脚分配均待定，以下为占位实现。
    """

    def __init__(self,
                 motor_pin: Optional[int] = None,
                 upper_limit_pin: Optional[int] = None,
                 lower_limit_pin: Optional[int] = None,
                 stroke_mm: float = 150.0):
        self._motor_pin = motor_pin
        self._upper_limit_pin = upper_limit_pin
        self._lower_limit_pin = lower_limit_pin

        self._state = SleeveState(
            action=SleeveAction.RAISED,
            stroke_mm=stroke_mm,
            timestamp=time.time(),
        )

        # 延迟导入硬件库
        try:
            import RPi.GPIO as GPIO
            self._GPIO = GPIO
            # 占位：实际接线与 PWM 驱动待定
        except ImportError:
            logger.warning("RPi.GPIO 未安装，套取机构控制不可用")

        logger.info(f"ScrewSleeveLift 初始化: stroke={stroke_mm}mm（引脚/驱动待定）")

    @property
    def state(self) -> SleeveState:
        return self._state

    def lower(self, target_positions: Optional[dict] = None) -> bool:
        self._state.action = SleeveAction.LOWERED
        self._state.position_mm = self._state.stroke_mm
        self._state.timestamp = time.time()

        # 真实场景需要限位开关/电流检测判定是否到位
        # 当前简化：默认下降成功
        if target_positions:
            self._state.holding_ids = set(target_positions.keys())
            self._state.holding_count = len(target_positions)
            self._state.action = SleeveAction.HOLD

        return True

    def raise_up(self) -> bool:
        self._state.action = SleeveAction.RAISED
        self._state.position_mm = 0.0
        self._state.holding_ids.clear()
        self._state.holding_count = 0
        self._state.timestamp = time.time()
        return True

    def hold(self) -> bool:
        self._state.action = SleeveAction.HOLD
        return True

    def is_holding(self) -> bool:
        return self._state.holding_count > 0

    def cleanup(self) -> None:
        self.raise_up()


# ============================================================
# SG90 舵机套取机构 —— 真实硬件（已确认参数）
# ============================================================

class ServoSleeveLift(AbstractSleeveLift):
    """
    SG90 舵机驱动的套取机构（从上往下套住目标）。

    已确认参数：
      - 舵机：SG90（PWM 控制）
      - 行程：0°（平行地面，释放）⇄ 90°（垂直地面，套住）
      - 夹爪臂长：90mm

    状态映射：
      - RAISED  = 0°（夹爪平行地面，抬起）
      - LOWERED = 90°（夹爪垂直地面，下压套住）
      - HOLD    = 90°（套住目标运送中）

    ⚠️ PWM 输出到真实舵机由硬件层实现（pwm_pin 待定），当前记录角度。
    """

    ANGLE_RAISED_DEG = 0.0    # 平行地面（释放）
    ANGLE_LOWERED_DEG = 90.0  # 垂直地面（套住）
    MOVE_TIME_S = 0.4         # 0°↔90° 移动耗时

    def __init__(self, pwm_pin: Optional[int] = None,
                 arm_length_mm: float = 90.0):
        self._pwm_pin = pwm_pin          # 待定（主控/STM32 PWM 输出）
        self._arm_length_mm = arm_length_mm
        self._angle_deg = self.ANGLE_RAISED_DEG
        self._target_positions: dict = {}

        self._state = SleeveState(
            action=SleeveAction.RAISED,
            stroke_mm=arm_length_mm,
            timestamp=time.time(),
        )
        logger.info(f"ServoSleeveLift(SG90) 初始化: 行程 0~90°, "
                    f"夹爪 {arm_length_mm}mm, pwm_pin={pwm_pin}")

    @property
    def state(self) -> SleeveState:
        return self._state

    @property
    def angle_deg(self) -> float:
        return self._angle_deg

    def lower(self, target_positions: Optional[dict] = None) -> bool:
        """下压到 90° 套住目标。"""
        if target_positions is not None:
            self._target_positions = target_positions

        time.sleep(self.MOVE_TIME_S)
        self._angle_deg = self.ANGLE_LOWERED_DEG
        self._state.action = SleeveAction.LOWERED
        self._state.position_mm = self._arm_length_mm
        self._state.timestamp = time.time()

        captured = set(self._target_positions.keys())
        if captured:
            self._state.holding_ids = captured
            self._state.holding_count = len(captured)
            self._state.action = SleeveAction.HOLD
            logger.info(f"舵机下压套住: {len(captured)} 个目标, IDs={captured}")
            return True
        else:
            self._state.holding_ids.clear()
            self._state.holding_count = 0
            logger.debug("舵机下压未套住目标")
            return False

    def lower_with_retry(self, target_positions=None, max_retries=3) -> bool:
        """带重试的下压套取。"""
        for attempt in range(1, max_retries + 1):
            self.raise_up()
            if self.lower(target_positions):
                return True
            logger.warning("套取重试 %d/%d", attempt, max_retries)
            time.sleep(0.3)
        logger.error("套取失败（%d 次重试后）", max_retries)
        return False

    def raise_up(self) -> bool:
        """抬起到 0°（平行地面），释放目标。"""
        if self._state.action == SleeveAction.RAISED:
            return True

        time.sleep(self.MOVE_TIME_S)
        released = self._state.holding_ids.copy()
        self._angle_deg = self.ANGLE_RAISED_DEG
        self._state.action = SleeveAction.RAISED
        self._state.position_mm = 0.0
        self._state.holding_ids.clear()
        self._state.holding_count = 0
        self._state.timestamp = time.time()
        logger.info(f"舵机抬起释放: IDs={released}")
        return True

    def hold(self) -> bool:
        self._state.action = SleeveAction.HOLD
        return True

    def is_holding(self) -> bool:
        return self._state.holding_count > 0

    def set_angle(self, deg: float) -> None:
        """设置舵机角度（真实 PWM 输出 TODO：写 pwm_pin）。"""
        self._angle_deg = deg
        self._state.timestamp = time.time()
        # TODO: 真实 PWM 输出（RDK/STM32 的 PWM 接口）

    # 放置（推+上调）：U型槽后方实心板，推入时渐进上调跨过紫边斜坡（外低内高）
    RAMP_ANGLE_DEG = 34.0   # 斜坡角度（30 宽 / 20 高 ≈ 34°）
    RAMP_STEPS = 4          # 渐进上调步数

    def place_ramp(self) -> bool:
        """放置：舵机从 90°(套住) 渐进下调到 0°(放平)，配合底盘前进"推"入放置区。

        ⚠️ 精确角度序列 / 与底盘前进的协同需真机标定（SG90 定位，斜坡 34°）。
        """
        released = self._state.holding_ids.copy()
        for step in range(self.RAMP_STEPS + 1):
            frac = step / self.RAMP_STEPS
            deg = self.ANGLE_LOWERED_DEG * (1.0 - frac)  # 90 → 0
            self.set_angle(deg)
            time.sleep(0.15)
        self._state.action = SleeveAction.RAISED
        self._state.position_mm = 0.0
        self._state.holding_ids.clear()
        self._state.holding_count = 0
        self._state.timestamp = time.time()
        logger.info(f"放置(推+上调): 渐进放平释放 IDs={released}")
        return True

    def cleanup(self) -> None:
        self.raise_up()


# ============================================================
# 串口舵机套取机构 —— 通过下位机 SERVO 命令控制（协议 v1.1）
# ============================================================

class SerialServoLift(AbstractSleeveLift):
    """
    套取机构通过下位机（STM32）控制：上位机发 SERVO,RAISE/LOWER/HOLD 命令。

    与 SERVO 命令对应的动作：
      - RAISE → 转到安全抬起位置
      - LOWER → 转到放下套取位置（套住）
      - HOLD  → 保持放下位置

    依赖一个具备 send_servo() / wait_for() 方法的 chassis 对象
    （即 SerialChassis），用 duck typing 避免循环依赖。
    """

    MOVE_TIME_S = 0.4  # 机械动作耗时（等舵机到位）

    def __init__(self, chassis, arm_length_mm: float = 90.0,
                 move_time_s: float = 0.4):
        self._chassis = chassis
        self._arm_length_mm = arm_length_mm
        self._move_time_s = move_time_s
        self._target_positions: dict = {}
        self._state = SleeveState(
            action=SleeveAction.RAISED,
            stroke_mm=arm_length_mm,
            timestamp=time.time(),
        )
        logger.info(f"SerialServoLift 初始化: 夹爪 {arm_length_mm}mm（经串口 SERVO 命令）")

    @property
    def state(self) -> SleeveState:
        return self._state

    def lower(self, target_positions: Optional[dict] = None) -> bool:
        """发 SERVO,LOWER 套住目标（等待 ACK）。"""
        if target_positions is not None:
            self._target_positions = target_positions

        if not self._chassis.send_servo("LOWER"):
            logger.warning("SERVO,LOWER 发送失败")
            return False
        self._chassis.wait_for("ACK,SERVO,LOWER", 0.5)
        time.sleep(self._move_time_s)

        self._state.action = SleeveAction.LOWERED
        self._state.position_mm = self._arm_length_mm

        captured = set(self._target_positions.keys())
        if captured:
            self._state.holding_ids = captured
            self._state.holding_count = len(captured)
            self._state.action = SleeveAction.HOLD
            logger.info(f"SERVO,LOWER 套住: {len(captured)} 个目标")
            return True
        else:
            self._state.holding_ids.clear()
            self._state.holding_count = 0
            logger.debug("SERVO,LOWER 未套住目标")
            return False

    def lower_with_retry(self, target_positions=None, max_retries=3) -> bool:
        """带重试的 SERVO,LOWER 套取。"""
        for attempt in range(1, max_retries + 1):
            self.raise_up()
            if self.lower(target_positions):
                return True
            logger.warning("套取重试 %d/%d", attempt, max_retries)
            time.sleep(0.3)
        logger.error("套取失败（%d 次重试后）", max_retries)
        return False

    def raise_up(self) -> bool:
        """发 SERVO,RAISE 抬起释放目标。"""
        if self._state.action == SleeveAction.RAISED:
            return True

        if not self._chassis.send_servo("RAISE"):
            logger.warning("SERVO,RAISE 发送失败")
            return False
        self._chassis.wait_for("ACK,SERVO,RAISE", 0.5)
        time.sleep(self._move_time_s)

        released = self._state.holding_ids.copy()
        self._state.action = SleeveAction.RAISED
        self._state.position_mm = 0.0
        self._state.holding_ids.clear()
        self._state.holding_count = 0
        logger.info(f"SERVO,RAISE 抬起释放: IDs={released}")
        return True

    def hold(self) -> bool:
        self._state.action = SleeveAction.HOLD
        return True

    def is_holding(self) -> bool:
        return self._state.holding_count > 0

    # 放置（推+上调）：U型槽后方实心板，推入时渐进上调跨过紫边斜坡（外低内高）
    RAMP_ANGLE_DEG = 34.0
    RAMP_STEPS = 4

    def place_ramp(self) -> bool:
        """放置：发 SERVO,ANGLE 渐进上调（90→0），配合底盘前进"推"入放置区。

        ⚠️ 精确角度序列 / 与底盘前进协同需真机标定（斜坡 34°，SG90 定位）。
        """
        released = self._state.holding_ids.copy()
        for step in range(self.RAMP_STEPS + 1):
            frac = step / self.RAMP_STEPS
            deg = int(round(90.0 * (1.0 - frac)))  # 90 → 0
            self._chassis.send_servo_angle(deg)
            time.sleep(self._move_time_s / self.RAMP_STEPS)
        self._state.action = SleeveAction.RAISED
        self._state.position_mm = 0.0
        self._state.holding_ids.clear()
        self._state.holding_count = 0
        logger.info(f"放置(推+上调): 渐进放平释放 IDs={released}")
        return True

    def cleanup(self) -> None:
        self.raise_up()
