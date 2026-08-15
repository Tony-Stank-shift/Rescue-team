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
