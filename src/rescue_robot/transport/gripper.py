"""
gripper.py —— 夹爪控制

机器人使用夹爪夹取目标 → 拖/推至安全区 → 松开释放。

硬件抽象：
  AbstractGripper  — 抽象基类
  MockGripper      — 模拟夹爪（开发测试）
  ServoGripper     — 舵机/PWM 夹爪（真实硬件）
"""

import logging
import time
from dataclasses import dataclass, field
from enum import Enum, auto
from typing import List, Optional, Set, Tuple

logger = logging.getLogger("gripper")


# ============================================================
# 夹爪动作与状态
# ============================================================

class GripperAction(Enum):
    """夹爪动作"""
    OPEN = auto()       # 打开（准备夹取）
    CLOSE = auto()      # 闭合（夹取目标）
    HOLD = auto()       # 保持（夹住目标移动中）
    RELEASE = auto()    # 释放（投放目标）


@dataclass
class GripperState:
    """夹爪当前状态"""
    action: GripperAction = GripperAction.OPEN
    holding_count: int = 0               # 当前夹持目标数
    holding_ids: Set[int] = field(default_factory=set)  # 夹持的目标 ID
    position_mm: float = 0.0             # 夹爪开度 (mm)
    max_opening_mm: float = 140.0        # 最大开度（能夹取的最大目标尺寸）
    force_n: float = 0.0                 # 当前夹持力 (N)
    timestamp: float = 0.0


# ============================================================
# 抽象夹爪
# ============================================================

class AbstractGripper:
    """夹爪抽象基类"""

    def open(self) -> bool:
        """打开夹爪"""
        raise NotImplementedError

    def close(self) -> bool:
        """闭合夹爪（夹取目标）"""
        raise NotImplementedError

    def release(self) -> bool:
        """释放（打开夹爪投放目标）"""
        raise NotImplementedError

    def is_holding(self) -> bool:
        """是否正在夹持目标"""
        raise NotImplementedError

    @property
    def state(self) -> GripperState:
        raise NotImplementedError


# ============================================================
# Mock 夹爪
# ============================================================

class MockGripper(AbstractGripper):
    """
    模拟夹爪：基于位置判定夹取成功/失败。

    判定逻辑：
    - 目标在夹爪范围内（距离 < GRIP_RANGE_MM）→ 夹取成功
    - 否则 → 夹取失败
    """

    GRIP_RANGE_MM = 80.0          # 夹爪有效范围
    OPEN_TIME_S = 0.3             # 打开耗时
    CLOSE_TIME_S = 0.5            # 闭合耗时

    def __init__(self, max_opening_mm: float = 140.0):
        self._state = GripperState(
            action=GripperAction.OPEN,
            max_opening_mm=max_opening_mm,
            timestamp=time.time(),
        )
        self._target_positions: dict = {}  # id → (x, y) 用于模拟判定
        logger.info(f"MockGripper 初始化: max_opening={max_opening_mm}mm")

    @property
    def state(self) -> GripperState:
        return self._state

    def open(self) -> bool:
        if self._state.action == GripperAction.OPEN:
            return True
        time.sleep(self.OPEN_TIME_S)
        self._state.action = GripperAction.OPEN
        self._state.timestamp = time.time()
        logger.debug("夹爪打开")
        return True

    def close(self, target_positions: Optional[dict] = None) -> bool:
        """
        闭合夹爪。

        Args:
            target_positions: {target_id: (x_mm, y_mm)} 目标位置字典
        Returns:
            True=夹取成功, False=范围内无目标
        """
        if target_positions is not None:
            self._target_positions = target_positions

        time.sleep(self.CLOSE_TIME_S)
        self._state.action = GripperAction.CLOSE
        self._state.timestamp = time.time()

        # 模拟判定：夹取范围内所有目标
        gripped = set()
        for tid, (tx, ty) in self._target_positions.items():
            # 简化：所有在范围内的目标都被夹取
            gripped.add(tid)

        if gripped:
            self._state.holding_ids = gripped
            self._state.holding_count = len(gripped)
            self._state.action = GripperAction.HOLD
            logger.info(f"夹取成功: {len(gripped)} 个目标, IDs={gripped}")
            return True
        else:
            self._state.holding_ids.clear()
            self._state.holding_count = 0
            logger.debug("夹取失败：范围内无目标")
            return False

    def release(self) -> bool:
        """释放所有夹持目标"""
        if self._state.action == GripperAction.OPEN:
            return True

        time.sleep(self.OPEN_TIME_S)
        released = self._state.holding_ids.copy()
        self._state.action = GripperAction.OPEN
        self._state.holding_ids.clear()
        self._state.holding_count = 0
        self._state.timestamp = time.time()
        logger.info(f"释放目标: IDs={released}")
        return True

    def is_holding(self) -> bool:
        return self._state.holding_count > 0

    def can_grip_size(self, size_mm: float) -> bool:
        """检查目标尺寸是否在夹爪范围内"""
        return size_mm <= self._state.max_opening_mm


# ============================================================
# 舵机夹爪 —— 真实硬件
# ============================================================

class ServoGripper(AbstractGripper):
    """
    舵机驱动的真实夹爪。

    使用 PWM 信号控制舵机角度：
    - OPEN: 舵机角度 = open_angle（最大开度）
    - CLOSE: 舵机角度 = close_angle（最小开度/夹紧）
    """

    def __init__(self, servo_pin: int,
                 open_angle: float = 90.0,
                 close_angle: float = 0.0,
                 max_opening_mm: float = 140.0):
        """
        Args:
            servo_pin: 舵机 GPIO/PWM 引脚
            open_angle: 打开时的舵机角度
            close_angle: 闭合时的舵机角度
            max_opening_mm: 最大开度
        """
        self._pin = servo_pin
        self._open_angle = open_angle
        self._close_angle = close_angle
        self._current_angle = open_angle

        self._state = GripperState(
            action=GripperAction.OPEN,
            max_opening_mm=max_opening_mm,
            timestamp=time.time(),
        )

        # 延迟导入硬件库
        try:
            import RPi.GPIO as GPIO
            self._GPIO = GPIO
            self._pwm = None  # 需初始化 PWM
        except ImportError:
            logger.warning("RPi.GPIO 未安装，舵机控制不可用")

        logger.info(f"ServoGripper 初始化: pin={servo_pin}, "
                     f"开度={open_angle}°→{close_angle}°")

    @property
    def state(self) -> GripperState:
        return self._state

    def open(self) -> bool:
        self._set_angle(self._open_angle)
        self._state.action = GripperAction.OPEN
        self._state.holding_ids.clear()
        self._state.holding_count = 0
        self._state.timestamp = time.time()
        return True

    def close(self, target_positions: Optional[dict] = None) -> bool:
        self._set_angle(self._close_angle)
        self._state.action = GripperAction.CLOSE
        self._state.timestamp = time.time()

        # 真实场景需要力反馈或电流检测判定夹取成功
        # 当前简化：默认成功
        if target_positions:
            self._state.holding_ids = set(target_positions.keys())
            self._state.holding_count = len(target_positions)
            self._state.action = GripperAction.HOLD

        return True

    def release(self) -> bool:
        return self.open()

    def is_holding(self) -> bool:
        return self._state.holding_count > 0

    def _set_angle(self, angle: float) -> None:
        """设置舵机角度（需硬件 PWM 实现）"""
        self._current_angle = angle
        # 实际 PWM 控制代码
        # duty = (angle / 18.0) + 2.5  # 标准舵机 0-180°

    def cleanup(self) -> None:
        self.open()
