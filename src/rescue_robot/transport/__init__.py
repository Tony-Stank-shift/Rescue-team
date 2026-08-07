"""
transport —— 转运控制模块

夹爪控制 + 装载管理 + 安全区投放判定。

子模块：
  gripper             — 夹爪控制（Mock / 舵机）
  load_manager        — 装载计数 + 规则校验
  safe_zone_placer    — 安全区投放判定
  transport_pipeline  — 转运主控管线
"""

from .gripper import (
    AbstractGripper, MockGripper, ServoGripper,
    GripperAction, GripperState,
)
from .load_manager import (
    LoadManager, LoadState, Violation,
    VIOLATION_CONSEQUENCES,
)
from .safe_zone_placer import (
    SafeZonePlacer, PlacementResult, PlacementZone,
)
from .transport_pipeline import (
    TransportPipeline, TransportPhase, TransportStatus,
)

__all__ = [
    "AbstractGripper", "MockGripper", "ServoGripper",
    "GripperAction", "GripperState",
    "LoadManager", "LoadState", "Violation", "VIOLATION_CONSEQUENCES",
    "SafeZonePlacer", "PlacementResult", "PlacementZone",
    "TransportPipeline", "TransportPhase", "TransportStatus",
]
