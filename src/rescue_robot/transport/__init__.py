"""
transport —— 转运控制模块

套取机构控制 + 装载管理 + 安全区投放判定。

子模块：
  sleeve_lift         — 升降套取机构（Mock / 丝杆）
  load_manager        — 装载计数 + 规则校验
  safe_zone_placer    — 安全区投放判定
  transport_pipeline  — 转运主控管线
"""

from .sleeve_lift import (
    AbstractSleeveLift, MockSleeveLift, ScrewSleeveLift,
    SleeveAction, SleeveState,
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
    "AbstractSleeveLift", "MockSleeveLift", "ScrewSleeveLift",
    "SleeveAction", "SleeveState",
    "LoadManager", "LoadState", "Violation", "VIOLATION_CONSEQUENCES",
    "SafeZonePlacer", "PlacementResult", "PlacementZone",
    "TransportPipeline", "TransportPhase", "TransportStatus",
]
