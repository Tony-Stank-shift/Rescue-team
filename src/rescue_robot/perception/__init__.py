"""
perception —— 感知模块

救援机器人的完整感知管线：
  摄像头 → 检测 → 分类 → 位置估算 → 世界地图 → 对方跟踪

子模块：
  target_types      — 目标类型定义与配置表
  detection         — 目标检测器（CV + Mock）
  classification    — 目标分类器
  world_map         — 动态世界地图
  field_elements    — 场地元素定义
  opponent_tracker  — 对方机器人跟踪
  sensor_fusion     — 多传感器融合
  perception_pipeline — 主感知管线
"""

from .target_types import (
    TargetType, TargetShape, TargetColor, TargetStatus,
    TargetInfo, CompetitionPhase,
    DetectedTarget, Detection,
    PRELIMINARY_TARGETS, FINAL_TARGETS,
    get_target_config, get_point_value,
)
from .detection import AbstractDetector, CVDetector, MockDetector
from .classification import TargetClassifier
from .world_map import WorldMap, TrackedTarget
from .field_elements import (
    FieldLayout, FieldElement, FieldElementType,
    SafeZoneColor, StandardFieldLayout, RectRegion,
)
from .opponent_tracker import OpponentTracker, OpponentState
from .sensor_fusion import SensorFusion, FusedObservation, IMUReading, OdomReading
from .perception_pipeline import PerceptionPipeline

__all__ = [
    # types
    "TargetType", "TargetShape", "TargetColor", "TargetStatus",
    "TargetInfo", "CompetitionPhase",
    "DetectedTarget", "Detection",
    "PRELIMINARY_TARGETS", "FINAL_TARGETS",
    "get_target_config", "get_point_value",
    # detection
    "AbstractDetector", "CVDetector", "MockDetector",
    # classification
    "TargetClassifier",
    # world map
    "WorldMap", "TrackedTarget",
    # field
    "FieldLayout", "FieldElement", "FieldElementType",
    "SafeZoneColor", "StandardFieldLayout", "RectRegion",
    # opponent
    "OpponentTracker", "OpponentState",
    # sensor fusion
    "SensorFusion", "FusedObservation", "IMUReading", "OdomReading",
    # pipeline
    "PerceptionPipeline",
]
