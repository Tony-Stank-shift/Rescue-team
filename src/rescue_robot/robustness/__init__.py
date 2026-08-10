"""
robustness —— 系统健壮性包

板块 8 实现，包含三大子模块：
  fault_tolerance  — 8.1 故障容错（传感器降级、电机故障、通信中断、电源监控）
  stability        — 8.2 稳定性（多场比赛、内存管理、启动成功率）
  logging_system   — 8.3 日志与复盘（结构化日志、事件记录、本地存储）
"""

from .fault_tolerance import (
    SensorHealthMonitor,
    MotorFaultDetector,
    MotorStatus,
    CommWatchdog,
    PowerMonitor,
    PowerStatus,
    SensorHealthReport,
    DegradationLevel,
    SensorID,
    SensorStatus,
    SystemHealthReport,
    MotorFaultReport,
    PowerReport,
)
from .stability import (
    MatchStabilityTracker,
    MemoryMonitor,
    BootSuccessTracker,
    MatchRecord,
    MemoryReport,
)
from .logging_system import (
    StructuredLogger,
    EventRecorder,
    EventType,
    LogStorage,
)

__all__ = [
    # fault_tolerance
    "SensorHealthMonitor", "MotorFaultDetector", "MotorStatus",
    "CommWatchdog", "PowerMonitor", "PowerStatus",
    "SensorHealthReport", "DegradationLevel",
    "SensorID", "SensorStatus", "SystemHealthReport",
    "MotorFaultReport", "PowerReport",
    # stability
    "MatchStabilityTracker", "MemoryMonitor", "BootSuccessTracker",
    "MatchRecord", "MemoryReport",
    # logging_system
    "StructuredLogger", "EventRecorder", "EventType", "LogStorage",
]
