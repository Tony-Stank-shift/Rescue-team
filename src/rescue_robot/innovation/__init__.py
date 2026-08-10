"""
innovation —— 创新实践适配 (板块 7)

决赛现场编程调试全流程支撑。

子模块:
  config_loader    (7.1.1)  参数化 YAML/JSON 配置系统
  model_switcher   (7.1.2)  初赛/决赛模型一键切换
  hardware_profile (7.1.3)  硬件抽象层 + 标定流程
  hot_reloader     (7.1.4)  策略参数热更新
  deploy           (7.2.1)  快速部署流水线
  debug_dashboard  (7.2.2)  调试可视化工具
"""

from .config_loader import (
    ConfigLoader,
    FieldConfig,
    RectZone,
    RobotConfig,
    SafeZoneConfig,
)

from .deploy import (
    CheckResult,
    Deployer,
    DeployReport,
)

from .hardware_profile import (
    CalibrationRoutine,
    CameraParams,
    ChassisParams,
    ComponentAdapter,
    HardwareProfile,
    IMUParams,
    MotorParams,
    SensorCalibration,
)

from .hot_reloader import (
    HotReloader,
    get_global_reloader,
)

from .model_switcher import (
    ModelSwitcher,
)

from .debug_dashboard import (
    DebugDashboard,
)

__all__ = [
    # config_loader
    "ConfigLoader",
    "RobotConfig",
    "FieldConfig",
    "RectZone",
    "SafeZoneConfig",
    #
    # model_switcher
    "ModelSwitcher",
    # hardware_profile
    "HardwareProfile",
    "MotorParams",
    "ChassisParams",
    "IMUParams",
    "CameraParams",
    "SensorCalibration",
    "CalibrationRoutine",
    "ComponentAdapter",
    # hot_reloader
    "HotReloader",
    "get_global_reloader",
    # deploy
    "Deployer",
    "DeployReport",
    "CheckResult",
    # debug_dashboard
    "DebugDashboard",
]
