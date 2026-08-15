"""
navigation —— 自主导航模块

救援机器人的完整导航系统：
  定位 → 路径规划 → 运动控制 → 禁区管理

子模块：
  localization       — 位姿估计（里程计 + IMU）
  path_planner       — A*全局规划 + 局部避障
  motion_control     — PID 运动控制
  forbidden_zones    — 禁区管理
  navigation_pipeline — 主导航管线
"""

from .localization import (
    AbstractLocalizer, MockLocalizer, OdometryLocalizer, RobotPose,
)
from .path_planner import (
    CostMap, AStarPlanner, LocalPlanner, PlanResult, Point,
)
from .motion_control import (
    MotionController, VelocityCommand, PIDController,
)
from .omni_kinematics import OmniDriveKinematics
from .forbidden_zones import (
    ForbiddenZoneManager, ForbiddenZone,
)
from .navigation_pipeline import (
    NavigationPipeline, NavState,
)

__all__ = [
    "AbstractLocalizer", "MockLocalizer", "OdometryLocalizer", "RobotPose",
    "CostMap", "AStarPlanner", "LocalPlanner", "PlanResult", "Point",
    "MotionController", "VelocityCommand", "PIDController",
    "OmniDriveKinematics",
    "ForbiddenZoneManager", "ForbiddenZone",
    "NavigationPipeline", "NavState",
]
