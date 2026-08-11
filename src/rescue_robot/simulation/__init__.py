"""
2D 仿真包 —— 板块 9: 开发工具链与测试

提供:
  - Sim2D: 2D 仿真引擎（场地 + 机器人运动 + 硬件状态模拟）
  - HardwareState: 硬件实时状态
  - MatchState: 比赛状态
"""

from .sim_2d import Sim2D, HardwareState, MatchState, RobotPose, TargetInfo

__all__ = ["Sim2D", "HardwareState", "MatchState", "RobotPose", "TargetInfo"]
