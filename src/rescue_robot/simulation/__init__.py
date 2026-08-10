"""
rescue_robot.simulation —— MuJoCo 3D 比赛仿真 (板块 9)

为救援机器人竞赛提供完整的 3D 物理仿真环境。
支持 GUI 交互查看和 headless 自动化测试。

核心组件:
  - SimWorld: 仿真世界管理器（主循环 + 物理 + 计分）
  - SimField: 3D 比赛场地（围栏/安全区/减速带/出发区）
  - SimRobot: 我方机器人（差分驱动 + 推板）
  - SimTarget: 救援目标（8 种 3D 形状 + 物理属性）
  - SimRunner: CLI 启动器（GUI/headless）

快速开始:
  # GUI 交互模式
  python -m rescue_robot.simulation.sim_runner --mode gui

  # Headless 测试
  python -m rescue_robot.simulation.sim_runner --mode headless --duration 30

  # Python API
  from rescue_robot.simulation import SimWorld
  world = SimWorld(mode="headless", seed=42)
  world.setup_match(target_count=20)
  while not world.is_done():
      world.step()
  result = world.get_match_stats()
  world.close()
"""

from .sim_world import SimWorld, MatchState, MatchResult
from .sim_field import SimField
from .sim_robot import SimRobot
from .sim_target import SimTarget, TargetFactory
from .sim_models import COLORS, mm_to_m, m_to_mm, rgba
from .sim_runner import run_simulation

__all__ = [
    # World
    "SimWorld",
    "MatchState",
    "MatchResult",
    # Entities
    "SimField",
    "SimRobot",
    "SimTarget",
    "TargetFactory",
    # Utils
    "COLORS",
    "mm_to_m",
    "m_to_mm",
    "rgba",
    "run_simulation",
]
