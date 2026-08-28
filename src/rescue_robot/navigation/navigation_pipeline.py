"""
navigation_pipeline.py —— 主导航管线

整合定位、路径规划、运动控制、禁区管理，提供统一导航接口：
  navigate_to(target) → 规划路径 → 跟踪路径 → 到达判定

对接 autonomous_state 主循环：
  nav = NavigationPipeline(field_layout, my_color=RED)
  ...
  cmd = nav.update(target, current_pose=None)  # 返回速度指令
"""

import logging
import math
import time
from typing import List, Optional, Tuple

from .localization import AbstractLocalizer, MockLocalizer, RobotPose
from .path_planner import (
    CostMap, AStarPlanner, LocalPlanner, PlanResult, Point,
    CELL_SIZE_MM, GRID_SIZE, COST_OBSTACLE, COST_OPPONENT,
)
from .motion_control import MotionController, VelocityCommand
from .forbidden_zones import ForbiddenZoneManager, ForbiddenZone
from ..perception.field_elements import FieldLayout, SafeZoneColor

logger = logging.getLogger("navigation_pipeline")

from enum import Enum, auto


class NavState(Enum):
    """导航状态"""
    IDLE = auto()            # 空闲
    PLANNING = auto()        # 规划路径中
    MOVING = auto()          # 沿路径移动中
    AVOIDING = auto()        # 局部避障中
    BUMP_CROSSING = auto()   # 越障中
    ARRIVED = auto()         # 已到达
    STUCK = auto()           # 被困
    BLOCKED = auto()         # 路径被阻断


class NavigationPipeline:
    """
    主导航管线。

    使用方式：
      nav = NavigationPipeline(field_layout)
      nav.set_target(target_x, target_y)

      # 每帧调用
      cmd = nav.update(pose)  # → VelocityCommand

      if nav.is_arrived():
          # 到达目标
    """

    def __init__(self,
                 field_layout: FieldLayout,
                 my_color: SafeZoneColor = SafeZoneColor.RED,
                 use_mock: bool = True):
        """
        Args:
            field_layout: 场地布局
            my_color: 本队安全区颜色
            use_mock: True=Mock 定位器
        """
        # 子模块
        if use_mock:
            self._localizer: AbstractLocalizer = MockLocalizer()
        else:
            from .localization import OdometryLocalizer
            self._localizer: AbstractLocalizer = OdometryLocalizer()

        self._cost_map = CostMap()
        self._astar = AStarPlanner(self._cost_map)
        self._local_planner = LocalPlanner()
        self._motion = MotionController()
        self._forbidden = ForbiddenZoneManager(field_layout, my_color)

        # 状态
        self._state = NavState.IDLE
        self._target: Optional[Point] = None
        self._current_path: List[Point] = []
        self._plan_result: Optional[PlanResult] = None
        self._replan_counter = 0
        self._replan_interval = 30  # 每 30 帧（0.6s）重规划一次
        self._close_range_mm = 150.0  # 接近段：距目标小于此值时直接精确接近

        # 统计
        self._total_distance = 0.0
        self._frame_count = 0

        # 将禁区写入 CostMap
        self._forbidden.write_to_cost_map(self._cost_map)

        logger.info(f"NavigationPipeline 初始化: mock={use_mock}, "
                     f"my_color={my_color.name}")

    # ---- 属性 ----

    @property
    def state(self) -> NavState:
        return self._state

    @property
    def pose(self) -> RobotPose:
        return self._localizer.pose

    @property
    def target(self) -> Optional[Point]:
        return self._target

    @property
    def current_path(self) -> List[Point]:
        return self._current_path

    @property
    def forbidden(self) -> ForbiddenZoneManager:
        return self._forbidden

    # ---- 目标设置 ----

    def set_target(self, x: float, y: float) -> None:
        """设置导航目标"""
        self._target = (x, y)
        self._state = NavState.PLANNING
        logger.info(f"新导航目标: ({x:.0f}, {y:.0f})")

    def clear_target(self) -> None:
        self._target = None
        self._current_path.clear()
        self._state = NavState.IDLE

    # ---- 主循环 ----

    def update(self,
               current_pose: Optional[Tuple[float, float, float]] = None,
               opponent_position: Optional[Point] = None,
               near_speed_bump: bool = False,
               dt: float = 0.02) -> VelocityCommand:
        """
        单帧导航更新。

        Args:
            current_pose: (x, y, theta)，None 则从 localizer 获取
            opponent_position: 对方机器人位置（用于动态避障）
            near_speed_bump: 是否接近减速带
            dt: 时间步长

        Returns:
            VelocityCommand: 速度指令
        """
        self._frame_count += 1

        # 获取当前位姿
        if current_pose is None:
            pose = self._localizer.pose
            current_pose = (pose.x, pose.y, pose.theta)

        # 无目标 → 停止
        if self._target is None:
            return VelocityCommand(linear=0.0, angular=0.0, timestamp=time.time())

        # 越障处理
        if near_speed_bump and not self._motion.is_bump_mode:
            self._motion.enter_bump_mode()
            self._state = NavState.BUMP_CROSSING

        if self._motion.is_bump_mode:
            cmd = self._motion.compute_bump_velocity(dt)
            self._localizer.update(cmd.linear, cmd.angular, dt)
            if self._motion._bump_count >= 3:
                self._motion.exit_bump_mode()
                self._state = NavState.MOVING
            return cmd

        # 更新动态障碍（对方机器人）
        self._cost_map.clear_dynamic()
        if opponent_position:
            self._cost_map.add_obstacle_circle(
                opponent_position[0], opponent_position[1],
                radius_mm=350, cost=COST_OPPONENT,
            )

        # 接近段：距目标很近时直接精确接近，跳过 A* 重规划 / 纯追踪 / prune，
        # 避免这些环节在目标附近的抖动导致"路径空 + 速度归零"卡死。
        dist_to_target = math.hypot(
            current_pose[0] - self._target[0],
            current_pose[1] - self._target[1],
        )
        if dist_to_target < self._close_range_mm:
            if self._motion.is_at_target(self._target, current_pose):
                self._state = NavState.ARRIVED
                return VelocityCommand(linear=0.0, angular=0.0, timestamp=time.time())
            cmd = self._motion.compute_velocity(self._target, current_pose, dt=dt)
            self._total_distance += abs(cmd.linear) * dt
            self._localizer.update(cmd.linear, cmd.angular, dt)
            self._state = NavState.MOVING
            return cmd

        # 重规划
        need_replan = (
            self._state == NavState.PLANNING or
            self._state == NavState.BLOCKED or
            (self._replan_counter >= self._replan_interval and len(self._current_path) < 5)
        )

        if need_replan and self._target:
            self._replan_counter = 0
            plan = self._astar.plan(
                (current_pose[0], current_pose[1]),
                self._target,
            )
            if plan.success:
                self._current_path = plan.waypoints
                self._plan_result = plan
                self._state = NavState.MOVING
            else:
                self._state = NavState.BLOCKED
                logger.warning("路径规划失败 — 无可行路径")
                return VelocityCommand(linear=0.0, angular=0.0, timestamp=time.time())

        self._replan_counter += 1

        # 路径跟踪
        if self._current_path:
            self._prune_path(current_pose)

            if not self._current_path:
                if self._motion.is_at_target(self._target, current_pose):
                    self._state = NavState.ARRIVED
                    logger.debug("到达目标!")
                    return VelocityCommand(linear=0.0, angular=0.0, timestamp=time.time())
                # 路径已被 prune 空但尚未到达目标（最后一段精确接近）：
                # 直接朝目标点做位置控制，避免 track_path(空路径) 返回零速度而卡死。
                cmd = self._motion.compute_velocity(self._target, current_pose, dt=dt)
                self._total_distance += abs(cmd.linear) * dt
                self._localizer.update(cmd.linear, cmd.angular, dt)
                return cmd

            # 纯追踪
            cmd = self._motion.track_path(self._current_path, current_pose, dt=dt)

            # 局部避障修正
            if self._is_near_obstacle(current_pose):
                best_vw = self._local_planner.plan(
                    current_pose, (cmd.linear, cmd.angular),
                    self._current_path, self._cost_map,
                )
                cmd = VelocityCommand(
                    linear=best_vw[0], angular=best_vw[1],
                    timestamp=time.time(),
                )
                self._state = NavState.AVOIDING
            else:
                self._state = NavState.MOVING

            self._total_distance += abs(cmd.linear) * dt
            self._localizer.update(cmd.linear, cmd.angular, dt)

            # 禁区检查
            violation = self._forbidden.check_violation(
                current_pose[0], current_pose[1]
            )
            if violation:
                logger.warning(f"⚠️ 进入禁区: {violation.name} — {violation.penalty}")
                return VelocityCommand(linear=-200.0, angular=0.0, timestamp=time.time())

            return cmd

        return VelocityCommand(linear=0.0, angular=0.0, timestamp=time.time())

    # ---- 重定位 ----

    def reset_pose(self, x: float, y: float, theta: float) -> None:
        """重置位姿（强制分离后）"""
        self._localizer.reset_pose(x, y, theta)
        self._state = NavState.PLANNING
        logger.info(f"导航重定位: ({x:.0f}, {y:.0f}), "
                     f"heading={math.degrees(theta):.0f}°")

    # ---- 状态查询 ----

    def is_arrived(self) -> bool:
        return self._state == NavState.ARRIVED

    def is_blocked(self) -> bool:
        return self._state == NavState.BLOCKED

    def distance_to_target(self,
                           current_pose: Optional[Tuple[float, float, float]] = None) -> float:
        if self._target is None:
            return 0.0
        if current_pose is None:
            cx, cy = self._localizer.pose.x, self._localizer.pose.y
        else:
            cx, cy = current_pose[0], current_pose[1]
        tx, ty = self._target
        return math.sqrt((tx - cx) ** 2 + (ty - cy) ** 2)

    def get_progress(self) -> dict:
        return {
            "state": self._state.name,
            "target": self._target,
            "path_length": len(self._current_path),
            "distance_to_target": self.distance_to_target(),
            "total_distance": self._total_distance,
            "frame_count": self._frame_count,
        }

    # ---- 内部 ----

    def _prune_path(self, current_pose: Tuple[float, float, float]) -> None:
        cx, cy, _ = current_pose
        prune_threshold = 80.0  # mm
        while self._current_path:
            wx, wy = self._current_path[0]
            if math.sqrt((cx - wx) ** 2 + (cy - wy) ** 2) < prune_threshold:
                self._current_path.pop(0)
            else:
                break

    def _is_near_obstacle(self, current_pose: Tuple[float, float, float]) -> bool:
        cx, cy, _ = current_pose
        gx = int(cx / CELL_SIZE_MM)
        gy = int(cy / CELL_SIZE_MM)
        for dx in range(-2, 3):
            for dy in range(-2, 3):
                nx, ny = gx + dx, gy + dy
                if 0 <= nx < GRID_SIZE and 0 <= ny < GRID_SIZE:
                    if self._cost_map._grid[ny][nx] >= COST_OBSTACLE:
                        return True
        return False

    def explore(self, robot_pose=None) -> None:
        """探索模式：驶向场地随机位置"""
        import random
        if robot_pose is None:
            robot_pose = (self._localizer.pose.x, self._localizer.pose.y)
        rx, ry = robot_pose[0], robot_pose[1]
        tx = random.randint(300, 2700)
        ty = random.randint(300, 2200)
        self.set_target(tx, ty)
        logger.info("探索模式: target=(%d, %d)", tx, ty)

    def survival_circle(self, robot_pose=None) -> None:
        """保命模式：在原地附近做小圈运动"""
        import math, time
        if robot_pose is None:
            robot_pose = (self._localizer.pose.x, self._localizer.pose.y)
        rx, ry = robot_pose[0], robot_pose[1]
        radius = 400
        angle = (time.time() * 0.5) % (2 * math.pi)
        tx = rx + radius * math.cos(angle)
        ty = ry + radius * math.sin(angle)
        tx = max(200, min(2800, tx))
        ty = max(200, min(2800, ty))
        self.set_target(tx, ty)

    def summary(self) -> str:
        return (
            f"导航管线: state={self._state.name}, "
            f"target={self._target}, "
            f"path_waypoints={len(self._current_path)}, "
            f"dist_to_target={self.distance_to_target():.0f}mm"
        )


# ============================================================
# 独立测试
# ============================================================

if __name__ == "__main__":
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s [%(levelname)-7s] %(name)s: %(message)s",
        datefmt="%H:%M:%S",
    )

    print("=" * 50)
    print("  导航管线 — Mock 模式测试")
    print("=" * 50)

    field = FieldLayout.standard()
    nav = NavigationPipeline(field, use_mock=True)

    # 测试 1：水平穿越
    start = (500.0, 1500.0)
    goal = (2500.0, 1500.0)
    nav.reset_pose(start[0], start[1], 0.0)
    nav.set_target(goal[0], goal[1])

    print(f"\n测试 1: 水平穿越 {start} → {goal}")
    print(f"初始: {nav.summary()}")

    for i in range(300):
        cmd = nav.update()
        if nav.is_arrived():
            print(f"  ✅ 到达目标! (帧 #{i}/{i*0.02:.1f}s)")
            break
        if i % 100 == 0:
            pose = nav.pose
            print(f"  帧#{i}: pos=({pose.x:.0f}, {pose.y:.0f}), "
                  f"v={cmd.linear:.0f}mm/s, dist={nav.distance_to_target():.0f}mm")

    print(f"最终: {nav.summary()}")
    print(f"进度: {nav.get_progress()}")

    # 测试 2：禁区测试
    print(f"\n测试 2: 禁区检查")
    print(nav.forbidden.summary())
    # 检查对方安全区
    blue_safe_x, blue_safe_y = 2500, 2800  # 蓝色安全区位置
    safe_check = nav.forbidden.is_safe(blue_safe_x, blue_safe_y)
    print(f"  蓝色安全区 ({blue_safe_x}, {blue_safe_y}) 安全: {safe_check}")
    open_field = nav.forbidden.is_safe(1500, 1500)
    print(f"  场地中央 (1500, 1500) 安全: {open_field}")

    # 测试 3：重定位
    print(f"\n测试 3: 重定位（模拟强制分离）")
    nav.reset_pose(300, 2700, math.pi / 2)
    print(f"  位姿: ({nav.pose.x:.0f}, {nav.pose.y:.0f}), "
          f"heading={nav.pose.heading_deg:.0f}°")

    print("\n✅ 导航管线测试完成")
