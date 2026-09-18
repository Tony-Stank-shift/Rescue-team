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
from ..perception.field_elements import FIELD_SIZE, FieldLayout, SafeZoneColor

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
        self._rejected_targets = 0   # 越界/非法目标被拒绝的次数（诊断用）
        self._bump_done = False      # 本次减速带越障是否已完成（防止同一次越障里反复重入）
        # T1-4：可选的"到位朝向"要求。投放落点 = 车心 + L·朝向，朝向错则落点错
        # （投歪 -10 分/个）。置位后：位置到位但朝向不对 → 原地对准，不算到达。
        self._final_heading: Optional[float] = None
        # T2-2：BLOCKED 退避重规划的时间戳（避免每帧全空间 A* + 刷屏）
        self._blocked_last_plan_ts = 0.0
        self._blocked_warn_ts = 0.0
        self._caution_warn_ts = 0.0     # T1-15 预警日志限频
        self._replan_counter = 0
        self._replan_interval = 30  # 每 30 帧（0.6s）重规划一次
        self._close_range_mm = 150.0  # 接近段：距目标小于此值时直接精确接近

        # 被障碍包住时的脱离计数器（见 `_escape_from_obstacle`）
        self._escape_frames = 0

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

    def set_target(self, x: float, y: float) -> bool:
        """设置导航目标。

        两道闸门（顺序不能换）：
          ① **越界拒绝**：目标点在场地之外（<0 或 >3000）→ 直接拒绝，不设置、打 WARNING。
             为什么不能"夹紧"：`CostMap._to_grid` 会把越界坐标 clamp 进网格 → A* 返回
             success=True → 上层以为规划成功、导航判"到达"，而 `is_in_field()` 为 False
             （test-author 评审发现的 S-NEW）。静默夹紧会把"给了个场外目标"变成
             "看起来正常但永远走不到"的幽灵任务。
          ② 禁区钳制：落在 hard 禁区（对方安全区 / 边界安全带）内的目标点，钳制到最近合法点
             —— 否则车会径直开进对方安全区（赛项：进入对方安全区 → 比赛结束）。

        Returns:
            True=已设置（可能被钳制）；False=被拒绝（越界），原目标保持不变
        """
        if not self._forbidden.is_in_field(x, y):
            self._rejected_targets += 1
            logger.warning(
                f"⚠️ 导航目标越界 ({x:.0f}, {y:.0f}) 在场地外 → 拒绝设置"
                f"（保持原目标 {self._target}）；累计拒绝 {self._rejected_targets} 次"
            )
            return False

        clamped = self._forbidden.clamp_to_safe(x, y)
        if clamped is None:
            # T1-2：钳制找不到合法替代点时**明确拒绝**（旧实现静默返回场地中心）
            self._rejected_targets += 1
            logger.warning(f"⚠️ 导航目标 ({x:.0f},{y:.0f}) 在禁区内且无合法替代点 "
                           f"→ 拒绝设置；累计拒绝 {self._rejected_targets} 次")
            return False
        safe_x, safe_y = clamped
        if (safe_x, safe_y) != (x, y):
            logger.warning(f"⚠️ 导航目标 ({x:.0f}, {y:.0f}) 落在禁区内 → "
                           f"钳制到 ({safe_x:.0f}, {safe_y:.0f})")

        # ── ③ 可达性闸门（T1-1）──
        # ⚠️ 只判"在场地内"是不够的：最外圈 50mm 是"边界安全带"（costmap 写满 255），
        # 于是 `set_target(2999,1500)` 会返回 True，而 A* **永远规划失败** →
        # 导航每帧重规划、每帧零速度、`is_arrived()` 永远 False（实测 60 帧
        # state=BLOCKED cmd=(0,0)）→ 正是本文件 docstring 说要消灭的"幽灵任务"。
        # 这里再加一道"必须落在可通行格"的校验，并把点朝场地内侧拉回来。
        if not self._cost_map.is_free(safe_x, safe_y):
            pulled = self._pull_to_traversable(safe_x, safe_y)
            if pulled is None:
                self._rejected_targets += 1
                logger.warning(
                    f"⚠️ 导航目标 ({safe_x:.0f},{safe_y:.0f}) 不可通行（A* 永远到不了）"
                    f"且无法拉回 → 拒绝设置；累计拒绝 {self._rejected_targets} 次")
                return False
            logger.warning(f"⚠️ 导航目标 ({safe_x:.0f},{safe_y:.0f}) 落在不可通行区"
                           f"（边界安全带）→ 拉回 ({pulled[0]:.0f},{pulled[1]:.0f})")
            safe_x, safe_y = pulled

        self._target = (safe_x, safe_y)
        self._state = NavState.PLANNING
        logger.info(f"新导航目标: ({safe_x:.0f}, {safe_y:.0f})")
        return True

    #: 把不可通行的目标点朝场地中心拉回时，每次的步长与最大尝试距离（mm）
    #: 硬禁区预警带距离（mm）：进入后主动减速（T1-15）。
    CAUTION_DIST_MM = 150.0
    #: 预警带内的速度比例。
    CAUTION_SPEED_RATIO = 0.5

    # ---- 被障碍包住时的果断脱离（2026-09-18 现场：2mm/s 干蹭十几秒）--------
    #: 局部规划器输出低于此速度（mm/s）即视为"规划器冻住"（不是真的想慢走）。
    STUCK_CMD_MM_S = 40.0
    #: 脱离动作的速度（mm/s，负=倒车）
    ESCAPE_SPEED_MM_S = 180.0
    #: 连续脱离帧数上限（50Hz 下 150 帧 ≈ 3s、约 500mm）。
    #: 超过就交回规划器 —— 防止"障碍其实是真实对手"时一路倒出场。
    ESCAPE_MAX_FRAMES = 150

    #: BLOCKED 状态下的最小重规划间隔（秒）。T2-2：旧实现每帧重规划（最坏 15.3ms/帧）。
    BLOCKED_REPLAN_S = 0.5
    PULL_STEP_MM = 25.0
    PULL_MAX_MM = 300.0

    #: 把不可通行的目标点拉回时可选的 8 个方向（先近后远、先正向后对角）
    _PULL_DIRS = ((1, 0), (-1, 0), (0, 1), (0, -1),
                  (0.7071, 0.7071), (0.7071, -0.7071),
                  (-0.7071, 0.7071), (-0.7071, -0.7071))

    def _pull_to_traversable(self, x: float, y: float):
        """把不可通行的点拉到**最近的可通行格**（T1-1）。

        ⚠️ 不能只朝"场地中心"方向拉：若该点紧贴一块大禁区（例如贴着对方安全区
        外侧的边界带），朝中心走会**穿过更大的禁区** → 永远拉不到 → 只能拒绝，
        而拒绝会让上层拿着上一次的陈旧目标继续跑。改为在 8 个方向上做
        "由近及远"的最近可通行点搜索。

        Returns:
            (x, y) 可通行的点；找不到则 None（调用方**明确拒绝**该目标）。
        """
        steps = max(1, int(self.PULL_MAX_MM / self.PULL_STEP_MM))
        best = None
        best_d = None
        for i in range(1, steps + 1):
            r = self.PULL_STEP_MM * i
            for ux, uy in self._PULL_DIRS:
                px, py = x + ux * r, y + uy * r
                if not self._forbidden.is_in_field(px, py):
                    continue
                if self._forbidden.check_violation(px, py) is not None:
                    continue
                if self._cost_map.is_free(px, py):
                    d = (px - x) ** 2 + (py - y) ** 2
                    if best_d is None or d < best_d:
                        best_d, best = d, (px, py)
            if best is not None:
                return best          # 由近及远：当前半径上已找到最近解
        return None

    def _apply_caution(self, cmd, current_pose):
        """硬禁区**预警带**内主动减速（T1-15）。

        用现成的 `ForbiddenZoneManager.get_violation_warning()`（距硬禁区 150mm 内），
        把速度压到 `CAUTION_SPEED_RATIO`。旧实现该接口零调用 → 只有"已经踩进去"的
        事后倒车，罚分已经发生。
        """
        try:
            zone = self._forbidden.get_violation_warning(
                current_pose[0], current_pose[1], self.CAUTION_DIST_MM)
        except Exception:
            zone = None
        if zone is None:
            return cmd
        now = time.time()
        if now - self._caution_warn_ts >= 1.0:
            self._caution_warn_ts = now
            logger.warning(f"⚠️ 接近硬禁区 {zone.name}（<{self.CAUTION_DIST_MM:.0f}mm）"
                           f"→ 主动减速到 {self.CAUTION_SPEED_RATIO*100:.0f}%")
        return VelocityCommand(linear=cmd.linear * self.CAUTION_SPEED_RATIO,
                               angular=cmd.angular,
                               timestamp=now)

    def _bump_heading_correction(self, current_pose, dt: float = 0.02) -> float:
        """越障期间的朝向修正（rad/s）——朝当前导航目标纠偏，限幅在 motion 侧。

        旧实现越障时 `angular=0`，全程不看目标：车头朝哪就朝哪冲 600mm，
        可能撞围栏/骑上减速带/直入对方安全区。这里给一个小幅纠偏即可，
        真正急转留给越障结束后的正常导航。
        """
        if self._target is None:
            return 0.0
        x, y, theta = current_pose[0], current_pose[1], current_pose[2]
        dx, dy = self._target[0] - x, self._target[1] - y
        if (dx * dx + dy * dy) ** 0.5 < 1.0:
            return 0.0
        err = math.atan2(dy, dx) - theta
        # 归一到 [-pi, pi]
        while err > math.pi:
            err -= 2 * math.pi
        while err < -math.pi:
            err += 2 * math.pi
        return 0.6 * err          # 比例系数取小值：只做温和纠偏

    def require_final_heading(self, theta_rad: float) -> None:
        """要求"到位时朝向也对准 theta_rad"（T1-4）。

        为什么需要：`is_at_target()` 只看位置，而 `compute_velocity()` 在
        `distance < POSITION_TOLERANCE_MM` 时**无条件返回零速** → "到达"可以在朝向
        完全不对时成立。投放落点 = 车心 + L·朝向（`transport.drop_position`），
        朝向错 → 落点错 → 投歪（-10 分/个）。转运管线在接近投放点时置位本要求。
        """
        self._final_heading = float(theta_rad)

    def clear_final_heading(self) -> None:
        self._final_heading = None

    def _aligned(self, current_pose) -> bool:
        """朝向是否已满足要求（未要求时恒 True）。"""
        if self._final_heading is None:
            return True
        err = self._final_heading - current_pose[2]
        while err > math.pi:
            err -= 2 * math.pi
        while err < -math.pi:
            err += 2 * math.pi
        return abs(err) <= self._motion.ANGLE_TOLERANCE_RAD

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

        # ── 硬禁区检查（对方安全区 / 场地边界）──
        # ⚠️ 必须在**所有分支之前**统一执行。旧实现只在"路径跟踪"分支里查，
        # 于是"接近段(dist < close_range)"与"到达"两个分支完全绕过检查：
        # 只要目标点落在对方安全区附近，车就会直接开进去 → 赛项判违规/比赛结束。
        violation = self._forbidden.check_violation(current_pose[0], current_pose[1])
        if violation is not None:
            logger.warning(f"⚠️ 进入禁区: {violation.name} — {violation.penalty}")
            back_cmd = VelocityCommand(linear=-200.0, angular=0.0, timestamp=time.time())
            self._localizer.update(back_cmd.linear, back_cmd.angular, dt)
            self._state = NavState.AVOIDING
            return back_cmd

        # 无目标 → 停止
        if self._target is None:
            return VelocityCommand(linear=0.0, angular=0.0, timestamp=time.time())

        # ── 越障处理（T0-4 已修）──
        # 旧实现三个问题：
        #   ① 退出判据是**墙钟** `int((now-_bump_timer)/1.0) >= 3`：主循环被套取/投放
        #      阻塞（单次 0.9s，重试可达 6s）时墙钟照走 → 还没跨过减速带就退出；
        #   ② `enter_bump_mode()` 把计时清零，而只要 `near_speed_bump` 仍为 True 就会
        #      下一帧**立刻重入** → 反复直线冲（每轮 200mm/s × 3s = 600mm），
        #      实测 10 秒内从 (300,300) 直冲到 (300,2300)，完全无视目标 (1500,1500)；
        #   ③ 越障期间 `angular=0` → 全程不看目标方向，车头朝哪就朝哪冲。
        # 现在：按**里程**（`motion.bump_finished`，含最长时长兜底）判退出；
        #       加"本次已完成"闩锁（离开减速带区才解除），不再同一次越障里重入；
        #       越障期间保留**限幅**的朝向修正，朝当前目标纠偏。
        if not near_speed_bump:
            self._bump_done = False         # 离开减速带区 → 解除闩锁，允许下次再进
        elif not self._motion.is_bump_mode and not self._bump_done:
            self._motion.enter_bump_mode()
            self._state = NavState.BUMP_CROSSING

        if self._motion.is_bump_mode:
            cmd = self._motion.compute_bump_velocity(
                dt, angular=self._bump_heading_correction(current_pose, dt))
            self._localizer.update(cmd.linear, cmd.angular, dt)
            if self._motion.bump_finished:
                self._motion.exit_bump_mode()
                self._bump_done = True      # 闩锁：同一次越障不重入
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
            if (self._motion.is_at_target(self._target, current_pose)
                    and self._aligned(current_pose)):
                self._state = NavState.ARRIVED
                return VelocityCommand(linear=0.0, angular=0.0, timestamp=time.time())
            cmd = self._motion.compute_velocity(self._target, current_pose, dt=dt,
                                                align_heading=self._final_heading)
            self._total_distance += abs(cmd.linear) * dt
            self._localizer.update(cmd.linear, cmd.angular, dt)
            self._state = NavState.MOVING
            return cmd

        # ── T2-2：BLOCKED 退避 ──
        # 旧实现：`need_replan` 含 `state == BLOCKED`，而规划失败后又把状态设回 BLOCKED
        # → **每帧都跑一次全空间 A***（实测最坏 15.3ms，占 50Hz 单帧预算 20ms 的 76%），
        # 并以 50Hz 刷 "路径规划失败" 日志。RBK/ARM 上叠加感知后很可能断流
        # （VEL 断供 → 下位机 300ms 保持/800ms 停速看门狗接管，车会停）。
        # 现在：BLOCKED 时按 BLOCKED_REPLAN_S 退避重规划，其间返回零速且不打日志。
        if self._state == NavState.BLOCKED:
            now_blocked = time.time()
            if now_blocked - self._blocked_last_plan_ts < self.BLOCKED_REPLAN_S:
                return VelocityCommand(linear=0.0, angular=0.0, timestamp=now_blocked)
            self._blocked_last_plan_ts = now_blocked

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
                now_w = time.time()
                if now_w - self._blocked_warn_ts >= 1.0:
                    self._blocked_warn_ts = now_w
                    logger.warning("路径规划失败 — 无可行路径（已进入退避重规划）")
                return VelocityCommand(linear=0.0, angular=0.0, timestamp=time.time())

        self._replan_counter += 1

        # 路径跟踪
        if self._current_path:
            self._prune_path(current_pose)

            if not self._current_path:
                if (self._motion.is_at_target(self._target, current_pose)
                        and self._aligned(current_pose)):
                    self._state = NavState.ARRIVED
                    logger.debug("到达目标!")
                    return VelocityCommand(linear=0.0, angular=0.0, timestamp=time.time())
                # 路径已被 prune 空但尚未到达目标（最后一段精确接近）：
                # 直接朝目标点做位置控制，避免 track_path(空路径) 返回零速度而卡死。
                cmd = self._motion.compute_velocity(self._target, current_pose, dt=dt,
                                                    align_heading=self._final_heading)
                self._total_distance += abs(cmd.linear) * dt
                self._localizer.update(cmd.linear, cmd.angular, dt)
                return cmd

            # 纯追踪
            cmd = self._motion.track_path(self._current_path, current_pose, dt=dt)

            # 局部避障修正
            escaping = False
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
                # ⚠️⚠️ 防"被障碍包住时冻住"（2026-09-18 现场）。
                # `_is_near_obstacle` 只看"车附近有没有高代价格子"。当障碍圆
                # **把车自己包在里面**时（实测对手圆半径 350mm，而"对手"被检测到
                # 离车仅 139mm），所有候选速度的代价都很差，规划器的"最优"就退化成
                # "几乎不动" —— 现场实测连续 9 秒 `cmd v=2~18mm/s`，而目标在 2.4m 外。
                # 而车不动 → 障碍位置估计不变 → **永远出不来**，形成死锁。
                # 物理上正确的做法是**先离开这片高代价区域**：倒车 + 朝目标转向。
                if abs(cmd.linear) < self.STUCK_CMD_MM_S:
                    self._escape_frames += 1
                    if self._escape_frames <= self.ESCAPE_MAX_FRAMES:
                        escaping = True
                        cmd = self._escape_from_obstacle(current_pose, cmd)
                        if self._escape_frames == 1:
                            logger.warning(
                                f"⚠️ 局部规划器只给出 {best_vw[0]:+.0f}mm/s（<"
                                f"{self.STUCK_CMD_MM_S:.0f}）→ 判定被障碍包住，"
                                f"改为果断倒车脱离（最多 {self.ESCAPE_MAX_FRAMES} 帧）")
                    # 超过上限就不再倒车（否则可能一路倒出场），交回规划器
                else:
                    # 规划器给出了正常速度 = 没被包住 → 计数复位。
                    # ⚠️ 复位必须**同时**放在这里和"附近没障碍"分支：安全区/场地边界
                    #    的代价是**永久写进代价地图**的，车沿安全区走时
                    #    `_is_near_obstacle` 恒为 True（只是不"冻住"），
                    #    只在 else 分支复位的话计数会一直涨，跑满上限后
                    #    **脱离功能永久失效**（自检 scenario 4 抓到的就是这个）。
                    self._escape_frames = 0
            else:
                self._escape_frames = 0
                self._state = NavState.MOVING

            # T1-15：硬禁区**预警带减速**。旧实现只有"已经进入禁区"的事后反应
            # （此时 -5 分/次的判罚已成立、进对方安全区更是比赛结束），而现成的
            # `get_violation_warning()`（150mm 预警）**零生产调用**。
            # 这里在预警带内主动把速度压到一半，给倒车规避留出反应距离。
            #
            # ⚠️ 但**脱离倒车不减速**：它只在"车被障碍包住、规划器冻住"时触发，
            #    是**有时限**（≤ESCAPE_MAX_FRAMES ≈3s）的主动撤离。砍半会让
            #    3 秒只退出 270mm，不足以离开 350mm 的障碍圆 → 撞上限后放弃 → 还是卡住
            #    （自检 scenario 1 实测：-180mm/s 被压到 -90mm/s）。
            #    这与第 343 行"进禁区倒车 -200mm/s"的处理一致 —— 应急撤离动作
            #    不受预警带减速约束。
            if not escaping:
                cmd = self._apply_caution(cmd, current_pose)

            self._total_distance += abs(cmd.linear) * dt
            self._localizer.update(cmd.linear, cmd.angular, dt)

            # 注：禁区检查已上移到 update() 开头统一执行（覆盖接近段/到达等所有分支），
            # 此处不再重复检查。
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

    def _escape_from_obstacle(self, current_pose: Tuple[float, float, float],
                              planned: VelocityCommand) -> VelocityCommand:
        """被障碍物包住时的果断脱离：**倒车 + 朝目标转向**。

        为什么不"接着用规划器那个小速度"：
            规划器是在代价地图上选最优 ``(v, w)``；当车**已经位于高代价格子内部**
            时，所有候选的代价都很差，"最优"退化成"几乎不动"（现场 2~18mm/s）。
            继续用它 = 车不动 → 障碍估计不变 → 死锁。
            物理上唯一有效的动作是**先离开这片高代价区域**，所以这里直接给一个
            确定的倒车速度，同时把车头朝目标方向转，避免脱离后绕远路。

        上限保护：连续脱离不会超过 :data:`ESCAPE_MAX_FRAMES` 帧（50Hz 下 ≈3s、
        约 500mm），之后交回规划器 —— 防止障碍是**真实**的（比如对手真的挡在
        前面）时一路倒出场。
        """
        # 朝向误差：车头 vs 目标（角度归一化到 [-π, π]）
        angular = 0.0
        if self._target is not None:
            desired = math.atan2(self._target[1] - current_pose[1],
                                 self._target[0] - current_pose[0])
            err = (desired - current_pose[2] + math.pi) % (2 * math.pi) - math.pi
            angular = max(-1.5, min(1.5, 1.5 * err))
        return VelocityCommand(linear=-self.ESCAPE_SPEED_MM_S,  # 负 = 倒车
                               angular=angular, timestamp=time.time())

    def _is_near_obstacle(self, current_pose: Tuple[float, float, float]) -> bool:
        cx, cy, _ = current_pose
        gx = int(cx / CELL_SIZE_MM)
        gy = int(cy / CELL_SIZE_MM)
        for dx in range(-2, 3):
            for dy in range(-2, 3):
                nx, ny = gx + dx, gy + dy
                if 0 <= nx < GRID_SIZE and 0 <= ny < GRID_SIZE:
                    # T2-1：对手的代价档是 COST_OPPONENT(200) < COST_OBSTACLE(255)，
                    # 旧实现只比 COST_OBSTACLE → 对手**永远无法触发**局部避障
                    # （实测对手就在正前方 200mm 处 60 帧，`_is_near_obstacle` 恒 False）。
                    # 现在把"对手档"也视为需要避障的障碍。
                    if self._cost_map._grid[ny][nx] >= COST_OPPONENT:
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
