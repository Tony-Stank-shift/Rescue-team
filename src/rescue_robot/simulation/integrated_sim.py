"""
integrated_sim.py —— 集成仿真引擎

把已写好的软件程序接进仿真，验证「导航 / 定位 / 套取 / 投放」全流程。

设计原则：
  - 仿真只扮演「真实世界 + 硬件」，把场地目标的真值喂给感知地图，
    把导航产出的速度指令积分成小车位姿，其余决策/导航/转运逻辑全部
    复用已写的 rescue_robot 模块（不修改它们）。
  - 统一单位：毫米 (mm)，朝向 theta 单位 rad，theta=0 表示朝 +Y（场地"上"）。

接线的四大模块：
  1. WorldMap（感知地图）      —— 仿真把目标真值注入其中，作为"感知结果"
  2. DecisionEngine（决策大脑） —— 输出 Action（NAVIGATE_TO / GRIP / TRANSPORT_TO / …）
  3. NavigationPipeline（导航） —— set_target + update 产出 VelocityCommand，负责定位/路径/避障
  4. TransportPipeline（转运）  —— start_trip + update 负责套取/装载计数/投放判定

运行一帧（step）的顺序：
  决策 → 执行 Action → 导航产出速度 → 积分位姿 → 转运推进 → 套取/投放联动 → 计时
"""

import logging
import math
import random
import time
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

from ..perception.field_elements import FieldLayout, SafeZoneColor
from ..perception.world_map import WorldMap
from ..perception.target_types import (
    TargetInfo, TargetType, TargetColor, TargetShape,
    CompetitionPhase, get_target_config, get_point_value,
)
from ..navigation.navigation_pipeline import NavigationPipeline
from ..navigation.motion_control import VelocityCommand
from ..decision.decision_engine import DecisionEngine, Action, ActionType
from ..decision.target_selector import StrategyState
from ..transport.transport_pipeline import TransportPipeline, TransportPhase

logger = logging.getLogger("integrated_sim")


# ============================================================
# 仿真目标（真值状态）
# ============================================================

@dataclass
class SimTarget:
    """仿真维护的目标真值。world_id 与 WorldMap 中 TrackedTarget.id 一致。"""
    world_id: int
    info: TargetInfo
    x: float                 # mm
    y: float                 # mm
    carried: bool = False    # 是否正被套取（跟随机器人）
    delivered: bool = False  # 是否已投放
    delivered_valid: bool = False  # 投放位置是否正确（物资区/伤员区匹配）

    @property
    def dangerous(self) -> bool:
        return self.info.is_dangerous

    @property
    def points(self) -> int:
        return self.info.points


# ============================================================
# 集成仿真
# ============================================================

class IntegratedSim:
    """
    集成仿真引擎（单机器人，无对手）。

    用法:
      sim = IntegratedSim(seed=42, my_color=SafeZoneColor.RED)
      sim.setup_match()
      for _ in range(int(180 / sim.DT)):
          frame = sim.step()
          if frame["is_terminal"]:
              break
    """

    MATCH_DURATION_S = 180.0
    DT = 0.02                 # 决策步长（50Hz，与 autonomous loop 一致）
    ROBOT_HALF_SIZE_MM = 150.0  # 300×300mm 车身，中心距边界的最小距离
    # 到达判定容差：导航管线的路径会被 prune（80mm），但其到达容差只有 40mm，
    # 导致 40~80mm 区间"路径空 + 速度归零 + 不判到达"而卡住。
    # 仿真（autonomous loop 层）用套取范围做到达判定，覆盖这一区间。
    ARRIVE_TOLERANCE_MM = 120.0

    # 出发区中心（mm），编号沿用 README 1-4 号（左/右/上/下）
    START_CENTERS = {
        1: (150.0, 150.0),     # 左下
        2: (2850.0, 150.0),    # 右下
        3: (2850.0, 2850.0),   # 右上
        4: (150.0, 2850.0),    # 左上
    }

    def __init__(self,
                 seed: Optional[int] = None,
                 my_color: SafeZoneColor = SafeZoneColor.RED,
                 start_zone: int = 1,
                 phase: CompetitionPhase = CompetitionPhase.PRELIMINARY,
                 dt: float = 0.02):
        self._seed = seed if seed is not None else random.randint(0, 2 ** 31 - 1)
        random.seed(self._seed)

        self.my_color = my_color
        self.start_zone = start_zone
        self.phase = phase
        self.DT = dt

        # ── 四大模块 ──
        self.field = FieldLayout.standard()
        self.world_map = WorldMap(field_layout=self.field)
        self.nav = NavigationPipeline(self.field, my_color=my_color, use_mock=True)
        self.decision = DecisionEngine(self.world_map, my_color=my_color)
        self.transport = TransportPipeline(
            field_layout=self.field, my_color=my_color, use_mock=True,
        )

        # ── 状态 ──
        self.targets: List[SimTarget] = []
        self._pose: Tuple[float, float, float] = (150.0, 150.0, 0.0)  # (x, y, theta) mm/rad
        self._time_elapsed = 0.0
        self._done = False
        self._sim_t0 = 0.0
        self._events: List[str] = []
        self._trajectory: List[Tuple[float, float]] = []
        self._last_action: Optional[Action] = None

        # 一趟转运的标志协调（DecisionEngine 与 TransportPipeline 之间）
        self._trip_gripped = False   # 当前目标是否已套取
        self._trip_released = False  # 当前目标是否已投放
        self._prev_current_id = None  # 决策引擎上一帧选中的目标 id（用于检测"新一趟"）

        logger.info(f"IntegratedSim 初始化: seed={self._seed}, "
                    f"my_color={my_color.name}, phase={phase.name}")

    # ============================================================
    # 比赛初始化
    # ============================================================

    def setup_match(self, target_specs: Optional[List[Tuple[TargetInfo, float, float]]] = None) -> None:
        """
        初始化一场比赛。

        Args:
            target_specs: [(TargetInfo, x_mm, y_mm), ...]，None 则用默认初赛/决赛分布
        """
        self._time_elapsed = 0.0
        self._done = False
        self._events = [f"MATCH_START seed={self._seed} phase={self.phase.name}"]
        self._trajectory = []
        self._trip_gripped = False
        self._trip_released = False
        self._prev_current_id = None
        self._last_action = None

        # 出发位姿
        sx, sy = self.START_CENTERS.get(self.start_zone, self.START_CENTERS[1])
        self._pose = (sx, sy, 0.0)
        self.nav.reset_pose(sx, sy, 0.0)
        self.nav.clear_target()
        self.transport.reset()

        # 生成并注入目标真值
        if target_specs is None:
            target_specs = self._default_target_specs()

        self.targets = []
        for info, x, y in target_specs:
            world_id = self.world_map.add_target(info, (x, y))
            self.targets.append(SimTarget(world_id=world_id, info=info, x=x, y=y))

        # 决策引擎开始计时（用真实时间锚点 + 仿真流逝时间）
        self.decision.start_match()
        self._sim_t0 = time.time()

        self._trajectory.append((sx, sy))
        logger.info(f"比赛初始化完成: {len(self.targets)} 个目标")

    def _default_target_specs(self) -> List[Tuple[TargetInfo, float, float]]:
        """生成默认目标分布（避开安全区/出发区/场地中央/边界）。"""
        cfg = get_target_config(self.phase)

        # 每种目标的数量（初赛/决赛通用一份演示分布）
        shapes = {
            TargetType.REGULAR_SUPPLY: list(cfg.keys())[0] if cfg else None,
            TargetType.CORE_SUPPLY: None,
            TargetType.INJURED: None,
            TargetType.DANGEROUS: None,
        }
        # 从配置表里按类型找 key（初赛/决赛 key 不同）
        regular_key = self._find_key(cfg, TargetType.REGULAR_SUPPLY)
        core_key = self._find_key(cfg, TargetType.CORE_SUPPLY)
        injured_key = self._find_key(cfg, TargetType.INJURED)
        dangerous_key = self._find_key(cfg, TargetType.DANGEROUS)

        spec_counts = [
            (regular_key, 6),
            (core_key, 3),
            (injured_key, 3),
            (dangerous_key, 2),
        ]

        specs: List[Tuple[TargetInfo, float, float]] = []
        for key, count in spec_counts:
            if key is None:
                continue
            info = cfg[key]
            for _ in range(count):
                x, y = self._random_position()
                specs.append((info, x, y))
        return specs

    @staticmethod
    def _find_key(cfg, target_type: TargetType):
        for key, info in cfg.items():
            if info.type == target_type:
                return key
        return None

    def _random_position(self) -> Tuple[float, float]:
        """随机生成一个可用目标位置（mm），避开禁区/出发区/场地中央。"""
        avoid = [
            (100, 2200, 700, 3000),      # 红安全区
            (2300, 2200, 2900, 3000),    # 蓝安全区
            (0, 0, 300, 300), (2700, 0, 3000, 300),
            (2700, 2700, 3000, 3000), (0, 2700, 300, 3000),
        ]
        for _ in range(5000):
            x = random.uniform(200, 2800)
            y = random.uniform(350, 2650)
            # 避开场地中央（决策引擎把中央 <500mm 视为"裁判重放"，会造成目标重选抖动）
            if math.hypot(x - 1500, y - 1500) < 600:
                continue
            if any(ax1 <= x <= ax2 and ay1 <= y <= ay2 for ax1, ay1, ax2, ay2 in avoid):
                continue
            return (x, y)
        return (1500.0, 1500.0)

    # ============================================================
    # 单步仿真
    # ============================================================

    def step(self) -> dict:
        """推进一帧仿真，返回帧状态 dict。"""
        if self._done:
            return self._build_frame()

        dt = self.DT
        x, y, theta = self._pose

        # ── 1. 读取进度标志（上一帧状态）──
        # 检测"新一趟"：决策引擎选中了与上一帧不同的目标 → 复位本趟进度标志
        cur = self.decision._current_target
        cur_id = cur.id if cur is not None else None
        if cur_id is not None and cur_id != self._prev_current_id:
            self._trip_gripped = False
            self._trip_released = False
        self._prev_current_id = cur_id

        nav_arrived = self.nav.is_arrived()
        if (not nav_arrived and self.nav.target is not None and
                self.nav.distance_to_target(self._pose) < self.ARRIVE_TOLERANCE_MM):
            nav_arrived = True  # 容差内视为到达，让决策引擎推进到套取阶段
        grip_done = self._trip_gripped
        release_done = self._trip_released

        # ── 2. 决策（使用仿真时间戳，而非墙钟时间）──
        ts = self._sim_t0 + self._time_elapsed
        try:
            action = self.decision.update(
                (x, y, theta),
                nav_arrived=nav_arrived,
                grip_done=grip_done,
                release_done=release_done,
                timestamp=ts,
            )
        except Exception as e:  # 决策异常不打断仿真，记录后继续
            self._events.append(f"DECISION_ERROR: {e}")
            action = Action(type=ActionType.WAIT, detail=f"decision error: {e}")
        self._last_action = action

        # ── 3. 执行 Action ──
        if action.type == ActionType.NAVIGATE_TO:
            self._set_nav_target(action.target_position)
        elif action.type == ActionType.GRIP:
            if self.transport.is_idle():
                tracks = [self.world_map.targets[tid]
                          for tid in action.target_ids if tid in self.world_map.targets]
                if tracks:
                    self.transport.start_trip(tracks)
                    self._trip_gripped = False
        elif action.type == ActionType.TRANSPORT_TO:
            self._set_nav_target(action.target_position)
        elif action.type == ActionType.EMERGENCY_STOP:
            self._done = True
            self._events.append("EMERGENCY_STOP")
            return self._build_frame()
        # WAIT / RECOVER 等 → 不驱动导航

        # ── 4. 导航：产出速度指令 ──
        try:
            cmd = self.nav.update((x, y, theta), dt=dt)
        except Exception as e:
            self._events.append(f"NAV_ERROR: {e}")
            cmd = VelocityCommand(linear=0.0, angular=0.0, timestamp=time.time())

        # ── 5. 积分真值位姿（差速运动模型，与 localization 一致）──
        x, y, theta = self._integrate(x, y, theta, cmd.linear, cmd.angular, dt)
        self._pose = (x, y, theta)

        # ── 6. 转运推进 ──
        try:
            self.transport.update((x, y, theta), self.world_map, self.nav)
        except Exception as e:
            self._events.append(f"TRANSPORT_ERROR: {e}")

        # ── 7. 套取联动：装载的目标跟随机器人 ──
        loaded_ids = self.transport.load_manager.state.target_ids
        if loaded_ids:
            self._trip_gripped = True
            for t in self.targets:
                if t.world_id in loaded_ids and not t.carried and not t.delivered:
                    t.carried = True
                    self._events.append(f"PICKUP #{t.world_id} {t.info.description}")

        for t in self.targets:
            if t.carried:
                t.x, t.y = x, y

        # ── 8. 投放联动：套取的目标落到机器人当前位置 ──
        if self.transport.is_complete():
            for t in self.targets:
                if t.carried:
                    t.carried = False
                    t.delivered = True
                    t.x, t.y = x, y
                    res = self.transport.placer.classify((x, y), t.info)
                    t.delivered_valid = res.is_valid
                    self._events.append(
                        f"DELIVER #{t.world_id} {t.info.description} valid={res.is_valid}"
                    )
                    # 本趟投放完成，置位"已投放"，决策引擎据此 mark_in_safe_zone
                    self._trip_released = True

        # ── 9. 时间推进 / 轨迹 / 结束判定 ──
        self._time_elapsed += dt
        self._trajectory.append((x, y))

        if self._time_elapsed >= self.MATCH_DURATION_S:
            self._done = True
            self._events.append("TIME_UP")
        elif self.decision.strategy_state == StrategyState.DONE:
            self._done = True
            self._events.append("ALL_DELIVERED")

        return self._build_frame()

    # ============================================================
    # 内部
    # ============================================================

    def _set_nav_target(self, pos: Optional[Tuple[float, float]]) -> None:
        """设置导航目标（去重，避免每帧重复触发重规划）。"""
        if pos is None:
            return
        if self.nav.target != pos:
            self.nav.set_target(pos[0], pos[1])

    @staticmethod
    def _integrate(x: float, y: float, theta: float,
                   v: float, w: float, dt: float) -> Tuple[float, float, float]:
        """差速运动模型积分（mm, rad）。"""
        ds = v * dt
        if abs(w) < 0.001:
            x += ds * math.cos(theta)
            y += ds * math.sin(theta)
        else:
            radius = v / w
            dtheta = w * dt
            x += radius * (math.sin(theta + dtheta) - math.sin(theta))
            y -= radius * (math.cos(theta + dtheta) - math.cos(theta))
            theta += dtheta
        theta = math.atan2(math.sin(theta), math.cos(theta))

        # 边界钳制（禁区 margin 100mm + 半车身）
        m = IntegratedSim.ROBOT_HALF_SIZE_MM
        x = max(m, min(3000.0 - m, x))
        y = max(m, min(3000.0 - m, y))
        return x, y, theta

    def _build_frame(self) -> dict:
        """构建帧状态（供可视化/测试使用）。"""
        x, y, theta = self._pose
        action_name = self._last_action.type.name if self._last_action else "WAIT"
        delivered = sum(1 for t in self.targets if t.delivered)
        carried_id = next((t.world_id for t in self.targets if t.carried), None)

        return {
            "time_elapsed_s": round(self._time_elapsed, 2),
            "time_remaining_s": round(max(0, self.MATCH_DURATION_S - self._time_elapsed), 2),
            "score": self.transport.load_manager.total_score,
            "targets_delivered": delivered,
            "targets_total": len(self.targets),
            "robot_pose": (x, y, theta),
            "is_terminal": self._done,
            "trip_count": self.transport.load_manager.total_trips,
            "carried_id": carried_id,
            "action": action_name,
            "action_detail": self._last_action.detail if self._last_action else "",
            "nav_state": self.nav.state.name,
            "transport_phase": self.transport.phase.name,
            "decision_state": self.decision.strategy_state.name,
        }

    # ============================================================
    # 查询
    # ============================================================

    @property
    def pose(self) -> Tuple[float, float, float]:
        return self._pose

    @property
    def trajectory(self) -> List[Tuple[float, float]]:
        return list(self._trajectory)

    @property
    def events(self) -> List[str]:
        return list(self._events)

    @property
    def score(self) -> int:
        return self.transport.load_manager.total_score

    @property
    def is_terminal(self) -> bool:
        return self._done

    def summary(self) -> str:
        x, y, theta = self._pose
        return (
            f"t={self._time_elapsed:.1f}s | score={self.score} | "
            f"delivered={sum(1 for t in self.targets if t.delivered)}/{len(self.targets)} | "
            f"pose=({x:.0f},{y:.0f}) | action={self._last_action.type.name if self._last_action else 'WAIT'} | "
            f"nav={self.nav.state.name} | transport={self.transport.phase.name} | "
            f"decision={self.decision.strategy_state.name}"
        )


# ============================================================
# 独立测试 / CLI
# ============================================================

if __name__ == "__main__":
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s [%(levelname)-7s] %(name)s: %(message)s",
        datefmt="%H:%M:%S",
    )

    print("=" * 60)
    print("  集成仿真 — 全流程测试（导航/定位/套取/投放）")
    print("=" * 60)

    sim = IntegratedSim(seed=42, my_color=SafeZoneColor.RED)
    sim.setup_match()
    print(f"\n初始化: {len(sim.targets)} 个目标")
    for t in sim.targets:
        print(f"  #{t.world_id} {t.info.description} @ ({t.x:.0f}, {t.y:.0f}) "
              f"{'⚠️危险' if t.dangerous else str(t.points)+'分'}")

    max_steps = int(sim.MATCH_DURATION_S / sim.DT)
    t0 = time.time()
    last_report = 0.0
    for i in range(max_steps):
        frame = sim.step()
        # 每 5 秒打印一次进度
        if frame["time_elapsed_s"] - last_report >= 5.0:
            last_report = frame["time_elapsed_s"]
            print(f"  {sim.summary()}")
        if frame["is_terminal"]:
            break
    elapsed_real = time.time() - t0

    print(f"\n{'=' * 60}")
    print(f"  仿真结束: {sim.summary()}")
    print(f"  真实耗时: {elapsed_real:.1f}s (仿真 {frame['time_elapsed_s']:.0f}s)")
    print(f"  最终比分: {sim.score}")
    print(f"  最近事件:")
    for ev in sim.events[-10:]:
        print(f"    - {ev}")

    delivered = [t for t in sim.targets if t.delivered]
    valid = [t for t in delivered if t.delivered_valid]
    print(f"\n  投放统计: {len(delivered)} 个投放, {len(valid)} 个位置正确")
    print(f"  危险目标是否被误转运: "
          f"{any(t.dangerous and t.delivered for t in sim.targets)}")
