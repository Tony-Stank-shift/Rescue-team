"""
sim_world.py —— MuJoCo 仿真世界管理器

封装完整的比赛仿真：场地 → 机器人 → 目标 → 物理循环。
对接现有 DecisionEngine 和 StateMachine。

用法:
  world = SimWorld(phase=CompetitionPhase.PRELIMINARY, mode="headless")
  world.setup_match(scenario="default")
  while not world.is_done():
      world.step()
  stats = world.get_match_stats()
  world.close()
"""

import logging
import math
import random
import time
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Tuple

import mujoco
import numpy as np

from .sim_models import (
    FIELD_SIZE_MM,
    SAFE_ZONE_SIZE_MM,
    mm_to_m,
    m_to_mm,
    rgba,
)
from .sim_field import SimField
from .sim_robot import SimRobot
from .sim_target import SimTarget, TargetFactory

logger = logging.getLogger("sim_world")


# ============================================================
# 比赛状态
# ============================================================

@dataclass
class MatchState:
    """单步仿真状态快照。"""
    time_elapsed_s: float = 0.0
    time_remaining_s: float = 180.0
    score: int = 0
    targets_delivered: int = 0
    robot_pose: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    strategy_state: str = "FIRST_TRIP"
    is_terminal: bool = False
    violations: int = 0
    trip_count: int = 0
    event: Optional[str] = None


@dataclass
class MatchResult:
    """比赛结果。"""
    score: int = 0
    targets_delivered: int = 0
    trip_count: int = 0
    violations: int = 0
    total_time_s: float = 0.0
    events: List[str] = field(default_factory=list)


# ============================================================
# SimWorld
# ============================================================

class SimWorld:
    """
    MuJoCo 仿真世界管理器。

    封装：
      - MuJoCo model + data
      - 场地 (SimField)
      - 机器人 (SimRobot)
      - 目标列表 (SimTarget[])
      - 比赛逻辑（计分、计时、规则）
    """

    MATCH_DURATION_S = 180.0
    PHYSICS_TIMESTEP = 0.004          # 250 Hz
    DECISION_TIMESTEP = 0.02          # 50 Hz (每 5 个物理步做 1 次决策)
    PHYSICS_STEPS_PER_DECISION = 5

    # 安全区坐标 (m) — 从 sim_field 复制
    SAFE_ZONE_RED_POS_M = (mm_to_m(50)[0], mm_to_m(2550)[0])
    SAFE_ZONE_BLUE_POS_M = (mm_to_m(2350)[0], mm_to_m(2550)[0])
    SAFE_ZONE_SIZE_M = (mm_to_m(600)[0], mm_to_m(400)[0])

    def __init__(self, phase=None, mode: str = "headless",
                 start_zone: int = 1, seed: Optional[int] = None):
        """
        Args:
            phase: CompetitionPhase (PRELIMINARY or FINAL)
            mode: "gui" | "headless"
            start_zone: 出发区编号 (1-4)
            seed: 随机种子
        """
        self.phase = phase
        self.mode = mode
        self.start_zone = start_zone
        self._seed = seed or random.randint(0, 2**31)
        random.seed(self._seed)

        # MuJoCo 核心
        self._spec: Optional[mujoco.MjSpec] = None
        self._model: Optional[mujoco.MjModel] = None
        self._data: Optional[mujoco.MjData] = None
        self._renderer: Optional[mujoco.Renderer] = None
        self._viewer: Any = None

        # 实体
        self.field: Optional[SimField] = None
        self.robot: Optional[SimRobot] = None
        self.targets: List[SimTarget] = []
        self._target_factory: Optional[TargetFactory] = None

        # 比赛状态
        self._time_elapsed = 0.0
        self._score = 0
        self._trip_count = 0
        self._violations = 0
        self._events: List[str] = []
        self._done = False

        # 决策引擎 (可选，后续对接)
        self.decision_engine = None
        self._manual_control = True  # 默认手动/AI 控制

        # 我方安全区
        self._my_safe_zone_pos = self.SAFE_ZONE_RED_POS_M  # 默认红队

    # ---- 生命周期 ----

    def setup_match(self, target_configs: dict = None,
                     target_count: int = 20,
                     scenario: str = "default") -> None:
        """
        初始化仿真场景。

        Args:
            target_configs: 目标配置字典 {(color, shape): TargetInfo}
            target_count: 目标数量（初赛 20，决赛 25）
            scenario: 场景名
        """
        # 创建 spec
        self._spec = mujoco.MjSpec()
        self._spec.option.timestep = self.PHYSICS_TIMESTEP
        self._spec.option.gravity = [0, 0, -9.81]
        # Enable iterative solver for better contact
        self._spec.option.solver = mujoco.mjtSolver.mjSOL_NEWTON
        self._spec.option.iterations = 50

        wb = self._spec.worldbody

        # 构建场地
        self.field = SimField()
        self.field.build(self._spec)
        self._target_factory = TargetFactory(wb)

        # 构建机器人
        self.robot = SimRobot(wb, start_zone=self.start_zone)

        # 放置目标
        self._spawn_targets(target_configs, target_count)

        # 编译模型
        self._model = self._spec.compile()
        self._data = mujoco.MjData(self._model)

        # 初始化机器人位姿
        self.robot.init_pose(self._data)
        mujoco.mj_forward(self._model, self._data)

        # 创建渲染器 (headless 可后续创建)
        if self.mode == "headless":
            self._renderer = mujoco.Renderer(self._model, 480, 640)

        self._time_elapsed = 0.0
        self._score = 0
        self._done = False
        self._events.append(f"MATCH_START seed={self._seed}")

        logger.info(f"仿真初始化完成: {len(self.targets)} 个目标, "
                     f"seed={self._seed}")

    def close(self) -> None:
        """清理资源。"""
        if self._renderer:
            self._renderer.close()
            self._renderer = None
        if self._viewer:
            self._viewer.close()
            self._viewer = None
        self._data = None
        self._model = None
        self._spec = None

    # ---- 主循环 ----

    def step(self) -> MatchState:
        """
        单步仿真 (1 个决策步 = 5 个物理步)。

        Returns:
            MatchState 当前状态快照
        """
        if self._done:
            return self._get_state()

        # 决策
        if self.decision_engine and not self._manual_control:
            sensor_data = self._get_sensor_data()
            action = self.decision_engine.update(sensor_data)
            self.robot.execute_action(self._data, action)
        else:
            # 手动/AI: 简单的最近目标追逐
            self._simple_ai_step()

        # 物理步进
        for _ in range(self.PHYSICS_STEPS_PER_DECISION):
            mujoco.mj_step(self._model, self._data)

        # 更新时间
        self._time_elapsed += self.DECISION_TIMESTEP

        # 检查目标送达
        self._check_deliveries()

        # 检查比赛结束
        if self._time_elapsed >= self.MATCH_DURATION_S:
            self._done = True
            self._events.append("MATCH_TIME_UP")
        elif all(t.is_delivered for t in self.targets):
            self._done = True
            self._events.append("ALL_DELIVERED")

        return self._get_state()

    def run(self, duration_s: float = None, callback=None) -> MatchResult:
        """
        运行完整比赛。

        Args:
            duration_s: 比赛时长 (默认 180s)
            callback: 每步回调 callback(state)

        Returns:
            MatchResult
        """
        dur = duration_s or self.MATCH_DURATION_S
        steps = int(dur / self.DECISION_TIMESTEP)

        for _ in range(steps):
            state = self.step()
            if callback:
                callback(state)
            if state.is_terminal:
                break

        return self.get_match_stats()

    # ---- 内部方法 ----

    def _spawn_targets(self, configs: dict, count: int) -> None:
        """在场地中央区域随机放置目标。"""
        if configs is None:
            # 使用简化的默认目标属性
            configs = self._default_target_configs()

        keys = list(configs.values())
        if not keys:
            return

        # 随机位置 (避开安全区和出发区)
        positions = self._random_positions(count)
        self.targets = self._target_factory.from_config_dict(configs, positions)

    @staticmethod
    def _default_target_configs() -> dict:
        """生成默认目标配置 (避免导入 target_types 依赖)。"""
        from dataclasses import dataclass

        @dataclass
        class _Info:
            shape: Any
            color: Any
            type: Any
            points: int
            description: str

        # 使用简单的枚举模拟
        class _Shape:
            CUBE = "cube"
            PYRAMID = "pyramid"
            CUBOID = "cuboid"
            SPHERE = "sphere"
        class _Color:
            GREEN = "green"
            BLACK = "black"
            ORANGE = "orange"
            LIGHT_BLUE = "light_blue"
        class _Type:
            REGULAR_SUPPLY = "REGULAR_SUPPLY"
            CORE_SUPPLY = "CORE_SUPPLY"
            INJURED = "INJURED"

        return {
            ("green", "cube"): _Info(_Shape.CUBE, _Color.GREEN, _Type.REGULAR_SUPPLY, 5, "普通物资"),
            ("black", "pyramid"): _Info(_Shape.PYRAMID, _Color.BLACK, _Type.CORE_SUPPLY, 10, "核心物资"),
            ("orange", "cuboid"): _Info(_Shape.CUBOID, _Color.ORANGE, _Type.INJURED, 15, "伤员"),
            ("light_blue", "cube"): _Info(_Shape.CUBE, _Color.LIGHT_BLUE, _Type.REGULAR_SUPPLY, 5, "危险品"),
        }

    def _random_positions(self, count: int) -> List[Tuple[float, float]]:
        """生成随机目标位置 (避开安全区和出发区)。"""
        positions = []
        fw, fd = mm_to_m(FIELD_SIZE_MM[0], FIELD_SIZE_MM[1])

        # 避开区域 (m)
        avoid = [
            # 安全区
            (0.05, 2.55, 0.05 + 0.6, 2.55 + 0.4),     # 红
            (2.35, 2.55, 2.35 + 0.6, 2.55 + 0.4),     # 蓝
            # 出发区
            (0, 0, 0.3, 0.3),
            (fw - 0.3, 0, fw, 0.3),
            (fw - 0.3, fd - 0.3, fw, fd),
            (0, fd - 0.3, 0.3, fd),
        ]

        for _ in range(count * 3):  # 尝试次数
            if len(positions) >= count:
                break
            x = random.uniform(0.1, fw - 0.1)
            y = random.uniform(0.5, fd - 0.5)
            # 检查是否在避开区内
            blocked = False
            for ax1, ay1, ax2, ay2 in avoid:
                if ax1 <= x <= ax2 and ay1 <= y <= ay2:
                    blocked = True
                    break
            if not blocked:
                positions.append((x, y))

        return positions

    def _simple_ai_step(self) -> None:
        """简易 AI: 朝向最近未送达目标。"""
        if not self.targets:
            return

        # 找最近未送达目标
        rx, ry, _ = self.robot.get_pose(self._data)
        closest = None
        closest_dist = float("inf")
        for t in self.targets:
            if t.is_delivered:
                continue
            tx, ty = t._pos_m
            d = math.sqrt((tx - rx)**2 + (ty - ry)**2)
            if d < closest_dist:
                closest_dist = d
                closest = t

        if closest is None:
            self.robot.set_velocity(self._data, 0.0, 0.0)
            return

        # 朝向目标
        tx, ty = closest._pos_m
        target_yaw = math.atan2(ty - ry, tx - rx)
        _, _, current_yaw = self.robot.get_pose(self._data)
        yaw_error = target_yaw - current_yaw
        # Normalize to [-pi, pi]
        yaw_error = math.atan2(math.sin(yaw_error), math.cos(yaw_error))

        if abs(yaw_error) > 0.2:
            # Turn toward target
            w = min(2.0, abs(yaw_error) * 3) * (1 if yaw_error > 0 else -1)
            self.robot.set_velocity(self._data, 0.0, w)
        elif closest_dist > 0.1:
            # Drive forward
            v = min(0.5, closest_dist * 2)
            self.robot.set_velocity(self._data, v, yaw_error * 3)
        else:
            # Near target → push
            self.robot.set_pusher(self._data, 1.0)
            v = 0.2
            self.robot.set_velocity(self._data, v, 0.0)

    def _check_deliveries(self) -> None:
        """检查目标是否进入我方安全区。"""
        sz_x, sz_y = self._my_safe_zone_pos
        sz_w, sz_h = self.SAFE_ZONE_SIZE_M

        for t in self.targets:
            if t.is_delivered:
                continue
            pos = t.get_position(self._data)
            x, y = pos[0], pos[1]
            if sz_x <= x <= sz_x + sz_w and sz_y <= y <= sz_y + sz_h:
                t.mark_delivered()
                pts = t.points
                if t.is_dangerous:
                    pts = -10  # 危险品送入安全区扣分
                    self._violations += 1
                self._score += pts
                self._events.append(
                    f"DELIVERED target#{t.target_id} +{pts}pts"
                )

    def _get_sensor_data(self) -> dict:
        """获取传感器仿真数据。"""
        rx, ry, rt = self.robot.get_pose(self._data)
        visible = []
        for t in self.targets:
            if t.is_delivered:
                continue
            tx, ty, _ = t.get_position(self._data)
            dist = math.sqrt((tx - rx)**2 + (ty - ry)**2)
            if dist < 2.0:  # 2m 感知范围
                visible.append({
                    "id": t.target_id,
                    "x": tx, "y": ty,
                    "distance": dist,
                    "points": t.points,
                })
        return {
            "robot_pose": (rx, ry, rt),
            "visible_targets": visible,
            "time_elapsed": self._time_elapsed,
        }

    def _get_state(self) -> MatchState:
        """获取当前比赛状态快照。"""
        rx, ry, rt = self.robot.get_pose(self._data)
        delivered = sum(1 for t in self.targets if t.is_delivered)

        return MatchState(
            time_elapsed_s=self._time_elapsed,
            time_remaining_s=max(0, self.MATCH_DURATION_S - self._time_elapsed),
            score=self._score,
            targets_delivered=delivered,
            robot_pose=(rx, ry, rt),
            is_terminal=self._done,
            violations=self._violations,
            trip_count=self._trip_count,
        )

    # ---- 查询 ----

    def is_done(self) -> bool:
        return self._done

    def get_match_stats(self) -> MatchResult:
        return MatchResult(
            score=self._score,
            targets_delivered=sum(1 for t in self.targets if t.is_delivered),
            trip_count=self._trip_count,
            violations=self._violations,
            total_time_s=self._time_elapsed,
            events=self._events.copy(),
        )

    def render(self) -> np.ndarray:
        """离屏渲染当前帧。"""
        if self._renderer is None:
            self._renderer = mujoco.Renderer(self._model, 480, 640)
        self._renderer.update_scene(self._data)
        return self._renderer.render()

    def get_viewer(self):
        """获取交互式查看器。"""
        if self._viewer is None and self.mode == "gui":
            from mujoco import viewer
            self._viewer = viewer.launch(self._model, self._data)
        return self._viewer


# ============================================================
# 独立测试
# ============================================================

if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO, format="%(message)s")

    print("=" * 60)
    print("  SimWorld 独立测试")
    print("=" * 60)

    # Test 1: Setup and first steps
    print("\n--- 测试 1: 初始化 ---")
    world = SimWorld(mode="headless", seed=42)
    world.setup_match(target_count=10)
    assert world._model is not None
    assert len(world.targets) == 10
    print(f"  {len(world.targets)} 目标, seed=42")
    print("  ✅ 通过")

    # Test 2: Step simulation
    print("\n--- 测试 2: 物理仿真 ---")
    for i in range(10):
        state = world.step()
    assert not state.is_terminal
    x, y, _ = state.robot_pose
    print(f"  10 步后: pos=({x:.3f}, {y:.3f}), score={state.score}")
    print("  ✅ 通过")

    # Test 3: Render
    print("\n--- 测试 3: 渲染 ---")
    px = world.render()
    assert px.shape[2] == 3
    print(f"  渲染: {px.shape}")
    print("  ✅ 通过")

    # Test 4: Run full match (short duration)
    print("\n--- 测试 4: 短期比赛 (2s) ---")
    world2 = SimWorld(mode="headless", seed=99)
    world2.setup_match(target_count=5)
    result = world2.run(duration_s=2.0)
    assert result.total_time_s > 0
    print(f"  分数: {result.score}, 送达: {result.targets_delivered}")
    print(f"  耗时: {result.total_time_s:.1f}s")
    print("  ✅ 通过")

    # Cleanup
    world.close()
    world2.close()

    print(f"\n{'=' * 60}")
    print("  SimWorld — 全部通过 ✅")
    print(f"{'=' * 60}")
