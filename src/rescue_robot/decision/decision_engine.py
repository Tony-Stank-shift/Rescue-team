"""
decision_engine.py —— 决策引擎

比赛状态机——机器人自主运行的"大脑"。
集成感知、导航、转运，执行完整的比赛策略。

状态转移：
  FIRST_TRIP ──(1普通物资已入物资区)──→ FREE_RUN
  FREE_RUN ──(剩余<30s)──→ TIME_PRESSURE
  FREE_RUN ──(目标全清/时间到)──→ DONE
  ANY ──(异常)──→ ANOMALY
  ANY ──(强制分离)──→ FORCED_RESET

Action 类型（输出到 autonomous loop）:
  NAVIGATE_TO(target)  — 导航到目标
  GRIP(targets)        — 夹取目标
  TRANSPORT_TO(safe)   — 运送至安全区
  RELEASE              — 投放
  RECOVER(pose)        — 恢复/重定位
  WAIT                 — 等待
  EMERGENCY_STOP       — 急停
"""

import logging
import math
import time
from dataclasses import dataclass, field
from enum import Enum, auto
from typing import List, Optional, Set, Tuple

from .target_selector import TargetSelector, StrategyState, ScoredTarget
from .anomaly_handler import (
    AnomalyHandler, AnomalyType, AnomalyReport, RecoveryAction,
)

class FallbackLevel:
    """降级层级"""
    NORMAL = 0        # 正常工作
    RETRY = 1         # 重试（换参数）
    SWITCH_TARGET = 2 # 换目标
    EXPLORE = 3       # 探索扫描
    SURVIVAL = 4      # 保命绕圈
from ..perception.target_types import TargetType, TargetInfo, get_point_value
from ..perception.world_map import WorldMap, TrackedTarget, TargetStatus
from ..perception.field_elements import SafeZoneColor, FieldLayout

logger = logging.getLogger("decision_engine")


# ============================================================
# Action（输出到 autonomous loop）
# ============================================================

class ActionType(Enum):
    NAVIGATE_TO = auto()       # 导航到目标点
    GRIP = auto()              # 夹取目标
    TRANSPORT_TO = auto()      # 运送至安全区
    RELEASE = auto()           # 释放投放
    RECOVER = auto()           # 恢复/重定位
    WAIT = auto()              # 等待
    EMERGENCY_STOP = auto()    # 急停


@dataclass
class Action:
    """决策引擎输出的动作"""
    type: ActionType
    target_position: Optional[Tuple[float, float]] = None
    target_ids: List[int] = field(default_factory=list)
    detail: str = ""


# ============================================================
# 决策引擎
# ============================================================

class DecisionEngine:
    """
    决策引擎——比赛策略执行者。

    使用方式（在 autonomous loop 中）：
      engine = DecisionEngine(world_map, my_color=RED)
      engine.start_match()

      # 每帧
      action = engine.update(robot_pose, timestamp)

      # 执行 action
      if action.type == NAVIGATE_TO:
          nav.set_target(*action.target_position)
      elif action.type == GRIP:
          transport.start_trip(...)
      ...
    """

    # 比赛参数
    MATCH_DURATION_S = 180.0     # 3 分钟
    TIME_PRESSURE_S = 30.0       # 剩余 < 30s → 时间紧迫

    # 动作超时
    NAV_TIMEOUT_S = 10.0         # 导航超时
    GRIP_TIMEOUT_S = 3.0         # 夹取超时
    TRANSPORT_TIMEOUT_S = 15.0   # 运送超时

    def __init__(self,
                 world_map: WorldMap,
                 my_color: SafeZoneColor = SafeZoneColor.RED):
        self._world_map = world_map
        self._my_color = my_color

        # 子模块
        self._selector = TargetSelector()
        self._anomaly = AnomalyHandler()

        # 状态
        self._strategy_state = StrategyState.FIRST_TRIP
        self._action_phase = ActionType.WAIT  # 当前动作阶段
        self._action_start_time = 0.0

        # 比赛计时
        self._match_start_time = 0.0
        self._match_elapsed = 0.0

        # 当前目标
        self._current_target: Optional[TrackedTarget] = None
        self._transporting = False

        # 降级系统
        self._fallback_level = FallbackLevel.NORMAL
        self._fallback_retries = 0
        self._max_retries = 3
        self._last_action_time = time.time()
        self._explore_waypoints = []

        # 统计
        self._trips_completed = 0
        self._targets_delivered = 0
        self._score = 0

        logger.info(f"DecisionEngine 初始化: my_color={my_color.name}")

    # ---- 属性 ----

    @property
    def strategy_state(self) -> StrategyState:
        return self._strategy_state

    @property
    def time_remaining_s(self) -> float:
        return max(0, self.MATCH_DURATION_S - self._match_elapsed)

    @property
    def is_time_pressure(self) -> bool:
        return self.time_remaining_s < self.TIME_PRESSURE_S

    @property
    def score(self) -> int:
        return self._score

    @property
    def targets_delivered(self) -> int:
        return self._targets_delivered

    # ---- 比赛控制 ----

    def start_match(self) -> None:
        """开始比赛计时"""
        self._match_start_time = time.time()
        self._match_elapsed = 0.0
        self._strategy_state = StrategyState.FIRST_TRIP
        self._action_phase = ActionType.WAIT
        logger.info("🏁 比赛开始! 状态=FIRST_TRIP")

    # ---- 主决策循环 ----

    def update(self,
               robot_pose: Tuple[float, float, float],
               nav_arrived: bool = False,
               grip_done: bool = False,
               release_done: bool = False,
               imu_data: Optional[dict] = None,
               contact_duration_s: float = 0.0,
               timestamp: Optional[float] = None) -> Action:
        """
        单帧决策。

        Args:
            robot_pose: (x, y, theta) 机器人位姿
            nav_arrived: 导航是否已到达目标
            grip_done: 夹取是否完成
            release_done: 投放是否完成
            imu_data: IMU 数据（用于异常检测）
            contact_duration_s: 对方接触时长
            timestamp: 时间戳

        Returns:
            Action: 要执行的动作
        """
        if timestamp is None:
            timestamp = time.time()

        self._match_elapsed = timestamp - self._match_start_time
        rx, ry, rtheta = robot_pose

        # ─── 异常检测 ───
        anomaly = self._anomaly.check(
            robot_pose, (0, 0),  # velocity from nav
            imu_data, contact_duration_s,
        )
        if anomaly.type != AnomalyType.NONE:
            self._strategy_state = StrategyState.ANOMALY
            return self._handle_anomaly(anomaly)

        # 通知有动作（正常运行时）
        self._anomaly.notify_action()

        # ─── 时间管理 ───
        if self.is_time_pressure and \
           self._strategy_state == StrategyState.FREE_RUN:
            self._strategy_state = StrategyState.TIME_PRESSURE
            logger.info(f"⏰ 时间紧迫! 剩余 {self.time_remaining_s:.0f}s")

        # ─── 检查比赛结束 ───
        if self.time_remaining_s <= 0:
            self._strategy_state = StrategyState.DONE
            logger.info("比赛时间到!")
            return Action(type=ActionType.WAIT, detail="比赛结束")

        if not self._world_map.active_targets:
            self._strategy_state = StrategyState.DONE
            logger.info("所有目标已清空!")
            return Action(type=ActionType.WAIT, detail="所有目标已清空")

        # ─── 状态机 ───
        if self._strategy_state == StrategyState.FIRST_TRIP:
            return self._handle_first_trip(rx, ry, nav_arrived, grip_done, release_done)
        elif self._strategy_state in (StrategyState.FREE_RUN,
                                       StrategyState.TIME_PRESSURE):
            return self._handle_free_run(rx, ry, nav_arrived, grip_done, release_done)
        elif self._strategy_state == StrategyState.FORCED_RESET:
            return self._handle_forced_reset()
        elif self._strategy_state == StrategyState.DONE:
            return Action(type=ActionType.WAIT, detail="比赛完成")
        else:
            return Action(type=ActionType.WAIT, detail="未知状态")

    # ---- FIRST_TRIP ----

    def _handle_first_trip(self, rx: float, ry: float,
                           nav_arrived: bool, grip_done: bool,
                           release_done: bool) -> Action:
        """处理首次转运状态"""
        # 选择目标
        if self._current_target is None:
            self._current_target = self._selector.select_best_for_first_trip(
                self._world_map, (rx, ry)
            )
            if self._current_target is None:
                self._fallback_level = FallbackLevel.EXPLORE
                explore_pos = self._get_explore_target(rx, ry)
                logger.warning("FIRST_TRIP: 无普通物资 → 探索模式")
                return Action(type=ActionType.NAVIGATE_TO,
                              target_position=explore_pos,
                              detail="探索: 搜索普通物资")

            logger.info(f"FIRST_TRIP: 选择 {self._current_target.info.description} "
                        f"@ ({self._current_target.position[0]:.0f}, "
                        f"{self._current_target.position[1]:.0f})")
            return Action(
                type=ActionType.NAVIGATE_TO,
                target_position=self._current_target.position,
                detail=f"首次转运: 前往普通物资",
            )

        # 导航到目标
        if not nav_arrived:
            return Action(
                type=ActionType.NAVIGATE_TO,
                target_position=self._current_target.position,
            )

        # 夹取
        if not grip_done:
            return Action(
                type=ActionType.GRIP,
                target_ids=[self._current_target.id],
                detail="FIRST_TRIP: 夹取普通物资",
            )

        # 运送至物资区
        safe_region = self._get_supply_area_position()
        if not release_done:
            return Action(
                type=ActionType.TRANSPORT_TO,
                target_position=safe_region,
                detail="FIRST_TRIP: 运送至物资区",
            )

        # 投放完成 → 进入 FREE_RUN
        self._world_map.mark_in_safe_zone(self._current_target.id)
        self._targets_delivered += 1
        self._score += get_point_value(TargetType.REGULAR_SUPPLY)

        self._strategy_state = StrategyState.FREE_RUN
        self._current_target = None
        self._trips_completed += 1

        logger.info("✅ FIRST_TRIP 完成! 进入 FREE_RUN")
        return Action(type=ActionType.WAIT, detail="首次转运完成")

    # ---- FREE_RUN / TIME_PRESSURE ----

    def _check_invalid_transport(self, rx: float, ry: float) -> bool:
        """检测转运无效恢复：目标被裁判重新放在场地中央"""
        import math
        for tid, t in self._world_map.targets.items():
            from rescue_robot.perception.target_types import TargetStatus
            if t.status == TargetStatus.ACTIVE:
                dist_to_center = math.sqrt(
                    (t.position[0] - 1500)**2 + (t.position[1] - 1500)**2
                )
                if dist_to_center < 500:
                    logger.info("检测到目标在场地中央(无效转运恢复): ID=%d", tid)
                    return True
        return False

    def _handle_free_run(self, rx: float, ry: float,
                         nav_arrived: bool, grip_done: bool,
                         release_done: bool) -> Action:
        """处理自由转运状态"""
        # 检测转运无效恢复
        if self._check_invalid_transport(rx, ry):
            self._current_target = None  # 重新选择目标

        # 选择目标
        if self._current_target is None:
            self._current_target = self._selector.select_best(
                self._world_map, (rx, ry),
                self._strategy_state, self.time_remaining_s,
            )
            if self._current_target is None:
                if self._fallback_level < FallbackLevel.EXPLORE:
                    self._fallback_level = FallbackLevel.EXPLORE
                explore_pos = self._get_explore_target(rx, ry)
                logger.warning("FREE_RUN: 无可用目标 → 探索模式")
                return Action(type=ActionType.NAVIGATE_TO,
                              target_position=explore_pos,
                              detail="探索: 扫描新目标")

            logger.info(f"目标: {self._current_target.info.description} "
                        f"({self._current_target.info.points}分) "
                        f"@ dist={math.sqrt((self._current_target.position[0]-rx)**2 + (self._current_target.position[1]-ry)**2):.0f}mm")

            return Action(
                type=ActionType.NAVIGATE_TO,
                target_position=self._current_target.position,
            )

        # 导航 → 夹取 → 运送 → 投放
        if not nav_arrived:
            return Action(
                type=ActionType.NAVIGATE_TO,
                target_position=self._current_target.position,
            )

        if not grip_done:
            return Action(
                type=ActionType.GRIP,
                target_ids=[self._current_target.id],
            )

        # 运送到正确的区域
        if self._current_target.info.type == TargetType.INJURED:
            target_zone = self._get_injured_area_position()
        else:
            target_zone = self._get_supply_area_position()

        if not release_done:
            return Action(
                type=ActionType.TRANSPORT_TO,
                target_position=target_zone,
            )

        # 投放完成
        self._world_map.mark_in_safe_zone(self._current_target.id)
        self._targets_delivered += 1
        self._score += get_point_value(self._current_target.info.type)
        self._trips_completed += 1
        self._current_target = None

        return Action(type=ActionType.WAIT, detail="投放完成，选择下一目标")

    # ---- 强制分离恢复 ----

    def handle_forced_separation(self, new_pose: Tuple[float, float, float]) -> None:
        """
        处理强制分离。

        裁判将机器人放回出发区后调用。
        """
        self._strategy_state = StrategyState.FORCED_RESET
        # 保留当前目标和进度
        logger.warning(f"强制分离! 重置位姿到 ({new_pose[0]:.0f}, {new_pose[1]:.0f})")
        # 不清除 _current_target — 恢复后继续

    def _handle_forced_reset(self) -> Action:
        """处理强制分离恢复"""
        self._strategy_state = StrategyState.FREE_RUN
        logger.info("强制分离恢复完成，继续运行")
        if self._current_target:
            return Action(
                type=ActionType.NAVIGATE_TO,
                target_position=self._current_target.position,
            )
        return Action(type=ActionType.WAIT)

    # ---- 异常处理 ----

    def _handle_anomaly(self, report: AnomalyReport) -> Action:
        """处理异常：不再直接停止，改为降级保活"""
        logger.error("处理异常: %s → %s", report.type.name, report.recovery_action.name)

        if report.type == AnomalyType.NO_ACTION_15S:
            self._fallback_level = FallbackLevel.SURVIVAL
            pos = self._get_survival_target(1500, 1500)
            logger.warning("15s异常 → 保命绕圈")
            return Action(type=ActionType.NAVIGATE_TO,
                          target_position=pos,
                          detail="保命: 绕圈移动")

        if report.recovery_action == RecoveryAction.EMERGENCY_STOP:
            self._fallback_level = FallbackLevel.SURVIVAL
            pos = self._get_survival_target(1500, 1500)
            logger.warning("紧急停止 → 降级为保命绕圈")
            return Action(type=ActionType.NAVIGATE_TO,
                          target_position=pos,
                          detail="保命: 紧急降级")

        if report.recovery_action == RecoveryAction.ESCAPE_MANEUVER:
            self._anomaly.start_escape()
            self._last_action_time = time.time()
            return Action(type=ActionType.WAIT,
                          detail="脱困中: %s" % report.detail)

        if report.recovery_action == RecoveryAction.DEGRADE_SENSORS:
            self._fallback_level = FallbackLevel.EXPLORE
            logger.warning("传感器降级 → 探索模式")
            return Action(type=ActionType.WAIT,
                          detail="传感器降级: %s" % report.detail)

        return Action(type=ActionType.WAIT, detail=str(report))

    # ---- 区域位置 ----

    def _check_fallback_needed(self, rx: float, ry: float, action: Action) -> bool:
        """检查是否需要降级"""
        if action.type == ActionType.WAIT:
            idle_time = time.time() - self._last_action_time
            if idle_time > 10:
                self._fallback_level = FallbackLevel.EXPLORE
                logger.warning("10s无动作 → 探索模式")
                return True
            if idle_time > 13:
                self._fallback_level = FallbackLevel.SURVIVAL
                logger.warning("13s无动作 → 保命模式!")
                return True
        else:
            self._last_action_time = time.time()
            if self._fallback_level >= FallbackLevel.EXPLORE:
                self._fallback_level = FallbackLevel.NORMAL
                logger.info("恢复运动 → 降级解除")
        return False

    def _get_explore_target(self, rx: float, ry: float) -> tuple:
        """生成探索目标：场地中央 + 随机偏移"""
        cx = 1500 + random.randint(-600, 600)
        cy = 1500 + random.randint(-600, 600)
        cx = max(200, min(2800, cx))
        cy = max(200, min(2200, cy))
        return (cx, cy)

    def _get_survival_target(self, rx: float, ry: float) -> tuple:
        """保命绕圈：以当前位置为中心的圆形路径点"""
        radius = 500
        angle = time.time() % (2 * 3.14159)
        tx = rx + radius * 3.14159 * 0.001  # 微小移动
        ty = ry + radius * 0.001
        return (tx, ty)

    def _get_supply_area_position(self) -> Tuple[float, float]:
        """获取本队物资区中心位置"""
        if self._my_color == SafeZoneColor.RED:
            return (200.0, 2800.0)
        return (2700.0, 2800.0)

    def _get_injured_area_position(self) -> Tuple[float, float]:
        """获取本队伤员区中心位置"""
        if self._my_color == SafeZoneColor.RED:
            return (500.0, 2800.0)
        return (2400.0, 2800.0)

    # ---- 查询 ----

    def get_stats(self) -> dict:
        return {
            "strategy_state": self._strategy_state.name,
            "time_remaining_s": self.time_remaining_s,
            "is_time_pressure": self.is_time_pressure,
            "trips_completed": self._trips_completed,
            "targets_delivered": self._targets_delivered,
            "score": self._score,
        }

    def summary(self) -> str:
        stats = self.get_stats()
        return (
            f"决策引擎: state={stats['strategy_state']}, "
            f"剩余={stats['time_remaining_s']:.0f}s, "
            f"{stats['trips_completed']}趟/{stats['targets_delivered']}个, "
            f"{stats['score']}分"
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
    print("  决策引擎 — Mock 模式测试")
    print("=" * 50)

    from ..perception.world_map import WorldMap
    from ..perception.field_elements import FieldLayout
    from ..perception.target_types import (
        PRELIMINARY_TARGETS, TargetColor, TargetShape,
    )

    field = FieldLayout.standard()
    world_map = WorldMap(field_layout=field)
    engine = DecisionEngine(world_map)

    # 添加模拟目标到世界地图
    targets_config = [
        (TargetColor.GREEN, TargetShape.CUBE, (1000, 1500)),        # 普通物资
        (TargetColor.BLACK, TargetShape.TRIANGULAR_PYRAMID, (1500, 1000)),  # 核心物资
        (TargetColor.ORANGE, TargetShape.CUBOID, (2000, 1800)),     # 伤员
        (TargetColor.LIGHT_BLUE, TargetShape.CUBE, (800, 800)),     # 危险目标
        (TargetColor.GREEN, TargetShape.CUBE, (1200, 2000)),        # 普通物资 2
    ]
    from ..perception.target_types import DetectedTarget
    for i, (color, shape, pos) in enumerate(targets_config):
        info = PRELIMINARY_TARGETS[(color, shape)]
        det = DetectedTarget(id=i+1, info=info, position=pos)
        world_map._create_new_target(det, time.time())

    print(f"\n初始: {engine.summary()}")
    print(f"场上目标: {len(world_map.active_targets)} 个")

    engine.start_match()

    # 测试 1：FIRST_TRIP 选择普通物资
    action = engine.update((500, 500, 0))
    print(f"\n测试 1: FIRST_TRIP action={action.type.name}, {action.detail}")
    print(f"  target: {action.target_position}")
    assert action.type == ActionType.NAVIGATE_TO, "首次应NAVIGATE_TO"
    assert engine.strategy_state == StrategyState.FIRST_TRIP
    print("  ✅ 通过")

    # 测试 2：模拟到达目标 → GRIP
    action = engine.update((1000, 1500, 0), nav_arrived=True)
    print(f"\n测试 2: 到达后 action={action.type.name}")
    assert action.type == ActionType.GRIP
    print("  ✅ 通过")

    # 测试 3：夹取完成 → TRANSPORT_TO
    action = engine.update((1000, 1500, 0), nav_arrived=True, grip_done=True)
    print(f"\n测试 3: 夹取后 action={action.type.name}")
    assert action.type == ActionType.TRANSPORT_TO
    print("  ✅ 通过")

    # 测试 4：投放完成 → FREE_RUN
    action = engine.update((200, 2800, 0), nav_arrived=True, grip_done=True, release_done=True)
    print(f"\n测试 4: 投放后 state={engine.strategy_state.name}")
    assert engine.strategy_state == StrategyState.FREE_RUN, \
        f"应进入FREE_RUN，实际={engine.strategy_state.name}"
    print("  ✅ 通过")

    # 测试 5：FREE_RUN 选择最高分（伤员）
    action = engine.update((200, 2800, 0))
    print(f"\n测试 5: FREE_RUN action={action.type.name}, target={action.target_position}")
    assert action.type == ActionType.NAVIGATE_TO
    print("  ✅ 通过")

    # 测试 6：异常检测
    print(f"\n测试 6: 异常检测")
    anomaly = engine._anomaly.check((500, 500, 0), (0, 0))
    print(f"  正常: type={anomaly.type.name}")
    assert anomaly.type == AnomalyType.NONE
    # 模拟 16 秒无动作
    engine._anomaly._last_action_time = time.time() - 16
    anomaly = engine._anomaly.check((500, 500, 0), (0, 0))
    print(f"  16s无动作: type={anomaly.type.name}, fatal={anomaly.is_fatal}")
    assert anomaly.type == AnomalyType.NO_ACTION_15S
    print("  ✅ 通过")

    print(f"\n最终: {engine.summary()}")
    print(f"\n{'='*50}")
    print("  决策引擎测试全部通过 ✅")
    print(f"{'='*50}")
