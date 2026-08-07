"""
target_selector.py —— 目标优先级排序

选择最优转运目标，综合考虑：
  - 分值：伤员(15) > 核心(10) > 普通(5)
  - 距离：越近越好
  - 时间压力：时间紧迫时高分目标权重加大
  - 对手因素：对方正在接近的目标适当降权
"""

import logging
import math
from dataclasses import dataclass
from enum import Enum, auto
from typing import List, Optional, Set, Tuple

from ..perception.target_types import TargetType, get_point_value
from ..perception.world_map import WorldMap, TrackedTarget

logger = logging.getLogger("target_selector")


class StrategyState(Enum):
    """策略状态"""
    FIRST_TRIP = auto()       # 首次转运
    FREE_RUN = auto()         # 自由转运
    TIME_PRESSURE = auto()    # 时间紧迫
    FORCED_RESET = auto()     # 强制分离恢复
    DONE = auto()             # 完成
    ANOMALY = auto()          # 异常


@dataclass
class ScoredTarget:
    """评分后的目标"""
    target: TrackedTarget
    score: float
    distance_mm: float
    is_priority: bool = False   # 是否高优先级


class TargetSelector:
    """
    目标选择器。

    评分公式：
      score = points * distance_factor * time_factor * opponent_factor

    其中：
      - distance_factor = 1.0 - (dist / 3000)  # 越近越高
      - time_factor = 1.0 + urgency_bonus      # 时间紧时高分目标提权
      - opponent_factor = 0.5 if opponent_is_heading_to else 1.0
    """

    # 场地对角线（最大可能距离）
    MAX_FIELD_DISTANCE_MM = 4242.0  # sqrt(3000^2 + 3000^2)

    # 时间压力阈值
    TIME_PRESSURE_S = 30.0   # 剩余 < 30s → 只运高分
    URGENCY_BONUS_MAX = 2.0  # 时间因子最大加成

    def __init__(self):
        self._opponent_targets: Set[int] = set()  # 对方正在前往的目标 ID

    def set_opponent_target(self, target_id: int) -> None:
        """标记对方正在前往的目标"""
        self._opponent_targets.add(target_id)

    def clear_opponent_targets(self) -> None:
        self._opponent_targets.clear()

    def score_target(self, target: TrackedTarget,
                     robot_position: Tuple[float, float],
                     time_remaining_s: float = 180.0) -> ScoredTarget:
        """
        对单个目标评分。

        Returns:
            ScoredTarget: 含评分的目标
        """
        # 基础分值
        points = float(get_point_value(target.info.type))
        if points <= 0:
            return ScoredTarget(target=target, score=-1.0, distance_mm=0)

        # 距离因子
        dist = self._distance(robot_position, target.position)
        distance_factor = max(0.05, 1.0 - dist / self.MAX_FIELD_DISTANCE_MM)

        # 时间因子：时间越少，高分目标权重越高
        if time_remaining_s < self.TIME_PRESSURE_S:
            urgency = (self.TIME_PRESSURE_S - time_remaining_s) / self.TIME_PRESSURE_S
            time_factor = 1.0 + urgency * self.URGENCY_BONUS_MAX * (points / 15.0)
        else:
            time_factor = 1.0

        # 对手因子
        opponent_factor = 0.5 if target.id in self._opponent_targets else 1.0

        # 综合评分
        score = points * distance_factor * time_factor * opponent_factor

        return ScoredTarget(
            target=target,
            score=score,
            distance_mm=dist,
            is_priority=(time_remaining_s < self.TIME_PRESSURE_S and points >= 10),
        )

    def select_best(self, world_map: WorldMap,
                    robot_position: Tuple[float, float],
                    strategy_state: StrategyState,
                    time_remaining_s: float = 180.0) -> Optional[TrackedTarget]:
        """
        选择最优目标。

        Args:
            world_map: 世界地图
            robot_position: 机器人位置
            strategy_state: 当前策略状态
            time_remaining_s: 剩余时间

        Returns:
            最优目标或 None
        """
        # 根据策略状态获取候选目标
        if strategy_state == StrategyState.FIRST_TRIP:
            candidates = world_map.get_regular_supplies()
            if not candidates:
                logger.warning("FIRST_TRIP: 场上无普通物资!")
                return None
        elif strategy_state == StrategyState.TIME_PRESSURE:
            # 时间紧迫：只选高分（核心 + 伤员）
            candidates = (world_map.get_injured() +
                          world_map.get_core_supplies())
            if not candidates:
                # fallback：选最近的
                candidates = world_map.active_targets
        else:
            # FREE_RUN：所有活跃目标
            candidates = world_map.active_targets

        if not candidates:
            return None

        # 评分
        scored = [
            self.score_target(t, robot_position, time_remaining_s)
            for t in candidates
        ]
        # 排除负分（危险目标）
        scored = [s for s in scored if s.score > 0]
        if not scored:
            return None

        # 按评分降序排列
        scored.sort(key=lambda s: -s.score)

        best = scored[0]
        logger.debug(
            f"目标选择: {best.target.info.description}, "
            f"score={best.score:.1f}, dist={best.distance_mm:.0f}mm, "
            f"priority={best.is_priority}, "
            f"state={strategy_state.name}"
        )
        return best.target

    def select_best_for_first_trip(self, world_map: WorldMap,
                                    robot_position: Tuple[float, float]) -> Optional[TrackedTarget]:
        """首次转运专用：只选最近的普通物资"""
        supplies = world_map.get_regular_supplies()
        if not supplies:
            return None

        rx, ry = robot_position
        nearest = min(supplies,
                      key=lambda t: (t.position[0] - rx) ** 2 +
                                    (t.position[1] - ry) ** 2)
        return nearest

    def select_targets_for_trip(self, world_map: WorldMap,
                                 robot_position: Tuple[float, float],
                                 max_count: int = 3,
                                 include_injured: bool = False,
                                 time_remaining_s: float = 180.0) -> List[TrackedTarget]:
        """
        为一趟转运选择多个目标。

        规则：
        - FIRST_TRIP 时返回 1 个普通物资
        - 伤员单独出车
        - 普通+核心可混合（≤3个）
        """
        if world_map.active_targets is None:
            return []

        # 时间紧迫时优先伤员
        if time_remaining_s < self.TIME_PRESSURE_S:
            injured = world_map.get_injured()
            if injured:
                nearest_injured = min(
                    injured,
                    key=lambda t: self._distance(robot_position, t.position),
                )
                return [nearest_injured]

        # 伤员优先级最高
        if include_injured:
            injured = world_map.get_injured()
            if injured:
                nearest = min(injured,
                              key=lambda t: self._distance(robot_position, t.position))
                return [nearest]

        # 普通+核心混合
        all_supplies = (world_map.get_regular_supplies() +
                        world_map.get_core_supplies())
        all_supplies.sort(
            key=lambda t: (-get_point_value(t.info.type),
                           self._distance(robot_position, t.position))
        )
        return all_supplies[:max_count]

    @staticmethod
    def _distance(p1: Tuple[float, float],
                  p2: Tuple[float, float]) -> float:
        return math.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)
