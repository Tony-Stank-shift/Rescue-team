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

    评分公式（T2-14 起由 `robot.strategy_weights.*` 现场可调）：
      score = points ** exponent * distance_factor * time_factor * opponent_factor

    其中：
      - distance_factor = (1.0 - dist / 4242) ** (distance_weight / 0.3)
      - time_factor = 1.0 + urgency * (2.0 · time_weight/0.2) * (points/15)
      - opponent_factor = opponent_weight if opponent_is_heading_to else 1.0

    ⚠️ **权重映射的设计约束：默认值下必须与旧公式逐位相同**（回归基线
    `score 80/80/85/80/80`、`delivered 7/7/8/7/7` 不能动）。因此这里的映射不是
    "重写评分函数"，而是**以默认 YAML 值为参照点做比例缩放**：

        权重 = 默认值 → 比例 1.0 → 走 `if ratio == 1.0` 分支 → 执行的字面表达式
                                与旧实现**完全一致**（不是"数学上相等"，是不执行任何额外运算）

    现场语义（比例 = 权重 / 默认值，越大越强调该因素）：

      - distance_weight ↑：更强调"就近"（=0 → 距离完全不参与，只看分值/时间）
      - points_weight  ↑：更强调分值（=0.5 基准；↑ → 更愿意跑远拿高分）
      - time_weight    ↑：更早进入时间紧迫模式、末段加成更强（=0 → 时间不参与）
      - opponent_factor ↓：对方盯上的目标被压得更狠（=1.0 → 完全不避让）
    """

    # 场地对角线（最大可能距离）
    MAX_FIELD_DISTANCE_MM = 4242.0  # sqrt(3000^2 + 3000^2)

    # 时间压力阈值
    TIME_PRESSURE_S = 30.0   # 剩余 < 30s → 只运高分
    URGENCY_BONUS_MAX = 2.0  # 时间因子最大加成

    # ── 现场可调策略权重（YAML: robot.strategy_weights.*）──
    #    由 `config.apply_robot_config(cfg)` 通过 `set_weights()` 注入，
    #    注入方式与既有的 `TargetSelector.TIME_PRESSURE_S` 一致（类属性）。
    DISTANCE_WEIGHT = 0.3
    POINTS_WEIGHT = 0.5
    TIME_WEIGHT = 0.2
    OPPONENT_FACTOR = 0.5

    # 参照点 = config/robot.default.yaml 的默认值（比例 1.0 的那个点）。
    # 它们**不是**"第二个可调参数"：只用来把权重换算成与旧公式对齐的比例。
    REF_DISTANCE_WEIGHT = 0.3
    REF_POINTS_WEIGHT = 0.5
    REF_TIME_WEIGHT = 0.2
    # score_target 里 points 的超线性基准指数（旧实现写死的 1.3）
    BASE_POINTS_EXPONENT = 1.3

    @classmethod
    def set_weights(cls, distance_weight: Optional[float] = None,
                    points_weight: Optional[float] = None,
                    time_weight: Optional[float] = None,
                    opponent_factor: Optional[float] = None) -> None:
        """注入现场策略权重（越界值会被夹到 [0,1] 并打 ERROR，不静默接受）。"""
        for name, value in (("distance_weight", distance_weight),
                            ("points_weight", points_weight),
                            ("time_weight", time_weight),
                            ("opponent_factor", opponent_factor)):
            if value is None:
                continue
            attr = name.upper()
            try:
                v = float(value)
            except (TypeError, ValueError):
                logger.error("strategy_weights.%s=%r 不是数字 → 忽略，保持 %s",
                             name, value, getattr(cls, attr))
                continue
            if not 0.0 <= v <= 1.0:
                logger.error("strategy_weights.%s=%s 越界（合法 0~1）→ 夹到边界值",
                             name, v)
                v = min(1.0, max(0.0, v))
            setattr(cls, attr, v)
        logger.info("策略权重已注入: %s", cls.get_weights())

    @classmethod
    def get_weights(cls) -> dict:
        """当前生效的策略权重（供自检/日志核对）。"""
        return {"distance_weight": cls.DISTANCE_WEIGHT,
                "points_weight": cls.POINTS_WEIGHT,
                "time_weight": cls.TIME_WEIGHT,
                "opponent_factor": cls.OPPONENT_FACTOR}

    # ---- 权重 → 公式系数的换算（默认值下恒等）----

    @classmethod
    def _distance_ratio(cls) -> float:
        return float(cls.DISTANCE_WEIGHT) / cls.REF_DISTANCE_WEIGHT

    @classmethod
    def _points_ratio(cls) -> float:
        return float(cls.POINTS_WEIGHT) / cls.REF_POINTS_WEIGHT

    @classmethod
    def _time_ratio(cls) -> float:
        return float(cls.TIME_WEIGHT) / cls.REF_TIME_WEIGHT

    @classmethod
    def _time_pressure_threshold(cls) -> float:
        """有效时间紧迫阈值 = TIME_PRESSURE_S · (time_weight/0.2)。"""
        ratio = cls._time_ratio()
        if ratio == 1.0:
            return float(cls.TIME_PRESSURE_S)
        return float(cls.TIME_PRESSURE_S) * ratio

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
        _dist_ratio = self._distance_ratio()
        if _dist_ratio != 1.0:
            # distance_weight 相对默认值的比例 → 距离因子的幂次
            distance_factor = distance_factor ** _dist_ratio

        # 时间因子：时间越少，高分目标权重越高
        _tps = self._time_pressure_threshold()
        if time_remaining_s < _tps:
            urgency = (_tps - time_remaining_s) / _tps
            _bonus_max = self.URGENCY_BONUS_MAX
            if self._time_ratio() != 1.0:
                _bonus_max = self.URGENCY_BONUS_MAX * self._time_ratio()
            time_factor = 1.0 + urgency * _bonus_max * (points / 15.0)
        else:
            time_factor = 1.0

        # 对手因子（YAML robot.strategy_weights.opponent_factor，默认 0.5 = 旧写死值）
        opponent_factor = (float(self.OPPONENT_FACTOR)
                           if target.id in self._opponent_targets else 1.0)

        # 综合评分：分值超线性(points^1.3)，避免"离得远的高分目标"被"近的低分目标"压过。
        # 危险目标 points=0 → 分数必为 0，被 select_best 的 score>0 过滤。
        _points_factor = points ** self.BASE_POINTS_EXPONENT
        _pw_ratio = self._points_ratio()
        if _pw_ratio != 1.0:
            # points_weight 相对默认值 → points 项整体提权/降权
            _points_factor = points ** (self.BASE_POINTS_EXPONENT * _pw_ratio)
        score = _points_factor * distance_factor * time_factor * opponent_factor

        return ScoredTarget(
            target=target,
            score=score,
            distance_mm=dist,
            is_priority=(time_remaining_s < _tps and points >= 10),
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
                # ⚠️ 必须用 selectable_targets（排除安全区内目标），不能用
                #    active_targets —— 否则会选中已投放进安全区的物资去扑空
                #    （2026-09-18 现场：车贴着禁区边缘干蹭）。
                candidates = world_map.selectable_targets
        else:
            # FREE_RUN：所有**可抓取**的活跃目标（安全区内的已被排除）
            candidates = world_map.selectable_targets

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

    # ── 趟次价值密度模型（用于在"送伤员"与"送物资"之间做取舍）──
    # 旧实现是"伤员绝对优先"：只要场上有伤员，就永远只送最近的那个伤员，
    # 普通/核心物资**一个都不选**。实测（赛题真实配比，180s）：
    #   决赛 10普/5核/5伤 → 只送掉 5 个伤员，10 普通 + 5 核心一个没动。
    # 改为比较"单位时间价值"：伤员 15/(overhead+2d/v) vs 物资趟 Σ分值/(overhead+(d0+簇内跨度+300)/v)。
    TRIP_OVERHEAD_S = 9.0          # 一趟固定开销（对位+套取+投放），来自现场实测估计
    NOMINAL_SPEED_MM_S = 880.0     # 名义巡航速度（mm/s），仅用于估算，不需很准
    SAFE_RETURN_MM = 300.0         # 末段回安全区的估算附加路程（mm）

    def _effective_capacity(self, max_count: int) -> int:
        """
        本趟**真正能带走**几个目标 = min(规则上限, 套取机构物理容量)。

        为什么必须用它：机构只有一个自由度（单只 SG90，套住/释放），实际一趟只能带 1 个。
        若按 3 个估价值密度，会把"3 个 5 分物资"高估成 15 分/趟 → 系统性偏爱物资、
        把 15 分的伤员饿死；反过来若容量真是 3，则按 1 个估算又会低估物资趟。
        """
        try:
            from ..config import Placement as _P
            hold = int(getattr(_P, "SLEEVE_MAX_HOLD", 1))
        except Exception:
            hold = 1
        return max(1, min(int(max_count), max(1, hold)))

    def _trip_time_s(self, robot_position: Tuple[float, float],
                     targets: List[TrackedTarget]) -> float:
        """估算跑完这趟的耗时（秒）：固定开销 + 路程/名义速度。

        `robot.strategy_weights.distance_weight` 在这里生效（T2-14 的生产路径）：
        路程代价按 `distance_weight / 0.3` 缩放 → 权重越大越偏爱"就近的一趟"，
        =0 时距离完全不参与（只看分值密度）。默认值下比例 = 1.0，
        `if ratio != 1.0` 不成立 → **与旧实现的表达式逐位相同**。
        """
        if not targets:
            return float("inf")
        if len(targets) == 1:
            # 单目标：去 + 回安全区（用目标距离近似回程）
            path = 2.0 * self._distance(robot_position, targets[0].position)
        else:
            path = self._distance(robot_position, targets[0].position)
            for a, b in zip(targets, targets[1:]):
                path += self._distance(a.position, b.position)
            path += self.SAFE_RETURN_MM
        _ratio = self._distance_ratio()
        if _ratio != 1.0:
            path = path * _ratio
        return self.TRIP_OVERHEAD_S + path / self.NOMINAL_SPEED_MM_S

    def _value_density(self, robot_position: Tuple[float, float],
                       targets: List[TrackedTarget]) -> float:
        """价值密度 = 本趟总分值 / 估算耗时（分/秒）。

        `robot.strategy_weights.points_weight` 在这里生效：总分值按
        `points_weight / 0.5` 作幂次缩放 → 权重越大越倾向"总分更高的趟次"
        （即使这一趟更远）。默认值下比例 = 1.0 → 走原表达式（int/float 除法，
        与旧实现逐位相同）。

        ⚠️ 这里与 `score_target` 的 points 映射**不同**（那边基准是 points^1.3
        的超线性形态，这边基准是 points/time 的线性比值），原因是两处公式的
        "默认形态"本就不同；共同点是：**权重取默认值时两处都与旧公式逐位相同**。
        """
        points = sum(get_point_value(t.info.type) for t in targets)
        _pw_ratio = self._points_ratio()
        if _pw_ratio != 1.0:
            points = float(points) ** _pw_ratio
        return points / max(self._trip_time_s(robot_position, targets), 1e-6)

    def select_targets_for_trip(self, world_map: WorldMap,
                                 robot_position: Tuple[float, float],
                                 max_count: int = 3,
                                 include_injured: bool = False,
                                 time_remaining_s: float = 180.0) -> List[TrackedTarget]:
        """
        为一趟转运选择目标（按**价值密度**取舍，不再"伤员绝对优先"）。

        规则约束：
        - 伤员必须单独出车（候选里每项最多 1 个伤员，不与任何目标混装）
        - 普通 + 核心可混装，且 ≤ max_count
        - 危险目标 / 已在安全区的目标由 WorldMap 过滤
        """
        candidates: List[List[TrackedTarget]] = []
        cap = self._effective_capacity(max_count)

        # ① 伤员：只能单个成趟（所有伤员各作为一个候选）
        if include_injured:
            for t in world_map.get_injured():
                candidates.append([t])

        # ② 物资：按**实际容量**取就近 1..cap 个作为候选簇，另加"单个最高分"候选
        supplies = (world_map.get_regular_supplies() +
                    world_map.get_core_supplies())
        nearest = sorted(supplies,
                         key=lambda t: self._distance(robot_position, t.position))
        for k in range(1, min(cap, len(nearest)) + 1):
            candidates.append(list(nearest[:k]))
        if supplies:
            best_single = max(
                supplies,
                key=lambda t: (get_point_value(t.info.type),
                               -self._distance(robot_position, t.position)),
            )
            if [best_single] not in candidates:
                candidates.append([best_single])

        if not candidates:
            return []

        # 时间紧迫：只在"高分候选"（≥10 分/趟）里挑，避免末段去捡 5 分目标。
        # 阈值 = TIME_PRESSURE_S ·（time_weight/0.2）：权重越大越早进入紧迫模式，
        # =0 时永不进入（时间因素完全不参与）。默认值下与旧实现同值。
        if time_remaining_s < self._time_pressure_threshold():
            high = [c for c in candidates
                    if sum(get_point_value(t.info.type) for t in c) >= 10]
            if high:
                candidates = high

        best = max(candidates, key=lambda c: self._value_density(robot_position, c))
        logger.debug(
            f"趟次选择: {[t.id for t in best]} "
            f"(密度={self._value_density(robot_position, best):.2f}分/秒, "
            f"耗时≈{self._trip_time_s(robot_position, best):.1f}s)"
        )
        return best

    @staticmethod
    def _distance(p1: Tuple[float, float],
                  p2: Tuple[float, float]) -> float:
        return math.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)
