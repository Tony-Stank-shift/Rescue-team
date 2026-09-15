"""
load_manager.py —— 装载计数与规则校验

转运规则引擎（核心安全模块）：

规则清单（来自赛题 PDF）：
  1. 首次转运必须且仅转运 1 个普通物资至物资区（>1 无效 → 本轮结束）
  2. 之后每次转运 ≤ 3 个（>3 → 本轮结束 + 成绩无效）
  3. 可同时转运普通物资 + 核心物资
  4. 伤员必须单独转运（1 个/次）
  5. 危险目标绝对禁止
  6. 禁止抓取救援目标及将目标放置在机器人上
"""

import logging
from dataclasses import dataclass, field
from enum import Enum, auto
from typing import Dict, List, Optional, Set, Tuple

from ..perception.target_types import TargetType, TargetInfo, get_point_value
from ..config import thresholds as _thresholds

logger = logging.getLogger("load_manager")


# ============================================================
# 违规类型
# ============================================================

class Violation(Enum):
    """转运违规类型"""
    NONE = auto()                        # 无违规
    FIRST_TRIP_MULTI = auto()            # 首次转运超过 1 个
    FIRST_TRIP_WRONG_TYPE = auto()       # 首次转运不是普通物资
    OVER_LIMIT = auto()                  # 单次超过 3 个
    INJURED_MULTI = auto()               # 转运多个伤员
    DANGEROUS_TARGET = auto()            # 夹取了危险目标
    PLACEMENT_WRONG_ZONE = auto()        # 投放位置错误（物资入伤员区/反之）


# 违规后果映射
# 违规后果映射（依据赛项原文，不再自行加码）
#   "多于一个无效 / 超过 3 个无效 / 伤员不单独无效"= 该次转运**无效**，
#   规则并未写"本轮结束"——旧实现把它们标成 is_fatal=True 会导致程序自我终止/卡死。
VIOLATION_CONSEQUENCES = {
    Violation.NONE:                    ("无", False),
    Violation.FIRST_TRIP_MULTI:        ("本次转运无效；首趟必须单独送 1 个普通物资到物资区围栏内侧", False),
    Violation.FIRST_TRIP_WRONG_TYPE:   ("本次转运无效；首趟必须是普通物资", False),
    Violation.OVER_LIMIT:              ("本次转运无效；单次最多 3 个", False),
    Violation.INJURED_MULTI:           ("本次转运无效；伤员必须单独且一次只能 1 个", False),
    Violation.DANGEROUS_TARGET:        ("不得转运危险目标；该目标无效", False),
    Violation.PLACEMENT_WRONG_ZONE:    ("无效目标被取出重新随机放置场地中央（扣分细则待确认）", False),
}


# ============================================================
# 装载状态
# ============================================================

@dataclass
class LoadState:
    """当前装载状态"""
    targets: List[TargetInfo] = field(default_factory=list)  # 装载的目标信息
    target_ids: Set[int] = field(default_factory=set)          # 目标 ID
    count: int = 0
    has_injured: bool = False
    has_dangerous: bool = False
    trip_number: int = 0                  # 第几次转运（首次 = 1）


# ============================================================
# 装载管理器
# ============================================================

class LoadManager:
    """
    装载管理器 + 规则引擎。

    维护当前转运的装载状态，每次装载/卸载时校验规则。

    使用方式：
      mgr = LoadManager()
      # 套取前检查
      ok, violation = mgr.can_load(target_info)
      if ok:
          sleeve.lower()
          mgr.load(target_info, target_id)
      # 投放时
      mgr.release_all()
    """

    # 单次装载上限
    MAX_LOAD = 3

    def __init__(self):
        self._state = LoadState()
        self._total_trips = 0           # 累计转运次数
        self._total_delivered = 0       # 累计投放目标数
        self._total_score = 0           # 累计得分
        # 首趟是否**有效**完成（唯一判据；见 is_first_trip）
        self._first_trip_done = False
        # 最近一次投放结果（供 DecisionEngine/AutonomousState 判定是否入区）
        self._last_release_valid: Optional[bool] = None
        self._last_release_types: List[TargetType] = []

    # ---- 属性 ----

    @property
    def state(self) -> LoadState:
        return self._state

    @property
    def is_first_trip(self) -> bool:
        """
        是否仍处于"首趟未有效完成"状态。

        ⚠️ 判据是 `_first_trip_done`（**首趟有效送达后**才置位），
        不是"跑过一趟"。旧实现用 `_total_trips == 0`：
          - 只要 release_all() 被调用过（哪怕空装载、哪怕投在围栏外）就不再算首趟
            → "出发后必须先单独送 1 个普通物资到物资区围栏内侧"这条硬规则被绕过。
        """
        return not self._first_trip_done

    @property
    def is_first_trip_done(self) -> bool:
        """首趟是否**有效**完成（1 个普通物资送进本队物资区围栏内侧）"""
        return self._first_trip_done

    def mark_first_trip_done(self) -> None:
        """显式标记首趟已完成（供自测/复位使用，正常流程由 release_all 判定）。"""
        self._first_trip_done = True

    @property
    def current_count(self) -> int:
        return self._state.count

    @property
    def has_injured(self) -> bool:
        return self._state.has_injured

    @property
    def trip_number(self) -> int:
        return self._state.trip_number or (self._total_trips + 1)

    @property
    def total_trips(self) -> int:
        return self._total_trips

    @property
    def total_delivered(self) -> int:
        return self._total_delivered

    @property
    def total_score(self) -> int:
        return self._total_score

    @property
    def last_release_valid(self) -> Optional[bool]:
        """最近一次投放是否有效（None=还没投放过）"""
        return self._last_release_valid

    @property
    def last_release_types(self) -> List[TargetType]:
        return list(self._last_release_types)

    # ---- 规则校验 ----

    def can_load(self, target_info: TargetInfo,
                 count: int = 1) -> Tuple[bool, Violation]:
        """
        检查是否可以装载指定目标。

        Args:
            target_info: 目标信息
            count: 拟装载数量

        Returns:
            (ok: bool, violation: Violation)
        """
        # 规则 5：危险目标绝对禁止
        if target_info.type == TargetType.DANGEROUS:
            return (False, Violation.DANGEROUS_TARGET)

        # 规则 1：首次转运检查
        if self.is_first_trip:
            # 必须是普通物资
            if target_info.type != TargetType.REGULAR_SUPPLY:
                return (False, Violation.FIRST_TRIP_WRONG_TYPE)
            # 必须恰好 1 个
            if count != 1 or self._state.count + count > 1:
                return (False, Violation.FIRST_TRIP_MULTI)

        # 规则 2：单次 ≤ 3 个
        new_total = self._state.count + count
        if new_total > self.MAX_LOAD:
            return (False, Violation.OVER_LIMIT)

        # 规则 3：混合转运检查（普通+核心可混合 ✓）
        # 无需额外检查

        # 规则 4：伤员必须单独
        if target_info.type == TargetType.INJURED:
            if self._state.count > 0 or count > 1:
                return (False, Violation.INJURED_MULTI)

        # 如果当前已装载伤员，不能再装其他
        if self._state.has_injured and target_info.type != TargetType.INJURED:
            return (False, Violation.INJURED_MULTI)

        return (True, Violation.NONE)

    def can_load_batch(self, targets: List[TargetInfo]) -> Tuple[bool, Violation]:
        """
        批量检查整趟装载是否合规。

        ⚠️ 判定必须与顺序无关。旧实现按列表顺序逐项累加，导致
        `[伤员, 普通]` 被放行（先看到伤员时车上还是空的），
        而 `[普通, 伤员]` 才被拦下 —— 同一趟货因顺序不同结论相反，
        直接违反"伤员必须单独转运且一次只能转运 1 个"。
        """
        # 空批次 = 没有任何装载动作：直接放行（也不触碰 targets[0]，避免 IndexError）
        if not targets:
            return (True, Violation.NONE)

        # 先整体统计本批次构成，再做与顺序无关的判定
        injured_in_batch = 0
        other_in_batch = 0
        for info in targets:
            if info.type == TargetType.DANGEROUS:
                return (False, Violation.DANGEROUS_TARGET)
            if info.type == TargetType.INJURED:
                injured_in_batch += 1
            else:
                other_in_batch += 1

        # 规则 4：伤员必须单独转运，且一次只能 1 个
        #   - 本批含伤员 → 本批只能有这 1 个伤员，且必须空车开局
        #   - 车上已装伤员 → 本批不得再装任何东西
        if injured_in_batch:
            if injured_in_batch > 1 or other_in_batch > 0:
                return (False, Violation.INJURED_MULTI)
            if self._state.count > 0 or self._state.has_injured:
                return (False, Violation.INJURED_MULTI)
        elif self._state.has_injured:
            return (False, Violation.INJURED_MULTI)

        total_count = self._state.count + len(targets)

        # 规则 1：首次转运必须且仅转运 1 个普通物资
        #
        # ⚠️ 曾经考虑过"解除首趟对其它批次的连带封杀"（见 INCREMENTAL_AUDIT 的 N-6 建议①），
        # **实测证明那是空操作**：本判据与"只在本批含非普通物资时拒绝"逐项等价
        # （[1普通]通过；[2普通]、[普通+核心] → MULTI；[1核心]、[1伤员] → WRONG_TYPE），
        # 因为规则本身就是"首趟必须且仅送 1 个普通物资，完成后才能碰核心/伤员"，
        # **不能放宽**（放宽 = 直接违规）。
        # 所以 N-6 的正解不是改这里，而是**让首趟真的成功**：
        # 落点投影钳回本队子区域（transport_pipeline PLACING）+ 前伸量默认值降到与
        # 区域几何相容（YAML `drop_forward_mm`）。首趟成功率上去了，本分支自然不再是
        # "永久封杀"。
        if self.is_first_trip:
            if total_count != 1:
                return (False, Violation.FIRST_TRIP_MULTI)
            if targets[0].type != TargetType.REGULAR_SUPPLY:
                return (False, Violation.FIRST_TRIP_WRONG_TYPE)

        # 规则 2：单次 ≤ 3 个
        if total_count > self.MAX_LOAD:
            return (False, Violation.OVER_LIMIT)

        return (True, Violation.NONE)

    # ---- 装载操作 ----

    def load(self, target_info: TargetInfo, target_id: int) -> Tuple[bool, Violation]:
        """
        装载单个目标（夹爪闭合成功后调用）。

        Returns:
            (ok, violation): 成功则 ok=True, violation=NONE
        """
        ok, violation = self.can_load(target_info)
        if not ok:
            logger.warning(f"装载拒绝: {violation.name} — "
                           f"target={target_info.description}")
            return (False, violation)

        self._state.targets.append(target_info)
        self._state.target_ids.add(target_id)
        self._state.count += 1
        self._state.trip_number = self._total_trips + 1

        if target_info.type == TargetType.INJURED:
            self._state.has_injured = True

        logger.info(f"装载: {target_info.description} (ID={target_id}), "
                     f"当前={self._state.count}/{self.MAX_LOAD}, "
                     f"第{self.trip_number}趟")
        return (True, Violation.NONE)

    # ---- 投放操作 ----

    def release_all(self,
                    placement_ok: bool = True,
                    points_per_target: Optional[Dict[int, int]] = None) -> List[TargetInfo]:
        """
        释放所有装载目标（夹爪打开后调用）。

        Args:
            placement_ok: 投放位置是否正确（物资入物资区 / 伤员入伤员区），
                由调用方按**目标落点**判定后传入
            points_per_target: {target_id: points} 得分映射（保留参数）

        Returns:
            已释放的目标列表（空装载/无效投放时语义见下）

        规则语义（依据赛项原文）：
          - 空装载调用：什么都不做，**不计趟次**（旧实现照样 +1，
            把"首趟未完成"状态清掉 → 首趟硬规则被绕过）；
          - 投放到错误区域：目标不计分，并按 penalty_per_target 扣分（❓数值待确认）；
          - 首趟只有在"恰好 1 个普通物资 + 有效投进本队物资区围栏内侧"时才算完成，
            否则保持 `is_first_trip=True`，必须重做首趟。
        """
        released = list(self._state.targets)

        # ── 空装载：不是一趟转运，直接返回，不污染趟次/首趟状态 ──
        if not released:
            logger.info("release_all(): 当前无装载目标 → 忽略（不计趟次）")
            return []

        was_first_trip = self.is_first_trip
        self._last_release_valid = bool(placement_ok)
        self._last_release_types = [i.type for i in released]

        if not placement_ok:
            # 扣分（数值来源不明，见 config.Thresholds.PLACEMENT_PENALTY_PER_TARGET）
            penalty_per = getattr(
                _thresholds, "PLACEMENT_PENALTY_PER_TARGET", 10)
            penalty = penalty_per * len(released)
            self._total_score = max(0, self._total_score - penalty)
            logger.warning(f"投放位置错误: {len(released)} 个目标 → 不计分, "
                           f"扣 {penalty} 分 (总={self._total_score})")

        # 累计得分：只有**投放到正确区域**的目标才计分（旧实现投错了也照样加分，
        # 先扣 10 再全部加回 → 惩罚被抵消，净收益仍为正）
        points = 0
        if placement_ok:
            for info in released:
                points += get_point_value(info.type)
            self._total_score += points

        self._total_trips += 1
        self._total_delivered += len(released)

        logger.info(f"投放: {len(released)} 个目标, +{points}分 "
                     f"(总={self._total_score}分, {self._total_delivered}个, "
                     f"{self._total_trips}趟)")

        # ── 首趟闭环判定 ──
        if was_first_trip:
            first_ok = (
                placement_ok
                and len(released) == 1
                and released[0].type == TargetType.REGULAR_SUPPLY
            )
            if first_ok:
                self._first_trip_done = True
                logger.info("✅ 首趟有效完成（1 个普通物资已进本队物资区围栏内侧）")
            else:
                logger.error(
                    "❌ 首趟无效（要求：单独 1 个普通物资，且投进本队物资区围栏内侧）"
                    "→ 必须重做首趟；在首趟有效完成前，其它目标一律无效"
                )

        # 重置装载状态
        self._state = LoadState()

        return released

        return released

    # ---- 查询 ----

    def get_target_ids(self) -> Set[int]:
        """获取当前装载的目标 ID 列表"""
        return self._state.target_ids.copy()

    def get_violation_info(self, violation: Violation) -> Tuple[str, bool]:
        """获取违规信息"""
        return VIOLATION_CONSEQUENCES.get(
            violation, ("未知违规", False)
        )

    def discard_load(self) -> List[int]:
        """作废当前装载（清空货舱但不计趟次、不计分）。

        T1-5/T1-7：本趟被判作废（VIOLATION）或投放无效时，目标其实仍在场上，
        不能让它们留在"已装载"台账里（否则容量/首趟判定被污染，且目标状态永远
        停在 BEING_TRANSPORTED）。与 `release_all()` 的区别：**不 +1 趟、不结算分数**。
        """
        ids = sorted(self._state.target_ids)
        if not ids:
            return []
        logger.warning(f"作废当前装载（不计趟次）: {ids}")
        self._state.targets = []
        self._state.target_ids = set()
        self._state.count = 0
        self._state.has_injured = False
        self._state.has_dangerous = False
        return ids

    def reset(self) -> None:
        """完全重置（新比赛开始）"""
        self._state = LoadState()
        self._total_trips = 0
        self._total_delivered = 0
        self._total_score = 0
        self._first_trip_done = False
        self._last_release_valid = None
        self._last_release_types = []
        logger.info("LoadManager 完全重置")

    def summary(self) -> str:
        return (
            f"装载状态: {self._state.count}/{self.MAX_LOAD} 个, "
            f"第{self.trip_number}趟, "
            f"伤员={'是' if self._state.has_injured else '否'}, "
            f"首次={'是' if self.is_first_trip else '否'} | "
            f"累计: {self._total_trips}趟, {self._total_delivered}个, "
            f"{self._total_score}分"
        )
