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
VIOLATION_CONSEQUENCES = {
    Violation.NONE:                    ("无", False),
    Violation.FIRST_TRIP_MULTI:        ("本轮结束，本次转运成绩无效", True),
    Violation.FIRST_TRIP_WRONG_TYPE:   ("本轮结束，本次转运成绩无效", True),
    Violation.OVER_LIMIT:              ("本轮结束，本次转运成绩无效", True),
    Violation.INJURED_MULTI:           ("本轮结束，本次转运成绩无效", True),
    Violation.DANGEROUS_TARGET:        ("本轮结束", True),
    Violation.PLACEMENT_WRONG_ZONE:    ("扣 10 分/个，目标放回场地中心", False),
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
      # 夹取前检查
      ok, violation = mgr.can_load(target_info)
      if ok:
          gripper.close()
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

    # ---- 属性 ----

    @property
    def state(self) -> LoadState:
        return self._state

    @property
    def is_first_trip(self) -> bool:
        """是否是首次转运（首次转运完成前）"""
        return self._total_trips == 0

    @property
    def is_first_trip_done(self) -> bool:
        """首次转运是否已完成"""
        return self._total_trips > 0

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
        """批量检查"""
        total_count = self._state.count
        has_injured = self._state.has_injured

        for info in targets:
            if info.type == TargetType.DANGEROUS:
                return (False, Violation.DANGEROUS_TARGET)
            if info.type == TargetType.INJURED:
                if has_injured or total_count > 0:
                    return (False, Violation.INJURED_MULTI)
                has_injured = True
                total_count += 1
            else:
                total_count += 1

        if self.is_first_trip:
            if total_count != 1:
                return (False, Violation.FIRST_TRIP_MULTI)
            if targets[0].type != TargetType.REGULAR_SUPPLY:
                return (False, Violation.FIRST_TRIP_WRONG_TYPE)

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
            placement_ok: 投放位置是否正确（物资入物资区 / 伤员入伤员区）
            points_per_target: {target_id: points} 得分映射

        Returns:
            已释放的目标列表
        """
        released = list(self._state.targets)
        ids = self._state.target_ids.copy()

        if not placement_ok:
            logger.warning(f"投放位置错误: {len(released)} 个目标 — "
                           f"{Violation.PLACEMENT_WRONG_ZONE}")
            # 扣 10 分/个
            penalty = 10 * len(released)
            self._total_score = max(0, self._total_score - penalty)
            logger.warning(f"扣分: -{penalty} (总={self._total_score})")

        # 累计得分
        points = 0
        for info in released:
            p = get_point_value(info.type)
            points += p
        self._total_score += points

        self._total_trips += 1
        self._total_delivered += len(released)

        logger.info(f"投放: {len(released)} 个目标, +{points}分 "
                     f"(总={self._total_score}分, {self._total_delivered}个, "
                     f"{self._total_trips}趟)")

        # 重置状态
        self._state = LoadState()

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

    def reset(self) -> None:
        """完全重置（新比赛开始）"""
        self._state = LoadState()
        self._total_trips = 0
        self._total_delivered = 0
        self._total_score = 0
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
