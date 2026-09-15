"""
forbidden_zones.py —— 禁区管理

定义和管理导航禁区：
  - 对方安全区（进入扣 5 分/次）
  - 场地边界
  - 静态障碍区

对接 path_planner.CostMap，将禁区写入代价地图。
"""

import logging
import math
from dataclasses import dataclass
from typing import List, Optional, Tuple

from ..perception.field_elements import (
    FieldLayout, FieldElement, FieldElementType,
    SafeZoneColor, RectRegion, FIELD_SIZE, FENCE_HEIGHT,
)

logger = logging.getLogger("forbidden_zones")


# ============================================================
# 禁区定义
# ============================================================

@dataclass
class ForbiddenZone:
    """单个禁区"""
    name: str                    # 名称
    region: RectRegion           # 区域
    reason: str                  # 禁止原因
    penalty: str = ""            # 违规惩罚
    is_hard: bool = True         # True=绝对不可进入, False=高风险

    def contains(self, x: float, y: float) -> bool:
        return self.region.contains(x, y)


# ============================================================
# 禁区管理器
# ============================================================

class ForbiddenZoneManager:
    """
    禁区管理器。

    维护所有导航禁区列表，提供安全检查接口。
    每个更新周期将禁区写入 CostMap。
    """

    # 对方安全区外扩的安全边距
    SAFE_MARGIN_MM = 50.0

    # 场地边界安全边距
    BOUNDARY_MARGIN_MM = 100.0

    def __init__(self, field_layout: FieldLayout,
                 my_color: SafeZoneColor = SafeZoneColor.RED):
        self._field = field_layout
        self._my_color = my_color
        self._zones: List[ForbiddenZone] = []
        self._build_zones()

        logger.info(f"ForbiddenZoneManager 初始化: my_color={my_color.name}, "
                     f"{len(self._zones)} 个禁区")

    def _build_zones(self) -> None:
        """从场地布局构建禁区列表"""
        self._zones.clear()

        opponent_color = (SafeZoneColor.BLUE if self._my_color == SafeZoneColor.RED
                          else SafeZoneColor.RED)

        # 对方安全区（含扩展边距）
        for elem in self._field.elements:
            if elem.type == FieldElementType.SAFE_ZONE:
                if elem.metadata.get("color_enum") == opponent_color:
                    r = elem.region
                    expanded = RectRegion(
                        x=r.x - self.SAFE_MARGIN_MM,
                        y=r.y - self.SAFE_MARGIN_MM,
                        width=r.width + 2 * self.SAFE_MARGIN_MM,
                        height=r.height + 2 * self.SAFE_MARGIN_MM,
                    )
                    self._zones.append(ForbiddenZone(
                        name=f"对方安全区 ({opponent_color.name})",
                        region=expanded,
                        reason="进入对方安全区",
                        penalty="-5 分/次",
                        is_hard=True,
                    ))

        # 场地边界
        margin = self.BOUNDARY_MARGIN_MM
        field_size = FIELD_SIZE

        # 底边
        self._zones.append(ForbiddenZone(
            name="场地底边界",
            region=RectRegion(-margin, -margin, field_size + 2 * margin, margin),
            reason="驶出场地",
            penalty="比赛结束",
            is_hard=True,
        ))
        # 顶边
        self._zones.append(ForbiddenZone(
            name="场地顶边界",
            region=RectRegion(-margin, field_size, field_size + 2 * margin, margin * 2),
            reason="驶出场地",
            penalty="比赛结束",
            is_hard=True,
        ))
        # 左边
        self._zones.append(ForbiddenZone(
            name="场地左边界",
            region=RectRegion(-margin, 0, margin, field_size),
            reason="驶出场地",
            penalty="比赛结束",
            is_hard=True,
        ))
        # 右边
        self._zones.append(ForbiddenZone(
            name="场地右边界",
            region=RectRegion(field_size, 0, margin * 2, field_size),
            reason="驶出场地",
            penalty="比赛结束",
            is_hard=True,
        ))

    # ---- 查询接口 ----

    def is_safe(self, x: float, y: float) -> bool:
        """检查位置是否安全（不在任何 hard 禁区内）"""
        for zone in self._zones:
            if zone.is_hard and zone.contains(x, y):
                return False
        return True

    def is_in_field(self, x: float, y: float) -> bool:
        """检查是否在场内"""
        return (0 <= x <= FIELD_SIZE and 0 <= y <= FIELD_SIZE)

    def check_violation(self, x: float, y: float) -> Optional[ForbiddenZone]:
        """检查是否进入禁区，返回第一个违规禁区"""
        for zone in self._zones:
            if zone.is_hard and zone.contains(x, y):
                return zone
        return None

    # 把点推出禁区时额外留的余量（mm）。
    # RectRegion.contains 是闭区间（px <= x_max 才算在内），所以必须 > 0，
    # 否则会"推出到边界上"仍被判为在禁区内，形成死循环。
    CLAMP_EPS_MM = 1.0

    def clamp_to_safe(self, x: float, y: float,
                      margin_mm: float = 0.0) -> Tuple[float, float]:
        """
        把落在 hard 禁区内的点推到禁区外最近的合法位置。

        用途：决策/转运给出的导航目标点若落在对方安全区（或场外）内部，
        **必须**先钳制掉再下发，否则车会径直开进对方安全区
        （赛项：不能进入对方安全区，否则比赛结束）。

        多禁区重叠时迭代推出（最多 8 次）；仍无法推出时退到场地中心一带。
        """
        eps = max(margin_mm, self.CLAMP_EPS_MM)
        for _ in range(8):
            zone = self.check_violation(x, y)
            if zone is None:
                return (x, y)
            r = zone.region
            candidates = [
                (r.x - eps, y),                 # 往左推出
                (r.x + r.width + eps, y),       # 往右推出
                (x, r.y - eps),                 # 往下推出
                (x, r.y + r.height + eps),      # 往上推出
            ]
            nxt = min(candidates, key=lambda p: (p[0] - x) ** 2 + (p[1] - y) ** 2)
            if nxt == (x, y):
                break
            x, y = nxt

        # 兜底：推到场地中心；若中心也在禁区内则取场地四分之一处
        for fallback in ((FIELD_SIZE / 2, FIELD_SIZE / 2),
                         (FIELD_SIZE * 0.25, FIELD_SIZE * 0.25)):
            if self.check_violation(*fallback) is None:
                logger.error(f"目标点无法推出禁区，回退到 {fallback}")
                return fallback
        return (x, y)

    def get_violation_warning(self, x: float, y: float,
                              warning_distance_mm: float = 150.0) -> Optional[ForbiddenZone]:
        """检查是否接近禁区（预警距离内）"""
        for zone in self._zones:
            if not zone.is_hard:
                continue
            # 检测是否在扩展预警区域内
            r = zone.region
            expanded = RectRegion(
                x=r.x - warning_distance_mm,
                y=r.y - warning_distance_mm,
                width=r.width + 2 * warning_distance_mm,
                height=r.height + 2 * warning_distance_mm,
            )
            if expanded.contains(x, y) and not zone.contains(x, y):
                return zone
        return None

    def distance_to_nearest_forbidden(self, x: float, y: float) -> float:
        """到达最近禁区的距离（mm）"""
        min_dist = float('inf')
        for zone in self._zones:
            if not zone.is_hard:
                continue
            r = zone.region
            # 矩形外最近距离
            dx = max(r.x - x, 0, x - r.x - r.width)
            dy = max(r.y - y, 0, y - r.y - r.height)
            dist = math.sqrt(dx * dx + dy * dy)
            min_dist = min(min_dist, dist)
        return min_dist

    # ---- 更新 ----

    def set_my_color(self, color: SafeZoneColor) -> None:
        """更新本队安全区颜色（抽签后调用）"""
        if color != self._my_color:
            self._my_color = color
            self._build_zones()
            logger.info(f"切换安全区颜色: {color.name}")

    def write_to_cost_map(self, cost_map, cost: int = 255) -> None:
        """将禁区写入代价地图"""
        for zone in self._zones:
            if zone.is_hard:
                r = zone.region
                cost_map.add_forbidden_rect(r.x, r.y, r.width, r.height)

    def get_all_zones(self) -> List[ForbiddenZone]:
        return list(self._zones)

    def summary(self) -> str:
        lines = [f"禁区管理器 (本队={self._my_color.name}):"]
        for zone in self._zones:
            lines.append(f"  {'🚫' if zone.is_hard else '⚠️'} {zone.name}: "
                         f"{zone.reason} ({zone.penalty})")
        return "\n".join(lines)
