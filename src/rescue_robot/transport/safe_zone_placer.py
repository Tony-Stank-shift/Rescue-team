"""
safe_zone_placer.py —— 安全区投放判定

判断投放位置是否正确：
  - 物资 → 物资区 ✓
  - 伤员 → 伤员区 ✓
  - 物资 → 伤员区 ✗（-10 分/个）
  - 伤员 → 物资区 ✗（-10 分/个）
  - 目标未完全进入安全区（含围栏上）→ 不计入
"""

import logging
from dataclasses import dataclass
from enum import Enum, auto
from typing import List, Optional, Tuple

from ..perception.field_elements import (
    FieldLayout, FieldElement, FieldElementType,
    SafeZoneColor, RectRegion,
)
from ..perception.target_types import TargetType, TargetInfo

logger = logging.getLogger("safe_zone_placer")


# ============================================================
# 投放结果
# ============================================================

class PlacementZone(Enum):
    """投放区域分类"""
    SUPPLY_AREA = auto()        # 物资区 ✓
    INJURED_AREA = auto()       # 伤员区 ✓
    ON_FENCE = auto()           # 围栏上 ✗（不计入）
    OUTSIDE = auto()            # 安全区外 ✗
    WRONG_SUPPLY_IN_INJURED = auto()   # 物资放入伤员区 ✗
    WRONG_INJURED_IN_SUPPLY = auto()   # 伤员放入物资区 ✗


@dataclass
class PlacementResult:
    """投放判定结果"""
    zone: PlacementZone
    is_valid: bool              # 是否有效投放
    penalty: int = 0            # 扣分（>0 = 扣分）
    target_info: Optional[TargetInfo] = None
    detail: str = ""


# ============================================================
# 安全区投放判定器
# ============================================================

class SafeZonePlacer:
    """
    安全区投放判定器。

    判定逻辑：
    1. 检查目标是否完全在安全区内（进入判定不含围栏上）
    2. 分类目标落在物资区还是伤员区
    3. 校验类型匹配（物资→物资区，伤员→伤员区）
    """

    # "完全进入"判定：目标在安全区内且距围栏边界的边距
    FULLY_INSIDE_MARGIN_MM = 10.0

    def __init__(self, field_layout: FieldLayout,
                 my_color: SafeZoneColor = SafeZoneColor.RED):
        self._field = field_layout
        self._my_color = my_color

        # 缓存我方安全区区域
        self._my_safe_zone: Optional[FieldElement] = None
        self._my_supply_area: Optional[FieldElement] = None
        self._my_injured_area: Optional[FieldElement] = None
        self._cache_zones()

    def _cache_zones(self) -> None:
        """缓存我方安全区的子区域"""
        for elem in self._field.elements:
            if elem.type == FieldElementType.SAFE_ZONE:
                if elem.metadata.get("color_enum") == self._my_color:
                    self._my_safe_zone = elem
            elif elem.type == FieldElementType.SUPPLY_AREA:
                if elem.metadata.get("safe_zone") == self._my_color:
                    self._my_supply_area = elem
            elif elem.type == FieldElementType.INJURED_AREA:
                if elem.metadata.get("safe_zone") == self._my_color:
                    self._my_injured_area = elem

    # ---- 判定接口 ----

    def classify(self, position: Tuple[float, float],
                 target_info: TargetInfo) -> PlacementResult:
        """
        判定目标放置位置的正确性。

        Args:
            position: 目标当前位置 (x_mm, y_mm)
            target_info: 目标信息

        Returns:
            PlacementResult
        """
        x, y = position

        # 步骤 1：是否在安全区内？
        if self._my_safe_zone is None:
            return PlacementResult(
                zone=PlacementZone.OUTSIDE,
                is_valid=False,
                target_info=target_info,
                detail="未找到我方安全区",
            )

        safe_region = self._my_safe_zone.region

        # 检查是否在安全区外
        if not safe_region.contains(x, y):
            # 检查是否在围栏上
            if self._is_on_fence(x, y, safe_region):
                return PlacementResult(
                    zone=PlacementZone.ON_FENCE,
                    is_valid=False,
                    target_info=target_info,
                    detail="目标在围栏上，不计入",
                )
            return PlacementResult(
                zone=PlacementZone.OUTSIDE,
                is_valid=False,
                target_info=target_info,
                detail="目标在安全区外",
            )

        # 检查是否完全进入（距边界有足够边距）
        if not self._is_fully_inside(x, y, safe_region):
            return PlacementResult(
                zone=PlacementZone.ON_FENCE,
                is_valid=False,
                target_info=target_info,
                detail="目标未完全进入安全区（靠近围栏）",
            )

        # 步骤 2：在物资区还是伤员区？
        in_supply = (self._my_supply_area is not None and
                     self._my_supply_area.region.contains(x, y))
        in_injured = (self._my_injured_area is not None and
                      self._my_injured_area.region.contains(x, y))

        # 步骤 3：类型匹配校验
        if target_info.type == TargetType.REGULAR_SUPPLY or \
           target_info.type == TargetType.CORE_SUPPLY:
            # 物资类 → 应在物资区
            if in_injured and not in_supply:
                return PlacementResult(
                    zone=PlacementZone.WRONG_SUPPLY_IN_INJURED,
                    is_valid=False,
                    penalty=10,
                    target_info=target_info,
                    detail=f"物资放入伤员区: -10 分",
                )
            if in_supply:
                return PlacementResult(
                    zone=PlacementZone.SUPPLY_AREA,
                    is_valid=True,
                    target_info=target_info,
                    detail="物资正确放入物资区 ✓",
                )

        elif target_info.type == TargetType.INJURED:
            # 伤员 → 应在伤员区
            if in_supply and not in_injured:
                return PlacementResult(
                    zone=PlacementZone.WRONG_INJURED_IN_SUPPLY,
                    is_valid=False,
                    penalty=10,
                    target_info=target_info,
                    detail=f"伤员放入物资区: -10 分",
                )
            if in_injured:
                return PlacementResult(
                    zone=PlacementZone.INJURED_AREA,
                    is_valid=True,
                    target_info=target_info,
                    detail="伤员正确放入伤员区 ✓",
                )

        # 未匹配具体子区域
        return PlacementResult(
            zone=PlacementZone.OUTSIDE,
            is_valid=False,
            target_info=target_info,
            detail="目标位置未匹配物资区或伤员区",
        )

    def classify_batch(self,
                       positions: List[Tuple[float, float]],
                       target_infos: List[TargetInfo]) -> List[PlacementResult]:
        """批量判定"""
        return [
            self.classify(pos, info)
            for pos, info in zip(positions, target_infos)
        ]

    def is_fully_inside(self, position: Tuple[float, float]) -> bool:
        """目标是否完全进入安全区"""
        if self._my_safe_zone is None:
            return False
        return self._is_fully_inside(
            position[0], position[1],
            self._my_safe_zone.region,
        )

    # ---- 内部 ----

    def _is_fully_inside(self, x: float, y: float,
                         region: RectRegion) -> bool:
        """目标完全在区域内（含边距）"""
        m = self.FULLY_INSIDE_MARGIN_MM
        return (region.x + m <= x <= region.x + region.width - m and
                region.y + m <= y <= region.y + region.height - m)

    def _is_on_fence(self, x: float, y: float,
                     region: RectRegion) -> bool:
        """目标是否在围栏上（接近边界但不在区域内）"""
        # 围栏是安全区外围的紫色三角围栏
        # 简化：距离安全区边界 < 20mm 视为围栏上
        fence_margin = 20.0
        return (region.x - fence_margin <= x <= region.x + region.width + fence_margin and
                region.y - fence_margin <= y <= region.y + region.height + fence_margin)

    # ---- 查询 ----

    def get_my_safe_zone_region(self) -> Optional[RectRegion]:
        if self._my_safe_zone:
            return self._my_safe_zone.region
        return None

    def set_my_color(self, color: SafeZoneColor) -> None:
        self._my_color = color
        self._cache_zones()
