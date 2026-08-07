"""
field_elements.py —— 场地元素定义

定义比赛场地的完整几何模型：
  - 场地边界（3000×3000mm + 围栏）
  - 出发区 ×4（洋红色）
  - 安全区 ×2（红/蓝，各含物资区 + 伤员区）
  - 紫色三角形围栏
  - 减速带 ×3
"""

import logging
from dataclasses import dataclass, field
from enum import Enum, auto
from typing import List, Optional, Tuple

logger = logging.getLogger("field_elements")


# ============================================================
# 场地元素枚举
# ============================================================

class SafeZoneColor(Enum):
    """安全区颜色"""
    RED = "red"
    BLUE = "blue"


class FieldElementType(Enum):
    """场地元素类型"""
    FIELD_BOUNDARY = auto()      # 场地边界
    START_ZONE = auto()          # 出发区
    SAFE_ZONE = auto()           # 安全区
    SUPPLY_AREA = auto()         # 物资区（安全区内）
    INJURED_AREA = auto()        # 伤员区（安全区内）
    DIVIDER = auto()             # 隔板
    PURPLE_FENCE = auto()        # 紫色三角围栏
    SPEED_BUMP = auto()          # 减速带
    OBSTACLE = auto()            # 静态障碍物


# ============================================================
# 几何区域
# ============================================================

@dataclass
class RectRegion:
    """矩形区域（轴对齐）"""
    x: float          # 左下角 X（mm）
    y: float          # 左下角 Y（mm）
    width: float      # 宽（mm）
    height: float     # 高（mm）

    @property
    def center(self) -> Tuple[float, float]:
        return (self.x + self.width / 2, self.y + self.height / 2)

    @property
    def x_max(self) -> float:
        return self.x + self.width

    @property
    def y_max(self) -> float:
        return self.y + self.height

    def contains(self, px: float, py: float) -> bool:
        """判断点是否在区域内"""
        return (self.x <= px <= self.x_max and
                self.y <= py <= self.y_max)

    def overlaps(self, other: "RectRegion") -> bool:
        """判断两个矩形是否重叠"""
        return (self.x < other.x_max and self.x_max > other.x and
                self.y < other.y_max and self.y_max > other.y)


@dataclass
class FieldElement:
    """单个场地元素"""
    id: int
    type: FieldElementType
    region: RectRegion
    color: str = ""          # 颜色描述
    label: str = ""          # 标签（如"出发区 1"）
    metadata: dict = field(default_factory=dict)


# ============================================================
# 标准场地布局
# ============================================================

# 场地尺寸常量（mm）
FIELD_SIZE = 3000
FENCE_THICKNESS = 20
FENCE_HEIGHT = 100

# 出发区
START_ZONE_SIZE = 300       # 300×300mm
START_ZONE_COLOR = "洋红色"

# 安全区
SAFE_ZONE_WIDTH = 600       # 估算值
SAFE_ZONE_HEIGHT = 800      # 估算值
DIVIDER_SIZE = (300, 20, 20)  # 长×宽×高

# 减速带
SPEED_BUMP_WIDTH = 400      # 估算值
SPEED_BUMP_SPACING = 50     # 间隔
SPEED_BUMP_COUNT = 3


class StandardFieldLayout:
    """
    标准比赛场地布局。

    坐标系：场地左下角为原点 (0, 0)，右上角为 (3000, 3000)。
    机器人前方为 +Y，右方为 +X。

    场地示意（俯视图）：
    ```
    3000 ┌───────────────────────────────────┐
         │      安全区(红)      安全区(蓝)      │
         │   ┌──┬──┐         ┌──┬──┐        │
         │   │物│伤│         │物│伤│        │
         │   └──┴──┘         └──┴──┘        │
         │                                    │
         │  ① 出发区              ② 出发区    │
         │  ██ 减速带 ×3          ██           │
         │                                    │
         │  ③ 出发区              ④ 出发区    │
         │  ██                    ██           │
         │                                    │
       0 └───────────────────────────────────┘
         0                                   3000
    ```
    """

    def __init__(self):
        self._elements: List[FieldElement] = []
        self._build_layout()

    def _build_layout(self) -> None:
        """构建标准场地"""
        elem_id = 0

        # --- 场地边界围栏 ---
        # 四边
        self._elements.append(FieldElement(
            id=(elem_id := elem_id + 1),
            type=FieldElementType.FIELD_BOUNDARY,
            region=RectRegion(0, 0, FIELD_SIZE, FENCE_HEIGHT),
            color="围栏",
            label="场地底边",
        ))

        # --- 出发区（场地四角） ---
        start_positions = [
            (0, FIELD_SIZE - START_ZONE_SIZE, "出发区 ①"),           # 左上
            (FIELD_SIZE - START_ZONE_SIZE, FIELD_SIZE - START_ZONE_SIZE, "出发区 ②"),  # 右上
            (0, 0, "出发区 ③"),                                      # 左下
            (FIELD_SIZE - START_ZONE_SIZE, 0, "出发区 ④"),           # 右下
        ]
        for sx, sy, label in start_positions:
            self._elements.append(FieldElement(
                id=(elem_id := elem_id + 1),
                type=FieldElementType.START_ZONE,
                region=RectRegion(sx, sy, START_ZONE_SIZE, START_ZONE_SIZE),
                color=START_ZONE_COLOR,
                label=label,
            ))

        # --- 安全区 ---
        # 红色安全区（场地左侧）
        safe_zone_y = FIELD_SIZE - SAFE_ZONE_HEIGHT
        red_safe_x = 100  # 距左边界
        red_safe = RectRegion(red_safe_x, safe_zone_y,
                              SAFE_ZONE_WIDTH, SAFE_ZONE_HEIGHT)
        self._elements.append(FieldElement(
            id=(elem_id := elem_id + 1),
            type=FieldElementType.SAFE_ZONE,
            region=red_safe,
            color="红色",
            label="安全区 (红)",
            metadata={"color_enum": SafeZoneColor.RED},
        ))

        # 红色安全区-物资区（上半）
        self._elements.append(FieldElement(
            id=(elem_id := elem_id + 1),
            type=FieldElementType.SUPPLY_AREA,
            region=RectRegion(red_safe_x, safe_zone_y + SAFE_ZONE_HEIGHT / 2,
                              SAFE_ZONE_WIDTH / 2, SAFE_ZONE_HEIGHT / 2),
            color="红色",
            label="物资区 (红)",
            metadata={"safe_zone": SafeZoneColor.RED, "area_type": "supply"},
        ))

        # 红色安全区-伤员区（下半）
        self._elements.append(FieldElement(
            id=(elem_id := elem_id + 1),
            type=FieldElementType.INJURED_AREA,
            region=RectRegion(red_safe_x + SAFE_ZONE_WIDTH / 2, safe_zone_y + SAFE_ZONE_HEIGHT / 2,
                              SAFE_ZONE_WIDTH / 2, SAFE_ZONE_HEIGHT / 2),
            color="红色",
            label="伤员区 (红)",
            metadata={"safe_zone": SafeZoneColor.RED, "area_type": "injured"},
        ))

        # 蓝色安全区（场地右侧）
        blue_safe_x = FIELD_SIZE - 100 - SAFE_ZONE_WIDTH
        blue_safe = RectRegion(blue_safe_x, safe_zone_y,
                               SAFE_ZONE_WIDTH, SAFE_ZONE_HEIGHT)
        self._elements.append(FieldElement(
            id=(elem_id := elem_id + 1),
            type=FieldElementType.SAFE_ZONE,
            region=blue_safe,
            color="蓝色",
            label="安全区 (蓝)",
            metadata={"color_enum": SafeZoneColor.BLUE},
        ))

        # 蓝色安全区-物资区
        self._elements.append(FieldElement(
            id=(elem_id := elem_id + 1),
            type=FieldElementType.SUPPLY_AREA,
            region=RectRegion(blue_safe_x, safe_zone_y + SAFE_ZONE_HEIGHT / 2,
                              SAFE_ZONE_WIDTH / 2, SAFE_ZONE_HEIGHT / 2),
            color="蓝色",
            label="物资区 (蓝)",
            metadata={"safe_zone": SafeZoneColor.BLUE, "area_type": "supply"},
        ))

        # 蓝色安全区-伤员区
        self._elements.append(FieldElement(
            id=(elem_id := elem_id + 1),
            type=FieldElementType.INJURED_AREA,
            region=RectRegion(blue_safe_x + SAFE_ZONE_WIDTH / 2, safe_zone_y + SAFE_ZONE_HEIGHT / 2,
                              SAFE_ZONE_WIDTH / 2, SAFE_ZONE_HEIGHT / 2),
            color="蓝色",
            label="伤员区 (蓝)",
            metadata={"safe_zone": SafeZoneColor.BLUE, "area_type": "injured"},
        ))

        # --- 减速带（各出发区前方） ---
        bump_start_y = SAFE_ZONE_HEIGHT + 200
        for i, (sx, sy, _) in enumerate(start_positions):
            for b in range(SPEED_BUMP_COUNT):
                bump_y = bump_start_y + b * (SPEED_BUMP_SPACING + 10)  # 10mm bump width
                self._elements.append(FieldElement(
                    id=(elem_id := elem_id + 1),
                    type=FieldElementType.SPEED_BUMP,
                    region=RectRegion(sx, bump_y, START_ZONE_SIZE, 10),
                    color="减速带",
                    label=f"减速带 出发区{i+1}-{b+1}",
                ))

        logger.info(f"标准场地布局构建完成: {len(self._elements)} 个元素")

    @property
    def elements(self) -> List[FieldElement]:
        return self._elements

    def get_elements_by_type(self, element_type: FieldElementType) -> List[FieldElement]:
        return [e for e in self._elements if e.type == element_type]

    def get_start_zones(self) -> List[FieldElement]:
        return self.get_elements_by_type(FieldElementType.START_ZONE)

    def get_safe_zone(self, color: SafeZoneColor) -> Optional[FieldElement]:
        for e in self._elements:
            if e.type == FieldElementType.SAFE_ZONE:
                if e.metadata.get("color_enum") == color:
                    return e
        return None

    def is_in_safe_zone(self, x: float, y: float,
                        color: SafeZoneColor) -> bool:
        """判断坐标是否在指定安全区内"""
        safe_zone = self.get_safe_zone(color)
        if safe_zone is None:
            return False
        return safe_zone.region.contains(x, y)

    def is_in_opponent_safe_zone(self, x: float, y: float,
                                  my_color: SafeZoneColor) -> bool:
        """判断是否进入对方安全区"""
        opponent_color = (SafeZoneColor.BLUE if my_color == SafeZoneColor.RED
                          else SafeZoneColor.RED)
        return self.is_in_safe_zone(x, y, opponent_color)

    def is_on_field(self, x: float, y: float) -> bool:
        """判断坐标是否在场内"""
        margin = FENCE_HEIGHT
        return (margin <= x <= FIELD_SIZE - margin and
                margin <= y <= FIELD_SIZE - margin)


# ============================================================
# FieldLayout（供 WorldMap 使用）
# ============================================================

@dataclass
class FieldLayout:
    """场地布局（WorldMap 使用的简化版）"""
    elements: List[FieldElement] = field(default_factory=list)
    field_size_mm: float = FIELD_SIZE
    fence_height_mm: float = FENCE_HEIGHT

    @classmethod
    def standard(cls) -> "FieldLayout":
        """创建标准比赛场地"""
        std = StandardFieldLayout()
        return cls(
            elements=std.elements,
            field_size_mm=FIELD_SIZE,
            fence_height_mm=FENCE_HEIGHT,
        )

    @classmethod
    def mock(cls) -> "FieldLayout":
        """创建 Mock 场地（同标准）"""
        return cls.standard()
