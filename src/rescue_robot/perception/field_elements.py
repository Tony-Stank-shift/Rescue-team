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
SAFE_ZONE_WIDTH = 600       # 安全区内部宽（PDF 图7）
SAFE_ZONE_HEIGHT = 300      # 安全区内部高（分区各 300×300）
# 安全区在场地正中间（x 1200~1800，中心 1500），紫边 30 包围
SAFE_ZONE_CENTER_X = 1500   # 安全区中心 x
FENCE_THICKNESS_SAFE = 30   # 安全区紫色围栏厚度
DIVIDER_SIZE = (300, 20, 20)  # 长×宽×高

# 减速带（长 300 × 宽 60 × 高 10）
SPEED_BUMP_LENGTH = 300     # 长（沿出发区边）
SPEED_BUMP_DEPTH = 60       # 宽（伸出方向）
SPEED_BUMP_SPACING = 50     # 间隔（出发区→第一条 50，每条间 50）
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

        # --- 出发区（场地四角，洋红 300×300） ---
        start_positions = [
            (0, FIELD_SIZE - START_ZONE_SIZE, "出发区 1"),           # 左上
            (FIELD_SIZE - START_ZONE_SIZE, FIELD_SIZE - START_ZONE_SIZE, "出发区 2"),  # 右上
            (0, 0, "出发区 3"),                                      # 左下
            (FIELD_SIZE - START_ZONE_SIZE, 0, "出发区 4"),           # 右下
        ]
        for sx, sy, label in start_positions:
            self._elements.append(FieldElement(
                id=(elem_id := elem_id + 1),
                type=FieldElementType.START_ZONE,
                region=RectRegion(sx, sy, START_ZONE_SIZE, START_ZONE_SIZE),
                color=START_ZONE_COLOR,
                label=label,
            ))

        # --- 安全区（红顶/蓝底，正中间 x 1200~1800，内部 600×300，分区 300×300） ---
        safe_x = SAFE_ZONE_CENTER_X - SAFE_ZONE_WIDTH / 2  # 1200
        half_w = SAFE_ZONE_WIDTH / 2                        # 300

        # 红色安全区（顶部，出发区1/2 之间）：物资区左 x[1200,1500]、伤员区右 x[1500,1800]
        red_safe_y = FIELD_SIZE - SAFE_ZONE_HEIGHT - FENCE_THICKNESS_SAFE  # 2670（顶部留 30 紫边）
        # 红色紫色围栏（紧贴，30mm，含上方）
        self._elements.append(FieldElement(
            id=(elem_id := elem_id + 1),
            type=FieldElementType.PURPLE_FENCE,
            region=RectRegion(safe_x - FENCE_THICKNESS_SAFE, red_safe_y - FENCE_THICKNESS_SAFE,
                              SAFE_ZONE_WIDTH + 2 * FENCE_THICKNESS_SAFE,
                              SAFE_ZONE_HEIGHT + 2 * FENCE_THICKNESS_SAFE),
            color="紫色", label="紫色围栏 (红)", metadata={"safe_zone": SafeZoneColor.RED},
        ))
        self._elements.append(FieldElement(
            id=(elem_id := elem_id + 1), type=FieldElementType.SAFE_ZONE,
            region=RectRegion(safe_x, red_safe_y, SAFE_ZONE_WIDTH, SAFE_ZONE_HEIGHT),
            color="红色", label="安全区 (红)", metadata={"color_enum": SafeZoneColor.RED},
        ))
        self._elements.append(FieldElement(
            id=(elem_id := elem_id + 1), type=FieldElementType.SUPPLY_AREA,
            region=RectRegion(safe_x, red_safe_y, half_w, SAFE_ZONE_HEIGHT),  # 物资区 左半
            color="红色", label="物资区 (红)", metadata={"safe_zone": SafeZoneColor.RED, "area_type": "supply"},
        ))
        self._elements.append(FieldElement(
            id=(elem_id := elem_id + 1), type=FieldElementType.INJURED_AREA,
            region=RectRegion(safe_x + half_w, red_safe_y, half_w, SAFE_ZONE_HEIGHT),  # 伤员区 右半
            color="红色", label="伤员区 (红)", metadata={"safe_zone": SafeZoneColor.RED, "area_type": "injured"},
        ))

        # 蓝色安全区（底部，出发区3/4 之间）：伤员区左 x[1200,1500]、物资区右 x[1500,1800]
        blue_safe_y = FENCE_THICKNESS_SAFE                  # 30（底部留 30 紫边）
        self._elements.append(FieldElement(
            id=(elem_id := elem_id + 1),
            type=FieldElementType.PURPLE_FENCE,
            region=RectRegion(safe_x - FENCE_THICKNESS_SAFE, blue_safe_y - FENCE_THICKNESS_SAFE,
                              SAFE_ZONE_WIDTH + 2 * FENCE_THICKNESS_SAFE,
                              SAFE_ZONE_HEIGHT + 2 * FENCE_THICKNESS_SAFE),
            color="紫色", label="紫色围栏 (蓝)", metadata={"safe_zone": SafeZoneColor.BLUE},
        ))
        self._elements.append(FieldElement(
            id=(elem_id := elem_id + 1), type=FieldElementType.SAFE_ZONE,
            region=RectRegion(safe_x, blue_safe_y, SAFE_ZONE_WIDTH, SAFE_ZONE_HEIGHT),
            color="蓝色", label="安全区 (蓝)", metadata={"color_enum": SafeZoneColor.BLUE},
        ))
        self._elements.append(FieldElement(
            id=(elem_id := elem_id + 1), type=FieldElementType.INJURED_AREA,
            region=RectRegion(safe_x, blue_safe_y, half_w, SAFE_ZONE_HEIGHT),  # 伤员区 左半
            color="蓝色", label="伤员区 (蓝)", metadata={"safe_zone": SafeZoneColor.BLUE, "area_type": "injured"},
        ))
        self._elements.append(FieldElement(
            id=(elem_id := elem_id + 1), type=FieldElementType.SUPPLY_AREA,
            region=RectRegion(safe_x + half_w, blue_safe_y, half_w, SAFE_ZONE_HEIGHT),  # 物资区 右半
            color="蓝色", label="物资区 (蓝)", metadata={"safe_zone": SafeZoneColor.BLUE, "area_type": "supply"},
        ))

        # --- 减速带（长300×宽60，出发区两条内边各 3 根，边→第一条 50，每条间 50） ---
        for i, (sx, sy, _) in enumerate(start_positions):
            # 水平内边：顶部出发区(1,2)内边=下边 sy；底部出发区(3,4)内边=上边 sy+300
            horiz_edge = sy if sy > 0 else sy + START_ZONE_SIZE
            # 垂直内边：左侧出发区(1,3)内边=右边 sx+300；右侧出发区(2,4)内边=左边 sx
            vert_edge = sx + START_ZONE_SIZE if sx == 0 else sx
            for b in range(SPEED_BUMP_COUNT):
                off = SPEED_BUMP_SPACING + b * (SPEED_BUMP_DEPTH + SPEED_BUMP_SPACING)  # 50 + b*110
                # 横向减速带（沿 x，长300）：顶部出发区朝下（y=sy-off-DEPTH），底部朝上（y=sy+300+off）
                if sy > 0:  # 顶部出发区(1,2)
                    y0 = horiz_edge - off - SPEED_BUMP_DEPTH
                    rx = sx
                else:       # 底部出发区(3,4)
                    y0 = horiz_edge + off
                    rx = sx
                self._elements.append(FieldElement(
                    id=(elem_id := elem_id + 1), type=FieldElementType.SPEED_BUMP,
                    region=RectRegion(rx, y0, SPEED_BUMP_LENGTH, SPEED_BUMP_DEPTH),
                    color="减速带", label=f"减速带 出发区{i+1}-横{b+1}",
                ))
                # 纵向减速带（沿 y，长300）：左侧出发区(1,3)朝右（x=sx+300+off），右侧(2,4)朝左（x=sx-off-DEPTH）
                if sx == 0:  # 左侧出发区(1,3)
                    x0 = vert_edge + off
                else:        # 右侧出发区(2,4)
                    x0 = vert_edge - off - SPEED_BUMP_DEPTH
                self._elements.append(FieldElement(
                    id=(elem_id := elem_id + 1), type=FieldElementType.SPEED_BUMP,
                    region=RectRegion(x0, sy, SPEED_BUMP_DEPTH, SPEED_BUMP_LENGTH),
                    color="减速带", label=f"减速带 出发区{i+1}-纵{b+1}",
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
