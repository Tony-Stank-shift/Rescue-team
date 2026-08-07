"""
target_types.py —— 救援目标类型定义

定义所有目标相关的枚举、数据类和配置表。
初赛和决赛目标形状完全不同，通过 phase 参数切换。

对照 README.md 救援目标章节：
  初赛（20 个）：绿正方体(8) / 黑三棱锥(4) / 橘长方体(4) / 浅蓝正方体(4)
  决赛（25 个）：圆柱体(10) / 圆锥台(5) / 长方体(5) / 球体(5)
"""

import math
from dataclasses import dataclass, field
from enum import Enum, auto
from typing import Dict, List, Optional, Tuple


# ============================================================
# 枚举定义
# ============================================================

class TargetType(Enum):
    """救援目标类型"""
    REGULAR_SUPPLY = auto()   # 普通物资 — 5 分
    CORE_SUPPLY = auto()      # 核心物资 — 10 分
    INJURED = auto()          # 伤员 — 15 分
    DANGEROUS = auto()        # 危险目标 — 禁止转运


class TargetShape(Enum):
    """目标几何形状"""
    CUBE = "cube"                             # 正方体
    TRIANGULAR_PYRAMID = "triangular_pyramid" # 正三棱锥
    CUBOID = "cuboid"                         # 长方体
    CYLINDER = "cylinder"                     # 圆柱体
    CONE_FRUSTUM = "cone_frustum"             # 圆锥台
    SPHERE = "sphere"                         # 球体
    UNKNOWN = "unknown"                       # 未知


class TargetColor(Enum):
    """目标颜色"""
    RED = "red"
    YELLOW = "yellow"
    BLUE = "blue"
    ORANGE = "orange"
    GREEN = "green"
    BROWN = "brown"
    BLACK = "black"
    LIGHT_BLUE = "light_blue"
    WHITE = "white"
    UNKNOWN = "unknown"


class CompetitionPhase(Enum):
    """比赛阶段"""
    PRELIMINARY = "preliminary"   # 初赛
    FINAL = "final"               # 决赛


class TargetStatus(Enum):
    """目标状态"""
    ACTIVE = "active"             # 在场上
    BEING_TRANSPORTED = "being_transported"  # 正在被转运
    IN_SAFE_ZONE = "in_safe_zone"  # 已在安全区
    REMOVED = "removed"           # 被裁判移除


# ============================================================
# 目标信息
# ============================================================

@dataclass(frozen=True)
class TargetInfo:
    """目标完整信息（不可变）"""
    type: TargetType
    shape: TargetShape
    color: TargetColor
    size_mm: Tuple[float, ...]   # 尺寸 (长, 宽, 高) 或 (直径, 高)
    weight_g: float               # 重量（克）
    points: int                   # 分值
    material: str = "ABS"         # 材料 ABS/PLA
    description: str = ""         # 中文描述

    @property
    def is_dangerous(self) -> bool:
        return self.type == TargetType.DANGEROUS

    @property
    def max_dimension_mm(self) -> float:
        """最大尺寸（用于距离估算）"""
        return max(self.size_mm)


# ============================================================
# 检测结果
# ============================================================

@dataclass
class Detection:
    """原始检测结果（分类前）"""
    color: TargetColor
    shape: TargetShape
    bbox: Tuple[int, int, int, int]  # (x, y, w, h) 像素坐标
    confidence: float = 1.0           # 置信度
    contour_area: float = 0.0         # 轮廓面积（像素）
    contour_vertices: int = 0         # 轮廓近似顶点数

    @property
    def center_pixel(self) -> Tuple[float, float]:
        """bounding box 中心像素坐标"""
        x, y, w, h = self.bbox
        return (x + w / 2, y + h / 2)


@dataclass
class DetectedTarget:
    """已分类的检测目标"""
    id: int                         # 唯一 ID
    info: TargetInfo                # 目标信息
    position: Tuple[float, float]   # 场地坐标 (x, y) mm
    confidence: float = 1.0
    timestamp: float = 0.0
    pixel_position: Tuple[float, float] = (0, 0)  # 像素坐标


# ============================================================
# 初赛目标配置表
# ============================================================

PRELIMINARY_TARGETS: Dict[Tuple[TargetColor, TargetShape], TargetInfo] = {
    # 普通物资 — 绿色正方体 40mm — 5 分
    (TargetColor.GREEN, TargetShape.CUBE): TargetInfo(
        type=TargetType.REGULAR_SUPPLY,
        shape=TargetShape.CUBE,
        color=TargetColor.GREEN,
        size_mm=(40, 40, 40),
        weight_g=50,
        points=5,
        material="ABS",
        description="普通物资（初赛）— 绿色正方体",
    ),
    # 核心物资 — 黑色正三棱锥 40mm — 10 分
    (TargetColor.BLACK, TargetShape.TRIANGULAR_PYRAMID): TargetInfo(
        type=TargetType.CORE_SUPPLY,
        shape=TargetShape.TRIANGULAR_PYRAMID,
        color=TargetColor.BLACK,
        size_mm=(40, 40, 40),  # 底边 × 底边 × 高
        weight_g=50,
        points=10,
        material="ABS",
        description="核心物资（初赛）— 黑色正三棱锥",
    ),
    # 伤员 — 橘色长方体 80×40×40mm — 15 分
    (TargetColor.ORANGE, TargetShape.CUBOID): TargetInfo(
        type=TargetType.INJURED,
        shape=TargetShape.CUBOID,
        color=TargetColor.ORANGE,
        size_mm=(80, 40, 40),
        weight_g=80,
        points=15,
        material="PLA",
        description="伤员（初赛）— 橘色长方体",
    ),
    # 危险目标 — 浅蓝正方体 40mm — 禁止转运
    (TargetColor.LIGHT_BLUE, TargetShape.CUBE): TargetInfo(
        type=TargetType.DANGEROUS,
        shape=TargetShape.CUBE,
        color=TargetColor.LIGHT_BLUE,
        size_mm=(40, 40, 40),
        weight_g=50,
        points=0,
        material="ABS",
        description="危险目标（初赛）— 浅蓝正方体",
    ),
}


# ============================================================
# 决赛目标配置表
# ============================================================

FINAL_TARGETS: Dict[Tuple[TargetColor, TargetShape], TargetInfo] = {
    # 普通物资 — 圆柱体 φ40×60mm — 5 分
    (TargetColor.GREEN, TargetShape.CYLINDER): TargetInfo(
        type=TargetType.REGULAR_SUPPLY,
        shape=TargetShape.CYLINDER,
        color=TargetColor.GREEN,
        size_mm=(40, 60),  # 直径 × 高
        weight_g=60,
        points=5,
        material="ABS",
        description="普通物资（决赛）— 绿色圆柱体",
    ),
    # 核心物资 — 圆锥台 φ40×φ30×60mm — 10 分
    (TargetColor.BLACK, TargetShape.CONE_FRUSTUM): TargetInfo(
        type=TargetType.CORE_SUPPLY,
        shape=TargetShape.CONE_FRUSTUM,
        color=TargetColor.BLACK,
        size_mm=(40, 30, 60),  # 下底直径 × 上底直径 × 高
        weight_g=60,
        points=10,
        material="ABS",
        description="核心物资（决赛）— 黑色圆锥台",
    ),
    # 伤员 — 长方体 80×40×40mm — 15 分（决赛颜色现场公布）
    (TargetColor.ORANGE, TargetShape.CUBOID): TargetInfo(
        type=TargetType.INJURED,
        shape=TargetShape.CUBOID,
        color=TargetColor.ORANGE,
        size_mm=(80, 40, 40),
        weight_g=80,
        points=15,
        material="PLA",
        description="伤员（决赛）— 长方体（颜色以现场为准）",
    ),
    # 危险目标 — 球体 φ40mm — 禁止转运
    (TargetColor.LIGHT_BLUE, TargetShape.SPHERE): TargetInfo(
        type=TargetType.DANGEROUS,
        shape=TargetShape.SPHERE,
        color=TargetColor.LIGHT_BLUE,
        size_mm=(40,),  # 直径
        weight_g=50,
        points=0,
        material="ABS",
        description="危险目标（决赛）— 浅蓝球体",
    ),
}


# ============================================================
# 工具函数
# ============================================================

def get_target_config(phase: CompetitionPhase) -> Dict[Tuple[TargetColor, TargetShape], TargetInfo]:
    """获取当前比赛阶段的目标配置"""
    if phase == CompetitionPhase.PRELIMINARY:
        return PRELIMINARY_TARGETS
    return FINAL_TARGETS


def get_point_value(target_type: TargetType) -> int:
    """获取目标类型对应的分值"""
    points_map = {
        TargetType.REGULAR_SUPPLY: 5,
        TargetType.CORE_SUPPLY: 10,
        TargetType.INJURED: 15,
        TargetType.DANGEROUS: 0,
    }
    return points_map[target_type]


def get_target_display_name(info: TargetInfo) -> str:
    """目标的中文显示名"""
    type_names = {
        TargetType.REGULAR_SUPPLY: "普通物资",
        TargetType.CORE_SUPPLY: "核心物资",
        TargetType.INJURED: "伤员",
        TargetType.DANGEROUS: "⚠️危险目标",
    }
    return f"{type_names[info.type]} ({info.points}分)"
