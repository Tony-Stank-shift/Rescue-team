"""
target_types.py —— 救援目标类型定义

定义所有目标相关的枚举、数据类和配置表。
初赛和决赛目标形状完全不同，通过 phase 参数切换。

对照 README.md 救援目标章节：
  初赛（20 个）：绿正方体(8) / 黑三棱锥(4) / 橘长方体(4) / 浅蓝正方体(4)
  决赛（25 个）：圆柱体(10) / 圆锥台(5) / 长方体(5) / 球体(5)
"""

import math
import logging
from dataclasses import dataclass, field
from enum import Enum, auto
from typing import Dict, List, Optional, Tuple

logger = logging.getLogger(__name__)


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
    orientation: float = 0.0          # 朝向 (rad)

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
    pixel_position: Tuple[float, float] = (0, 0)  # 像素坐标（检测框中心）
    #: 真实检测框 (x, y, w, h)；地平面测距需要底边 y+h，故必须保留
    pixel_bbox: Tuple[int, int, int, int] = (0, 0, 0, 0)
    orientation: float = 0.0               # 目标朝向 (rad)，0=正对机器人

    @property
    def pixel_bottom(self) -> float:
        """检测框底边像素 y（目标与地面的接触处，地平面测距用）。"""
        _x, y, _w, h = self.pixel_bbox
        return float(y + h)

    @property
    def has_pixel_bbox(self) -> bool:
        """是否有有效的检测框（Mock/异常路径可能没有）。"""
        _x, _y, w, h = self.pixel_bbox
        return w > 0 and h > 0


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
    """获取当前比赛阶段的目标配置（已应用现场"类型→颜色"覆盖）。

    ⚠️ 必须走 `_apply_color_override`：决赛现场会改颜色，而 (颜色,形状)→类型 是本表
    唯一的判定依据。旧实现直接把写死的表返回 → **YAML 里改了 `target_color_map`
    也不生效、且完全不报错**（现场以为改好了，实际仍按旧颜色判 → 目标判不出来
    → 整场 0 分）。这就是 U5/B13 的核心。
    """
    base = (PRELIMINARY_TARGETS if phase == CompetitionPhase.PRELIMINARY
            else FINAL_TARGETS)
    return _apply_color_override(base)


# ============================================================
# 现场"类型 → 颜色"覆盖（决赛颜色现场可改，无需改代码）
# ============================================================
#: {TargetType: TargetColor}；空 = 使用代码内置颜色
_COLOR_OVERRIDE: Dict[TargetType, TargetColor] = {}
#: 覆盖结果缓存（按 base 表 id 缓存；set_color_override 时清空）
_OVERRIDE_CACHE: Dict[int, Dict[Tuple[TargetColor, TargetShape], TargetInfo]] = {}

#: 现场配置里允许出现的类型名 → TargetType
_COLOR_NAME_TO_TYPE = {"regular": "REGULAR_SUPPLY", "core": "CORE_SUPPLY",
                       "injured": "INJURED", "dangerous": "DANGEROUS"}


def set_color_override(mapping: Optional[Dict[str, str]]) -> List[str]:
    """按现场配置（`perception.target_color_map`）设置"类型 → 颜色"覆盖。

    Args:
        mapping: 形如 ``{"regular": "green", "core": "black",
                         "injured": "orange", "dangerous": "light_blue"}``
    Returns:
        **现场配置问题**列表，调用方必须据此**拒绝启动**（忽略它就等于
        "以为改好了其实没改"）。两类问题都会返回：

        ① 无法识别的条目，如 ``["core=purple(不是合法颜色名…)"]``；
        ② **合法但冲突**的覆盖（T2-18）：两类目标被映射到同一个
           `(颜色, 形状)` 判定键，如 ``{"regular": "light_blue"}`` 会让
           "普通物资"和"危险目标"都变成 `LIGHT_BLUE/CUBE` —— 冲突时
           `_apply_color_override` 只能留下其中一个，**另一类目标整场检不到**，
           而旧实现只在 logger.error 里说一句就继续跑（启动不报错）。
    """
    global _COLOR_OVERRIDE, _OVERRIDE_CACHE
    problems: List[str] = []
    new: Dict[TargetType, TargetColor] = {}
    if mapping:
        for key, value in mapping.items():
            attr = _COLOR_NAME_TO_TYPE.get(str(key).strip().lower())
            if attr is None:
                problems.append(f"{key}(未知类型名，应为 "
                                f"{'/'.join(sorted(_COLOR_NAME_TO_TYPE))})")
                continue
            try:
                new[TargetType[attr]] = TargetColor(str(value).strip().lower())
            except (KeyError, ValueError):
                problems.append(f"{key}={value}(不是合法颜色名，合法值："
                                f"{'/'.join(c.name.lower() for c in TargetColor)})")
    _COLOR_OVERRIDE = new
    _OVERRIDE_CACHE = {}          # 覆盖变了 → 缓存失效
    logger.info("目标颜色覆盖已应用: %s",
                {t.name: c.name for t, c in new.items()} or "无（使用内置颜色）")

    # ── T2-18：把"合法但冲突"的覆盖也**并入返回值**（调用方据此拒绝启动）──
    #    冲突与 base 表有关（初赛/决赛表不同），所以两张表都要算。
    #    这里顺便把结果写进缓存，行为与 `_apply_color_override` 的懒计算一致。
    if new:
        for base, phase_label in ((PRELIMINARY_TARGETS, "初赛目标表"),
                                  (FINAL_TARGETS, "决赛目标表")):
            out, collisions = _build_override_table(base)
            _OVERRIDE_CACHE[id(base)] = out
            for color_name, shape_name, type_a, type_b in collisions:
                problems.append(
                    f"{phase_label}颜色冲突: {color_name}/{shape_name} 同时对应 "
                    f"{type_a} 与 {type_b} → 其中一类目标**整场都判不出来**"
                    f"（请让每类目标的 (颜色,形状) 组合唯一）")
    return problems


def get_color_override() -> Dict[TargetType, TargetColor]:
    """当前生效的颜色覆盖（只读，供自检/日志核对）。"""
    return dict(_COLOR_OVERRIDE)


def _build_override_table(base) -> Tuple[dict, List[tuple]]:
    """按当前 `_COLOR_OVERRIDE` 重映射 base 表，并返回 (新表, 冲突列表)。

    冲突 = 两类目标落到同一个 `(颜色, 形状)` 键（其中一类永远判不出来）。
    冲突同时：① logger.error 喊出来；② 由 `set_color_override` 并入返回值。
    """
    out: Dict[Tuple[TargetColor, TargetShape], TargetInfo] = {}
    collisions: List[tuple] = []
    for (color, shape), info in base.items():
        new_color = _COLOR_OVERRIDE.get(info.type, color)
        key = (new_color, shape)
        if key in out and out[key].type != info.type:
            collisions.append((new_color.name, shape.name,
                               out[key].type.name, info.type.name))
        out[key] = info
    if collisions:
        # 覆盖后两类目标撞到同一个 (颜色,形状) 键 → 其中一类永远判不出来。
        # 这是**现场配置错误**，必须显式喊出来，不能静默取其一。
        logger.error("颜色覆盖导致判定键冲突（会有一类目标无法识别）：%s",
                     "；".join(f"{c}/{s} 同时对应 {a} 与 {b}"
                               for c, s, a, b in collisions))
    return out, collisions


def _apply_color_override(base):
    """把 base 表按"类型→颜色"重映射。

    **无覆盖时原样返回 base** —— 恒等变换，保证现有行为零变化。
    """
    if not _COLOR_OVERRIDE:
        return base
    cached = _OVERRIDE_CACHE.get(id(base))
    if cached is not None:
        return cached

    out, _collisions = _build_override_table(base)
    _OVERRIDE_CACHE[id(base)] = out
    return out


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
