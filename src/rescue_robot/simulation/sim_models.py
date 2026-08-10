"""
sim_models.py —— 程序化 3D 模型生成工具

为 MuJoCo 仿真提供 3D 几何体工厂函数和颜色映射。
所有尺寸以 mm 为输入，内部转换为 m（MuJoCo 单位）。

用法:
  from rescue_robot.simulation.sim_models import create_target_geom, COLORS
"""

import math
import numpy as np
from typing import Dict, List, Optional, Tuple

# ============================================================
# 颜色映射 (RGBA, 0-1)
# ============================================================

COLORS: Dict[str, Tuple[float, float, float, float]] = {
    # 场地颜色
    "field_green":       (0.15, 0.65, 0.15, 1.0),
    "wall_gray":         (0.50, 0.50, 0.50, 1.0),
    "start_zone_magenta":(1.00, 0.00, 1.00, 0.5),
    "safe_zone_red":     (1.00, 0.20, 0.20, 0.4),
    "safe_zone_blue":    (0.20, 0.20, 1.00, 0.4),
    "speed_bump_yellow": (0.95, 0.90, 0.05, 1.0),
    "purple_fence":      (0.60, 0.20, 0.80, 1.0),

    # 目标颜色
    "target_green":      (0.10, 0.90, 0.10, 1.0),
    "target_black":      (0.15, 0.15, 0.15, 1.0),
    "target_orange":     (1.00, 0.50, 0.05, 1.0),
    "target_light_blue": (0.30, 0.75, 1.00, 1.0),

    # 机器人颜色
    "robot_chassis":     (0.20, 0.20, 0.20, 1.0),
    "robot_pusher":      (0.70, 0.70, 0.75, 1.0),
    "robot_wheel":       (0.10, 0.10, 0.10, 1.0),

    # 对手颜色
    "opponent_chassis":  (0.90, 0.15, 0.15, 1.0),
}


def rgba(color_name: str, alpha: float = None) -> Tuple[float, float, float, float]:
    """获取颜色 RGBA，可选覆盖透明度。"""
    c = COLORS.get(color_name, COLORS["field_green"])
    if alpha is not None:
        return (c[0], c[1], c[2], alpha)
    return c


# ============================================================
# 单位转换
# ============================================================

def mm_to_m(*vals: float) -> Tuple[float, ...]:
    """毫米 → 米（MuJoCo 单位）。"""
    if len(vals) == 1:
        return (vals[0] / 1000.0,)
    return tuple(v / 1000.0 for v in vals)


def m_to_mm(*vals: float) -> Tuple[float, ...]:
    """米 → 毫米。"""
    if len(vals) == 1:
        return (vals[0] * 1000.0,)
    return tuple(v * 1000.0 for v in vals)


# ============================================================
# 几何体工厂
# ============================================================

def create_box_geom(half_x: float, half_y: float, half_z: float,
                     rgba_color: tuple) -> dict:
    """创建长方体几何参数。"""
    return {
        "type": "box",
        "size": [half_x, half_y, half_z],
        "rgba": list(rgba_color),
    }


def create_cylinder_geom(radius: float, half_length: float,
                          rgba_color: tuple) -> dict:
    """创建圆柱几何参数。"""
    return {
        "type": "cylinder",
        "size": [radius, half_length, 0.0],
        "rgba": list(rgba_color),
    }


def create_sphere_geom(radius: float, rgba_color: tuple) -> dict:
    """创建球体几何参数。"""
    return {
        "type": "sphere",
        "size": [radius, 0.0, 0.0],
        "rgba": list(rgba_color),
    }


# ============================================================
# 三棱锥 / 圆锥台 顶点生成
# ============================================================

def pyramid_vertices(base_w: float, base_d: float, height: float) -> np.ndarray:
    """
    生成三棱锥（四面体）的凸包顶点。
    底面为三角形 (等边)，顶点在中心上方。

    Returns: (N, 3) float array
    """
    r = base_w / math.sqrt(3)  # 外接圆半径
    # 底面 3 个顶点 (等边三角形)
    base = np.array([
        [r * math.cos(0 * 2 * math.pi / 3), r * math.sin(0 * 2 * math.pi / 3), 0],
        [r * math.cos(1 * 2 * math.pi / 3), r * math.sin(1 * 2 * math.pi / 3), 0],
        [r * math.cos(2 * 2 * math.pi / 3), r * math.sin(2 * 2 * math.pi / 3), 0],
    ])
    apex = np.array([[0, 0, height]])
    return np.vstack([base, apex])


def cone_frustum_vertices(bottom_r: float, top_r: float,
                           height: float, n_segments: int = 16) -> np.ndarray:
    """
    生成圆锥台（截锥体）的顶点。

    Returns: (2 * n_segments, 3) float array
    """
    vertices = []
    for i in range(n_segments):
        theta = 2 * math.pi * i / n_segments
        vertices.append([bottom_r * math.cos(theta),
                         bottom_r * math.sin(theta), 0])
    for i in range(n_segments):
        theta = 2 * math.pi * i / n_segments
        vertices.append([top_r * math.cos(theta),
                         top_r * math.sin(theta), height])
    return np.array(vertices)


# ============================================================
# 目标形状规格
# ============================================================

# 各目标形状的物理尺寸 (mm) 和 MuJoCo 几何类型
TARGET_SHAPE_SPECS = {
    "cube": {
        "mujoco_type": "box",
        # 正方体 40×40×40mm → half-extents 20mm
        "half_size_mm": (20, 20, 20),
    },
    "pyramid": {
        "mujoco_type": "pyramid",
        # 三棱锥 底 40×40mm 高 40mm
        "base_mm": 40,
        "height_mm": 40,
    },
    "cuboid": {
        "mujoco_type": "box",
        # 长方体 80×40×40mm
        "half_size_mm": (40, 20, 20),
    },
    "cylinder": {
        "mujoco_type": "cylinder",
        # 圆柱 φ40mm × 高 60mm
        "radius_mm": 20,
        "half_height_mm": 30,
    },
    "cone_frustum": {
        "mujoco_type": "cone_frustum",
        # 圆锥台 底φ40mm → 顶φ30mm, 高 60mm
        "bottom_r_mm": 20,
        "top_r_mm": 15,
        "height_mm": 60,
    },
    "sphere": {
        "mujoco_type": "sphere",
        # 球体 φ40mm
        "radius_mm": 20,
    },
}


def get_target_spec(shape_name: str) -> dict:
    """获取指定形状的规格。"""
    if shape_name not in TARGET_SHAPE_SPECS:
        raise ValueError(f"未知形状: {shape_name}. 有效值: {list(TARGET_SHAPE_SPECS.keys())}")
    return TARGET_SHAPE_SPECS[shape_name]


# ============================================================
# 场地常量 (mm)
# ============================================================

FIELD_SIZE_MM = (3000, 3000)         # 比赛场地 3000×3000mm
WALL_HEIGHT_MM = 100                 # 四周围栏高 100mm
WALL_THICKNESS_MM = 20               # 围栏厚 20mm
SAFE_ZONE_SIZE_MM = (600, 400)       # 安全区尺寸
SPEED_BUMP_WIDTH_MM = 30             # 减速带宽 30mm
SPEED_BUMP_HEIGHT_MM = 5             # 减速带高 5mm
SPEED_BUMP_COUNT = 3                 # 每个出发区 3 根减速带
SPEED_BUMP_SPACING_MM = 80           # 减速带间距 50mm + 自身 30mm = 80mm 间隔
START_ZONE_SIZE_MM = (300, 300)      # 出发区尺寸
ROBOT_SIZE_MM = (300, 300, 200)      # 机器人底盘 (长×宽×高)
ROBOT_MASS_KG = 1.5                  # 机器人质量
TARGET_MASS_KG = 0.05                # 目标质量 (普通)


if __name__ == "__main__":
    # 独立测试
    print("颜色映射:")
    for name, c in COLORS.items():
        print(f"  {name}: {c}")
    print(f"\n单位转换: 3000mm = {mm_to_m(3000)[0]:.1f}m")
    print(f"目标形状规格: {list(TARGET_SHAPE_SPECS.keys())}")
    print(f"金字塔顶点: {pyramid_vertices(40, 40, 40).shape}")
    print(f"圆锥台顶点: {cone_frustum_vertices(20, 15, 60).shape}")
    print("sim_models.py ✅")
