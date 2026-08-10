"""
sim_target.py —— 救援目标实体

为 MuJoCo 仿真创建可推动的 3D 目标。
支持初赛和决赛全部 8 种目标形态。

用法:
  spec = mujoco.MjSpec()
  target = SimTarget(spec.worldbody, info, pos_m=(1.5, 1.5))
  body_id = target.body  # MjsBody 对象
"""

import math
import numpy as np
from typing import Dict, List, Optional, Tuple

import mujoco

from .sim_models import (
    TARGET_MASS_KG,
    TARGET_SHAPE_SPECS,
    mm_to_m,
    rgba,
)

_BOX = mujoco.mjtGeom.mjGEOM_BOX
_CYLINDER = mujoco.mjtGeom.mjGEOM_CYLINDER
_SPHERE = mujoco.mjtGeom.mjGEOM_SPHERE
_FREE = mujoco.mjtJoint.mjJNT_FREE

_unnamed_counter = 0  # 匿名目标计数器


def _color_for_target(color_name: str) -> Tuple[float, float, float, float]:
    """将 TargetColor 名称映射到 RGBA。"""
    mapping = {
        "green":      rgba("target_green"),
        "black":      rgba("target_black"),
        "orange":     rgba("target_orange"),
        "light_blue": rgba("target_light_blue"),
    }
    return mapping.get(color_name.lower(), rgba("target_green"))


class SimTarget:
    """
    救援目标 3D 实体。

    在 MuJoCo 场景中创建一个可被推动的物理实体。
    目标通过 free joint 连接到世界，允许 6DoF 运动。
    质量非常轻 (50-80g)，可被机器人推动。
    """

    def __init__(self, wb, target_info, position_m: Tuple[float, float],
                 target_id: Optional[int] = None):
        """
        Args:
            wb: spec.worldbody (MjsBody)
            target_info: TargetInfo from perception.target_types
            position_m: (x, y) 目标位置 (m)
            target_id: 可选目标 ID
        """
        self.target_info = target_info
        self._pos_m = position_m
        self._id = target_id
        self._delivered = False
        self._is_dangerous = getattr(target_info, 'type', None)
        self._body = None

        self._create(wb)

    def _create(self, wb) -> None:
        """在 MuJoCo 场景中创建目标实体。"""
        info = self.target_info
        shape_name = info.shape.value if hasattr(info.shape, 'value') else str(info.shape)
        color_name = info.color.value if hasattr(info.color, 'value') else str(info.color)
        color_rgba = _color_for_target(color_name)
        points = info.points

        spec = TARGET_SHAPE_SPECS.get(shape_name)
        if spec is None:
            raise ValueError(f"未知目标形状: {shape_name}")

        gt = spec.get("mujoco_type", "box")
        pos_x, pos_y = self._pos_m

        # 创建 body（确保名字唯一）
        global _unnamed_counter
        if self._id is not None:
            body_name = f"target_{self._id}"
        else:
            _unnamed_counter += 1
            body_name = f"target_{_unnamed_counter}"
        self._body = wb.add_body(name=body_name, pos=[pos_x, pos_y, 0.03])

        # Free joint (可被推动)
        self._body.add_joint(type=_FREE)

        # Geometry based on shape type
        if gt == "box":
            hs = mm_to_m(*spec["half_size_mm"])
            self._body.add_geom(
                type=_BOX, size=list(hs),
                rgba=color_rgba, mass=TARGET_MASS_KG,
            )
            self._half_z = hs[2]

        elif gt == "cylinder":
            r = mm_to_m(spec["radius_mm"])[0]
            hh = mm_to_m(spec["half_height_mm"])[0]
            self._body.add_geom(
                type=_CYLINDER, size=[r, hh, 0.0],
                rgba=color_rgba, mass=TARGET_MASS_KG * 1.2,
            )
            self._half_z = hh

        elif gt == "sphere":
            r = mm_to_m(spec["radius_mm"])[0]
            self._body.add_geom(
                type=_SPHERE, size=[r, 0.0, 0.0],
                rgba=color_rgba, mass=TARGET_MASS_KG,
            )
            self._half_z = r

        elif gt == "pyramid":
            # 三棱锥 — 用 BOX 近似 (赛事中视觉识别精度足够)
            base = mm_to_m(spec["base_mm"])[0] / 2
            h = mm_to_m(spec["height_mm"])[0] / 2
            self._body.add_geom(
                type=_BOX, size=[base, base, h],
                rgba=color_rgba, mass=TARGET_MASS_KG,
            )
            self._half_z = h

        elif gt == "cone_frustum":
            # 圆锥台 — 用 CYLINDER 近似 (视觉区别在渲染层)
            r = mm_to_m(spec["bottom_r_mm"])[0]
            hh = mm_to_m(spec["height_mm"])[0] / 2
            self._body.add_geom(
                type=_CYLINDER, size=[r, hh, 0.0],
                rgba=color_rgba, mass=TARGET_MASS_KG * 1.2,
            )
            self._half_z = hh

        else:
            # Fallback: box
            self._body.add_geom(
                type=_BOX, size=[0.02, 0.02, 0.02],
                rgba=color_rgba, mass=TARGET_MASS_KG,
            )
            self._half_z = 0.02

        # 轻摩擦
        self._body.geoms[0].friction = [0.4, 0.1, 0.4]

    # ---- 属性 ----

    @property
    def body(self):
        """MuJoCo body 对象。"""
        return self._body

    @property
    def is_delivered(self) -> bool:
        return self._delivered

    @property
    def pos_m(self) -> Tuple[float, float]:
        return self._pos_m

    @property
    def target_id(self):
        return self._id

    @property
    def half_z(self) -> float:
        return self._half_z

    @property
    def points(self) -> int:
        return self.target_info.points

    @property
    def is_dangerous(self) -> bool:
        t = self.target_info.type
        name = t.name if hasattr(t, 'name') else str(t)
        return name.upper() == "DANGEROUS"

    def mark_delivered(self) -> None:
        """标记为已送达。"""
        self._delivered = True

    def reset_delivered(self) -> None:
        """重置送达状态。"""
        self._delivered = False

    def get_position(self, data) -> Tuple[float, float, float]:
        """从 MuJoCo data 中获取当前 3D 位置 (m)。"""
        if self._body is None:
            return (0.0, 0.0, 0.0)
        name = self._body.name
        if not name:
            return (0.0, 0.0, 0.0)
        try:
            b = data.body(name)
            return (b.xpos[0], b.xpos[1], b.xpos[2])
        except KeyError:
            return (0.0, 0.0, 0.0)


# ============================================================
# 目标工厂
# ============================================================

class TargetFactory:
    """批量创建救援目标。"""

    def __init__(self, wb):
        self.wb = wb
        self._counter = 0

    def from_target_info(self, info, position_m: Tuple[float, float]) -> SimTarget:
        """从 TargetInfo 创建单个目标。"""
        self._counter += 1
        return SimTarget(self.wb, info, position_m, target_id=self._counter)

    def from_config_dict(self, config: dict, positions_m: List[Tuple[float, float]]) -> List[SimTarget]:
        """
        从目标配置字典和位置列表批量创建。

        Args:
            config: {(color, shape): TargetInfo} 字典 (如 PRELIMINARY_TARGETS)
            positions_m: 位置列表

        Returns:
            创建的 SimTarget 列表
        """
        targets = []
        keys = list(config.values())
        for i, pos in enumerate(positions_m):
            info = keys[i % len(keys)]
            targets.append(self.from_target_info(info, pos))
        return targets


# ============================================================
# 独立测试
# ============================================================

if __name__ == "__main__":
    from dataclasses import dataclass

    @dataclass
    class MockTargetInfo:
        shape: str
        color: str
        type: str
        points: int
        description: str

    from itertools import cycle
    shapes = ["cube", "pyramid", "cuboid", "cylinder", "cone_frustum", "sphere"]
    colors = ["green", "black", "orange", "light_blue", "green", "black"]

    print("=" * 50)
    print("  SimTarget 独立测试")
    print("=" * 50)

    spec = mujoco.MjSpec()
    spec.option.timestep = 0.002
    spec.option.gravity = [0, 0, -9.81]

    # Ground
    spec.worldbody.add_geom(
        type=mujoco.mjtGeom.mjGEOM_PLANE,
        size=[0, 0, 0.001],
        pos=[1.5, 1.5, 0],
    )

    # Test 1: Create all shapes
    print("\n--- 测试 1: 所有形状创建 ---")
    targets = []
    for i, (shape, color) in enumerate(zip(shapes, colors)):
        info = MockTargetInfo(shape=shape, color=color,
            type="REGULAR_SUPPLY", points=5,
            description=f"{color} {shape}")
        t = SimTarget(spec.worldbody, info,
            position_m=(0.5 + i * 0.4, 0.5), target_id=i + 1)
        targets.append(t)
        print(f"  {info.description}: created")
    print("  ✅ 通过")

    # Test 2: Compile and step
    print("\n--- 测试 2: 编译 + 物理步进 ---")
    model = spec.compile()
    assert model.ngeom >= 7, f"至少 7 geoms: {model.ngeom}"
    print(f"  ngeom: {model.ngeom}, nbody: {model.nbody}")

    data = mujoco.MjData(model)
    for _ in range(100):
        mujoco.mj_step(model, data)
    print("  100 步 OK")
    print("  ✅ 通过")

    # Test 3: Position query
    print("\n--- 测试 3: 位置查询 ---")
    for t in targets:
        pos = t.get_position(data)
        print(f"  {t.target_id}: ({pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.3f})")
    print("  ✅ 通过")

    # Test 4: Render
    print("\n--- 测试 4: 渲染 ---")
    rnd = mujoco.Renderer(model, 480, 640)
    rnd.update_scene(data)
    px = rnd.render()
    assert px.shape[2] == 3, f"Expected 3 channels: {px.shape}"
    rnd.close()
    print(f"  渲染: {px.shape}")
    print("  ✅ 通过")

    print(f"\n{'=' * 50}")
    print("  SimTarget — 全部通过 ✅")
    print(f"{'=' * 50}")
