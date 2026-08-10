"""
sim_field.py —— 3D 比赛场地构建

基于 MuJoCo MjSpec API 程序化构建完整的 3000×3000mm 比赛场地:
  - 绿色地板
  - 四周围栏 (高 100mm, 厚 20mm)
  - 红/蓝安全区 (600×400mm × 2)
  - 紫色围栏 (安全区面向场地侧)
  - 4 个洋红色出发区 (300×300mm)
  - 12 根减速带 (每个出发区前 3 根)

坐标系统:
  - 原点 bottom-left (场地左下角)
  - +X 向右, +Y 向前 (朝安全区方向), +Z 向上
  - 单位: 米 (MuJoCo 标准)

用法:
  spec = mujoco.MjSpec()
  field = SimField()
  field.build(spec)
  model = spec.compile()
"""

import mujoco

from .sim_models import (
    FIELD_SIZE_MM,
    SAFE_ZONE_SIZE_MM,
    SPEED_BUMP_COUNT,
    SPEED_BUMP_HEIGHT_MM,
    SPEED_BUMP_SPACING_MM,
    SPEED_BUMP_WIDTH_MM,
    START_ZONE_SIZE_MM,
    WALL_HEIGHT_MM,
    WALL_THICKNESS_MM,
    mm_to_m,
    rgba,
)

# MuJoCo 几何类型
_BOX = mujoco.mjtGeom.mjGEOM_BOX
_PLANE = mujoco.mjtGeom.mjGEOM_PLANE
_CYLINDER = mujoco.mjtGeom.mjGEOM_CYLINDER


class SimField:
    """3D 比赛场地构建器。"""

    # 安全区左下角坐标 (mm)
    SAFE_ZONE_RED_POS = (50, 2550)
    SAFE_ZONE_BLUE_POS = (2350, 2550)

    # 4 个出发区左下角坐标 (mm)
    START_ZONES = [
        (0, 0),
        (2700, 0),
        (2700, 2700),
        (0, 2700),
    ]

    def __init__(self):
        self._fw_m = 0.0
        self._fd_m = 0.0

    def build(self, spec) -> None:
        """
        在 spec.worldbody 上构建完整场地。

        Args:
            spec: mujoco.MjSpec 实例
        """
        fw, fd = mm_to_m(FIELD_SIZE_MM[0], FIELD_SIZE_MM[1])
        self._fw_m = fw
        self._fd_m = fd
        wb = spec.worldbody

        self._build_ground(wb)
        self._build_walls(wb)
        self._build_safe_zones(wb)
        self._build_start_zones(wb)
        self._build_speed_bumps(wb)
        self._build_purple_fences(wb)

    # ---- 地板 ----

    def _build_ground(self, wb) -> None:
        fw, fd = self._fw_m, self._fd_m
        wb.add_geom(
            name="field_ground",
            type=_PLANE,
            size=[0, 0, 0.001],
            pos=[fw / 2, fd / 2, 0],
            rgba=rgba("field_green"),
        )

    # ---- 围栏 ----

    def _build_walls(self, wb) -> None:
        fw, fd = self._fw_m, self._fd_m
        hw = mm_to_m(WALL_HEIGHT_MM / 2)[0]   # half height
        tw = mm_to_m(WALL_THICKNESS_MM / 2)[0] # half thickness
        zh = mm_to_m(WALL_HEIGHT_MM / 2)[0]    # Z center

        walls = [
            ("wall_bottom", [fw / 2, 0,      zh], [fw / 2, tw, hw]),
            ("wall_top",    [fw / 2, fd,     zh], [fw / 2, tw, hw]),
            ("wall_left",   [0,      fd / 2, zh], [tw, fd / 2, hw]),
            ("wall_right",  [fw,     fd / 2, zh], [tw, fd / 2, hw]),
        ]
        for name, pos, size in walls:
            wb.add_geom(
                name=name, type=_BOX, size=size, pos=pos,
                rgba=rgba("wall_gray"),
            )

    # ---- 安全区 ----

    def _build_safe_zones(self, wb) -> None:
        sx, sy = mm_to_m(SAFE_ZONE_SIZE_MM[0], SAFE_ZONE_SIZE_MM[1])
        half_z = 0.002      # plate thickness
        border_h = 0.006    # border height
        bw = 0.003           # border width

        for zone_name, (px, py), color_key in [
            ("red",  self.SAFE_ZONE_RED_POS,  "safe_zone_red"),
            ("blue", self.SAFE_ZONE_BLUE_POS, "safe_zone_blue"),
        ]:
            cx, cy = mm_to_m(px, py)
            plate_x = cx + sx / 2
            plate_y = cy + sy / 2

            # Plate
            wb.add_geom(
                name=f"safe_{zone_name}_plate",
                type=_BOX, pos=[plate_x, plate_y, half_z],
                size=[sx / 2, sy / 2, half_z],
                rgba=rgba(color_key),
            )
            # Border
            wb.add_geom(
                name=f"safe_{zone_name}_b_border",
                type=_BOX, pos=[plate_x, cy, border_h],
                size=[sx / 2, bw, border_h],
                rgba=rgba("wall_gray", 0.7),
            )
            wb.add_geom(
                name=f"safe_{zone_name}_t_border",
                type=_BOX, pos=[plate_x, cy + sy, border_h],
                size=[sx / 2, bw, border_h],
                rgba=rgba("wall_gray", 0.7),
            )
            wb.add_geom(
                name=f"safe_{zone_name}_l_border",
                type=_BOX, pos=[cx, plate_y, border_h],
                size=[bw, sy / 2, border_h],
                rgba=rgba("wall_gray", 0.7),
            )
            wb.add_geom(
                name=f"safe_{zone_name}_r_border",
                type=_BOX, pos=[cx + sx, plate_y, border_h],
                size=[bw, sy / 2, border_h],
                rgba=rgba("wall_gray", 0.7),
            )

    # ---- 出发区 ----

    def _build_start_zones(self, wb) -> None:
        sx, sy = mm_to_m(START_ZONE_SIZE_MM[0], START_ZONE_SIZE_MM[1])
        half_z = 0.001

        for i, (px, py) in enumerate(self.START_ZONES):
            cx, cy = mm_to_m(px, py)
            plate_x = cx + sx / 2
            plate_y = cy + sy / 2
            wb.add_geom(
                name=f"start_zone_{i + 1}",
                type=_BOX,
                pos=[plate_x, plate_y, half_z],
                size=[sx / 2, sy / 2, half_z],
                rgba=rgba("start_zone_magenta"),
            )

    # ---- 减速带 ----

    def _build_speed_bumps(self, wb) -> None:
        r = mm_to_m(SPEED_BUMP_WIDTH_MM / 2)[0]        # radius
        length = mm_to_m(START_ZONE_SIZE_MM[1])[0]       # start zone width
        h = mm_to_m(SPEED_BUMP_HEIGHT_MM)[0]             # height (for Z pos)

        for i, (px_mm, py_mm) in enumerate(self.START_ZONES):
            cx, cy = mm_to_m(px_mm, py_mm)
            sxs = mm_to_m(START_ZONE_SIZE_MM[0])[0]

            base_offset_mm = START_ZONE_SIZE_MM[1] + 20
            for j in range(SPEED_BUMP_COUNT):
                offset_m = mm_to_m(base_offset_mm + j * SPEED_BUMP_SPACING_MM)[0]
                bump_x = cx + sxs / 2
                bump_y = cy + offset_m
                bump_z = h / 2

                wb.add_geom(
                    name=f"bump_z{i + 1}_{j}",
                    type=_CYLINDER,
                    pos=[bump_x, bump_y, bump_z],
                    size=[r, length / 2, 0.0],
                    rgba=rgba("speed_bump_yellow"),
                )

    # ---- 紫色围栏 ----

    def _build_purple_fences(self, wb) -> None:
        sx, sy = SAFE_ZONE_SIZE_MM  # mm
        fh2 = mm_to_m(50)[0]        # half height
        fw = mm_to_m(6)[0]           # half width
        p_color = rgba("purple_fence")

        for zone_name, safe_pos in [
            ("red", self.SAFE_ZONE_RED_POS),
            ("blue", self.SAFE_ZONE_BLUE_POS),
        ]:
            px, py = mm_to_m(safe_pos[0], safe_pos[1])
            sx_m, sy_m = mm_to_m(sx, sy)

            fence_x = px + sx_m / 2
            fence_y = py

            wb.add_geom(
                name=f"fence_{zone_name}",
                type=_BOX,
                pos=[fence_x, fence_y, fh2],
                size=[sx_m / 2, fw, fh2],
                rgba=p_color,
            )
            # Top cap
            wb.add_geom(
                name=f"fence_{zone_name}_cap",
                type=_BOX,
                pos=[fence_x, fence_y, fh2 * 2 - 0.001],
                size=[sx_m / 2, mm_to_m(5)[0], mm_to_m(3)[0]],
                rgba=[0.5, 0.15, 0.7, 1.0],
            )


def build_field_on_spec(spec):
    """在给定 spec 上构建场地（便捷函数）。"""
    field = SimField()
    field.build(spec)
    return field


# ============================================================
# 独立测试
# ============================================================

if __name__ == "__main__":
    import mujoco as _mj

    print("=" * 50)
    print("  SimField 独立测试")
    print("=" * 50)

    spec = _mj.MjSpec()
    spec.option.timestep = 0.002
    spec.option.gravity = [0, 0, -9.81]

    # 测试 1: 构建
    print("\n--- 测试 1: 构建完整场地 ---")
    field = SimField()
    field.build(spec)
    model = spec.compile()
    assert model.ngeom >= 25, f"至少 25 个几何体, 实际: {model.ngeom}"
    print(f"  几何体: {model.ngeom}")
    print("  ✅ 通过")

    # 测试 2: 物理
    print("\n--- 测试 2: 物理步进 ---")
    data = _mj.MjData(model)
    for _ in range(100):
        _mj.mj_step(model, data)
    print("  100 步 OK")
    print("  ✅ 通过")

    # 测试 3: 渲染
    print("\n--- 测试 3: 渲染 ---")
    rnd = _mj.Renderer(model, 480, 480)
    rnd.update_scene(data)
    px = rnd.render()
    assert px.shape == (480, 480, 3)
    rnd.close()
    print(f"  渲染: {px.shape}")
    print("  ✅ 通过")

    print(f"\n{'=' * 50}")
    print("  SimField — 全部通过 ✅")
    print(f"{'=' * 50}")
