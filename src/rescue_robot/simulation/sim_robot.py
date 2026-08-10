"""
sim_robot.py —— 我方机器人实体

在 MuJoCo 场景中创建救援机器人 (差分驱动底盘 + 推拨机构)。
机器人通过 qvel 直接控制运动 (模拟轮式驱动)。

物理参数:
  - 底盘: 300×300×200mm (长×宽×高)
  - 质量: 1.5 kg
  - 最大速度: 1000 mm/s
  - 最大角速度: 3.0 rad/s

用法:
  wb = spec.worldbody
  robot = SimRobot(wb, start_zone=1)
  model = spec.compile()
  data = mujoco.MjData(model)
  robot.set_velocity(data, v_linear=0.5, v_angular=0.0)
"""

import math
from typing import Dict, List, Optional, Tuple

import mujoco
import numpy as np

from .sim_models import (
    ROBOT_MASS_KG,
    ROBOT_SIZE_MM,
    mm_to_m,
    rgba,
)

_BOX = mujoco.mjtGeom.mjGEOM_BOX
_CYLINDER = mujoco.mjtGeom.mjGEOM_CYLINDER
_FREE = mujoco.mjtJoint.mjJNT_FREE
_SLIDE = mujoco.mjtJoint.mjJNT_SLIDE

# 出发区中心 (mm)
_START_ZONE_CENTERS_MM = {
    1: (150, 150),       # 左下
    2: (2850, 150),      # 右下
    3: (2850, 2850),     # 右上
    4: (150, 2850),      # 左上
}


class SimRobot:
    """
    救援机器人实体 (MuJoCo 物理仿真)。

    通过直接设置 qvel 控制机器人运动，模拟差速驱动。
    推板机构通过滑动关节实现伸缩。
    """

    MAX_LINEAR_SPEED_MS = 1.0      # 最大线速度 (m/s)
    MAX_ANGULAR_SPEED_RS = 3.0     # 最大角速度 (rad/s)
    WHEEL_BASE_M = 0.200            # 轮距 (m)

    def __init__(self, wb, config=None, start_zone: int = 1):
        """
        Args:
            wb: spec.worldbody (MjsBody)
            config: RobotConfig (可选，使用默认值)
            start_zone: 出发区编号 (1-4)
        """
        self.config = config
        self.start_zone = start_zone
        self._body = None
        self._pusher_body = None
        self._pusher_joint = None

        # 半尺寸 (m)
        sx, sy, sz = mm_to_m(*ROBOT_SIZE_MM)
        self._half_x = sx / 2
        self._half_y = sy / 2
        self._half_z = sz / 2

        # 出发位置
        start_mm = _START_ZONE_CENTERS_MM.get(start_zone, _START_ZONE_CENTERS_MM[1])
        start_x, start_y = mm_to_m(start_mm[0], start_mm[1])
        self._start_pos = [start_x, start_y, self._half_z + 0.001]  # on ground

        self._create(wb)

    def _create(self, wb) -> None:
        """构建机器人 3D 模型。"""
        sx, sy, sz = self._half_x, self._half_y, self._half_z

        # === 底盘 Body ===
        self._body = wb.add_body(
            name="robot_chassis",
            pos=self._start_pos,
        )

        # Free joint → 6DoF motion
        self._body.add_joint(type=_FREE, name="robot_free")

        # Chassis visual + collision
        self._body.add_geom(
            name="robot_chassis_geom",
            type=_BOX,
            size=[sx, sy, sz],
            rgba=rgba("robot_chassis"),
            mass=ROBOT_MASS_KG,
            friction=[0.6, 0.1, 0.4],
        )

        # === 推板 (Pusher) ===
        # 推板 + slider joint 连接到前方
        pusher_w = 0.15    # 推板宽度 (half)
        pusher_d = 0.005   # 推板厚度 (half)
        pusher_h = 0.06    # 推板高度 (half)

        self._pusher_body = wb.add_body(
            name="robot_pusher",
            pos=[sx + pusher_d, 0, 0],   # 前方
        )

        # 滑动关节 (沿 X 轴伸缩)
        self._pusher_joint = self._pusher_body.add_joint(
            type=_SLIDE,
            name="pusher_slide",
            axis=[1, 0, 0],
            range=[0, 0.05],     # 伸出 0~50mm
            limited=True,
        )

        self._pusher_body.add_geom(
            name="robot_pusher_geom",
            type=_BOX,
            size=[pusher_d, pusher_w, pusher_h],
            rgba=rgba("robot_pusher"),
            mass=0.05,            # 50g 推板
        )

        # === 4 个轮子 (视觉装饰) ===
        wheel_r = mm_to_m(25)[0]      # 轮子半径 25mm
        wheel_hw = mm_to_m(6)[0]      # 轮子半宽 6mm
        wheel_offset_x = sx * 0.7
        wheel_offset_y = sy * 0.9
        wheel_z = wheel_r

        wheel_positions = [
            ( wheel_offset_x,  wheel_offset_y, wheel_z),  # 前左
            ( wheel_offset_x, -wheel_offset_y, wheel_z),  # 前右
            (-wheel_offset_x,  wheel_offset_y, wheel_z),  # 后左
            (-wheel_offset_x, -wheel_offset_y, wheel_z),  # 后右
        ]
        for i, (wx, wy, wz) in enumerate(wheel_positions):
            wheel_body = self._body.add_body(
                name=f"wheel_{i + 1}",
                pos=[wx, wy, -sz + wz],
            )
            wheel_body.add_geom(
                name=f"wheel_{i + 1}_geom",
                type=_CYLINDER,
                size=[wheel_r, wheel_hw, 0.0],
                rgba=rgba("robot_wheel"),
                mass=0.02,
            )

    # ---- 姿态初始化 ----

    def init_pose(self, data) -> None:
        """
        将机器人初始化到出发区位置。
        必须在 model.compile() 之后调用。
        """
        # 通过关节名获取 qpos 地址
        try:
            jid = data.model.joint("robot_free").id
            addr = data.model.jnt_qposadr[jid]
        except KeyError:
            # Fallback: use body name to find
            addr = 0

        # 位置
        data.qpos[addr] = self._start_pos[0]
        data.qpos[addr + 1] = self._start_pos[1]
        data.qpos[addr + 2] = self._start_pos[2]
        # 朝向 (无旋转, w=1)
        data.qpos[addr + 3] = 1.0
        data.qpos[addr + 4] = 0.0
        data.qpos[addr + 5] = 0.0
        data.qpos[addr + 6] = 0.0
        # 速度清零
        for i in range(6):
            data.qvel[addr + i] = 0.0

    # ---- 运动控制 ----

    def set_velocity(self, data, v_linear: float, v_angular: float):
        """
        设置机器人速度 (差分驱动模式)。

        Args:
            data: mujoco.MjData
            v_linear: 线速度 (m/s)，正向为 robot X 方向
            v_angular: 角速度 (rad/s)，正值为逆时针
        """
        # 限制速度
        v = max(-self.MAX_LINEAR_SPEED_MS, min(self.MAX_LINEAR_SPEED_MS, v_linear))
        w = max(-self.MAX_ANGULAR_SPEED_RS, min(self.MAX_ANGULAR_SPEED_RS, v_angular))

        # 获取朝向
        quat = self.get_orientation(data)
        yaw = self._quat_to_yaw(quat)

        # 世界系速度
        vx_world = v * math.cos(yaw)
        vy_world = v * math.sin(yaw)

        # 设置 qvel (按关节名查找地址)
        try:
            jid = data.model.joint("robot_free").id
            addr = data.model.jnt_dofadr[jid]
        except KeyError:
            return

        data.qvel[addr] = vx_world
        data.qvel[addr + 1] = vy_world
        data.qvel[addr + 2] = 0.0       # 无 Z 速度
        data.qvel[addr + 3] = 0.0       # 无 pitch
        data.qvel[addr + 4] = 0.0       # 无 roll
        data.qvel[addr + 5] = w         # 角速度绕 Z

    def set_pusher(self, data, extend: float):
        """
        控制推板伸缩。

        Args:
            data: mujoco.MjData
            extend: 0.0 (收回) ~ 1.0 (完全伸出 50mm)
        """
        if self._pusher_joint is None:
            return
        target = extend * 0.05  # 0~50mm
        addr = self._pusher_joint.qposadr[0]
        data.qpos[addr] = target
        # 设置速度为 0 (位置控制)
        data.qvel[addr] = 0.0

    def execute_action(self, data, action):
        """
        执行决策引擎输出的 Action。

        Args:
            data: mujoco.MjData
            action: Action 对象 (from decision_engine)
        """
        # 简化实现: 直接操控 qvel
        # 后续可对接完整 Action 类型
        action_type = getattr(action, 'type', None)
        if action_type is None:
            return

        type_name = action_type.name if hasattr(action_type, 'name') else str(action_type)
        # Navigate
        if 'NAVIGATE' in type_name:
            tx = getattr(action, 'target_x', None) or getattr(action, 'target_position', (0, 0))[0]
            ty = getattr(action, 'target_y', None) or getattr(action, 'target_position', (0, 0))[1]
            # 简单导航: 朝向目标
            pos = self.get_position(data)
            dx = tx - pos[0]
            dy = ty - pos[1]
            dist = math.sqrt(dx * dx + dy * dy)
            if dist > 0.05:  # > 5cm away
                v = min(0.5, dist * 2)  # proportional
                self.set_velocity(data, v, 0.0)
            else:
                self.set_velocity(data, 0.0, 0.0)
        # Grip
        elif 'GRIP' in type_name:
            self.set_pusher(data, 1.0)
        # Release
        elif 'RELEASE' in type_name:
            self.set_pusher(data, 0.0)
        else:
            self.set_velocity(data, 0.0, 0.0)

    # ---- 位置/朝向查询 ----

    def get_position(self, data) -> Tuple[float, float, float]:
        """获取当前 3D 位置 (m)。"""
        try:
            b = data.body("robot_chassis")
            return (float(b.xpos[0]), float(b.xpos[1]), float(b.xpos[2]))
        except KeyError:
            return (0.0, 0.0, 0.0)

    def get_orientation(self, data) -> Tuple[float, float, float, float]:
        """获取朝向四元数 (w, x, y, z)。"""
        try:
            b = data.body("robot_chassis")
            return (float(b.xquat[0]), float(b.xquat[1]),
                    float(b.xquat[2]), float(b.xquat[3]))
        except KeyError:
            return (1.0, 0.0, 0.0, 0.0)

    def get_pose(self, data) -> Tuple[float, float, float]:
        """获取 2D 位姿 (x, y, theta) in m and rad。"""
        x, y, _ = self.get_position(data)
        q = self.get_orientation(data)
        theta = self._quat_to_yaw(q)
        return (x, y, theta)

    @staticmethod
    def _quat_to_yaw(q: Tuple[float, float, float, float]) -> float:
        """四元数 → yaw 角。"""
        w, x, y, z = q
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)


def get_start_center_m(start_zone: int) -> Tuple[float, float]:
    """获取出发区中心位置 (m)。"""
    mm = _START_ZONE_CENTERS_MM.get(start_zone, _START_ZONE_CENTERS_MM[1])
    return mm_to_m(mm[0], mm[1])


# ============================================================
# 独立测试
# ============================================================

if __name__ == "__main__":
    import mujoco as _mj

    print("=" * 50)
    print("  SimRobot 独立测试")
    print("=" * 50)

    spec = _mj.MjSpec()
    spec.option.timestep = 0.002
    spec.option.gravity = [0, 0, -9.81]

    # Ground
    spec.worldbody.add_geom(
        type=_mj.mjtGeom.mjGEOM_PLANE,
        size=[0, 0, 0.001], pos=[1.5, 1.5, 0],
    )

    # Create robot
    robot = SimRobot(spec.worldbody, start_zone=1)

    model = spec.compile()
    data = _mj.MjData(model)
    print(f"  nbody: {model.nbody}, ngeom: {model.ngeom}")

    # Initialize robot pose and run forward kinematics
    robot.init_pose(data)
    _mj.mj_forward(model, data)

    # Test 1: Initial pose
    print("\n--- 测试 1: 初始位姿 ---")
    x, y, theta = robot.get_pose(data)
    assert 0.14 < x < 0.16, f"X should be ~0.15, got {x:.2f}"
    assert 0.14 < y < 0.16, f"Y should be ~0.15, got {y:.2f}"
    print(f"  位姿: ({x:.3f}, {y:.3f}, {theta:.3f})")
    print("  ✅ 通过")

    # Test 2: Forward motion
    print("\n--- 测试 2: 直线前进 ---")
    for _ in range(50):
        robot.set_velocity(data, 0.3, 0.0)
        _mj.mj_step(model, data)
    x2, y2, _ = robot.get_pose(data)
    dx = x2 - x
    assert dx > 0.02, f"应向前移动 >0.02m (沿 X), 实际: {dx:.3f}"
    print(f"  移动后: ({x2:.3f}, {y2:.3f}), Δx = {dx:.3f}m (沿 X 前进)")
    print("  ✅ 通过")

    # Test 3: Turning
    print("\n--- 测试 3: 原地旋转 ---")
    robot.set_velocity(data, 0.0, 2.0)
    for _ in range(50):
        _mj.mj_step(model, data)
    _, _, theta3 = robot.get_pose(data)
    assert abs(theta3) > 0.1, f"应旋转 >0.1rad, 实际: {theta3:.3f}"
    print(f"  旋转后 theta = {theta3:.3f} rad")
    print("  ✅ 通过")

    # Test 4: Speed limit
    print("\n--- 测试 4: 速度限制 ---")
    robot.set_velocity(data, 10.0, 10.0)  # 超限
    # Check qvel after setting
    jid = data.model.joint("robot_free").id
    addr = data.model.jnt_dofadr[jid]
    vx = data.qvel[addr]
    wz = data.qvel[addr + 5]
    assert abs(vx) <= robot.MAX_LINEAR_SPEED_MS + 0.01, f"vx 超限: {vx}"
    assert abs(wz) <= robot.MAX_ANGULAR_SPEED_RS + 0.01, f"wz 超限: {wz}"
    print(f"  限制后 vx={vx:.2f}, wz={wz:.2f}")
    print("  ✅ 通过")

    # Test 5: Render
    print("\n--- 测试 5: 渲染 ---")
    rnd = _mj.Renderer(model, 480, 640)
    rnd.update_scene(data)
    px = rnd.render()
    assert px.shape[2] == 3
    rnd.close()
    print(f"  渲染: {px.shape}")
    print("  ✅ 通过")

    print(f"\n{'=' * 50}")
    print("  SimRobot — 全部通过 ✅")
    print(f"{'=' * 50}")
