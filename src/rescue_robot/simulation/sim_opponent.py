"""
sim_opponent.py —— 对手机器人 AI

创建对方机器人及其 AI 行为策略。
支持 3 种行为模式:
  - random: 随机游走
  - competitive: 竞争高价值目标
  - aggressive: 拦截我方机器人

用法:
  spec = mujoco.MjSpec()
  opponent = SimOpponent(spec.worldbody, behavior="competitive", start_zone=2)
  model = spec.compile()
  opponent.init_pose(data)
  opponent.update(data, targets, our_robot_pose)
"""

import math
import random
from typing import Any, List, Optional, Tuple

import mujoco

from .sim_models import (
    ROBOT_MASS_KG,
    ROBOT_SIZE_MM,
    START_ZONE_SIZE_MM,
    mm_to_m,
    rgba,
)

_BOX = mujoco.mjtGeom.mjGEOM_BOX
_CYLINDER = mujoco.mjtGeom.mjGEOM_CYLINDER
_FREE = mujoco.mjtJoint.mjJNT_FREE

# 出发区中心 (mm)
_START_CENTERS_MM = {
    1: (150, 150),
    2: (2850, 150),
    3: (2850, 2850),
    4: (150, 2850),
}


class SimOpponent:
    """
    对手机器人实体 + AI 行为。

    与 SimRobot 类似的物理模型 (300×300mm 底盘)，
    但由 AI 策略驱动而非决策引擎。
    """

    MAX_LINEAR_SPEED = 0.8       # m/s (略慢于我方)
    MAX_ANGULAR_SPEED = 2.5      # rad/s

    def __init__(self, wb, behavior: str = "competitive",
                 start_zone: int = 2):
        """
        Args:
            wb: spec.worldbody
            behavior: "random" | "competitive" | "aggressive"
            start_zone: 出发区 (1-4)
        """
        self.behavior = behavior
        self.start_zone = start_zone
        self._body = None

        # 状态
        self._current_target_pos: Optional[Tuple[float, float]] = None
        self._stuck_timer = 0.0
        self._last_pos: Optional[Tuple[float, float]] = None

        # 半尺寸
        sx, sy, sz = mm_to_m(*ROBOT_SIZE_MM)
        self._half_x = sx / 2
        self._half_y = sy / 2
        self._half_z = sz / 2

        # 初始位置
        start_mm = _START_CENTERS_MM.get(start_zone, _START_CENTERS_MM[2])
        self._start_pos = [
            mm_to_m(start_mm[0])[0],
            mm_to_m(start_mm[1])[0],
            self._half_z + 0.001,
        ]

        self._create(wb)

    def _create(self, wb) -> None:
        """创建 3D 模型。"""
        sx, sy, sz = self._half_x, self._half_y, self._half_z

        self._body = wb.add_body(
            name="opponent_chassis",
            pos=self._start_pos,
        )
        self._body.add_joint(type=_FREE, name="opponent_free")

        # 底盘
        self._body.add_geom(
            name="opponent_chassis_geom",
            type=_BOX,
            size=[sx, sy, sz],
            rgba=rgba("opponent_chassis"),
            mass=ROBOT_MASS_KG,
            friction=[0.6, 0.1, 0.4],
        )

        # 轮子 (装饰)
        wheel_r = mm_to_m(25)[0]
        wheel_hw = mm_to_m(6)[0]
        wheel_z = wheel_r
        wheel_positions = [
            (sx * 0.7,  sy * 0.9, -sz + wheel_z),
            (sx * 0.7, -sy * 0.9, -sz + wheel_z),
            (-sx * 0.7,  sy * 0.9, -sz + wheel_z),
            (-sx * 0.7, -sy * 0.9, -sz + wheel_z),
        ]
        for i, (wx, wy, wz) in enumerate(wheel_positions):
            w = self._body.add_body(
                name=f"opponent_wheel_{i + 1}",
                pos=[wx, wy, wz],
            )
            w.add_geom(
                name=f"opponent_wheel_{i + 1}_geom",
                type=_CYLINDER,
                size=[wheel_r, wheel_hw, 0.0],
                rgba=rgba("opponent_chassis"),
                mass=0.02,
            )

    # ---- 初始化 ----

    def init_pose(self, data) -> None:
        """设置初始位姿。"""
        try:
            jid = data.model.joint("opponent_free").id
            addr = data.model.jnt_qposadr[jid]
        except KeyError:
            return
        data.qpos[addr] = self._start_pos[0]
        data.qpos[addr + 1] = self._start_pos[1]
        data.qpos[addr + 2] = self._start_pos[2]
        data.qpos[addr + 3] = 1.0  # w
        data.qpos[addr + 4] = 0.0  # x
        data.qpos[addr + 5] = 0.0  # y
        data.qpos[addr + 6] = 0.0  # z
        for i in range(6):
            data.qvel[data.model.jnt_dofadr[jid] + i] = 0.0

    # ---- 运动 ----

    def set_velocity(self, data, v_linear: float, v_angular: float) -> None:
        """设置速度。"""
        v = max(-self.MAX_LINEAR_SPEED, min(self.MAX_LINEAR_SPEED, v_linear))
        w = max(-self.MAX_ANGULAR_SPEED, min(self.MAX_ANGULAR_SPEED, v_angular))

        try:
            jid = data.model.joint("opponent_free").id
            dof = data.model.jnt_dofadr[jid]
            qpos = data.model.jnt_qposadr[jid]
        except KeyError:
            return

        # 朝向
        wq = data.qpos[qpos + 3]
        xq = data.qpos[qpos + 4]
        yq = data.qpos[qpos + 5]
        zq = data.qpos[qpos + 6]
        yaw = self._quat_to_yaw((wq, xq, yq, zq))

        vx_w = v * math.cos(yaw)
        vy_w = v * math.sin(yaw)

        data.qvel[dof] = vx_w
        data.qvel[dof + 1] = vy_w
        data.qvel[dof + 2] = 0.0
        data.qvel[dof + 3] = 0.0
        data.qvel[dof + 4] = 0.0
        data.qvel[dof + 5] = w

    def get_pose(self, data) -> Tuple[float, float, float]:
        """获取位姿 (x, y, yaw)。"""
        try:
            b = data.body("opponent_chassis")
            w, x, y, z = (b.xquat[0], b.xquat[1], b.xquat[2], b.xquat[3])
            yaw = self._quat_to_yaw((w, x, y, z))
            return (float(b.xpos[0]), float(b.xpos[1]), yaw)
        except KeyError:
            return (0.0, 0.0, 0.0)

    @staticmethod
    def _quat_to_yaw(q: Tuple[float, ...]) -> float:
        w, x, y, z = q
        return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))

    # ---- AI 策略 ----

    def update(self, data, targets: List[Any],
               our_robot_pose: Tuple[float, float, float],
               dt: float = 0.02) -> None:
        """
        运行 AI 决策。

        Args:
            data: mujoco.MjData
            targets: 目标列表 (需要 .is_delivered, ._pos_m, .points)
            our_robot_pose: (x, y, yaw) 我方位姿
            dt: 决策步长
        """
        ox, oy, oyaw = self.get_pose(data)

        if self.behavior == "random":
            self._behavior_random(data, ox, oy, oyaw)
        elif self.behavior == "competitive":
            self._behavior_competitive(data, ox, oy, oyaw, targets, our_robot_pose)
        elif self.behavior == "aggressive":
            self._behavior_aggressive(data, ox, oy, oyaw, targets, our_robot_pose)
        else:
            self.set_velocity(data, 0.0, 0.0)

        # Stuck detection
        if self._last_pos is not None:
            dp = math.sqrt((ox - self._last_pos[0])**2 +
                           (oy - self._last_pos[1])**2)
            if dp < 0.005:
                self._stuck_timer += dt
            else:
                self._stuck_timer = max(0, self._stuck_timer - dt * 2)
        self._last_pos = (ox, oy)

        # Unstick by reversing
        if self._stuck_timer > 0.5:
            self.set_velocity(data, -0.3, 0.3)
            self._stuck_timer = 0.0

    def _behavior_random(self, data, ox, oy, oyaw) -> None:
        """随机游走。"""
        if self._current_target_pos is None:
            self._current_target_pos = (
                random.uniform(0.3, 2.7),
                random.uniform(0.3, 2.7),
            )

        tx, ty = self._current_target_pos
        dx, dy = tx - ox, ty - oy
        dist = math.sqrt(dx * dx + dy * dy)

        if dist < 0.2:
            self._current_target_pos = None  # pick new target next tick
            self.set_velocity(data, 0.0, 0.0)
        else:
            target_angle = math.atan2(dy, dx)
            err = target_angle - oyaw
            err = math.atan2(math.sin(err), math.cos(err))
            v = min(0.4, dist * 1.5)
            self.set_velocity(data, v, err * 4)

    def _behavior_competitive(self, data, ox, oy, oyaw,
                               targets, our_pose) -> None:
        """竞争策略: 选择我方附近高价值目标。"""
        rx, ry, _ = our_pose

        # 找到我方附近的高价值目标
        best_target = None
        best_score = -1
        for t in targets:
            if t.is_delivered:
                continue
            tx, ty = t._pos_m[0], t._pos_m[1]
            # 目标离我方近 = 高优先级
            d_to_us = math.sqrt((tx - rx)**2 + (ty - ry)**2)
            if d_to_us < 1.5:  # 我方 1.5m 范围内的目标
                pts = t.points
                if pts > best_score:
                    best_score = pts
                    best_target = (tx, ty)

        # 如果附近没有我方关注的目标，随机游走
        if best_target is None:
            self._behavior_random(data, ox, oy, oyaw)
            return

        self._navigate_to(data, ox, oy, oyaw, best_target[0], best_target[1])

    def _behavior_aggressive(self, data, ox, oy, oyaw,
                               targets, our_pose) -> None:
        """进攻策略: 尝试拦截我方机器人。"""
        rx, ry, _ = our_pose

        # 拦截: 朝我方机器人前方移动
        our_heading = math.cos(our_pose[2]), math.sin(our_pose[2])
        intercept_x = rx + our_heading[0] * 0.3
        intercept_y = ry + our_heading[1] * 0.3

        self._navigate_to(data, ox, oy, oyaw, intercept_x, intercept_y)

    def _navigate_to(self, data, ox, oy, oyaw, tx, ty) -> None:
        """导航到目标点。"""
        dx, dy = tx - ox, ty - oy
        dist = math.sqrt(dx * dx + dy * dy)

        if dist < 0.15:
            self.set_velocity(data, 0.0, 0.0)
            return

        target_angle = math.atan2(dy, dx)
        err = target_angle - oyaw
        err = math.atan2(math.sin(err), math.cos(err))

        if abs(err) > 0.3:
            # 先旋转
            w = min(2.0, abs(err) * 4) * (1 if err > 0 else -1)
            self.set_velocity(data, 0.1, w)
        else:
            # 直行
            v = min(0.6, dist * 2)
            self.set_velocity(data, v, err * 3)


# ============================================================
# 独立测试
# ============================================================

if __name__ == "__main__":
    import mujoco as _mj
    from dataclasses import dataclass

    @dataclass
    class _FakeTarget:
        target_id: int
        _pos_m: tuple
        is_delivered: bool = False
        points: int = 5
        is_dangerous: bool = False

    print("=" * 50)
    print("  SimOpponent 独立测试")
    print("=" * 50)

    targets = [
        _FakeTarget(1, (1.0, 1.0), points=5),
        _FakeTarget(2, (2.0, 1.5), points=10),
        _FakeTarget(3, (0.5, 2.0), points=15),
    ]
    our_pose = (1.5, 1.5, 0.0)

    for behavior in ["random", "competitive", "aggressive"]:
        print(f"\n--- 测试: {behavior} ---")
        spec = _mj.MjSpec()
        spec.option.timestep = 0.004
        spec.option.gravity = [0, 0, -9.81]
        spec.worldbody.add_geom(
            type=_mj.mjtGeom.mjGEOM_PLANE,
            size=[0, 0, 0.001], pos=[1.5, 1.5, 0],
        )

        opp = SimOpponent(spec.worldbody, behavior=behavior, start_zone=2)
        model = spec.compile()
        data = _mj.MjData(model)
        opp.init_pose(data)
        _mj.mj_forward(model, data)

        # Run steps
        for _ in range(30):
            opp.update(data, targets, our_pose)
            _mj.mj_step(model, data)

        x, y, _ = opp.get_pose(data)
        print(f"  行为: {behavior}")
        print(f"  位置: ({x:.3f}, {y:.3f})")
        assert 0 < x < 3, f"X 应在场地内: {x}"
        assert 0 < y < 3, f"Y 应在场地内: {y}"
        print("  ✅ 通过")

    print(f"\n{'=' * 50}")
    print("  SimOpponent — 全部通过 ✅")
    print(f"{'=' * 50}")
