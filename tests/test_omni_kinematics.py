"""
test_omni_kinematics.py —— 三全向轮运动学与全向控制测试

验证:
  - 正/逆运动学往返一致性 forward(inverse(v)) ≈ v
  - 已知向量（纯前向 / 纯横向 / 原地旋转）的符号与幅值
  - MotionController 全向行为（目标在正侧方时产生显著 lateral 横移）
"""

import logging
import math
import random

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)-7s] %(name)s: %(message)s",
    datefmt="%H:%M:%S",
)


def test_roundtrip():
    from rescue_robot.navigation.omni_kinematics import OmniDriveKinematics
    kin = OmniDriveKinematics()
    random.seed(0)
    max_err = 0.0
    for _ in range(5000):
        vx = random.uniform(-1500, 1500)
        vy = random.uniform(-1500, 1500)
        w = random.uniform(-3.0, 3.0)
        wheels = kin.inverse(vx, vy, w)
        rx, ry, rw = kin.forward(wheels)
        err = max(abs(rx - vx), abs(ry - vy), abs(rw - w))
        max_err = max(max_err, err)
    assert max_err < 1e-6, f"往返误差过大: {max_err}"
    print(f"  往返最大误差: {max_err:.2e} (5000 组随机)  ✅")


def test_known_vectors():
    from rescue_robot.navigation.omni_kinematics import OmniDriveKinematics
    kin = OmniDriveKinematics()

    # 纯前向 (vx=0, vy=100)：0° 轮 cos0·100=100，120°/240° 轮 cos120/240·100=-50
    ws = kin.inverse(0.0, 100.0, 0.0)
    assert abs(ws[0] - 100.0) < 1e-9
    assert abs(ws[1] - (-50.0)) < 1e-9
    assert abs(ws[2] - (-50.0)) < 1e-9

    # 纯横向 (vx=100, vy=0)：0° 轮 -sin0·100=0，120° 轮 -sin120·100≈-86.6
    ws = kin.inverse(100.0, 0.0, 0.0)
    assert abs(ws[0]) < 1e-9
    assert abs(ws[1] - (-86.6025)) < 1e-3
    assert abs(ws[2] - 86.6025) < 1e-3

    # 原地旋转 (ω=1)：三轮线速度均 = L·ω = 150
    ws = kin.inverse(0.0, 0.0, 1.0)
    for v in ws:
        assert abs(v - kin.wheel_mount_radius_mm) < 1e-9
    print("  纯前向/纯横向/原地旋转 符号幅值  ✅")


def test_controller_holonomic():
    from rescue_robot.navigation.motion_control import MotionController
    mc = MotionController()

    # 目标在正前方 → 前进为主，侧向约 0
    cmd = mc.compute_velocity((2000.0, 1500.0), (1500.0, 1500.0, 0.0), dt=0.02)
    assert cmd.linear > 0, f"前向目标应前进: linear={cmd.linear}"
    assert abs(cmd.lateral) < 1e-6, f"前向目标侧向应≈0: lateral={cmd.lateral}"

    # 目标在正侧方（heading=0，目标在 +y）→ 应产生显著 lateral
    cmd = mc.compute_velocity((1500.0, 2000.0), (1500.0, 1500.0, 0.0), dt=0.02)
    assert abs(cmd.lateral) > 100, f"侧方目标应横移: lateral={cmd.lateral}"
    assert abs(cmd.linear) < 1e-6, f"侧方目标前向应≈0: linear={cmd.linear}"
    print(f"  前向: linear={cmd.linear:.0f}; 侧方目标 lateral={cmd.lateral:.0f} (横移)  ✅")


def test_localizer_holonomic():
    from rescue_robot.navigation.localization import MockLocalizer
    loc = MockLocalizer(start_x=0.0, start_y=0.0, start_theta=0.0)

    # 纯横向运动（heading=0，vy=100 → 世界系 +y 方向移动）
    p0 = loc.pose
    loc.update(linear_velocity=0.0, lateral_velocity=100.0, angular_velocity=0.0, dt=1.0)
    p1 = loc.pose
    assert abs(p1.y - 100.0) < 1e-6, f"纯横向应沿 +y 移动: y={p1.y}"
    assert abs(p1.x) < 1e-6
    print(f"  全向里程计: (0,0) → ({p1.x:.1f}, {p1.y:.1f})  ✅")


if __name__ == "__main__":
    print("=" * 60)
    print("  三全向轮运动学与全向控制测试")
    print("=" * 60)
    test_roundtrip()
    test_known_vectors()
    test_controller_holonomic()
    test_localizer_holonomic()
    print(f"\n{'=' * 60}")
    print("  全部测试通过 ✅")
    print(f"{'=' * 60}")
