"""
omni_kinematics.py —— 三全向轮运动学

三个全向轮以 120° 均布，实现完整全向（holonomic）运动。
车体坐标系：+x 前向，+y 左侧，+ω 逆时针（自顶向下俯视）。

逆运动学（车体速度 → 车轮线速度）：
    设车轮 i 的安装角为 φᵢ（逆时针，自车体 +x 轴起算），
    车轮到车体中心的距离（回转半径）为 L，车体速度为 (vx, vy, ω)，
    则车轮 i 沿其驱动方向的线速度：

        vᵢ = −vx·sin(φᵢ) + vy·cos(φᵢ) + L·ω

    默认 φ = [0°, 120°, 240°]，L = 150mm（底盘 300mm 的半径）。

前运动学（车轮线速度 → 车体速度）：
    上述线性方程组对应的 3×3 矩阵（车轮 120° 均布时满秩）求逆，
    用伴随矩阵法精确求解，无需外部依赖。

自洽性由往返测试 forward(inverse(v)) ≈ v 保证（与方向/符号约定无关）。
"""

import logging
import math
from dataclasses import dataclass, field
from typing import List, Tuple

logger = logging.getLogger("omni_kinematics")


@dataclass
class OmniDriveKinematics:
    """三全向轮运动学（车体速度 ↔ 三车轮线速度）。"""

    wheel_angles_deg: List[float] = field(
        default_factory=lambda: [0.0, 120.0, 240.0]
    )
    wheel_mount_radius_mm: float = 150.0   # 车轮回转半径 L
    wheel_diameter_mm: float = 65.0        # 轮径（半径 32.5mm）

    def __post_init__(self):
        if len(self.wheel_angles_deg) != 3:
            raise ValueError(
                f"全向轮必须为 3 个，当前: {len(self.wheel_angles_deg)}"
            )
        self._angles_rad = [math.radians(a) for a in self.wheel_angles_deg]
        self._sin = [math.sin(a) for a in self._angles_rad]
        self._cos = [math.cos(a) for a in self._angles_rad]

        L = self.wheel_mount_radius_mm
        # 逆运动学矩阵 M，行 i = [-sin(φᵢ), cos(φᵢ), L]
        self._M = [[-s, c, L] for s, c in zip(self._sin, self._cos)]
        self._M_inv = self._inverse3(self._M)

    # ---- 逆运动学 ----

    def inverse(self, vx: float, vy: float, omega: float) -> List[float]:
        """
        车体速度 → 三个车轮线速度。

        Args:
            vx: 车体前向速度 (mm/s)
            vy: 车体侧向速度 (mm/s)，左为正
            omega: 车体角速度 (rad/s)，逆时针为正

        Returns:
            [v0, v1, v2] 三个车轮沿驱动方向的线速度 (mm/s)
        """
        L = self.wheel_mount_radius_mm
        return [
            -s * vx + c * vy + L * omega
            for s, c in zip(self._sin, self._cos)
        ]

    # ---- 前运动学 ----

    def forward(self, wheel_speeds: List[float]) -> Tuple[float, float, float]:
        """
        三个车轮线速度 → 车体速度。

        Args:
            wheel_speeds: [v0, v1, v2] 三个车轮线速度 (mm/s)

        Returns:
            (vx, vy, omega) 车体速度 (mm/s, mm/s, rad/s)
        """
        if len(wheel_speeds) != 3:
            raise ValueError(f"需要 3 个车轮速度，当前: {len(wheel_speeds)}")
        inv = self._M_inv
        return (
            inv[0][0] * wheel_speeds[0] + inv[0][1] * wheel_speeds[1] + inv[0][2] * wheel_speeds[2],
            inv[1][0] * wheel_speeds[0] + inv[1][1] * wheel_speeds[1] + inv[1][2] * wheel_speeds[2],
            inv[2][0] * wheel_speeds[0] + inv[2][1] * wheel_speeds[1] + inv[2][2] * wheel_speeds[2],
        )

    # ---- 单位换算 ----

    def wheel_speed_to_rpm(self, v_mm_s: float,
                           reduction_ratio: float = 30.0) -> float:
        """
        车轮线速度 → 电机 RPM。

        车轮角速度 ω_wheel = v / r，电机转速 = ω_wheel × 减速比。
        """
        radius_mm = self.wheel_diameter_mm / 2.0
        if radius_mm <= 0:
            return 0.0
        wheel_rps = v_mm_s / (2.0 * math.pi * radius_mm)  # 轮子每秒转数
        return wheel_rps * 60.0 * reduction_ratio

    # ---- 工具 ----

    @staticmethod
    def _inverse3(m: List[List[float]]) -> List[List[float]]:
        """3×3 矩阵求逆（伴随矩阵 / 克拉默法则），精确无外部依赖。"""
        a, b, c = m[0]
        d, e, f = m[1]
        g, h, i = m[2]

        det = (a * (e * i - f * h)
               - b * (d * i - f * g)
               + c * (d * h - e * g))
        if abs(det) < 1e-12:
            raise ValueError("全向轮安装角线性相关，运动学矩阵不可逆")

        return [
            [(e * i - f * h) / det, (c * h - b * i) / det, (b * f - c * e) / det],
            [(f * g - d * i) / det, (a * i - c * g) / det, (c * d - a * f) / det],
            [(d * h - e * g) / det, (b * g - a * h) / det, (a * e - b * d) / det],
        ]


# ============================================================
# 独立测试
# ============================================================

if __name__ == "__main__":
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s [%(levelname)-7s] %(name)s: %(message)s",
        datefmt="%H:%M:%S",
    )

    print("=" * 60)
    print("  三全向轮运动学 — 独立测试")
    print("=" * 60)

    kin = OmniDriveKinematics()
    print(f"  安装角: {kin.wheel_angles_deg}°")
    print(f"  回转半径: {kin.wheel_mount_radius_mm}mm, 轮径: {kin.wheel_diameter_mm}mm")

    # ---- 测试 1: 往返一致性 ----
    print("\n--- 测试 1: 往返 forward(inverse(v)) ≈ v ---")
    import random
    random.seed(0)
    max_err = 0.0
    for _ in range(1000):
        vx = random.uniform(-1000, 1000)
        vy = random.uniform(-1000, 1000)
        w = random.uniform(-3, 3)
        ws = kin.inverse(vx, vy, w)
        rx, ry, rw = kin.forward(ws)
        err = max(abs(rx - vx), abs(ry - vy), abs(rw - w))
        max_err = max(max_err, err)
    print(f"  最大往返误差: {max_err:.2e} (应 < 1e-6)")
    assert max_err < 1e-6, f"往返误差过大: {max_err}"
    print("  ✅ 通过")

    # ---- 测试 2: 已知向量校验 ----
    print("\n--- 测试 2: 已知向量校验 ---")
    # 纯前向: (0, 100, 0) → 各轮
    ws = kin.inverse(0, 100, 0)
    print(f"  纯前向 (vx=0, vy=100): wheels={[round(v, 1) for v in ws]}")
    assert abs(ws[0] - 100) < 1e-9          # 0° 轮: cos0·100 = 100
    assert abs(ws[1] - (-50)) < 1e-9        # 120° 轮: cos120·100 = -50
    assert abs(ws[2] - (-50)) < 1e-9        # 240° 轮: cos240·100 = -50

    # 纯横向: (100, 0, 0)
    ws = kin.inverse(100, 0, 0)
    print(f"  纯横向 (vx=100, vy=0): wheels={[round(v, 1) for v in ws]}")
    assert abs(ws[0]) < 1e-9                # 0° 轮: -sin0·100 = 0
    assert abs(ws[1] - (-86.6025)) < 1e-3   # -sin120·100 ≈ -86.60
    assert abs(ws[2] - 86.6025) < 1e-3      # -sin240·100 ≈ +86.60

    # 原地旋转: (0, 0, ω)
    ws = kin.inverse(0, 0, 1.0)
    print(f"  原地旋转 (ω=1): wheels={[round(v, 1) for v in ws]}")
    for v in ws:
        assert abs(v - kin.wheel_mount_radius_mm) < 1e-9
    print("  ✅ 通过")

    # ---- 测试 3: RPM 换算 ----
    print("\n--- 测试 3: RPM 换算 ---")
    # 轮径 65mm → 半径 32.5mm；v=100mm/s → ω=100/(2π·32.5) rps
    rpm = kin.wheel_speed_to_rpm(100.0, reduction_ratio=30.0)
    expect = 100.0 / (2 * math.pi * 32.5) * 60.0 * 30.0
    assert abs(rpm - expect) < 1e-9
    print(f"  v=100mm/s → {rpm:.1f} RPM (减速比 30)")
    print("  ✅ 通过")

    print(f"\n{'=' * 60}")
    print("  三全向轮运动学 — 全部测试通过 ✅")
    print(f"{'=' * 60}")
