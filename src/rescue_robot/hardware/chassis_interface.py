"""
chassis_interface.py —— 下位机（底盘控制）接口适配层

上位机与下位机之间的坐标/单位转换 + 命令格式化。

下位机约定（底盘层同学提供）：
  - 提供：编码器原始计数、左右轮速度、纯轮式里程计 (x, y, theta)
  - 坐标：x 向前、y 向左、theta 逆时针为正
  - 单位：m、rad、m/s、rad/s

上位机约定（rescue_robot 上层代码）：
  - 坐标：x 向右、y 向前（前方 +Y）、theta 从 +X 逆时针（前方 = pi/2）
  - 单位：mm、rad、mm/s、rad/s

串口命令（上位机 → 下位机）：
  - 速度指令：VEL,v_mm_s,w_mrad_s（v 整数 mm/s，w 整数 mrad/s）
  - 启动：START（比赛启动发一次，下位机将轮式里程计清零为 (0,0,0)）

分工：
  - 下位机负责：差速换算、限速、左右轮 PID（速度闭环）。
  - 上位机负责：传感器融合 + 全局位姿、地图初始偏移管理。
"""

import math
from typing import Tuple


class ChassisInterface:
    """
    下位机接口适配层。

    职责：
      1. 坐标/单位转换：下位机里程计 (m, rad) → 上层全局位姿 (mm, rad)
      2. 速度命令格式化：上层 (mm/s, rad/s) → "VEL,v_mm_s,w_mrad_s"
      3. 初始偏移管理：出发区对齐（下位机里程计原点 → 场地坐标）
    """

    def __init__(self,
                 start_x_mm: float = 150.0,
                 start_y_mm: float = 150.0,
                 start_theta_rad: float = math.pi / 2):
        """
        Args:
            start_x_mm, start_y_mm: 出发区中心在场地坐标中的位置 (mm)
            start_theta_rad: 机器人初始朝向（场地坐标，前方 +Y = pi/2）
        """
        self._start_x = start_x_mm
        self._start_y = start_y_mm
        self._start_theta = start_theta_rad

        # 诊断/冗余数据（下位机原始输出，可选记录）
        self.encoder_counts = (0, 0)
        self.wheel_speeds = (0.0, 0.0)  # 左右轮速度 (m/s)

    # ---- 初始偏移 ----

    def set_start_pose(self, x_mm: float, y_mm: float, theta_rad: float) -> None:
        """设置初始位姿（出发区对齐，比赛启动前调用）。"""
        self._start_x = x_mm
        self._start_y = y_mm
        self._start_theta = theta_rad

    # ---- 坐标转换 ----

    def odom_to_upper(self, x_f_m: float, y_f_m: float,
                      theta_f_rad: float) -> Tuple[float, float, float]:
        """
        下位机纯轮式里程计 (m, rad) → 上层全局位姿 (mm, rad)。

        下位机相对位移在「初始朝向参考系」里：x 向前、y 向左。
        本方法把它旋转到场地坐标系，并加上初始偏移。
        """
        # 初始朝向方向向量（向前 = theta0，向左 = theta0 + pi/2）
        cos_f = math.cos(self._start_theta)
        sin_f = math.sin(self._start_theta)
        cos_l = math.cos(self._start_theta + math.pi / 2)  # = -sin_f
        sin_l = math.sin(self._start_theta + math.pi / 2)  # = cos_f

        dx_mm = (x_f_m * cos_f + y_f_m * cos_l) * 1000.0
        dy_mm = (x_f_m * sin_f + y_f_m * sin_l) * 1000.0

        x = self._start_x + dx_mm
        y = self._start_y + dy_mm
        theta = self._normalize(self._start_theta + theta_f_rad)
        return (x, y, theta)

    def update_raw(self, encoder_counts: Tuple[int, int],
                   wheel_speeds: Tuple[float, float]) -> None:
        """记录下位机原始诊断数据（编码器计数、左右轮速）。"""
        self.encoder_counts = encoder_counts
        self.wheel_speeds = wheel_speeds

    # ---- 命令格式化 ----

    @staticmethod
    def velocity_to_command(v_mm_s: float, w_rad_s: float) -> str:
        """
        上层速度 (mm/s, rad/s) → 串口 "VEL,v_mm_s,w_mrad_s"。

        v 取整为 mm/s，w 换算为 mrad/s（×1000）取整。
        """
        v_int = int(round(v_mm_s))
        w_mrad_int = int(round(w_rad_s * 1000.0))
        return f"VEL,{v_int},{w_mrad_int}"

    @staticmethod
    def start_command() -> str:
        """比赛启动命令：下位机收到后清零轮式里程计为 (0,0,0)。"""
        return "START"

    @staticmethod
    def _normalize(theta: float) -> float:
        while theta > math.pi:
            theta -= 2 * math.pi
        while theta < -math.pi:
            theta += 2 * math.pi
        return theta


# ============================================================
# 独立测试
# ============================================================

if __name__ == "__main__":
    print("=" * 60)
    print("  底盘接口适配层 — 独立测试")
    print("=" * 60)

    # 机器人初始朝 +Y（前方），出发区 (150, 150)
    ci = ChassisInterface(start_x_mm=150.0, start_y_mm=150.0,
                          start_theta_rad=math.pi / 2)

    # --- 测试 1：向前 1m ---
    x, y, theta = ci.odom_to_upper(1.0, 0.0, 0.0)
    print(f"\n向前 1m: pos=({x:.0f}, {y:.0f}), theta={math.degrees(theta):.0f}°")
    assert abs(x - 150) < 1 and abs(y - 1150) < 1, "向前 1m 应沿 +Y"
    assert abs(theta - math.pi / 2) < 1e-9, "初始朝向应为 +Y (90°)"
    print("  ✅ 通过")

    # --- 测试 2：向左 1m ---
    x, y, theta = ci.odom_to_upper(0.0, 1.0, 0.0)
    print(f"向左 1m: pos=({x:.0f}, {y:.0f})")
    assert abs(x - (-850)) < 1 and abs(y - 150) < 1, "向左 1m 应沿 -X"
    print("  ✅ 通过")

    # --- 测试 3：原地转 90°（逆时针）---
    x, y, theta = ci.odom_to_upper(0.0, 0.0, math.pi / 2)
    print(f"转 90°: theta={math.degrees(theta):.0f}°")
    assert abs(theta - math.pi) < 1e-9, "逆时针 90° 应转到 180°（朝 -X）"
    print("  ✅ 通过")

    # --- 测试 4：速度命令格式化 ---
    cmd = ci.velocity_to_command(500.0, 1.5)
    print(f"\n速度 (500mm/s, 1.5rad/s) → {cmd}")
    assert cmd == "VEL,500,1500", f"应为 VEL,500,1500，实际 {cmd}"
    cmd2 = ci.velocity_to_command(-200.0, -0.5)
    print(f"速度 (-200mm/s, -0.5rad/s) → {cmd2}")
    assert cmd2 == "VEL,-200,-500", f"应为 VEL,-200,-500，实际 {cmd2}"
    print("  ✅ 通过")

    # --- 测试 5：START 命令 ---
    assert ci.start_command() == "START"
    print(f"\n启动命令: {ci.start_command()}")
    print("  ✅ 通过")

    print(f"\n{'=' * 60}")
    print("  底盘接口适配层 — 全部测试通过 ✅")
    print(f"{'=' * 60}")
