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

import logging
import math
from typing import Tuple

logger = logging.getLogger("chassis_interface")


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

        # 里程计基线：odom_to_upper 会先减掉它（见 set_odom_baseline 的说明）。
        self._odom_baseline = (0.0, 0.0, 0.0)

        # 诊断/冗余数据（下位机原始输出，可选记录）
        self.encoder_counts = (0, 0)
        self.wheel_speeds = (0.0, 0.0)  # 左右轮速度 (m/s)

    # ---- 初始偏移 ----

    def set_start_pose(self, x_mm: float, y_mm: float, theta_rad: float) -> None:
        """设置初始位姿（出发区对齐，比赛启动前调用）。"""
        self._start_x = x_mm
        self._start_y = y_mm
        self._start_theta = theta_rad

    def set_odom_baseline(self, x_f_m: float, y_f_m: float,
                          theta_f_rad: float) -> None:
        """把"当前下位机里程计读数"记为基线，之后 ``odom_to_upper`` 会先减掉它。

        ⚠️ 为什么必须要有这个（2026-09-16 真机实测，代价是整场跑不起来）：

        协议 §START 声称"下位机收到 START 后清零局部里程计为 (0,0,0)"，
        但**实测并不会** —— 发 START 前后 ODOM 与编码器累计值**逐位相同**：
            前: x=+0.6823 y=+1.0959 th=-1.7043 enc=(921273,1240079)
            后: x=+0.6823 y=+1.0959 th=-1.7043 enc=(921273,1240079)

        而本类把 ODOM 当作**相对位移**使用（``x = _start_x + dx``），
        于是 **MCU 上电以来累积的全部位移都会被当成"从出发区开始走的距离"**
        加到位姿上。实测自主运行日志：
            localization: 位姿重置: (2850, 2850), heading=-90°
            path_planner: A*: 起点 (3973, 2160) 不可通行
        场地只有 3000×3000，(3973,2160) 已是**场外约 1 米** →
        A* 起点不可通行 → 路径规划永远失败 → 车一动不动。

        记基线之后，无论下位机清不清零、之前手动测试跑过多少，
        进 AUTONOMOUS 的那一刻位姿都能正确归零。
        """
        self._odom_baseline = (float(x_f_m), float(y_f_m), float(theta_f_rad))
        logger.info(
            f"里程计基线已记录: ({x_f_m:+.4f}, {y_f_m:+.4f}) m, "
            f"θ={theta_f_rad:+.4f} rad（之后位姿从 0 起算）")

    # ---- 坐标转换 ----

    def odom_to_upper(self, x_f_m: float, y_f_m: float,
                      theta_f_rad: float) -> Tuple[float, float, float]:
        """
        下位机纯轮式里程计 (m, rad) → 上层全局位姿 (mm, rad)。

        下位机相对位移在「初始朝向参考系」里：x 向前、y 向左。
        本方法把它旋转到场地坐标系，并加上初始偏移。

        ⚠️ 两处必须先扣掉「里程计基线」（``set_odom_baseline``）：
        ① 位移 ② **旋转角**。下位机的 START 并不清零里程计，详见那里的说明。
        """
        # ① 先减基线：把"MCU 上电以来的累计量"还原成"相对出发区的位移"
        bx, by, bth = self._odom_baseline
        x_f_m = x_f_m - bx
        y_f_m = y_f_m - by
        theta_f_rad = theta_f_rad - bth

        # ② 旋转角必须用「**里程计坐标系**在场地里的朝向」，即
        #      base_theta = 出发区车头朝向 θ0 − 记基线那一刻里程计自己的 θ
        #
        #    为什么（2026-09-16 现场踩到，直接导致整场跑不起来）：
        #    ODOM 的 x 轴 = **MCU 上电那一刻的车头方向**；而 ``_start_theta``
        #    是**出发区**的车头方向。两者相差"从上电到把车摆好之间，车被转过多少度"。
        #    旧实现直接用 θ0 旋转 → 只要上电后转过车，整个位移方向就整体转错
        #    （大小对、方向错）。
        #    实测：记基线时 ODOM θ=-1.7043rad(-97.7°)，而 θ0 假设 -90°，
        #    于是"直线后退 1.5m"被算成方向错误的 (1513,-369)，位姿跑到场外
        #    (4363,2481) → A* 起点不可通行 → 车一动不动。
        base_theta = self._start_theta - bth

        # 初始朝向方向向量（向前 = base_theta，向左 = base_theta + pi/2）
        cos_f = math.cos(base_theta)
        sin_f = math.sin(base_theta)
        cos_l = math.cos(base_theta + math.pi / 2)  # = -sin_f
        sin_l = math.sin(base_theta + math.pi / 2)  # = cos_f

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
