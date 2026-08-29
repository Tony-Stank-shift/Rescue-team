"""
motion_control.py —— 运动控制

差速驱动机器人的底层运动控制：
  - PID 控制器（位置 + 速度级联）
  - 目标到达判定
  - 越障模式（减速带）
  - 防打滑监测
"""

import logging
import math
import time
from dataclasses import dataclass
from typing import Optional, Tuple

logger = logging.getLogger("motion_control")


# ============================================================
# 速度指令
# ============================================================

@dataclass
class VelocityCommand:
    """速度指令"""
    linear: float = 0.0    # 线速度 (mm/s)
    angular: float = 0.0   # 角速度 (rad/s)
    timestamp: float = 0.0


# ============================================================
# PID 控制器
# ============================================================

class PIDController:
    """
    离散 PID 控制器。

    包含积分限幅和微分滤波。
    """

    def __init__(self, kp: float, ki: float = 0.0, kd: float = 0.0,
                 output_min: float = float('-inf'),
                 output_max: float = float('inf'),
                 integral_max: float = float('inf')):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_min = output_min
        self.output_max = output_max
        self.integral_max = integral_max

        self._prev_error = 0.0
        self._integral = 0.0
        self._last_time = 0.0

    def reset(self) -> None:
        self._prev_error = 0.0
        self._integral = 0.0
        self._last_time = 0.0

    def compute(self, setpoint: float, measurement: float,
                dt: Optional[float] = None) -> float:
        """计算 PID 输出"""
        if dt is None:
            now = time.time()
            dt = now - self._last_time if self._last_time > 0 else 0.02
            self._last_time = now

        if dt <= 0:
            return 0.0

        error = setpoint - measurement

        # P 项
        p_out = self.kp * error

        # I 项（带限幅）
        self._integral += error * dt
        self._integral = max(-self.integral_max,
                             min(self.integral_max, self._integral))
        i_out = self.ki * self._integral

        # D 项（基于误差变化率）
        d_out = self.kd * (error - self._prev_error) / dt
        self._prev_error = error

        output = p_out + i_out + d_out
        return max(self.output_min, min(self.output_max, output))


# ============================================================
# 运动控制器
# ============================================================

class MotionController:
    """
    差速驱动机器人运动控制器。

    控制模式：
    - POSITION: 精确到达目标点
    - VELOCITY: 跟踪速度指令（用于路径跟踪）
    - BUMP_CROSSING: 越障模式（减速带）
    """

    # 默认 PID 参数
    PID_DISTANCE = (0.8, 0.01, 0.05)    # 距离控制 (kp, ki, kd)
    PID_ANGLE = (1.5, 0.0, 0.1)         # 角度控制

    # 到达判定阈值
    POSITION_TOLERANCE_MM = 40.0        # 位置容差（略大于网格分辨率）
    ANGLE_TOLERANCE_RAD = 0.15          # 角度容差 (~9°)，容忍网格离散误差
    VELOCITY_ZERO_THRESHOLD = 10.0      # 静止判定速度 (mm/s)

    # 越障参数
    BUMP_SPEED_MM_S = 200.0             # 越障速度
    BUMP_CROSS_TIME_S = 1.0             # 单根减速带通过时间

    def __init__(self,
                 max_linear_speed: float = 1000.0,
                 max_angular_speed: float = 3.0,
                 wheel_base_mm: float = 209.0):
        self._max_v = max_linear_speed
        self._max_w = max_angular_speed
        self._wheel_base = wheel_base_mm

        # PID 控制器
        self._pid_distance = PIDController(
            *self.PID_DISTANCE,
            output_min=-max_linear_speed,
            output_max=max_linear_speed,
            integral_max=500.0,
        )
        self._pid_angle = PIDController(
            *self.PID_ANGLE,
            output_min=-max_angular_speed,
            output_max=max_angular_speed,
            integral_max=1.0,
        )

        # 状态
        self._current_command = VelocityCommand()
        self._is_bump_mode = False
        self._bump_timer = 0.0
        self._bump_count = 0

        logger.info(f"MotionController 初始化: max_v={max_linear_speed}mm/s, "
                     f"max_w={max_angular_speed}rad/s")

    # ---- 主控制接口 ----

    def compute_velocity(self,
                         target: Tuple[float, float],
                         current_pose: Tuple[float, float, float],
                         dt: float = 0.02) -> VelocityCommand:
        """
        计算到达目标点所需的速度指令。

        Args:
            target: (x_mm, y_mm) 目标点
            current_pose: (x, y, theta) 当前位姿
            dt: 时间步长

        Returns:
            VelocityCommand
        """
        cx, cy, ctheta = current_pose
        tx, ty = target

        # 目标方向
        dx = tx - cx
        dy = ty - cy
        distance = math.sqrt(dx * dx + dy * dy)
        target_angle = math.atan2(dy, dx)

        # 角度误差（朝向目标）
        angle_error = target_angle - ctheta
        angle_error = self._normalize_angle(angle_error)

        # 已到达？
        if distance < self.POSITION_TOLERANCE_MM:
            self._pid_distance.reset()
            self._pid_angle.reset()
            return VelocityCommand(linear=0.0, angular=0.0, timestamp=time.time())

        # 连续过渡：角度误差大时减速+转向，小时全速前进
        angle_ratio = min(1.0, abs(angle_error) / (self.ANGLE_TOLERANCE_RAD * 3))
        linear = self._pid_distance.compute(0.0, -distance, dt)
        linear = max(0, linear)  # 前进（不倒车）
        linear *= (1.0 - angle_ratio)  # 角度越大速度越低
        angular = self._pid_angle.compute(angle_error, 0.0, dt)

        # 限幅
        linear = max(0.0, min(self._max_v, linear))
        angular = max(-self._max_w, min(self._max_w, angular))

        self._current_command = VelocityCommand(
            linear=linear, angular=angular, timestamp=time.time()
        )
        return self._current_command

    def track_path(self,
                   path: list,
                   current_pose: Tuple[float, float, float],
                   lookahead_idx: int = 2,
                   dt: float = 0.02) -> VelocityCommand:
        """
        跟踪路径（纯追踪）。

        选取路径上 lookahead_idx 步后的点作为子目标。
        """
        if not path:
            return VelocityCommand(linear=0.0, angular=0.0, timestamp=time.time())

        target_idx = min(lookahead_idx, len(path) - 1)
        target = path[target_idx]

        return self.compute_velocity(target, current_pose, dt)

    # ---- 越障模式 ----

    def enter_bump_mode(self) -> None:
        """进入越障模式"""
        self._is_bump_mode = True
        self._bump_timer = time.time()
        self._bump_count = 0
        logger.info("进入越障模式 — 即将通过减速带")

    def exit_bump_mode(self) -> None:
        """退出越障模式"""
        self._is_bump_mode = False
        logger.info(f"退出越障模式 — 通过了 {self._bump_count} 根减速带")

    def compute_bump_velocity(self, dt: float = 0.02) -> VelocityCommand:
        """越障模式下的速度指令（恒速直线）"""
        elapsed = time.time() - self._bump_timer
        # 每根减速带约 1 秒
        self._bump_count = int(elapsed / self.BUMP_CROSS_TIME_S)

        return VelocityCommand(
            linear=self.BUMP_SPEED_MM_S,
            angular=0.0,
            timestamp=time.time(),
        )

    # ---- 状态查询 ----

    @property
    def is_bump_mode(self) -> bool:
        return self._is_bump_mode

    @property
    def current_command(self) -> VelocityCommand:
        return self._current_command

    def is_at_target(self, target: Tuple[float, float],
                     current_pose: Tuple[float, float, float]) -> bool:
        """判断是否已到达目标"""
        cx, cy, ctheta = current_pose
        dx = target[0] - cx
        dy = target[1] - cy
        return math.sqrt(dx * dx + dy * dy) < self.POSITION_TOLERANCE_MM

    def is_stopped(self, velocity: Tuple[float, float]) -> bool:
        """判断是否停止"""
        return abs(velocity[0]) < self.VELOCITY_ZERO_THRESHOLD

    def reset(self) -> None:
        self._pid_distance.reset()
        self._pid_angle.reset()
        self._is_bump_mode = False
        self._bump_count = 0

    # ---- 工具 ----

    @staticmethod
    def _normalize_angle(theta: float) -> float:
        while theta > math.pi:
            theta -= 2 * math.pi
        while theta < -math.pi:
            theta += 2 * math.pi
        return theta

    # ---- 高级接口：设置 PID 参数 ----

    def set_pid_distance(self, kp: float, ki: float, kd: float) -> None:
        self._pid_distance.kp = kp
        self._pid_distance.ki = ki
        self._pid_distance.kd = kd
        self._pid_distance.reset()

    def set_pid_angle(self, kp: float, ki: float, kd: float) -> None:
        self._pid_angle.kp = kp
        self._pid_angle.ki = ki
        self._pid_angle.kd = kd
        self._pid_angle.reset()
