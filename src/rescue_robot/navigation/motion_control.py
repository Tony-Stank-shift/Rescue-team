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

    # ── 2026-09-17 真机新增：解决"只在原地转、永远差最后 10cm" ──
    #: 超过这个角度误差才允许"原地转正"（不前进）。低于它一律保留前进分量
    #: （走弧线靠近），避免 `linear` 被角度项压成 0 而卡死。
    PIVOT_ONLY_RAD = math.radians(90.0)
    #: 弧线靠近时前进速度的最小比例（相对 PID 输出）。
    MIN_APPROACH_RATIO = 0.25
    #: 近距爬行段：距离小于此值时不再要求"先对准"（近距方位角不可靠）。
    CRAWL_RANGE_MM = 150.0
    #: 爬行速度上限 / 下限（mm/s）与比例增益（1/s）：v = clamp(gain·distance, min, max)。
    CRAWL_SPEED_MM_S = 120.0
    CRAWL_MIN_MM_S = 30.0
    CRAWL_GAIN = 1.0
    VELOCITY_ZERO_THRESHOLD = 10.0      # 静止判定速度 (mm/s)

    # 越障参数
    BUMP_SPEED_MM_S = 200.0             # 越障速度
    BUMP_CROSS_TIME_S = 1.0             # 单根减速带通过时间（仅诊断用）
    #: 越障需要实际驶过的距离（mm）。3 条减速带 + 间距 + 车长 ≈ 450mm。
    #: ⚠️ 退出条件必须按**里程**算，不能按墙钟（T0-4）：主循环被套取/投放阻塞时
    #: 墙钟仍在走 → 会在**没跨过减速带**时就退出；反之卡在减速带上时会永远重入。
    BUMP_TRAVEL_MM = 450.0
    #: 越障最长持续时间（秒）——兜底，防止里程因堵转不增长而永远停在 bump 模式。
    BUMP_MAX_S = 8.0
    #: 越障期间的角速度限幅（rad/s）：低速纠偏可以，但不许原地打转蹭减速带。
    BUMP_MAX_ANGULAR_RAD_S = 0.6

    # 限速 / 轮距（可由 YAML robot.motors.* 覆盖，config.apply_robot_config 注入）
    DEFAULT_MAX_LINEAR_SPEED = 850.0
    DEFAULT_MAX_ANGULAR_SPEED = 3.0
    DEFAULT_WHEEL_BASE_MM = 209.0

    def __init__(self,
                 max_linear_speed: Optional[float] = None,
                 max_angular_speed: Optional[float] = None,
                 wheel_base_mm: Optional[float] = None):
        # None → 用类级默认值（可被 YAML 注入覆盖）
        max_linear_speed = (self.DEFAULT_MAX_LINEAR_SPEED
                            if max_linear_speed is None else max_linear_speed)
        max_angular_speed = (self.DEFAULT_MAX_ANGULAR_SPEED
                             if max_angular_speed is None else max_angular_speed)
        wheel_base_mm = (self.DEFAULT_WHEEL_BASE_MM
                         if wheel_base_mm is None else wheel_base_mm)
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
        self._bump_travel_mm = 0.0      # 越障期间累计里程（按它判退出，T0-4）

        logger.info(f"MotionController 初始化: max_v={max_linear_speed}mm/s, "
                     f"max_w={max_angular_speed}rad/s")

    # ---- 主控制接口 ----

    def compute_velocity(self,
                         target: Tuple[float, float],
                         current_pose: Tuple[float, float, float],
                         dt: float = 0.02,
                         align_heading: Optional[float] = None) -> VelocityCommand:
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
            # T1-4：到位时若要求指定朝向（投放对准），**不能直接零速收工** ——
            # 旧实现无条件返回 (0,0)，于是"到达"可以在朝向完全不对时成立，
            # 而投放落点 = 车心 + L·朝向 → 朝向错则落点错（投歪 -10 分/个）。
            # 现在：位置到了但朝向没对 → 原地只给角速度，对准了再算到达。
            if align_heading is not None:
                herr = self._normalize_angle(align_heading - ctheta)
                if abs(herr) > self.ANGLE_TOLERANCE_RAD:
                    ang = self._pid_angle.compute(herr, 0.0, dt)
                    ang = max(-self._max_w, min(self._max_w, ang))
                    self._current_command = VelocityCommand(
                        linear=0.0, angular=ang, timestamp=time.time())
                    return self._current_command
            self._pid_angle.reset()
            return VelocityCommand(linear=0.0, angular=0.0, timestamp=time.time())

        # ── 近距爬行段（普通到点，align_heading=None）──────────────────────
        # ⚠️ 2026-09-17 真机踩到：40~150mm 这段原本也走"先对准再前进"，但**近距时
        #    方位角在数学上就不稳定**：目标位置抖动 δ 时方位角抖动 ≈ atan(δ/距离)，
        #    δ=50mm、距离=100mm → 抖动 27° > 25.8° 阈值 ⇒ linear 恒为 0，
        #    **最后 10cm 永远进不去**。而套住物体恰恰必须走完这 10cm。
        #    现在：近距不再要求朝向，只要求接近（带角度阻尼慢速爬行），
        #    "到位"仍由 POSITION_TOLERANCE_MM 判。罩住物体靠位置，不靠车头角度。
        #    注意 align_heading 非空（投放对准）时不走这条 —— 投放落点依赖朝向。
        if align_heading is None and distance < self.CRAWL_RANGE_MM:
            v = min(self.CRAWL_SPEED_MM_S, max(self.CRAWL_MIN_MM_S,
                                              self.CRAWL_GAIN * distance))
            ang = self._pid_angle.compute(angle_error, 0.0, dt)
            # 近距把角速度限一半：避免贴着物体时甩头把它蹭飞
            ang = max(-self._max_w * 0.5, min(self._max_w * 0.5, ang))
            self._current_command = VelocityCommand(
                linear=v, angular=ang, timestamp=time.time())
            return self._current_command

        # 连续过渡：角度误差大时减速+转向，小时全速前进
        angle_ratio = min(1.0, abs(angle_error) / (self.ANGLE_TOLERANCE_RAD * 3))
        linear = self._pid_distance.compute(0.0, -distance, dt)
        linear = max(0, linear)
        # ⚠️ 2026-09-17 真机踩到：`linear *= (1 - angle_ratio)` 在角度误差 ≥ 25.8°
        #    时把前进速度**压成 0** → 车只原地转、位置不变 → 位置看门狗 10s 后判"卡死"
        #    并抢走导航目标 → 车被支使去别处，物体出视野。现场现象就是
        #    "识别到物体却不去套，反而开走，然后说目标没了"。
        #    现在分两档：
        #      · 角度误差 ≤ PIVOT_ONLY_RAD（90°）：保留速度下限，**走弧线**靠近；
        #      · 角度误差 > 90°：朝向差太多，先原地转正再走（此时不前进是合理的）；
        #      · 需要指定朝向的场合（投放对准，align_heading 非空）保持旧行为。
        if align_heading is not None or abs(angle_error) > self.PIVOT_ONLY_RAD:
            linear *= (1.0 - angle_ratio)
        else:
            linear *= max(self.MIN_APPROACH_RATIO, 1.0 - angle_ratio)
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
                   dt: float = 0.02,
                   align_heading: Optional[float] = None) -> VelocityCommand:
        """
        跟踪路径（纯追踪）。

        选取路径上 lookahead_idx 步后的点作为子目标。
        """
        if not path:
            return VelocityCommand(linear=0.0, angular=0.0, timestamp=time.time())

        target_idx = min(lookahead_idx, len(path) - 1)
        target = path[target_idx]

        return self.compute_velocity(target, current_pose, dt,
                                     align_heading=align_heading)

    # ---- 越障模式 ----

    def enter_bump_mode(self) -> None:
        """进入越障模式"""
        self._is_bump_mode = True
        self._bump_timer = time.time()
        self._bump_count = 0
        self._bump_travel_mm = 0.0
        logger.info("进入越障模式 — 即将通过减速带")

    def exit_bump_mode(self) -> None:
        """退出越障模式"""
        self._is_bump_mode = False
        logger.info(f"退出越障模式 — 越过 {self._bump_travel_mm:.0f}mm "
                    f"（折算约 {self._bump_count} 根减速带）")

    @property
    def bump_finished(self) -> bool:
        """本次越障是否已完成：按**里程**判，另有最长时长兜底（T0-4）。

        ⚠️ 旧实现用 `int((now - _bump_timer)/1.0) >= 3`（墙钟）：主循环被套取/投放
        阻塞时墙钟照走 → 没跨过就退出；而 `enter_bump_mode()` 又把计时清零，
        于是只要还在减速带附近就**反复重入**，每轮直线冲 600mm 且 `angular=0` 不看目标。
        """
        if self._bump_travel_mm >= self.BUMP_TRAVEL_MM:
            return True
        return (time.time() - self._bump_timer) > self.BUMP_MAX_S

    def compute_bump_velocity(self, dt: float = 0.02,
                              angular: float = 0.0) -> VelocityCommand:
        """越障模式下的速度指令（恒速前进 + 有限纠偏）。

        Args:
            angular: 由导航给出的朝向修正（rad/s），会被限幅到
                ``BUMP_MAX_ANGULAR_RAD_S``。旧实现恒为 0 → 越障全程**无视目标方向**，
                车头朝哪就朝哪冲（可能撞围栏/直入对方安全区）。
        """
        self._bump_travel_mm += abs(self.BUMP_SPEED_MM_S) * dt
        # 折算"越过了几根"（每根 110mm = 深 60 + 间 50）——仅用于日志/诊断
        self._bump_count = int(self._bump_travel_mm / 110.0)
        ang = max(-self.BUMP_MAX_ANGULAR_RAD_S,
                  min(self.BUMP_MAX_ANGULAR_RAD_S, float(angular)))

        return VelocityCommand(
            linear=self.BUMP_SPEED_MM_S,
            angular=ang,
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
        self._bump_travel_mm = 0.0

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
