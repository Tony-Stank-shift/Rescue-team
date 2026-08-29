"""
sim_2d.py —— 2D 仿真引擎

无外部物理引擎依赖。纯 Python 实现：
  - 场地几何（3000×3000mm 比赛场地）
  - 机器人运动学（差分驱动 + 限速 + 碰撞边界）
  - 简易 AI（最近目标追逐）
  - 硬件状态模拟（电机/IMU/编码器/电池/超声波/通信）
  - 计分规则（推入安全区）

用法:
  sim = Sim2D(seed=42)
  sim.setup_match(target_count=15)
  for _ in range(100):
      state = sim.step()
"""

import math
import random
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Tuple


# ============================================================
# 常量
# ============================================================

FIELD_SIZE_MM = (3000, 3000)
FIELD_SIZE_M = (3.0, 3.0)

# 安全区 (mm)
SAFE_ZONE_RED = (50, 2550, 600, 400)
SAFE_ZONE_BLUE = (2350, 2550, 600, 400)

# 出发区 (mm)
START_ZONES = [
    (0, 0, 300, 300),        # 左下
    (2700, 0, 300, 300),     # 右下
    (2700, 2700, 300, 300),  # 右上
    (0, 2700, 300, 300),     # 左上
]
# 出发区中心
START_CENTERS = {
    1: (150, 150),
    2: (2850, 150),
    3: (2850, 2850),
    4: (150, 2850),
}

# 目标形状与颜色
SHAPES = ["cube", "pyramid", "cuboid", "sphere", "cylinder", "cone_frustum"]
COLORS = {
    "green":      {"hex": "#4caf50", "name": "普通物资", "points": 5},
    "black":      {"hex": "#212121", "name": "核心物资", "points": 10},
    "orange":     {"hex": "#ff9800", "name": "伤员",     "points": 15},
    "light_blue": {"hex": "#81d4fa", "name": "危险品",   "points": -10},
}

# 默认目标配置（形状, 颜色, 点数, 是否危险品）
DEFAULT_TARGET_SPECS = [
    ("cube",     "green",      5,  False),
    ("cube",     "green",      5,  False),
    ("cube",     "green",      5,  False),
    ("cube",     "green",      5,  False),
    ("pyramid",  "black",     10,  False),
    ("pyramid",  "black",     10,  False),
    ("pyramid",  "black",     10,  False),
    ("cuboid",   "orange",    15,  False),
    ("cuboid",   "orange",    15,  False),
    ("cuboid",   "orange",    15,  False),
    ("sphere",   "green",      5,  False),
    ("sphere",   "green",      5,  False),
    ("cylinder", "black",     10,  False),
    ("cylinder", "black",     10,  False),
    ("cone_frustum", "orange",    15,  False),
    ("cube",     "light_blue", -10, True),
    ("cube",     "light_blue", -10, True),
    ("sphere",   "light_blue", -10, True),
    ("pyramid",  "green",      5,  False),
    ("cuboid",   "green",      5,  False),
    ("cylinder", "light_blue", -10, True),
    ("cone_frustum", "black", 10,  False),
    ("sphere",   "black",     10,  False),
    ("pyramid",  "orange",    15,  False),
    ("cuboid",   "black",     10,  False),
]

# 机器人参数
ROBOT_SIZE_MM = (300, 300)
ROBOT_MAX_SPEED_MM_S = 1000.0   # 最大线速度 mm/s
ROBOT_MAX_ANGULAR_RAD_S = 3.0   # 最大角速度 rad/s
ROBOT_WHEEL_BASE_MM = 182.0     # 轮距 mm（mg370 两差速）
ROBOT_WHEEL_DIAMETER_MM = 65.0  # 轮径 mm（驱动轮 φ65）
MOTOR_MAX_RPM = 300             # 电机最大转速（输出轴，12V 1:34 减速后约 294rpm）
MOTOR_REDUCTION = 34.0          # 减速比 1:34
ENCODER_PPR = 11                # 编码器线数（占位，真实值由底盘层负责）

# 物理参数
DECISION_TIMESTEP_S = 0.02      # 决策步长 (50Hz)
BOUNDARY_MARGIN_MM = 50         # 禁区边距


# ============================================================
# 数据类
# ============================================================

@dataclass
class RobotPose:
    """机器人位姿 (m, rad)."""
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0


@dataclass
class TargetInfo:
    """目标物状态."""
    id: int
    x_m: float
    y_m: float
    shape: str
    color: str
    points: int
    is_dangerous: bool = False
    is_delivered: bool = False


@dataclass
class MatchState:
    """比赛状态快照."""
    time_elapsed_s: float = 0.0
    time_remaining_s: float = 180.0
    score: int = 0
    targets_delivered: int = 0
    robot_pose: RobotPose = field(default_factory=RobotPose)
    is_terminal: bool = False
    violations: int = 0
    trip_count: int = 0
    event: str = ""


@dataclass
class HardwareState:
    """硬件实时状态."""
    # 电机 (4个)
    motor_rpm: List[float] = field(default_factory=lambda: [0.0]*4)
    motor_current_ma: List[float] = field(default_factory=lambda: [120.0]*4)
    # IMU
    imu_gyro_rad_s: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    imu_accel_m_s2: Tuple[float, float, float] = (0.0, 0.0, -9.81)
    imu_temp_c: float = 35.0
    # 超声波
    ultrasonic_distance_mm: float = 2000.0
    # 编码器
    encoder_counts: List[int] = field(default_factory=lambda: [0]*4)
    # 电源
    battery_voltage_v: float = 12.2
    battery_current_ma: float = 800.0
    # 舵机
    pusher_position_mm: float = 0.0
    # 通信
    websocket_connected: bool = True
    heartbeat_latency_ms: float = 5.0
    # 里程计
    odom_x_m: float = 0.0
    odom_y_m: float = 0.0
    odom_yaw_rad: float = 0.0


# ============================================================
# Sim2D
# ============================================================

class Sim2D:
    """2D 仿真引擎。"""

    MATCH_DURATION_S = 180.0

    def __init__(self, seed: int = None, start_zone: int = 1):
        self._seed = seed if seed is not None else random.randint(0, 2**31)
        random.seed(self._seed)

        self.start_zone = start_zone
        sx, sy = START_CENTERS.get(start_zone, START_CENTERS[1])
        self._robot_pose = RobotPose(x=sx/1000.0, y=sy/1000.0, yaw=0.0)

        self.targets: List[TargetInfo] = []
        self._time_elapsed = 0.0
        self._score = 0
        self._trip_count = 0
        self._violations = 0
        self._done = False
        self._events: List[str] = []
        self._step_count = 0
        self._trajectory: List[Tuple[float, float]] = []

        # 硬件状态初始化
        self.hw = HardwareState()
        self._odom_drift_x = 0.0
        self._odom_drift_y = 0.0
        self._odom_drift_yaw = 0.0
        self._encoder_accum = [0, 0, 0, 0]

        # 我方安全区
        self._my_safe_zone = SAFE_ZONE_RED  # 默认红队

        # 搬运状态
        self._carried_target: Optional[TargetInfo] = None

    # ---- 生命周期 ----

    def setup_match(self, target_count: int = 15) -> None:
        """初始化比赛场景。"""
        self._time_elapsed = 0.0
        self._score = 0
        self._trip_count = 0
        self._violations = 0
        self._done = False
        self._step_count = 0
        self._events = [f"MATCH_START seed={self._seed}"]
        self._trajectory = []

        sx, sy = START_CENTERS.get(self.start_zone, START_CENTERS[1])
        self._robot_pose = RobotPose(x=sx/1000.0, y=sy/1000.0, yaw=0.0)

        # 生成目标
        count = min(target_count, len(DEFAULT_TARGET_SPECS))
        specs = random.sample(DEFAULT_TARGET_SPECS, count) if count < len(DEFAULT_TARGET_SPECS) else list(DEFAULT_TARGET_SPECS)
        specs = specs[:target_count]

        positions = self._random_positions(len(specs))
        self.targets = []
        for i, ((shape, color, pts, dangerous), (px, py)) in enumerate(zip(specs, positions)):
            self.targets.append(TargetInfo(
                id=i + 1,
                x_m=px, y_m=py,
                shape=shape, color=color,
                points=pts, is_dangerous=dangerous,
            ))

        self._trajectory.append((self._robot_pose.x, self._robot_pose.y))

    def _random_positions(self, count: int) -> List[Tuple[float, float]]:
        """生成避开禁区的位置 (返回 m)."""
        fw, fd = 3.0, 3.0
        avoid = [
            (0.05, 2.55, 0.05 + 0.6, 2.55 + 0.4),     # 红安全区
            (2.35, 2.55, 2.35 + 0.6, 2.55 + 0.4),     # 蓝安全区
            (0, 0, 0.3, 0.3), (fw - 0.3, 0, fw, 0.3),
            (fw - 0.3, fd - 0.3, fw, fd), (0, fd - 0.3, 0.3, fd),
        ]
        result = []
        for _ in range(count * 5):
            if len(result) >= count:
                break
            x = random.uniform(0.15, fw - 0.15)
            y = random.uniform(0.5, fd - 0.5)
            if not any(ax1 <= x <= ax2 and ay1 <= y <= ay2 for ax1, ay1, ax2, ay2 in avoid):
                result.append((x, y))
        return result

    # ---- 主循环 ----

    def step(self) -> MatchState:
        """单步仿真。返回比赛状态。"""
        if self._done:
            return self._build_state()

        # 简易 AI
        self._simple_ai()

        # 物理积分
        dt = DECISION_TIMESTEP_S
        self._robot_pose.x += self._cmd_vx * dt
        self._robot_pose.y += self._cmd_vy * dt
        self._robot_pose.yaw += self._cmd_w * dt
        self._robot_pose.yaw = math.atan2(
            math.sin(self._robot_pose.yaw), math.cos(self._robot_pose.yaw))

        # 边界约束
        margin_m = BOUNDARY_MARGIN_MM / 1000.0
        fw, fd = FIELD_SIZE_M
        self._robot_pose.x = max(margin_m, min(fw - margin_m, self._robot_pose.x))
        self._robot_pose.y = max(margin_m, min(fd - margin_m, self._robot_pose.y))

        # 检查送达
        self._check_deliveries()

        # 时间
        self._time_elapsed += dt
        self._step_count += 1

        # 更新硬件状态
        self._update_hardware_state()

        # 轨迹
        if self._step_count % 5 == 0:
            self._trajectory.append((self._robot_pose.x, self._robot_pose.y))

        # 结束条件
        if self._time_elapsed >= self.MATCH_DURATION_S:
            self._done = True
            self._events.append("MATCH_TIME_UP")
        elif all(t.is_delivered for t in self.targets):
            self._done = True
            self._events.append("ALL_DELIVERED")

        return self._build_state()

    # ---- 内部: AI ----

    def _simple_ai(self) -> None:
        """搬运 AI：取目标 → 送安全区 → 重复。"""
        self._cmd_vx = 0.0
        self._cmd_vy = 0.0
        self._cmd_w = 0.0

        rx, ry, ryaw = self._robot_pose.x, self._robot_pose.y, self._robot_pose.yaw
        max_v = ROBOT_MAX_SPEED_MM_S / 1000.0
        max_w = ROBOT_MAX_ANGULAR_RAD_S

        if self._carried_target is not None:
            # 已在搬运 → 开往安全区
            sz = self._my_safe_zone
            drop_x = (sz[0] + sz[2] / 2) / 1000.0  # 安全区中心
            drop_y = (sz[1] + sz[3] / 2) / 1000.0
            dx = drop_x - rx
            dy = drop_y - ry
            dist = math.sqrt(dx*dx + dy*dy)
            target_yaw = math.atan2(dy, dx)
        else:
            # 空载 → 找最近未送达目标
            available = [t for t in self.targets if not t.is_delivered]
            if not available:
                return
            closest = min(available, key=lambda t: (t.x_m - rx)**2 + (t.y_m - ry)**2)
            dx = closest.x_m - rx
            dy = closest.y_m - ry
            dist = math.sqrt(dx*dx + dy*dy)
            target_yaw = math.atan2(dy, dx)

        yaw_err = math.atan2(math.sin(target_yaw - ryaw), math.cos(target_yaw - ryaw))

        if abs(yaw_err) > 0.2:
            w = max(-max_w, min(max_w, yaw_err * 4.0))
            v = 0.0
        elif dist > 0.06:
            v = min(0.5, dist * 3.0)
            w = max(-max_w, min(max_w, yaw_err * 3.0))
        else:
            v = 0.15  # 慢推/靠泊
            w = 0.0

        self._cmd_vx = v * math.cos(ryaw)
        self._cmd_vy = v * math.sin(ryaw)
        self._cmd_w = w

    # ---- 内部: 计分 ----

    def _check_deliveries(self) -> None:
        """检查目标拾取与送达。"""
        rx, ry = self._robot_pose.x, self._robot_pose.y
        PICKUP_RANGE_M = 0.15  # 15cm 拾取范围

        # 安全区边界 (m)
        sx, sy, sw, sh = self._my_safe_zone
        sx_m, sy_m = sx / 1000.0, sy / 1000.0
        sw_m, sh_m = sw / 1000.0, sh / 1000.0
        in_safe_zone = (sx_m <= rx <= sx_m + sw_m and sy_m <= ry <= sy_m + sh_m)

        # 送达：机器人在安全区内 + 携带目标 → 放下
        if in_safe_zone and self._carried_target is not None:
            t = self._carried_target
            t.is_delivered = True
            if t.is_dangerous:
                self._score += t.points
                self._violations += 1
            else:
                self._score += t.points
            self._events.append(
                f"DELIVERED target#{t.id} ({t.color} {t.shape}) +{t.points}pts"
            )
            self._trip_count += 1
            self._carried_target = None

        # 拾取：空载 + 接近目标 → 抓起
        if self._carried_target is None:
            for t in self.targets:
                if t.is_delivered:
                    continue
                d = math.sqrt((t.x_m - rx)**2 + (t.y_m - ry)**2)
                if d < PICKUP_RANGE_M:
                    self._carried_target = t
                    self._events.append(
                        f"PICKUP target#{t.id} ({t.color} {t.shape})"
                    )
                    break

    # ---- 内部: 硬件状态 ----

    def _update_hardware_state(self) -> None:
        """根据机器人运动生成模拟硬件状态。"""
        v_linear = math.sqrt(self._cmd_vx**2 + self._cmd_vy**2)
        v_angular = self._cmd_w

        # 轮速 → 电机 RPM
        wheel_radius_m = ROBOT_WHEEL_DIAMETER_MM / 2000.0  # m
        half_wb_m = ROBOT_WHEEL_BASE_MM / 2000.0  # m
        v_left = v_linear - v_angular * half_wb_m
        v_right = v_linear + v_angular * half_wb_m
        rpm_left = (v_left / (2*math.pi*wheel_radius_m)) * 60.0 * MOTOR_REDUCTION
        rpm_right = (v_right / (2*math.pi*wheel_radius_m)) * 60.0 * MOTOR_REDUCTION

        self.hw.motor_rpm = [
            rpm_left + random.gauss(0, 2),
            rpm_right + random.gauss(0, 2),
            rpm_left + random.gauss(0, 3),
            rpm_right + random.gauss(0, 3),
        ]
        # 电流 (空载 100mA + 负载比例)
        load_factor = abs(v_linear) / (ROBOT_MAX_SPEED_MM_S / 1000.0)
        self.hw.motor_current_ma = [
            100 + load_factor * random.uniform(400, 800) for _ in range(4)
        ]

        # IMU
        self.hw.imu_gyro_rad_s = (
            random.gauss(0, 0.005),
            random.gauss(0, 0.005),
            v_angular + random.gauss(0, 0.01),
        )
        self.hw.imu_accel_m_s2 = (
            random.gauss(0, 0.02),
            random.gauss(0, 0.02),
            -9.81 + random.gauss(0, 0.03),
        )

        # 编码器累计
        pulses_per_mm = (ENCODER_PPR * MOTOR_REDUCTION) / (math.pi * ROBOT_WHEEL_DIAMETER_MM)
        self._encoder_accum[0] += int(v_left * 1000 * DECISION_TIMESTEP_S * pulses_per_mm)
        self._encoder_accum[1] += int(v_right * 1000 * DECISION_TIMESTEP_S * pulses_per_mm)
        self._encoder_accum[2] += int(v_left * 1000 * DECISION_TIMESTEP_S * pulses_per_mm)
        self._encoder_accum[3] += int(v_right * 1000 * DECISION_TIMESTEP_S * pulses_per_mm)
        self.hw.encoder_counts = list(self._encoder_accum)

        # 电池
        self.hw.battery_voltage_v = 12.6 - self._time_elapsed * 0.005 + random.gauss(0, 0.02)
        self.hw.battery_current_ma = 500 + load_factor * random.uniform(500, 2000)

        # 推板 (携带时伸出)
        self.hw.pusher_position_mm = 50.0 if self._carried_target is not None else 0.0

        # 超声波
        nearest_dist = 3000.0
        for t in self.targets:
            if t.is_delivered:
                continue
            if t is self._carried_target:
                continue  # 已抓起的不算障碍
            d = math.sqrt((t.x_m - self._robot_pose.x)**2 + (t.y_m - self._robot_pose.y)**2)
            nearest_dist = min(nearest_dist, d)
        self.hw.ultrasonic_distance_mm = nearest_dist * 1000 + random.gauss(0, 3)

        # 里程计 (含漂移)
        self._odom_drift_x += random.gauss(0, 0.0001)
        self._odom_drift_y += random.gauss(0, 0.0001)
        self._odom_drift_yaw += random.gauss(0, 0.00005)
        self.hw.odom_x_m = self._robot_pose.x + self._odom_drift_x
        self.hw.odom_y_m = self._robot_pose.y + self._odom_drift_y
        self.hw.odom_yaw_rad = self._robot_pose.yaw + self._odom_drift_yaw

        # 通信
        self.hw.heartbeat_latency_ms = max(1, 5 + random.gauss(0, 2))

    # ---- 查询 ----

    def _build_state(self) -> MatchState:
        delivered = sum(1 for t in self.targets if t.is_delivered)
        return MatchState(
            time_elapsed_s=round(self._time_elapsed, 2),
            time_remaining_s=round(max(0, self.MATCH_DURATION_S - self._time_elapsed), 2),
            score=self._score,
            targets_delivered=delivered,
            robot_pose=RobotPose(
                x=round(self._robot_pose.x, 4),
                y=round(self._robot_pose.y, 4),
                yaw=round(self._robot_pose.yaw, 4),
            ),
            is_terminal=self._done,
            violations=self._violations,
            trip_count=self._trip_count,
        )

    @property
    def trajectory(self) -> List[Tuple[float, float]]:
        return list(self._trajectory)

    @property
    def events(self) -> List[str]:
        return list(self._events)


# ============================================================
# 独立测试
# ============================================================

if __name__ == "__main__":
    print("=" * 50)
    print("  Sim2D 独立测试")
    print("=" * 50)

    sim = Sim2D(seed=42)
    sim.setup_match(target_count=10)
    assert len(sim.targets) == 10, f"应为 10 目标: {len(sim.targets)}"
    print(f"  初始化: {len(sim.targets)} 目标, seed={sim._seed}")

    # 运行 steps
    for i in range(50):
        state = sim.step()
    print(f"  50 步后: score={state.score}, pos=({state.robot_pose.x:.2f}, {state.robot_pose.y:.2f})")
    print(f"  电机: {[round(r) for r in sim.hw.motor_rpm]} RPM")
    print(f"  电池: {sim.hw.battery_voltage_v:.2f}V")
    print(f"  超声波: {sim.hw.ultrasonic_distance_mm:.0f}mm")
    assert len(sim.trajectory) > 0
    print("  ✅ 通过")

    # 全场比赛
    sim2 = Sim2D(seed=99)
    sim2.setup_match(target_count=5)
    for _ in range(int(180.0 / DECISION_TIMESTEP_S)):
        s = sim2.step()
        if s.is_terminal:
            break
    print(f"\n  全场比赛: score={s.score}, delivered={s.targets_delivered}, "
          f"time={s.time_elapsed_s:.1f}s")
    assert s.is_terminal or s.time_elapsed_s > 179

    print(f"\n{'=' * 50}")
    print("  Sim2D — 全部测试通过 ✅")
    print(f"{'=' * 50}")
