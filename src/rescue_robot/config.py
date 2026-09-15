"""
config.py —— 全局配置常量

所有可调参数集中管理，方便现场快速修改。
"""

import logging
from dataclasses import dataclass
from enum import IntEnum

logger = logging.getLogger("config")


# ============================================================
# GPIO 引脚定义（主控 RDK，非树莓派；Mock 模式下忽略）
# ============================================================
class Pin(IntEnum):
    """
    下位机（STM32F103C8Tx）引脚参考 —— 上位机经串口控制，不直接 GPIO。

    按钮/状态灯/舵机/IMU 都在下位机（来自同学引脚图/原理图，用户确认）：
      - 一键启动  START_BTN       = PB9  → 下位机发 EVENT,START_BUTTON 给上位机
      - 状态灯    STATUS_LED      = PC13 → 下位机点亮（协议未含，下位机自行控制）
      - 舵机      SERVO_PWM       = PB6  → 上位机发 SERVO 命令，下位机转 PWM
      - 串口      USART1          = PA9(TX)/PA10(RX) → 上位机 serial_chassis
      - 电机      LPWM=PA8 / RPWM=PA11 / MOTOR_STBY=PB8
      - 编码器    LGMRA=PA0/LGMRB=PA1 / RGMRA=PA6/RGMRB=PA7
      - IMU       MPU6050_INT=PB5 / MPU6050_SCL=PB10 / MPU6050_SDA=PB11 (I2C2)
    上位机(主控 RDK)不经 GPIO 直连上述，逻辑走串口；此处仅记录供参考。
    """
    BUTTON_START = 0        # 下位机 PB9 (START_BTN 一键启动)
    LED_GREEN = 1           # 下位机 PC13 (STATUS_LED)
    LED_RED = 2             # 预留
    BUZZER = 3              # 预留（无）
    SERVO_PWM = 4           # 下位机 PB6 (SERVO_PWM 舵机)
    MPU6050_INT = 5         # 下位机 PB5 (MPU6050_INT)
    MPU6050_SCL = 6         # 下位机 PB10 (MPU6050_SCL, I2C2)
    MPU6050_SDA = 7         # 下位机 PB11 (MPU6050_SDA, I2C2)


# ============================================================
# 时序常量（单位：毫秒）
# ============================================================
@dataclass(frozen=True)
class Timing:
    """所有时序参数"""
    BUTTON_DEBOUNCE_MS = 50         # 按钮防抖时间
    BUTTON_LONG_PRESS_MS = 500      # 长按确认时间（防误触）
    LED_BOOT_BLINK_INTERVAL_MS = 500   # BOOT 状态 LED 闪烁间隔
    LED_AUTO_BLINK_INTERVAL_MS = 200    # AUTONOMOUS 状态 LED 快闪间隔
    SELF_CHECK_TIMEOUT_S = 10       # 自检超时时间
    SENSOR_CHECK_TIMEOUT_MS = 3000  # 单个传感器检查超时
    MOTOR_CHECK_DURATION_MS = 500   # 电机测试转动时长
    POST_START_DELAY_MS = 1000      # 一键启动后延迟（裁判离开时间）


# ============================================================
# 硬件阈值
# ============================================================
@dataclass(frozen=True)
class Thresholds:
    """硬件检查阈值"""
    BATTERY_MIN_VOLTAGE = 11.0      # 最低电池电压（3S 锂电池）
    BATTERY_MAX_VOLTAGE = 12.6      # 最高电池电压
    MOTOR_MIN_CURRENT_MA = 50       # 电机空载最小电流（待定）
    MOTOR_MAX_CURRENT_MA = 5000     # 电机堵转最大电流（待定，应高于正常行驶 3-4A）
    CAMERA_MIN_FPS = 10             # 摄像头最低帧率
    # ❓ 该数值来源不明：赛项 PDF 只写"转运至安全区的无效救援目标将被取出重新
    #    随机放置在场地中央"，**未给出扣分细则**。暂按 10 分/个；现场确认后
    #    改 YAML `robot.placement.penalty_per_target` 即可（无需重编译）。
    PLACEMENT_PENALTY_PER_TARGET = 10


# ============================================================
# 套取/投放机构几何（真机标定项，可由 YAML 覆盖）
# ============================================================
class Placement:
    """
    机构几何标定参数。

    DROP_FORWARD_MM：释放瞬间，目标（在车头 U 型槽内）相对**车心**的前伸距离。
      投放有效性判定必须用它把"车身位置"换算成"目标落点"：
        drop_point = (x + L·cosθ, y + L·sinθ)
      误差直接决定"有效/无效投放"，也就决定首趟成败。
      ⚠️ 必须真机标定：把目标放进槽里量前伸距离，再改 YAML。
    PUSH_DIST_MM：推式放置时朝斜坡方向的推入距离（同样需真机标定）。

    SLEEVE_MAX_HOLD：一趟最多能**真正套住**几个目标 = 套取机构的物理容量。
      默认 1：本车 U 型槽由单只 SG90 驱动，"套住(0°) / 释放(70°)"只有一个
      自由度，且每次下压前都必须先抬爪（抬爪 = 释放）→ 先前套住的目标会被放掉。
      因此"一趟带 3 个"在物理上不成立。**必须默认 1**，否则软件会把根本没带上
      的目标也算作已送达（虚高得分；首趟"必须且仅送 1 个"还会假成功）。
      ⚠️ 仅当机构组确认"槽内可同时容纳多个且行进中不脱落"时才可调大；
         调大后必须先回归集成仿真（决策引擎 grip_done 耦合需同步调整）。
    """
    DROP_FORWARD_MM: float = 150.0
    PUSH_DIST_MM: float = 100.0
    SLEEVE_MAX_HOLD: int = 1


placement = Placement()



# ============================================================
# 运行模式
# ============================================================
class RunMode:
    """运行模式：MOCK 用于本地开发测试，REAL 用于真机"""
    MOCK = "mock"
    REAL = "real"


# ============================================================
# 默认配置实例
# ============================================================
timing = Timing()
thresholds = Thresholds()


# ============================================================
# 摄像头几何（地平面测距：底边 + 相机倾角）
# ============================================================
class Camera:
    """
    相机安装几何 —— 地平面测距的输入参数。

    测距原理（目标贴地时远优于面积法）：
        f  = W / (2·tan(FOV/2))                 焦距(px)
        α  = TILT + atan((y_bottom − H/2) / f)  视线俯角
        d  = HEIGHT / tan(α)                    地面距离(mm)

    ⚠️ TILT_DEG 是真机安装角度，**必须标定**：
       把目标放在已知距离（如 500/1000/1500mm），量其检测框底边像素 y，
       反解出倾角。倾角误差对远处距离影响很大。
    """
    HEIGHT_MM: float = 210.0      # 光心离地高度 (mm)
    TILT_DEG: float = 30.0        # 下倾角（相对水平，向下为正）—— 已确认 30°
    FOV_DEG: float = 77.0         # 视场角（硬件：800W 77°）
    RES: tuple = (640, 480)       # 处理分辨率 (W, H)

    # ── 套取视觉确认（本车无硬件"套住检测"，用摄像头看 U 型槽是否有目标）──
    # U 型槽在图像中的区域，**归一化**坐标 (x1, y1, x2, y2)，相对图像宽高 ∈ [0,1]。
    # ⚠️ 必须真机标定：把目标放进槽里，看槽落在图像哪个区域，再把这里改成实测值。
    SLEEVE_ROI: tuple = (0.32, 0.55, 0.68, 0.98)
    # 是否启用套取视觉确认（ROI 未标定/看不到槽时可先关掉）
    SLEEVE_CONFIRM: bool = True


camera = Camera()

# 模块级别名（方便现场直接改 / getattr 读取）
CAMERA_HEIGHT_MM = Camera.HEIGHT_MM
CAMERA_TILT_DEG = Camera.TILT_DEG
CAMERA_FOV_DEG = Camera.FOV_DEG
CAMERA_RES = Camera.RES
SLEEVE_ROI = Camera.SLEEVE_ROI
SLEEVE_CONFIRM = Camera.SLEEVE_CONFIRM


# ============================================================
# 从 YAML 配置覆盖默认值（决赛创新实践环节现场修改，无需重编译）
# ============================================================
def apply_robot_config(cfg) -> None:
    """
    把 RobotConfig（来自 innovation.config_loader.RobotConfig.from_yaml）
    应用到全局 timing / thresholds 常量。

    使用 duck typing，避免 config 模块反向依赖 config_loader。
    """
    t = cfg.timing
    Timing.BUTTON_DEBOUNCE_MS = t.button_debounce_ms
    Timing.BUTTON_LONG_PRESS_MS = t.button_long_press_ms
    Timing.LED_BOOT_BLINK_INTERVAL_MS = t.led_boot_blink_interval_ms
    Timing.LED_AUTO_BLINK_INTERVAL_MS = t.led_auto_blink_interval_ms
    Timing.SELF_CHECK_TIMEOUT_S = t.self_check_timeout_s
    Timing.SENSOR_CHECK_TIMEOUT_MS = t.sensor_check_timeout_ms
    Timing.MOTOR_CHECK_DURATION_MS = t.motor_check_duration_ms
    Timing.POST_START_DELAY_MS = t.post_start_delay_ms

    th = cfg.thresholds
    Thresholds.BATTERY_MIN_VOLTAGE = th.battery_min_voltage
    Thresholds.BATTERY_MAX_VOLTAGE = th.battery_max_voltage
    Thresholds.MOTOR_MIN_CURRENT_MA = th.motor_min_current_ma
    Thresholds.MOTOR_MAX_CURRENT_MA = th.motor_max_current_ma
    Thresholds.CAMERA_MIN_FPS = th.camera_min_fps

    # ── 机构几何 / 扣分（现场标定项，YAML 改了立刻生效）──
    pl = getattr(cfg, "placement", None)
    if pl is not None:
        Placement.DROP_FORWARD_MM = pl.drop_forward_mm
        Placement.PUSH_DIST_MM = pl.push_dist_mm
        # 机构容量：夹到 [1,3]（规则上限 3 个/趟）
        # ⚠️ >1 目前**未接线完成**，必须挡掉：多目标逐个套取的机构侧已实现，但
        #    决策引擎的 grip_done 契约仍假设"一趟只套 1 个"，容量 >1 时决策会在
        #    套取途中下发 TRANSPORT_TO 把导航目标抢走 → 空耗时间（实测 180s 只送 4 个，
        #    比容量 1 的 7 个还差）。宁可回退到唯一验证过的 1，也不让现场调到坑里。
        try:
            _hold = int(getattr(pl, "sleeve_max_hold", 1))
        except (TypeError, ValueError):
            _hold = 1
        if _hold > 1:
            import logging as _logging
            _logging.getLogger(__name__).error(
                f"placement.sleeve_max_hold={_hold} 暂不支持（需先改决策引擎 "
                f"grip_done 耦合，见 docs/audit/FIXES.md 的 S-40）→ 强制回退 1")
            _hold = 1
        Placement.SLEEVE_MAX_HOLD = max(1, min(3, _hold))
        Thresholds.PLACEMENT_PENALTY_PER_TARGET = pl.penalty_per_target

    # ── 比赛参数 / 超时 / 降级阈值：真正驱动行为（原来只写死在代码常量里）──
    # 延迟导入避免 config ←→ decision/navigation 的循环 import。
    m = getattr(cfg, "match", None)
    if m is not None:
        from .decision.decision_engine import DecisionEngine
        DecisionEngine.MATCH_DURATION_S = float(m.duration_s)
        DecisionEngine.TIME_PRESSURE_S = float(m.time_pressure_s)
        DecisionEngine.NAV_TIMEOUT_S = float(m.nav_timeout_s)
        DecisionEngine.GRIP_TIMEOUT_S = float(m.grip_timeout_s)
        DecisionEngine.TRANSPORT_TIMEOUT_S = float(m.transport_timeout_s)
        # 时间紧迫阈值同时约束目标选择器
        from .decision.target_selector import TargetSelector
        TargetSelector.TIME_PRESSURE_S = float(m.time_pressure_s)

    fb = getattr(cfg, "fallback", None)
    if fb is not None:
        from .decision.anomaly_handler import AnomalyHandler
        AnomalyHandler.WATCHDOG_WARN_S = float(fb.watchdog_warn_s)
        AnomalyHandler.WATCHDOG_CRITICAL_S = float(fb.watchdog_critical_s)
        AnomalyHandler.WATCHDOG_TIMEOUT_S = float(fb.watchdog_timeout_s)
        AnomalyHandler.STUCK_TIME_S = float(fb.stuck_time_s)
        AnomalyHandler.STUCK_DISTANCE_MM = float(fb.stuck_distance_mm)
        from .states.autonomous_state import AutonomousState
        AutonomousState.WATCHDOG_EXPLORE_S = float(fb.watchdog_warn_s)
        AutonomousState.WATCHDOG_SURVIVAL_S = float(fb.watchdog_critical_s)
        AutonomousState.WATCHDOG_HARD_LIMIT_S = float(fb.watchdog_timeout_s)

    # ── 电机 PID / 限速：接到运动控制器（原来 YAML 里写了也不生效）──
    mot = getattr(cfg, "motors", None)
    if mot is not None:
        from .navigation.motion_control import MotionController
        MotionController.PID_DISTANCE = (mot.pid.kp, mot.pid.ki, mot.pid.kd)
        MotionController.PID_ANGLE = (mot.pid_angle.kp, mot.pid_angle.ki,
                                      mot.pid_angle.kd)
        MotionController.DEFAULT_MAX_LINEAR_SPEED = float(mot.max_speed_mm_s)
        MotionController.DEFAULT_MAX_ANGULAR_SPEED = float(mot.max_angular_speed_rad_s)
        MotionController.DEFAULT_WHEEL_BASE_MM = float(mot.wheel_base_mm)

    logger.info(
        "已应用 YAML 配置: match.duration=%ss, time_pressure=%ss, "
        "watchdog=%s/%ss, placement.drop_forward=%smm, penalty=%s分/个",
        DecisionEngine.MATCH_DURATION_S, DecisionEngine.TIME_PRESSURE_S,
        AnomalyHandler.WATCHDOG_WARN_S, AnomalyHandler.WATCHDOG_TIMEOUT_S,
        Placement.DROP_FORWARD_MM, Thresholds.PLACEMENT_PENALTY_PER_TARGET,
    )
