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
    BUZZER = 3              # ⚠️ 本车未装蜂鸣器（2026-09-16 现场确认）；编号保留以免改动引脚映射，勿接线
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
    机构几何标定参数 —— **夹爪 V2**（2026-09-16 机构变更后）。

    ── 机构实物（实测自 SolidWorks 装配体 `装配体3 - 夹爪-2-2.STL`，2600 面）──
    套取方式**不变**：自上而下套住目标；舵机行程**不变**（套住 0° / 释放 70°，
    对齐下位机 `servo.h`）。**旧描述中的"U 型槽"已不存在**，本类及相关注释
    已按新机构改写。

      ① 套取开口：**150mm(横向) × 100mm(前后)** 的方形框（框高 60.7mm）。
         见 ``SLEEVE_OPENING_MM``。物理含义：目标必须落在
         **横向 ±75mm、前后 ±50mm** 的矩形内，套下去才真的套得住。

      ② 框后方三块**平行于地面的水平实心板**，自上而下依次变小：
           上板 157 × 40mm（最高、最大）
           中板 150 × 35mm
           下板 150 × 20mm（最低、最小）
         三块板用于**在斜坡上推目标**，按目标类型分工（机构是被动的，
         哪块板受力由目标高度决定，软件无需选择）：
           上板 + 中板 → 推 **橙色长方体（伤员）** 与 **绿色正方体（普通物资）**
           下板 + 中板 → 推 **黑色三棱锥（核心物资）**
         → 旧代码里"舵机带着单块后方板渐进上调"的那套动作，其**升降来源
           已由舵机改为这三块固定阶梯板**；舵机在推入过程中只需保持套住态，
           推入结束再一次性释放。见 ``transport_pipeline`` 的 ``PLACING`` 相。

    ── 标定参数 ──
    SLEEVE_OPENING_MM：套取开口 (横向, 前后) mm。**捕获判据必须用它**——
      旧代码只用 ``CAPTURE_RADIUS_MM`` 一个圆（150mm），与 150×100 的**矩形**
      开口不匹配：目标在圆内但在开口外时，软件会记成"已套住"，实车却套空
      （与 S-40 同族的"幽灵捕获"）。现已按矩形开口复核。
    DROP_FORWARD_MM：释放瞬间，目标（在套取框内）相对**车心**的前伸距离。
      投放有效性判定必须用它把"车身位置"换算成"目标落点"：
        drop_point = (x + L·cosθ, y + L·sinθ)
      误差直接决定"有效/无效投放"，也就决定首趟成败。
      ⚠️ 必须真机标定：把目标放进框里量前伸距离，再改 YAML。
    PUSH_DIST_MM：推式放置时朝斜坡方向的推入距离（同样需真机标定）。

    SLEEVE_MAX_HOLD：一趟最多能**真正套住**几个目标 = 套取机构的物理容量。
      默认 1（**机构组已确认夹爪 V2 仍为 1**）：单只 SG90 驱动，"套住(0°) /
      释放(70°)"只有一个自由度，且每次下压前都必须先抬爪（抬爪 = 释放）→
      先前套住的目标会被放掉。因此"一趟带 3 个"在物理上不成立。
      **必须默认 1**，否则软件会把根本没带上的目标也算作已送达（虚高得分；
      首趟"必须且仅送 1 个"还会假成功）。
      ⚠️ 仅当机构组确认"框内可同时容纳多个且行进中不脱落"时才可调大；
         调大后必须先回归集成仿真（决策引擎 grip_done 耦合需同步调整）。
    """
    # N-6：150mm 太大 —— 红方物资区 y 向只有 300mm（完全置入可用 280mm），
    # L=150 已占 54%，落点被推到围栏上 → 首趟有效投放接近抛硬币。
    # 实测（车心停在区域中心、L=150）：9 个朝向里 4 个判 ON_FENCE/超界；
    # L=70 且落点投影钳回子区域后，**所有朝向都有效**。仍是真机标定项。
    DROP_FORWARD_MM: float = 70.0
    PUSH_DIST_MM: float = 100.0
    SLEEVE_MAX_HOLD: int = 1

    # 夹爪 V2 套取开口 (横向, 前后) mm —— 实测 STL：X 63.5..213.5=150.0,
    # Z 1255.3..1355.3=100.0。捕获判据用它做矩形复核，避免"套空却记账"。
    SLEEVE_OPENING_MM: tuple = (150.0, 100.0)

    # ── 套取接近闸门（夹爪 V2 必须重标）──
    # 旧值 150.0 是按旧机构定的：150 的**圆**与 V2 的 150×100 **矩形**开口不匹配
    # （半宽 75 / 半深 50），且开口中心约在车心前方 DROP_FORWARD_MM=70 处 →
    # 目标与车心的前后距离必须落在 **[70-50, 70+50] = [20, 120]mm** 才在开口正下方。
    # **150 落在该区间之外 → 真机必然套空**（软件却会记账成功 = 幽灵捕获）。
    # 取 100：位于合法区间内、距远端边界留 20mm 余量，又不苛求导航精度到 70。
    # ⚠️ 真机标定：若导航能把目标稳定送到开口中心，可下调到 70（余量最大）。
    # 仿真对该值在 80~150 区间**不敏感**（实测四种取值结果逐位一致），故改动不影响回归。
    CAPTURE_RADIUS_MM: float = 100.0

    # ── 投放时的分步上调次数 ──
    # 4 = 保留旧行为（推入过程中舵机 0°→70° 分 4 步渐进抬）。
    # 0  = 推入全程保持 0°（套住），到位后一次性释放。
    # 夹爪 V2 的**推升来源已改为三块固定阶梯板**，故旧"舵机带板渐进上调"不再是
    # 升力来源；0 还是 4 哪个更稳属真机标定项（见 docs/GRIPPER_V2_GEOMETRY.md）。
    PROGRESSIVE_RAISE_STEPS: int = 4


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

    # ── 套取视觉确认（本车无硬件"套住检测"，用摄像头看套取框里是否有目标）──
    # 套取框（夹爪 V2：150×100 方形开口）在图像中的区域，**归一化**坐标
    # (x1, y1, x2, y2)，相对图像宽高 ∈ [0,1]。
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
        # 夹爪 V2 套取开口 [横向, 前后] mm —— 现场若换了框（150×100 为当前值）在此改
        _open = getattr(pl, "sleeve_opening_mm", None)
        if _open is not None:
            try:
                _ow, _od = float(_open[0]), float(_open[1])
                if _ow > 0 and _od > 0:
                    Placement.SLEEVE_OPENING_MM = (_ow, _od)
                else:
                    raise ValueError("must be positive")
            except (TypeError, ValueError, IndexError):
                import logging as _logging
                _logging.getLogger(__name__).warning(
                    f"placement.sleeve_opening_mm 非法（应为 [横向, 前后] 两个正数）"
                    f"→ 沿用 {Placement.SLEEVE_OPENING_MM}")
        # 套取接近闸门（夹爪 V2 关键标定项，见 SLEEVE_OPENING_MM 上方推导）
        _cap = getattr(pl, "capture_radius_mm", None)
        if _cap is not None:
            try:
                _capv = float(_cap)
                if _capv > 0:
                    Placement.CAPTURE_RADIUS_MM = _capv
                else:
                    raise ValueError("must be positive")
            except (TypeError, ValueError):
                import logging as _logging
                _logging.getLogger(__name__).warning(
                    f"placement.capture_radius_mm 非法 → 沿用 "
                    f"{Placement.CAPTURE_RADIUS_MM}")
        # 投放分步上调次数（0 = 保持套住到到位后一次性释放）
        _steps = getattr(pl, "progressive_raise_steps", None)
        if _steps is not None:
            try:
                Placement.PROGRESSIVE_RAISE_STEPS = int(_steps)
            except (TypeError, ValueError):
                import logging as _logging
                _logging.getLogger(__name__).warning(
                    f"placement.progressive_raise_steps 非法 → 沿用 "
                    f"{Placement.PROGRESSIVE_RAISE_STEPS}")
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
        # 开场退避（现场摆位需要）：进 AUTONOMOUS 后先直线退一段再开始自主。
        # 负值 = 后退，0 = 关闭。见 AutonomousState._run_startup_backup。
        from .states.autonomous_state import AutonomousState
        AutonomousState.STARTUP_BACKUP_MM_S = float(
            getattr(m, "startup_backup_mm_s", 0.0) or 0.0)
        AutonomousState.STARTUP_BACKUP_S = float(
            getattr(m, "startup_backup_s", 5.0) or 5.0)

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
        # T2-3：局部避障器必须用**同一个**限速源，否则 YAML 限速被它旁路
        from .navigation.path_planner import LocalPlanner
        LocalPlanner.DEFAULT_MAX_LINEAR_SPEED = float(mot.max_speed_mm_s)
        MotionController.DEFAULT_MAX_ANGULAR_SPEED = float(mot.max_angular_speed_rad_s)
        LocalPlanner.DEFAULT_MAX_ANGULAR_SPEED = float(mot.max_angular_speed_rad_s)
        MotionController.DEFAULT_WHEEL_BASE_MM = float(mot.wheel_base_mm)

    # ── 策略权重：接到目标选择器（T2-14，原来 YAML 里写了也不生效）──
    #    注入方式与上面 `TargetSelector.TIME_PRESSURE_S` 一致（类属性）。
    #    ⚠️ 映射保证"默认值下与旧公式逐位等价"，详见 TargetSelector 的类文档字符串。
    sw = getattr(cfg, "strategy_weights", None)
    if sw is not None:
        from .decision.target_selector import TargetSelector
        TargetSelector.set_weights(
            distance_weight=getattr(sw, "distance_weight", None),
            points_weight=getattr(sw, "points_weight", None),
            time_weight=getattr(sw, "time_weight", None),
            opponent_factor=getattr(sw, "opponent_factor", None),
        )

    logger.info(
        "已应用 YAML 配置: match.duration=%ss, time_pressure=%ss, "
        "watchdog=%s/%ss, placement.drop_forward=%smm, penalty=%s分/个",
        DecisionEngine.MATCH_DURATION_S, DecisionEngine.TIME_PRESSURE_S,
        AnomalyHandler.WATCHDOG_WARN_S, AnomalyHandler.WATCHDOG_TIMEOUT_S,
        Placement.DROP_FORWARD_MM, Thresholds.PLACEMENT_PENALTY_PER_TARGET,
    )
    if sw is not None:
        logger.info("策略权重已接线到 TargetSelector: %s",
                    TargetSelector.get_weights())
