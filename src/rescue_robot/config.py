"""
config.py —— 全局配置常量

所有可调参数集中管理，方便现场快速修改。
"""

from dataclasses import dataclass
from enum import IntEnum


# ============================================================
# GPIO 引脚定义（主控 RDK，非树莓派；Mock 模式下忽略）
# ============================================================
class Pin(IntEnum):
    """
    下位机（STM32F103C8Tx）引脚参考 —— 上位机经串口控制，不直接 GPIO。

    按钮/状态灯/舵机/IMU 都在下位机（来自同学引脚图/原理图，用户确认）：
      - 一键启动  START_BTN       = PB9  → 下位机发 BUTTON,ON 给上位机
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
