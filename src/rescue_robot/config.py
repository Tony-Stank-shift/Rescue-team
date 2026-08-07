"""
config.py —— 全局配置常量

所有可调参数集中管理，方便现场快速修改。
"""

from dataclasses import dataclass
from enum import IntEnum


# ============================================================
# GPIO 引脚定义（树莓派 BCM 编号，Mock 模式下忽略）
# ============================================================
class Pin(IntEnum):
    """GPIO 引脚分配"""
    BUTTON_START = 17       # 一键启动按钮（物理按钮）
    LED_GREEN = 22          # 绿色 LED（状态指示）
    LED_RED = 27            # 红色 LED（错误指示）
    BUZZER = 18             # 蜂鸣器（可选）


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
    MOTOR_MIN_CURRENT_MA = 50       # 电机空载最小电流
    MOTOR_MAX_CURRENT_MA = 5000     # 电机堵转最大电流
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
