"""
indicator.py —— 状态指示器

通过 LED 和蜂鸣器向外界传达机器人当前状态：
  - BOOT:        绿色 LED 慢闪（500ms 间隔）
  - DEBUG:       绿色 LED 常亮
  - AUTONOMOUS:  绿色 LED 快闪（200ms 间隔）
  - ERROR:       红色 LED 快闪 + 蜂鸣器间歇鸣叫
  - OFF:         所有 LED 关闭
"""

import logging
import threading
import time
from enum import Enum, auto

from ..config import timing

logger = logging.getLogger("indicator")


class IndicatorSignal(Enum):
    """指示灯信号类型"""
    OFF = auto()                    # 全灭
    BOOT_BLINK = auto()             # 绿 LED 慢闪（自检中）
    DEBUG_STEADY = auto()           # 绿 LED 常亮（等待启动）
    AUTONOMOUS_FAST_BLINK = auto()  # 绿 LED 快闪（全自主运行）
    ERROR = auto()                  # 红 LED 快闪 + 蜂鸣器


class AbstractIndicator:
    """指示器抽象基类"""

    def signal(self, signal_type: IndicatorSignal) -> None:
        raise NotImplementedError

    def cleanup(self) -> None:
        pass


# ============================================================
# Mock 指示器 —— 控制台输出
# ============================================================

class MockIndicator(AbstractIndicator):
    """
    Mock 指示器：在控制台输出 LED 状态（用于本地开发/CI）。
    """

    _SIGNAL_LABELS = {
        IndicatorSignal.OFF:                   "⚫ 指示灯: 全灭",
        IndicatorSignal.BOOT_BLINK:            "🔵 指示灯: 绿 LED 慢闪 (自检中…)",
        IndicatorSignal.DEBUG_STEADY:          "🟢 指示灯: 绿 LED 常亮 (等待启动)",
        IndicatorSignal.AUTONOMOUS_FAST_BLINK: "🔴 指示灯: 绿 LED 快闪 (全自主运行!!)",
        IndicatorSignal.ERROR:                 "🚨 指示灯: 红 LED 快闪 + 蜂鸣器 (错误!!)",
    }

    def signal(self, signal_type: IndicatorSignal) -> None:
        label = self._SIGNAL_LABELS.get(signal_type, f"未知信号: {signal_type}")
        logger.info(label)


# ============================================================
# LED 指示器 —— 真实硬件
# ============================================================

class LEDIndicator(AbstractIndicator):
    """
    真实 LED 指示器（树莓派 GPIO）。

    使用单独的线程驱动 LED 闪烁，避免阻塞主状态机。
    """

    def __init__(self, green_pin: int, red_pin: int, buzzer_pin: int | None = None):
        """
        Args:
            green_pin: 绿色 LED BCM 引脚
            red_pin: 红色 LED BCM 引脚
            buzzer_pin: 蜂鸣器 BCM 引脚（可选）
        """
        self._green_pin = green_pin
        self._red_pin = red_pin
        self._buzzer_pin = buzzer_pin

        # 延迟导入
        try:
            import RPi.GPIO as GPIO
            self._GPIO = GPIO
        except ImportError:
            logger.error("RPi.GPIO 未安装！")
            raise

        self._GPIO.setmode(GPIO.BCM)
        self._GPIO.setup(self._green_pin, GPIO.OUT, initial=GPIO.LOW)
        self._GPIO.setup(self._red_pin, GPIO.OUT, initial=GPIO.LOW)
        if self._buzzer_pin is not None:
            self._GPIO.setup(self._buzzer_pin, GPIO.OUT, initial=GPIO.LOW)

        # 闪烁控制
        self._current_signal = IndicatorSignal.OFF
        self._blink_thread: threading.Thread | None = None
        self._stop_event = threading.Event()

    def signal(self, signal_type: IndicatorSignal) -> None:
        """切换 LED 信号模式"""
        if signal_type == self._current_signal:
            return

        logger.info(f"LED 切换: {self._current_signal.name} → {signal_type.name}")
        self._current_signal = signal_type

        # 停止当前闪烁线程
        self._stop_event.set()
        if self._blink_thread and self._blink_thread.is_alive():
            self._blink_thread.join(timeout=1.0)

        # 全灭
        if signal_type == IndicatorSignal.OFF:
            self._set_green(False)
            self._set_red(False)
            self._set_buzzer(False)
            return

        # 常亮模式
        if signal_type == IndicatorSignal.DEBUG_STEADY:
            self._set_red(False)
            self._set_buzzer(False)
            self._set_green(True)
            return

        # 闪烁模式（需后台线程）
        self._stop_event.clear()
        self._blink_thread = threading.Thread(
            target=self._blink_loop,
            args=(signal_type,),
            name="led-blink",
            daemon=True,
        )
        self._blink_thread.start()

    def _blink_loop(self, signal_type: IndicatorSignal) -> None:
        """LED 闪烁循环"""
        if signal_type == IndicatorSignal.BOOT_BLINK:
            interval = timing.LED_BOOT_BLINK_INTERVAL_MS / 1000  # 500ms
            green = True
            while not self._stop_event.is_set():
                self._set_green(green)
                self._set_red(False)
                green = not green
                time.sleep(interval)

        elif signal_type == IndicatorSignal.AUTONOMOUS_FAST_BLINK:
            interval = timing.LED_AUTO_BLINK_INTERVAL_MS / 1000  # 200ms
            green = True
            while not self._stop_event.is_set():
                self._set_green(green)
                self._set_red(False)
                green = not green
                time.sleep(interval)

        elif signal_type == IndicatorSignal.ERROR:
            interval = 0.2  # 200ms
            state = True
            while not self._stop_event.is_set():
                self._set_green(False)
                self._set_red(state)
                self._set_buzzer(state)
                state = not state
                time.sleep(interval)

    def _set_green(self, on: bool) -> None:
        self._GPIO.output(self._green_pin, self._GPIO.HIGH if on else self._GPIO.LOW)

    def _set_red(self, on: bool) -> None:
        self._GPIO.output(self._red_pin, self._GPIO.HIGH if on else self._GPIO.LOW)

    def _set_buzzer(self, on: bool) -> None:
        if self._buzzer_pin is not None:
            self._GPIO.output(self._buzzer_pin, self._GPIO.HIGH if on else self._GPIO.LOW)

    def cleanup(self) -> None:
        self._stop_event.set()
        if self._blink_thread and self._blink_thread.is_alive():
            self._blink_thread.join(timeout=1.0)
        self._GPIO.output(self._green_pin, self._GPIO.LOW)
        self._GPIO.output(self._red_pin, self._GPIO.LOW)
        if self._buzzer_pin is not None:
            self._GPIO.output(self._buzzer_pin, self._GPIO.LOW)
        self._GPIO.cleanup()
