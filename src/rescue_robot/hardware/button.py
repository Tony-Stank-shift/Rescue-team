"""
button.py —— 一键启动按钮

提供平台无关的按钮抽象：
  - MockButton：本地开发/CI 测试（键盘模拟）
  - GPIOButton：树莓派物理按钮

特性：
  - 硬件去抖（50ms）
  - 短按 / 长按区分（长按 500ms 确认，防误触）
  - 事件回调注册
"""

import logging
import threading
import time
from enum import Enum, auto
from typing import Callable, List

from ..config import timing

logger = logging.getLogger("button")


class ButtonEvent(Enum):
    """按钮事件类型"""
    SHORT_PRESS = auto()   # 短按（< 500ms）
    LONG_PRESS = auto()    # 长按（≥ 500ms）—— 触发一键启动


class AbstractButton:
    """按钮抽象基类"""

    def __init__(self):
        self._callbacks: List[Callable[[ButtonEvent], None]] = []
        self._monitor_thread: threading.Thread | None = None
        self._stop_event = threading.Event()

    def on_event(self, callback: Callable[[ButtonEvent], None]) -> None:
        """注册按钮事件回调"""
        self._callbacks.append(callback)

    def _fire_event(self, event: ButtonEvent) -> None:
        """触发所有回调"""
        for cb in self._callbacks:
            try:
                cb(event)
            except Exception as e:
                logger.error(f"按钮回调异常: {e}")

    def start_monitoring(self) -> None:
        """启动按钮监听线程"""
        if self._monitor_thread and self._monitor_thread.is_alive():
            return
        self._stop_event.clear()
        self._monitor_thread = threading.Thread(
            target=self._monitor_loop,
            name="button-monitor",
            daemon=True,
        )
        self._monitor_thread.start()
        logger.info("按钮监听已启动")

    def stop_monitoring(self) -> None:
        """停止按钮监听"""
        self._stop_event.set()
        if self._monitor_thread:
            self._monitor_thread.join(timeout=2.0)
        logger.info("按钮监听已停止")

    def _monitor_loop(self) -> None:
        """按钮监听主循环 —— 子类实现 _is_pressed()"""
        raise NotImplementedError

    def _is_pressed(self) -> bool:
        """读取按钮当前状态（子类实现）"""
        raise NotImplementedError


# ============================================================
# Mock 按钮 —— 键盘模拟
# ============================================================

class MockButton(AbstractButton):
    """
    Mock 按钮：用键盘输入模拟物理按钮。

    在终端输入：
      's' + Enter → 短按
      'l' + Enter → 长按
      'q' + Enter → 退出监听
    """

    def _monitor_loop(self) -> None:
        logger.info("Mock 按钮: 输入 'l' 模拟长按(一键启动), 's' 模拟短按, 'q' 退出")
        while not self._stop_event.is_set():
            try:
                ch = input().strip().lower()
                if ch == 'l':
                    logger.info("→ 模拟长按事件")
                    time.sleep(0.1)  # 模拟按下持续时间
                    self._fire_event(ButtonEvent.LONG_PRESS)
                elif ch == 's':
                    logger.info("→ 模拟短按事件")
                    self._fire_event(ButtonEvent.SHORT_PRESS)
                elif ch == 'q':
                    logger.info("Mock 按钮退出")
                    break
            except EOFError:
                time.sleep(0.5)
            except Exception as e:
                logger.error(f"Mock 按钮读取异常: {e}")
                time.sleep(0.5)


# ============================================================
# GPIO 按钮 —— 真实硬件
# ============================================================

class GPIOButton(AbstractButton):
    """
    真实 GPIO 按钮（树莓派）。

    使用 RPi.GPIO 库监听物理按钮：
    - 硬件去抖 50ms
    - 按下持续时间 ≥ 500ms → 长按（触发一键启动）
    - 按下持续时间 < 500ms → 短按（忽略）
    """

    def __init__(self, pin: int):
        """
        Args:
            pin: BCM 引脚编号
        """
        super().__init__()
        self._pin = pin

        # 延迟导入（避免非树莓派环境报错）
        try:
            import RPi.GPIO as GPIO
            self._GPIO = GPIO
        except ImportError:
            logger.error("RPi.GPIO 未安装！请运行: pip install RPi.GPIO")
            raise

        self._GPIO.setmode(GPIO.BCM)
        self._GPIO.setup(self._pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)

    def _is_pressed(self) -> bool:
        """按钮按下 = 低电平（上拉模式）"""
        return self._GPIO.input(self._pin) == self._GPIO.LOW

    def _monitor_loop(self) -> None:
        """
        GPIO 中断 + 轮询混合监听。

        使用 GPIO.wait_for_edge 等待下降沿（按下），
        然后计时直到释放，根据持续时间判断短按/长按。
        """
        debounce_ms = timing.BUTTON_DEBOUNCE_MS
        long_press_ms = timing.BUTTON_LONG_PRESS_MS

        logger.info(f"GPIO 按钮监听已启动 (引脚: {self._pin}, "
                    f"防抖: {debounce_ms}ms, 长按: {long_press_ms}ms)")

        while not self._stop_event.is_set():
            # 等待下降沿（按钮按下）
            edge_detected = self._GPIO.wait_for_edge(
                self._pin,
                self._GPIO.FALLING,
                timeout=200,  # 200ms 超时，便于检查 stop_event
            )

            if edge_detected is None:
                continue  # 超时，回到循环

            # 记录按下时间
            press_time = time.time()

            # 等待释放
            while self._is_pressed() and not self._stop_event.is_set():
                time.sleep(0.01)  # 10ms 轮询

            # 计算持续时间
            duration_ms = (time.time() - press_time) * 1000

            if duration_ms < debounce_ms:
                # 太短，认为是抖动
                logger.debug(f"忽略抖动 ({duration_ms:.0f}ms < {debounce_ms}ms)")
                continue

            # 判断事件类型
            if duration_ms >= long_press_ms:
                logger.info(f"🔘 GPIO 长按 ({duration_ms:.0f}ms) — 一键启动！")
                self._fire_event(ButtonEvent.LONG_PRESS)
            else:
                logger.debug(f"GPIO 短按 ({duration_ms:.0f}ms) — 忽略")
                self._fire_event(ButtonEvent.SHORT_PRESS)

    def cleanup(self) -> None:
        """清理 GPIO 资源"""
        self._GPIO.cleanup(self._pin)
