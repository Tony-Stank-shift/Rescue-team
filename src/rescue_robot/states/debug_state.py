"""
debug_state.py —— DEBUG 状态

调试配置模式：
  - 允许与笔记本通信（配置参数、查看传感器数据、校准）
  - 监听一键启动按钮
  - 按钮被按下（长按确认）→ 触发一键启动 → 进入 AUTONOMOUS

这是人为干预的最后窗口。一旦离开此状态，再也回不来。
"""

import logging

from ..state_machine import RobotState
from ..hardware.button import ButtonEvent
from ..hardware.indicator import IndicatorSignal
from ..config import timing

logger = logging.getLogger("debug_state")


class DebugState:
    """
    DEBUG 状态处理器。

    职责：
    1. 通知外界"现在可以安全通信"
    2. 注册一键启动按钮回调
    3. 等待按钮事件触发状态转移
    """

    def __init__(self, state_machine, button, indicator, comm_server=None):
        """
        Args:
            state_machine: StateMachine 实例
            button: Button 实例（一键启动按钮）
            indicator: Indicator 实例
            comm_server: 可选，通信服务器（笔记本通信）
        """
        self._sm = state_machine
        self._button = button
        self._indicator = indicator
        self._comm_server = comm_server

        # 按钮回调
        self._button.on_event(self._on_button_event)

    # ---- 状态回调 ----

    def on_enter(self) -> None:
        """进入 DEBUG 状态：启动通信、等待按钮"""
        logger.info("🟢 进入 DEBUG 状态 — 等待一键启动...")
        logger.info("   （可与笔记本通信、配置参数、校准传感器）")
        logger.info(f"   （长按按钮 {timing.BUTTON_LONG_PRESS_MS}ms 确认启动）")

        self._indicator.signal(IndicatorSignal.DEBUG_STEADY)

        # 启动通信服务（如果已配置）
        if self._comm_server:
            self._comm_server.start()
            logger.info("通信服务已启动")

    def on_exit(self) -> None:
        """退出 DEBUG 状态：关闭通信"""
        logger.info("退出 DEBUG 状态")

        # 关闭通信服务
        if self._comm_server:
            self._comm_server.stop()
            logger.info("通信服务已停止")

        self._indicator.signal(IndicatorSignal.OFF)

    # ---- 按钮事件 ----

    def _on_button_event(self, event: ButtonEvent) -> None:
        """处理按钮事件"""
        if event == ButtonEvent.LONG_PRESS:
            logger.info("🔘 检测到长按 — 触发一键启动！")
            try:
                self._sm.one_key_start()
            except ValueError as e:
                logger.error(f"一键启动失败: {e}")
                self._indicator.signal(IndicatorSignal.ERROR)
        elif event == ButtonEvent.SHORT_PRESS:
            logger.debug("按钮短按 — 忽略（需长按确认）")
