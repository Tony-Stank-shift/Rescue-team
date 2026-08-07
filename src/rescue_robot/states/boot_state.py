"""
boot_state.py —— BOOT 状态

上电后进入的第一个状态，执行系统自检。
  自检通过 → 自动转入 DEBUG
  自检失败 → 转入 ERROR（LED 红灯闪烁，等待重启）
"""

import logging

from ..state_machine import RobotState
from ..system_check import SystemChecker, CheckReport
from ..hardware.indicator import IndicatorSignal

logger = logging.getLogger("boot_state")


class BootState:
    """
    BOOT 状态处理器。

    进入时执行 on_enter()，运行系统自检，根据结果自动转移。
    """

    def __init__(self, state_machine, system_checker: SystemChecker, indicator):
        """
        Args:
            state_machine: StateMachine 实例（用于触发转移）
            system_checker: SystemChecker 实例
            indicator: Indicator 实例（LED 指示）
        """
        self._sm = state_machine
        self._checker = system_checker
        self._indicator = indicator
        self._report: CheckReport | None = None

    @property
    def report(self) -> CheckReport | None:
        """获取最近一次自检报告"""
        return self._report

    # ---- 状态回调 ----

    def on_enter(self) -> None:
        """进入 BOOT 状态：执行自检"""
        logger.info("🔵 进入 BOOT 状态 — 系统自检中...")
        self._indicator.signal(IndicatorSignal.BOOT_BLINK)

        # 执行自检
        self._report = self._checker.run()

        if self._report.overall_pass:
            logger.info("自检通过，转入 DEBUG 状态")
            self._sm.transition(RobotState.DEBUG, reason="自检通过")
        else:
            logger.error("自检失败！转入 ERROR 状态")
            self._indicator.signal(IndicatorSignal.ERROR)
            self._sm.transition(
                RobotState.ERROR,
                reason=f"自检失败: {self._report.failed_count} 项未通过",
            )

    def on_exit(self) -> None:
        """退出 BOOT 状态"""
        logger.info("退出 BOOT 状态")
        self._indicator.signal(IndicatorSignal.OFF)
