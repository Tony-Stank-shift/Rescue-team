"""
autonomous_state.py —— AUTONOMOUS 状态

全自主运行状态：
  - 一键启动后进入，**不可逆**
  - 锁定所有外部输入（不接收通信指令、不响应调试接口）
  - 运行 感知 → 决策 → 执行 主循环
  - 异常检测（15 秒无动作、失控检测）

这是比赛的核心状态，机器人完全靠自己。
"""

import logging
import threading
import time
from datetime import datetime

from ..state_machine import RobotState
from ..hardware.indicator import IndicatorSignal
from ..config import timing

logger = logging.getLogger("autonomous_state")


class AutonomousState:
    """
    AUTONOMOUS 状态处理器。

    进入即锁定，运行主循环直至比赛结束或异常。
    对外部输入完全免疫。
    """

    def __init__(self, state_machine, indicator, perception=None, planner=None, controller=None):
        """
        Args:
            state_machine: StateMachine 实例
            indicator: Indicator 实例
            perception: 感知模块（占位，后续实现）
            planner: 决策规划模块（占位，后续实现）
            controller: 控制执行模块（占位，后续实现）
        """
        self._sm = state_machine
        self._indicator = indicator
        self._perception = perception
        self._planner = planner
        self._controller = controller

        # 主循环控制
        self._loop_thread: threading.Thread | None = None
        self._stop_event = threading.Event()

        # 看门狗：上一次有动作的时间
        self._last_action_time = time.time()
        self._watchdog_triggered = False

    # ---- 状态回调 ----

    def on_enter(self) -> None:
        """进入 AUTONOMOUS 状态：锁定 + 启动主循环"""
        logger.info("🔴 进入 AUTONOMOUS 状态 — 全自主运行")
        logger.info("   ⚠️  所有外部输入已锁定，不可逆！")
        logger.info("   ⚠️  禁止触碰机器人，禁止触碰笔记本电脑")

        self._indicator.signal(IndicatorSignal.AUTONOMOUS_FAST_BLINK)

        # 锁定状态机（已在 state_machine 层面保证不可逆，
        # 此处额外确保不接收通信指令）
        self._lock_external_inputs()

        # 延迟启动（让裁判离开场地）
        logger.info(f"等待 {timing.POST_START_DELAY_MS}ms 后开始运行...")
        time.sleep(timing.POST_START_DELAY_MS / 1000)

        # 启动主循环
        self._stop_event.clear()
        self._last_action_time = time.time()
        self._loop_thread = threading.Thread(
            target=self._main_loop,
            name="autonomous-loop",
            daemon=True,
        )
        self._loop_thread.start()
        logger.info("主循环已启动")

    def on_exit(self) -> None:
        """退出 AUTONOMOUS 状态：停止主循环"""
        logger.info("退出 AUTONOMOUS 状态 — 停止主循环")
        self._stop_event.set()

        if self._loop_thread and self._loop_thread.is_alive():
            self._loop_thread.join(timeout=2.0)
            if self._loop_thread.is_alive():
                logger.warning("主循环线程未能及时停止")

        self._indicator.signal(IndicatorSignal.OFF)

    # ---- 主循环 ----

    def _main_loop(self) -> None:
        """
        感知 → 决策 → 执行 主循环。

        每个循环周期：
        1. 感知：获取传感器数据、更新目标地图
        2. 决策：根据状态机选择行为
        3. 执行：控制底盘和转运机构
        4. 看门狗：检查是否有动作
        """
        logger.info("主循环开始运行")
        cycle_count = 0

        while not self._stop_event.is_set():
            cycle_start = time.time()
            cycle_count += 1

            try:
                # --- 1. 感知 ---
                if self._perception:
                    perception_data = self._perception.update()
                else:
                    perception_data = None  # 占位

                # --- 2. 决策 ---
                if self._planner:
                    action = self._planner.plan(perception_data)
                else:
                    action = None  # 占位

                # --- 3. 执行 ---
                if self._controller and action:
                    self._controller.execute(action)
                    self._last_action_time = time.time()  # 更新看门狗

                # --- 4. 看门狗 ---
                self._check_watchdog()

            except Exception as e:
                logger.error(f"主循环异常: {e}", exc_info=True)
                # 不轻易退出，尝试继续下一轮

            # 循环频率控制（暂定 50Hz）
            elapsed = time.time() - cycle_start
            sleep_time = max(0, 0.02 - elapsed)  # 目标 50Hz
            if sleep_time > 0:
                time.sleep(sleep_time)

        logger.info(f"主循环结束，共运行 {cycle_count} 个周期")

    # ---- 看门狗 ----

    def _check_watchdog(self) -> None:
        """
        看门狗检查：15 秒无动作 → 比赛结束。

        根据比赛规则，机器人停止运行 15 秒则本轮比赛结束。
        这里检测 last_action_time，超过 15 秒触发紧急停止。
        """
        idle_duration = time.time() - self._last_action_time
        if idle_duration > 15.0 and not self._watchdog_triggered:
            self._watchdog_triggered = True
            logger.warning(f"⚠️  看门狗触发：{idle_duration:.1f} 秒无动作")
            self._sm.emergency_stop("15 秒无动作，本轮结束")

    def _lock_external_inputs(self) -> None:
        """
        锁定所有外部输入。

        在 AUTONOMOUS 状态下：
        - 通信模块只发送不接收
        - 调试接口禁用
        - 配置文件只读
        """
        logger.info("外部输入已锁定")
        # 实际的锁定由各模块检查 state_machine.is_locked 实现
        # 此方法记录锁定意图，具体模块自行实现检查逻辑

    # ---- 状态查询 ----

    def get_loop_stats(self) -> dict:
        """获取主循环运行统计"""
        return {
            "running": self._loop_thread is not None and self._loop_thread.is_alive(),
            "last_action": datetime.fromtimestamp(self._last_action_time).isoformat(),
            "watchdog_triggered": self._watchdog_triggered,
        }
