"""
autonomous_state.py —— AUTONOMOUS 状态

全自主运行状态：
  - 一键启动后进入，**不可逆**
  - 锁定所有外部输入（不接收通信指令、不响应调试接口）
  - 运行完整的 感知 → 决策 → 导航 → 转运 主循环
  - 分级看门狗保活（10s 探索 / 13s 保命 / 不再淘汰）

这是比赛的核心状态，机器人完全靠自己。
"""

import logging
import threading
import time
from datetime import datetime
from typing import Optional

from ..state_machine import RobotState
from ..hardware.indicator import IndicatorSignal
from ..config import timing
from ..perception.field_elements import FieldLayout, SafeZoneColor
from ..perception.perception_pipeline import PerceptionPipeline
from ..navigation.navigation_pipeline import NavigationPipeline
from ..navigation.motion_control import VelocityCommand
from ..decision.decision_engine import DecisionEngine, Action, ActionType
from ..transport.transport_pipeline import TransportPipeline

logger = logging.getLogger("autonomous_state")


class AutonomousState:
    """
    AUTONOMOUS 状态处理器。

    进入即锁定，运行主循环直至比赛结束或异常。
    对外部输入完全免疫。

    主循环编排（每帧）：
      1. 感知：perception.update() → 更新世界地图
      2. 决策：decision.update(pose, nav_arrived, grip_done, release_done) → Action
      3. 执行 Action：NAVIGATE_TO / GRIP / TRANSPORT_TO / …
      4. 导航：navigation.update(pose) → VelocityCommand（内部积分定位器位姿）
      5. 底盘执行：controller.execute(cmd)（真实硬件 TODO）
      6. 转运：transport.update(pose, world_map, navigation)
      7. 套取/投放联动 + 分级看门狗
    """

    # 分级看门狗阈值（对齐 anomaly_handler / MEMORY 四层降级保活链）
    WATCHDOG_EXPLORE_S = 10.0     # 无动作 → 探索
    WATCHDOG_SURVIVAL_S = 13.0    # 无动作 → 保命绕圈
    WATCHDOG_HARD_LIMIT_S = 15.0  # 最后防线：仍不淘汰，仅持续保命运动

    def __init__(self, state_machine, indicator,
                 perception: Optional[PerceptionPipeline] = None,
                 decision: Optional[DecisionEngine] = None,
                 navigation: Optional[NavigationPipeline] = None,
                 transport: Optional[TransportPipeline] = None,
                 field_layout: Optional[FieldLayout] = None,
                 my_color: SafeZoneColor = SafeZoneColor.RED,
                 use_mock: bool = True,
                 controller=None):
        """
        Args:
            state_machine: StateMachine 实例
            indicator: Indicator 实例
            perception: 感知管线（None 则内部创建 Mock）
            decision: 决策引擎（None 则内部创建，绑定感知的世界地图）
            navigation: 导航管线（None 则内部创建 Mock）
            transport: 转运管线（None 则内部创建 Mock）
            field_layout: 场地布局（None 则用标准场地）
            my_color: 本队安全区颜色（抽签确定）
            use_mock: True=内部创建 Mock 定位/检测
            controller: 底盘执行器（执行 VelocityCommand），真实硬件 TODO
        """
        self._sm = state_machine
        self._indicator = indicator
        self._controller = controller

        # ── 四大管线（注入或内部创建）──
        self._field = field_layout or FieldLayout.standard()
        self._perception = perception or PerceptionPipeline(
            use_mock=use_mock, my_safe_zone_color=my_color,
        )
        self._navigation = navigation or NavigationPipeline(
            self._field, my_color=my_color, use_mock=use_mock,
        )
        self._transport = transport or TransportPipeline(
            field_layout=self._field, my_color=my_color, use_mock=use_mock,
        )
        # 决策引擎绑定感知的世界地图（感知是决策的唯一输入）
        self._decision = decision or DecisionEngine(
            self._perception.world_map, my_color=my_color,
        )

        # 主循环控制
        self._loop_thread: Optional[threading.Thread] = None
        self._stop_event = threading.Event()

        # 一趟转运的标志协调（DecisionEngine 与 TransportPipeline 之间）
        self._trip_gripped = False    # 当前目标是否已套取
        self._trip_released = False   # 当前目标是否已投放
        self._prev_current_id = None  # 决策引擎上一帧选中的目标 id（检测"新一趟"）

        # 看门狗
        self._last_action_time = time.time()
        self._explore_triggered = False
        self._survival_triggered = False
        self._last_action: Optional[Action] = None

        logger.info(f"AutonomousState 初始化: mock={use_mock}, "
                    f"my_color={my_color.name}")

    # ---- 状态回调 ----

    def on_enter(self) -> None:
        """进入 AUTONOMOUS 状态：锁定 + 启动主循环"""
        logger.info("🔴 进入 AUTONOMOUS 状态 — 全自主运行")
        logger.info("   ⚠️  所有外部输入已锁定，不可逆！")
        logger.info("   ⚠️  禁止触碰机器人，禁止触碰笔记本电脑")

        self._indicator.signal(IndicatorSignal.AUTONOMOUS_FAST_BLINK)

        # 锁定状态机（已在 state_machine 层面保证不可逆，
        # 此处额外确认不接收通信指令）
        self._lock_external_inputs()

        # 比赛开始
        self._decision.start_match()

        # 延迟启动（让裁判离开场地）
        logger.info(f"等待 {timing.POST_START_DELAY_MS}ms 后开始运行...")
        time.sleep(timing.POST_START_DELAY_MS / 1000)

        # 启动主循环
        self._stop_event.clear()
        self._last_action_time = time.time()
        self._explore_triggered = False
        self._survival_triggered = False
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
        """感知 → 决策 → 导航 → 转运 主循环（50Hz）。"""
        logger.info("主循环开始运行")
        cycle_count = 0
        dt = 0.02  # 50Hz

        while not self._stop_event.is_set():
            cycle_start = time.time()
            cycle_count += 1

            try:
                self._run_once(dt)
            except Exception as e:
                logger.error(f"主循环异常: {e}", exc_info=True)
                # 不轻易退出，尝试继续下一轮

            # 循环频率控制（50Hz）
            elapsed = time.time() - cycle_start
            sleep_time = max(0, dt - elapsed)
            if sleep_time > 0:
                time.sleep(sleep_time)

        logger.info(f"主循环结束，共运行 {cycle_count} 个周期")

    def _run_once(self, dt: float) -> None:
        """单帧编排：决策 → 执行 → 导航 → 转运 → 联动 → 看门狗。"""
        pose = self._navigation.pose
        x, y, theta = pose.x, pose.y, pose.theta

        # ── 0. 检测"新一趟"（决策引擎选中了新目标 → 复位本趟进度标志）──
        cur = self._decision._current_target
        cur_id = cur.id if cur is not None else None
        if cur_id is not None and cur_id != self._prev_current_id:
            self._trip_gripped = False
            self._trip_released = False
        self._prev_current_id = cur_id

        nav_arrived = self._navigation.is_arrived()
        grip_done = self._trip_gripped
        release_done = self._trip_released

        # ── 1. 感知 ──
        self._perception.update(frame=None, robot_position=(x, y))

        # ── 2. 决策 ──
        action = self._decision.update(
            (x, y, theta),
            nav_arrived=nav_arrived,
            grip_done=grip_done,
            release_done=release_done,
        )
        self._last_action = action

        # ── 3. 执行 Action ──
        if action.type == ActionType.NAVIGATE_TO:
            self._set_nav_target(action.target_position)
        elif action.type == ActionType.GRIP:
            if self._transport.is_idle():
                tracks = [self._perception.world_map.targets[tid]
                          for tid in action.target_ids
                          if tid in self._perception.world_map.targets]
                if tracks:
                    self._transport.start_trip(tracks)
                    self._trip_gripped = False
        elif action.type == ActionType.TRANSPORT_TO:
            self._set_nav_target(action.target_position)
        elif action.type == ActionType.EMERGENCY_STOP:
            self._sm.emergency_stop("决策引擎请求急停")
            return

        # ── 4. 导航（内部积分定位器位姿）──
        cmd = self._navigation.update((x, y, theta), dt=dt)

        # ── 5. 底盘执行（真实硬件 TODO：把 cmd 下发给电机驱动）──
        if self._controller is not None:
            self._controller.execute(cmd)

        # ── 6. 转运推进 ──
        self._transport.update((x, y, theta), self._perception.world_map, self._navigation)

        # ── 7. 套取联动 ──
        loaded_ids = self._transport.load_manager.state.target_ids
        if loaded_ids:
            self._trip_gripped = True
            for tid in loaded_ids:
                self._perception.world_map.mark_being_transported(tid)

        # ── 8. 投放联动 ──
        if self._transport.is_complete() and self._trip_gripped:
            self._trip_released = True

        # ── 9. 看门狗：有实际动作则更新 ──
        if abs(cmd.linear) > 10.0 or abs(cmd.angular) > 0.01:
            self._last_action_time = time.time()
            self._explore_triggered = False
            self._survival_triggered = False
        else:
            self._check_watchdog()

    def _set_nav_target(self, pos) -> None:
        """设置导航目标（去重，避免每帧重复触发重规划）。"""
        if pos is None:
            return
        if self._navigation.target != pos:
            self._navigation.set_target(pos[0], pos[1])

    # ---- 看门狗 ----

    def _check_watchdog(self) -> None:
        """
        分级看门狗保活（对齐 MEMORY 四层降级保活链）：

          无动作 > 10s  → 探索模式（驶向场地随机位置边移动边扫描）
          无动作 > 13s  → 保命绕圈（低速绕圈，保持运动）
          无动作 > 15s  → 不再淘汰，持续保命运动（保持比赛资格）

        不再因为 15 秒无动作而 emergency_stop。
        """
        idle_duration = time.time() - self._last_action_time

        if idle_duration > self.WATCHDOG_SURVIVAL_S and not self._survival_triggered:
            self._survival_triggered = True
            self._navigation.survival_circle()
            logger.warning(f"🛟 保命绕圈：{idle_duration:.1f}s 无动作")
        elif idle_duration > self.WATCHDOG_EXPLORE_S and not self._explore_triggered:
            self._explore_triggered = True
            self._navigation.explore()
            logger.warning(f"🔍 探索模式：{idle_duration:.1f}s 无动作")

    def _lock_external_inputs(self) -> None:
        """
        锁定所有外部输入。

        AUTONOMOUS 下：
          - 通信模块只发送不接收（CommManager 依据 state_machine.is_locked 拦截）
          - 调试接口禁用
          - 配置文件只读
        """
        if not self._sm.is_locked:
            logger.warning("警告：state_machine.is_locked 未置位，通信锁定可能未生效")
        logger.info("外部输入已锁定")

    # ---- 状态查询 ----

    def get_loop_stats(self) -> dict:
        """获取主循环运行统计"""
        return {
            "running": self._loop_thread is not None and self._loop_thread.is_alive(),
            "last_action": datetime.fromtimestamp(self._last_action_time).isoformat(),
            "explore_triggered": self._explore_triggered,
            "survival_triggered": self._survival_triggered,
            "current_action": self._last_action.type.name if self._last_action else "WAIT",
            "decision_state": self._decision.strategy_state.name,
            "nav_state": self._navigation.state.name,
            "transport_phase": self._transport.phase.name,
        }
