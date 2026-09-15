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
import math
import threading
import time
from datetime import datetime
from typing import Optional

from ..state_machine import RobotState
from ..hardware.indicator import IndicatorSignal
from ..config import timing
from ..perception.field_elements import FieldLayout, SafeZoneColor, FieldElementType
from ..perception.perception_pipeline import PerceptionPipeline
from ..navigation.navigation_pipeline import NavigationPipeline
from ..navigation.motion_control import VelocityCommand
from ..decision.decision_engine import (DecisionEngine, Action, ActionType,
                                       StrategyState)
from ..transport.transport_pipeline import TransportPipeline, TransportPhase
from ..hardware.serial_chassis import SerialChassis

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

    # ── "有没有动作"的判据：里程计**实际位移**，不是下发速度 ──
    # 旧实现看 `abs(cmd.linear) > 10`：轮子卡在围栏上/打滑空转时指令一直是几百 mm/s，
    # 看门狗永远不触发 → 车原地耗到比赛时间结束（整场 0 分）。
    WATCHDOG_MOVE_MM = 40.0       # 一个窗口内累计位移超过此值 = 确实在动
    WATCHDOG_STEP_MIN_MM = 1.0    # 单帧位移下限：滤掉里程计抖动，避免把噪声当运动
    STUCK_WARN_S = 3.0            # 在下发速度却迟迟不动 → 提前告警（现场定位用）
    #: 非"时间到"来源的 DONE 必须连续保持这么久才允许执行终场停车。
    #: 为什么需要确认窗口：DONE 一旦误判就是**不可恢复的整场结束**，
    #: 而它的判据曾包含"当前帧看不到目标"（开局未建图/摄像头掉线/目标在车上都会触发）。
    #: 时间到（权威终场）不等待，立即停车。
    DONE_CONFIRM_S = 3.0
    #: 车心到减速带矩形边界的距离小于此值即进入越障模式（U4）。
    #: 250mm 的量级依据：减速带本身 60mm 宽、间距 50mm、车长约 200mm，
    #: 需在车头接触之前就降速。属**真机标定项**，现场按实际制动距离调整。
    SPEED_BUMP_TRIGGER_MM = 250.0

    def __init__(self, state_machine, indicator,
                 perception: Optional[PerceptionPipeline] = None,
                 decision: Optional[DecisionEngine] = None,
                 navigation: Optional[NavigationPipeline] = None,
                 transport: Optional[TransportPipeline] = None,
                 chassis: Optional[SerialChassis] = None,
                 camera=None,
                 field_layout: Optional[FieldLayout] = None,
                 my_color: SafeZoneColor = SafeZoneColor.RED,
                 use_mock: bool = True,
                 controller=None,
                 start_zone: int = 3):
        """
        Args:
            state_machine: StateMachine 实例
            indicator: Indicator 实例
            perception: 感知管线（None 则内部创建 Mock）
            decision: 决策引擎（None 则内部创建，绑定感知的世界地图）
            navigation: 导航管线（None 则内部创建 Mock）
            transport: 转运管线（None 则内部创建 Mock）
            chassis: 串口底盘驱动（SerialChassis），真机联调用；None 则用导航定位器 + controller
            camera: cv2.VideoCapture 实例，真机视觉；None 则感知用 frame=None（Mock）
            field_layout: 场地布局（None 则用标准场地）
            my_color: 本队安全区颜色（抽签确定）
            use_mock: True=内部创建 Mock 定位/检测
            controller: 底盘执行器（执行 VelocityCommand），chassis 为空时使用
            start_zone: 抽签得到的出发区号 1~4（决定全场坐标系原点与朝向）
        """
        self._sm = state_machine
        self._indicator = indicator
        self._controller = controller
        self._chassis = chassis  # SerialChassis 实例（真机：位姿来源 + 速度下发）
        self._camera = camera  # cv2.VideoCapture 实例（真机视觉读帧）

        # 出发区位姿：坐标系唯一来源（现场抽签决定 1~4 号区）
        from ..perception.field_elements import StandardFieldLayout
        self._start_zone = int(start_zone)
        _pose = StandardFieldLayout().get_start_pose(self._start_zone) \
            or (150.0, 150.0, 1.5707963267948966)
        self._start_pose: tuple = _pose

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

        # ── 套取的两项注入（真机必需）──
        # ① 显式停车：套取会阻塞主循环若干秒，必须主动停车，
        #    否则底盘会一直执行最后一帧速度指令，靠看门狗 ~800ms 后才停（会前冲一段）。
        if chassis is not None:
            self._transport.set_stop_callback(self._stop_chassis)
        # ② 套取视觉确认：本车没有"套住检测"传感器 → 用摄像头看 U 型槽里有没有目标。
        #    只在真机摄像头可用时启用（Mock/无摄像头时保持"假设成功"，避免误判重试）。
        if camera is not None:
            self._transport.set_sleeve_confirm(self._perception.check_sleeve_occupied)
            self._transport.set_release_failed_callback(self._on_release_failed)
            logger.info("已启用套取视觉确认（槽内 ROI 判据）")
        else:
            logger.info("无摄像头：套取不启用视觉确认（按接住处理）")

        # 主循环控制
        self._loop_thread: Optional[threading.Thread] = None
        self._stop_event = threading.Event()

        # 一趟转运的标志协调（DecisionEngine 与 TransportPipeline 之间）
        self._trip_gripped = False    # 当前目标是否已套取
        self._trip_released = False   # 当前目标是否已投放
        self._trip_released_valid = False  # 本次投放是否**有效**（落点入正确区域）
        self._prev_current_id = None  # 决策引擎上一帧选中的目标 id（检测"新一趟"）
        self._finished_logged = False  # 终场停车只做一次（S-01）
        self._done_since = None       # 非"时间到"的 DONE 起始时刻（确认窗口用）

        # 看门狗
        self._last_action_time = time.time()
        self._explore_triggered = False
        self._survival_triggered = False
        self._last_action: Optional[Action] = None
        # 上一帧**下发**的速度（卡死检测要用"下发速度 vs 里程计位移"两路独立信息，见 T0-2）
        self._last_cmd: Optional[tuple] = None
        # T1-10：保活（探索/绕圈）优先级窗口。看门狗触发保活后，决策引擎**每帧**
        # 都会重发它自己的 NAVIGATE_TO/TRANSPORT_TO，把保活点覆盖掉（只存活 1 帧）
        # → 保活链形同虚设、车继续朝原目标硬顶。这里在保活触发后给一个短窗口，
        # 窗口内不接受决策引擎的目标，让车真的先动起来脱离困境。
        self._keepalive_until: float = 0.0
        # T2-23：主循环异常计数（可见性）
        self._loop_error_count = 0
        self._last_loop_error = ""
        # 上次位姿（串口无数据时保持）：初值 = 出发区位姿（见 on_enter 再次强制下发）
        self._pose: tuple = tuple(self._start_pose)

        # 看门狗：按里程计实际位移判定"是否在动"（见 _update_watchdog）
        self._last_motion_pose: Optional[tuple] = None   # 上一帧位姿
        self._motion_accum_mm: float = 0.0               # 本窗口累计位移
        self._last_motion_time: float = time.time()      # 上次"确实在动"的时刻
        self._stuck_warned: bool = False                 # 打滑/堵转告警去重
        self._last_velocity: tuple = (0.0, 0.0)          # 由位姿差算出的实际速度（mm/s）

        logger.info(f"AutonomousState 初始化: mock={use_mock}, "
                    f"my_color={my_color.name}, chassis={'是' if chassis else '否'}")

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

        # ── 按抽签出发区初始化坐标系（必须在任何导航之前）──
        # 旧实现把原点写死 (150,150,90°)（= 3 号区），抽到别的区全场错位。
        self._apply_start_pose()

        # 真机：若配置了串口底盘，先打开并启动（PING→START 确认连接）
        # ⚠️ T0-10：这里必须**fail-fast**。旧实现打开/握手失败只打一行 ⚠️ 就继续，
        # 而协议明确"上电后 STM32 默认禁止运动，必须先收到一次 START"，未 START 时
        # 所有运动命令都被拒（ERR,NOT_STARTED）→ 车一动不动，全场 0 分，
        # 日志里却只有一行"⚠️ 失败"，现场无从判断。
        if self._chassis is not None:
            if not self._chassis.is_open and not self._chassis.open():
                logger.critical(f"❌ 串口底盘打开失败（{self._chassis._port}）→ "
                                f"拒绝进入自主模式：无法下发速度、无法读取位姿")
                self._sm.emergency_stop("串口底盘不可用")
                return
            if not self._chassis.start_match():
                logger.critical("❌ 底盘 START 握手失败（PONG/ACK 超时）→ 拒绝进入自主模式："
                                "协议规定未 START 时所有运动命令都会被拒（ERR,NOT_STARTED）")
                self._sm.emergency_stop("底盘未启动")
                return
            logger.info("底盘连接+启动: ✅ 成功（PONG + ACK,START）")

        # 延迟启动（让裁判离开场地）
        logger.info(f"等待 {timing.POST_START_DELAY_MS}ms 后开始运行...")
        time.sleep(timing.POST_START_DELAY_MS / 1000)

        # 启动主循环
        self._stop_event.clear()
        self._last_action_time = time.time()
        self._explore_triggered = False
        self._survival_triggered = False
        self._finished_logged = False   # 终场停车标志复位（S-01）
        self._keepalive_until = 0.0     # 保活优先窗口复位（T1-10）
        # 看门狗位移窗口复位（否则上一轮/初始化前的位姿差会被算成本轮位移）
        self._last_motion_pose = None
        self._motion_accum_mm = 0.0
        self._last_motion_time = time.time()
        self._stuck_warned = False
        self._loop_thread = threading.Thread(
            target=self._main_loop,
            name="autonomous-loop",
            daemon=True,
        )
        self._loop_thread.start()
        logger.info("主循环已启动")

    def _apply_start_pose(self) -> None:
        """
        把出发区位姿下发到：① 下位机里程计参考系 ② 导航定位器 ③ 本状态内部位姿。

        三个地方必须一致，否则"车认为在 A、导航认为在 B"→ 目标点整体偏移。
        """
        x, y, theta = self._start_pose
        self._pose = (x, y, theta)
        if self._chassis is not None:
            try:
                self._chassis.set_start_pose(x, y, theta)
            except Exception as e:
                logger.warning(f"下发出发区位姿到串口底盘失败: {e}")
        try:
            self._navigation.reset_pose(x, y, theta)
        except Exception as e:
            logger.warning(f"同步导航定位器位姿失败: {e}")
        logger.info(f"坐标系初始化：出发区 {self._start_zone} 号，"
                    f"起点=({x:.0f}, {y:.0f})mm，朝向={math.degrees(theta):.0f}°"
                    f"（请现场核对是否与抽签结果一致）")

    def on_exit(self) -> None:
        """退出 AUTONOMOUS 状态：停止主循环"""
        logger.info("退出 AUTONOMOUS 状态 — 停止主循环")
        self._stop_event.set()

        if self._loop_thread and self._loop_thread.is_alive():
            self._loop_thread.join(timeout=2.0)
            if self._loop_thread.is_alive():
                logger.warning("主循环线程未能及时停止")

        # T1-9：退出自主态时**必须**再发一次停车。旧实现只置 `_stop_event` 就返回，
        # 主循环停了但底盘仍握有最后一帧 VEL → 只能等 800ms 看门狗。
        # 这条覆盖 Ctrl+C、ERROR 转移等所有退出路径（终场停车由 `_finish_match` 负责）。
        if self._chassis is not None:
            try:
                self._stop_chassis()
            except Exception as e:
                logger.warning(f"退出时停车失败: {e}")

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
                # T2-23：主循环吞异常是必要的（不能让单帧异常终止整场），
                # 但**必须可见**：本会话曾被它掩盖过一次"决策崩了但进程健康、0 分"。
                # 这里累计计数 + 每 250 帧（约 5s）再报一次，并保留最后一条错误供诊断。
                self._loop_error_count += 1
                self._last_loop_error = f"{type(e).__name__}: {e}"
                if self._loop_error_count == 1 or self._loop_error_count % 250 == 0:
                    logger.error(f"⚠️ 主循环异常（累计 {self._loop_error_count} 次，"
                                 f"最近一次: {self._last_loop_error}）", exc_info=True)
                # 不轻易退出，尝试继续下一轮

            # 循环频率控制（50Hz）
            elapsed = time.time() - cycle_start
            sleep_time = max(0, dt - elapsed)
            if sleep_time > 0:
                time.sleep(sleep_time)

        logger.info(f"主循环结束，共运行 {cycle_count} 个周期")

    def _run_once(self, dt: float) -> None:
        """单帧编排：决策 → 执行 → 导航 → 转运 → 联动 → 看门狗。"""
        # ── 终场停车（S-01）──
        # 旧实现：主循环只认 `_stop_event`（状态切换/Ctrl+C），**没有任何"比赛结束"处理**。
        # 时间到后决策引擎已经返回 WAIT，但导航里还留着最后一个目标、主循环仍以 50Hz
        # 下发上一帧速度 → 机器人继续朝场地里冲，撞紫边/撞对手/把已投放的目标撞出安全区
        # （现场直接判罚）。规则要求终场立即停止，所以这里必须停车。
        #
        # ⚠️ 但"决策引擎 DONE"**不能无条件当作终场**：DONE 一旦被误判就是不可恢复的
        # 整场结束（清目标 + 停车 + 退出主循环）。而 DONE 的判据曾包含"当前帧看不到
        # 目标"，开局未建图 / 摄像头掉线 / 目标在车上都会让它误触发
        # （见 docs/audit/INCREMENTAL_AUDIT.md 的 N-1）。
        # 因此分两档：
        #   ① **时间到** → 权威终场，立即停车退出；
        #   ② 其它来源的 DONE → 必须**连续保持** DONE_CONFIRM_S 秒才执行；一旦不再
        #      DONE 就复位计时并继续比赛（保证可恢复）。
        if self._decision.strategy_state == StrategyState.DONE:
            time_up = self._decision.time_remaining_s <= 0
            if time_up:
                self._finish_match("比赛时间到")
                return
            # 非时间到的 DONE：先观察，别急着停
            now = time.time()
            if self._done_since is None:
                self._done_since = now
                logger.warning(
                    "决策引擎进入 DONE 但**比赛时间未到** → 先观察 "
                    f"{self.DONE_CONFIRM_S:.0f}s 再决定是否终场（防止误停车退赛）")
            elif now - self._done_since >= self.DONE_CONFIRM_S:
                self._finish_match("决策引擎 DONE 持续确认")
                return
        else:
            # DONE 消失 → 复位确认计时（目标重新出现就继续比赛）
            if self._done_since is not None:
                logger.info("决策引擎已退出 DONE → 恢复比赛（不终场）")
            self._done_since = None

        # ── 位姿来源：优先串口（下位机里程计），否则用导航定位器 ──
        if self._chassis is not None:
            pose_data = self._chassis.read_pose()
            if pose_data is not None:
                self._pose = pose_data
            x, y, theta = self._pose
        else:
            pose = self._navigation.pose
            x, y, theta = pose.x, pose.y, pose.theta

        # ── 0a. 位姿跳变兜底（T1-11）：单帧位移过大 = 车被**搬动**过 ──
        # 赛规：接触超 10s 裁判会把机器人强制分离、放回出发区继续（计时不停）。
        # 软件无法直接感知"被搬回"，但可以检测**位姿跳变**：一帧内位移 > 400mm
        # （正常行驶 50Hz 下每帧最多 ~17mm）几乎只可能是被搬动/重放。
        # 旧实现无任何处理 → 分离后仍按旧坐标跑，最坏朝错误方向冲出场/撞围栏。
        # 处理：把坐标系重置回出发区（下位机 + 导航定位器 + 内部位姿），并打 CRITICAL。
        # ⚠️ 必须用**上一帧**位姿比较：`self._pose` 在本帧开头的"位姿来源"块里
        # 已经被更新成本帧值了，拿它比恒为 0。`_last_motion_pose` 由看门狗每帧维护，
        # 此刻仍保存着上一帧的 (x, y)。
        if self._last_motion_pose is not None:
            jump = math.hypot(x - self._last_motion_pose[0],
                              y - self._last_motion_pose[1])
            if jump > self.POSE_JUMP_MM:
                logger.critical(
                    f"⚠️ 检测到位姿跳变 {jump:.0f}mm（正常每帧 ≤20mm）→ 判定被搬动/"
                    f"强制分离，重置坐标系回出发区 {self._start_zone} 号")
                self._apply_start_pose()
                x, y, theta = self._pose
                # 复位本趟进度与保活状态，避免带着"上一处"的判断继续跑
                self._trip_gripped = False
                self._trip_released = False
                self._prev_current_id = None
                self._last_motion_pose = None
                self._keepalive_until = 0.0

        # ── 0. 检测"新一趟"（决策引擎选中了新目标 → 复位本趟进度标志）──
        cur = self._decision._current_target
        cur_id = cur.id if cur is not None else None
        if cur_id is not None and cur_id != self._prev_current_id:
            self._trip_gripped = False
            self._trip_released = False
            self._trip_released_valid = False
        self._prev_current_id = cur_id

        nav_arrived = self._navigation.is_arrived()
        grip_done = self._trip_gripped
        release_done = self._trip_released

        # ── 1. 感知（真机：读摄像头帧 → CVDetector；否则 frame=None → Mock）──
        frame = None
        if self._camera is not None:
            try:
                if hasattr(self._camera, "get_frame"):
                    # CameraReader：非阻塞取最新帧（拿不到就是 None，绝不等待，
                    # 避免摄像头未就绪/掉线时把 50Hz 主循环卡死）
                    frame = self._camera.get_frame()
                else:
                    # 兼容裸 cv2.VideoCapture
                    ok, frame = self._camera.read()
                    if not ok:
                        frame = None
            except Exception as e:
                logger.warning(f"读取摄像头失败: {e}")
                frame = None
        # 必须把航向一起传下去：车体系→场地系要按 theta 旋转（否则目标定位随车头方向整体错位）
        self._perception.update(frame=frame, robot_position=(x, y), robot_theta=theta)

        # ── 1b. 对方接触时长（T1-11）──
        # 旧实现从不上报：`perception_pipeline` 明明算出了 `contact_duration_s`（还带 7s 预警），
        # 但结果只进 `get_stats()`，不进决策 → 赛规"接触超 10 秒 → 强制分离"这条
        # 在软件里**完全无人知晓**，被顶住时也不会主动脱离。
        contact_s = 0.0
        try:
            contact_s = float(self._perception.opponent_tracker.contact_duration_s)
        except Exception:
            contact_s = 0.0

        # T2-4：把"视觉是否可用"报给异常链（只在**真的不可用**时才报，
        # Mock/仿真模式 `vision_available` 恒 True → 不影响仿真基线）。
        sensor_status = None
        try:
            if not self._perception.vision_available:
                sensor_status = {"camera": False}
        except Exception:
            sensor_status = None
        if contact_s > 0:
            logger.warning(f"⚠️ 与对方持续接触 {contact_s:.1f}s"
                           f"（≥10s 裁判会强制分离）→ 上报决策层尝试脱离")

        # ── 2. 决策（把由位姿差算出的实际速度传下去，供内置异常链判定"是否在动"）──
        action = self._decision.update(
            (x, y, theta),
            nav_arrived=nav_arrived,
            grip_done=grip_done,
            release_done=release_done,
            velocity=self._last_velocity,
            release_valid=self._trip_released_valid,
            # S-40：本趟**真正**送达的目标 id（计划 ≠ 实际装载）。
            # 没套上的目标必须留在场上等下一趟，不能标记成已入安全区。
            delivered_ids=self._transport.delivered_target_ids,
            # T0-2：把上一帧下发速度给异常链，卡死检测才有"要求它动"这一路信息。
            # 用上一帧而非本帧：本帧的 cmd 要到第 6 步导航之后才算得出来。
            commanded_velocity=self._last_cmd,
            # T1-11：接触时长（赛规"接触>10s → 裁判强制分离"的唯一输入）
            contact_duration_s=contact_s,
            sensor_status=sensor_status,
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
                    self._trip_released = False
                    self._trip_released_valid = False
        elif action.type == ActionType.TRANSPORT_TO:
            self._set_nav_target(action.target_position)
        elif action.type == ActionType.EMERGENCY_STOP:
            # T1-9：**先停车再切状态**。旧实现只切状态、不发任何底盘指令 →
            # 下位机继续执行最后一帧 VEL，要等它自己的 800ms 看门狗才停
            # （850mm/s 下最多再冲 ~0.68m）。注意用 VEL,0,0 + STOP 而**不是** ESTOP：
            # 协议 §4.4 明确 "ESTOP 锁定后 START 不能解除锁定" → 会把板子锁死，
            # 当天后续场次必须断电重启。ESTOP 仅保留为人工兜底手段。
            self._stop_chassis()
            self._sm.emergency_stop("决策引擎请求急停")
            return

        # ── 4. 导航（内部积分定位器位姿）──
        # U4：必须把"是否接近减速带"传下去。旧实现从不传 → `near_speed_bump` 恒 False
        # → 导航永远不会 enter_bump_mode()，出发区前那 3 条减速带（50mm 高）
        # 是**全速直冲**过去的：轻则颠簸丢定位、重则卡住/骑上减速带。
        # T1-12：把对方位置喂给导航的**动态避障**。旧实现生产代码从不传
        # `opponent_position`（且对手代价档 200 < 触发阈值 255，双重失效）→
        # 避障实际为 0，正对对方会直接撞上去。
        opponent_pos = None
        try:
            opponent_pos = self._perception.opponent_position
        except Exception:
            opponent_pos = None
        cmd = self._navigation.update((x, y, theta),
                                      opponent_position=opponent_pos,
                                      near_speed_bump=self._is_near_speed_bump(x, y),
                                      dt=dt)

        # 记下本帧下发速度（下一帧喂给卡死检测）
        self._last_cmd = (float(cmd.linear), 0.0)

        # ── 5. 底盘执行：串口下发（真机）或 controller（占位）──
        if self._chassis is not None:
            self._chassis.send_velocity(cmd.linear, cmd.angular)
        elif self._controller is not None:
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
            # 投放有效性来自转运管线的**落点判定**（LoadManager.release_all 回填）
            self._trip_released_valid = bool(
                self._transport.load_manager.last_release_valid)

        # ── 9. 看门狗：按【里程计实际位移】判定是否在动（不是看下发速度）──
        self._update_watchdog((x, y), cmd, dt=dt)

    #: 保活（探索/绕圈）优先窗口时长（秒）。窗口内忽略决策引擎的目标（T1-10）。
    KEEPALIVE_HOLD_S = 4.0
    #: 位姿跳变阈值（mm）：单帧位移超过它就判定"车被搬动/强制分离"（T1-11）。
    #: 正常行驶 50Hz 下每帧最多约 17mm（850mm/s × 0.02s），400mm 留足余量。
    POSE_JUMP_MM = 400.0

    def _set_nav_target(self, pos) -> None:
        """设置导航目标（去重，避免每帧重复触发重规划）。"""
        if pos is None:
            return
        # T1-10：保活窗口内不让决策引擎覆盖保活点（否则探索/绕圈只存活一帧，
        # 机器人继续朝原目标硬顶，保活链等于没有）。
        if time.time() < self._keepalive_until:
            logger.debug("保活窗口内 → 暂不接受决策引擎的新导航目标")
            return
        if self._navigation.target != pos:
            # set_target 现在会在目标越界时**拒绝**并返回 False（S-NEW）：
            # 保持原目标不动、不报错，由调用方下一帧重试。这里记一次计数便于现场排查。
            if not self._navigation.set_target(pos[0], pos[1]):
                logger.warning(f"导航目标被拒（越界）：{pos}，保持当前目标")

    def _is_near_speed_bump(self, x: float, y: float) -> bool:
        """车心是否已接近任一减速带（U4）。

        赛项：出发区外有 3 条减速带（`SPEED_BUMP_COUNT=3`，50mm 高），
        必须降速/越障通过。距离按"点到矩形的最近距离"算（在矩形内为 0），
        这样任何一条减速带被触发都能让导航进入越障模式。
        """
        try:
            # ⚠️ 用 `.elements` 直接过滤：`FieldLayout`（WorldMap 使用的简化版）是
            # dataclass，只有 elements 列表，没有 StandardFieldLayout 的查询方法。
            elems = [e for e in self._field.elements
                     if e.type == FieldElementType.SPEED_BUMP]
        except Exception as e:      # 场地布局异常不该让主循环挂掉
            logger.warning(f"读取减速带元素失败: {e}")
            return False
        for e in elems:
            r = e.region
            dx = max(r.x - x, 0.0, x - (r.x + r.width))
            dy = max(r.y - y, 0.0, y - (r.y + r.height))
            if (dx * dx + dy * dy) ** 0.5 < self.SPEED_BUMP_TRIGGER_MM:
                return True
        return False

    def _finish_match(self, reason: str) -> None:
        """终场收口：清导航目标 → 显式停车 → 退出主循环（幂等，只做一次）。"""
        if not self._finished_logged:
            self._finished_logged = True
            logger.info(f"🏁 比赛结束（{reason}）→ 清导航目标 + 停车 + 退出主循环")
            try:
                self._navigation.clear_target()
            except Exception as e:
                logger.warning(f"清导航目标失败: {e}")
            self._stop_chassis()
        self._stop_event.set()

    def _on_release_failed(self, targets) -> None:
        """投放被判无效 / 本趟作废 → 把目标状态放回 ACTIVE（T1-5/T1-6/T1-7）。

        为什么必须做：目标若一直停在 `BEING_TRANSPORTED`，`world_map` 的
        `get_regular_supplies()/get_core_supplies()/get_injured()` 会把它永久过滤掉
        → 那个 5/10/15 分永久丢掉；若它是场上最后的普通物资，**首趟永远无法重做**，
        按赛规"首趟未完成则后续全部无效"= 整场 0 分。
        """
        for t in targets or []:
            try:
                self._perception.world_map.return_to_field(t.id)
            except Exception as e:
                logger.warning(f"把目标 #{getattr(t, 'id', '?')} 放回场上失败: {e}")

    def _stop_chassis(self) -> None:
        """
        显式停车（供转运管线在套取前调用）。

        先发零速度让速度环目标归零，再发 STOP 让下位机立即停速度环。
        不这样做的话：套取阶段主循环被阻塞、没人发 VEL，
        下位机要等速度看门狗（300ms 保持 / 800ms 停）才停 —— 期间底盘仍在
        执行最后一帧速度指令，可能前冲几十厘米把目标撞飞。
        """
        if self._chassis is None:
            return
        try:
            self._chassis.send_velocity(0.0, 0.0)
            self._chassis.send_stop()
            logger.info("🛑 显式停车（套取前）")
        except Exception as e:
            logger.warning(f"显式停车失败: {e}")

    # ---- 看门狗 ----

    def _update_watchdog(self, pose_xy, cmd, dt: float = 0.02,
                         now: Optional[float] = None) -> float:
        """
        看门狗：按**里程计实际位移**判定"是否在动"，并做分级降级保活。

        为什么不能用"下发速度 cmd"当判据（旧实现的致命缺陷）：
          车轮卡在围栏/减速带上打滑空转时，指令一直是几十~几百 mm/s，
          `abs(cmd.linear) > 10` 恒成立 → 保活计时被无限刷新 →
          机器人在原地耗到比赛时间结束（整场 0 分，且日志里什么都看不到）。
        改为：只有下位机里程计位置**真的变了**（累计位移 ≥ WATCHDOG_MOVE_MM）才算动。

        Args:
            pose_xy: 本帧位姿 (x, y)（真机来自串口 ODOM，Mock 来自导航定位器）
            cmd: 本帧下发的速度指令（仅用于"打滑告警"诊断，不用于判定是否在动）
            dt: 帧间隔（秒），用于由位移差算实际速度
            now: 注入时钟（便于测试；None → time.time()）

        Returns:
            当前"无位移"持续时长（秒），便于上层/测试观察
        """
        if now is None:
            now = time.time()

        x, y = float(pose_xy[0]), float(pose_xy[1])

        if self._last_motion_pose is None:
            self._last_motion_pose = (x, y)

        step_mm = math.hypot(x - self._last_motion_pose[0],
                             y - self._last_motion_pose[1])
        self._last_motion_pose = (x, y)

        # 实际速度（mm/s）：供决策引擎内置异常链使用
        if dt > 0:
            self._last_velocity = (step_mm / dt, 0.0)

        # 累计位移：滤掉里程计抖动（小于 WATCHDOG_STEP_MIN_MM 的单帧不计）
        if step_mm >= self.WATCHDOG_STEP_MIN_MM:
            self._motion_accum_mm += step_mm

        if self._motion_accum_mm >= self.WATCHDOG_MOVE_MM:
            # 确实在动 → 刷新保活计时、解除降级
            self._motion_accum_mm = 0.0
            self._last_motion_time = now
            self._last_action_time = now
            self._explore_triggered = False
            self._survival_triggered = False
            self._stuck_warned = False
            return 0.0

        idle_duration = now - self._last_motion_time

        # 打滑/堵转诊断：在发速度却不动 → 提前告警（现场一眼看出是哪一类故障）
        commanded = abs(cmd.linear) > 10.0 or abs(cmd.angular) > 0.01
        if commanded and idle_duration > self.STUCK_WARN_S and not self._stuck_warned:
            self._stuck_warned = True
            logger.error(
                f"⚠️ 已下发速度(cmd v={cmd.linear:.0f}mm/s)但 {idle_duration:.1f}s 无位移 "
                f"→ 疑似打滑/堵转/编码器失效（位置=({x:.0f},{y:.0f})）"
            )

        # 套取/投放阶段本来就要车静止，属合法静止 → 不打断（否则看门狗会
        # 在套取过程中把导航目标改成"探索点"，与转运抢方向盘）
        if self._transport.phase in (TransportPhase.CAPTURING,
                                     TransportPhase.PLACING):
            return idle_duration

        self._check_watchdog(idle_duration)
        return idle_duration

    def _check_watchdog(self, idle_duration: Optional[float] = None) -> None:
        """
        分级看门狗保活（对齐 MEMORY 四层降级保活链）：

          无位移 > 10s  → 探索模式（驶向场地随机位置边移动边扫描）
          无位移 > 13s  → 保命绕圈（低速绕圈，保持运动）
          无位移 > 15s  → 不再淘汰，持续保命运动（保持比赛资格）

        不再因为 15 秒无动作而 emergency_stop。
        """
        if idle_duration is None:
            idle_duration = time.time() - self._last_motion_time

        if idle_duration > self.WATCHDOG_SURVIVAL_S and not self._survival_triggered:
            self._survival_triggered = True
            self._navigation.survival_circle()
            # T1-10：开保活优先窗口，否则决策引擎下一帧就把这个点覆盖掉
            self._keepalive_until = time.time() + self.KEEPALIVE_HOLD_S
            logger.warning(f"🛟 保命绕圈：{idle_duration:.1f}s 无位移"
                           f"（保活优先 {self.KEEPALIVE_HOLD_S:.0f}s）")
        elif idle_duration > self.WATCHDOG_EXPLORE_S and not self._explore_triggered:
            self._explore_triggered = True
            self._navigation.explore()
            self._keepalive_until = time.time() + self.KEEPALIVE_HOLD_S
            logger.warning(f"🔍 探索模式：{idle_duration:.1f}s 无位移"
                           f"（保活优先 {self.KEEPALIVE_HOLD_S:.0f}s）")

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
