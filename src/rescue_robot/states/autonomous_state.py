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
from ..perception.field_elements import FieldLayout, SafeZoneColor
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

        # 看门狗
        self._last_action_time = time.time()
        self._explore_triggered = False
        self._survival_triggered = False
        self._last_action: Optional[Action] = None
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
        if self._chassis is not None:
            if not self._chassis.is_open:
                self._chassis.open()
            if self._chassis.is_open:
                ok = self._chassis.start_match()
                logger.info(f"底盘连接+启动: {'✅ 成功' if ok else '⚠️ 失败'}")
            else:
                logger.warning("串口底盘打开失败，将无法下发速度/读取位姿")

        # 延迟启动（让裁判离开场地）
        logger.info(f"等待 {timing.POST_START_DELAY_MS}ms 后开始运行...")
        time.sleep(timing.POST_START_DELAY_MS / 1000)

        # 启动主循环
        self._stop_event.clear()
        self._last_action_time = time.time()
        self._explore_triggered = False
        self._survival_triggered = False
        self._finished_logged = False   # 终场停车标志复位（S-01）
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
        # ── 终场停车（S-01）──
        # 旧实现：主循环只认 `_stop_event`（状态切换/Ctrl+C），**没有任何"比赛结束"处理**。
        # 时间到（或目标全清）后决策引擎已经返回 WAIT，但导航里还留着最后一个目标、
        # 主循环仍以 50Hz 下发上一帧速度 → 机器人继续朝场地里冲，撞紫边/撞对手/
        # 把已投放的目标撞出安全区（现场直接判罚）。规则要求终场立即停止。
        # 这里把"决策引擎进入 DONE"当作终场信号：清导航目标 + 显式停车 + 退出主循环。
        if self._decision.strategy_state == StrategyState.DONE:
            if not self._finished_logged:
                self._finished_logged = True
                logger.info("🏁 比赛结束（决策引擎 DONE）→ 清导航目标 + 停车")
                try:
                    self._navigation.clear_target()
                except Exception as e:
                    logger.warning(f"清导航目标失败: {e}")
                self._stop_chassis()
            self._stop_event.set()
            return

        # ── 位姿来源：优先串口（下位机里程计），否则用导航定位器 ──
        if self._chassis is not None:
            pose_data = self._chassis.read_pose()
            if pose_data is not None:
                self._pose = pose_data
            x, y, theta = self._pose
        else:
            pose = self._navigation.pose
            x, y, theta = pose.x, pose.y, pose.theta

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
            self._sm.emergency_stop("决策引擎请求急停")
            return

        # ── 4. 导航（内部积分定位器位姿）──
        cmd = self._navigation.update((x, y, theta), dt=dt)

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

    def _set_nav_target(self, pos) -> None:
        """设置导航目标（去重，避免每帧重复触发重规划）。"""
        if pos is None:
            return
        if self._navigation.target != pos:
            self._navigation.set_target(pos[0], pos[1])

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
            logger.warning(f"🛟 保命绕圈：{idle_duration:.1f}s 无位移")
        elif idle_duration > self.WATCHDOG_EXPLORE_S and not self._explore_triggered:
            self._explore_triggered = True
            self._navigation.explore()
            logger.warning(f"🔍 探索模式：{idle_duration:.1f}s 无位移")

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
