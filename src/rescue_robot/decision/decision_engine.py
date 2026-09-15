"""
decision_engine.py —— 决策引擎

比赛状态机——机器人自主运行的"大脑"。
集成感知、导航、转运，执行完整的比赛策略。

状态转移：
  FIRST_TRIP ──(1普通物资已入物资区)──→ FREE_RUN
  FREE_RUN ──(剩余<30s)──→ TIME_PRESSURE
  FREE_RUN ──(目标全清/时间到)──→ DONE
  ANY ──(异常)──→ ANOMALY
  ANY ──(强制分离)──→ FORCED_RESET

Action 类型（输出到 autonomous loop）:
  NAVIGATE_TO(target)  — 导航到目标
  GRIP(targets)        — 夹取目标
  TRANSPORT_TO(safe)   — 运送至安全区
  RELEASE              — 投放
  RECOVER(pose)        — 恢复/重定位
  WAIT                 — 等待
  EMERGENCY_STOP       — 急停
"""

import logging
import math
import random
import time
from dataclasses import dataclass, field
from enum import Enum, auto
from typing import Dict, List, Optional, Set, Tuple

from .target_selector import TargetSelector, StrategyState, ScoredTarget
from .anomaly_handler import (
    AnomalyHandler, AnomalyType, AnomalyReport, RecoveryAction,
)

class FallbackLevel:
    """降级层级"""
    NORMAL = 0        # 正常工作
    RETRY = 1         # 重试（换参数）
    SWITCH_TARGET = 2 # 换目标
    EXPLORE = 3       # 探索扫描
    SURVIVAL = 4      # 保命绕圈
from ..perception.target_types import TargetType, TargetInfo, get_point_value
from ..perception.world_map import WorldMap, TrackedTarget, TargetStatus
from ..perception.field_elements import SafeZoneColor, FieldLayout

logger = logging.getLogger("decision_engine")


# ============================================================
# Action（输出到 autonomous loop）
# ============================================================

class ActionType(Enum):
    NAVIGATE_TO = auto()       # 导航到目标点
    GRIP = auto()              # 夹取目标
    TRANSPORT_TO = auto()      # 运送至安全区
    RELEASE = auto()           # 释放投放
    RECOVER = auto()           # 恢复/重定位
    WAIT = auto()              # 等待
    EMERGENCY_STOP = auto()    # 急停


@dataclass
class Action:
    """决策引擎输出的动作"""
    type: ActionType
    target_position: Optional[Tuple[float, float]] = None
    target_ids: List[int] = field(default_factory=list)
    detail: str = ""


# ============================================================
# 决策引擎
# ============================================================

class DecisionEngine:
    """
    决策引擎——比赛策略执行者。

    使用方式（在 autonomous loop 中）：
      engine = DecisionEngine(world_map, my_color=RED)
      engine.start_match()

      # 每帧
      action = engine.update(robot_pose, timestamp)

      # 执行 action
      if action.type == NAVIGATE_TO:
          nav.set_target(*action.target_position)
      elif action.type == GRIP:
          transport.start_trip(...)
      ...
    """

    # 比赛参数
    MATCH_DURATION_S = 180.0     # 3 分钟
    TIME_PRESSURE_S = 30.0       # 剩余 < 30s → 时间紧迫

    # 动作超时
    NAV_TIMEOUT_S = 10.0         # 导航超时
    GRIP_TIMEOUT_S = 3.0         # 夹取超时
    #: ANOMALY 最长停留时间（秒）：超过则**强制恢复**原策略状态（T0-3 兜底）。
    #: 为什么需要：ANOMALY 一旦成为"进得去出不来"的状态，决策层就永久只吐 WAIT，
    #: 整场报废。宁可恢复后继续跑（并打 ERROR 让人看见），也不要永久停滞。
    ANOMALY_MAX_S = 8.0
    #: 判定"导航有进展"的距离阈值（mm）：离目标比历史最好值又近了这么多才算推进。
    NAV_PROGRESS_MM = 20.0
    #: 距目标多近就"不判超时"（mm）：套取半径 150mm 由转运层判定，这里留足余量。
    NAV_NEAR_MM = 300.0
    #: 超时窗口内**总位移**小于此值才算"几乎没动"（mm）。两条同时成立才放弃目标。
    NAV_STUCK_MOVE_MM = 200.0
    #: 被判"不可达"的目标在这个时间内不再被选中（秒），期间改为探索其它区域。
    ABANDON_COOLDOWN_S = 30.0
    #: 探索目标保持时间（秒）：期间复用同一个点，避免导航目标每帧跳变。
    EXPLORE_HOLD_S = 8.0
    TRANSPORT_TIMEOUT_S = 15.0   # 运送超时

    def __init__(self,
                 world_map: WorldMap,
                 my_color: SafeZoneColor = SafeZoneColor.RED):
        self._world_map = world_map
        self._my_color = my_color

        # 子模块
        self._selector = TargetSelector()
        self._anomaly = AnomalyHandler()

        # 状态
        self._strategy_state = StrategyState.FIRST_TRIP
        self._action_phase = ActionType.WAIT  # 当前动作阶段
        self._action_start_time = 0.0

        # 比赛计时
        self._match_start_time = 0.0
        self._match_elapsed = 0.0

        # 当前目标
        self._current_target: Optional[TrackedTarget] = None
        self._pose: Tuple[float, float, float] = (150.0, 150.0, 0.0)  # 本帧位姿（保命/探索用）
        self._trip_targets: List[TrackedTarget] = []  # 本趟转运的所有目标（普通+核心可混合 ≤3）
        self._transporting = False

        # 降级系统
        self._fallback_level = FallbackLevel.NORMAL
        self._fallback_retries = 0
        self._max_retries = 3
        self._last_action_time = time.time()
        self._explore_waypoints = []

        # 统计
        self._trips_completed = 0
        self._targets_delivered = 0
        # 本队**已送达**的目标 id：用于识别"投放被判无效、目标被裁判重放场心"
        # （不能拿"任意目标靠近场心"当判据，见 _check_invalid_transport）
        self._delivered_ids: Set[int] = set()
        self._score = 0
        # "场上暂无活跃目标"的告警只打一次（避免开局/掉线时刷屏；见 update() 的终场判定）
        self._empty_map_warned = False
        # T0-3：ANOMALY 的"原策略状态"与进入时刻（异常消失后归还 / 超时强制恢复）
        self._anomaly_prev_state: Optional[StrategyState] = None
        self._anomaly_since: Optional[float] = None
        self._forced_prev_state: Optional[StrategyState] = None   # T2-6
        # T1-8：导航"进展感知"超时用——记录选目标以来的**最近进展时刻**与最好距离。
        # ⚠️ 不能用绝对超时：实测当前巡航速度只有 ~163mm/s，一趟 2.85m 要 22.3s，
        # 绝对超时（NAV_TIMEOUT_S=10）会把**正常但慢**的导航全部误弃。
        self._nav_best_dist: Optional[float] = None
        self._nav_last_progress_ts: Optional[float] = None
        # T1-8：已判定"到不了"的目标 id + 冷却截止时刻。必须记住它，否则同一帧就会
        # 把它重新选中（实测：放弃后立刻重选同一个点 → 10 秒一次的"放弃→重选"空转）。
        self._nav_prev_pose: Optional[tuple] = None
        self._nav_move_accum: float = 0.0
        self._abandoned_ids: set = set()
        self._abandon_until: float = 0.0
        # 探索点缓存：`_get_explore_target()` 每次调用都是**随机点**，
        # 若每帧都重新取，导航目标会每帧跳变 → 车原地抖动、永远走不到。
        self._explore_target: Optional[tuple] = None
        self._explore_until: float = 0.0

        logger.info(f"DecisionEngine 初始化: my_color={my_color.name}")

    # ---- 属性 ----

    @property
    def strategy_state(self) -> StrategyState:
        return self._strategy_state

    @property
    def time_remaining_s(self) -> float:
        return max(0, self.MATCH_DURATION_S - self._match_elapsed)

    @property
    def is_time_pressure(self) -> bool:
        return self.time_remaining_s < self.TIME_PRESSURE_S

    @property
    def score(self) -> int:
        return self._score

    @property
    def targets_delivered(self) -> int:
        return self._targets_delivered

    # ---- 比赛控制 ----

    def start_match(self) -> None:
        """开始比赛计时"""
        self._match_start_time = time.time()
        self._match_elapsed = 0.0
        self._strategy_state = StrategyState.FIRST_TRIP
        self._action_phase = ActionType.WAIT
        self._anomaly_prev_state = None      # T0-3
        self._anomaly_since = None
        self._nav_best_dist = None           # T1-8
        self._nav_last_progress_ts = None
        self._nav_prev_pose = None
        self._nav_move_accum = 0.0
        self._abandoned_ids = set()
        self._abandon_until = 0.0
        self._explore_target = None
        self._explore_until = 0.0
        logger.info("🏁 比赛开始! 状态=FIRST_TRIP")

    # ---- 主决策循环 ----

    def update(self,
               robot_pose: Tuple[float, float, float],
               nav_arrived: bool = False,
               grip_done: bool = False,
               release_done: bool = False,
               imu_data: Optional[dict] = None,
               contact_duration_s: float = 0.0,
               timestamp: Optional[float] = None,
               velocity: Optional[Tuple[float, float]] = None,
               release_valid: Optional[bool] = None,
               delivered_ids: Optional[List[int]] = None,
               commanded_velocity: Optional[Tuple[float, float]] = None,
               sensor_status: Optional[Dict[str, bool]] = None) -> Action:
        """
        单帧决策。

        Args:
            robot_pose: (x, y, theta) 机器人位姿
            nav_arrived: 导航是否已到达目标
            grip_done: 夹取是否完成
            release_done: 投放是否完成
            imu_data: IMU 数据（用于异常检测）
            contact_duration_s: 对方接触时长
            timestamp: 时间戳
            release_valid: 最近一次投放是否为**有效投放**（目标落点在正确的
                物资区/伤员区围栏内侧）。首趟只有有效才允许进入 FREE_RUN。
                None = 旧行为（视为有效）——便于自测/旧调用方零改动接入。
            velocity: 由里程计位姿差算出的**实际**速度 (vx, vy) mm/s。
                None = 上游没有运动学信息（仿真/单测）→ 内置"无动作"检测不启用；
                真机必须传，否则异常链形同虚设。
            commanded_velocity: 本帧**下发**的速度 (vx, vy) mm/s。
                卡死检测需要"下发速度 vs 里程计位移"两路独立信息（见 anomaly_handler
                的 T0-2）：只传 velocity（实际速度）时无法区分"真的卡住"与"正常行驶"。
                None → 跳过卡死检测。
            delivered_ids: 上一趟**真正**送达的目标 id（来自
                `TransportPipeline.delivered_target_ids`）。本趟**计划**可能多于
                套取机构的物理容量（本车 = 1），没套上的目标必须留在场上等下一趟，
                绝不能标记成"已入安全区"（标了就永不重选 = 永久丢分）。
                None = 上游没提供 → 按旧行为（视为本趟计划全部送达）。

        Returns:
            Action: 要执行的动作
        """
        if timestamp is None:
            timestamp = time.time()

        self._match_elapsed = timestamp - self._match_start_time
        self._pose = robot_pose
        rx, ry, rtheta = robot_pose

        # ─── 异常检测 ───
        # ⚠️ 旧实现固定传字面量 (0, 0) 当速度 → 内置"15s 无动作/卡死"判定**永不触发**；
        # 同时下面那句无条件 `notify_action()` 每帧刷新保活计时，把内置看门狗彻底架空。
        # 现在：由上游传真实速度；只有显式传了 velocity 才启用内置无动作检测。
        anomaly = self._anomaly.check(
            robot_pose, velocity,
            imu_data, contact_duration_s,
            commanded_velocity=commanded_velocity,
            # T2-4：传感器健康状况（摄像头/IMU）。旧实现从不传 → `_sensor_status`
            # 恒为全 True → `SENSOR_FAULT_CAMERA/IMU` 两个分支**永不触发**，
            # 摄像头掉线在决策层"无人知晓"。
            sensor_status=sensor_status,
        )
        if anomaly.type != AnomalyType.NONE:
            # T0-3：进入 ANOMALY 前**记住原策略状态**，异常消失后归还。
            # 旧实现只置 ANOMALY、从不还原 → 异常过后分派链全不匹配、落到
            # `else: WAIT "未知状态"` **永久**（实测：异常消失 50 帧后仍 ANOMALY+WAIT），
            # 叠加误判卡死（T0-2）就是"开车约 5 秒后整场报废"。
            if self._strategy_state != StrategyState.ANOMALY:
                self._anomaly_prev_state = self._strategy_state
                self._anomaly_since = timestamp
            self._strategy_state = StrategyState.ANOMALY
            # T0-3 兜底：ANOMALY 停留过久 → 强制恢复（否则可能永久只吐 WAIT）
            if (self._anomaly_since is not None
                    and timestamp - self._anomaly_since > self.ANOMALY_MAX_S):
                restored = (StrategyState.FIRST_TRIP
                            if self._strategy_state == StrategyState.FIRST_TRIP
                            else StrategyState.FREE_RUN)
                logger.error(
                    f"⚠️ ANOMALY 已持续 {timestamp - self._anomaly_since:.0f}s"
                    f"（>{self.ANOMALY_MAX_S:.0f}s）→ 强制恢复策略状态 {restored.name}，"
                    f"避免决策层永久停滞（本次异常={anomaly.type.name}）")
                self._strategy_state = restored
                self._anomaly_prev_state = None
                self._anomaly_since = None
            return self._handle_anomaly(anomaly, robot_pose)

        # ── 异常已消失：从 ANOMALY 恢复（T0-3）──
        if self._strategy_state == StrategyState.ANOMALY:
            restored = self._anomaly_prev_state
            if restored in (None, StrategyState.ANOMALY, StrategyState.DONE):
                # 兜底：首趟未完成则回首趟，否则回 FREE_RUN。
                # **不能**一律回 FREE_RUN —— 首趟未完成时那样会绕过"必须先单独送
                # 1 个普通物资"的硬规则（按赛规整场无效）。
                restored = (StrategyState.FIRST_TRIP
                            if self._strategy_state == StrategyState.FIRST_TRIP
                            else StrategyState.FREE_RUN)
            logger.warning(f"✅ 异常已解除 → 恢复策略状态 {restored.name}"
                           f"（不再停留在 ANOMALY）")
            self._strategy_state = restored
            self._anomaly_prev_state = None
            self._anomaly_since = None
        elif self._anomaly_since is not None:
            self._anomaly_since = None

        # velocity 为 None（无运动学信息）时，保持"不启用内置无动作检测"的旧行为，
        # 避免在拿不到速度反馈的场景里误判；真机路径由 check() 内部按 speed>10 自行刷新。
        if velocity is None:
            self._anomaly.notify_action()

        # ─── 时间管理 ───
        if self.is_time_pressure and \
           self._strategy_state == StrategyState.FREE_RUN:
            self._strategy_state = StrategyState.TIME_PRESSURE
            logger.info(f"⏰ 时间紧迫! 剩余 {self.time_remaining_s:.0f}s")

        # ─── 检查比赛结束 ───
        if self.time_remaining_s <= 0:
            self._strategy_state = StrategyState.DONE
            logger.info("比赛时间到!")
            return Action(type=ActionType.WAIT, detail="比赛结束")

        if not self._world_map.active_targets and not (grip_done or release_done):
            # ⚠️⚠️ 这里**绝不能**判终场（DONE）—— 这是 S-01 修复暴露出来的 blocker。
            #
            # "当前帧检测列表为空" ≠ "场上没有可救援目标"。下列**常见**情况都会让
            # `active_targets` 瞬间变空，而它们全都不是比赛结束：
            #   1. 开局头几帧：感知要先攒够命中才建跟踪目标 → 地图本来就是空的；
            #   2. 摄像头掉线/整帧无检测：`world_map.update([])` 每帧给所有目标
            #      `track_lost_count + 1`，3 秒（MAX_LOST_COUNT=150）后**全部删除**；
            #   3. 目标刚被套住 → 真机 `autonomous_state` 会把它标成 BEING_TRANSPORTED；
            #   4. 刚投放成功 → 标成 IN_SAFE_ZONE。
            #
            # 旧实现据此置 DONE，而 **DONE 是单向的**（`grep '_strategy_state = '` 显示
            # 没有任何分支能从 DONE 回到 FREE_RUN）。配合终场分支（清导航目标 + 停车 +
            # `_stop_event.set()` 退出主循环）= **整场提前结束、不可恢复**。
            # 最坏情况：开局第一帧地图为空 → t≈0 就停车退赛（0 分）。
            #
            # 赛规的结束条件（"救援目标被全部移至安全区内"）必须按**账面**判定，
            # 不能按"这一帧的检测列表"判定 —— 后者受遮挡/掉线/光照影响极大。
            # 因此：**只把"时间到"当硬终场**；场上暂无目标 → 保持当前策略、原地等待
            # 重新检测（与修复前的行为一致，但不退出主循环、不置 DONE）。
            #
            # ⚠️⚠️ 但**必须**用 `grip_done/release_done` 门控（T0-1）：
            # 目标一旦被套住就会被真机标成 BEING_TRANSPORTED，**不再计入 active_targets**。
            # 若它是场上最后一个 ACTIVE 目标（开局只识别到 1 个，或清场时最后一趟），
            # 早退 WAIT 会让决策层**永远到不了 TRANSPORT_TO** → 车带着目标原地不动、
            # 一趟都送不进去（开局只建 1 个目标时 → 首趟永不完成 → 按赛规整场无效）。
            # 手上/车上有货时**必须**继续走"运送→投放"分支。
            if not self._empty_map_warned:
                self._empty_map_warned = True
                logger.warning(
                    "场上暂无活跃目标 → 保持运行、原地等待重新检测"
                    "（**不判终场**：摄像头掉线/开局未建图/目标在车上都会造成这种情况）")
            return Action(type=ActionType.WAIT,
                          detail="场上暂无目标：保持运行，等待重新检测")

        # ─── T1-8：导航"不进展"超时 → 放弃当前目标（否则可整场卡在一个到不了的点上）──
        # 旧实现：NAV_TIMEOUT_S 只在类里声明 + 被 config 赋值，**零引用** → 没有任何超时，
        # 一旦当前目标不可达，决策层就会一直朝它发 NAVIGATE_TO（实测 300 帧全是
        # `NAVIGATE_TO (3200,1500)`）。现在按"**离目标越来越近**"判进展：
        # 连续 NAV_TIMEOUT_S 秒没有实质推进（≥ NAV_PROGRESS_MM）就放弃该目标重选。
        # ⚠️ 判据必须是"**既不接近、又几乎没动**"两条同时成立，否则会误弃正常目标：
        #    第一版只用"最近距离没再改进"就放弃，实测仿真里在 96mm/115mm/640mm 处
        #    被大量误弃（车其实一直在正常机动，只是短时间内没有新的 20mm 改进）
        #    → 集成仿真从 80 分/7 个掉到 21 分/3 个。**超时必须保守**。
        if self._nav_prev_pose is not None:
            self._nav_move_accum += math.hypot(rx - self._nav_prev_pose[0],
                                               ry - self._nav_prev_pose[1])
        self._nav_prev_pose = (rx, ry)

        if self._current_target is not None and not nav_arrived:
            tgt = self._current_target.position
            d = math.hypot(tgt[0] - rx, tgt[1] - ry)
            if d <= self.NAV_NEAR_MM:
                # 已经在目标附近（套取半径由转运层判定）→ 不判超时，交给转运层
                self._nav_best_dist = d
                self._nav_last_progress_ts = timestamp
                self._nav_move_accum = 0.0
            elif (self._nav_best_dist is None
                    or d < self._nav_best_dist - self.NAV_PROGRESS_MM):
                self._nav_best_dist = d
                self._nav_last_progress_ts = timestamp
                self._nav_move_accum = 0.0
            elif (self._nav_last_progress_ts is not None
                  and timestamp - self._nav_last_progress_ts > self.NAV_TIMEOUT_S
                  and self._nav_move_accum < self.NAV_STUCK_MOVE_MM):
                bad_id = self._current_target.id
                logger.error(
                    f"⚠️ 导航 {timestamp - self._nav_last_progress_ts:.0f}s 无实质进展"
                    f"（最好距离仍 {self._nav_best_dist:.0f}mm，目标 "
                    f"({tgt[0]:.0f},{tgt[1]:.0f})）→ 放弃目标 #{bad_id}，"
                    f"冷却 {self.ABANDON_COOLDOWN_S:.0f}s 并先去探索")
                self._abandoned_ids.add(bad_id)
                self._abandon_until = timestamp + self.ABANDON_COOLDOWN_S
                self._current_target = None
                self._trip_targets = []
                self._nav_best_dist = None
                self._nav_last_progress_ts = None
                self._nav_move_accum = 0.0
                return Action(type=ActionType.NAVIGATE_TO,
                              target_position=self._explore_target_stable(rx, ry, timestamp),
                              detail=f"目标 #{bad_id} 不可达 → 探索其它区域")
        elif nav_arrived:
            self._nav_best_dist = None
            self._nav_last_progress_ts = None
            self._nav_move_accum = 0.0

        # T1-8：冷却期内**不再**朝"刚判不可达"的目标导航 —— 选择器可能又把它选回来
        # （例如它就是最近的那个），必须在这里拦掉，否则会变成"放弃→重选"空转。
        if (self._current_target is not None
                and self._current_target.id in self._abandoned_ids
                and timestamp < self._abandon_until):
            return Action(type=ActionType.NAVIGATE_TO,
                          target_position=self._explore_target_stable(rx, ry, timestamp),
                          detail=f"目标 #{self._current_target.id} 冷却中 → 探索其它区域")

        # ─── 状态机 ───
        if self._strategy_state == StrategyState.FIRST_TRIP:
            return self._handle_first_trip(rx, ry, nav_arrived, grip_done,
                                           release_done, release_valid,
                                           delivered_ids)
        elif self._strategy_state in (StrategyState.FREE_RUN,
                                       StrategyState.TIME_PRESSURE):
            return self._handle_free_run(rx, ry, nav_arrived, grip_done,
                                          release_done, release_valid,
                                          delivered_ids)
        elif self._strategy_state == StrategyState.FORCED_RESET:
            return self._handle_forced_reset()
        elif self._strategy_state == StrategyState.DONE:
            return Action(type=ActionType.WAIT, detail="比赛完成")
        else:
            # T0-3 安全网：任何"没人处理"的策略状态都不允许永久卡住决策层。
            # 旧实现直接 `WAIT "未知状态"`，一旦状态进入未知值（历史上 ANOMALY 就是）
            # 就永久不动、且日志里看不出来。现在恢复到一个可工作的状态并打 ERROR。
            restored = (StrategyState.FIRST_TRIP
                        if self._strategy_state == StrategyState.FIRST_TRIP
                        else StrategyState.FREE_RUN)
            logger.error(f"⚠️ 未知策略状态 {self._strategy_state.name} → "
                         f"恢复为 {restored.name}（避免决策层永久 WAIT）")
            self._strategy_state = restored
            return self._handle_free_run(rx, ry, nav_arrived, grip_done,
                                         release_done, release_valid,
                                         delivered_ids)

    # ---- FIRST_TRIP ----

    def _handle_first_trip(self, rx: float, ry: float,
                           nav_arrived: bool, grip_done: bool,
                           release_done: bool, release_valid: Optional[bool] = None,
                           delivered_ids: Optional[List[int]] = None) -> Action:
        """处理首次转运状态"""
        # 选择目标
        if self._current_target is None:
            self._current_target = self._selector.select_best_for_first_trip(
                self._world_map, (rx, ry)
            )
            if self._current_target is None:
                self._fallback_level = FallbackLevel.EXPLORE
                explore_pos = self._get_explore_target(rx, ry)
                logger.warning("FIRST_TRIP: 无普通物资 → 探索模式")
                return Action(type=ActionType.NAVIGATE_TO,
                              target_position=explore_pos,
                              detail="探索: 搜索普通物资")

            logger.info(f"FIRST_TRIP: 选择 {self._current_target.info.description} "
                        f"@ ({self._current_target.position[0]:.0f}, "
                        f"{self._current_target.position[1]:.0f})")
            return Action(
                type=ActionType.NAVIGATE_TO,
                target_position=self._current_target.position,
                detail=f"首次转运: 前往普通物资",
            )

        # 导航到目标（仅未夹取阶段需要到达目标）
        if not grip_done:
            if not nav_arrived:
                return Action(
                    type=ActionType.NAVIGATE_TO,
                    target_position=self._current_target.position,
                )
            # 夹取
            return Action(
                type=ActionType.GRIP,
                target_ids=[self._current_target.id],
                detail="FIRST_TRIP: 夹取普通物资",
            )

        # 运送至物资区
        safe_region = self._get_supply_area_position()
        if not release_done:
            return Action(
                type=ActionType.TRANSPORT_TO,
                target_position=safe_region,
                detail="FIRST_TRIP: 运送至物资区",
            )

        # ⚠️ N-8：漏传 `release_valid` 必须**失败关闭（fail-closed）**，不能静默放行。
        # 首趟闸门是"投歪则首趟不算完成、后续全部无效"这条赛规的唯一守护；
        # 若某调用点忘了传参就放行，等于把守护悄悄摘掉（现场表现为"投歪也算首趟成功"）。
        # 因此 None → `False`（判无效、重做首趟），并打 ERROR 让人看见。
        # ⚠️ 注意这与 `delivered_ids` 的 None 语义**故意不同**：那个 None=不丢目标（宽松），
        #    这个 None=判无效（严格）。合规闸门必须偏严格。
        if release_valid is None:
            logger.error("未提供 release_valid（投放有效性）→ 按**无效**处理（fail-closed）。"
                         "请让调用方显式传入投放判定结果；"
                         "真机路径 autonomous_state 与仿真 integrated_sim 都已显式传参。")
            release_valid = False

        # ── 首趟完成判定 ──
        # 规则：出发后必须把**一个普通物资单独送到本队安全区物资区围栏内侧**，
        # 之后才能转运其它目标；投歪/投在围栏外 = 首趟未完成，后续全部无效。
        # 旧实现只看 release_done 就切 FREE_RUN（投歪也当成功 → 整轮作废）。
        if not release_valid:
            logger.error("❌ 首趟投放无效（未落在本队物资区围栏内侧）→ "
                         "保持 FIRST_TRIP，重新选普通物资重做首趟")
            self._current_target = None
            self._trip_targets = []
            self._trips_completed += 1
            return Action(type=ActionType.WAIT, detail="首趟无效，重做首趟")

        # S-40 兜底：首趟目标必须真的在车上（正常情况必然成立，防"假成功"）
        if (delivered_ids is not None
                and self._current_target.id not in delivered_ids):
            logger.error(f"❌ 首趟目标#{self._current_target.id}实际未套上 → "
                         "首趟不算完成，重做首趟")
            self._current_target = None
            self._trip_targets = []
            self._trips_completed += 1
            return Action(type=ActionType.WAIT, detail="首趟未套上，重做首趟")

        self._world_map.mark_in_safe_zone(self._current_target.id)
        self._delivered_ids.add(self._current_target.id)
        self._targets_delivered += 1
        self._score += get_point_value(TargetType.REGULAR_SUPPLY)

        self._strategy_state = StrategyState.FREE_RUN
        self._current_target = None
        self._trips_completed += 1

        logger.info("✅ FIRST_TRIP 完成（已有效送达物资区围栏内侧）! 进入 FREE_RUN")
        return Action(type=ActionType.WAIT, detail="首次转运完成")

    # ---- FREE_RUN / TIME_PRESSURE ----

    def _check_invalid_transport(self, rx: float, ry: float) -> bool:
        """检测"本队刚投放的目标被裁判判为无效、重新放回场地中央"。

        ⚠️ 旧实现：遍历**所有** ACTIVE 目标，只要有任意一个距场心 <500mm 就返回 True。
        现场目标多（决赛 25 个）且规则明确"无效目标由裁判取出重放场地中央"，所以
        场心附近有目标是常态 → 该判据常态误触发。误触发的后果不是"重选得分目标"
        这么轻：调用点会把 `_current_target` 清空并**在运送途中**改发 NAVIGATE_TO，
        把导航目标从安全区抢到下一个目标 → 那一趟永远到不了投放点（车一直在动，
        也不会被卡死看门狗发现）→ 整场再也开不出新趟、一个都送不到。
        现在只认**"我们已经送达过、却又变回 ACTIVE 且出现在场心"的目标**，
        这才是规则里"无效投放被取出重放"的真实特征，且不会因无关目标误触发。
        """
        import math
        from rescue_robot.perception.target_types import TargetStatus
        if not self._delivered_ids:
            return False
        for tid in self._delivered_ids:
            t = self._world_map.targets.get(tid)
            if t is None or t.status != TargetStatus.ACTIVE:
                continue
            dist_to_center = math.sqrt(
                (t.position[0] - 1500) ** 2 + (t.position[1] - 1500) ** 2)
            if dist_to_center < 500:
                logger.warning(
                    f"本次投放被判无效：目标#{tid}已回到场心附近"
                    f"(距场心 {dist_to_center:.0f}mm) → 重新选择目标")
                return True
        return False

    def _handle_free_run(self, rx: float, ry: float,
                         nav_arrived: bool, grip_done: bool,
                         release_done: bool, release_valid: Optional[bool] = None,
                         delivered_ids: Optional[List[int]] = None) -> Action:
        """处理自由转运状态"""
        # 检测转运无效恢复
        # ⚠️ 只有**手上没有货**时才允许因为"目标无效"而重选：运送途中改导航目标
        #    会让本趟永远送不到（见 _check_invalid_transport 的说明）。
        if not grip_done and self._check_invalid_transport(rx, ry):
            self._current_target = None  # 重新选择目标

        # 选择目标（伤员单独转运；普通+核心可混合 ≤3）
        if self._current_target is None:
            self._trip_targets = self._selector.select_targets_for_trip(
                self._world_map, (rx, ry), max_count=3,
                include_injured=True, time_remaining_s=self.time_remaining_s,
            )
            if not self._trip_targets:
                if self._fallback_level < FallbackLevel.EXPLORE:
                    self._fallback_level = FallbackLevel.EXPLORE
                explore_pos = self._get_explore_target(rx, ry)
                logger.warning("FREE_RUN: 无可用目标 → 探索模式")
                return Action(type=ActionType.NAVIGATE_TO,
                              target_position=explore_pos,
                              detail="探索: 扫描新目标")

            self._current_target = self._trip_targets[0]
            names = " + ".join(t.info.description for t in self._trip_targets)
            logger.info(f"目标: {names} (共{len(self._trip_targets)}个) "
                        f"@ dist={math.sqrt((self._current_target.position[0]-rx)**2 + (self._current_target.position[1]-ry)**2):.0f}mm")

            return Action(
                type=ActionType.NAVIGATE_TO,
                target_position=self._current_target.position,
            )

        # 导航 → 夹取（仅未夹取阶段需要到达目标）
        if not grip_done:
            if not nav_arrived:
                return Action(
                    type=ActionType.NAVIGATE_TO,
                    target_position=self._current_target.position,
                )
            return Action(
                type=ActionType.GRIP,
                target_ids=[t.id for t in self._trip_targets],
            )

        # 运送到正确的区域
        if self._current_target.info.type == TargetType.INJURED:
            target_zone = self._get_injured_area_position()
        else:
            target_zone = self._get_supply_area_position()

        if not release_done:
            return Action(
                type=ActionType.TRANSPORT_TO,
                target_position=target_zone,
            )

        # 投放完成（本趟可能包含多个目标）
        if not release_valid:
            # 无效投放：目标会被裁判取出重放场心，**不能**标记为已入安全区，
            # 否则世界地图把它当"已运走"，永远不会再被选中 → 永久丢分。
            logger.error("本趟投放无效（未落在正确区域内）→ 不计分、不标记入区，重选目标")
            self._trips_completed += 1
            self._current_target = None
            self._trip_targets = []
            return Action(type=ActionType.WAIT, detail="投放无效，重选目标")

        if release_valid is None:
            # N-8：与首趟分支一致，**失败关闭**。自由趟里"漏传就当成有效"会让
            # `mark_in_safe_zone` 把实际没投进区的目标记成已运走 → 幻影送达（与 S-40 同类）。
            logger.error("未提供 release_valid（投放有效性）→ 按**无效**处理（fail-closed）")
            release_valid = False

        for t in self._trip_targets:
            # S-40：只给**真正送达**的目标记账。本趟计划里没套上的（机构容量限制）
            # 必须留在场上，否则被当成"已运走"→ 永不重选 → 永久丢分。
            if delivered_ids is not None and t.id not in delivered_ids:
                logger.warning(
                    f"本趟计划含目标#{t.id}，但实际未套上（套取机构容量限制）→ "
                    f"不标记入安全区，留待下一趟重新选择")
                continue
            self._world_map.mark_in_safe_zone(t.id)
            self._delivered_ids.add(t.id)   # 供"投放被判无效"检测（见 _check_invalid_transport）
            self._targets_delivered += 1
            self._score += get_point_value(t.info.type)
        self._trips_completed += 1
        self._current_target = None
        self._trip_targets = []

        return Action(type=ActionType.WAIT, detail="投放完成，选择下一目标")

    # ---- 强制分离恢复 ----

    def handle_forced_separation(self, new_pose: Tuple[float, float, float]) -> None:
        """
        处理强制分离。

        裁判将机器人放回出发区后调用。
        """
        # T2-6：记住分离前的策略状态，恢复时归还（而不是无条件 FREE_RUN）
        if self._strategy_state != StrategyState.FORCED_RESET:
            self._forced_prev_state = self._strategy_state
        self._strategy_state = StrategyState.FORCED_RESET
        # 保留当前目标和进度
        logger.warning(f"强制分离! 重置位姿到 ({new_pose[0]:.0f}, {new_pose[1]:.0f})")
        # 不清除 _current_target — 恢复后继续

    def _handle_forced_reset(self) -> Action:
        """处理强制分离恢复（T2-6）。

        ⚠️ 旧实现**无条件** `= FREE_RUN`：若此时首趟尚未有效完成，就等于绕过
        "首趟必须先单独送 1 个普通物资到物资区围栏内侧"这条硬规则（按赛规整场无效）。
        现在恢复**分离前**的策略状态（默认回首趟）。
        """
        prev = self._forced_prev_state
        if prev in (None, StrategyState.ANOMALY, StrategyState.DONE,
                    StrategyState.FORCED_RESET):
            prev = StrategyState.FIRST_TRIP
        self._strategy_state = prev
        self._forced_prev_state = None
        logger.info(f"强制分离恢复完成 → 恢复到策略状态 {prev.name}"
                    f"（不无条件回 FREE_RUN，避免绕过首趟闸门）")
        if self._current_target:
            return Action(
                type=ActionType.NAVIGATE_TO,
                target_position=self._current_target.position,
            )
        return Action(type=ActionType.WAIT)

    # ---- 异常处理 ----

    def _handle_anomaly(self, report: AnomalyReport,
                        robot_pose: Tuple[float, float, float] = (1500.0, 1500.0, 0.0)) -> Action:
        """
        处理异常：不再直接停止，改为**产生真实运动**的降级保活。

        ⚠️ 旧实现里 ESCAPE_MANEUVER / DEGRADE_SENSORS 两个分支返回 `WAIT`
        → 决策不再给出导航目标 → 导航没有目标就零速度 → 车**真的停住不动**，
        而"无动作"又会再次触发异常，形成"异常→静止→更异常"的死循环
        （真机表现：卡住直到时间耗尽）。现在一律返回 NAVIGATE_TO(可达点)。
        """
        logger.error("处理异常: %s → %s", report.type.name, report.recovery_action.name)

        if report.type == AnomalyType.NO_ACTION_15S:
            self._fallback_level = FallbackLevel.SURVIVAL
            pos = self._get_survival_target()
            logger.warning("15s无动作 → 保命绕圈（真实绕圈路径点）")
            return Action(type=ActionType.NAVIGATE_TO,
                          target_position=pos,
                          detail="保命: 绕圈移动")

        if report.recovery_action == RecoveryAction.EMERGENCY_STOP:
            # 这里不再真的急停：赛项只在"进入对方安全区/损坏场地/超时"才结束；
            # 上位机侧保留"降级为保命运动"，避免因误判直接放弃整轮。
            self._fallback_level = FallbackLevel.SURVIVAL
            pos = self._get_survival_target()
            logger.warning("紧急停止请求 → 降级为保命绕圈（不放弃整轮）")
            return Action(type=ActionType.NAVIGATE_TO,
                          target_position=pos,
                          detail="保命: 紧急降级")

        if report.recovery_action == RecoveryAction.ESCAPE_MANEUVER:
            self._anomaly.start_escape()
            self._last_action_time = time.time()
            # 脱困必须有真实运动：给一个探索点让导航驱动底盘，而不是干等
            # 用稳定探索点：异常持续期间本函数每帧都会被调用，用随机点会让目标每帧跳变
            pos = self._explore_target_stable(robot_pose[0], robot_pose[1])
            logger.warning(f"脱困中: {report.detail} → 驶向 ({pos[0]:.0f}, {pos[1]:.0f})")
            return Action(type=ActionType.NAVIGATE_TO,
                          target_position=pos,
                          detail="脱困: %s" % report.detail)

        if report.recovery_action == RecoveryAction.DEGRADE_SENSORS:
            self._fallback_level = FallbackLevel.EXPLORE
            logger.warning("传感器降级 → 探索模式（保持运动，避免被判无动作）")
            pos = self._get_explore_target(robot_pose[0], robot_pose[1])
            return Action(type=ActionType.NAVIGATE_TO,
                          target_position=pos,
                          detail="传感器降级: %s" % report.detail)

        return Action(type=ActionType.WAIT, detail=str(report))

    # ---- 区域位置 ----

    def _check_fallback_needed(self, rx: float, ry: float, action: Action) -> bool:
        """检查是否需要降级。

        ⚠️ T2-5：本方法**当前没有任何调用方**，且 `_fallback_level` 只被写、从不被读
        → 这套"10s 探索 / 13s 保命"的分级降级阶梯**实际不生效**。
        真正生效的保活/降级链在 `states/autonomous_state.py::_check_watchdog`
        （>10s 无位移 → `navigation.explore()`；>13s → `survival_circle()`，都会产生真实运动，
        并有 T1-10 的"保活优先窗口"防止被决策引擎逐帧覆盖）。

        保留本方法是为了**不留陷阱**：① 修掉原先两处 `if` 的顺序错误
        （`>10` 在前直接 return 导致 `>13` 分支**永不可达**）；② 明确标注"未接线"，
        避免后来人以为已有三级降级保护。
        """
        if action.type == ActionType.WAIT:
            idle_time = time.time() - self._last_action_time
            # 顺序修正：先判更严重的（保命），再判探索
            if idle_time > 13:
                self._fallback_level = FallbackLevel.SURVIVAL
                logger.warning("13s无动作 → 保命模式（注意：本分支未被调用，"
                               "实际保活见 autonomous_state._check_watchdog）")
                return True
            if idle_time > 10:
                self._fallback_level = FallbackLevel.EXPLORE
                logger.warning("10s无动作 → 探索模式（注意：本分支未被调用）")
                return True
        else:
            self._last_action_time = time.time()
            if self._fallback_level >= FallbackLevel.EXPLORE:
                self._fallback_level = FallbackLevel.NORMAL
                logger.info("恢复运动 → 降级解除")
        return False

    def _explore_target_stable(self, rx: float, ry: float,
                               timestamp: float = None) -> tuple:
        """返回一个**在一段时间内保持不变**的探索点。

        为什么需要：`_get_explore_target()` 内部是 `random.randint`，每调一次就变一个点。
        决策层是 50Hz 调用的 → 若每帧都取新点，导航目标每帧跳变，车会在原地抖动、
        永远走不到任何地方（与"保活链被逐帧覆盖"是同一类问题，见 T1-10）。
        """
        if timestamp is None:
            timestamp = time.time()
        near = (self._explore_target is not None and
                math.hypot(self._explore_target[0] - rx,
                           self._explore_target[1] - ry) < 150.0)
        if (self._explore_target is None or near
                or timestamp >= self._explore_until):
            self._explore_target = self._get_explore_target(rx, ry)
            self._explore_until = timestamp + self.EXPLORE_HOLD_S
        return self._explore_target

    def _get_explore_target(self, rx: float, ry: float) -> tuple:
        """生成探索目标：场地中央 + 随机偏移（排除安全区内的点）。"""
        for _ in range(6):
            cx = 1500 + random.randint(-600, 600)
            cy = 1500 + random.randint(-600, 600)
            cx = max(200, min(2800, cx))
            # 旧实现这里被二次 clamp 到 2200，导致 y>2200 一侧（1/2 号出发区那半场）
            # 永远搜不到 —— 上安全区附近的目标要等很久才可能被发现。
            cy = max(200, min(2800, cy))
            if not self._is_in_any_safe_zone(cx, cy):
                return (cx, cy)
        return (1500.0, 1500.0)

    def _is_in_any_safe_zone(self, x: float, y: float) -> bool:
        """(x, y) 是否落在任一安全区内（世界地图持有场地布局）。"""
        try:
            return bool(self._world_map.is_in_safe_zone((x, y)))
        except Exception:
            return False

    def _get_survival_target(self) -> Tuple[float, float]:
        """
        保命绕圈：以**机器人当前位置**为中心的圆形路径点。

        ⚠️ 旧实现 `rx, ry` 形参被调用方传成常量 (1500, 1500)，
        且半径乘了 0.001 → 返回点距当前位置只有 ~1.6mm，
        导航判"已到达" → 零速度 → "保命绕圈"实际原地不动。

        另外：靠边/靠角时点位会被场地边界钳制，可能只剩几十 mm（同样会立刻"到达"），
        所以这里在多个角度里挑一个**确实够远**的点，保证绕圈真的在动。
        """
        rx, ry = self._pose[0], self._pose[1]
        radius = 400.0
        best: Tuple[float, float] = (rx, ry)
        best_d = -1.0
        for k in range(12):
            angle = (time.time() + k * 0.5) % (2 * math.pi)
            tx = max(200.0, min(2800.0, rx + radius * math.cos(angle)))
            ty = max(200.0, min(2800.0, ry + radius * math.sin(angle)))
            d = math.hypot(tx - rx, ty - ry)
            if d > best_d:
                best, best_d = (tx, ty), d
            if d >= radius * 0.6:
                return (tx, ty)
        return best

    def _get_supply_area_position(self) -> Tuple[float, float]:
        """获取本队物资区中心位置（安全区分区 290 宽，隔板 20 居中）"""
        if self._my_color == SafeZoneColor.RED:
            return (1345.0, 2820.0)   # 红物资区（左 x[1200,1490]）中心
        return (1655.0, 180.0)        # 蓝物资区（右 x[1510,1800]）中心

    def _get_injured_area_position(self) -> Tuple[float, float]:
        """获取本队伤员区中心位置"""
        if self._my_color == SafeZoneColor.RED:
            return (1655.0, 2820.0)   # 红伤员区（右 x[1510,1800]）中心
        return (1345.0, 180.0)        # 蓝伤员区（左 x[1200,1490]）中心

    # ---- 查询 ----

    def get_stats(self) -> dict:
        return {
            "strategy_state": self._strategy_state.name,
            "time_remaining_s": self.time_remaining_s,
            "is_time_pressure": self.is_time_pressure,
            "trips_completed": self._trips_completed,
            "targets_delivered": self._targets_delivered,
            "score": self._score,
        }

    def summary(self) -> str:
        stats = self.get_stats()
        return (
            f"决策引擎: state={stats['strategy_state']}, "
            f"剩余={stats['time_remaining_s']:.0f}s, "
            f"{stats['trips_completed']}趟/{stats['targets_delivered']}个, "
            f"{stats['score']}分"
        )


# ============================================================
# 独立测试
# ============================================================

if __name__ == "__main__":
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s [%(levelname)-7s] %(name)s: %(message)s",
        datefmt="%H:%M:%S",
    )

    print("=" * 50)
    print("  决策引擎 — Mock 模式测试")
    print("=" * 50)

    from ..perception.world_map import WorldMap
    from ..perception.field_elements import FieldLayout
    from ..perception.target_types import (
        PRELIMINARY_TARGETS, TargetColor, TargetShape,
    )

    field = FieldLayout.standard()
    world_map = WorldMap(field_layout=field)
    engine = DecisionEngine(world_map)

    # 添加模拟目标到世界地图
    targets_config = [
        (TargetColor.GREEN, TargetShape.CUBE, (1000, 1500)),        # 普通物资
        (TargetColor.BLACK, TargetShape.TRIANGULAR_PYRAMID, (1500, 1000)),  # 核心物资
        (TargetColor.ORANGE, TargetShape.CUBOID, (2000, 1800)),     # 伤员
        (TargetColor.LIGHT_BLUE, TargetShape.CUBE, (800, 800)),     # 危险目标
        (TargetColor.GREEN, TargetShape.CUBE, (1200, 2000)),        # 普通物资 2
    ]
    from ..perception.target_types import DetectedTarget
    for i, (color, shape, pos) in enumerate(targets_config):
        info = PRELIMINARY_TARGETS[(color, shape)]
        det = DetectedTarget(id=i+1, info=info, position=pos)
        world_map._create_new_target(det, time.time())

    print(f"\n初始: {engine.summary()}")
    print(f"场上目标: {len(world_map.active_targets)} 个")

    engine.start_match()

    # 测试 1：FIRST_TRIP 选择普通物资
    action = engine.update((500, 500, 0))
    print(f"\n测试 1: FIRST_TRIP action={action.type.name}, {action.detail}")
    print(f"  target: {action.target_position}")
    assert action.type == ActionType.NAVIGATE_TO, "首次应NAVIGATE_TO"
    assert engine.strategy_state == StrategyState.FIRST_TRIP
    print("  ✅ 通过")

    # 测试 2：模拟到达目标 → GRIP
    action = engine.update((1000, 1500, 0), nav_arrived=True)
    print(f"\n测试 2: 到达后 action={action.type.name}")
    assert action.type == ActionType.GRIP
    print("  ✅ 通过")

    # 测试 3：夹取完成 → TRANSPORT_TO
    action = engine.update((1000, 1500, 0), nav_arrived=True, grip_done=True)
    print(f"\n测试 3: 夹取后 action={action.type.name}")
    assert action.type == ActionType.TRANSPORT_TO
    print("  ✅ 通过")

    # 测试 4：投放完成（**有效投放**）→ FREE_RUN
    # ⚠️ 必须显式传 release_valid=True：这是"首趟有效投放"的唯一判据。
    #    漏传会按 fail-closed 判为无效（见 N-8 与测试 4b）。
    action = engine.update((200, 2800, 0), nav_arrived=True, grip_done=True,
                           release_done=True, release_valid=True)
    print(f"\n测试 4: 投放后 state={engine.strategy_state.name}")
    assert engine.strategy_state == StrategyState.FREE_RUN, \
        f"应进入FREE_RUN，实际={engine.strategy_state.name}"
    print("  ✅ 通过")

    # 测试 4b：漏传 release_valid 必须**失败关闭**（N-8），不得静默放行首趟闸门
    eng_probe = DecisionEngine(WorldMap(field_layout=FieldLayout.standard()),
                              my_color=SafeZoneColor.RED)
    eng_probe.start_match()      # 必须：否则 _match_start_time=0 → 时间判定直接判 DONE
    eng_probe._world_map._create_new_target(
        DetectedTarget(id=1, info=next(iter(PRELIMINARY_TARGETS.values())),
                       position=(1000, 1500)), 0.0)
    eng_probe.update((1000, 1500, 0), nav_arrived=True)
    eng_probe.update((1000, 1500, 0), nav_arrived=True, grip_done=True)
    eng_probe.update((200, 2800, 0), nav_arrived=True, grip_done=True,
                     release_done=True)          # ← 故意不传 release_valid
    print(f"\n测试 4b: 漏传 release_valid 后 state={eng_probe.strategy_state.name}")
    assert eng_probe.strategy_state == StrategyState.FIRST_TRIP, \
        f"漏传 release_valid 时不得放行首趟闸门，实际={eng_probe.strategy_state.name}"
    print("  ✅ 通过（fail-closed）")

    # 测试 5：FREE_RUN 选择最高分（伤员）
    action = engine.update((200, 2800, 0))
    print(f"\n测试 5: FREE_RUN action={action.type.name}, target={action.target_position}")
    assert action.type == ActionType.NAVIGATE_TO
    print("  ✅ 通过")

    # 测试 6：异常检测
    print(f"\n测试 6: 异常检测")
    anomaly = engine._anomaly.check((500, 500, 0), (0, 0))
    print(f"  正常: type={anomaly.type.name}")
    assert anomaly.type == AnomalyType.NONE
    # 模拟 16 秒无动作
    engine._anomaly._last_action_time = time.time() - 16
    anomaly = engine._anomaly.check((500, 500, 0), (0, 0))
    print(f"  16s无动作: type={anomaly.type.name}, fatal={anomaly.is_fatal}")
    assert anomaly.type == AnomalyType.NO_ACTION_15S
    print("  ✅ 通过")

    print(f"\n最终: {engine.summary()}")
    print(f"\n{'='*50}")
    print("  决策引擎测试全部通过 ✅")
    print(f"{'='*50}")
