"""
transport_pipeline.py —— 转运主控管线

协调套取机构、装载管理和安全区投放。

转运流程（单趟）：
  导航到目标 → 下降套住 → 保持 → 运送 →
  导航到安全区 → 升起释放

对接 autonomous_state 主循环。
"""

import logging
import math
import time
from dataclasses import dataclass, field
from enum import Enum, auto
from typing import List, Optional, Set, Tuple

from .sleeve_lift import AbstractSleeveLift, MockSleeveLift, SleeveAction
from .load_manager import LoadManager, Violation
from .safe_zone_placer import SafeZonePlacer, PlacementResult, PlacementZone
from ..perception.target_types import (
    TargetType, TargetInfo, get_point_value, CompetitionPhase,
)
from ..perception.world_map import TrackedTarget
from ..perception.field_elements import FieldLayout, SafeZoneColor
from ..config import Placement as cfg_placement

logger = logging.getLogger("transport_pipeline")


# ============================================================
# 转运阶段
# ============================================================

class TransportPhase(Enum):
    """转运阶段"""
    IDLE = auto()               # 空闲
    APPROACHING = auto()        # 接近目标中
    CAPTURING = auto()          # 套取中
    RETREAT = auto()            # 套取失败 → 抬爪后退，准备重试（不放弃整趟）
    TRANSPORTING = auto()       # 运送至安全区
    PLACING = auto()            # 投放中
    COMPLETE = auto()           # 完成
    VIOLATION = auto()          # 违规


@dataclass
class TransportStatus:
    """转运状态"""
    phase: TransportPhase = TransportPhase.IDLE
    trip_number: int = 0
    sleeve_action: SleeveAction = SleeveAction.RAISED
    load_count: int = 0
    load_max: int = 3
    target_ids: Set[int] = field(default_factory=set)
    distance_to_target_mm: float = 0.0
    violation: Violation = Violation.NONE


# ============================================================
# 转运管线
# ============================================================

class TransportPipeline:
    """
    转运主控管线。

    对接 autonomous loop 的典型使用：

      # 在自主循环中
      if transport.is_idle():
          target = select_best_target(world_map)
          transport.start_trip(target)

      status = transport.update(robot_pose, world_map, nav)

      if transport.is_complete():
          # 一趟完成，选择下一个目标
  """

    def __init__(self,
                 sleeve: Optional[AbstractSleeveLift] = None,
                 field_layout: Optional[FieldLayout] = None,
                 my_color: SafeZoneColor = SafeZoneColor.RED,
                 use_mock: bool = True):
        self._sleeve = sleeve or MockSleeveLift()
        self._load_mgr = LoadManager()
        self._placer = SafeZonePlacer(
            field_layout or FieldLayout.standard(), my_color
        ) if field_layout or True else None

        # 当前趟次状态
        self._phase = TransportPhase.IDLE
        self._current_targets: List[TrackedTarget] = []
        self._planning_trip = False

        # 统计
        self._total_trips = 0
        self._total_targets_delivered = 0
        self._total_score = 0

        # 放置（推+上调）框架参数 —— ⚠️ 真机需标定
        self._place_steps = 4           # 放置分步上调次数
        self._place_step = 0            # 当前放置步
        self._place_started = False     # 是否已开始推式放置
        # 推入斜坡距离 / 落点前伸量：真机标定项，来自 config.Placement（YAML 可改）
        self._push_dist_mm = float(getattr(cfg_placement, "PUSH_DIST_MM", 100.0))
        # 套取机构物理容量（一趟最多真正套住几个）：默认 1，见 config.Placement
        try:
            self._sleeve_max_hold = max(
                1, min(3, int(getattr(cfg_placement, "SLEEVE_MAX_HOLD", 1))))
        except (TypeError, ValueError):
            self._sleeve_max_hold = 1
        if self._sleeve_max_hold > 1:
            logger.error(
                f"SLEEVE_MAX_HOLD={self._sleeve_max_hold} > 1：逐个套取的机构侧已实现，"
                "但决策引擎的 grip_done 契约仍假设一趟只套 1 个（会在套取途中下发 "
                "TRANSPORT_TO 抢走导航目标）→ 实测比容量 1 更慢。请保持 1，"
                "除非先完成决策引擎耦合改造（见 docs/audit/FIXES.md 的 S-40）")

        # ── 本趟**实际**装在车上的目标（S-40 的核心状态）──
        # 旧实现用 `_current_targets`（= 本趟**计划**）当"已装载"，于是车只开到
        # 第 1 个目标就把计划里的全部目标一次性记入货舱 → 软件认为满载 3 个、
        # 实际只带 1 个：计分虚高、另外 2 个目标留在原地被下一轮重复选中。
        # 现在严格区分：`_current_targets` = 本趟计划（用于导航/规则校验），
        # `_captured` = 真正套住并记入装载的目标（用于投放判定与计分）。
        self._captured: List[TrackedTarget] = []
        self._capture_index = 0     # 当前正在前往/套取的是计划里的第几个目标
        # 上一趟**真正**送达的目标（S-40）：决策引擎据此只把"确实送到了"的目标
        # 标记为已入安全区；本趟计划里没套上的目标必须留在场上待下一趟重选，
        # 否则会被当成已运走 → 永远不再选中 → 永久丢分。
        self._last_delivered: List[TrackedTarget] = []

        # 套取失败重试（抬爪 + 后退 + 重新接近），超过次数才放弃本趟
        self._capture_retries = 0
        self._place_correct_tries = 0         # 投放位调整次数（N-6/N-7）
        self._violation = Violation.NONE      # 本趟违规类型（用于 VIOLATION 自恢复日志）

        # 注入项（由 AutonomousState 绑定真机硬件）：
        self._stop_cb = None        # 显式停车回调（套取前必须让底盘真停）
        self._sleeve_confirm = None  # 套取视觉确认回调（本车无硬件套住传感器）
        # T1-5/T1-7：投放被判无效 / 本趟作废时，回调上层把目标状态放回 ACTIVE，
        # 否则目标永久停在 BEING_TRANSPORTED → 永不被重选（永久丢分 / 首趟无法重做）。
        self._release_failed_cb = None
        self._confirm_fail_streak = 0   # 视觉确认连续失败次数（用于自动失效保护）
        self.MAX_CONFIRM_FAILS = 5      # 连续失败多少此后自动关闭视觉确认

        logger.info("TransportPipeline 初始化")

    # ---- 属性 ----

    @property
    def phase(self) -> TransportPhase:
        return self._phase

    @property
    def load_manager(self) -> LoadManager:
        return self._load_mgr

    @property
    def placer(self) -> SafeZonePlacer:
        return self._placer

    @property
    def delivered_target_ids(self) -> List[int]:
        """上一趟**真正**送达的目标 id（S-40：计划 ≠ 实际装载）。

        决策引擎在"投放完成"时只能给这些 id 标记入安全区；本趟计划里因机构容量
        限制没套上的目标仍留在场上，必须留给下一趟重新选择。
        """
        return [t.id for t in self._last_delivered]

    @property
    def sleeve_max_hold(self) -> int:
        """本车套取机构一趟最多能真正套住的个数（默认 1，见 config.Placement）。"""
        return self._sleeve_max_hold

    @property
    def compute_approach(self, robot_pose, target):
        """
        计算最优接近策略。

        Returns dict with approach_angle, speed, distance, side_offset.
        """
        import math
        rx, ry, rtheta = robot_pose
        tx, ty = target.position
        dx, dy = tx - rx, ty - ry
        dist = math.sqrt(dx*dx + dy*dy)
        target_angle = math.atan2(dy, dx)

        # 速度递减：远快近慢
        if dist > 1000:
            speed = 800
        elif dist > 300:
            speed = 500
        elif dist > 100:
            speed = 200
        else:
            speed = 100

        # 伤员从侧面接近（避免碰撞伤员）
        side_offset = 0.0
        if hasattr(target.info, 'type') and target.info.type.name == 'INJURED':
            side_offset = 0.3
            approach_angle = target_angle + (0.5 if ty > ry else -0.5)
        else:
            approach_angle = target_angle

        return {
            'approach_angle': approach_angle,
            'speed': speed,
            'distance': dist,
            'side_offset': side_offset,
        }

    #: 套取失败后允许的重试次数（每次：抬爪 → 后退 → 重新接近 → 再套）
    MAX_CAPTURE_RETRIES = 3
    #: 套取失败的后退距离（mm）：退开一点再重新对位，避免在同一个位置反复失败
    RETREAT_MM = 120.0
    #: 判定"车已到位、可以把该目标记入装载"的距离阈值（mm）。
    #: ⚠️ 这是 S-40 防线的关键：只有车真的在该目标处才允许记入，
    #:    绝不"隔着几米把没套到的目标算作已送达"。
    CAPTURE_RADIUS_MM = 150.0
    #: 到达投放点后，为让**真实落点**进入本队子区域，允许调整车位的次数（N-6/N-7）。
    MAX_PLACE_CORRECT = 3
    #: 距投放点多近开始要求朝向对准（mm）。T1-4：越早对齐，落点越稳。
    DROP_ALIGN_DIST_MM = 500.0

    def is_idle(self) -> bool:
        return self._phase in (TransportPhase.IDLE, TransportPhase.COMPLETE)

    # ---- 真机注入 ----

    def set_stop_callback(self, fn) -> None:
        """注入"立即停车"回调（绑定到串口底盘的 send_stop）。"""
        self._stop_cb = fn

    def set_release_failed_callback(self, fn) -> None:
        """注入"本趟目标没真正送达"的回调：fn(List[TrackedTarget]) → None。"""
        self._release_failed_cb = fn

    def _notify_release_failed(self, targets) -> None:
        if self._release_failed_cb is None or not targets:
            return
        try:
            self._release_failed_cb(list(targets))
        except Exception as e:
            logger.warning(f"投放失败回调异常: {e}")

    def set_sleeve_confirm(self, fn) -> None:
        """注入"槽内是否有目标"的视觉确认回调（无硬件套住传感器时使用）。

        返回 True=槽内有目标（视为套住），False=槽内空（视为没套住 → 重试）。
        """
        self._sleeve_confirm = fn

    def _halt_for_capture(self, nav) -> None:
        """
        进入套取前**显式停车**。

        为什么必须做：套取会阻塞主循环若干秒（舵机动作 + time.sleep），
        期间没有任何 VEL 下发 → 下位机只能靠速度看门狗（300ms 保持 / 800ms 停）
        被动停车，而这 800ms 里底盘仍在执行最后一帧速度指令，可能前冲几十厘米
        把目标撞飞/推走。显式停车让"套取时车静止"变成确定的事。

        两步：① 清导航目标（否则主循环恢复后导航又朝目标走）
              ② 立刻下发零速度 + STOP
        """
        if nav is not None:
            try:
                nav.clear_target()
            except Exception as e:
                logger.warning(f"清导航目标失败: {e}")
        if self._stop_cb is not None:
            try:
                self._stop_cb()
            except Exception as e:
                logger.warning(f"显式停车回调失败: {e}")
        else:
            logger.warning("未注入停车回调：套取期间底盘靠看门狗停车（可能前冲）")

    def _begin_retreat(self, rx: float, ry: float, rtheta: float, nav) -> None:
        """套取失败后：抬起夹爪 → 朝目标反方向后退 RETREAT_MM → 准备重试。

        不放弃整趟转运（旧实现直接 IDLE + 清空目标，等于整趟白跑）。
        """
        # ⚠️ 槽内**已有**目标时绝不能抬爪（抬爪 = 释放）：一抬就把先前套住的目标
        #    放回场地，而 `_captured` 仍认为在车上 → 又变成"记分虚高"。此时只后退。
        if not self._captured:
            self._sleeve.raise_up()
        if not self._current_targets or nav is None:
            # 无法规划后退（无导航/无目标）→ 原地重新接近重试
            self._phase = TransportPhase.APPROACHING
            return
        idx = min(self._capture_index, len(self._current_targets) - 1)
        tx, ty = self._current_targets[idx].position[0], self._current_targets[idx].position[1]
        dx, dy = rx - tx, ry - ty
        norm = (dx * dx + dy * dy) ** 0.5
        if norm < 1e-3:
            dx, dy, norm = -1.0, 0.0, 1.0
        bx = rx + dx / norm * self.RETREAT_MM
        by = ry + dy / norm * self.RETREAT_MM
        try:
            nav.set_target(bx, by)
        except Exception as e:
            logger.warning(f"后退目标设置失败({e})，原地重试")
            self._phase = TransportPhase.APPROACHING
            return
        self._phase = TransportPhase.RETREAT
        logger.info(f"套取失败 → 抬爪后退 {self.RETREAT_MM:.0f}mm 到 ({bx:.0f},{by:.0f}) 准备重试")

    # ---- 转运控制 ----

    def start_trip(self, targets: List[TrackedTarget]) -> Tuple[bool, Violation]:
        """
        开始一趟转运。

        Args:
            targets: 要转运的目标列表

        Returns:
            (ok, violation)
        """
        # ⚠️ 旧实现写成 `if not self.is_idle:`（漏了括号）→ 方法对象恒为真 →
        # `not ...` 恒 False → 守卫形同虚设，正在转运时还能二次 start_trip。
        if not self.is_idle():
            logger.warning(f"无法开始转运：当前阶段={self._phase.name}")
            return (False, Violation.NONE)

        # 规则校验
        target_infos = [t.info for t in targets]
        ok, violation = self._load_mgr.can_load_batch(target_infos)
        if not ok:
            consequence, is_fatal = self._load_mgr.get_violation_info(violation)
            logger.error(f"转运违规: {violation.name} — {consequence}")
            self._violation = violation
            self._phase = TransportPhase.VIOLATION
            return (False, violation)

        self._current_targets = targets
        # S-40：本趟"计划"与"实际装载"必须分开记账（详见 __init__ 注释）
        self._captured = []
        self._capture_index = 0
        self._phase = TransportPhase.APPROACHING
        self._planning_trip = True
        # 复位本趟投放/重试状态：否则第 2 趟起 _place_started 仍为 True，
        # 会跳过"推入斜坡 + 分步上调"直接释放（Mock 看不出，真机必错位）
        self._place_started = False
        self._place_step = 0
        self._capture_retries = 0
        self._confirm_fail_streak = 0
        self._place_correct_tries = 0

        names = [t.info.description for t in targets]
        logger.info(f"开始转运 (第{self._load_mgr.trip_number}趟): "
                     f"{len(targets)} 个目标 — {names}")

        return (True, Violation.NONE)

    def update(self,
               robot_pose: Tuple[float, float, float],
               world_map=None,
               nav=None) -> TransportStatus:
        """
        单帧转运更新。

        Returns:
            TransportStatus: 当前转运状态
        """
        rx, ry, rtheta = robot_pose

        # ── 违规阶段自恢复 ──
        # 旧实现：进入 VIOLATION 后既不在 is_idle() 集合里、也没有任何出口
        # → start_trip 永远被拒 → 之后所有 GRIP 被静默丢弃 → 整场不动。
        # 规则上"违规"只是**本次转运无效**（见 VIOLATION_CONSEQUENCES），
        # 因此这里作废本趟、清空目标、回到 IDLE，让上层重新规划。
        if self._phase == TransportPhase.VIOLATION:
            consequence, _ = self._load_mgr.get_violation_info(self._violation)
            logger.error(f"本次转运作废（{self._violation.name}: {consequence}）→ "
                         f"清空本趟目标，回到 IDLE 重新规划")
            # T1-5/T1-7：作废本趟时必须同时清掉"已装载"台账，并把目标放回场上，
            # 否则它们永远停在 BEING_TRANSPORTED（既不可选、又占着货舱容量）。
            self._notify_release_failed(self._captured or self._current_targets)
            self._load_mgr.discard_load()
            self._current_targets.clear()
            self._captured.clear()
            self._capture_index = 0
            self._last_delivered = []   # 作废的趟次不算送达
            self._place_started = False
            self._place_step = 0
            self._capture_retries = 0
            self._phase = TransportPhase.IDLE
            return self._get_status()

        if self._phase == TransportPhase.APPROACHING:
            # 接近目标：注意用 `_capture_index` 取"当前这一趟要套的那一个"，
            # 不能用 [0]（多目标逐个套取时会把车反复拉回第 1 个目标）。
            if self._current_targets and nav:
                target = self._current_targets[
                    min(self._capture_index, len(self._current_targets) - 1)]
                dist = self._distance((rx, ry), target.position)
                if dist < self.CAPTURE_RADIUS_MM:  # 到达套取范围
                    self._phase = TransportPhase.CAPTURING
                    # 显式停车：清导航目标 + 立即下发停车，保证套取全程底盘静止
                    self._halt_for_capture(nav)
                    logger.info(f"到达目标附近: dist={dist:.0f}mm — 已显式停车，开始套取")
                # 否则导航继续（由 autonomous loop 调用 nav 完成）

        elif self._phase == TransportPhase.CAPTURING:
            # ── 逐个套取（S-40）──
            # 旧实现：进入 CAPTURING 的唯一条件是"距**第 1 个**目标 <150mm"，然后
            # `for t in self._current_targets: load(t)` 把**整趟计划**一次性记入货舱，
            # 中间没有任何"逐个目标前往并分别套取"的状态转移 → 车只到过第 1 个目标，
            # 软件却认为 3 个都装上了。规则判的是真送达，所以这是纯虚高分。
            # 现在：只对"当前这一个"（`_capture_index`）做套取与记入，
            #       且记入前必须复核车确实在该目标处（距 > CAPTURE_RADIUS_MM 直接打回重接近）。
            if self._capture_index >= len(self._current_targets):
                # 兜底：计划已处理完却没转移阶段（理论上不可达）
                self._phase = TransportPhase.TRANSPORTING
                return self._get_status()

            target = self._current_targets[self._capture_index]
            dist = self._distance((rx, ry), target.position)
            if dist > self.CAPTURE_RADIUS_MM:
                logger.warning(
                    f"套取位姿复核不通过：距目标#{target.id} {dist:.0f}mm > "
                    f"{self.CAPTURE_RADIUS_MM:.0f}mm → 回到接近阶段重新对位"
                    f"（绝不隔着距离把没套到的目标记入装载）")
                self._phase = TransportPhase.APPROACHING
                return self._get_status()

            # 升起复位（槽内已有目标时**不能抬爪**：抬爪即释放）
            if not self._captured:
                self._sleeve.raise_up()
            # 只把**当前这一个**目标的位置交给机构：机构据"槽下方有什么"判定套取，
            # 传整趟计划等于让机构凭空套住不在跟前的目标（本次修复的根因）。
            positions = {target.id: target.position}
            if self._captured:
                success = self._sleeve.lower(positions)
            else:
                success = self._sleeve.lower_with_retry(positions, max_retries=3)
            # 无硬件"套住检测"→ 用摄像头确认 U 型槽里确实套住了目标。
            # 不可靠的确认（异常）按成功处理，避免误判导致无休止重试。
            if success and self._sleeve_confirm is not None:
                try:
                    confirmed = bool(self._sleeve_confirm())
                except Exception as e:
                    logger.warning(f"套取视觉确认异常({e})，按成功处理")
                    confirmed = True
                if not confirmed:
                    self._confirm_fail_streak += 1
                    # 失效保护：一直"确认不了"通常说明 ROI 没标定 or 摄像头看不到槽，
                    # 此时自动关闭确认，避免机器人卡在"失败→重试→放弃"的死循环里。
                    if self._confirm_fail_streak >= self.MAX_CONFIRM_FAILS:
                        logger.error(
                            f"视觉确认连续 {self._confirm_fail_streak} 次判失败 → 自动关闭视觉确认；"
                            "请检查 Camera.SLEEVE_ROI 是否已按真机标定")
                        self._sleeve_confirm = None
                    logger.warning("视觉确认：U 型槽内未见目标 → 判为套取失败，将抬爪后退重试")
                    success = False
                else:
                    self._confirm_fail_streak = 0
            if success:
                # 只记入**这一个**目标（数量、计分都以 `_captured` 为准）
                ok, v = self._load_mgr.load(target.info, target.id)
                if not ok:
                    self._violation = v
                    self._phase = TransportPhase.VIOLATION
                    return self._get_status()
                self._captured.append(target)
                self._capture_index += 1
                self._capture_retries = 0
                logger.info(
                    f"已套取并记入装载 {len(self._captured)} 个（目标#{target.id} "
                    f"{target.info.description}）")
                # 槽容量还够 + 计划里还有目标 → 去下一个；否则直接运送
                if (self._capture_index < len(self._current_targets)
                        and len(self._captured) < self._sleeve_max_hold):
                    nxt = self._current_targets[self._capture_index]
                    logger.info(
                        f"槽容量 {self._sleeve_max_hold}，继续前往下一个目标"
                        f"#{nxt.id} @({nxt.position[0]:.0f}, {nxt.position[1]:.0f})")
                    self._phase = TransportPhase.APPROACHING
                    if nav is not None:
                        nav.set_target(*nxt.position)
                else:
                    skipped = len(self._current_targets) - self._capture_index
                    if skipped > 0:
                        logger.warning(
                            f"本趟计划 {len(self._current_targets)} 个，但套取机构一趟只能"
                            f"真正套住 {self._sleeve_max_hold} 个 → 本趟只送 "
                            f"{len(self._captured)} 个；剩余 {skipped} 个目标留在场上，"
                            f"由下一趟重新选择（已从'已装载'中排除，不计分）")
                    self._phase = TransportPhase.TRANSPORTING
                    logger.info(f"套取完成（实际套住 {len(self._captured)} 个），开始运送")
            else:
                # 套取失败：不立刻放弃整趟 —— 抬爪 + 后退一小段 + 重新接近重试
                self._capture_retries += 1
                if self._capture_retries <= self.MAX_CAPTURE_RETRIES:
                    logger.warning(
                        f"套取失败({self._capture_retries}/{self.MAX_CAPTURE_RETRIES})："
                        "抬起夹爪 → 后退 → 重新接近重试")
                    self._begin_retreat(rx, ry, rtheta, nav)
                else:
                    logger.error(
                        f"套取连续 {self.MAX_CAPTURE_RETRIES} 次失败：放弃本趟转运")
                    self._capture_retries = 0
                    self._phase = TransportPhase.IDLE
                    self._current_targets.clear()
                    self._captured.clear()

        elif self._phase == TransportPhase.RETREAT:
            # 已抬爪后退 → 到位后重新接近同一目标，再试一次套取
            if (nav is None) or nav.is_arrived():
                logger.info("后退到位，重新接近目标（重试套取）")
                self._phase = TransportPhase.APPROACHING

        elif self._phase == TransportPhase.TRANSPORTING:
            # 运送至投放点（nav 目标已由 DecisionEngine 的 TRANSPORT_TO 设为
            # 物资区/伤员区中心）：到达投放点附近才投放，避免在安全区边缘提前释放。
            if nav is not None and nav.target is not None:
                dist = self._distance((rx, ry), nav.target)
                # T1-4：接近投放点时要求**朝向对准投放点**（落点 = 车心 + L·朝向，
                # 朝向错则落点错 → 投歪 -10 分/个）。导航会在"位置到了但朝向不对"时
                # 原地对准，对准后才算到达。
                if dist < self.DROP_ALIGN_DIST_MM and hasattr(nav, "require_final_heading"):
                    tx_, ty_ = nav.target
                    nav.require_final_heading(math.atan2(ty_ - ry, tx_ - rx))
                if dist < 80:  # 到达投放点（窄安全区 300 高，容差收紧保证投放准确）
                    # ── N-7/N-6：到达投放点后，先按**真实落点**确认车姿合适 ──
                    # 落点 = 车心 + L·朝向，朝向是自由变量；车停在区域中心时落点可能
                    # 被推到围栏外。这里用"钳制点 − 真实落点"算出**车需要挪动的位移**
                    # （钳制**只用于瞄准**），挪到位后再进 PLACING。
                    # ⚠️ 绝不能用钳制点做判定：那样 classify 的 ON_FENCE/OUTSIDE/
                    #    WRONG_* 分支永不可达 → 投放有效性恒为 True → B3 首趟闸门失效。
                    if (not self._drop_inside(rx, ry, rtheta)
                            and self._place_correct_tries < self.MAX_PLACE_CORRECT):
                        if self._nudge_to_valid_drop(rx, ry, rtheta, nav):
                            return self._get_status()      # 仍在 TRANSPORTING，等挪到位
                    # 进入投放前解除"朝向要求"，避免后续阶段因朝向判定卡住
                    if hasattr(nav, "clear_final_heading"):
                        nav.clear_final_heading()
                    self._phase = TransportPhase.PLACING
                    logger.debug(f"到达投放点: dist={dist:.0f}mm")

        elif self._phase == TransportPhase.PLACING:
            # 推式放置（框架）：底盘前推入斜坡 + 舵机渐进上调，分步完成。
            # ⚠️ 推入方向/距离/分步节奏需真机标定；与决策 nav.target 的协同需确认。
            if not self._place_started:
                self._place_started = True
                self._place_step = 0
                # 仅真机机构（有舵机角度控制）才设"推入点"；Mock/仿真保持投放点以免投放偏移
                if (hasattr(self._sleeve, 'set_angle')
                        or hasattr(self._sleeve, 'send_servo_angle')) \
                        and nav is not None and nav.target is not None:
                    tx, ty = nav.target
                    dx, dy = tx - rx, ty - ry
                    d = (dx * dx + dy * dy) ** 0.5
                    if d > 1e-6:
                        nav.set_target(tx + dx / d * self._push_dist_mm,
                                       ty + dy / d * self._push_dist_mm)

            # 分步上调：仅真机机构（有舵机角度控制）分步；Mock/仿真一次释放
            if (hasattr(self._sleeve, 'set_angle')
                    or hasattr(self._sleeve, 'send_servo_angle')) \
                    and self._place_step < self._place_steps:
                self._place_step += 1
                frac = self._place_step / self._place_steps
                deg = frac * 70.0   # 0 → 70（套住→释放，对齐下位机 servo.h）
                if hasattr(self._sleeve, 'set_angle'):
                    self._sleeve.set_angle(deg)
                else:
                    self._sleeve.send_servo_angle(int(round(deg)))
                return self._get_status()   # 未释放，等待下一帧继续推+上调

            # 步伐走完 → 投放判定 + 释放
            # ⚠️ 必须按**目标实际落点**判定，不能用车身位置：
            # 目标在车头 U 型槽内（前伸 L≈DROP_FORWARD_MM），释放瞬间它落在车身前方，
            # 用 (rx,ry) 判会系统性偏移一个 L —— 投对了被判无效（丢分/首趟失败），
            # 投错了被判有效（首趟"假成功"→ 按规则后续全部无效）。
            # N-6：落点还要**投影钳回本队该类型子区域**内 —— 红方物资区 y 向只有 300mm
            # （完全置入可用 280mm），而落点 = 车心 + L·朝向；从场地内部朝安全区接近时
            # 朝向指向围栏 → 落点被推到围栏上判 `ON_FENCE` → 首趟大概率无效。
            # 实测：车心停在区域中心时，9 个朝向里原本 4 个判无效，投影钳制后**全部有效**。
            base_drop = self.drop_position((rx, ry, rtheta))
            # S-40：投放判定/计分只针对**真正装在车上**的目标（`_captured`），
            # 不是本趟计划。用计划会让"没带上的目标"也按投对计分（虚高分）。
            dropped = self._captured or list(self._current_targets)
            infos = [t.info for t in dropped]
            # N-7【必须按**真实落点**判定】：绝不能用 `clamp_into_area()` 的结果去判定 ——
            # 钳制点按构造就落在"该类型应有的区域内侧"，会让 classify 的
            # ON_FENCE / OUTSIDE / WRONG_SUPPLY_IN_INJURED 等分支**永不可达**，
            # 投放有效性恒为 True（实测：场地中央 / 紫围栏南侧 / **对方安全区** /
            # 伤员区放物资 / 场外西侧 全部被判 valid）→
            #   ① B3「首趟必须投进物资区围栏内侧」闸门被反向废掉（投歪当成功 → 按赛规
            #      后续全部转运无效）；
            #   ② 计分虚高（与 S-40 同类的"谎报"）；
            #   ③ 赛规 -10 分/个的"物资入伤员区"永远不可见。
            # 钳制只用于**瞄准**（见 TRANSPORTING 里的 `_nudge_to_valid_drop`）。
            positions = [base_drop] * len(dropped)
            results = self._placer.classify_batch(positions, infos)
            all_valid = all(r.is_valid for r in results)
            # ⚠️ 曾有建议把判定放宽为"落点有效 **或** 车身位置有效"（INCREMENTAL_AUDIT N-6
            #    建议②），**已否决**：物体实际落在 drop_pt（车身前方 L 处），车身在区域内
            #    而落点在围栏上 = 物体真的落在围栏上。放宽判定会把这种"真错误"判成有效
            #    → 首趟**假成功** → 按规则后续全部无效，比判无效更糟。
            #    正确做法是把落点本身修对（见上面 clamp_into_area + YAML drop_forward_mm）。

            if hasattr(self._sleeve, 'place_ramp'):
                released = self._sleeve.place_ramp()
            else:
                released = self._sleeve.raise_up()

            if released:
                self._load_mgr.release_all(placement_ok=all_valid)
                self._total_trips += 1
                self._total_targets_delivered += len(dropped)
                self._total_score = self._load_mgr.total_score

                if not all_valid:
                    bad = [r for r in results if not r.is_valid]
                    for r in bad:
                        logger.warning(f"投放位置错误: {r.detail}")
                    # T1-5/T1-7：投放被判无效 → 目标其实还在场上（或被裁判重放场心），
                    # 必须放回可选中状态，否则永久丢分；首趟场景下还会导致
                    # "首趟永远无法重做"→ 按赛规整场无效。
                    self._notify_release_failed(dropped)

                self._phase = TransportPhase.COMPLETE
                # 记住"本趟真正送达了什么"，供决策引擎精确标记（只标真的送到的）
                self._last_delivered = list(dropped)
                self._current_targets.clear()
                self._captured.clear()
                self._capture_index = 0

                logger.info(f"转运完成: 得分={self._total_score}, "
                             f"累计={self._total_targets_delivered}个")

        return self._get_status()

    # ---- 查询 ----

    def is_complete(self) -> bool:
        return self._phase == TransportPhase.COMPLETE

    def is_violation(self) -> bool:
        return self._phase == TransportPhase.VIOLATION

    def is_idle(self) -> bool:
        return self._phase in (TransportPhase.IDLE, TransportPhase.COMPLETE)

    def _get_status(self) -> TransportStatus:
        state = self._load_mgr.state
        return TransportStatus(
            phase=self._phase,
            trip_number=state.trip_number,
            sleeve_action=self._sleeve.state.action,
            load_count=state.count,
            target_ids=state.target_ids,
            violation=None,  # not tracked at this level
        )

    def reset(self) -> None:
        self._phase = TransportPhase.IDLE
        self._current_targets.clear()
        self._captured = []
        self._capture_index = 0
        self._last_delivered = []
        self._place_started = False
        self._place_step = 0
        self._capture_retries = 0
        self._confirm_fail_streak = 0
        self._place_correct_tries = 0
        self._violation = Violation.NONE
        self._total_trips = 0
        self._total_targets_delivered = 0
        self._total_score = 0
        self._load_mgr.reset()
        self._sleeve.raise_up()

    def summary(self) -> str:
        return (
            f"转运管线: phase={self._phase.name}, "
            f"{self._load_mgr.summary()}"
        )

    # ---- 工具 ----

    def _drop_inside(self, rx: float, ry: float, rtheta: float) -> bool:
        """按**真实落点**判断本趟所有在车目标是否都落在各自区域的内侧。

        ⚠️ 绝不能用 `clamp_into_area()` 的结果来做这个判断（那会让判定恒真，
        见 N-7）。本方法只用未钳制的 `drop_position()`。
        """
        base = self.drop_position((rx, ry, rtheta))
        for t in (self._captured or self._current_targets):
            if not self._placer.classify(base, t.info).is_valid:
                return False
        return True

    def _nudge_to_valid_drop(self, rx: float, ry: float, rtheta: float, nav) -> bool:
        """落点不在区内时，算出让落点**进入区内**所需的车位调整量并下发（N-6/N-7）。

        这里 `clamp_into_area()` 只用来**瞄准**：它给出"最近的可接受落点"，与真实
        落点之差就是车需要平移的位移（车平移 Δ，落点也平移 Δ）。到位后再按**真实
        落点**重新判定。

        Returns:
            True=已下发调整（调用方应留在 TRANSPORTING 等到位）；False=无法调整
        """
        current = self._captured or self._current_targets
        if not current:
            return False
        info = current[0].info          # 容量=1 时只有一个；多目标同区域
        base = self.drop_position((rx, ry, rtheta))
        want = self._placer.clamp_into_area(base, info)
        dx, dy = want[0] - base[0], want[1] - base[1]
        if abs(dx) < 1e-6 and abs(dy) < 1e-6:
            return False
        self._place_correct_tries += 1
        tx, ty = rx + dx, ry + dy
        try:
            if not nav.set_target(tx, ty):
                logger.warning(f"投放位调整目标被拒（越界）：({tx:.0f},{ty:.0f})")
                return False
        except Exception as e:
            logger.warning(f"投放位调整失败: {e}")
            return False
        logger.info(f"落点不在区内 → 调整车位 {dx:+.0f},{dy:+.0f}mm 到 "
                    f"({tx:.0f},{ty:.0f})（第 {self._place_correct_tries}/"
                    f"{self.MAX_PLACE_CORRECT} 次）")
        return True

    def drop_position(self, pose: Tuple[float, float, float]) -> Tuple[float, float]:
        """
        由车身位姿推算**目标落点**：车心 + 槽内前伸距离 L 沿朝向方向。

            drop = (x + L·cosθ, y + L·sinθ)

        L = `config.Placement.DROP_FORWARD_MM`（真机标定项，YAML 可改）。
        投放有效性判定必须用这个点，而不是车身位置。
        """
        import math
        x, y, theta = pose[0], pose[1], pose[2]
        L = float(getattr(cfg_placement, "DROP_FORWARD_MM", 150.0))
        return (x + L * math.cos(theta), y + L * math.sin(theta))

    @staticmethod
    def _distance(p1: Tuple[float, float],
                  p2: Tuple[float, float]) -> float:
        import math
        return math.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)

    @staticmethod
    def _distance_to_region(x: float, y: float,
                            region) -> float:
        import math
        dx = max(region.x - x, 0, x - region.x - region.width)
        dy = max(region.y - y, 0, y - region.y - region.height)
        return math.sqrt(dx * dx + dy * dy)


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
    print("  转运控制 — Mock 模式测试")
    print("=" * 50)

    from ..perception.target_types import (
        PRELIMINARY_TARGETS, TargetColor, TargetShape,
    )
    from ..perception.world_map import TrackedTarget, TargetStatus

    # 获取目标配置
    regular_info = PRELIMINARY_TARGETS[(TargetColor.GREEN, TargetShape.CUBE)]
    core_info = PRELIMINARY_TARGETS[(TargetColor.BLACK, TargetShape.TRIANGULAR_PYRAMID)]
    injured_info = PRELIMINARY_TARGETS[(TargetColor.ORANGE, TargetShape.CUBOID)]
    dangerous_info = PRELIMINARY_TARGETS[(TargetColor.LIGHT_BLUE, TargetShape.CUBE)]

    field = FieldLayout.standard()
    tp = TransportPipeline(field_layout=field)

    # --- 测试 1：首次转运 = 1 个普通物资 ✓ ---
    print("\n测试 1: 首次转运（1普通物资）")
    target = TrackedTarget(id=1, info=regular_info, position=(500, 500))
    ok, v = tp.start_trip([target])
    print(f"  start_trip: ok={ok}, violation={v}")
    assert ok, "首次转运 1 普通物资应该成功"

    # 模拟转运流程
    tp.update((500, 500, 0))  # APPROACHING → CAPTURING
    tp.update((500, 500, 0))  # CAPTURING → capture
    tp.update((500, 500, 0))  # TRANSPORTING
    tp.update((500, 500, 0))  # 仍在 TRANSPORTING
    # 模拟到达安全区
    tp._phase = TransportPhase.PLACING
    tp.update((200, 2800, 0))  # PLACING → release
    print(f"  phase={tp.phase.name}")
    assert tp.is_complete(), "转运应该完成"
    print("  ✅ 通过")

    # --- 测试 2：首次转运 >1 个 → 违规 ---
    print("\n测试 2: 首次转运 2 个 → 违规")
    tp.reset()
    t1 = TrackedTarget(id=2, info=regular_info, position=(500, 500))
    t2 = TrackedTarget(id=3, info=regular_info, position=(600, 500))
    ok, v = tp.start_trip([t1, t2])
    print(f"  start_trip: ok={ok}, violation={v}")
    assert not ok and v == Violation.FIRST_TRIP_MULTI, \
        f"首次转运 2 个应违规，得到 {v}"
    print("  ✅ 通过")

    # --- 测试 3：危险目标 → 违规 ---
    print("\n测试 3: 危险目标 → 违规")
    tp.reset()
    # 先完成一次转运
    t = TrackedTarget(id=5, info=regular_info, position=(500, 500))
    tp.start_trip([t])
    tp._sleeve.lower({5: (500, 500)})
    tp._load_mgr.load(regular_info, 5)
    tp._load_mgr.release_all()
    tp._phase = TransportPhase.IDLE
    # 现在尝试装载危险目标
    danger = TrackedTarget(id=6, info=dangerous_info, position=(700, 500))
    ok, v = tp.start_trip([danger])
    print(f"  start_trip: ok={ok}, violation={v}")
    assert not ok and v == Violation.DANGEROUS_TARGET, \
        f"危险目标应违规，得到 {v}"
    print("  ✅ 通过")

    # --- 测试 4：伤员必须单独 ---
    print("\n测试 4: 装载 2 个伤员 → 违规")
    tp.reset()
    tp._load_mgr.mark_first_trip_done()  # 显式跳过首趟（自测用）
    i1 = TrackedTarget(id=7, info=injured_info, position=(500, 500))
    i2 = TrackedTarget(id=8, info=injured_info, position=(600, 500))
    ok, v = tp.start_trip([i1, i2])
    print(f"  start_trip: ok={ok}, violation={v}")
    assert not ok and v == Violation.INJURED_MULTI, \
        f"2 个伤员应违规，得到 {v}"
    print("  ✅ 通过")

    # --- 测试 5：超 3 个 → 违规 ---
    print("\n测试 5: 装载 4 个 → 违规")
    tp.reset()
    tp._load_mgr.mark_first_trip_done()
    targets_4 = [
        TrackedTarget(id=10+i, info=regular_info, position=(500+i*50, 500))
        for i in range(4)
    ]
    ok, v = tp.start_trip(targets_4)
    print(f"  start_trip: ok={ok}, violation={v}")
    assert not ok and v == Violation.OVER_LIMIT, \
        f"4 个应违规，得到 {v}"
    print("  ✅ 通过")

    # --- 测试 6：安全区投放判定 ---
    print("\n测试 6: 投放位置判定")
    placer = SafeZonePlacer(field)
    # 物资放入物资区 → 有效
    r = placer.classify((150, 2850), regular_info)
    print(f"  物资@物资区: valid={r.is_valid}, {r.detail}")
    # 物资放入伤员区 → 无效
    r = placer.classify((500, 2850), regular_info)
    print(f"  物资@伤员区: valid={r.is_valid}, penalty={r.penalty}, {r.detail}")
    # 目标在安全区外
    r = placer.classify((1500, 1500), regular_info)
    print(f"  场地中央: valid={r.is_valid}, {r.detail}")
    print("  ✅ 通过")

    print(f"\n{'='*50}")
    print("  转运控制测试全部通过 ✅")
    print(f"{'='*50}")
