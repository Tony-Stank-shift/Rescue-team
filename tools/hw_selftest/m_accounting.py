"""m_accounting —— 载荷台账一致性（S-40「多目标假装载」现场自查）

**这类故障不报错**：车只到第 1 个目标，软件却把本趟计划的全部目标记进货舱 →
自评"送了 3 个"、实际只送 1 个 → 分数虚高，翻日志也看不出来。
本模块把"计划 ≠ 实装"钉成可判 FAIL 的断言，现场跑一次就知道台账对不对。

三条检查（全部纯逻辑，**无硬件也真跑真判，绝不 SKIP**）：
  A. 计划 3 个、车只到第 1 个 → `load_manager.state.count` 必须是 **1**（旧实现=3）
  B. 套取前位姿复核：车离目标 2m 却处于 CAPTURING → **不得**记入装载，且被打回 APPROACHING
  C. 已持有目标时后退重试**不得抬爪**（抬爪=释放，会把已经套住的货丢掉）

判定依据（源码位置，便于复核）：
  `TransportPipeline._captured` / `CAPTURE_RADIUS_MM` / `_capture_index`
  （`src/rescue_robot/transport/transport_pipeline.py`）
"""

import math

from .framework import register, ok, bad

MODULE = "accounting"
TITLE = "载荷台账一致性（S-40 假装载：计划≠实装必须被抓住）"

#: 本趟计划的 3 个目标位置
PLAN = ((2400.0, 2500.0), (400.0, 400.0), (2600.0, 400.0))

# ⚠️ 2026-09-17：套取位的判据已从"车心圆"改为**套取框矩形开口**——开口中心在车心
#   **前方** DROP_FORWARD_MM=70mm。所以"车心压在目标上"（原来直接用 PLAN[0] 当车姿）
#   **不再**是合法套取位（那时目标落在开口后方 70mm，实车是压过去的）。
#   正确停位 = 目标坐标 − 70mm（朝向目标）。
CAPTURE_POSE = (PLAN[0][0] - 70.0, PLAN[0][1], 0.0)


class _NavStub:
    """最小导航桩：只记录目标，不做规划（与真机 nav 的接口子集一致）"""

    def __init__(self):
        self.target = None

    def set_target(self, x, y):
        self.target = (x, y)

    def clear_target(self):
        self.target = None

    def is_arrived(self):
        return True


def _make_target(wm, tid, pos):
    """按颜色从初赛目标表取 TargetInfo，建一个世界地图里的 TrackedTarget"""
    from rescue_robot.perception.target_types import (
        PRELIMINARY_TARGETS, DetectedTarget, TargetColor)
    info = next(v for (c, _s), v in PRELIMINARY_TARGETS.items()
                if c == TargetColor.GREEN)
    return wm._create_new_target(
        DetectedTarget(id=tid, info=info, position=pos), 0.0)


def _make_tp(field, color):
    """转运管线：显式标记"首趟已有效完成"，以便直接验证自由趟的装载规则"""
    from rescue_robot.transport.transport_pipeline import TransportPipeline
    tp = TransportPipeline(field_layout=field, my_color=color, use_mock=True)
    tp._load_mgr.mark_first_trip_done()
    return tp


@register(MODULE, TITLE)
def run(ctx):
    ev = []
    try:
        from rescue_robot.perception.field_elements import FieldLayout, SafeZoneColor
        from rescue_robot.perception.world_map import WorldMap
        from rescue_robot.transport.transport_pipeline import (
            TransportPhase, TransportPipeline)
    except Exception as e:
        return bad(MODULE, f"无法导入转运管线：{e!r}", ev)

    field = FieldLayout.standard()
    color = SafeZoneColor.RED
    wm = WorldMap(field_layout=field)
    ids = [_make_target(wm, 1000 + i, p) for i, p in enumerate(PLAN)]
    ev.append(f"构造本趟计划 {len(ids)} 个目标（id={ids}）："
              + "、".join(f"({p[0]:.0f},{p[1]:.0f})" for p in PLAN))

    # ── A. 计划 3 个、车只到第 1 个 ──────────────────────────
    # 阶段机跨帧推进：第 1 帧 APPROACHING→CAPTURING（并显式停车），第 2 帧真正下压套取
    tp = _make_tp(field, color)
    ok_start, violation = tp.start_trip([wm.targets[i] for i in ids])
    ev.append(f"[A] start_trip → ok={ok_start} violation={violation.name}")
    if not ok_start:
        return bad(MODULE, f"本趟计划无法启动（违规={violation.name}）", ev,
                   "检查 LoadManager.can_load_batch（首趟/≤3/伤员单独/危险拒绝）")

    nav = _NavStub()
    for _ in range(2):
        tp.update(CAPTURE_POSE, wm, nav)
    st = tp.load_manager.state
    real = len(tp._captured)
    ev.append(f"[A] 车只到过第 1 个目标 (2400,2500)：load_manager.count={st.count}、"
              f"_captured={real}、"
              f"_captured_ids={[t.id for t in tp._captured]}、target_ids={set(st.target_ids)}、"
              f"phase={tp.phase.name}")
    hint_a = ("①看 `TransportPipeline._capture_index` 是否逐个推进、"
              "`_captured` 是否只在 `sleeve.lower()` 成功后才 append；"
              "②确认记账用的是 `_captured` 而不是 `_current_targets`（整趟计划）；"
              "③对照 `src/rescue_robot/transport/transport_pipeline.py` 的 CAPTURING 分支")
    # A1：台账数量 = 实装数量
    if st.count != real:
        return bad(MODULE,
                   f"台账数与实装数不一致：load_manager.count={st.count} 但实际套住 {real} 个"
                   f"（计划 {len(ids)} 个、车只到过 1 个）→ 分数会虚高", ev, hint_a)
    # A2：车只到过第 1 个，就必须只装 1 个（旧 S-40 会把整趟计划记 3 个）
    if real != 1:
        return bad(MODULE,
                   f"车只到过第 1 个目标，却记入了 {real} 个（应为 1）→ "
                   f"旧 S-40「多目标假装载」：把整趟计划一次性记账，分数虚高", ev, hint_a)
    # A3：记账的必须是"当时真的套住的那一个"
    captured_ids = {t.id for t in tp._captured}
    if captured_ids != {ids[0]} or set(st.target_ids) != {ids[0]}:
        return bad(MODULE,
                   f"货舱里出现了没去过的目标：_captured_ids={captured_ids}、"
                   f"target_ids={set(st.target_ids)}（只应含 {ids[0]}）", ev,
                   "记入装载的必须是**当时真的套住的那一个**，不能成批写入；"
                   "对照 CAPTURING 分支里 `_load_mgr.load()` 的调用位置")
    ev.append("[A] ✓ 台账 = 实装（只记了真正套住的那 1 个，未去过的 2 个留在场上）")

    # ── B. 套取前位姿复核（隔着 2m 不许记账）────────────────
    tp2 = _make_tp(field, color)
    tp2.start_trip([wm.targets[ids[1]]])
    tp2._phase = TransportPhase.CAPTURING          # 直接进套取，模拟"没对位就下压"
    far_pose = PLAN[0] + (0.0,)                    # 车在 (2400,2500)，目标在 (400,400)
    dist = math.hypot(PLAN[0][0] - PLAN[1][0], PLAN[0][1] - PLAN[1][1])
    tp2.update(far_pose, wm, nav)
    ev.append(f"[B] 车在 ({far_pose[0]:.0f},{far_pose[1]:.0f})、目标在 {PLAN[1]}（相距 "
              f"{dist:.0f}mm）、强制 CAPTURING → count={tp2.load_manager.state.count}、"
              f"phase={tp2.phase.name}")
    if tp2.load_manager.state.count != 0:
        return bad(MODULE,
                   f"车距目标 {dist:.0f}mm 却把目标记入了装载 → 隔着距离虚记（分数虚高）",
                   ev, f"套取记账前必须有位姿复核：距目标 > CAPTURE_RADIUS_MM"
                       f"（{getattr(TransportPipeline, 'CAPTURE_RADIUS_MM', 150.0):.0f}mm）"
                       f"就不许记账并打回 APPROACHING")
    if tp2.phase != TransportPhase.APPROACHING:
        return bad(MODULE,
                   f"位姿复核不通过却没有退回接近阶段（phase={tp2.phase.name}）→ "
                   f"会继续对不上的位置下压套取", ev,
                   "复核失败必须把阶段打回 APPROACHING 并重新设置导航目标")
    ev.append("[B] ✓ 位姿复核生效（隔 2m 不记账、打回 APPROACHING 重新对位）")

    # ── C. 已持有目标时后退重试不得抬爪 ─────────────────────
    tp3 = _make_tp(field, color)
    tp3.start_trip([wm.targets[ids[0]]])
    tp3.update(CAPTURE_POSE, wm, nav)              # → CAPTURING
    tp3.update(CAPTURE_POSE, wm, nav)              # 套住 1 个
    held = len(tp3._captured)
    raised = {"n": 0}
    orig = tp3._sleeve.raise_up

    def _spy():
        raised["n"] += 1
        return orig()

    tp3._sleeve.raise_up = _spy
    tp3._begin_retreat(PLAN[0][0], PLAN[0][1], 0.0, nav)
    ev.append(f"[C] 已持有 {held} 个目标后触发后退重试 → raise_up 被调用 {raised['n']} 次"
              f"（应为 0）")
    if held == 0:
        return bad(MODULE, "套取没有记入任何目标，无法验证'持有后不得抬爪'", ev,
                   "先确认 A 检查通过（套取记账链路正常）")
    if raised["n"] != 0:
        return bad(MODULE,
                   f"已套住目标时后退重试抬了爪（raise_up 被调用 {raised['n']} 次）→ "
                   f"抬爪=释放，会把已经到手的货丢回场地",
                   ev, "`_begin_retreat` 必须判 `self._captured`：非空只后退、绝不抬爪")
    ev.append("[C] ✓ 持有目标时只后退不抬爪（不会把货物丢回场地）")

    return ok(MODULE,
              "载荷台账与实际一致（计划≠实装被正确区分、套取位姿复核生效）", ev,
              "现场核对：一趟送完，数一遍场上少了几个目标，应与自检/日志里的记账数一致")
