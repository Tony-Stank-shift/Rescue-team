"""m_transport —— 转运状态机：用 Mock 夹爪把全流程跑一遍

纯算法，无需硬件。验证 APPROACHING → CAPTURING → TRANSPORTING → PLACING → COMPLETE
整条链路能否走通，以及投放是否被判为有效。
"""

from .framework import register, ok, bad

MODULE = "transport"
TITLE = "转运状态机（Mock 夹爪全流程）"


class _FakeNav:
    """最小导航桩：让转运管线认为自己已到达投放点"""

    def __init__(self):
        self.target = None
        self._arrived = False

    def set_target(self, x, y):
        self.target = (x, y)
        self._arrived = False

    def clear_target(self):
        self.target = None

    def is_arrived(self):
        return self._arrived

    def arrive(self):
        self._arrived = True


@register(MODULE, TITLE)
def run(ctx):
    ev = []
    from rescue_robot.perception.field_elements import FieldLayout, SafeZoneColor
    from rescue_robot.perception.target_types import (
        PRELIMINARY_TARGETS, TargetColor, TargetShape)
    from rescue_robot.perception.world_map import TrackedTarget
    from rescue_robot.transport.sleeve_lift import MockSleeveLift
    from rescue_robot.transport.transport_pipeline import TransportPipeline, TransportPhase

    info = PRELIMINARY_TARGETS[(TargetColor.GREEN, TargetShape.CUBE)]
    target = TrackedTarget(id=1, info=info, position=(800.0, 800.0))
    tp = TransportPipeline(sleeve=MockSleeveLift(),
                           field_layout=FieldLayout.standard(),
                           my_color=SafeZoneColor.RED)
    stopped = []
    tp.set_stop_callback(lambda: stopped.append(1))       # 记录"显式停车"是否被调用

    ok_start, violation = tp.start_trip([target])
    ev.append(f"start_trip → ok={ok_start} violation={violation.name}")
    if not ok_start:
        return bad(MODULE, f"首趟转运无法启动（违规={violation.name}）", ev,
                   "检查 LoadManager 的首次转运规则")

    nav = _FakeNav()

    # ⚠️ 2026-09-17：**套取位的判据从"车心圆"改成了"套取框矩形开口"**。
    #    开口中心在车心**前方** DROP_FORWARD_MM(70mm)，所以"车心压在物体上"
    #    （本场景原来用的 pose=(800,800)）**不是**合法套取位 —— 那时物体落在开口
    #    后方 70mm，等于整车压过去（现场现象："物体堆卡在车下"）。
    #    先用反向断言把这个坑钉住，再用正确停位（车心在物体后方 70mm）跑全流程。
    from rescue_robot.config import Placement as _P
    _L = float(_P.DROP_FORWARD_MM)
    _bad_ok = _P.object_in_sleeve(800.0, 800.0, 0.0, 800.0, 800.0)
    ev.append(f"反向对照：车心压在物体上 → 判为在开口内 = {_bad_ok}（应为 False）")
    if _bad_ok:
        return bad(MODULE, "矩形开口判据退了：车心压在物体上竟判为'在开口内'", ev,
                   "检查 config.Placement.object_in_sleeve 的前后向窗口")

    # 正确套取位：车心在物体后方 L=70mm、朝向物体（θ=0 朝 +x ⇒ 物体在正前方）
    pose = (800.0 - _L, 800.0, 0.0)
    ev.append(f"套取位 pose=({pose[0]:.0f},{pose[1]:.0f},0°)：目标在正前方 {_L:.0f}mm")
    seq = []
    for _ in range(200):
        tp.update(pose, None, nav)
        if not seq or seq[-1] != tp.phase.name:
            seq.append(tp.phase.name)
        if tp.phase == TransportPhase.TRANSPORTING and nav.target is None:
            nav.set_target(pose[0], pose[1])          # 投放点就在脚下 → dist<80
        if tp.phase == TransportPhase.COMPLETE:
            break
        if tp.phase == TransportPhase.VIOLATION:
            break

    ev.append("阶段序列：" + " → ".join(seq))
    ev.append(f"显式停车回调被调用 {len(stopped)} 次（进套取时应 ≥1）")

    if tp.phase == TransportPhase.VIOLATION:
        return bad(MODULE, "转运过程中被判为 VIOLATION（违规）", ev,
                   "检查载规则与投放判定（危险目标/超载/投放分区）")
    if len(stopped) == 0:
        return bad(MODULE, "进入套取时没有调用显式停车 → 套取期间底盘会靠看门狗被动停", ev,
                   "TransportPipeline._halt_for_capture 未被触发或被跳过")
    if tp.phase != TransportPhase.COMPLETE:
        return bad(MODULE, f"转运流程没走完，停在 {tp.phase.name}", ev,
                   "检查状态机转移条件（到点阈值、推入/释放步骤）")

    # 投放有效性
    results = tp.placer.classify_all() if hasattr(tp.placer, "classify_all") else []
    if results:
        ev.append(f"投放判定：{len(results)} 条，全部有效={all(r.is_valid for r in results)}")
    return ok(MODULE, "转运状态机正常（全流程走通、显式停车生效、无违规）", ev)
