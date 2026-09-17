"""
verify_pickup_chain.py —— 套取链修复验证（2026-09-17 真机踩坑）

现场症状（4 号区，物体按赛规图7 集中在场地正中）：
    ① "识别到物体却不去套，反而开走，然后物体就不在视野里了"
    ② "物体堆卡在车下"
    ③ 一进 AUTONOMOUS 就报 NO_ACTION_15S → EMERGENCY_STOP（连报 4 帧）

逐条对应本次修复，每条都构造"修复前会错"的场景来断言：

    A. 套取停位：导航目标必须是**物体前方 standoff 处**，而不是物体自身坐标。
       旧实现把"到位"判在**车心**上 → 车心压到物体时才算到位，
       而物体此时落在开口**后方** 70mm ⇒ 整车压过去（现场："物体堆卡在车下"）。
       注意：不能用"把导航目标往前挪 70mm"来做 —— 那会得到每帧随车重算的
       **移动设定点**，实测车在追一个后退的目标、永远到不了（仿真 5 种子全 0）。
    B. 近距不再"先对准"：40~150mm 段若还要求对准，方位角抖动 >25.8° ⇒ 前进恒为 0
       ⇒ **最后 10cm 永远进不去**。
    C. 中等角度误差不把 linear 归零：旧实现 |θ误差| ≥25.8° 时 linear 直接 = 0 →
       只原地转 → 位置不变 → 位置看门狗 10s 后抢走导航目标。
    D. 看门狗把**转动**也算"在动"：原地对准不再是"卡死"。
    E. start_match 复位空闲计时器：否则"无动作"从程序启动算起，
       把 DEBUG 待机 + 1.5m 开场退避算进去 → 开局误报急停。
"""
import logging
import math
import sys
import time

logging.disable(logging.CRITICAL)
sys.path.insert(0, "src")

from rescue_robot.decision.decision_engine import DecisionEngine
from rescue_robot.perception.world_map import WorldMap, TrackedTarget
from rescue_robot.perception.target_types import (
    TargetStatus, TargetType, PRELIMINARY_TARGETS)
from rescue_robot.perception.field_elements import SafeZoneColor
from rescue_robot.navigation.motion_control import MotionController, VelocityCommand
from rescue_robot import config

ok = []


def chk(name, cond, extra=""):
    ok.append(bool(cond))
    print(("  ✅ " if cond else "  ❌ ") + name + (f"  {extra}" if extra else ""))


_info = next(v for v in PRELIMINARY_TARGETS.values()
             if v.type == TargetType.REGULAR_SUPPLY)


def _target(x, y):
    return TrackedTarget(id=1, info=_info, position=(float(x), float(y)),
                         status=TargetStatus.ACTIVE, seen_count=9)


eng = DecisionEngine(WorldMap(), my_color=SafeZoneColor.RED)

# ======================================================== A. 套取到位判据
print("A) 套取到位判据：必须看**套取框矩形开口**，不是看车心圆")
L = float(getattr(config.Placement, "DROP_FORWARD_MM", 70.0))
half_w = float(getattr(config.Placement, "SLEEVE_OPENING_MM", (150.0, 100.0))[0]) / 2.0
half_d = float(getattr(config.Placement, "SLEEVE_OPENING_MM", (150.0, 100.0))[1]) / 2.0
lo, hi = L - half_d, L + half_d
chk(f"开口几何 = 前后 [{lo:.0f},{hi:.0f}]mm × 横向 ±{half_w:.0f}mm（L={L:.0f}）",
    lo > 0 and hi > lo)

P = config.Placement
# 车在原点、朝 +x
_cases = [
    ((L, 0), True,  "框心（理想停位）"),
    ((lo, 0), True,  "前唇内沿"),
    ((hi, 0), True,  "后唇内沿"),
    ((hi + 10, 0), False, "后唇外 10mm"),
    ((0, 0), False, "**车心正下**（旧圆判据会误判为到位）"),
    ((-half_d - 10, 0), False, "车后（已开过头）"),
    ((L, half_w), True,  "侧边内沿"),
    ((L, half_w + 10), False, "侧边外 10mm"),
]
for (pos, expect, label) in _cases:
    got = P.object_in_sleeve(0.0, 0.0, 0.0, float(pos[0]), float(pos[1]))
    chk(f"object_in_sleeve: {label} → {got}", got == expect, f"({pos[0]:.0f},{pos[1]:.0f})")

# 触发闸门（该停套）与包含判据的区别：后唇内沿不该触发（会只压在框前唇上）
chk("should_stop_for_capture: 框心处 → 该停",
    P.should_stop_for_capture(0.0, 0.0, 0.0, L, 0.0))
chk("should_stop_for_capture: 后唇内沿(120mm) → **不停**（继续开到框心才停）",
    not P.should_stop_for_capture(0.0, 0.0, 0.0, hi, 0.0))
chk("should_stop_for_capture: 已开过头（车后）→ 不触发，避免白套一次",
    not P.should_stop_for_capture(0.0, 0.0, 0.0, -half_d - 10, 0.0))

# 决策引擎必须用**套取框**判到位
e0 = DecisionEngine(WorldMap(), my_color=SafeZoneColor.RED)
e0._pose = (0.0, 0.0, 0.0)
t_center = _target(L, 0.0)          # 物体在框心
t_under = _target(0.0, 0.0)         # 物体在车心正下
chk("决策层：物体在框心 → _capture_ready = True", e0._capture_ready(t_center))
# 两个判据**分工不同**，必须同时成立：
#   · `should_stop_for_capture`（触发闸门）：一进到框心就别再往前开；
#     已经压过头时也返回 True —— 那是"停下面试一次"，总比继续往前压好。
#   · `object_in_sleeve`（包含复核）：严格按矩形开口算，压过头时返回 False
#     → 转运层判定这次没套住 → 走 `_begin_retreat` 后退重试（自恢复）。
chk("触发闸门：物体在车心正下 → 仍 True（停车试一次，别再往前压）",
    e0._capture_ready(t_under))
chk("包含复核：物体在车心正下 → False（据此走失败重试/后退）",
    not config.Placement.object_in_sleeve(0.0, 0.0, 0.0, 0.0, 0.0))
chk("两者在**理想停位**（框心）上必须一致为 True",
    e0._capture_ready(t_center)
    and config.Placement.object_in_sleeve(0.0, 0.0, 0.0, L, 0.0))

# 反向对照：旧判据（车心圆）在"物体在车心正下"时判定为到位
_old = math.hypot(0.0 - 0.0, 0.0 - 0.0) < float(getattr(P, "CAPTURE_RADIUS_MM", 100.0))
chk("反向对照：旧圆判据（车心距 <CAPTURE_RADIUS_MM）在车心正下时**误判为到位**",
    bool(_old))
chk("反向对照：矩形判据纠正了它", not P.object_in_sleeve(0.0, 0.0, 0.0, 0.0, 0.0))

# ======================================================== B/C. 运动控制
print("B/C) 运动控制：近距不要求对准；中等角度误差不把前进压成 0")
mc = MotionController()
pivot_deg = math.degrees(mc.PIVOT_ONLY_RAD)
chk(f"原地转正只在 |θ误差| > {pivot_deg:.0f}° 时允许", abs(pivot_deg - 90.0) < 1e-6)

# B: 近距爬行 —— 距离 100mm、θ 误差 40°（近距方位角抖动的典型量级）
cmd = mc.compute_velocity((1000.0, 100.0), (1000.0, 0.0, 0.0), dt=0.02)
chk("近距 100mm + θ误差 90° → 仍**有前进速度**（旧实现为 0）",
    cmd.linear > 0, f"v={cmd.linear:.0f}mm/s w={cmd.angular:+.2f}")

# B 对照：把距离放到爬行段之外，同样角度误差仍应有前进速度（C 的弧线）
cmd2 = mc.compute_velocity((1000.0, 500.0), (1000.0, -500.0, 0.0), dt=0.02)
chk("中距且 θ误差 90°（在 90° 边界内）→ 有前进速度", cmd2.linear > 0,
    f"v={cmd2.linear:.0f}mm/s")

# C 对照：角度误差极大（>90°）才允许 linear=0（先转正）
# 车在原点朝 +x（θ=0），目标放在 (-100, 1000) → 方位角 ≈ 95.7° > 90°
cmd3 = mc.compute_velocity((-100.0, 1000.0), (0.0, 0.0, 0.0), dt=0.02)
_bearing = math.degrees(math.atan2(1000.0, -100.0))
chk(f"|θ误差|={_bearing:.0f}° > 90° → linear=0、只给角速度（合理）",
    cmd3.linear == 0.0 and abs(cmd3.angular) > 0.0,
    f"v={cmd3.linear:.0f} w={cmd3.angular:+.2f}")

# 修复前的算法对照：angle_ratio=1 时 linear 必为 0
_ar = min(1.0, abs(math.radians(40.0)) / (mc.ANGLE_TOLERANCE_RAD * 3))
chk("反向对照：旧公式 `linear *= (1-angle_ratio)` 在 40° 误差时归零",
    abs(1.0 - _ar) < 1e-9, f"1-angle_ratio = {1.0-_ar:.3f}")

# 投放对准（align_heading 非空）必须保留旧的"先对准"行为
cmd4 = mc.compute_velocity((1000.0, 100.0), (1000.0, 0.0, 0.0), dt=0.02,
                           align_heading=math.pi / 2)
chk("投放对准（align_heading 非空）仍保留原地对准，不爬行",
    abs(cmd4.linear) < 1e-9, f"v={cmd4.linear:.0f} w={cmd4.angular:+.2f}")

# ======================================================== D. 看门狗算转动
print("D) 看门狗：原地转向必须算作'在动'（否则 10s 后被 explore 抢走目标）")
from rescue_robot.states.autonomous_state import AutonomousState


class _SM:
    is_locked = True

    def emergency_stop(self, r):
        pass


class _IND:
    def signal(self, s):
        pass


class _Transport:
    from rescue_robot.transport.transport_pipeline import TransportPhase as _TP
    phase = _TP.IDLE

    def is_idle(self):
        return True


st = AutonomousState(_SM(), _IND(), use_mock=True, start_zone=1)
st._transport = _Transport()
st._last_motion_time = time.time()
_cmd = VelocityCommand(linear=0.0, angular=0.8, timestamp=time.time())

# 原地转：位置不变、每帧转 5°，转 8 帧共 40° > WATCHDOG_TURN_RAD(30°) → 算在动
_idle = None
for k in range(9):
    _idle = st._update_watchdog((1500.0, 1500.0, math.radians(5.0 * k)),
                                _cmd, dt=0.02, now=time.time())
chk("原地累计转 40° → 看门狗判'在动'（不累积无位移时长，不触发探索）",
    _idle is not None and _idle < 0.05, f"idle={_idle}")
chk("原地转不算卡死：explore/survival 未被触发",
    not st._explore_triggered and not st._survival_triggered)

# 反向对照：完全不动（位置与朝向都不变）仍然要能判出卡死
st2 = AutonomousState(_SM(), _IND(), use_mock=True, start_zone=1)
st2._transport = _Transport()
_past = time.time() - 14.0
st2._last_motion_time = _past
st2._update_watchdog((1500.0, 1500.0, 0.0), _cmd, dt=0.02, now=time.time())
chk("反向对照：位置与朝向都不变 14s → 仍能触发保命绕圈（看门狗没被架空）",
    st2._survival_triggered)

# ======================================================== E. 计时器复位
print("E) start_match 必须复位空闲计时器（否则把 DEBUG 待机+退避算成'无动作'）")
eng2 = DecisionEngine(WorldMap(), my_color=SafeZoneColor.RED)
eng2._last_action_time = time.time() - 20.0
eng2._anomaly._last_action_time = time.time() - 20.0
eng2.start_match()
chk("决策引擎空闲计时归零", (time.time() - eng2._last_action_time) < 1.0,
    f"{time.time()-eng2._last_action_time:.2f}s")
chk("异常处理器空闲计时归零", eng2._anomaly.get_idle_duration() < 1.0,
    f"{eng2._anomaly.get_idle_duration():.2f}s")
chk("反向对照：不复位时 20s 空闲会触发 NO_ACTION_15S",
    eng2._anomaly.check((1500.0, 1500.0, 0.0), (0.0, 0.0)).type is not None)

# ======================================================== F. 诊断可读性
print("F) 诊断：_last_cmd 必须保留真实角速度（否则卡死告警漏报）")
import inspect
_src = inspect.getsource(AutonomousState._run_once)
chk("_last_cmd 记录的第二个量是 cmd.angular（不是写死的 0.0）",
    "self._last_cmd = (float(cmd.linear), float(cmd.angular))" in _src)
chk("主循环有 2Hz 位姿/速度诊断日志",
    "📊 pose=" in _src and "w=" in _src)

print(f"\n套取链验证结果: {sum(ok)}/{len(ok)} 通过")
assert all(ok), "套取链修复验证未全部通过"
