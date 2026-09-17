"""P1-8 修复验证：看门狗按里程计实际位移判定；异常链不再返回 WAIT 空转。"""
import logging, sys, time
logging.disable(logging.CRITICAL)
sys.path.insert(0, "src")

from rescue_robot.decision.decision_engine import DecisionEngine, ActionType
from rescue_robot.decision.anomaly_handler import AnomalyHandler, AnomalyType
from rescue_robot.perception.world_map import WorldMap
from rescue_robot.perception.field_elements import FieldLayout, SafeZoneColor
from rescue_robot.navigation.navigation_pipeline import NavigationPipeline
from rescue_robot.states.autonomous_state import AutonomousState
from rescue_robot.navigation.motion_control import VelocityCommand
from rescue_robot.transport.transport_pipeline import TransportPhase

class Cmd:  # 假速度指令
    def __init__(s, v=0.0, w=0.0): s.linear, s.angular = v, w

class FakeNav(NavigationPipeline):
    """记录 explore/survival 调用，其余行为同真 nav。"""
    def __init__(s):
        super().__init__(FieldLayout.standard(), my_color=SafeZoneColor.RED, use_mock=True)
        s.explore_calls = 0; s.survival_calls = 0
    def explore(s, robot_pose=None): s.explore_calls += 1; super().explore(robot_pose)
    def survival_circle(s, robot_pose=None): s.survival_calls += 1; super().survival_circle(robot_pose)

class FakeTransport:
    class _P:
        def __init__(s, n): s.name = n
    def __init__(s): s.phase = FakeTransport._P("IDLE")
    def is_idle(s): return True
    def is_complete(s): return False
    def update(s, *a, **k): pass

def make_state():
    sm = type("SM", (), {"is_locked": True, "emergency_stop": lambda s, r: None})()
    ind = type("IND", (), {"signal": lambda s, x: None})()
    return AutonomousState(sm, ind, navigation=FakeNav(), transport=FakeTransport(),
                           use_mock=True)

ok = []
def chk(name, cond, extra=""):
    ok.append(bool(cond)); print(("  ✅ " if cond else "  ❌ ") + name + (f"  {extra}" if extra else ""))

print("A) 看门狗判据 = 实际位移（不是下发速度）:")
st = make_state(); nav = st._navigation
t0 = 1000.0
st._last_motion_time = t0
st._last_action_time = t0
# 场景1：一直下发速度但位置不动（轮子空转/卡围栏）→ 必须触发探索与保命
for i in range(700):
    st._update_watchdog((150.0, 2850.0), Cmd(400, 0.0), dt=0.02, now=t0 + i * 0.02)
chk("下发400mm/s但零位移 → 触发探索(>10s)", nav.explore_calls >= 1, f"explore={nav.explore_calls}")
chk("继续无位移 → 触发保命绕圈(>13s)", nav.survival_calls >= 1, f"survival={nav.survival_calls}")

# 场景2：位置真在变（即使下发速度很小）→ 不得触发降级
st2 = make_state(); nav2 = st2._navigation
st2._last_motion_time = t0
st2._last_action_time = t0
x = 150.0
for i in range(700):
    x += 5.0                      # 250mm/s 实际位移
    st2._update_watchdog((x, 2850.0), Cmd(20, 0.0), dt=0.02, now=t0 + i * 0.02)
chk("真实位移存在 → 不触发任何降级", nav2.explore_calls == 0 and nav2.survival_calls == 0,
    f"explore={nav2.explore_calls}, survival={nav2.survival_calls}")

# 场景3：套取/投放阶段合法静止 → 不打断
st3 = make_state(); nav3 = st3._navigation; st3._transport.phase = TransportPhase.CAPTURING
st3._last_motion_time = t0
for i in range(700):
    st3._update_watchdog((150.0, 2850.0), Cmd(0, 0), dt=0.02, now=t0 + i * 0.02)
chk("CAPTURING 阶段合法静止 → 不触发降级", nav3.explore_calls == 0 and nav3.survival_calls == 0)

print("B) 实际速度被算出并传给决策引擎:")
chk("速度 =(位移/dt) ≈ 250mm/s", abs(st2._last_velocity[0] - 250.0) < 1e-6,
    f"v={st2._last_velocity[0]:.1f}")

print("C) AnomalyHandler: 无运动学信息时不误判:")
h = AnomalyHandler(); h._last_action_time = time.time() - 20
r = h.check((0, 0, 0), None)
chk("velocity=None → 不报无动作", r.type == AnomalyType.NONE, f"type={r.type.name}")

print("D) 异常链必须产生真实运动（旧实现返回 WAIT 静止）:")
wm = WorldMap(field_layout=FieldLayout.standard())
e = DecisionEngine(wm, my_color=SafeZoneColor.RED); e.start_match()
e._pose = (300.0, 300.0, 0.0)
rep = e._anomaly._report(AnomalyType.NO_ACTION_15S, "t", e._anomaly.REPORT_EMERGENCY
                         if hasattr(e._anomaly, "REPORT_EMERGENCY") else __import__(
                             "rescue_robot.decision.anomaly_handler", fromlist=["x"]).RecoveryAction.EMERGENCY_STOP,
                         False)
a = e._handle_anomaly(rep)
chk("NO_ACTION_15S → NAVIGATE_TO(真实点)", a.type == ActionType.NAVIGATE_TO and a.target_position is not None,
    f"{a.type.name} {a.target_position}")
d = ((a.target_position[0]-300)**2 + (a.target_position[1]-300)**2) ** 0.5
chk("保命点距当前位置 ~400mm（旧实现只有1.6mm=原地不动）", d > 150, f"d={d:.1f}mm")

ESC = __import__("rescue_robot.decision.anomaly_handler", fromlist=["x"]).RecoveryAction.ESCAPE_MANEUVER
rep2 = e._anomaly._report(AnomalyType.STUCK, "卡死", ESC, False)
a2 = e._handle_anomaly(rep2, (300.0, 300.0, 0.0))
chk("脱困 → NAVIGATE_TO 而非 WAIT(静止)", a2.type == ActionType.NAVIGATE_TO, a2.type.name)

DEG = __import__("rescue_robot.decision.anomaly_handler", fromlist=["x"]).RecoveryAction.DEGRADE_SENSORS
rep3 = e._anomaly._report(AnomalyType.SENSOR_FAULT_CAMERA, "摄像头无数据", DEG, False)
a3 = e._handle_anomaly(rep3, (300.0, 300.0, 0.0))
chk("传感器降级 → NAVIGATE_TO 而非 WAIT(静止)", a3.type == ActionType.NAVIGATE_TO, a3.type.name)

print("E) 探索覆盖全场（旧实现 y 上限 2200 → 半场永远搜不到）:")
# 旧实现是"在 [900,2100]² 里纯随机取点"：外层 900mm 环带永远搜不到，
# 而且 y>2200 那半场（1/2 号出发区一侧）根本进不去。
# 现在改为**确定性搜索计划**：中央优先 + 全场蛇形车道。
# 断言比原来更强 —— 不再靠"采样随机点看 y 能到多大"，而是直接验证计划本身：
#   ① 覆盖：车道探测带并集在场地内部**无盲带**
#   ② 不分半场：上下两半都有车道（这正是旧 bug 的回归护栏）
#   ③ 安全：不落在安全区，且**行进路径**不穿过场心（中央可能有图7 那堆物体）
import math as _m
from rescue_robot.decision.decision_engine import (
    FIELD_CENTER_X as _CX, FIELD_CENTER_Y as _CY, FIELD_SIZE as _FS,
)

e._search_plan = None; e._search_plan_exhausted = False
plan = e._build_search_plan(200.0, 200.0)
ys = sorted({p[1] for p in plan})
HB, KO = e.EXPLORE_LANE_HALF_BAND_MM, e.EXPLORE_CENTER_KEEPOUT_MM

chk("车道覆盖上下两个半场（旧 bug：y>2200 那半场永远搜不到）",
    min(ys) < _CY and max(ys) > _CY, f"车道 y = {[round(v) for v in ys]}")

_gaps = [gy for gy in range(50, _FS - 50 + 1, 25)
         if not any(abs(gy - ly) <= HB for ly in ys)]
chk("车道探测带并集无盲带（单侧半宽 %.0fmm）" % HB, not _gaps, f"盲带 y={_gaps}")

chk("搜索路点不落在任何安全区内",
    not [p for p in plan if e._is_in_any_safe_zone(*p)])

_worst = 1e9
for (ax, ay), (bx, by) in zip(plan, plan[1:]):
    _dx, _dy = bx - ax, by - ay
    _l2 = _dx * _dx + _dy * _dy
    _t = 0.0 if _l2 == 0 else max(0.0, min(1.0, ((_CX - ax) * _dx + (_CY - ay) * _dy) / _l2))
    _worst = min(_worst, _m.hypot(ax + _t * _dx - _CX, ay + _t * _dy - _CY))
chk("搜索行进路径不穿越场心禁入区（不碾中央物体堆）",
    _worst >= KO, f"最近 {_worst:.0f}mm ≥ {KO:.0f}mm")

_plan2 = e._build_search_plan(2850.0, 150.0)
_d0 = _m.hypot(_plan2[0][0] - _CX, _plan2[0][1] - _CY)
chk("中央优先段停在 standoff 处（看得见场心、不压物体）",
    abs(_d0 - e.EXPLORE_CENTER_STANDOFF_MM) < 1.0, f"首点距场心 {_d0:.0f}mm")
chk("搜索计划确定且有限（不重复、不空）",
    len(_plan2) > 0 and len(_plan2) == len(set(_plan2)), f"{len(_plan2)} 个路点")

print(f"\nP1-8 结果: {sum(ok)}/{len(ok)} 通过")
assert all(ok), "P1-8 未全部通过"
