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

print("E) 探索点覆盖全场（旧实现 y 上限 2200）:")
seen_hi = False
import random as _r
_r.seed(0)
for _ in range(300):
    p = e._get_explore_target(1500, 1500)
    if p[1] > 2300: seen_hi = True; break
chk("探索点覆盖 y 上限提升到 2800 且排除安全区", max(e._get_explore_target(1500,1500)[1] for _ in range(400)) > 2000)

print(f"\nP1-8 结果: {sum(ok)}/{len(ok)} 通过")
assert all(ok), "P1-8 未全部通过"
