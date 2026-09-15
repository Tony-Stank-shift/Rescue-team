"""P1.5 验证：改 YAML 真的生效（改前/改后），且默认值不变。"""
import logging, sys, os, re, shutil, tempfile
logging.disable(logging.CRITICAL)
sys.path.insert(0, "src")
pass

from rescue_robot.innovation.config_loader import RobotConfig
from rescue_robot import config as C
from rescue_robot.decision.decision_engine import DecisionEngine
from rescue_robot.decision.anomaly_handler import AnomalyHandler
from rescue_robot.states.autonomous_state import AutonomousState
from rescue_robot.navigation.motion_control import MotionController
from rescue_robot.transport.load_manager import LoadManager

ok=[]
def chk(n,c,extra=""):
    ok.append(bool(c)); print(("  ✅ " if c else "  ❌ ")+n+(f"  {extra}" if extra else ""))

print("默认值（YAML 未改）:")
cfg = RobotConfig.from_yaml("config/robot.default.yaml")
C.apply_robot_config(cfg)
chk("MATCH_DURATION_S=180", DecisionEngine.MATCH_DURATION_S == 180.0, str(DecisionEngine.MATCH_DURATION_S))
chk("TIME_PRESSURE_S=30", DecisionEngine.TIME_PRESSURE_S == 30.0)
chk("ANOMALY watchdog=15", AnomalyHandler.WATCHDOG_TIMEOUT_S == 15.0)
chk("AutonomousState 探索=10/保命=13", AutonomousState.WATCHDOG_EXPLORE_S == 10.0 and AutonomousState.WATCHDOG_SURVIVAL_S == 13.0)
chk("PID_DISTANCE=(0.8,0.01,0.05)", MotionController.PID_DISTANCE == (0.8,0.01,0.05))
_yaml_default = float(re.search(r"drop_forward_mm:\s*([0-9.]+)",
                                open("config/robot.default.yaml", encoding="utf-8").read()).group(1))
chk(f"DropForward={_yaml_default:g}（与 YAML 一致）/ penalty=10",
    abs(C.Placement.DROP_FORWARD_MM - _yaml_default) < 1e-9
    and C.Thresholds.PLACEMENT_PENALTY_PER_TARGET == 10,
    f"代码={C.Placement.DROP_FORWARD_MM} YAML={_yaml_default}")

print("改 YAML → 立刻生效:")
tmp = tempfile.mkdtemp()
p = os.path.join(tmp, "robot.test.yaml")
data = open("config/robot.default.yaml").read()
data = data.replace("duration_s: 180", "duration_s: 60")
data = data.replace("time_pressure_s: 30", "time_pressure_s: 45")
data = data.replace("penalty_per_target: 10", "penalty_per_target: 7")
data = re.sub(r"drop_forward_mm:\s*[0-9.]+", "drop_forward_mm: 220", data)
open(p,"w").write(data)
cfg2 = RobotConfig.from_yaml(p)
C.apply_robot_config(cfg2)
chk("duration 180 → 60", DecisionEngine.MATCH_DURATION_S == 60.0, str(DecisionEngine.MATCH_DURATION_S))
chk("time_pressure 30 → 45", DecisionEngine.TIME_PRESSURE_S == 45.0)
chk("penalty 10 → 7", C.Thresholds.PLACEMENT_PENALTY_PER_TARGET == 7)
chk(f"drop_forward {_yaml_default:g} → 220", C.Placement.DROP_FORWARD_MM == 220.0)

print("还原默认（再次加载默认 YAML）:")
C.apply_robot_config(RobotConfig.from_yaml("config/robot.default.yaml"))
chk("duration 回到 180", DecisionEngine.MATCH_DURATION_S == 180.0)
chk("penalty 回到 10", C.Thresholds.PLACEMENT_PENALTY_PER_TARGET == 10)
chk(f"drop_forward 回到 {_yaml_default:g}",
    abs(C.Placement.DROP_FORWARD_MM - _yaml_default) < 1e-9,
    f"实际={C.Placement.DROP_FORWARD_MM}")
shutil.rmtree(tmp)

print(f"\nP1.5 结果: {sum(ok)}/{len(ok)} 通过")
assert all(ok)
