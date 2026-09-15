"""B1 验证：出发区 1~4 的坐标系初始化（不再写死 3 号区）。"""
import logging, sys, math
logging.disable(logging.CRITICAL)
sys.path.insert(0, "src")
from rescue_robot.perception.field_elements import StandardFieldLayout, FIELD_SIZE

ok=[]
def chk(n,c,extra=""):
    ok.append(bool(c)); print(("  ✅ " if c else "  ❌ ")+n+(f"  {extra}" if extra else ""))

L = StandardFieldLayout()
exp = {1: (150.0, 2850.0, -math.pi/2),
       2: (2850.0, 2850.0, -math.pi/2),
       3: (150.0, 150.0, math.pi/2),
       4: (2850.0, 150.0, math.pi/2)}
print("出发区位姿（唯一来源 = field_elements）:")
for z,(ex,ey,et) in exp.items():
    got = L.get_start_pose(z)
    chk(f"{z} 号区 = ({ex:.0f},{ey:.0f}, {math.degrees(et):.0f}°)",
        got and abs(got[0]-ex)<1e-6 and abs(got[1]-ey)<1e-6 and abs(got[2]-et)<1e-6, str(got))
chk("非法区号 0/5 → None", L.get_start_pose(0) is None and L.get_start_pose(5) is None)

print("AutonomousState 按 start_zone 初始化位姿/导航定位器:")
from rescue_robot.states.autonomous_state import AutonomousState
from rescue_robot.navigation.navigation_pipeline import NavigationPipeline
from rescue_robot.perception.field_elements import SafeZoneColor
class SM:
    is_locked=True
    def emergency_stop(self,r): pass
class IND:
    def signal(self,s): pass
for z in (1,2,3,4):
    nav = NavigationPipeline(StandardFieldLayout().to_field_layout() if hasattr(StandardFieldLayout,'to_field_layout') else __import__("rescue_robot.perception.field_elements",fromlist=['x']).FieldLayout.standard(),
                             my_color=SafeZoneColor.RED, use_mock=True)
    st = AutonomousState(SM(), IND(), navigation=nav, use_mock=True, start_zone=z)
    st._apply_start_pose()
    p = nav.pose
    ex,ey,et = exp[z]
    chk(f"{z} 号区 → 车/导航位姿 = ({ex:.0f},{ey:.0f})",
        abs(p.x-ex)<1e-6 and abs(p.y-ey)<1e-6 and abs(st._pose[0]-ex)<1e-6,
        f"nav=({p.x:.0f},{p.y:.0f}) internal=({st._pose[0]:.0f},{st._pose[1]:.0f})")

print(f"\nB1 结果: {sum(ok)}/{len(ok)} 通过")
assert all(ok)
