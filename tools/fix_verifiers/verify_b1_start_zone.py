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

# ---------------------------------------------------------------------------
# 车头朝向：由出发区自动推导（现场摆位约定 = 车头朝外侧角）
# 背景：朝向是「每换一个出发区就要改的量」，而它是区号的纯函数。
#       以前要人每次手算再填进命令，填错 = 整张地图旋转错。
# ---------------------------------------------------------------------------
import os
from rescue_robot.perception.field_elements import (
    auto_start_heading_deg, resolve_start_heading,
)

print("车头朝向自动推导（车头朝出发区外侧角）:")
EXP_OUT = {1: 135.0, 2: 45.0, 3: -135.0, 4: -45.0}
_saved = os.environ.pop("START_HEADING_DEG", None)
try:
    for z, deg in EXP_OUT.items():
        got = auto_start_heading_deg(z)
        chk(f"{z} 号区 自动朝向 = {deg:+.0f}°", got is not None and abs(got - deg) < 1e-9, str(got))
        # 无环境变量时应取自动值，且来源标为 auto
        d, src = resolve_start_heading(z)
        chk(f"{z} 号区 未设变量 → 用自动值且来源=auto",
            abs(d - deg) < 1e-9 and src == "auto", f"({d}, {src!r})")
        # outward 模式必须与自动推导完全一致（两处口径不许分叉）
        o = L.get_start_pose(z, heading_mode="outward")
        chk(f"{z} 号区 outward 模式 == 自动推导",
            abs(math.degrees(o[2]) - deg) < 1e-9, f"{math.degrees(o[2]):+.1f}°")
    chk("非法区号 → 自动朝向为 None（来源 none）",
        auto_start_heading_deg(0) is None and resolve_start_heading(9) == (None, "none"))

    # 显式 START_HEADING_DEG 必须**覆盖**自动推导（现场摆位不标准时的逃生口）
    os.environ["START_HEADING_DEG"] = "-90"
    chk("显式 START_HEADING_DEG=-90 → 覆盖自动推导，来源=env",
        resolve_start_heading(4) == (-90.0, "env"), str(resolve_start_heading(4)))
    # 非法值应回落到自动推导，而不是静默变 None（旧行为）
    os.environ["START_HEADING_DEG"] = "abc"
    chk("非法 START_HEADING_DEG → 回落自动推导（不是静默 None）",
        resolve_start_heading(4) == (-45.0, "auto"), str(resolve_start_heading(4)))
finally:
    os.environ.pop("START_HEADING_DEG", None)
    if _saved is not None:
        os.environ["START_HEADING_DEG"] = _saved
chk("默认(inward)朝向未被自动推导改动（1/2 号-90°, 3/4 号+90°）",
    all(abs(L.get_start_pose(z)[2] - e) < 1e-9
        for z, e in {1: -math.pi/2, 2: -math.pi/2, 3: math.pi/2, 4: math.pi/2}.items()))

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
