"""
verify_search_coverage.py —— 搜索算法（中央优先 + 全场蛇形）验证

为什么需要**单独**一个验证器（而不是靠集成仿真兜底）：
    实测 `IntegratedSim` 在整场 9000 帧里只有 **7 帧**"无目标可用"，
    `_get_explore_target()` 的调用次数是 **0** —— 也就是说**集成仿真根本不走探索分支**。
    （原因：仿真把目标散布在半径 600~1414mm 的环带里，相机总能看见一个。）
    所以仿真基线"没回退"对本改动**毫无证明力**，必须在这里正面验证。

而真实比赛按赛规第 16 页的布置图（图7），20 个目标是**紧凑堆在场地正中**
（6×4 格、每格 ≈40mm ⇒ 约 240×160mm，物体之间紧挨着）。从出发区角落看过去
很可能一个都看不见 → **必须探索**。所以探索路径恰恰是真实场景的关键路径。

本脚本验证四件事：
    A. 覆盖：全场地面上任意点，都能在搜索路径的某个位姿下被相机看到
    B. 安全：路径不穿越场心禁入区（中央可能是图7 那堆紧挨的物体，碾过去就废了）
    C. 真实场景：按图7 的中央堆摆法，第 1 个路点必须能看见它
    D. 集成：决策引擎在"无目标"时确实给出搜索计划的路点，且到达后会推进

⚠️ 相机可见性用的是**几何模型**（FOV 圆锥 + 成像面积门限），
   不是真实检测器。模型参数取自 config.py 与 detection.py，见下。
"""
import logging
import math
import sys

logging.disable(logging.CRITICAL)
sys.path.insert(0, "src")

from rescue_robot.decision.decision_engine import (
    DecisionEngine, ActionType,
    FIELD_SIZE, FIELD_CENTER_X, FIELD_CENTER_Y,
)
from rescue_robot.perception.field_elements import SafeZoneColor
from rescue_robot.perception.world_map import WorldMap
from rescue_robot import config

ok = []


def chk(name, cond, extra=""):
    ok.append(bool(cond))
    print(("  ✅ " if cond else "  ❌ ") + name + (f"  {extra}" if extra else ""))


# ---------------------------------------------------------------- 相机模型
# f = W / (2·tan(FOV/2))；目标成像面积 ≈ (size·f/d)²；门限 = detection._min_contour_area
FOV_DEG = float(getattr(config, "CAMERA_FOV_DEG", 77.0))
RES_W = int(getattr(config, "CAMERA_RES", (640, 480))[0])
FOCAL_PX = RES_W / (2.0 * math.tan(math.radians(FOV_DEG / 2.0)))
MIN_AREA_PX2 = 200.0          # detection.py: self._min_contour_area
HALF_FOV = math.radians(FOV_DEG / 2.0)
TARGET_MM = 40.0              # 最小目标边长（普通物资/危险目标）


def visible(px, py, ptheta, tx, ty, size=TARGET_MM):
    """目标 (tx,ty) 能否被位于 (px,py,ptheta) 的相机看见（几何模型）。"""
    dx, dy = tx - px, ty - py
    fwd = dx * math.cos(ptheta) + dy * math.sin(ptheta)
    lat = -dx * math.sin(ptheta) + dy * math.cos(ptheta)
    if fwd <= 0:                                  # 在身后
        return False
    if abs(math.atan2(lat, fwd)) > HALF_FOV:      # 出视野
        return False
    d = math.hypot(fwd, lat)
    if d < 1.0:
        return True
    return (size * FOCAL_PX / d) ** 2 >= MIN_AREA_PX2


def path_poses(plan, step_mm=120.0):
    """把路点序列离散成带朝向的位姿序列（朝向 = 该段行进方向）。"""
    poses = []
    for i in range(len(plan) - 1):
        (ax, ay), (bx, by) = plan[i], plan[i + 1]
        seg = math.hypot(bx - ax, by - ay)
        th = math.atan2(by - ay, bx - ax)
        n = max(1, int(seg / step_mm))
        for k in range(n):
            t = k / n
            poses.append((ax + (bx - ax) * t, ay + (by - ay) * t, th))
    if plan:
        (ax, ay), (bx, by) = plan[-2] if len(plan) > 1 else plan[0], plan[-1]
        poses.append((bx, by, math.atan2(by - ay, bx - ax)))
    return poses


eng = DecisionEngine(WorldMap(), my_color=SafeZoneColor.RED)

# ============================================================ A. 覆盖
print("A) 全场覆盖：地面每个网格点都能在搜索路径上被看到")
plan = eng._build_search_plan(200.0, 200.0)
poses = path_poses(plan)
print(f"   计划 {len(plan)} 个路点 → 离散成 {len(poses)} 个位姿")

miss = []
for gx in range(100, FIELD_SIZE, 100):
    for gy in range(100, FIELD_SIZE, 100):
        if eng._is_in_any_safe_zone(gx, gy):
            continue
        if not any(visible(px, py, pt, gx, gy) for px, py, pt in poses):
            miss.append((gx, gy))

# 贴墙 150mm 以内允许漏：车体 300mm 宽，那段本来就没法贴上去作业；
# 实测残余盲区只有 x∈[800,900] × y∈[50,150] 这一小块（紧贴底边围栏）。
# 计划走完后的随机兜底会继续覆盖全场（见 __get_explore_target 的兜底分支）。
miss_inner = [p for p in miss
              if 150 <= p[0] <= FIELD_SIZE - 150 and 150 <= p[1] <= FIELD_SIZE - 150]
chk("作业区（离墙 ≥150mm）全覆盖，无漏检", not miss_inner,
    f"漏 {len(miss_inner)} 个" + (f"，例如 {miss_inner[:5]}" if miss_inner else ""))
if miss:
    xs = [p[0] for p in miss]; ys = [p[1] for p in miss]
    print(f"   （已知残余盲区：x {min(xs)}~{max(xs)}, y {min(ys)}~{max(ys)}，"
          f"全部在贴墙 150mm 内）")

# 兜底必须能覆盖到外圈（旧实现只在 [900,2100]² 抽点，外圈永远抽不到）
e_fb = DecisionEngine(WorldMap(), my_color=SafeZoneColor.RED)
e_fb._search_plan = []
e_fb._search_plan_exhausted = True
fb_pts = [e_fb._get_explore_target(1500.0, 1500.0) for _ in range(400)]
chk("兜底随机点能覆盖到场地外圈（旧实现被限制在 [900,2100]²）",
    max(p[0] for p in fb_pts) > 2100 and min(p[0] for p in fb_pts) < 900,
    f"x 范围 {min(p[0] for p in fb_pts):.0f}~{max(p[0] for p in fb_pts):.0f}")
chk("兜底随机点全部落在场地内且不在安全区",
    all(0 <= p[0] <= FIELD_SIZE and 0 <= p[1] <= FIELD_SIZE for p in fb_pts)
    and not any(e_fb._is_in_any_safe_zone(*p) for p in fb_pts))

# ============================================================ B. 安全
print("B) 安全：搜索路径不穿越场心禁入区")
KO = eng.EXPLORE_CENTER_KEEPOUT_MM
worst = 1e9
for (ax, ay), (bx, by) in zip(plan, plan[1:]):
    dx, dy = bx - ax, by - ay
    l2 = dx * dx + dy * dy
    t = 0.0 if l2 == 0 else max(0.0, min(1.0,
        ((FIELD_CENTER_X - ax) * dx + (FIELD_CENTER_Y - ay) * dy) / l2))
    worst = min(worst, math.hypot(ax + t * dx - FIELD_CENTER_X,
                                  ay + t * dy - FIELD_CENTER_Y))
chk(f"整条路径距场心最近 {worst:.0f}mm ≥ 禁入 {KO:.0f}mm", worst >= KO)

chk("搜索路点不落在任何安全区",
    not [p for p in plan if eng._is_in_any_safe_zone(*p)])

# ============================================================ C. 图7 中央堆
print("C) 赛规图7 场景：20 个目标紧凑堆在场地正中（约 240×160mm）")
# 按图7 量得的 6 格 × 4 格、每格 40mm 复原（只是覆盖验证用的代表点）
cluster = []
for c in range(6):
    for r in range(4):
        cluster.append((FIELD_CENTER_X + (c - 2.5) * 40.0,
                        FIELD_CENTER_Y + (r - 1.5) * 40.0))
first = plan[0]
d0 = math.hypot(first[0] - FIELD_CENTER_X, first[1] - FIELD_CENTER_Y)
th0 = math.atan2(FIELD_CENTER_Y - first[1], FIELD_CENTER_X - first[0])
seen0 = sum(1 for (tx, ty) in cluster if visible(first[0], first[1], th0, tx, ty))
chk(f"第 1 个路点（距场心 {d0:.0f}mm、正对场心）能看见中央目标堆",
    seen0 >= 1, f"可见 {seen0}/{len(cluster)} 个")
seen_any = sum(1 for (tx, ty) in cluster
               if any(visible(px, py, pt, tx, ty) for px, py, pt in poses))
chk("整个搜索过程中中央目标堆始终有机会被看到",
    seen_any == len(cluster), f"可见 {seen_any}/{len(cluster)} 个")

# 反向对照：说明本改动的必要性 —— 出发区到中央堆的距离远超相机可靠探测距离
_d_start = math.hypot(2850 - FIELD_CENTER_X, 150 - FIELD_CENTER_Y)
_th_start = math.radians(135.0)      # 4 号区按摆位约定朝向场心方向的反向（见 RUNBOOK §1.7.1）
chk(f"对照：4 号区出发时距中央堆 {_d_start:.0f}mm 远超探测上限 "
    f"{TARGET_MM * FOCAL_PX / math.sqrt(MIN_AREA_PX2):.0f}mm → 必须靠探索",
    _d_start > TARGET_MM * FOCAL_PX / math.sqrt(MIN_AREA_PX2))

# ============================================================ D. 集成
print("D) 集成：决策引擎在无目标时给出搜索路点，到达后推进")
for start, label in (((2850.0, 150.0), "4 号区"), ((150.0, 2850.0), "1 号区")):
    e2 = DecisionEngine(WorldMap(), my_color=SafeZoneColor.RED)
    e2._search_plan = None
    e2._search_plan_exhausted = False
    wp = e2._explore_target_stable(start[0], start[1], timestamp=1.0)
    dd = math.hypot(wp[0] - FIELD_CENTER_X, wp[1] - FIELD_CENTER_Y)
    chk(f"{label}出发：首个探索点距场心 = {dd:.0f}mm（中央优先，不压物体）",
        abs(dd - e2.EXPLORE_CENTER_STANDOFF_MM) < 1.0, f"({wp[0]:.0f},{wp[1]:.0f})")
    # 到达后必须推进
    before = e2._get_explore_target(wp[0], wp[1])
    e2._search_plan = None; e2._search_plan_exhausted = False
    e3 = DecisionEngine(WorldMap(), my_color=SafeZoneColor.RED)
    e3._search_plan = None; e3._search_plan_exhausted = False
    w1 = e3._explore_target_stable(start[0], start[1], timestamp=1.0)
    e3._explore_target = None                       # 模拟"到点后重新取点"
    w2 = e3._get_explore_target(w1[0], w1[1])       # 站在 w1 上再取 → 应消费 w1
    chk(f"{label}出发：到达第 1 点后会推进到下一个路点",
        (w2[0], w2[1]) != (w1[0], w1[1]), f"{w1} → {w2}")

# 计划走完后必须有兜底，不能"无处可去"
e4 = DecisionEngine(WorldMap(), my_color=SafeZoneColor.RED)
e4._search_plan = []; e4._search_plan_exhausted = True
fb = e4._get_explore_target(1500.0, 1500.0)
chk("计划走完后退回随机兜底（不会无处可去）",
    isinstance(fb, tuple) and len(fb) == 2 and not e4._is_in_any_safe_zone(*fb),
    f"兜底点 {fb}")

# 确定性与有限性
p_a = DecisionEngine(WorldMap(), my_color=SafeZoneColor.RED)._build_search_plan(200.0, 200.0)
p_b = DecisionEngine(WorldMap(), my_color=SafeZoneColor.RED)._build_search_plan(200.0, 200.0)
chk("计划是确定性的（同输入同输出，不依赖随机）", p_a == p_b)
chk("计划有限且无重复路点", len(p_a) > 0 and len(p_a) == len(set(p_a)),
    f"{len(p_a)} 个路点")

print(f"\n搜索验证结果: {sum(ok)}/{len(ok)} 通过")
assert all(ok), "搜索覆盖验证未全部通过"
