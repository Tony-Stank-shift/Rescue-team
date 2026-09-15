"""m_navigation —— 导航：能否从出发区规划并走到目标 / 异常目标不崩

纯算法，无需硬件。
⚠️ 驱动方式必须与真机一致：`nav.update()` 吃的是**下位机里程计位姿**，
   所以这里也用外部位姿积分来驱动（对照 states/autonomous_state._run_once 的用法），
   而不是传 None 让导航用自己的内部积分器。

【test-author 已回退本人改动，恢复队长版本；我提议的增量补强见下方 REVIEW 注释】
REVIEW 提议（未落地，等队长定夺）：
  · 补 0) 纯 A* 绕障/封死两条判定（当前只测 pipeline，不测 planner 本身）
  · 补 1) 合法目标行进全程 `forbidden.is_in_field()` 判定（驶出场地=比赛结束，属致命项）
  · 补 3) 不可达目标必须**显式停止**：现在 "对方安全区内部 (1500,100)" 会一路 MOVING
    到 (1472,1472) 并报 ARRIVED（实测），残距目标 1372mm —— 不是崩溃，但会被当成"到达"。
"""

import math

from .framework import register, ok, bad

MODULE = "navigation"
TITLE = "导航（规划与行进 / 异常目标不崩）"

MAX_STEPS = 4000                      # 4000 × 20ms = 80s 仿真
DT = 0.02
START = (150.0, 150.0, math.pi / 2)   # 3 号出发区，朝 +Y（前进方向）


def _drive(nav, target, max_steps=MAX_STEPS, arrive_mm=80.0):
    """喂外部位姿驱动 nav，返回 (末位姿, 步数, 有速度输出的步数, 是否到达)"""
    x, y, th = START
    moved = 0
    for step in range(max_steps):
        cmd = nav.update((x, y, th), dt=DT)
        if abs(cmd.linear) > 1.0 or abs(cmd.angular) > 0.01:
            moved += 1
        # 差速积分：线速度沿当前朝向（与真机里程计一致）
        x += cmd.linear * math.cos(th) * DT
        y += cmd.linear * math.sin(th) * DT
        th += cmd.angular * DT
        if math.hypot(target[0] - x, target[1] - y) < arrive_mm or nav.is_arrived():
            return (x, y, th), step + 1, moved, True
    return (x, y, th), max_steps, moved, False


@register(MODULE, TITLE)
def run(ctx):
    ev = []
    from rescue_robot.navigation.navigation_pipeline import NavigationPipeline
    from rescue_robot.perception.field_elements import FieldLayout, SafeZoneColor

    def make_nav():
        return NavigationPipeline(FieldLayout.standard(),
                                  my_color=SafeZoneColor.RED, use_mock=True)

    # ── 1) 从 3 号出发区走到对角 (2750,2750) ──
    goal = (2750.0, 2750.0)
    nav = make_nav()
    nav.set_target(*goal)
    (x, y, _th), steps, moved, arrived = _drive(nav, goal)
    dist = math.hypot(goal[0] - x, goal[1] - y)
    ev.append(f"目标 {goal}：{steps} 步（{steps * DT:.1f}s）→ 位置 ({x:.0f},{y:.0f})，"
              f"剩余 {dist:.0f}mm，有速度输出 {moved} 步")

    if moved == 0:
        return bad(MODULE, "导航不产生任何速度指令（车不会动）", ev,
                   "检查 NavigationPipeline.update 的目标/规划/控制链路")
    if not arrived:
        return bad(MODULE, f"未能到达目标（剩余 {dist:.0f}mm）→ 规划或控制有问题", ev,
                   "检查 A*/纯追踪/到点判定；若中途 BLOCKED 需看避障；"
                   "注意本测试喂的是外部位姿，与真机一致")

    # ── 2) 异常目标：对方安全区内部 / 场外坐标，不应崩溃、不应冲进禁区 ──
    for label, (tx, ty) in (("对方安全区内部", (1500.0, 100.0)),
                            ("场外坐标", (9000.0, 9000.0))):
        nav2 = make_nav()
        try:
            nav2.set_target(tx, ty)
            (x2, y2, _t2), _s, _m, _a = _drive(nav2, (tx, ty), max_steps=400)
            inside = nav2.forbidden.check_violation(x2, y2) if hasattr(nav2, "forbidden") else None
            ev.append(f"{label} ({tx:.0f},{ty:.0f})：400 步后停在 ({x2:.0f},{y2:.0f})，未崩溃 ✓"
                      + (f"，终点仍在禁区内（{inside.name}）⚠️" if inside is not None else ""))
        except Exception as e:
            return bad(MODULE, f"目标为「{label}」时导航抛异常：{e!r}", ev,
                       "异常目标必须被安全处理（拒绝/停在禁区外），不能崩")

    return ok(MODULE, "导航正常（可规划并到达目标；异常目标不崩）", ev)
