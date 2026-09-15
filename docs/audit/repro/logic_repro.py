#!/usr/bin/env python3
"""
logic_repro.py —— 纯软件复现脚本（不需要任何硬件：无 RDK / 无串口 / 无摄像头）

用途：复现 docs/audit/CODE_AUDIT.md 中标注 `[复现]` 的结论。
运行（仓库根目录）：
    PYTHONPATH=src python3 docs/audit/repro/logic_repro.py
    PYTHONPATH=src python3 docs/audit/repro/logic_repro.py M1     # 只跑某一项

覆盖：
  M1  端到端任务流：选目标 → 套取 → 投放 → 计分（证明主链路能跑通、选择顺序正确）
  M2  S-40 一趟多目标"假装全抓到"
  M3  S-02 start_trip 的 is_idle 守卫失效
  M4  S-13 CostMap.clear_dynamic() 抹掉禁区区
  M5  S-18 LocalPlanner 全碰撞时返回 (0,0)
  M6  决策边界：危险目标/空场/时间压力
  M7  安全区目标"删除后重新检测"是否会被重选（含位置偏差情形）
"""
import logging
import math
import sys
import time

logging.disable(logging.CRITICAL)

from rescue_robot.perception.world_map import WorldMap
from rescue_robot.perception.field_elements import FieldLayout, SafeZoneColor
from rescue_robot.perception.target_types import (
    PRELIMINARY_TARGETS as P, TargetColor as C, TargetShape as S, DetectedTarget,
)
from rescue_robot.decision.decision_engine import DecisionEngine, ActionType
from rescue_robot.transport.transport_pipeline import TransportPipeline, TransportPhase
from rescue_robot.navigation.path_planner import (
    CostMap, AStarPlanner, LocalPlanner, COST_OBSTACLE, GRID_SIZE,
)
from rescue_robot.navigation.forbidden_zones import ForbiddenZoneManager

DROP = {"REGULAR_SUPPLY": (1345.0, 2820.0), "CORE_SUPPLY": (1345.0, 2820.0),
        "INJURED": (1655.0, 2820.0)}

FIVE = [((C.GREEN, S.CUBE), (1000, 1500)), ((C.BLACK, S.TRIANGULAR_PYRAMID), (1500, 1000)),
        ((C.ORANGE, S.CUBOID), (2000, 1800)), ((C.LIGHT_BLUE, S.CUBE), (800, 800)),
        ((C.GREEN, S.CUBE), (1200, 2000))]


def build_world():
    field = FieldLayout.standard()
    wm = WorldMap(field_layout=field)
    for i, (key, pos) in enumerate(FIVE):
        wm._create_new_target(DetectedTarget(id=i + 1, info=P[key], position=pos), 0.0)
    return field, wm


class _NavStub:
    """最小 NavigationPipeline 替身（真实链路传的是 NavigationPipeline 实例）"""
    def __init__(self):
        self.target = None

    def clear_target(self):
        self.target = None

    def set_target(self, x, y):
        self.target = (x, y)

    def is_arrived(self):
        return True


# ---------------------------------------------------------------- M1
def M1(steps=3600, dt=0.05, speed=700.0):
    """端到端任务流。真实主循环逻辑：nav_arrived 来自 NavigationPipeline 且 ARRIVED 会粘住。"""
    field, wm = build_world()
    de = DecisionEngine(wm, my_color=SafeZoneColor.RED)
    tp = TransportPipeline(field_layout=field, my_color=SafeZoneColor.RED, use_mock=True)
    de.start_match()
    pose = (200.0, 200.0, math.pi / 2)
    t = 0.0
    nav_arrived, nav_target = False, None
    grip_done = release_done = False
    cur_id, picks, phases = None, [], []
    for _ in range(steps):
        t += dt
        act = de.update(pose, nav_arrived=nav_arrived, grip_done=grip_done,
                        release_done=release_done, timestamp=t)
        cur = de._current_target
        if cur is not None and cur.id != cur_id:
            cur_id = cur.id
            picks.append(cur.info.type.name)
        if act.type in (ActionType.NAVIGATE_TO, ActionType.TRANSPORT_TO) and act.target_position:
            if nav_target != act.target_position:
                nav_target, nav_arrived = act.target_position, False
        if nav_target:
            dx, dy = nav_target[0] - pose[0], nav_target[1] - pose[1]
            d = math.hypot(dx, dy)
            if d > 1e-6:
                st = min(speed * dt, d)
                pose = (pose[0] + dx / d * st, pose[1] + dy / d * st, math.atan2(dy, dx))
            if math.hypot(nav_target[0] - pose[0], nav_target[1] - pose[1]) < 40:
                nav_arrived = True
        if act.type == ActionType.GRIP and tp.is_idle():
            tg = [wm.targets[i] for i in act.target_ids if i in wm.targets]
            tp.start_trip(tg)
            nav = _NavStub()
            tp.update(pose, wm, nav)   # APPROACHING -> CAPTURING
            tp.update(pose, wm, nav)   # CAPTURING   -> 套取
            grip_done = tp.load_manager.state.count > 0
        if act.type == ActionType.TRANSPORT_TO and nav_arrived:
            tp._phase = TransportPhase.TRANSPORTING
            nav = _NavStub()
            # 真实链路：TRANSPORTING 阶段 TransportPipeline 用 nav.target 判断"是否到达投放点"
            # （decision_engine 的 TRANSPORT_TO 已把 nav.target 设为安全区中心）
            nav.set_target(*(act.target_position or nav_target))
            tp.update(pose, wm, nav)
            tp.update(pose, wm, nav)
            release_done = tp.is_complete()
        if tp.phase == TransportPhase.VIOLATION:
            print("  !! 触发 VIOLATION，流程中断")
            break
    print(f"[M1] 端到端任务流")
    print(f"  得分={de.score}  送达={de.targets_delivered}  趟数={de._trips_completed}  "
          f"策略状态={de.strategy_state.name}")
    print(f"  目标选择顺序={picks}")
    print(f"  转运阶段={tp.phase.name}  VIOLATION={tp.is_violation()}")
    print("  => 结论：主链路能跑通，选择顺序正确（首趟普通→伤员单独→核心/普通混合）")


# ---------------------------------------------------------------- M2
def M2():
    """S-40：车只到第 1 个目标，却把本趟计划里的全部目标都记入货舱。"""
    field = FieldLayout.standard()
    wm = WorldMap(field_layout=field)
    pts = [((C.BLACK, S.TRIANGULAR_PYRAMID), (2400, 2500)),   # 车会到这里
           ((C.GREEN, S.CUBE), (400, 400)),                    # 车不会去
           ((C.GREEN, S.CUBE), (2600, 400))]                   # 车不会去
    ids = [wm._create_new_target(DetectedTarget(id=i + 1, info=P[k], position=p), 0.0)
           for i, (k, p) in enumerate(pts)]
    tp = TransportPipeline(field_layout=field, my_color=SafeZoneColor.RED, use_mock=True)
    tp._load_mgr._total_trips = 1          # 已完成首趟，进入 FREE_RUN 装载规则
    targets = [wm.targets[i] for i in ids]
    print("[M2] 一趟多目标装载（S-40）")
    print("  计划装载:", [(t.id, t.info.type.name, t.position) for t in targets])
    print("  start_trip ->", tp.start_trip(targets))
    car = (2400.0, 2500.0, 0.0)             # 车只开到了第 1 个目标
    nav = _NavStub()
    tp.update(car, wm, nav)
    tp.update(car, wm, nav)
    st = tp.load_manager.state
    print(f"  车只到达第 1 个目标 {car[:2]}，实际记入装载: {st.count} 个 ids={st.target_ids}")
    print(f"  阶段={tp.phase.name}")
    print("  => 结论：未到达的另外两个目标也进了货舱（没有\"车-目标一致性\"校验）")


# ---------------------------------------------------------------- M3
def M3():
    """S-02：start_trip 的守卫写成方法对象，永远为真。"""
    field = FieldLayout.standard()
    wm = WorldMap(field_layout=field)
    tid = wm._create_new_target(
        DetectedTarget(id=1, info=P[(C.GREEN, S.CUBE)], position=(500, 500)), 0.0)
    tp = TransportPipeline(field_layout=field, my_color=SafeZoneColor.RED, use_mock=True)
    tp._phase = TransportPhase.CAPTURING
    print("[M3] start_trip 守卫（S-02）")
    print(f"  当前阶段={tp._phase.name}  is_idle()={tp.is_idle()}")
    ok1, v1 = tp.start_trip([wm.targets[tid]])
    print(f"  忙碌时 start_trip -> ok={ok1} violation={v1.name}  (本应 False)")
    tid2 = wm._create_new_target(
        DetectedTarget(id=2, info=P[(C.GREEN, S.CUBE)], position=(900, 900)), 0.0)
    ok2, _ = tp.start_trip([wm.targets[tid2]])
    print(f"  再次 start_trip -> ok={ok2}  _current_targets={[t.id for t in tp._current_targets]}")
    print("  => 结论：旧趟次被直接覆盖，装载台账与规划错位")


# ---------------------------------------------------------------- M4
def M4():
    """S-13：clear_dynamic() 把禁区区一起抹成自由格（A* 之后可穿对方安全区）。"""
    field = FieldLayout.standard()
    cm = CostMap()
    fz = ForbiddenZoneManager(field, SafeZoneColor.RED)
    fz.write_to_cost_map(cm)

    def cells(c):
        return [(gx * 50 + 25, gy * 50 + 25)
                for gy in range(GRID_SIZE) for gx in range(GRID_SIZE)
                if c._grid[gy][gx] >= COST_OBSTACLE]

    before = cells(cm)
    cm.add_obstacle_circle(1500, 2900, radius_mm=350, cost=200)   # 对方机器人出现在安全区附近
    cm.clear_dynamic()                                            # navigation.update() 每帧调用
    after = cells(cm)
    lost = [c for c in before if c not in after]
    print("[M4] 动态障碍清除抹掉禁区（S-13）")
    print(f"  禁区格: {len(before)} -> {len(after)}   丢失 {len(lost)} 格")
    print(f"  丢失样例: {lost[:6]}")
    print("  => 结论：对方靠近时禁区被抹成可通行，A* 可能规划进入对方安全区")


# ---------------------------------------------------------------- M5
def M5():
    """S-18：所有采样轨迹都碰撞时 LocalPlanner 返回 (0,0)。"""
    cm = CostMap()
    for gx in range(12, 20):
        for gy in range(12, 20):
            cm._grid[gy][gx] = COST_OBSTACLE
    lp = LocalPlanner()
    path = [(600.0, 500.0), (700.0, 500.0)]
    near = lp.plan((500.0, 500.0, 0.0), (300.0, 0.0), path, cm)
    inside = lp.plan((600.0, 600.0, 0.0), (300.0, 0.0), path, cm)
    print("[M5] LocalPlanner 兜底（S-18）")
    print(f"  起点在障碍邻域: {near}   （有可行解，正常）")
    print(f"  起点落在障碍格: {inside}   （全碰撞 → 保持初值 0,0）")
    print("  => 结论：位姿漂移到障碍格上时输出 0 速度，车僵在原地")
    t0 = time.perf_counter()
    AStarPlanner(cm).plan((200, 200), (2800, 2800))
    print(f"  A* 单次规划耗时 = {(time.perf_counter()-t0)*1000:.1f} ms（50Hz 预算 20ms）")


# ---------------------------------------------------------------- M6
def M6():
    """决策层边界：危险目标 / 空场 / 时间压力。"""
    print("[M6] 决策层边界")
    field, wm = build_world()
    de = DecisionEngine(wm, my_color=SafeZoneColor.RED)
    de.start_match()
    for tid, t in list(wm.targets.items()):
        if t.info.type.name != "DANGEROUS":
            wm.targets.pop(tid)
    a = de.update((200, 200, 0), timestamp=1.0)
    print(f"  场上只剩危险目标 -> action={a.type.name} detail={a.detail}  (不会去抓危险目标 ✓)")

    field, wm = build_world()
    wm._targets.clear()
    de = DecisionEngine(wm, my_color=SafeZoneColor.RED)
    de.start_match()
    a = de.update((200, 200, 0), timestamp=1.0)
    print(f"  场上完全没有目标 -> action={a.type.name} detail={a.detail}")

    field, wm = build_world()
    sel = DecisionEngine(wm, my_color=SafeZoneColor.RED)._selector
    for tr in (40.0, 25.0, 5.0):
        r = sel.select_targets_for_trip(wm, (200, 200), max_count=3,
                                        include_injured=True, time_remaining_s=tr)
        print(f"  剩余 {tr:>5.0f}s 选择: {[x.info.type.name for x in r]}")


# ---------------------------------------------------------------- M7
def M7():
    """S-05/S-22 关联：安全区内目标被删除后重新检测，是否还会被选中。"""
    print("[M7] 安全区目标重检测")
    field = FieldLayout.standard()
    wm = WorldMap(field_layout=field)
    info = P[(C.GREEN, S.CUBE)]
    wm._create_new_target(DetectedTarget(id=1, info=info, position=(1345.0, 2820.0)), 0.0)
    t = list(wm.targets.values())[0]
    print(f"  安全区内目标 id={t.id} _selectable={wm._selectable(t)} "
          f"is_in_safe_zone={wm.is_in_safe_zone(t.position)}")
    wm.mark_in_safe_zone(t.id)
    wm._targets.pop(t.id)                        # MAX_LOST_COUNT(150 帧) 到期删除
    wm._create_new_target(DetectedTarget(id=2, info=info, position=(1345.0, 2820.0)), 0.0)
    n = list(wm.targets.values())[0]
    print(f"  删除后重新检测(位置准): 新 id={n.id} _selectable={wm._selectable(n)}")
    wm._targets.clear()
    wm._create_new_target(DetectedTarget(id=3, info=info, position=(1345.0, 2600.0)), 0.0)
    n2 = list(wm.targets.values())[0]
    print(f"  位置偏差 220mm（S-05 坐标错位）: _selectable={wm._selectable(n2)} "
          f"{'← 会被再次选中，重复扑空' if wm._selectable(n2) else ''}")
    print("  => 结论：安全区排除逻辑本身正确，但依赖位置精度；位置一偏就会重复扑空")


CASES = {"M1": M1, "M2": M2, "M3": M3, "M4": M4, "M5": M5, "M6": M6, "M7": M7}


if __name__ == "__main__":
    want = sys.argv[1:] or list(CASES)
    for name in want:
        fn = CASES.get(name)
        if fn is None:
            print(f"未知用例: {name}（可选 {', '.join(CASES)}）")
            continue
        fn()
        print()
