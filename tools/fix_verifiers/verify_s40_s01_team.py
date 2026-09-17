"""
verify_s40_s01_team.py —— 队长本轮修复的回归护栏（S-40 / S-40第二半 / S-01 / S-02 / B8）

用法（仓库根目录）：
    PYTHONPATH=src python3 tools/fix_verifiers/verify_s40_s01_team.py

背景：这几条是本项目最容易"改回去还看不出来"的缺陷——它们都不会报错，
只会让**分数悄悄变高**（把没做到的事记成做到了）或让**整场悄悄卡住**。
因此每条都钉一个可复现的断言。

覆盖：
  1. S-40      车只到第 1 个目标时，本趟计划里的其它目标**不得**被记入装载
  2. S-40      套取前必须做"车确实在该目标处"的位姿复核（隔着距离不许记入）
  3. S-40 二半 决策引擎只给**真正送达**的目标标记"已入安全区"
  4. S-40      已持有目标时后退重试**不得抬爪**（抬爪=释放，会把货丢掉）
  5. S-01      决策引擎进入 DONE 后，主循环必须清导航目标 + 停车 + 退出
  6. S-02      忙碌时 start_trip 必须被拒
  7. B8        无关目标靠近场心**不得**误判为"投放无效"（否则运送途中被抢走导航目标）
  8. B8        已送达的目标回到场心**必须**被识别为"投放无效"
  9. 配置      sleeve_max_hold 填 >1 必须被强制回退到 1（防现场调到更差的档位）
"""
import logging
import sys
import types

logging.disable(logging.CRITICAL)

from rescue_robot.decision.decision_engine import DecisionEngine, StrategyState  # noqa: E402
from rescue_robot.perception.field_elements import FieldLayout, SafeZoneColor  # noqa: E402
from rescue_robot.perception.target_types import DetectedTarget              # noqa: E402
from rescue_robot.perception.world_map import WorldMap                      # noqa: E402
from rescue_robot.transport.transport_pipeline import (                     # noqa: E402
    TransportPhase, TransportPipeline,
)

PASS = 0
FAIL = 0


def check(name, cond, detail=""):
    global PASS, FAIL
    if cond:
        PASS += 1
        print(f"  [ok]   {name}")
    else:
        FAIL += 1
        print(f"  [FAIL] {name}  {detail}")


class _NavStub:
    """最小导航桩：只记录目标，不做规划。"""

    def __init__(self):
        self.target = None
        self._arrived = True

    def set_target(self, x, y):
        self.target = (x, y)

    def clear_target(self):
        self.target = None

    def is_arrived(self):
        return self._arrived


def _mk_target(wm, tid, pos, kind="GREEN"):
    """按颜色取初赛目标表里的 TargetInfo（与 decision/transport 用的是同一张表）。"""
    from rescue_robot.perception.target_types import PRELIMINARY_TARGETS as P
    from rescue_robot.perception.target_types import TargetColor, TargetShape
    col = getattr(TargetColor, kind)
    info = next(v for (c, _s), v in P.items() if c == col)
    return wm._create_new_target(DetectedTarget(id=tid, info=info, position=pos), 0.0)


def _tp(wm):
    tp = TransportPipeline(field_layout=FieldLayout.standard(),
                           my_color=SafeZoneColor.RED, use_mock=True)
    # 首趟必须且仅 1 个普通物资：显式标记"首趟已有效完成"，
    # 以便验证自由装载规则（判据是 _first_trip_done，不是 _total_trips）
    tp._load_mgr.mark_first_trip_done()
    return tp


# ---------------------------------------------------------------- 1/2/4
def test_s40_no_phantom_load():
    print("[1] S-40 车没去过的目标不得被记入装载")
    field = FieldLayout.standard()
    wm = WorldMap(field_layout=field)
    ids = [_mk_target(wm, 1000 + i, p) for i, p in enumerate(
        [(2400.0, 2500.0), (400.0, 400.0), (2600.0, 400.0)])]
    tp = _tp(wm)
    targets = [wm.targets[i] for i in ids]
    ok, _ = tp.start_trip(targets)
    check("start_trip 接受本趟计划(≤3)", ok)

    # ⚠️ 2026-09-17：套取位的判据已从"车心圆"改为**套取框矩形开口**
    #    （开口中心在车心**前方** DROP_FORWARD_MM=70mm）。因此"车心压在目标上"
    #    （原来用的 pose = 目标坐标）**不再**是合法套取位 —— 那时目标落在开口
    #    后方 70mm，实车是压过去的（现场现象："物体堆卡在车下"）。
    #    这里改用**正确停位**：车心在目标后方 70mm、朝向目标（θ=0 朝 +x）。
    _cap = (2400.0 - 70.0, 2500.0, 0.0)      # 车心在目标后方 70mm
    # 车只开到第 1 个目标（第 1 帧：APPROACHING→CAPTURING 并显式停车；
    # 第 2 帧：真正下压套取 —— 套取动作本身要跨帧完成）
    tp.update(_cap, wm, _NavStub())
    tp.update(_cap, wm, _NavStub())
    st = tp.load_manager.state
    check("只到第 1 个目标 → 只记入 1 个（旧实现=3）", st.count == 1,
          f"实际 count={st.count} ids={st.target_ids}")
    check("阶段已进入运送", tp.phase == TransportPhase.TRANSPORTING,
          f"phase={tp.phase.name}")
    check("未去过的 2 个目标不在货舱", set(st.target_ids) == {ids[0]},
          f"ids={st.target_ids}")

    # 隔着距离不许记入：把车挪到远处再走一帧 CAPTURING
    print("[2] S-40 套取前必须有'车确实在该目标处'的位姿复核")
    tp2 = _tp(wm)
    tp2.start_trip([wm.targets[ids[1]]])
    tp2._phase = TransportPhase.CAPTURING          # 直接进套取
    tp2.update((2400.0, 2500.0, 0.0), wm, _NavStub())   # 车离目标 2000+mm
    check("车不在目标处 → 不记入装载（打回重接近）",
          tp2.load_manager.state.count == 0,
          f"count={tp2.load_manager.state.count} phase={tp2.phase.name}")
    check("且被退回接近阶段", tp2.phase == TransportPhase.APPROACHING,
          f"phase={tp2.phase.name}")

    print("[4] S-40 已持有目标时后退重试不得抬爪（抬爪=释放）")
    tp3 = _tp(wm)
    tp3.start_trip([wm.targets[ids[0]]])
    _cap3 = (2400.0 - 70.0, 2500.0, 0.0)     # 同上：正确套取位
    tp3.update(_cap3, wm, _NavStub())        # → CAPTURING
    tp3.update(_cap3, wm, _NavStub())        # 套住 1 个
    check("已持有 1 个", len(tp3._captured) == 1,
          f"captured={len(tp3._captured)} phase={tp3.phase.name}")
    raised = {"n": 0}
    orig = tp3._sleeve.raise_up

    def spy():
        raised["n"] += 1
        return orig()

    tp3._sleeve.raise_up = spy
    tp3._begin_retreat(2400.0, 2500.0, 0.0, _NavStub())
    check("持有目标时 _begin_retreat 不调用抬爪", raised["n"] == 0,
          f"raise_up 被调用 {raised['n']} 次")


# ---------------------------------------------------------------- 3/7/8
def test_s40_second_half_and_b8():
    print("[3] S-40 第二半 决策引擎只给真正送达的目标标记入安全区")
    field = FieldLayout.standard()
    wm = WorldMap(field_layout=field)
    ids = [_mk_target(wm, 2000 + i, p) for i, p in enumerate(
        [(500.0, 500.0), (600.0, 500.0)])]
    eng = DecisionEngine(wm, my_color=SafeZoneColor.RED)
    eng._trip_targets = [wm.targets[i] for i in ids]
    eng._current_target = wm.targets[ids[0]]
    eng._strategy_state = StrategyState.FREE_RUN
    before = eng._targets_delivered
    eng._handle_free_run(500.0, 500.0, nav_arrived=True, grip_done=True,
                         release_done=True, release_valid=True,
                         delivered_ids=[ids[0]])
    from rescue_robot.perception.target_types import TargetStatus
    check("真正送达的那个 → 标记已入安全区",
          wm.targets[ids[0]].status == TargetStatus.IN_SAFE_ZONE)
    check("本趟计划里没套上的那个 → 保持 ACTIVE（留待下一趟重选）",
          wm.targets[ids[1]].status == TargetStatus.ACTIVE,
          f"status={wm.targets[ids[1]].status}")
    check("送达计数只 +1（旧实现 +2）", eng._targets_delivered == before + 1,
          f"delivered={eng._targets_delivered}")

    print("[7] B8 无关目标靠近场心不得误判为'投放无效'")
    wm2 = WorldMap(field_layout=field)
    near_center = _mk_target(wm2, 3000, (1520.0, 1490.0))   # 场心附近的无关目标
    eng2 = DecisionEngine(wm2, my_color=SafeZoneColor.RED)
    check("没有已送达目标时 → 不判无效", eng2._check_invalid_transport(300, 300) is False,
          "旧实现会因场心附近任意目标返回 True → 运送途中被抢走导航目标")
    check("无关目标在场心附近 → 仍不判无效",
          eng2._check_invalid_transport(1520, 1490) is False)

    print("[8] B8 已送达的目标回到场心 → 必须识别为投放无效")
    eng2._delivered_ids.add(near_center)
    check("我们送达过的目标回到场心 → 判为投放无效",
          eng2._check_invalid_transport(300, 300) is True)

    print("[5] S-01 终场必须清导航目标 + 停车 + 退出主循环")
    from rescue_robot.states.autonomous_state import AutonomousState
    import inspect
    src = inspect.getsource(AutonomousState._run_once)
    # 终场收口已抽成 _finish_match()（这样"时间到"与"确认后的 DONE"共用同一条路径，
    # 且只在首次执行——避免每帧重复停车/重复刷日志）
    fin = inspect.getsource(AutonomousState._finish_match)
    check("_run_once 里有终场分支（DONE → 收口）",
          "StrategyState.DONE" in src and "_finish_match" in src)
    check("终场收口 = 清导航目标 + 停车 + 退出主循环",
          "clear_target" in fin and "_stop_chassis" in fin and "_stop_event.set()" in fin,
          "缺失项：" + ", ".join(
              k for k in ("clear_target", "_stop_chassis", "_stop_event.set()")
              if k not in fin))


def test_terminal_guard():
    """N-1【blocker】：'当前帧看不到目标' **绝不许**被当成终场。

    为什么单列：这条在集成仿真里**永远测不出来**（仿真在 setup_match 就把真值灌进
    world_map，且从不调用 mark_being_transported，所以"地图为空"这个状态在仿真里
    永不出现）。一旦回归，真机上的表现是**开局第一帧就停车退赛（0 分）**，且不可恢复。
    """
    print("[10] N-1 终场判定：'看不到目标' 不得等于 '比赛结束'")
    field = FieldLayout.standard()

    # (a) 开局地图为空（真机头几帧：感知还没建图）
    wm = WorldMap(field_layout=field)
    eng = DecisionEngine(wm, my_color=SafeZoneColor.RED)
    eng.update((300.0, 300.0, 0.0), timestamp=0.0)
    check("开局地图为空 → 不得置 DONE（否则 t≈0 停车退赛）",
          eng.strategy_state != StrategyState.DONE,
          f"strategy={eng.strategy_state.name}")

    # (b) 唯一目标被套住（真机 autonomous_state 会标 BEING_TRANSPORTED）
    wm2 = WorldMap(field_layout=field)
    tid = _mk_target(wm2, 5000, (1500.0, 800.0))
    eng2 = DecisionEngine(wm2, my_color=SafeZoneColor.RED)
    eng2.update((300.0, 300.0, 0.0), timestamp=0.0)
    wm2.mark_being_transported(tid)
    eng2.update((300.0, 300.0, 0.0), timestamp=0.1)
    check("唯一目标在车上 → 不得置 DONE",
          eng2.strategy_state != StrategyState.DONE,
          f"strategy={eng2.strategy_state.name}")

    # (c) 真·时间到 → 仍必须是 DONE（终场停车不能被这次修复改坏）
    eng3 = DecisionEngine(WorldMap(field_layout=field), my_color=SafeZoneColor.RED)
    eng3._match_start_time = 0.0
    eng3.update((300.0, 300.0, 0.0), timestamp=10 ** 6)
    check("真·时间到 → 仍是 DONE（S-01 终场停车保留）",
          eng3.strategy_state == StrategyState.DONE,
          f"strategy={eng3.strategy_state.name}")

    # (d) 终场执行必须带确认窗口，且"时间到"不等待
    import inspect
    from rescue_robot.states.autonomous_state import AutonomousState
    src = inspect.getsource(AutonomousState._run_once)
    check("非'时间到'的 DONE 必须走确认窗口", "DONE_CONFIRM_S" in src and "_done_since" in src)
    check("'时间到'必须立即终场（不等确认窗口）", "time_remaining_s <= 0" in src)
    check("DONE 消失时必须能恢复比赛（复位确认计时）",
          "self._done_since = None" in src)


def test_u4_u5():
    """U4 减速带越障接线 + U5 现场颜色配置真实生效。"""
    print("[11] U4 减速带：navigation.update 必须拿到 near_speed_bump")
    import inspect
    from rescue_robot.states.autonomous_state import AutonomousState
    from rescue_robot.perception.field_elements import FieldElementType
    src = inspect.getsource(AutonomousState._run_once)
    check("_run_once 把 near_speed_bump 传给导航", "near_speed_bump=" in src)
    check("存在 _is_near_speed_bump 判定", hasattr(AutonomousState, "_is_near_speed_bump"))

    f = FieldLayout.standard()
    st = AutonomousState.__new__(AutonomousState)
    st._field = f
    n_bumps = len([e for e in f.elements if e.type == FieldElementType.SPEED_BUMP])
    check("场地里确实有减速带元素", n_bumps >= 3, f"实际 {n_bumps} 条")
    # 3 号出发区 (150,150) 前方就是减速带带区；场心不该命中
    check("出发点附近 → 判定接近减速带", st._is_near_speed_bump(150.0, 300.0) is True)
    check("场心附近 → 判定不在减速带区", st._is_near_speed_bump(1500.0, 1500.0) is False)

    print("[12] U5 现场颜色配置：TEAM_COLOR 严格校验 + YAML 颜色映射真实生效")
    from rescue_robot.main import resolve_team_color
    c_red, e_red = resolve_team_color("red")
    c_blue, e_blue = resolve_team_color("蓝")
    c_bad, e_bad = resolve_team_color("rd")          # 旧实现会静默当 BLUE！
    check("TEAM_COLOR=red → RED", c_red is not None and c_red.name == "RED")
    check("TEAM_COLOR=蓝 → BLUE", c_blue is not None and c_blue.name == "BLUE")
    check("TEAM_COLOR=rd（拼错）→ **拒绝**而不是静默 BLUE",
          c_bad is None and e_bad is not None,
          f"得到 {c_bad} / {e_bad}")

    from rescue_robot.perception.target_types import (
        get_target_config, set_color_override, PRELIMINARY_TARGETS, TargetColor, TargetShape)
    from rescue_robot.perception.target_types import CompetitionPhase as _CP
    default = {"regular": "green", "core": "black",
               "injured": "orange", "dangerous": "light_blue"}
    set_color_override(None)
    check("无覆盖时 get_target_config 原样返回内置表（零行为变化）",
          get_target_config(_CP.PRELIMINARY) is PRELIMINARY_TARGETS)
    set_color_override(default)
    sig = lambda t: sorted((c.name, s.name, i.type.name) for (c, s), i in t.items())
    check("默认 YAML 颜色与内置表**内容等价**",
          sig(get_target_config(_CP.PRELIMINARY)) == sig(PRELIMINARY_TARGETS))
    set_color_override({"regular": "red", "core": "black",
                        "injured": "orange", "dangerous": "light_blue"})
    tbl = get_target_config(_CP.PRELIMINARY)
    check("改 YAML 后 RED 立方体变成普通物资（映射真的生效）",
          (TargetColor.RED, TargetShape.CUBE) in tbl
          and tbl[(TargetColor.RED, TargetShape.CUBE)].type.name == "REGULAR_SUPPLY")
    check("改 YAML 后 GREEN 立方体不再被当普通物资",
          (TargetColor.GREEN, TargetShape.CUBE) not in tbl)
    problems = set_color_override({"core": "purple"})
    check("非法颜色必须被报出（供调用方拒绝启动）", bool(problems), f"problems={problems}")
    set_color_override(None)      # 复原，避免影响后续断言


def test_n6_drop_geometry():
    """N-6：落点不得被算到围栏上（否则首趟有效投放接近抛硬币）。"""
    print("[13] N-6 投放落点几何：任意朝向都必须落在本队子区域内")
    import math
    from rescue_robot.transport.safe_zone_placer import SafeZonePlacer
    from rescue_robot.perception.target_types import (PRELIMINARY_TARGETS as _P,
                                                      TargetColor as _TC,
                                                      TargetShape as _TS)
    from rescue_robot.config import Placement as _P2

    f = FieldLayout.standard()
    info = _P[(_TC.GREEN, _TS.CUBE)]
    L = _P2.DROP_FORWARD_MM
    check("DROP_FORWARD_MM 与区域几何相容（红物资区 y 向仅 300mm）", L <= 100.0,
          f"当前 L={L}；L>100 时落点极易被推到围栏上")

    for color in (SafeZoneColor.RED, SafeZoneColor.BLUE):
        pl = SafeZonePlacer(f, color)
        r = pl.target_area_region(info)
        cx, cy = r.x + r.width / 2, r.y + r.height / 2
        bad = []
        for deg in range(0, 360, 15):
            th = math.radians(deg)
            raw = (cx + L * math.cos(th), cy + L * math.sin(th))
            pt = pl.clamp_into_area(raw, info)
            if not pl.classify(pt, info).is_valid:
                bad.append(deg)
        check(f"{color.name} 方物资区：24 个朝向全部有效", not bad,
              f"仍无效的朝向={bad}（旧实现 9 个朝向里 4 个判 ON_FENCE/超界）")

    print("[14] N-6 反向护栏：首趟规则**不得**被放宽来'假修'")
    from rescue_robot.transport.load_manager import LoadManager
    from rescue_robot.perception.target_types import TargetType as _TT

    def _mk2(t):
        return _P[(_TC.GREEN, _TS.CUBE)].__class__(
            type=t, color=_TC.GREEN, shape=_TS.CUBE, size_mm=(40, 40, 40),
            weight_g=100, points=5, material="ABS", description=t.name)

    R_, C_ = _TT.REGULAR_SUPPLY, _TT.CORE_SUPPLY
    lm = LoadManager()
    ok1, _ = lm.can_load_batch([_mk2(R_)])
    ok2, _ = lm.can_load_batch([_mk2(C_)])
    ok3, _ = lm.can_load_batch([_mk2(R_), _mk2(C_)])
    check("首趟未完成时 1 个普通物资 → 放行（重做首趟的路径必须通）", ok1 is True)
    check("首趟未完成时 核心物资 → 拒绝（规则要求，不得放宽）", ok2 is False)
    check("首趟未完成时 普通+核心 → 拒绝（规则要求，不得放宽）", ok3 is False)
    lm.mark_first_trip_done()
    ok4, _ = lm.can_load_batch([_mk2(R_), _mk2(C_)])
    check("首趟完成后 普通+核心 → 放行", ok4 is True)

    print("[15] N-6 落点判定**不得**被放宽为'车身在区域内也算有效'")
    import inspect
    from rescue_robot.transport.transport_pipeline import TransportPipeline as _TP
    src = inspect.getsource(_TP.update)
    check("投放判定仍以落点为准（未被放大掩盖真错误）",
          "clamp_into_area" in src and "is_valid or" not in src,
          "若出现'落点 or 车身'的或判定，会把'物体落在围栏上'判成有效 → 首趟假成功")


def test_n7_n8():
    """N-7（blocker）：投放判定**不得**被钳制变成恒真；N-8：漏传 release_valid 必须失败关闭。"""
    print("[16] N-7 投放判定必须保留判别力（钳制只用于瞄准，不用于判定）")
    import inspect
    from rescue_robot.transport.transport_pipeline import TransportPipeline as _TP2
    from rescue_robot.transport.safe_zone_placer import SafeZonePlacer
    from rescue_robot.perception.target_types import (PRELIMINARY_TARGETS as _P3,
                                                      TargetColor as _TC3,
                                                      TargetShape as _TS3)
    src_upd = inspect.getsource(_TP2.update)
    check("判定用的是**未钳制**的真实落点",
          "positions = [base_drop] * len(dropped)" in src_upd,
          "若判定点来自 clamp_into_area，则 ON_FENCE/OUTSIDE/WRONG_* 分支永不可达")
    check("钳制只出现在瞄准路径 _nudge_to_valid_drop / _drop_inside",
          "clamp_into_area" not in src_upd
          or "positions" not in src_upd.split("clamp_into_area")[0][-200:])

    f = FieldLayout.standard()
    info = _P3[(_TC3.GREEN, _TS3.CUBE)]
    pl = SafeZonePlacer(f, SafeZoneColor.RED)
    bad_pts = [(1500.0, 1500.0, "场地中央"), (1345.0, 2600.0, "紫围栏南侧"),
               (1500.0, 150.0, "对方安全区"), (1655.0, 2820.0, "伤员区放物资"),
               (-800.0, 2820.0, "场外西侧")]
    false_valid = [tag for x, y, tag in bad_pts if pl.classify((x, y), info).is_valid]
    check("5 个坏落点必须全部判 INVALID（钳制曾让它们全变 valid）", not false_valid,
          f"被误判为 valid 的：{false_valid}")
    check("正确落点仍判 valid（判定没被改坏）",
          pl.classify((1345.0, 2820.0), info).is_valid is True)
    check("'物资入伤员区'的 -10 分场景可见（原被钳制掩盖）",
          "伤员区" in pl.classify((1655.0, 2820.0), info).detail)

    print("[17] N-8 漏传 release_valid 必须失败关闭（fail-closed）")
    from rescue_robot.decision.decision_engine import DecisionEngine as _DE
    from rescue_robot.perception.world_map import WorldMap as _WM
    from rescue_robot.perception.target_types import (DetectedTarget as _DT,
                                                      PRELIMINARY_TARGETS as _P4,
                                                      CompetitionPhase as _CP4)
    probe = _DE(_WM(field_layout=FieldLayout.standard()), my_color=SafeZoneColor.RED)
    probe.start_match()                      # 否则 _match_start_time=0 会直接判 DONE
    probe._world_map._create_new_target(
        _DT(id=1, info=next(iter(_P4.values())), position=(1000.0, 1500.0)), 0.0)
    probe.update((1000.0, 1500.0, 0.0), nav_arrived=True)
    probe.update((1000.0, 1500.0, 0.0), nav_arrived=True, grip_done=True)
    probe.update((200.0, 2800.0, 0.0), nav_arrived=True, grip_done=True,
                 release_done=True)          # ← 故意漏传 release_valid
    check("漏传 release_valid → 不得据投放完成进入 FREE_RUN",
          probe.strategy_state != StrategyState.FREE_RUN,
          f"strategy={probe.strategy_state.name}（旧实现 None→True 会静默放行）")


# ---------------------------------------------------------------- 6/9
def test_s02_and_config():
    print("[6] S-02 忙碌时 start_trip 必须被拒")
    field = FieldLayout.standard()
    wm = WorldMap(field_layout=field)
    tid = _mk_target(wm, 4000, (500.0, 500.0))
    tp = _tp(wm)
    tp._phase = TransportPhase.CAPTURING
    ok, _ = tp.start_trip([wm.targets[tid]])
    check("CAPTURING 中再 start_trip → 拒绝", ok is False,
          "旧实现漏了括号，方法对象恒为真 → 守卫失效")

    print("[9] sleeve_max_hold 填 >1 必须被强制回退 1")
    from rescue_robot import config as C
    from rescue_robot.innovation.config_loader import RobotConfig
    cfg = RobotConfig.from_yaml("config/robot.default.yaml")
    cfg.placement.sleeve_max_hold = 3
    C.apply_robot_config(cfg)
    check("请求 3 → 实际生效 1（填 >1 实测更差：4 个 vs 7~8 个）",
          C.Placement.SLEEVE_MAX_HOLD == 1,
          f"实际={C.Placement.SLEEVE_MAX_HOLD}")


def main():
    test_s40_no_phantom_load()
    test_s40_second_half_and_b8()
    test_terminal_guard()
    test_u4_u5()
    test_n6_drop_geometry()
    test_n7_n8()
    test_s02_and_config()
    print("-" * 66)
    print(f"  S-40 / S-01 / S-02 / B8 回归护栏: {PASS} 通过, {FAIL} 失败")
    return 1 if FAIL else 0


if __name__ == "__main__":
    sys.exit(main())
