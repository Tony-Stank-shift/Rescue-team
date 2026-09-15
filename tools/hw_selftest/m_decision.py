"""m_decision —— 决策与载规则：直接对着赛项硬规则检查

纯算法，无硬件依赖。检查的是"会不会因为规则理解错而违规丢分"：
  1. 首次转运必须**单独 1 个普通物资**
  2. 其后单次 **≤3 个**
  3. **伤员必须单独、一次 1 个**（不得与其它混装）
  4. **危险目标必须被拒绝**
  5. 首次必须送**普通物资**（不是核心物资/伤员）
"""

from .framework import register, ok, bad

MODULE = "decision"
TITLE = "决策与载规则（首次单独1个普通/≤3/伤员单独/危险拒绝）"


@register(MODULE, TITLE)
def run(ctx):
    from rescue_robot.perception.field_elements import FieldLayout
    from rescue_robot.perception.target_types import (
        PRELIMINARY_TARGETS, TargetColor, TargetShape)
    from rescue_robot.perception.world_map import WorldMap
    from rescue_robot.decision.target_selector import TargetSelector
    from rescue_robot.transport.load_manager import LoadManager, Violation

    ev = []
    regular = PRELIMINARY_TARGETS[(TargetColor.GREEN, TargetShape.CUBE)]
    core = PRELIMINARY_TARGETS[(TargetColor.BLACK, TargetShape.TRIANGULAR_PYRAMID)]
    injured = PRELIMINARY_TARGETS[(TargetColor.ORANGE, TargetShape.CUBOID)]
    danger = PRELIMINARY_TARGETS[(TargetColor.LIGHT_BLUE, TargetShape.CUBE)]

    # ── 1) 选择器：首次应选普通物资；自由跑应排除伤员与危险目标 ──
    wm = WorldMap(field_layout=FieldLayout.standard())
    wm.add_target(regular, (900, 900))
    wm.add_target(core, (2100, 900))
    wm.add_target(injured, (900, 2100))
    wm.add_target(danger, (2100, 2100))

    sel = TargetSelector()
    first = sel.select_best_for_first_trip(wm, (150.0, 150.0))
    if first is None:
        return bad(MODULE, "首次转运选不出目标（应为普通物资）", ev,
                   "检查 select_best_for_first_trip 的过滤条件")
    ev.append(f"首次选择：{first.info.description}（id={first.id}）")
    if first.info.type.name != "REGULAR_SUPPLY":
        return bad(MODULE, f"首次选中的不是普通物资（实际 {first.info.type.name}）→ 直接违规",
                   ev, "首次转运必须单独送 1 个普通物资")

    trip = sel.select_targets_for_trip(wm, (150.0, 150.0), max_count=3,
                                       include_injured=False, time_remaining_s=180.0)
    names = [t.info.type.name for t in trip]
    ev.append(f"自由跑选择：{names}")
    if len(trip) > 3:
        return bad(MODULE, f"单次选了 {len(trip)} 个（>3）→ 违规", ev)
    if any(n == "INJURED" for n in names):
        return bad(MODULE, "普通物资趟次里混入了伤员 → 伤员必须单独成趟", ev)
    if any(n == "DANGEROUS" for n in names):
        return bad(MODULE, "选中了危险目标 → 违规", ev)

    # ── 2) 载规则引擎 ──
    problems = []

    def expect(label, got, want):
        okk = (got == want)
        ev.append(f"{label} → {got.name if hasattr(got, 'name') else got}"
                  f"{'' if okk else f'（期望 {want.name}）'}")
        if not okk:
            problems.append(label)

    lm = LoadManager()
    # 空批次：只要求"不崩溃"；返回什么违规码属设计取舍，仅记录不判失败
    try:
        res = lm.can_load_batch([])
        ev.append(f"空批次 → {res[1].name}（不崩溃即可，此处仅记录）")
    except Exception as e:
        problems.append("空批次抛异常")
        ev.append(f"空批次 → 抛异常 {e!r} ❌ 不应崩溃（主循环会吞掉后静默不动作）")

    expect("首趟 1 普通", lm.can_load_batch([regular])[1], Violation.NONE)
    expect("首趟 普通+核心", lm.can_load_batch([regular, core])[1], Violation.FIRST_TRIP_MULTI)
    expect("首趟 仅核心", lm.can_load_batch([core])[1], Violation.FIRST_TRIP_WRONG_TYPE)
    expect("首趟 含危险", lm.can_load_batch([danger])[1], Violation.DANGEROUS_TARGET)

    # 完成首趟
    lm.load(regular, 1)
    lm.release_all()
    if lm.is_first_trip:
        problems.append("首趟趟次状态")
        ev.append("首趟放行后 is_first_trip 仍为 True（状态没推进）")

    expect("普通趟 3 个", lm.can_load_batch([regular, core, regular])[1], Violation.NONE)
    expect("普通趟 4 个", lm.can_load_batch([regular, core, regular, core])[1], Violation.OVER_LIMIT)
    expect("伤员单独", lm.can_load_batch([injured])[1], Violation.NONE)
    expect("伤员+普通", lm.can_load_batch([injured, regular])[1], Violation.INJURED_MULTI)
    expect("普通+伤员（顺序相反）", lm.can_load_batch([regular, injured])[1], Violation.INJURED_MULTI)
    expect("危险目标", lm.can_load_batch([danger])[1], Violation.DANGEROUS_TARGET)

    if problems:
        return bad(MODULE, "载规则有 " + str(len(problems)) + " 处不符合赛项规则："
                   + "、".join(problems[:3]), ev,
                   "这些都是**直接违规丢分**项，按上面逐条修 can_load_batch 的判定")

    # ── 3) S-01 / B8：两条"不报错但整场白跑"的静默故障 ──
    r_b8 = _check_b8(ev)
    if r_b8 is not None:
        return bad(MODULE, r_b8[0], ev, r_b8[1])
    r_s01 = _check_s01(ev)
    if r_s01 is not None:
        return bad(MODULE, r_s01[0], ev, r_s01[1])

    return ok(MODULE, "决策与载规则符合赛项要求（首次单独1普通/≤3/伤员单独/危险拒绝；"
                      "终场停车与场心误判防护正常）", ev)


# ============================================================
# S-01 / B8 —— 静默故障现场自查（无硬件也必须真跑真判，不得 SKIP）
# ============================================================

class _IndStub:
    def signal(self, *a, **k):
        pass


class _SmStub:
    def __init__(self):
        try:
            from rescue_robot.state_machine import RobotState
            self.state = RobotState.AUTONOMOUS
        except Exception:
            self.state = None

    def transition(self, *a, **k):
        pass


class _NavStub:
    """最小导航桩：只记目标 + 统计 clear_target 调用次数"""

    def __init__(self):
        self.target = None
        self.cleared = 0

    def set_target(self, x, y):
        self.target = (x, y)
        return True          # 新契约：返回是否接受（见 S_NEW_NAV_TARGET_CONTRACT.md）

    def clear_target(self):
        self.target = None
        self.cleared += 1

    def is_arrived(self):
        return True

    def update(self, *a, **k):
        # _run_once 的第 4 步会调它；终场路径本应在此前 return，
        # 但桩要能容忍"没 return 时"的调用，否则自检自己会抛异常
        return type("_Cmd", (), {"linear": 0.0, "angular": 0.0})()

    @property
    def pose(self):
        class _P:
            x = 150.0
            y = 150.0
            theta = 1.5707963267948966
        return _P()


class _ChassisStub:
    """最小底盘桩：记录停车链路调用（真机停车 = send_velocity(0,0) + send_stop）"""

    def __init__(self):
        self._port = "/dev/stub"
        self._baudrate = 115200
        self.is_open = False
        self.stops = 0
        self.zero_vel = 0

    def open(self):
        return False

    def close(self):
        pass

    def start_match(self):
        return False

    def read_pose(self):
        return None

    def send_velocity(self, *a, **k):
        if len(a) >= 2 and abs(a[0]) < 1e-9 and abs(a[1]) < 1e-9:
            self.zero_vel += 1

    def send_stop(self):
        self.stops += 1


def _check_b8(ev):
    """B8：'无关目标靠近场心'不得被判为投放无效（旧实现会把运送途中的导航目标抢走）"""
    from rescue_robot.perception.field_elements import FieldLayout, SafeZoneColor
    from rescue_robot.perception.target_types import (
        PRELIMINARY_TARGETS, DetectedTarget, TargetColor, TargetShape)
    from rescue_robot.perception.world_map import WorldMap
    from rescue_robot.decision.decision_engine import DecisionEngine

    field = FieldLayout.standard()
    wm = WorldMap(field_layout=field)
    info = PRELIMINARY_TARGETS[(TargetColor.GREEN, TargetShape.CUBE)]
    tid = wm._create_new_target(
        DetectedTarget(id=3000, info=info, position=(1520.0, 1490.0)), 0.0)
    eng = DecisionEngine(wm, my_color=SafeZoneColor.RED)

    r1 = eng._check_invalid_transport(300.0, 300.0)
    ev.append(f"[B8-1] 没有已送达目标时、场心附近有无关目标 → 判无效={r1}（应 False）")
    if r1:
        return ("B8 误判：场心附近有无关目标就被判『投放无效』→ 运送途中导航目标被抢走，"
                "那一趟永远送不到",
                "判据必须只认『我们送达过(_delivered_ids)、又回到场心』的目标；"
                "查 DecisionEngine._check_invalid_transport")

    r2 = eng._check_invalid_transport(1520.0, 1490.0)
    ev.append(f"[B8-2] 无关目标就在场心附近 → 判无效={r2}（应 False）")
    if r2:
        return ("B8 误判：无关目标靠近场心即判无效 → 整场再也开不出新趟",
                "同上：判据要限定在 _delivered_ids 内")

    eng._delivered_ids.add(tid)
    r3 = eng._check_invalid_transport(300.0, 300.0)
    ev.append(f"[B8-3] 我们送达过的目标回到场心 → 判无效={r3}（应 True）")
    if not r3:
        return ("B8 漏判：投放过的目标被裁判放回场心却识别不出『投放无效』→ "
                "会一直以为已送达、不再补送",
                "查 _delivered_ids 的写入时机（投放成功时加入）与 ACTIVE 状态判据")
    return None


def _check_s01(ev):
    """S-01：决策引擎进 DONE 时，主循环必须清导航目标 + 显式停车 + 退出循环"""
    from rescue_robot.perception.field_elements import FieldLayout, SafeZoneColor
    from rescue_robot.decision.decision_engine import DecisionEngine, StrategyState
    from rescue_robot.perception.world_map import WorldMap
    from rescue_robot.transport.transport_pipeline import TransportPipeline
    from rescue_robot.transport.sleeve_lift import MockSleeveLift

    # ① 源码护栏：防止这一段被改回去（旧实现完全没有 DONE 分支）
    #    注意：终场收口已抽成 `_finish_match()`（"时间到"与"确认后的 DONE"共用同一条
    #    路径，且只在首次执行），所以停车三件套要连同 `_finish_match` 的源码一起查 ——
    #    只查 `_run_once` 会在重构后误报。
    try:
        import inspect
        from rescue_robot.states.autonomous_state import AutonomousState
        src = inspect.getsource(AutonomousState._run_once)
        try:
            src += inspect.getsource(AutonomousState._finish_match)
        except AttributeError:
            pass
        missing = [k for k in ("StrategyState.DONE", "_finish_match", "clear_target",
                               "_stop_chassis", "_stop_event.set()") if k not in src]
        ev.append(f"[S-01-1] 终场源码护栏：缺少 {missing or '无'}（应全都有）")
        if missing:
            return (f"终场分支被改回去了：缺少 {missing} → 比赛时间到后机器人会"
                    f"继续朝场地里冲",
                    "恢复终场收口（_finish_match 或 _run_once 开头的 DONE 分支）："
                    "clear_target() + _stop_chassis() + _stop_event.set()")
    except Exception as e:
        ev.append(f"[S-01-1] 源码护栏无法执行（{e!r}），仅做行为验证")

    # ①b N-1 护栏：**"当前帧看不到目标"绝不许被当成终场**
    #     这一条在集成仿真里永远测不出来（仿真在 setup_match 就灌真值、且不调用
    #     mark_being_transported，"地图为空"状态在仿真里不出现）。一旦回归，真机上的
    #     表现是**开局第一帧就停车退赛（0 分）**，且 DONE 单向不可恢复。
    try:
        import inspect
        from rescue_robot.decision.decision_engine import StrategyState as _SS
        eng_src = inspect.getsource(DecisionEngine.update)
        bad_empty_done = ("not self._world_map.active_targets" in eng_src
                          and "StrategyState.DONE" in eng_src.split(
                              "not self._world_map.active_targets")[1][:260])
        asrc = inspect.getsource(AutonomousState._run_once)
        has_confirm = "DONE_CONFIRM_S" in asrc or "DONE_CONFIRM_S" in inspect.getsource(
            AutonomousState)
        ev.append(f"[N-1-1] '地图为空→DONE' 是否已去除={not bad_empty_done}；"
                  f"终场确认窗口存在={has_confirm}")
        if bad_empty_done:
            return ("N-1 回归：决策引擎又把'当前帧看不到目标'当成比赛结束 → 真机开局"
                    "几帧（或摄像头掉线 3 秒后）会直接停车退赛，DONE 单向不可恢复",
                    "update() 里删除 `if not active_targets: strategy = DONE`；"
                    "只把'时间到'当硬终场，场上暂无目标保持运行等待重新检测")
        if not has_confirm:
            return ("N-1 防护缺失：终场执行没有确认窗口 → 任何误判的 DONE 都会不可恢复地"
                    "结束整场",
                    "AutonomousState 保留 DONE_CONFIRM_S 确认窗口：非'时间到'来源的 DONE "
                    "需连续保持若干秒才执行终场")
    except Exception as e:
        ev.append(f"[N-1-1] 护栏无法执行（{e!r}）")

    # ② 行为验证：真对象 + 桩底盘/桩导航/桩状态机（不启线程）
    field = FieldLayout.standard()
    wm = WorldMap(field_layout=field)
    nav = _NavStub()
    chassis = _ChassisStub()
    tp = TransportPipeline(field_layout=field, my_color=SafeZoneColor.RED,
                           use_mock=True, sleeve=MockSleeveLift())
    eng = DecisionEngine(wm, my_color=SafeZoneColor.RED)
    per = type("_PStub", (), {"world_map": wm,
                              "check_sleeve_occupied": staticmethod(lambda: True),
                              "update": staticmethod(lambda **k: None)})()
    try:
        from rescue_robot.states.autonomous_state import AutonomousState
        st = AutonomousState(
            _SmStub(), _IndStub(), perception=per, decision=eng, navigation=nav,
            transport=tp, chassis=chassis, camera=None, field_layout=field,
            my_color=SafeZoneColor.RED, use_mock=True)
    except Exception as e:
        return (f"无法构造 AutonomousState 做终场路径验证：{e!r}",
                "这是自检自身的问题，请把异常报给自检维护者")

    nav.set_target(2750.0, 2750.0)                 # 假装正在朝场地里跑
    # ⚠️ 必须用**真·时间到**触发终场：DONE 已不再由"当前帧看不到目标"产生（N-1），
    #    而非时间到来源的 DONE 要过 DONE_CONFIRM_S 确认窗口才执行（防误停车退赛）。
    #    这里把比赛起点推到很久以前 → time_remaining_s <= 0 → 权威终场、立即停车。
    eng._strategy_state = StrategyState.DONE
    eng._match_start_time = 0.0
    eng._match_elapsed = 10 ** 9      # time_remaining_s → 0（权威终场，立即停车）
    before_cleared = nav.cleared
    st._run_once(0.02)

    ev.append(f"[S-01-2] 时间到后：clear_target +{nav.cleared - before_cleared} 次、"
              f"send_stop {chassis.stops} 次、send_velocity(0,0) {chassis.zero_vel} 次、"
              f"退出主循环={st._stop_event.is_set()}")
    if nav.cleared == before_cleared or nav.target is not None:
        return ("终场不停车：时间到后导航目标没清 → 机器人继续朝上一帧目标冲",
                "终场收口必须先 navigation.clear_target()")
    if chassis.stops == 0:
        return ("终场不停车：时间到后没有下发 STOP → 底盘靠看门狗也要 ~800ms 才停",
                "终场收口必须调 _stop_chassis()（含 send_velocity(0,0)+send_stop）")
    if not st._stop_event.is_set():
        return ("终场不退出：时间到后主循环没有置停止事件 → 会一直以 50Hz 下发速度",
                "终场收口必须 _stop_event.set()")
    return None

