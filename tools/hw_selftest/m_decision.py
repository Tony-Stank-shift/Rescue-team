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
    return ok(MODULE, "决策与载规则符合赛项要求（首次单独1普通/≤3/伤员单独/危险拒绝）", ev)
