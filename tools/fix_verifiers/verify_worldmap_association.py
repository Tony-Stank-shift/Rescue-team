#!/usr/bin/env python3
"""verify_worldmap_association —— 世界地图"同一个物体"的数据关联阈值回归

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
现场事故（2026-09-18）：视野里识别出了绿色/橙色物体，车却不去抓
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

根因：`WorldMap.ASSOCIATION_DISTANCE_MM` 定死 **100mm**，而真机上**同一个物体**
的逐帧位置估计抖动实测就有 48~95mm（地平面测距误差 ∝ 距离²，再叠加里程计误差）。
抖动一超阈值，数据关联断链 → 该检测被判成"新目标"扔进待确认缓冲 →
`hits` 永远攒不到 `MIN_SEEN_COUNT=3` → **目标永远不被确认** →
`active_targets` 里没有可抓的普通物资 → 决策层转探索 → 车开走。

用的就是他现场日志里的**真实抖动序列**（不是编的）：
  run1 绿块 @~450mm：(698,1091)→(651,1103)→(641,1110)→(637,1113)→(579,1123)
                     →(545,1119)→(536,1115)→(541,1110)→(533,1108)
  run3 绿块 @~620mm：(2039,393)→(1949,419)   ← 单帧 93.7mm，距 100mm 只差 6mm

本自检同时验证**两个方向**：
  A. 旧阈值（固定 100mm）必须让这些序列**失败**（证明这个 bug 真实存在）；
  B. 新阈值（随距离自适应）必须让它们**通过**；
  C. 反向：把阈值放宽**不许**把两个真实相邻物体并成一个（防过度合并）。
"""

from __future__ import annotations

import sys
import traceback
from typing import List, Tuple

#: run1 实测：同一绿块的逐帧位置（mm）
RUN1_CORRECT = [(698, 1091), (651, 1103), (641, 1110), (637, 1113),
                (579, 1123), (545, 1119), (536, 1115), (541, 1110), (533, 1108)]
#: run3 实测：**日志里只留下 2 个采样点**（之后目标点就被禁区钳制了），
#: 凑不满 `MIN_SEEN_COUNT=3` 所以不能直接拿来当序列 —— 这里以实测的
#: 单帧抖动 93.7mm 为步长，构造一条与实测同量级的 6 帧序列。
RUN3_STEP = 93.7
RUN3_CORRECT = [(2039 - i * RUN3_STEP, 393 + i * 26.0 / 90.0 * RUN3_STEP)
                for i in range(6)]

#: 车的位置：取日志里的实际值（决定自适应阈值）
RUN1_ROBOT = (1341, 1001)
RUN3_ROBOT = (2011, 1015)


def _tgt(color_shape, pos):
    from rescue_robot.perception.target_types import (
        PRELIMINARY_TARGETS, TargetColor, TargetShape, DetectedTarget)
    info = PRELIMINARY_TARGETS[color_shape]
    return DetectedTarget(id=0, info=info, position=(float(pos[0]), float(pos[1])),
                          confidence=0.75, timestamp=0.0)


GREEN = None      # 延迟导入后再填


def _confirmed_after(wm, series, robot, fixed_gate=None, type_key=None):
    """把一串逐帧观测喂给世界地图，返回最终是否产生了**已确认**的该类型目标。

    fixed_gate 不为 None 时，临时把地图的阈值钉死成该值（用于复现旧行为）。
    """
    from rescue_robot.perception.target_types import (
        PRELIMINARY_TARGETS, TargetColor, TargetShape)
    info = PRELIMINARY_TARGETS[type_key]

    saved = None
    if fixed_gate is not None:
        saved = (wm.ASSOCIATION_DISTANCE_MM, wm.ASSOCIATION_DISTANCE_REL,
                 wm.ASSOCIATION_DISTANCE_MAX_MM)
        wm.ASSOCIATION_DISTANCE_MM = fixed_gate
        wm.ASSOCIATION_DISTANCE_REL = 0.0      # 关掉自适应 → 完全复现旧行为
        wm.ASSOCIATION_DISTANCE_MAX_MM = fixed_gate
    try:
        for i, p in enumerate(series):
            wm.update([_tgt(type_key, p)], robot_position=robot,
                      timestamp=100.0 + i * 0.02)
        return [t for t in wm.active_targets if t.info.type == info.type]
    finally:
        if saved is not None:
            (wm.ASSOCIATION_DISTANCE_MM, wm.ASSOCIATION_DISTANCE_REL,
             wm.ASSOCIATION_DISTANCE_MAX_MM) = saved


def _wm():
    from rescue_robot.perception.field_elements import FieldLayout
    from rescue_robot.perception.world_map import WorldMap
    return WorldMap(field_layout=FieldLayout.standard())


def s_old_gate_fails(ev):
    """A. 复现旧行为：固定 100mm 下，run3 量级的抖动序列确认不出目标。

    ⚠️ 如实记录：**run1 的序列在旧阈值下是能通过的**（最大抖动 58.9mm）。
    真正卡死的机制是待确认缓冲的 **50/50 位置平滑**：候选点每次都只向新检测
    移动一半，所以从平滑后位置到下一个检测的距离 ≈ 1.5×原始抖动 ——
    也就是说**有效容忍度只有阈值的约 2/3**（100mm → ≈66mm）。
    run1 的 58.9mm 离这个 66mm 只差 7mm，纯属侥幸；run3 的 93.7mm 必挂。
    """
    from rescue_robot.perception.target_types import TargetColor, TargetShape
    key = (TargetColor.GREEN, TargetShape.CUBE)

    # run3 量级（93.7mm 步长）在旧阈值下必须失败 —— 这就是现场"看到却不抓"
    got = _confirmed_after(_wm(), RUN3_CORRECT, RUN3_ROBOT,
                           fixed_gate=100.0, type_key=key)
    if got:
        return ("run3 量级的抖动序列在固定 100mm 下居然确认出了目标 —— "
                "本自检的前提不成立，请核对序列构造")
    ev.append("旧阈值(固定100mm) + run3 抖动(93.7mm/帧) → **0 个确认目标**（复现事故）")

    # 同时如实记录 run1 是"侥幸通过"的，避免后人误以为旧值一直够用
    got1 = _confirmed_after(_wm(), RUN1_CORRECT, RUN1_ROBOT,
                            fixed_gate=100.0, type_key=key)
    if not got1:
        return ("run1 序列在旧阈值下也没通过 —— 与现场日志（run1 实际确实抓到了）"
                "不符，说明本自检的数据或建模有问题")
    ev.append("旧阈值 + run1 抖动(58.9mm/帧) → 侥幸确认（有效容忍度≈66mm，只差 7mm）")
    return None


def s_new_gate_passes(ev):
    """B. 新阈值：同样的真实抖动序列必须能确认出目标。"""
    from rescue_robot.perception.target_types import TargetColor, TargetShape
    key = (TargetColor.GREEN, TargetShape.CUBE)
    for name, series, robot in (("run1", RUN1_CORRECT, RUN1_ROBOT),
                                ("run3", RUN3_CORRECT, RUN3_ROBOT)):
        wm = _wm()
        got = _confirmed_after(wm, series, robot, type_key=key)
        if not got:
            gate = wm._assoc_gate(series[0], robot)
            return (f"{name} 抖动序列仍然确认不出目标（自适应阈值 = {gate:.0f}mm，"
                    f"而逐帧抖动最大 "
                    f"{max(((series[i+1][0]-series[i][0])**2 + (series[i+1][1]-series[i][1])**2)**0.5 for i in range(len(series)-1)):.0f}mm）"
                    f" → 现场仍会'看到却不抓'")
        t = got[0]
        ev.append(f"新阈值(自适应) + {name} 抖动序列 → 确认出 {len(got)} 个目标，"
                  f"seen={t.seen_count} @({t.position[0]:.0f},{t.position[1]:.0f})")
    return None


def s_no_over_merge(ev):
    """C. 反向护栏：两个**真实相邻**物体不许被并成一个。

    取两个相距 300mm 的普通物资，在 500mm 距离下反复交替观测：
    自适应阈值在 500mm 处约 150mm < 300mm，应当**始终是两个目标**。
    """
    from rescue_robot.perception.target_types import TargetColor, TargetShape
    key = (TargetColor.GREEN, TargetShape.CUBE)
    robot = (1000.0, 1000.0)
    a = (1450.0, 1000.0)      # 距车 450mm
    b = (1750.0, 1000.0)      # 距车 750mm，与 a 相距 300mm

    wm = _wm()
    for i in range(12):
        wm.update([_tgt(key, a), _tgt(key, b)], robot_position=robot,
                  timestamp=100.0 + i * 0.02)
    n = len([t for t in wm.active_targets if t.info.type.name == "REGULAR_SUPPLY"])
    if n != 2:
        return (f"两个相距 300mm 的真实物体被并成了 {n} 个目标 → 阈值放太宽，"
                f"会把相邻物资当成同一个（自驾去一处、漏掉另一处）")
    ev.append(f"不过度合并：相距 300mm 的两个物资 → 始终 {n} 个目标")
    return None


def s_gate_scales_with_distance(ev):
    """阈值本身要随距离单调放大并且封顶。"""
    wm = _wm()
    # 注意：点必须按**到车的距离**递增排列（车的坐标是 (1000,1000)）
    pts = [(1300.0, 1000.0), (1800.0, 1000.0), (2500.0, 1000.0), (4000.0, 1000.0)]
    dists = [h for h in (300.0, 800.0, 1500.0, 3000.0)]
    gates = [wm._assoc_gate(p, (1000.0, 1000.0)) for p in pts]
    if not (gates[0] <= gates[1] <= gates[2] <= gates[3]):
        return f"阈值没有随距离单调放大: {[round(g, 1) for g in gates]}"
    if gates[-1] > wm.ASSOCIATION_DISTANCE_MAX_MM:
        return "阈值没有封顶 → 远处会把两个物体并成一个"
    if gates[0] < wm.ASSOCIATION_DISTANCE_MM:
        return "近处阈值低于基线（不该比基线还严）"
    ev.append("阈值随距离自适应且封顶: " + " / ".join(
        f"d={int(d)}mm→{g:.0f}mm" for d, g in zip(dists, gates)))
    return None


def s_safe_zone_not_selectable(ev):
    """E. 安全区里的目标**不许**被选为抓取目标（含 FREE_RUN 这条曾漏掉的路径）。

    2026-09-18 现场：`FIRST_TRIP` 走的 `get_regular_supplies()` 有 `_selectable`
    过滤（正确），但 `FREE_RUN` 与"时间紧迫兜底"直接写了
    `world_map.active_targets` —— **只过滤状态、不过滤安全区**。
    于是首趟之后，安全区里已投放的物资重新变成可选目标 → 车开过去扑空；
    导航的禁区又不让车开进安全区 → 目标点被钳制 → 车贴在禁区边缘 2mm/s 干蹭。
    用户报的"识别的都是安全区里的物体，这个不能抓"。
    """
    from rescue_robot.perception.target_types import (
        TargetColor, TargetShape)
    from rescue_robot.decision.target_selector import TargetSelector
    from rescue_robot.decision.decision_engine import StrategyState

    key = (TargetColor.GREEN, TargetShape.CUBE)
    robot = (1500.0, 1500.0)
    inside = (1500.0, 2800.0)      # 红安全区 x[1200,1800] y[2670,2970] 内
    outside = (700.0, 2000.0)      # 场地里正常位置

    wm = _wm()
    for i in range(6):             # 喂够帧数让两个目标都被确认
        wm.update([_tgt(key, inside), _tgt(key, outside)],
                  robot_position=robot, timestamp=100.0 + i * 0.02)

    if len(wm.active_targets) < 2:
        return (f"两个目标没都被确认（active={len(wm.active_targets)}）→ "
                f"本场景前提不成立，请检查喂帧数量")

    sel = TargetSelector()
    pick = sel.select_best(wm, robot, strategy_state=StrategyState.FREE_RUN,
                           time_remaining_s=120.0)
    if pick is None:
        return "FREE_RUN 下连场地里的真实物资都选不出来（过滤过严）"
    tgt = getattr(pick, "target", pick)
    sx, sy = tgt.position
    if wm.is_in_safe_zone((sx, sy)):
        return (f"FREE_RUN 选中了安全区里的目标 @({sx:.0f},{sy:.0f}) → "
                f"车会开去扑空、再被禁区钳制卡住（现场事故原样复现）")
    bad = [t for t in wm.selectable_targets if wm.is_in_safe_zone(t.position)]
    if bad:
        return f"selectable_targets 里仍含 {len(bad)} 个安全区内的目标"
    ev.append(f"安全区目标被正确排除：FREE_RUN 选中 @({sx:.0f},{sy:.0f})，"
              f"安全区内那个没被选（active={len(wm.active_targets)}, "
              f"selectable={len(wm.selectable_targets)}）")
    return None


SCENARIOS = (
    ("A 复现旧 bug：固定100mm 下真实抖动序列确认不出目标", s_old_gate_fails),
    ("B 新阈值：同样的抖动序列必须能确认出目标", s_new_gate_passes),
    ("C 反向：两个相距300mm的真实物体不许被并成一个", s_no_over_merge),
    ("D 阈值随距离单调放大并封顶", s_gate_scales_with_distance),
    ("E 安全区里的目标不许被选为抓取目标（FREE_RUN 路径）", s_safe_zone_not_selectable),
)


def main() -> int:
    print("=" * 78)
    print("  verify_worldmap_association —— 数据关联阈值（看到却不抓的直接原因）")
    print("=" * 78)
    ev, fails = [], []
    for name, fn in SCENARIOS:
        try:
            problem = fn(ev)
        except Exception:
            problem = "护栏自身抛异常:\n" + traceback.format_exc()
        print(f"  [{'PASS' if problem is None else 'FAIL'}] {name}")
        if problem is not None:
            fails.append(name)
            for line in problem.splitlines():
                print(f"         {line}")
    print("-" * 78)
    for line in ev:
        print(f"  · {line}")
    print("-" * 78)
    if fails:
        print(f"  结果: {len(SCENARIOS) - len(fails)}/{len(SCENARIOS)} 通过")
        print("  ❌ 目标确认链路仍有问题 → 现场会'看到却不抓'")
        return 1
    print(f"  结果: {len(SCENARIOS)}/{len(SCENARIOS)} 通过")
    print("  ✅ 真实抖动序列能被确认，且不会把相邻物体并成一个")
    return 0


if __name__ == "__main__":
    sys.exit(main())
