#!/usr/bin/env python3
"""verify_nav_deadlock —— 导航"被障碍包住时冻住"的脱离回归

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
现场事故（2026-09-18）：车贴着禁区边缘以 2mm/s 干蹭十几秒，目标在 2.4m 外
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

日志原样：
    pose=(1919,399) cmd v=+18mm/s | 目标=(1345,2820) | nav=AVOIDING
    pose=(1910,397) cmd v=+10mm/s | nav=AVOIDING
    pose=(1903,396) cmd v=+2mm/s  | nav=AVOIDING
    ⚠️ 已下发速度(cmd v=8mm/s)但 3.0s 无位移
    ...（持续 9 秒以上，位置几乎不变）

机制：
    `_is_near_obstacle` 只看"车附近 5×5 格内有没有代价 ≥ COST_OPPONENT 的格子"。
    对手被塞进代价地图时是以 350mm 半径的圆写入的，而当时 `opponent_tracker`
    报的"接触距离 139mm" —— **车已经在这个高代价圆的内部**。
    此时局部规划器的所有候选速度代价都很差，"最优"退化成"几乎不动"（2~18mm/s）。
    车不动 → 障碍位置估计不变 → **死锁，永远出不来**。

本自检验证四条：
  1. 被障碍包住时必须**果断脱离**（倒车速度达到 ESCAPE_SPEED_MM_S，而不是爬）
  2. 脱离有**上限**（不会一路倒出场）
  3. 正常无阻碍时**不许**误触发脱离
  4. 障碍清掉后自动恢复常规导航（计数器复位）
"""

from __future__ import annotations

import math
import sys
import traceback
from typing import List, Optional

PASS, FAIL = "PASS", "FAIL"


def _nav():
    from rescue_robot.perception.field_elements import FieldLayout, SafeZoneColor
    from rescue_robot.navigation.navigation_pipeline import NavigationPipeline
    return NavigationPipeline(FieldLayout.standard(), SafeZoneColor.RED, use_mock=True)


def _run(nav, pose, frames: int, opponent=None, dt: float = 0.02) -> List[float]:
    """跑若干帧，返回每帧的线速度列表。"""
    out = []
    for _ in range(frames):
        cmd = nav.update(pose, opponent_position=opponent, dt=dt)
        out.append(float(cmd.linear))
    return out


def s_trapped_escapes(ev):
    """1. 障碍圆包住车 → 必须果断倒车脱离，而不是 2mm/s 爬。"""
    from rescue_robot.navigation.navigation_pipeline import NavigationPipeline

    nav = _nav()
    pose = (1900.0, 400.0, math.radians(-170.0))
    nav.set_target(1345.0, 2820.0)          # 目标 2.4m 外（现场原样）

    # 障碍/"对手"就在车身边（现场 opponent_tracker 报的 139mm）
    opp = (1900.0 + 139.0, 400.0)
    vs = _run(nav, pose, frames=6, opponent=opp)

    escaped = [v for v in vs if v <= -NavigationPipeline.ESCAPE_SPEED_MM_S * 0.9]
    if not escaped:
        return (f"被障碍包住时没有触发果断脱离：本帧线速度 = "
                f"{[round(v, 1) for v in vs]}（应当出现 ≈"
                f"{-NavigationPipeline.ESCAPE_SPEED_MM_S:.0f}mm/s 的倒车）")
    if nav._escape_frames == 0:
        return "触发了倒车但 `_escape_frames` 没有计数（上限保护会失效）"
    ev.append(f"被障碍包住：触发果断倒车 {escaped[0]:.0f}mm/s"
              f"（对比现场干蹭 2mm/s），_escape_frames={nav._escape_frames}")
    return None


def s_escape_is_bounded(ev):
    """2. 长按脱离不许无限倒车：超过 ESCAPE_MAX_FRAMES 必须交回规划器。"""
    from rescue_robot.navigation.navigation_pipeline import NavigationPipeline

    nav = _nav()
    pose = (1900.0, 400.0, math.radians(-170.0))
    nav.set_target(1345.0, 2820.0)
    opp = (1900.0 + 139.0, 400.0)

    n = NavigationPipeline.ESCAPE_MAX_FRAMES
    vs = _run(nav, pose, frames=n + 10, opponent=opp)
    after = vs[n + 5:]
    still_escaping = [v for v in after
                      if v <= -NavigationPipeline.ESCAPE_SPEED_MM_S * 0.9]
    if still_escaping:
        return (f"超过脱离上限（{n} 帧）后仍在倒车 {still_escaping[0]:.0f}mm/s "
                f"→ 障碍是真实对手时会一路倒出场")
    ev.append(f"脱离有上限：第 {n} 帧后不再强行倒车，交回规划器")
    return None


def s_no_false_escape(ev):
    """3. 没有障碍时不许误触发脱离（否则正常行驶会莫名倒车）。"""
    from rescue_robot.navigation.navigation_pipeline import NavigationPipeline

    nav = _nav()
    pose = (500.0, 500.0, math.radians(45.0))
    nav.set_target(2500.0, 2500.0)          # 开阔地，畅通无阻
    vs = _run(nav, pose, frames=30, opponent=None)

    bad = [v for v in vs if v <= -NavigationPipeline.ESCAPE_SPEED_MM_S * 0.9]
    if bad:
        return f"开阔地上误触发脱离倒车 {bad[0]:.0f}mm/s（不该倒车）"
    if nav._escape_frames != 0:
        return f"开阔地上 `_escape_frames` 被累加到 {nav._escape_frames}"
    moving = [v for v in vs if v > 20.0]
    if not moving:
        return (f"开阔地上 30 帧里没有任何前进速度（{vs[:6]}）→ "
                f"导航在正常工况下也不走，护栏本身没意义")
    ev.append(f"开阔地不误触发：最大前进 {max(vs):.0f}mm/s，"
              f"_escape_frames 保持 0")
    return None


def s_recovers_after_cleared(ev):
    """4. 障碍清掉（且车移到开阔地）后计数器复位、恢复常规导航。

    ⚠️ 场景必须把车**移到开阔地**：安全区/场地边界的代价是**永久**写进代价地图的，
    车停在安全区旁时 `_is_near_obstacle` 恒为 True（即使"对手"已消失）。
    如果只在"附近无高代价格"时才复位，计数会一直涨、跑满上限后脱离功能永久失效
    —— 这正是第二次跑到这个场景时抓出来的 bug（修法：规划器给出正常速度时也复位）。
    """
    nav = _nav()
    pose = (1900.0, 400.0, math.radians(-170.0))
    nav.set_target(1345.0, 2820.0)
    opp = (1900.0 + 139.0, 400.0)

    _run(nav, pose, frames=5, opponent=opp)
    if nav._escape_frames == 0:
        return "前 5 帧没有进入脱离状态，无法验证恢复路径"

    # 移到开阔地（远离安全区）再跑几帧 → 规划器恢复正常速度 → 计数必须复位
    open_pose = (900.0, 1500.0, math.radians(90.0))
    _run(nav, open_pose, frames=5, opponent=None)
    if nav._escape_frames != 0:
        return (f"回到开阔地后 `_escape_frames` 仍为 {nav._escape_frames} "
                f"→ 下一次遇到障碍会提前耗尽上限，脱离功能最终永久失效")
    ev.append("回到开阔地：脱离计数复位，恢复常规导航")
    return None


SCENARIOS = (
    ("1 被障碍包住 → 果断倒车脱离（而非 2mm/s 干蹭）", s_trapped_escapes),
    ("2 脱离有上限，不会一路倒出场", s_escape_is_bounded),
    ("3 开阔地不误触发脱离", s_no_false_escape),
    ("4 障碍清掉后自动恢复", s_recovers_after_cleared),
)


def main() -> int:
    print("=" * 78)
    print("  verify_nav_deadlock —— 导航被障碍包住时的脱离回归")
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
        print("  ❌ 导航仍可能卡死在障碍里")
        return 1
    print(f"  结果: {len(SCENARIOS)}/{len(SCENARIOS)} 通过")
    print("  ✅ 被障碍包住会果断脱离、有上限、不误触发、可恢复")
    return 0


if __name__ == "__main__":
    sys.exit(main())
