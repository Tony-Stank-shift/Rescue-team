"""
measure_delivery_rate.py —— 分场景测量"送达率/得分"（用于改进项 ①：取消伤员绝对优先）

为什么需要它：默认仿真局只有 12 个可得分目标，看不出"目标选择策略"的差别。
本脚本额外构造**赛题真实配比**的局（初赛 8普通/4核心/4伤员/4危险、决赛 10/5/5/5），
才能量化"伤员绝对优先"造成的损失（普通/核心永远不入选）。

用法（仓库根目录）：
    PYTHONPATH=src python3 tools/fix_verifiers/measure_delivery_rate.py
    PYTHONPATH=src python3 tools/fix_verifiers/measure_delivery_rate.py --scenario final --seeds 1,7,42,99,123
"""
import argparse
import logging
import os
import random
import sys

logging.disable(logging.CRITICAL)
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "src"))

from rescue_robot.perception.field_elements import SafeZoneColor  # noqa: E402
from rescue_robot.perception.target_types import (  # noqa: E402
    PRELIMINARY_TARGETS, FINAL_TARGETS, TargetColor, TargetShape,
)
from rescue_robot.simulation.integrated_sim import IntegratedSim  # noqa: E402

RED = SafeZoneColor.RED

# 赛题真实配比（初赛/决赛）——与 docs/audit/COMPLIANCE_AUDIT.md 的实物清单一致
SCENARIOS = {
    "preliminary": [
        (PRELIMINARY_TARGETS[(TargetColor.GREEN, TargetShape.CUBE)], 8),           # 普通
        (PRELIMINARY_TARGETS[(TargetColor.BLACK, TargetShape.TRIANGULAR_PYRAMID)], 4),  # 核心
        (PRELIMINARY_TARGETS[(TargetColor.ORANGE, TargetShape.CUBOID)], 4),        # 伤员
        (PRELIMINARY_TARGETS[(TargetColor.LIGHT_BLUE, TargetShape.CUBE)], 4),      # 危险
    ],
    "final": [
        (FINAL_TARGETS[(TargetColor.GREEN, TargetShape.CYLINDER)], 10),
        (FINAL_TARGETS[(TargetColor.BLACK, TargetShape.CONE_FRUSTUM)], 5),
        (FINAL_TARGETS[(TargetColor.ORANGE, TargetShape.CUBOID)], 5),
        (FINAL_TARGETS[(TargetColor.LIGHT_BLUE, TargetShape.SPHERE)], 5),
    ],
}


def build_specs(seed, scenario):
    """按赛题配比生成目标清单（位置随机，避开安全区与出发区）。"""
    rng = random.Random(seed * 7919 + 13)
    specs = []
    for info, count in SCENARIOS[scenario]:
        for _ in range(count):
            while True:
                x, y = rng.uniform(400, 2600), rng.uniform(400, 2600)
                if 1150 <= x <= 1850 and (y <= 380 or y >= 2620):
                    continue                      # 避开双方安全区
                if x <= 400 and (y <= 400 or y >= 2600):
                    continue                      # 避开出发区
                specs.append((info, x, y))
                break
    return specs


def run(seed, scenario, use_default=False):
    if use_default:
        sim = IntegratedSim(seed=seed, my_color=RED, start_zone=1)
        sim.setup_match()
    else:
        sim = IntegratedSim(seed=seed, my_color=RED, start_zone=1)
        sim.setup_match(target_specs=build_specs(seed, scenario))
    n = 0
    while not sim.is_terminal and n < 9000:
        f = sim.step()
        n += 1
    dv = [t for t in sim.targets if t.delivered]
    vd = [t for t in dv if t.delivered_valid]
    scoreable = [t for t in sim.targets if not t.dangerous]
    return {
        "score": f["score"], "delivered": len(dv), "valid": len(vd),
        "scoreable": len(scoreable), "steps": n,
    }


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--scenario", default="all",
                    choices=["all", "default", "preliminary", "final"])
    ap.add_argument("--seeds", default="1,7,42,99,123")
    args = ap.parse_args()
    seeds = [int(s) for s in args.seeds.split(",") if s.strip()]

    print("=" * 84)
    print("  送达率/得分测量（目标选择策略效果）")
    print("=" * 84)
    plans = []
    if args.scenario in ("all", "default"):
        plans.append(("默认局(12 可得分)", "default", True))
    if args.scenario in ("all", "preliminary"):
        plans.append(("初赛配比 8普/4核/4伤", "preliminary", False))
    if args.scenario in ("all", "final"):
        plans.append(("决赛配比 10普/5核/5伤", "final", False))

    totals = {}
    for title, scenario, use_default in plans:
        rows = [run(s, scenario, use_default) for s in seeds]
        avg = {k: sum(r[k] for r in rows) / len(rows) for k in rows[0]}
        print(f"\n▶ {title}")
        for s, r in zip(seeds, rows):
            print(f"    seed={s:>3}  score={r['score']:>4}  delivered={r['delivered']:>2}/"
                  f"{r['scoreable']:<2} valid={r['valid']}")
        print(f"    ── 平均: score={avg['score']:.1f}  delivered={avg['delivered']:.1f}/"
              f"{avg['scoreable']:.0f}  valid={avg['valid']:.1f}")
        totals[title] = avg
    print("\n" + "=" * 84)
    for title, avg in totals.items():
        print(f"  {title:<24} 平均分 {avg['score']:.1f}｜平均送达 {avg['delivered']:.1f} 个")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
