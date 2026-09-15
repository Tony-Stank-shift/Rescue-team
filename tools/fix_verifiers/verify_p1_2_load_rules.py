"""
verify_p1_2_load_rules.py —— P1-2 验证：载规则与顺序无关 / 伤员必须单独 / 空批次不崩溃

问题（修复前）：`can_load_batch` 按列表顺序累加判定 →
  `[伤员, 普通]` 被放行（先看到伤员时车上还是空的），`[普通, 伤员]` 才拦下。
  同源漏洞：`[伤员, 核心]`、以及"车上已装伤员后再批量装普通"同样被放行。
  另有 `can_load_batch([])` 在"首趟已装 1 个"时 IndexError 崩溃。

运行：PYTHONPATH=src python3 tools/fix_verifiers/verify_p1_2_load_rules.py
"""
import logging
import sys

logging.disable(logging.CRITICAL)
sys.path.insert(0, "src")

from rescue_robot.perception.target_types import (  # noqa: E402
    PRELIMINARY_TARGETS, TargetColor, TargetShape,
)
from rescue_robot.transport.load_manager import LoadManager, Violation  # noqa: E402

R = PRELIMINARY_TARGETS[(TargetColor.GREEN, TargetShape.CUBE)]              # 普通物资
C = PRELIMINARY_TARGETS[(TargetColor.BLACK, TargetShape.TRIANGULAR_PYRAMID)]  # 核心物资
I = PRELIMINARY_TARGETS[(TargetColor.ORANGE, TargetShape.CUBOID)]           # 伤员
D = PRELIMINARY_TARGETS[(TargetColor.LIGHT_BLUE, TargetShape.CUBE)]         # 危险目标

checks = []


def lm(trips_done: bool = True) -> LoadManager:
    """trips_done=True → 首趟已完成（自由趟）；False → 首趟未完成"""
    m = LoadManager()
    if trips_done:
        m.mark_first_trip_done()
    return m


def chk(name, got, exp):
    ok = got == exp
    checks.append(ok)
    print(("  ✅ " if ok else "  ❌ ") + f"{name}: {got.name} (期望 {exp.name})")


print("自由趟（首趟已完成）:")
chk("[伤员,普通]", lm().can_load_batch([I, R])[1], Violation.INJURED_MULTI)
chk("[普通,伤员]", lm().can_load_batch([R, I])[1], Violation.INJURED_MULTI)
chk("[伤员,核心]", lm().can_load_batch([I, C])[1], Violation.INJURED_MULTI)
chk("[伤员,伤员]", lm().can_load_batch([I, I])[1], Violation.INJURED_MULTI)
chk("[伤员]单独", lm().can_load_batch([I])[1], Violation.NONE)
chk("[普通,核心]混合≤3", lm().can_load_batch([R, C])[1], Violation.NONE)
chk("[普通,核心,普通]", lm().can_load_batch([R, C, R])[1], Violation.NONE)
chk("4个普通", lm().can_load_batch([R, R, R, R])[1], Violation.OVER_LIMIT)
chk("含危险", lm().can_load_batch([R, D])[1], Violation.DANGEROUS_TARGET)

print("首趟（未有效完成）:")
chk("[普通]", lm(False).can_load_batch([R])[1], Violation.NONE)
chk("[普通,普通]", lm(False).can_load_batch([R, R])[1], Violation.FIRST_TRIP_MULTI)
chk("[核心]", lm(False).can_load_batch([C])[1], Violation.FIRST_TRIP_WRONG_TYPE)
chk("[伤员]", lm(False).can_load_batch([I])[1], Violation.FIRST_TRIP_WRONG_TYPE)

print("车上已装伤员后不得再混装:")
m = lm()
m.load(I, 1)
chk("装伤员后再装普通", m.can_load_batch([R])[1], Violation.INJURED_MULTI)

print("空批次不崩溃（旧实现 IndexError）:")
m2 = LoadManager()
m2.load(R, 1)          # 首趟已装 1 个、尚未投放 → is_first_trip 仍为 True
try:
    got = m2.can_load_batch([])[1]
    print(f"  ✅ can_load_batch([]) 未崩溃 → {got.name}")
    checks.append(True)
except Exception as e:  # noqa: BLE001
    print(f"  ❌ can_load_batch([]) 崩溃: {type(e).__name__}: {e}")
    checks.append(False)

print(f"\nP1-2 结果: {sum(checks)}/{len(checks)} 通过")
if not all(checks):
    raise SystemExit(1)
