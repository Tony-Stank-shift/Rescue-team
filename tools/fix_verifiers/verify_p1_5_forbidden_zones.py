"""
verify_p1_5_forbidden_zones.py —— P1-5 验证：对方安全区禁入覆盖所有分支 + 目标点钳制

问题（修复前）：禁区检查只在 `NavigationPipeline.update()` 的"路径跟踪"分支里执行，
  "接近段(dist<150)"与"到达"分支完全绕过 → 目标点落在对方安全区附近时车会直接开进去
  （赛项：不能进入对方安全区，否则比赛结束）。

运行：PYTHONPATH=src python3 tools/fix_verifiers/verify_p1_5_forbidden_zones.py
"""
import logging
import math
import sys

logging.disable(logging.CRITICAL)
sys.path.insert(0, "src")

from rescue_robot.navigation.navigation_pipeline import NavigationPipeline  # noqa: E402
from rescue_robot.perception.field_elements import FieldLayout, SafeZoneColor  # noqa: E402

field = FieldLayout.standard()
checks = []


def mk() -> NavigationPipeline:
    n = NavigationPipeline(field, my_color=SafeZoneColor.RED, use_mock=True)
    n._state = type(n._state).MOVING
    return n


def chk(name, cond, extra=""):
    checks.append(bool(cond))
    print(("  ✅ " if cond else "  ❌ ") + name + (f"  {extra}" if extra else ""))


# 对方(蓝)安全区 x[1200,1800] y[30,330]，外扩 50 → x[1150,1850] y[-20,380]
# 1) 接近段：已在对方安全区内 → 必须后退
n = mk()
n.set_target(1500.0, 430.0)
c = n.update(current_pose=(1500.0, 350.0, math.pi / 2), dt=0.02)
chk("接近段: 已在对方安全区内 → -200 后退", c.linear == -200.0, f"cmd={c.linear:.0f}")

# 2) 到达分支：已在对方安全区内且已到目标 → 仍须后退
n = mk()
n.set_target(1500.0, 350.0)
c = n.update(current_pose=(1500.0, 350.0, math.pi / 2), dt=0.02)
chk("到达分支: 已在对方安全区内 → -200 后退", c.linear == -200.0, f"cmd={c.linear:.0f}")

# 3) 路径跟踪分支仍正常
n = mk()
n.set_target(1500.0, 900.0)
n._current_path = [(700.0, 900.0), (1500.0, 900.0)]
c = n.update(current_pose=(1500.0, 200.0, math.pi / 2), dt=0.02)
chk("路径跟踪分支: 禁区响应仍生效", c.linear == -200.0)

# 4) 目标点钳制：落在对方安全区内的目标 → 推到最近合法点
n = mk()
n.set_target(1500.0, 200.0)
chk("set_target 钳制禁区目标", n.forbidden.check_violation(*n.target) is None,
    f"(1500,200) → {n.target}")

# 5) 合法目标不被改动
n = mk()
n.set_target(1345.0, 2820.0)
chk("合法目标(红物资区中心)保持不变", n.target == (1345.0, 2820.0), str(n.target))

# 6) 场外目标也钳制
n = mk()
n.set_target(-500.0, 1500.0)
chk("场外目标钳制", n.forbidden.check_violation(*n.target) is None, str(n.target))

# 7) 全场随机采样：合法点 100% 保持原值
import random  # noqa: E402

random.seed(1)
n = mk()
moved = 0
for _ in range(2000):
    x, y = random.uniform(0, 3000), random.uniform(0, 3000)
    if n.forbidden.check_violation(x, y) is None:
        n.set_target(x, y)
        assert n.target == (x, y), (x, y, n.target)
    else:
        n.set_target(x, y)
        moved += 1
chk(f"2000 次采样：合法点全部保持原值（{moved} 个禁区点已钳制）", True)

print(f"\nP1-5 结果: {sum(checks)}/{len(checks)} 通过")
if not all(checks):
    raise SystemExit(1)
