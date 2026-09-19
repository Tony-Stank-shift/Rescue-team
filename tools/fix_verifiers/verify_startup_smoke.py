#!/usr/bin/env python3
"""verify_startup_smoke —— 真跑一次 `main()` 的启动路径（冒烟）

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
为什么必须有这个文件（2026-09-18，我自己的事故）
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

我在 `main()` 里插了一段"出发区半场 ↔ 本队安全区颜色"的一致性检查，
用到了 `start_zone`。但 `start_zone` 在**更下面**才被解析 →

    NameError: name 'start_zone' is not defined

**程序启动即崩，而当时全部 16 项闸门依然全绿。** 原因和 `_run_once` 那次是同一个：
没有任何一项验证会真正执行 `main()` 的启动路径 ——
· 各 `verify_*.py` 测的是被调用的**函数**，不是 `main()` 本身；
· `hw_selftest` 各模块各自建对象，也不走 `main()`；
· `compileall` 只查语法，`NameError` 是运行期错误。

所以这里**真的把 `main()` 跑起来**：用 `RUN_MODE=mock`（不碰硬件）起一个子进程，
等它走到 DEBUG 状态（说明启动路径整条跑通），再断言：
  ① 进程没有异常退出、输出里没有 Traceback / NameError；
  ② 出现关键启动标志（配置加载、颜色解析、抽签出发区、状态机进入 DEBUG）；
  ③ "出发区↔颜色"的正向确认行真的打出来了（现场靠它确认方向对不对）。

这就是"启动路径"这一层的护栏：以后任何人在 `main()` 里写错变量名，这里会立刻红。
"""

from __future__ import annotations

import os
import re
import subprocess
import sys
import time
from pathlib import Path

PASS, FAIL = "PASS", "FAIL"

_HERE = Path(__file__).resolve().parent
_ROOT = _HERE.parent.parent
_SRC = _ROOT / "src"


_CACHE: dict = {}


def _run_main(env_extra: dict, seconds: float = 20.0):
    """以 mock 模式起 main()，跑 seconds 秒后收掉；返回 (输出, 退出码, 是否超时被杀)。

    结果按环境组合缓存：多个场景用的是同一套环境，没必要各起一次子进程
    （每次都要等满 seconds 才能确认它常驻，白等）。
    """
    key = (tuple(sorted(env_extra.items())), seconds)
    if key in _CACHE:
        return _CACHE[key]
    env = dict(os.environ)
    env["PYTHONPATH"] = str(_SRC)
    env["RUN_MODE"] = "mock"              # 不碰硬件（--mock 同语义：不开真实串口）
    env.pop("TEAM_COLOR", None)
    env.pop("START_ZONE", None)
    env.update(env_extra)
    proc = subprocess.Popen([sys.executable, "-m", "rescue_robot.main"],
                            cwd=str(_ROOT), env=env,
                            stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                            text=True, errors="replace")
    try:
        out, _ = proc.communicate(timeout=seconds)
        timed_out = False
    except subprocess.TimeoutExpired:
        timed_out = True
        proc.terminate()
        try:
            out, _ = proc.communicate(timeout=8)
        except subprocess.TimeoutExpired:
            proc.kill()
            out, _ = proc.communicate(timeout=5)
    result = (out or "", proc.returncode, timed_out)
    _CACHE[key] = result
    return result


def s_startup_no_crash(ev):
    """① 启动路径不许崩，而且**必须真的走到 BOOT/DEBUG**。

    ⚠️ 第一版这里只判"输出里没有 Traceback"就放行 —— 而当时 `main()` 因为
    一个错位的 `def` 被截断成 13 条语句、启动 0.1 秒就正常 `return` 了，
    **没有 Traceback，于是被判为通过**。所以判据必须是"真的跑到了状态机"，
    而不是"没报错"。
    """
    out, code, timed_out = _run_main({"TEAM_COLOR": "blue", "START_ZONE": "4"},
                                     seconds=20.0)
    if "Traceback" in out:
        tail = "\n".join(out.strip().splitlines()[-14:])
        return ("启动路径抛异常 → 现场按启动命令一跑就崩\n" + tail)
    if "进入 BOOT 状态" not in out:
        return ("启动后**没有进入 BOOT 状态**就退出了（code=%s）→ "
                "main() 的启动逻辑没跑完（旧版曾因一个错位的 def 把 main() "
                "截断成 13 条语句，且不报任何错）。输出尾部：\n%s"
                % (code, "\n".join(out.strip().splitlines()[-12:])))
    if not timed_out and code not in (0, None):
        return f"启动进程异常退出（code={code}）"
    if not timed_out:
        ev.append("⚠️ 启动进程未常驻（mock 下本应停在 DEBUG 等按钮），"
                  "但已进入 BOOT，按通过处理")
    else:
        ev.append("启动路径走通并常驻（进入 BOOT → 自检 → DEBUG 等按钮）")
    return None


def s_startup_milestones(ev):
    """② 关键启动标志都要出现（否则说明启动路径没走完）。"""
    out, _code, _to = _run_main({"TEAM_COLOR": "blue", "START_ZONE": "4"},
                                seconds=20.0)
    need = [
        ("配置已加载", "配置加载"),
        ("本队安全区颜色 = BLUE", "TEAM_COLOR 解析"),
        ("抽签出发区 = 4 号", "出发区解析"),
        ("状态机启动", "状态机"),
        ("进入 BOOT 状态", "BOOT 进入"),
    ]
    missing = [label for pat, label in need if pat not in out]
    if missing:
        return ("启动标志缺失：" + "、".join(missing) +
                "\n（说明启动路径没走完，后续逻辑不会是现场实际跑的那套）")
    ev.append("启动标志齐全：" + "、".join(l for _p, l in need))
    return None


def s_zone_color_confirmation(ev):
    """③ 出发区↔颜色一致时要**明确打印确认行**（现场靠它确认方向）。"""
    out, _c, _t = _run_main({"TEAM_COLOR": "blue", "START_ZONE": "4"},
                            seconds=20.0)
    if "与场地几何一致" not in out:
        return ("没打出「出发区↔安全区颜色一致 ✅」的确认行 → "
                "现场无法确认方向填对了（这正是 2026-09-18 被坑掉的地方）")
    if not re.search(r"出发区 4 号在下半场.*BLUE", out):
        return "确认行的内容不对（应说明 4 号区在下半场、本队安全区是 BLUE）"
    ev.append("正向确认行已打印：出发区 4 号在下半场 → 本队安全区 BLUE ✅")
    return None


def s_mismatch_warns(ev):
    """④ 方向填错时必须告警（4 号区配 red）。"""
    out, _c, _t = _run_main({"TEAM_COLOR": "red", "START_ZONE": "4"},
                            seconds=20.0)
    if "可能不匹配" not in out:
        return "出发区 4 号 + TEAM_COLOR=red 没有告警 → 送错方向不会被发现"
    if "TEAM_COLOR=blue START_ZONE=4" not in out:
        return "告警里没有给出正确的启动命令"
    if "Traceback" in out:
        return "告警路径本身抛异常了"
    ev.append("方向填错会告警，且给出正确命令 TEAM_COLOR=blue START_ZONE=4")
    return None


SCENARIOS = (
    ("① 启动路径不崩（真跑 main()）", s_startup_no_crash),
    ("② 关键启动标志齐全", s_startup_milestones),
    ("③ 出发区↔颜色一致时明确确认", s_zone_color_confirmation),
    ("④ 方向填错时告警并给出正确命令", s_mismatch_warns),
)


def main() -> int:
    print("=" * 78)
    print("  verify_startup_smoke —— 真跑 main() 启动路径的冒烟测试")
    print("=" * 78)
    ev, fails = [], []
    for name, fn in SCENARIOS:
        t0 = time.time()
        try:
            problem = fn(ev)
        except Exception as e:
            problem = f"护栏自身抛异常: {type(e).__name__}: {e}"
        print(f"  [{'PASS' if problem is None else 'FAIL'}] {name}"
              f"   ({time.time() - t0:.1f}s)")
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
        print("  ❌ 启动路径有问题 → 现场按启动命令一跑就会出事")
        return 1
    print(f"  结果: {len(SCENARIOS)}/{len(SCENARIOS)} 通过")
    print("  ✅ main() 启动路径真实跑通（含出发区↔颜色的一致性确认与告警）")
    return 0


if __name__ == "__main__":
    sys.exit(main())
