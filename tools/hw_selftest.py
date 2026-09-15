#!/usr/bin/env python3
"""
hw_selftest —— 智能救援机器人「分部实机自检」总入口

比赛现场跑实机时用它：一次运行就能指出**哪一个具体板块出了问题**。
每个模块独立可跑，统一输出 `[PASS]/[FAIL]/[SKIP]` + 一句人话结论 + 关键证据。

用法::

    PYTHONPATH=src python3 tools/hw_selftest.py                 # 跑全部
    PYTHONPATH=src python3 tools/hw_selftest.py --list          # 列出模块
    PYTHONPATH=src python3 tools/hw_selftest.py --only servo    # 只跑一个（可逗号分隔）
    PYTHONPATH=src python3 tools/hw_selftest.py --mock          # 无硬件：环境项按 SKIP
    PYTHONPATH=src python3 tools/hw_selftest.py --yes-motion    # 允许驱动电机（先架起轮子！）
    PYTHONPATH=src python3 tools/hw_selftest.py --image a.jpg   # 视觉模块用离线图
    PYTHONPATH=src python3 tools/hw_selftest.py --port /dev/ttyS1 --duration 3

退出码：0 = 无 FAIL；1 = 有 FAIL（便于脚本/CI 判定）。
"""

import argparse
import importlib
import os
import pkgutil
import sys
import traceback

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
for _p in (HERE, os.path.join(ROOT, "src")):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import logging

from hw_selftest import framework as fw  # noqa: E402

# 默认只显示 ERROR：被检模块正常运行时也会打大量 INFO/WARNING
# （例如导航会逐帧打印"进入禁区"），会淹没自检结论本身。加 --verbose 可看全部。
logging.basicConfig(level=logging.ERROR,
                    format="      [%(levelname)s] %(name)s: %(message)s")


def _load_all_modules() -> None:
    """导入 hw_selftest 包下所有 m_*.py，触发模块注册"""
    import hw_selftest
    for m in pkgutil.iter_modules(hw_selftest.__path__):
        if m.name.startswith("m_"):
            try:
                importlib.import_module(f"hw_selftest.{m.name}")
            except Exception as e:
                print(f"⚠️  模块 {m.name} 导入失败：{e!r}")


def _ordered_names(only: str):
    names = [n for n in fw.ORDER if n in fw.REGISTRY]
    # 注册了但没写进 ORDER 的模块也带上，避免漏跑
    names += [n for n in fw.REGISTRY if n not in names]
    if only:
        want = [s.strip() for s in only.split(",") if s.strip()]
        unknown = [w for w in want if w not in fw.REGISTRY]
        if unknown:
            print(f"⚠️  未知模块：{unknown}（用 --list 查看可用模块）")
        names = [n for n in names if n in want]
    return names


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(
        description="智能救援机器人 分部实机自检（现场定位到具体板块）",
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--list", action="store_true", help="列出所有模块")
    ap.add_argument("--only", default="", help="只跑指定模块（逗号分隔）")
    ap.add_argument("--port", default=os.environ.get("CHASSIS_PORT", "/dev/ttyS1"),
                    help="串口设备（默认取环境变量 CHASSIS_PORT，否则 /dev/ttyS1）")
    ap.add_argument("--duration", type=float, default=3.0, help="遥测/速度类测试的采样秒数")
    ap.add_argument("--mock", action="store_true",
                    help="无硬件模式：环境相关项按 SKIP 呈现（而不是 FAIL）")
    ap.add_argument("--yes-motion", action="store_true",
                    help="允许驱动电机（务必先架起轮子离地空转）")
    ap.add_argument("--image", default=None, help="离线图片路径（vision 模块用）")
    ap.add_argument("--verbose", action="store_true", help="显示被检模块的 INFO 日志")
    args = ap.parse_args(argv)

    if args.verbose:
        logging.getLogger().setLevel(logging.INFO)

    _load_all_modules()

    if args.list:
        print("可用模块：")
        for n in _ordered_names(""):
            print(f"  {n:14s} {fw.REGISTRY[n][0]}")
        return 0

    names = _ordered_names(args.only)
    if not names:
        print("没有可跑的模块。")
        return 1

    if args.yes_motion:
        print("⚠️  --yes-motion 已开启：将驱动电机！请确认轮子已架起（离地空转）。")
    if args.mock:
        print("ℹ️  --mock：环境相关项按 SKIP 呈现（不视为故障）。")

    ctx = fw.Ctx(port=args.port, yes_motion=args.yes_motion, mock=args.mock,
                 image=args.image, duration_s=args.duration)

    print("=" * 78)
    print(f"  分部实机自检 —— 共 {len(names)} 个模块   串口={args.port}  "
          f"mock={args.mock}  motion={args.yes_motion}")
    print("=" * 78)

    results = []
    try:
        for name in names:
            title, fn = fw.REGISTRY[name]
            print(f"\n▶ {name} — {title}")
            try:
                r = fn(ctx)
            except Exception as e:                    # 模块内部异常不能带崩整体
                r = fw.bad(name, f"模块内部异常：{e!r}",
                           [traceback.format_exc().strip().splitlines()[-1]])
            results.append(r)
            print(f"   {r.icon} [{r.status.value}] {r.conclusion}")
            for line in r.evidence:
                print(f"        · {line}")
            if r.hint:
                print(f"        ↳ 处置：{r.hint}")
    finally:
        try:
            ctx.close()
        except Exception:
            pass

    # ── 汇总 ──
    n_pass = sum(1 for r in results if r.status == fw.Status.PASS)
    n_fail = sum(1 for r in results if r.status == fw.Status.FAIL)
    n_skip = sum(1 for r in results if r.status == fw.Status.SKIP)

    print("\n" + "=" * 78)
    print("  汇总")
    print("=" * 78)
    print(f"  {'模块':<14}{'结果':<8}结论")
    print("  " + "-" * 74)
    for r in results:
        print(f"  {r.module:<14}{r.status.value:<8}{r.conclusion[:52]}")

    failed = [r.module for r in results if r.status == fw.Status.FAIL]
    print("\n" + "=" * 78)
    print(f"  PASS={n_pass}  FAIL={n_fail}  SKIP={n_skip}")
    if failed:
        print(f"  ❌ 结论：故障模块 = {', '.join(failed)}")
        print("     按上面每个 FAIL 模块的「处置」逐条排查。")
    else:
        print("  ✅ 结论：未发现故障模块" + ("（部分模块因环境受限 SKIP）" if n_skip else ""))
    print("=" * 78)
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
