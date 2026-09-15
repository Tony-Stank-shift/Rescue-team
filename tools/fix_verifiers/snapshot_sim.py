"""
snapshot_sim.py —— 快照隔离版集成仿真测量器

为什么需要它：工作树是多人**并发编辑**的。直接在工作树里跑仿真，可能刚好读到
某个文件被写到一半的状态 → 异常被 integrated_sim 的 catch-all 吞掉 → 决策退化成
WAIT → 输出 "score=0 delivered=0"，看起来像"回归"，其实是非确定性测量噪声。

本脚本先把当前代码**整体复制到 /tmp 快照**，再在快照里跑仿真：
  · 测量期间不受任何并发写入影响（同一份代码，结果可复现）；
  · 可选 `--revert` 把指定文件回退到 git HEAD，用于定位"是哪个文件的改动导致差异"。

用法（仓库根目录）：
    PYTHONPATH=src python3 tools/fix_verifiers/snapshot_sim.py
    PYTHONPATH=src python3 tools/fix_verifiers/snapshot_sim.py --revert src/rescue_robot/transport/transport_pipeline.py
    PYTHONPATH=src python3 tools/fix_verifiers/snapshot_sim.py --label "只回退我的5个文件" \
        --revert src/rescue_robot/decision/decision_engine.py,src/rescue_robot/states/autonomous_state.py
"""
import argparse
import os
import shutil
import subprocess
import sys
import tempfile
import time

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))
SEEDS = [1, 7, 42, 99, 123]

SIM = r'''
from rescue_robot.perception.field_elements import SafeZoneColor
from rescue_robot.simulation.integrated_sim import IntegratedSim
seeds = %r
for s in seeds:
    sim = IntegratedSim(seed=s, my_color=SafeZoneColor.RED, start_zone=1)
    sim.setup_match()
    n = 0
    while not sim.is_terminal and n < 9000:
        f = sim.step(); n += 1
    dv = [t for t in sim.targets if t.delivered]
    vd = [t for t in dv if t.delivered_valid]
    err = [e for e in sim.events if "ERROR" in e]
    print("RESULT|%%d|%%s|%%d|%%d|%%d|%%s" %% (
        s, f["score"], len(dv), len(sim.targets), len(vd),
        (err[0][:40] if err else "")))
'''


def build_snapshot(reverts):
    tmp = tempfile.mkdtemp(prefix="rescue_snap_")
    for d in ("src", "config", "tests"):
        src = os.path.join(ROOT, d)
        if os.path.isdir(src):
            shutil.copytree(src, os.path.join(tmp, d),
                            ignore=shutil.ignore_patterns("__pycache__", "*.pyc"))
    for rel in reverts:
        full = os.path.join(tmp, rel)
        blob = subprocess.run(["git", "show", f"HEAD:{rel}"], cwd=ROOT,
                              capture_output=True, text=True)
        if blob.returncode != 0:
            print(f"  ⚠️  回退失败 {rel}: {blob.stderr.strip()[:80]}")
            continue
        with open(full, "w") as fh:
            fh.write(blob.stdout)
    return tmp


def run(snap):
    env = dict(os.environ)
    env["PYTHONPATH"] = os.path.join(snap, "src")
    proc = subprocess.run([sys.executable, "-c", SIM % (SEEDS,)], cwd=snap, env=env,
                          capture_output=True, text=True)
    rows = []
    for line in proc.stdout.splitlines():
        if line.startswith("RESULT|"):
            _, seed, score, dv, total, vd, err = line.split("|")
            rows.append((seed, score, dv, total, vd, err.strip()))
    if not rows:
        print("  ⚠️  无结果输出，stderr 尾部：")
        print("      " + (proc.stderr.strip().splitlines() or ["(空)"])[-1][:160])
    return rows


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--revert", default="",
                    help="逗号分隔的仓库相对路径，回退到 git HEAD（用于定位改动来源）")
    ap.add_argument("--label", default="当前工作树快照")
    args = ap.parse_args()

    reverts = [p.strip() for p in args.revert.split(",") if p.strip()]
    t0 = time.time()
    snap = build_snapshot(reverts)
    print("=" * 78)
    print(f"  快照隔离仿真测量 —— {args.label}")
    print(f"  快照目录: {snap}")
    if reverts:
        print(f"  已回退到 HEAD: {', '.join(os.path.basename(r) for r in reverts)}")
    print("=" * 78)
    rows = run(snap)
    for seed, score, dv, total, vd, err in rows:
        flag = "✅" if int(vd) == int(dv.split("/")[0]) and int(score) > 0 else "❌"
        print(f"  {flag} seed={seed:>3}  score={score:>4}  delivered={dv:<5} valid={vd}"
              + (f"  异常={err}" if err else ""))
    print("-" * 78)
    if rows:
        scores = [int(r[1]) for r in rows]
        vals = [int(r[4]) for r in rows]
        print(f"  小结: score {min(scores)}~{max(scores)}｜valid {min(vals)}~{max(vals)}"
              f"｜耗时 {time.time() - t0:.1f}s")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
