"""
run_all.py —— 一次性跑完本轮全部修复验证脚本（供验证员/审查员复现）

用法（仓库根目录）：
    PYTHONPATH=src python3 tools/fix_verifiers/run_all.py

每个脚本内部自带断言，失败返回非 0。
"""
import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))

SCRIPTS = [
    ("P1-2  载规则顺序无关 / 伤员单独 / 空批次", "verify_p1_2_load_rules.py"),
    ("P1-5  对方安全区禁入全覆盖 + 目标点钳制", "verify_p1_5_forbidden_zones.py"),
    ("P1-8  看门狗按实际位移 + 异常链真实运动", "verify_p1_8_watchdog.py"),
    ("P1.5  YAML 配置真生效（改前/改后）", "verify_p15_config.py"),
    ("B1    出发区 1~4 坐标系初始化", "verify_b1_start_zone.py"),
    ("B5/B6 形状/颜色识别防误判", "verify_b5b6_perception.py"),
]


def main() -> int:
    env = dict(os.environ)
    env["PYTHONPATH"] = os.path.join(ROOT, "src")
    failed = []
    print("=" * 78)
    print("  修复验证套件（每项内部自带断言）")
    print("=" * 78)
    for title, name in SCRIPTS:
        path = os.path.join(HERE, name)
        proc = subprocess.run([sys.executable, path], cwd=ROOT, env=env,
                              capture_output=True, text=True)
        last = [ln for ln in proc.stdout.strip().splitlines() if ln.strip()]
        line = last[-1] if last else "(无输出)"
        status = "PASS" if proc.returncode == 0 else "FAIL"
        if proc.returncode != 0:
            failed.append(name)
        print(f"  [{status}] {title}\n         ↳ {line}")
    print("-" * 78)
    print(f"  合计: {len(SCRIPTS) - len(failed)}/{len(SCRIPTS)} 通过"
          + (f"；失败: {failed}" if failed else ""))
    return 1 if failed else 0


if __name__ == "__main__":
    raise SystemExit(main())
