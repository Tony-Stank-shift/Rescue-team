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
    ("S-40/S-01/N-1/B8/U4/U5/N-6 队长护栏", "verify_s40_s01_team.py"),
    ("SEARCH 搜索覆盖：中央优先 + 全场蛇形不碾物体", "verify_search_coverage.py"),
    ("PICKUP 套取链：停位/近距爬行/看门狗算转动/计时复位", "verify_pickup_chain.py"),
    ("SAFETY 自检闸门：--mock 不得碰硬件 / 动作模块必须挂闸门", "verify_selftest_safety.py"),
    ("TRUTH  自检不得说谎：遥测字段数取自固件 / 舵机助手不吃 ERR", "verify_selftest_truthfulness.py"),
    # ⚠️ 2026-09-17 现场事故后补入：真机主控循环 `AutonomousState._run_once` 的正文
    #    此前是**零覆盖** —— m_decision 调它时先造成"时间到"→ 正文不执行就 return；
    #    仿真走 integrated_sim.py，那是另写了一遍控制循环；pyflakes/compileall
    #    对"条件赋值 + 无条件使用"都不报警。事故工况在第一个场景。
    ("RUNONCE 主控循环分支矩阵：非DONE无目标/有目标/诊断分支/终场/感知异常", "verify_run_once_loop.py"),
    ("VIDEO  实时画面：端点可用 + 无帧/感知异常/端口被占都降级 + 不饿主循环", "verify_video_server.py"),
    ("ASSOC  世界地图数据关联：真实抖动序列能确认 / 不过度合并 / 安全区不被选中", "verify_worldmap_association.py"),
    ("DEADLK 导航被障碍包住：果断脱离 / 有上限 / 不误触发 / 可恢复", "verify_nav_deadlock.py"),
    ("GROUND 贴地物体几何门：腿/贴边/细长被拒 + 真物资150~1100mm不误杀", "verify_detector_ground_gates.py"),
    # ⚠️ 真跑 main() 启动路径。加它的原因：我在 main() 里插检查时写错变量位置，
    #    程序启动即 NameError 崩溃，而当时 16 项闸门**全绿** —— 没有任何一项
    #    会执行 main()。这一项专门守"启动路径"这一层。
    ("STARTUP 启动冒烟：真跑 main() 不崩 + 走到 BOOT + 出发区↔颜色确认/告警", "verify_startup_smoke.py"),
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
