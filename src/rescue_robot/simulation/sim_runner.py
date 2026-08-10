"""
sim_runner.py —— 仿真启动器 (CLI + GUI)

一键启动 3D MuJoCo 比赛仿真。

用法:
  # GUI 模式 (交互式查看)
  python -m rescue_robot.simulation.sim_runner --mode gui

  # Headless (自动化测试)
  python -m rescue_robot.simulation.sim_runner --mode headless --duration 30

  # 指定比赛阶段
  python -m rescue_robot.simulation.sim_runner --phase final --mode gui

  # 自定义目标数和种子
  python -m rescue_robot.simulation.sim_runner --targets 10 --seed 42
"""

import argparse
import logging
import os
import sys
import time
from typing import Optional

logger = logging.getLogger("sim_runner")


def run_simulation(
    phase: str = "preliminary",
    mode: str = "headless",
    duration_s: float = None,
    target_count: Optional[int] = None,
    seed: Optional[int] = None,
    start_zone: int = 1,
) -> dict:
    """
    运行仿真并返回结果。

    Args:
        phase: "preliminary" | "final"
        mode: "gui" | "headless"
        duration_s: 比赛时长 (默认 180s)
        target_count: 目标数 (默认 20)
        seed: 随机种子
        start_zone: 出发区 (1-4)

    Returns:
        比赛结果字典
    """
    from .sim_world import SimWorld

    # 默认参数
    if target_count is None:
        target_count = 20 if phase == "preliminary" else 25

    logger.info(f"启动仿真: phase={phase}, mode={mode}, "
                f"targets={target_count}, seed={seed}")

    world = SimWorld(
        phase=phase,
        mode=mode,
        start_zone=start_zone,
        seed=seed,
    )
    world.setup_match(target_count=target_count)

    stats = None

    if mode == "gui":
        stats = _run_gui(world, duration_s)
    else:
        stats = _run_headless(world, duration_s)

    world.close()
    return stats


def _run_headless(world, duration_s=None) -> dict:
    """Headless 模式运行。"""
    from .sim_world import SimWorld
    result = world.run(duration_s=duration_s)
    logger.info(
        f"比赛结束: score={result.score}, "
        f"delivered={result.targets_delivered}, "
        f"trips={result.trip_count}, time={result.total_time_s:.1f}s"
    )
    return {
        "score": result.score,
        "targets_delivered": result.targets_delivered,
        "trip_count": result.trip_count,
        "violations": result.violations,
        "total_time_s": result.total_time_s,
        "events": result.events,
    }


def _run_gui(world, duration_s=None) -> dict:
    """GUI 模式运行 (使用 MuJoCo viewer)。"""
    import mujoco
    from mujoco import viewer as mujoco_viewer

    dur = duration_s or world.MATCH_DURATION_S

    logger.info("启动 MuJoCo 交互式查看器...")
    logger.info("  鼠标右键拖动 → 旋转")
    logger.info("  鼠标滚轮 → 缩放")
    logger.info("  鼠标中键拖动 → 平移")

    viewer = mujoco_viewer.launch(world._model, world._data)
    if viewer is None:
        # Fallback to headless if viewer fails
        logger.warning("MuJoCo viewer 启动失败，回退到 headless 模式")
        return _run_headless(world, duration_s)

    try:
        steps = int(dur / world.DECISION_TIMESTEP)
        for i in range(steps):
            state = world.step()

            # Update viewer
            viewer.sync()

            # Print status every 100 steps (2s)
            if i % 100 == 0:
                logger.info(
                    f"[{state.time_elapsed_s:.0f}s] "
                    f"score={state.score} "
                    f"delivered={state.targets_delivered} "
                    f"state={state.strategy_state}"
                )

            if state.is_terminal:
                break

            # Small yield to keep GUI responsive
            if i % 10 == 0:
                time.sleep(0.001)

    finally:
        viewer.close()

    return world.get_match_stats().__dict__


# ============================================================
# CLI
# ============================================================

def _cli_main():
    """CLI 入口。"""
    parser = argparse.ArgumentParser(
        description="救援机器人 - MuJoCo 3D 仿真",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例:
  python -m rescue_robot.simulation.sim_runner --mode gui
  python -m rescue_robot.simulation.sim_runner --mode headless --duration 30
  python -m rescue_robot.simulation.sim_runner --phase final --targets 25
  python -m rescue_robot.simulation.sim_runner --seed 42 --mode headless
        """,
    )
    parser.add_argument("--mode", choices=["gui", "headless"], default="headless",
                        help="运行模式 (默认: headless)")
    parser.add_argument("--phase", choices=["preliminary", "final"],
                        default="preliminary",
                        help="比赛阶段 (默认: preliminary)")
    parser.add_argument("--duration", type=float, default=None,
                        help="比赛时长秒数 (默认: 180)")
    parser.add_argument("--targets", type=int, default=None,
                        help="目标数量 (默认: 初赛20, 决赛25)")
    parser.add_argument("--seed", type=int, default=None,
                        help="随机种子 (默认: 随机)")
    parser.add_argument("--start-zone", type=int, choices=[1, 2, 3, 4],
                        default=1, help="出发区 (默认: 1)")
    parser.add_argument("--quiet", action="store_true",
                        help="静默模式")

    args = parser.parse_args()

    # 日志
    level = logging.WARNING if args.quiet else logging.INFO
    logging.basicConfig(
        level=level,
        format="%(asctime)s [%(levelname)-7s] %(message)s",
        datefmt="%H:%M:%S",
    )

    print("=" * 60)
    print("  救援机器人 - MuJoCo 3D 仿真")
    print("=" * 60)
    print(f"  阶段: {args.phase}")
    print(f"  模式: {args.mode}")
    if args.duration:
        print(f"  时长: {args.duration}s")
    if args.seed:
        print(f"  种子: {args.seed}")
    print(f"  出发区: {args.start_zone}")
    print("=" * 60)

    t0 = time.time()
    result = run_simulation(
        phase=args.phase,
        mode=args.mode,
        duration_s=args.duration,
        target_count=args.targets,
        seed=args.seed,
        start_zone=args.start_zone,
    )
    elapsed = time.time() - t0

    print(f"\n{'=' * 60}")
    print(f"  比赛结果")
    print(f"{'=' * 60}")
    print(f"  分数: {result['score']}")
    print(f"  送达: {result['targets_delivered']}")
    print(f"  趟数: {result['trip_count']}")
    print(f"  违规: {result['violations']}")
    print(f"  比赛时长: {result['total_time_s']:.1f}s")
    print(f"  仿真耗时: {elapsed:.1f}s")
    print(f"{'=' * 60}")


if __name__ == "__main__":
    _cli_main()
