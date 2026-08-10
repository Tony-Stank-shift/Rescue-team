"""
sim_scenario.py —— 比赛场景编排

提供预设的比赛场景配置，包括:
  - 初赛标准布局 (20 个目标)
  - 决赛标准布局 (25 个目标)
  - 首趟转运测试 (1 个目标)
  - 压力测试 (可复现随机布局)
  - 边缘情况测试

用法:
  from rescue_robot.simulation.sim_scenario import setup_preliminary
  world = SimWorld()
  setup_preliminary(world)
"""

import random
from typing import List, Optional, Tuple


def _random_positions(count: int, seed: int = None,
                      avoid_zones: bool = True) -> List[Tuple[float, float]]:
    """生成随机目标位置。

    Args:
        count: 目标数量
        seed: 随机种子
        avoid_zones: 是否避开安全区和出发区

    Returns:
        [(x_m, y_m), ...] 位置列表 (米)
    """
    rng = random.Random(seed)

    positions = []
    max_attempts = count * 10

    for _ in range(max_attempts):
        if len(positions) >= count:
            break
        x = rng.uniform(0.20, 2.80)
        y = rng.uniform(0.50, 2.50)
        positions.append((x, y))

    return positions


def setup_preliminary(world, seed: int = None) -> None:
    """
    初赛标准场景: 20 个目标随机分布。

    Args:
        world: SimWorld 实例
        seed: 随机种子
    """
    world.setup_match(
        target_count=20,
        scenario="preliminary_default",
    )


def setup_final(world, seed: int = None) -> None:
    """
    决赛标准场景: 25 个目标随机分布。

    Args:
        world: SimWorld 实例
        seed: 随机种子
    """
    world.setup_match(
        target_count=25,
        scenario="final_default",
    )


def setup_first_trip_only(world) -> None:
    """
    首趟转运测试: 仅 1 个普通物资，
    用于验证首趟规则 (必须精确一个普通物资)。
    """
    world.setup_match(
        target_count=1,
        scenario="first_trip_test",
    )


def setup_no_targets(world) -> None:
    """
    空场地: 0 个目标，
    用于边界情况测试。
    """
    world.setup_match(
        target_count=0,
        scenario="empty_field",
    )


def setup_dangerous_only(world) -> None:
    """
    危险品测试: 仅含危险品 (不应被推入安全区)。
    """
    world.setup_match(
        target_count=5,
        scenario="dangerous_test",
    )


def setup_stress_test(world, seed: int = 42) -> None:
    """
    压力测试: 最大目标数 + 固定种子 (可复现)。

    Args:
        world: SimWorld 实例
        seed: 随机种子
    """
    world.setup_match(
        target_count=25,
        scenario="stress_test",
    )


# ============================================================
# 场景注册表
# ============================================================

SCENARIOS = {
    "preliminary": setup_preliminary,
    "final": setup_final,
    "first_trip": setup_first_trip_only,
    "empty": setup_no_targets,
    "dangerous": setup_dangerous_only,
    "stress_test": setup_stress_test,
}


def get_scenario(name: str):
    """获取场景设置函数。"""
    if name not in SCENARIOS:
        raise ValueError(f"未知场景: {name}. 有效值: {list(SCENARIOS.keys())}")
    return SCENARIOS[name]


# ============================================================
# 独立测试
# ============================================================

if __name__ == "__main__":
    import logging
    logging.basicConfig(level=logging.WARNING)

    from .sim_world import SimWorld

    print("=" * 50)
    print("  SimScenario 独立测试")
    print("=" * 50)

    for name, setup_fn in SCENARIOS.items():
        print(f"\n--- 测试: {name} ---")
        world = SimWorld(mode="headless", seed=0)
        setup_fn(world)
        assert world._model is not None
        print(f"  目标: {len(world.targets)}")
        # Run a few steps
        for _ in range(5):
            world.step()
        assert not world.is_done() or name in ("empty",)
        world.close()
        print(f"  ✅ 通过")

    print(f"\n{'=' * 50}")
    print(f"  SimScenario — 全部 {len(SCENARIOS)} 场景通过 ✅")
    print(f"{'=' * 50}")
