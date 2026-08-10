"""
sim_debug_gui.py —— 仿真信息叠加面板

在仿真运行时以文本/命令行方式显示实时比赛数据。
MuJoCo 自身提供 3D 可视化，此模块提供数据层面的监控。

用法:
  monitor = SimMonitor()
  state = world.step()
  monitor.update(state)
  print(monitor.status_line())
"""

import time
from dataclasses import dataclass
from typing import Any, Dict, List, Optional


@dataclass
class SimMonitorConfig:
    """监控配置。"""
    refresh_interval_s: float = 0.5
    max_event_history: int = 50
    show_fps: bool = True
    show_sensors: bool = False
    color_output: bool = True


class SimMonitor:
    """
    仿真监视器 — 收集并格式化仿真状态。

    提供:
      - 实时状态行 (score, time, state)
      - 事件历史
      - 性能统计
    """

    def __init__(self, config: SimMonitorConfig = None):
        self.config = config or SimMonitorConfig()

        self._events: List[str] = []
        self._step_count = 0
        self._t0 = time.time()
        self._last_update = time.time()
        self._fps = 0.0

        # 当前状态
        self.score = 0
        self.targets_delivered = 0
        self.targets_total = 0
        self.time_elapsed = 0.0
        self.time_remaining = 180.0
        self.robot_pose = (0.0, 0.0, 0.0)
        self.strategy = "IDLE"
        self.violations = 0

    def update(self, state) -> None:
        """
        更新监视器数据。

        Args:
            state: MatchState 或兼容对象
        """
        self._step_count += 1

        # FPS
        now = time.time()
        elapsed = now - self._last_update
        if elapsed > 0:
            self._fps = 1.0 / elapsed if elapsed > 0.001 else 0
        self._last_update = now

        # 状态数据
        self.score = getattr(state, 'score', 0)
        self.targets_delivered = getattr(state, 'targets_delivered', 0)
        self.time_elapsed = getattr(state, 'time_elapsed_s', 0.0)
        self.time_remaining = getattr(state, 'time_remaining_s', 180.0)
        self.robot_pose = getattr(state, 'robot_pose', (0.0, 0.0, 0.0))
        self.strategy = getattr(state, 'strategy_state', 'IDLE')
        self.violations = getattr(state, 'violations', 0)

        # 事件
        event = getattr(state, 'event', None)
        if event:
            self._events.append(event)
            if len(self._events) > self.config.max_event_history:
                self._events = self._events[-self.config.max_event_history:]

    def status_line(self) -> str:
        """单行状态字符串。"""
        parts = [
            f"T={self.time_elapsed:5.1f}s",
            f"Score={self.score:3d}",
            f"Del={self.targets_delivered:2d}",
            f"State={self.strategy:>12s}",
            f"Pos=({self.robot_pose[0]:.2f},{self.robot_pose[1]:.2f})",
        ]
        if self.config.show_fps:
            parts.append(f"FPS={self._fps:.0f}")
        return " | ".join(parts)

    def status_dict(self) -> dict:
        """完整状态字典。"""
        return {
            "time_elapsed_s": self.time_elapsed,
            "time_remaining_s": self.time_remaining,
            "score": self.score,
            "targets_delivered": self.targets_delivered,
            "robot_pose": self.robot_pose,
            "strategy": self.strategy,
            "violations": self.violations,
            "fps": round(self._fps, 1),
            "step": self._step_count,
            "events": self._events[-10:],
        }

    def print_status(self) -> None:
        """打印多行状态信息。"""
        print(f"\033[2J\033[H")  # Clear screen
        print(f"{'=' * 60}")
        print(f"  比赛时间: {self.time_elapsed:5.1f}s / 180s  "
              f"(剩余 {self.time_remaining:.0f}s)")
        print(f"  分数: {self.score:3d}  |  送达: {self.targets_delivered}  "
              f"|  策略: {self.strategy}")
        print(f"  机器人: ({self.robot_pose[0]:.2f}, "
              f"{self.robot_pose[1]:.2f}) @ {self.robot_pose[2]:.2f} rad")
        if self.config.show_fps:
            print(f"  FPS: {self._fps:.0f}  |  步数: {self._step_count}")
        print(f"{'─' * 60}")
        print(f"  最近事件:")
        for evt in self._events[-5:]:
            print(f"    {evt}")
        print(f"{'=' * 60}")

    def reset(self) -> None:
        """重置监视器。"""
        self._events.clear()
        self._step_count = 0
        self._t0 = time.time()
        self._last_update = time.time()
        self.score = 0
        self.targets_delivered = 0
        self.time_elapsed = 0.0


# ============================================================
# 独立测试
# ============================================================

if __name__ == "__main__":
    from dataclasses import dataclass

    @dataclass
    class _FakeState:
        time_elapsed_s: float = 0.0
        time_remaining_s: float = 180.0
        score: int = 0
        targets_delivered: int = 0
        robot_pose: tuple = (1.5, 1.5, 0.0)
        strategy_state: str = "FIRST_TRIP"
        is_terminal: bool = False
        violations: int = 0
        event: str = None

    print("=" * 50)
    print("  SimMonitor 独立测试")
    print("=" * 50)

    monitor = SimMonitor()

    # Test 1: Update
    print("\n--- 测试 1: 状态更新 ---")
    for i in range(10):
        s = _FakeState(
            time_elapsed_s=i * 0.5,
            score=i * 5,
            targets_delivered=i // 2,
            event=f"STEP_{i}" if i % 3 == 0 else None,
        )
        monitor.update(s)

    assert monitor.score == 45
    assert monitor._step_count == 10
    print(f"  {monitor.status_line()}")
    print("  ✅ 通过")

    # Test 2: Status dict
    print("\n--- 测试 2: 状态字典 ---")
    d = monitor.status_dict()
    assert d["score"] == 45
    assert "events" in d
    print(f"  keys: {list(d.keys())}")
    print("  ✅ 通过")

    # Test 3: Reset
    print("\n--- 测试 3: 重置 ---")
    monitor.reset()
    assert monitor._step_count == 0
    assert monitor.score == 0
    print("  ✅ 通过")

    print(f"\n{'=' * 50}")
    print("  SimMonitor — 全部通过 ✅")
    print(f"{'=' * 50}")
