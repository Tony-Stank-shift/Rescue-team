#!/usr/bin/env python3
"""verify_run_once_loop —— 真机主控循环 ``AutonomousState._run_once`` 的端到端护栏

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
为什么必须有这个文件（2026-09-17 现场事故，车倒车进场后一动不动）
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

事故现象：4 号区启动 → 自检全绿 → 倒车 1.5m 进场（`_run_startup_backup()` 走了
独立路径，所以正常）→ 进场地中央后**主循环一帧都没跑**，车原地不动，
紧接着被"无动作 15s"异常接管，日志里只剩 `保命绕圈` 刷屏。

根因：`_run_once` 里 `now` 只在 `策略状态 == DONE` 分支中被赋值，
却在函数末尾的 2Hz 诊断日志里**无条件使用**。那次赋值让 `now` 成为局部变量，
于是只要策略状态不是 DONE（比赛绝大多数时间都不是），第一帧就抛
``UnboundLocalError: local variable 'now' referenced before assignment``。

**为什么全部验证都没拦住它**（这才是真正要修的）：

  * `hw_selftest/m_decision.py` 确实调了 `_run_once`，但它先把
    `eng._strategy_state = DONE` + `_match_elapsed = 1e9` 造成"时间到"，
    函数在终场收口分支就 ``return`` 了 —— **正文一行没执行**。
  * `snapshot_sim` 走 `simulation/integrated_sim.py`，那里把控制循环
    **另写了一遍**（自己的 `_set_nav_target` 与位姿积分），根本不 import
    `AutonomousState`。
  * `pyflakes 2.4.0` **抓不到**这类"条件赋值 + 无条件使用"（实测只报未用 import）。
  * `compileall` 只查语法，不查数据流。

结论：真机上 100% 时间都在执行的那段正文，此前验证覆盖率是 **0**。
本文件用真·决策/导航/转运管线 + 桩感知/桩底盘，把 `_run_once` 按**分支矩阵**
逐条真跑一遍，任何一帧抛异常都判 FAIL —— 事故工况（非 DONE + 场上无目标）
是第一个场景，不再被"终场分支提前 return"掩盖。
"""

from __future__ import annotations

import math
import sys
import time
import traceback
from typing import List, Optional, Tuple

PASS = "PASS"
FAIL = "FAIL"


# ────────────────────────────── 桩件 ──────────────────────────────

class _IndStub:
    """指示灯桩：记录信号，不碰硬件。"""

    def __init__(self) -> None:
        self.signals: List[str] = []

    def signal(self, *a, **k):
        self.signals.append(a[0] if a else "?")


class _SmStub:
    """状态机桩：`_run_once` 里只在急停时回调它。"""

    def __init__(self) -> None:
        try:
            from rescue_robot.state_machine import RobotState
            self.state = RobotState.AUTONOMOUS
        except Exception:
            self.state = None
        self.emergencies: List[str] = []

    def transition(self, *a, **k):
        pass

    def emergency_stop(self, reason: str = ""):
        self.emergencies.append(reason)


class _PerceptionStub:
    """感知桩：提供 `_run_once` 需要的全部属性（世界地图是真的，视觉是桩）。

    为什么不用真 PerceptionPipeline：它会拉摄像头/跑 HSV，属于探测模块，
    本文件要验证的是**控制循环本身**能不能跑完整帧。视觉误检由
    `verify_b5_b6_shapes.py` 等专题验证覆盖，职责不重叠。
    """

    def __init__(self, world_map, opponent_position=None) -> None:
        self.world_map = world_map
        self.opponent_position = opponent_position
        self.vision_available = True
        self.updates = 0
        self.raise_on_update: Optional[BaseException] = None

    def update(self, *a, **k):
        if self.raise_on_update is not None:
            raise self.raise_on_update
        self.updates += 1

    def check_sleeve_occupied(self) -> bool:
        return True

    def set_frame(self, *a, **k):
        pass


class _ChassisStub:
    """底盘桩：真机接口面（read_pose / send_velocity / send_stop）。

    `read_pose` 返回**内部积分的位姿**，让导航真的能算出一帧速度指令 ——
    否则 `cmd` 恒为 0，就分不清"循环没跑"和"循环跑了但没下发"。
    """

    def __init__(self, x: float = 2850.0, y: float = 150.0,
                 theta: float = math.radians(-45.0)) -> None:
        self._port = "/dev/stub"
        self._baudrate = 115200
        self.is_open = True
        self._pose = [x, y, theta]
        self.velocities: List[Tuple[float, float]] = []
        self.stops = 0
        self.zero_vel = 0
        self.start_calls = 0

    # 生命周期
    def open(self) -> bool:
        return True

    def close(self) -> None:
        pass

    def start_match(self) -> bool:
        self.start_calls += 1
        return True

    def set_start_pose(self, x, y, theta) -> None:
        self._pose = [x, y, theta]

    def read_pose(self):
        return (self._pose[0], self._pose[1], self._pose[2])

    def set_pose(self, x, y, theta) -> None:
        self._pose = [x, y, theta]

    # 执行
    def send_velocity(self, v_mm_s: float = 0.0, w_rad_s: float = 0.0) -> bool:
        self.velocities.append((float(v_mm_s), float(w_rad_s)))
        if abs(v_mm_s) < 1e-9 and abs(w_rad_s) < 1e-9:
            self.zero_vel += 1
        # 按指令推进内部位姿，模拟"真的在走"
        dt = 0.02
        self._pose[2] += float(w_rad_s) * dt
        self._pose[0] += float(v_mm_s) * dt * math.cos(self._pose[2])
        self._pose[1] += float(v_mm_s) * dt * math.sin(self._pose[2])
        return True

    def send_stop(self) -> bool:
        self.stops += 1
        return True

    def send_ping(self, timeout: float = 0.5) -> bool:
        return True

    def send_servo(self, action: str) -> bool:
        return True

    def send_servo_angle(self, deg: float) -> bool:
        return True

    def send_command_await(self, *a, **k):
        return "ACK"

    def read_start_request(self):
        return None

    def drain_lines(self, *a, **k) -> int:
        return 0


# ────────────────────────────── 场景搭建 ──────────────────────────────

def _build(seed_targets: bool, chassis_pose=None, done: bool = False,
           time_up: bool = False, use_mock_nav: bool = True):
    """按真机接线方式构造一套 AutonomousState（真决策/导航/转运 + 桩感知/桩底盘）。"""
    from rescue_robot.perception.field_elements import FieldLayout, SafeZoneColor
    from rescue_robot.perception.world_map import WorldMap
    from rescue_robot.perception.target_types import (
        PRELIMINARY_TARGETS, TargetColor, TargetShape)
    from rescue_robot.decision.decision_engine import DecisionEngine, StrategyState
    from rescue_robot.navigation.navigation_pipeline import NavigationPipeline
    from rescue_robot.transport.transport_pipeline import TransportPipeline
    from rescue_robot.transport.sleeve_lift import MockSleeveLift
    from rescue_robot.states.autonomous_state import AutonomousState

    field = FieldLayout.standard()
    wm = WorldMap(field_layout=field)
    if seed_targets:
        regular = PRELIMINARY_TARGETS[(TargetColor.GREEN, TargetShape.CUBE)]
        wm.add_target(regular, (1500.0, 900.0))
        wm.add_target(regular, (2200.0, 1500.0))

    per = _PerceptionStub(wm)
    eng = DecisionEngine(wm, my_color=SafeZoneColor.RED)
    # ⚠️ 必须调 start_match()：真机上由 `AutonomousState.on_enter()`（第 228 行）调用。
    #    漏掉它 → `_match_start_time` 保持 0.0 → `_match_elapsed = now - 0` 巨大 →
    #    `time_remaining_s <= 0` **恒定成立** → 每一帧都在终场分支 return，正文一行不跑。
    #    本护栏第一版就踩了这个坑（造成 3 个假失败），与 decision_engine.py 第 1273 行
    #    注释里记的是同一个坑。
    eng.start_match()
    if time_up:
        # 造"比赛已进行很久"：`_match_elapsed` 会在 update() 里按
        # `now - _match_start_time` 重算，所以只能改 `_match_start_time`。
        eng._match_start_time = time.time() - 10 ** 6
    if done:
        eng._strategy_state = StrategyState.DONE
    nav = NavigationPipeline(field_layout=field, my_color=SafeZoneColor.RED,
                            use_mock=use_mock_nav)
    tp = TransportPipeline(field_layout=field, my_color=SafeZoneColor.RED,
                           use_mock=True, sleeve=MockSleeveLift())
    pos = chassis_pose or (2850.0, 150.0, math.radians(-45.0))
    ch = _ChassisStub(*pos)

    st = AutonomousState(
        _SmStub(), _IndStub(), perception=per, decision=eng, navigation=nav,
        transport=tp, chassis=ch, camera=None, field_layout=field,
        start_zone=4, my_color=SafeZoneColor.RED, use_mock=True)
    return st, eng, per, nav, ch


def _drive(st, frames: int, dt: float = 0.02, expect_no_raise: bool = True):
    """跑 frames 帧 `_run_once`；返回 (是否全部无异常, 首个异常文本/None)。"""
    for i in range(frames):
        try:
            st._run_once(dt)
        except Exception:
            return False, f"第 {i + 1} 帧抛异常：\n{traceback.format_exc()}"
    return True, None


# ────────────────────────────── 场景矩阵 ──────────────────────────────

def scenario_non_done_no_target(ev: List[str]) -> Optional[str]:
    """★ 事故现场工况：策略状态**非 DONE** + 场上暂无活跃目标。

    这正是 2026-09-17 崩溃的那一帧（日志末句 =
    `decision_engine: 场上暂无活跃目标 → 保持运行、原地等待重新检测`）。
    旧代码在这里第一帧就 UnboundLocalError。
    """
    st, eng, per, nav, ch = _build(seed_targets=False)
    okc, err = _drive(st, frames=60)
    if not okc:
        return ("主控循环在『非 DONE + 场上无目标』工况下崩溃 —— "
                "这就是真机上'倒车进场后一动不动'的直接原因\n" + err)
    if per.updates < 60:
        return f"感知未被逐帧调用（{per.updates}/60）→ 主循环没跑完整帧"
    ev.append(f"非DONE+无目标：60 帧无异常，感知调用 {per.updates} 次，"
              f"下发速度 {len(ch.velocities)} 次")
    return None


def scenario_non_done_with_target(ev: List[str]) -> Optional[str]:
    """非 DONE + 有目标：必须真的规划并下发速度（否则'循环跑了但车不走'）。"""
    st, eng, per, nav, ch = _build(seed_targets=True)
    okc, err = _drive(st, frames=120)
    if not okc:
        return "主控循环在『非 DONE + 有目标』工况下崩溃\n" + err
    moving = [v for v in ch.velocities if abs(v[0]) > 10.0 or abs(v[1]) > 0.01]
    if not moving:
        return ("120 帧里没有下发任何非零速度指令 → 车不会动。"
                "检查第 4 步 navigation.update 的入参是否传全（对面位置/减速带/dt）")
    ev.append(f"非DONE+有目标：120 帧无异常，非零速度指令 {len(moving)} 次"
              f"（示例 v={moving[0][0]:+.0f}mm/s w={moving[0][1]:+.2f}rad/s）")
    return None


def scenario_diag_branch(ev: List[str]) -> Optional[str]:
    """★ 精确锁定事故分支：2Hz 诊断日志块（`now` 的唯一无条件使用点）。

    前面两个场景靠"跑起来不炸"间接覆盖；这里再强制跨过 0.5s 诊断周期
    （把 `_diag_last_log` 设为很久以前），确认该分支**非 DONE 时**也能走到。
    """
    st, eng, per, nav, ch = _build(seed_targets=False)
    st._diag_last_log = 0.0        # 强制下一帧命中诊断分支
    logs: List[str] = []

    import logging

    class _Cap(logging.Handler):
        def emit(self, record):
            logs.append(record.getMessage())

    # 注意：必须挂到 **root** 并临时把级别降到 INFO —— 应用里的诊断日志走
    # `logger.info(...)`，而 Python 默认 root 级别是 WARNING，不降级的话这些行
    # 会被直接丢弃，护栏就会误报"诊断分支没被执行到"（第一版就踩了这个坑）。
    root = logging.getLogger()
    old_level = root.level
    h = _Cap()
    root.addHandler(h)
    root.setLevel(logging.INFO)
    try:
        okc, err = _drive(st, frames=3)
    finally:
        root.removeHandler(h)
        root.setLevel(old_level)
    if not okc:
        return ("2Hz 诊断日志分支在非 DONE 时崩溃（`now` 未定义）→ "
                "真机上主循环第一帧即死\n" + err)
    if not any("📊 pose=" in m for m in logs):
        return ("诊断分支没有被执行到（没看到 '📊 pose=' 日志）→ "
                "本护栏没真正覆盖到事故点，需同步更新")
    ev.append("诊断分支：非 DONE 下命中并打印 '📊 pose=' 日志，无异常")
    return None


def scenario_done_confirm_window(ev: List[str]) -> Optional[str]:
    """DONE 但时间未到：先观察 DONE_CONFIRM_S，期间不得终场停车。"""
    st, eng, per, nav, ch = _build(seed_targets=False, done=True, time_up=False)
    st._done_since = None
    okc, err = _drive(st, frames=5)
    if not okc:
        return "DONE（时间未到）工况崩溃\n" + err
    if st._stop_event.is_set():
        return ("DONE 尚在确认窗口内就终场停车了 → 误判 DONE 会直接断送整场"
                f"（DONE_CONFIRM_S={st.DONE_CONFIRM_S}s）")
    ev.append(f"DONE确认窗口：5 帧无异常、未提前终场（_done_since={st._done_since}）")

    # 把确认计时推到窗口之外 → 应执行终场
    st2, eng2, per2, nav2, ch2 = _build(seed_targets=False, done=True, time_up=False)
    st2._done_since = 0.0
    okc, err = _drive(st2, frames=3)
    if not okc:
        return "DONE 持续确认后终场路径崩溃\n" + err
    if not st2._stop_event.is_set():
        return "DONE 持续超过确认窗口仍未终场 → 比赛不会收口"
    ev.append(f"DONE持续确认后终场：stop_event 已置位，停车 {ch2.stops} 次")
    return None


def scenario_done_time_up(ev: List[str]) -> Optional[str]:
    """时间到：权威终场，必须停车并停止循环。"""
    st, eng, per, nav, ch = _build(seed_targets=False, done=True, time_up=True)
    okc, err = _drive(st, frames=2)
    if not okc:
        return "时间到终场路径崩溃\n" + err
    if not st._stop_event.is_set():
        return "比赛时间到却没有置停止事件 → 主循环不会退出"
    if ch.stops == 0:
        return "比赛时间到却没有任何停车指令 → 车会靠下位机看门狗才停（最多再冲 ~0.68m）"
    ev.append(f"时间到终场：stop_event 置位、停车 {ch.stops} 次、零速 {ch.zero_vel} 次")
    return None


def scenario_perception_raises(ev: List[str]) -> Optional[str]:
    """感知抛异常：一帧的视觉故障不得让主控循环死掉。"""
    st, eng, per, nav, ch = _build(seed_targets=False)
    per.raise_on_update = RuntimeError("模拟视觉偶发异常")
    okc, err = _drive(st, frames=5)
    if not okc:
        return "感知抛异常导致主控循环崩溃（视觉偶发故障 = 整车停摆）\n" + err
    ev.append("感知抛异常：主循环存活 5 帧（降级而非死亡）")
    return None


def scenario_transport_owns_nav(ev):
    """★ 转运 RETREAT 期间，决策层不许抢走导航目标。

    现场（2026-09-18）日志原样：
        套取失败 → 抬爪后退 120mm 到 (2618,423) 准备重试
        新导航目标: (2618, 423)          ← 转运设的后退点
        新导航目标: (2705, 262)          ← 决策层立刻改回"物资位置"
        pose=(2797,211) 目标=(2705,262) 转运=RETREAT
        pose=(2855,152) 目标=(2705,262) 转运=RETREAT
        pose=(2913,53)  目标=(2705,262) 转运=RETREAT   ← 一路冲到场地角
    两边互相覆盖 → "后退 → 重新接近"这套重试从未真正完成。
    """
    from rescue_robot.transport.transport_pipeline import TransportPhase

    st, eng, per, nav, ch = _build(seed_targets=True)
    st._transport._phase = TransportPhase.RETREAT
    retreat_pt = (2618.0, 423.0)
    nav.set_target(*retreat_pt)

    if not st._transport_owns_navigation():
        return "RETREAT 阶段没有被识别为『转运掌控导航』→ 闸门不会生效"
    okc, err = _drive(st, frames=5)
    if not okc:
        return "RETREAT 阶段主循环崩溃\n" + err
    t = nav.target
    if t is None or math.hypot(t[0] - retreat_pt[0], t[1] - retreat_pt[1]) > 1.0:
        return (f"RETREAT 期间导航目标被决策层改成了 {t}（应为后退点 {retreat_pt}）"
                f" → 车会朝物资冲，后退重试失效")
    ev.append(f"RETREAT 期间导航目标被保护在后退点 {retreat_pt}，决策层未抢走")
    return None


def scenario_nav_target_deadband(ev):
    """★ 导航目标死区：抖动幅度小于阈值时不重设（否则每帧强制全量重规划）。"""
    from rescue_robot.transport.transport_pipeline import TransportPhase

    st, eng, per, nav, ch = _build(seed_targets=False)
    st._transport._phase = TransportPhase.IDLE

    base = (1500.0, 1500.0)
    st._set_nav_target(base)
    if nav.target is None:
        return "基线目标都没设上"

    st._set_nav_target((1550.0, 1520.0))          # 移动 58mm < 死区
    t = nav.target
    if math.hypot(t[0] - base[0], t[1] - base[1]) > 1.0:
        return f"58mm 的抖动就把目标重设成了 {t} → 死区没生效，仍会每帧重规划"

    far = (1500.0 + st.NAV_TARGET_DEADBAND_MM + 150.0, 1500.0)
    st._set_nav_target(far)                       # 明显移动 > 死区
    t = nav.target
    if math.hypot(t[0] - far[0], t[1] - far[1]) > 1.0:
        return f"超过死区的真实移动没有更新目标（仍是 {t}）→ 去抖过度，会跟不上目标"
    ev.append(f"死区 {st.NAV_TARGET_DEADBAND_MM:.0f}mm：58mm 抖动被忽略、"
              f"250mm 真实移动正常更新")
    return None


SCENARIOS = (
    ("非DONE+无目标(事故工况)", scenario_non_done_no_target),
    ("非DONE+有目标", scenario_non_done_with_target),
    ("2Hz诊断分支(事故点)", scenario_diag_branch),
    ("DONE确认窗口/终场", scenario_done_confirm_window),
    ("时间到权威终场", scenario_done_time_up),
    ("感知抛异常", scenario_perception_raises),
    ("转运RETREAT期间导航目标不被抢", scenario_transport_owns_nav),
    ("导航目标死区去抖", scenario_nav_target_deadband),
)


def main() -> int:
    print("=" * 78)
    print("  verify_run_once_loop —— 真机主控循环 AutonomousState._run_once 端到端护栏")
    print("=" * 78)
    ev: List[str] = []
    failures: List[Tuple[str, str]] = []

    for name, fn in SCENARIOS:
        try:
            problem = fn(ev)
        except Exception:
            problem = "护栏自身抛异常（先修护栏，它现在是坏的）：\n" + traceback.format_exc()
        status = PASS if problem is None else FAIL
        print(f"  [{status}] {name}")
        if problem is not None:
            failures.append((name, problem))
            for line in problem.splitlines():
                print(f"         {line}")

    print("-" * 78)
    for line in ev:
        print(f"  · {line}")
    print("-" * 78)
    if failures:
        print(f"  结果: {len(SCENARIOS) - len(failures)}/{len(SCENARIOS)} 通过，"
              f"{len(failures)} 个场景失败")
        print("  ❌ 主控循环存在会让整车停摆的缺陷，禁止上场")
        return 1
    print(f"  结果: {len(SCENARIOS)}/{len(SCENARIOS)} 通过")
    print("  ✅ 主循环分支矩阵全部真跑通过（含 2026-09-17 事故工况）")
    return 0


if __name__ == "__main__":
    sys.exit(main())
