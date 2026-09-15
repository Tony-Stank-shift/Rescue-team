"""
verify_s40_s01_team.py —— 队长本轮修复的回归护栏（S-40 / S-40第二半 / S-01 / S-02 / B8）

用法（仓库根目录）：
    PYTHONPATH=src python3 tools/fix_verifiers/verify_s40_s01_team.py

背景：这几条是本项目最容易"改回去还看不出来"的缺陷——它们都不会报错，
只会让**分数悄悄变高**（把没做到的事记成做到了）或让**整场悄悄卡住**。
因此每条都钉一个可复现的断言。

覆盖：
  1. S-40      车只到第 1 个目标时，本趟计划里的其它目标**不得**被记入装载
  2. S-40      套取前必须做"车确实在该目标处"的位姿复核（隔着距离不许记入）
  3. S-40 二半 决策引擎只给**真正送达**的目标标记"已入安全区"
  4. S-40      已持有目标时后退重试**不得抬爪**（抬爪=释放，会把货丢掉）
  5. S-01      决策引擎进入 DONE 后，主循环必须清导航目标 + 停车 + 退出
  6. S-02      忙碌时 start_trip 必须被拒
  7. B8        无关目标靠近场心**不得**误判为"投放无效"（否则运送途中被抢走导航目标）
  8. B8        已送达的目标回到场心**必须**被识别为"投放无效"
  9. 配置      sleeve_max_hold 填 >1 必须被强制回退到 1（防现场调到更差的档位）
"""
import logging
import sys
import types

logging.disable(logging.CRITICAL)

from rescue_robot.decision.decision_engine import DecisionEngine, StrategyState  # noqa: E402
from rescue_robot.perception.field_elements import FieldLayout, SafeZoneColor  # noqa: E402
from rescue_robot.perception.target_types import DetectedTarget              # noqa: E402
from rescue_robot.perception.world_map import WorldMap                      # noqa: E402
from rescue_robot.transport.transport_pipeline import (                     # noqa: E402
    TransportPhase, TransportPipeline,
)

PASS = 0
FAIL = 0


def check(name, cond, detail=""):
    global PASS, FAIL
    if cond:
        PASS += 1
        print(f"  [ok]   {name}")
    else:
        FAIL += 1
        print(f"  [FAIL] {name}  {detail}")


class _NavStub:
    """最小导航桩：只记录目标，不做规划。"""

    def __init__(self):
        self.target = None
        self._arrived = True

    def set_target(self, x, y):
        self.target = (x, y)

    def clear_target(self):
        self.target = None

    def is_arrived(self):
        return self._arrived


def _mk_target(wm, tid, pos, kind="GREEN"):
    """按颜色取初赛目标表里的 TargetInfo（与 decision/transport 用的是同一张表）。"""
    from rescue_robot.perception.target_types import PRELIMINARY_TARGETS as P
    from rescue_robot.perception.target_types import TargetColor, TargetShape
    col = getattr(TargetColor, kind)
    info = next(v for (c, _s), v in P.items() if c == col)
    return wm._create_new_target(DetectedTarget(id=tid, info=info, position=pos), 0.0)


def _tp(wm):
    tp = TransportPipeline(field_layout=FieldLayout.standard(),
                           my_color=SafeZoneColor.RED, use_mock=True)
    # 首趟必须且仅 1 个普通物资：显式标记"首趟已有效完成"，
    # 以便验证自由装载规则（判据是 _first_trip_done，不是 _total_trips）
    tp._load_mgr.mark_first_trip_done()
    return tp


# ---------------------------------------------------------------- 1/2/4
def test_s40_no_phantom_load():
    print("[1] S-40 车没去过的目标不得被记入装载")
    field = FieldLayout.standard()
    wm = WorldMap(field_layout=field)
    ids = [_mk_target(wm, 1000 + i, p) for i, p in enumerate(
        [(2400.0, 2500.0), (400.0, 400.0), (2600.0, 400.0)])]
    tp = _tp(wm)
    targets = [wm.targets[i] for i in ids]
    ok, _ = tp.start_trip(targets)
    check("start_trip 接受本趟计划(≤3)", ok)

    # 车只开到第 1 个目标（第 1 帧：APPROACHING→CAPTURING 并显式停车；
    # 第 2 帧：真正下压套取 —— 套取动作本身要跨帧完成）
    tp.update((2400.0, 2500.0, 0.0), wm, _NavStub())
    tp.update((2400.0, 2500.0, 0.0), wm, _NavStub())
    st = tp.load_manager.state
    check("只到第 1 个目标 → 只记入 1 个（旧实现=3）", st.count == 1,
          f"实际 count={st.count} ids={st.target_ids}")
    check("阶段已进入运送", tp.phase == TransportPhase.TRANSPORTING,
          f"phase={tp.phase.name}")
    check("未去过的 2 个目标不在货舱", set(st.target_ids) == {ids[0]},
          f"ids={st.target_ids}")

    # 隔着距离不许记入：把车挪到远处再走一帧 CAPTURING
    print("[2] S-40 套取前必须有'车确实在该目标处'的位姿复核")
    tp2 = _tp(wm)
    tp2.start_trip([wm.targets[ids[1]]])
    tp2._phase = TransportPhase.CAPTURING          # 直接进套取
    tp2.update((2400.0, 2500.0, 0.0), wm, _NavStub())   # 车离目标 2000+mm
    check("车不在目标处 → 不记入装载（打回重接近）",
          tp2.load_manager.state.count == 0,
          f"count={tp2.load_manager.state.count} phase={tp2.phase.name}")
    check("且被退回接近阶段", tp2.phase == TransportPhase.APPROACHING,
          f"phase={tp2.phase.name}")

    print("[4] S-40 已持有目标时后退重试不得抬爪（抬爪=释放）")
    tp3 = _tp(wm)
    tp3.start_trip([wm.targets[ids[0]]])
    tp3.update((2400.0, 2500.0, 0.0), wm, _NavStub())    # → CAPTURING
    tp3.update((2400.0, 2500.0, 0.0), wm, _NavStub())    # 套住 1 个
    check("已持有 1 个", len(tp3._captured) == 1,
          f"captured={len(tp3._captured)} phase={tp3.phase.name}")
    raised = {"n": 0}
    orig = tp3._sleeve.raise_up

    def spy():
        raised["n"] += 1
        return orig()

    tp3._sleeve.raise_up = spy
    tp3._begin_retreat(2400.0, 2500.0, 0.0, _NavStub())
    check("持有目标时 _begin_retreat 不调用抬爪", raised["n"] == 0,
          f"raise_up 被调用 {raised['n']} 次")


# ---------------------------------------------------------------- 3/7/8
def test_s40_second_half_and_b8():
    print("[3] S-40 第二半 决策引擎只给真正送达的目标标记入安全区")
    field = FieldLayout.standard()
    wm = WorldMap(field_layout=field)
    ids = [_mk_target(wm, 2000 + i, p) for i, p in enumerate(
        [(500.0, 500.0), (600.0, 500.0)])]
    eng = DecisionEngine(wm, my_color=SafeZoneColor.RED)
    eng._trip_targets = [wm.targets[i] for i in ids]
    eng._current_target = wm.targets[ids[0]]
    eng._strategy_state = StrategyState.FREE_RUN
    before = eng._targets_delivered
    eng._handle_free_run(500.0, 500.0, nav_arrived=True, grip_done=True,
                         release_done=True, release_valid=True,
                         delivered_ids=[ids[0]])
    from rescue_robot.perception.target_types import TargetStatus
    check("真正送达的那个 → 标记已入安全区",
          wm.targets[ids[0]].status == TargetStatus.IN_SAFE_ZONE)
    check("本趟计划里没套上的那个 → 保持 ACTIVE（留待下一趟重选）",
          wm.targets[ids[1]].status == TargetStatus.ACTIVE,
          f"status={wm.targets[ids[1]].status}")
    check("送达计数只 +1（旧实现 +2）", eng._targets_delivered == before + 1,
          f"delivered={eng._targets_delivered}")

    print("[7] B8 无关目标靠近场心不得误判为'投放无效'")
    wm2 = WorldMap(field_layout=field)
    near_center = _mk_target(wm2, 3000, (1520.0, 1490.0))   # 场心附近的无关目标
    eng2 = DecisionEngine(wm2, my_color=SafeZoneColor.RED)
    check("没有已送达目标时 → 不判无效", eng2._check_invalid_transport(300, 300) is False,
          "旧实现会因场心附近任意目标返回 True → 运送途中被抢走导航目标")
    check("无关目标在场心附近 → 仍不判无效",
          eng2._check_invalid_transport(1520, 1490) is False)

    print("[8] B8 已送达的目标回到场心 → 必须识别为投放无效")
    eng2._delivered_ids.add(near_center)
    check("我们送达过的目标回到场心 → 判为投放无效",
          eng2._check_invalid_transport(300, 300) is True)

    print("[5] S-01 终场必须清导航目标 + 停车 + 退出主循环")
    from rescue_robot.states.autonomous_state import AutonomousState
    import inspect
    src = inspect.getsource(AutonomousState._run_once)
    check("_run_once 里有终场分支（DONE → 停车）",
          "StrategyState.DONE" in src and "_stop_chassis" in src and
          "_stop_event.set()" in src)
    check("终场同时清导航目标", "clear_target" in src)


# ---------------------------------------------------------------- 6/9
def test_s02_and_config():
    print("[6] S-02 忙碌时 start_trip 必须被拒")
    field = FieldLayout.standard()
    wm = WorldMap(field_layout=field)
    tid = _mk_target(wm, 4000, (500.0, 500.0))
    tp = _tp(wm)
    tp._phase = TransportPhase.CAPTURING
    ok, _ = tp.start_trip([wm.targets[tid]])
    check("CAPTURING 中再 start_trip → 拒绝", ok is False,
          "旧实现漏了括号，方法对象恒为真 → 守卫失效")

    print("[9] sleeve_max_hold 填 >1 必须被强制回退 1")
    from rescue_robot import config as C
    from rescue_robot.innovation.config_loader import RobotConfig
    cfg = RobotConfig.from_yaml("config/robot.default.yaml")
    cfg.placement.sleeve_max_hold = 3
    C.apply_robot_config(cfg)
    check("请求 3 → 实际生效 1（填 >1 实测更差：4 个 vs 7~8 个）",
          C.Placement.SLEEVE_MAX_HOLD == 1,
          f"实际={C.Placement.SLEEVE_MAX_HOLD}")


def main():
    test_s40_no_phantom_load()
    test_s40_second_half_and_b8()
    test_s02_and_config()
    print("-" * 66)
    print(f"  S-40 / S-01 / S-02 / B8 回归护栏: {PASS} 通过, {FAIL} 失败")
    return 1 if FAIL else 0


if __name__ == "__main__":
    sys.exit(main())
