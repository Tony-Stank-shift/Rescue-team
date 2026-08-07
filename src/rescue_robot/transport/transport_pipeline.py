"""
transport_pipeline.py —— 转运主控管线

协调夹爪、装载管理和安全区投放。

转运流程（单趟）：
  导航到目标 → 打开夹爪 → 夹取 → 闭合 →
  导航到安全区 → 投放 → 释放

对接 autonomous_state 主循环。
"""

import logging
import time
from dataclasses import dataclass, field
from enum import Enum, auto
from typing import List, Optional, Set, Tuple

from .gripper import AbstractGripper, MockGripper, GripperAction
from .load_manager import LoadManager, Violation
from .safe_zone_placer import SafeZonePlacer, PlacementResult, PlacementZone
from ..perception.target_types import (
    TargetType, TargetInfo, get_point_value, CompetitionPhase,
)
from ..perception.world_map import TrackedTarget
from ..perception.field_elements import FieldLayout, SafeZoneColor

logger = logging.getLogger("transport_pipeline")


# ============================================================
# 转运阶段
# ============================================================

class TransportPhase(Enum):
    """转运阶段"""
    IDLE = auto()               # 空闲
    APPROACHING = auto()        # 接近目标中
    GRIPPING = auto()           # 夹取中
    TRANSPORTING = auto()       # 运送至安全区
    PLACING = auto()            # 投放中
    COMPLETE = auto()           # 完成
    VIOLATION = auto()          # 违规


@dataclass
class TransportStatus:
    """转运状态"""
    phase: TransportPhase = TransportPhase.IDLE
    trip_number: int = 0
    gripper_action: GripperAction = GripperAction.OPEN
    load_count: int = 0
    load_max: int = 3
    target_ids: Set[int] = field(default_factory=set)
    distance_to_target_mm: float = 0.0
    violation: Violation = Violation.NONE


# ============================================================
# 转运管线
# ============================================================

class TransportPipeline:
    """
    转运主控管线。

    对接 autonomous loop 的典型使用：

      # 在自主循环中
      if transport.is_idle():
          target = select_best_target(world_map)
          transport.start_trip(target)

      status = transport.update(robot_pose, world_map, nav)

      if transport.is_complete():
          # 一趟完成，选择下一个目标
  """

    def __init__(self,
                 gripper: Optional[AbstractGripper] = None,
                 field_layout: Optional[FieldLayout] = None,
                 my_color: SafeZoneColor = SafeZoneColor.RED,
                 use_mock: bool = True):
        self._gripper = gripper or MockGripper()
        self._load_mgr = LoadManager()
        self._placer = SafeZonePlacer(
            field_layout or FieldLayout.standard(), my_color
        ) if field_layout or True else None

        # 当前趟次状态
        self._phase = TransportPhase.IDLE
        self._current_targets: List[TrackedTarget] = []
        self._planning_trip = False

        # 统计
        self._total_trips = 0
        self._total_targets_delivered = 0
        self._total_score = 0

        logger.info("TransportPipeline 初始化")

    # ---- 属性 ----

    @property
    def phase(self) -> TransportPhase:
        return self._phase

    @property
    def load_manager(self) -> LoadManager:
        return self._load_mgr

    @property
    def placer(self) -> SafeZonePlacer:
        return self._placer

    @property
    def is_idle(self) -> bool:
        return self._phase in (TransportPhase.IDLE, TransportPhase.COMPLETE)

    # ---- 转运控制 ----

    def start_trip(self, targets: List[TrackedTarget]) -> Tuple[bool, Violation]:
        """
        开始一趟转运。

        Args:
            targets: 要转运的目标列表

        Returns:
            (ok, violation)
        """
        if not self.is_idle:
            logger.warning(f"无法开始转运：当前阶段={self._phase.name}")
            return (False, Violation.NONE)

        # 规则校验
        target_infos = [t.info for t in targets]
        ok, violation = self._load_mgr.can_load_batch(target_infos)
        if not ok:
            consequence, is_fatal = self._load_mgr.get_violation_info(violation)
            logger.error(f"转运违规: {violation.name} — {consequence}")
            self._phase = TransportPhase.VIOLATION
            return (False, violation)

        self._current_targets = targets
        self._phase = TransportPhase.APPROACHING
        self._planning_trip = True

        names = [t.info.description for t in targets]
        logger.info(f"开始转运 (第{self._load_mgr.trip_number}趟): "
                     f"{len(targets)} 个目标 — {names}")

        return (True, Violation.NONE)

    def update(self,
               robot_pose: Tuple[float, float, float],
               world_map=None,
               nav=None) -> TransportStatus:
        """
        单帧转运更新。

        Returns:
            TransportStatus: 当前转运状态
        """
        rx, ry, rtheta = robot_pose

        if self._phase == TransportPhase.APPROACHING:
            # 接近目标
            if self._current_targets and nav:
                target = self._current_targets[0]
                dist = self._distance(rx, ry, target.position)
                if dist < 150:  # 到达夹取范围
                    self._phase = TransportPhase.GRIPPING
                    logger.debug(f"到达目标附近: dist={dist:.0f}mm")
                # 否则导航继续（由 autonomous loop 调用 nav 完成）

        elif self._phase == TransportPhase.GRIPPING:
            # 夹取
            if not self._gripper.is_holding():
                # 打开夹爪
                self._gripper.open()
                # 构建目标位置
                positions = {t.id: t.position for t in self._current_targets}
                # 闭合夹爪
                success = self._gripper.close(positions)
                if success:
                    for t in self._current_targets:
                        ok, v = self._load_mgr.load(t.info, t.id)
                        if not ok:
                            self._phase = TransportPhase.VIOLATION
                            return self._get_status()
                    self._phase = TransportPhase.TRANSPORTING
                    logger.info("夹取完成，开始运送")
            # 夹爪已闭合时继续

        elif self._phase == TransportPhase.TRANSPORTING:
            # 运送至安全区
            safe_zone = self._placer.get_my_safe_zone_region()
            if safe_zone:
                dist_to_safe = self._distance_to_region(rx, ry, safe_zone)
                if dist_to_safe < 100:  # 到达安全区附近
                    self._phase = TransportPhase.PLACING
                    logger.debug(f"到达安全区附近: dist={dist_to_safe:.0f}mm")

        elif self._phase == TransportPhase.PLACING:
            # 投放
            positions = [(rx, ry)] * len(self._current_targets)
            infos = [t.info for t in self._current_targets]

            results = self._placer.classify_batch(positions, infos)
            all_valid = all(r.is_valid for r in results)

            # 释放夹爪
            released = self._gripper.release()

            if released:
                # 投放完成
                self._load_mgr.release_all(placement_ok=all_valid)
                self._total_trips += 1
                self._total_targets_delivered += len(self._current_targets)
                self._total_score = self._load_mgr.total_score

                if not all_valid:
                    bad = [r for r in results if not r.is_valid]
                    for r in bad:
                        logger.warning(f"投放位置错误: {r.detail}")

                self._phase = TransportPhase.COMPLETE
                self._current_targets.clear()

                logger.info(f"转运完成: 得分={self._total_score}, "
                             f"累计={self._total_targets_delivered}个")

        return self._get_status()

    # ---- 查询 ----

    def is_complete(self) -> bool:
        return self._phase == TransportPhase.COMPLETE

    def is_violation(self) -> bool:
        return self._phase == TransportPhase.VIOLATION

    def is_idle(self) -> bool:
        return self._phase in (TransportPhase.IDLE, TransportPhase.COMPLETE)

    def _get_status(self) -> TransportStatus:
        state = self._load_mgr.state
        return TransportStatus(
            phase=self._phase,
            trip_number=state.trip_number,
            gripper_action=self._gripper.state.action,
            load_count=state.count,
            target_ids=state.target_ids,
            violation=None,  # not tracked at this level
        )

    def reset(self) -> None:
        self._phase = TransportPhase.IDLE
        self._current_targets.clear()
        self._load_mgr.reset()
        self._gripper.release()

    def summary(self) -> str:
        return (
            f"转运管线: phase={self._phase.name}, "
            f"{self._load_mgr.summary()}"
        )

    # ---- 工具 ----

    @staticmethod
    def _distance(p1: Tuple[float, float],
                  p2: Tuple[float, float]) -> float:
        import math
        return math.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)

    @staticmethod
    def _distance_to_region(x: float, y: float,
                            region) -> float:
        import math
        dx = max(region.x - x, 0, x - region.x - region.width)
        dy = max(region.y - y, 0, y - region.y - region.height)
        return math.sqrt(dx * dx + dy * dy)


# ============================================================
# 独立测试
# ============================================================

if __name__ == "__main__":
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s [%(levelname)-7s] %(name)s: %(message)s",
        datefmt="%H:%M:%S",
    )

    print("=" * 50)
    print("  转运控制 — Mock 模式测试")
    print("=" * 50)

    from ..perception.target_types import (
        PRELIMINARY_TARGETS, TargetColor, TargetShape,
    )
    from ..perception.world_map import TrackedTarget, TargetStatus

    # 获取目标配置
    regular_info = PRELIMINARY_TARGETS[(TargetColor.GREEN, TargetShape.CUBE)]
    core_info = PRELIMINARY_TARGETS[(TargetColor.BLACK, TargetShape.TRIANGULAR_PYRAMID)]
    injured_info = PRELIMINARY_TARGETS[(TargetColor.ORANGE, TargetShape.CUBOID)]
    dangerous_info = PRELIMINARY_TARGETS[(TargetColor.LIGHT_BLUE, TargetShape.CUBE)]

    field = FieldLayout.standard()
    tp = TransportPipeline(field_layout=field)

    # --- 测试 1：首次转运 = 1 个普通物资 ✓ ---
    print("\n测试 1: 首次转运（1普通物资）")
    target = TrackedTarget(id=1, info=regular_info, position=(500, 500))
    ok, v = tp.start_trip([target])
    print(f"  start_trip: ok={ok}, violation={v}")
    assert ok, "首次转运 1 普通物资应该成功"

    # 模拟转运流程
    tp.update((500, 500, 0))  # APPROACHING → GRIPPING
    tp.update((500, 500, 0))  # GRIPPING → grip
    tp.update((500, 500, 0))  # TRANSPORTING
    tp.update((500, 500, 0))  # 仍在 TRANSPORTING
    # 模拟到达安全区
    tp._phase = TransportPhase.PLACING
    tp.update((200, 2800, 0))  # PLACING → release
    print(f"  phase={tp.phase.name}")
    assert tp.is_complete(), "转运应该完成"
    print("  ✅ 通过")

    # --- 测试 2：首次转运 >1 个 → 违规 ---
    print("\n测试 2: 首次转运 2 个 → 违规")
    tp.reset()
    t1 = TrackedTarget(id=2, info=regular_info, position=(500, 500))
    t2 = TrackedTarget(id=3, info=regular_info, position=(600, 500))
    ok, v = tp.start_trip([t1, t2])
    print(f"  start_trip: ok={ok}, violation={v}")
    assert not ok and v == Violation.FIRST_TRIP_MULTI, \
        f"首次转运 2 个应违规，得到 {v}"
    print("  ✅ 通过")

    # --- 测试 3：危险目标 → 违规 ---
    print("\n测试 3: 危险目标 → 违规")
    tp.reset()
    # 先完成一次转运
    t = TrackedTarget(id=5, info=regular_info, position=(500, 500))
    tp.start_trip([t])
    tp._gripper.close({5: (500, 500)})
    tp._load_mgr.load(regular_info, 5)
    tp._load_mgr.release_all()
    tp._phase = TransportPhase.IDLE
    # 现在尝试装载危险目标
    danger = TrackedTarget(id=6, info=dangerous_info, position=(700, 500))
    ok, v = tp.start_trip([danger])
    print(f"  start_trip: ok={ok}, violation={v}")
    assert not ok and v == Violation.DANGEROUS_TARGET, \
        f"危险目标应违规，得到 {v}"
    print("  ✅ 通过")

    # --- 测试 4：伤员必须单独 ---
    print("\n测试 4: 装载 2 个伤员 → 违规")
    tp.reset()
    tp._load_mgr._total_trips = 1  # 绕过首次检查
    i1 = TrackedTarget(id=7, info=injured_info, position=(500, 500))
    i2 = TrackedTarget(id=8, info=injured_info, position=(600, 500))
    ok, v = tp.start_trip([i1, i2])
    print(f"  start_trip: ok={ok}, violation={v}")
    assert not ok and v == Violation.INJURED_MULTI, \
        f"2 个伤员应违规，得到 {v}"
    print("  ✅ 通过")

    # --- 测试 5：超 3 个 → 违规 ---
    print("\n测试 5: 装载 4 个 → 违规")
    tp.reset()
    tp._load_mgr._total_trips = 1
    targets_4 = [
        TrackedTarget(id=10+i, info=regular_info, position=(500+i*50, 500))
        for i in range(4)
    ]
    ok, v = tp.start_trip(targets_4)
    print(f"  start_trip: ok={ok}, violation={v}")
    assert not ok and v == Violation.OVER_LIMIT, \
        f"4 个应违规，得到 {v}"
    print("  ✅ 通过")

    # --- 测试 6：安全区投放判定 ---
    print("\n测试 6: 投放位置判定")
    placer = SafeZonePlacer(field)
    # 物资放入物资区 → 有效
    r = placer.classify((150, 2850), regular_info)
    print(f"  物资@物资区: valid={r.is_valid}, {r.detail}")
    # 物资放入伤员区 → 无效
    r = placer.classify((500, 2850), regular_info)
    print(f"  物资@伤员区: valid={r.is_valid}, penalty={r.penalty}, {r.detail}")
    # 目标在安全区外
    r = placer.classify((1500, 1500), regular_info)
    print(f"  场地中央: valid={r.is_valid}, {r.detail}")
    print("  ✅ 通过")

    print(f"\n{'='*50}")
    print("  转运控制测试全部通过 ✅")
    print(f"{'='*50}")
