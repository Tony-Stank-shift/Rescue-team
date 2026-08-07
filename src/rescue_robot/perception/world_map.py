"""
world_map.py —— 动态世界地图

全局环境模型，是感知模块的核心输出，也是决策模块的唯一输入。

维护内容：
  - 已跟踪目标列表（含位置、速度、状态）
  - 场地布局
  - 对方机器人状态

动态特性：
  - 裁判可能将无效目标取出重新随机放置
  - 目标可能被对方推入我方安全区
  - 目标可能在碰撞中移动
"""

import logging
import math
import time
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Set, Tuple

from .target_types import (
    DetectedTarget, TargetInfo, TargetType, TargetStatus,
    get_point_value,
)
from .field_elements import FieldLayout

logger = logging.getLogger("world_map")


# ============================================================
# 跟踪目标
# ============================================================

@dataclass
class TrackedTarget:
    """世界地图中跟踪的单个目标"""
    id: int                              # 唯一 ID
    info: TargetInfo                     # 目标信息
    position: Tuple[float, float]        # 场地坐标 (x, y) mm
    velocity: Tuple[float, float] = (0.0, 0.0)  # (vx, vy) mm/s
    status: TargetStatus = TargetStatus.ACTIVE
    confidence: float = 1.0
    first_seen: float = 0.0              # 首次发现时间
    last_seen: float = 0.0               # 最近一次被检测到的时间
    seen_count: int = 0                  # 被检测到的帧数
    track_lost_count: int = 0            # 连续丢失的帧数

    @property
    def age_s(self) -> float:
        """自首次发现以来的时间（秒）"""
        return time.time() - self.first_seen if self.first_seen > 0 else 0.0

    @property
    def is_stale(self) -> bool:
        """是否已过时（超过 2 秒未被检测到）"""
        if self.last_seen == 0:
            return False
        return (time.time() - self.last_seen) > 2.0

    @property
    def distance_to_origin(self) -> float:
        """距离场地原点的距离"""
        return math.sqrt(self.position[0] ** 2 + self.position[1] ** 2)


# ============================================================
# 世界地图
# ============================================================

class WorldMap:
    """
    动态世界地图。

    每帧更新流程：
    1. 接收新检测结果
    2. 数据关联（匈牙利匹配 / 最近邻）
    3. 更新已跟踪目标
    4. 移除过时目标
    5. 检测新出现的目标
    """

    # 关联阈值：两个检测被视为同一目标的最大距离（mm）
    ASSOCIATION_DISTANCE_MM = 100.0

    # 目标被认为稳定跟踪的最小检测次数
    MIN_SEEN_COUNT = 3

    # 目标被移除前的最大丢失帧数
    MAX_LOST_COUNT = 30  # ~0.6 秒（50Hz）

    def __init__(self, field_layout: Optional[FieldLayout] = None):
        self._targets: Dict[int, TrackedTarget] = {}
        self._field = field_layout
        self._next_id = 1000  # 全局目标 ID（与 DetectedTarget.id 区分）

        # 统计
        self._update_count = 0
        self._total_detections = 0

    # ---- 属性 ----

    @property
    def targets(self) -> Dict[int, TrackedTarget]:
        """所有跟踪中的目标"""
        return self._targets

    @property
    def field(self) -> Optional[FieldLayout]:
        return self._field

    @field.setter
    def field(self, layout: FieldLayout) -> None:
        self._field = layout

    @property
    def active_targets(self) -> List[TrackedTarget]:
        """场上活跃的目标"""
        return [t for t in self._targets.values()
                if t.status == TargetStatus.ACTIVE]

    # ---- 目标查询 ----

    def get_targets_by_type(self, target_type: TargetType) -> List[TrackedTarget]:
        """按类型查询目标"""
        return [t for t in self._targets.values()
                if t.info.type == target_type and t.status == TargetStatus.ACTIVE]

    def get_regular_supplies(self) -> List[TrackedTarget]:
        return self.get_targets_by_type(TargetType.REGULAR_SUPPLY)

    def get_core_supplies(self) -> List[TrackedTarget]:
        return self.get_targets_by_type(TargetType.CORE_SUPPLY)

    def get_injured(self) -> List[TrackedTarget]:
        return self.get_targets_by_type(TargetType.INJURED)

    def get_dangerous(self) -> List[TrackedTarget]:
        return self.get_targets_by_type(TargetType.DANGEROUS)

    def get_nearest_target(self, position: Tuple[float, float],
                           types: Optional[List[TargetType]] = None,
                           exclude_ids: Optional[Set[int]] = None) -> Optional[TrackedTarget]:
        """获取最近的目标（可选按类型过滤）"""
        candidates = self.active_targets

        if types:
            candidates = [t for t in candidates if t.info.type in types]
        if exclude_ids:
            candidates = [t for t in candidates if t.id not in exclude_ids]
        if not candidates:
            return None

        px, py = position
        return min(candidates,
                   key=lambda t: (t.position[0] - px) ** 2 + (t.position[1] - py) ** 2)

    def get_targets_sorted_by_value(self) -> List[TrackedTarget]:
        """按分值排序（伤员 15 > 核心 10 > 普通 5），高分在前"""
        return sorted(
            self.active_targets,
            key=lambda t: (-t.info.points, t.distance_to_origin),
        )

    def get_targets_sorted_by_distance(self,
                                        position: Tuple[float, float]) -> List[TrackedTarget]:
        """按距离排序，近的在前"""
        px, py = position
        return sorted(
            self.active_targets,
            key=lambda t: (t.position[0] - px) ** 2 + (t.position[1] - py) ** 2,
        )

    def count_by_type(self) -> Dict[TargetType, int]:
        """统计各类型目标数量"""
        counts = {t: 0 for t in TargetType}
        for target in self._targets.values():
            if target.status == TargetStatus.ACTIVE:
                counts[target.info.type] += 1
        return counts

    def count_in_safe_zone(self) -> Dict[TargetType, int]:
        """统计已转运到安全区的目标"""
        counts = {t: 0 for t in TargetType}
        for target in self._targets.values():
            if target.status == TargetStatus.IN_SAFE_ZONE:
                counts[target.info.type] += 1
        return counts

    def total_score(self) -> int:
        """计算当前已得分"""
        score = 0
        in_safe = self.count_in_safe_zone()
        for ttype, count in in_safe.items():
            score += count * get_point_value(ttype)
        return score

    # ---- 更新循环 ----

    def update(self, detected_targets: List[DetectedTarget],
               robot_position: Tuple[float, float] = (0, 0),
               timestamp: float = 0.0) -> None:
        """
        主更新方法：融合新检测结果。

        Args:
            detected_targets: 已分类的检测目标
            robot_position: 机器人当前位置
            timestamp: 当前时间戳
        """
        if timestamp == 0.0:
            timestamp = time.time()

        self._update_count += 1
        self._total_detections += len(detected_targets)

        # 步骤 1：所有现有目标丢失计数 +1
        for target in self._targets.values():
            target.track_lost_count += 1

        # 步骤 2：数据关联（最近邻）
        matched_existing: Set[int] = set()
        matched_new: Set[int] = set()

        for new_idx, det in enumerate(detected_targets):
            best_match = None
            best_distance = self.ASSOCIATION_DISTANCE_MM

            for track_id, tracked in self._targets.items():
                if track_id in matched_existing:
                    continue
                if tracked.info.type != det.info.type:
                    continue  # 类型不同不匹配

                dist = self._distance(tracked.position, det.position)
                if dist < best_distance:
                    best_distance = dist
                    best_match = track_id

            if best_match is not None:
                # 更新已有目标
                self._update_tracked(best_match, det, timestamp)
                matched_existing.add(best_match)
                matched_new.add(new_idx)

        # 步骤 3：创建新目标（未匹配的新检测 + 达到最小检测次数）
        for new_idx, det in enumerate(detected_targets):
            if new_idx in matched_new:
                continue
            # 新检测，直接创建
            self._create_new_target(det, timestamp)

        # 步骤 4：移除过时目标
        stale_ids = [
            tid for tid, t in self._targets.items()
            if t.track_lost_count > self.MAX_LOST_COUNT
        ]
        for tid in stale_ids:
            target = self._targets.pop(tid)
            logger.debug(f"移除过时目标: ID={tid}, {target.info.description}")

    def _update_tracked(self, track_id: int, det: DetectedTarget,
                        timestamp: float) -> None:
        """更新已跟踪目标的位置和状态"""
        target = self._targets[track_id]

        # 计算速度
        dt = timestamp - target.last_seen if target.last_seen > 0 else 0.05
        if dt > 0:
            vx = (det.position[0] - target.position[0]) / dt
            vy = (det.position[1] - target.position[1]) / dt
            # 指数平滑
            alpha = 0.3
            target.velocity = (
                alpha * vx + (1 - alpha) * target.velocity[0],
                alpha * vy + (1 - alpha) * target.velocity[1],
            )

        target.position = det.position
        target.confidence = det.confidence
        target.last_seen = timestamp
        target.seen_count += 1
        target.track_lost_count = 0

    def _create_new_target(self, det: DetectedTarget, timestamp: float) -> int:
        """创建新的跟踪目标"""
        target_id = self._next_id
        self._next_id += 1

        self._targets[target_id] = TrackedTarget(
            id=target_id,
            info=det.info,
            position=det.position,
            confidence=det.confidence,
            first_seen=timestamp,
            last_seen=timestamp,
            seen_count=1,
        )
        logger.debug(f"新目标: ID={target_id}, {det.info.description}, "
                      f"pos=({det.position[0]:.0f}, {det.position[1]:.0f})")
        return target_id

    # ---- 裁判操作（场地动态变化） ----

    def remove_target(self, target_id: int) -> None:
        """裁判移除目标（无效转运）"""
        if target_id in self._targets:
            target = self._targets.pop(target_id)
            logger.info(f"目标被移除: ID={target_id}, {target.info.description}")

    def add_target(self, info: TargetInfo, position: Tuple[float, float]) -> int:
        """裁判在场地中央添加目标（无效转运后重新放置）"""
        target_id = self._next_id
        self._next_id += 1
        now = time.time()

        self._targets[target_id] = TrackedTarget(
            id=target_id,
            info=info,
            position=position,
            first_seen=now,
            last_seen=now,
            seen_count=self.MIN_SEEN_COUNT,  # 直接视为稳定
        )
        logger.info(f"目标被添加: ID={target_id}, {info.description}, "
                     f"pos=({position[0]:.0f}, {position[1]:.0f})")
        return target_id

    def mark_in_safe_zone(self, target_id: int) -> None:
        """标记目标已进入安全区"""
        if target_id in self._targets:
            self._targets[target_id].status = TargetStatus.IN_SAFE_ZONE
            logger.info(f"目标已入安全区: ID={target_id}")

    def mark_being_transported(self, target_id: int) -> None:
        """标记目标正在被转运"""
        if target_id in self._targets:
            self._targets[target_id].status = TargetStatus.BEING_TRANSPORTED

    # ---- 工具 ----

    @staticmethod
    def _distance(p1: Tuple[float, float], p2: Tuple[float, float]) -> float:
        return math.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)

    def get_stats(self) -> dict:
        """世界地图统计信息"""
        counts = self.count_by_type()
        return {
            "total_targets": len(self._targets),
            "active": len(self.active_targets),
            "regular_supplies": counts.get(TargetType.REGULAR_SUPPLY, 0),
            "core_supplies": counts.get(TargetType.CORE_SUPPLY, 0),
            "injured": counts.get(TargetType.INJURED, 0),
            "dangerous": counts.get(TargetType.DANGEROUS, 0),
            "in_safe_zone": self.count_in_safe_zone(),
            "total_score": self.total_score(),
            "update_count": self._update_count,
        }

    def summary(self) -> str:
        """世界地图文本摘要"""
        stats = self.get_stats()
        lines = [
            f"世界地图 (更新 #{stats['update_count']}):",
            f"  场上目标: {stats['active']}",
            f"  普通物资: {stats['regular_supplies']} | "
            f"核心物资: {stats['core_supplies']} | "
            f"伤员: {stats['injured']} | "
            f"⚠️危险: {stats['dangerous']}",
            f"  已得分: {stats['total_score']}",
        ]
        return "\n".join(lines)
