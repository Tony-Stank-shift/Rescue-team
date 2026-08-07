"""
perception_pipeline.py —— 主感知管线

整合所有感知子模块，提供统一的 update() 接口：
  原始帧 → 检测 → 分类 → 位置估算 → 世界地图更新 → 对方跟踪

对接 autonomous_state 的主循环：
  pipeline = PerceptionPipeline(phase=PRELIMINARY)
  ...
  world_map = pipeline.update(frame, robot_position)
  # → 决策模块从 world_map 获取信息
"""

import logging
import time
from typing import List, Optional, Tuple

from .target_types import (
    Detection, DetectedTarget,
    CompetitionPhase, TargetType, TargetColor, TargetShape,
)
from .detection import AbstractDetector, MockDetector, CVDetector
from .classification import TargetClassifier
from .world_map import WorldMap, TrackedTarget
from .field_elements import FieldLayout, SafeZoneColor
from .opponent_tracker import OpponentTracker
from .sensor_fusion import SensorFusion, FusedObservation

logger = logging.getLogger("perception_pipeline")


class PerceptionPipeline:
    """
    主感知管线。

    每帧调用 update() → 返回更新后的 WorldMap。

    管线流程：
    1. 检测：detector.detect(frame) → List[Detection]
    2. 分类：classifier.classify_batch(detections) → List[DetectedTarget]
    3. 定位：对每个 DetectedTarget 估算场地坐标
    4. 融合：更新 WorldMap
    5. 对方的处理：更新 OpponentTracker
    """

    def __init__(self,
                 phase: CompetitionPhase = CompetitionPhase.PRELIMINARY,
                 use_mock: bool = True,
                 my_safe_zone_color: SafeZoneColor = SafeZoneColor.RED):
        """
        Args:
            phase: 初赛/决赛
            use_mock: True=MockDetector, False=CVDetector（需 OpenCV）
            my_safe_zone_color: 本队安全区颜色（抽签确定）
        """
        self._phase = phase
        self._my_color = my_safe_zone_color

        # 子模块
        if use_mock:
            self._detector: AbstractDetector = MockDetector(phase=phase)
        else:
            self._detector: AbstractDetector = CVDetector(phase=phase)

        self._classifier = TargetClassifier(phase=phase)
        self._world_map = WorldMap(field_layout=FieldLayout.standard())
        self._opponent_tracker = OpponentTracker()
        self._sensor_fusion = SensorFusion()

        # 统计
        self._frame_count = 0
        self._total_latency_ms = 0.0

        logger.info(f"PerceptionPipeline 初始化: phase={phase.name}, "
                     f"mock={use_mock}, my_color={my_safe_zone_color.name}")

    # ---- 属性 ----

    @property
    def world_map(self) -> WorldMap:
        return self._world_map

    @property
    def opponent_tracker(self) -> OpponentTracker:
        return self._opponent_tracker

    @property
    def sensor_fusion(self) -> SensorFusion:
        return self._sensor_fusion

    @property
    def phase(self) -> CompetitionPhase:
        return self._phase

    @phase.setter
    def phase(self, value: CompetitionPhase) -> None:
        """切换比赛阶段（创新实践环节）"""
        self._phase = value
        self._classifier.phase = value
        logger.info(f"感知管线阶段切换: {value.name}")

    @property
    def my_safe_zone_color(self) -> SafeZoneColor:
        return self._my_color

    @my_safe_zone_color.setter
    def my_safe_zone_color(self, value: SafeZoneColor) -> None:
        self._my_color = value

    # ---- 主循环接口 ----

    def update(self, frame=None,
               robot_position: Optional[Tuple[float, float]] = None,
               timestamp: Optional[float] = None) -> WorldMap:
        """
        单帧感知更新。

        Args:
            frame: 摄像头帧（BGR numpy 数组），Mock 模式下可为 None
            robot_position: 机器人当前场地坐标，None 则从 sensor_fusion 获取
            timestamp: 时间戳

        Returns:
            更新后的 WorldMap
        """
        t_start = time.time()
        self._frame_count += 1

        if timestamp is None:
            timestamp = t_start

        if robot_position is None:
            robot_position = self._sensor_fusion.get_position()

        # ─── 步骤 1：检测 ───
        detections: List[Detection] = self._detector.detect(frame)

        # ─── 步骤 2：分类 ───
        detected_targets: List[DetectedTarget] = self._classifier.classify_batch(
            detections, timestamp
        )

        # ─── 步骤 3：位置估算 ───
        for target in detected_targets:
            pos = self._detector.estimate_position(
                # 从 target.info 反查 detection（近似）
                Detection(
                    color=target.info.color,
                    shape=target.info.shape,
                    bbox=(0, 0, 30, 30),
                    confidence=target.confidence,
                ),
            )
            # 转换到场地坐标（机器人位置 + 相对位置）
            target.position = (
                robot_position[0] + pos[0],
                robot_position[1] + pos[1],
            )

        # ─── 步骤 4：世界地图更新 ───
        self._world_map.update(detected_targets, robot_position, timestamp)

        # ─── 步骤 5：对方跟踪 ───
        # 从检测中提取对方机器人（不是目标的其他移动物体）
        # 当前简化：使用非目标的检测作为对方机器人候选
        opponent_pos = self._extract_opponent(detections, detected_targets)
        self._opponent_tracker.update(opponent_pos, robot_position, timestamp)

        # ─── 统计 ───
        latency_ms = (time.time() - t_start) * 1000
        self._total_latency_ms += latency_ms

        if self._frame_count % 50 == 0:  # 每秒一次日志
            logger.debug(
                f"帧 #{self._frame_count}: {len(detections)} 检测 → "
                f"{len(detected_targets)} 分类 → "
                f"{len(self._world_map.active_targets)} 活跃目标, "
                f"{latency_ms:.1f}ms"
            )

        return self._world_map

    # ---- 对方机器人提取 ----

    def _extract_opponent(self, detections: List[Detection],
                          targets: List[DetectedTarget]) -> Optional[Tuple[float, float]]:
        """
        从检测结果中提取对方机器人位置。

        策略：未分类为救援目标的移动物体 + 尺寸接近 300×300mm
        当前简化实现：如果有检测但未分类，取最大的未分类检测。
        """
        classified_pixels = {t.pixel_position for t in targets}

        unclassified = []
        for det in detections:
            if det.center_pixel not in classified_pixels:
                unclassified.append(det)

        if not unclassified:
            return None

        # 取面积最大的（对方机器人较大）
        largest = max(unclassified, key=lambda d: d.contour_area)
        pos = self._detector.estimate_position(largest)
        return pos

    # ---- 切换配置 ----

    def switch_phase(self, phase: CompetitionPhase) -> None:
        """切换比赛阶段"""
        self.phase = phase
        # 重新初始化检测器
        if isinstance(self._detector, MockDetector):
            self._detector = MockDetector(phase=phase)
        else:
            self._detector = CVDetector(phase=phase)

    # ---- 查询 ----

    def get_stats(self) -> dict:
        """获取感知管线统计"""
        avg_latency = (self._total_latency_ms / self._frame_count
                       if self._frame_count > 0 else 0.0)
        return {
            "frame_count": self._frame_count,
            "avg_latency_ms": avg_latency,
            "world_map": self._world_map.get_stats(),
            "opponent": self._opponent_tracker.get_stats(),
        }

    def summary(self) -> str:
        """感知管线文本摘要"""
        stats = self.get_stats()
        return (
            f"感知管线 (帧 #{stats['frame_count']}, "
            f"平均 {stats['avg_latency_ms']:.1f}ms)\n"
            f"{self._world_map.summary()}"
        )


# ============================================================
# 独立测试入口
# ============================================================

if __name__ == "__main__":
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s [%(levelname)-7s] %(name)s: %(message)s",
        datefmt="%H:%M:%S",
    )

    print("=" * 50)
    print("  感知管线 — Mock 模式测试")
    print("=" * 50)

    # 创建初赛管线
    pipeline = PerceptionPipeline(
        phase=CompetitionPhase.PRELIMINARY,
        use_mock=True,
    )

    # 模拟 10 帧
    for i in range(10):
        world_map = pipeline.update()
        if i == 0 or i == 9:
            print(world_map.summary())

    print("\n--- 切换到决赛模式 ---")
    pipeline.switch_phase(CompetitionPhase.FINAL)

    for i in range(5):
        world_map = pipeline.update()

    print(world_map.summary())
    print(f"\n最终统计: {pipeline.get_stats()}")
    print("✅ 感知管线 Mock 测试完成")
