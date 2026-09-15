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
import math
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

        # 最近一帧的原始检测结果（供"套取视觉确认"等使用，像素空间）
        self._last_detections: List[Detection] = []
        self._last_frame_size: Optional[Tuple[int, int]] = None
        self._theta_warned = False
        #: 视觉是否可用（T0-5）。False 时**绝不**伪造目标：无帧就是空地图，
        #: 让决策层原地等待/安全停车，而不是去追幻影目标。
        self._vision_available = True
        self._vision_lost_frames = 0
        #: 因"地平面测距解算失败"而被丢弃的检测数（T1-13 诊断）
        self._dropped_no_range = 0
        #: 本帧是否真的拿到了图像（T1-14：套取确认要区分"无帧"与"槽内空"）
        self._frame_ok = False
        self._no_frame_warned = 0   # 只告警一次：调用方未传 robot_theta

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

    def set_vision_available(self, available: bool) -> None:
        """标记视觉是否可用（T0-5）。不可用时感知输出空地图，绝不伪造目标。"""
        self._vision_available = bool(available)
        self._vision_lost_frames = 0
        #: 因"地平面测距解算失败"而被丢弃的检测数（T1-13 诊断）
        self._dropped_no_range = 0
        #: 本帧是否真的拿到了图像（T1-14：套取确认要区分"无帧"与"槽内空"）
        self._frame_ok = False
        self._no_frame_warned = 0
        if not available:
            logger.critical("感知已标记为「视觉不可用」：将输出**空世界地图**，"
                            "不会伪造任何目标（决策层会原地等待/安全停车）")

    @property
    def vision_available(self) -> bool:
        return self._vision_available

    def check_sleeve_occupied(self, roi_norm=None,
                              min_confidence: float = 0.35) -> Optional[bool]:
        """
        判断 U 型槽内是否检测到目标（本车无硬件"套住检测"时的视觉替代）。

        判据：检测框中心落在归一化 ROI 内即认为槽里有东西。
        ROI 来自 ``config.Camera.SLEEVE_ROI``（归一化，需真机标定）。

        Returns:
            True  = 槽内检测到目标（视为已套住）
            False = **有图像**且槽内为空（视为没套住 → 上层抬爪后退重试）
            None  = **本帧没有图像**（无法判断，调用方不应据此判失败）
        """
        from .. import config as _cfg
        if roi_norm is None:
            roi_norm = getattr(_cfg, "SLEEVE_ROI", None)
        if not roi_norm or len(roi_norm) != 4:
            return True                     # 未配置 ROI → 不做判断（按成功处理）
        # T1-14：必须区分"**这一帧根本没有图像**"与"有图像但槽内空"。
        # 旧实现两者都 `return False`（判套取失败）→ 摄像头掉帧/未就绪时会**连续判失败**，
        # 上层"抬爪后退重试"白扔几趟（只能靠连续 5 次后自动关闭确认自愈）。
        # 现在：无帧 → 返回 None（未知），由调用方按"无法判断 → 不否定"处理。
        if not self._frame_ok:
            if self._no_frame_warned == 0:
                self._no_frame_warned = 1
                logger.warning("套取视觉确认：本帧无图像 → 返回『未知』（不判失败；"
                               "掉帧/摄像头未就绪时不应把套取误判为失败）")
            return None
        if not self._last_detections:
            return False                    # 有帧但没有任何检测 → 槽内空
        img_w, img_h = self._last_frame_size or (640, 480)
        x1, y1, x2, y2 = roi_norm
        px1, py1, px2, py2 = x1 * img_w, y1 * img_h, x2 * img_w, y2 * img_h
        for det in self._last_detections:
            cx, cy = det.center_pixel
            if px1 <= cx <= px2 and py1 <= cy <= py2:
                if det.confidence >= min_confidence:
                    return True
        return False

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
               timestamp: Optional[float] = None,
               robot_theta: Optional[float] = None) -> WorldMap:
        """
        单帧感知更新。

        Args:
            frame: 摄像头帧（BGR numpy 数组），Mock 模式下可为 None
            robot_position: 机器人当前场地坐标，None 则从 sensor_fusion 获取
            timestamp: 时间戳
            robot_theta: 机器人航向（rad，从 +X 逆时针）。**必须传**，
                否则车体系→场地系的换算会退化成"只平移不旋转"，目标定位随车头方向整体错位。

        Returns:
            更新后的 WorldMap
        """
        t_start = time.time()
        self._frame_count += 1

        # T0-5：视觉不可用时的显式提醒（每 5 秒一次，不刷屏）
        if not self._vision_available and frame is None:
            self._vision_lost_frames += 1
            if self._vision_lost_frames % 250 == 1:      # 50Hz → 约每 5s
                logger.error("⚠️ 视觉不可用：本帧无摄像头数据 → 世界地图保持为空"
                             "（不会伪造目标）。请检查摄像头接线/索引，"
                             f"已持续 {self._vision_lost_frames} 帧")

        if timestamp is None:
            timestamp = t_start

        if robot_position is None:
            robot_position = self._sensor_fusion.get_position()

        # ─── 步骤 1：检测 ───
        self._frame_ok = frame is not None      # T1-14
        detections: List[Detection] = self._detector.detect(frame)
        # 保留原始检测（像素空间），供套取视觉确认等使用
        self._last_detections = detections
        if frame is not None:
            try:
                _h, _w = frame.shape[:2]
                self._last_frame_size = (int(_w), int(_h))
            except Exception:
                pass

        # ─── 步骤 2：分类 ───
        detected_targets: List[DetectedTarget] = self._classifier.classify_batch(
            detections, timestamp
        )

        # ─── 步骤 3：位置估算 ───
        # ① 首选：底边 + 相机倾角的【地平面测距】——目标贴地时准确且与形状无关；
        #    旧实现此处传的是硬编码假框 Detection(bbox=(0,0,30,30))，导致
        #    center_pixel=(15,15)、contour_area=0 → 距离恒为兜底值 500mm，视觉定位形同虚设。
        # ② 兜底：面积法（仅当没有有效检测框 / 无法解算时使用，精度差）。
        positioned = []
        for target in detected_targets:
            pos = None
            _has_gp = hasattr(self._detector, "estimate_ground_position")
            if target.has_pixel_bbox and _has_gp:
                bx, by, bw, bh = target.pixel_bbox
                pos = self._detector.estimate_ground_position(
                    center_x_px=bx + bw / 2.0,
                    bottom_y_px=by + bh,
                )
            if pos is None and not _has_gp:
                # 只有"检测器本身没有地平面测距"（Mock）时才走旧的面积法兜底。
                #
                # T1-13：**不能**在"有地平面测距但解算失败"时也退到面积法 ——
                # 兜底路径构造 `Detection` 时没传 `contour_area`（默认 0.0）→
                # `px_size = sqrt(0) = 0` → 距离恒为硬编码 `500.0mm`，再减 105mm 偏置 →
                # 任何解算失败的检测都会被放到"车前 0.5m"这个**假位置**，
                # 经世界地图 3 帧确认后变成"车前 0.5m 的假目标"→ 白跑一趟。
                # 正确做法：解算不出来的帧**丢弃该目标**（宁可少一个目标，也不要假坐标）。
                pos = self._detector.estimate_position(
                    Detection(
                        color=target.info.color,
                        shape=target.info.shape,
                        bbox=target.pixel_bbox,
                        confidence=target.confidence,
                        contour_area=target.contour_area,
                    ),
                )
            if pos is None:
                self._dropped_no_range += 1
                if self._dropped_no_range % 50 == 1:
                    logger.warning(f"⚠️ 地平面测距解算失败 → 丢弃该检测（累计 "
                                   f"{self._dropped_no_range} 次）。视野内目标过远/过近"
                                   f"或相机倾角未标定时会出现，**不会再伪造一个坐标**")
                continue
            target.position = (0.0, 0.0)
            positioned.append((target, pos))

            # ── 车体系 → 场地系：**必须按机器人航向旋转**（修复 S-05）──
            # 旧实现直接把车体系偏移加到场地坐标（只平移、不旋转）→ 目标定位随车头方向
            # 整体错位，表现为"明明感知到目标却永远抓不到"。
            # 约定：theta 从 +X 逆时针；车体"前"=(cosθ,sinθ)、车体"右"=(sinθ,-cosθ)；
            #      estimate_* 返回 (右向偏移, 前向距离)。
            right_off, forward = pos[0], pos[1]
            if robot_theta is None:
                # 未提供航向：保持旧行为，但**只告警一次**（不再静默错位）
                if not self._theta_warned:
                    logger.warning(
                        "perception.update 未收到 robot_theta → 目标定位忽略机器人朝向，"
                        "会随车头方向整体错位；请在调用处传 robot_theta=theta")
                    self._theta_warned = True
                dx, dy = right_off, forward
            else:
                c, s = math.cos(robot_theta), math.sin(robot_theta)
                dx = forward * c + right_off * s
                dy = forward * s - right_off * c
            target.position = (robot_position[0] + dx, robot_position[1] + dy)

        # ─── 步骤 4：世界地图更新 ───
        self._world_map.update([t for t, _p in positioned], robot_position, timestamp)

        # ─── 步骤 5：对方跟踪 ───
        # 从检测中提取对方机器人（不是目标的其他移动物体）
        # T2-15：必须传 robot_position/robot_theta —— 旧实现把 `_extract_opponent`
        # 返回的**车体系偏移**直接当**场地坐标**交给 tracker，而 tracker 拿它与
        # `robot_position` 比距离（判定"接触"）→ 判定随车头方向漂移、完全不可用。
        # 现在与目标走同一套"车体系→场地系"旋转。
        opponent_pos = self._extract_opponent(detections, detected_targets,
                                             robot_position=robot_position,
                                             robot_theta=robot_theta)
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
                          targets: List[DetectedTarget],
                          robot_position: Optional[Tuple[float, float]] = None,
                          robot_theta: Optional[float] = None) -> Optional[Tuple[float, float]]:
        """从检测结果中提取对方机器人的**场地坐标**（T2-15）。

        策略：未分类为救援目标的检测里取最大的一个（对方机器人最大）。

        ⚠️ 返回值必须是**场地坐标**：旧实现用 `estimate_position()`（面积法，返回
        车体系"右向/前向"偏移）直接当场地坐标交给 tracker，而 tracker 拿它与
        `robot_position` 比距离判"接触" → 判定随车头方向漂移，完全不可用。
        现在：① 优先用地平面测距（与主路径一致，面积法在 4 种形状混装时不可用）；
        ② 按 θ 把车体系偏移旋到场地系（与目标同一套公式）。
        """
        # 用中心像素 + 尺寸做匹配（浮点相等不可靠，T3 级问题顺手规避）
        def _same(a, b, tol=4.0):
            return abs(a[0] - b[0]) <= tol and abs(a[1] - b[1]) <= tol

        unclassified = [d for d in detections
                        if not any(_same(d.center_pixel, t.pixel_position)
                                   for t in targets)]
        if not unclassified:
            return None
        largest = max(unclassified, key=lambda d: d.contour_area)

        # ① 车体系偏移（右向, 前向）
        rel = None
        if hasattr(self._detector, "estimate_ground_position"):
            try:
                bx, by, bw, bh = largest.bbox
                rel = self._detector.estimate_ground_position(
                    center_x_px=bx + bw / 2.0, bottom_y_px=by + bh)
            except Exception:
                rel = None
        if rel is None:
            rel = self._detector.estimate_position(largest)

        # ② 车体系 → 场地系（与目标一致）
        if robot_position is None or robot_theta is None:
            return None      # 拿不到位姿就不上报，宁可没有对手信息也不要错的坐标
        right_off, forward = rel[0], rel[1]
        c, sn = math.cos(robot_theta), math.sin(robot_theta)
        dx = forward * c + right_off * sn
        dy = forward * sn - right_off * c
        return (robot_position[0] + dx, robot_position[1] + dy)

    @property
    def opponent_position(self) -> Optional[Tuple[float, float]]:
        """对方机器人的场地坐标（置信度足够且近期可见时；否则 None）（T1-12）。

        旧实现算出对手位置后**没有任何生产消费者**（只进 `get_stats()`）→
        导航的动态避障参数 `opponent_position` 永远是 None，避障形同虚设。
        """
        try:
            st = self._opponent_tracker.state
        except Exception:
            return None
        if st is None or st.confidence <= 0.0 or st.seen_count <= 0:
            return None
        return (float(st.position[0]), float(st.position[1]))

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
