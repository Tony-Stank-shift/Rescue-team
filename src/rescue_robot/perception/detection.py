"""
detection.py —— 目标检测

提供可插拔的检测器架构：
  AbstractDetector  — 抽象基类
  CVDetector        — 传统 CV（HSV 颜色分割 + 轮廓检测 + 形状分析）
  MockDetector      — 模拟检测器（场地中放置假目标）

CV 检测管线：
  原始帧 → HSV 颜色阈值分割 → 形态学处理 →
  轮廓查找 → 形状判定 → 位置估算 → Detection 列表
"""

import logging
import math
import time
from typing import Dict, List, Optional, Tuple

from .target_types import (
    Detection, TargetColor, TargetShape, TargetType, CompetitionPhase,
    get_target_config,
)

logger = logging.getLogger("detection")


# ============================================================
# HSV 颜色阈值表
# ============================================================
# 格式：(H_min, S_min, V_min), (H_max, S_max, V_max)
# OpenCV HSV 范围：H∈[0,179], S∈[0,255], V∈[0,255]

HSV_RANGES: Dict[TargetColor, Tuple[Tuple[int, int, int], Tuple[int, int, int]]] = {
    TargetColor.RED:       ((0, 100, 100), (10, 255, 255)),   # 红色（含低 H 值）
    TargetColor.YELLOW:    ((22, 100, 100), (35, 255, 255)),   # 黄色
    TargetColor.GREEN:     ((40, 80, 60), (80, 255, 255)),     # 绿色
    TargetColor.BLUE:      ((95, 80, 60), (125, 255, 255)),    # 蓝色
    TargetColor.ORANGE:    ((8, 100, 100), (20, 255, 255)),    # 橘色
    TargetColor.BLACK:     ((0, 0, 0), (179, 255, 60)),        # 黑色（低 V）
    TargetColor.LIGHT_BLUE:((85, 40, 100), (105, 200, 255)),   # 浅蓝
    TargetColor.BROWN:     ((10, 100, 50), (25, 200, 150)),    # 棕色
    TargetColor.WHITE:     ((0, 0, 180), (179, 30, 255)),      # 白色（低 S）
}

# 红色还有第二个范围（H 接近 179）
HSV_RANGES_RED2 = ((160, 100, 100), (179, 255, 255))


# ============================================================
# 形状判定参数
# ============================================================

# 基于轮廓近似顶点数判断形状
SHAPE_VERTEX_RANGES = {
    TargetShape.CUBE:              (4, 8),    # 正方体 → 4-8 个顶点（含透视变形）
    TargetShape.TRIANGULAR_PYRAMID:(3, 5),    # 三棱锥 → 3-5 个顶点
    TargetShape.CUBOID:            (4, 8),    # 长方体 → 4-8 个顶点
    TargetShape.CYLINDER:          (8, 20),   # 圆柱体 → 8+ 个顶点（近似圆形）
    TargetShape.CONE_FRUSTUM:      (8, 20),   # 圆锥台 → 8+ 个顶点
    TargetShape.SPHERE:            (8, 30),   # 球体 → 8+ 个顶点
}

# 轮廓面积比（contourArea / bboxArea）辅助区分
SHAPE_AREA_RATIOS = {
    # 正方体：轮廓面积 ≈ bbox 面积（俯视图是正方形）
    TargetShape.CUBE: (0.5, 1.0),
    # 球体 / 圆形：轮廓面积 ≈ π/4 × bbox 面积 ≈ 0.785
    TargetShape.SPHERE: (0.6, 0.9),
    TargetShape.CYLINDER: (0.55, 0.95),
    TargetShape.CONE_FRUSTUM: (0.5, 0.9),
    # 三棱锥：相对小的面积比
    TargetShape.TRIANGULAR_PYRAMID: (0.3, 0.7),
}


# ============================================================
# 抽象检测器
# ============================================================

class AbstractDetector:
    """目标检测器抽象基类"""

    def detect(self, frame) -> List[Detection]:
        """
        从图像帧中检测目标。

        Args:
            frame: numpy.ndarray (H, W, 3) BGR 图像

        Returns:
            List[Detection]: 检测到的目标列表
        """
        raise NotImplementedError

    def estimate_position(self, detection: Detection,
                          camera_height_mm: float = 200,
                          camera_fov_deg: float = 70,
                          image_size: Tuple[int, int] = (640, 480)) -> Tuple[float, float]:
        """
        根据像素位置估算目标在场地中的坐标。

        使用针孔相机模型 + 已知目标尺寸估算距离。

        Args:
            detection: 检测结果
            camera_height_mm: 相机安装高度
            camera_fov_deg: 相机视场角
            image_size: 图像尺寸 (width, height)

        Returns:
            (x_mm, y_mm): 相对于机器人的场地坐标
        """
        # 默认实现：使用目标像素大小与已知物理尺寸的比例估算距离
        return (0.0, 0.0)


# ============================================================
# CV 检测器
# ============================================================

class CVDetector(AbstractDetector):
    """
    基于传统计算机视觉的目标检测器。

    管线：
    1. 对每种目标颜色做 HSV 阈值分割
    2. 形态学开闭运算去噪
    3. 轮廓查找
    4. 轮廓近似 → 顶点数判断形状
    5. 面积比辅助判定
    6. 输出 Detection 列表
    """

    def __init__(self, phase: CompetitionPhase = CompetitionPhase.PRELIMINARY):
        self._phase = phase
        self._target_config = get_target_config(phase)

        # 需要检测的颜色（从当前阶段配置中提取）
        self._colors_to_detect = self._get_colors_to_detect()

        # 最小轮廓面积（过滤噪点）
        self._min_contour_area = 200  # 像素²

        logger.info(f"CVDetector 初始化: phase={phase.name}, "
                     f"检测颜色={[c.name for c in self._colors_to_detect]}")

    def _get_colors_to_detect(self) -> List[TargetColor]:
        """获取当前阶段需要检测的所有颜色"""
        colors = set()
        for color, shape in self._target_config:
            colors.add(color)
        return list(colors)

    def detect(self, frame) -> List[Detection]:
        """
        对单帧图像执行目标检测。

        注意：此方法依赖 OpenCV（cv2）。在 WSL/Mock 环境下，
        cv2 可能不可用，此时应使用 MockDetector。
        """
        try:
            import cv2
            import numpy as np
        except ImportError:
            logger.warning("OpenCV (cv2) 未安装，CVDetector 无法使用。"
                           "请使用 MockDetector 进行测试，"
                           "或在树莓派上安装: pip install opencv-python")
            return []

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        detections = []

        for color in self._colors_to_detect:
            # 获取颜色掩码
            mask = self._create_color_mask(hsv, color)

            # 形态学处理：开运算去噪 + 闭运算填洞
            kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

            # 查找轮廓
            contours, _ = cv2.findContours(
                mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
            )

            for contour in contours:
                area = cv2.contourArea(contour)
                if area < self._min_contour_area:
                    continue

                # Bounding box
                x, y, w, h = cv2.boundingRect(contour)

                # 轮廓近似 → 顶点数
                peri = cv2.arcLength(contour, True)
                approx = cv2.approxPolyDP(contour, 0.04 * peri, True)
                vertices = len(approx)

                # 形状判定
                shape = self._classify_shape(contour, area, w, h, vertices)

                detections.append(Detection(
                    color=color,
                    shape=shape,
                    bbox=(x, y, w, h),
                    confidence=self._calc_confidence(area, vertices),
                    contour_area=area,
                    contour_vertices=vertices,
                ))

        return detections

    def _create_color_mask(self, hsv, color: TargetColor):
        """为指定颜色创建 HSV 掩码"""
        import cv2
        import numpy as np

        ranges = HSV_RANGES.get(color)
        if ranges is None:
            return np.zeros(hsv.shape[:2], dtype=np.uint8)

        lower, upper = ranges
        mask = cv2.inRange(hsv, np.array(lower), np.array(upper))

        # 红色需要合并两个 HSV 区间
        if color == TargetColor.RED:
            lower2, upper2 = HSV_RANGES_RED2
            mask2 = cv2.inRange(hsv, np.array(lower2), np.array(upper2))
            mask = cv2.bitwise_or(mask, mask2)

        return mask

    def _classify_shape(self, contour, area: float, w: int, h: int,
                        vertices: int) -> TargetShape:
        """根据轮廓特征判定形状"""
        import cv2

        bbox_area = w * h
        area_ratio = area / bbox_area if bbox_area > 0 else 0

        # Hu 矩（形状描述子）
        moments = cv2.moments(contour)
        hu = cv2.HuMoments(moments)

        # 策略：先根据顶点数筛选，再用面积比和 Hu 矩细化
        candidates = []

        for shape, (v_min, v_max) in SHAPE_VERTEX_RANGES.items():
            if v_min <= vertices <= v_max:
                # 检查面积比
                r_min, r_max = SHAPE_AREA_RATIOS.get(shape, (0.2, 1.0))
                ratio_match = r_min <= area_ratio <= r_max
                candidates.append((shape, ratio_match))

        if not candidates:
            return TargetShape.UNKNOWN

        # 优先选择面积比匹配的
        for shape, ratio_match in candidates:
            if ratio_match:
                return shape

        # 否则返回顶点数最匹配的
        return candidates[0][0]

    def _calc_confidence(self, area: float, vertices: int) -> float:
        """计算检测置信度（启发式）"""
        # 面积越大越可信，顶点数在合理范围内越可信
        area_score = min(1.0, area / 2000.0)  # 2000 像素² → 满分
        vertex_score = 0.5 if 3 <= vertices <= 20 else 0.2
        return (area_score + vertex_score) / 2.0

    def estimate_position(self, detection: Detection,
                          camera_height_mm: float = 200,
                          camera_fov_deg: float = 70,
                          image_size: Tuple[int, int] = (640, 480)) -> Tuple[float, float]:
        """
        根据像素位置和已知目标尺寸估算场地坐标。

        距离 ≈ (已知目标尺寸 × 焦距) / 像素尺寸
        水平偏移 ≈ (像素偏移 / 图像宽度) × 距离视野范围
        """
        img_w, img_h = image_size
        cx, cy = detection.center_pixel

        # 如果检测到目标的实际尺寸（取配置中对应项的 max_dimension）
        # 这里使用默认值 40mm
        known_size_mm = 40.0
        px_size = math.sqrt(detection.contour_area)

        # 焦距估算（像素）
        fov_rad = math.radians(camera_fov_deg)
        focal_length_px = img_w / (2.0 * math.tan(fov_rad / 2.0))

        # 距离估算（小孔成像）
        if px_size > 0:
            distance_mm = (known_size_mm * focal_length_px) / px_size
        else:
            distance_mm = 500.0  # 默认 500mm

        # 角度偏移
        offset_x_px = cx - img_w / 2.0
        angle_x = math.atan2(offset_x_px, focal_length_px)

        # 场地坐标（机器人前方为 +y，右方为 +x）
        x_mm = distance_mm * math.sin(angle_x)
        y_mm = distance_mm * math.cos(angle_x) - camera_height_mm * 0.5

        return (x_mm, y_mm)


# ============================================================
# Mock 检测器
# ============================================================

class MockDetector(AbstractDetector):
    """
    模拟检测器：返回预定义的假目标列表。

    用于本地开发、CI 测试和无摄像头环境。
    模拟一个 3000×3000 场地，随机放置 20 个目标。
    """

    def __init__(self, phase: CompetitionPhase = CompetitionPhase.PRELIMINARY,
                 seed: int = 42):
        import random
        self._phase = phase
        self._target_config = get_target_config(phase)
        self._rng = random.Random(seed)
        self._frame_count = 0

        # 生成 mock 目标列表
        self._mock_targets: List[Detection] = self._generate_mock_targets()

        logger.info(f"MockDetector 初始化: phase={phase.name}, "
                     f"mock 目标数={len(self._mock_targets)}")

    def _generate_mock_targets(self) -> List[Detection]:
        """生成模拟场地上的目标"""
        detections = []

        # 初赛：8 普 + 4 核 + 4 伤 + 4 危 = 20
        # 决赛：10 普 + 5 核 + 5 伤 + 5 危 = 25
        counts = {
            TargetType.REGULAR_SUPPLY: 8 if self._phase == CompetitionPhase.PRELIMINARY else 10,
            TargetType.CORE_SUPPLY: 4 if self._phase == CompetitionPhase.PRELIMINARY else 5,
            TargetType.INJURED: 4 if self._phase == CompetitionPhase.PRELIMINARY else 5,
            TargetType.DANGEROUS: 4 if self._phase == CompetitionPhase.PRELIMINARY else 5,
        }

        target_id = 0
        for (color, shape), info in self._target_config.items():
            count = counts.get(info.type, 2)
            for _ in range(count):
                # 随机位置（场地 3000×3000mm，转换成像素）
                px = self._rng.randint(50, 590)
                py = self._rng.randint(50, 430)
                w, h = 30, 30  # 模拟 bbox

                detections.append(Detection(
                    color=color,
                    shape=shape,
                    bbox=(px, py, w, h),
                    confidence=self._rng.uniform(0.85, 0.99),
                    contour_area=w * h * 0.7,
                    contour_vertices=self._rng.randint(4, 12),
                ))
                target_id += 1

        return detections

    def detect(self, frame=None) -> List[Detection]:
        """返回模拟检测结果"""
        self._frame_count += 1

        # 模拟帧间微小位置变化
        result = []
        for det in self._mock_targets:
            dx = self._rng.randint(-2, 2)
            dy = self._rng.randint(-2, 2)
            x, y, w, h = det.bbox
            result.append(Detection(
                color=det.color,
                shape=det.shape,
                bbox=(x + dx, y + dy, w, h),
                confidence=det.confidence,
                contour_area=det.contour_area,
                contour_vertices=det.contour_vertices,
            ))
        return result

    def estimate_position(self, detection: Detection,
                          camera_height_mm: float = 200,
                          camera_fov_deg: float = 70,
                          image_size: Tuple[int, int] = (640, 480)) -> Tuple[float, float]:
        """
        Mock 位置估算：将像素坐标映射到场地坐标。

        场地 3000×3000mm → 图像 640×480px
        简单线性映射（模拟俯视相机）。
        """
        img_w, img_h = image_size
        field_w, field_h = 3000.0, 3000.0

        cx, cy = detection.center_pixel
        x_mm = (cx / img_w) * field_w - field_w / 2.0  # 居中
        y_mm = field_h - (cy / img_h) * field_h        # 翻转 Y

        return (x_mm, y_mm)
