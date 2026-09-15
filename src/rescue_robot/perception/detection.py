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
    # 蓝色：提高饱和度下界（S≥120），把"淡淡的浅蓝"让给 LIGHT_BLUE。
    # 旧范围 S≥80 与 LIGHT_BLUE 的 H95~108/S80~255 完全重叠，检测顺序又让 BLUE 先命中，
    # 于是真实的浅蓝"危险目标"会被判成蓝色、蓝色救援目标被判成浅蓝(=危险目标) → 直接丢分。
    TargetColor.BLUE:      ((95, 120, 40), (128, 255, 255)),   # 蓝色（饱和度高）
    TargetColor.ORANGE:    ((4, 205, 35), (24, 255, 255)),   # 橘色（H 4~24 认橙，S≥205 排红色，红 S≤201）
    TargetColor.BLACK:     ((0, 0, 0), (179, 255, 60)),        # 黑色（低 V）
    # 浅蓝（危险目标）：H 85~110 + 较高亮度。
    # ⚠️ 与 BLUE 的 H 区间天然重叠，**只能靠检测顺序**（LIGHT_BLUE 先测）区分，
    # 因此真正防误判的闸门放在分类器：_fuzzy_match 已禁止任何颜色容差命中
    # DANGEROUS（见 classification.py），避免"蓝色救援目标被判成危险目标永不被搬"。
    # 若决赛现场公布的"蓝/浅蓝"无法区分，需现场标定这两个范围（config/YAML 已留口）。
    TargetColor.LIGHT_BLUE:((85, 20, 110), (110, 255, 255)),   # 浅蓝
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

# 长宽比判据（max(w,h)/min(w,h)）——区分正方体与长方体的**主判据**。
# 伤员是 80×40×40 的长方体，俯视长宽比≈2；正方体≈1。旧实现完全没用这个信息。
SHAPE_ASPECT_RATIOS = {
    TargetShape.CUBE:   (0.60, 1.45),
    TargetShape.CUBOID: (1.45, 3.60),
}

# 轮廓面积比（contourArea / bboxArea）辅助区分
SHAPE_AREA_RATIOS = {
    # 正方体：收紧到 (0.75,1.0)——旧值 (0.5,1.0) 把长方体的取值区间也包住了，
    # 加上 CUBE 在字典里排在 CUBOID 之前，导致所有方体都被判成 CUBE（伤员必误判）。
    TargetShape.CUBE: (0.75, 1.0),
    # 长方体（伤员）：显式给区间，不再落到默认 (0.2,1.0) 兜底
    TargetShape.CUBOID: (0.45, 0.9),
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
                          camera_height_mm: float = 210,
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

    def __init__(self, phase: CompetitionPhase = CompetitionPhase.PRELIMINARY,
                 camera_height_mm: Optional[float] = None,
                 camera_tilt_deg: Optional[float] = None,
                 camera_fov_deg: Optional[float] = None,
                 image_size: Optional[Tuple[int, int]] = None):
        self._phase = phase
        self._target_config = get_target_config(phase)

        # 需要检测的颜色（从当前阶段配置中提取）
        self._colors_to_detect = self._get_colors_to_detect()

        # 最小轮廓面积（过滤噪点）
        self._min_contour_area = 200  # 像素²

        # ── 相机几何（地平面测距用；默认取 config.Camera，可显式覆盖）──
        from .. import config as _cfg
        self._camera_height_mm = (camera_height_mm if camera_height_mm is not None
                                  else getattr(_cfg, "CAMERA_HEIGHT_MM", 210.0))
        self._camera_tilt_deg = (camera_tilt_deg if camera_tilt_deg is not None
                                 else getattr(_cfg, "CAMERA_TILT_DEG", 30.0))
        self._camera_fov_deg = (camera_fov_deg if camera_fov_deg is not None
                                else getattr(_cfg, "CAMERA_FOV_DEG", 77.0))
        self._image_size = tuple(image_size if image_size is not None
                                 else getattr(_cfg, "CAMERA_RES", (640, 480)))

        logger.info(f"CVDetector 初始化: phase={phase.name}, "
                     f"检测颜色={[c.name for c in self._colors_to_detect]}, "
                     f"相机 h={self._camera_height_mm:.0f}mm "
                     f"tilt={self._camera_tilt_deg:.1f}° "
                     f"fov={self._camera_fov_deg:.0f}° "
                     f"res={self._image_size}")

    def _get_colors_to_detect(self) -> List[TargetColor]:
        """
        需要检测的颜色（顺序敏感）。

        ⚠️ 顺序必须"特殊色在前"：LIGHT_BLUE（浅蓝=危险目标）要先于 BLUE。
        旧实现按配置表顺序（BLUE 在前），配合重叠的 HSV 区间，会把浅蓝判成蓝、
        把蓝判成浅蓝 → 前者导致危险目标被当救援目标去搬（违规），
        后者导致真实救援目标被当危险目标永不被搬（丢分）。
        """
        colors = {c for c, _ in self._target_config}
        # 特殊色优先：LIGHT_BLUE（浅蓝/危险）必须先于 BLUE 检测
        priority = [TargetColor.LIGHT_BLUE, TargetColor.BLUE]
        ordered = [c for c in priority if c in colors]
        ordered += [c for c in colors if c not in priority]
        return ordered

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

        if frame is None:
            # 无帧（摄像头未就绪/掉线）：视为"本帧无目标"，不是错误。
            # 旧实现在这里对 None 调 cv2.cvtColor 会抛异常，被主循环捕获后
            # 整轮 _run_once 被跳过 → 决策/导航/转运全不执行（表现为"原地去世"）。
            return []

        # 用【真实帧尺寸】覆盖假设值：摄像头实际分辨率常常不是 config 里的假设值，
        # 而地平面测距（焦距/主点）强依赖分辨率，尺寸错了距离会整体算错。
        try:
            _h, _w = frame.shape[:2]
            if _w > 0 and _h > 0:
                self._image_size = (int(_w), int(_h))
        except Exception:
            pass

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

                orient = self._estimate_orientation(contour)
                detections.append(Detection(
                    color=color,
                    shape=shape,
                    bbox=(x, y, w, h),
                    confidence=self._calc_confidence(area, vertices),
                    contour_area=area,
                    contour_vertices=vertices,
                    orientation=orient,
                ))

        return self._suppress_overlapping(detections)

    @staticmethod
    def _suppress_overlapping(detections: "List[Detection]",
                              iou_thr: float = 0.45,
                              center_thr_px: float = 12.0) -> "List[Detection]":
        """同一物理物体在多颜色掩码重叠区会被检出**多次** → 去重（T2-17）。

        为什么必须去重：`LIGHT_BLUE` 与 `BLUE` 的 HSV 桶在 H∈[95,110] 区间重叠，
        而 `detect()` 对每种颜色**独立**跑掩码并把结果全部 append，代码注释声称
        "顺序特殊色在前"就防住了 —— 实际上顺序**只决定谁先命中，不能阻止后者再命中**。
        实测：一块饱和浅蓝面同时产出 `LIGHT_BLUE` 与 `BLUE` 两个 Detection，
        经分类器可能变成"危险目标 + 核心物资"两个目标进世界地图 →
        最坏是**把危险目标当救援目标搬**（赛项违规）。

        做法：保留先出现的（即 `_get_colors_to_detect()` 排好序的**高优先级**颜色），
        抑制与它 bbox 高度重叠 / 中心过近的后继检测。
        """
        kept = []
        for det in detections:
            dup = False
            for k in kept:
                if CVDetector._iou(det.bbox, k.bbox) >= iou_thr or \
                   CVDetector._center_dist(det.bbox, k.bbox) <= center_thr_px:
                    dup = True
                    break
            if not dup:
                kept.append(det)
        if len(kept) != len(detections):
            logger.debug(f"颜色重叠去重: {len(detections)} → {len(kept)} 个检测")
        return kept

    @staticmethod
    def _iou(a, b) -> float:
        ax, ay, aw, ah = a
        bx, by, bw, bh = b
        x1, y1 = max(ax, bx), max(ay, by)
        x2, y2 = min(ax + aw, bx + bw), min(ay + ah, by + bh)
        if x2 <= x1 or y2 <= y1:
            return 0.0
        inter = (x2 - x1) * (y2 - y1)
        union = aw * ah + bw * bh - inter
        return inter / union if union > 0 else 0.0

    @staticmethod
    def _center_dist(a, b) -> float:
        import math as _m
        acx, acy = a[0] + a[2] / 2.0, a[1] + a[3] / 2.0
        bcx, bcy = b[0] + b[2] / 2.0, b[1] + b[3] / 2.0
        return _m.hypot(acx - bcx, acy - bcy)

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

        # ── 先用长宽比区分正方体 / 长方体（二者顶点区间相同，必须靠这个区分）──
        ratio = (max(w, h) / min(w, h)) if min(w, h) > 0 else 1.0
        cube_ok = (SHAPE_ASPECT_RATIOS[TargetShape.CUBE][0] <= ratio
                   <= SHAPE_ASPECT_RATIOS[TargetShape.CUBE][1])
        cuboid_ok = (SHAPE_ASPECT_RATIOS[TargetShape.CUBOID][0] <= ratio
                     <= SHAPE_ASPECT_RATIOS[TargetShape.CUBOID][1])
        if cube_ok != cuboid_ok:
            guess = TargetShape.CUBE if cube_ok else TargetShape.CUBOID
            v_min, v_max = SHAPE_VERTEX_RANGES[guess]
            if v_min <= vertices <= v_max:
                return guess

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

    def _estimate_orientation(self, contour) -> float:
        """用轮廓 PCA 估算目标朝向 (rad)"""
        import cv2
        import numpy as np
        if len(contour) < 5:
            return 0.0
        try:
            data_pts = contour.reshape(-1, 2).astype(np.float32)
            mean, eigenvectors = cv2.PCACompute(data_pts, mean=None)
            angle = __import__("math").atan2(eigenvectors[0][1], eigenvectors[0][0])
            return angle
        except Exception:
            return 0.0

    def _calc_confidence(self, area: float, vertices: int) -> float:
        """计算检测置信度（启发式）"""
        # 面积越大越可信，顶点数在合理范围内越可信
        area_score = min(1.0, area / 2000.0)  # 2000 像素² → 满分
        vertex_score = 0.5 if 3 <= vertices <= 20 else 0.2
        return (area_score + vertex_score) / 2.0

    def estimate_position(self, detection: Detection,
                          camera_height_mm: float = 210,
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

    def estimate_ground_position(self,
                                 center_x_px: float,
                                 bottom_y_px: float,
                                 image_size: Optional[Tuple[int, int]] = None,
                                 ) -> Optional[Tuple[float, float]]:
        """
        底边 + 相机倾角的**地平面测距**（目标贴地时的正解）。

        为什么不用面积法：目标形状不同（正方体/三棱锥/长方体/圆柱）在同一距离下
        成像面积差异巨大，且受 HSV 阈值/光照/遮挡影响 → 距离不可用。
        而"检测框底边"是目标与地面的接触线，配合已知相机高度与倾角，
        由地平面几何即可解出距离，**与目标尺寸/形状无关**。

        公式::

            f  = W / (2·tan(FOV/2))                  焦距(px)
            α  = TILT + atan((y_bottom − H/2) / f)   视线俯角
            d  = HEIGHT / tan(α)                     地面距离(mm)
            x  = d · (cx − W/2) / f                  横向偏移(mm)

        Returns:
            (x_mm, y_mm) 机器人坐标系（+y 前方 / +x 右方）；无法解算返回 None。
        """
        img_w, img_h = image_size or self._image_size
        if img_w <= 0 or img_h <= 0:
            return None
        f = img_w / (2.0 * math.tan(math.radians(self._camera_fov_deg) / 2.0))
        if f <= 0:
            return None
        alpha = (math.radians(self._camera_tilt_deg)
                 + math.atan((bottom_y_px - img_h / 2.0) / f))
        # 视线接近水平 → 距离发散（tan(α)→0），该帧拒绝（避免算出无穷远）
        if alpha <= math.radians(1.0):
            return None
        d = self._camera_height_mm / math.tan(alpha)
        x_off = (center_x_px - img_w / 2.0) / f * d
        return (x_off, d)


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
                    orientation=self._rng.uniform(-3.14, 3.14),
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
                orientation=det.orientation,
            ))
        return result

    def estimate_position(self, detection: Detection,
                          camera_height_mm: float = 210,
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
        # 相对机器人的坐标：目标在机器人前方一定范围（模拟摄像头视野），
        # 避免映射到全场导致目标落在禁区/场地外。
        rel_x = (cx / img_w - 0.5) * 1000.0            # 左右 ±500mm
        rel_y = 200.0 + (1.0 - cy / img_h) * 800.0     # 前方 200~1000mm

        return (rel_x, rel_y)
