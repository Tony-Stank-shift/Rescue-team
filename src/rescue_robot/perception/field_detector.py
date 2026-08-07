"""
field_detector.py -- 场地元素视觉检测

从摄像头实时识别场地元素（安全区颜色、减速带等）。
当前提供接口 + Mock 实现，真实视觉检测待部署时适配。
"""

import logging
from dataclasses import dataclass
from enum import Enum, auto
from typing import List

logger = logging.getLogger("field_detector")


class DetectedFieldElement(Enum):
    """检测到的场地元素"""
    RED_SAFE_ZONE = auto()
    BLUE_SAFE_ZONE = auto()
    PURPLE_FENCE = auto()
    START_ZONE_MAGENTA = auto()
    SPEED_BUMP = auto()
    FIELD_BOUNDARY = auto()


@dataclass
class FieldDetection:
    """场地元素检测结果"""
    element: DetectedFieldElement
    confidence: float = 1.0
    region_hint: str = ""


class AbstractFieldDetector:
    """场地元素检测器抽象基类"""

    def detect(self, frame) -> List[FieldDetection]:
        raise NotImplementedError


class MockFieldDetector(AbstractFieldDetector):
    """Mock 场地检测器"""

    def detect(self, frame=None) -> List[FieldDetection]:
        return [
            FieldDetection(DetectedFieldElement.RED_SAFE_ZONE, 0.95, "top-left"),
            FieldDetection(DetectedFieldElement.BLUE_SAFE_ZONE, 0.95, "top-right"),
            FieldDetection(DetectedFieldElement.SPEED_BUMP, 0.8, "bottom"),
        ]


class CVFieldDetector(AbstractFieldDetector):
    """基于 HSV 颜色分割的场地元素检测"""

    def __init__(self):
        self._color_map = {
            DetectedFieldElement.RED_SAFE_ZONE: ((0, 100, 100), (10, 255, 255)),
            DetectedFieldElement.BLUE_SAFE_ZONE: ((95, 80, 60), (125, 255, 255)),
            DetectedFieldElement.START_ZONE_MAGENTA: ((140, 50, 50), (170, 255, 255)),
        }

    def detect(self, frame) -> List[FieldDetection]:
        try:
            import cv2
            import numpy as np
        except ImportError:
            return []

        results = []
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        for element, (lower, upper) in self._color_map.items():
            mask = cv2.inRange(hsv, np.array(lower), np.array(upper))
            if cv2.countNonZero(mask) > 500:
                results.append(FieldDetection(element, 0.8))

        return results
