"""
classification.py —— 目标分类器

根据颜色 + 形状将检测结果分类为目标类型。
支持初赛和决赛两套规则，含颜色容差匹配。
"""

import logging
from typing import Dict, List, Optional, Tuple

from .target_types import (
    Detection, DetectedTarget, TargetInfo,
    TargetColor, TargetShape, TargetType,
    CompetitionPhase, get_target_config,
)

logger = logging.getLogger("classification")


class TargetClassifier:
    """
    目标分类器。

    规则：
    - 初赛：(颜色, 形状) → 查 PRELIMINARY_TARGETS 表
    - 决赛：(颜色, 形状) → 查 FINAL_TARGETS 表
    - 未匹配 → 标记为 UNKNOWN（可能是场地元素或误检）
    """

    def __init__(self, phase: CompetitionPhase = CompetitionPhase.PRELIMINARY):
        self._phase = phase
        self._config = get_target_config(phase)
        self._next_id = 0

        logger.info(f"TargetClassifier 初始化: phase={phase.name}, "
                     f"规则数={len(self._config)}")

    @property
    def phase(self) -> CompetitionPhase:
        return self._phase

    @phase.setter
    def phase(self, value: CompetitionPhase) -> None:
        """切换比赛阶段（创新实践环节需要）"""
        self._phase = value
        self._config = get_target_config(value)
        logger.info(f"分类器阶段切换: {value.name}")

    def classify(self, detection: Detection, timestamp: float = 0.0) -> Optional[DetectedTarget]:
        """
        将原始检测结果分类为目标类型。

        Args:
            detection: 原始检测
            timestamp: 时间戳

        Returns:
            DetectedTarget 或 None（无法分类）
        """
        color = detection.color
        shape = detection.shape

        # 查表匹配
        key = (color, shape)
        info = self._config.get(key)

        if info is None:
            # 尝试颜色容差匹配
            info = self._fuzzy_match(color, shape)

        if info is None:
            logger.debug(f"未匹配: color={color.name}, shape={shape.name}")
            return None

        target_id = self._next_id
        self._next_id += 1

        return DetectedTarget(
            id=target_id,
            info=info,
            position=(0.0, 0.0),  # 位置由 detector.estimate_position 填充
            confidence=detection.confidence,
            timestamp=timestamp,
            pixel_position=detection.center_pixel,
        )

    def classify_batch(self, detections: List[Detection],
                       timestamp: float = 0.0) -> List[DetectedTarget]:
        """批量分类"""
        results = []
        for det in detections:
            result = self.classify(det, timestamp)
            if result is not None:
                results.append(result)
        logger.debug(f"分类: {len(detections)} 检测 → {len(results)} 已分类")
        return results

    def _fuzzy_match(self, color: TargetColor,
                     shape: TargetShape) -> Optional[TargetInfo]:
        """
        颜色容差匹配。

        当精确 (color, shape) 不在配置表中时，尝试：
        1. 相同形状 + 相近颜色
        2. 相同颜色 + 相近形状
        """
        # 颜色相似度映射（用于容差匹配）
        color_similarity: Dict[TargetColor, List[TargetColor]] = {
            TargetColor.LIGHT_BLUE: [TargetColor.BLUE, TargetColor.WHITE],
            TargetColor.BLUE: [TargetColor.LIGHT_BLUE],
            TargetColor.ORANGE: [TargetColor.RED, TargetColor.YELLOW],
            TargetColor.RED: [TargetColor.ORANGE],
            TargetColor.BROWN: [TargetColor.BLACK, TargetColor.ORANGE],
            TargetColor.BLACK: [TargetColor.BROWN],
        }

        # 1. 相同形状 + 相近颜色
        similar_colors = color_similarity.get(color, [])
        for sim_color in similar_colors:
            key = (sim_color, shape)
            if key in self._config:
                info = self._config[key]
                logger.info(f"容差匹配 (颜色): {color.name}→{sim_color.name}, "
                            f"→ {info.description}")
                return info

        # 2. 相同颜色 + 任意形状（宽松匹配）
        for (c, s), info in self._config.items():
            if c == color:
                logger.info(f"容差匹配 (形状): {shape.name}→{s.name}, "
                            f"→ {info.description}")
                return info

        return None

    def get_target_info(self, target_type: TargetType) -> List[TargetInfo]:
        """获取指定类型的所有可能 TargetInfo"""
        results = []
        for (color, shape), info in self._config.items():
            if info.type == target_type:
                results.append(info)
        return results

    def get_dangerous_info(self) -> List[TargetInfo]:
        """获取危险目标的信息列表"""
        return self.get_target_info(TargetType.DANGEROUS)
