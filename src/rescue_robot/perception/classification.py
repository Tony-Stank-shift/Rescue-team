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
            pixel_bbox=detection.bbox,   # 保留真实检测框：地平面测距要用其底边
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
        颜色容差匹配（**保守**）。

        规则（改自审计 B5）：
          1. 只做"同形状 + 相近颜色"，且**相近颜色之间不得跨越分值/类型**
             （旧的 ORANGE→RED/YELLOW、BLACK↔BROWN 会命中不同分值的类型）；
          2. 删除旧的"相同颜色 + 任意形状"宽松兜底 —— 它会把
             "蓝色 + 未匹配形状"的场地元素当成浅蓝(危险目标)或反之；
          3. 危险目标（DANGEROUS）**只认精确匹配**：宁可漏检，绝不把
             救援目标判成危险目标（漏运=丢分）或把危险目标判成救援目标（违规）。
        """
        # 相近颜色映射：只保留"相邻色相、同分值语义"的映射
        #   LIGHT_BLUE ↔ BLUE 已删除：两者分属 危险 vs 救援，混判代价最高。
        color_similarity: Dict[TargetColor, List[TargetColor]] = {
            TargetColor.RED: [TargetColor.ORANGE],
            TargetColor.ORANGE: [TargetColor.RED],
            TargetColor.BLACK: [],      # 不再映射到 BROWN（棕色可能是另一种分值）
            TargetColor.BROWN: [],
        }

        for sim_color in color_similarity.get(color, []):
            info = self._config.get((sim_color, shape))
            if info is None:
                continue
            if info.type == TargetType.DANGEROUS:
                continue            # 绝不通过容差判成危险目标
            logger.info(f"容差匹配 (颜色): {color.name}→{sim_color.name}, "
                        f"→ {info.description}")
            return info

        # ── 受限的"同色兜底"（形状被判错时救回来，但不碰危险目标）──
        # 只有满足全部条件才允许：
        #   ① 该颜色在配置表里**只对应一种类型**（无色歧义）；
        #   ② 映射结果不是 DANGEROUS（绝不因形状判错而把东西判成危险目标）；
        #   ③ 该颜色本身不是 LIGHT_BLUE（危险目标只认精确匹配）；
        #   ④ **形状不是 UNKNOWN**（2026-09-18 新增）。
        # 旧的实现是无条件"同色任意形状"兜底 → 会把"蓝色 + 未匹配形状"直接判成
        # 浅蓝危险目标（救援目标永不被搬 = 丢分）。
        #
        # ④ 为什么必须加：兜底的初衷是"形状**判错**了，救回来"。而 UNKNOWN 表示
        #    **根本没判出形状** —— 那就是一团没有物体轮廓的色块。
        #    现场实测：同学的腿 `BLACK/UNKNOWN bbox=[407,289,77,191]` 就是这样一路
        #    兜底成"核心物资"的。没有形状可言的色块，救回来的只可能是幻影目标。
        if shape == TargetShape.UNKNOWN:
            logger.debug(f"放弃同色兜底：形状未识别（color={color.name}）→ "
                         f"无形状可言，不当物资")
            return None
        infos_same_color = [info for (c, _s), info in self._config.items() if c == color]
        if (color != TargetColor.LIGHT_BLUE
                and len(infos_same_color) == 1
                and infos_same_color[0].type != TargetType.DANGEROUS):
            info = infos_same_color[0]
            logger.info(f"容差匹配 (受限同色兜底): {color.name} → {info.description}"
                        f"（该颜色在配置表中唯一且非危险目标）")
            return info

        logger.debug(f"容差匹配失败（宁漏不误）: color={color.name}, shape={shape.name}")
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
