"""
opponent_tracker.py —— 对方机器人感知与跟踪

跟踪对方机器人的位置、速度和轨迹，提供：
  - 实时位置与速度估计
  - 轨迹预测
  - 接触检测（用于 10 秒强制分离计时）
  - 意图预测（对方正在前往哪个目标）
"""

import logging
import math
import time
from dataclasses import dataclass
from typing import List, Optional, Tuple

logger = logging.getLogger("opponent_tracker")


# ============================================================
# 对方状态
# ============================================================

@dataclass
class OpponentState:
    """对方机器人状态"""
    position: Tuple[float, float] = (0.0, 0.0)   # (x, y) mm
    velocity: Tuple[float, float] = (0.0, 0.0)   # (vx, vy) mm/s
    heading: float = 0.0                          # 朝向（弧度）
    last_seen: float = 0.0                        # 最近检测时间
    confidence: float = 0.0                       # 检测置信度
    seen_count: int = 0                           # 被检测到的帧数

    @property
    def speed(self) -> float:
        """移动速度 mm/s"""
        return math.sqrt(self.velocity[0] ** 2 + self.velocity[1] ** 2)

    @property
    def is_stale(self) -> bool:
        """超过 1 秒未被检测到"""
        return (time.time() - self.last_seen) > 1.0

    @property
    def age_s(self) -> float:
        """自上次检测以来的时间"""
        return time.time() - self.last_seen if self.last_seen > 0 else 999.0


# ============================================================
# 对方跟踪器
# ============================================================

class OpponentTracker:
    """
    对方机器人跟踪器。

    功能：
    - 卡尔曼滤波（简化）：指数平滑位置 + 速度估计
    - 轨迹预测：基于当前速度的线性预测
    - 接触检测：基于距离阈值
    - 10 秒接触计时器
    """

    # 对方机器人的外观特征（用于视觉检测）
    # 对方机器人 ≤ 300×300mm 投影面积，高度 ≤ 200mm
    OPPONENT_MAX_SIZE_MM = 300

    # 接触判定距离（两台机器人中心距离 < 此值 → 可能接触）
    CONTACT_DISTANCE_MM = 350.0

    # 跟踪丢失最大时间
    MAX_TRACKING_LOST_S = 3.0

    def __init__(self):
        self._state = OpponentState()
        self._trajectory: List[Tuple[float, float]] = []  # 历史轨迹

        # 接触计时
        self._contact_start_time: Optional[float] = None
        self._in_contact = False

        # 速度平滑系数
        self._alpha = 0.3

        logger.info("OpponentTracker 初始化")

    # ---- 属性 ----

    @property
    def state(self) -> OpponentState:
        return self._state

    @property
    def in_contact(self) -> bool:
        return self._in_contact

    @property
    def contact_duration_s(self) -> float:
        """接触持续时间（秒）"""
        if self._contact_start_time is None:
            return 0.0
        return time.time() - self._contact_start_time

    @property
    def contact_warning(self) -> bool:
        """是否接近 10 秒强制分离阈值"""
        return self.contact_duration_s > 7.0  # 提前 3 秒预警

    # ---- 更新 ----

    def update(self, opponent_position: Optional[Tuple[float, float]],
               robot_position: Tuple[float, float],
               timestamp: Optional[float] = None) -> None:
        """
        更新对方机器人状态。

        Args:
            opponent_position: 检测到的对方位置（None = 本帧未检测到）
            robot_position: 本队机器人当前位置
            timestamp: 时间戳
        """
        if timestamp is None:
            timestamp = time.time()

        if opponent_position is not None:
            # 有检测 → 更新状态
            dt = timestamp - self._state.last_seen if self._state.last_seen > 0 else 0.05

            if dt > 0 and self._state.last_seen > 0:
                # 速度估计
                vx = (opponent_position[0] - self._state.position[0]) / dt
                vy = (opponent_position[1] - self._state.position[1]) / dt
                self._state.velocity = (
                    self._alpha * vx + (1 - self._alpha) * self._state.velocity[0],
                    self._alpha * vy + (1 - self._alpha) * self._state.velocity[1],
                )

            self._state.position = opponent_position
            self._state.last_seen = timestamp
            self._state.seen_count += 1
            self._state.confidence = min(1.0, self._state.confidence + 0.1)

            # 记录轨迹
            self._trajectory.append(opponent_position)
            if len(self._trajectory) > 100:
                self._trajectory = self._trajectory[-100:]

        else:
            # 无检测 → 降低置信度
            self._state.confidence = max(0.0, self._state.confidence - 0.05)

        # 接触检测
        self._check_contact(robot_position)

    def _check_contact(self, robot_position: Tuple[float, float]) -> None:
        """检测是否与对方发生物理接触"""
        if self._state.is_stale:
            self._in_contact = False
            self._contact_start_time = None
            return

        dist = math.sqrt(
            (robot_position[0] - self._state.position[0]) ** 2 +
            (robot_position[1] - self._state.position[1]) ** 2
        )

        was_in_contact = self._in_contact
        self._in_contact = dist < self.CONTACT_DISTANCE_MM

        if self._in_contact and not was_in_contact:
            # 开始接触
            self._contact_start_time = time.time()
            logger.info(f"⚡ 检测到接触！距离={dist:.0f}mm")
        elif not self._in_contact and was_in_contact:
            # 接触结束
            duration = (time.time() - self._contact_start_time) if self._contact_start_time else 0
            logger.info(f"接触结束，持续 {duration:.1f}s")
            self._contact_start_time = None

    # ---- 预测 ----

    def predict_position(self, ahead_s: float = 1.0) -> Optional[Tuple[float, float]]:
        """
        预测对方 future 位置（线性预测）。

        Args:
            ahead_s: 预测多少秒后

        Returns:
            预测位置或 None（对手状态过时）
        """
        if self._state.is_stale:
            return None

        px = self._state.position[0] + self._state.velocity[0] * ahead_s
        py = self._state.position[1] + self._state.velocity[1] * ahead_s
        return (px, py)

    def predict_trajectory(self, steps: int = 5,
                           step_s: float = 0.2) -> List[Tuple[float, float]]:
        """预测对方未来轨迹点"""
        if self._state.is_stale:
            return []

        points = []
        px, py = self._state.position
        vx, vy = self._state.velocity

        for i in range(1, steps + 1):
            t = i * step_s
            points.append((px + vx * t, py + vy * t))

        return points

    def is_moving_toward(self, target_pos: Tuple[float, float]) -> bool:
        """判断对方是否朝指定目标移动"""
        if self._state.speed < 50:  # 几乎静止
            return False

        tx, ty = target_pos
        dx = tx - self._state.position[0]
        dy = ty - self._state.position[1]
        angle_to_target = math.atan2(dy, dx)

        # 速度方向
        vx, vy = self._state.velocity
        if abs(vx) < 1 and abs(vy) < 1:
            return False
        velocity_angle = math.atan2(vy, vx)

        # 角度差 < 45° 认为朝目标移动
        angle_diff = abs(angle_to_target - velocity_angle)
        angle_diff = min(angle_diff, 2 * math.pi - angle_diff)
        return angle_diff < math.pi / 4

    # ---- 查询 ----

    def get_trajectory(self) -> List[Tuple[float, float]]:
        """获取对方历史轨迹"""
        return list(self._trajectory)

    def get_stats(self) -> dict:
        """获取跟踪统计"""
        return {
            "position": self._state.position,
            "speed": self._state.speed,
            "heading": self._state.heading,
            "confidence": self._state.confidence,
            "in_contact": self._in_contact,
            "contact_duration_s": self.contact_duration_s,
            "contact_warning": self.contact_warning,
        }
