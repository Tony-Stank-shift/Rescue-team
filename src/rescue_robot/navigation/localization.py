"""
localization.py —— 机器人定位

提供机器人在场地中的实时位姿估计。

特性：
  - 里程计 + IMU 融合（简化 EKF）
  - 重定位支持（强制分离后回到出发区）
  - 定位置信度估计
  - Mock 模式用于开发测试
"""

import logging
import math
import time
from dataclasses import dataclass, field
from typing import Optional, Tuple

logger = logging.getLogger("localization")


# ============================================================
# 位姿
# ============================================================

@dataclass
class RobotPose:
    """机器人位姿"""
    x: float = 0.0             # X 坐标 (mm)
    y: float = 0.0             # Y 坐标 (mm)
    theta: float = 0.0         # 朝向 (rad)，0 = 正前方 (+Y)
    confidence: float = 1.0    # 定位置信度 [0, 1]
    timestamp: float = 0.0

    @property
    def position(self) -> Tuple[float, float]:
        return (self.x, self.y)

    @property
    def heading_deg(self) -> float:
        return math.degrees(self.theta) % 360

    def distance_to(self, other: "RobotPose") -> float:
        return math.sqrt((self.x - other.x) ** 2 + (self.y - other.y) ** 2)

    def copy(self) -> "RobotPose":
        return RobotPose(
            x=self.x, y=self.y, theta=self.theta,
            confidence=self.confidence, timestamp=self.timestamp,
        )


# ============================================================
# 抽象定位器
# ============================================================

class AbstractLocalizer:
    """定位器抽象基类"""

    def update(self,
               linear_velocity: float = 0.0,
               angular_velocity: float = 0.0,
               dt: float = 0.02) -> RobotPose:
        """更新位姿估计"""
        raise NotImplementedError

    def reset_pose(self, x: float = 0.0, y: float = 0.0,
                   theta: float = 0.0, confidence: float = 1.0) -> None:
        """重置位姿（强制分离后使用）"""
        raise NotImplementedError

    @property
    def pose(self) -> RobotPose:
        raise NotImplementedError


# ============================================================
# Mock 定位器
# ============================================================

class MockLocalizer(AbstractLocalizer):
    """
    Mock 定位器：基于速度指令模拟位置。

    假设无滑移、无噪声，积分速度得到位置。
    """

    def __init__(self,
                 start_x: float = 1500.0,
                 start_y: float = 300.0,
                 start_theta: float = math.pi / 2):
        """
        Args:
            start_x, start_y: 初始位置（默认场地中央偏下）
            start_theta: 初始朝向（默认朝正前方 +Y）
        """
        self._pose = RobotPose(
            x=start_x, y=start_y, theta=start_theta,
            confidence=1.0, timestamp=time.time(),
        )
        self._odom_distance = 0.0  # 累计里程
        logger.info(f"MockLocalizer 初始化: pos=({start_x:.0f}, {start_y:.0f}), "
                     f"heading={math.degrees(start_theta):.0f}°")

    @property
    def pose(self) -> RobotPose:
        return self._pose.copy()

    def update(self,
               linear_velocity: float = 0.0,
               angular_velocity: float = 0.0,
               dt: float = 0.02) -> RobotPose:
        """运动模型更新（差速驱动）"""
        # 简化运动模型
        if abs(angular_velocity) < 0.001:
            # 直线运动
            ds = linear_velocity * dt
            self._pose.x += ds * math.cos(self._pose.theta)
            self._pose.y += ds * math.sin(self._pose.theta)
        else:
            # 圆弧运动
            radius = linear_velocity / angular_velocity
            dtheta = angular_velocity * dt
            self._pose.x += radius * (math.sin(self._pose.theta + dtheta) -
                                      math.sin(self._pose.theta))
            self._pose.y -= radius * (math.cos(self._pose.theta + dtheta) -
                                      math.cos(self._pose.theta))
            self._pose.theta += dtheta

        # 归一化角度
        self._pose.theta = self._normalize_angle(self._pose.theta)
        self._pose.timestamp = time.time()

        # 累计里程
        self._odom_distance += abs(linear_velocity * dt)

        return self.pose

    def reset_pose(self, x: float = 0.0, y: float = 0.0,
                   theta: float = 0.0, confidence: float = 1.0) -> None:
        """重置位姿（强制分离后）"""
        old_pos = self._pose.position
        self._pose.x = x
        self._pose.y = y
        self._pose.theta = theta
        self._pose.confidence = confidence
        self._pose.timestamp = time.time()
        logger.info(f"位姿重置: ({old_pos[0]:.0f}, {old_pos[1]:.0f}) → "
                     f"({x:.0f}, {y:.0f}), heading={math.degrees(theta):.0f}°")

    @staticmethod
    def _normalize_angle(theta: float) -> float:
        """归一化角度到 [-π, π]"""
        while theta > math.pi:
            theta -= 2 * math.pi
        while theta < -math.pi:
            theta += 2 * math.pi
        return theta

    @property
    def odom_distance(self) -> float:
        """累计里程 (mm)"""
        return self._odom_distance


# ============================================================
# 真实定位器（里程计 + IMU）
# ============================================================

class OdometryLocalizer(AbstractLocalizer):
    """
    真实定位器：编码器里程计 + IMU 姿态融合。

    使用互补滤波（简化版 EKF）：
    - 位置：主要由里程计积分
    - 朝向：IMU 陀螺仪短期 + 里程计长期校正
    - 置信度：根据里程计累积误差估算
    """

    # 里程计误差模型参数
    ODOM_ERROR_PER_METER = 0.02    # 2% 线性误差
    ODOM_ERROR_PER_RADIAN = 0.01   # 1% 角度误差
    CONFIDENCE_DECAY_RATE = 0.001  # 每米置信度衰减

    def __init__(self,
                 start_x: float = 1500.0,
                 start_y: float = 300.0,
                 start_theta: float = math.pi / 2):
        self._pose = RobotPose(
            x=start_x, y=start_y, theta=start_theta,
            confidence=1.0, timestamp=time.time(),
        )

        # IMU 偏置
        self._gyro_bias = 0.0

        # 滤波系数
        self._alpha_heading = 0.95   # IMU 权重大（短期稳定）

        # 里程计累计
        self._odom_distance = 0.0

        logger.info(f"OdometryLocalizer 初始化: pos=({start_x:.0f}, {start_y:.0f})")

    @property
    def pose(self) -> RobotPose:
        return self._pose.copy()

    def update(self,
               linear_velocity: float = 0.0,
               angular_velocity: float = 0.0,
               dt: float = 0.02,
               imu_gyro_z: Optional[float] = None,
               odom_left: Optional[float] = None,
               odom_right: Optional[float] = None) -> RobotPose:
        """
        更新位姿。

        Args:
            linear_velocity: 线速度 (mm/s)
            angular_velocity: 角速度 (rad/s)
            dt: 时间步长 (s)
            imu_gyro_z: IMU 陀螺仪 Z 轴读数 (rad/s)，None 则使用 angular_velocity
            odom_left/odom_right: 编码器读数
        """
        # 里程计增量
        ds = linear_velocity * dt

        # 角度融合：优先 IMU，fallback 里程计
        if imu_gyro_z is not None:
            gyro_rate = imu_gyro_z - self._gyro_bias
            dtheta = gyro_rate * dt
            # 互补滤波：IMU短期 + 里程计长期
            dtheta = (self._alpha_heading * dtheta +
                      (1 - self._alpha_heading) * angular_velocity * dt)
        else:
            dtheta = angular_velocity * dt

        # 更新位姿（同 Mock 运动模型）
        if abs(dtheta) < 0.0001:
            self._pose.x += ds * math.cos(self._pose.theta)
            self._pose.y += ds * math.sin(self._pose.theta)
        else:
            radius = ds / dtheta if abs(dtheta) > 0 else float('inf')
            self._pose.x += radius * (math.sin(self._pose.theta + dtheta) -
                                      math.sin(self._pose.theta))
            self._pose.y -= radius * (math.cos(self._pose.theta + dtheta) -
                                      math.cos(self._pose.theta))
            self._pose.theta += dtheta

        self._pose.theta = self._normalize_angle(self._pose.theta)
        self._pose.timestamp = time.time()

        # 累积里程 + 置信度衰减
        self._odom_distance += abs(ds)
        error = self._odom_distance * self.ODOM_ERROR_PER_METER
        self._pose.confidence = max(0.1, 1.0 - error / 1000.0)

        return self.pose

    def reset_pose(self, x: float = 0.0, y: float = 0.0,
                   theta: float = 0.0, confidence: float = 1.0) -> None:
        self._pose.x = x
        self._pose.y = y
        self._pose.theta = theta
        self._pose.confidence = confidence
        self._pose.timestamp = time.time()
        self._odom_distance = 0.0  # 重置累积误差
        logger.info(f"位姿重置: ({x:.0f}, {y:.0f}), heading={math.degrees(theta):.0f}°")

    def calibrate_gyro(self, num_samples: int = 100) -> None:
        """
        校准 IMU 陀螺仪偏置。

        需在静止状态下调用。
        """
        logger.info(f"陀螺仪校准中... ({num_samples} 采样)")
        # 实际需要采集 IMU 数据，这里为占位
        self._gyro_bias = 0.0
        logger.info(f"陀螺仪校准完成: bias={self._gyro_bias:.6f} rad/s")

    @staticmethod
    def _normalize_angle(theta: float) -> float:
        while theta > math.pi:
            theta -= 2 * math.pi
        while theta < -math.pi:
            theta += 2 * math.pi
        return theta

    @property
    def odom_distance(self) -> float:
        return self._odom_distance
