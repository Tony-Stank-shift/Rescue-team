"""
sensor_fusion.py —— 多传感器融合框架

融合多种传感器数据，提供统一的观测：
  - 视觉（摄像头）→ 目标检测
  - IMU（加速度计 + 陀螺仪）→ 姿态/加速度
  - 里程计（编码器）→ 速度/位移
  - 温度传感器 → 辅助目标识别
  - 振动传感器 → 碰撞/越障检测

当前状态：框架已搭建，视觉为主要传感器，其他传感器接口预留。
"""

import logging
import math
import time
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

logger = logging.getLogger("sensor_fusion")


# ============================================================
# 传感器读数
# ============================================================

@dataclass
class IMUReading:
    """IMU 读数"""
    accel_x: float = 0.0     # 加速度 X (m/s²)
    accel_y: float = 0.0     # 加速度 Y
    accel_z: float = 0.0     # 加速度 Z
    gyro_x: float = 0.0      # 角速度 X (rad/s)
    gyro_y: float = 0.0      # 角速度 Y
    gyro_z: float = 0.0      # 角速度 Z
    timestamp: float = 0.0


@dataclass
class OdomReading:
    """里程计读数"""
    left_wheel_mm: float = 0.0     # 左轮位移
    right_wheel_mm: float = 0.0    # 右轮位移
    velocity_linear: float = 0.0   # 线速度 (mm/s)
    velocity_angular: float = 0.0  # 角速度 (rad/s)
    timestamp: float = 0.0


@dataclass
class TempReading:
    """温度传感器读数"""
    ambient_temp_c: float = 25.0    # 环境温度
    target_temp_c: float = 25.0     # 目标温度（红外）
    timestamp: float = 0.0


@dataclass
class VibrationReading:
    """振动传感器读数"""
    magnitude: float = 0.0       # 振动幅度
    frequency_hz: float = 0.0    # 主频
    is_collision: bool = False   # 是否疑似碰撞
    is_speed_bump: bool = False  # 是否疑似越障
    timestamp: float = 0.0


# ============================================================
# 融合观测
# ============================================================

@dataclass
class FusedObservation:
    """多传感器融合后的统一观测"""
    timestamp: float = 0.0

    # 位姿估计
    position: Tuple[float, float] = (0.0, 0.0)  # (x, y) mm
    heading: float = 0.0                         # 朝向 (rad)
    velocity: Tuple[float, float] = (0.0, 0.0)   # (vx, vy) mm/s

    # 状态
    is_moving: bool = False
    is_colliding: bool = False
    is_crossing_bump: bool = False

    # 传感器原始数据
    imu: Optional[IMUReading] = None
    odom: Optional[OdomReading] = None
    temp: Optional[TempReading] = None
    vibration: Optional[VibrationReading] = None


# ============================================================
# 传感器融合器
# ============================================================

class SensorFusion:
    """
    多传感器融合器。

    当前实现：
    - 视觉 + 里程计：主要位姿估计
    - IMU：加速度/角速度辅助
    - 温度/振动：辅助检测（预留）
    """

    def __init__(self):
        self._last_imu: Optional[IMUReading] = None
        self._last_odom: Optional[OdomReading] = None
        self._last_temp: Optional[TempReading] = None
        self._last_vibration: Optional[VibrationReading] = None

        # 位姿状态
        self._position: Tuple[float, float] = (0.0, 0.0)
        self._heading: float = 0.0
        self._velocity: Tuple[float, float] = (0.0, 0.0)

        # 碰撞检测
        self._collision_threshold = 2.0  # 加速度突增阈值 (m/s²)
        self._last_accel_magnitude = 0.0

        logger.info("SensorFusion 初始化")

    # ---- 更新传感器数据 ----

    def update_imu(self, reading: IMUReading) -> None:
        self._last_imu = reading
        # 碰撞检测：加速度突增
        accel_mag = math.sqrt(
            reading.accel_x ** 2 + reading.accel_y ** 2 + reading.accel_z ** 2
        )
        if abs(accel_mag - self._last_accel_magnitude) > self._collision_threshold:
            logger.debug(f"加速度突增: {self._last_accel_magnitude:.1f} → {accel_mag:.1f}")
        self._last_accel_magnitude = accel_mag

    def update_odom(self, reading: OdomReading) -> None:
        self._last_odom = reading
        # 更新位姿
        ds = reading.velocity_linear * 0.02  # 假设 50Hz，20ms 位移
        self._position = (
            self._position[0] + ds * math.cos(self._heading),
            self._position[1] + ds * math.sin(self._heading),
        )
        self._heading += reading.velocity_angular * 0.02

    def update_temp(self, reading: TempReading) -> None:
        self._last_temp = reading

    def update_vibration(self, reading: VibrationReading) -> None:
        self._last_vibration = reading

    # ---- 融合输出 ----

    def get_fused(self) -> FusedObservation:
        """获取当前融合观测"""
        return FusedObservation(
            timestamp=time.time(),
            position=self._position,
            heading=self._heading,
            velocity=self._velocity,
            is_moving=(abs(self._velocity[0]) + abs(self._velocity[1])) > 10,
            is_colliding=(self._last_vibration.is_collision
                          if self._last_vibration else False),
            is_crossing_bump=(self._last_vibration.is_speed_bump
                              if self._last_vibration else False),
            imu=self._last_imu,
            odom=self._last_odom,
            temp=self._last_temp,
            vibration=self._last_vibration,
        )

    def get_position(self) -> Tuple[float, float]:
        return self._position

    def get_heading(self) -> float:
        return self._heading

    def reset_position(self, position: Tuple[float, float] = (0, 0),
                       heading: float = 0.0) -> None:
        """重置位姿（强制分离回出发区后使用）"""
        self._position = position
        self._heading = heading
        logger.info(f"位姿重置: pos=({position[0]:.0f}, {position[1]:.0f}), heading={heading:.2f}")
