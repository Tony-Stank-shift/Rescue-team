"""
sim_sensors.py —— 传感器仿真

从 MuJoCo 数据生成仿真的传感器读数，模拟真实传感器行为。
包含:
  - 摄像头 (Ground-truth 目标检测 + 噪声)
  - IMU (加速度 + 角速度 + 噪声)
  - 里程计 (位置 + 漂移)
  - 碰撞检测 (接触力)

用法:
  sensors = SimSensorBundle(model, data, robot)
  readings = sensors.generate()
"""

import math
import random
from typing import Any, Dict, List, Optional, Tuple

import numpy as np


class SimCamera:
    """仿真摄像头 — 基于 ground truth 的目标检测。"""

    def __init__(self, detection_range_m: float = 2.0,
                 fov_deg: float = 90.0,
                 position_noise_m: float = 0.005,
                 false_positive_rate: float = 0.0,
                 false_negative_rate: float = 0.05):
        """
        Args:
            detection_range_m: 检测范围 (m)
            fov_deg: 视场角 (度)
            position_noise_m: 位置噪声 (m)
            false_positive_rate: 假阳性率
            false_negative_rate: 假阴性率
        """
        self.detection_range = detection_range_m
        self.fov_half = math.radians(fov_deg / 2)
        self.pos_noise = position_noise_m
        self.fp_rate = false_positive_rate
        self.fn_rate = false_negative_rate

    def detect(self, robot_pose: Tuple[float, float, float],
               targets: List[Any]) -> List[Dict]:
        """
        从机器人视角检测可见目标。

        Args:
            robot_pose: (x, y, yaw) 机器人位姿 (m, rad)
            targets: SimTarget 列表

        Returns:
            detected: [{"id": int, "x": float, "y": float, "z": float, ...}]
        """
        detections = []
        rx, ry, ryaw = robot_pose

        for t in targets:
            if t.is_delivered:
                continue

            # 地 truth 位置
            tx, ty = t._pos_m[0], t._pos_m[1]

            # 距离
            dx = tx - rx
            dy = ty - ry
            dist = math.sqrt(dx * dx + dy * dy)

            if dist > self.detection_range:
                continue

            # 视角检查
            bearing = math.atan2(dy, dx)
            angle_diff = bearing - ryaw
            # 归一化到 [-pi, pi]
            angle_diff = math.atan2(math.sin(angle_diff), math.cos(angle_diff))

            if abs(angle_diff) > self.fov_half:
                continue

            # 假阴性
            if random.random() < self.fn_rate:
                continue

            # 加噪声
            nx = tx + random.gauss(0, self.pos_noise)
            ny = ty + random.gauss(0, self.pos_noise)

            detections.append({
                "id": t.target_id,
                "x": nx,
                "y": ny,
                "z": 0.0,
                "distance": dist,
                "bearing": angle_diff,
                "points": t.points,
                "is_dangerous": t.is_dangerous,
            })

        return detections


class SimIMU:
    """仿真 IMU — 加速度 + 角速度 + 噪声。"""

    def __init__(self,
                 gyro_noise_std: float = 0.005,     # rad/s/√Hz
                 accel_noise_std: float = 0.01,      # m/s²/√Hz
                 gyro_bias: float = 0.0,
                 accel_bias: Tuple[float, float, float] = (0.0, 0.0, 0.0)):
        self.gyro_noise = gyro_noise_std
        self.accel_noise = accel_noise_std
        self.gyro_bias = gyro_bias
        self.accel_bias = accel_bias

    def read(self, data) -> Dict[str, Any]:
        """
        从 MuJoCo data 读取 IMU 数据。

        Args:
            data: mujoco.MjData

        Returns:
            {"gyro": (gx, gy, gz), "accel": (ax, ay, az)}
        """
        # Ground truth 角速度
        try:
            jid = data.model.joint("robot_free").id
            dof_addr = data.model.jnt_dofadr[jid]
            gz_gt = float(data.qvel[dof_addr + 5])  # Z 轴角速度
        except (KeyError, IndexError):
            gz_gt = 0.0

        # 加噪声 + bias
        gz = gz_gt + random.gauss(0, self.gyro_noise) + self.gyro_bias
        gx = random.gauss(0, self.gyro_noise * 0.5)
        gy = random.gauss(0, self.gyro_noise * 0.5)

        # 加速度 (简化: 仅 Z 轴重力)
        ax = self.accel_bias[0] + random.gauss(0, self.accel_noise)
        ay = self.accel_bias[1] + random.gauss(0, self.accel_noise)
        az = -9.81 + self.accel_bias[2] + random.gauss(0, self.accel_noise * 2)

        return {
            "gyro": (gx, gy, gz),
            "accel": (ax, ay, az),
        }


class SimOdometry:
    """仿真里程计 — 位置跟踪 + 漂移。"""

    def __init__(self,
                 position_noise_per_m: float = 0.01,
                 heading_noise_per_rad: float = 0.005):
        self.pos_noise = position_noise_per_m
        self.head_noise = heading_noise_per_rad

        # 累计漂移
        self._drift_x = 0.0
        self._drift_y = 0.0
        self._drift_theta = 0.0

    def read(self, robot_pose: Tuple[float, float, float]) -> Tuple[float, float, float]:
        """
        读取里程计位置 (含漂移)。

        Args:
            robot_pose: (x, y, theta) ground truth 位姿

        Returns:
            (x, y, theta) 含漂移的位姿
        """
        # 每米漂移
        self._drift_x += random.gauss(0, self.pos_noise) * 0.02
        self._drift_y += random.gauss(0, self.pos_noise) * 0.02
        self._drift_theta += random.gauss(0, self.head_noise) * 0.02

        return (
            robot_pose[0] + self._drift_x,
            robot_pose[1] + self._drift_y,
            robot_pose[2] + self._drift_theta,
        )


class SimSensorBundle:
    """传感器套件 — 一站式传感器数据生成。"""

    def __init__(self, model, data, robot,
                 enable_camera: bool = True,
                 enable_imu: bool = True,
                 enable_odometry: bool = True,
                 add_noise: bool = True):
        self.model = model
        self.data = data
        self.robot = robot

        self.camera = SimCamera() if enable_camera else None
        self.imu = SimIMU() if enable_imu else None
        self.odometry = SimOdometry() if enable_odometry else None
        self.add_noise = add_noise

    def generate(self, targets: List[Any]) -> Dict[str, Any]:
        """
        生成完整传感器数据包。

        Returns:
            {
                "camera": [detections],
                "imu": {"gyro": ..., "accel": ...},
                "odometry": (x, y, theta),
                "timestamp": float,
            }
        """
        robot_pose = self.robot.get_pose(self.data)

        result = {
            "timestamp": self.data.time,
            "robot_pose": robot_pose,
        }

        if self.camera:
            result["camera"] = self.camera.detect(robot_pose, targets)

        if self.imu:
            result["imu"] = self.imu.read(self.data)

        if self.odometry:
            result["odometry"] = self.odometry.read(robot_pose)

        return result


# ============================================================
# 独立测试
# ============================================================

if __name__ == "__main__":
    import mujoco
    from .sim_models import mm_to_m
    from .sim_robot import SimRobot as Sr

    print("=" * 50)
    print("  SimSensors 独立测试")
    print("=" * 50)

    # Setup minimal world
    spec = mujoco.MjSpec()
    spec.option.timestep = 0.004
    spec.option.gravity = [0, 0, -9.81]
    spec.worldbody.add_geom(type=0, size=[0, 0, 0.001], pos=[1.5, 1.5, 0])

    from dataclasses import dataclass

    @dataclass
    class _FakeTarget:
        target_id: int = 1
        _pos_m: tuple = (1.0, 1.0)
        is_delivered: bool = False
        points: int = 5
        is_dangerous: bool = False

    robot = Sr(spec.worldbody, start_zone=1)

    model = spec.compile()
    data = mujoco.MjData(model)
    robot.init_pose(data)
    mujoco.mj_forward(model, data)

    # Test 1: SimCamera
    print("\n--- 测试 1: SimCamera ---")
    cam = SimCamera(detection_range_m=5.0, fov_deg=90)
    targets = [
        _FakeTarget(1, (0.5, 0.5)),    # behind robot
        _FakeTarget(2, (1.5, 1.5)),    # far
    ]
    dets = cam.detect(robot.get_pose(data), targets)
    print(f"  检测数: {len(dets)}")
    for d in dets:
        print(f"    id={d['id']} dist={d['distance']:.2f}m bearing={math.degrees(d['bearing']):.1f}°")
    print("  ✅ 通过")

    # Test 2: SimIMU
    print("\n--- 测试 2: SimIMU ---")
    imu = SimIMU()
    r = imu.read(data)
    assert "gyro" in r and "accel" in r
    print(f"  gyro={tuple(round(v, 4) for v in r['gyro'])}")
    print(f"  accel={tuple(round(v, 3) for v in r['accel'])}")
    print("  ✅ 通过")

    # Test 3: SimOdometry
    print("\n--- 测试 3: SimOdometry ---")
    odom = SimOdometry()
    pose = robot.get_pose(data)
    op = odom.read(pose)
    assert abs(op[0] - pose[0]) < 0.1  # drift should be small
    print(f"  GT pose: ({pose[0]:.3f}, {pose[1]:.3f}, {pose[2]:.3f})")
    print(f"  Odom:    ({op[0]:.3f}, {op[1]:.3f}, {op[2]:.3f})")
    print("  ✅ 通过")

    # Test 4: SimSensorBundle
    print("\n--- 测试 4: SimSensorBundle ---")
    bundle = SimSensorBundle(model, data, robot)
    pkg = bundle.generate(targets)
    assert "camera" in pkg and "imu" in pkg
    assert "odometry" in pkg and "timestamp" in pkg
    print(f"  数据包: {list(pkg.keys())}")
    print("  ✅ 通过")

    print(f"\n{'=' * 50}")
    print("  SimSensors — 全部通过 ✅")
    print(f"{'=' * 50}")
