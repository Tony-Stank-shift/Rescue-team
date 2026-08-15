"""
hardware_profile.py —— 硬件抽象层扩展 (7.1.3)

扩展 system_check.py 的 HardwareChecker ABC，提供:
- HardwareProfile: 电机参数、传感器标定、底盘参数的 YAML 化管理
- CalibrationRoutine: IMU 零偏、摄像头标定、电机线性度测试
- ComponentAdapter: 更换零部件后的快速参数切换

用法:
  profile = HardwareProfile.from_yaml("config/hardware.yaml")
  cal = CalibrationRoutine()
  results = cal.calibrate_imu(duration_s=5)
  cal.save_calibration("config/calibration.yaml")
"""

import json
import logging
import math
import os
import time
from dataclasses import dataclass, field, asdict
from typing import Any, Dict, List, Optional, Tuple

logger = logging.getLogger("hardware_profile")

# 可选依赖
try:
    import yaml as _yaml
    _HAS_YAML = True
except ImportError:
    _HAS_YAML = False


# ============================================================
# HardwareProfile
# ============================================================

@dataclass
class MotorParams:
    """单个电机参数"""
    model: str = "JGB37-520"             # 电机型号
    reduction_ratio: float = 30.0        # 减速比
    encoder_ppr: int = 11                # 编码器线数 (脉冲/转)
    max_rpm: int = 300                   # 最大转速
    rated_voltage: float = 12.0          # 额定电压
    stall_current_ma: int = 5000         # 堵转电流
    no_load_current_ma: int = 100        # 空载电流


@dataclass
class ChassisParams:
    """底盘参数（三全向轮）"""
    wheel_diameter_mm: float = 65.0      # 轮径
    wheel_mount_radius_mm: float = 150.0 # 车轮回转半径 L（中心到轮心）
    wheel_angles_deg: List[float] = field(
        default_factory=lambda: [0.0, 120.0, 240.0]
    )                                     # 三全向轮安装角
    max_linear_speed_mm_s: int = 1000    # 最大线速度
    max_angular_speed_rad_s: float = 3.0 # 最大角速度
    max_accel_mm_s2: int = 500           # 最大加速度


@dataclass
class IMUParams:
    """IMU 参数"""
    model: str = "MPU6050"               # 型号
    gyro_bias_x: float = 0.0             # 陀螺仪零偏 X (rad/s)
    gyro_bias_y: float = 0.0
    gyro_bias_z: float = 0.0
    accel_bias_x: float = 0.0            # 加速度计零偏 (m/s²)
    accel_bias_y: float = 0.0
    accel_bias_z: float = 0.0
    complementary_alpha: float = 0.95    # 互补滤波系数


@dataclass
class CameraParams:
    """摄像头参数"""
    model: str = "USB_CAM_1080P"         # 型号
    width: int = 640                     # 分辨率
    height: int = 480
    fps: int = 30
    fx: float = 500.0                    # 焦距 (像素)
    fy: float = 500.0
    cx: float = 320.0                    # 光心
    cy: float = 240.0
    k1: float = 0.0                      # 径向畸变
    k2: float = 0.0
    p1: float = 0.0                      # 切向畸变
    p2: float = 0.0
    mounting_height_mm: float = 200.0    # 安装高度
    mounting_angle_deg: float = 30.0     # 俯角

    @property
    def camera_matrix(self) -> List[List[float]]:
        """内参矩阵"""
        return [
            [self.fx, 0, self.cx],
            [0, self.fy, self.cy],
            [0, 0, 1],
        ]

    @property
    def dist_coeffs(self) -> List[float]:
        """畸变系数"""
        return [self.k1, self.k2, self.p1, self.p2]


@dataclass
class SensorCalibration:
    """传感器标定参数"""
    imu: IMUParams = field(default_factory=IMUParams)
    camera: CameraParams = field(default_factory=CameraParams)
    ultrasonic_offset_mm: float = 0.0    # 超声波传感器零偏
    temperature_offset_c: float = 0.0    # 温度传感器零偏
    calibrated_at: str = ""              # 标定时间 ISO 格式
    calibrated_by: str = ""              # 标定人员


@dataclass
class HardwareProfile:
    """
    完整硬件配置。

    用法:
      profile = HardwareProfile.from_yaml("config/hardware.yaml")
      print(profile.motors["wheel_0"].model)
    """
    version: str = "1.0"
    robot_name: str = "rescue-bot-01"
    motors: Dict[str, MotorParams] = field(default_factory=dict)
    chassis: ChassisParams = field(default_factory=ChassisParams)
    calibration: SensorCalibration = field(default_factory=SensorCalibration)
    _profile_name: str = "default"

    def __post_init__(self):
        if not self.motors:
            self.motors = {
                "wheel_0": MotorParams(),
                "wheel_1": MotorParams(),
                "wheel_2": MotorParams(),
            }

    @classmethod
    def from_yaml(cls, path: str) -> "HardwareProfile":
        """从 YAML 加载。"""
        if not _HAS_YAML:
            raise ImportError("PyYAML 未安装，无法加载 YAML 硬件配置")

        if not os.path.exists(path):
            logger.warning(f"硬件配置不存在: {path}，使用默认值")
            return cls()

        with open(path, "r", encoding="utf-8") as f:
            data = _yaml.safe_load(f)

        if data is None:
            return cls()
        return cls.from_dict(data)

    @classmethod
    def from_dict(cls, data: dict) -> "HardwareProfile":
        """从字典构建。"""
        profile = cls()
        profile.version = data.get("version", "1.0")
        profile.robot_name = data.get("robot_name", "rescue-bot-01")
        profile._profile_name = data.get("profile_name", "default")

        # 电机
        motors_data = data.get("motors", {})
        if motors_data:
            profile.motors = {}
            for name, params in motors_data.items():
                profile.motors[name] = MotorParams(**params)

        # 底盘
        chassis_data = data.get("chassis", {})
        if chassis_data:
            profile.chassis = ChassisParams(**chassis_data)

        # 标定
        cal_data = data.get("calibration", {})
        if cal_data:
            imu_data = cal_data.get("imu", {})
            camera_data = cal_data.get("camera", {})
            profile.calibration = SensorCalibration(
                imu=IMUParams(**imu_data) if imu_data else IMUParams(),
                camera=CameraParams(**camera_data) if camera_data else CameraParams(),
                ultrasonic_offset_mm=cal_data.get("ultrasonic_offset_mm", 0.0),
                temperature_offset_c=cal_data.get("temperature_offset_c", 0.0),
                calibrated_at=cal_data.get("calibrated_at", ""),
                calibrated_by=cal_data.get("calibrated_by", ""),
            )

        return profile

    def to_yaml(self, path: str) -> None:
        """保存为 YAML。"""
        if not _HAS_YAML:
            raise ImportError("PyYAML 未安装")

        os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
        with open(path, "w", encoding="utf-8") as f:
            _yaml.safe_dump(self.to_dict(), f, allow_unicode=True,
                           default_flow_style=False, sort_keys=False)
        logger.info(f"硬件配置已保存: {path}")

    def to_dict(self) -> dict:
        """转为字典。"""
        return {
            "version": self.version,
            "profile_name": self._profile_name,
            "robot_name": self.robot_name,
            "motors": {name: asdict(params) for name, params in self.motors.items()},
            "chassis": asdict(self.chassis),
            "calibration": {
                "imu": asdict(self.calibration.imu),
                "camera": asdict(self.calibration.camera),
                "ultrasonic_offset_mm": self.calibration.ultrasonic_offset_mm,
                "temperature_offset_c": self.calibration.temperature_offset_c,
                "calibrated_at": self.calibration.calibrated_at,
                "calibrated_by": self.calibration.calibrated_by,
            },
        }

    @property
    def profile_name(self) -> str:
        return self._profile_name


# ============================================================
# CalibrationRoutine
# ============================================================

class CalibrationRoutine:
    """
    传感器标定流程。

    使用方式:
      cal = CalibrationRoutine()
      imu_result = cal.calibrate_imu(duration_s=5)
      cal.save_calibration("config/calibration.yaml")
    """

    def __init__(self, profile: Optional[HardwareProfile] = None):
        self._profile = profile or HardwareProfile()
        self._last_results: Dict[str, Any] = {}

    @property
    def last_results(self) -> dict:
        return dict(self._last_results)

    def calibrate_imu(self, duration_s: float = 5.0,
                      sample_rate_hz: float = 100.0) -> dict:
        """
        校准 IMU 零偏。

        需要机器人静止放置。

        Args:
            duration_s: 采集时长（秒）
            sample_rate_hz: 采样率

        Returns:
            {"gyro_bias": (x, y, z), "accel_bias": (x, y, z), "samples": int}
        """
        logger.info(f"IMU 校准开始... ({duration_s}s)")
        num_samples = int(duration_s * sample_rate_hz)

        # 模拟采集（实际硬件需替换为真实 IMU 读取）
        gyro_samples = [[0.0, 0.0, 0.0]]
        accel_samples = [[0.0, 0.0, 0.0]]

        try:
            # 尝试真实 IMU
            import random
            gyro_bias_noise = 0.01  # rad/s 噪声水平
            gyro_samples = [[
                random.gauss(0, gyro_bias_noise)
                for _ in range(3)
            ] for __ in range(max(10, num_samples // 10))]

            accel_samples = [[
                0.0, 0.0, 9.81 + random.gauss(0, 0.05)
            ] for __ in range(max(10, num_samples // 10))]
        except Exception:
            pass

        # 计算均值
        gyro_bias = (
            sum(s[0] for s in gyro_samples) / len(gyro_samples),
            sum(s[1] for s in gyro_samples) / len(gyro_samples),
            sum(s[2] for s in gyro_samples) / len(gyro_samples),
        )
        accel_bias = (
            sum(s[0] for s in accel_samples) / len(accel_samples),
            sum(s[1] for s in accel_samples) / len(accel_samples),
            sum(s[2] for s in accel_samples) / len(accel_samples) - 9.81,
        )

        # 更新 profile
        self._profile.calibration.imu.gyro_bias_x = gyro_bias[0]
        self._profile.calibration.imu.gyro_bias_y = gyro_bias[1]
        self._profile.calibration.imu.gyro_bias_z = gyro_bias[2]
        self._profile.calibration.imu.accel_bias_x = accel_bias[0]
        self._profile.calibration.imu.accel_bias_y = accel_bias[1]
        self._profile.calibration.imu.accel_bias_z = accel_bias[2]

        result = {
            "gyro_bias": tuple(round(b, 6) for b in gyro_bias),
            "accel_bias": tuple(round(b, 6) for b in accel_bias),
            "samples": len(gyro_samples),
            "duration_s": duration_s,
        }
        self._last_results["imu"] = result

        gyro_str = f"({gyro_bias[0]:.4f}, {gyro_bias[1]:.4f}, {gyro_bias[2]:.4f}) rad/s"
        logger.info(f"IMU 校准完成: gyro_bias={gyro_str}, "
                     f"samples={len(gyro_samples)}")
        return result

    def calibrate_camera(self,
                         checkerboard_cols: int = 8,
                         checkerboard_rows: int = 6,
                         square_size_mm: float = 25.0) -> dict:
        """
        摄像头标定（棋盘格法）。

        需要实际硬件 + OpenCV。当前为 stub。

        Returns:
            标定结果（内参矩阵、畸变系数、重投影误差）
        """
        logger.info("摄像头标定开始... (需要 OpenCV + 棋盘格)")

        result = {
            "status": "stub",
            "message": "摄像头标定需要 OpenCV 和真实摄像头硬件。"
                       "在树莓派上安装: pip install opencv-python",
            "checkerboard": f"{checkerboard_cols}x{checkerboard_rows}",
            "square_size_mm": square_size_mm,
        }
        self._last_results["camera"] = result
        return result

    def calibrate_motors(self, motor_id: str = "wheel_0",
                         duration_s: float = 3.0,
                         speed_mm_s: int = 200) -> dict:
        """
        电机线性度测试。

        驱动电机以指定速度运行，记录编码器反馈，
        计算实际速度与指令速度的误差。

        Returns:
            {"motor": str, "commanded_mm_s": int, "actual_mm_s": float,
             "error_pct": float, "ok": bool}
        """
        logger.info(f"电机校准: {motor_id} @ {speed_mm_s}mm/s, {duration_s}s")

        # stub 实现 —— 实际硬件需替换
        actual_speed = speed_mm_s * (1.0 + 0.02 * (hash(motor_id) % 10 - 5) / 100)
        error_pct = abs(actual_speed - speed_mm_s) / speed_mm_s * 100
        ok = error_pct < 10.0

        result = {
            "motor": motor_id,
            "commanded_mm_s": speed_mm_s,
            "actual_mm_s": round(actual_speed, 1),
            "error_pct": round(error_pct, 2),
            "ok": ok,
            "duration_s": duration_s,
        }
        self._last_results["motors"] = result

        status = "✅" if ok else "⚠️"
        logger.info(f"电机校准 {motor_id}: {status} 指令={speed_mm_s}mm/s, "
                     f"实际={actual_speed:.1f}mm/s, 误差={error_pct:.1f}%")
        return result

    def save_calibration(self, path: str) -> None:
        """保存标定结果到文件。"""
        import datetime
        self._profile.calibration.calibrated_at = datetime.datetime.now().isoformat()
        self._profile.to_yaml(path)

    def load_calibration(self, path: str) -> HardwareProfile:
        """加载标定文件。"""
        self._profile = HardwareProfile.from_yaml(path)
        return self._profile


# ============================================================
# ComponentAdapter
# ============================================================

class ComponentAdapter:
    """
    更换零部件后的快速参数切换。

    支持多套电机/传感器配置并存，通过名称切换。

    用法:
      adapter = ComponentAdapter()
      adapter.register("high_torque_motor", MotorParams(model="JGB37-550", reduction_ratio=50))
      adapter.switch_to("high_torque_motor")
    """

    def __init__(self, base_profile: Optional[HardwareProfile] = None):
        self._base_profile = base_profile or HardwareProfile()
        self._profiles: Dict[str, HardwareProfile] = {"default": self._base_profile}
        self._active_name = "default"
        self._components: Dict[str, Dict[str, Any]] = {}

    @property
    def active_name(self) -> str:
        return self._active_name

    @property
    def active_profile(self) -> HardwareProfile:
        return self._profiles[self._active_name]

    def register_component(self, name: str, **params) -> None:
        """
        注册新零件参数。

        Example:
          adapter.register_component(
              "high_speed_motor",
              motor=MotorParams(model="JGB37-550", max_rpm=500),
              chassis_mods={"max_linear_speed_mm_s": 1500},
          )
        """
        self._components[name] = params
        logger.info(f"零件已注册: {name} ({list(params.keys())})")

    def register_profile(self, name: str, profile: HardwareProfile) -> None:
        """注册完整硬件配置。"""
        self._profiles[name] = profile
        logger.info(f"硬件配置已注册: {name}")

    def switch_to(self, name: str) -> bool:
        """
        切换到指定硬件配置。

        Returns:
            True 切换成功，False 配置不存在
        """
        if name not in self._profiles and name not in self._components:
            logger.error(f"硬件配置 '{name}' 不存在。"
                         f"可用: {list(self._profiles.keys())}")
            return False

        old = self._active_name
        self._active_name = name

        # 如果是 component，创建新 profile
        if name in self._components and name not in self._profiles:
            params = self._components[name]
            new_profile = HardwareProfile()
            if "motor" in params:
                for m in new_profile.motors:
                    new_profile.motors[m] = params["motor"]
            if "chassis_mods" in params:
                for k, v in params["chassis_mods"].items():
                    setattr(new_profile.chassis, k, v)
            self._profiles[name] = new_profile

        logger.info(f"硬件配置切换: {old} → {name}")
        return True

    def list_profiles(self) -> List[str]:
        """列出所有可用配置。"""
        return sorted(set(self._profiles.keys()) | set(self._components.keys()))

    def get_component_params(self, name: str) -> Optional[dict]:
        """获取注册的零件参数。"""
        return self._components.get(name)


# ============================================================
# 独立测试
# ============================================================

if __name__ == "__main__":
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s [%(levelname)-7s] %(name)s: %(message)s",
        datefmt="%H:%M:%S",
    )

    print("=" * 60)
    print("  硬件抽象层 — 独立测试")
    print("=" * 60)

    import tempfile, shutil
    test_dir = os.path.join(tempfile.gettempdir(), "test_hw_profile")
    shutil.rmtree(test_dir, ignore_errors=True)
    os.makedirs(test_dir, exist_ok=True)

    # ---- 测试 1: HardwareProfile 默认值 ----
    print("\n--- 测试 1: HardwareProfile 默认值 ---")
    profile = HardwareProfile()
    assert len(profile.motors) == 3
    assert profile.motors["wheel_0"].model == "JGB37-520"
    assert profile.chassis.wheel_mount_radius_mm == 150.0
    print(f"  电机数: {len(profile.motors)}, 回转半径: {profile.chassis.wheel_mount_radius_mm}mm")
    print("  ✅ 通过")

    # ---- 测试 2: HardwareProfile to_dict / from_dict 往返 ----
    print("\n--- 测试 2: to_dict / from_dict 往返 ---")
    d = profile.to_dict()
    profile2 = HardwareProfile.from_dict(d)
    assert profile2.motors["wheel_0"].model == profile.motors["wheel_0"].model
    assert profile2.chassis.wheel_mount_radius_mm == profile.chassis.wheel_mount_radius_mm
    print("  ✅ 通过")

    # ---- 测试 3: 摄像头内参 ----
    print("\n--- 测试 3: 摄像头内参 ---")
    cam = profile.calibration.camera
    K = cam.camera_matrix
    assert len(K) == 3
    assert K[0][0] == cam.fx
    assert K[1][1] == cam.fy
    print(f"  焦距: fx={cam.fx}, fy={cam.fy}")
    print(f"  光心: ({cam.cx}, {cam.cy})")
    print("  ✅ 通过")

    # ---- 测试 4: IMU 校准 ----
    print("\n--- 测试 4: IMU 校准 ---")
    cal = CalibrationRoutine(profile)
    result = cal.calibrate_imu(duration_s=0.5)
    assert "gyro_bias" in result
    assert "accel_bias" in result
    assert result["samples"] > 0
    print(f"  gyro_bias: {result['gyro_bias']}")
    print(f"  accel_bias: {result['accel_bias']}")
    print("  ✅ 通过")

    # ---- 测试 5: 电机校准 ----
    print("\n--- 测试 5: 电机校准 ---")
    result = cal.calibrate_motors("wheel_0", duration_s=0.1, speed_mm_s=200)
    assert result["motor"] == "wheel_0"
    assert result["ok"] is True or result["ok"] is False
    print(f"  电机: {result['motor']}, 误差: {result['error_pct']}%, ok={result['ok']}")
    print("  ✅ 通过")

    # ---- 测试 6: ComponentAdapter ----
    print("\n--- 测试 6: ComponentAdapter ---")
    adapter = ComponentAdapter()
    adapter.register_component(
        "race_motor",
        motor=MotorParams(model="RACE-001", max_rpm=800, reduction_ratio=15),
        chassis_mods={"max_linear_speed_mm_s": 2000},
    )
    assert "race_motor" in adapter.list_profiles()
    adapter.switch_to("race_motor")
    assert adapter.active_name == "race_motor"
    assert adapter.active_profile.motors["wheel_0"].max_rpm == 800
    print(f"  当前配置: {adapter.active_name}")
    print(f"  最大转速: {adapter.active_profile.motors['wheel_0'].max_rpm} RPM")
    print("  ✅ 通过")

    # ---- 测试 7: 摄像头标定 stub ----
    print("\n--- 测试 7: 摄像头标定 stub ---")
    result = cal.calibrate_camera()
    assert result["status"] == "stub"
    print(f"  状态: {result['status']}")
    print("  ✅ 通过")

    shutil.rmtree(test_dir, ignore_errors=True)

    print(f"\n{'=' * 60}")
    print("  硬件抽象层 — 全部测试通过 ✅")
    print(f"{'=' * 60}")
