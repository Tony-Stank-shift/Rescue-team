"""
imu_fusion.py —— MPU6050 陀螺仪 yaw 处理（协议 v1.1）

负责：
  1. 零偏校准：机器人静止时收集约 100~200 个 IMU 帧，求 gz 均值作为零偏。
  2. 运行时校正：gz_corrected = (gz_mrad_s - bias_mrad_s) / 1000  → rad/s。
  3. 供上位机把 gz_corrected_rad_s 传给「编码器 + IMU 航向融合器」。

单位说明（协议 5.2）：
  - 加速度：mg（1000 mg = 1 g）
  - 角速度：mrad/s
  - 温度：0.01 °C
  - 换算：accel_m_s2 = accel_mg * 9.80665 / 1000；gyro_rad_s = gyro_mrad_s / 1000；
          temp_C = temp_cC / 100
"""

import logging

logger = logging.getLogger("imu_fusion")


class ImuGyroFusion:
    """
    MPU6050 陀螺仪 yaw 角速度处理。

    上位机使用流程：
      fusion = ImuGyroFusion()
      # START 后机器人静止
      fusion.start_calibration()
      # 每收到一个 IMU 帧，若在静止校准期则 collect
      fusion.add_calibration_sample(gz_mrad_s)
      # 收集满后 finish（或自动）
      # 运行时：
      gz_rad_s = fusion.corrected_gyro_z(gz_mrad_s)
    """

    CALIBRATION_SAMPLES = 150   # 零偏校准需要的样本数（协议建议 100~200）
    GRAVITY = 9.80665           # 标准重力加速度

    def __init__(self, calibration_samples: int = 150):
        self.CALIBRATION_SAMPLES = calibration_samples
        self._bias_mrad_s = 0.0
        self._calibration_samples = []
        self._calibrated = False

    # ---- 零偏校准 ----

    def start_calibration(self) -> None:
        """开始零偏校准（机器人应保持静止）。"""
        self._calibration_samples = []
        self._calibrated = False
        logger.info("IMU 零偏校准开始（保持静止，收集约 %d 帧）", self.CALIBRATION_SAMPLES)

    def add_calibration_sample(self, gz_mrad_s: float) -> bool:
        """
        在校准期收集一个 gz 样本，返回是否已集满并完成校准。
        """
        self._calibration_samples.append(gz_mrad_s)
        if len(self._calibration_samples) >= self.CALIBRATION_SAMPLES:
            self.finish_calibration()
            return True
        return False

    def finish_calibration(self) -> None:
        """完成校准：计算 gz 均值作为零偏。"""
        if self._calibration_samples:
            self._bias_mrad_s = sum(self._calibration_samples) / len(self._calibration_samples)
            self._calibrated = True
            logger.info(
                f"IMU 零偏校准完成: gz_bias={self._bias_mrad_s:.3f} mrad/s "
                f"（{self._bias_mrad_s / 1000.0:.6f} rad/s, 样本={len(self._calibration_samples)}）"
            )

    # ---- 运行时 ----

    @property
    def calibrated(self) -> bool:
        return self._calibrated

    @property
    def bias_mrad_s(self) -> float:
        return self._bias_mrad_s

    def corrected_gyro_z(self, gz_mrad_s: float) -> float:
        """
        校正后的 z 轴角速度（rad/s）。

        未校准时直接除 1000（用原始值）；校准后扣除零偏再除 1000。
        """
        if not self._calibrated:
            return gz_mrad_s / 1000.0
        return (gz_mrad_s - self._bias_mrad_s) / 1000.0

    # ---- 工具换算（协议单位 -> 工程单位） ----

    @staticmethod
    def accel_to_m_s2(accel_mg: float) -> float:
        """加速度 mg → m/s²。"""
        return accel_mg * ImuGyroFusion.GRAVITY / 1000.0

    @staticmethod
    def gyro_to_rad_s(gyro_mrad_s: float) -> float:
        """角速度 mrad/s → rad/s。"""
        return gyro_mrad_s / 1000.0

    @staticmethod
    def temp_to_c(temp_cC: float) -> float:
        """温度 0.01°C → °C。"""
        return temp_cC / 100.0


# ============================================================
# 独立测试
# ============================================================

if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO, format="%(message)s")

    print("=" * 60)
    print("  IMU 融合 — 零偏校准 + 换算")
    print("=" * 60)

    fusion = ImuGyroFusion(calibration_samples=10)

    # --- 测试 1：校准前，i.e. gz 直接用 ---
    raw = fusion.corrected_gyro_z(23000)  # 23 rad/s = 23000 mrad/s
    print(f"\n校准前 gz=23000 mrad/s → {raw} rad/s")
    assert abs(raw - 23.0) < 1e-9
    print("  ✅ 通过")

    # --- 测试 2：静止校准（模拟 -5 mrad/s 零偏）---
    fusion.start_calibration()
    for _ in range(10):
        fusion.add_calibration_sample(-5.0)
    assert fusion.calibrated
    print(f"\n校准完成: bias={fusion.bias_mrad_s:.1f} mrad/s")
    assert abs(fusion.bias_mrad_s - (-5.0)) < 1e-6
    print("  ✅ 通过")

    # --- 测试 3：校准后 gz=95 mrad/s 应扣零偏 ---
    corrected = fusion.corrected_gyro_z(95.0)
    print(f"校准后 gz=95 mrad/s → {corrected:.6f} rad/s（扣除 -5 零偏后应为 0.1）")
    assert abs(corrected - 0.1) < 1e-6
    print("  ✅ 通过")

    # --- 测试 4：单位换算 ---
    assert abs(ImuGyroFusion.accel_to_m_s2(1000) - 9.80665) < 1e-6
    assert abs(ImuGyroFusion.gyro_to_rad_s(1000) - 1.0) < 1e-9
    assert abs(ImuGyroFusion.temp_to_c(3653) - 36.53) < 1e-6
    print("单位换算 (mg→m/s², mrad/s→rad/s, 0.01°C→°C) ✅")

    print(f"\n{'=' * 60}")
    print("  IMU 融合 — 全部测试通过 ✅")
    print(f"{'=' * 60}")
