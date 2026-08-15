"""
fault_tolerance.py —— 故障容错

持续监控系统健康状态，检测并处理：
  1. 传感器降级（摄像头/IMU 故障时的退化策略）
  2. 电机故障（堵转/空转/过热检测）
  3. 通信中断（DEBUG 模式下连接丢失不影响本体运行）
  4. 电源监控（分级低电量告警 + 电压曲线记录）
"""

import logging
import math
import time
from dataclasses import dataclass, field
from enum import Enum, auto
from typing import Dict, List, Optional, Tuple, Callable

logger = logging.getLogger("fault_tolerance")


# ============================================================
# 通用枚举与数据结构
# ============================================================

class DegradationLevel(Enum):
    """传感器降级级别"""
    FULL = 0          # 全功能正常
    PARTIAL = 1       # 部分降级（可继续运行）
    CRITICAL = 2      # 关键传感器故障（安全停止）


class SensorID(Enum):
    """传感器标识"""
    CAMERA = "camera"
    IMU = "imu"
    TEMPERATURE = "temperature"
    VIBRATION = "vibration"
    MOTOR_LEFT = "motor_left"
    MOTOR_RIGHT = "motor_right"


class SensorStatus(Enum):
    """传感器状态"""
    OK = "ok"
    DEGRADED = "degraded"    # 性能下降但仍在工作
    FAILED = "failed"        # 完全失效
    UNKNOWN = "unknown"      # 无法判断


@dataclass
class SensorHealthReport:
    """传感器健康报告"""
    sensor_id: SensorID
    status: SensorStatus = SensorStatus.UNKNOWN
    detail: str = ""
    timestamp: float = field(default_factory=time.time)
    critical: bool = True    # 是否为关键传感器

    def to_dict(self) -> dict:
        return {
            "sensor": self.sensor_id.value,
            "status": self.status.value,
            "detail": self.detail,
            "timestamp": self.timestamp,
            "critical": self.critical,
        }


@dataclass
class SystemHealthReport:
    """系统整体健康报告"""
    sensors: Dict[SensorID, SensorHealthReport] = field(default_factory=dict)
    degradation_level: DegradationLevel = DegradationLevel.FULL
    overall_ok: bool = True

    def summary(self) -> str:
        lines = [f"系统健康: 降级级别={self.degradation_level.name}, "
                 f"整体={'正常' if self.overall_ok else '异常'}"]
        for sid, r in self.sensors.items():
            icon = ("✓" if r.status == SensorStatus.OK else
                    "⚠" if r.status == SensorStatus.DEGRADED else "✗")
            lines.append(f"  {icon} {sid.value}: {r.status.value} ({r.detail})")
        return "\n".join(lines)


class MotorStatus(Enum):
    """电机状态"""
    OK = "ok"
    STALL = "stall"          # 堵转（高电流 + 低速度）
    IDLE_SPIN = "idle_spin"  # 空转（低电流 + 高速度指令）
    OVERHEAT = "overheat"    # 过热
    FAULT = "fault"          # 未知故障


@dataclass
class MotorFaultReport:
    """电机故障报告"""
    motor_id: int
    status: MotorStatus = MotorStatus.OK
    current_ma: float = 0.0
    speed_mm_s: float = 0.0
    temperature_c: float = 25.0
    fault_count: int = 0
    detail: str = ""
    timestamp: float = field(default_factory=time.time)

    def to_dict(self) -> dict:
        return {
            "motor_id": self.motor_id,
            "status": self.status.value,
            "current_ma": self.current_ma,
            "speed_mm_s": self.speed_mm_s,
            "temperature_c": self.temperature_c,
            "fault_count": self.fault_count,
            "detail": self.detail,
            "timestamp": self.timestamp,
        }


class PowerStatus(Enum):
    """电源状态"""
    OK = "ok"
    WARNING = "warning"    # < 11.5V
    CRITICAL = "critical"  # < 11.0V


@dataclass
class PowerReport:
    """电源报告"""
    voltage: float = 0.0
    status: PowerStatus = PowerStatus.OK
    voltage_history: List[Tuple[float, float]] = field(default_factory=list)
    timestamp: float = field(default_factory=time.time)

    def to_dict(self) -> dict:
        return {
            "voltage": self.voltage,
            "status": self.status.value,
            "timestamp": self.timestamp,
            "history_count": len(self.voltage_history),
        }


# ============================================================
# 1. SensorHealthMonitor —— 持续传感器健康监控
# ============================================================

class SensorHealthMonitor:
    """
    传感器健康监控器。

    定期检查各传感器状态，输出降级级别和恢复建议。
    关键传感器（摄像头/IMU）全部故障 → 安全停止。

    使用方式:
        monitor = SensorHealthMonitor(hardware_checker)
        report = monitor.check_all()
        if report.degradation_level == DegradationLevel.CRITICAL:
            # 触发安全停止
    """

    # 降级阈值
    DEGRADED_FPS_THRESHOLD = 5.0        # 低于此FPS视为降级
    FAILED_FPS_THRESHOLD = 1.0          # 低于此FPS视为失效

    def __init__(self, hardware_checker=None):
        """
        Args:
            hardware_checker: HardwareChecker 实例（用于实际轮询传感器）
                              None 时使用内置模拟状态（Mock 模式）
        """
        self._hw = hardware_checker
        self._sensor_status: Dict[SensorID, SensorHealthReport] = {
            sid: SensorHealthReport(sensor_id=sid, critical=self._is_critical(sid))
            for sid in SensorID
        }
        self._last_check_time = 0.0
        self._check_interval_s = 1.0    # 每秒检查一次

        # 降级后的替代策略标记
        self._using_fallback_camera = False
        self._using_fallback_imu = False

        logger.info("SensorHealthMonitor 初始化")

    # ---- 主检测接口 ----

    def check_all(self, force: bool = False) -> SystemHealthReport:
        """
        检查所有传感器健康状态。

        Args:
            force: True 时无论间隔强制检查

        Returns:
            SystemHealthReport: 整体健康报告
        """
        now = time.time()
        if not force and now - self._last_check_time < self._check_interval_s:
            # 返回缓存的整体状态
            return self._build_report()

        self._last_check_time = now

        # 逐传感器检查
        self._check_camera()
        self._check_imu()
        self._check_temperature()
        self._check_vibration()
        self._check_motors_general()

        return self._build_report()

    def degradation_level(self) -> DegradationLevel:
        """快速获取当前降级级别"""
        report = self.check_all()
        return report.degradation_level

    def is_critical(self) -> bool:
        """关键传感器全部故障？"""
        return self.degradation_level() == DegradationLevel.CRITICAL

    # ---- 传感器特定检查 ----

    def _check_camera(self) -> None:
        """检查摄像头健康"""
        sid = SensorID.CAMERA
        report = self._sensor_status[sid]

        if self._hw and hasattr(self._hw, "check_camera"):
            try:
                ok = self._hw.check_camera()
                if hasattr(self._hw, "get_camera_fps"):
                    fps = self._hw.get_camera_fps()
                else:
                    fps = 30.0 if ok else 0.0
            except Exception as e:
                logger.error(f"摄像头检查异常: {e}")
                ok, fps = False, 0.0
        else:
            ok, fps = True, 30.0

        if not ok or fps < self.FAILED_FPS_THRESHOLD:
            report.status = SensorStatus.FAILED
            report.detail = f"摄像头失效 (fps={fps:.0f})"
            self._using_fallback_camera = True
        elif fps < self.DEGRADED_FPS_THRESHOLD:
            report.status = SensorStatus.DEGRADED
            report.detail = f"摄像头降级 (fps={fps:.0f})"
        else:
            report.status = SensorStatus.OK
            report.detail = f"正常 (fps={fps:.0f})"
            self._using_fallback_camera = False

        report.timestamp = time.time()

    def _check_imu(self) -> None:
        """检查 IMU 健康"""
        sid = SensorID.IMU
        report = self._sensor_status[sid]

        if self._hw and hasattr(self._hw, "check_imu"):
            try:
                ok = self._hw.check_imu()
            except Exception as e:
                logger.error(f"IMU 检查异常: {e}")
                ok = False
        else:
            ok = True

        if not ok:
            report.status = SensorStatus.FAILED
            report.detail = "IMU 无数据输出"
            self._using_fallback_imu = True
        else:
            report.status = SensorStatus.OK
            report.detail = "正常"
            self._using_fallback_imu = False

        report.timestamp = time.time()

    def _check_temperature(self) -> None:
        """检查温度传感器（非关键）"""
        sid = SensorID.TEMPERATURE
        report = self._sensor_status[sid]

        if self._hw and hasattr(self._hw, "check_temperature_sensor"):
            try:
                ok = self._hw.check_temperature_sensor()
            except Exception:
                ok = False
        else:
            ok = True

        report.status = SensorStatus.OK if ok else SensorStatus.FAILED
        report.detail = "正常" if ok else "无数据"
        report.timestamp = time.time()

    def _check_vibration(self) -> None:
        """检查振动传感器（非关键）"""
        sid = SensorID.VIBRATION
        report = self._sensor_status[sid]
        # 振动传感器是可选的
        report.status = SensorStatus.OK
        report.detail = "待接入"
        report.timestamp = time.time()

    def _check_motors_general(self) -> None:
        """检查电机通信状态（详细故障由 MotorFaultDetector 处理）"""
        for sid in [SensorID.MOTOR_LEFT, SensorID.MOTOR_RIGHT]:
            report = self._sensor_status[sid]
            if self._hw and hasattr(self._hw, "check_motor"):
                motor_id = 0 if sid == SensorID.MOTOR_LEFT else 1
                try:
                    ok = self._hw.check_motor(motor_id)
                except Exception:
                    ok = False

                report.status = SensorStatus.OK if ok else SensorStatus.FAILED
                report.detail = "正常" if ok else "电机无响应"
            else:
                report.status = SensorStatus.OK
                report.detail = "正常（Mock）"
            report.timestamp = time.time()

    # ---- 降级策略 ----

    def get_navigation_fallback(self) -> Dict[str, bool]:
        """返回导航系统的降级配置"""
        return {
            "use_camera": not self._using_fallback_camera,
            "use_imu": not self._using_fallback_imu,
            "use_odometry": True,
            "use_world_map_memory": self._using_fallback_camera,
        }

    # ---- 反馈 ----

    def notify_sensor_recovered(self, sensor_id: SensorID) -> None:
        """通知某传感器已恢复"""
        report = self._sensor_status[sensor_id]
        if report.status != SensorStatus.OK:
            logger.info(f"传感器恢复: {sensor_id.value}")
            report.status = SensorStatus.OK
            report.detail = "已恢复"
            report.timestamp = time.time()

            if sensor_id == SensorID.CAMERA:
                self._using_fallback_camera = False
            elif sensor_id == SensorID.IMU:
                self._using_fallback_imu = False

    # ---- 内部 ----

    def _build_report(self) -> SystemHealthReport:
        """根据传感器状态构建整体报告"""
        critical_failed = 0

        for sid, report in self._sensor_status.items():
            if report.critical and report.status == SensorStatus.FAILED:
                critical_failed += 1

        if critical_failed >= 2:  # 摄像头 + IMU 都坏了
            level = DegradationLevel.CRITICAL
            ok = False
        elif critical_failed >= 1:
            level = DegradationLevel.PARTIAL
            ok = True
        else:
            level = DegradationLevel.FULL
            ok = True

        return SystemHealthReport(
            sensors=dict(self._sensor_status),
            degradation_level=level,
            overall_ok=ok,
        )

    @staticmethod
    def _is_critical(sensor_id: SensorID) -> bool:
        return sensor_id in {
            SensorID.CAMERA, SensorID.IMU,
            SensorID.MOTOR_LEFT, SensorID.MOTOR_RIGHT,
        }


# ============================================================
# 2. MotorFaultDetector —— 电机故障检测
# ============================================================

class MotorFaultDetector:
    """
    电机故障检测器。

    每帧调用 check() 检测单个电机的状态：
      - 堵转：电流 > 堵转阈值 且 速度 < 最低速度
      - 空转：电流 < 最低电流 且 速度指令 > 最低速度
      - 过热：温度 > 过热阈值

    累计故障次数，超阈值触发告警。
    """

    # 检测阈值
    STALL_CURRENT_MA = 5000       # 堵转电流阈值（待定，应高于正常行驶 3-4A）
    STALL_SPEED_MM_S = 20.0       # 堵转判定速度上限
    IDLE_CURRENT_MA = 100         # 空载判定电流下限
    IDLE_SPEED_MM_S = 100.0       # 空转判定速度指令下限
    OVERHEAT_TEMP_C = 70.0        # 过热温度阈值
    FAULT_COUNT_WARN = 5          # 连续故障告警阈值

    def __init__(self, motor_count: int = 2):
        self._motor_count = motor_count
        self._motor_faults: Dict[int, List[MotorFaultReport]] = {
            i: [] for i in range(motor_count)
        }
        self._consecutive_faults: Dict[int, int] = {
            i: 0 for i in range(motor_count)
        }
        logger.info(f"MotorFaultDetector 初始化: {motor_count} 个电机")

    # ---- 主检测接口 ----

    def check(self,
              motor_id: int,
              current_ma: float,
              speed_mm_s: float = 0.0,
              target_speed_mm_s: float = 0.0,
              temperature_c: float = 25.0) -> MotorFaultReport:
        """
        检测单个电机状态。
        """
        if motor_id < 0 or motor_id >= self._motor_count:
            return MotorFaultReport(
                motor_id=motor_id,
                status=MotorStatus.FAULT,
                detail=f"无效电机ID: {motor_id}",
            )

        status = MotorStatus.OK
        detail = "正常"
        abs_target = abs(target_speed_mm_s)
        abs_speed = abs(speed_mm_s)

        # 检测1: 过热
        if temperature_c > self.OVERHEAT_TEMP_C:
            status = MotorStatus.OVERHEAT
            detail = f"过热: {temperature_c:.0f}°C > {self.OVERHEAT_TEMP_C:.0f}°C"

        # 检测2: 堵转
        elif (current_ma > self.STALL_CURRENT_MA and
              abs_speed < self.STALL_SPEED_MM_S and
              abs_target > self.STALL_SPEED_MM_S):
            status = MotorStatus.STALL
            detail = (f"堵转: 电流={current_ma:.0f}mA > "
                      f"{self.STALL_CURRENT_MA}mA, "
                      f"速度={speed_mm_s:.0f}mm/s")

        # 检测3: 空转（脱轴/皮带断裂）
        elif (current_ma < self.IDLE_CURRENT_MA and
              abs_target > self.IDLE_SPEED_MM_S):
            status = MotorStatus.IDLE_SPIN
            detail = (f"空转(疑似脱轴): 电流={current_ma:.0f}mA < "
                      f"{self.IDLE_CURRENT_MA}mA")

        # 检测4: 未知异常
        elif (current_ma > self.IDLE_CURRENT_MA and
              abs_speed < self.STALL_SPEED_MM_S and
              abs_target > self.STALL_SPEED_MM_S):
            status = MotorStatus.FAULT
            detail = f"异常: 电流={current_ma:.0f}mA, 速度={speed_mm_s:.0f}mm/s"

        # 更新故障计数
        if status == MotorStatus.OK:
            self._consecutive_faults[motor_id] = 0
        else:
            self._consecutive_faults[motor_id] += 1

        report = MotorFaultReport(
            motor_id=motor_id,
            status=status,
            current_ma=current_ma,
            speed_mm_s=speed_mm_s,
            temperature_c=temperature_c,
            fault_count=self._consecutive_faults[motor_id],
            detail=detail,
        )

        if status != MotorStatus.OK:
            self._motor_faults[motor_id].append(report)

        if self._consecutive_faults[motor_id] >= self.FAULT_COUNT_WARN:
            logger.warning(
                f"电机 #{motor_id} 连续故障 {self._consecutive_faults[motor_id]} 次: "
                f"{status.value} — {detail}"
            )

        return report

    def check_all(self,
                  currents_ma: List[float],
                  speeds_mm_s: List[float],
                  targets_mm_s: List[float],
                  temps_c: Optional[List[float]] = None
                  ) -> List[MotorFaultReport]:
        """批量检测所有电机"""
        if temps_c is None:
            temps_c = [25.0] * self._motor_count

        reports = []
        for i in range(self._motor_count):
            report = self.check(
                motor_id=i,
                current_ma=currents_ma[i] if i < len(currents_ma) else 0,
                speed_mm_s=speeds_mm_s[i] if i < len(speeds_mm_s) else 0,
                target_speed_mm_s=targets_mm_s[i] if i < len(targets_mm_s) else 0,
                temperature_c=temps_c[i] if i < len(temps_c) else 25.0,
            )
            reports.append(report)
        return reports

    def get_fault_history(self, motor_id: int) -> List[MotorFaultReport]:
        """获取指定电机的故障历史"""
        if motor_id in self._motor_faults:
            return list(self._motor_faults[motor_id])
        return []

    def get_consecutive_faults(self, motor_id: int) -> int:
        return self._consecutive_faults.get(motor_id, 0)

    def has_any_fault(self) -> bool:
        return any(c > 0 for c in self._consecutive_faults.values())

    def reset(self) -> None:
        self._motor_faults = {i: [] for i in range(self._motor_count)}
        self._consecutive_faults = {i: 0 for i in range(self._motor_count)}
        logger.info("MotorFaultDetector 已重置")


# ============================================================
# 3. CommWatchdog —— 通信中断处理
# ============================================================

class CommWatchdog:
    """
    通信看门狗。

    - DEBUG 模式：检测通信连接是否正常，断开不影响机器人运行
    - AUTONOMOUS 模式：不做接收检查，仅追踪广播统计
    """

    HEARTBEAT_TIMEOUT_S = 3.0
    RECONNECT_BACKOFF_S = 1.0
    MAX_RECONNECT_BACKOFF_S = 30.0

    def __init__(self):
        self._last_heartbeat = time.time()
        self._connection_lost = False
        self._reconnect_count = 0
        self._total_disconnects = 0
        self._heartbeats_received = 0
        self._heartbeats_missed = 0
        logger.info("CommWatchdog 初始化")

    def notify_heartbeat_received(self) -> None:
        self._last_heartbeat = time.time()
        if self._connection_lost:
            logger.info(f"通信恢复 (断开 {self._reconnect_count} 次后重连成功)")
            self._connection_lost = False
            self._reconnect_count = 0
        self._heartbeats_received += 1

    def check_connection(self) -> bool:
        elapsed = time.time() - self._last_heartbeat
        if elapsed > self.HEARTBEAT_TIMEOUT_S:
            if not self._connection_lost:
                self._connection_lost = True
                self._reconnect_count += 1
                self._total_disconnects += 1
                logger.warning(
                    f"通信中断 ({elapsed:.1f}s 无心跳) — "
                    f"机器人继续自主运行，不受影响"
                )
            self._heartbeats_missed += 1
            return False
        return True

    def is_connection_lost(self) -> bool:
        return self._connection_lost

    def get_reconnect_delay(self) -> float:
        delay = self.RECONNECT_BACKOFF_S * (2 ** (self._reconnect_count - 1))
        return min(delay, self.MAX_RECONNECT_BACKOFF_S)

    @property
    def total_disconnects(self) -> int:
        return self._total_disconnects

    @property
    def heartbeats_received(self) -> int:
        return self._heartbeats_received

    def get_stats(self) -> dict:
        return {
            "connection_ok": not self._connection_lost,
            "last_heartbeat_ago_s": time.time() - self._last_heartbeat,
            "total_disconnects": self._total_disconnects,
            "heartbeats_received": self._heartbeats_received,
            "heartbeats_missed": self._heartbeats_missed,
            "reconnect_count": self._reconnect_count,
        }


# ============================================================
# 4. PowerMonitor —— 电源监控
# ============================================================

class PowerMonitor:
    """
    电源监控器。

    持续记录电池电压，分级告警：
      - > 11.5V: OK
      - 11.0-11.5V: WARNING
      - < 11.0V: CRITICAL
    """

    VOLTAGE_OK = 11.5
    VOLTAGE_WARN = 11.0
    VOLTAGE_CRITICAL = 10.5
    MAX_HISTORY_POINTS = 600

    def __init__(self, voltage_reader: Optional[Callable[[], float]] = None):
        self._read_voltage = voltage_reader or self._mock_read_voltage
        self._voltage_history: List[Tuple[float, float]] = []
        self._current_voltage = 0.0
        self._current_status = PowerStatus.OK
        self._warning_triggered = False
        self._critical_triggered = False
        self._min_voltage = float("inf")
        self._max_voltage = 0.0
        logger.info("PowerMonitor 初始化")

    def read(self) -> PowerReport:
        try:
            voltage = self._read_voltage()
        except Exception as e:
            logger.error(f"电压读取异常: {e}")
            voltage = -1.0

        self._current_voltage = voltage
        now = time.time()

        if voltage > 0:
            self._voltage_history.append((now, voltage))
            if len(self._voltage_history) > self.MAX_HISTORY_POINTS:
                self._voltage_history = self._voltage_history[-self.MAX_HISTORY_POINTS:]
            self._min_voltage = min(self._min_voltage, voltage)
            self._max_voltage = max(self._max_voltage, voltage)

        if voltage < 0:
            self._current_status = PowerStatus.CRITICAL
        elif voltage < self.VOLTAGE_CRITICAL:
            self._current_status = PowerStatus.CRITICAL
            if not self._critical_triggered:
                self._critical_triggered = True
                logger.critical(
                    f"🔋 电池电压危急: {voltage:.1f}V < "
                    f"{self.VOLTAGE_CRITICAL}V！可能随时断电！"
                )
        elif voltage < self.VOLTAGE_WARN:
            self._current_status = PowerStatus.WARNING
            if not self._warning_triggered:
                self._warning_triggered = True
                logger.warning(
                    f"🔋 电池电压偏低: {voltage:.1f}V < "
                    f"{self.VOLTAGE_WARN}V，建议充电"
                )
        elif voltage < self.VOLTAGE_OK:
            self._current_status = PowerStatus.WARNING
        else:
            self._current_status = PowerStatus.OK

        return PowerReport(
            voltage=voltage,
            status=self._current_status,
            voltage_history=list(self._voltage_history),
            timestamp=now,
        )

    def get_power_status(self) -> PowerStatus:
        return self._current_status

    def is_battery_low(self) -> bool:
        return self._current_status in (PowerStatus.WARNING, PowerStatus.CRITICAL)

    def get_voltage_trend(self, last_n: int = 30) -> Optional[float]:
        if len(self._voltage_history) < last_n:
            return None

        recent = self._voltage_history[-last_n:]
        n = len(recent)
        sum_x = sum(i for i in range(n))
        sum_y = sum(v for _, v in recent)
        sum_xy = sum(i * v for i, (_, v) in enumerate(recent))
        sum_x2 = sum(i * i for i in range(n))

        denominator = n * sum_x2 - sum_x * sum_x
        if denominator == 0:
            return None

        slope = (n * sum_xy - sum_x * sum_y) / denominator
        return slope

    def get_stats(self) -> dict:
        return {
            "current_voltage": self._current_voltage,
            "status": self._current_status.value,
            "min_voltage": self._min_voltage if self._min_voltage != float("inf") else 0,
            "max_voltage": self._max_voltage,
            "samples": len(self._voltage_history),
            "trend": self.get_voltage_trend(),
            "warning_triggered": self._warning_triggered,
            "critical_triggered": self._critical_triggered,
        }

    @staticmethod
    def _mock_read_voltage() -> float:
        elapsed = time.time() % 3600
        return 12.4 - (elapsed / 3600) * 0.6


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
    print("  故障容错模块 — 独立测试")
    print("=" * 60)

    # ---- 测试 1: SensorHealthMonitor ----
    print("\n--- 测试 1: SensorHealthMonitor ---")
    monitor = SensorHealthMonitor()
    report = monitor.check_all()
    print(report.summary())
    assert report.overall_ok, "Mock 模式下传感器应全部正常"
    assert report.degradation_level == DegradationLevel.FULL
    print("✅ 测试 1 通过")

    # ---- 测试 2: 传感器降级 ----
    print("\n--- 测试 2: 传感器降级 ---")
    # 使用带故障返回的硬件检查器来模拟
    class _FaultyChecker:
        def check_camera(self): return False
        def check_imu(self): return True
        def check_temperature_sensor(self): return True
        def check_motor(self, mid): return True
        def get_camera_fps(self): return 0.0
        def get_motor_count(self): return 2
    monitor2 = SensorHealthMonitor(hardware_checker=_FaultyChecker())
    report = monitor2.check_all(force=True)
    print(f"  降级级别: {report.degradation_level.name}")
    assert report.degradation_level == DegradationLevel.PARTIAL, \
        f"摄像头故障应为 PARTIAL，实际为 {report.degradation_level.name}"
    print("✅ 测试 2 通过")

    # ---- 测试 3: 关键传感器全部故障 → CRITICAL ----
    print("\n--- 测试 3: 全部关键传感器故障 ---")
    class _DoubleFaultChecker:
        def check_camera(self): return False
        def check_imu(self): return False
        def check_temperature_sensor(self): return True
        def check_motor(self, mid): return True
        def get_camera_fps(self): return 0.0
        def get_motor_count(self): return 2
    monitor3 = SensorHealthMonitor(hardware_checker=_DoubleFaultChecker())
    report = monitor3.check_all(force=True)
    print(f"  降级级别: {report.degradation_level.name}")
    assert report.degradation_level == DegradationLevel.CRITICAL, \
        "双传感器故障应为 CRITICAL"
    print("✅ 测试 3 通过")

    # ---- 测试 4: MotorFaultDetector ----
    print("\n--- 测试 4: MotorFaultDetector ---")
    detector = MotorFaultDetector(motor_count=2)

    # 正常
    r = detector.check(0, current_ma=500, speed_mm_s=200, target_speed_mm_s=200)
    print(f"  正常: {r.status.value} — {r.detail}")
    assert r.status == MotorStatus.OK
    print("  ✅ 正常检测通过")

    # 堵转
    r = detector.check(0, current_ma=5500, speed_mm_s=5, target_speed_mm_s=200)
    print(f"  堵转: {r.status.value} — {r.detail}")
    assert r.status == MotorStatus.STALL, f"应为 STALL，实际为 {r.status.value}"
    print("  ✅ 堵转检测通过")

    # 空转
    r = detector.check(1, current_ma=30, speed_mm_s=5, target_speed_mm_s=300)
    print(f"  空转: {r.status.value} — {r.detail}")
    assert r.status == MotorStatus.IDLE_SPIN, f"应为 IDLE_SPIN，实际为 {r.status.value}"
    print("  ✅ 空转检测通过")

    # 过热
    r = detector.check(0, current_ma=500, speed_mm_s=200, target_speed_mm_s=200,
                       temperature_c=85)
    print(f"  过热: {r.status.value} — {r.detail}")
    assert r.status == MotorStatus.OVERHEAT
    print("  ✅ 过热检测通过")

    # 连续故障
    for _ in range(6):
        detector.check(0, current_ma=5500, speed_mm_s=5, target_speed_mm_s=200)
    assert detector.get_consecutive_faults(0) >= 5
    print("  ✅ 连续故障计数通过")

    print("✅ 测试 4 全部通过")

    # ---- 测试 5: CommWatchdog ----
    print("\n--- 测试 5: CommWatchdog ---")
    wd = CommWatchdog()
    assert wd.check_connection(), "初始应正常"
    wd._last_heartbeat = time.time() - 5.0
    assert not wd.check_connection(), "应检测到断连"
    print(f"  断连检测: OK (心跳间隔={time.time() - wd._last_heartbeat:.0f}s)")
    wd.notify_heartbeat_received()
    assert wd.check_connection(), "应恢复连接"
    print(f"  恢复检测: OK")
    stats = wd.get_stats()
    print(f"  统计: 断连{stats['total_disconnects']}次, "
          f"收到{stats['heartbeats_received']}次心跳")
    print("✅ 测试 5 通过")

    # ---- 测试 6: PowerMonitor ----
    print("\n--- 测试 6: PowerMonitor ---")
    pm = PowerMonitor()
    report = pm.read()
    print(f"  电压: {report.voltage:.1f}V, 状态: {report.status.value}")
    assert report.status == PowerStatus.OK, \
        f"Mock 电压应在正常范围，实际={report.status.value}"
    print("  ✅ 正常电压通过")

    pm_low = PowerMonitor(voltage_reader=lambda: 10.3)
    report = pm_low.read()
    print(f"  低压(危急): {report.voltage:.1f}V, 状态: {report.status.value}")
    assert report.status == PowerStatus.CRITICAL, \
        f"应触发 CRITICAL，实际={report.status.value}"
    print("  ✅ 低压告警通过")

    for i in range(30):
        pm._voltage_history.append((time.time() - 30 + i, 12.0 - i * 0.01))
    trend = pm.get_voltage_trend(last_n=30)
    print(f"  电压趋势: {trend:.4f} V/采样 (负值=下降中)")
    assert trend is not None and trend < 0, "应为下降趋势"
    print("  ✅ 电压趋势通过")

    print("✅ 测试 6 全部通过")

    print(f"\n{'=' * 60}")
    print("  故障容错模块 — 全部测试通过 ✅")
    print(f"{'=' * 60}")
