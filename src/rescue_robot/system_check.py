"""
system_check.py —— 系统自检

在 BOOT 状态下运行，逐一检查所有关键子系统：
  1. 传感器数据流（摄像头、IMU、温度等）
  2. 电机驱动（每个电机测试正反转）
  3. 电源电压（电池是否在安全范围）

所有检查项设计为平台无关的抽象接口，方便 Mock 测试。
"""

import logging
import os
import time
from dataclasses import dataclass, field
from enum import Enum
from typing import List, Optional, Callable

from .config import thresholds

logger = logging.getLogger("system_check")


# ============================================================
# 检查结果
# ============================================================

class CheckStatus(Enum):
    """检查状态"""
    PENDING = "pending"     # 未开始
    RUNNING = "running"     # 检查中
    PASS = "pass"           # 通过
    FAIL = "fail"           # 失败
    SKIPPED = "skipped"     # 跳过（依赖项失败）


@dataclass
class CheckItem:
    """单项检查结果"""
    name: str                           # 检查项名称
    status: CheckStatus = CheckStatus.PENDING
    message: str = ""                   # 详细信息
    duration_ms: float = 0.0            # 耗时（毫秒）
    critical: bool = True               # 是否为关键项（失败则整体不通过）

    def to_dict(self) -> dict:
        return {
            "name": self.name,
            "status": self.status.name,
            "message": self.message,
            "duration_ms": self.duration_ms,
            "critical": self.critical,
        }


@dataclass
class CheckReport:
    """自检报告"""
    items: List[CheckItem] = field(default_factory=list)
    overall_pass: bool = False
    total_duration_ms: float = 0.0

    @property
    def passed_count(self) -> int:
        return sum(1 for i in self.items if i.status == CheckStatus.PASS)

    @property
    def failed_count(self) -> int:
        return sum(1 for i in self.items if i.status == CheckStatus.FAIL)

    @property
    def failed_critical(self) -> List[CheckItem]:
        return [i for i in self.items if i.status == CheckStatus.FAIL and i.critical]

    def summary(self) -> str:
        lines = [f"自检完成: {self.passed_count} 通过, {self.failed_count} 失败"]
        for item in self.items:
            icon = "✓" if item.status == CheckStatus.PASS else "✗" if item.status == CheckStatus.FAIL else "○"
            lines.append(f"  {icon} {item.name}: {item.message} ({item.duration_ms:.0f}ms)")
        return "\n".join(lines)


# ============================================================
# 硬件检查接口（抽象层）
# ============================================================

class HardwareChecker:
    """
    硬件检查器抽象基类。
    实际硬件需要实现这些方法；Mock 模式返回假数据。
    """

    def check_camera(self) -> bool:
        """检查摄像头是否正常输出数据流"""
        raise NotImplementedError

    def check_imu(self) -> bool:
        """检查 IMU 是否正常输出数据"""
        raise NotImplementedError

    def check_temperature_sensor(self) -> bool:
        """检查温度传感器"""
        raise NotImplementedError

    def check_motor(self, motor_id: int) -> bool:
        """检查指定电机是否能正常响应"""
        raise NotImplementedError

    def check_battery_voltage(self) -> float:
        """返回电池电压（V），返回 -1 表示读取失败"""
        raise NotImplementedError

    def get_motor_count(self) -> int:
        """返回电机总数"""
        raise NotImplementedError


class MockHardwareChecker(HardwareChecker):
    """Mock 硬件检查器 —— 用于本地开发和 CI 测试"""

    def check_camera(self) -> bool:
        time.sleep(0.2)
        return True

    def check_imu(self) -> bool:
        time.sleep(0.1)
        return True

    def check_temperature_sensor(self) -> bool:
        time.sleep(0.05)
        return True

    def check_motor(self, motor_id: int) -> bool:
        time.sleep(0.15)
        return True

    def check_battery_voltage(self) -> float:
        return 12.0  # 模拟满电

    def get_motor_count(self) -> int:
        return 2  # 2 个驱动电机（差速）


class RealHardwareChecker(HardwareChecker):
    """
    真实硬件检查器（框架版）。

    ⚠️ 真值读取（摄像头/IMU/电池/电机）依赖真机硬件，此处提供接口 + 优雅降级：
      - 摄像头：尝试打开并读一帧（未接摄像头/失败 → False）
      - IMU：IMU 在 STM32，尝试从串口读 IMU 帧
      - 电机：尝试通过串口 PING 探测底盘连通
      - 电池电压：需真机（串口 TEL 或 ADC），未实现返回 -1
    「真机就绪」后，把上面读取逻辑替换为真实硬件采集即可。
    """

    def __init__(self, chassis=None, camera_index: int = 1):
        self._chassis = chassis          # SerialChassis 实例（可选）
        self._camera_index = camera_index

    def check_camera(self) -> bool:
        # 台架调试（未接摄像头）可设 SKIP_CAMERA_CHECK=1 跳过该项
        if os.environ.get("SKIP_CAMERA_CHECK", "").strip().lower() in ("1", "true", "yes"):
            logger.info("SKIP_CAMERA_CHECK=1，跳过摄像头自检（台架调试）")
            return True
        # 用后台采集线程探测（带超时），避免 BOOT 阶段被阻塞式 read() 卡住：
        # 旧实现直接 cv2.VideoCapture + cap.read()，摄像头未就绪时会长时间阻塞自检。
        try:
            from .hardware.camera_reader import CameraReader
            cam = CameraReader(self._camera_index, name="cam-check")
            if not cam.start():
                return False
            ok = cam.wait_first_frame(timeout=2.0)
            cam.stop()
            return ok
        except Exception as e:
            logger.warning(f"摄像头自检异常: {e}")
            return False

    def check_imu(self) -> bool:
        # IMU 在 STM32，通过串口 IMU 帧获取；未接串口/无 IMU 帧按失败
        if self._chassis is not None and self._chassis.is_open:
            try:
                return self._chassis.read_imu() is not None
            except Exception:
                return False
        return False

    def check_temperature_sensor(self) -> bool:
        # 无独立温度传感器（非关键项），跳过即通过
        return True

    #: 底盘连通性探测的重试策略。
    #: 下位机的**命令接收存在秒级静默窗口**（2026-09-16 实测：PING 应答晚到
    #: 2010ms 才回来，而同一时刻下一条 PING 却 2ms 就通）——偶发、可自愈。
    #: 单次探测撞进窗口就会误判"串口故障"→ BOOT 自检失败 → 车拒绝启动
    #: （现场实际发生，而电机本身完全正常）。故重试若干次并留间隔。
    PROBE_ATTEMPTS: int = 4
    PROBE_TIMEOUT_S: float = 1.5
    PROBE_GAP_S: float = 0.4

    def check_motor(self, motor_id: int) -> bool:
        """底盘连通性探测（PING → PONG，带重试）。

        ⚠️ 名字叫"电机检查"，但它**不检测电机本体** —— 电机正反转/测速必须真的驱动
        轮子，见 ``tools/hw_selftest.py`` 的 motors / velocity 模块（需 ``--yes-motion``）。
        本项实际只验证一件事：**上位机 ↔ 下位机串口链路是否活着**。

        现场实测踩到的三个坑（都会导致 BOOT 自检失败、车拒绝启动）：

        1) **超时不能短**：下位机主循环穿插发送约 84 行/秒遥测，PONG 往返偶发变慢。
           旧代码用 ``send_ping()`` 默认 0.5s → 误报 FAIL
           （日志 ``电机 #1: 检查未通过 (505ms)``，505ms 正是超时到期）。

        2) **发 PING 前必须排空残留行**：否则上一条命令迟到的应答会被本次
           ``wait_for`` 立刻捡到，出现假通过
           （日志 ``电机 #2: 正常 (3ms)`` —— 3ms 就是捡到 #1 残留的 PONG）。

        3) **必须重试**：下位机命令接收有**秒级静默窗口**且可自愈
           （实测 ``电机 #1: 检查未通过 (2010ms)`` 之后，``电机 #2`` 2ms 就通）。
           单次探测撞进去 = 误判整机故障。现重试 ``PROBE_ATTEMPTS`` 次、每次留间隔。
        """
        if self._chassis is None or not self._chassis.is_open:
            return False
        for attempt in range(1, self.PROBE_ATTEMPTS + 1):
            try:
                self._chassis.drain_lines()
                if self._chassis.send_ping(timeout=self.PROBE_TIMEOUT_S):
                    if attempt > 1:
                        logger.info(
                            f"底盘连通性探测第 {attempt} 次成功（前 {attempt - 1} 次"
                            f"撞上下位机接收静默窗口，属已知偶发）")
                    return True
            except Exception as e:
                logger.warning(f"底盘连通性探测第 {attempt} 次异常: {e}")
            if attempt < self.PROBE_ATTEMPTS:
                time.sleep(self.PROBE_GAP_S)
        return False

    def check_battery_voltage(self) -> float:
        # 电池电压需真机（串口 TEL 或 ADC 分压），未实现返回 -1
        return -1.0

    def get_motor_count(self) -> int:
        return 2  # 2 个驱动电机（差速）


# ============================================================
# 自检流程
# ============================================================

class SystemChecker:
    """
    系统自检器。

    按顺序检查所有子系统，收集结果并生成报告。
    关键项失败 → 整体不通过。
    """

    def __init__(self, hardware: HardwareChecker):
        self._hw = hardware
        self._items: List[CheckItem] = []

    def run(self) -> CheckReport:
        """执行完整自检流程"""
        t_start = time.time()
        self._items.clear()

        logger.info("=" * 40)
        logger.info("开始系统自检...")
        logger.info("=" * 40)

        # 第 1 步：传感器检查
        self._check_sensors()

        # 第 2 步：电机检查
        self._check_motors()

        # 第 3 步：电源检查
        self._check_power()

        total_ms = (time.time() - t_start) * 1000

        # 判定整体结果：所有关键项都通过才算通过
        critical_failures = [i for i in self._items
                             if i.status == CheckStatus.FAIL and i.critical]
        overall_pass = len(critical_failures) == 0

        report = CheckReport(
            items=list(self._items),
            overall_pass=overall_pass,
            total_duration_ms=total_ms,
        )

        if overall_pass:
            logger.info(f"✅ 系统自检通过！({self._items.__len__()} 项检查, {total_ms:.0f}ms)")
        else:
            failed_names = [i.name for i in critical_failures]
            logger.error(f"❌ 系统自检失败！关键项未通过: {failed_names}")

        return report

    def _check_sensors(self) -> None:
        """检查所有传感器"""
        logger.info("--- 传感器检查 ---")

        # 摄像头
        item = self._run_check("摄像头", self._hw.check_camera, critical=True)
        self._items.append(item)

        # IMU
        # ⚠️ 默认**非致命**（只报警不阻断），与固件的降级策略保持一致：
        #    固件侧 `MPU6050_Init` 失败已改为"在无 IMU 的情况下继续运行"
        #    （遥测里 IMU 行自动跳过，ODOM/TEL 照发），定位退化为**纯轮式里程计**
        #    （少了陀螺仪修正，长距离会漂）。上位机若仍在这里硬阻断，
        #    就出现"固件允许跑、上位机不让跑"的自相矛盾，整场直接报废。
        #    实测 2026-09-17：串口只剩 ODOM/TEL，IMU 行数为 0 → 自检 FAIL → 车拒绝启动。
        #    需要恢复"没有 IMU 就不许启动"的严格行为时：设 REQUIRE_IMU=1。
        _imu_critical = os.environ.get("REQUIRE_IMU", "").strip() not in ("", "0", "false", "no")
        item = self._run_check("IMU", self._hw.check_imu, critical=_imu_critical)
        if item.status == CheckStatus.FAIL and not _imu_critical:
            logger.warning("⚠️⚠️ IMU 未检出：本场将**只用轮式里程计定位**（无陀螺仪修正），"
                           "长距离/多次转向后航向会漂 → 落点精度下降。"
                           "请优先检查 MPU6050 接线（SCL→PB10 / SDA→PB11 / VCC / GND）。"
                           "如需严格模式（无 IMU 拒绝启动）请设 REQUIRE_IMU=1")
        self._items.append(item)

        # 温度传感器（非关键）
        item = self._run_check("温度传感器", self._hw.check_temperature_sensor, critical=False)
        self._items.append(item)

    def _check_motors(self) -> None:
        """检查所有电机"""
        logger.info("--- 电机检查 ---")

        motor_count = self._hw.get_motor_count()
        for i in range(motor_count):
            item = self._run_check(
                f"电机 #{i + 1}",
                lambda idx=i: self._hw.check_motor(idx),
                critical=True,
            )
            self._items.append(item)

    def _check_power(self) -> None:
        """检查电源"""
        logger.info("--- 电源检查 ---")

        def _check() -> bool:
            voltage = self._hw.check_battery_voltage()
            if voltage < 0:
                # 本车未接电压传感器 → 视为"未知"，不计为失败（原实现直接 FAIL 会卡住启动）
                logger.info("电池电压: 未接电压传感器（未知），跳过该项")
                return True
            if voltage < thresholds.BATTERY_MIN_VOLTAGE:
                logger.warning(f"电池电压偏低: {voltage:.1f}V (最低 {thresholds.BATTERY_MIN_VOLTAGE}V)")
                return False
            logger.info(f"电池电压: {voltage:.1f}V")
            return True

        item = self._run_check("电池电压", _check, critical=True)
        self._items.append(item)

    @staticmethod
    def _run_check(name: str, check_fn: Callable[[], bool], critical: bool) -> CheckItem:
        """执行单个检查项并计时"""
        item = CheckItem(name=name, status=CheckStatus.RUNNING, critical=critical)
        t0 = time.time()

        try:
            result = check_fn()
            item.duration_ms = (time.time() - t0) * 1000
            if result:
                item.status = CheckStatus.PASS
                item.message = "正常"
            else:
                item.status = CheckStatus.FAIL
                item.message = "检查未通过"
        except Exception as e:
            item.duration_ms = (time.time() - t0) * 1000
            item.status = CheckStatus.FAIL
            item.message = f"异常: {e}"

        log_fn = logger.info if item.status == CheckStatus.PASS else logger.error
        log_fn(f"  [{item.status.name}] {name}: {item.message} ({item.duration_ms:.0f}ms)")
        return item
