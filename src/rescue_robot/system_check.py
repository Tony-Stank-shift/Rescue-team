"""
system_check.py —— 系统自检

在 BOOT 状态下运行，逐一检查所有关键子系统：
  1. 传感器数据流（摄像头、IMU、温度等）
  2. 电机驱动（每个电机测试正反转）
  3. 电源电压（电池是否在安全范围）

所有检查项设计为平台无关的抽象接口，方便 Mock 测试。
"""

import logging
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
        return 4  # 4 个驱动电机


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
        item = self._run_check("IMU", self._hw.check_imu, critical=True)
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
                return False
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
