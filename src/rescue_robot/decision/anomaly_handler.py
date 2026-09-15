"""
anomaly_handler.py —— 异常检测与处理

检测并处理运行中的异常情况：
  1. 15 秒无动作 → 本轮结束
  2. 失控检测 → 紧急停止
  3. 传感器故障 → 降级运行
  4. 卡死检测 → 脱困尝试
"""

import logging
import math
import time
from dataclasses import dataclass, field
from enum import Enum, auto
from typing import Dict, List, Optional, Tuple

logger = logging.getLogger("anomaly_handler")


# ============================================================
# 异常类型与恢复动作
# ============================================================

class AnomalyType(Enum):
    """异常类型"""
    NONE = auto()
    NO_ACTION_15S = auto()        # 15 秒无动作 — 本轮结束
    LOSS_OF_CONTROL = auto()      # 失控 — IMU 异常突增
    SENSOR_FAULT_CAMERA = auto()  # 摄像头故障
    SENSOR_FAULT_IMU = auto()     # IMU 故障
    SENSOR_FAULT_MOTOR = auto()   # 电机故障
    STUCK = auto()                # 卡死 — 5 秒位置不变
    COLLISION_STUCK = auto()      # 碰撞卡死 — 接触 > 10 秒


class RecoveryAction(Enum):
    """恢复动作"""
    NONE = auto()
    EMERGENCY_STOP = auto()       # 紧急停止
    DEGRADE_SENSORS = auto()      # 传感器降级
    ESCAPE_MANEUVER = auto()      # 脱困操作（后退+旋转）
    CONTINUE_REDUCED = auto()     # 降速继续
    ROUND_END = auto()            # 本轮结束


@dataclass
class AnomalyReport:
    """异常报告"""
    type: AnomalyType = AnomalyType.NONE
    detail: str = ""
    timestamp: float = 0.0
    recovery_action: RecoveryAction = RecoveryAction.NONE
    is_fatal: bool = False


# ============================================================
# 异常处理器
# ============================================================

class AnomalyHandler:
    """
    异常检测与处理器。

    每帧调用 check() 检测异常，返回报告。
    异常处理由 DecisionEngine 根据报告执行。
    """

    # 阈值
    WATCHDOG_TIMEOUT_S = 15.0        # 无动作超时（触发保活，不淘汰）
    WATCHDOG_WARN_S = 10.0           # 10s预警（触发探索）
    WATCHDOG_CRITICAL_S = 13.0       # 13s临界（触发保命）
    STUCK_TIME_S = 5.0               # 卡死判定时间
    STUCK_DISTANCE_MM = 30.0         # 卡死判定距离（此距离内无移动=卡死）
    #: 卡死检测：下发速度超过此值才算"要求它动"（mm/s）。
    STUCK_CMD_MIN_MM_S = 50.0
    #: 卡死检测：单帧位移下限（mm）。低于它视为"没有实际移动"，滤掉里程计抖动。
    #: ⚠️ 判据必须"下发速度"对比"里程计位移"（两路独立信息），见 check() 第 4 步。
    STUCK_STEP_MIN_MM = 1.0
    ACCEL_ANOMALY_THRESHOLD = 30.0   # 加速度异常阈值 (m/s²)
    GYRO_ANOMALY_THRESHOLD = 10.0    # 角速度异常阈值 (rad/s)

    def __init__(self):
        self._last_action_time = time.time()
        self._last_position: Optional[Tuple[float, float]] = None
        self._stuck_start_time: Optional[float] = None

        # 逃逸状态
        self._escape_phase = 0        # 脱困阶段（0=正常, 1=后退, 2=旋转, 3=前进）
        self._escape_start_time = 0.0

        # 传感器状态
        self._sensor_status: Dict[str, bool] = {
            "camera": True, "imu": True, "motor_left": True,
            "motor_right": True, "temp": True, "vibration": True,
        }

        self._anomaly_count: Dict[AnomalyType, int] = {
            t: 0 for t in AnomalyType
        }

        logger.info("AnomalyHandler 初始化")

    # ---- 主检测接口 ----

    def check(self,
              robot_pose: Tuple[float, float, float],
              velocity: Optional[Tuple[float, float]] = None,
              imu_data: Optional[dict] = None,
              contact_duration_s: float = 0.0,
              sensor_status: Optional[Dict[str, bool]] = None,
              commanded_velocity: Optional[Tuple[float, float]] = None) -> AnomalyReport:
        """
        综合异常检测。

        Args:
            robot_pose: (x, y, theta)
            velocity: **实际**速度 (vx, vy) mm/s（由里程计位姿差算出）。
                None = 上游没有运动学信息 → 跳过"无动作"检测（绝不凭猜测判无动作）。
            commanded_velocity: 本帧**下发**的速度 (vx, vy) mm/s（导航的速度指令）。
                卡死检测需要它：只有"下发了速度"且"里程计不动"才是真卡死。
                None → 跳过卡死检测（仿真/单测默认，绝不凭空判卡死）。
        Returns:
            AnomalyReport: 异常报告（无异常时 type=NONE）
        """
        if sensor_status:
            self._sensor_status.update(sensor_status)

        timestamp = time.time()
        rx, ry, rtheta = robot_pose
        # 兜底初始化（N-1）：下方卡死检测在 `velocity is not None` 分支**之外**
        # 也会用到 speed；当前已在守卫内使用，但显式初始化可防止日后有人挪动判断
        # 顺序时出现 NameError（velocity=None 时决策引擎默认就会这么调）。
        speed = 0.0

        # 1. "无动作"检测（仅在拿到真实速度反馈时启用）
        # 旧实现由调用方传字面量 (0,0) → speed 恒 0；且调用方每帧 notify_action()
        # 刷新计时 → 这一段永远不触发，整条保活链形同虚设。现已改为由上游传实际速度。
        if velocity is not None:
            vx, vy = velocity
            speed = math.sqrt(vx ** 2 + vy ** 2)
            idle_s = timestamp - self._last_action_time
            if speed > 10:  # 确实在动
                self._last_action_time = timestamp
            elif idle_s > self.WATCHDOG_TIMEOUT_S:
                return self._report(
                    AnomalyType.NO_ACTION_15S,
                    "无动作 %.1fs → 保命模式" % idle_s,
                    RecoveryAction.EMERGENCY_STOP, False,  # 不再fatal!
                )
            elif idle_s > self.WATCHDOG_CRITICAL_S:
                return self._report(
                    AnomalyType.NO_ACTION_15S,
                    "无动作 %.1fs → 保命预警" % idle_s,
                    RecoveryAction.ESCAPE_MANEUVER, False,
                )
            elif idle_s > self.WATCHDOG_WARN_S:
                return self._report(
                    AnomalyType.NO_ACTION_15S,
                    "无动作 %.1fs → 探索预警" % idle_s,
                    RecoveryAction.ESCAPE_MANEUVER, False,
                )

        # 2. 失控检测（IMU 异常）
        if imu_data:
            accel = imu_data.get("accel_magnitude", 0)
            gyro = imu_data.get("gyro_magnitude", 0)
            if accel > self.ACCEL_ANOMALY_THRESHOLD:
                return self._report(
                    AnomalyType.LOSS_OF_CONTROL,
                    f"加速度异常: {accel:.1f} m/s²",
                    RecoveryAction.EMERGENCY_STOP, True,
                )
            if gyro > self.GYRO_ANOMALY_THRESHOLD:
                return self._report(
                    AnomalyType.LOSS_OF_CONTROL,
                    f"角速度异常: {gyro:.1f} rad/s",
                    RecoveryAction.EMERGENCY_STOP, True,
                )

        # 3. 传感器故障
        if not self._sensor_status.get("camera", True):
            return self._report(
                AnomalyType.SENSOR_FAULT_CAMERA,
                "摄像头无数据",
                RecoveryAction.DEGRADE_SENSORS, False,
            )
        if not self._sensor_status.get("imu", True):
            return self._report(
                AnomalyType.SENSOR_FAULT_IMU,
                "IMU 无数据",
                RecoveryAction.DEGRADE_SENSORS, False,
            )

        # 4. 卡死检测（"下发了速度但里程计不动"）
        #
        # ⚠️⚠️ 必须用**两路独立信息**：① 本帧下发的速度（导航要它动）
        #     ② 里程计**实际位移**（它到底动没动）。
        #
        # 旧实现（T0-2，已修）拿"由位姿差算出的 speed"与"当前帧位姿差 dist"相比
        # —— 两者是**同一个量的相邻两帧**，于是 dt=0.02 时"每帧位移落在 (2,30)mm"
        # 就同时满足 `speed>100` 与 `dist<30`，而那正是 **100~1500mm/s 的正常行驶**：
        # 实测 300mm/s 正常直行，**t=5.03s 被判"卡死 5.0s"**（车实际走了 1878mm）。
        # 该误判随后会经 ANOMALY 状态把决策层永久锁死（T0-3）→ 整场报废。
        #
        # 现在：只有"确实下发了速度"且"里程计几乎没动"才累计卡死时间。
        # `commanded_velocity=None`（仿真/单测未提供）→ 跳过本检测，绝不凭空判卡死。
        current_pos = (rx, ry)
        if (commanded_velocity is not None and velocity is not None
                and self._last_position is not None):
            cmd_speed = math.sqrt(commanded_velocity[0] ** 2
                                  + commanded_velocity[1] ** 2)
            dist = math.sqrt(
                (current_pos[0] - self._last_position[0]) ** 2 +
                (current_pos[1] - self._last_position[1]) ** 2
            )
            wants_to_move = cmd_speed > self.STUCK_CMD_MIN_MM_S
            actually_moved = dist >= self.STUCK_STEP_MIN_MM
            if wants_to_move and not actually_moved:
                # 要求它动、里程计却不动 → 真的卡住（打滑/顶住/编码器失效）
                if self._stuck_start_time is None:
                    self._stuck_start_time = timestamp
                elif timestamp - self._stuck_start_time > self.STUCK_TIME_S:
                    return self._report(
                        AnomalyType.STUCK,
                        f"卡死 {timestamp - self._stuck_start_time:.1f}s"
                        f"（下发 {cmd_speed:.0f}mm/s，里程计位移 {dist:.2f}mm/帧）",
                        RecoveryAction.ESCAPE_MANEUVER, False,
                    )
            else:
                self._stuck_start_time = None

        self._last_position = current_pos

        # 5. 碰撞卡死（接触 > 10 秒 → 强制分离已在 opponent_tracker 处理）
        if contact_duration_s > 10.0:
            return self._report(
                AnomalyType.COLLISION_STUCK,
                f"接触 {contact_duration_s:.1f}s",
                RecoveryAction.ESCAPE_MANEUVER, False,
            )

        return self._report(AnomalyType.NONE, "", RecoveryAction.NONE, False)

    # ---- 恢复动作 ----

    def get_escape_command(self, dt: float = 0.02) -> Tuple[float, float]:
        """
        获取脱困速度指令。

        脱困序列：
          阶段 0（0-1s）：后退 200mm/s
          阶段 1（1-2s）：原地旋转 90°
          阶段 2（2-3s）：前进 300mm/s
          完成后复位
        """
        elapsed = time.time() - self._escape_start_time

        if self._escape_phase == 0:
            if elapsed < 1.0:
                return (-200.0, 0.0)  # 后退
            else:
                self._escape_phase = 1
                self._escape_start_time = time.time()

        if self._escape_phase == 1:
            if elapsed < 1.5:
                return (0.0, 1.5)  # 旋转
            else:
                self._escape_phase = 2
                self._escape_start_time = time.time()

        if self._escape_phase == 2:
            if elapsed < 2.0:
                return (300.0, 0.0)  # 前进
            else:
                # 脱困完成
                self._escape_phase = 0
                self._stuck_start_time = None
                logger.info("脱困完成")

        return (0.0, 0.0)

    def start_escape(self) -> None:
        """开始脱困"""
        self._escape_phase = 0
        self._escape_start_time = time.time()
        logger.info("开始脱困程序: 后退 → 旋转 → 前进")

    def is_escaping(self) -> bool:
        return self._escape_phase > 0 or (
            self._escape_phase == 0 and
            time.time() - self._escape_start_time < 0.1
        )

    # ---- 反馈 ----

    def notify_action(self) -> None:
        """通知有动作（重置看门狗）"""
        self._last_action_time = time.time()

    def notify_sensor_ok(self, sensor_name: str) -> None:
        """通知传感器恢复正常"""
        self._sensor_status[sensor_name] = True

    # ---- 查询 ----

    def get_sensor_status(self) -> Dict[str, bool]:
        return dict(self._sensor_status)

    def get_anomaly_count(self, anomaly_type: AnomalyType) -> int:
        return self._anomaly_count.get(anomaly_type, 0)

    # ---- 内部 ----

    def get_idle_duration(self) -> float:
        """获取当前空闲时长"""
        return time.time() - self._last_action_time

    def keep_moving_fallback(self, robot_pose) -> tuple:
        """保命动作：根据空闲时长返回移动指令"""
        idle_s = self.get_idle_duration()
        rx, ry, rtheta = robot_pose
        if idle_s > 13:
            # 紧急：大圈移动
            angle = time.time() % (2 * 3.14159)
            return (300.0, 0.5)  # 前进+微转
        elif idle_s > 10:
            # 探索：向场地中央
            return (400.0, (1500 - rx) * 0.001)
        return (200.0, 0.0)

    def _report(self, anomaly_type: AnomalyType, detail: str,
                recovery: RecoveryAction, fatal: bool) -> AnomalyReport:
        if anomaly_type != AnomalyType.NONE:
            self._anomaly_count[anomaly_type] += 1
            log_fn = logger.error if fatal else logger.warning
            log_fn(f"异常 [{anomaly_type.name}]: {detail} "
                   f"(恢复={recovery.name}, 致命={fatal})")

        return AnomalyReport(
            type=anomaly_type,
            detail=detail,
            timestamp=time.time(),
            recovery_action=recovery,
            is_fatal=fatal,
        )
