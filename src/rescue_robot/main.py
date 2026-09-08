"""
main.py —— 救援机器人主入口

用法:
  # Mock 模式（本地开发/CI）
  python3 -m rescue_robot.main

  # 真实硬件模式
  RUN_MODE=real python3 -m rescue_robot.main

启动流程:
  1. 初始化硬件抽象层（Mock 或真实 GPIO）
  2. 创建状态机 + 三个状态处理器
  3. 调用 sm.start() → 触发 BOOT 状态 on_enter
  4. BOOT（自检）→ DEBUG（等待一键启动）→ AUTONOMOUS（全自主）
"""

import logging
import os
import signal
import sys
import threading
import time

from .config import RunMode
from .state_machine import StateMachine, RobotState
from .system_check import SystemChecker, MockHardwareChecker
from .states.boot_state import BootState
from .states.debug_state import DebugState
from .states.autonomous_state import AutonomousState
from .hardware.button import MockButton, GPIOButton
from .hardware.indicator import MockIndicator, LEDIndicator
from .perception.field_elements import FieldLayout, SafeZoneColor
from .perception.perception_pipeline import PerceptionPipeline
from .navigation.navigation_pipeline import NavigationPipeline
from .decision.decision_engine import DecisionEngine
from .transport.transport_pipeline import TransportPipeline
from .communication.comm_manager import CommManager


# ============================================================
# 日志配置
# ============================================================

def setup_logging(level: int = logging.INFO) -> None:
    """配置统一日志格式"""
    logging.basicConfig(
        level=level,
        format="%(asctime)s [%(levelname)-7s] %(name)s: %(message)s",
        datefmt="%H:%M:%S",
    )


# ============================================================
# 工厂函数：根据运行模式创建硬件实例
# ============================================================

def _create_hardware(mode: str):
    """根据运行模式创建硬件实例"""
    from .config import Pin

    if mode == RunMode.MOCK:
        logger = logging.getLogger("main")
        logger.info("🔧 运行模式: MOCK (本地开发)")
        button = MockButton()
        indicator = MockIndicator()
        hw_checker = MockHardwareChecker()
    elif mode == RunMode.REAL:
        logger = logging.getLogger("main")
        logger.info("🔧 运行模式: REAL (真实硬件)")
        from .system_check import RealHardwareChecker
        # 按钮/状态灯在下位机(F103)：上位机经串口交互（BUTTON,ON 事件 / 下位机 LED），不经 GPIO
        button = MockButton()
        indicator = MockIndicator()
        cam_idx = int(os.environ.get("CAM_INDEX", "1"))
        hw_checker = RealHardwareChecker(chassis=None, camera_index=cam_idx)
        logger.info("已创建真实硬件检查器（真机就绪后补全自检读取）")
    else:
        raise ValueError(f"未知运行模式: {mode}，可选: {RunMode.MOCK}, {RunMode.REAL}")

    return button, indicator, hw_checker


# ============================================================
# 主函数
# ============================================================

def main():
    """救援机器人主入口"""
    setup_logging(logging.INFO)
    logger = logging.getLogger("main")

    logger.info("=" * 50)
    logger.info("  智能救援机器人 — 启动中...")
    logger.info("=" * 50)

    # 确定运行模式
    run_mode = os.environ.get("RUN_MODE", RunMode.MOCK)
    logger.info(f"运行模式: {run_mode}")

    # 加载配置文件（决赛创新实践环节现场修改 YAML 即可，无需重编译）
    try:
        from .innovation.config_loader import RobotConfig
        from . import config as _config
        robot_cfg = RobotConfig.from_yaml("config/robot.default.yaml")
        _config.apply_robot_config(robot_cfg)
        logger.info("已加载 config/robot.default.yaml")
    except Exception as e:
        logger.warning(f"配置文件加载失败，使用默认参数: {e}")

    try:
        # 创建硬件实例
        button, indicator, hw_checker = _create_hardware(run_mode)

        # 创建状态机
        sm = StateMachine()

        # 创建系统自检器
        system_checker = SystemChecker(hw_checker)

        # 创建四大管线 + 通信（按运行模式使用 Mock/真实实现）
        use_mock = (run_mode == RunMode.MOCK)
        field_layout = FieldLayout.standard()
        my_color = (SafeZoneColor.RED
                    if os.environ.get("TEAM_COLOR", "red").lower() == "red"
                    else SafeZoneColor.BLUE)

        perception = PerceptionPipeline(use_mock=use_mock, my_safe_zone_color=my_color)
        navigation = NavigationPipeline(field_layout, my_color=my_color, use_mock=use_mock)
        decision = DecisionEngine(perception.world_map, my_color=my_color)
        comm_manager = CommManager(state_machine=sm)

        # 创建串口底盘驱动（真机联调：电脑 USB-TTL 或 RDK 的 UART）
        chassis = None
        camera = None
        if not use_mock:
            from .hardware.serial_chassis import SerialChassis
            chassis = SerialChassis(port=os.environ.get("CHASSIS_PORT", "/dev/ttyUSB0"))
            logger.info(f"已创建串口底盘驱动: {chassis._port} @ {chassis._baudrate}")
            # 创建摄像头（真机视觉；CAM_INDEX 可配，默认 1=外接 USB）
            try:
                import cv2
                cam_idx = int(os.environ.get("CAM_INDEX", "1"))
                camera = cv2.VideoCapture(cam_idx)
                if not camera.isOpened():
                    logger.warning(f"摄像头 {cam_idx} 打开失败，感知将退化为 Mock")
                    camera = None
                else:
                    logger.info(f"摄像头 {cam_idx} 打开成功")
            except Exception as e:
                logger.warning(f"创建摄像头失败: {e}")
                camera = None

        # 摄像头失败 → 感知降级为 Mock（避免 CVDetector 收 frame=None 崩溃）
        if camera is None and not use_mock:
            logger.warning("无可用摄像头，感知降级为 Mock（视觉不可用）")
            perception = PerceptionPipeline(use_mock=True, my_safe_zone_color=my_color)
            decision = DecisionEngine(perception.world_map, my_color=my_color)

        # 若使用真实硬件检查器，注入串口底盘（自检 IMU/电机连通用）
        if hasattr(hw_checker, "_chassis"):
            hw_checker._chassis = chassis

        # 转运管线：真机用串口舵机（发 SERVO 命令），Mock 用默认 MockSleeveLift
        if chassis is not None:
            from .transport.sleeve_lift import SerialServoLift
            transport = TransportPipeline(
                field_layout=field_layout, my_color=my_color, use_mock=False,
                sleeve=SerialServoLift(chassis),
            )
        else:
            transport = TransportPipeline(
                field_layout=field_layout, my_color=my_color, use_mock=True,
            )

        # 创建三个状态处理器
        boot_state = BootState(sm, system_checker, indicator)
        debug_state = DebugState(sm, button, indicator, comm_server=comm_manager)
        autonomous_state = AutonomousState(
            sm, indicator,
            perception=perception,
            decision=decision,
            navigation=navigation,
            transport=transport,
            chassis=chassis,
            camera=camera,
            field_layout=field_layout,
            my_color=my_color,
            use_mock=use_mock,
        )

        # 注册到状态机
        sm.register_handler(RobotState.BOOT, boot_state)
        sm.register_handler(RobotState.DEBUG, debug_state)
        sm.register_handler(RobotState.AUTONOMOUS, autonomous_state)

        # 注册状态变更回调
        def _on_state_change(from_state, to_state):
            logger.info(f"[状态变更] {from_state.name} → {to_state.name}")

        sm.set_state_change_callback(_on_state_change)

        # 处理 Ctrl+C
        shutdown_flag = threading.Event()

        def _sig_handler(signum, frame):
            logger.info("收到中断信号，正在安全停止...")
            shutdown_flag.set()
            sm.emergency_stop("用户中断 (Ctrl+C)")

        signal.signal(signal.SIGINT, _sig_handler)
        signal.signal(signal.SIGTERM, _sig_handler)

        # 启动按钮监听
        button.start_monitoring()

        # ═══════════════════════════════════════
        # 启动状态机流程
        # ═══════════════════════════════════════
        # sm.start() → 触发 BOOT.on_enter()
        #   → 自检通过 → 自动 transition 到 DEBUG
        #   → 自检失败 → 自动 transition 到 ERROR
        # DEBUG.on_enter() → 等待一键启动按钮
        #   → 按钮长按 → one_key_start() → AUTONOMOUS（不可逆）
        logger.info("状态机流程: BOOT → (自检) → DEBUG → (一键启动) → AUTONOMOUS")
        sm.start()

        # 等待结束
        while not shutdown_flag.is_set():
            current_state = sm.state
            if current_state == RobotState.ERROR:
                logger.error("状态机进入 ERROR，退出")
                break
            # 一键启动按钮在下位机(F103)：DEBUG 等待启动时轮询 BUTTON,ON → 触发 one_key_start
            if current_state == RobotState.DEBUG and chassis is not None and chassis.is_open:
                if chassis.read_button():
                    logger.info("收到一键启动按钮事件 (BUTTON,ON)")
                    sm.one_key_start()
                    continue
            if current_state == RobotState.AUTONOMOUS:
                # 已在主循环中运行，等待主循环结束或中断
                status = sm.get_status()
                logger.debug(f"自主运行中: {status}")
            time.sleep(0.5)

    except KeyboardInterrupt:
        logger.info("用户中断")
    except Exception as e:
        logger.error(f"未捕获异常: {e}", exc_info=True)
        return 1
    finally:
        # 清理
        logger.info("正在清理资源...")
        try:
            button.stop_monitoring()
        except Exception:
            pass
        try:
            indicator.cleanup()
        except Exception:
            pass
        logger.info("机器人已安全停止。")

    return 0


if __name__ == "__main__":
    sys.exit(main())
