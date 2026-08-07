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
        button = GPIOButton(Pin.BUTTON_START)
        indicator = LEDIndicator(Pin.LED_GREEN, Pin.LED_RED, Pin.BUZZER)
        # TODO: 替换为真实 HardwareChecker
        hw_checker = MockHardwareChecker()
        logger.warning("⚠️  硬件检查器仍为 Mock，请实现真实 HardwareChecker")
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

    try:
        # 创建硬件实例
        button, indicator, hw_checker = _create_hardware(run_mode)

        # 创建状态机
        sm = StateMachine()

        # 创建系统自检器
        system_checker = SystemChecker(hw_checker)

        # 创建三个状态处理器
        boot_state = BootState(sm, system_checker, indicator)
        debug_state = DebugState(sm, button, indicator)
        autonomous_state = AutonomousState(sm, indicator)

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
