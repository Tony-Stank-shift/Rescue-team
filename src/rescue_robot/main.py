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
from typing import Optional

from .config import RunMode
from .state_machine import StateMachine, RobotState
from .system_check import SystemChecker, MockHardwareChecker
from .states.boot_state import BootState
from .states.debug_state import DebugState
from .states.autonomous_state import AutonomousState
from .hardware.button import MockButton, GPIOButton
from .hardware.indicator import MockIndicator, LEDIndicator
from .perception.field_elements import FieldLayout, SafeZoneColor, StandardFieldLayout
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
        # 按钮/状态灯在下位机(F103)：上位机经串口交互（EVENT,START_BUTTON / 下位机 LED），不经 GPIO
        button = MockButton()
        indicator = MockIndicator()
        cam_idx = get_camera_index()
        hw_checker = RealHardwareChecker(chassis=None, camera_index=cam_idx)
        logger.info("已创建真实硬件检查器（真机就绪后补全自检读取）")
    else:
        raise ValueError(f"未知运行模式: {mode}，可选: {RunMode.MOCK}, {RunMode.REAL}")

    return button, indicator, hw_checker


# ============================================================
# 现场配置硬校验（U5）
# ============================================================

#: TEAM_COLOR 的合法写法 → 本队安全区颜色
_TEAM_RED_ALIASES = {"red", "r", "红", "红队", "hong", "red_team", "hongdui"}
_TEAM_BLUE_ALIASES = {"blue", "b", "蓝", "蓝队", "lan", "blue_team", "landui"}

#: 模块级 logger：下面的校验函数在 main() 之外，不能用 main() 里的局部 logger
logger = logging.getLogger(__name__)


#: 摄像头索引默认值（**唯一来源**）。T0-8：曾出现自检用 1、采集用 0 的不一致，
#: 而摄像头是 BOOT 自检的**关键项** → 单摄枚举在 0 时"自检失败 → 上电即 ERROR → 程序退出"。
_DEFAULT_CAM_INDEX = 0


def default_config_path() -> str:
    """配置文件的**稳定路径**（T2-9）。

    旧实现用裸相对路径 "config/robot.default.yaml"：从别的目录启动（`sudo` 从 `/`、
    systemd 无 WorkingDirectory、或 `cd ~` 后跑 `python3 -m rescue_robot.main`）
    就会走到两个坏结果之一：① `validate_field_config()` 报"找不到配置文件"→
    **拒绝启动**（而日志把人指向"改配置/TEAM_COLOR"，真正原因却是 cwd）；
    ② `RobotConfig.from_yaml()` 那处被 except 吞成 warning → **YAML 调参静默失效**。
    现在：优先当前目录（兼容老用法），否则按 `__file__` 定位仓库根的 config/。
    """
    rel = os.path.join("config", "robot.default.yaml")
    if os.path.isfile(rel):
        return rel
    # main.py = <root>/src/rescue_robot/main.py → 上溯三层到仓库根
    here = os.path.dirname(os.path.abspath(__file__))
    root = os.path.dirname(os.path.dirname(here))
    cand = os.path.join(root, rel)
    return cand


def get_camera_index() -> int:
    """摄像头索引的唯一取值入口（自检与采集必须用同一个值）。"""
    try:
        return int(os.environ.get("CAM_INDEX", str(_DEFAULT_CAM_INDEX)))
    except ValueError:
        logger.warning(f"CAM_INDEX 不是数字 → 回退 {_DEFAULT_CAM_INDEX}")
        return _DEFAULT_CAM_INDEX


def resolve_team_color(raw: Optional[str]):
    """把 TEAM_COLOR 的现场写法解析成本队安全区颜色。

    Returns:
        (SafeZoneColor 或 None, 问题描述或 None)

    ⚠️ 为什么必须严格：旧实现是 `== "red" ? RED : BLUE`，**任何拼写错误都会被静默
    当成 BLUE**。而颜色判错 = 全场物资运进**对方**安全区 = 违规 + 0 有效分，
    是"一次性报废整场"级别。所以无法识别时必须返回问题、由调用方拒绝启动，
    绝不默认成 BLUE。
    """
    from .perception.field_elements import SafeZoneColor
    text = (raw if raw is not None else "red").strip().lower()
    if not text:
        # 显式空值（如 `export TEAM_COLOR=`）不含颜色意图 → 按文档默认 red 处理，
        # 但要**大声告警**，避免"以为设了其实没设"。
        logger.warning("TEAM_COLOR 为空 → 按默认 red 处理，请确认现场本队安全区是红色！")
        return SafeZoneColor.RED, None
    if text in _TEAM_RED_ALIASES:
        return SafeZoneColor.RED, None
    if text in _TEAM_BLUE_ALIASES:
        return SafeZoneColor.BLUE, None
    return None, (f"TEAM_COLOR={raw!r} 无法识别（合法值："
                  f"red/红/r/hong 或 blue/蓝/b/lan；不设置则默认 red）")


def validate_field_config(yaml_path: str = None):
    """校验"现场可改、改错就是整场 0 分"的配置项，并把它们真正接线。

    覆盖两项（U5）：
      ① `TEAM_COLOR` —— 严格解析，无法识别即报问题（旧实现静默当 BLUE）；
      ② `perception.target_color_map` —— **真正生效**（旧实现零消费点：现场改了
         YAML 既不生效也不报错，目标会按旧颜色判 → 判不出来 → 整场 0 分）。

    Returns:
        (team_color, problems:list[str])；problems 非空时调用方应**拒绝启动**。
    """
    problems: list = []
    if yaml_path is None:
        yaml_path = default_config_path()

    # ① YAML 颜色映射：接线 + 校验
    try:
        from .perception.target_types import set_color_override
        import yaml as _yaml
        with open(yaml_path, encoding="utf-8") as f:
            raw = _yaml.safe_load(f) or {}
        cmap = ((raw.get("perception") or {}).get("target_color_map")) or None
        for p in set_color_override(cmap):
            problems.append(f"perception.target_color_map: {p}")
    except FileNotFoundError:
        problems.append(f"找不到配置文件 {yaml_path}（已尝试当前目录与仓库根；"
                        f"若你是从其它目录启动，请检查部署目录是否完整）")
    except Exception as e:
        problems.append(f"perception.target_color_map 校验失败: {e!r}")

    # ② TEAM_COLOR：严格解析
    color, err = resolve_team_color(os.environ.get("TEAM_COLOR"))
    if err:
        problems.append(err)

    return color, problems


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
    # ⚠️ T0-9：默认必须是 **real**。旧实现默认 `mock` → 现场漏设 RUN_MODE 时
    # **一条 VEL 都不会发**（chassis=None、感知也走 Mock），而日志首行却是正常的
    # "🔧 运行模式: MOCK" → 表现是"程序完全正常、有日志、有决策输出但车不动"，
    # 全场 0 分且极难现场定位。"一个不肯启动的机器人"远比"一个静静不动的机器人"好。
    run_mode = os.environ.get("RUN_MODE", RunMode.REAL)
    if run_mode == RunMode.MOCK:
        logger.error("=" * 68)
        logger.error("⚠️  RUN_MODE=mock（Mock 模式）：**不会驱动真实底盘、也不会读摄像头**！")
        logger.error("    仅用于本机开发/仿真。真机比赛请设 RUN_MODE=real。")
        logger.error("=" * 68)
    logger.info(f"运行模式: {run_mode}")

    # 加载配置文件（决赛创新实践环节现场修改 YAML 即可，无需重编译）
    try:
        from .innovation.config_loader import RobotConfig
        from . import config as _config
        robot_cfg = RobotConfig.from_yaml(default_config_path())
        _config.apply_robot_config(robot_cfg)
        logger.info(f"已加载配置文件: {default_config_path()}")
    except Exception as e:
        logger.warning(f"配置文件加载失败，使用默认参数: {e}")

    # ── 现场配置硬校验（U5）── 详见 validate_field_config() 的说明
    _team_color, _fatal_cfg_problems = validate_field_config()
    if _fatal_cfg_problems:
        logger.error("=" * 68)
        logger.error("❌ 现场配置有误，**拒绝启动**（继续跑下去大概率整场 0 分）：")
        for _p in _fatal_cfg_problems:
            logger.error(f"   · {_p}")
        logger.error("   处置：改正 config/robot.default.yaml 与 TEAM_COLOR 环境变量后重试。")
        logger.error("=" * 68)
        return 1
    logger.info(f"本队安全区颜色 = {_team_color.name}"
                f"（TEAM_COLOR={os.environ.get('TEAM_COLOR', 'red')!r}）"
                f"  ⚠️ 请与现场实际颜色核对：判错 = 全场物资运进对方安全区")

    # 提前声明：保证 finally 清理时一定存在（异常早退也不会 NameError）
    chassis = None
    camera = None

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
        # 本队安全区颜色：已在上方做严格校验与归一（U5），这里直接用。
        # 旧实现 `== "red" ? RED : BLUE` 会把任何拼写错误静默当 BLUE
        # → 全场物资运进对方安全区（违规 + 0 有效分）。
        my_color = _team_color
        # 抽签得到的出发区（1~4，默认 3）；决定全场坐标系原点与朝向
        try:
            start_zone = int(os.environ.get("START_ZONE", "3"))
        except ValueError:
            logger.warning("START_ZONE 不是数字，回退 3 号出发区")
            start_zone = 3
        if not 1 <= start_zone <= 4:
            logger.warning(f"START_ZONE={start_zone} 非法（应为 1~4），回退 3 号出发区")
            start_zone = 3
        start_pose = StandardFieldLayout().get_start_pose(start_zone)
        logger.info(f"抽签出发区 = {start_zone} 号 → 起点 {start_pose}")

        perception = PerceptionPipeline(use_mock=use_mock, my_safe_zone_color=my_color)
        navigation = NavigationPipeline(field_layout, my_color=my_color, use_mock=use_mock)
        decision = DecisionEngine(perception.world_map, my_color=my_color)
        comm_manager = CommManager(state_machine=sm)

        # 创建串口底盘驱动（真机联调：电脑 USB-TTL 或 RDK 的 UART）
        chassis = None
        camera = None
        if not use_mock:
            from .hardware.serial_chassis import SerialChassis
            # 默认端口：RDK 板载 40PIN UART1 = /dev/ttyS1（已实测跑通 ODOM/IMU/TEL 与 VEL/STOP）；
            # 电脑 USB-TTL 调试时用 CHASSIS_PORT=/dev/ttyUSB0 覆盖。
            chassis = SerialChassis(port=os.environ.get("CHASSIS_PORT", "/dev/ttyS1"))
            # 出发区初始位姿（真机原点），与导航定位器同步在 AutonomousState.on_enter 里强制执行
            if start_pose is not None:
                chassis.set_start_pose(*start_pose)
            # 必须显式 open()：否则 is_open=False，自检的 IMU/电机 会误判失败，且后续无法下发 VEL
            if not chassis.open():
                # T0-10：真机路径必须 fail-fast。旧实现只打一行 ERROR 就继续 →
                # ① chassis 仍非 None，舵机命令静默无效；
                # ② main.py 的串口一键启动判断带 is_open → 永远进不了 AUTONOMOUS；
                # ③ 表现是"程序在跑但车一动不动"，现场只有一行日志可查。
                logger.critical("=" * 68)
                logger.critical(f"❌ 串口打开失败: {chassis._port} → **拒绝启动**")
                logger.critical("   检查：接线 / 下位机上电 / 当前用户是否在 dialout 组 / 端口是否被占用")
                logger.critical("   临时排查：ls -l /dev/ttyS1 ; groups ; sudo fuser -v /dev/ttyS1")
                logger.critical("=" * 68)
                return 1
            logger.info(f"已创建串口底盘驱动: {chassis._port} @ {chassis._baudrate} "
                        f"(is_open={chassis.is_open})")
            # 创建摄像头：用后台采集线程（CameraReader），避免主循环被 read() 阻塞。
            # 旧实现直接在 50Hz 主循环里 cv2.VideoCapture.read()，摄像头未就绪/掉线时会卡死主循环。
            try:
                from .hardware.camera_reader import CameraReader
                cam_idx = get_camera_index()
                cam = CameraReader(cam_idx)
                if cam.start():
                    # 预热：等首帧（超时则视为不可用 → 降级 Mock）
                    warmup = float(os.environ.get("CAM_WARMUP_S", "3.0"))
                    if cam.wait_first_frame(timeout=warmup):
                        logger.info(f"摄像头 {cam_idx} 就绪（首帧已到）")
                        camera = cam
                    else:
                        logger.warning(
                            f"摄像头 {cam_idx} {warmup:.0f}s 内未出帧，感知降级为 Mock")
                        cam.stop()
                        camera = None
                else:
                    logger.warning(f"摄像头 {cam_idx} 打开失败，感知将退化为 Mock")
                    camera = None
            except Exception as e:
                logger.warning(f"创建摄像头失败: {e}")
                camera = None

        # 摄像头失败 → **不再降级为 Mock**（T0-5）
        # 旧实现 `PerceptionPipeline(use_mock=True)` 会换成 MockDetector：它**伪造**
        # 20 个随机目标（每帧 ±2px 抖动），且没有 `estimate_ground_position`，
        # 走兜底后目标永远落在"车前 200~1000mm、横向 ±500mm"→ 机器人会一路导航+套取
        # **幻影目标**，整场拿不到真实目标（0 分），还可能撞紫边/撞对手。
        # 现在：保持真实感知管线，只是没有帧 → 检测为空 → 世界地图为空 →
        # 决策层原地等待/安全停车（宁可不动，也不要乱跑），并打显式故障标志。
        if camera is None and not use_mock:
            logger.critical("⚠️ 无可用摄像头：保持**真实**感知管线（不降级为 Mock，"
                            "Mock 会伪造 20 个随机目标导致整场追幻影）。"
                            "无帧 → 空地图 → 原地等待/安全停车。请修好摄像头后重启。")
            try:
                perception.set_vision_available(False)
            except Exception as e:
                logger.warning(f"设置视觉故障标志失败: {e}")

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
            start_zone=start_zone,
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
            # 一键启动按钮在下位机(F103)：DEBUG 等待启动时轮询 EVENT,START_BUTTON → 触发 one_key_start
            if current_state == RobotState.DEBUG and chassis is not None and chassis.is_open:
                if chassis.read_button():
                    logger.info("收到一键启动按钮事件 (EVENT,START_BUTTON)")
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
        # 停掉摄像头采集线程（否则线程/设备句柄泄漏）
        try:
            if camera is not None and hasattr(camera, "stop"):
                camera.stop()
        except Exception:
            pass
        # 关闭串口（释放 /dev/ttyS1，避免下次启动被占用）
        try:
            if chassis is not None and hasattr(chassis, "close"):
                chassis.close()
        except Exception:
            pass
        logger.info("机器人已安全停止。")

    return 0


if __name__ == "__main__":
    sys.exit(main())
