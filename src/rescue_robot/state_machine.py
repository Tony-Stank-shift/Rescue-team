"""
state_machine.py —— 核心状态机

管理救援机器人的三种运行状态：
  BOOT        — 上电自检
  DEBUG       — 调试配置模式（允许与笔记本通信）
  AUTONOMOUS  — 全自主运行（一键启动后，不可逆）
  ERROR       — 自检失败 / 运行错误

状态转移规则：
  BOOT  ──(自检通过)──→ DEBUG
  BOOT  ──(自检失败)──→ ERROR
  DEBUG ──(一键启动)──→ AUTONOMOUS  [不可逆！]
  ANY   ──(紧急停止)──→ ERROR
"""

import logging
import threading
from enum import Enum, auto
from typing import Optional, Callable, Dict

logger = logging.getLogger("state_machine")


class RobotState(Enum):
    """机器人运行状态枚举"""
    BOOT = auto()           # 上电自检中
    DEBUG = auto()          # 调试配置模式
    AUTONOMOUS = auto()     # 全自主运行（不可逆）
    ERROR = auto()          # 错误/紧急停止


# 合法的状态转移（from → to）
_ALLOWED_TRANSITIONS: Dict[RobotState, frozenset] = {
    RobotState.BOOT:        frozenset({RobotState.DEBUG, RobotState.ERROR}),
    RobotState.DEBUG:       frozenset({RobotState.AUTONOMOUS, RobotState.ERROR}),
    RobotState.AUTONOMOUS:  frozenset({RobotState.ERROR}),  # 不可回退到 DEBUG
    RobotState.ERROR:       frozenset(),                     # 终态，需手动重启
}


class StateMachine:
    """
    救援机器人核心状态机。

    特性：
    - 状态转移校验，非法转移会抛出异常
    - AUTONOMOUS 状态下锁定，禁止回到 DEBUG
    - on_exit/on_enter 在锁外执行，避免阻塞
    - 线程安全的状态读写
    """

    def __init__(self):
        self._state = RobotState.BOOT
        self._lock = threading.Lock()
        self._state_handlers: Dict[RobotState, object] = {}
        self._on_state_change: Optional[Callable[[RobotState, RobotState], None]] = None

        # 一键启动标记（一旦置 True 不可复位）
        self._one_key_started = False

        # 外部输入锁定标志（AUTONOMOUS 状态置 True）
        self._external_inputs_locked = False

    # ---- 状态属性 ----

    @property
    def state(self) -> RobotState:
        """线程安全地读取当前状态"""
        with self._lock:
            return self._state

    @property
    def is_autonomous(self) -> bool:
        """是否已进入全自主模式（线程安全）"""
        with self._lock:
            return self._state == RobotState.AUTONOMOUS

    @property
    def is_locked(self) -> bool:
        """AUTONOMOUS 状态下所有外部输入被锁定（线程安全）"""
        return self._external_inputs_locked

    @property
    def one_key_started(self) -> bool:
        """一键启动是否已被触发"""
        return self._one_key_started

    # ---- 状态处理器注册 ----

    def register_handler(self, state: RobotState, handler: object) -> None:
        """注册状态处理器（需实现 on_enter / on_exit 方法）"""
        self._state_handlers[state] = handler
        logger.info(f"注册状态处理器: {state.name} → {handler.__class__.__name__}")

    def set_state_change_callback(self, callback: Callable[[RobotState, RobotState], None]) -> None:
        """设置状态变更回调（用于 LED 指示等外部组件）"""
        self._on_state_change = callback

    # ---- 状态转移 ----

    def transition(self, to_state: RobotState, reason: str = "") -> None:
        """
        执行状态转移。

        on_exit/on_enter 回调在锁外执行，避免长时间操作阻塞状态读取。

        Args:
            to_state: 目标状态
            reason: 转移原因（日志用）

        Raises:
            ValueError: 非法状态转移
        """
        old_handler = None
        new_handler = None

        with self._lock:
            from_state = self._state

            # 校验转移合法性
            if to_state not in _ALLOWED_TRANSITIONS.get(from_state, frozenset()):
                raise ValueError(
                    f"非法状态转移: {from_state.name} → {to_state.name}"
                    f" (允许的转移: {[s.name for s in _ALLOWED_TRANSITIONS.get(from_state, frozenset())]})"
                )

            # 额外安全检查：AUTONOMOUS 不能回退到 DEBUG
            if from_state == RobotState.AUTONOMOUS and to_state == RobotState.DEBUG:
                raise ValueError("AUTONOMOUS 状态下禁止回到 DEBUG 模式！")

            # 先切换状态（锁内只做状态变更，保证原子性）
            self._state = to_state
            old_handler = self._state_handlers.get(from_state)
            new_handler = self._state_handlers.get(to_state)

            logger.info(f"状态转移: {from_state.name} → {to_state.name}" +
                        (f" (原因: {reason})" if reason else ""))

        # ═══ 锁外执行回调（避免长时间阻塞） ═══

        # 退出旧状态
        if old_handler and hasattr(old_handler, "on_exit"):
            try:
                old_handler.on_exit()
            except Exception as e:
                logger.error(f"状态 {from_state.name} on_exit 异常: {e}")

        # 进入新状态
        if new_handler and hasattr(new_handler, "on_enter"):
            try:
                new_handler.on_enter()
            except Exception as e:
                logger.error(f"状态 {to_state.name} on_enter 异常: {e}")

        # 状态变更回调
        if self._on_state_change:
            try:
                self._on_state_change(from_state, to_state)
            except Exception as e:
                logger.error(f"状态变更回调异常: {e}")

    # ---- 便捷方法 ----

    def start(self) -> None:
        """
        启动状态机：触发初始 BOOT 状态的 on_enter。

        应在所有 handler 注册完成后调用。
        初始状态为 BOOT，调用后会执行自检并自动转移。
        """
        if self._state != RobotState.BOOT:
            raise ValueError(f"状态机已启动，当前状态: {self._state.name}")

        boot_handler = self._state_handlers.get(RobotState.BOOT)
        if boot_handler and hasattr(boot_handler, "on_enter"):
            logger.info("状态机启动 — 进入 BOOT 状态")
            # 锁外调用 on_enter（此时不可能有并发 transition）
            try:
                boot_handler.on_enter()
            except Exception as e:
                logger.error(f"BOOT on_enter 异常: {e}")

    def one_key_start(self) -> None:
        """
        一键启动：DEBUG → AUTONOMOUS。

        此操作不可逆！一旦调用：
        1. 机器人进入全自主模式
        2. 所有外部输入被锁定
        3. 不可回到 DEBUG 状态
        """
        if self._one_key_started:
            logger.warning("一键启动已被触发，忽略重复调用")
            return

        if self._state != RobotState.DEBUG:
            raise ValueError(f"只能在 DEBUG 状态下一键启动，当前状态: {self._state.name}")

        self._one_key_started = True
        self._external_inputs_locked = True
        logger.info("⚡ 一键启动触发！进入全自主运行模式（外部输入已锁定）")
        self.transition(RobotState.AUTONOMOUS, reason="一键启动")

    def emergency_stop(self, reason: str = "") -> None:
        """紧急停止：任意状态 → ERROR"""
        logger.critical(f"🛑 紧急停止！原因: {reason or '未知'}")
        try:
            self.transition(RobotState.ERROR, reason=f"紧急停止: {reason}")
        except ValueError:
            # 如果已经在 ERROR 或转移不合法，强制覆盖
            with self._lock:
                self._state = RobotState.ERROR
            logger.critical("强制进入 ERROR 状态")

    # ---- 调试 ----

    def get_status(self) -> dict:
        """获取状态机当前状态摘要"""
        with self._lock:
            state_name = self._state.name
        return {
            "state": state_name,
            "is_autonomous": self.is_autonomous,
            "is_locked": self.is_locked,
            "one_key_started": self._one_key_started,
        }
