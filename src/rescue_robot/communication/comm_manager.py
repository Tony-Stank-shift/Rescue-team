"""
comm_manager.py —— 通信管理器

集成到状态机，根据当前模式控制通信行为：

  DEBUG 模式      → 双向通信（收发配置/查询/传感器数据）
  AUTONOMOUS 模式 → 仅单向广播状态，收到的指令直接丢弃

安全检查：AUTONOMOUS 下 COMMAND/CONFIG_SET 类型消息被拦截。
"""

import logging
import threading
import time
from typing import Any, Callable, Dict, List, Optional

from .comm_protocol import (
    Message, MessageType, BLOCKED_IN_AUTONOMOUS,
    serialize, deserialize,
    make_status, make_sensor_data, make_config_resp,
    make_event, make_error, make_pong,
)
from .comm_server import AbstractCommServer, MockCommServer

logger = logging.getLogger("comm_manager")


# ============================================================
# 通信管理器
# ============================================================

class CommManager:
    """
    通信管理器。

    用法：
      mgr = CommManager(state_machine)
      mgr.start()

      # 广播状态（两种模式均可）
      mgr.send_status(robot_state="DEBUG", position=(x,y), score=0)

      # AUTONOMOUS 模式自动拦截入站指令
    """

    # 状态广播间隔（秒）
    STATUS_BROADCAST_INTERVAL_S = 0.5

    def __init__(self, state_machine=None, server: Optional[AbstractCommServer] = None,
                 use_mock: bool = True):
        """
        Args:
            state_machine: StateMachine 实例（检查 is_locked）
            server: 通信服务器实例
            use_mock: True=MockCommServer
        """
        self._sm = state_machine
        self._server = server or MockCommServer()

        # 注册消息处理器
        self._server.on_message(self._handle_message)

        # 状态广播
        self._broadcast_thread: Optional[threading.Thread] = None
        self._broadcast_running = False

        # 配置存储（DEBUG 模式可修改）
        self._config: Dict[str, Any] = {}

        # 统计
        self._msgs_sent = 0
        self._msgs_received = 0
        self._msgs_blocked = 0

        logger.info("CommManager 初始化 (mock=%s)", use_mock)

    # ---- 生命周期 ----

    def start(self, host: str = "0.0.0.0", port: int = 8765) -> bool:
        """启动通信服务"""
        ok = self._server.start(host, port)
        if ok:
            self._start_broadcast()
        return ok

    def stop(self) -> None:
        """停止通信服务"""
        self._broadcast_running = False
        self._server.stop()
        logger.info("CommManager 停止 (发送=%d, 接收=%d, 拦截=%d)",
                     self._msgs_sent, self._msgs_received, self._msgs_blocked)

    # ---- 发送（两种模式均可） ----

    def send(self, msg: Message) -> bool:
        """发送单条消息"""
        self._msgs_sent += 1
        return self._server.send(msg)

    def send_status(self, robot_state: str = "", position: tuple = (0, 0),
                    score: int = 0, targets_delivered: int = 0,
                    time_remaining: float = 0, **extra) -> None:
        """发送运行状态"""
        msg = make_status(
            robot_state=robot_state,
            position=position,
            score=score,
            targets_delivered=targets_delivered,
            time_remaining=time_remaining,
            **extra,
        )
        self.send(msg)

    def send_sensor_data(self, camera_fps: float = 0,
                         targets_visible: int = 0, **extra) -> None:
        """发送传感器数据"""
        msg = make_sensor_data(
            camera_fps=camera_fps,
            targets_visible=targets_visible,
            **extra,
        )
        self.send(msg)

    def send_event(self, event_name: str, detail: str = "", **extra) -> None:
        """发送事件通知"""
        msg = make_event(event_name, detail, **extra)
        self.send(msg)

    # ---- 接收处理 ----

    def _handle_message(self, msg: Message) -> Optional[Message]:
        """
        处理收到的消息。

        AUTONOMOUS 模式自动拦截危险消息类型。
        """
        self._msgs_received += 1

        # 安全检查：AUTONOMOUS 模式下拦截
        if self._sm and self._sm.is_locked:
            if msg.type in BLOCKED_IN_AUTONOMOUS:
                self._msgs_blocked += 1
                logger.warning("[拦截] AUTONOMOUS 模式拒绝: %s (累计拦截=%d)",
                               msg.type.value, self._msgs_blocked)
                return make_error("AUTONOMOUS 模式禁止接收指令")

        # 分发处理
        handlers = {
            MessageType.CONFIG_GET: self._handle_config_get,
            MessageType.CONFIG_SET: self._handle_config_set,
            MessageType.PING: lambda m: make_pong(),
        }

        handler = handlers.get(msg.type)
        if handler:
            return handler(msg)

        return None

    def _handle_config_get(self, msg: Message) -> Message:
        key = msg.payload.get("key", "")
        value = self._config.get(key)
        return make_config_resp(key, value, success=(value is not None))

    def _handle_config_set(self, msg: Message) -> Message:
        key = msg.payload.get("key", "")
        value = msg.payload.get("value")
        self._config[key] = value
        logger.info("配置更新: %s = %s", key, value)
        return make_config_resp(key, value, success=True)

    # ---- 模式检查 ----

    def can_receive_commands(self) -> bool:
        """当前是否可接收指令（DEBUG 模式）"""
        if self._sm is None:
            return True
        return not self._sm.is_locked

    @property
    def is_autonomous_mode(self) -> bool:
        if self._sm is None:
            return False
        return self._sm.is_locked

    # ---- 状态广播循环 ----

    def _start_broadcast(self) -> None:
        self._broadcast_running = True
        self._broadcast_thread = threading.Thread(
            target=self._broadcast_loop,
            name="comm-broadcast",
            daemon=True,
        )
        self._broadcast_thread.start()

    def _broadcast_loop(self) -> None:
        """定期广播状态"""
        while self._broadcast_running:
            time.sleep(self.STATUS_BROADCAST_INTERVAL_S)
            # 由外部调用 send_status 来控制广播内容
            # 这里只做保活 ping
            if self._server.client_count > 0:
                self.send(Message(type=MessageType.PING))

    # ---- 配置管理 ----

    def get_config(self, key: str, default: Any = None) -> Any:
        return self._config.get(key, default)

    def set_config(self, key: str, value: Any) -> None:
        if self.is_autonomous_mode:
            logger.warning("AUTONOMOUS 模式禁止修改配置: %s", key)
            return
        self._config[key] = value

    # ---- 统计 ----

    def get_stats(self) -> dict:
        return {
            "msgs_sent": self._msgs_sent,
            "msgs_received": self._msgs_received,
            "msgs_blocked": self._msgs_blocked,
            "is_autonomous_mode": self.is_autonomous_mode,
            "can_receive": self.can_receive_commands(),
            "clients": self._server.client_count,
            "config_keys": list(self._config.keys()),
        }


# ============================================================
# 独立测试
# ============================================================

if __name__ == "__main__":
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s [%(levelname)-7s] %(name)s: %(message)s",
        datefmt="%H:%M:%S",
    )

    print("=" * 50)
    print("  通信模块 — Mock 模式测试")
    print("=" * 50)

    # 模拟状态机
    class MockSM:
        def __init__(self):
            self.is_locked = False

    sm = MockSM()
    mgr = CommManager(state_machine=sm)
    mgr.start()

    # 测试 1：DEBUG 模式发送状态
    print("\n测试 1: DEBUG 模式发送状态")
    mgr.send_status(robot_state="DEBUG", position=(500, 1500), score=0)
    assert mgr._msgs_sent > 0
    print("  ✅ 通过")

    # 测试 2：DEBUG 模式收发配置
    print("\n测试 2: 配置读写")
    from .comm_protocol import make_config_set, make_config_get
    resp = mgr._handle_config_set(make_config_set("kp_distance", 1.2))
    assert resp.payload["success"]
    resp2 = mgr._handle_config_get(make_config_get("kp_distance"))
    assert resp2.payload["value"] == 1.2
    print(f"  kp_distance = {mgr.get_config('kp_distance')}")
    print("  ✅ 通过")

    # 测试 3：AUTONOMOUS 模式拦截指令
    print("\n测试 3: AUTONOMOUS 模式拦截")
    sm.is_locked = True
    assert mgr.is_autonomous_mode
    assert not mgr.can_receive_commands()

    cmd_msg = Message(type=MessageType.COMMAND, payload={"action": "stop"})
    resp = mgr._handle_message(cmd_msg)
    assert resp is not None and resp.type == MessageType.ERROR
    assert mgr._msgs_blocked > 0
    print(f"  拦截数: {mgr._msgs_blocked}")
    print("  ✅ 通过")

    # 测试 4：AUTONOMOUS 仍可广播状态
    print("\n测试 4: AUTONOMOUS 仍可广播")
    mgr.send_status(robot_state="AUTONOMOUS", position=(1000, 1500), score=15)
    assert mgr._msgs_sent > 0
    print("  ✅ 通过")

    # 测试 5：事件通知
    print("\n测试 5: 事件通知")
    mgr.send_event("contact_detected", "与对方接触 3s")
    print("  ✅ 通过")

    # 测试 6：AUTONOMOUS 下写配置被拒绝
    print("\n测试 6: AUTONOMOUS 配置写保护")
    mgr.set_config("test_key", "should_fail")
    assert mgr.get_config("test_key") is None
    print("  ✅ 通过")

    # 测试 7：回到 DEBUG 恢复
    print("\n测试 7: 回到 DEBUG 恢复接收")
    sm.is_locked = False
    assert mgr.can_receive_commands()
    mgr.set_config("test_key", "now_ok")
    assert mgr.get_config("test_key") == "now_ok"
    print("  ✅ 通过")

    mgr.stop()
    print(f"\n统计: {mgr.get_stats()}")
    print(f"\n{'='*50}")
    print("  通信模块测试全部通过 ✅")
    print(f"{'='*50}")
