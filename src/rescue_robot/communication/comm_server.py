"""
comm_server.py —— 通信服务器

提供机器人 ↔ 笔记本的通信通道。

  AbstractCommServer  — 抽象基类
  MockCommServer      — 控制台模拟（开发测试）
  WebSocketServer     — 真实 WebSocket 服务
"""

import logging
import threading
import time
from typing import Callable, Dict, List, Optional

from .comm_protocol import Message, serialize, deserialize, make_pong

logger = logging.getLogger("comm_server")


class AbstractCommServer:
    """通信服务器抽象基类"""

    def start(self, host: str = "0.0.0.0", port: int = 8765) -> bool:
        raise NotImplementedError

    def stop(self) -> None:
        raise NotImplementedError

    def send(self, msg: Message) -> bool:
        raise NotImplementedError

    def broadcast(self, msg: Message) -> bool:
        raise NotImplementedError

    def on_message(self, callback: Callable[[Message], Optional[Message]]) -> None:
        """注册消息处理回调。返回非 None 则自动回复。"""
        raise NotImplementedError

    @property
    def is_running(self) -> bool:
        raise NotImplementedError

    @property
    def client_count(self) -> int:
        raise NotImplementedError


# ============================================================
# Mock 服务器
# ============================================================

class MockCommServer(AbstractCommServer):
    """
    Mock 通信服务器：控制台输入输出模拟。

    用法：
      server = MockCommServer()
      server.start()
      server.send(make_status(...))  # 打印到控制台
      # 输入 "config_get:key" 模拟接收消息
    """

    def __init__(self):
        self._running = False
        self._callback: Optional[Callable] = None
        self._input_thread: Optional[threading.Thread] = None
        self._sent_count = 0
        self._recv_count = 0

    def start(self, host: str = "", port: int = 0) -> bool:
        self._running = True
        self._input_thread = threading.Thread(
            target=self._input_loop, name="mock-comm", daemon=True
        )
        self._input_thread.start()
        logger.info("MockCommServer 启动（控制台模拟）")
        return True

    def stop(self) -> None:
        self._running = False
        if self._input_thread and self._input_thread.is_alive():
            self._input_thread.join(timeout=1.0)
        logger.info("MockCommServer 停止")

    def send(self, msg: Message) -> bool:
        self._sent_count += 1
        logger.info("[发送] %s: %s", msg.type.value,
                     str(msg.payload)[:80])
        return True

    def broadcast(self, msg: Message) -> bool:
        return self.send(msg)

    def on_message(self, callback: Callable[[Message], Optional[Message]]) -> None:
        self._callback = callback

    @property
    def is_running(self) -> bool:
        return self._running

    @property
    def client_count(self) -> int:
        return 1 if self._running else 0

    def _input_loop(self) -> None:
        """简易控制台输入模拟接收消息"""
        logger.info("Mock 通信: 输入 's:key=val' 模拟 CONFIG_SET, 'g:key' 模拟 CONFIG_GET")
        while self._running:
            try:
                line = input().strip()
                if not line:
                    continue
                if line == "q":
                    break

                # 解析简易协议: "s:key=val" / "g:key"
                msg = self._parse_input(line)
                if msg and self._callback:
                    self._recv_count += 1
                    response = self._callback(msg)
                    if response:
                        self.send(response)
            except (EOFError, KeyboardInterrupt):
                break
            except Exception as e:
                logger.error("Mock 输入异常: %s", e)

    def _parse_input(self, line: str) -> Optional[Message]:
        """解析简易控制台输入"""
        from .comm_protocol import MessageType
        if line.startswith("s:") and "=" in line:
            parts = line[2:].split("=", 1)
            return Message(
                type=MessageType.CONFIG_SET,
                payload={"key": parts[0], "value": parts[1]},
            )
        elif line.startswith("g:"):
            return Message(
                type=MessageType.CONFIG_GET,
                payload={"key": line[2:]},
            )
        elif line == "ping":
            return Message(type=MessageType.PING)
        return None


# ============================================================
# WebSocket 服务器（桩）
# ============================================================

class WebSocketServer(AbstractCommServer):
    """
    基于 asyncio 的真实 WebSocket 服务器。

    需要: pip install websockets
    端口: 8765
    """

    def __init__(self):
        self._running = False
        self._callback: Optional[Callable] = None
        self._clients: set = set()
        self._host = "0.0.0.0"
        self._port = 8765
        logger.info("WebSocketServer 初始化 (需 websockets 库)")

    def start(self, host: str = "0.0.0.0", port: int = 8765) -> bool:
        self._host = host
        self._port = port
        try:
            import asyncio
            import websockets
            self._websockets = websockets
            self._asyncio = asyncio

            self._running = True
            threading.Thread(
                target=self._run_event_loop,
                name="ws-server",
                daemon=True,
            ).start()
            logger.info("WebSocket 服务器启动: ws://%s:%d", host, port)
            return True
        except ImportError:
            logger.warning("websockets 库未安装，WebSocket 不可用。"
                           "请运行: pip install websockets")
            return False

    def stop(self) -> None:
        self._running = False
        logger.info("WebSocket 服务器停止")

    def send(self, msg: Message) -> bool:
        data = serialize(msg)
        for client in list(self._clients):
            try:
                self._asyncio.run_coroutine_threadsafe(
                    client.send(data),
                    self._loop,
                )
            except Exception:
                self._clients.discard(client)
        return True

    def broadcast(self, msg: Message) -> bool:
        return self.send(msg)

    def on_message(self, callback: Callable[[Message], Optional[Message]]) -> None:
        self._callback = callback

    @property
    def is_running(self) -> bool:
        return self._running

    @property
    def client_count(self) -> int:
        return len(self._clients)

    def _run_event_loop(self) -> None:
        """运行 asyncio 事件循环"""
        loop = self._asyncio.new_event_loop()
        self._loop = loop
        self._asyncio.set_event_loop(loop)

        async def handler(websocket, path):
            self._clients.add(websocket)
            logger.info("客户端连接: %s (共 %d 个)",
                         websocket.remote_address, len(self._clients))
            try:
                async for raw in websocket:
                    msg = deserialize(raw)
                    if msg and self._callback:
                        response = self._callback(msg)
                        if response:
                            await websocket.send(serialize(response))
            except Exception as e:
                logger.warning("客户端异常: %s", e)
            finally:
                self._clients.discard(websocket)
                logger.info("客户端断开 (共 %d 个)", len(self._clients))

        async def serve():
            async with self._websockets.serve(handler, self._host, self._port):
                await self._asyncio.Future()  # run forever

        try:
            loop.run_until_complete(serve())
        except Exception as e:
            logger.error("WebSocket 事件循环异常: %s", e)
