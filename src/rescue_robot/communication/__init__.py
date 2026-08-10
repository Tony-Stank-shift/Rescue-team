"""
communication —— 通信模块

机器人与笔记本的通信系统：
  DEBUG → 双向通信（配置/查询/数据）
  AUTONOMOUS → 仅单向广播（状态/事件），禁止接收指令

子模块：
  comm_protocol  — 消息格式 + 序列化
  comm_server    — 通信服务器（Mock + WebSocket）
  comm_manager   — 通信管理器（模式感知 + 安全检查）
"""

from .comm_protocol import (
    Message, MessageType, BLOCKED_IN_AUTONOMOUS,
    serialize, deserialize,
    make_status, make_sensor_data, make_config_get,
    make_config_set, make_config_resp, make_event,
    make_error, make_ping, make_pong,
)
from .comm_server import (
    AbstractCommServer, MockCommServer, WebSocketServer,
)
from .comm_manager import (
    CommManager,
)

__all__ = [
    "Message", "MessageType", "BLOCKED_IN_AUTONOMOUS",
    "serialize", "deserialize",
    "make_status", "make_sensor_data", "make_config_get",
    "make_config_set", "make_config_resp", "make_event",
    "make_error", "make_ping", "make_pong",
    "AbstractCommServer", "MockCommServer", "WebSocketServer",
    "CommManager",
]
