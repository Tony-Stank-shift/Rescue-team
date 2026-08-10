"""
comm_protocol.py —— 通信协议定义

JSON 消息格式 + 序列化/反序列化。
AUTONOMOUS 模式下命令类消息直接丢弃。
"""

import json
import logging
import time
from dataclasses import dataclass, field
from enum import Enum, auto
from typing import Any, Dict, List, Optional

logger = logging.getLogger("comm_protocol")


class MessageType(Enum):
    """消息类型"""
    # 双向（仅 DEBUG 模式）
    CONFIG_GET = "config_get"         # 读取配置
    CONFIG_SET = "config_set"         # 修改配置
    CONFIG_RESP = "config_resp"       # 配置响应
    COMMAND = "command"               # 控制命令

    # 单向广播（两种模式均可）
    STATUS = "status"                 # 运行状态
    SENSOR_DATA = "sensor_data"       # 传感器数据
    LOG_ENTRY = "log_entry"           # 日志条目
    EVENT = "event"                   # 事件通知

    # 系统
    PING = "ping"
    PONG = "pong"
    ERROR = "error"


# AUTONOMOUS 模式下被阻止的入站消息类型
BLOCKED_IN_AUTONOMOUS = {
    MessageType.CONFIG_SET,
    MessageType.COMMAND,
}


@dataclass
class Message:
    """通信消息"""
    type: MessageType
    payload: Dict[str, Any] = field(default_factory=dict)
    timestamp: float = 0.0
    msg_id: str = ""

    def __post_init__(self):
        if self.timestamp == 0.0:
            self.timestamp = time.time()
        if not self.msg_id:
            import uuid
            self.msg_id = uuid.uuid4().hex[:8]


# ============================================================
# 序列化
# ============================================================

def serialize(msg: Message) -> str:
    """消息 → JSON 字符串"""
    return json.dumps({
        "type": msg.type.value,
        "payload": msg.payload,
        "timestamp": msg.timestamp,
        "msg_id": msg.msg_id,
    }, ensure_ascii=False)


def deserialize(data: str) -> Optional[Message]:
    """JSON 字符串 → 消息"""
    try:
        obj = json.loads(data)
        msg_type = MessageType(obj["type"])
        return Message(
            type=msg_type,
            payload=obj.get("payload", {}),
            timestamp=obj.get("timestamp", time.time()),
            msg_id=obj.get("msg_id", ""),
        )
    except (json.JSONDecodeError, KeyError, ValueError) as e:
        logger.error("消息解析失败: %s", e)
        return None


# ============================================================
# 消息工厂
# ============================================================

def make_status(robot_state: str, position: tuple, score: int,
                targets_delivered: int, time_remaining: float,
                **extra) -> Message:
    """构建状态消息"""
    return Message(
        type=MessageType.STATUS,
        payload={
            "robot_state": robot_state,
            "position": list(position),
            "score": score,
            "targets_delivered": targets_delivered,
            "time_remaining_s": time_remaining,
            **extra,
        },
    )


def make_sensor_data(camera_fps: float = 0, imu: Optional[dict] = None,
                     targets_visible: int = 0, **extra) -> Message:
    """构建传感器数据消息"""
    return Message(
        type=MessageType.SENSOR_DATA,
        payload={
            "camera_fps": camera_fps,
            "imu": imu or {},
            "targets_visible": targets_visible,
            **extra,
        },
    )


def make_config_get(key: str) -> Message:
    return Message(type=MessageType.CONFIG_GET, payload={"key": key})


def make_config_set(key: str, value: Any) -> Message:
    return Message(type=MessageType.CONFIG_SET, payload={"key": key, "value": value})


def make_config_resp(key: str, value: Any, success: bool = True) -> Message:
    return Message(type=MessageType.CONFIG_RESP,
                   payload={"key": key, "value": value, "success": success})


def make_event(event_name: str, detail: str = "", **extra) -> Message:
    return Message(type=MessageType.EVENT,
                   payload={"event": event_name, "detail": detail, **extra})


def make_error(error_msg: str) -> Message:
    return Message(type=MessageType.ERROR, payload={"error": error_msg})


def make_ping() -> Message:
    return Message(type=MessageType.PING)


def make_pong() -> Message:
    return Message(type=MessageType.PONG)
