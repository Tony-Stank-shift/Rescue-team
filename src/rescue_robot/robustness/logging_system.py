"""
logging_system.py —— 日志与复盘

结构化日志 + 关键事件记录 + 本地存储管理：
  1. StructuredLogger — JSON 行格式的结构化日志
  2. EventRecorder — 关键事件时间线记录
  3. LogStorage — 日志文件管理（轮转、存储、导出）
"""

import csv
import json
import logging
import os
import threading
import time
from dataclasses import dataclass, field
from enum import Enum, auto
from typing import Any, Dict, List, Optional

logger = logging.getLogger("logging_system")


# ============================================================
# 事件类型
# ============================================================

class EventType(Enum):
    """关键事件类型"""
    # 状态机
    STATE_CHANGE = auto()
    BOOT_SUCCESS = auto()
    BOOT_FAIL = auto()
    ONE_KEY_START = auto()

    # 感知
    TARGET_DETECTED = auto()
    TARGET_LOST = auto()
    NEW_TARGET_APPEARED = auto()

    # 转运
    TRIP_START = auto()
    TRIP_COMPLETE = auto()
    GRIP_SUCCESS = auto()
    GRIP_FAILED = auto()
    PLACEMENT_SUCCESS = auto()
    PLACEMENT_WRONG_ZONE = auto()

    # 异常
    ANOMALY_DETECTED = auto()
    ANOMALY_RECOVERED = auto()
    FALLBACK_TRIGGERED = auto()

    # 对抗
    CONTACT_START = auto()
    CONTACT_END = auto()
    FORCED_SEPARATION = auto()

    # 硬件
    BATTERY_LOW = auto()
    SENSOR_DEGRADED = auto()
    SENSOR_RECOVERED = auto()
    MOTOR_FAULT = auto()

    # 通信
    COMM_LOST = auto()
    COMM_RECOVERED = auto()
    COMMAND_BLOCKED = auto()

    # 导航
    PATH_BLOCKED = auto()
    ARRIVED_AT_TARGET = auto()
    NAV_TIMEOUT = auto()

    # 规则
    VIOLATION_RISK = auto()
    RULE_VIOLATION = auto()

    # 比赛
    MATCH_START = auto()
    MATCH_END = auto()
    MATCH_TIME_PRESSURE = auto()

    # 通用
    SYSTEM_INFO = auto()
    CALIBRATION = auto()


@dataclass
class EventRecord:
    """单个事件记录"""
    event_type: EventType
    timestamp: float = field(default_factory=time.time)
    metadata: Dict[str, Any] = field(default_factory=dict)

    def to_dict(self) -> dict:
        return {
            "event": self.event_type.name,
            "timestamp": self.timestamp,
            "metadata": self.metadata,
        }


# ============================================================
# 1. StructuredLogger —— 结构化日志
# ============================================================

class StructuredLogger:
    """
    结构化日志器。

    包装 Python logging，额外输出 JSON 行格式日志文件。
    每条日志包含：timestamp, level, module, message, extra_data。
    """

    def __init__(self, log_dir: str = "./logs", level: int = logging.INFO):
        self._log_dir = log_dir
        self._level = level
        os.makedirs(log_dir, exist_ok=True)
        self._json_log_path = os.path.join(log_dir, "structured.jsonl")
        self._json_lock = threading.Lock()
        self._entries_written = 0
        self._events_written = 0
        logger.info(f"StructuredLogger 初始化: {self._json_log_path}")

    def debug(self, module: str, message: str, **extra) -> None:
        self._write("DEBUG", module, message, extra)

    def info(self, module: str, message: str, **extra) -> None:
        self._write("INFO", module, message, extra)

    def warning(self, module: str, message: str, **extra) -> None:
        self._write("WARNING", module, message, extra)

    def error(self, module: str, message: str, **extra) -> None:
        self._write("ERROR", module, message, extra)

    def critical(self, module: str, message: str, **extra) -> None:
        self._write("CRITICAL", module, message, extra)

    def log_event(self, category: str, event: str, **extra) -> None:
        self._write("EVENT", category, event, extra)
        self._events_written += 1

    def _write(self, level: str, module: str, message: str,
               extra: Dict[str, Any]) -> None:
        entry = {
            "ts": time.time(),
            "level": level,
            "module": module,
            "msg": message,
        }
        if extra:
            clean_extra = {}
            for k, v in extra.items():
                try:
                    json.dumps(v)
                    clean_extra[k] = v
                except (TypeError, ValueError):
                    clean_extra[k] = str(v)
            entry["data"] = clean_extra

        line = json.dumps(entry, ensure_ascii=False)
        with self._json_lock:
            try:
                with open(self._json_log_path, "a", encoding="utf-8") as f:
                    f.write(line + "\n")
                    self._entries_written += 1
            except Exception as e:
                logger.error(f"写入结构化日志失败: {e}")

    @property
    def log_path(self) -> str:
        return self._json_log_path

    @property
    def entries_written(self) -> int:
        return self._entries_written

    @property
    def events_written(self) -> int:
        return self._events_written

    def get_stats(self) -> dict:
        return {
            "log_path": self._json_log_path,
            "entries_written": self._entries_written,
            "events_written": self._events_written,
            "log_dir": self._log_dir,
        }


# ============================================================
# 2. EventRecorder —— 关键事件记录
# ============================================================

class EventRecorder:
    """
    关键事件记录器。

    记录比赛中所有关键事件，每个事件有精确时间戳。
    赛后可用于时间线可视化、决策复盘、违规分析。
    """

    def __init__(self):
        self._events: List[EventRecord] = []
        self._event_lock = threading.Lock()
        self._type_counts: Dict[EventType, int] = {}
        logger.info("EventRecorder 初始化")

    def record(self,
               event_type: EventType,
               timestamp: Optional[float] = None,
               **metadata) -> EventRecord:
        if timestamp is None:
            timestamp = time.time()

        record = EventRecord(
            event_type=event_type,
            timestamp=timestamp,
            metadata=dict(metadata),
        )
        with self._event_lock:
            self._events.append(record)
            self._type_counts[event_type] = self._type_counts.get(event_type, 0) + 1
        return record

    def record_batch(self, events: List[tuple]) -> List[EventRecord]:
        records = []
        with self._event_lock:
            for item in events:
                evt_type = item[0]
                ts = item[1] if len(item) > 1 and item[1] is not None else time.time()
                meta = item[2] if len(item) > 2 else {}
                record = EventRecord(
                    event_type=evt_type,
                    timestamp=ts,
                    metadata=meta,
                )
                self._events.append(record)
                self._type_counts[evt_type] = self._type_counts.get(evt_type, 0) + 1
                records.append(record)
        return records

    def get_timeline(self,
                     event_types: Optional[List[EventType]] = None,
                     start_time: Optional[float] = None,
                     end_time: Optional[float] = None) -> List[EventRecord]:
        with self._event_lock:
            events = list(self._events)

        if event_types:
            type_set = set(event_types)
            events = [e for e in events if e.event_type in type_set]
        if start_time is not None:
            events = [e for e in events if e.timestamp >= start_time]
        if end_time is not None:
            events = [e for e in events if e.timestamp <= end_time]

        events.sort(key=lambda e: e.timestamp)
        return events

    def get_events_by_type(self, event_type: EventType) -> List[EventRecord]:
        with self._event_lock:
            return [e for e in self._events if e.event_type == event_type]

    def get_event_count(self, event_type: Optional[EventType] = None) -> int:
        if event_type:
            return self._type_counts.get(event_type, 0)
        return len(self._events)

    def get_stats(self) -> dict:
        return {
            "total_events": len(self._events),
            "type_counts": {t.name: c for t, c in self._type_counts.items()},
            "first_event_time": self._events[0].timestamp if self._events else None,
            "last_event_time": self._events[-1].timestamp if self._events else None,
        }

    def export_json(self, filepath: str) -> None:
        timeline = self.get_timeline()
        data = {
            "export_time": time.time(),
            "total_events": len(timeline),
            "events": [e.to_dict() for e in timeline],
        }
        os.makedirs(os.path.dirname(filepath), exist_ok=True)
        with open(filepath, "w", encoding="utf-8") as f:
            json.dump(data, f, indent=2, ensure_ascii=False)
        logger.info(f"事件时间线已导出: {filepath} ({len(timeline)} 个事件)")

    def export_csv(self, filepath: str) -> None:
        timeline = self.get_timeline()
        os.makedirs(os.path.dirname(filepath), exist_ok=True)
        with open(filepath, "w", encoding="utf-8", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["timestamp", "datetime", "event_type", "metadata"])
            for e in timeline:
                dt_str = time.strftime(
                    "%Y-%m-%d %H:%M:%S",
                    time.localtime(e.timestamp)
                )
                writer.writerow([
                    f"{e.timestamp:.3f}",
                    dt_str,
                    e.event_type.name,
                    json.dumps(e.metadata, ensure_ascii=False),
                ])
        logger.info(f"事件时间线已导出 CSV: {filepath} ({len(timeline)} 行)")

    def reset(self) -> None:
        with self._event_lock:
            self._events.clear()
            self._type_counts.clear()
        logger.info("EventRecorder 已重置")


# ============================================================
# 3. LogStorage —— 日志存储管理
# ============================================================

class LogStorage:
    """
    日志存储管理器。

    管理日志文件的目录结构、轮转、清理和查询。
    适配 SD 卡存储（容量限制）。
    """

    MAX_LOG_SIZE_MB = 50
    MAX_TOTAL_SIZE_MB = 500
    KEEP_RECENT_MATCHES = 20

    def __init__(self, base_dir: str = "./logs"):
        self._base_dir = base_dir
        self._current_match_dir: Optional[str] = None
        self._current_match_id: Optional[int] = None
        os.makedirs(base_dir, exist_ok=True)
        logger.info(f"LogStorage 初始化: {base_dir}")

    def start_new_match(self, match_id: Optional[int] = None) -> str:
        if match_id is None:
            existing = self.list_matches()
            match_id = len(existing) + 1
        date_str = time.strftime("%Y%m%d_%H%M%S")
        dir_name = f"match_{match_id:03d}_{date_str}"
        self._current_match_dir = os.path.join(self._base_dir, dir_name)
        self._current_match_id = match_id
        os.makedirs(self._current_match_dir, exist_ok=True)
        logger.info(f"新比赛日志目录: {self._current_match_dir}")
        return self._current_match_dir

    def end_match(self) -> None:
        if self._current_match_dir:
            logger.info(f"比赛 #{self._current_match_id} 日志已保存: "
                        f"{self._current_match_dir}")
            self._current_match_dir = None
            self._current_match_id = None
        self._cleanup_old_logs()

    def rotate_if_needed(self, max_size_mb: Optional[int] = None) -> bool:
        if max_size_mb is None:
            max_size_mb = self.MAX_LOG_SIZE_MB
        max_bytes = max_size_mb * 1024 * 1024
        if self._current_match_dir:
            total_size = self._dir_size(self._current_match_dir)
            if total_size > max_bytes:
                logger.warning(
                    f"当前比赛日志过大 ({total_size / 1024 / 1024:.1f}MB)"
                )
                return True
        return False

    def get_total_size_mb(self) -> float:
        return self._dir_size(self._base_dir) / (1024 * 1024)

    def list_matches(self) -> List[dict]:
        matches = []
        if not os.path.isdir(self._base_dir):
            return matches
        for entry in os.listdir(self._base_dir):
            full_path = os.path.join(self._base_dir, entry)
            if not os.path.isdir(full_path):
                continue
            if not entry.startswith("match_"):
                continue
            info = self._parse_match_dir(entry)
            info["path"] = full_path
            info["size_mb"] = self._dir_size(full_path) / (1024 * 1024)
            matches.append(info)
        matches.sort(key=lambda m: m.get("match_id", 0))
        return matches

    def get_match_log(self, match_id: int) -> Optional[str]:
        for match in self.list_matches():
            if match.get("match_id") == match_id:
                return match["path"]
        return None

    def get_current_match_dir(self) -> Optional[str]:
        return self._current_match_dir

    def get_match_summary(self, match_id: int) -> dict:
        path = self.get_match_log(match_id)
        if not path:
            return {"match_id": match_id, "found": False}
        files = []
        for f in os.listdir(path):
            fpath = os.path.join(path, f)
            if os.path.isfile(fpath):
                files.append({
                    "name": f,
                    "size_mb": os.path.getsize(fpath) / (1024 * 1024),
                })
        return {
            "match_id": match_id,
            "found": True,
            "path": path,
            "size_mb": self._dir_size(path) / (1024 * 1024),
            "files": sorted(files, key=lambda f: f["name"]),
        }

    def get_storage_stats(self) -> dict:
        total_mb = self.get_total_size_mb()
        matches = self.list_matches()
        return {
            "base_dir": self._base_dir,
            "total_size_mb": total_mb,
            "max_size_mb": self.MAX_TOTAL_SIZE_MB,
            "usage_percent": (total_mb / self.MAX_TOTAL_SIZE_MB * 100
                              if self.MAX_TOTAL_SIZE_MB > 0 else 0),
            "total_matches": len(matches),
            "current_match": self._current_match_id,
        }

    def _cleanup_old_logs(self) -> None:
        matches = self.list_matches()
        if len(matches) <= self.KEEP_RECENT_MATCHES:
            return
        to_delete = matches[:-self.KEEP_RECENT_MATCHES]
        for match in to_delete:
            try:
                import shutil
                shutil.rmtree(match["path"])
                logger.info(f"清理旧日志: {match['dir_name']}")
            except Exception as e:
                logger.error(f"清理失败 {match['path']}: {e}")

    @staticmethod
    def _dir_size(path: str) -> int:
        total = 0
        if not os.path.isdir(path):
            return 0
        for dirpath, _, filenames in os.walk(path):
            for f in filenames:
                fp = os.path.join(dirpath, f)
                try:
                    total += os.path.getsize(fp)
                except OSError:
                    pass
        return total

    @staticmethod
    def _parse_match_dir(dir_name: str) -> dict:
        parts = dir_name.split("_")
        match_id = 0
        date_str = ""
        time_str = ""
        try:
            match_id = int(parts[1]) if len(parts) > 1 else 0
            date_str = parts[2] if len(parts) > 2 else ""
            time_str = parts[3] if len(parts) > 3 else ""
        except (ValueError, IndexError):
            pass
        return {
            "dir_name": dir_name,
            "match_id": match_id,
            "date": date_str,
            "time": time_str,
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

    print("=" * 60)
    print("  日志与复盘模块 — 独立测试")
    print("=" * 60)

    test_dir = "/tmp/test_logging_system"
    import shutil
    shutil.rmtree(test_dir, ignore_errors=True)
    os.makedirs(test_dir, exist_ok=True)

    # ---- 测试 1: StructuredLogger ----
    print("\n--- 测试 1: StructuredLogger ---")
    slog = StructuredLogger(log_dir=os.path.join(test_dir, "structured"))
    slog.info("perception", "摄像头初始化成功", fps=30, resolution="640x480")
    slog.warning("navigation", "路径规划超时", attempts=3, timeout_ms=500)
    slog.error("motor", "电机 #2 堵转", motor_id=2, current_ma=4500)
    slog.log_event("transport", "首次转运完成", score=5, target="普通物资")
    slog.log_event("decision", "进入时间紧迫模式", remaining_s=28)

    assert os.path.exists(slog.log_path), "日志文件应存在"
    with open(slog.log_path, "r", encoding="utf-8") as f:
        lines = f.readlines()
    print(f"  写入 {len(lines)} 行 JSON 日志")
    assert len(lines) >= 5, f"应至少 5 行，实际 {len(lines)} 行"

    for line in lines:
        entry = json.loads(line.strip())
        assert "ts" in entry and "level" in entry and "module" in entry and "msg" in entry
    print("✅ 测试 1 通过")

    # ---- 测试 2: EventRecorder ----
    print("\n--- 测试 2: EventRecorder ---")
    recorder = EventRecorder()
    recorder.record(EventType.MATCH_START)
    recorder.record(EventType.BOOT_SUCCESS, reason="自检通过")
    recorder.record(EventType.ONE_KEY_START)
    recorder.record(EventType.TARGET_DETECTED,
                    target_id=3, target_type="伤员",
                    position=(1200, 1800))
    recorder.record(EventType.TRIP_START, targets=1, trip_number=1)
    recorder.record(EventType.GRIP_SUCCESS, target_id=3)
    recorder.record(EventType.ARRIVED_AT_TARGET, distance_mm=35)
    recorder.record(EventType.PLACEMENT_SUCCESS, zone="物资区")
    recorder.record(EventType.TRIP_COMPLETE, targets=1, score=15, duration_s=22.5)
    recorder.record(EventType.ANOMALY_DETECTED, anomaly_type="STUCK")
    recorder.record(EventType.ANOMALY_RECOVERED, duration_s=1.8)
    recorder.record(EventType.MATCH_END, score=45, targets_delivered=5)

    timeline = recorder.get_timeline()
    print(f"  记录 {len(timeline)} 个事件")

    trip_events = recorder.get_timeline(
        event_types=[EventType.TRIP_START, EventType.TRIP_COMPLETE]
    )
    print(f"  转运事件: {len(trip_events)} 个")

    for i in range(len(timeline) - 1):
        assert timeline[i].timestamp <= timeline[i + 1].timestamp, \
            "事件应按时序排列"

    assert recorder.get_event_count(EventType.MATCH_START) == 1
    assert recorder.get_event_count(EventType.TRIP_COMPLETE) == 1
    print("✅ 测试 2 通过")

    # ---- 测试 3: EventRecorder 导出 ----
    print("\n--- 测试 3: EventRecorder 导出 ---")
    json_path = os.path.join(test_dir, "timeline_export.json")
    recorder.export_json(json_path)
    assert os.path.exists(json_path), "JSON 导出文件应存在"

    csv_path = os.path.join(test_dir, "timeline_export.csv")
    recorder.export_csv(csv_path)
    assert os.path.exists(csv_path), "CSV 导出文件应存在"

    print(f"  JSON: {os.path.getsize(json_path)} 字节")
    print(f"  CSV: {os.path.getsize(csv_path)} 字节")
    print("✅ 测试 3 通过")

    # ---- 测试 4: LogStorage ----
    print("\n--- 测试 4: LogStorage ---")
    storage = LogStorage(base_dir=os.path.join(test_dir, "match_logs"))
    match_dir = storage.start_new_match(match_id=1)
    print(f"  比赛目录: {match_dir}")
    assert os.path.isdir(match_dir), "比赛目录应创建"

    for i in range(5):
        with open(os.path.join(match_dir, f"sensor_{i}.jsonl"), "w") as f:
            f.write(json.dumps({"ts": time.time(), "data": f"sample_{i}"}) + "\n")

    storage.end_match()

    matches = storage.list_matches()
    print(f"  比赛列表: {len(matches)} 场")
    assert len(matches) == 1, f"应有 1 场，实际 {len(matches)} 场"
    assert matches[0]["match_id"] == 1
    assert matches[0]["size_mb"] > 0

    summary = storage.get_match_summary(1)
    print(f"  比赛摘要: match_id={summary['match_id']}, "
          f"size={summary['size_mb']:.3f}MB, "
          f"files={len(summary['files'])}")
    assert summary["found"]

    stats = storage.get_storage_stats()
    print(f"  存储统计: 总量={stats['total_size_mb']:.3f}MB, "
          f"比赛数={stats['total_matches']}")
    print("✅ 测试 4 通过")

    # ---- 测试 5: 轮转与清理 ----
    print("\n--- 测试 5: 轮转与清理 ---")
    storage2 = LogStorage(
        base_dir=os.path.join(test_dir, "match_logs_rotate"),
    )
    storage2.KEEP_RECENT_MATCHES = 3

    for i in range(6):
        storage2.start_new_match(match_id=i + 1)
        with open(os.path.join(storage2.get_current_match_dir(), "log.jsonl"), "w") as f:
            f.write("test\n")
        storage2.end_match()

    matches = storage2.list_matches()
    print(f"  保留比赛: {len(matches)} 场 (应 ≤ {storage2.KEEP_RECENT_MATCHES})")
    assert len(matches) <= 3, \
        f"应只保留 3 场，实际 {len(matches)} 场"
    print("✅ 测试 5 通过")

    # ---- 测试 6: 超大日志处理 ----
    print("\n--- 测试 6: 超大日志处理 ---")
    storage3 = LogStorage(base_dir=os.path.join(test_dir, "oversized"))
    storage3.start_new_match(match_id=1)
    big_file = os.path.join(storage3.get_current_match_dir(), "big.log")
    with open(big_file, "w") as f:
        f.write("x" * (1024 * 1024 + 1))  # slightly > 1MB (1MB+1 byte)
    rotated = storage3.rotate_if_needed(max_size_mb=1)
    print(f"  轮转触发: {rotated}")
    assert rotated, "超过 1MB 应触发轮转"
    print("✅ 测试 6 通过")

    # ---- 测试 7: 不可序列化数据过滤 ----
    print("\n--- 测试 7: 不可序列化数据过滤 ---")
    slog2 = StructuredLogger(log_dir=os.path.join(test_dir, "filtered"))
    try:
        slog2.info("test", "测试不可序列化", obj=object(), valid="ok")
    except Exception as e:
        assert False, f"不可序列化数据应被过滤，不应抛异常: {e}"
    print("✅ 测试 7 通过")

    # ---- 测试 8: 批量事件记录 ----
    print("\n--- 测试 8: 批量事件记录 ---")
    r2 = EventRecorder()
    batch = [
        (EventType.STATE_CHANGE, time.time(), {"from": "BOOT", "to": "DEBUG"}),
        (EventType.STATE_CHANGE, time.time(), {"from": "DEBUG", "to": "AUTONOMOUS"}),
        (EventType.TARGET_DETECTED, None, {"target_id": 1}),
    ]
    records = r2.record_batch(batch)
    assert len(records) == 3
    assert r2.get_event_count() == 3
    print(f"  批量记录 {len(records)} 个事件")
    print("✅ 测试 8 通过")

    # ---- 测试 9: 事件重置 ----
    print("\n--- 测试 9: 事件重置 ---")
    recorder.reset()
    assert recorder.get_event_count() == 0, "重置后应为 0"
    print("✅ 测试 9 通过")

    # 清理
    shutil.rmtree(test_dir, ignore_errors=True)

    print(f"\n{'=' * 60}")
    print("  日志与复盘模块 — 全部测试通过 ✅")
    print(f"{'=' * 60}")
