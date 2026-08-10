"""
stability.py —— 多场比赛稳定性

跟踪和保证系统长期运行的稳定性：
  1. MatchStabilityTracker — 多场比赛成绩记录与一致性分析
  2. MemoryMonitor — 内存使用追踪，泄漏检测
  3. BootSuccessTracker — 启动成功率追踪（目标 ≥ 99%）
"""

import json
import logging
import math
import os
import threading
import time
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

logger = logging.getLogger("stability")


# ============================================================
# 通用数据结构
# ============================================================

@dataclass
class MatchRecord:
    """单场比赛记录"""
    match_id: int
    start_time: float = 0.0
    end_time: float = 0.0
    score: int = 0
    targets_delivered: int = 0
    trips_completed: int = 0
    anomaly_count: int = 0
    completed: bool = False
    forced_separations: int = 0
    notes: str = ""

    @property
    def duration_s(self) -> float:
        return max(0, self.end_time - self.start_time) if self.end_time > 0 else 0

    def to_dict(self) -> dict:
        return {
            "match_id": self.match_id,
            "score": self.score,
            "targets_delivered": self.targets_delivered,
            "trips_completed": self.trips_completed,
            "anomaly_count": self.anomaly_count,
            "duration_s": self.duration_s,
            "completed": self.completed,
            "forced_separations": self.forced_separations,
            "notes": self.notes,
        }

    @staticmethod
    def from_dict(d: dict) -> "MatchRecord":
        return MatchRecord(
            match_id=d.get("match_id", 0),
            score=d.get("score", 0),
            targets_delivered=d.get("targets_delivered", 0),
            trips_completed=d.get("trips_completed", 0),
            anomaly_count=d.get("anomaly_count", 0),
            completed=d.get("completed", False),
            forced_separations=d.get("forced_separations", 0),
            notes=d.get("notes", ""),
        )


@dataclass
class MemorySnapshot:
    """内存快照"""
    timestamp: float
    rss_mb: float
    vms_mb: float
    percent: float


@dataclass
class MemoryReport:
    """内存报告"""
    snapshots: List[MemorySnapshot] = field(default_factory=list)
    peak_rss_mb: float = 0.0
    avg_rss_mb: float = 0.0
    trend_slope: float = 0.0
    leak_suspected: bool = False
    leak_detail: str = ""

    def summary(self) -> str:
        return (
            f"内存: 峰值={self.peak_rss_mb:.1f}MB, "
            f"均值={self.avg_rss_mb:.1f}MB, "
            f"趋势={'↑' if self.trend_slope > 0 else '↓'} "
            f"{abs(self.trend_slope*60):.3f} MB/min, "
            f"泄漏={'⚠ 疑似' if self.leak_suspected else '✓ 无'}"
        )


# ============================================================
# 1. MatchStabilityTracker —— 多场比赛稳定性
# ============================================================

class MatchStabilityTracker:
    """
    多场比赛稳定性追踪器。

    记录每场比赛成绩，分析一致性。
    比赛成绩取平均值，单场赌博式策略会反映为高方差异常。
    """

    MIN_MATCHES_FOR_STABILITY = 3
    STABLE_VARIANCE_THRESHOLD = 0.15

    def __init__(self, storage_path: str = "./logs/stability.json"):
        self._storage_path = storage_path
        self._matches: List[MatchRecord] = []
        self._current_match: Optional[MatchRecord] = None
        self._match_counter = 0
        self._load()
        logger.info(f"MatchStabilityTracker 初始化: "
                     f"历史 {len(self._matches)} 场, 路径={storage_path}")

    def start_match(self, match_id: Optional[int] = None) -> MatchRecord:
        if match_id is None:
            self._match_counter += 1
            match_id = self._match_counter
        self._current_match = MatchRecord(
            match_id=match_id,
            start_time=time.time(),
        )
        logger.info(f"🏁 比赛 #{match_id} 开始")
        return self._current_match

    def end_match(self,
                  score: int = 0,
                  targets_delivered: int = 0,
                  trips_completed: int = 0,
                  anomalies: int = 0,
                  forced_separations: int = 0,
                  completed: bool = True,
                  notes: str = "") -> MatchRecord:
        if self._current_match is None:
            logger.warning("没有进行中的比赛，创建新记录")
            self._current_match = MatchRecord(
                match_id=self._match_counter + 1,
                start_time=time.time() - 180,
            )
        rec = self._current_match
        rec.end_time = time.time()
        rec.score = score
        rec.targets_delivered = targets_delivered
        rec.trips_completed = trips_completed
        rec.anomaly_count = anomalies
        rec.forced_separations = forced_separations
        rec.completed = completed
        rec.notes = notes
        self._matches.append(rec)
        self._current_match = None
        logger.info(
            f"🏁 比赛 #{rec.match_id} 结束: "
            f"{rec.score}分, {rec.targets_delivered}个目标, "
            f"{rec.duration_s:.0f}s"
        )
        self._save()
        return rec

    def get_current_match(self) -> Optional[MatchRecord]:
        return self._current_match

    def get_stability_score(self, last_n: int = 5) -> float:
        if len(self._matches) < self.MIN_MATCHES_FOR_STABILITY:
            return 1.0
        recent = self._matches[-last_n:] if len(self._matches) >= last_n else self._matches
        scores = [m.score for m in recent]
        mean_score = sum(scores) / len(scores)
        if mean_score == 0:
            return 0.0
        variance = sum((s - mean_score) ** 2 for s in scores) / len(scores)
        normalized_variance = variance / (mean_score ** 2) if mean_score > 0 else 1.0
        return max(0.0, min(1.0, 1.0 - normalized_variance))

    def is_stable(self, last_n: int = 5) -> bool:
        if len(self._matches) < self.MIN_MATCHES_FOR_STABILITY:
            return False
        return self.get_stability_score(last_n) >= self.STABLE_VARIANCE_THRESHOLD

    def get_average_score(self, last_n: Optional[int] = None) -> float:
        matches = self._matches
        if last_n is not None:
            matches = matches[-last_n:]
        return sum(m.score for m in matches) / len(matches) if matches else 0

    def get_completion_rate(self, last_n: Optional[int] = None) -> float:
        matches = self._matches
        if last_n is not None:
            matches = matches[-last_n:]
        completed = sum(1 for m in matches if m.completed)
        return completed / len(matches) if matches else 0

    def get_stats(self) -> dict:
        if not self._matches:
            return {
                "total_matches": 0,
                "avg_score": 0,
                "stability_score": 1.0,
                "is_stable": False,
                "completion_rate": 0,
            }
        scores = [m.score for m in self._matches]
        return {
            "total_matches": len(self._matches),
            "avg_score": sum(scores) / len(scores),
            "max_score": max(scores),
            "min_score": min(scores),
            "stability_score": self.get_stability_score(),
            "is_stable": self.is_stable(),
            "completion_rate": self.get_completion_rate(),
            "total_targets_delivered": sum(m.targets_delivered for m in self._matches),
            "total_anomalies": sum(m.anomaly_count for m in self._matches),
        }

    def _save(self) -> None:
        try:
            os.makedirs(os.path.dirname(self._storage_path), exist_ok=True)
            data = {
                "match_counter": self._match_counter,
                "matches": [m.to_dict() for m in self._matches],
            }
            with open(self._storage_path, "w", encoding="utf-8") as f:
                json.dump(data, f, indent=2, ensure_ascii=False)
        except Exception as e:
            logger.error(f"保存稳定性数据失败: {e}")

    def _load(self) -> None:
        if not os.path.exists(self._storage_path):
            return
        try:
            with open(self._storage_path, "r", encoding="utf-8") as f:
                data = json.load(f)
            self._match_counter = data.get("match_counter", 0)
            self._matches = [MatchRecord.from_dict(m) for m in data.get("matches", [])]
            logger.info(f"加载稳定性数据: {len(self._matches)} 场比赛记录")
        except Exception as e:
            logger.warning(f"加载稳定性数据失败: {e}，将使用空记录")


# ============================================================
# 2. MemoryMonitor —— 内存管理
# ============================================================

class MemoryMonitor:
    """
    内存监控器。

    定期采样进程内存使用，检测持续增长趋势（内存泄漏）。
    psutil 不可用时降级为仅日志告警，不影响比赛运行。
    """

    LEAK_SLOPE_THRESHOLD_MB_PER_S = 0.01
    MAX_SNAPSHOTS = 200

    def __init__(self):
        self._snapshots: List[MemorySnapshot] = []
        self._monitor_thread: Optional[threading.Thread] = None
        self._stop_event = threading.Event()
        self._psutil_available = False

        try:
            import psutil  # noqa: F401
            self._psutil_available = True
            self._process = psutil.Process()
        except ImportError:
            logger.info("psutil 未安装，内存监控降级为基本模式")

        logger.info("MemoryMonitor 初始化")

    def start_monitoring(self, interval_s: float = 5.0) -> None:
        if self._monitor_thread and self._monitor_thread.is_alive():
            logger.warning("内存监控已在运行中")
            return
        self._stop_event.clear()
        self._snapshots.clear()
        self._monitor_thread = threading.Thread(
            target=self._sampling_loop,
            args=(interval_s,),
            name="memory-monitor",
            daemon=True,
        )
        self._monitor_thread.start()
        logger.info(f"内存监控已启动 (间隔={interval_s}s)")

    def stop_monitoring(self) -> None:
        self._stop_event.set()
        if self._monitor_thread and self._monitor_thread.is_alive():
            self._monitor_thread.join(timeout=2.0)
            if self._monitor_thread.is_alive():
                logger.warning("内存监控线程未能及时停止")
        logger.info(f"内存监控已停止 ({len(self._snapshots)} 个采样点)")

    def take_snapshot(self) -> Optional[MemorySnapshot]:
        if not self._psutil_available:
            return None
        try:
            mem = self._process.memory_info()
            snapshot = MemorySnapshot(
                timestamp=time.time(),
                rss_mb=mem.rss / (1024 * 1024),
                vms_mb=mem.vms / (1024 * 1024),
                percent=self._process.memory_percent() or 0.0,
            )
            self._snapshots.append(snapshot)
            if len(self._snapshots) > self.MAX_SNAPSHOTS:
                self._snapshots = self._snapshots[-self.MAX_SNAPSHOTS:]
            return snapshot
        except Exception as e:
            logger.error(f"采样内存失败: {e}")
            return None

    def check_leak(self) -> bool:
        if len(self._snapshots) < 10:
            return False
        return self._compute_trend_slope() > self.LEAK_SLOPE_THRESHOLD_MB_PER_S

    def get_memory_report(self) -> MemoryReport:
        if not self._snapshots:
            return MemoryReport(leak_detail="无采样数据")

        rss_values = [s.rss_mb for s in self._snapshots]
        peak = max(rss_values)
        avg = sum(rss_values) / len(rss_values)
        slope = self._compute_trend_slope()
        leak = slope > self.LEAK_SLOPE_THRESHOLD_MB_PER_S

        detail = ""
        if leak:
            mb_per_min = slope * 60
            detail = (f"疑似内存泄漏: 增长速率 {mb_per_min:.3f} MB/min, "
                      f"峰值={peak:.1f}MB, 均值={avg:.1f}MB")

        return MemoryReport(
            snapshots=list(self._snapshots),
            peak_rss_mb=peak,
            avg_rss_mb=avg,
            trend_slope=slope,
            leak_suspected=leak,
            leak_detail=detail,
        )

    def get_current_rss_mb(self) -> Optional[float]:
        snapshot = self.take_snapshot()
        return snapshot.rss_mb if snapshot else None

    def get_stats(self) -> dict:
        report = self.get_memory_report()
        return {
            "peak_rss_mb": report.peak_rss_mb,
            "avg_rss_mb": report.avg_rss_mb,
            "leak_suspected": report.leak_suspected,
            "snapshot_count": len(report.snapshots),
            "trend_mb_per_min": report.trend_slope * 60,
            "psutil_available": self._psutil_available,
            "monitoring_active": (
                self._monitor_thread is not None and
                self._monitor_thread.is_alive()
            ),
        }

    def _sampling_loop(self, interval_s: float) -> None:
        logger.debug("内存采样循环已启动")
        while not self._stop_event.is_set():
            self.take_snapshot()
            self._stop_event.wait(interval_s)

    def _compute_trend_slope(self) -> float:
        if len(self._snapshots) < 5:
            return 0.0
        recent = self._snapshots[-60:]
        n = len(recent)
        t0 = recent[0].timestamp
        sum_t = 0.0
        sum_rss = 0.0
        sum_t_rss = 0.0
        sum_t2 = 0.0
        for s in recent:
            t = s.timestamp - t0
            sum_t += t
            sum_rss += s.rss_mb
            sum_t_rss += t * s.rss_mb
            sum_t2 += t * t
        denominator = n * sum_t2 - sum_t * sum_t
        if denominator == 0:
            return 0.0
        return (n * sum_t_rss - sum_t * sum_rss) / denominator


# ============================================================
# 3. BootSuccessTracker —— 启动成功率追踪
# ============================================================

class BootSuccessTracker:
    """
    启动成功率追踪器。

    记录每次 BOOT 状态的结果，计算成功率。
    比赛要求：一键启动成功率 ≥ 99%（只有一次启动机会）。
    """

    TARGET_SUCCESS_RATE = 0.99

    def __init__(self, storage_path: str = "./logs/boot_history.json"):
        self._storage_path = storage_path
        self._boots: List[dict] = []
        self._total_attempts = 0
        self._total_successes = 0
        self._last_boot_result: Optional[dict] = None
        self._load()
        logger.info(f"BootSuccessTracker 初始化: "
                     f"{self._total_successes}/{self._total_attempts} "
                     f"({self.get_success_rate()*100:.1f}%)")

    def record_boot(self, success: bool, reason: str = "") -> None:
        record = {
            "success": success,
            "reason": reason,
            "timestamp": time.time(),
        }
        self._boots.append(record)
        self._total_attempts += 1
        if success:
            self._total_successes += 1
        self._last_boot_result = record
        level = logging.INFO if success else logging.WARNING
        logger.log(level,
                   f"启动{'成功' if success else '失败'}: {reason} "
                   f"(成功率: {self.get_success_rate()*100:.1f}%)")
        self._save()

    def get_success_rate(self) -> float:
        if self._total_attempts == 0:
            return 1.0
        return self._total_successes / self._total_attempts

    def is_meeting_target(self) -> bool:
        return self.get_success_rate() >= self.TARGET_SUCCESS_RATE

    def get_recent_failures(self, n: int = 10) -> List[dict]:
        failures = [b for b in self._boots if not b["success"]]
        return failures[-n:]

    def get_failure_reasons(self) -> Dict[str, int]:
        reasons: Dict[str, int] = {}
        for b in self._boots:
            if not b["success"]:
                reason = b.get("reason", "未知")
                reasons[reason] = reasons.get(reason, 0) + 1
        return reasons

    def get_last_boot_result(self) -> Optional[dict]:
        return self._last_boot_result

    def get_stats(self) -> dict:
        return {
            "total_attempts": self._total_attempts,
            "total_successes": self._total_successes,
            "success_rate": self.get_success_rate(),
            "meeting_target": self.is_meeting_target(),
            "target_rate": self.TARGET_SUCCESS_RATE,
            "recent_failures": len(self.get_recent_failures(5)),
            "top_failure_reasons": sorted(
                self.get_failure_reasons().items(),
                key=lambda x: x[1], reverse=True,
            )[:3],
        }

    def _save(self) -> None:
        try:
            os.makedirs(os.path.dirname(self._storage_path), exist_ok=True)
            data = {
                "total_attempts": self._total_attempts,
                "total_successes": self._total_successes,
                "boots": self._boots[-200:],
            }
            with open(self._storage_path, "w", encoding="utf-8") as f:
                json.dump(data, f, indent=2, ensure_ascii=False)
        except Exception as e:
            logger.error(f"保存启动记录失败: {e}")

    def _load(self) -> None:
        if not os.path.exists(self._storage_path):
            return
        try:
            with open(self._storage_path, "r", encoding="utf-8") as f:
                data = json.load(f)
            self._total_attempts = data.get("total_attempts", 0)
            self._total_successes = data.get("total_successes", 0)
            self._boots = data.get("boots", [])
        except Exception as e:
            logger.warning(f"加载启动记录失败: {e}，将使用空记录")


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
    print("  稳定性模块 — 独立测试")
    print("=" * 60)

    test_dir = "/tmp/test_robustness_stability"
    import shutil
    shutil.rmtree(test_dir, ignore_errors=True)
    os.makedirs(test_dir, exist_ok=True)

    # ---- 测试 1: MatchStabilityTracker ----
    print("\n--- 测试 1: MatchStabilityTracker ---")
    tracker = MatchStabilityTracker(
        storage_path=f"{test_dir}/test_stability.json"
    )
    for i in range(5):
        tracker.start_match()
        score = 40 + i * 3
        tracker.end_match(
            score=score,
            targets_delivered=5 + i,
            trips_completed=3,
            anomalies=1 if i == 2 else 0,
        )
    stats = tracker.get_stats()
    print(f"  总场次: {stats['total_matches']}")
    print(f"  平均分: {stats['avg_score']:.1f}")
    print(f"  稳定性: {stats['stability_score']:.3f}")
    print(f"  完赛率: {stats['completion_rate']*100:.0f}%")
    assert stats["total_matches"] == 5
    assert stats["avg_score"] > 0
    assert 0 <= stats["stability_score"] <= 1.0
    print("✅ 测试 1 通过")

    # ---- 测试 2: 稳定性评分 ----
    print("\n--- 测试 2: 稳定性评分 (高方差) ---")
    tracker2 = MatchStabilityTracker(
        storage_path=f"{test_dir}/test_stability2.json"
    )
    tracker2._matches = [
        MatchRecord(match_id=1, score=20),
        MatchRecord(match_id=2, score=55),
        MatchRecord(match_id=3, score=10),
        MatchRecord(match_id=4, score=60),
        MatchRecord(match_id=5, score=5),
    ]
    stability = tracker2.get_stability_score()
    print(f"  不稳定场景 稳定性评分: {stability:.3f}")
    assert stability < 0.5, f"不稳定场景应有低评分，实际={stability:.3f}"
    print("✅ 测试 2 通过")

    # ---- 测试 3: MemoryMonitor ----
    print("\n--- 测试 3: MemoryMonitor ---")
    mm = MemoryMonitor()
    for i in range(20):
        mm._snapshots.append(MemorySnapshot(
            timestamp=time.time() - 20 + i,
            rss_mb=50.0 + i * 0.5,
            vms_mb=100.0,
            percent=1.0,
        ))
    report = mm.get_memory_report()
    print(f"  峰值: {report.peak_rss_mb:.1f}MB")
    print(f"  趋势斜率: {report.trend_slope*60:.3f} MB/min")
    print(f"  泄漏: {report.leak_suspected}")
    assert report.peak_rss_mb > 50
    assert report.trend_slope > 0
    print("✅ 测试 3 通过")

    # ---- 测试 4: 无泄漏检测 ----
    print("\n--- 测试 4: 无泄漏检测 ---")
    mm2 = MemoryMonitor()
    for i in range(20):
        mm2._snapshots.append(MemorySnapshot(
            timestamp=time.time() - 20 + i,
            rss_mb=50.0 + (i % 3) * 0.2,
            vms_mb=100.0,
            percent=1.0,
        ))
    report2 = mm2.get_memory_report()
    print(f"  稳定场景 趋势斜率: {report2.trend_slope*60:.4f} MB/min")
    print("✅ 测试 4 通过")

    # ---- 测试 5: BootSuccessTracker ----
    print("\n--- 测试 5: BootSuccessTracker ---")
    bst = BootSuccessTracker(
        storage_path=f"{test_dir}/test_boot.json"
    )
    for i in range(100):
        if i == 50:
            bst.record_boot(success=False, reason="IMU 自检超时")
        else:
            bst.record_boot(success=True, reason="自检通过")

    rate = bst.get_success_rate()
    print(f"  成功率: {rate*100:.1f}%")
    print(f"  达标: {bst.is_meeting_target()}")
    assert rate == 0.99, f"成功率应为 99%，实际={rate*100:.1f}%"
    assert bst.is_meeting_target(), "应达标"

    reasons = bst.get_failure_reasons()
    print(f"  失败原因: {reasons}")
    assert "IMU 自检超时" in reasons
    print("✅ 测试 5 通过")

    # ---- 测试 6: 统计查询 ----
    print("\n--- 测试 6: 统计查询 ---")
    mem_stats = mm.get_stats()
    print(f"  内存统计: 峰值={mem_stats['peak_rss_mb']:.1f}MB, "
          f"采样={mem_stats['snapshot_count']}次")
    boot_stats = bst.get_stats()
    print(f"  启动统计: {boot_stats['total_successes']}/{boot_stats['total_attempts']}")
    print("✅ 测试 6 通过")

    # 清理
    import shutil
    shutil.rmtree(test_dir, ignore_errors=True)

    print(f"\n{'=' * 60}")
    print("  稳定性模块 — 全部测试通过 ✅")
    print(f"{'=' * 60}")
