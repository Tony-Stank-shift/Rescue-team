"""
hot_reloader.py —— 策略参数热更新 (7.1.4)

监控 YAML 配置文件变更，自动重载策略参数，无需重启进程。
仅 DEBUG 模式活跃，AUTONOMOUS 自动禁用。

用法:
  reloader = HotReloader()
  reloader.watch("config/strategy/default.yaml", on_change)
  reloader.start()
  ...
  reloader.stop()
"""

import logging
import os
import threading
import time
from typing import Any, Callable, Dict, List, Optional, Set, Tuple

logger = logging.getLogger("hot_reloader")

# 可选 YAML 支持
try:
    import yaml as _yaml
    _HAS_YAML = True
except ImportError:
    _HAS_YAML = False


# ============================================================
# HotReloader
# ============================================================

class HotReloader:
    """
    配置文件热更新器。

    特性:
    - mtime 轮询检测文件变更（默认 1s 间隔）
    - 检测后自动重载 + 回调通知
    - 变更历史追踪
    - DEBUG 模式活跃，AUTONOMOUS 模式自动禁用
    - daemon 线程，不影响主循环
    """

    POLL_INTERVAL_S = 1.0       # 轮询间隔
    MAX_HISTORY = 50             # 最大变更历史

    def __init__(self):
        self._watched: Dict[str, dict] = {}       # path → {"mtime": float, "callback": callable, "content": dict}
        self._thread: Optional[threading.Thread] = None
        self._stop_event = threading.Event()
        self._lock = threading.Lock()
        self._enabled = True
        self._reload_count = 0
        self._history: List[dict] = []             # 变更历史

        logger.info("HotReloader 初始化")

    # ---- 属性 ----

    @property
    def is_running(self) -> bool:
        return self._thread is not None and self._thread.is_alive()

    @property
    def reload_count(self) -> int:
        return self._reload_count

    @property
    def watched_paths(self) -> List[str]:
        with self._lock:
            return sorted(self._watched.keys())

    # ---- 启用/禁用 ----

    @property
    def enabled(self) -> bool:
        return self._enabled

    @enabled.setter
    def enabled(self, value: bool) -> None:
        old = self._enabled
        self._enabled = value
        if old != value:
            logger.info(f"热加载 {'启用' if value else '禁用'}")

    def disable_for_autonomous(self) -> None:
        """进入 AUTONOMOUS 模式时调用（安全锁定）。"""
        self._enabled = False
        logger.info("AUTONOMOUS 模式: 热加载已锁定")

    def enable_for_debug(self) -> None:
        """进入 DEBUG 模式时调用。"""
        self._enabled = True
        logger.info("DEBUG 模式: 热加载已激活")

    # ---- 监控 ----

    def watch(self, path: str, callback: Optional[Callable[[dict, dict], None]] = None) -> None:
        """
        监控配置文件。

        Args:
            path: 文件路径
            callback: 变更回调 (old_data, new_data) -> None
        """
        full_path = os.path.abspath(path)
        if not os.path.exists(full_path):
            logger.warning(f"监控文件不存在: {full_path}，将在创建后生效")
            mtime = 0.0
        else:
            mtime = os.path.getmtime(full_path)

        with self._lock:
            # 尝试读取初始内容
            content = None
            try:
                content = self._read_file(full_path)
            except Exception as e:
                logger.warning(f"读取监控文件失败: {full_path}: {e}")

            self._watched[full_path] = {
                "mtime": mtime,
                "callback": callback,
                "content": content,
            }
        logger.info(f"监控文件: {full_path} (callback={'yes' if callback else 'no'})")

    def unwatch(self, path: str) -> None:
        """停止监控。"""
        full_path = os.path.abspath(path)
        with self._lock:
            if full_path in self._watched:
                del self._watched[full_path]
                logger.info(f"已停止监控: {full_path}")

    def unwatch_all(self) -> None:
        """停止所有监控。"""
        with self._lock:
            count = len(self._watched)
            self._watched.clear()
        logger.info(f"已停止所有监控 ({count} 个文件)")

    # ---- 生命周期 ----

    def start(self) -> None:
        """启动后台监控线程。"""
        if self.is_running:
            logger.warning("热加载已在运行中")
            return

        self._stop_event.clear()
        self._thread = threading.Thread(
            target=self._poll_loop,
            name="hot-reloader",
            daemon=True,
        )
        self._thread.start()
        logger.info(f"热加载监控已启动 (间隔={self.POLL_INTERVAL_S}s)")

    def stop(self) -> None:
        """停止后台监控。"""
        self._stop_event.set()
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=2.0)
        logger.info(f"热加载监控已停止 ({self._reload_count} 次重载)")

    # ---- 手动重载 ----

    def reload_if_changed(self) -> int:
        """
        手动检查所有监控文件，有变更则重载。

        Returns:
            重载的文件数
        """
        if not self._enabled:
            return 0

        changed = []
        with self._lock:
            for path, info in list(self._watched.items()):
                if not os.path.exists(path):
                    continue
                current_mtime = os.path.getmtime(path)
                if current_mtime > info["mtime"]:
                    changed.append((path, info))

        for path, info in changed:
            try:
                old_content = info["content"]
                new_content = self._read_file(path)

                with self._lock:
                    self._watched[path]["mtime"] = os.path.getmtime(path)
                    self._watched[path]["content"] = new_content

                self._reload_count += 1

                # 记录历史
                self._add_history(path, old_content, new_content)

                # 回调
                if info["callback"]:
                    try:
                        info["callback"](old_content, new_content)
                    except Exception as e:
                        logger.error(f"热加载回调异常 ({path}): {e}")

                logger.info(f"配置已热加载: {path} "
                            f"(#{self._reload_count})")

            except Exception as e:
                logger.error(f"热加载失败 ({path}): {e}")

        return len(changed)

    def force_reload(self, path: str) -> bool:
        """强制重载指定文件。"""
        full_path = os.path.abspath(path)
        if full_path not in self._watched:
            logger.warning(f"未监控的文件: {full_path}")
            return False

        try:
            new_content = self._read_file(full_path)
            info = self._watched[full_path]
            old_content = info["content"]

            with self._lock:
                self._watched[full_path]["content"] = new_content
                if os.path.exists(full_path):
                    self._watched[full_path]["mtime"] = os.path.getmtime(full_path)

            self._reload_count += 1
            if info["callback"]:
                info["callback"](old_content, new_content)

            logger.info(f"配置强制重载: {path}")
            return True
        except Exception as e:
            logger.error(f"强制重载失败 ({path}): {e}")
            return False

    # ---- 历史 ----

    def get_history(self, n: int = 10) -> List[dict]:
        """获取最近 N 次变更历史。"""
        return self._history[-n:]

    def get_stats(self) -> dict:
        """获取统计信息。"""
        with self._lock:
            return {
                "running": self.is_running,
                "enabled": self._enabled,
                "watched_count": len(self._watched),
                "watched_paths": sorted(self._watched.keys()),
                "reload_count": self._reload_count,
                "history_count": len(self._history),
                "poll_interval_s": self.POLL_INTERVAL_S,
            }

    # ---- 内部 ----

    def _poll_loop(self) -> None:
        """后台轮询循环。"""
        logger.debug("热加载轮询循环启动")
        while not self._stop_event.is_set():
            try:
                self.reload_if_changed()
            except Exception as e:
                logger.error(f"热加载轮询异常: {e}")
            self._stop_event.wait(self.POLL_INTERVAL_S)

    @staticmethod
    def _read_file(path: str) -> Optional[dict]:
        """读取文件内容。"""
        if not os.path.exists(path):
            return None
        with open(path, "r", encoding="utf-8") as f:
            content = f.read()
        if _HAS_YAML:
            return _yaml.safe_load(content)
        import json
        return json.loads(content)

    def _add_history(self, path: str, old: Any, new: Any) -> None:
        """记录变更历史。"""
        entry = {
            "path": path,
            "timestamp": time.time(),
            "reload_number": self._reload_count,
        }
        self._history.append(entry)
        if len(self._history) > self.MAX_HISTORY:
            self._history = self._history[-self.MAX_HISTORY:]


# ============================================================
# 便捷函数
# ============================================================

# 全局实例（可选）
_global_reloader: Optional[HotReloader] = None


def get_global_reloader() -> HotReloader:
    """获取全局 HotReloader 实例（懒初始化）。"""
    global _global_reloader
    if _global_reloader is None:
        _global_reloader = HotReloader()
    return _global_reloader


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
    print("  热加载器 — 独立测试")
    print("=" * 60)

    import tempfile, shutil
    test_dir = os.path.join(tempfile.gettempdir(), "test_hot_reloader")
    shutil.rmtree(test_dir, ignore_errors=True)
    os.makedirs(test_dir, exist_ok=True)

    # ---- 测试 1: 基本监控 ----
    print("\n--- 测试 1: 基本监控 ---")
    test_file = os.path.join(test_dir, "test_config.yaml")
    with open(test_file, "w", encoding="utf-8") as f:
        f.write("key: value1\n")

    reloader = HotReloader()
    reloader.watch(test_file)
    assert len(reloader.watched_paths) == 1
    print(f"  监控文件: {reloader.watched_paths}")
    print("  ✅ 通过")

    # ---- 测试 2: 文件变更检测 ----
    print("\n--- 测试 2: 文件变更检测 ---")
    # mtime 更新：重新写入文件
    time.sleep(0.1)
    with open(test_file, "w", encoding="utf-8") as f:
        f.write("key: value2\n")

    changed = reloader.reload_if_changed()
    assert changed == 1
    assert reloader.reload_count == 1
    print(f"  变更数: {changed}, 重载次数: {reloader.reload_count}")
    print("  ✅ 通过")

    # ---- 测试 3: 无变更不重载 ----
    print("\n--- 测试 3: 无变更不重载 ---")
    changed = reloader.reload_if_changed()
    assert changed == 0
    assert reloader.reload_count == 1  # 未增加
    print("  ✅ 通过")

    # ---- 测试 4: 禁用 ----
    print("\n--- 测试 4: 禁用 ---")
    reloader.enabled = False
    with open(test_file, "w", encoding="utf-8") as f:
        f.write("key: value3\n")
    time.sleep(0.1)
    changed = reloader.reload_if_changed()
    assert changed == 0, f"禁用后应无变更，实际={changed}"
    reloader.enabled = True
    print("  ✅ 通过")

    # ---- 测试 5: 回调触发 ----
    print("\n--- 测试 5: 回调触发 ---")
    callback_results = []

    def _on_change(old, new):
        callback_results.append((old, new))

    reloader.unwatch_all()
    test_file2 = os.path.join(test_dir, "callback_test.yaml")
    with open(test_file2, "w", encoding="utf-8") as f:
        f.write("param: 1\n")
    reloader.watch(test_file2, _on_change)

    time.sleep(0.1)
    with open(test_file2, "w", encoding="utf-8") as f:
        f.write("param: 2\n")
    changed = reloader.reload_if_changed()
    assert changed == 1
    assert len(callback_results) == 1
    assert callback_results[0][1]["param"] == 2
    print(f"  回调触发: old={callback_results[0][0]}, new={callback_results[0][1]}")
    print("  ✅ 通过")

    # ---- 测试 6: 强制重载 ----
    print("\n--- 测试 6: 强制重载 ---")
    with open(test_file2, "w", encoding="utf-8") as f:
        f.write("param: 3\n")
    ok = reloader.force_reload(test_file2)
    assert ok
    assert reloader.reload_count >= 3
    print("  ✅ 通过")

    # ---- 测试 7: 后台线程 ----
    print("\n--- 测试 7: 后台线程 ---")
    assert not reloader.is_running
    reloader.start()
    assert reloader.is_running
    time.sleep(0.5)  # 让线程运行一下
    reloader.stop()
    assert not reloader.is_running
    print("  ✅ 通过")

    # ---- 测试 8: disable_for_autonomous ----
    print("\n--- 测试 8: AUTONOMOUS 禁用 ---")
    reloader.enable_for_debug()
    assert reloader.enabled
    reloader.disable_for_autonomous()
    assert not reloader.enabled
    reloader.enable_for_debug()
    assert reloader.enabled
    print("  ✅ 通过")

    # ---- 测试 9: 统计 ----
    print("\n--- 测试 9: 统计 ---")
    stats = reloader.get_stats()
    print(f"  已监控: {stats['watched_count']} 个文件")
    print(f"  已重载: {stats['reload_count']} 次")
    assert stats["reload_count"] >= 3
    print("  ✅ 通过")

    shutil.rmtree(test_dir, ignore_errors=True)

    print(f"\n{'=' * 60}")
    print("  热加载器 — 全部测试通过 ✅")
    print(f"{'=' * 60}")
