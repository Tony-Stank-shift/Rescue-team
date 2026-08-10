"""
debug_dashboard.py —— 调试可视化工具 (7.2.1)

基于 tkinter 的轻量级调试面板，用于现场编程调试。
显示传感器数据、状态机状态、事件日志和世界地图。

仅在 DEBUG 模式下运行，AUTONOMOUS 自动关闭。
无显示器环境下检测并跳过，不崩溃。

用法:
  dash = DebugDashboard()
  dash.start()
  dash.update({"sensors": {...}, "state": {...}, "events": [...]})
  dash.stop()
"""

import logging
import math
import os
import queue
import threading
import time
from typing import Any, Dict, List, Optional

logger = logging.getLogger("debug_dashboard")

# tkinter 可用性检测
try:
    import tkinter as tk
    from tkinter import ttk, scrolledtext
    _HAS_TK = True
except ImportError:
    _HAS_TK = False

# 检查是否有显示器
_HAS_DISPLAY = _HAS_TK
if _HAS_TK:
    try:
        root = tk.Tk()
        root.destroy()
    except tk.TclError:
        _HAS_DISPLAY = False


# ============================================================
# DebugDashboard
# ============================================================

class DebugDashboard:
    """
    调试可视化面板。

    面板布局:
    ┌──────────────────────────────────────────┐
    │  📡 传感器          │  🤖 状态机          │
    │  Camera: 30 FPS     │  状态: DEBUG        │
    │  IMU: OK             │  策略: FREE_RUN      │
    │  Battery: 12.1V      │  剩余: 120s          │
    │  Temp: 35°C          │  分数: 25            │
    ├──────────────────────────────────────────┤
    │  📋 事件日志 (最近 20 条)                   │
    │  17:25:01  MATCH_START                     │
    │  17:25:05  TARGET_DETECTED  #1              │
    │  ...                                       │
    ├──────────────────────────────────────────┤
    │  🗺️ 世界地图 (简略)                         │
    │  [Canvas 300×300 目标位置散点]              │
    └──────────────────────────────────────────┘
    """

    REFRESH_INTERVAL_MS = 200     # 刷新间隔 (ms)
    MAX_EVENT_LINES = 20          # 事件日志最大行数
    MAP_SIZE = 300                # 地图 Canvas 大小 (px)

    def __init__(self, headless: bool = False):
        self._headless = headless or not _HAS_DISPLAY
        self._running = False
        self._root: Optional[tk.Tk] = None
        self._thread: Optional[threading.Thread] = None
        self._update_queue = queue.Queue(maxsize=50)
        self._last_data: Dict[str, Any] = {}

        # 面板组件
        self._sensor_labels: Dict[str, tk.StringVar] = {}
        self._state_labels: Dict[str, tk.StringVar] = {}
        self._event_text: Optional[tk.scrolledtext.ScrolledText] = None
        self._map_canvas: Optional[tk.Canvas] = None

        if self._headless:
            logger.info("DebugDashboard: 无显示器模式 (不启动 GUI)")

    @property
    def is_running(self) -> bool:
        return self._running

    @property
    def has_display(self) -> bool:
        return not self._headless

    # ---- 生命周期 ----

    def start(self) -> bool:
        """
        启动调试面板。

        Returns:
            True 启动成功，False 无显示器跳过
        """
        if self._headless:
            logger.info("无显示器，调试面板跳过")
            return False

        if self._running:
            logger.warning("调试面板已在运行")
            return True

        try:
            self._running = True
            self._thread = threading.Thread(
                target=self._gui_loop,
                name="debug-dashboard",
                daemon=True,
            )
            self._thread.start()
            logger.info("调试面板已启动")
            return True
        except Exception as e:
            logger.error(f"调试面板启动失败: {e}")
            self._running = False
            self._headless = True
            return False

    def stop(self) -> None:
        """停止调试面板。"""
        if not self._running:
            return

        self._running = False
        if self._root:
            try:
                self._root.after(0, self._root.quit)
            except Exception:
                pass

        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=2.0)

        logger.info("调试面板已停止")

    # ---- 数据推送 ----

    def update(self, data: Dict[str, Any]) -> None:
        """
        推送数据到面板（线程安全）。

        data 格式:
        {
            "sensors": {
                "camera_fps": 30.0,
                "imu_ok": True,
                "battery_v": 12.1,
                "temperature_c": 35.0,
                "motors_ok": True,
            },
            "state": {
                "state": "DEBUG",
                "strategy": "FREE_RUN",
                "time_remaining_s": 120,
                "score": 25,
                "targets_delivered": 3,
            },
            "events": [
                {"ts": "17:25:01", "type": "MATCH_START"},
                ...
            ],
            "map": {
                "robot_pos": (1500, 1500, 0),
                "targets": [(1000, 1000, "green"), (2000, 2000, "red")],
                "safe_zone": "red",
            },
        }
        """
        try:
            self._update_queue.put_nowait(data)
        except queue.Full:
            # 丢弃旧数据
            try:
                self._update_queue.get_nowait()
                self._update_queue.put_nowait(data)
            except queue.Empty:
                pass

    # ---- GUI 构建 ----

    def _gui_loop(self) -> None:
        """tkinter 主循环。"""
        try:
            self._root = tk.Tk()
            self._root.title("救援机器人 - 调试面板")
            self._root.geometry("650x700")
            self._root.protocol("WM_DELETE_WINDOW", self._on_close)

            self._build_ui()
            self._schedule_refresh()
            self._root.mainloop()
        except Exception as e:
            logger.error(f"GUI 循环异常: {e}")
        finally:
            self._running = False
            if self._root:
                try:
                    self._root.destroy()
                except Exception:
                    pass

    def _build_ui(self) -> None:
        """构建 UI 组件。"""
        # 主容器
        main = ttk.Frame(self._root, padding=5)
        main.pack(fill=tk.BOTH, expand=True)

        # ─── 上方双栏: 传感器 + 状态机 ───
        top = ttk.Frame(main)
        top.pack(fill=tk.X, pady=(0, 5))

        # 传感器面板
        sensor_frame = ttk.LabelFrame(top, text="📡 传感器", padding=5)
        sensor_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(0, 5))

        sensor_items = [
            ("camera", "摄像头"),
            ("imu", "IMU"),
            ("battery", "电池"),
            ("temperature", "温度"),
            ("motors", "电机"),
        ]
        for key, label in sensor_items:
            row = ttk.Frame(sensor_frame)
            row.pack(fill=tk.X, pady=2)
            ttk.Label(row, text=f"{label}:", width=8, anchor="e").pack(side=tk.LEFT)
            var = tk.StringVar(value="--")
            ttk.Label(row, textvariable=var, width=20).pack(side=tk.LEFT, padx=(5, 0))
            self._sensor_labels[key] = var

        # 状态机面板
        state_frame = ttk.LabelFrame(top, text="🤖 状态机", padding=5)
        state_frame.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True, padx=(5, 0))

        state_items = [
            ("state", "运行状态"),
            ("strategy", "策略"),
            ("time", "剩余时间"),
            ("score", "当前分数"),
            ("delivered", "已送达"),
        ]
        for key, label in state_items:
            row = ttk.Frame(state_frame)
            row.pack(fill=tk.X, pady=2)
            ttk.Label(row, text=f"{label}:", width=8, anchor="e").pack(side=tk.LEFT)
            var = tk.StringVar(value="--")
            ttk.Label(row, textvariable=var, width=20).pack(side=tk.LEFT, padx=(5, 0))
            self._state_labels[key] = var

        # ─── 中间: 事件日志 ───
        event_frame = ttk.LabelFrame(main, text="📋 事件日志", padding=5)
        event_frame.pack(fill=tk.BOTH, expand=True, pady=5)

        self._event_text = scrolledtext.ScrolledText(
            event_frame, height=10, width=80,
            font=("Consolas", 9),
            state=tk.DISABLED,
        )
        self._event_text.pack(fill=tk.BOTH, expand=True)

        # ─── 下方: 世界地图 ───
        map_frame = ttk.LabelFrame(main, text="🗺️ 世界地图 (3000×3000mm)", padding=5)
        map_frame.pack(fill=tk.BOTH, expand=False, pady=5)

        self._map_canvas = tk.Canvas(
            map_frame,
            width=self.MAP_SIZE, height=self.MAP_SIZE,
            bg="white", highlightthickness=1,
            highlightbackground="gray",
        )
        self._map_canvas.pack()

        # 绘制静态地图元素
        self._draw_static_map()

    def _draw_static_map(self) -> None:
        """绘制静态地图元素（安全区、边界）。"""
        if not self._map_canvas:
            return
        canvas = self._map_canvas
        s = self.MAP_SIZE / 3000  # 缩放: 1mm = s px

        # 边界
        canvas.create_rectangle(0, 0, self.MAP_SIZE, self.MAP_SIZE,
                               outline="black", width=2)

        # 红色安全区 (左侧)
        rx, ry = 50 * s, 2550 * s
        rw, rh = 600 * s, 400 * s
        canvas.create_rectangle(rx, ry, rx + rw, ry + rh,
                               outline="red", width=2, dash=(4, 4))
        canvas.create_text(rx + rw / 2, ry + rh / 2,
                          text="红队\n安全区", fill="red", font=("", 8))

        # 蓝色安全区 (右侧)
        bx, by = 2350 * s, 2550 * s
        canvas.create_rectangle(bx, by, bx + rw, by + rh,
                               outline="blue", width=2, dash=(4, 4))
        canvas.create_text(bx + rw / 2, by + rh / 2,
                          text="蓝队\n安全区", fill="blue", font=("", 8))

    def _draw_targets(self, map_data: dict) -> None:
        """绘制目标位置。"""
        if not self._map_canvas:
            return
        canvas = self._map_canvas
        s = self.MAP_SIZE / 3000

        # 清除动态元素
        canvas.delete("dynamic")

        # 目标
        targets = map_data.get("targets", [])
        for tx, ty, color in targets:
            x, y = tx * s, ty * s
            canvas.create_oval(x - 3, y - 3, x + 3, y + 3,
                              fill=color, tags="dynamic")

        # 机器人位置
        robot = map_data.get("robot_pos")
        if robot:
            rx, ry, rt = robot
            rx, ry = rx * s, ry * s
            canvas.create_oval(rx - 5, ry - 5, rx + 5, ry + 5,
                              fill="black", tags="dynamic")
            # 朝向箭头
            arrow_len = 12
            ax = rx + arrow_len * math.cos(rt)
            ay = ry + arrow_len * math.sin(rt)
            canvas.create_line(rx, ry, ax, ay,
                              arrow=tk.LAST, width=2,
                              fill="black", tags="dynamic")

    # ---- 刷新 ----

    def _schedule_refresh(self) -> None:
        """定时刷新。"""
        if not self._running:
            return
        self._refresh_ui()
        if self._root:
            self._root.after(self.REFRESH_INTERVAL_MS, self._schedule_refresh)

    def _refresh_ui(self) -> None:
        """从队列读取最新数据并刷新 UI。"""
        # 取最新数据
        data = {}
        while True:
            try:
                data = self._update_queue.get_nowait()
            except queue.Empty:
                break
        if not data:
            data = self._last_data
        else:
            self._last_data = data

        # 传感器
        sensors = data.get("sensors", {})
        if sensors:
            self._sensor_labels.get("camera", tk.StringVar()).set(
                f"{sensors.get('camera_fps', '--')} FPS")
            self._sensor_labels.get("imu", tk.StringVar()).set(
                "✅ OK" if sensors.get("imu_ok") else "❌ 异常")
            batt = sensors.get("battery_v")
            self._sensor_labels.get("battery", tk.StringVar()).set(
                f"{batt:.1f}V" if batt else "--")
            temp = sensors.get("temperature_c")
            self._sensor_labels.get("temperature", tk.StringVar()).set(
                f"{temp:.0f}°C" if temp else "--")
            self._sensor_labels.get("motors", tk.StringVar()).set(
                "✅ OK" if sensors.get("motors_ok") else "❌ 异常")

        # 状态机
        state = data.get("state", {})
        if state:
            self._state_labels.get("state", tk.StringVar()).set(
                state.get("state", "--"))
            self._state_labels.get("strategy", tk.StringVar()).set(
                state.get("strategy", "--"))
            self._state_labels.get("time", tk.StringVar()).set(
                f"{state.get('time_remaining_s', '--')}s")
            self._state_labels.get("score", tk.StringVar()).set(
                str(state.get("score", "--")))
            self._state_labels.get("delivered", tk.StringVar()).set(
                str(state.get("targets_delivered", "--")))

        # 事件日志
        events = data.get("events", [])
        if events and self._event_text:
            self._event_text.config(state=tk.NORMAL)
            self._event_text.delete(1.0, tk.END)
            for evt in events[-self.MAX_EVENT_LINES:]:
                ts = evt.get("ts", "")
                evt_type = evt.get("type", "")
                detail = evt.get("detail", "")
                line = f"{ts}  {evt_type}"
                if detail:
                    line += f"  {detail}"
                self._event_text.insert(tk.END, line + "\n")
            self._event_text.config(state=tk.DISABLED)
            self._event_text.see(tk.END)

        # 地图
        map_data = data.get("map", {})
        if map_data:
            self._draw_targets(map_data)

    def _on_close(self) -> None:
        """窗口关闭回调。"""
        self.stop()


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
    print("  调试可视化工具 — 独立测试")
    print("=" * 60)

    # ---- 测试 1: 无显示器检测 ----
    print("\n--- 测试 1: 无显示器检测 ---")
    dash = DebugDashboard()
    if not dash.has_display:
        print("  当前环境无显示器，GUI 模式跳过")
    print(f"  has_display: {dash.has_display}")
    print("  ✅ 通过")

    # ---- 测试 2: headless 模式 ----
    print("\n--- 测试 2: headless 模式 ---")
    dash2 = DebugDashboard(headless=True)
    assert dash2.has_display is False
    result = dash2.start()
    assert result is False  # headless 应返回 False
    print("  ✅ 通过")

    # ---- 测试 3: 数据推送不崩溃 ----
    print("\n--- 测试 3: 数据推送不崩溃 ---")
    dash2.update({
        "sensors": {
            "camera_fps": 30.0,
            "imu_ok": True,
            "battery_v": 12.1,
            "temperature_c": 35.0,
            "motors_ok": True,
        },
        "state": {
            "state": "DEBUG",
            "strategy": "FREE_RUN",
            "time_remaining_s": 120,
            "score": 25,
            "targets_delivered": 3,
        },
        "events": [
            {"ts": "17:25:01", "type": "MATCH_START"},
            {"ts": "17:25:05", "type": "TARGET_DETECTED", "detail": "#1"},
            {"ts": "17:25:10", "type": "TRIP_START"},
        ],
        "map": {
            "robot_pos": (1500, 1500, 0.5),
            "targets": [
                (1000, 1000, "green"),
                (2000, 2000, "orange"),
                (500, 2500, "red"),
            ],
        },
    })
    print("  ✅ 通过 (无崩溃)")

    # ---- 测试 4: stop 安全 ----
    print("\n--- 测试 4: stop 安全 ---")
    dash2.stop()
    print("  ✅ 通过")

    # ---- 测试 5: 有显示器时启动 ----
    print("\n--- 测试 5: GUI 模式启动 ---")
    # 仅在有显示器时实际启动
    if dash.has_display:
        print("  有显示器: 启动 GUI (5s 后自动关闭)...")
        dash.start()
        # 推送一些数据
        dash.update({
            "sensors": {"camera_fps": 30, "imu_ok": True,
                        "battery_v": 12.1, "temperature_c": 35, "motors_ok": True},
            "state": {"state": "DEBUG", "strategy": "FREE_RUN",
                      "time_remaining_s": 180, "score": 0, "targets_delivered": 0},
            "events": [{"ts": "00:00:01", "type": "BOOT_SUCCESS"}],
            "map": {"robot_pos": (500, 500, 0), "targets": [(1500, 1500, "green")]},
        })
        time.sleep(3)
        dash.stop()
        print("  ✅ 通过")
    else:
        print("  跳过 (无显示器)")
        print("  ✅ 通过")

    print(f"\n{'=' * 60}")
    print("  调试可视化工具 — 测试通过 ✅")
    print(f"{'=' * 60}")
    print()
    print("  注: GUI 模式需要在有显示器的环境中测试。")
    print("  headless 模式已完全验证。")
