#!/usr/bin/env python3
"""verify_video_server —— 内嵌实时画面服务（monitoring/video_server）的自检

为什么单独一个自检文件：
    这个模块是**现场调试用**的，唯一硬要求是"**它坏了不许影响比赛**"。
    所以这里测的不是画质，而是四种恶劣情况下的**降级行为**：
      1) 没有摄像头（frame=None）→ 出占位图而不是抛异常/白屏；
      2) 感知取检测框时抛异常 → 退化成原图，推流不断；
      3) HUD 快照抛异常 → 状态栏空着，推流不断；
      4) 端口被占用 → start() 返回 False 并只记日志，**绝不抛**。
    另外验证 HTTP 各端点真的可用（/、/snapshot.jpg、/status、/stream）。
"""

from __future__ import annotations

import json
import socket
import sys
import threading
import time
import traceback
import urllib.request

PASS, FAIL = "PASS", "FAIL"


# ────────────────────────────── 桩件 ──────────────────────────────

class _CamStub:
    """摄像头桩：返回合成帧（模拟"检测到若干色块"的场地）。"""

    def __init__(self):
        self.frames = 0

    def get_frame(self):
        import numpy as np
        self.frames += 1
        img = np.full((480, 640, 3), 90, dtype=np.uint8)
        img[200:280, 260:340] = (0, 200, 0)      # 绿块
        img[300:360, 420:500] = (20, 20, 20)     # 暗块（真机上被误判成核心物资的那种）
        return img


class _CamDead:
    """总是没有帧的摄像头桩。"""

    def get_frame(self):
        return None


class _DetStub:
    def __init__(self, x, y, w, h, color="BLACK", shape="TRIANGULAR_PYRAMID", conf=0.75):
        self.color = type("C", (), {"name": color})()
        self.shape = type("S", (), {"name": shape})()
        self.confidence = conf
        self.bbox = (x, y, w, h)


class _PercStub:
    def __init__(self, dets=None, raise_on_read=False):
        self._dets = dets if dets is not None else [_DetStub(420, 300, 80, 60)]
        self._raise = raise_on_read

    @property
    def last_detections(self):
        if self._raise:
            raise RuntimeError("模拟感知取检测框失败")
        return list(self._dets)


def _free_port() -> int:
    s = socket.socket()
    s.bind(("127.0.0.1", 0))
    p = s.getsockname()[1]
    s.close()
    return p


def _opener():
    """**强制绕过代理**的 urllib opener。

    ⚠️ 为什么必须显式绕：本机 shell 常带 `http_proxy=http://127.0.0.1:7890`，
    而 Python 的 `urllib` **不认识 `no_proxy` 里的通配符写法**（如 `127.*`），
    于是连 `http://127.0.0.1:<port>/` 都会被丢给代理 → `Connection refused`。
    实测症状极具迷惑性：**裸 socket 连得通、urllib 却拒连**，
    看起来像"服务没起来"，其实只是环境变量。护栏不能依赖现场环境。
    """
    return urllib.request.build_opener(urllib.request.ProxyHandler({}))


def _get(url: str, timeout: float = 5.0):
    with _opener().open(url, timeout=timeout) as r:
        return r.status, r.read(), dict(r.headers)


def _mk(cam, perc, hud, port):
    from rescue_robot.monitoring.video_server import MjpegServer
    return MjpegServer(camera=cam, perception=perc, hud_provider=hud,
                       host="127.0.0.1", port=port, fps=15.0)


# ────────────────────────────── 场景 ──────────────────────────────

def s_endpoints(ev):
    """正常路径：四个端点都要能用，且检测框要真的画进去。"""
    port = _free_port()
    srv = _mk(_CamStub(), _PercStub(), lambda: {"pose": (1.0, 2.0, 3.0)}, port)
    if not srv.start():
        return f"3 号端口 {port} 上启动失败（自检自己有问题）"
    try:
        st, body, hdr = _get(f"http://127.0.0.1:{port}/")
        if st != 200 or b"<img src=\"/stream\"" not in body:
            return "首页不含 /stream 的 <img> → 浏览器看不到画面"
        st, jpg, _ = _get(f"http://127.0.0.1:{port}/snapshot.jpg")
        if st != 200 or not jpg.startswith(b"\xff\xd8"):
            return "snapshot.jpg 不是合法 JPEG（JPEG 以 FFD8 开头）"
        st, raw, _ = _get(f"http://127.0.0.1:{port}/status")
        d = json.loads(raw.decode("utf-8"))
        if d.get("detection_count") != 1 or d["detections"][0]["color"] != "BLACK":
            return f"/status 里的检测列表不对: {d.get('detections')}"
        # 叠加验证：标注过的帧应当与被标注的原图不同（检测框画上去了）
        plain = srv.frame_jpeg(annotate=False)
        marked = srv.frame_jpeg(annotate=True)
        if plain == marked:
            return "叠加检测框后 JPEG 完全一样 → 检测框没画上去（现场就看不出检测位置）"
        ev.append(f"端点：/ 200、/snapshot.jpg {len(jpg)}B、/status 检测 {d['detection_count']} 个、"
                  f"叠加生效（{len(plain)}B → {len(marked)}B）")
        # /stream 的分片边界
        req = urllib.request.Request(f"http://127.0.0.1:{port}/stream")
        with _opener().open(req, timeout=5.0) as r:
            if b"multipart/x-mixed-replace" not in r.headers.get("Content-Type", "").encode():
                return "/stream 的 Content-Type 不是 multipart/x-mixed-replace"
            chunk = r.read(4096)
        if b"--frame" not in chunk or b"Content-Type: image/jpeg" not in chunk:
            return "/stream 没按 MJPEG 分片（浏览器会一直转圈）"
        ev.append("/stream：multipart/x-mixed-replace + --frame 分片正常")
        return None
    except Exception:
        return "端点自检抛异常（自检或服务有 bug）:\n" + traceback.format_exc()
    finally:
        srv.stop()


def s_no_camera(ev):
    """没有摄像头帧：必须出占位图，不是异常/白屏。"""
    port = _free_port()
    srv = _mk(_CamDead(), _PercStub(), lambda: {"pose": (0, 0, 0)}, port)
    if not srv.start():
        return f"端口 {port} 启动失败"
    try:
        jpg = srv.frame_jpeg()
        if not jpg.startswith(b"\xff\xd8"):
            return "无帧时没有产出可显示的占位图"
        if srv.stats["encode_failures"] > 0:
            return "无帧路径出现编码失败"
        st, _, _ = _get(f"http://127.0.0.1:{port}/snapshot.jpg")
        if st != 200:
            return f"无帧时 /snapshot.jpg 返回 {st}（应当给占位图）"
        ev.append("无帧：出占位图，端点仍 200（浏览器能看出'服务活着但没帧'）")
        return None
    finally:
        srv.stop()


def s_perception_raises(ev):
    """感知取检测框抛异常：必须退化成原图继续推流，不许影响主程序。"""
    port = _free_port()
    srv = _mk(_CamStub(), _PercStub(raise_on_read=True), lambda: {}, port)
    if not srv.start():
        return f"端口 {port} 启动失败"
    try:
        jpg = srv.frame_jpeg()
        if not jpg.startswith(b"\xff\xd8"):
            return "感知抛异常后连原图都推不出来了"
        st, _, _ = _get(f"http://127.0.0.1:{port}/status")
        if st != 200:
            return f"感知抛异常后 /status 返回 {st}"
        ev.append("感知抛异常：退化成原图继续推流，/status 仍 200")
        return None
    finally:
        srv.stop()


def s_hud_raises(ev):
    """HUD 快照抛异常：只有状态栏变空，画面不受影响。"""
    def _boom():
        raise RuntimeError("模拟状态快照失败")
    port = _free_port()
    srv = _mk(_CamStub(), _PercStub(), _boom, port)
    if not srv.start():
        return f"端口 {port} 启动失败"
    try:
        jpg = srv.frame_jpeg()
        if not jpg.startswith(b"\xff\xd8"):
            return "HUD 抛异常后画面都没了"
        if srv._hud_lines() != []:
            return "HUD 抛异常时应返回空行列表"
        ev.append("HUD 抛异常：画面正常、叠加行为空（只丢状态栏）")
        return None
    finally:
        srv.stop()


def s_port_busy(ev):
    """端口被占：start() 必须返回 False 且**不抛异常**（否则会把整场比赛带崩）。"""
    port = _free_port()
    holder = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    holder.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    holder.bind(("127.0.0.1", port))
    holder.listen(1)
    try:
        srv = _mk(_CamStub(), _PercStub(), lambda: {}, port)
        try:
            ok = srv.start()
        except Exception:
            return "端口被占时 start() 抛了异常（会把 main 的启动流程带崩）:\n" \
                   + traceback.format_exc()
        if ok:
            return "端口被占却报告启动成功"
        if not srv.stats["last_error"]:
            return "端口被占但没有记录错误原因（现场无法定位）"
        ev.append(f"端口被占：start() 返回 False 且未抛，错误已记录 "
                  f"({srv.stats['last_error'][:40]})")
        return None
    finally:
        holder.close()


def s_does_not_starve_loop(ev):
    """最关键的一条：推流不能让 50Hz 控制循环挨饿。

    模拟：一边开推流客户端，一边跑一个 50Hz（20ms 周期）的"控制循环"，
    统计实际周期抖动。要求平均周期不劣化超过 20%。
    """
    port = _free_port()
    srv = _mk(_CamStub(), _PercStub(), lambda: {"pose": (0, 0, 0)}, port)
    if not srv.start():
        return f"端口 {port} 启动失败"
    stop = threading.Event()

    def _consume():
        try:
            req = urllib.request.Request(f"http://127.0.0.1:{port}/stream")
            with _opener().open(req, timeout=5.0) as r:
                while not stop.is_set():
                    if not r.read(8192):
                        break
        except Exception:
            pass

    clients = [threading.Thread(target=_consume, daemon=True) for _ in range(3)]
    for c in clients:
        c.start()
    try:
        time.sleep(0.6)                      # 让客户端连上并开始拉流
        N, dt = 100, 0.02
        t0 = time.time()
        over = 0
        for _ in range(N):
            target = t0 + (_ + 1) * dt
            now = time.time()
            if now < target:
                time.sleep(target - now)
            if time.time() - target > 0.010:   # 单周期超时 10ms = 明显被饿
                over += 1
        elapsed = time.time() - t0
        expected = N * dt
        ratio = elapsed / expected
        if over > N * 0.05:
            return (f"3 个推流客户端下，100 个 20ms 周期里有 {over} 个超时 >10ms "
                    f"→ 控制循环被推流饿到了")
        if ratio > 1.25:
            return f"控制循环耗时被拉长到 {ratio:.2f}×（要求 ≤1.25×）"
        ev.append(f"不饿主循环：3 客户端推流下 100 周期耗时 {ratio:.2f}× 、"
                  f"超时(>10ms) {over} 个")
        return None
    finally:
        stop.set()
        time.sleep(0.2)
        srv.stop()


def s_stop_clean(ev):
    """stop() 要能真正释放端口。"""
    port = _free_port()
    srv = _mk(_CamStub(), _PercStub(), lambda: {}, port)
    if not srv.start():
        return "启动失败"
    srv.stop()
    try:
        s2 = _mk(_CamStub(), _PercStub(), lambda: {}, port)
        ok = s2.start()
        s2.stop()
        if not ok:
            return "stop() 之后端口没释放 → 下次启动 run.sh 会起不来"
        ev.append("stop() 释放端口，可重复启动")
        return None
    except Exception:
        return "stop() 后重启抛异常:\n" + traceback.format_exc()


SCENARIOS = (
    ("HTTP 端点可用 + 检测框叠加", s_endpoints),
    ("无摄像头 → 占位图", s_no_camera),
    ("感知抛异常 → 降级继续推流", s_perception_raises),
    ("HUD 抛异常 → 只丢状态栏", s_hud_raises),
    ("端口被占 → 不抛异常、只降级", s_port_busy),
    ("★ 推流不饿 50Hz 控制循环", s_does_not_starve_loop),
    ("stop() 干净释放端口", s_stop_clean),
)


def main() -> int:
    print("=" * 78)
    print("  verify_video_server —— 内嵌实时画面服务的降级与不干扰自检")
    print("=" * 78)
    ev, failures = [], []
    for name, fn in SCENARIOS:
        try:
            problem = fn(ev)
        except Exception:
            problem = "护栏自身抛异常:\n" + traceback.format_exc()
        status = PASS if problem is None else FAIL
        print(f"  [{status}] {name}")
        if problem is not None:
            failures.append(name)
            for line in problem.splitlines():
                print(f"         {line}")
    print("-" * 78)
    for line in ev:
        print(f"  · {line}")
    print("-" * 78)
    if failures:
        print(f"  结果: {len(SCENARIOS) - len(failures)}/{len(SCENARIOS)} 通过，"
              f"{len(failures)} 个失败")
        print("  ❌ 实时画面服务存在会影响主程序的缺陷")
        return 1
    print(f"  结果: {len(SCENARIOS)}/{len(SCENARIOS)} 通过")
    print("  ✅ 实时画面服务可用，且各种恶劣情况下都会降级而不会影响主程序")
    return 0


if __name__ == "__main__":
    sys.exit(main())
