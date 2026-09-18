"""
video_server.py —— 内嵌 MJPEG 实时画面服务（真机联调用）

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
为什么必须"内嵌"而不是单独写个看图程序
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

1. **摄像头是独占资源**。V4L2 下 ``/dev/video0`` 通常只允许一个进程打开；
   另起一个脚本去看图，要么打不开，要么把正在跑的主程序的采集踢掉。
   所以这里**只读取主程序已经在用的那一帧**（``CameraReader.get_frame()``），
   自己绝不碰摄像头。

2. **RDK 没有显示器**。这台机器是通过 SSH 用的，``cv2.imshow`` 会直接失败
   （无 ``$DISPLAY``），SSH X11 转发又要笔记本装 X server。
   改成 HTTP + MJPEG：**任意浏览器**打开 ``http://<RDK_IP>:8080/`` 就是实时画面，
   笔记本/手机/平板都行，零安装。

3. **它绝不能拖慢 50Hz 控制循环**。本模块的所有工作（JPEG 编码、HTTP、
   每个客户端一个线程）都在**独立守护线程**里；控制循环只通过
   ``get_frame()``（本来就带锁、非阻塞）与 ``hud_provider()``（读几个标量）
   与它交互。端口被占、编码失败、浏览器断开……一律吞掉并降级，**不允许**
   把异常带回控制循环。

4. **要看的是"检测器看到了什么"**，不是原始画面。所以流上默认叠加
   检测框 + 颜色/形状/置信度（来自 ``PerceptionPipeline`` 的最近一帧检测），
   这正是"为什么把安全区里的东西当成物资"这类问题的现场证据。

用法（正常由 ``main.py`` 自动启动，不需要手动调）::

    srv = MjpegServer(camera=cam, perception=perception, hud_provider=snapshot)
    srv.start()          # 失败只记日志，不抛
    ...
    srv.stop()
"""

from __future__ import annotations

import json
import logging
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Callable, Optional

logger = logging.getLogger("video_server")

#: 检测框叠加用的 BGR 颜色（按颜色枚举名索引，取不到时用白色）
_BOX_COLORS = {
    "GREEN": (0, 220, 0),
    "BLACK": (255, 255, 255),
    "ORANGE": (0, 140, 255),
    "LIGHT_BLUE": (255, 220, 120),
    "RED": (0, 0, 255),
    "BLUE": (255, 0, 0),
    "PURPLE": (200, 0, 200),
    "BROWN": (42, 87, 128),
}


class MjpegServer:
    """把主程序正在用的摄像头帧以 MJPEG 推给浏览器。"""

    #: 每客户端最大推流帧率（真机 CPU 有限，默认 12 够看清了）
    DEFAULT_FPS = 12.0
    #: JPEG 质量
    DEFAULT_QUALITY = 70
    #: 单帧编码前的最大宽度（超过则等比缩小，省 CPU/带宽）
    DEFAULT_MAX_WIDTH = 800

    def __init__(self,
                 camera=None,
                 perception=None,
                 hud_provider: Optional[Callable[[], dict]] = None,
                 host: str = "0.0.0.0",
                 port: int = 8080,
                 fps: float = DEFAULT_FPS,
                 quality: int = DEFAULT_QUALITY,
                 max_width: int = DEFAULT_MAX_WIDTH,
                 annotate: bool = True):
        """
        Args:
            camera: ``CameraReader``（只需有 ``get_frame()``）；None → 只显示占位图
            perception: ``PerceptionPipeline``（用于取最近一帧检测框）；可为 None
            hud_provider: 无参可调用，返回一份状态 dict（见 ``hud_snapshot``）；
                **必须足够快**（读几个属性），每次 /status 或每帧叠加时调用
            host/port: 监听地址；0.0.0.0 表示局域网都能看
            fps: 推流帧率上限
            quality: JPEG 质量 1~100
            max_width: 编码前缩放上限
            annotate: 是否叠加检测框
        """
        self._camera = camera
        self._perception = perception
        self._hud = hud_provider
        self._host = host
        self._port = int(port)
        self._fps = max(1.0, float(fps))
        self._quality = max(10, min(100, int(quality)))
        self._max_width = int(max_width)
        self._annotate = bool(annotate)

        self._httpd: Optional[ThreadingHTTPServer] = None
        self._thread: Optional[threading.Thread] = None
        self._running = False
        self._clients = 0
        self._frames_served = 0
        self._encode_failures = 0
        self._last_error = ""

    # ────────────────────────────── 生命周期 ──────────────────────────────

    @property
    def url(self) -> str:
        """给人看的访问地址（拿不到本机 IP 时退化成 127.0.0.1）。"""
        host = self._local_ip() or "127.0.0.1"
        return f"http://{host}:{self._port}/"

    @staticmethod
    def _local_ip() -> str:
        """取本机在局域网里的 IP（不真的发包，只让内核选出口地址）。"""
        import socket
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            try:
                s.connect(("192.168.50.1", 9))
                return s.getsockname()[0]
            finally:
                s.close()
        except Exception:
            return ""

    def start(self) -> bool:
        """启动服务；失败只记日志并返回 False（**绝不抛异常**）。"""
        if self._running:
            return True
        try:
            handler = self._make_handler()
            httpd = ThreadingHTTPServer((self._host, self._port), handler)
            httpd.daemon_threads = True
            self._httpd = httpd
            self._running = True
            self._thread = threading.Thread(
                target=self._serve_forever, name="mjpeg-http", daemon=True)
            self._thread.start()
            logger.info(f"📹 实时画面已启动: {self.url}"
                        f"（{self._fps:.0f}fps, JPEG q{self._quality}, "
                        f"叠加检测框={'是' if self._annotate else '否'}）")
            logger.info(f"   ↳ 笔记本浏览器打开 {self.url} 即可；"
                        f"不要就设 VIDEO_STREAM=0 关掉")
            return True
        except OSError as e:
            self._last_error = f"{type(e).__name__}: {e}"
            logger.warning(f"实时画面启动失败（端口 {self._port} 可能被占）: {e}"
                           f" → 不影响比赛，继续运行")
            return False
        except Exception as e:
            self._last_error = f"{type(e).__name__}: {e}"
            logger.warning(f"实时画面启动失败: {e} → 不影响比赛，继续运行")
            return False

    def _serve_forever(self) -> None:
        try:
            self._httpd.serve_forever(poll_interval=0.2)
        except Exception as e:
            logger.debug(f"实时画面服务退出: {e}")

    def stop(self) -> None:
        self._running = False
        if self._httpd is not None:
            try:
                self._httpd.shutdown()
                self._httpd.server_close()
            except Exception:
                pass
            self._httpd = None
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None
        logger.info(f"实时画面已停止（共推 {self._frames_served} 帧）")

    @property
    def stats(self) -> dict:
        return {"clients": self._clients, "frames": self._frames_served,
                "encode_failures": self._encode_failures,
                "last_error": self._last_error, "running": self._running}

    # ────────────────────────────── 取帧与叠加 ──────────────────────────────

    def _grab(self):
        """取主程序当前的帧（**不打开摄像头**）；没有则返回 None。"""
        if self._camera is None:
            return None
        try:
            getter = getattr(self._camera, "get_frame", None)
            return getter() if callable(getter) else None
        except Exception:
            return None

    def _detections(self):
        """取最近一帧检测结果（用于叠加框）；拿不到就返回空。"""
        if self._perception is None:
            return []
        try:
            return list(getattr(self._perception, "last_detections", []) or [])
        except Exception:
            return []

    def _placeholder(self, text: str):
        """没有帧时给一张占位图，让浏览器知道服务是活的（而不是白屏）。"""
        import numpy as np
        img = np.full((360, 640, 3), 28, dtype=np.uint8)
        try:
            import cv2
            cv2.putText(img, text, (24, 170), cv2.FONT_HERSHEY_SIMPLEX,
                        0.9, (0, 220, 255), 2, cv2.LINE_AA)
            cv2.putText(img, time.strftime("%H:%M:%S"), (24, 210),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (170, 170, 170), 2, cv2.LINE_AA)
        except Exception:
            pass
        return img

    def _annotate_frame(self, frame):
        """叠加检测框 + HUD 行。**只读** perception/hud，不改任何控制状态。"""
        import cv2
        img = frame
        for det in self._detections():
            try:
                x, y, w, h = getattr(det, "bbox", None) or (0, 0, 0, 0)
                cname = getattr(getattr(det, "color", None), "name", "?")
                sname = getattr(getattr(det, "shape", None), "name", "?")
                conf = float(getattr(det, "confidence", 0.0) or 0.0)
                col = _BOX_COLORS.get(cname, (255, 255, 255))
                cv2.rectangle(img, (int(x), int(y)), (int(x + w), int(y + h)), col, 2)
                label = f"{cname}/{sname} {conf:.2f}"
                cv2.putText(img, label, (int(x), max(14, int(y) - 6)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, col, 1, cv2.LINE_AA)
            except Exception:
                continue

        lines = self._hud_lines()
        for i, txt in enumerate(lines):
            y = 22 + i * 20
            # 先描黑边再写字，保证在亮/暗背景上都看得清
            cv2.putText(img, txt, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                        (0, 0, 0), 4, cv2.LINE_AA)
            cv2.putText(img, txt, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                        (0, 255, 120), 1, cv2.LINE_AA)

        # ── 套取框 ROI 叠加（`Camera.SLEEVE_ROI`）────────────────────────────
        # 为什么必须画出来：`SLEEVE_ROI` 是**必须真机标定**的量，标错的后果极隐蔽 ——
        # 2026-09-18 现场：夹爪明明下压了（`SERVO,LOWER 套住: 1 个目标`），
        # 视觉确认却判"套取框内未见目标" → 判套取失败 → 抬爪后退 →
        # 从外面看就是"它根本不去抓"。根因是 ROI 还是**旧夹爪**标的值，
        # 而夹爪 V2 换成了 150×100 方口、后方三块实心板。
        # 把它画在实时画面上，标定就变成"把物资放进槽里，看黄框有没有套住它"，
        # 同时这里按 `check_sleeve_occupied` 的**同一判据**统计框内命中数，
        # 直接显示它此刻会判 OCCUPIED 还是 EMPTY。
        try:
            from .. import config as _cfg
            roi = getattr(_cfg, "SLEEVE_ROI", None)
            if roi and len(roi) == 4:
                ih, iw = img.shape[:2]
                x1, y1, x2, y2 = roi
                p1 = (int(x1 * iw), int(y1 * ih))
                p2 = (int(x2 * iw), int(y2 * ih))
                cv2.rectangle(img, p1, p2, (0, 255, 255), 2)
                n_in = 0
                for det in self._detections():
                    try:
                        cx, cy = det.center_pixel
                        if (p1[0] <= cx <= p2[0] and p1[1] <= cy <= p2[1]
                                and float(det.confidence) >= 0.35):
                            n_in += 1
                            cv2.circle(img, (int(cx), int(cy)), 6, (0, 255, 255), 2)
                    except Exception:
                        continue
                on = bool(getattr(_cfg, "SLEEVE_CONFIRM", True))
                tag = (f"SLEEVE_ROI {'ON' if on else 'OFF'} in={n_in} "
                       f"-> {'OCCUPIED' if n_in else 'EMPTY'}")
                cv2.putText(img, tag, (p1[0], max(16, p1[1] - 8)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 4, cv2.LINE_AA)
                cv2.putText(img, tag, (p1[0], max(16, p1[1] - 8)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1, cv2.LINE_AA)
        except Exception:
            pass
        return img

    def _hud_lines(self):
        """把状态压成几行 ASCII 文字（``cv2.putText`` 不支持中文）。"""
        d = self._snapshot()
        if not d:
            return []
        lines = []
        try:
            p = d.get("pose") or (0.0, 0.0, 0.0)
            lines.append(f"pose=({p[0]:.0f},{p[1]:.0f}) th={p[2]:+.0f}deg")
            c = d.get("cmd") or (0.0, 0.0)
            lines.append(f"cmd v={c[0]:+.0f}mm/s w={c[1]:+.2f}rad/s")
            lines.append(f"state={d.get('robot_state','?')} "
                         f"nav={d.get('nav_state','?')} "
                         f"trans={d.get('transport','?')}")
            nt = d.get("nav_target")
            lines.append(f"nav_target={'(%.0f,%.0f)' % (nt[0], nt[1]) if nt else 'None'}")
            lines.append(f"t={d.get('time_remaining', 0.0):.0f}s "
                         f"targets={d.get('target_count', 0)} "
                         f"dets={len(self._detections())}")
            if d.get("vision_errors"):
                lines.append(f"VISION_ERR={d['vision_errors']}")
            if d.get("loop_errors"):
                lines.append(f"LOOP_ERR={d['loop_errors']}")
        except Exception:
            pass
        return lines

    def _snapshot(self) -> dict:
        if self._hud is None:
            return {}
        try:
            d = self._hud()
            return d if isinstance(d, dict) else {}
        except Exception:
            return {}

    def _encode(self, img) -> Optional[bytes]:
        """BGR → JPEG 字节；失败返回 None（调用方会退化成占位图）。"""
        try:
            import cv2
            h, w = img.shape[:2]
            if self._max_width and w > self._max_width:
                scale = self._max_width / float(w)
                img = cv2.resize(img, (self._max_width, max(1, int(h * scale))),
                                 interpolation=cv2.INTER_AREA)
            ok, buf = cv2.imencode(".jpg", img,
                                   [int(cv2.IMWRITE_JPEG_QUALITY), self._quality])
            if not ok:
                self._encode_failures += 1
                return None
            return buf.tobytes()
        except Exception as e:
            self._encode_failures += 1
            self._last_error = f"{type(e).__name__}: {e}"
            return None

    def frame_jpeg(self, annotate: Optional[bool] = None) -> bytes:
        """取当前帧并编码成 JPEG（**任何失败都返回一张占位图，绝不抛**）。"""
        frame = self._grab()
        if frame is None:
            return self._encode(self._placeholder("WAITING FOR CAMERA FRAME...")) or b""
        use_annotate = self._annotate if annotate is None else annotate
        if use_annotate:
            try:
                frame = self._annotate_frame(frame.copy())
            except Exception:
                pass          # 叠加失败就用原图，绝不让它影响推流
        data = self._encode(frame)
        if data is None:
            return self._encode(self._placeholder("JPEG ENCODE FAILED")) or b""
        self._frames_served += 1
        return data

    # ────────────────────────────── HTTP ──────────────────────────────

    PAGE = """<!DOCTYPE html>
<html lang="zh-CN"><head><meta charset="utf-8">
<title>救援机器人 实时画面</title>
<style>
 body{margin:0;background:#111;color:#ddd;font:14px/1.6 system-ui,"Microsoft YaHei",sans-serif}
 .wrap{display:flex;flex-wrap:wrap;gap:16px;padding:16px}
 img{max-width:min(100%,900px);border:1px solid #333;background:#000}
 table{border-collapse:collapse;min-width:320px}
 td{padding:2px 10px 2px 0;vertical-align:top}
 td.k{color:#888;white-space:nowrap}
 h1{font-size:16px;margin:16px 16px 0;color:#0f8}
 .warn{color:#fa0} .err{color:#f55}
</style></head>
<body>
<h1>救援机器人 · 实时画面（只读监视，不参与控制）</h1>
<div class="wrap">
  <img src="/stream" alt="live">
  <div><table id="st"></table></div>
</div>
<script>
async function tick(){
  try{
    const d = await (await fetch('/status')).json();
    const rows = [];
    const add=(k,v,cls)=>rows.push(`<tr><td class="k">${k}</td><td class="${cls||''}">${v}</td></tr>`);
    add('运行状态', d.robot_state || '-');
    add('位姿', d.pose ? `(${d.pose[0].toFixed(0)}, ${d.pose[1].toFixed(0)}) θ=${d.pose[2].toFixed(1)}°` : '-');
    add('下发速度', d.cmd ? `v=${d.cmd[0].toFixed(0)}mm/s w=${d.cmd[1].toFixed(2)}rad/s` : '-');
    add('导航', `${d.nav_state||'-'} → ${d.nav_target?`(${d.nav_target[0].toFixed(0)}, ${d.nav_target[1].toFixed(0)})`:'无目标'}`);
    add('转运', d.transport || '-');
    add('策略', d.strategy || '-');
    add('剩余时间', d.time_remaining!=null ? d.time_remaining.toFixed(0)+' s' : '-');
    add('世界地图目标', d.target_count != null ? d.target_count : '-');
    add('本帧检测数', d.detection_count != null ? d.detection_count : '-');
    if (d.detections && d.detections.length)
      add('检测', d.detections.map(x=>`<span>${x.color}/${x.shape} ${(+x.conf).toFixed(2)}</span>`).join('<br>'));
    if (d.vision_errors) add('视觉异常累计', d.vision_errors, 'warn');
    if (d.loop_errors) add('主循环异常累计', d.loop_errors, 'err');
    add('推流', `${d.stream.frames} 帧 / ${d.stream.clients} 客户端`);
    if (d.stream.last_error) add('流错误', d.stream.last_error, 'err');
    document.getElementById('st').innerHTML = rows.join('');
  }catch(e){}
}
setInterval(tick, 500); tick();
</script>
</body></html>
"""

    def _make_handler(self):
        srv = self

        class _Handler(BaseHTTPRequestHandler):
            protocol_version = "HTTP/1.1"

            def log_message(self, fmt, *args):
                pass                      # 静音：别把控制循环的日志刷掉

            def _send(self, code, ctype, body: bytes, extra=None):
                self.send_response(code)
                self.send_header("Content-Type", ctype)
                self.send_header("Content-Length", str(len(body)))
                self.send_header("Cache-Control", "no-store")
                for k, v in (extra or {}).items():
                    self.send_header(k, v)
                self.end_headers()
                self.wfile.write(body)

            def do_GET(self):
                path = self.path.split("?")[0]
                try:
                    if path in ("/", "/index.html"):
                        self._send(200, "text/html; charset=utf-8",
                                   srv.PAGE.encode("utf-8"))
                    elif path == "/snapshot.jpg":
                        self._send(200, "image/jpeg", srv.frame_jpeg())
                    elif path == "/stream":
                        raw = "raw=1" in self.path
                        self._stream(raw=raw)
                    elif path == "/status":
                        self._send(200, "application/json; charset=utf-8",
                                   json.dumps(srv.status_payload(),
                                              ensure_ascii=False).encode("utf-8"))
                    else:
                        self._send(404, "text/plain; charset=utf-8",
                                   b"404  (try /  /stream  /snapshot.jpg  /status)")
                except (BrokenPipeError, ConnectionResetError):
                    pass                  # 浏览器关页面，正常
                except Exception as e:
                    logger.debug(f"HTTP 处理失败 {path}: {e}")

            def _stream(self, raw: bool = False):
                boundary = "frame"
                self.send_response(200)
                self.send_header("Content-Type",
                                 f"multipart/x-mixed-replace; boundary={boundary}")
                self.send_header("Cache-Control", "no-store")
                self.end_headers()
                srv._clients += 1
                period = 1.0 / srv._fps
                try:
                    while srv._running:
                        t0 = time.time()
                        jpg = srv.frame_jpeg(annotate=(not raw))
                        if jpg:
                            self.wfile.write(b"--" + boundary.encode() + b"\r\n")
                            self.wfile.write(b"Content-Type: image/jpeg\r\n")
                            self.wfile.write(b"Content-Length: "
                                             + str(len(jpg)).encode() + b"\r\n\r\n")
                            self.wfile.write(jpg)
                            self.wfile.write(b"\r\n")
                        dt = time.time() - t0
                        if dt < period:
                            time.sleep(period - dt)
                except (BrokenPipeError, ConnectionResetError):
                    pass
                except Exception as e:
                    logger.debug(f"推流结束: {e}")
                finally:
                    srv._clients = max(0, srv._clients - 1)

        return _Handler

    def status_payload(self) -> dict:
        """给网页状态栏用的 JSON（全部来自 hud_provider，只读）。"""
        d = dict(self._snapshot())
        dets = []
        for det in self._detections():
            try:
                dets.append({
                    "color": getattr(getattr(det, "color", None), "name", "?"),
                    "shape": getattr(getattr(det, "shape", None), "name", "?"),
                    "conf": round(float(getattr(det, "confidence", 0.0) or 0.0), 2),
                    "bbox": list(getattr(det, "bbox", (0, 0, 0, 0))),
                })
            except Exception:
                continue
        d["detection_count"] = len(dets)
        d["detections"] = dets[:12]
        d["stream"] = self.stats
        return d
