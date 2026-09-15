"""
camera_reader.py —— 摄像头后台采集线程

背景（真机联调暴露的问题）：
    直接在 50Hz 主循环里调用 ``cv2.VideoCapture.read()`` 会 **阻塞**：
    摄像头刚启动/曝光未就绪/掉线时 ``read()`` 会一直等待，导致整个
    主循环卡死（"原地去世"）。同时 ``frame=None`` 传给 CVDetector 会抛异常，
    使该轮 ``_run_once`` 整体跳过（决策/导航/转运全不执行）。

本模块把采集与消费解耦：
    - 后台线程持续 ``read()``，只保留**最新一帧**；
    - 主循环用 :meth:`get_frame` **非阻塞**取帧（拿不到就是 None，不等待）；
    - 连续读失败自动重连，掉线不会拖死主循环。

用法::

    cam = CameraReader(index=0)
    cam.start()
    if cam.wait_first_frame(timeout=3.0):     # 预热：等首帧
        frame = cam.get_frame()                # 主循环里非阻塞取帧
    cam.stop()
"""

import logging
import threading
import time
from typing import Optional, Tuple

logger = logging.getLogger("camera_reader")


class CameraReader:
    """后台线程持续采集，主循环非阻塞取最新帧。"""

    #: 连续失败多少次后尝试重连摄像头
    RECONNECT_AFTER_FAILS = 10
    #: 重连间隔（秒）
    RECONNECT_COOLDOWN_S = 1.0

    def __init__(self,
                 index: int = 0,
                 width: Optional[int] = None,
                 height: Optional[int] = None,
                 name: str = "camera"):
        self._index = index
        self._width = width
        self._height = height
        self._name = name

        self._cap = None
        self._frame = None                 # 最新帧
        self._frame_ts = 0.0               # 最新帧时间戳
        self._lock = threading.Lock()
        self._thread: Optional[threading.Thread] = None
        self._running = False
        self._fail_count = 0
        self._frame_count = 0

    # ---- 生命周期 ----

    def _open(self) -> bool:
        """打开（或重开）摄像头。"""
        try:
            import cv2
        except ImportError:
            logger.error("OpenCV (cv2) 未安装，摄像头不可用")
            return False
        try:
            cap = cv2.VideoCapture(self._index)
            if not cap.isOpened():
                cap.release()
                return False
            # 只留 1 帧缓冲：避免取到过期帧（延迟更小）
            try:
                cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            except Exception:
                pass
            if self._width:
                cap.set(cv2.CAP_PROP_FRAME_WIDTH, self._width)
            if self._height:
                cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self._height)
            self._cap = cap
            return True
        except Exception as e:
            logger.warning(f"打开摄像头失败 (index={self._index}): {e}")
            return False

    def start(self) -> bool:
        """启动采集线程（不等待首帧）。"""
        if self._running:
            return True
        if not self._open():
            return False
        self._running = True
        self._thread = threading.Thread(
            target=self._loop, name=f"{self._name}-reader", daemon=True)
        self._thread.start()
        logger.info(f"摄像头采集线程已启动 (index={self._index})")
        return True

    def stop(self) -> None:
        """停止采集线程并释放摄像头。"""
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None
        if self._cap is not None:
            try:
                self._cap.release()
            except Exception:
                pass
            self._cap = None
        logger.info("摄像头采集线程已停止")

    @property
    def is_open(self) -> bool:
        return self._cap is not None

    @property
    def frame_count(self) -> int:
        return self._frame_count

    # ---- 采集线程 ----

    def _loop(self) -> None:
        last_reconnect = 0.0
        while self._running:
            cap = self._cap
            if cap is None:
                # 尝试重连（带冷却，避免疯狂重试）
                now = time.time()
                if now - last_reconnect >= self.RECONNECT_COOLDOWN_S:
                    last_reconnect = now
                    if self._open():
                        logger.info("摄像头已重连")
                else:
                    time.sleep(0.05)
                continue

            try:
                ok, frame = cap.read()
            except Exception as e:
                logger.warning(f"摄像头读取异常: {e}")
                ok, frame = False, None

            if ok and frame is not None:
                with self._lock:
                    self._frame = frame
                    self._frame_ts = time.time()
                    self._frame_count += 1
                self._fail_count = 0
            else:
                self._fail_count += 1
                if self._fail_count >= self.RECONNECT_AFTER_FAILS:
                    logger.warning(
                        f"摄像头连续 {self._fail_count} 次读帧失败，尝试重连")
                    self._fail_count = 0
                    try:
                        cap.release()
                    except Exception:
                        pass
                    self._cap = None
                time.sleep(0.01)

    # ---- 消费接口 ----

    def get_frame(self):
        """非阻塞取最新帧；尚无帧时返回 None（**绝不阻塞主循环**）。"""
        with self._lock:
            return self._frame

    def get_frame_with_ts(self):
        """返回 (frame, timestamp)；无帧返回 (None, 0.0)。"""
        with self._lock:
            return self._frame, self._frame_ts

    def wait_first_frame(self, timeout: float = 3.0, poll: float = 0.05) -> bool:
        """预热：在 timeout 内等待首帧到达；拿到返回 True。"""
        deadline = time.time() + timeout
        while time.time() < deadline:
            if self.get_frame() is not None:
                return True
            time.sleep(poll)
        return False

    def read(self) -> Tuple[bool, Optional[object]]:
        """兼容 ``cv2.VideoCapture.read()`` 的接口（非阻塞）。"""
        frame = self.get_frame()
        return (frame is not None), frame
