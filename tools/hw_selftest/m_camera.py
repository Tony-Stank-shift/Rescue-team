"""m_camera —— 摄像头：能否打开 / 分辨率 / 出帧率 / 能否取到帧

用后台采集线程（CameraReader）探测，**不会阻塞**——这是修过的历史 bug：
直接在 50Hz 主循环里 read()，摄像头没就绪时会卡死整个主循环。
"""

import os
import time

from .framework import register, ok, bad, skip

MODULE = "camera"
TITLE = "摄像头（打开 / 分辨率 / 出帧率）"


@register(MODULE, TITLE)
def run(ctx):
    ev = []
    try:
        from rescue_robot.hardware.camera_reader import CameraReader
    except Exception as e:
        return bad(MODULE, f"无法导入 CameraReader：{e!r}", ev)

    idx = int(os.environ.get("CAM_INDEX", "0"))
    cam = CameraReader(idx, name="selftest-cam")
    if not cam.start():
        mk = skip if ctx.mock else bad
        return mk(MODULE, f"摄像头 index={idx} 打不开（/dev/video{idx}）", ev,
                  "确认 USB 摄像头已识别（ls /dev/video*）；用 CAM_INDEX 指定正确序号；"
                  "开发机无摄像头请加 --mock")
    ev.append(f"采集线程已启动（index={idx}）")

    if not cam.wait_first_frame(timeout=5.0):
        cam.stop()
        mk = skip if ctx.mock else bad
        return mk(MODULE, "摄像头打开了但 5 秒内出不了帧（黑屏/带宽不足/被占用）", ev,
                  "换 USB 口或降低分辨率；确认没有其它程序占用摄像头")

    t0 = time.time()
    n0 = cam.frame_count
    time.sleep(2.0)
    fps = (cam.frame_count - n0) / max(0.1, time.time() - t0)
    frame = cam.get_frame()
    cam.stop()

    if frame is None:
        return bad(MODULE, "取帧返回 None（线程拿到过帧却又取不到）", ev)
    h, w = frame.shape[:2]
    from rescue_robot.config import Thresholds
    min_fps = float(getattr(Thresholds, "CAMERA_MIN_FPS", 10))
    ev.append(f"分辨率 {w}x{h}，实测出帧率 {fps:.1f} FPS（要求 ≥{min_fps:.0f}）")

    if fps < min_fps:
        return bad(MODULE, f"出帧率过低（{fps:.1f} < {min_fps:.0f} FPS）→ 感知会滞后", ev,
                   "换 USB 3.0 口、降低分辨率、关闭其它占用摄像头的程序")
    return ok(MODULE, f"摄像头正常（{w}x{h} @ {fps:.1f}FPS）", ev)
