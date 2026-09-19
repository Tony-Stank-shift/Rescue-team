#!/usr/bin/env python3
"""
live_view.py —— 只看画面：摄像头 + 检测 + 浏览器实时流（**不开串口、不动电机**）

用途：
  * 赛前/标定时把相机架好，浏览器里看检测框到底套住了什么；
  * 排查"某个颜色的东西老被认成某类目标"这类问题 —— 看画面 10 秒胜过看日志 10 分钟；
  * 确认 HSV 阈值 / 相机倾角 / 高度是否还合适。

与 `run.sh` 的区别：本脚本**完全不碰底盘串口**，不 START、不下发 VEL，
所以可以在机器人静止、甚至没接底盘的时候安全使用。

用法::

    cd ~/rescue && PYTHONPATH=src python3 tools/live_view.py
    # 然后笔记本浏览器打开 http://<RDK_IP>:8080/
    # 另存一张叠加图: PYTHONPATH=src python3 tools/live_view.py --save /tmp/live.jpg

参数（也可用环境变量）：
    --port 8080         HTTP 端口（环境变量 VIDEO_STREAM_PORT）
    --fps 12            推流帧率
    --cam 0             摄像头序号
    --save PATH         采集 N 秒后存一张叠加图并退出
    --seconds 5         配合 --save 的采集时长
"""

from __future__ import annotations

import argparse
import logging
import os
import sys
import time


def main() -> int:
    ap = argparse.ArgumentParser(description="只开摄像头+检测+实时流，不碰底盘")
    ap.add_argument("--port", type=int,
                    default=int(os.environ.get("VIDEO_STREAM_PORT", "8080")))
    ap.add_argument("--fps", type=float,
                    default=float(os.environ.get("VIDEO_STREAM_FPS", "12")))
    ap.add_argument("--cam", type=int, default=0)
    ap.add_argument("--save", default="", help="存一张叠加图后退出")
    ap.add_argument("--seconds", type=float, default=5.0)
    ap.add_argument("--color",
                    default=os.environ.get("TEAM_COLOR", "blue").strip().lower(),
                    choices=("red", "blue"),
                    help="本队安全区颜色（影响禁区/安全区判定）")
    args = ap.parse_args()

    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s [%(levelname)-7s] %(name)s: %(message)s",
        datefmt="%H:%M:%S")

    from rescue_robot.hardware.camera_reader import CameraReader
    from rescue_robot.perception.perception_pipeline import PerceptionPipeline
    from rescue_robot.perception.field_elements import SafeZoneColor
    from rescue_robot.monitoring.video_server import MjpegServer

    my_color = (SafeZoneColor.RED if args.color == "red" else SafeZoneColor.BLUE)

    cam = CameraReader(args.cam)
    if not cam.start():
        logging.error("摄像头打不开（可能被别的进程占着：先 Ctrl+C 掉 run.sh）")
        return 2
    if not cam.wait_first_frame(timeout=5.0):
        logging.error("5 秒内没等到首帧，检查摄像头排线/序号")
        cam.stop()
        return 3
    logging.info("摄像头就绪")

    perception = PerceptionPipeline(use_mock=False, my_safe_zone_color=my_color)

    # 姿态用"车在场地中心"的假值：本脚本只看识别，不定位
    fake_pose = (1500.0, 1500.0, 1.5707963267948966)

    srv = MjpegServer(camera=cam, perception=perception,
                      hud_provider=lambda: {"pose": (fake_pose[0], fake_pose[1], 90.0),
                                            "cmd": (0.0, 0.0),
                                            "robot_state": "LIVE_VIEW",
                                            "target_count": 0},
                      port=args.port, fps=args.fps)
    srv.start()

    def _pump() -> None:
        """按 ~15Hz 把帧喂给感知，让检测框保持新鲜（不启动任何控制逻辑）。"""
        while True:
            f = cam.get_frame()
            if f is not None:
                try:
                    perception.update(frame=f, robot_position=(fake_pose[0], fake_pose[1]),
                                      robot_theta=fake_pose[2])
                except Exception as e:
                    logging.warning(f"感知更新异常（已忽略）: {e}")
            time.sleep(1.0 / 15.0)

    import threading
    threading.Thread(target=_pump, name="live-view-pump", daemon=True).start()

    if args.save:
        time.sleep(max(0.5, args.seconds))
        import cv2
        img = cam.get_frame()
        if img is None:
            logging.error("采集期间没有帧，无法存图")
            srv.stop(); cam.stop()
            return 4
        marked = srv._annotate_frame(img.copy())
        cv2.imwrite(args.save, marked)
        dets = perception.last_detections
        logging.info(f"已存 {args.save}（本帧检测 {len(dets)} 个）")
        for d in dets:
            logging.info(f"   {d.color.name}/{d.shape.name} conf={d.confidence:.2f} "
                         f"bbox={tuple(d.bbox)}")
        srv.stop(); cam.stop()
        return 0

    logging.info("=" * 60)
    logging.info("  只读画面已启动（不碰串口/电机）。Ctrl+C 退出。")
    logging.info(f"  浏览器打开: {srv.url}")
    logging.info("=" * 60)
    try:
        while True:
            time.sleep(1.0)
    except KeyboardInterrupt:
        logging.info("收到中断")
    finally:
        srv.stop()
        cam.stop()
    return 0


if __name__ == "__main__":
    sys.exit(main())
