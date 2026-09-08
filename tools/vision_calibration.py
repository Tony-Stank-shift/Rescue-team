#!/usr/bin/env python3
"""
vision_calibration.py —— 摄像头视觉标定/测试工具（Windows 可直接跑，只依赖 OpenCV）

功能：
  1. 打开指定摄像头，实时显示原图。
  2. 对四种目标颜色（绿/黑/橘/浅蓝）做 HSV 分割，分开显示。
  3. 在 "RAW" 窗口点鼠标 → 显示该点的 HSV/BGR，方便看暗面/反光的实际取值。

用法（Windows）：
  pip install opencv-python      # 只需这个
  python vision_calibration.py 1    # 1 = 外接 USB 摄像头（0 = 内置）

按键：
  q     退出
  s     保存当前帧 + 各通道分割图到 ./vision_frames/
  点鼠标  在 RAW 窗口点击任意点，打印该点 HSV/BGR
"""

import sys
import os
import cv2
import numpy as np

# 四种救援目标颜色（初赛）的 HSV 分割范围（H:0-180, S:0-255, V:0-255）
# ⚠️ 默认经验值，已按 77° 摄像头实测微调（橘色覆盖亮暗面、浅蓝放宽 S 上限）
HSV_RANGES = {
    "green":     ((40, 80, 60),  (80, 255, 255)),   # 普通物资（绿色）
    "orange":    ((4, 205, 35), (24, 255, 255)),    # 伤员（橘色，H 4~24 认橙，S≥205 排红色，红 S≤201）
    "black":     ((0, 0, 0),     (179, 255, 60)),   # 核心物资（黑色，低 V）
    "light_blue":((85, 50, 110), (108, 255, 255)),  # 危险目标（浅蓝，S 上限放宽到 255）
}


def color_mask(hsv, name):
    low, high = HSV_RANGES[name]
    lower = np.array(low, dtype=np.uint8)
    upper = np.array(high, dtype=np.uint8)
    mask = cv2.inRange(hsv, lower, upper)
    # 形态学去噪
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    return mask


def main(cam_index=1):
    cap = cv2.VideoCapture(cam_index)
    if not cap.isOpened():
        print(f"❌ 无法打开摄像头 {cam_index}（试试 0 或 2）")
        return 1
    print(f"✅ 摄像头 {cam_index} 打开成功")
    print("   按键: q=退出, s=保存当前帧")
    print("   操作: 在 RAW 窗口点击任意点 → 显示该点 HSV/BGR（看暗面/反光实际值）")
    print("   提示: 按 HSV_RANGES 调四种颜色阈值，对着实物看分割是否干净")

    os.makedirs("vision_frames", exist_ok=True)
    frame_no = 0

    # 鼠标点击 → 显示该点 HSV/BGR
    state = {'hsv': None, 'frame': None, 'clicked': None}

    def on_mouse(event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN and state['hsv'] is not None:
            h, s, v = state['hsv'][y, x]
            b, g, r = state['frame'][y, x]
            state['clicked'] = (x, y, int(h), int(s), int(v))
            print(f"🎯 点击 ({x},{y}) → HSV({int(h)},{int(s)},{int(v)})"
                  f"  BGR({int(b)},{int(g)},{int(r)})")

    cv2.namedWindow("RAW")
    cv2.setMouseCallback("RAW", on_mouse)

    while True:
        ret, frame = cap.read()
        if not ret:
            print("⚠️ 读帧失败")
            break
        frame = cv2.resize(frame, (640, 480))

        # 转 HSV
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        state['hsv'] = hsv
        state['frame'] = frame

        # 四路分割
        masks = {name: color_mask(hsv, name) for name in HSV_RANGES}

        cv2.putText(frame, "RAW (q=quit, s=save)", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        # 覆盖显示已点击点的 HSV
        if state['clicked']:
            cx, cy, h, s, v = state['clicked']
            cv2.circle(frame, (cx, cy), 5, (0, 255, 255), -1)
            cv2.putText(frame, f"HSV({h},{s},{v})", (cx + 10, cy),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
        cv2.imshow("RAW", frame)

        # 单独显示四路彩色分割
        for name, mask in masks.items():
            colored = cv2.bitwise_and(frame, frame, mask=mask)
            cv2.putText(colored, name, (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
            cv2.imshow(f"MASK_{name}", colored)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        if key == ord('s'):
            cv2.imwrite(f"vision_frames/frame_{frame_no}.png", frame)
            for name, mask in masks.items():
                colored = cv2.bitwise_and(frame, frame, mask=mask)
                cv2.imwrite(f"vision_frames/{name}_{frame_no}.png", colored)
            print(f"📁 已保存第 {frame_no} 帧到 ./vision_frames/")
            frame_no += 1

    cap.release()
    cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    idx = int(sys.argv[1]) if len(sys.argv) > 1 else 1
    sys.exit(main(idx))
