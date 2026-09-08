#!/usr/bin/env python3
"""
vision_quick.py —— 自包含视觉快速测试（RDK/电脑直接跑，无需 rescue_robot 包）

读 USB 摄像头 → 对四种目标色做 HSV 颜色分割 → 找轮廓 → 形状判定 →
按 (颜色, 形状) 判四类目标类型，实时显示 + 打印。

依赖：opencv-python, numpy（RDK 一般自带 / pip install opencv-python）

用法：python3 vision_quick.py [cam_index]
  cam_index：USB 摄像头索引（默认 0）。键盘 q 退出，空格暂停。
"""

import sys
import time

import cv2
import numpy as np

# 四种目标色的 HSV 范围（OpenCV: H 0-179, S/V 0-255）
# 值是对初赛目标的经验估计，光照不同需现场调
HSV_COLORS = {
    'green':      ((40, 80, 60),  (80, 255, 255)),   # 普通物资（绿正方体）
    'black':      ((0, 0, 0),     (179, 255, 60)),   # 核心物资（黑三棱锥）
    'orange':     ((8, 100, 100), (20, 255, 255)),   # 伤员（橘长方体）
    'light_blue': ((85, 40, 100), (105, 200, 255)),  # 危险目标（浅蓝正方体）
}

# 形状判定（按轮廓近似顶点数 + 面积比）
SHAPES = {
    'cube':    (4, 8, 0.5, 1.0),    # 正方体
    'pyramid': (3, 5, 0.3, 0.7),    # 三棱锥
    'cuboid':  (4, 8, 0.5, 1.0),    # 长方体
    'sphere':  (8, 30, 0.6, 0.9),   # 球体
    'cylinder':(8, 20, 0.55, 0.95), # 圆柱
}

MIN_AREA = 200  # 最小轮廓面积（过滤噪点，像素²）


def classify_shape(vertices: int, area_ratio: float) -> str:
    """按顶点数+面积比猜形状。"""
    for name, (vmin, vmax, rmin, rmax) in SHAPES.items():
        if vmin <= vertices <= vmax and rmin <= area_ratio <= rmax:
            return name
    # 兜底：只按顶点数
    if 3 <= vertices <= 5:
        return 'pyramid'
    elif 4 <= vertices <= 8:
        return 'cube/cuboid'
    elif vertices > 8:
        return 'sphere/cylinder'
    return 'unknown'


def classify_target(color: str, shape: str) -> str:
    """颜色 + 形状 → 目标类型。"""
    # 初赛：颜色即大致定类型
    if color == 'green':
        return '普通物资(绿)'
    if color == 'black':
        return '核心物资(黑)'
    if color == 'orange':
        return '伤员(橘)'
    if color == 'light_blue':
        return '⚠️危险目标(浅蓝)'
    return 'unknown'


def main():
    cam_idx = int(sys.argv[1]) if len(sys.argv) > 1 else 0
    print(f"打开摄像头 {cam_idx} ...")
    cap = cv2.VideoCapture(cam_idx)
    if not cap.isOpened():
        print(f"❌ 摄像头 {cam_idx} 打开失败，检查 /dev/video* 或换索引")
        return 1
    print("✅ 摄像头打开成功。识别中... 按 q 退出，空格暂停")
    print("-" * 55)

    frame_no = 0
    while True:
        ok, frame = cap.read()
        if not ok:
            time.sleep(0.05)
            continue
        frame_no += 1
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        found = []
        for color, (lo, hi) in HSV_COLORS.items():
            mask = cv2.inRange(hsv, np.array(lo), np.array(hi))
            kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                           cv2.CHAIN_APPROX_SIMPLE)
            for c in contours:
                area = cv2.contourArea(c)
                if area < MIN_AREA:
                    continue
                x, y, w, h = cv2.boundingRect(c)
                peri = cv2.arcLength(c, True)
                approx = cv2.approxPolyDP(c, 0.04 * peri, True)
                vertices = len(approx)
                area_ratio = area / (w * h) if w * h > 0 else 0
                shape = classify_shape(vertices, area_ratio)
                target = classify_target(color, shape)
                found.append((color, shape, target, area, x, y, w, h))
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 1)
                label = f"{color}/{shape}"
                cv2.putText(frame, label, (x, max(12, y - 4)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 255), 1)

        if frame_no % 10 == 0:
            if found:
                # 按面积排序取大的前几个
                found.sort(key=lambda t: t[3], reverse=True)
                lines = "; ".join(f"{t[2]}({t[1]})" for t in found[:5])
                print(f"帧#{frame_no}: {len(found)} 目标 → {lines}")
            else:
                print(f"帧#{frame_no}: 未识别到目标")

        cv2.imshow("vision_quick", frame)
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord(' '):
            while cv2.waitKey(1) & 0xFF != ord(' '):
                pass

    cap.release()
    cv2.destroyAllWindows()
    print("\n测试结束")
    return 0


if __name__ == "__main__":
    sys.exit(main())
