#!/usr/bin/env python3
"""
vision_test.py —— 在 RDK/电脑上跑真机视觉识别测试

读 USB 摄像头 → CVDetector（HSV 颜色分割 + 形状判定）→ TargetClassifier 分类
→ 实时显示标注 + 打印检测到的目标类型/颜色/形状。

用法（在能访问摄像头 + 装了 opencv-python 的机器上）：
    python3 tools/vision_test.py [cam_index]

  cam_index  摄像头编号：USB 摄像头通常 0 或 1（默认 0）
  键盘：q 退出，空格 暂停
"""

import sys
import os
import time

# 让 rescue_robot 包可导入（仓库根目录 / 或已安装）
_HERE = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.dirname(_HERE)
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)

import cv2  # noqa: E402

from rescue_robot.perception.detection import CVDetector  # noqa: E402
from rescue_robot.perception.classification import TargetClassifier  # noqa: E402
from rescue_robot.perception.target_types import CompetitionPhase, get_target_display_name  # noqa: E402


def main():
    cam_idx = int(sys.argv[1]) if len(sys.argv) > 1 else 0
    phase = CompetitionPhase.PRELIMINARY  # 初赛；决赛改成 FINAL

    print(f"打开摄像头 {cam_idx} ...")
    cap = cv2.VideoCapture(cam_idx)
    if not cap.isOpened():
        print(f"❌ 摄像头 {cam_idx} 打开失败，检查 /dev/video* 或换索引")
        return 1

    detector = CVDetector(phase=phase)
    classifier = TargetClassifier(phase=phase)

    print(f"✅ 摄像头打开成功（{phase.name} 初赛）")
    print("正在识别... 按 q 退出，空格暂停")
    print("-" * 50)

    frame_no = 0
    while True:
        ok, frame = cap.read()
        if not ok:
            time.sleep(0.05)
            continue
        frame_no += 1

        # 检测 + 分类
        detections = detector.detect(frame)
        targets = classifier.classify_batch(detections)

        # 画检测框（所有检测）
        for det in detections:
            x, y, w, h = det.bbox
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 1)

        # 已分类目标标注类型
        for t in targets:
            cx, cy = int(t.pixel_position[0]), int(t.pixel_position[1])
            label = get_target_display_name(t.info)
            cv2.putText(frame, label, (max(0, cx - 20), max(12, cy - 4)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 255), 1)

        # 周期性打印
        if frame_no % 10 == 0:
            names = ", ".join(t.info.description for t in targets) if targets else "无"
            print(f"帧#{frame_no}: {len(detections)} 检测 → {len(targets)} 分类 | {names}")

        cv2.imshow("vision_test", frame)
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
