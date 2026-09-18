#!/usr/bin/env python3
"""
calibrate_sleeve_roi.py —— 自动量出 `Camera.SLEEVE_ROI`（套取框视觉确认区域）

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
为什么必须标定（2026-09-18 现场，代价是整场白跑）
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

现场日志原样：
    transport_pipeline: 到达目标附近: dist=71mm — 已显式停车，开始套取
    sleeve_lift: SERVO,LOWER 套住: 1 个目标              ← 夹爪**确实下压了**
    transport_pipeline: 视觉确认：套取框内未见目标 → 判为套取失败，将抬爪后退重试

夹爪下压后 1 秒内又被抬起，所以从外面看完全是"它根本不去抓"。
根因是 `Camera.SLEEVE_ROI` 仍是**旧夹爪**标的值，而夹爪 V2 换成了
150×100 方形开口、后方还有三块实心板 → ROI 框的位置全变了 →
确认**永远**判"没套住" → 抬爪 → 后退 → 再试 → 整场消耗在重试上。

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
怎么用（两步，1 分钟）
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

**第一步：把物资放进夹爪槽里**（就用比赛用的那个绿色正方体），
           夹爪停在**下压位**（`SERVO,ANGLE,0`），保持不动。

**第二步：跑这个脚本**（不碰电机、不碰串口，只开摄像头）：

    cd ~/rescue && PYTHONPATH=src python3 tools/calibrate_sleeve_roi.py

它会：
  1. 连续采若干帧，找出**位置最稳定**的那个检测（= 槽里的那个物资）；
  2. 用它的检测框（带一点余量）算出归一化 ROI；
  3. 打印可直接粘贴到 `config.py` 的 `SLEEVE_ROI = (...)` 行；
  4. 存一张标注图 `/tmp/sleeve_roi.jpg`，你可以拉回来核对框得对不对。

把打印出来的那行替换掉 `src/rescue_robot/config.py` 里的 `SLEEVE_ROI` 即可。

可选：
    --save PATH   标注图存放路径（默认 /tmp/sleeve_roi.jpg）
    --frames N    采样帧数（默认 30，约 2 秒）
    --margin F    检测框外扩比例（默认 0.35，因为槽比物资大）
"""

from __future__ import annotations

import argparse
import logging
import math
import sys
import time
from collections import Counter
from typing import Dict, List, Tuple


def main() -> int:
    ap = argparse.ArgumentParser(description="自动量 SLEEVE_ROI")
    ap.add_argument("--cam", type=int, default=0)
    ap.add_argument("--frames", type=int, default=30)
    ap.add_argument("--margin", type=float, default=0.35)
    ap.add_argument("--save", default="/tmp/sleeve_roi.jpg")
    args = ap.parse_args()

    logging.basicConfig(level=logging.WARNING,
                        format="%(asctime)s [%(levelname)-7s] %(name)s: %(message)s",
                        datefmt="%H:%M:%S")

    from rescue_robot.hardware.camera_reader import CameraReader
    from rescue_robot.perception.detection import CVDetector

    cam = CameraReader(args.cam)
    if not cam.start():
        print("❌ 摄像头打不开（先确认 run.sh / live_view 没在跑）")
        return 2
    if not cam.wait_first_frame(timeout=5.0):
        print("❌ 5 秒内没有首帧，检查排线/序号")
        cam.stop()
        return 3

    det = CVDetector()
    print("=" * 74)
    print("  自动标定 Camera.SLEEVE_ROI")
    print("=" * 74)
    print(f"  采集中（{args.frames} 帧 ≈ {args.frames / 15:.0f}s）…"
          f"请保持车与夹爪不动")

    # 采样：按"颜色+形状+大致位置"聚簇，取出现次数最多的一簇
    samples: List[Tuple[Tuple[str, str], Tuple[int, int, int, int], float]] = []
    img_size = None
    last_frame = None
    for _ in range(args.frames):
        f = cam.get_frame()
        if f is None:
            time.sleep(1.0 / 15.0)
            continue
        last_frame = f
        try:
            img_size = (f.shape[1], f.shape[0])
            for d in det.detect(f):
                key = (getattr(d.color, "name", "?"), getattr(d.shape, "name", "?"))
                samples.append((key, tuple(int(v) for v in d.bbox),
                                 float(d.confidence)))
        except Exception as e:
            print(f"   检测异常（已忽略）: {e}")
        time.sleep(1.0 / 15.0)

    cam.stop()
    if not samples:
        print("\n❌ 一帧都没检出任何东西 —— 槽里的物资没有被识别到。")
        print("   ① 夹爪先下压到 0°（`SERVO,ANGLE,0`）把物资套住；")
        print("   ② 用 tools/live_view.py 看画面，确认物资在视野里且颜色阈值对；")
        print("   ③ 若夹爪后方实心板把槽挡住，说明**看不到槽** →")
        print("      那就无法用视觉确认套取，改用 `SLEEVE_CONFIRM=0` 关闭确认运行。")
        return 4

    # 按"位置稳定性"选主簇：同一类别里，中心点最集中的一堆
    by_key: Dict[Tuple[str, str], List[Tuple[int, int, int, int]]] = {}
    for key, bbox, _c in samples:
        by_key.setdefault(key, []).append(bbox)

    best_key, best_bboxes, best_spread = None, None, None
    for key, bxs in by_key.items():
        if len(bxs) < max(3, args.frames // 6):
            continue
        cx = [b[0] + b[2] / 2.0 for b in bxs]
        cy = [b[1] + b[3] / 2.0 for b in bxs]
        spread = math.hypot(max(cx) - min(cx), max(cy) - min(cy))
        if best_spread is None or spread < best_spread:
            best_key, best_bboxes, best_spread = key, bxs, spread

    if best_key is None:
        cand = ", ".join(f"{k[0]}/{k[1]}×{len(v)}" for k, v in
                         sorted(by_key.items(), key=lambda kv: -len(kv[1]))[:5])
        print(f"\n❌ 没有稳定的检测（各类别出现次数：{cand}）")
        print("   → 物资可能没进视野 / 光在变 / 夹爪在动。请固定好再试，")
        print("     或先跑 tools/live_view.py 看画面。")
        return 5

    ws = [b[2] for b in best_bboxes]
    hs = [b[3] for b in best_bboxes]
    xs = [b[0] for b in best_bboxes]
    ys = [b[1] for b in best_bboxes]
    mx = sum(ws) / len(ws) * args.margin        # 外扩量（按平均尺寸的比例）
    my = sum(hs) / len(hs) * args.margin
    x1 = max(0.0, min(xs) - mx)
    y1 = max(0.0, min(ys) - my)
    x2 = min(float(img_size[0]), max(x + w for x, _y, w, _h in best_bboxes) + mx)
    y2 = min(float(img_size[1]), max(y + h for _x, y, _w, h in best_bboxes) + my)

    W, H = float(img_size[0]), float(img_size[1])
    roi = (round(x1 / W, 3), round(y1 / H, 3), round(x2 / W, 3), round(y2 / H, 3))

    print(f"\n  最稳定的检测：{best_key[0]}/{best_key[1]}"
          f"（{len(best_bboxes)} 帧，中心漂移仅 {best_spread:.0f}px）")
    print(f"  框（像素）: ({x1:.0f}, {y1:.0f}) – ({x2:.0f}, {y2:.0f})"
          f"   图像 {img_size[0]}×{img_size[1]}")
    print()
    print("  ──────────── 复制这一行替换 src/rescue_robot/config.py 里的 SLEEVE_ROI ────────────")
    print(f"    SLEEVE_ROI: tuple = {roi}")
    print("  ───────────────────────────────────────────────────────────────────────────────")
    print()
    print("  核对方法：改完重跑 run.sh，浏览器看实时画面 —— 黄框标注的")
    print("  `SLEEVE_ROI ON in=N -> OCCUPIED/EMPTY` 里 in 应 ≥1、状态应为 OCCUPIED。")

    # 存标注图供核对
    try:
        import cv2
        img = last_frame.copy()
        cv2.rectangle(img, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 255), 2)
        for b in best_bboxes[-1:]:
            bx, by, bw, bh = b
            cv2.rectangle(img, (bx, by), (bx + bw, by + bh), (0, 255, 0), 2)
        cv2.putText(img, f"ROI {roi}", (int(x1), max(16, int(y1) - 8)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1, cv2.LINE_AA)
        cv2.imwrite(args.save, img)
        print(f"  已存标注图: {args.save}（拉回来核对黄框有没有套住槽里的物资）")
    except Exception as e:
        print(f"  （存图失败: {e}）")
    return 0


if __name__ == "__main__":
    sys.exit(main())
