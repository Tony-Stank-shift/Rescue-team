#!/usr/bin/env python3
"""verify_detector_ground_gates —— "这轮廓可能是贴地物资吗"的几何门回归

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
现场事故（2026-09-18）：**同学的腿/脚被识别成"黑色正三棱锥"，置信度 0.75**
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

现场抓到的真实检测（`/status` 原样）：
    BLACK TRIANGULAR_PYRAMID 0.75  bbox=[0,   0, 100, 378]   ← 同学的腿
    BLACK UNKNOWN            0.34  bbox=[407,289,  77, 191]
    BLACK CUBOID             0.75  bbox=[0, 426,  81,  54]   ← 贴左边缘
    BLACK CUBE               0.30  bbox=[357,195,  21,  29]

它们经"同色兜底"变成核心物资进世界地图 → 幻影目标 → 车跑去扑空。

本自检两边都要管：
  A. 上述"物理上不可能是贴地物资"的轮廓必须被拒（用真实坐标复现）；
  B. **真物资绝不能被误伤** —— 用**正向相机几何**把 40mm 的物资投影到
     300/500/800/1100/150mm 各距离，生成的 bbox 必须全部通过。
     （B 是这份自检的重点：只测"能拦住坏东西"而不测"不误伤好东西"的护栏，
      会把检测率一起干掉，比不加还糟。）
"""

from __future__ import annotations

import math
import sys
import traceback
from typing import Optional, Tuple

PASS, FAIL = "PASS", "FAIL"


def _det():
    from rescue_robot.perception.detection import CVDetector
    return CVDetector()


# ────────────────────── 正向几何：把真物资投影成 bbox ──────────────────────

def project_cube_bbox(size_mm: float, dist_mm: float, det) -> Tuple[int, int, int, int]:
    """把"边长 size_mm、贴地站在 dist_mm 远处"的物体投影成图像 bbox。

    相机下倾 tilt、高 h、焦距 f：
        地面接触点俯角  dep_b = atan(h / d)      → v_bottom = cy + f·tan(dep_b - tilt)
        物体顶端俯角    dep_t = atan((h-s) / d)  → v_top    = cy + f·tan(dep_t - tilt)
        水平宽度        w = s · f / d
    """
    f, cy, cx = det._f_px, det._cy_px, det._cx_px
    tilt = math.radians(det._camera_tilt_deg)
    h = det._camera_height_mm
    dep_b = math.atan2(h, dist_mm)
    dep_t = math.atan2(max(1.0, h - size_mm), dist_mm)
    v_b = cy + f * math.tan(dep_b - tilt)
    v_t = cy + f * math.tan(dep_t - tilt)
    w = size_mm * f / dist_mm
    x = int(round(cx - w / 2.0))
    y = int(round(v_t))
    return (x, y, max(2, int(round(w))), max(2, int(round(v_b - v_t))))


# ────────────────────────────── 场景 ──────────────────────────────

def s_reject_field_false_positives(ev):
    """A. 现场那三个坏检测必须被拒。"""
    det = _det()
    bad = [
        ("同学的腿 100×378（顶边越过地平线）", (0, 0, 100, 378)),
        ("贴左边缘 81×54（宽度被裁）", (0, 426, 81, 54)),
        ("贴右边缘（宽度被裁）", (631, 200, 9, 40)),
    ]
    for name, bbox in bad:
        if det._plausible_ground_object(*bbox):
            return (f"'{name}' 仍然通过了几何门 → 现场仍会把腿/边缘色块当物资 "
                    f"（bbox={bbox}）")
    ev.append("拒绝现场假阳性：" + "、".join(n for n, _ in bad))

    # 长宽比：378/100 已由地平线拦下，这里单独造一个"不越地平线但极细长"的
    slim = (200, 100, 20, 200)
    if det._plausible_ground_object(*slim):
        return f"极细长轮廓 {slim}（长宽比 10）未被拒"
    ev.append("拒绝极细长轮廓 (20×200, 长宽比 10)")
    return None


def s_unknown_shape_no_fallback(ev):
    """A2. 形状 UNKNOWN 的色块不许被"同色兜底"救成物资。

    现场 `BLACK/UNKNOWN bbox=[407,289,77,191]` 就是这样一路变成核心物资的：
    它的形状根本没判出来，却被"该颜色在配置表里唯一"这条兜底救了回来。
    """
    from rescue_robot.perception.classification import TargetClassifier
    from rescue_robot.perception.target_types import (
        TargetColor, TargetShape, Detection, CompetitionPhase)
    clf = TargetClassifier(phase=CompetitionPhase.PRELIMINARY)

    # UNKNOWN 形状 + 唯一颜色（黑）→ 必须不认
    d = Detection(color=TargetColor.BLACK, shape=TargetShape.UNKNOWN,
                  bbox=(407, 289, 77, 191), confidence=0.34,
                  contour_area=8000, contour_vertices=5)
    if clf.classify(d) is not None:
        return "形状 UNKNOWN 的黑色色块仍被兜底成核心物资 → 幻影目标（现场原样）"

    # 形状认得出来、只是颜色略有偏差 → 兜底仍应生效（这是兜底的正当用途）
    d2 = Detection(color=TargetColor.BLACK, shape=TargetShape.CUBE,
                   bbox=(360, 200, 30, 30), confidence=0.6,
                   contour_area=900, contour_vertices=4)
    if clf.classify(d2) is None:
        return ("形状可辨的黑色方块被一起拒了 → 兜底被误伤，"
                "会丢掉真实的黑色物资（宁可漏也不许误杀真物资）")
    ev.append("UNKNOWN 形状不再兜底；形状可辨时兜底照常生效")
    return None


def s_no_over_rejection(ev):
    """B. ★ 真物资（40mm）在 150~1100mm 各距离都必须通过几何门。"""
    det = _det()
    ok_rows = []
    for dist in (150.0, 300.0, 500.0, 800.0, 1100.0):
        bbox = project_cube_bbox(40.0, dist, det)
        if not det._plausible_ground_object(*bbox):
            return (f"{dist:.0f}mm 处的 40mm 物资被几何门误杀了 "
                    f"（投影 bbox={bbox}）→ 检测距离被砍，得不偿失")
        x, y, w, h = bbox
        asp = max(w, h) / min(w, h)
        ok_rows.append(f"{dist:.0f}mm→{w}×{h}px(v_top={y})")
    ev.append("真物资不误伤（40mm 正方体，正向几何投影）: " + "、".join(ok_rows))
    return None


def s_horizon_math(ev):
    """C. 地平线公式自检 + "贴地物资永远在地平线以下"的物理论证。"""
    det = _det()
    expect = det._cy_px - det._f_px * math.tan(math.radians(det._camera_tilt_deg))
    if abs(det._horizon_v - expect) > 0.51:
        return f"地平线行号算错: {det._horizon_v} != {expect}"
    if not (0 <= det._horizon_v <= 20):
        return (f"地平线行号 {det._horizon_v:.1f} 不在预期范围（本项目 ≈8）→ "
                f"相机倾角/焦距参数可能不对")
    # 相机高 210mm ≫ 物资高 120mm ⇒ 任何贴地物资顶端都低于相机 ⇒ 成像在地平线以下
    if det.TARGET_MAX_SIZE_MM >= det._camera_height_mm:
        return "TARGET_MAX_SIZE_MM 不小于相机高度 → '物资永远在地平线以下'的论证不成立"
    ev.append(f"地平线 v={det._horizon_v:.1f}（f={det._f_px:.0f}px, "
              f"tilt={det._camera_tilt_deg:.0f}°, 相机高 {det._camera_height_mm:.0f}mm "
              f"≫ 物资高 {det.TARGET_MAX_SIZE_MM:.0f}mm）")
    return None


SCENARIOS = (
    ("A 拒绝现场假阳性（腿/贴边/细长）", s_reject_field_false_positives),
    ("A2 UNKNOWN 形状不再被兜底成物资", s_unknown_shape_no_fallback),
    ("B ★ 真物资（40mm，150~1100mm）不许被误杀", s_no_over_rejection),
    ("C 地平线公式与'物资永远在地平线以下'的论证", s_horizon_math),
)


def main() -> int:
    print("=" * 78)
    print("  verify_detector_ground_gates —— '这轮廓可能是贴地物资吗'的几何门")
    print("=" * 78)
    ev, fails = [], []
    for name, fn in SCENARIOS:
        try:
            problem = fn(ev)
        except Exception:
            problem = "护栏自身抛异常:\n" + traceback.format_exc()
        print(f"  [{'PASS' if problem is None else 'FAIL'}] {name}")
        if problem is not None:
            fails.append(name)
            for line in problem.splitlines():
                print(f"         {line}")
    print("-" * 78)
    for line in ev:
        print(f"  · {line}")
    print("-" * 78)
    if fails:
        print(f"  结果: {len(SCENARIOS) - len(fails)}/{len(SCENARIOS)} 通过")
        print("  ❌ 检测器仍会把非物资当成物资（或把真物资误杀）")
        return 1
    print(f"  结果: {len(SCENARIOS)}/{len(SCENARIOS)} 通过")
    print("  ✅ 几何上台不可能的轮廓被拒，真物资在 150~1100mm 全部保留")
    return 0


if __name__ == "__main__":
    sys.exit(main())
