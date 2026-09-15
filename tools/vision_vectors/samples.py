"""samples.py —— 色卡定义（HSV 量纲）+ 色卡 ↔ HSV_RANGES 的判定工具

被 `generate_vectors.py`（画图）与 `run_vectors.py`（比对/打印标定表）共用，
保证"画出来的颜色"和"文档里打印的颜色"是同一份定义。

色卡全部按 **OpenCV HSV 量纲**（H∈[0,179], S/V∈[0,255]）给出 —— 因为
`perception/detection.py::HSV_RANGES` 就是这个量纲，现场用
`tools/vision_calibration.py` 取样后可以直接和本表对齐。
"""

import numpy as np
import cv2

from rescue_robot.perception.detection import HSV_RANGES, HSV_RANGES_RED2
from rescue_robot.perception.target_types import TargetColor

#: (标签, H, S, V)；H = -1 表示无 hue（灰色）
SAMPLES = {
    "green_h60":        ("绿(普通物资)",                    60, 240, 200),
    "green_dark_h60":   ("暗绿(同一物体阴影下 V=70)",        60, 240,  70),
    "green_dark55_h60": ("更暗的绿(V=55，跌破 BLACK 上界 60)", 60, 240,  55),
    "black_v20":        ("黑(核心物资)",                     0,   0,  20),
    "black_v55":        ("灰黑(V=55，卡在黑色上界内)",        0,   0,  55),
    "orange_h15":       ("橘(伤员)",                        15, 255, 255),
    "orange_s205":      ("橘(正好压 ORANGE 的 S 下界 205)",  15, 205, 235),
    "orange_light_h15": ("浅橘(强光下 S=180，掉出阈值)",      15, 180, 235),
    "brown_h18":        ("褐(木纹/桌面色)",                 18, 180, 120),
    "lightblue_h96":    ("浅蓝(初赛危险目标)",               96,  63, 230),
    "blue_h112":        ("饱和蓝(深蓝)",                   112, 229, 200),
    "blue_pure_h120":   ("纯蓝(最经典的蓝 BGR 255,0,0)",     120, 255, 255),
    "blue_h111":        ("蓝 H111(卡在 BLUE 下界之外)",      111, 255, 255),
    "bluepurple_h125":  ("蓝紫(H 上界附近)",                125, 255, 255),
    "cyan_h90":         ("青(蓝/浅蓝重叠区)",                90, 255, 255),
    "palepaleblue_h98": ("极淡蓝(近白，S=10)",               98,  10, 250),
    "white_s0":         ("白(灯光/白纸)",                    0,   0, 255),
    "gray_s0":          ("亮灰(底色对照)",                  -1,   0, 128),
}


def hsv_of_sample(key):
    """样本的 HSV（灰色标 -1 表示无 hue）"""
    _label, h, s, v = SAMPLES[key]
    return h, s, v


def bgr_of_sample(key):
    """样本的 BGR 像素值（cv2 HSV→BGR）"""
    h, s, v = hsv_of_sample(key)
    b, g, r = cv2.cvtColor(np.uint8([[[max(0, h), s, v]]]), cv2.COLOR_HSV2BGR)[0, 0]
    return (int(b), int(g), int(r))


def hsv_roundtrip(key):
    """BGR→HSV 往返实测值（会与标称值差 ±1，这正是"实测"的意义）"""
    col = bgr_of_sample(key)
    hsv = cv2.cvtColor(np.uint8([[list(col)]]), cv2.COLOR_BGR2HSV)[0, 0]
    return int(hsv[0]), int(hsv[1]), int(hsv[2])


def ranges_for(color: TargetColor):
    """某目标色的 HSV 区间列表（红色两个区间）——与检测器 _create_color_mask 同源"""
    rng = HSV_RANGES.get(color)
    if rng is None:
        return []
    return [rng, HSV_RANGES_RED2] if color == TargetColor.RED else [rng]


def in_range(h, s, v, rng) -> bool:
    (h0, s0, v0), (h1, s1, v1) = rng
    if h < 0:                      # 无 hue（灰/白）：只比 S/V
        return s0 <= s <= s1 and v0 <= v <= v1
    return h0 <= h <= h1 and s0 <= s <= s1 and v0 <= v <= v1


def matched_colors(h, s, v):
    """该像素值命中了哪些 HSV 桶（按检测顺序：LIGHT_BLUE 优先于 BLUE）"""
    ordered = [TargetColor.LIGHT_BLUE, TargetColor.BLUE]
    ordered += [c for c in HSV_RANGES if c not in ordered]
    return [c for c in ordered if any(in_range(h, s, v, r) for r in ranges_for(c))]
