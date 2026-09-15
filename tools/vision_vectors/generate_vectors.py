"""generate_vectors.py —— 确定性视觉识别测试向量集（合成图生成 + 期望标签）

**目的**：把"识别到底准不准"从口头结论变成**可复现证据**，并给现场 HSV 标定一把现成标尺。

为什么要这个东西
----------------
1. B5/B6 两个 blocker（蓝色被判成危险目标 / 伤员橘色长方体被判成正方体）修完之后，
   "改没改对"必须有**固定输入 → 固定期望**的回归标尺，否则任何人再动 HSV 阈值
   都可能悄悄改回去；
2. 现场真正的风险是**灯光/相机不同导致 HSV 漂移**。浅蓝(危险)与蓝在 H/S 上天然相邻，
   只能靠两道闸门区分（① 检测顺序 LIGHT_BLUE 先于 BLUE ② 分类器禁止容差命中 DANGEROUS）。
   本向量集把这两道闸门的"边界点"画出来，标定完直接跑本脚本即可判定是否还安全。

特点
----
* **确定性**：全部图像用几何绘制 + `cv2.rectangle`，不拍照、不联网、不用随机数；
  同一份代码任何机器上生成逐像素一致（PNG 内不含时间戳）。
* **不依赖摄像头**：纯离线合成图，可在 WSL/CI 跑。
* **期望值不是手写的**：期望 (颜色, 形状) → 目标类型 由 `get_target_config(phase)`
  查表得出，与生产代码同源；"旧实现会错"的样本显式标注 `old_verdict != new_verdict`。

输出
----
* `img/vec_*.png`    每个样本一张图（640×480，亮灰底 BGR(128,128,128)）
* `vectors.json`     向量清单：每张图的色块 bbox + 期望颜色/形状/类型 + 旧实现预期结果

用法::

    python3 tools/vision_vectors/generate_vectors.py     # 生成图 + 清单
    python3 tools/vision_vectors/run_vectors.py          # 逐样本比对并打印对照表
"""

import json
import logging
import os
import sys

# 让脚本可以"直接跑"（不装包、不设 PYTHONPATH）
_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
_SRC = os.path.join(_ROOT, "src")
if _SRC not in sys.path:
    sys.path.insert(0, _SRC)

import cv2          # noqa: E402
import numpy as np  # noqa: E402

logging.disable(logging.CRITICAL)   # 本脚本只要结果，不要检测器日志

from rescue_robot.perception.target_types import (   # noqa: E402
    CompetitionPhase, TargetColor, TargetShape, TargetType, get_target_config,
)

HERE = os.path.dirname(os.path.abspath(__file__))
IMG_DIR = os.path.join(HERE, "img")
MANIFEST = os.path.join(HERE, "vectors.json")

FRAME_W, FRAME_H = 640, 480
#: 底色取亮灰 BGR(128,128,128)。三条约束决定这个值：
#:   ① 不属于任何目标色（灰 → HSV 的 S=0，GREEN/ORANGE/BLUE 的 S 下界都 ≥20 以上）；
#:   ② 必须躲开 BLACK 阈值（V ≤ 60）——旧自检曾用 V=60 的底色把整幅画面吃成"黑色目标"；
#:   ③ 必须躲开 WHITE 阈值（S ≤ 30 且 V ≥ 180 且 H 落在两端）——128 达不到 V 下界。
BG_BGR = (128, 128, 128)


# ============================================================
# 色卡：统一放在 samples.py（与 run_vectors.py 共用同一份定义）
# ============================================================

from samples import SAMPLES, hsv_of_sample, bgr_of_sample    # noqa: E402,F401


def hsv_to_bgr(h, s, v):
    """HSV(OpenCV 量纲) → BGR；与 samples.bgr_of_sample 同一实现，保留供并排样本使用"""
    px = np.uint8([[[max(0, h), s, v]]])
    b, g, r = cv2.cvtColor(px, cv2.COLOR_HSV2BGR)[0, 0]
    return (int(b), int(g), int(r))


# ============================================================
# 像素 → 实物尺寸的比例尺
# ============================================================
# 只在**同一张图内部**用比例尺比较形状，不参与任何绝对测距：
# 赛项目标 40mm 立方体 → 40px，80mm 长边 → 80px（即 1px = 1mm）。
# 现场真实距离下 40mm 目标成像远小于 40px，但"长宽比"是无量纲量，
# 缩放不影响判据有效性（这也是选长宽比当主判据的原因之一）。
MM_PER_PX = 1.0


def px(mm: float) -> int:
    """毫米 → 像素（四舍五入到整数，保证可复现）"""
    return int(round(mm / MM_PER_PX))


# ============================================================
# 向量定义
# ============================================================
# 每个 case：
#   name      文件名/记录名
#   sample    SAMPLES 的 key
#   w_mm,h_mm 色块实物尺寸（分帧）
#   kind      形状类别（记录用，判据请以脚本输出的长宽比/面积比为准）
#   expect    期望 (颜色, 形状)；形状给 None = 形状判定必须为指定值，给 "*" = 不判定形状
#   old       旧实现（B5/B6 未修时）预期结果：None / "DANGEROUS" / "INJURED" / "REGULAR_SUPPLY"
#   note      说明
CASES = [
    # ── 基准样本（4 类目标，赛项尺寸与配色）──
    dict(name="vec_01_normal_green_cube", sample="green_h60", w_mm=40, h_mm=40,
         kind="cube", expect=("GREEN", "CUBE"), old="REGULAR_SUPPLY", verdict="ok",
         note="普通物资基准（绿正方体 40mm）"),
    dict(name="vec_02_normal_black_pyramid_triangle", sample="black_v20", w_mm=40, h_mm=40,
         draw="triangle", kind="pyramid(三角形投影)", expect=("BLACK", "TRIANGULAR_PYRAMID"),
         old="CORE_SUPPLY", verdict="ok",
         note="核心物资基准（黑三棱锥 40mm）。★判据要点：**三角形轮廓（顶点3、面积比≈0.53）"
              "才是三棱锥的可靠判据**；画成正方形投影时会被判 CUBE（见 vec_19）"),
    dict(name="vec_03_normal_lightblue_dangerous_cube", sample="lightblue_h96", w_mm=40, h_mm=40,
         kind="cube", expect=("LIGHT_BLUE", "CUBE"), old="DANGEROUS", verdict="ok",
         note="危险目标基准（浅蓝正方体 40mm）"),
    dict(name="vec_04_orange_injured_cuboid_80x40", sample="orange_h15", w_mm=80, h_mm=40,
         kind="cuboid", expect=("ORANGE", "CUBOID"), old="REGULAR_SUPPLY", verdict="ok",
         note="★回归样本：伤员 80×40 扁盒（15 分/个）。旧实现 CUBE/CUBOID 判据完全重叠且 "
              "CUBE 先命中 → 判成 CUBE → 伤员必误判"),

    # ── 形状判据样本 ──
    dict(name="vec_05_orange_cuboid_front_40x40", sample="orange_h15", w_mm=40, h_mm=40,
         kind="cuboid(短边正对相机)", expect=("ORANGE", "*"), old="INJURED", verdict="limitation",
         note="伤员正对短边成像 40×40 —— **长宽比在此视角下不可区分**（见文档『不可区分清单』），"
              "靠『橘色在初赛/决赛都只对应伤员』这一颜色唯一性救回 15 分"),
    dict(name="vec_06_black_cube_40x40", sample="black_v20", w_mm=40, h_mm=40,
         kind="cube", expect=("BLACK", "CUBE"), old="CORE_SUPPLY", verdict="ok",
         expect_type_override="CORE_SUPPLY",
         note="⚠黑色正方形投影的**形状必然误判**：40×40 方框 → CUBE（真相是三棱锥的正方形投影）"
              "。颜色信息（黑=核心物资）把它救成 CORE_SUPPLY；想靠形状认三棱锥必须能看到三角形轮廓"),
    dict(name="vec_07_orange_circle_d40", sample="orange_h15", w_mm=40, h_mm=40,
         draw="circle", kind="circle(圆/圆柱/球俯视)", expect=("ORANGE", "*"), old="CORE_SUPPLY", verdict="limitation",
         note="★判据暴露：φ40 圆（画布真值 顶点=8、面积比=0.71；检测掩码 顶点=6）正好骑在 CUBE 的面积比下界 0.75 之下，顶点数又落进 4~8 → 形状判成 CUBE。"
              "颜色/phase 组合救回：橘→伤员。但**决赛的圆柱/球若为白色/其他色，形状必错**"),
    dict(name="vec_08_green_circle_d40", sample="green_h60", w_mm=40, h_mm=40,
         draw="circle", kind="circle(决赛圆柱俯视)", expect=("GREEN", "*"),
         phase="FINAL", old="REGULAR_SUPPLY", verdict="limitation",
         note="★决赛普通物资（绿圆柱 φ40×60）俯视投影。形状判成 CUBE（应为 CYLINDER）→ "
              "靠 phase=FINAL + 绿色唯一性救回 5 分；若决赛现场圆柱是别的颜色，此路不通"),

    # ── 蓝色族（B5，最高风险）──
    dict(name="vec_09_regression_blue_saturated_h112", sample="blue_h112", w_mm=40, h_mm=40,
         kind="cube", expect=(None, None), old="DANGEROUS", verdict="phase",
         note="★回归样本：饱和蓝 H=112/S=229。旧实现容差命中 LIGHT_BLUE → 判成危险目标 → 该搬的不搬；"
              "现实现 → 未识别（初赛没有蓝色规则）。**在初赛它必须永远不是 DANGEROUS**"),
    dict(name="vec_10_regression_pure_blue_h120", sample="blue_pure_h120", w_mm=40, h_mm=40,
         kind="cube", expect=(None, None), old="DANGEROUS", verdict="phase",
         note="★回归样本：纯蓝 BGR(255,0,0) = H120/S255/V255（最经典的「蓝」）。"
              "**在初赛必须检不出/不判危险**；在决赛（phase=FINAL）必须是 BLUE 而不是 LIGHT_BLUE"),
    dict(name="vec_11_cyan_h90_overlap", sample="cyan_h90", w_mm=40, h_mm=40,
         kind="cube", expect=("LIGHT_BLUE", "CUBE"), old="DANGEROUS", verdict="boundary",
         note="⚠薄弱边界：青 H=90/S=255 同时满足 BLUE 与 LIGHT_BLUE 阈值，靠『LIGHT_BLUE 先检测』"
              "判为浅蓝(危险)。现场若真出现青色目标会被当危险物排除"),
    dict(name="vec_12_blue_h111_gap", sample="blue_h111", w_mm=40, h_mm=40,
         kind="cube", expect=(None, None), old="DANGEROUS", verdict="phase",
         note="⚠★**阈值空隙（最危险的一条）**：H=111/S=255/V=255 两边都不沾——"
              "LIGHT_BLUE 的 H 上界是 110、BLUE 的 H 下界是 95（本该命中），"
              "但初赛配置表里没有 BLUE → 该颜色根本不在检测颜色列表里 → **整个目标消失**。"
              "蓝系目标只要落到 H≥111，初赛必漏检"),
    dict(name="vec_13_palepaleblue_h98_s10", sample="palepaleblue_h98", w_mm=40, h_mm=40,
         kind="cube", expect=(None, None), old="DANGEROUS", verdict="field-calib",
         note="⚠薄弱边界：S=10 的极淡蓝（反光/过曝）—— **实测完全检不出**（LIGHT_BLUE 的 S 下界是 20），危险目标在此光照下会漏检；但若把 S 下界压到 10 以下，白墙/灯光又会大量误检，属于两难，需现场权衡",
         tolerant=True),

    # ── 橘色族（伤员=15 分，标定敏感）──
    dict(name="vec_14_orange_light_s180", sample="orange_light_h15", w_mm=80, h_mm=40,
         kind="cuboid", expect=(None, None), old="INJURED", verdict="field-calib",
         note="⚠★**饱和度边界（现场高发）**：强光下的橘色 S=180 掉到 ORANGE 的 S 下界 205 之下 → "
              "**完全检不出** → 15 分/个丢。ORANGE 的 S 下界 205 是排红色用的，"
              "必须按现场实物取样值重新标定（见文档建议）",
         tolerant=True),
    dict(name="vec_15_orange_s205_boundary", sample="orange_s205", w_mm=80, h_mm=40,
         kind="cuboid", expect=("ORANGE", "CUBOID"), old="INJURED", verdict="ok",
         note="橘色 S=205 正好压在下界上 → 必须命中（留给现场标定的「最小值」参考）"),

    # ── 绿色/黑色亮度边界 ──
    dict(name="vec_16_green_dark_v70", sample="green_dark_h60", w_mm=40, h_mm=40,
         kind="cube", expect=("GREEN", "CUBE"), old="REGULAR_SUPPLY", verdict="ok",
         note="阴影里的绿 V=70（GREEN 的 V 下界是 60）→ 仍正确识别"),
    dict(name="vec_17_green_dark_v55_black_collision", sample="green_dark55_h60",
         w_mm=40, h_mm=40, kind="cube", expect=("BLACK", "CUBE"), old="REGULAR_SUPPLY", verdict="limitation",
         note="★★**实测碰撞**：绿 V=55 一旦跌破 60，就被 BLACK 全包区间 (S0~255,V0~60) 吃成"
              "『黑色核心物资』→ 5 分当 10 分、形状还会被判成三棱锥。"
              "现场灯光不足时这是**整块目标串类**的严重风险"),

    # ── 背景/干扰必须检不出 ──
    dict(name="vec_18_white_background", sample="white_s0", w_mm=200, h_mm=140,
         kind="flat", expect=(None, None), old=None, verdict="ok",
         note="白纸/灯光（S=0,V=255）必须检不出 —— 否则规则⑦的白色标注会被当成目标"),
    dict(name="vec_19_white_wide", sample="white_s0", w_mm=400, h_mm=100,
         kind="flat", expect=(None, None), old=None, verdict="ok",
         note="白色宽条（场地边线/白板）必须检不出"),
    dict(name="vec_20_gray_floor", sample="gray_s0", w_mm=200, h_mm=140,
         kind="flat", expect=(None, None), old=None, verdict="ok",
         note="亮灰地面/桌面色（V=128）必须检不出"),
    dict(name="vec_21_brown_h18_field", sample="brown_h18", w_mm=80, h_mm=40,
         kind="cuboid", expect=(None, None), old=None, verdict="ok",
         note="⚠薄弱边界：木纹/桌面色 H=18/S=180/V=120。靠 ORANGE 的 S 下界 205 躲开，"
              "但与『橘色』只隔一个饱和度；场地偏橘时首选误检对象"),
]

# ============================================================
# HSV_RANGES 判定（与检测器完全同源的判据）
# ============================================================

def _ranges_for(color: TargetColor):
    """取某目标色的 HSV 区间列表（红色是两个区间）"""
    from rescue_robot.perception.detection import HSV_RANGES, HSV_RANGES_RED2
    rng = HSV_RANGES.get(color)
    if rng is None:
        return []
    out = [rng]
    if color == TargetColor.RED:
        out.append(HSV_RANGES_RED2)
    return out


def in_range(h, s, v, rng) -> bool:
    (h0, s0, v0), (h1, s1, v1) = rng
    if h < 0:                       # 灰色等无 hue 的样本：只要 S 命中即可
        return s0 <= s <= s1 and v0 <= v <= v1
    return h0 <= h <= h1 and s0 <= s <= s1 and v0 <= v <= v1


def matched_colors(h, s, v):
    """该像素值命中了哪些 HSV_RANGES 桶（按检测顺序，LIGHT_BLUE 在前）"""
    ordered = [TargetColor.LIGHT_BLUE, TargetColor.BLUE]
    from rescue_robot.perception.detection import HSV_RANGES
    ordered += [c for c in HSV_RANGES if c not in ordered]
    hits = []
    for c in ordered:
        if any(in_range(h, s, v, r) for r in _ranges_for(c)):
            hits.append(c)
    return hits


def hsv_of_sample(key):
    """样本的实测 HSV（灰卡特殊处理：OpenCV 对 S=0 的 H 返回 0，这里标 -1 表示无 hue）"""
    _label, h, s, v = SAMPLES[key]
    if key == "gray_s0":
        h = -1
    return h, s, v


# ============================================================
# 几何绘制
# ============================================================

def draw_blob(frame, rect, bgr, shape="rect"):
    """
    画一个色块。形状贴近目标的**正面/俯视投影**，便于现场肉眼比对：
      rect     矩形（长方体俯视/正视投影、立方体正视投影都是矩形）
      triangle 等腰三角形（正三棱锥正视投影 —— 三角形轮廓才是"三棱锥"的可靠判据）
      circle   圆（圆柱/圆锥台/球的俯视投影）
    注意：这些都是**投影轮廓**，不是 3D 渲染；本向量集只测"投影轮廓 → 判据"这一段，
    3D 立体感不是被测对象（真机照片请用 tools/vision_test.py --image）。
    """
    x, y, w, h = rect
    if shape == "triangle":
        pts = np.array([[x, y + h - 1], [x + w - 1, y + h - 1], [x + w // 2, y]], np.int32)
        cv2.fillPoly(frame, [pts], bgr)
    elif shape == "circle":
        cv2.circle(frame, (x + w // 2, y + h // 2), max(2, min(w, h) // 2), bgr, -1)
    else:
        cv2.rectangle(frame, (x, y), (x + w, y + h), bgr, -1)


def darken(bgr, factor):
    return tuple(max(0, min(255, int(round(c * factor)))) for c in bgr)


def make_image(case):
    """
    生成单样本图 + 色块 bbox。

    底部/背景元素先画（深灰板，灰度上不命中任何目标色阈值），
    待测色块画在其上，避免与底色混合出"半个像素"的中间色。
    """
    frame = np.full((FRAME_H, FRAME_W, 3), BG_BGR, np.uint8)
    w, h = px(case["w_mm"]), px(case["h_mm"])
    x = (FRAME_W - w) // 2
    y = (FRAME_H - h) // 2 - 30
    bgr = hsv_to_bgr(*hsv_of_sample(case["sample"]))
    draw_blob(frame, (x, y, w, h), bgr, case.get("draw", "rect"))
    return frame, [(x, y, w, h)]


def make_sidebyside_image(case, left_hsv, right_hsv, w_mm, h_mm):
    """
    并排样本：同一帧里放两个色块，用来验证"同一帧里两色不互相串"。
    两块的**像素值独立**，绘制顺序不影响（不相邻）。
    """
    frame = np.full((FRAME_H, FRAME_W, 3), BG_BGR, np.uint8)
    w, h = px(w_mm), px(h_mm)
    y = (FRAME_H - h) // 2
    gap = 40                       # 两块之间留 40px 空白：避免形态学把两块连成一个轮廓
    lx = FRAME_W // 2 - w - gap // 2
    rx = FRAME_W // 2 + gap // 2
    blobs = []
    for x, hsv in ((lx, left_hsv), (rx, right_hsv)):
        draw_blob(frame, (x, y, w, h), hsv_to_bgr(*hsv), "rect")
        blobs.append((x, y, w, h))
    return frame, blobs


def sidebyside_expected(phase):
    """『浅蓝 vs 蓝并排』样本的期望值：两块分别查表"""
    cfg = get_target_config(phase)
    out = []
    for color in (TargetColor.LIGHT_BLUE, TargetColor.BLUE):
        infos = [i for (c, _s), i in cfg.items() if c == color]
        # 该颜色在表里唯一 → 类型可查；不唯一 → 必须靠形状定
        tname = infos[0].type.name if len(infos) == 1 else "*"
        out.append((color.name, tname))
    return out


# ============================================================
# 期望/旧实现预期 解析
# ============================================================

def expect_resolve(phase, color_name, shape_name):
    """期望 (颜色, 形状) → 目标类型名；无此组合 → None"""
    if color_name is None:
        return None
    cfg = get_target_config(phase)
    for (color, shape), info in cfg.items():
        if color.name == color_name and shape.name == shape_name:
            return info.type.name
    return None


def build_records():
    """构造全部向量记录（分帧样本 + 并排样本）"""
    img_dir_rel = os.path.join("img")
    records = []
    for case in CASES:
        c = dict(case)
        c["phase"] = CompetitionPhase.PRELIMINARY.name
        c["image"] = os.path.join(img_dir_rel, case["name"] + ".png")
        color_name = case["expect"][0]
        shape_name = case["expect"][1]
        c["kind_expect"] = ("none" if color_name is None else
                            ("color+shape" if shape_name not in (None, "*") else "color"))
        c["expect_shape"] = ["*" if shape_name is None else shape_name][0]
        # 期望形状：颜色为 None（应该检不出）时形状判定也必须是 None
        if color_name is None:
            c["expect_shape"] = None
        c["expect_type"] = expect_resolve(CompetitionPhase.PRELIMINARY,
                                          color_name, shape_name)
        records.append(c)

    # ── 并排样本（同一帧里 浅蓝 + 蓝）──
    for name, blobs_cfg, note in (
        ("vec_22_lightblue_vs_blue_sbs",
         [(("lightblue_h96"), (TargetColor.LIGHT_BLUE, "CUBE")),
          (("blue_pure_h120"), (TargetColor.BLUE, None))],
         "★回归样本：浅蓝与蓝**并排同帧** —— 两者必须分别判对；"
         "旧实现两块都会落进重叠阈值，靠『先到先得』互相串色"),
    ):
        exp_pairs = sidebyside_expected(CompetitionPhase.PRELIMINARY)
        blobs = []
        for (sample, (color, shape)), (exp_color, exp_type) in zip(blobs_cfg, exp_pairs):
            blobs.append(dict(sample=sample, color=color.name, shape=shape,
                              expect_type=exp_type, expect_color=exp_color))
        records.append(dict(
            name=name, sample=None, w_mm=40, h_mm=40, kind="cube", image=None,
            phase=CompetitionPhase.PRELIMINARY.name, blobs=blobs,
            side_by_side=True, note=note,
            # 旧实现：浅蓝块先命中浅蓝阈值但蓝块也命中浅蓝（重叠）→ 两块都 DANGEROUS；
            # 现实现：浅蓝块 LIGHT_BLUE，蓝块 BLUE(→ 初赛无蓝色规则 → None)
            old_each=[exp_pairs[0][1], "DANGEROUS"],
            verdict="limitation",
        ))
    return records


def main():
    os.makedirs(IMG_DIR, exist_ok=True)
    records = build_records()

    # 分帧样本图
    for rec in records:
        if rec.get("side_by_side"):
            hsv_l = hsv_of_sample("lightblue_h96")
            hsv_b = hsv_of_sample("blue_h112")
            case = dict(w_mm=rec["w_mm"], h_mm=rec["h_mm"])
            frame, blobs = make_sidebyside_image(case, hsv_l, hsv_b,
                                                 rec["w_mm"], rec["h_mm"])
            rec["blob_boxes"] = blobs
        else:
            frame, blobs = make_image(rec)
            rec["blob_boxes"] = blobs
        rec["bg_bgr"] = list(BG_BGR)
        rec["frame_wh"] = [FRAME_W, FRAME_H]
        path = os.path.join(HERE, rec["image"] or "")
        if rec.get("side_by_side"):
            path = os.path.join(IMG_DIR, rec["name"] + ".png")
            rec["image"] = os.path.join("img", rec["name"] + ".png")
        ok = cv2.imwrite(path, frame)
        if not ok:
            raise SystemExit(f"写图失败: {path}")

    manifest = dict(
        schema=1,
        generated_by="tools/vision_vectors/generate_vectors.py",
        deterministic=True,
        note="合成图，非真实照片；固定底色 BGR(128,128,128)，无随机数",
        frame_wh=[FRAME_W, FRAME_H],
        bg_bgr=list(BG_BGR),
        records=records,
    )
    with open(MANIFEST, "w", encoding="utf-8") as f:
        json.dump(manifest, f, ensure_ascii=False, indent=2)

    print(f"✅ 生成 {len(records)} 个向量 → {IMG_DIR}")
    for rec in records:
        print(f"   {rec['name']:42s} {rec['kind']:32s} {rec.get('note', '')[:40]}")
    print(f"✅ 清单 → {MANIFEST}")


if __name__ == "__main__":
    main()
