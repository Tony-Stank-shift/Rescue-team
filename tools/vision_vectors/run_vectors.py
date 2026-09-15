"""run_vectors.py —— 视觉识别测试向量集比对脚本（逐样本 期望/实际/是否一致）

对 `tools/vision_vectors/img/` 下每一张确定性合成图跑**真实的** `CVDetector` +
`TargetClassifier`，逐样本打印 期望 / 实际 / 是否一致，末尾给 `通过 N/M`。
文档里的所有数字都是本脚本的原始输出，不是手写的。

为什么以"最终目标类型"为判据
----------------------------
丢分的不是颜色也不是形状，而是**最终类型**：类型错 → 决策层搬错/不搬。
所以每行都给到 期望类型 / 实际类型，颜色与形状作为"为什么"的证据一起打印。

期望值来源
----------
* 期望颜色/形状：合成图是我们自己画的，向量清单 `vectors.json` 里写死；
  `*` = 该样本的形状不参与判定（见文档『不可区分清单』）。
* 期望类型：由 `get_target_config(phase)` **查表得出**，与生产代码同源。

phase 说明
----------
初赛与决赛的**检测颜色列表**不同（`CVDetector._colors_to_detect` 只取当前阶段配置表里
出现过的颜色）：初赛/决赛都是 GREEN / BLACK / ORANGE / LIGHT_BLUE，**都没有 BLUE**。
所以"纯蓝"在初赛阶段根本不在检测颜色里；它是否被检出，取决于它有没有被 LIGHT_BLUE
阈值吃掉（吃掉了 → 判 DANGEROUS，这是最严重的一类错误）。末段会把全部样本在
初赛与决赛各跑一遍做对照。

用法::

    python3 tools/vision_vectors/run_vectors.py              # 全部样本 + 对照表
    python3 tools/vision_vectors/run_vectors.py --index      # 附测得特征值与判据区间证据
    python3 tools/vision_vectors/run_vectors.py --markdown   # 追加 markdown 表格
"""

import argparse
import json
import logging
import math
import os
import sys

_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
_SRC = os.path.join(_ROOT, "src")
_HERE = os.path.dirname(os.path.abspath(__file__))
for _p in (_SRC, _HERE):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import cv2          # noqa: E402
import numpy as np  # noqa: E402

logging.disable(logging.CRITICAL)   # 只要结果，不要检测器日志

from rescue_robot.perception.classification import TargetClassifier     # noqa: E402
from rescue_robot.perception.detection import (                        # noqa: E402
    CVDetector, HSV_RANGES,
    SHAPE_AREA_RATIOS, SHAPE_ASPECT_RATIOS, SHAPE_VERTEX_RANGES,
)
from rescue_robot.perception.target_types import (                     # noqa: E402
    CompetitionPhase, TargetColor, TargetShape, get_target_config,
)
from samples import (                                                  # noqa: E402
    SAMPLES, hsv_of_sample, bgr_of_sample, hsv_roundtrip,
    matched_colors, ranges_for,
)

MANIFEST = os.path.join(_HERE, "vectors.json")
PHASES = [CompetitionPhase.PRELIMINARY, CompetitionPhase.FINAL]


# ============================================================
# 基础工具
# ============================================================

def iou(a, b) -> float:
    ax, ay, aw, ah = a
    bx, by, bw, bh = b
    x1, y1 = max(ax, bx), max(ay, by)
    x2, y2 = min(ax + aw, bx + bw), min(ay + ah, by + bh)
    if x2 <= x1 or y2 <= y1:
        return 0.0
    inter = (x2 - x1) * (y2 - y1)
    union = aw * ah + bw * bh - inter
    return inter / union if union > 0 else 0.0


def pick(dets, box):
    """挑出与该色块对应的检测：先比 IoU，再比中心距"""
    if not dets:
        return None
    cx, cy = box[0] + box[2] / 2.0, box[1] + box[3] / 2.0
    best, key0 = None, None
    for d in dets:
        key = (iou(d.bbox, box), -math.hypot(d.center_pixel[0] - cx, d.center_pixel[1] - cy))
        if key0 is None or key > key0:
            best, key0 = d, key
    return best


def contour_features(img, box):
    """
    直接测量"我们画的那块图形"→ 与 HSV 阈值**无关**的真值特征：长宽比 / 面积比 / 顶点数。

    做法：以画面底色（合成图固定 BGR(128,128,128)）为基准做颜色距离阈值分割，
    取最大轮廓。**不用 Otsu**——浅色块在灰底上会被 Otsu 反相，量出的是色块之外的背景
    （第一版就踩了这个坑：橘圆的面积比被量成 0.18，其实真值是 0.75）。

    这些数字是文档『不可区分清单』与判据阈值讨论的依据。
    """
    x, y, w, h = box
    sub = img[max(0, y - 6):y + h + 6, max(0, x - 6):x + w + 6]
    bg = np.array([128, 128, 128], dtype=np.int16)
    dist = np.abs(sub.astype(np.int16) - bg).sum(axis=2)
    mask = np.where(dist > 25, 255, 0).astype(np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,
                            cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3)))
    cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not cnts:
        return None
    c = max(cnts, key=cv2.contourArea)
    area = cv2.contourArea(c)
    bw, bh = cv2.boundingRect(c)[2:]
    fill = area / float(bw * bh) if bw * bh else 0.0
    ratio = max(bw, bh) / float(min(bw, bh)) if min(bw, bh) else 1.0
    peri = cv2.arcLength(c, True)
    return dict(w=bw, h=bh, ratio=ratio, fill=fill,
                verts=len(cv2.approxPolyDP(c, 0.04 * peri, True)),
                n_edges=len(c), n_px=int(area))


def shape_reason(shape: TargetShape, feat) -> str:
    """一句话说明"形状判定命中了哪个区间"——阈值直接取自生产代码"""
    if feat is None:
        return "无特征"
    r, a, v = feat["ratio"], feat["fill"], feat["verts"]
    head = f"长宽比={r:.2f} 面积比={a:.2f} 顶点={v}"
    if shape == TargetShape.UNKNOWN:
        return head + " → 未命中任何形状区间 → UNKNOWN"
    ar, vr, asp = (SHAPE_AREA_RATIOS.get(shape), SHAPE_VERTEX_RANGES.get(shape),
                   SHAPE_ASPECT_RATIOS.get(shape))
    hit = []
    if ar:
        hit.append(f"面积比∈[{ar[0]},{ar[1]}]{'✓' if ar[0] <= a <= ar[1] else '✗'}")
    if vr:
        hit.append(f"顶点∈[{vr[0]},{vr[1]}]{'✓' if vr[0] <= v <= vr[1] else '✗'}")
    if asp:
        hit.append(f"长宽比∈[{asp[0]},{asp[1]}]{'✓' if asp[0] <= r <= asp[1] else '✗'}")
    return head + " → " + shape.name + "（" + " ".join(hit) + "）"


def classify_one(detector, clf, img, box):
    """对一张图里的一个色块：检测 → 选块 → 分类；返回 (detection, 类型名, 全部检测)"""
    dets = detector.detect(img)
    d = pick(dets, box)
    tname = None
    if d is not None:
        t = clf.classify(d, 0.0)
        tname = t.info.type.name if t is not None else None
    return d, tname, dets


def run_record(rec, phase_override=None):
    """跑一个分帧样本 → (结果 dict, 全部检测)"""
    phase = phase_override or CompetitionPhase[rec["phase"]]
    det = CVDetector(phase=phase)
    clf = TargetClassifier(phase=phase)
    img = cv2.imread(os.path.join(_HERE, rec["image"]))
    if img is None:
        raise SystemExit(f"读不到向量图 {rec['image']}（先跑 generate_vectors.py）")

    # 期望类型随 phase 变化：只有"该阶段配置表里存在该 (颜色,形状)"才有期望类型
    cfg = get_target_config(phase)
    exp_color, exp_shape = rec["expect"][0], rec["expect_shape"]
    exp_type = None
    if exp_color is not None:
        for (c, s), info in cfg.items():
            if c.name == exp_color and (exp_shape in (None, "*") or s.name == exp_shape):
                exp_type = info.type.name
                break
    # (颜色,形状) 不在配置表里时（例如"黑色+正方形"——初赛只定义了黑+三棱锥），
    # 分类器仍有"该颜色唯一且非危险 → 按颜色兜底"的规则（见 classification.py::_fuzzy_match）。
    # 这类样本的期望值在向量清单里显式标注，并注明来源。
    exp_type_src = "查表(get_target_config)"
    if rec.get("expect_type_override") is not None:
        exp_type = rec["expect_type_override"]
        exp_type_src = "显式标注（颜色唯一兜底规则）"
    d, got_type, dets = classify_one(det, clf, img, rec["blob_boxes"][0])
    got_color = d.color.name if d else None
    got_shape = d.shape.name if d else None
    feat = contour_features(img, rec["blob_boxes"][0])

    ok_color = (exp_color == got_color)
    ok_shape = (exp_shape in (None, "*")) or (exp_shape == got_shape)
    ok_type = (exp_type == got_type)
    return dict(rec=rec, phase=phase.name, d=d, dets=dets, feat=feat,
                exp_type_src=exp_type_src,
                exp_color=exp_color, exp_shape=exp_shape, exp_type=exp_type,
                got_color=got_color, got_shape=got_shape, got_type=got_type,
                ok_color=ok_color, ok_shape=ok_shape, ok_type=ok_type,
                passed=(ok_color and ok_shape and ok_type),
                shape_why=shape_reason(d.shape, feat) if d else "无检测"), dets


def run_sidebyside(rec):
    """并排样本：同一帧里两个色块，分别比对"""
    phase = CompetitionPhase[rec["phase"]]
    det = CVDetector(phase=phase)
    clf = TargetClassifier(phase=phase)
    img = cv2.imread(os.path.join(_HERE, rec["image"]))
    rows = []
    for blob, box in zip(rec["blobs"], rec["blob_boxes"]):
        d, got_type, dets = classify_one(det, clf, img, box)
        rows.append(dict(blob=blob, d=d,
                         got_color=d.color.name if d else None,
                         got_shape=d.shape.name if d else None,
                         got_type=got_type,
                         ok_color=(blob["color"] == (d.color.name if d else None)),
                         ok_type=(blob["expect_type"] == got_type)))
    return rows, det


# ============================================================
# 主流程
# ============================================================

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--index", action="store_true", help="附测得特征值 / 判据区间证据")
    ap.add_argument("--markdown", action="store_true", help="追加 markdown 表格")
    args = ap.parse_args()

    with open(MANIFEST, encoding="utf-8") as f:
        man = json.load(f)
    records = man["records"]

    # ── ① HSV_RANGES 现状 ──
    note_map = {
        TargetColor.BLUE: "S 下界 120（靠饱和度认蓝）",
        TargetColor.LIGHT_BLUE: "H 上界 110 / BLUE 下界 95 → H∈[95,110] 重叠",
        TargetColor.ORANGE: "S 下界 205 用来排红色（很强，现场最容易踩）",
        TargetColor.BLACK: "H/S 全包，只靠 V≤60",
        TargetColor.GREEN: "V 下界 60 与 BLACK 的 V 上界 60 紧邻",
        TargetColor.WHITE: "S≤30 且 V≥180",
    }
    print("=" * 128)
    print("① 当前 HSV_RANGES（src/rescue_robot/perception/detection.py；打印顺序 = 检测顺序）")
    print("=" * 128)
    print(f"{'颜色':<12}{'H_min':>6}{'S_min':>6}{'V_min':>6}   {'H_max':>5}{'S_max':>6}{'V_max':>6}"
          f"   备注")
    for color, rng in HSV_RANGES.items():
        for lo, hi in ranges_for(color):
            print(f"{color.name:<12}{lo[0]:>6}{lo[1]:>6}{lo[2]:>6}   "
                  f"{hi[0]:>5}{hi[1]:>6}{hi[2]:>6}   {note_map.get(color, '')}")
    print()

    # ── ② 逐样本比对 ──
    results = []
    print("=" * 128)
    print("② 逐样本比对（期望 ← vectors.json / 实际 ← 真实 CVDetector + TargetClassifier）")
    print("=" * 128)
    print(f"{'样本':<42}{'phase':<12}{'期望颜色':<11}{'实际颜色':<11}"
          f"{'期望形状':<21}{'实际形状':<21}{'期望类型':<17}{'实际类型':<17}{'一致':<6}{'类别'}")
    print("-" * 128)
    for rec in records:
        if rec.get("side_by_side"):
            rows, _det = run_sidebyside(rec)
            for row in rows:
                nm = f"{rec['name']}[{row['blob']['sample']}]"
                ok = row["ok_color"] and row["ok_type"]
                print(f"{nm:<42}{rec['phase']:<12}{row['blob']['color']:<11}"
                      f"{str(row['got_color']):<11}{'*':<21}{str(row['got_shape']):<21}"
                      f"{row['blob']['expect_type']:<17}{str(row['got_type']):<17}"
                      f"{'✅' if ok else '❌':<6}{rec.get('verdict', '?')}")
                results.append(dict(name=nm, rec=rec, exp_color=row["blob"]["color"],
                                    got_color=row["got_color"],
                                    exp_type=row["blob"]["expect_type"],
                                    got_type=row["got_type"], passed=ok,
                                    exp_shape="*", got_shape=row["got_shape"]))
            if args.index:
                print(f"      └ 备注: {rec.get('note', '')}")
            continue

        r, dets = run_record(rec)
        exp_color = rec["expect"][0] or "(应检不出)"
        exp_shape = ("(检不出)" if rec["expect"][0] is None else (rec["expect_shape"] or "*"))
        print(f"{rec['name']:<42}{r['phase']:<12}{exp_color:<11}{str(r['got_color']):<11}"
              f"{exp_shape:<21}{str(r['got_shape']):<21}"
              f"{str(r['exp_type']):<17}{str(r['got_type']):<17}"
              f"{'✅' if r['passed'] else '❌':<6}{rec.get('verdict', '?')}")
        results.append(dict(name=rec["name"], rec=rec, exp_color=exp_color,
                            got_color=r["got_color"], exp_type=r["exp_type"],
                            got_type=r["got_type"], passed=r["passed"],
                            exp_shape=exp_shape, got_shape=r["got_shape"]))
        if args.index:
            if r["d"]:
                print(f"      └ 检出: bbox={r['d'].bbox} 顶点={r['d'].contour_vertices} "
                      f"conf={r['d'].confidence:.2f} 面积={r['d'].contour_area:.0f}px²")
            print(f"      └ 期望类型来源: {r.get('exp_type_src', '查表')}")
            if r["feat"]:
                f = r["feat"]
                print(f"      └ 画布真值: {f['w']}×{f['h']}px 长宽比={f['ratio']:.2f} "
                      f"面积比={f['fill']:.2f} 顶点={f['verts']}")
            print(f"      └ 形状判据: {r['shape_why']}")
            extra = [x for x in dets if x is not r["d"]]
            if extra:
                print(f"      └ 同帧其它检测: "
                      f"{[(x.color.name, x.shape.name, x.bbox) for x in extra]}")
            print(f"      └ 备注: {rec.get('note', '')}")
    print("-" * 128)

    total, passed = len(results), sum(1 for x in results if x["passed"])
    print(f"\n通过 {passed}/{total}")
    cat = {}
    for x in results:
        cat.setdefault(x["rec"].get("verdict", "?"), []).append(x)
    legend = {
        "ok": "基准确认（现状正确）",
        "limitation": "判据局限（形状本身不可分，靠 颜色+phase 救回，见不可区分清单）",
        "phase": "phase 相关（初赛配置表无该颜色 → 检不出，属正确行为）",
        "boundary": "阈值边界（现场需确认）",
        "field-calib": "现场标定项（阈值与现场光照不匹配 → 丢目标）",
        "error": "★识别错误（判成了错的目标类型，会丢分/违规）",
    }
    print("分类统计：")
    for k in ("ok", "limitation", "phase", "boundary", "field-calib", "error"):
        if k in cat:
            print(f"  {k:<12}{len(cat[k]):>2} 条   {legend.get(k, '')}")
            for x in cat[k]:
                if k in ("error", "boundary", "field-calib"):
                    flag = "❌ 未通过" if not x["passed"] else "✅ 现状可接受"
                    print(f"      {flag}  {x['name']}  期望{x['exp_color']}/{x['exp_type']} "
                          f"→ 实际{x['got_color']}/{x['got_type']}")

    fails = [x for x in results if not x["passed"]]
    if fails:
        print("\n未通过样本明细（每条都是『待现场标定』或『待返工』的证据）：")
        for x in fails:
            print(f"  ❌ {x['name']}  [{x['rec'].get('verdict', '?')}]")
            print(f"      颜色 期望={x['exp_color']} 实际={x['got_color']} | "
                  f"形状 期望={x['exp_shape']} 实际={x['got_shape']} | "
                  f"类型 期望={x['exp_type']} 实际={x['got_type']}")
            print(f"      说明: {x['rec'].get('note', '')}")
    else:
        print("\n没有未通过样本。")

    # ── ③ 回归样本 ──
    print("\n" + "=" * 128)
    print("③ 回归样本（旧实现会错 → 本向量集把它锁死，防止改回去）")
    print("=" * 128)
    print(f"{'样本':<42}{'旧实现结果':<20}{'现在结果':<20}{'结论'}")
    reg, same = [], []
    for x in results:
        old = x["rec"].get("old")
        if old in (None, "-"):
            continue
        (reg if old != x["got_type"] else same).append(x)
    for x in reg:
        print(f"{x['name']:<42}{str(x['rec']['old']):<20}{str(x['got_type']):<20}"
              f"✅ 已修好（旧实现会判成 {x['rec']['old']}）")
    print(f"（真回归锁 {len(reg)} 条：旧实现≠现在，改回去会被本脚本抓到）")
    if same:
        print("\n以下样本新旧一致（用来确认『没把对的改错』，不属于回归锁）：")
        for x in same:
            print(f"  · {x['name']:<42}旧={str(x['rec']['old']):<18}现在={x['got_type']}")

    # ── ④ 双 phase 对照 ──
    print("\n" + "=" * 128)
    print("④ 双 phase 对照（同一张图在 初赛 / 决赛 下的实际 颜色/形状/类型）")
    print("=" * 128)
    print(f"{'样本':<42}{'初赛':<34}{'决赛':<34}")
    print("-" * 128)
    for rec in records:
        if rec.get("side_by_side"):
            continue
        cells = []
        for ph in PHASES:
            rr, _ = run_record(rec, phase_override=ph)
            cells.append(f"{rr['got_color']}/{rr['got_shape']}/{rr['got_type']}")
        mark = "" if cells[0] == cells[1] else "   ← phase 相关（决赛配置表里没有该 (颜色,形状)）"
        print(f"{rec['name']:<42}{cells[0]:<34}{cells[1]:<34}{mark}")

    # ── ⑤ RGB↔HSV 实测 + 桶覆盖 ──
    print("\n" + "=" * 128)
    print("⑤ RGB↔HSV 实测与 HSV_RANGES 覆盖（中心像素 BGR→HSV 往返实测；命中桶按检测顺序）")
    print("=" * 128)
    print(f"{'色卡样本':<34}{'BGR':<18}{'实测HSV':<16}{'命中桶':<30}{'现场含义'}")
    print("-" * 128)
    order = ["green_h60", "green_dark_h60", "green_dark55_h60", "black_v20", "black_v55",
             "orange_h15", "orange_s205", "orange_light_h15", "brown_h18",
             "lightblue_h96", "palepaleblue_h98",
             "cyan_h90", "blue_h111", "blue_h112", "blue_pure_h120", "bluepurple_h125",
             "white_s0", "gray_s0"]
    seen = set()
    for key in order:
        if key in seen or key not in SAMPLES:
            continue
        seen.add(key)
        label = SAMPLES[key][0]
        bgr = bgr_of_sample(key)
        h, s, v = hsv_roundtrip(key)
        hits = [c.name for c in matched_colors(h, s, v)]
        print(f"{label:<34}{str(bgr):<18}{f'({h},{s},{v})':<16}"
              f"{('/'.join(hits) if hits else '—（无桶命中）'):<30}"
              f"{'落进 ' + '/'.join(hits) + ' 桶' if hits else '任何桶都不命中 → 必检不出'}")

    # 「命中桶」≠「会被检出」：检测只跑当前 phase 配置表里出现过的颜色
    for ph in PHASES:
        cols = [c.name for c in CVDetector(phase=ph)._colors_to_detect]
        print(f"\n   {ph.name} 实际参与检测的颜色列表（顺序敏感）: {cols}")
        if TargetColor.BLUE.name not in cols:
            print(f"      ⚠ {ph.name} 没有 BLUE → 即使像素落进 BLUE 桶，也不会被检出；"
                  f"若同时落进 LIGHT_BLUE 桶，就会被判成危险目标(浅蓝)")
    print("   ⚠ 重叠区（H∈[95,110]）：BLUE 与 LIGHT_BLUE 两个桶都命中 → "
          "由检测顺序决定取 LIGHT_BLUE（危险）")

    # ── ⑥ markdown ──
    if args.markdown:
        print("\n<!-- markdown 表格（贴文档用） -->")
        print("| 样本 | 期望颜色 | 实际颜色 | 期望形状 | 实际形状 | 期望类型 | 实际类型 | 一致 |")
        print("|---|---|---|---|---|---|---|---|")
        for x in results:
            print(f"| `{x['name']}` | {x['exp_color']} | {x['got_color']} | "
                  f"{x['exp_shape']} | {x['got_shape']} | {x['exp_type']} | {x['got_type']} | "
                  f"{'✅' if x['passed'] else '❌'} |")

    return 0 if passed == total else 1


if __name__ == "__main__":
    sys.exit(main())
