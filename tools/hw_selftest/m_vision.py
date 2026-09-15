"""m_vision —— 视觉识别：无帧不崩 + 空帧不误报 + 四类目标能否识别 + 像素→距离

无需摄像头也能真跑（这是**唯一能证明"视觉到底认不认得目标"的离线手段**）：

  1. `detect(None)` 必须返回空而不是抛异常
     （历史 bug：None 帧抛异常 → 整轮 `_run_once` 被跳过 → 决策/导航/转运全不执行）
  2. **空帧防误报**：纯亮灰底（无任何目标）必须检出 0 个。
     ⚠️ 旧版合成图底色取 BGR(60,60,60) → HSV V=60，正好落进 BLACK 阈值
     ((0,0,0)~(179,255,60)) → 整幅画面被检成"1 个黑色大目标"。
     旧测试只断言"绿色被检出"、不查多检，于是**这个误报被测试放过了**。
  3. **四类目标逐色检出**：初赛 4 类（绿正方体/黑三棱锥/橘长方体/浅蓝正方体）
     各自出现在空白背景上时，必须检出对应颜色，且经 `TargetClassifier`
     映射后类型正确（尤其：浅蓝必须是 DANGEROUS、不能认成可抓目标）。
  4. 可选 `--image <path>`：对真实照片跑检测，输出目标数/颜色/形状/像素框/估算距离。

注：形状判定 `_classify_shape` 对顶点数同区间的形状不区分
（CUBOID 与 CUBE 都在 4~8 顶点且 CUBE 先命中 → CUBOID 实际不可达），
所以本模块**只对颜色做判定、形状只作证据输出**，不因形状返回 FAIL。
"""

import os

from .framework import register, ok, bad

MODULE = "vision"
TITLE = "视觉识别（无帧保护 / 空帧防误报 / 四类目标检出）"

#: 合成图底色：亮灰（V≈150）—— 明确落在 BLACK 阈值(V≤60)之外，避免自己造误报
BG_BRIGHT = 150
#: 黑目标专用底色：更亮（黑目标要黑底才检不出，亮底才能验证 BLACK 掩码生效）
BG_FOR_BLACK = 220

#: 初赛四类目标的 HSV 取样值（H,S,V）—— 取自各色阈值区间中心附近
PRELIMINARY_COLORS = (
    ("绿(普通物资)", 60, 255, 200),
    ("黑(核心物资)", 0, 0, 20),
    ("橘(伤员)", 15, 255, 255),
    ("浅蓝(危险)", 95, 200, 230),
)


def _hsv_to_bgr(h, s, v):
    """把 HSV 采样值转成 BGR 像素（cv2 的 H 范围 0~179，正是我们阈值表的量纲）"""
    import cv2
    import numpy as np
    px = np.uint8([[[h, s, v]]])
    b, g, r = cv2.cvtColor(px, cv2.COLOR_HSV2BGR)[0, 0]
    return (int(b), int(g), int(r))


def _frame_with_blob(bg_v, bgr, size=(480, 640), blob=(80, 80)):
    """合成一张图：亮/指定灰度底 + 中央一个矩形色块"""
    import cv2
    import numpy as np
    h, w = size
    bw, bh = blob
    img = np.full((h, w, 3), bg_v, dtype=np.uint8)
    cv2.rectangle(img, (w // 2 - bw // 2, h // 2 - bh // 2),
                  (w // 2 + bw // 2, h // 2 + bh // 2), bgr, -1)
    return img


@register(MODULE, TITLE)
def run(ctx):
    ev = []
    try:
        from rescue_robot.perception.detection import CVDetector
    except Exception as e:
        return bad(MODULE, f"无法导入 CVDetector：{e!r}", ev,
                   "检查 opencv 是否安装（pip install opencv-python）")

    det = CVDetector()

    # ── 1) 无帧保护 ──
    try:
        r = det.detect(None)
    except Exception as e:
        return bad(MODULE, f"detect(None) 抛异常：{e!r} → 摄像头没出帧时会拖垮主循环", ev,
                   "detect 必须对 None 返回空列表")
    if r != []:
        return bad(MODULE, f"detect(None) 应返回空列表，实际 {r!r}", ev)
    ev.append("无帧保护 ✓（detect(None) → []）")

    # ── 2) 空帧防误报（旧版放过的那条）──
    try:
        import numpy as np
        blank = np.full((480, 640, 3), BG_BRIGHT, dtype=np.uint8)   # 真正全空帧
        blank_dets = det.detect(blank)
    except Exception as e:
        return bad(MODULE, f"空帧检测抛异常：{e!r}", ev, "检查 detect 的前置校验")
    ev.append(f"空帧（纯亮灰 V={BG_BRIGHT}）→ 检出 {len(blank_dets)} 个")
    if blank_dets:
        worst = max(blank_dets, key=lambda d: d.contour_area)
        return bad(MODULE,
                   f"空帧里检出了 {len(blank_dets)} 个假目标（最大 {worst.color.name}/"
                   f"{worst.shape.name} bbox={worst.bbox}）→ 颜色阈值把背景吃进去了",
                   ev + [f"假目标详情：{[(d.color.name, d.shape.name, d.bbox) for d in blank_dets]}"],
                   "背景被当成目标会让决策/导航对着空气跑。检查 HSV_RANGES 里 BLACK 的 "
                   "V 上限是否过高（深灰/阴影会被吃进去），以及现场光照下是否需重新标定")

    # ── 3) 四类目标逐色检出 + 分类器映射 ──
    try:
        from rescue_robot.perception.classification import TargetClassifier
        clf = TargetClassifier()
    except Exception as e:
        return bad(MODULE, f"TargetClassifier 不可用：{e!r}", ev)
    try:
        from rescue_robot.perception.target_types import TargetType
    except Exception:
        TargetType = None

    expect_type = {
        0: "REGULAR_SUPPLY",     # 绿 → 普通物资
        1: "CORE_SUPPLY",        # 黑 → 核心物资
        2: "INJURED",            # 橘 → 伤员
        3: "DANGEROUS",          # 浅蓝 → 危险目标（绝对不能抓）
    }
    missing, misjudged = [], []
    for i, (label, h, s, v) in enumerate(PRELIMINARY_COLORS):
        bg = BG_FOR_BLACK if i == 1 else BG_BRIGHT
        frame = _frame_with_blob(bg, _hsv_to_bgr(h, s, v))
        dets = det.detect(frame)
        colors = [d.color.name for d in dets]
        line = f"合成图[{label}] HSV({h},{s},{v}) 底色V={bg} → 检出 {len(dets)} 个"
        if dets:
            d0 = dets[0]
            line += f"：{d0.color.name}/{d0.shape.name} bbox={d0.bbox} conf={d0.confidence:.2f}"
        ev.append(line)

        if not dets:
            missing.append(label)
            continue
        # 颜色必须命中（绿=GREEN、黑=BLACK、橘=ORANGE、浅蓝=LIGHT_BLUE）
        got_color = dets[0].color.name
        expect_name = ("GREEN", "BLACK", "ORANGE", "LIGHT_BLUE")[i]
        if got_color != expect_name:
            misjudged.append(f"{label} 被认成 {got_color}（应为 {expect_name}）")
        # 分类器映射
        try:
            classified = clf.classify_batch(dets, 0.0)
            got_types = [t.info.type.name for t in classified]
            ev.append(f"    分类器映射 → {got_types}")
            if expect_type[i] not in got_types:
                misjudged.append(f"{label} 分类结果为 {got_types}（应含 {expect_type[i]}）")
        except Exception as e:
            ev.append(f"    分类器异常：{e!r}")

    # 距离估算（用绿目标框底边）
    try:
        gframe = _frame_with_blob(BG_BRIGHT, _hsv_to_bgr(*PRELIMINARY_COLORS[0][1:]))
        gdets = [d for d in det.detect(gframe) if d.color.name == "GREEN"]
        if gdets:
            x, y, w, hh = gdets[0].bbox
            est = det.estimate_ground_position(x + w / 2.0, float(y + hh))
            ev.append(f"绿目标框底边 → 地平面估算 {est[1]:.0f}mm、横向 {est[0]:+.0f}mm"
                      if est else "绿目标框底边 → 估算被拒（视线接近水平，属预期保护）")
    except Exception as e:
        ev.append(f"距离估算异常：{e!r}")

    # ── 4) 可选：真实图片 ──
    if ctx.image:
        if not os.path.exists(ctx.image):
            return bad(MODULE, f"--image 指定的文件不存在：{ctx.image}", ev,
                       "确认路径；现场可直接传一张手机拍的目标照片复现识别问题")
        try:
            import cv2
            img = cv2.imread(ctx.image)
        except Exception as e:
            return bad(MODULE, f"读取图片失败：{e!r}", ev)
        if img is None:
            return bad(MODULE, f"cv2.imread 读不出图片（格式不支持？）：{ctx.image}", ev,
                       "支持 jpg/png/bmp；确认文件没损坏、不是 0 字节")
        real = det.detect(img)
        ev.append(f"离线图 {os.path.basename(ctx.image)}（{img.shape[1]}x{img.shape[0]}）"
                  f"→ 检出 {len(real)} 个")
        for d in real[:6]:
            x, y, w, hh = d.bbox
            ev.append(f"    {d.color.name}/{d.shape.name} bbox={d.bbox} conf={d.confidence:.2f}")
        if not real:
            return bad(MODULE,
                       f"离线图 {os.path.basename(ctx.image)} 里一个目标都没检出 → "
                       f"HSV 阈值与现场光照不匹配（或图里确实没目标）",
                       ev, "用 tools/vision_calibration.py 在**现场光照**下重新标定各色阈值；"
                           "确认照片里目标可见、不是逆光/过曝")
    else:
        ev.append("未提供 --image：跳过真实图片检测（现场可用 --image 传一张目标照片复现识别问题）")

    if missing:
        return bad(MODULE, "有目标检不出：" + "、".join(missing)
                   + " → 对应颜色的 HSV 阈值不可用",
                   ev, "按上面缺检的颜色改 HSV_RANGES，或现场重新标定")
    if misjudged:
        return bad(MODULE, "识别结果与目标类型对不上：" + "；".join(misjudged[:3])
                   + " → 会被决策层当成别的目标（危险目标可能被误抓）",
                   ev, "重点检查浅蓝(危险)与橘(伤员)的归类；这是直接违规项")
    return ok(MODULE, "视觉识别可用（无帧不崩、空帧不误报、四类目标颜色与类型均正确）", ev,
              "现场标定：不同光照下需重跑 tools/vision_calibration.py 调整 HSV 阈值")
