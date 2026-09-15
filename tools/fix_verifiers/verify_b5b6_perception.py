"""B5/B6 验证：形状（正方体 vs 长方体）、颜色（蓝 vs 浅蓝）判据不再误判。"""
import logging, sys, numpy as np, cv2
logging.disable(logging.CRITICAL)
sys.path.insert(0, "src")
from rescue_robot.perception.detection import CVDetector, HSV_RANGES
from rescue_robot.perception.classification import TargetClassifier
from rescue_robot.perception.target_types import (CompetitionPhase, TargetColor,
                                                 TargetShape, Detection)

ok=[]
def chk(n,c,extra=""):
    ok.append(bool(c)); print(("  ✅ " if c else "  ❌ ")+n+(f"  {extra}" if extra else ""))

d = CVDetector(phase=CompetitionPhase.PRELIMINARY)
clf = TargetClassifier(phase=CompetitionPhase.PRELIMINARY)
GRAY = 128

def frame_with(rects, bgr, size=(480,640)):
    f = np.full((size[0],size[1],3), GRAY, np.uint8)   # 灰底：不属于任何目标色
    for (x,y,w,h) in rects:
        cv2.rectangle(f,(x,y),(x+w,y+h),bgr,-1)
    return f

print("形状：正方体(40×40) vs 长方体(80×40=伤员) —— 旧实现两者都判 CUBE")
rects = [(100,300,40,40), (300,300,80,40)]
det = d.detect(frame_with(rects,(0,200,0)))
by_w = {}
for x in det:
    for (rx,ry,rw,rh) in rects:
        if abs(x.bbox[0]-rx) <= 3 and abs(x.bbox[1]-ry) <= 3:
            by_w[rw] = x.shape
for k in sorted(by_w): print(f"    画 {k:3d}×40 → {by_w[k].name}")
chk("40×40 → CUBE（普通物资）", by_w.get(40) == TargetShape.CUBE, str(by_w.get(40)))
chk("80×40 → CUBOID（伤员；旧实现= CUBE → 15 分目标必误判）",
    by_w.get(80) == TargetShape.CUBOID, str(by_w.get(80)))

print("颜色：浅蓝(危险目标) 与 高饱和蓝 不再互串")
f2 = frame_with([(100,300,40,40)], (230,200,150))     # 浅蓝：低饱和高亮
det2 = d.detect(f2)
hsv2 = cv2.cvtColor(np.uint8([[[230,200,150]]]), cv2.COLOR_BGR2HSV)[0][0]
print(f"    浅蓝样本 HSV={tuple(int(v) for v in hsv2)}")
chk("浅蓝物体 → LIGHT_BLUE（旧实现被判成 BLUE）",
    bool(det2) and det2[0].color == TargetColor.LIGHT_BLUE,
    det2[0].color.name if det2 else "无检测")
f3 = frame_with([(100,300,40,40)], (200,30,30))       # 高饱和蓝 BGR
det3 = d.detect(f3)
hsv3 = cv2.cvtColor(np.uint8([[[200,30,30]]]), cv2.COLOR_BGR2HSV)[0][0]
print(f"    饱和蓝样本 HSV={tuple(int(v) for v in hsv3)}")
chk("饱和蓝绝不被判成 LIGHT_BLUE(危险)",
    all(x.color != TargetColor.LIGHT_BLUE for x in det3),
    str([x.color.name for x in det3]))
lo_b, hi_b = HSV_RANGES[TargetColor.BLUE][0], HSV_RANGES[TargetColor.BLUE][1]
lo_l, hi_l = HSV_RANGES[TargetColor.LIGHT_BLUE][0], HSV_RANGES[TargetColor.LIGHT_BLUE][1]
# 说明：H/S 区间仍可能重叠（真机"浅蓝 vs 蓝"只能靠检测顺序+分类器闸门区分），
# 因此这里核查的是**防误判闸门**是否就位：
chk("分类器禁止任何颜色容差命中 DANGEROUS（闸门 1）",
    clf.classify(Detection(color=TargetColor.BLUE, shape=TargetShape.CUBE,
                           bbox=(0,0,10,10), confidence=0.9)) is None)
chk("LIGHT_BLUE 颜色本身禁止同色兜底（闸门 2）",
    clf.classify(Detection(color=TargetColor.LIGHT_BLUE, shape=TargetShape.SPHERE,
                           bbox=(0,0,10,10), confidence=0.9)) is None)
chk("检测顺序：LIGHT_BLUE 先于 BLUE",
    [c.name for c in d._colors_to_detect][:1] == ["LIGHT_BLUE"] or
    TargetColor.BLUE not in d._colors_to_detect,
    str([c.name for c in d._colors_to_detect]))

print("分类：救援目标不会被误判成危险目标（反之亦然）")
chk("BLUE+CUBE 不再容差命中浅蓝危险目标（旧实现=危险）",
    clf.classify(Detection(color=TargetColor.BLUE, shape=TargetShape.CUBE,
                           bbox=(0,0,10,10), confidence=0.9)) is None)
r = clf.classify(Detection(color=TargetColor.LIGHT_BLUE, shape=TargetShape.CUBE,
                           bbox=(0,0,10,10), confidence=0.9))
chk("LIGHT_BLUE+CUBE 精确命中危险目标", r is not None and r.info.type.name == "DANGEROUS")
chk("LIGHT_BLUE+未匹配形状 → None（不再同色任意形状兜底）",
    clf.classify(Detection(color=TargetColor.LIGHT_BLUE, shape=TargetShape.SPHERE,
                           bbox=(0,0,10,10), confidence=0.9)) is None)
inj = clf.classify(Detection(color=TargetColor.ORANGE, shape=TargetShape.CUBOID,
                             bbox=(0,0,10,10), confidence=0.9))
chk("ORANGE+CUBOID 仍正确判为伤员（未破坏既有识别）",
    inj is not None and inj.info.type.name == "INJURED")
print(f"\nB5/B6 结果: {sum(ok)}/{len(ok)} 通过")
assert all(ok)
