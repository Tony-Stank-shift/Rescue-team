#!/usr/bin/env python3
"""
stl_gripper_probe.py —— 从 STL 实测夹爪几何（用于机构变更后重新标定代码常量）。

为什么需要它：夹爪的开口尺寸、后方阶梯板的尺寸/高度直接决定
`config.Placement` 的取值（尤其 `SLEEVE_OPENING_MM` 与 `CAPTURE_RADIUS_MM`）。
靠人拿卡尺量容易错（本项目实测中"10cm 正方形"实为 15cm×10cm），
所以留一个可复现的几何测量脚本。

用法：
    python3 tools/stl_gripper_probe.py "C:/path/to/装配体.stl"
    python3 tools/stl_gripper_probe.py part.stl --axis-up Y

输出：
    1. 三角形数 / 包围盒 / 三轴跨度
    2. 水平面（法线沿"上"轴）按高度分层 → 阶梯板尺寸与堆叠高度
    3. 三个正交投影的最大贯通空腔 → 套取开口尺寸

⚠️ 坐标系约定（本项目实测结论，STL 里没有元信息，必须人工确认）：
    装配体 `装配体3 - 夹爪-2-2.STL` 的 **+Y 是竖直向上**，
    X = 横向(宽), Z = 纵深(前后)，Y 范围 14.2~74.9mm（整机装配坐标，含安装偏置）。
    判据：按 Y 为"上"解读时，三块水平板尺寸自上而下 157×40 → 150×35 → 150×20
    **依次递减**，与机构组描述一致；换任何别的轴都得不到该规律。
"""
import argparse
import math
import struct
import sys

import numpy as np


def load_stl(path):
    """支持二进制 STL（SolidWorks 导出默认）。返回 (法线 N, 顶点 V)。"""
    raw = open(path, "rb").read()
    if len(raw) < 84:
        raise SystemExit("文件过小，不是有效的二进制 STL")
    n = struct.unpack("<I", raw[80:84])[0]
    if 84 + n * 50 != len(raw):
        raise SystemExit(
            f"不是标准二进制 STL（头部声明 {n} 面，期望 {84 + n * 50} 字节，"
            f"实际 {len(raw)}）。ASCII STL 请先转换。")
    rows, off = [], 84
    for _ in range(n):
        rows.append(struct.unpack("<12f", raw[off:off + 48]))
        off += 50
    a = np.array(rows)
    return a[:, 0:3], a[:, 3:12].reshape(-1, 3, 3)


def report_bbox(V):
    mn, mx = V.reshape(-1, 3).min(axis=0), V.reshape(-1, 3).max(axis=0)
    print("=== 包围盒（STL 单位，通常 mm）===")
    for i, ax in enumerate("XYZ"):
        print(f"  {ax}: {mn[i]:9.2f} .. {mx[i]:9.2f}   跨度 = {mx[i]-mn[i]:9.2f}")
    return mn, mx


def report_plates(N, V, up_axis, tol=2.0):
    """水平面（法线平行于"上"轴）按高度分层。"""
    nn = np.linalg.norm(N, axis=1)
    nn[nn == 0] = 1.0
    Nu = N / nn[:, None]
    cen = V.mean(axis=1)
    e1, e2 = V[:, 1] - V[:, 0], V[:, 2] - V[:, 0]
    area = np.linalg.norm(np.cross(e1, e2), axis=1) / 2.0

    hor = np.where(np.abs(Nu[:, up_axis]) > 0.98)[0]
    bands = {}
    for i in hor:
        bands.setdefault(round(float(cen[i, up_axis]), 0), []).append(i)
    groups = []
    for h in sorted(bands):
        if groups and h - groups[-1][-1] <= tol:
            groups[-1].append(h)
        else:
            groups.append([h])

    other = [a for a in range(3) if a != up_axis]
    print(f"\n=== 水平面按高度分层（'上'轴 = {'XYZ'[up_axis]}）===")
    print("注：同一高度可能同时存在**外框边沿**与**内部板**的共面三角形，会被并进同一层"
          "\n    而显得偏大；板的真实长宽请再看竖直面（法线垂直于上轴的面）。")
    print(f"{'层':>3} {'高度范围':>18} {'横跨1':>8} {'横跨2':>8} "
          f"{'面积mm2':>9}   范围1 / 范围2")
    for k, g in enumerate(groups, 1):
        ids = [i for h in g for i in bands[h]]
        P = V[ids].reshape(-1, 3)
        d1 = P[:, other[0]].max() - P[:, other[0]].min()
        d2 = P[:, other[1]].max() - P[:, other[1]].min()
        A = area[ids].sum()
        print(f"{k:>3} {min(g):7.1f}..{max(g):<7.1f} {d1:8.1f} {d2:8.1f} {A:9.0f}   "
              f"{'XYZ'[other[0]]} {P[:, other[0]].min():7.1f}..{P[:, other[0]].max():<7.1f} "
              f"{'XYZ'[other[1]]} {P[:, other[1]].min():7.1f}..{P[:, other[1]].max():<7.1f}")


def largest_empty_rect(V, ax_a, ax_b, res=110):
    """(a,b) 平面上最大"无任何三角形投影覆盖"的矩形 = 贯通开口。"""
    mn, mx = V.reshape(-1, 3).min(axis=0), V.reshape(-1, 3).max(axis=0)
    A = np.linspace(mn[ax_a], mx[ax_a], res)
    B = np.linspace(mn[ax_b], mx[ax_b], res)
    p0, p1, p2 = V[:, 0][:, [ax_a, ax_b]], V[:, 1][:, [ax_a, ax_b]], V[:, 2][:, [ax_a, ax_b]]
    d00 = ((p1 - p0) ** 2).sum(1)
    d01 = ((p1 - p0) * (p2 - p0)).sum(1)
    d11 = ((p2 - p0) ** 2).sum(1)
    den = d00 * d11 - d01 * d01
    ok = np.abs(den) > 1e-9
    p0, p1, p2, d00, d01, d11, den = (x[ok] for x in (p0, p1, p2, d00, d01, d11, den))
    grid = np.zeros((res, res), bool)
    for ia, a in enumerate(A):
        for ib in range(res):
            q = np.array([a, B[ib]])
            d20 = ((q - p0) * (p1 - p0)).sum(1)
            d21 = ((q - p0) * (p2 - p0)).sum(1)
            v = (d11 * d20 - d01 * d21) / den
            w = (d00 * d21 - d01 * d20) / den
            grid[ia, ib] = ((v >= -1e-9) & (w >= -1e-9) & (v + w <= 1 + 1e-9)).any()
    best = (0, 0, 0, 0, 0)
    for top in range(res):
        run = np.ones(res, int)
        for bot in range(top, res):
            run = run * (~grid[:, bot])
            if not run.any():
                break
            s = 0
            for e in range(res + 1):
                if e < res and run[e]:
                    continue
                if e > s and (bot - top + 1) * (e - s) > best[0]:
                    best = ((bot - top + 1) * (e - s), s, e - 1, top, bot)
                s = e + 1
    _, i0, i1, j0, j1 = best
    return A[i0], A[i1], B[j0], B[j1]


def main():
    ap = argparse.ArgumentParser(description="从 STL 实测夹爪几何")
    ap.add_argument("stl", help="STL 文件路径")
    ap.add_argument("--axis-up", default="Y", choices=list("XYZ"),
                    help="哪个轴是竖直向上（本项目实测为 Y）")
    args = ap.parse_args()

    up = "XYZ".index(args.axis_up)
    N, V = load_stl(args.stl)
    print(f"文件: {args.stl}")
    print(f"三角形 {len(N)} 个（'上'轴 = {args.axis_up}，如不符请用 --axis-up 指定）")
    report_bbox(V)
    report_plates(N, V, up)

    print("\n=== 三个正交投影的最大贯通空腔（= 套取开口候选）===")
    for a, b, name in ((0, 1, "X-Y"), (0, 2, "X-Z"), (1, 2, "Y-Z")):
        x0, x1, y0, y1 = largest_empty_rect(V, a, b)
        print(f"  {name}: {x1-x0:7.1f} × {y1-y0:7.1f} mm   "
              f"({'XYZ'[a]} {x0:.1f}..{x1:.1f}, {'XYZ'[b]} {y0:.1f}..{y1:.1f})")

    print("\n提示：开口确定后，用 config.Placement 的推导关系设标定值 ——")
    print("  capture_radius_mm ≤ drop_forward_mm + sleeve_opening_mm[1] / 2")
    print("  （实测 150×100 开口、drop_forward 70 → 上限 120；旧值 150 必然套空）")


if __name__ == "__main__":
    main()
