"""m_telemetry —— 下位机数据流：帧格式 / 频率 / 数值合理性"""

import time
from collections import Counter

from .framework import register, ok, bad, read_lines_for, parse_odom, parse_imu

MODULE = "telemetry"
TITLE = "下位机遥测（ODOM 8 字段@20Hz / IMU 10 字段@50Hz / TEL）"

#: 期望频率（Hz）与容差
EXPECT = {
    "ODOM": (20.0, 0.45),
    "IMU": (50.0, 0.45),
    "TEL": (10.0, 0.55),
}
EXPECT_FIELDS = {"ODOM": 8, "IMU": 10, "TEL": 10}


@register(MODULE, TITLE)
def run(ctx):
    ev = []
    ser, err = ctx.raw_serial()
    if err is not None:
        return err

    dur = max(2.0, float(ctx.duration_s))
    lines = read_lines_for(ser, dur)
    if not lines:
        return bad(
            MODULE,
            f"串口打开成功但 {dur:.1f}s 内收到 0 行 → 下位机没有在发数据（未运行 / 波特率不符 / 单向接线）",
            [f"嗅探 {dur:.1f}s 收到 0 行"],
            "确认下位机固件在跑（正常应持续发 ODOM/IMU/TEL）；确认两端都是 115200",
        )
    ev.append(f"嗅探 {dur:.1f}s，共收到 {len(lines)} 行")

    prefixes = Counter(l.split(",")[0] for l in lines)
    parser = ctx.parser()

    problems = []
    for name, (freq, tol) in EXPECT.items():
        n = prefixes.get(name, 0)
        if n == 0:
            problems.append(f"{name} 一帧都没有")
            continue
        measured = n / dur
        ev.append(f"{name}: {n} 帧 → {measured:.1f}Hz（期望 {freq:.0f}Hz）")
        if abs(measured - freq) / freq > tol:
            problems.append(f"{name} 频率异常：{measured:.1f}Hz（期望 {freq:.0f}Hz±{tol*100:.0f}%）")

    # 字段数 + 数值合理性
    for name in EXPECT:
        sample = next((l for l in lines if l.startswith(name + ",")), None)
        if sample is None:
            continue
        n_fields = len(sample.split(","))
        if n_fields != EXPECT_FIELDS[name]:
            problems.append(f"{name} 字段数 {n_fields}（期望 {EXPECT_FIELDS[name]}）：{sample}")

    # ODOM 能否被既有解析器解析（协议一致性）
    odom_line = next((l for l in lines if l.startswith("ODOM,")), None)
    if odom_line:
        if parse_odom(parser, odom_line) is None:
            problems.append(f"ODOM 无法被 SerialChassis.parse_frame 解析：{odom_line}")
        else:
            ev.append(f"ODOM 解析 ✓：{odom_line}")

    # IMU 能否解析 + 静态时 az≈1000mg
    imu_line = next((l for l in lines if l.startswith("IMU,")), None)
    if imu_line:
        d = parse_imu(parser, imu_line)
        if d is None:
            problems.append(f"IMU 无法被 SerialChassis.parse_imu 解析：{imu_line}")
        else:
            az = d.get("az_mg")
            ev.append(f"IMU 解析 ✓：az={az} mg（静止应≈1000mg）、temp={d.get('temp_cC')} cC")
            if az is not None and not (600 <= abs(az) <= 1400):
                problems.append(f"IMU 重力分量异常：az={az}mg（静止应在 600~1400mg）")

    if problems:
        return bad(MODULE, "下位机数据流有问题：" + "；".join(problems[:3]),
                   ev, "按上面列出的具体帧/频率问题逐项排查（多半是固件版本或接线）")
    return ok(MODULE, "遥测正常（帧格式、频率、数值合理性均通过）", ev)
