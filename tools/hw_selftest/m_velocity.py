"""m_velocity —— 速度闭环：50Hz 持续下发 VEL，实测速度 vs 目标速度

⚠️ 关键：下位机有速度看门狗（300ms 保持 / 800ms 停）。
   **单发一次 VEL 会被看门狗停掉**；上位机真机必须 50Hz 持续下发。
   本模块即以 50Hz 下发，验证"上位机控制方式 + 下位机速度环"是否成立。
"""

import time

from .framework import register, ok, bad

MODULE = "velocity"
TITLE = "速度闭环（50Hz 持续 VEL → 实测 vs 目标）"

TARGET_MM_S = 200.0
TOLERANCE_PCT = 25.0        # 悬空/地面差异较大，放宽到 25%


@register(MODULE, TITLE)
def run(ctx):
    gate = ctx.need_motion(MODULE)
    if gate is not None:
        return gate
    ev = []
    sc, err = ctx.require_cli(MODULE)
    if err is not None:
        return err

    sc.send_start()
    if sc.wait_for("ACK,START", 1.5) is None:
        return bad(MODULE, "START 无 ACK，无法测速度环", ev)

    dur = max(3.0, float(ctx.duration_s))
    t0 = time.time()
    last_tx = 0.0
    frames = []
    watchdog_events = 0
    while time.time() - t0 < dur:
        now = time.time()
        if now - last_tx >= 0.02:                     # 50Hz 持续下发（与真机主循环一致）
            sc.send_velocity(TARGET_MM_S, 0.0)
            last_tx = now
        line = sc._read_line()
        if not line:
            continue
        if line.startswith("EVENT,WATCHDOG"):
            watchdog_events += 1
            continue
        f = sc.parse_frame(line)
        if f:
            frames.append(f)

    sc._send("STOP")
    sc.wait_for("ACK,STOP", 1.0)

    if len(frames) < 10:
        return bad(MODULE, f"持续发 VEL 期间只收到 {len(frames)} 帧 ODOM → 无法判定",
                   ev, "先跑 telemetry / serial 模块")
    dx_mm = (frames[-1]["x_m"] - frames[0]["x_m"]) * 1000.0
    mean_speed = dx_mm / dur
    err_pct = (mean_speed - TARGET_MM_S) / TARGET_MM_S * 100.0
    v_l = sum(f["vL"] for f in frames) / len(frames)
    v_r = sum(f["vR"] for f in frames) / len(frames)
    ev.append(f"目标 {TARGET_MM_S:.0f}mm/s，50Hz 下发 {int(dur/0.02)} 次，收到 {len(frames)} 帧 ODOM")
    ev.append(f"位移 {dx_mm:.1f}mm / {dur:.1f}s → 实测均值 {mean_speed:.1f}mm/s（偏差 {err_pct:+.1f}%）")
    ev.append(f"轮速均值 vL={v_l:.3f} vR={v_r:.3f} m/s；看门狗事件 {watchdog_events} 次")

    if watchdog_events:
        return bad(MODULE, f"持续下发 VEL 期间仍出现 {watchdog_events} 次 EVENT,WATCHDOG* "
                           f"→ 下发链路有断流（真机会中途停车）", ev,
                   "检查主循环是否被阻塞（套取/视觉/串口读等待），确保每 20ms 必发一帧 VEL")
    if abs(err_pct) > TOLERANCE_PCT:
        return bad(MODULE, f"实测速度与目标偏差 {err_pct:+.1f}%（超出 ±{TOLERANCE_PCT:.0f}%）",
                   ev, "速度环 PI 未标定（固件注释写明是 12V 架空标定值，落地需复测）；"
                       "或轮子悬空/打滑导致偏差")
    return ok(MODULE, f"速度环正常（目标 {TARGET_MM_S:.0f} → 实测 {mean_speed:.1f}mm/s，偏差 {err_pct:+.1f}%）", ev)
