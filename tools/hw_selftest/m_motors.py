"""m_motors —— 电机与底盘：TESTPWM 正反转 + STOP 立即停

用下位机的调试命令 `TESTPWM,l,r,ms`（开环 PWM，到点自动停并回 EVENT,TEST_DONE）。
判定依据：编码器增量方向 + 实测轮速 vL/vR 符号。
"""

import time

from .framework import register, ok, bad, skip

MODULE = "motors"
TITLE = "电机与底盘（TESTPWM 正反转 / STOP 立即停）"

PWM = 30
BURST_MS = 2000


def _sample(sc, seconds):
    """在 seconds 内采样 ODOM 帧（只取含 encL 的里程计帧）"""
    out = []
    end = time.time() + seconds
    while time.time() < end:
        f = sc.read_frame()
        if f and "encL" in f:
            out.append(f)
    return out


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
        return bad(MODULE, "START 无 ACK，无法测电机", ev, "先解决串口/下位机状态问题")

    results = {}
    for label, pwm in (("正转", PWM), ("反转", -PWM)):
        sc._send("ODOM_RESET")                 # 下位机调试命令，未在上位机封装
        sc.wait_for("ACK,ODOM_RESET", 1.0)
        sc._send(f"TESTPWM,{pwm},{pwm},{BURST_MS}")
        ack = sc.wait_for("ACK,TESTPWM", 1.0)
        ev.append(f"{label}：TESTPWM,{pwm},{pwm},{BURST_MS} → {ack}")
        frames = _sample(sc, BURST_MS / 1000.0 + 0.3)
        if len(frames) < 5:
            return bad(MODULE, f"{label}期间收不到 ODOM 帧（{len(frames)} 帧）→ 无法判定电机是否转动",
                       ev, "先跑 telemetry 模块确认数据流")
        d_enc_l = frames[-1]["encL"] - frames[0]["encL"]
        d_enc_r = frames[-1]["encR"] - frames[0]["encR"]
        v_l = sum(f["vL"] for f in frames) / len(frames)
        v_r = sum(f["vR"] for f in frames) / len(frames)
        results[label] = (d_enc_l, d_enc_r, v_l, v_r)
        ev.append(f"  ΔencL={d_enc_l:+d} ΔencR={d_enc_r:+d}  平均 vL={v_l:+.3f} vR={v_r:+.3f} m/s")
        sc.wait_for("EVENT,TEST_DONE", 1.5)

    # 正转应两轮同向为正、反转应同向为负
    dl, dr, vl, vr = results["正转"]
    if not (dl > 0 and dr > 0 and vl > 0 and vr > 0):
        return bad(MODULE, "PWM 正转时轮子没有正向转动（编码器/轮速符号不对）",
                   ev, "检查电机接线极性、TB6612 的 IN1/IN2，或左右轮接反")
    dl, dr, vl, vr = results["反转"]
    if not (dl < 0 and dr < 0 and vl < 0 and vr < 0):
        return bad(MODULE, "PWM 反转时轮子没有反向转动", ev, "同上，检查极性与接线")

    # STOP 后应迅速停住
    sc._send("STOP")
    ack = sc.wait_for("ACK,STOP", 1.0)
    ev.append(f"STOP → {ack}")
    time.sleep(0.5)
    after = _sample(sc, 0.6)
    if after:
        v_after = max(abs(after[-1]["vL"]), abs(after[-1]["vR"]))
        ev.append(f"停车后残余轮速 {v_after:.3f} m/s")
        if v_after > 0.05:
            return bad(MODULE, f"STOP 后轮子仍在转（{v_after:.3f} m/s）→ 停车不可靠",
                       ev, "检查 STOP 链路与速度环；确认下位机收到 ACK,STOP 后确实清零")

    return ok(MODULE, "电机与底盘正常（正反转方向正确、STOP 能停住）", ev)
