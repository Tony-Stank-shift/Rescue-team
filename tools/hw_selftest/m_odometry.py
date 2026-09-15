"""m_odometry —— 里程计：坐标换算正确性（纯软件）+ 实际运动符号（需动电机）

两部分：
  A. **纯软件**：`odom_to_upper` 的换算——前进应 +Y、左移应 −X、theta 应累加。
     起始位姿取 3 号出发区 (150,150) 朝 +Y，属于"任何环境都能跑"的基本盘检查。
  B. **需硬件 + --yes-motion**：实际驱动轮子，验证
     前进 → 下位机 x 增加；左转 → theta 增加；右转 → theta 减少。
"""

import math
import time

from .framework import register, ok, bad, skip

MODULE = "odometry"
TITLE = "里程计（坐标换算 + 前进/左转符号）"

SPIN_PWM = 25
BURST_MS = 1500


def _pure_math_checks(ev):
    """返回 (是否全部通过, 失败的项列表)"""
    from rescue_robot.hardware.chassis_interface import ChassisInterface

    ci = ChassisInterface()
    ci.set_start_pose(150.0, 150.0, math.pi / 2)      # 3 号出发区，朝 +Y
    cases = []

    x, y, _ = ci.odom_to_upper(0.1, 0.0, 0.0)
    cases.append(("车体前进 0.1m → 应 (150, 250)", (round(x), round(y)), (150, 250)))

    x, y, _ = ci.odom_to_upper(0.0, 0.1, 0.0)
    cases.append(("车体左移 0.1m → 应 (50, 150)", (round(x), round(y)), (50, 150)))

    _, _, th = ci.odom_to_upper(0.0, 0.0, 0.5)
    cases.append(("航向 +0.5rad → 应 π/2+0.5", round(th, 3), round(math.pi / 2 + 0.5, 3)))

    failed = []
    for label, got, want in cases:
        ev.append(f"[换算] {label}：实际 {got}" + ("" if got == want else f" ❌（期望 {want}）"))
        if got != want:
            failed.append(label)
    return (not failed), failed


@register(MODULE, TITLE)
def run(ctx):
    ev = []
    passed, failed = _pure_math_checks(ev)
    if not passed:
        return bad(MODULE, "坐标换算错误：" + "；".join(failed), ev,
                   "检查 ChassisInterface.odom_to_upper 的旋转矩阵与初始位姿注入")

    gate = ctx.need_motion(MODULE)
    if gate is not None:
        return skip(MODULE, "坐标换算通过；实际运动符号检查需要 --yes-motion（会驱动轮子）",
                    ev, "架起轮子后加 --yes-motion 重跑")

    sc, err = ctx.require_cli(MODULE)
    if err is not None:
        return err
    sc.send_start()
    if sc.wait_for("ACK,START", 1.5) is None:
        return bad(MODULE, "START 无 ACK，无法测里程计符号", ev)

    def burst(left_pwm, right_pwm, seconds):
        sc._send("ODOM_RESET")
        sc.wait_for("ACK,ODOM_RESET", 1.0)
        sc._send(f"TESTPWM,{left_pwm},{right_pwm},{int(seconds * 1000)}")
        sc.wait_for("ACK,TESTPWM", 1.0)
        frames = []
        end = time.time() + seconds + 0.2
        while time.time() < end:
            f = sc.read_frame()
            if f and "encL" in f:
                frames.append(f)
        sc.wait_for("EVENT,TEST_DONE", 1.5)
        return frames

    problems = []

    f = burst(SPIN_PWM, SPIN_PWM, BURST_MS / 1000.0)
    if len(f) < 5:
        return bad(MODULE, "前进测试期间收不到足够 ODOM 帧", ev, "先跑 telemetry 模块")
    dx = f[-1]["x_m"] - f[0]["x_m"]
    dth = f[-1]["theta_rad"] - f[0]["theta_rad"]
    ev.append(f"[前进] Δx={dx*1000:+.1f}mm  Δtheta={math.degrees(dth):+.2f}°  (应 Δx>0、Δtheta≈0)")
    if dx <= 0:
        problems.append("前进时下位机 x 没有增加")
    if abs(math.degrees(dth)) > 15:
        problems.append(f"前进时航向漂了 {math.degrees(dth):+.1f}°（应≈0）")

    f = burst(SPIN_PWM, -SPIN_PWM, BURST_MS / 1000.0)
    if len(f) >= 5:
        dth = f[-1]["theta_rad"] - f[0]["theta_rad"]
        ev.append(f"[左转] Δtheta={math.degrees(dth):+.2f}°  (应 >0)")
        if dth <= 0:
            problems.append("左转（左轮正/右轮反）时航向没有增大 → 转向符号反了")

    f = burst(-SPIN_PWM, SPIN_PWM, BURST_MS / 1000.0)
    if len(f) >= 5:
        dth = f[-1]["theta_rad"] - f[0]["theta_rad"]
        ev.append(f"[右转] Δtheta={math.degrees(dth):+.2f}°  (应 <0)")
        if dth >= 0:
            problems.append("右转时航向没有减小 → 转向符号反了")

    sc._send("STOP")
    sc.wait_for("ACK,STOP", 1.0)

    if problems:
        return bad(MODULE, "里程计符号异常：" + "；".join(problems), ev,
                   "检查左右编码器 A/B 相接线（是否反相）、左右轮定义是否互换")
    return ok(MODULE, "里程计正常（换算正确、前进/左转/右转符号一致）", ev)
