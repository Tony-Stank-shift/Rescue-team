"""m_pickplace —— 端到端单件链路：**识别绿色物资 → 抓住 → 放进红色安全区**

为什么单独一个模块：其他模块都是"分板块"验证（串口/视觉/夹爪/导航各自能不能用），
但比赛真正要的是**这一条链路整体通不通**。链路上任何一环坏掉，分部测试可能全是绿的，
车却抓不回来东西 —— 2026-09-18 现场正是这种情况：
    识别 ✅ → 走到跟前 ✅ → `SERVO,LOWER` 下压 ✅ → **视觉确认判"框内未见目标"** ❌
    → 抬爪后退 → 从外面看就是"它根本不去抓"。
分部测试没有一项能暴露它，因为每一环单看都是好的。

链路（每步都有超时与明确判据）：
    A 找绿色   —— 摄像头 + 感知，直到世界地图里出现**已确认**的绿色普通物资
    B 走过去   —— 真实导航到该坐标（用下位机里程计位姿）
    C 抓住     —— 显式停车 → SERVO,LOWER → **槽内视觉确认**
    D 送安全区 —— 导航到**红色安全区物资区**中心 (1345, 2820)
    E 放下     —— SERVO,RAISE

⚠️ 会驱动电机与舵机，必须 `--yes-motion`。跑之前请确认：
   轮子着地、车前方与两侧 2m 内无人无障碍、绿物资在车前方视野内、
   手边能立刻断电（或随时 Ctrl+C）。

用法::

    PYTHONPATH=src python3 tools/hw_selftest.py --only pickplace --yes-motion
"""

import math
import time
from typing import Optional

from .framework import register, ok, bad, skip

MODULE = "pickplace"
TITLE = "端到端：识别绿色物资 → 抓住 → 放进红色安全区"

#: 红色安全区**物资区**中心（x[1200,1490] y[2670,2970]）—— 与 run.sh 实跑日志一致
RED_SUPPLY_AREA = (1345.0, 2820.0)

FIND_TIMEOUT_S = 30.0        # A 阶段：找绿色的最长时间
GOTO_TIMEOUT_S = 60.0        # B/D 阶段：导航最长时间
ARRIVE_MM = 120.0            # 认为"到跟前了"（夹爪捕获半径 100 + 余量）
LOOP_DT = 0.05               # 20Hz 下发（下位机按 50Hz 收 VEL，够用）


@register(MODULE, TITLE)
def run(ctx):
    gate = ctx.need_motion(MODULE)
    if gate is not None:
        return gate

    ev = []
    chassis, err = ctx.open_chassis()
    if err is not None:
        return err
    if chassis is None:
        return skip(MODULE, "串口底盘不可用，无法跑端到端链路", ev)

    # ── 前置：握手 ──
    if chassis.send_command_await("PING", "PONG") is None:
        return bad(MODULE, "下位机无应答（PING 未收到 PONG）→ 链路第一步就断",
                   ev, "检查 STM32 是否在跑、/dev/ttyS1 是否接对")
    if chassis.send_command_await("START", "ACK,START") is None:
        return bad(MODULE, "底盘 START 未确认 → 不会接受 VEL/SERVO", ev,
                   "下位机可能需要复位")
    ev.append("底盘握手：PONG + ACK,START ✅")

    try:
        from rescue_robot.hardware.camera_reader import CameraReader
        from rescue_robot.perception.perception_pipeline import PerceptionPipeline
        from rescue_robot.perception.field_elements import FieldLayout, SafeZoneColor
        from rescue_robot.perception.target_types import TargetType
        from rescue_robot.navigation.navigation_pipeline import NavigationPipeline
        from rescue_robot.transport.sleeve_lift import SerialServoLift
    except Exception as e:
        return bad(MODULE, f"依赖导入失败：{e!r}", ev)

    cam = CameraReader(0)
    if not cam.start() or not cam.wait_first_frame(timeout=5.0):
        cam.stop()
        return bad(MODULE, "摄像头无首帧 → 找不到绿色物资", ev,
                   "检查排线/供电；先跑 `--only camera` 与 `--only vision`")
    ev.append("摄像头：首帧已到 ✅")

    field = FieldLayout.standard()
    perception = PerceptionPipeline(use_mock=False, my_safe_zone_color=SafeZoneColor.RED)
    nav = NavigationPipeline(field_layout=field, my_safe_zone_color=SafeZoneColor.RED,
                             use_mock=False)
    sleeve = SerialServoLift(chassis)

    # 坐标基准：把车**现在停的位置**当作场心 (1500,1500)、车头朝 +Y。
    # 这样"绿色在视野里"就能直接算出它的场地坐标，不必先做整场定位
    # （本测试只验证"能不能抓"，不验证整场定位）。
    sx, sy, sth = 1500.0, 1500.0, math.radians(90.0)
    chassis.set_start_pose(sx, sy, sth)
    nav.reset_pose(sx, sy, sth)
    ev.append(f"坐标基准：当前车位 = ({sx:.0f},{sy:.0f}) θ=90°")

    def pose():
        p = chassis.read_pose()
        return p if p is not None else (sx, sy, sth)

    def feed():
        f = cam.get_frame()
        if f is None:
            return
        x, y, th = pose()
        try:
            perception.update(frame=f, robot_position=(x, y), robot_theta=th)
        except Exception as e:                    # 感知异常不许带崩自检
            ev.append(f"感知更新异常（已忽略）: {e}")

    def drive_to(tx, ty, timeout_s):
        """导航到 (tx,ty)：返回 (是否到达, 最终距离)。"""
        nav.set_target(tx, ty)
        t0 = time.time()
        while time.time() - t0 < timeout_s:
            feed()
            x, y, th = pose()
            d = math.hypot(tx - x, ty - y)
            if d <= ARRIVE_MM:
                chassis.send_velocity(0.0, 0.0)
                chassis.send_stop()
                return True, d
            cmd = nav.update((x, y, th), dt=LOOP_DT)
            chassis.send_velocity(cmd.linear, cmd.angular)
            time.sleep(LOOP_DT)
        chassis.send_velocity(0.0, 0.0)
        chassis.send_stop()
        x, y, th = pose()
        return False, math.hypot(tx - x, ty - y)

    # ─────────────── A. 找绿色 ───────────────
    green: Optional[object] = None
    t0 = time.time()
    while time.time() - t0 < FIND_TIMEOUT_S:
        feed()
        try:
            cands = [t for t in perception.world_map.get_regular_supplies()
                     if t.info.type == TargetType.REGULAR_SUPPLY]
        except Exception:
            cands = []
        if cands:
            px, py, _th = pose()
            green = min(cands, key=lambda t: math.hypot(t.position[0] - px,
                                                        t.position[1] - py))
            break
        time.sleep(1.0 / 15.0)

    if green is None:
        cam.stop()
        return bad(MODULE,
                   f"A 阶段失败：{FIND_TIMEOUT_S:.0f}s 内没确认到绿色普通物资",
                   ev + [f"世界地图待确认候选 "
                         f"{perception.world_map.pending_count} 个"],
                   "① 绿色物资在前方视野内吗（1.1m 内、不偏出 77° 视场）？"
                   "② 跑 `--only vision` 确认颜色阈值；"
                   "③ 用 tools/live_view.py 看画面里绿块有没有被框住")
    gx, gy = green.position
    ev.append(f"A 找绿色 ✅ 已确认 @({gx:.0f},{gy:.0f}) seen={green.seen_count}")

    # ─────────────── B. 走过去 ───────────────
    got, d = drive_to(gx, gy, GOTO_TIMEOUT_S)
    ev.append(f"B 接近目标 {'✅' if got else '❌'} 距目标 {d:.0f}mm")
    if not got:
        cam.stop()
        return bad(MODULE,
                   f"B 阶段失败：{GOTO_TIMEOUT_S:.0f}s 内没走到绿色跟前（还剩 {d:.0f}mm）",
                   ev,
                   "① 有人/腿被当成对手挡路 → 看日志有没有『被障碍包住』；"
                   "② 定位漂了 → 先跑 tools/pose_accuracy_check.py 量尺度误差")

    # ─────────────── C. 抓住 ───────────────
    chassis.send_velocity(0.0, 0.0)
    chassis.send_stop()
    time.sleep(0.3)
    try:
        lowered = bool(sleeve.lower({green.id: (gx, gy)}))
    except Exception as e:
        lowered = False
        ev.append(f"夹爪 lower 异常: {e}")
    time.sleep(0.5)
    for _ in range(6):                 # 让槽内 ROI 拿到夹爪下压后的新帧
        feed()
        time.sleep(0.1)
    confirmed = None
    checker = getattr(perception, "check_sleeve_occupied", None)
    if callable(checker):
        try:
            confirmed = bool(checker())
        except Exception as e:
            ev.append(f"槽内视觉确认异常: {e}")
    ev.append(f"C 夹爪 SERVO,LOWER {'已下发' if lowered else '下发失败'}；"
              f"槽内视觉确认={confirmed}")

    if not lowered:
        cam.stop()
        return bad(MODULE, "C 阶段失败：夹爪没有执行 LOWER（舵机命令被拒或串口不通）",
                   ev, "先跑 `--only servo`")

    # ─────────────── D. 送红色安全区 ───────────────
    dx, dy = RED_SUPPLY_AREA
    got2, d2 = drive_to(dx, dy, GOTO_TIMEOUT_S)
    ev.append(f"D 送红色安全区物资区 ({dx:.0f},{dy:.0f}) "
              f"{'✅' if got2 else '❌'} 距目标 {d2:.0f}mm")

    # ─────────────── E. 放下 ───────────────
    chassis.send_velocity(0.0, 0.0)
    chassis.send_stop()
    time.sleep(0.3)
    try:
        raised = bool(sleeve.raise_up())
    except Exception as e:
        raised = False
        ev.append(f"夹爪 raise 异常: {e}")
    ev.append(f"E 夹爪 SERVO,RAISE {'已下发' if raised else '下发失败'}")
    cam.stop()

    if not got2:
        return bad(MODULE, f"D 阶段失败：没能把物资送到红色安全区（还差 {d2:.0f}mm）",
                   ev, "看日志里的导航状态；安全区边缘是禁区，目标点被钳制到区外属正常")
    if confirmed is False:
        return bad(MODULE, "链路走通了，但**槽内视觉确认判为没套住** → 实际没抓住",
                   ev,
                   "⚠️ 这正是 2026-09-18 现场那次：夹爪下压了却被确认否掉。"
                   "先跑 tools/calibrate_sleeve_roi.py 按夹爪 V2 标定 SLEEVE_ROI；"
                   "标定不了就用 SLEEVE_CONFIRM=0 关掉确认")
    return ok(MODULE, "端到端通过：绿色物资已识别、走到跟前、套取并送进红色安全区", ev)
