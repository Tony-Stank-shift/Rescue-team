#!/usr/bin/env python3
"""
pose_accuracy_check.py —— 定位链路精度标定（"走到某个位置量一量，识别到的物体也量一量"）

为什么需要这个工具：
    关于"看到物体却不抓"到底是里程计问题还是视觉问题，一直是靠猜。
    本工具把两个误差**分开测出来**，各给一个数：

      ① **里程计误差**：让车直走一段已知距离，把"里程计报的位移"和
         "卷尺量的实际位移"对比 → 得到**尺度误差**（mm/1000mm）与航向漂移。
      ② **视觉测距误差**：把物资放在卷尺量好的场地坐标，看视觉算出来的坐标差多少；
         再把 ① 的位姿误差扣掉，剩下的才是**视觉本身**的误差。

    有了这两个数才能决定要不要做"视觉辅助定位"：视觉误差小 → 值得做闭环/路标修正；
    视觉误差本身就很大 → 先修视觉，做修正只会把噪声引进来。

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
⚠️ 安全：本工具**会驱动电机**（`--drive`），必须显式加 `--yes-motion`。
   与 hw_selftest 同一约定：默认只读不动作。
   使用前请确认：轮子着地、前方 2m 内无人无障碍、手边能立刻断电。
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

用法示例::

    # 1) 只测视觉：车摆在卷尺量好的位置，物资也摆在量好的位置
    PYTHONPATH=src python3 tools/pose_accuracy_check.py \
        --x 1500 --y 1500 --theta 90 \
        --target-color GREEN --target-x 1500 --target-y 1200

    # 2) 测里程计尺度：直走 1000mm，停稳后用卷尺量实际走的距离回填
    PYTHONPATH=src python3 tools/pose_accuracy_check.py \
        --x 1500 --y 1500 --theta 90 --drive 1000 --actual 960 --yes-motion
"""

from __future__ import annotations

import argparse
import logging
import math
import sys
import time


def _fmt(v: float) -> str:
    return f"{v:+.1f}"


def main() -> int:
    ap = argparse.ArgumentParser(description="定位链路精度标定（里程计 + 视觉）")
    ap.add_argument("--port", default="/dev/ttyS1")
    ap.add_argument("--cam", type=int, default=0)
    ap.add_argument("--x", type=float, required=True, help="卷尺量的车心 x（mm）")
    ap.add_argument("--y", type=float, required=True, help="卷尺量的车心 y（mm）")
    ap.add_argument("--theta", type=float, required=True,
                    help="卷尺/角度尺量的车头朝向（度，+X 逆时针为正，前方=+Y=90）")
    ap.add_argument("--drive", type=float, default=0.0,
                    help="直走距离（mm）；0 = 不驱动")
    ap.add_argument("--speed", type=float, default=200.0, help="直走速度（mm/s）")
    ap.add_argument("--actual", type=float, default=None,
                    help="卷尺量的**实际**直走距离（mm），用于算里程计尺度误差")
    ap.add_argument("--target-color", default=None,
                    choices=("GREEN", "BLACK", "ORANGE", "LIGHT_BLUE", "RED", "BLUE"),
                    help="场地里那个已知坐标的物资的颜色")
    ap.add_argument("--target-x", type=float, default=None)
    ap.add_argument("--target-y", type=float, default=None)
    ap.add_argument("--color", default="red", choices=("red", "blue"),
                    help="本队安全区颜色")
    ap.add_argument("--frames", type=int, default=25,
                    help="喂给感知的帧数（要 ≥3 才能通过世界地图的确认门槛）")
    ap.add_argument("--yes-motion", action="store_true",
                    help="允许驱动电机（务必先确认场地安全）")
    args = ap.parse_args()

    logging.basicConfig(level=logging.WARNING,
                        format="%(asctime)s [%(levelname)-7s] %(name)s: %(message)s",
                        datefmt="%H:%M:%S")

    from rescue_robot.hardware.serial_chassis import SerialChassis
    from rescue_robot.hardware.camera_reader import CameraReader
    from rescue_robot.perception.perception_pipeline import PerceptionPipeline
    from rescue_robot.perception.field_elements import SafeZoneColor

    theta_rad = math.radians(args.theta)
    my_color = (SafeZoneColor.RED if args.color == "red" else SafeZoneColor.BLUE)

    ch = SerialChassis(port=args.port)
    if not ch.open():
        print("❌ 串口打不开（先确认没有 run.sh 在跑）")
        return 2
    if ch.send_command_await("PING", "PONG") is None:
        print("❌ 下位机无应答")
        ch.close()
        return 3
    # 把卷尺量的位置设为坐标基准（之后所有位姿都从这里起算）
    ch.set_start_pose(args.x, args.y, theta_rad)
    if ch.send_command_await("START", "ACK,START") is None:
        print("❌ 底盘 START 失败")
        ch.close()
        return 3

    print("=" * 74)
    print("  定位链路精度标定")
    print(f"  基准位姿（卷尺）: ({args.x:.0f}, {args.y:.0f})  θ={args.theta:+.1f}°")
    print("=" * 74)

    result: dict = {}

    # ────────────────── ① 里程计：直走一段，量实际距离 ──────────────────
    if args.drive > 0:
        if not args.yes_motion:
            print("\n① 里程计尺度：需要 --yes-motion 才驱动电机，已跳过")
        else:
            p0 = ch.read_pose() or (args.x, args.y, theta_rad)
            print(f"\n① 里程计直走：目标 {args.drive:.0f}mm @ {args.speed:.0f}mm/s")
            print(f"   起始（里程计）: ({p0[0]:.0f}, {p0[1]:.0f}) θ={math.degrees(p0[2]):+.1f}°")
            t0 = time.time()
            last_report = 0.0
            try:
                while True:
                    p = ch.read_pose()
                    if p is None:
                        time.sleep(0.02)
                        continue
                    travelled = math.hypot(p[0] - p0[0], p[1] - p0[1])
                    if travelled >= args.drive or (time.time() - t0) > 30.0:
                        break
                    ch.send_velocity(args.speed, 0.0)
                    if time.time() - last_report > 0.5:
                        last_report = time.time()
                        print(f"   …已走 {travelled:.0f}mm / {args.drive:.0f}mm")
                    time.sleep(0.02)
            finally:
                ch.send_velocity(0.0, 0.0)
                time.sleep(0.2)
                ch.send_stop()
            time.sleep(0.8)
            p1 = ch.read_pose() or p0
            odom_d = math.hypot(p1[0] - p0[0], p1[1] - p0[1])
            dtheta = math.degrees((p1[2] - p0[2] + math.pi) % (2 * math.pi) - math.pi)
            result["odom_d"] = odom_d
            result["odom_dtheta"] = dtheta
            print(f"   结束（里程计）: ({p1[0]:.0f}, {p1[1]:.0f}) θ={math.degrees(p1[2]):+.1f}°")
            print(f"   里程计报的位移 = {odom_d:.0f}mm，航向变化 = {dtheta:+.1f}°")
            if args.actual is not None:
                scale = args.actual / odom_d if odom_d > 1 else float("nan")
                result["scale"] = scale
                print(f"   卷尺实测位移 = {args.actual:.0f}mm")
                print(f"   ⇒ **里程计尺度误差 = {(scale - 1) * 100:+.1f}%**"
                      f"（走 3000mm 会偏 {(scale - 1) * 3000:+.0f}mm）")
                if abs(dtheta) > 5.0:
                    print(f"   ⇒ **直行跑偏 {dtheta:+.1f}°**（走 3000mm 横向偏 "
                          f"{3000 * math.tan(math.radians(abs(dtheta))):.0f}mm）")
            else:
                print("   （想算尺度误差：用卷尺量实际走的距离，加 --actual <mm> 重跑）")

    # ────────────────── ② 视觉：已知坐标的物资，看算出来差多少 ──────────────────
    print("\n② 视觉测距：")
    cam = CameraReader(args.cam)
    if not cam.start() or not cam.wait_first_frame(timeout=5.0):
        print("   ❌ 摄像头不可用（可能被 run.sh 占着）")
        cam.stop()
        ch.close()
        return 4
    perception = PerceptionPipeline(use_mock=False, my_safe_zone_color=my_color)
    pose_now = ch.read_pose() or (args.x, args.y, theta_rad)
    for _ in range(max(3, args.frames)):
        f = cam.get_frame()
        if f is not None:
            try:
                perception.update(frame=f, robot_position=(pose_now[0], pose_now[1]),
                                  robot_theta=pose_now[2])
            except Exception as e:
                print(f"   感知异常（已忽略）: {e}")
        time.sleep(1.0 / 15.0)

    wm = perception.world_map
    targets = list(wm.active_targets)
    print(f"   当前（里程计）位姿: ({pose_now[0]:.0f}, {pose_now[1]:.0f}) "
          f"θ={math.degrees(pose_now[2]):+.1f}°")
    print(f"   世界地图已确认目标 {len(targets)} 个，待确认候选 {wm.pending_count} 个")
    for t in targets:
        print(f"     {t.info.type.name:16} @({t.position[0]:7.0f},{t.position[1]:7.0f}) "
              f"seen={t.seen_count}")

    if args.target_color and args.target_x is not None and args.target_y is not None:
        want = [t for t in targets
                if t.info.color is not None and t.info.color.name == args.target_color]
        if not want:
            print(f"   ⚠️ 没有确认到 {args.target_color} 的目标 —— 检查：物资在视野里吗？"
                  f"光照下颜色阈值对吗？（可用 tools/live_view.py 看画面）")
        else:
            t = min(want, key=lambda t: math.hypot(t.position[0] - args.target_x,
                                                   t.position[1] - args.target_y))
            ex = t.position[0] - args.target_x
            ey = t.position[1] - args.target_y
            print(f"   对比 {args.target_color}：卷尺=({args.target_x:.0f},{args.target_y:.0f})"
                  f"  视觉=({t.position[0]:.0f},{t.position[1]:.0f})")
            print(f"   ⇒ **视觉坐标误差 = ({_fmt(ex)}, {_fmt(ey)})，"
                  f"距离 {math.hypot(ex, ey):.0f}mm**")
            # 视觉误差里混着位姿误差：把 ① 的尺度误差按"车到物资的距离"折算掉
            d_robot_to_target = math.hypot(t.position[0] - pose_now[0],
                                           t.position[1] - pose_now[1])
            if "scale" in result:
                pose_err = abs(result["scale"] - 1) * d_robot_to_target
                print(f"   其中位姿尺度误差在 {d_robot_to_target:.0f}mm 距离上贡献 ≈"
                      f"{pose_err:.0f}mm ⇒ 纯视觉误差 ≈ "
                      f"{max(0.0, math.hypot(ex, ey) - pose_err):.0f}mm")
            print("   提示：把车放到离物资不同距离（如 0.5m / 1m / 2m）各测一次，"
                  "就能看出视觉误差随距离的增长速度（理论上 ∝ 距离²）")

    cam.stop()
    ch.send_velocity(0.0, 0.0)
    ch.send_stop()
    ch.close()
    print("\n完成。把上面的数字发我，我来判断要不要做视觉闭环/路标修正。")
    return 0


if __name__ == "__main__":
    sys.exit(main())
