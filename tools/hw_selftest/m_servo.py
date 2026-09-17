"""m_servo —— 夹爪舵机：RAISE/LOWER/HOLD/ANGLE(0~70) + 越界保护"""

import time

from .framework import register, ok, bad, skip

MODULE = "servo"
TITLE = "夹爪舵机（RAISE/LOWER/HOLD/ANGLE 0~70 + 越界保护）"

#: 下位机 servo.h：0°=下压套住、70°=抬起释放（物理范围 0~180，上层只用 0~70）
MAX_ANGLE = 70


@register(MODULE, TITLE)
def run(ctx):
    ev = []
    # ⚠️ 2026-09-17 补：本模块会**真的驱动舵机**（抬起/下压/三个角度），
    #    但原来**没有** `need_motion` 闸门 —— 于是"不加 --yes-motion"也会动夹爪。
    #    实测事故：在 RDK 上直接跑 `hw_selftest`（无 --mock / 无 --yes-motion），
    #    轮子因为其它模块有闸门而没动，**夹爪却动了**。已按同一安全约定补闸门。
    gate = ctx.need_motion(MODULE)
    if gate is not None:
        return gate
    sc, err = ctx.require_cli(MODULE)
    if err is not None:
        return err

    def send(cmd: str, wait: float = 0.5):
        sc._send(cmd)
        time.sleep(wait)
        return sc.wait_for("ACK,SERVO", 1.0) or sc.wait_for("ERR,", 0.3)

    # 动作命令
    for cmd, want in (("SERVO,RAISE", "ACK,SERVO,RAISE"),
                      ("SERVO,LOWER", "ACK,SERVO,LOWER"),
                      ("SERVO,HOLD", "ACK,SERVO,HOLD")):
        got = send(cmd)
        ev.append(f"{cmd} → {got}")
        if got != want:
            return bad(MODULE, f"{cmd} 未按预期应答（应为 {want}，实际 {got}）",
                       ev, "确认下位机固件已实现 SERVO 动作命令；若回 ERR，检查是否已 START")

    # 角度命令：边界 0 / 中间 35 / 最大 70
    for deg in (0, 35, MAX_ANGLE):
        got = send(f"SERVO,ANGLE,{deg}")
        ev.append(f"SERVO,ANGLE,{deg} → {got}")
        if got != f"ACK,SERVO,ANGLE,{deg}":
            return bad(MODULE, f"角度 {deg}° 未被接受（实际 {got}）", ev,
                       f"0~{MAX_ANGLE}° 都应被接受；若被拒，说明下位机角度域与协议不一致")

    # 越界必须拒绝（不能静默接受）
    for deg in (MAX_ANGLE + 1, 180, -1):
        got = send(f"SERVO,ANGLE,{deg}")
        ev.append(f"SERVO,ANGLE,{deg} → {got}")
        if got != "ERR,SERVO_ANGLE":
            return bad(MODULE, f"越界角度 {deg} 未被拒绝（实际 {got}）—— 存在把舵机顶到限位的风险",
                       ev, f"下位机应回复 ERR,SERVO_ANGLE（合法域 0~{MAX_ANGLE}）")

    # 收尾：抬起，避免夹爪停在压住地面的位置
    send("SERVO,RAISE")
    return ok(MODULE, "夹爪舵机正常（动作命令、0~70° 全行程、越界保护均正确）", ev)
