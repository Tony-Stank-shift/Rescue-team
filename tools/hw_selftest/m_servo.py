"""m_servo —— 夹爪舵机：RAISE/LOWER/HOLD/ANGLE(0~85) + 越界保护"""

import time

from .framework import register, ok, bad, skip

MODULE = "servo"
TITLE = "夹爪舵机（RAISE/LOWER/HOLD/ANGLE 0~85 + 越界保护）"

#: 实机标定端点：0°=下压、85°=抬起；ANGLE 命令最大允许 85°。
MAX_ANGLE = 85


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
        """发一条 SERVO 命令，返回**第一条** ACK/ERR 应答；1.3s 内无应答返回 None。

        ⚠️ 2026-09-17 修：旧实现是
            `sc.wait_for("ACK,SERVO", 1.0) or sc.wait_for("ERR,", 0.3)`
        而 `wait_for` 是**边读边丢弃**不匹配的行（见 `SerialChassis.wait_for`）。
        于是当固件**正确**回了 `ERR,SERVO_ANGLE` 时，第一个 wait_for 会把这个 ERR 行
        读掉并丢进垃圾桶，第二个 wait_for 在缓冲里再也找不到它 → 返回 None
        → 自检误报"越界角度 86 未被拒绝（实际 None）"。

        实测就是这么误报的：报的是"实际 **None**"（什么都没收到），而不是
        "收到了错误的 ACK" —— 这两个含义完全不同，前者一眼就该怀疑读取逻辑。
        固件侧其实是对的（`command.c` 里 86 > `SERVO_MAX_ANGLE_DEG=85` → 回 ERR）。

        现在改成**一次遍历、ACK 与 ERR 都认**，不再有"读掉再找"的窗口。
        """
        sc._send(cmd)
        time.sleep(wait)
        deadline = time.time() + 1.3
        while time.time() < deadline:
            line = sc._read_line()
            if not line:
                continue
            up = line.upper()
            if up.startswith("ACK,SERVO") or up.startswith("ERR,"):
                return line
        return None

    # 动作命令
    for cmd, want in (("SERVO,RAISE", "ACK,SERVO,RAISE"),
                      ("SERVO,LOWER", "ACK,SERVO,LOWER"),
                      ("SERVO,HOLD", "ACK,SERVO,HOLD")):
        got = send(cmd)
        ev.append(f"{cmd} → {got}")
        if got != want:
            return bad(MODULE, f"{cmd} 未按预期应答（应为 {want}，实际 {got}）",
                       ev, "确认下位机固件已实现 SERVO 动作命令；若回 ERR，检查是否已 START")

    # 标定角度命令：边界 0 / 中间 45 / 最大 85；默认 RAISE 也是 85°。
    for deg in (0, 45, MAX_ANGLE):
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
    return ok(MODULE, "夹爪舵机正常（动作命令、0~85° 标定范围、越界保护均正确）", ev)
