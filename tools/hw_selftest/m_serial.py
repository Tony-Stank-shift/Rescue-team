"""m_serial —— 串口链路：能否打开 / PING→PONG / START→ACK"""

import time

from .framework import register, ok, bad

MODULE = "serial"
TITLE = "串口链路（打开 / PING→PONG / START→ACK）"


@register(MODULE, TITLE)
def run(ctx):
    ev = []
    sc, err = ctx.require_cli(MODULE)
    if err is not None:
        return err
    ev.append(f"串口已打开：{ctx.port} @ {ctx.baud}")

    # ── 1) PING → PONG（真正校验收到 PONG，而不是只看写入成功）──
    t0 = time.time()
    if not sc.send_ping(timeout=1.5):
        return bad(
            MODULE,
            "串口能打开但 PING 收不到 PONG → 下位机没有在响应（固件没跑/复位中/只接了单向）",
            ev + [f"PING 发出后 1.5s 内无 PONG（耗时 {time.time() - t0:.2f}s）"],
            "依次检查：①下位机是否上电 ②固件是否烧录并复位 ③TX/RX 是否交叉 "
            "④GND 是否共地 ⑤是否被别的进程占用",
        )
    ev.append(f"PING → PONG ✓（{time.time() - t0:.2f}s）")

    # ── 2) START → ACK,START ──
    sc.send_start()
    ack = sc.wait_for("ACK,START", 1.5)
    if ack is None:
        return bad(
            MODULE,
            "PING 正常但 START 收不到 ACK → 下位机可能处于 ESTOP 锁定或状态异常",
            ev + ["START 发出后 1.5s 内未收到 ACK,START"],
            "如果之前发过 ESTOP，下位机会锁定，需要给 STM32 断电重上电复位",
        )
    ev.append(f"START → {ack} ✓")

    return ok(MODULE, "串口链路正常（可打开、PING→PONG、START→ACK）", ev)
