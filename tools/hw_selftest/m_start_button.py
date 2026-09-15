"""m_start_button —— 一键启动链路：下位机按钮事件 → 上位机识别

两层检查：
  1. **软件层**（总能跑）：上位机的 read_button 是否真的认得下位机发的
     `EVENT,START_BUTTON` 事件行（历史 bug：曾按 `BUTTON,ON` 匹配，导致一键启动失灵）
  2. **硬件层**（可选，需人工按按钮）：设 `HW_SELFTEST_INTERACTIVE=1` 后，
     提示现场按下 STM32 上的启动开关，等待真实事件到达
"""

import os
import time

from .framework import register, ok, bad, skip

MODULE = "start_button"
TITLE = "一键启动链路（EVENT,START_BUTTON → AUTONOMOUS）"

EVENT_LINE = "EVENT,START_BUTTON"


@register(MODULE, TITLE)
def run(ctx):
    ev = []

    # ── 1) 软件层：read_button 能否识别官方事件行 ──
    sc = ctx.parser()                      # 未打开串口，仅用于调用方法
    orig = sc._read_line
    try:
        sc._read_line = lambda: EVENT_LINE
        got = sc.read_button()
    finally:
        sc._read_line = orig

    if got is None:
        return bad(MODULE,
                   f"上位机识别不了下位机的一键启动事件（{EVENT_LINE}）→ 现场按按钮不会启动",
                   ev + [f"喂入 {EVENT_LINE!r} 后 read_button() 返回 None"],
                   "检查 SerialChassis.read_button 的匹配前缀是否与协议一致")
    ev.append(f"软件层：read_button 正确识别 {EVENT_LINE} ✓")

    # 顺带验证协议文档与实现一致
    try:
        from rescue_robot.hardware.serial_chassis import SerialChassis
        import inspect
        src = inspect.getsource(SerialChassis.read_button)
        if "BUTTON,ON" in src:
            return bad(MODULE, "read_button 里仍有旧的 BUTTON,ON 匹配残留", ev,
                       "应只匹配 EVENT,START_BUTTON")
    except Exception:
        pass

    # ── 2) 硬件层（可选，交互）──
    if not os.environ.get("HW_SELFTEST_INTERACTIVE"):
        return ok(MODULE, "软件层正常（一键启动事件可被识别）",
                  ev + ["未做真机按键测试：设置 HW_SELFTEST_INTERACTIVE=1 可启用"],
                  "实机验证：启动主程序后按 STM32 上的启动开关，观察是否进入 AUTONOMOUS")

    sch, err = ctx.require_cli(MODULE)
    if err is not None:
        return err
    print("      ▶ 请在 15 秒内按下 STM32 上的启动开关（自锁开关拨到 ON）...")
    t0 = time.time()
    line = sch.wait_for(EVENT_LINE, 15.0)
    if line is None:
        return skip(MODULE, "15 秒内没有收到按键事件（可能没按，或按钮/接线有问题）",
                    ev + ["未收到 EVENT,START_BUTTON"],
                    "确认 START_BTN(PB9) 接线与自锁开关状态；按下时 LED 应点亮")
    ev.append(f"硬件层：收到 {line} ✓（{time.time() - t0:.1f}s）")
    return ok(MODULE, "一键启动链路贯通（按钮 → 事件 → 上位机识别）", ev)
