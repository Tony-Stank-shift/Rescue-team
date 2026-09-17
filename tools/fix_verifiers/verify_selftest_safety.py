"""
verify_selftest_safety.py —— 硬件自检的安全闸门验证（2026-09-17 事故后补）

**事故经过**：在连着真车的 RDK 上跑 `python3 tools/hw_selftest.py --mock`，
结果**夹爪先动 → 轮子转 → 小车往前走**。原因有两处叠加：

    ① `need_motion()` 第一行是 `if self.mock: return None` —— `--mock` 把
       "会驱动执行机构"的闸门**直接放行**了；
    ② `open_chassis()` / `raw_serial()` **根本不看 mock**，照样打开真实 /dev/ttyS1。

于是 `--mock` 不但没保护，反而**解锁**了 servo / odometry / motors / velocity
四个模块，让它们对着真车发指令。（当时输出 `SKIP=0` 就是铁证。）

**正确语义**：`--mock` = **不碰硬件**。本脚本验证三件事：
    A. mock 模式下**任何**路径都不得打开串口（用间谍替换 SerialChassis / serial.Serial）
    B. `need_motion()` 的判据只看 `yes_motion`，与 mock 无关
    C. 静态扫描：凡是会发**执行机构指令**的模块，都必须挂 `need_motion` 闸门
       （这条正是当初能抓住 `m_servo` 漏挂的检查）
"""
import importlib.util
import inspect
import logging
import pathlib
import sys

logging.disable(logging.CRITICAL)
sys.path.insert(0, "src")

# 直接按路径加载框架（避免依赖 tools 是个包）
_ROOT = pathlib.Path(__file__).resolve().parents[2]
sys.path.insert(0, str(_ROOT / "tools"))
from hw_selftest import framework as FW  # noqa: E402

ok = []


def chk(name, cond, extra=""):
    ok.append(bool(cond))
    print(("  ✅ " if cond else "  ❌ ") + name + (f"  {extra}" if extra else ""))


# ============================================== A. mock 不得打开串口
print("A) --mock 模式绝不允许打开串口（间谍验证，不碰真实硬件）")

_opened = {"n": 0}


class _SpyChassis:
    def __init__(self, *a, **k):
        _opened["n"] += 1
        raise AssertionError("mock 模式竟然构造了 SerialChassis！")


class _SpySerial:
    def __init__(self, *a, **k):
        _opened["n"] += 1
        raise AssertionError("mock 模式竟然构造了 serial.Serial！")


# `SerialChassis` 是在 open_chassis() **函数内** import 的，
# 所以要补在它所在的模块属性上才能被看到。
from rescue_robot.hardware import serial_chassis as _sc_mod   # noqa: E402
try:
    import serial as _serial_mod                              # noqa: E402
except Exception:
    _serial_mod = None

_orig_chassis = _sc_mod.SerialChassis
_orig_serial_cls = getattr(_serial_mod, "Serial", None) if _serial_mod else None


def _install_spies():
    _sc_mod.SerialChassis = _SpyChassis
    if _serial_mod is not None:
        _serial_mod.Serial = _SpySerial


def _restore_spies():
    _sc_mod.SerialChassis = _orig_chassis
    if _serial_mod is not None and _orig_serial_cls is not None:
        _serial_mod.Serial = _orig_serial_cls


_opened["n"] = 0
_install_spies()
try:
    ctx = FW.Ctx(port="/dev/ttyS1", mock=True)
    sc, err = ctx.open_chassis()
    chk("mock: open_chassis() 不打开串口且返回 SKIP", sc is None and err is not None,
        f"sc={sc} status={getattr(err, 'status', None)}")
    chk("mock: open_chassis() 后 _chassis 仍为空", ctx._chassis is None)

    ctx2 = FW.Ctx(port="/dev/ttyS1", mock=True)
    sc2, err2 = ctx2.require_cli("servo")
    chk("mock: require_cli() 不打开串口", sc2 is None and err2 is not None)

    ctx3 = FW.Ctx(port="/dev/ttyS1", mock=True)
    raw, err3 = ctx3.raw_serial()
    chk("mock: raw_serial() 不打开串口", raw is None and err3 is not None)

    chk("mock: 全程一次都没有构造真实串口对象（间谍计数=0）", _opened["n"] == 0,
        f"构造次数={_opened['n']}")
finally:
    _restore_spies()

# 反向对照：非 mock 时必须**真的去尝试**构造（证明间谍有效、上面不是恒真）
_opened["n"] = 0
_install_spies()
try:
    ctx4 = FW.Ctx(port="/dev/ttyS1", mock=False)
    try:
        ctx4.open_chassis()
    except AssertionError:
        pass
    chk("反向对照：非 mock 时**会**尝试构造串口对象（间谍有效）", _opened["n"] >= 1,
        f"构造次数={_opened['n']}")
finally:
    _restore_spies()

# ============================================== B. need_motion 判据
print("B) need_motion 的判据只能是 yes_motion（与 --mock 无关）")
chk("mock=True  → 仍然拦（不是 None）",
    FW.Ctx(mock=True).need_motion("m") is not None)
chk("mock=True  + yes_motion=True → 放行（这是显式确认过的）",
    FW.Ctx(mock=True, yes_motion=True).need_motion("m") is None)
chk("mock=False + yes_motion=False → 拦",
    FW.Ctx(mock=False).need_motion("m") is not None)
chk("mock=False + yes_motion=True → 放行",
    FW.Ctx(mock=False, yes_motion=True).need_motion("m") is None)
_src = inspect.getsource(FW.Ctx.need_motion)
chk("need_motion 源码里不再有 `if self.mock: return None` 短路",
    "if self.mock:\n            return None" not in _src
    and "return None                       # Mock 模式不需要真机" not in _src)

# ============================================== C. 静态扫描
print("C) 静态扫描：会发执行机构指令的模块必须挂 need_motion 闸门")
# 只认**上线命令的字面量或真正下发速度的调用**；
# `def send_velocity(...)` 是桩函数定义（m_decision 用的 _ChassisStub），要排除 ——
# 否则既会误判 m_decision，又会漏掉 m_velocity（它靠 send_velocity 持续前进，
# 正是 2026-09-17 那次"小车往前走"的来源）。
WIRE = ("TESTPWM", "ODOM_RESET", "SERVO,", '"VEL', "PWM,")


def _actuates(code: str) -> list:
    hits = [w for w in WIRE if w in code]
    if "send_velocity(" in code and "def send_velocity" not in code:
        hits.append("send_velocity(")
    return hits
_dir = pathlib.Path(__file__).resolve().parents[1] / "hw_selftest"
_missing, _checked = [], 0
for f in sorted(_dir.glob("m_*.py")):
    text = f.read_text(encoding="utf-8")
    # 去掉注释行再判断，避免"注释里提到 TESTPWM"被误判
    code = "\n".join(ln for ln in text.splitlines()
                     if not ln.lstrip().startswith("#"))
    hits = _actuates(code)
    if not hits:
        continue
    _checked += 1
    if "need_motion" not in code:
        _missing.append((f.name, hits))
chk(f"检查了 {_checked} 个会发执行机构指令的模块，全部挂了 need_motion 闸门",
    not _missing, f"漏挂：{_missing}")

# 具体点名 servo（本次事故就是它漏挂）
_servo = (_dir / "m_servo.py").read_text(encoding="utf-8")
chk("m_servo.py 已挂 need_motion（事故模块回归护栏）", "need_motion" in _servo)

print(f"\n自检安全验证结果: {sum(ok)}/{len(ok)} 通过")
assert all(ok), "硬件自检安全闸门验证未全部通过"
