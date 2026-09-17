"""
verify_selftest_truthfulness.py —— 自检脚本自身不得说谎（2026-09-17）

背景：一次真机自检报了 3 个 FAIL，其中**两个是自检自己的 bug**，固件没问题：

  ① `TEL 字段数 8（期望 10）` —— `EXPECT_FIELDS["TEL"]` 是从 `IMU` 抄错的 10。
     协议与固件都是 **8 段**（`TEL` + 7 个值）。
  ② `越界角度 71 未被拒绝（实际 None）` —— 固件**正确**回了 `ERR,SERVO_ANGLE`，
     但自检助手写的是 `wait_for("ACK,SERVO",1.0) or wait_for("ERR,",0.3)`，
     而 `wait_for` **边读边丢弃**不匹配的行 → 第一个 wait_for 把 ERR 行读掉丢了，
     第二个再也找不到 → None。报出来像"固件没校验"，实际是读取逻辑吃掉了应答。

两处的共同教训：**"自检失败"必须先怀疑自检本身**，尤其当"实际值"是 None/空的时候。
本脚本把这两类错误钉住：

  A. 遥测期望字段数**从固件格式串推导**，与自检表逐项比对（杜绝手抄漂移）
  B. 舵机读取助手必须一次遍历、ACK/ERR 都认（静态 + 行为双重验证）
  C. 越界期望必须与固件实际的 `SERVO_MAX_ANGLE_DEG` 一致
"""
import logging
import pathlib
import re
import sys

logging.disable(logging.CRITICAL)
_ROOT = pathlib.Path(__file__).resolve().parents[2]
sys.path.insert(0, str(_ROOT / "src"))
sys.path.insert(0, str(_ROOT / "tools"))

ok = []


def chk(name, cond, extra=""):
    ok.append(bool(cond))
    print(("  ✅ " if cond else "  ❌ ") + name + (f"  {extra}" if extra else ""))


# ============================================================ 工具
_STR_LIT = re.compile(r'"((?:[^"\\]|\\.)*)"')


def _firmware_format_strings(src: str):
    """把 C 源码里**相邻的字符串字面量**拼成完整格式串（C 会自动拼接）。

    为什么必须拼接：`telemetry.c` 里 ODOM 的格式串跨了两个字面量
    （`"ODOM,...%ld," "..." "%s%lu.%06lu\r\n"`），不拼就会数错段数。
    """
    out = []
    for m in re.finditer(r'((?:\s*"(?:[^"\\]|\\.)*"\s*)+)', src):
        joined = "".join(_STR_LIT.findall(m.group(1)))
        if joined:
            out.append(joined)
    return out


def _field_count(fmt: str) -> int:
    """格式串 -> 上线字段（逗号段）数。

    段 = 逗号分隔的块；**一个块里可以有多个转换指示符**
    （例如 `%s%lu.%03lu` 表示"带符号的定点小数"，在线上是**一个**字段）。
    """
    body = fmt.replace("\\r", "").replace("\\n", "")
    return len(body.split(","))


# ============================================================ A. 遥测字段数
print("A) 遥测期望字段数必须与**固件格式串**一致（不靠手抄）")
_fw = (_ROOT / "firmware" / "Core" / "Src" / "telemetry.c").read_text(
    encoding="utf-8", errors="replace")
_fmts = {}
for s in _firmware_format_strings(_fw):
    for name in ("ODOM", "IMU", "TEL"):
        if s.startswith(name + ","):
            _fmts[name] = s
chk("从 telemetry.c 里提取到 ODOM/IMU/TEL 三条格式串", len(_fmts) == 3,
    f"拿到 {sorted(_fmts)}")

from hw_selftest.m_telemetry import EXPECT_FIELDS  # noqa: E402

for name in ("ODOM", "IMU", "TEL"):
    if name not in _fmts:
        chk(f"{name}: 固件里有格式串", False)
        continue
    fw_n = _field_count(_fmts[name])
    chk(f"{name}: 固件 {fw_n} 段 == 自检期望 {EXPECT_FIELDS.get(name)}",
        fw_n == EXPECT_FIELDS.get(name),
        f"固件串={_fmts[name][:60]!r}")

# 协议文档也必须与固件一致（文档是给人看的权威口径）
_proto = (_ROOT / "chassis_serial_protocol.md").read_text(encoding="utf-8")
_m = re.search(r"```text\s*\n(TEL,[^\n]*)\n```", _proto)
chk("协议文档里 TEL 的字段数与固件一致",
    _m is not None and _field_count(_m.group(1)) == _field_count(_fmts.get("TEL", "")),
    f"协议={_field_count(_m.group(1)) if _m else '未找到'} 固件={_field_count(_fmts.get('TEL',''))}")

# 反向对照：确认这个检查真的能抓到当初那个错值
chk("反向对照：把 TEL 期望改回错的 10，本检查会失败",
    _field_count(_fmts.get("TEL", "")) != 10)

# ============================================================ B. 舵机读取助手
print("B) 舵机助手必须一次遍历、ACK/ERR 都认（旧写法会吃掉 ERR 行）")
import ast          # noqa: E402
import time as _time  # noqa: E402

_servo_path = _ROOT / "tools" / "hw_selftest" / "m_servo.py"
_servo_src = _servo_path.read_text(encoding="utf-8")
_tree = ast.parse(_servo_src)
_send_fn = None
for node in ast.walk(_tree):
    if isinstance(node, ast.FunctionDef) and node.name == "send":
        _send_fn = node
        break
chk("在 m_servo.run() 里找到 send() 助手", _send_fn is not None)


def _body_without_docstring(fn):
    """取函数体的**真正代码**（去掉 docstring），避免注释/文档里引用旧代码造成误判。"""
    body = list(fn.body)
    if body and isinstance(body[0], ast.Expr) and isinstance(
            getattr(body[0], "value", None), ast.Constant):
        body = body[1:]
    return ast.unparse(ast.Module(body=body, type_ignores=[]))


_code = _body_without_docstring(_send_fn) if _send_fn else ""
chk("send() 的**代码**里不再有两段式 `wait_for(A) or wait_for(B)`",
    "wait_for" not in _code, f"代码体含 wait_for：{'wait_for' in _code}")
# 注意：ast.unparse 会把字符串统一成单引号，断言必须与引号无关
chk("send() 的代码里同时认 ACK,SERVO 与 ERR,",
    "startswith" in _code and "ACK,SERVO" in _code and "ERR," in _code,
    f"代码体={_code[:120]!r}")


class _FakeSC:
    """假串口：发完命令后只回一行 ERR,SERVO_ANGLE（模拟固件的正确拒绝）。"""

    def __init__(self, reply="ERR,SERVO_ANGLE"):
        self._q = []
        self.sent = []
        self._reply = reply

    def _send(self, cmd):
        self.sent.append(cmd)
        self._q.append(self._reply)

    def _read_line(self):
        return self._q.pop(0) if self._q else None

    def wait_for(self, prefix, timeout=0.5):
        # 复刻真实 wait_for 的"边读边丢弃"语义，用来做反向对照
        end = _time.time() + min(timeout, 0.05)
        while _time.time() < end:
            line = self._read_line()
            if line and line.upper().startswith(prefix.upper()):
                return line
        return None


_ns = {"time": _time, "sc": _FakeSC()}
exec("def send(cmd, wait=0.5):\n" + "\n".join(
    "    " + ln for ln in _code.splitlines()), _ns)          # noqa: S102
_got = _ns["send"]("SERVO,ANGLE,71")
chk("行为验证：固件回 ERR,SERVO_ANGLE 时助手能读到它（旧写法会得到 None）",
    _got == "ERR,SERVO_ANGLE", f"得到 {_got!r}")

# 正常路径也必须仍能读到 ACK
_ns2 = {"time": _time, "sc": _FakeSC("ACK,SERVO,ANGLE,35")}
exec("def send(cmd, wait=0.5):\n" + "\n".join(
    "    " + ln for ln in _code.splitlines()), _ns2)         # noqa: S102
_got2 = _ns2["send"]("SERVO,ANGLE,35")
chk("行为验证：正常路径仍能读到 ACK,SERVO,ANGLE,35",
    _got2 == "ACK,SERVO,ANGLE,35", f"得到 {_got2!r}")

# 反向对照：用旧的"两段式"写法喂同一流，必然得到 None —— 证明这条断言有意义
_f2 = _FakeSC()
_old_got = _f2.wait_for("ACK,SERVO", 1.0) or _f2.wait_for("ERR,", 0.3)
chk("反向对照：旧两段式写法在同样输入下得到 None（正是当初误报的原因）",
    _old_got is None, f"旧写法得到 {_old_got!r}")

# ============================================================ C. 角度域一致
print("C) 自检的角度域必须与固件 SERVO_MAX_ANGLE_DEG 一致")
_servo_h = (_ROOT / "firmware" / "Core" / "Inc" / "servo.h").read_text(
    encoding="utf-8", errors="replace")
_mx = re.search(r"#define\s+SERVO_MAX_ANGLE_DEG\s+(\d+)", _servo_h)
_fw_max = int(_mx.group(1)) if _mx else None
_tst_max = int(re.search(r"^MAX_ANGLE\s*=\s*(\d+)", _servo_src, re.M).group(1))
chk(f"自检 MAX_ANGLE={_tst_max} == 固件 SERVO_MAX_ANGLE_DEG={_fw_max}",
    _fw_max == _tst_max)

_cmd_src = (_ROOT / "firmware" / "Core" / "Src" / "command.c").read_text(
    encoding="utf-8", errors="replace")
chk("固件确实在 ANGLE 分支做了越界校验并回 ERR,SERVO_ANGLE",
    "ERR,SERVO_ANGLE" in _cmd_src and "SERVO_MAX_ANGLE_DEG" in _cmd_src)
# 越界分支必须在 Servo_SetAngleDeg **之前** return（否则会先动舵机再报错）
_seg = _cmd_src[_cmd_src.index('strcmp(fields[1], "ANGLE")'):]
_seg = _seg[:_seg.index("Servo_SetAngleDeg")]
chk("越界时**先 return、不驱动舵机**（所以测越界这一项本身不会动机构）",
    "return;" in _seg and "ERR,SERVO_ANGLE" in _seg)

print(f"\n自检真实性验证结果: {sum(ok)}/{len(ok)} 通过")
assert all(ok), "自检真实性验证未全部通过"
