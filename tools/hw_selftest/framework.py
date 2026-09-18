"""
hw_selftest.framework —— 分部自检的公共框架

三条铁律：
1. **绝不抛异常**：任何模块出错都要变成 FAIL/SKIP + 人话结论，不能把整个自检带崩。
2. **结论要能指导现场动作**：区分"设备不存在 / 权限不足 / 被占用 / 软件逻辑错"。
3. **无硬件要 SKIP 而不是 FAIL**：本机（WSL）没有串口权限、没有摄像头，
   仍要能跑完整套并给出"哪些模块因环境受限未测"。
"""

import os
import sys
import time
from dataclasses import dataclass, field
from enum import Enum
from typing import Callable, Dict, List, Optional, Tuple

# 让 tools/hw_selftest/ 能被独立运行，同时能 import 到 src/
_HERE = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.dirname(os.path.dirname(_HERE))
_SRC = os.path.join(_ROOT, "src")
if _SRC not in sys.path:
    sys.path.insert(0, _SRC)


# ============================================================
# 结果模型
# ============================================================

class Status(str, Enum):
    PASS = "PASS"
    FAIL = "FAIL"
    SKIP = "SKIP"


@dataclass
class Result:
    """单个模块的测试结果"""
    module: str
    status: Status
    conclusion: str                       # 一句人话结论（现场照着这句就知道怎么办）
    evidence: List[str] = field(default_factory=list)
    hint: str = ""                        # 可选的处置建议

    @property
    def icon(self) -> str:
        return {"PASS": "✅", "FAIL": "❌", "SKIP": "⏭️"}[self.status.value]


def ok(module: str, conclusion: str, evidence: Optional[List[str]] = None,
       hint: str = "") -> Result:
    return Result(module, Status.PASS, conclusion, evidence or [], hint)


def bad(module: str, conclusion: str, evidence: Optional[List[str]] = None,
        hint: str = "") -> Result:
    return Result(module, Status.FAIL, conclusion, evidence or [], hint)


def skip(module: str, conclusion: str, evidence: Optional[List[str]] = None,
         hint: str = "") -> Result:
    return Result(module, Status.SKIP, conclusion, evidence or [], hint)


# ============================================================
# 模块注册表
# ============================================================

#: name -> (标题, run 函数)
REGISTRY: Dict[str, Tuple[str, Callable]] = {}


def register(name: str, title: str):
    """把一个模块的 run(ctx) 注册进自检表（顺序即执行顺序）"""
    def deco(fn):
        REGISTRY[name] = (title, fn)
        return fn
    return deco


#: 模块执行顺序（近似"从链路到上层"的排障顺序）
ORDER = [
    "ranging",      # 纯数学，无硬件依赖，最先给个"基本盘"
    "decision",     # 纯算法
    "navigation",   # 纯算法
    "transport",    # 纯算法（Mock 夹爪）
    "accounting",   # 纯算法：载荷台账一致性（S-40 假装载）+ 终场/场心误判（S-01/B8）
    "serial",       # 串口链路
    "telemetry",    # 下位机数据流
    "start_button", # 一键启动链路
    "servo",        # 夹爪
    "odometry",     # 里程计（要动）
    "motors",       # 电机（要动）
    "velocity",     # 速度环（要动）
    "camera",       # 摄像头
    "vision",       # 视觉识别
    "pickplace",    # 端到端：识别绿色 → 抓住 → 放进红色安全区（要动）
]


# ============================================================
# 运行上下文
# ============================================================

class Ctx:
    """共享运行上下文：串口/摄像头等资源 + 命令行开关"""

    def __init__(self,
                 port: str = "/dev/ttyS1",
                 baud: int = 115200,
                 yes_motion: bool = False,
                 mock: bool = False,
                 image: Optional[str] = None,
                 verbose: bool = False,
                 duration_s: float = 3.0):
        self.port = port
        self.baud = baud
        self.yes_motion = yes_motion
        self.mock = mock
        self.image = image
        self.verbose = verbose
        self.duration_s = duration_s

        self._chassis = None            # SerialChassis（已打开）
        self._chassis_error = None      # 打开失败时的 Result
        self._raw = None                # 裸 pyserial（仅用于嗅探原始行）
        self._parser = None             # 只用来调用 parse_frame / parse_imu

    # ---- 懒加载资源 ----

    def parser(self):
        """返回一个未打开串口的 SerialChassis，仅用于 parse_* 方法（复用协议解析）"""
        if self._parser is None:
            from rescue_robot.hardware.serial_chassis import SerialChassis
            self._parser = SerialChassis(port=self.port, baudrate=self.baud)
        return self._parser

    def open_chassis(self):
        """打开串口底盘；失败时返回 (None, Result)（含人话原因分类）"""
        if self.mock:
            # ⚠️ 2026-09-17 事故修复：`--mock` 的语义必须是**不碰硬件**。
            #    旧实现这里**照样打开真实串口**，而 `need_motion()` 又因为 mock 放行
            #    → 在连着真车的机器上跑 `--mock` 会**对着真车发指令**。
            return None, skip(
                "serial", "--mock 模式不访问真实硬件（未打开串口）",
                ["--mock 的语义 = 不碰硬件"],
                "要真机自检请去掉 --mock；会驱动执行机构的项还需 --yes-motion")
        if self._chassis is not None:
            return self._chassis, None
        if self._chassis_error is not None:
            return None, self._chassis_error
        from rescue_robot.hardware.serial_chassis import SerialChassis
        sc = SerialChassis(port=self.port, baudrate=self.baud, timeout=0.05)
        try:
            opened = sc.open()
        except Exception as e:                      # 极端情况下 open 都可能抛
            opened = False
            err = e
        else:
            err = None
        if opened and sc.is_open:
            self._chassis = sc
            return sc, None
        # 打开失败：SerialChassis.open() 内部把异常吞掉、只返回 False，
        # 这里做一次**裸探针**复现真实原因，才能把"不存在/权限不足/被占用"分开说清。
        probe_err = err
        if not os.path.exists(self.port):
            probe_err = FileNotFoundError(2, "No such file or directory", self.port)
        elif err is None:
            try:
                import serial
                s = serial.Serial(self.port, self.baud, timeout=0.05)
                s.close()
                probe_err = PermissionError(13, "无法打开（可能被占用或驱动异常）", self.port)
            except Exception as e2:
                probe_err = e2
        conclusion, hint = self._classify_serial_error(probe_err)
        # --mock（明确声明"当前没有硬件"）→ 按 SKIP 呈现；真机运行 → FAIL（那就是真故障）
        mk = skip if self.mock else bad
        self._chassis_error = mk("serial", conclusion,
                                 [f"设备={self.port} 波特率={self.baud}",
                                  f"探针结果={probe_err!r}"], hint)
        return None, self._chassis_error

    def _classify_serial_error(self, err) -> Tuple[str, str]:
        """把串口打开失败翻译成"不存在 / 权限不足 / 被占用 / 其他"四类"""
        errno = getattr(err, "errno", None)
        score_txt = f"{err}".lower()
        if errno == 13 or "permission" in score_txt:
            return (f"设备 {self.port} 存在但当前用户**无权限**",
                    f"把用户加入 dialout 组后重新登录：sudo usermod -aG dialout $USER "
                    f"（或临时 sudo chmod 666 {self.port}）")
        if errno == 2 or "no such file" in score_txt or err is None:
            return (f"设备 {self.port} **不存在**",
                    "检查下位机是否上电、USB/排线是否插好；"
                    "RDK 板载 40PIN UART1 对应 /dev/ttyS1，USB-TTL 一般是 /dev/ttyUSB0")
        if errno == 16 or "busy" in score_txt or "resource" in score_txt:
            return (f"设备 {self.port} **被其他进程占用**",
                    "先停掉正在跑的 rescue_robot 主程序（或 VSCode 调试会话）再测")
        return (f"打开 {self.port} 失败（未知原因）", f"原始异常：{err!r}")

    def raw_serial(self):
        """打开一个裸串口用于**只读嗅探**原始行（不改变协议实现）"""
        if self.mock:
            return None, skip("telemetry", "--mock 模式不访问真实硬件（未打开串口）",
                              ["--mock 的语义 = 不碰硬件"],
                              "要真机自检请去掉 --mock")
        if self._raw is not None:
            return self._raw, None
        try:
            import serial
            self._raw = serial.Serial(self.port, self.baud, timeout=0.5)
            return self._raw, None
        except Exception as e:
            conclusion, hint = self._classify_serial_error(e)
            mk = skip if self.mock else bad
            return None, mk("telemetry", conclusion, [f"异常={e!r}"], hint)

    def close(self):
        for obj, attr in ((self._chassis, "close"), (self._raw, "close")):
            if obj is None:
                continue
            try:
                getattr(obj, attr)()
            except Exception:
                pass
        self._chassis = None
        self._raw = None

    # ---- 便捷方法 ----

    def require_cli(self, module: str):
        """需要串口命令通道的模块统一入口；失败时把结论归因到**调用方模块**"""
        sc, err = self.open_chassis()
        if err is None:
            return sc, None
        return None, Result(module, err.status, err.conclusion,
                            list(err.evidence), err.hint)

    def need_motion(self, module: str) -> Optional[Result]:
        """会驱动**执行机构**（电机/舵机）的测试的统一闸门。

        ⚠️ 2026-09-17 事故修复：原来第一行是 `if self.mock: return None`
        （注释写着"Mock 模式不需要真机"→ 直接放行）。但 `open_chassis()` 当时
        **根本不看 mock**，照样打开真实 /dev/ttyS1 —— 于是 `--mock`
        **不但没保护，反而关掉了动作闸门**，让 servo / odometry / motors / velocity
        四个模块对着真车发指令。

        实测（RDK 上跑 `hw_selftest.py --mock`）：夹爪先动 → 轮子转 → 小车往前走，
        且输出 `SKIP=0`（一个都没跳过，就是铁证）。

        闸门的判据只能是"**会不会碰真实执行机构**"，与 --mock 无关。
        `--mock` 现在会让串口直接打不开（见 `open_chassis`），
        所以各模块会在更早的地方 SKIP 掉。
        """
        if not self.yes_motion:
            return skip(module,
                        "该测试会**驱动执行机构**（电机/舵机），需显式加 --yes-motion 才执行",
                        ["安全约定：默认不动作，避免在场地/台架上突然动起来"],
                        "先把轮子架起（离地空转）、并确认夹爪动作范围内无人，"
                        "再加 --yes-motion 重跑")
        return None


# ============================================================
# 小工具
# ============================================================

def elapsed_str(t0: float) -> str:
    return f"{time.time() - t0:.1f}s"


def read_lines_for(ser, seconds: float, limit: int = 4000) -> List[str]:
    """从裸串口读若干秒，按行切分返回（用于统计帧率与字段数）"""
    end = time.time() + seconds
    buf = ""
    lines: List[str] = []
    while time.time() < end and len(lines) < limit:
        try:
            chunk = ser.read(256)
        except Exception:
            break
        if not chunk:
            continue
        buf += chunk.decode("ascii", errors="replace")
        while "\n" in buf:
            line, buf = buf.split("\n", 1)
            line = line.strip()
            if line:
                lines.append(line)
    if buf.strip():
        lines.append(buf.strip())
    return lines


def parse_odom(parser, line: str) -> Optional[dict]:
    try:
        return parser.parse_frame(line)
    except Exception:
        return None


def parse_imu(parser, line: str) -> Optional[dict]:
    try:
        return parser.parse_imu(line)
    except Exception:
        return None
