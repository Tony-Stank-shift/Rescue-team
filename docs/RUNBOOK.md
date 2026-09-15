# RUNBOOK —— 软件负责人交付件（部署 / 应急 / 性能预算 / 风险登记）

> 目标读者：**比赛现场坐在电脑前的人**。
> 目标：从零把上位机跑起来；出故障时能在 60 秒内定位到"哪个模块坏了"并做不换硬件的处置。
>
> **文档纪律**
> - 每个事实都带 `文件:行` 引用，可自己点开核对。
> - 凡是代码里没有、需要真机量出来的，一律标 **【待确认】** 或 **【待标定】**，不编数字。
> - 本文只描述**仓库现有代码的真实行为**（含已知缺陷）。缺陷属于事实，不是建议。
>
> 本文与 `docs/audit/CODE_AUDIT.md`（代码审计）、`docs/audit/COMPLIANCE_AUDIT.md`（合规审计）互补：本文是"操作手册"，它们是"问题清单"。

> ⚠️ **时效性声明**：本文撰写期间 `src/` 仍在被修改（`forbidden_zones.py` / `navigation_pipeline.py` / `load_manager.py` 有未提交改动）。文中 `文件:行` 引用以**撰写时的工作区**为准；若某行对不上，用文中给出的**函数名/常量名**去 grep（例如 `grep -n "clamp_to_safe" src/rescue_robot/navigation/forbidden_zones.py`），不要盲信行号。

---

## 0. 系统一句话架构

```
RDK X5 (Ubuntu 22.04，Python 上位机)
  ├── USB 摄像头 800W 77°  → CameraReader 后台线程 → CVDetector(HSV)
  └── /dev/ttyS1 @115200   → SerialChassis → STM32F103 (USART1 PA9/PA10) 底盘
                                  ├─ 上行: PING / START / VEL / SERVO / STOP / ESTOP
                                  └─ 下行: ODOM(20Hz) / IMU(50Hz) / ACK / EVENT / ERR / TEL
软件状态机: BOOT(自检) → DEBUG(等待一键启动) → AUTONOMOUS(全自主，不可逆)
主循环: 50Hz (每帧 20ms)  见 src/rescue_robot/states/autonomous_state.py:217
```

- 主入口：`src/rescue_robot/main.py:88`（`main()`），启动流程注释见 `main.py:11-15`。
- 状态机转移规则：`src/rescue_robot/state_machine.py:34-39`。
- 坐标/单位约定（**改任何东西前先读这三行**）：
  - 场地坐标系：左下角为原点 (0,0)，右上角 (3000,3000)，**x 向右 / y 向前 / theta 逆时针为正**（`perception/field_elements.py:122-123`）。
  - 上层内部单位一律 **mm / rad / mm/s / rad/s**（`hardware/chassis_interface.py:11-13`）。
  - 下位机约定 **m / rad / m/s**，x 前 / y 左 / theta 逆时针 → 由 `ChassisInterface.odom_to_upper()` 转换（`chassis_interface.py:65-85`）。
  - 机器人**初始朝向 = +Y = theta π/2**（`chassis_interface.py:41`、`serial_chassis.py:59`）。

---

## 1. 部署 / 启动检查单（RDK X5 从零到跑起来）

> 每一步都有 **【验证】**：看到什么才算这步成功。**上一步没过，不要做下一步。**

### 1.0 前置：物料与线束

| # | 项 | 说明 | 【验证】 |
|---|----|------|---------|
| 1 | RDK X5 已烧写 Ubuntu 22.04 | 官方镜像 | `cat /etc/os-release` 显示 `22.04` |
| 2 | 上位机 ↔ 下位机串口线 | RDK 40PIN UART1 → STM32 **USART1 (PA9=TX / PA10=RX)**；**TX/RX 必须交叉，GND 必须共地** | 见 1.2/1.3 |
| 3 | USB 摄像头 | 800W 77° 免驱，装车前**锁死自动对焦**（`HARDWARE_DEPENDENCIES.md:46`） | 见 1.6 |
| 4 | 下位机已上电并烧好固件 | 固件协议见 `chassis_serial_protocol.md` | 见 1.3 |
| 5 | 一键启动按钮 + 状态灯 | ⚠️ 赛规硬性要求「明确标识的一键启动按钮」，而 **当前物料状态：还没有**（`HARDWARE_DEPENDENCIES.md:89-95`）。软件侧按钮/灯都走**下位机**（`EVENT,START_BUTTON`），上位机不直连 GPIO（`config.py:14-27`、`main.py:72-74`） | 见 1.8 |

### 1.1 依赖安装

仓库**没有** `requirements.txt` / `pyproject.toml` / `setup.py`（已确认不存在）。依赖靠手装：

```bash
sudo apt update
sudo apt install -y python3-pip python3-opencv        # OpenCV 走 apt（RDK 上 pip 装 cv2 常失败）
pip3 install pyserial pyyaml
```

| 包 | 用途 | 代码里的引用 | 本机（开发机 WSL）实测版本 | 【待确认】 |
|----|------|-------------|--------------------------|-----------|
| `pyserial` | 串口 | `hardware/serial_chassis.py:75`（**延迟导入**，缺了也不崩，只是串口不可用） | 3.5 | RDK X5 上版本 |
| `pyyaml` | 配置 | `innovation/config_loader.py:24`（缺了降级为 JSON，YAML 会解析失败） | 5.4.1 | RDK X5 上版本 |
| `opencv` (`cv2`) | 视觉 | `perception/detection.py:181`、`hardware/camera_reader.py:64` | 5.0.0 | RDK X5 上版本 |
| Python | — | `scripts/deploy.sh:50` 硬要求 **≥ 3.10**（源码用了 `X \| None` 语法，见 `states/boot_state.py:35`） | 3.10.12 | RDK X5 自带 |

**【验证】**
```bash
python3 -c "import serial, yaml, cv2, sys; print(sys.version.split()[0], serial.__version__, yaml.__version__, cv2.__version__)"
```
成功判据：一行打印出 4 个版本号，**无 ImportError**。
> ⚠️ 若 `cv2` import 失败：**不要**以为整个程序会挂——`CVDetector.detect()` 会打印警告并返回空列表（`detection.py:183-187`），表现为「能跑但永远看不到目标」。见 §2 故障 F8。

### 1.2 串口权限（dialout 组）—— **本机实测会 Errno 13**

**这是最容易卡住的一步。** 上位机以普通用户运行，串口设备默认属主 `root:dialout`、权限 `crw-rw----`，不在 `dialout` 组就是 `PermissionError: [Errno 13] Permission denied`。

```bash
# 1) 把自己加进 dialout 组
sudo usermod -aG dialout $USER
# 2) 必须重新登录（新 shell/重开 SSH）才生效——sudo 不解决问题，因为组在登录时确定
id | tr ',' '\n' | grep dialout
```

**【验证】**
```bash
id -nG | grep -qw dialout && echo "组OK" || echo "组缺失"
ls -l /dev/ttyS1
python3 -c "import serial; s=serial.Serial('/dev/ttyS1',115200,timeout=0.2); print('OPEN OK'); s.close()"
```
成功判据：`组OK` + `ls` 显示 `dialout` + 打印 `OPEN OK`。

失败分诊（自检程序会自动分这四类，见 `tools/hw_selftest/framework.py:175-190`）：

| 报错 | 含义 | 处置 |
|------|------|------|
| `Errno 13 Permission denied` | 存在但**无权限** | 上面两步；临时应急 `sudo chmod 666 /dev/ttyS1`（重启失效，只当兜底） |
| `FileNotFoundError: /dev/ttyS1` | 设备**不存在** | 设备树没启用该 UART / 线没插 / 下位机没上电。见 1.3 确权 |
| `Errno 16 Device or resource busy` | **被占用** | `sudo lsof /dev/ttyS1`、`sudo fuser -v /dev/ttyS1`；先停掉在跑的 `rescue_robot.main`、VSCode 串口监视器、`screen/minicom` |
| 打开成功但收不到任何数据 | 权限对、**波特率/接线**错 | 见 §2 故障 F1/F2 |

### 1.3 设备确权：`/dev/ttyS1` 到底是不是下位机

`CHASSIS_PORT` 的**代码默认值是 `/dev/ttyUSB0`，不是 `/dev/ttyS1`**（`main.py:142`）。
→ **在 RDK 上必须显式导出 `CHASSIS_PORT=/dev/ttyS1`**，否则会去开一个不存在的 USB 设备。
（代码注释里 RDK 端口写的是 `/dev/ttyS0`，`serial_chassis.py:27`，**与本题给定的 `/dev/ttyS1` 不一致 → 【待确认】RDK X5 40PIN UART1 的实际设备名**。）

确权三步，**不要跳**：

```bash
# ① 列出可用串口
ls -l /dev/ttyS* /dev/ttyUSB* /dev/ttyACM* 2>/dev/null

# ② 物理确权：拔掉/插上下位机那条线，看哪个节点消失了 → 那个就是它
#    （板载 UART 不会因拔线消失，所以 RDK 上更可靠的判据是 ③）

# ③ 数据确权（最快）：用仓库自带的解析器读 3 秒，看有没有合法的 ODOM/IMU 行
PYTHONPATH=src python3 - <<'EOF'
import time
from rescue_robot.hardware.serial_chassis import SerialChassis
sc = SerialChassis(port="/dev/ttyS1", baudrate=115200, timeout=0.2)
print("open:", sc.open(), "is_open:", sc.is_open)
n_odom = n_imu = 0
t0 = time.time()
while time.time() - t0 < 3.0:
    t = sc._read_line()
    if not t:
        continue
    if t.upper().startswith("ODOM") and sc.parse_frame(t): n_odom += 1
    elif t.upper().startswith("IMU") and sc.parse_imu(t): n_imu += 1
sc.close()
print(f"3 秒内 ODOM={n_odom}  IMU={n_imu}")
EOF
```

**【验证】** 成功判据：`open: True`，且 `ODOM` 帧数 ≈ **60**（20Hz×3s，`serial_chassis.py:18`）、`IMU` 帧数 ≈ **150**（50Hz×3s，`serial_chassis.py:19`）。
若 0 帧 → 见 §2 F1/F2。若帧数只有一半 → 波特率或丢包，见 §2 F2。

### 1.4 代码上传（仓库没有网络时的两条路）

**方式 A：`scripts/deploy.sh`（rsync / scp）**
```bash
TARGET=<RDK-IP> TARGET_USER=sunrise TARGET_PATH=/home/sunrise/rescue-robot \
  bash scripts/deploy.sh --target $TARGET
```
- ⚠️ **三个坑（必须先改脚本或绕开）**：
  1. 脚本默认目标是**树莓派**：`TARGET=raspberrypi.local` / `USER=pi` / `PATH=/home/pi/rescue-robot`（`scripts/deploy.sh:11-13`）。
  2. `rsync -avz --delete` 带 `--delete`（`scripts/deploy.sh:82`）→ **会删掉目标机上 `src/` 里多余的文件**。若 RDK 上有手工改动，先备份。
  3. 它在远端 `sudo systemctl restart rescue-robot`（`scripts/deploy.sh:103`）——**这个 systemd 服务在本仓库里不存在**（已确认无 `*.service` 文件）。所以那条路一定会 fallback 到提示「手动启动」。

**方式 B：base64 单文件/打包传输（无 rsync、无 scp 场景）**

在开发机：
```bash
tar czf /tmp/rescue.tgz --exclude='__pycache__' --exclude='*.pyc' --exclude='.git' \
    -C /home/ony_uang/Rescue-team src config scripts tools
base64 -w0 /tmp/rescue.tgz > /tmp/rescue.b64
```
把 `rescue.b64` 用**任何可用通道**（U 盘 / 聊天工具 / 串口终端粘贴）送到 RDK，然后：
```bash
base64 -d /tmp/rescue.b64 > /tmp/rescue.tgz
mkdir -p ~/rescue-robot && tar xzf /tmp/rescue.tgz -C ~/rescue-robot
```

**【验证】**
```bash
cd ~/rescue-robot && PYTHONPATH=src python3 -c "import rescue_robot.main; print('IMPORT OK')"
ls config/robot.default.yaml    # 【关键】配置是相对 CWD 找的，见 1.5
```
成功判据：打印 `IMPORT OK`，且 `config/robot.default.yaml` 存在。

### 1.5 启动命令与环境变量

**必须在仓库根目录启动**：配置路径 `config/robot.default.yaml` 是**相对当前工作目录**解析的（`main.py:105` + `innovation/config_loader.py:259-263` 用 `os.getcwd()`）。在别处启动 → 配置加载失败 → 打一条 warning 后**用默认参数继续跑**（`main.py:108-109`），非常隐蔽。

```bash
cd ~/rescue-robot
export RUN_MODE=real              # 默认是 mock！不设这个，程序不会碰真硬件
export CHASSIS_PORT=/dev/ttyS1    # 默认 /dev/ttyUSB0 —— RDK 上必须改
export CAM_INDEX=0                # 摄像头 index；注意自检与实际采集默认值不一致，见下表
export SKIP_CAMERA_CHECK=1        # 台架调试（没接摄像头）时跳过摄像头自检
export CAM_WARMUP_S=3.0           # 摄像头首帧预热超时
PYTHONPATH=src python3 -m rescue_robot.main
```

**环境变量全表**（逐个核对过代码）：

| 变量 | 代码位置 | 默认值 | 作用 | 备注 |
|------|---------|--------|------|------|
| `RUN_MODE` | `main.py:98` | `mock` | `mock`/`real` | **默认 mock**——忘了设就会"能跑但车不动"，见 §2 F4 |
| `CHASSIS_PORT` | `main.py:142` | `/dev/ttyUSB0` | 上位机↔下位机串口 | RDK 上要设 `/dev/ttyS1` |
| `CAM_INDEX` | `main.py:152`（实际采集）<br>`main.py:75`（自检） | 实际采集 **0**<br>自检 **1** | 摄像头序号 | ⚠️ **同一个变量两处默认值不同**（0 vs 1）。若只设一次，自检和采集可能开的是**两个不同设备** |
| `CAM_WARMUP_S` | `main.py:156` | `3.0` | 首帧超时；超时 → 摄像头判不可用 → 感知降级 Mock | 见 `main.py:160-167` |
| `SKIP_CAMERA_CHECK` | `main.py:162`（`system_check.py:162`） | 未设（=检查） | `1/true/yes` → 跳过摄像头自检项 | 只影响**自检**，不影响采集 |
| `TEAM_COLOR` | `main.py:129` | `red` | 本队安全区颜色 | 抽签后必改；非 `red` 一律当 `blue` |

**启动后的正常日志（逐行对照，对不上就是有问题）**
```
🔧 运行模式: REAL (真实硬件)                         ← main.py:70
已加载 config/robot.default.yaml                     ← main.py:107
🔵 进入 BOOT 状态 — 系统自检中...                     ← boot_state.py:46
  [PASS] 摄像头: 正常 (...ms)                        ← system_check.py:333
  [PASS] IMU: 正常 (...ms)
  [PASS] 电机 #1/#2: 正常 (...ms)
  [PASS] 电池电压: 未接电压传感器（未知），跳过该项       ← system_check.py:301
✅ 系统自检通过！                                     ← system_check.py:257
自检通过，转入 DEBUG 状态                             ← boot_state.py:53
🟢 进入 DEBUG 状态 — 等待一键启动...                  ← debug_state.py:52
串口已打开: /dev/ttyS1 @ 115200                       ← serial_chassis.py:84
摄像头 0 就绪（首帧已到）                              ← main.py:158
状态机流程: BOOT → (自检) → DEBUG → (一键启动) → AUTONOMOUS
```
- **自检 5 项全 PASS** 才算 BOOT 通过（关键项任一 FAIL → 直接 ERROR，`boot_state.py:52-61`）。
- 自检大概耗时：Mock 约 0.6s；真机受摄像头探测影响，`SKIP_CAMERA_CHECK=1` 时约 0.1s，否则最多 2s（`system_check.py:172`）。

### 1.6 摄像头确认

```bash
ls -l /dev/video*                                    # 看有哪些 video 节点
PYTHONPATH=src python3 tools/vision_quick.py         # 仓库自带的视觉快检脚本
PYTHONPATH=src python3 tools/vision_test.py          # 完整视觉测试
PYTHONPATH=src python3 tools/vision_calibration.py   # 标定（倾角/ROI）
```
**【验证】** `/dev/video0` 存在；脚本能出图（`tools/` 下三个视觉脚本均为可独立运行入口）。

⚠️ **摄像头 index 是最常见的坑**：RDK 上除了 USB 摄像头，还可能有自带 MIPI/虚拟节点。用 `--list` 或逐个 index 试：

```bash
for i in 0 1 2; do CAM_INDEX=$i PYTHONPATH=src python3 - <<EOF
import os
from rescue_robot.hardware.camera_reader import CameraReader
c = CameraReader(int(os.environ["CAM_INDEX"]))
print(f"index={os.environ['CAM_INDEX']} open={c.start()}", end=" ")
print("first_frame=", c.wait_first_frame(timeout=2.0)); c.stop()
EOF
done
```

### 1.7 一键启动链路（进 AUTONOMOUS 的唯一途径）

代码路径（**这条链路是分开的两段，必须都通**）：

1. 下位机按钮按下 → F103 发 `EVENT,START_BUTTON`（`serial_chassis.py:257-258`）。
2. 上位机主线程 0.5s 轮询一次读串口（`main.py:247-257`）→ `chassis.read_button()` 命中 → `sm.one_key_start()`。
3. `one_key_start()` 置锁 + `DEBUG → AUTONOMOUS`（`state_machine.py:184-203`，不可逆）。
4. `AutonomousState.on_enter()`：示波灯快闪 → 锁外部输入 → `start_match()` → `start_match()` 串口握手（PING→PONG / START→ACK,START）→ 等 `POST_START_DELAY_MS=1000ms` → 起主循环线程（`autonomous_state.py:137-177`）。

**【验证】** 按下按钮后日志出现：
```
收到一键启动按钮事件 (EVENT,START_BUTTON)      ← main.py:255
⚡ 一键启动触发！进入全自主运行模式（外部输入已锁定）  ← state_machine.py:202
🔴 进入 AUTONOMOUS 状态 — 全自主运行
底盘连接+启动: ✅ 成功                          ← autonomous_state.py:158
主循环已启动                                    ← autonomous_state.py:177
```
> **调试期旁路**：`RUN_MODE=real` 下 `button` 仍然是 `MockButton`（`main.py:73`）——它不是真硬件按钮，而是**键盘模拟**：在跑程序的终端里输入 `l`+回车 = 长按 = 一键启动，`s` = 短按，`q` = 退出（`hardware/button.py:94-113`）。无头/无 TTY 环境下 `input()` 抛 `EOFError`，被捕获后 sleep 0.5s（`button.py:109-110`），**不会崩**，但也**不会触发**。所以**没有下位机按钮时必须走串口事件**。

### 1.8 赛前 14 项硬检查（开赛前 5 分钟照着打勾）

| ☐ | 检查 | 命令 / 判据 |
|---|------|-------------|
| ☐ | 仓库根目录 | `pwd` == `~/rescue-robot` |
| ☐ | 配置存在 | `ls config/robot.default.yaml` |
| ☐ | 队伍颜色 | `grep my_color config/field.default.yaml` + `echo $TEAM_COLOR` 一致 |
| ☐ | 依赖 | 1.1 的四版本一行打印成功 |
| ☐ | 串口权限 | `id -nG \| grep -qw dialout` |
| ☐ | 串口未被占用 | `sudo fuser -v /dev/ttyS1` 无输出 |
| ☐ | 下位机遥测 | 1.3 的 3 秒脚本 ODOM≈60 / IMU≈150 |
| ☐ | PING→PONG | 1.3 成功 + `PYTHONPATH=src python3 tools/hw_selftest.py --only serial` |
| ☐ | 摄像头 | `ls /dev/video*` + `tools/vision_quick.py` 出图 |
| ☐ | 一键启动按钮存在且标识清楚 | 赛规硬性要求（`HARDWARE_DEPENDENCIES.md:93`） |
| ☐ | 电量 | ≥ 11.0V（阈值 `config.py:60`）；⚠️ **软件读不到电压**，真机自检该项直接放行（`system_check.py:300-302`），**必须人工量** |
| ☐ | 全部电池/线束固定 | 防赛场脱落 |
| ☐ | 整机重量/尺寸 | ≤1.5kg、≤300×300mm、高≤200mm（`README.md:74-82`） |

---

## 2. 现场应急手册（故障现象 → 快速判断 → 处置动作）

> 处置原则：**先降级/跳过/改参数，最后才动硬件**。
> 所有处置都不需要换件；需要重启的顺序统一是：**Ctrl+C → 等串口释放 → 重新启动**（`main.py:269-292` 的 finally 会 `close()` 串口与摄像头线程，正常退出不会留下占用）。

### 2.1 串口类

| # | 故障现象 | 快速判断 | 处置动作（不换硬件） |
|---|---------|---------|---------------------|
| **F1** | 串口打不开：**不存在**<br>`FileNotFoundError: /dev/ttyS1` | `ls -l /dev/ttyS1` 无此文件 | ① 确认下位机已上电（F103 的 3.3V 灯亮）；② 确认 40PIN 排线/USB-TTL 插好；③ 换端口名试：`CHASSIS_PORT=/dev/ttyS0` / `/dev/ttyUSB0` / `/dev/ttyAMA0`（RDK 板载 UART1 实际名**【待确认】**）；④ 确认设备树里该 UART overlay 已启用；⑤ 台架无下位机时 → 用 `RUN_MODE=mock` 先把软件跑通（感知/决策/导航逻辑仍可验证） |
| **F2** | 串口打不开：**权限不足**<br>`PermissionError: [Errno 13]` | `id -nG \| grep dialout` 空 | ① `sudo usermod -aG dialout $USER` **然后重新登录**（关键）；② 应急：`sudo chmod 666 /dev/ttyS1`；③ 别用 `sudo python3` 绕——会让生成的文件属主变 root，后面更麻烦 |
| **F3** | 串口打不开：**被占用**<br>`Errno 16 Device or resource busy` | `sudo lsof /dev/ttyS1` 有进程 | ① `pkill -f rescue_robot.main`；② 关掉 VSCode 串口监视器 / `screen` / `minicom`；③ 确认没有第二个终端在跑同一程序；④ 重试 |
| **F4** | 能收数据但**车不动**（最隐蔽的一个） | 日志首行是 `🔧 运行模式: MOCK` | **就是 `RUN_MODE` 没设**！`RUN_MODE` 默认 `mock`（`main.py:98`），mock 下 `chassis=None` → 根本不发 `VEL`。`export RUN_MODE=real` 重启 |
| **F5** | `PING` 无 `PONG` | 日志 `PING 未收到 PONG，连通性检查失败`（`serial_chassis.py:160`） | ① 说明**收得到但发不出/对不出**：先按 F1.3 确认能收到 ODOM；② 检查 TX/RX 是否交叉（PA9=TX 接下位机 RX）；③ 确认双方 115200 8N1 无流控（`serial_chassis.py:8`）；④ 波特率不符会看到乱码行 → 试 9600/57600 排除；⑤ **该项只影响 `start_match()` 的返回值和自检电机项**，主循环仍会继续跑（`autonomous_state.py:157-160` 只记日志），所以别因为它误判"整机挂了" |
| **F6** | **收不到遥测**（`read_pose()` 一直 None） | 1.3 脚本 ODOM=0；或日志里 `_pose` 卡在初始 (150,150,π/2) | ① 先按 F1/F2/F3 排除打开问题；② 若打开成功但无数据 → **TX/RX 接反**或**没共地**（最常见）；③ 检查下位机是否在发（用串口助手裸看）；④ 降级：无遥测时上位机**不会崩**，会一直用初始位姿——但导航会彻底失准，属于必须修好的项，不能带病比赛 |
| **F7** | 遥测**帧率不对**（ODOM 少于 20Hz / IMU 少于 50Hz） | 1.3 脚本读数明显偏小 | ① 波特率不匹配 → 换波特率复测；② 若只是 ODOM 慢：**这是代码的正常行为**——主循环每帧只读**一行**（`autonomous_state.py:241` → `read_pose()` → `_read_line()` 单次读），串口是 IMU/ODOM 混流，实际位姿更新率被"每帧一行"限制。**不是故障**，但要知道位姿延迟见 §3 |

### 2.2 运动/机构类

| # | 故障现象 | 快速判断 | 处置动作（不换硬件） |
|---|---------|---------|---------------------|
| **F8** | 能收数据、能发 VEL，但**车不动** | `sudo cat /dev/ttyS1` 看不到 `VEL,...`；或下位机没响应 | ① 确认 `RUN_MODE=real`（F4）；② 确认到了 AUTONOMOUS（只有主循环发 VEL）；③ 看是否卡在 `STOP`/`ESTOP` 状态（急停锁定需重启）；④ 用 `tools/hw_selftest.py --only velocity` 直接测闭环；⑤ **速度符号存疑**：上层 `VelocityCommand.linear` 是 mm/s，`velocity_to_command()` 取整下发（`chassis_interface.py:95-104`）。若发现"发正速度车后退"→ 符号约定与下位机不一致，**【待确认】**（见 §4 R6） |
| **F9** | **速度明显不对**（太快/太慢/单位像差了 1000 倍） | 实测 vs `VEL` 值 | ① 单位链：上层 mm/s → `VEL,v_mm_s`（mm/s）→ 下位机自行换算轮速（`HARDWARE_DEPENDENCIES.md:148`）。若差了 1000 倍，就是 mm/m 混用；② 上限 850mm/s（`config/robot.default.yaml:39`）；③ 调 `robot.motors.max_speed_mm_s` 或 PID（`yaml:31-38`）后**重启**（YAML 只在启动时读，`main.py:105`，**没有热加载**——虽然 YAML 注释写了"热加载生效"，实际未接，见 §4 R9） |
| **F10** | **舵机不动** | 日志无 `SERVO` 相关行 | ① 确认 `chassis` 非空（真机模式下会自动注入 `SerialServoLift`，`main.py:183-188`）；② 手工发一条：`tools/hw_selftest.py --only servo`；③ 确认舵机信号线在 STM32 **PB6**（`config.py:21`）；④ 下位机固件是否支持 `SERVO,RAISE/LOWER/HOLD`（协议 v1.1，`serial_chassis.py:13`） |
| **F11** | 舵机**方向反** | 下压变成抬起 | 角度语义：**0° = 下压套住，70° = 抬起释放**（`sleeve_lift.py:185-199`，对齐下位机 `servo.h`）。若实测相反 → 下位机角度映射反了，**改下位机**（上位机侧改会破坏 `place_ramp()` 的 0→70 递增序列，`sleeve_lift.py:423-440`） |
| **F12** | **套取总是失败**（反复抬爪后退重试） | 日志 `视觉确认：U 型槽内未见目标 → 判为套取失败，将抬爪后退重试`（`transport_pipeline.py:329`） | ① **首选处置：关掉视觉确认**——`config.py:109` `Camera.SLEEVE_CONFIRM = False`（或直接设 `SLEEVE_ROI` 为非法值）；② 或者重新标定 `Camera.SLEEVE_ROI`（归一化 x1,y1,x2,y2，`config.py:107`）——把目标真的放进槽里再看检测框中心落在哪；③ 有**自动失效保护**：连续 5 次确认失败会自动关闭确认（`transport_pipeline.py:114-115, 324-328`），所以最多白试 5 次；④ 连续 3 次套取失败会放弃本趟（`transport_pipeline.py:173, 345-355`） |

### 2.3 视觉类

| # | 故障现象 | 快速判断 | 处置动作（不换硬件） |
|---|---------|---------|---------------------|
| **F13** | **摄像头打不开 / 画面黑** | 日志 `摄像头 N 打开失败，感知将退化为 Mock`（`main.py:166`）或 `... 3s 内未出帧`（`main.py:161`） | ① 换 index：`for i in 0 1 2; do CAM_INDEX=$i ...`（1.6）；② 加大预热：`CAM_WARMUP_S=8`（USB 摄像头冷启动慢）；③ 台架没摄像头：`SKIP_CAMERA_CHECK=1` 让自检放行，**但要清楚后果**——感知会降级 Mock（`main.py:173-176`），机器人会在**假目标**上跑来跑去；④ **真机比赛绝不允许带病跑**：无摄像头 = 没有视觉定位/识别 |
| **F14** | **视觉识别不到目标** | `tools/vision_quick.py` 出图但无框 | ① 光照：HSV 阈值是硬编码的（`detection.py:33-46`），现场黄光/白光会让 H 漂移 → 用 `tools/vision_calibration.py` 重标；② `min_contour_area=200` 过滤太小目标（`detection.py:146`）→ 目标远/小时会被滤掉；③ 目标颜色与配置不符：颜色↔类型映射在 `config/robot.default.yaml:70-74`（**决赛现场可能变更**，`README` 也提示）；④ 分辨率不匹配会自动用真实帧尺寸纠正（`detection.py:195-202`） |
| **F15** | **识别到假目标**（跑去套一个不存在的东西） | 日志频繁新建目标；决策反复扑空 | ① 已有**纵深过滤**：新目标需连续 `MIN_SEEN_COUNT=3` 帧命中才成立（`world_map.py:87-89, 332-366`）——所以"单帧误检"理论上进不来；② 若仍误判 → 缩小 HSV 范围（`detection.py:33-46`），或提高 `min_confidence`（`yaml:78` 是 0.6，注意 **`_calc_confidence` 的口径**：面积≥2000px² 且顶点 3~20 才 0.5+，见 `detection.py:317-322`——**阈值 0.6 意味着置信度上限只有 (1.0+0.5)/2=0.75**，实际可用区间很窄）；③ 应急：把 `config/robot.default.yaml` 里 `perception.detection.association_threshold_mm` 调小（默认 100），让误检更难跟丢重关联 |
| **F16** | **套取 ROI 指错地方**（槽里明明有目标却判空） | 见 F12 | 同 F12 ②；标定 ROI 时可先把 `SLEEVE_CONFIRM=False` 跑通流程，再回来标 |

### 2.4 状态机/流程类

| # | 故障现象 | 快速判断 | 处置动作（不换硬件） |
|---|---------|---------|---------------------|
| **F17** | **进不了 AUTONOMOUS** | 一直停在 DEBUG；或直接 ERROR | 分支判断：<br>① 停在 DEBUG + 按按钮无反应 → 按 F1.7 检查 `EVENT,START_BUTTON` 是否到上位机（**这是最常见的**：按钮/固件/串口任一断都不行）；<br>② 进了 ERROR → 看自检报告哪项 FAIL（`system_check.py:75-80` 会打印逐项）；<br>③ **摄像头是关键项**，台架必设 `SKIP_CAMERA_CHECK=1`（`system_check.py:161-164`）；<br>④ **电池电压**：读不到时放行（`system_check.py:300-302`），真机没接过电压采样也不会卡；<br>⑤ 重复触发被忽略：`_one_key_started` 置位后不可再启动（`state_machine.py:193-195`）→ **必须重启程序** |
| **F18** | **进 AUTONOMOUS 后立刻又掉出去 / 卡死** | 日志 `主循环异常:` 反复刷 | 主循环**单帧异常不会退出循环**（`autonomous_state.py:223-227` 捕获后继续），所以"反复异常"= 某模块每帧都抛。看 `exc_info` 的栈顶模块名定位；常见：感知喂 `frame=None`（已修，`detection.py:189-193` 返回空）、串口异常（`serial_chassis.py:177-179` 只告警返回 None） |
| **F19** | **比赛中间卡死**（车停住，日志不动） | 看最后一条日志 + 主循环计数 | ① 若卡在套取：**这是已知阻塞**——套取全程在**主循环线程里 `time.sleep`**（`transport_pipeline.py:303-355` → `sleeve_lift.py:353-389`），最长路径 = `wait_for(0.5) + MOVE_TIME_S(0.4)`，失败重试再 ×3 → **单帧可达 ~1.4s**，期间不发 `VEL`。下位机靠速度看门狗兜底停车（`autonomous_state.py:327-330`）。② 若日志停在 `place_ramp`：也是 `sleep`（`sleeve_lift.py:423-440`，4 步 ×0.1s）。③ 处置：**等**（会自动恢复），或 `Ctrl+C` 安全退出（`main.py:224-230`）。④ 若彻底无响应 → 硬件复位（先断电机电源，再断主控） |
| **F20** | **车原地打转/不动，看门狗反复触发** | 日志 `🔍 探索模式：Xs 无位移` / `🛟 保命绕圈：Xs 无位移`（`autonomous_state.py:449, 445`） | 分级看门狗判据是**下位机里程计实测位移**，不是"有没有下发速度"：一个窗口内累计位移 ≥40mm 才算在动，单帧 <1mm 视为抖动滤掉（`autonomous_state.py:59-60, 389-415`）；>10s 探索、>13s 保命绕圈、>15s **不淘汰**只保持运动（`autonomous_state.py:52-54, 429-449`）。⚠️ **车在转但里程计不动 = 仍会触发**（打滑、编码器没接）。处置：① 先确认里程计真的有数（§1.3）；② 确认世界地图里真有目标（无目标 → 决策 `DONE`，`decision_engine.py:242-245`）；③ 若导航 `BLOCKED`（`navigation_pipeline.py:241-245`）→ 看目标点是否落在代价 255 的格子上（`forbidden_zones.py:235-240`） |

### 2.5 应急手册速查（只记这三条）

1. **车不动** → 先看日志第一行是不是 `MOCK`（F4）。
2. **串口不行** → `id -nG | grep dialout`，再 `sudo fuser -v /dev/ttyS1`（F2/F3）。
3. **视觉/套取不行** → 关掉它，别死磕：`SLEEVE_CONFIRM=False`（F12）、`SKIP_CAMERA_CHECK=1`（F17）。**带病跑 > 不跑**。

---

## 3. 性能预算表（50Hz 主循环，每帧 20ms）

### 3.1 主循环真实编排顺序

每帧调用链（`autonomous_state.py:237-334`）：

```
① 串口读位姿   read_pose()            [阻塞 ≤ 20ms，见下]
② 决策         decision.update()      （含异常检测、无效转运扫描）
③ 执行 Action  set_nav_target / start_trip / 急停
④ 感知         camera.get_frame() → perception.update()   ← 最重
⑤ 导航         navigation.update(dt=0.02)  → VelocityCommand  ← A* 可能重
⑥ 底盘下发     chassis.send_velocity()
⑦ 转运         transport.update()     ← 可能阻塞数百毫秒
⑧ 联动+看门狗
⑨ 帧尾 sleep  max(0, 0.02 - elapsed)  ← 超预算时不补睡，直接下一帧
```

**关键结构性事实**：`sleep_time = max(0, dt - elapsed)`（`autonomous_state.py:231`）。
→ **超预算时不会补偿，只是频率掉下来**（掉到 e.g. 10Hz），**不会雪崩式堆积**。这是设计上做对的。

### 3.2 逐项预算（估算，非实测）

> **口径声明**：下表是**基于代码的静态估算**（数循环次数、数 OpenCV 调用次数、读代码里的常量）。**本机（WSL / x86）与 RDK X5（ARM Cortex-A55）差一个数量级**，所以列的是"量级"而非精确值。所有 ⚠️ 项需要在 RDK 上实测确认（§3.4 给了实测方法）。

| 环节 | 主要开销来源（代码位置） | 典型量级 (X5) | 最坏情况 | 超预算风险 |
|------|------------------------|--------------|---------|-----------|
| **① 串口读位姿** | `read_pose()` 单次 `readline()`，`timeout=0.02`（`serial_chassis.py:56, 171-183`） | 0.1–2ms（有数据时几乎立即返回） | **20ms**（恰好无数据时顶满 timeout） | ⚠️ **中**：独占整帧 |
| **② 感知-HSV 分割** | `cv2.cvtColor` 1 次 + `cv2.inRange` **每颜色 1~2 次**（`detection.py:204-268`）。初赛颜色集合见 `target_types`，红色需 2 次 inRange（`detection.py:262-266`） | 640×480 上每次 inRange ≈ 0.3–1ms；总 **2–4ms** | 颜色多 + 分辨率被自动改成实测值（`detection.py:195-202`，可能 >640×480） | ⚠️ **高**：这是最重的单块 |
| **② 感知-形态学** | 每颜色 2 次 `morphologyEx`（OPEN+CLOSE，5×5 椭圆核，`detection.py:212-214`）。核**每颜色重建一次**（未复用） | 每次 ≈ 0.5–2ms → **2–8ms** | 颜色数 × 核尺寸 | ⚠️ **高**；**优化点**：kernel 提到循环外（见 §3.3-1） |
| **② 感知-轮廓** | `findContours` + 对**每个**轮廓 `contourArea`/`boundingRect`/`arcLength`/`approxPolyDP` + `moments` + `HuMoments` + `PCACompute`（`detection.py:217-246, 270-315`） | 轮廓数少时 **<1ms**；每轮廓约 10–30µs | **假轮廓爆炸**（花哨场地/噪点）→ 数千轮廓 → 几十 ms | ⚠️ **中高**：与场地噪声强相关 |
| **② 感知-分类** | `classify_batch` 逐条查表 + 容差匹配（`classification.py:88-97, 99-135`）。容差匹配的"任意形状"分支会**遍历整张配置表** | 每次 **<10µs** | — | ✅ 低 |
| **② 感知-测距** | 每目标 `estimate_ground_position`（纯三角，`detection.py:362-398`） | 每目标 µs 级 | — | ✅ 低 |
| **③ 世界地图关联** | **朴素最近邻双重循环**：`O(新检测 × 已有目标)`（`world_map.py:262-281`），外加 `_offer_pending` 再扫一遍（`world_map.py:332-366`） | 5 目标 × 5 检测 = 25 次 → **<0.1ms** | **25 目标 × 25 检测 = 625 次**（决赛满场）+ 每帧还有 `_age_pending` O(pending) | ✅ 低（625 次纯浮点，µs 量级） |
| **④ 决策** | 异常检测（含 STUCK 判定）+ `_check_invalid_transport` 遍历全部目标算到中心距离（`decision_engine.py:325-337`）；`select_targets_for_trip` 排序全部候选（`target_selector.py:216-223`） | 25 目标 <0.3ms | 每帧都对 25 个目标做 **sqrt + 排序** | ✅ 低 |
| **⑤ 导航-A\*** | **8 邻域 A\*，60×60 网格**（`path_planner.py:26, 163-240`）。⚠️ **实现缺陷**：每轮 `min(open_set, key=f_score)` 是 **O(n) 线性扫描**，且 **没有 closed set** → 最坏 **O(V²) ≈ 3600² ≈ 1.3e7** 次字典查找 | 短路径（几十格）≈ **1–5ms** | 长对角线穿越 3000mm + 高代价绕行 → **50–300ms** | 🔴 **高**：**最高风险项**。见 §3.3-2 |
| **⑤ 导航-纯追踪** | `track_path` → `compute_velocity`，纯三角 + 2 个 PID（`motion_control.py:153-219`） | **<0.1ms** | — | ✅ 低 |
| **⑤ 导航-局部避障(DWA)** | 仅在 `_is_near_obstacle` 为真时触发（`navigation_pipeline.py:266-275`）；`LocalPlanner.plan` = **50 采样 × 2 符号 × 5 步** = 500 次碰撞查询（`path_planner.py:318-331, 333-369`） | 不触发时 **0** | 触发时 **1–5ms** | ⚠️ **中**：注意 **`plan()` 的默认 `dt=0.5`**（`path_planner.py:302`），而主循环按 20ms 思考——速度窗口被放大 25 倍 |
| **⑥ 串口发送** | `send_velocity` → `_send`，一次 `write()` ≈ 20 字节 + `\r\n`（`serial_chassis.py:119-147`） | 115200 下 **≈1.8ms/帧**（按 20 字节） | — | ✅ 低。**带宽核算**：下行 20Hz ODOM + 50Hz IMU ≈ 4.2kB/s，占 115200 的 **≈45%**，留有余量 |
| **⑥ 串口接收(隐)** | 每帧只读**一行**，串口是混流（IMU/ODOM/ACK/TEL）→ 大部分行被丢弃 | 每次 readline 有数据 ≈ 0.1ms | — | ⚠️ **中（不是性能问题，是延迟问题）**：位姿实际更新率被"每帧一行"限制，**位姿可能延迟 1–3 帧（20–60ms）** |
| **⑦ 转运** | **在 50Hz 主循环里 `time.sleep` 阻塞**：<br>· `_halt_for_capture` → `send_stop`（`transport_pipeline.py:193-216`）<br>· `sleeve.lower_with_retry` → `wait_for(0.5) + MOVE_TIME_S(0.4)`，重试再 ×3（`sleeve_lift.py:380-389`）<br>· `place_ramp` 4 步 × `0.4/4`（`sleeve_lift.py:423-440`） | 套取单次 **≈0.9–1.0s**；投放 **≈0.4–0.5s** | 套取失败重试 → **单帧约 1.4–2.8s** | 🔴 **高（设计如此，非缺陷，但必须知晓）**：期间停发 `VEL`。已由显式停车 + 下位机看门狗兜底（`autonomous_state.py:323-339`） |
| **⑧ 看门狗** | 3 次时间比较（`autonomous_state.py:389-415, 429-449`） | **<10µs** | — | ✅ 低 |
| **⑨ 帧尾 sleep** | `time.sleep(max(0, 0.02 - elapsed))`（`autonomous_state.py:230-233`） | 补足到 20ms | 超预算时不睡 | — |

### 3.3 结论与优化点（按性价比排序）

**结论：有超预算风险，但不在"每帧都超"，而在"偶发单帧严重超"（A* 与转运）。**
- **稳态**（无 A* 重规划、无套取）：①+②+③+④+⑤+⑥ ≈ **4–12ms** → **在 20ms 预算内，可行**。
- **偶发超预算**：A* 长路径重规划、套取阶段（0.9–2.8s）、轮廓爆炸。这三者**都会让当帧直接跳过 sleep**，主循环瞬时掉到 0.5–20Hz。
- 好在：**不掉就不崩**（`max(0,...)`），且下位机有速度看门狗兜底。**风险是控制质量下降，不是程序死掉。**

**优化点（按收益排序，都是小改动）**

1. **HSV 核复用 + 掩码复用**（预期省 2–6ms/帧）
   `detection.py:212-214` 的 `getStructuringElement` 在**每个颜色的循环体内**重复创建同一个 5×5 椭圆核 → 提到循环外创建一次。
   进一步：`cvtColor(BGR2HSV)` 每帧只做一次是对的（`detection.py:204`，在循环外），保持。

2. **A\* 改 `heapq` + 加 closed set**（预期把最坏 50–300ms 压到 <10ms）—— **最高优先**
   现状：`min(open_set, key=...)`（`path_planner.py:192`）+ `open_set.remove(current)`（`:209`）+ 无 closed set → 节点会被反复入队/重算。
   改法：`heapq` 维护 f 值 + 弹堆时用 `g_score` 校验是否过期 + 用 `closed` 集合防止重复扩展。**A\* 是纯计算模块，改完必须用 `path_planner.py:375-427` 的独立测试 + `navigation_pipeline.py` 的 Mock 测试回归**。
   *低成本兜底*：若不想改算法，至少把重规划间隔从"每 30 帧"（`navigation_pipeline.py:88`）拉长，或把网格从 50mm 放大到 100mm（`path_planner.py:25-26`，代价是路径变粗糙）。

3. **转运阻塞移出主循环**（消除最长 2.8s 的卡顿）
   现状：套取/投放在主循环线程里 `sleep`。已做对的缓解是"先显式停车"（`autonomous_state.py:323-339`）。
   改法（较大）：把机构动作放独立线程/状态机，主循环只查 `is_holding()`/`is_complete()`；期间继续发 `VEL,0,0` 保活。
   **比赛前不建议重构**——现有"停车 + 看门狗"链路已经能保命。

4. **位姿读取提频**（消除 20–60ms 位姿延迟）
   现状：每帧只读一行（`autonomous_state.py:241`）。改法：每帧**循环读到无数据为止**（带上限，如最多 20 行），只保留最后一条 ODOM——因为 `read_pose()` 内部会做 `update_raw` 且解析 ODOM 才返回（`serial_chassis.py:209-225`），提速后位姿更新能跟上 IMU 的 50Hz。
   ⚠️ 改动要保证**不阻塞**：用 `in_waiting` 判断，别把 timeout 拉长。

5. **`LocalPlanner` 的 `dt` 对齐主循环**
   `plan()` 默认 `dt=0.5`（`path_planner.py:302`），但调用处没传 dt（`navigation_pipeline.py:248-251`），而主循环是 20ms。这会让速度窗口 `v_now ± 500*0.5 = ±250mm/s`（`path_planner.py:315`）严重放宽，DWA 选的加速度实际上做不出来。**建议传 `dt=0.02`**。

### 3.4 怎么在 RDK 上实测这套预算

代码里唯一的测速是感知的平均延迟（`perception_pipeline.py:222-232`，每 50 帧打一条 `logger.debug`）。**默认日志级别是 INFO（`main.py:90`），所以这条 debug 看不到**。实测方法：

```bash
# 方案 A：临时把日志调到 DEBUG（不改代码，用环境无关的方式起）
PYTHONPATH=src python3 -c "
import logging; logging.basicConfig(level=logging.DEBUG, format='%(asctime)s %(name)s: %(message)s', datefmt='%H:%M:%S')
from rescue_robot.main import main; main()
" 2>&1 | grep -E "帧 #|A\* |主循环"
```

**要看的三个数**：
1. `感知管线 ... latency_ms`（`perception_pipeline.py:227-232`）→ §3.2 的"感知"合计。
2. `A* 成功: N waypoints, Xmm, Yms`（`path_planner.py:199-200`）→ **A\* 绝对值**，最该盯的数。
3. 主循环实际频率：`主循环结束，共运行 N 个周期`（`autonomous_state.py:235`）÷ 运行秒数 → 稳态是否真 50Hz。

**【待确认】**：以上全部为静态估算，**没有在 RDK X5 上实测过**。比赛前必须在实机上跑一次 3.4，把三个数字填回本表。

> **`docs/audit/CODE_AUDIT.md`（S-23）的实测数据可交叉参考**：审计报告记录了「`read_pose()` 串口超时 + `A*` 单次 **26 ms**」都超出 20ms 预算——**26ms 单次 A\* 与本文 §3.2 的"典型 1–5ms / 最坏 50–300ms"区间相符**（26ms 落在中段，说明典型场景就已经贴近甚至超过预算）。**A\* 优化（§3.3-2）的优先级应视为最高**，不要因为"平时看着还行"而忽略。
> ⚠️ 注意：S-23 的行号是**审计当时的代码状态**；`autonomous_state.py`、`navigation_pipeline.py` 等在撰写期间仍在改动，**以函数名 grep 为准**。

---

## 4. 风险登记（Top 10）

> 排序原则：**先列会导致"本轮结束/取消资格"的**，再列丢大分的。
> 概率是主观估计（H/M/L），基于"代码现状 + 无实机标定"的前提。

| # | 风险 | 触发条件 | 影响 | 概率 | 预案 |
|---|------|---------|------|------|------|
| **R1** | **首次转运违规：不是"恰好 1 个普通物资"** → 本轮结束 + 本次成绩无效 | `select_targets_for_trip()` 在 `include_injured=True` 时**优先返回伤员**（`target_selector.py:209-214`），而 `DecisionEngine` 恰好用 `include_injured=True` 调它（`decision_engine.py:348-352`）；首次趟若判成伤员 → `can_load_batch` 报 `FIRST_TRIP_WRONG_TYPE`（`load_manager.py:221-226`） | 🔴 **本轮结束** | **M** | ① 代码侧：`FIRST_TRIP` 分支必须只走普通物资（`decision_engine.py:266-269` 用的是 `select_best_for_first_trip`，**只选普通物资** `target_selector.py:169-180`——所以主路径是对的，**风险在于状态机被切到 FREE_RUN 而首次还没完成**）；② 现场：**盯住第一条 `开始转运 (第1趟): 1 个目标 — 绿色...` 日志**（`transport_pipeline.py:274`），确认是"1 个 + 普通物资"；③ 兜底：`start_trip` 的 `can_load_batch` 会拒绝违规批次（`transport_pipeline.py:262-267`），机器会停在那趟——**停下来比违规好** |
| **R2** | **危险目标被移入安全区** → 本轮结束 | 危险目标（浅蓝，`yaml:74`）被误分类为普通物资；或推的过程中把危险目标一起带进安全区 | 🔴 **本轮结束** | **L–M** | ① **代码已有多层防护**：`can_load` 首条就是危险目标拒绝（`load_manager.py:151-153`）、`can_load_batch` 同样（`load_manager.py:196-201`）、`TargetSelector` 给危险目标 `points=0` → 分数必为 0 → 被 `score>0` 过滤（`target_selector.py:82-84, 101-102, 153`）；② 现场：把浅蓝识别阈值收紧（`detection.py:40`）；③ **赛前必做**：把"浅蓝"和"白"两个色卡放在场地里实测，确认不混（`_fuzzy_match` 把 `LIGHT_BLUE` 和 `WHITE` 视为相近色！`classification.py:110`） |
| **R3** | **单次转运 >3 个 / 伤员与其他混装** → 本轮结束 | 一趟里选了 4 个；或伤员 + 普通混装 | 🔴 **本轮结束** | **L** | ① 代码已挡：`max_count=3`（`decision_engine.py:350`）、`MAX_LOAD=3`（`load_manager.py:89`）、伤员独占检查（`load_manager.py:207-215`（已修为与顺序无关））；② 现场：无需处置，但要**核对日志里的 `N/3`**（`transport_pipeline.py:274-275`） |
| **R4** | **机器人停止运行 15 秒** → 本轮结束 | 主循环因异常/阻塞长时间不产出运动 | 🔴 **本轮结束** | **M** | ① **代码有意规避**：看门狗 15s 时**不再急停、只持续保命运动**（`autonomous_state.py:52-54, 429-437`），是有意的"保资格"设计；② 但要保证 >10s 时确实触发探索（`autonomous_state.py:446-449`）；③ 现场：**一旦看到 `🔍 探索模式` 就要立刻知道上游没产出动作**（F20），别等到 15s |
| **R5** | **进入对方安全区** → -5 分/次（可累加） | A* 规划出的路径穿过对方安全区；或定位漂移 | 🟠 丢分（可累加） | **M–H** | ① 代码已把对方安全区外扩 50mm 写成代价 255（`forbidden_zones.py:74-94` → `write_to_cost_map` `:235-240`）；② **目标点钳制**：任何导航目标若落在硬禁区内，`set_target()` 会先钳到最近合法点（`navigation_pipeline.py:125-137` → `forbidden_zones.py:156-192` `clamp_to_safe`）；③ **越界回退**：每帧在**所有分支之前**统一查硬禁区，命中就倒车 200mm/s（`navigation_pipeline.py:170-180`）；④ **但**：`A* 穿墙` 曾实测存在（`success=True` 却产出穿对方安全区的路径，见 `tools/hw_selftest.py --only navigation` 的输出），所以**别只看规划成功**；⑤ 现场：**首要盯定位漂移**（无视觉闭环，纯里程计 `chassis_interface.py:65-85`），跑两圈后对比实际位置 |
| **R6** | **角度/速度方向符号与下位机不一致** → 车往反方向走 | `VEL` 的 w 符号约定（正=逆时针）与下位机不一致 | 🟠 丢大分（跑反 = 基本拿不到分） | **M** | ① **必须上电实测一次**：发 `VEL,200,0`（应直行）、`VEL,0,300`（应原地左转）——用 `tools/hw_selftest.py --only velocity` / `--only odometry`；② 若反向 → 在 `chassis_interface.velocity_to_command`（`chassis_interface.py:95-104`）或下位机侧取反，**二选一，别两边都改**；③ 单位链已核对一致（mm/s + mrad/s） |
| **R7** | **A\* 偶发数百毫秒卡顿** → 控制不稳/丢帧 | 长路径 + 高代价绕行；`O(V²)` 实现（`path_planner.py:190-240`） | 🟠 丢分（路径抖动/震荡） | **M–H** | 见 §3.3-2。**低成本兜底**：拉长重规划间隔（`navigation_pipeline.py:88`）、加大网格（`path_planner.py:25-26`）、或 limit A* 的扩展节点数并在超限时退化为"直线接近 + 局部避障" |
| **R8** | **套取/投放阶段阻塞 1–3 秒** → 期间车不停或前冲 | `time.sleep` 在主循环里（`sleeve_lift.py:353-389, 423-440`） | 🟠 丢分（把目标撞飞/推歪，甚至推入错误分区） | **M** | ① 已有的正确缓解：**套取前显式停车**（清导航目标 + 零速 + `STOP`，`transport_pipeline.py:193-216` → `autonomous_state.py:323-339`）——**必须确认这条链路真的生效**，日志有 `🛑 显式停车（套取前）`；② 现场验证：架起轮子跑一次套取，看轮子有没有在套取瞬间转动 |
| **R9** | **现场改 YAML 不生效** → 以为改了参数其实没改 | YAML 只在启动时读一次（`main.py:105`），**没有实现热加载**；但 `config/robot.default.yaml:4-5` 的注释写着"保存后热加载生效" | 🟠 丢分（现场调参无效，浪费时间） | **H（几乎必中）** | ① **改完 YAML 必须重启程序**；② 或改 `src/rescue_robot/config.py` 里的常量并重启；③ **别信那句注释**（`yaml:4-5` 与 `main.py:105` 矛盾，属文档/实现不一致，见 CODE_AUDIT） |
| **R10** | **摄像头/串口在赛中掉线** → 感知降级为"假装有目标" | USB 松动/供电不足；串口被干扰 | 🔴 可能本轮结束（车乱跑） | **M** | ① 现状：摄像头失败 → 感知降级 **Mock**（`main.py:173-176`），机器人会追**假目标**——**这是最危险的一条**；② 串口掉线 → `read_pose` 返回 None，位姿冻结在最后一次（`autonomous_state.py:240-244`）→ 车按错误位姿继续跑；③ 预案：**线束固定 + 供电独立 + 赛前振动测试**；④ 代码侧建议（**未实现，属改进项**）：串口连续 N 帧无 ODOM → 主动停车而不是继续跑 |

### R1–R10 之外的两条"资格级"提醒（非软件风险，但会被软件带崩）

- **一键启动按钮**：赛规硬性要求机器人上有明确标识的一键启动按钮，而**当前还没这件硬件**（`HARDWARE_DEPENDENCIES.md:89-95`）。软件侧链路已就绪（走 `EVENT,START_BUTTON`），**但没按钮就进不了 AUTONOMOUS**。
- **抓取禁令**：赛规禁止"抓取"救援目标（只许推/套）。本项目机构是**套住后贴着地面推**（`HARDWARE_DEPENDENCIES.md:31-35`），`sleeve_lift.py` 的动作抽象是 `RAISE/LOWER/HOLD`（`sleeve_lift.py:32-36`），**没有夹紧动作** → ✅ 合规。**但现场别让机构做"夹"的动作**（如把舵机角度设到下限硬压）。

---

## 5. 交付核对表

> 生成时间：本文件撰写时。**状态以仓库实际文件为准**，未完成项已标注。

### 5.1 本次交付物

| # | 交付物 | 路径 | 状态 | 说明 |
|---|--------|------|------|------|
| 1 | 合规审计报告（对照赛项官方要求） | `docs/audit/COMPLIANCE_AUDIT.md` | ✅ 已存在（316 行） | 逐条对照官方要求核查 |
| 2 | 代码审计报告 | `docs/audit/CODE_AUDIT.md` | ✅ 已存在（630 行） | 阻塞/异常/并发/协议/坐标系/状态机/死代码/边界 |
| 3 | 修复后的代码 | `src/rescue_robot/**` | ⏳ **进行中（未提交）** | 撰写时 `git status` 显示 3 个文件有改动：`nav/forbidden_zones.py`（+41）、`nav/navigation_pipeline.py`（+36/-19）、`transport/load_manager.py`（+39/-19）。**尚未 commit** |
| 4 | 分模块实机测试程序 | `tools/hw_selftest/` + `tools/hw_selftest.py` | ✅ **已可运行** | 总入口 `tools/hw_selftest.py`（162 行）+ 公共框架 `framework.py` + **13 个模块**（见 5.2）。**已实测**：`--list` 正常列出 13 个模块；`--mock --only decision,navigation,transport` 跑通并正确报出 `PASS=2 FAIL=1`、结论 `故障模块 = navigation` |
| 5 | 测试程序使用手册 | `docs/` 下（待定名） | ⏳ **未见文件** | 见 5.2（用法取自 `tools/hw_selftest.py --help` 与 `tools/hw_selftest/__init__.py` docstring） |
| 6 | 独立验证报告 | `docs/audit/` 下（待定名） | ⏳ **未见文件** | verifier 产出 |
| 7 | 质量审查报告 | `docs/audit/REVIEW_REPORT.md`（待定名） | ⏳ **未见文件** | reviewer 产出 |
| 8 | 交付说明（本文） | `docs/RUNBOOK.md` | ✅ 本次产出 | 部署检查单 + 应急手册 + 性能预算 + 风险登记 + 交付核对 |
| 9 | 已有单元测试 | `tests/test_core_units.py`、`tests/test_innovation_integration.py` | ✅ 已存在 | 非本次新增 |

> ⚠️ 上表中标"未见文件"的项，可能是**其他成员尚未完成或提交**。请以 `ls docs/audit/` 与 `git log` 为准，不要以本表为准。

### 5.2 分模块测试程序怎么用（现场排障主工具）

**13 个可用模块**（`tools/hw_selftest.py --list` 实测输出）：

| 模块名 | 覆盖 |
|--------|------|
| `serial` | 串口链路（打开 / PING→PONG / START→ACK） |
| `telemetry` | 下位机遥测（ODOM 8 字段@20Hz / IMU 10 字段@50Hz / TEL） |
| `odometry` | 里程计（坐标换算 + 前进/左转符号） |
| `velocity` | 速度闭环（50Hz 持续 VEL → 实测 vs 目标） |
| `motors` | 电机与底盘（正反转 / STOP 立即停） |
| `servo` | 夹爪舵机（RAISE/LOWER/HOLD/ANGLE 0~70 + 越界保护） |
| `start_button` | 一键启动链路（EVENT,START_BUTTON → AUTONOMOUS） |
| `camera` | 摄像头（打开 / 分辨率 / 帧率） |
| `ranging` | 视觉测距（底边+倾角地平面法） |
| `vision` | 视觉识别（无帧保护 / 空帧防误报 / 四类目标检出） |
| `decision` | 决策与载规则（首次单独 1 个普通 / ≤3 / 伤员单独 / 危险拒绝） |
| `navigation` | 导航（绕障 / 不可达显式停止 / 全程不出界） |
| `transport` | 转运状态机（Mock 夹爪全流程） |

```bash
PYTHONPATH=src python3 tools/hw_selftest.py                    # 跑全部（无硬件自动 SKIP）
PYTHONPATH=src python3 tools/hw_selftest.py --list             # 列出模块
PYTHONPATH=src python3 tools/hw_selftest.py --only servo       # 只跑舵机（逗号可多个）
PYTHONPATH=src python3 tools/hw_selftest.py --mock             # 明确声明"没硬件"→环境相关项按 SKIP
PYTHONPATH=src python3 tools/hw_selftest.py --yes-motion       # 允许驱动电机（先架起轮子！）
PYTHONPATH=src python3 tools/hw_selftest.py --port /dev/ttyS1  # 指定串口（默认取 CHASSIS_PORT，否则 /dev/ttyS1）
PYTHONPATH=src python3 tools/hw_selftest.py --image x.jpg      # vision 模块用离线图片
PYTHONPATH=src python3 tools/hw_selftest.py --duration 5       # 遥测/速度类采样秒数
```

**输出长这样（实测）**，最后三行就是"哪个模块坏了"的答案：
```
▶ navigation — 导航（绕障 / 不可达显式停止 / 全程不出界）
   ❌ [FAIL] 目标为「对方安全区内部」时导航仍在持续输出速度（末指令 linear=200）→ 车会一直朝这个方向跑
        · A* 穿墙：success=True 航点=52 路径长=3171mm（直线 2600mm）
        · 路径点落在不可通行格内的个数：0 ✓
        ↳ 处置：不可达目标必须显式停止：state=BLOCKED 且速度归零
==============================================================================
  模块            结果      结论
  decision      PASS    ...
  navigation    FAIL    ...
  transport     PASS    ...
  PASS=2  FAIL=1  SKIP=0
  ❌ 结论：故障模块 = navigation
     按上面每个 FAIL 模块的「处置」逐条排查。
```

设计约定（`tools/hw_selftest/framework.py:1-9`，正好满足现场排障的三个要求）：
- **绝不抛异常**：任何模块出错 → FAIL/SKIP + 人话结论，不会带崩整个自检。
- **结论能指导动作**：串口错误被翻译成"不存在 / 权限不足 / 被占用 / 其他"四类并给处置（`framework.py:175-190`）。
- **无硬件 SKIP 而非 FAIL**：本机没串口/没摄像头也能跑完整套，并告诉你"哪些因环境受限未测"（`framework.py:168-172`）。
- **动电机的测试有安全闸门**：默认不动作，必须显式 `--yes-motion`；提示"先架起轮子离地空转"（`framework.py:226-234`）。
- **复用仓库既有串口实现**：解析用 `SerialChassis.parse_frame/parse_imu`（`framework.py:129-134`、`268-275`），**没有另写第二套协议**。✅ 符合"不改协议"的纪律。

### 5.3 待真机标定 / 待确认项（上场前必须闭环）

| # | 项 | 在哪改 | 现状 | 怎么标 |
|---|----|--------|------|--------|
| 1 | **相机下倾角 `TILT_DEG`** | `config.py:100`（当前 30.0°） | 声明"已确认 30°"，但仍需实测反解 | 把目标放在已知 500/1000/1500mm，量检测框**底边**像素 y，反解倾角（`config.py:95-97` 给了公式） |
| 2 | **相机光心高度 `HEIGHT_MM`** | `config.py:99`（当前 210mm） | 与文档一致（`HARDWARE_DEPENDENCIES.md:41`） | 卷尺实测 |
| 3 | **套取 ROI `SLEEVE_ROI`** | `config.py:107`（当前 `(0.32,0.55,0.68,0.98)`） | **从未标定**，但 `SLEEVE_CONFIRM=True` **默认开启**（`config.py:109`） | 把目标放进 U 型槽，看检测框中心落在归一化坐标哪个区间；**标定前建议先关掉** |
| 4 | **HSV 阈值（9 种颜色）** | `detection.py:33-46` | 硬编码，未按现场光照标定 | `tools/vision_calibration.py` |
| 5 | **相机 index** | 环境变量 `CAM_INDEX` | 代码两处默认值不一致（0 vs 1） | 1.6 的循环脚本 |
| 6 | **RDK X5 UART 设备名** | `CHASSIS_PORT` | 代码默认 `/dev/ttyUSB0`，注释写 `/dev/ttyS0`，本题要求 `/dev/ttyS1` | 1.3 的数据确权脚本 |
| 7 | **电池低压阈值** | `config.py:60`（11.0V） | 电池类型未最终确认（12V/2500mAh，`HARDWARE_DEPENDENCIES.md:128-134`） | 量一次低电量实压 |
| 8 | **`VEL` 的 w 符号 / 直行方向** | — | 未实测 | §4 R6 |
| 9 | **推进距离 `_push_dist_mm`** | `transport_pipeline.py:106`（当前 100mm） | 注释标明"真机标定" | 实测推入斜坡所需距离 |
| 10 | **`place_ramp` 角度序列** | `sleeve_lift.py:423-440` | 注释标明"需真机标定（SG90，斜坡 34°）" | 分步录视频，确定不卡不飞 |
| 11 | **50Hz 帧预算** | 本文 §3.2 | **全部为静态估算，未实测** | §3.4 的三个数 |
| 12 | **整机重量/尺寸** | 硬件 | ≤1.5kg / ≤300×300mm / 高≤200mm（`README.md:74-82`） | 称重量尺 |

---

## 附录 A：环境变量速查（复制粘贴用）

```bash
# ── RDK X5 比赛现场（真机）──
export RUN_MODE=real
export CHASSIS_PORT=/dev/ttyS1
export CAM_INDEX=0
export TEAM_COLOR=red          # 抽签结果
export CAM_WARMUP_S=3.0
# export SKIP_CAMERA_CHECK=1   # 仅台架无摄像头时打开

cd ~/rescue-robot
PYTHONPATH=src python3 -m rescue_robot.main

# ── 台架 / 无硬件（开发机）──
export RUN_MODE=mock
PYTHONPATH=src python3 -m rescue_robot.main

# ── 分模块排障 ──
PYTHONPATH=src python3 tools/hw_selftest.py --list
PYTHONPATH=src python3 tools/hw_selftest.py --only serial
```

## 附录 B：关键常量速查（改这些前先确认影响面）

| 常量 | 值 | 位置 | 影响 |
|------|----|------|------|
| 主循环频率 | 50Hz / dt=0.02 | `autonomous_state.py:217` | 全局时序 |
| 看门狗 探索/保命/硬限 | 10 / 13 / 15 s | `autonomous_state.py:52-54` | 15s 规章风险 |
| 启动后延迟 | 1000 ms | `config.py:51`（`POST_START_DELAY_MS`） | 裁判离场时间 |
| 比赛时长 / 时间紧迫 | 180 / 30 s | `decision_engine.py:98-99` | 与 `yaml:52-53` 一致 |
| 装载上限 | 3 | `load_manager.py:89` | 违规红线 |
| 导航目标容差 | 40 mm | `motion_control.py:112` | 到位判定 |
| 套取触发距离 | 150 mm | `transport_pipeline.py:296` | 何时开始套 |
| 投放触发距离 | 80 mm | `transport_pipeline.py:369` | 何时开始投 |
| 网格尺寸 | 50 mm / 60×60 | `path_planner.py:25-26` | A* 精度与耗时 |
| 重规划间隔 | 30 帧 | `navigation_pipeline.py:88` | A* 调用频率 |
| 串口超时 | 20 ms | `serial_chassis.py:56` | 单帧最大阻塞 |
| 目标关联阈值 | 100 mm | `world_map.py:85` | 目标跟踪稳定性 |
| 新目标确认帧数 | 3 | `world_map.py:89` | 假目标过滤 |
| 目标丢失移除阈值 | 150 帧（3s） | `world_map.py:97` | 丢目标 vs 重复扑空 |
| 套取失败重试 | 3 次 / 后退 120mm | `transport_pipeline.py:173-175` | 单趟兜底 |
| 视觉确认自动关闭 | 连续 5 次失败 | `transport_pipeline.py:115, 324-328` | ROI 未标定的保护 |
| 禁区外扩（对方安全区） | 50 mm | `forbidden_zones.py:55` | -5 分/次红线 |
| 场边禁区外扩 | 100 mm | `forbidden_zones.py:58` | 出界=比赛结束 |
| 禁区越界回退速度 | -200 mm/s（倒车） | `navigation_pipeline.py:177` | 越界后的自救 |
| 目标点禁区钳制 | 钳到最近合法点，最多迭代 8 次 | `forbidden_zones.py:156-192` | 防止开进对方安全区 |

## 附录 C：本文档引用过的源码位置（便于复核）

```
main.py:72-76, 98, 105-109, 129, 137-176, 179-192, 224-230, 247-262, 269-292
config.py:14-35, 60-64, 99-109, 126-148
state_machine.py:34-39, 184-203
system_check.py:156-207, 225-262, 293-310, 312-334
states/boot_state.py:44-61
states/debug_state.py:50-61
states/autonomous_state.py:52-54, 59-60, 143, 189, 213-235, 237-334, 339-354, 389-449
hardware/chassis_interface.py:11-13, 41, 65-104
hardware/serial_chassis.py:8, 18-19, 56, 72-85, 106-167, 171-263, 265-327
hardware/camera_reader.py:1-22, 61-99, 125-189
hardware/button.py:84-113
perception/detection.py:33-46, 146, 173-248, 250-268, 317-322, 324-398
perception/classification.py:49-97, 99-135
perception/perception_pipeline.py:145-234, 274-283
perception/world_map.py:85-97, 157-161, 237-298, 300-366
perception/field_elements.py:93-115, 118-267
decision/decision_engine.py:98-99, 247-257, 261-320, 325-337, 339-407
decision/target_selector.py:42-109, 169-223
navigation/navigation_pipeline.py:59-99, 138-273, 286-301, 315-335
navigation/path_planner.py:25-34, 163-240, 275-331, 333-369
navigation/motion_control.py:107-116, 153-219
navigation/forbidden_zones.py:54-58, 70-131, 194-199
transport/transport_pipeline.py:106, 111-115, 173-175, 193-242, 246-277, 279-431
transport/sleeve_lift.py:32-36, 181-315, 321-443
transport/load_manager.py:29-49, 89, 139-181, 183-234, 236-312
innovation/config_loader.py:24, 75-112, 259-263, 420-425
tools/hw_selftest/framework.py:1-9, 100-234, 245-279
tools/hw_selftest/__init__.py:1-16
config/robot.default.yaml:1-6, 31-41, 52-53, 70-78
config/field.default.yaml（全文）
HARDWARE_DEPENDENCIES.md:31-35, 41, 89-95, 128-134
README.md:74-82, 339-395
```
