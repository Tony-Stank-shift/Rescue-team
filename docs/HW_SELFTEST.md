# HW_SELFTEST —— 分部实机自检手册

> **一句话**：比赛现场出问题时，不猜、不重启，先跑 `tools/hw_selftest.py`，它会告诉你
> **「故障模块 = XXX」**，并给出该模块的处置动作。
>
> 与 `docs/RUNBOOK.md` 的关系：RUNBOOK §5.2 是速览（一页表格），**本手册是它的展开版**——
> 每个模块测什么、PASS/FAIL 怎么判读、报错怎么处置。部署/环境/性能问题看 RUNBOOK，
> "到底哪个板块坏了"看本手册。

---

## 1. 怎么跑

```bash
cd ~/Rescue-team

# ① 无硬件（开发机 / WSL / 台架没接线）：环境相关项按 SKIP，不当故障
PYTHONPATH=src python3 tools/hw_selftest.py --mock

# ② 实机全量（默认 /dev/ttyS1 @115200；环境变量 CHASSIS_PORT 可覆盖）
PYTHONPATH=src python3 tools/hw_selftest.py

# ③ 只跑某个模块（可逗号分隔多个）——现场排障主力用法
PYTHONPATH=src python3 tools/hw_selftest.py --only servo
PYTHONPATH=src python3 tools/hw_selftest.py --only serial,telemetry

# ④ 看有哪些模块
PYTHONPATH=src python3 tools/hw_selftest.py --list
```

| 开关 | 作用 | 什么时候用 |
|---|---|---|
| `--list` | 列出 14 个模块名与覆盖内容 | 忘了模块名时 |
| `--only a,b` | 只跑指定模块 | 已知大概哪个板块坏了，单点复现 |
| `--mock` | 明确声明"现在没硬件" → 环境相关项按 **SKIP** 而不是 FAIL | 开发机、赛前桌面演练 |
| `--yes-motion` | 允许驱动电机（**必须先架起轮子离地**） | 测里程计符号 / 电机 / 速度环 |
| `--port /dev/ttyUSB0` | 指定串口（默认 `/dev/ttyS1`） | RDK 板载 UART 与 USB-TTL 互换调试 |
| `--image a.jpg` | 给 `vision` 模块喂一张离线图 | 现场复现"认不出目标" |
| `--duration 5` | 遥测/速度类采样秒数（默认 3） | 频率抖动、速度环要看更长时间 |
| `--verbose` | 显示被检模块的 INFO 日志 | 模块 PASS 但你怀疑它被跳过时 |

**退出码**：`0` = 没有 FAIL；`1` = 有 FAIL。可以直接被脚本/CI 判读。

⚠️ **不要与主程序同时跑**：`rescue_robot.main` 和自检会抢 `/dev/ttyS1`，
抢不到的那一方报"被其他进程占用"。自检前先停主程序。

---

## 2. 结果怎么读

每个模块输出一行结论 + 若干证据行 + 一条处置建议：

```
▶ servo — 夹爪舵机（RAISE/LOWER/HOLD/ANGLE 0~85 + 越界保护）
   ✅ [PASS] 夹爪舵机正常（动作命令、0~85° 标定范围、越界保护均正确）
        · SERVO,RAISE → ACK,SERVO,RAISE
        · SERVO,ANGLE,85 → ACK,SERVO,ANGLE,85
        · SERVO,ANGLE,86 → ERR,SERVO_ANGLE
        ↳ 处置：...
```

| 标记 | 含义 | 现场动作 |
|---|---|---|
| ✅ **PASS** | 该模块实测通过 | 不用管 |
| ❌ **FAIL** | 该模块真的有问题（真机模式下设备打不开也算 FAIL，因为那就是故障） | 照该模块的「处置」逐条做 |
| ⏭️ **SKIP** | 因**环境/参数**未测（`--mock` 没硬件、动电机没给 `--yes-motion`） | 看是不是正常跳过；该测的没测就补参数重跑 |

**结论行**（汇总表最后两行）：

```
  PASS=7  FAIL=4  SKIP=3
  ❌ 结论：故障模块 = serial, telemetry, servo, camera
```

- 有 FAIL → 先修 FAIL 的模块，**不要**先看 SKIP。
- 全 SKIP 且 `--mock` → 正常（本机没硬件），代码链路已自检。
- `SKIP=2` 里若是 `odometry/motors/velocity` → 那是"动电机需要 `--yes-motion`"，
  架好轮子重跑即可。

---

## 3. 14 个模块：测什么 / 判定 / 失败怎么办

排障顺序即模块执行顺序（从纯算法 → 链路 → 硬件）：
`ranging → decision → navigation → transport → accounting → serial → telemetry →
start_button → servo → odometry → motors → velocity → camera → vision`

| 模块 | 测什么 | PASS 判定 | FAIL 典型原因与处置 |
|---|---|---|---|
| `ranging` | 视觉测距（底边+倾角地平面法）公式回环 + 倾角敏感性 | 200~3000mm 回环误差 <1% | 公式/参数接线错；`config.Camera` 与 `CVDetector` 参数不一致。**注意**：这只是公式自检，真实距离仍必须标定 `TILT_DEG` |
| `decision` | ①载规则：首次单独 1 个普通物资 / ≤3 / 伤员单独 / 危险拒绝 / 空批次不崩；②**B8** 场心误判；③**S-01** 终场停车（见 §10） | 规则判定正确 + B8 不误判不漏判 + DONE 后清目标/停车/退出 | 规则项**直接丢分**：查 `LoadManager.can_load_batch` 的判定顺序。S-01/B8 的处置见 §10.3 |
| `navigation` | ①从出发区走到对角目标；②异常目标（场外/对方安全区）按**新契约**显式拒绝或钳制、不崩不冲进禁区 | 能到达（残距 <80mm）；场外坐标 `set_target` 返回 `False` 且不改原目标；禁区内目标被钳制到合法点 | 未到达：看 A*/纯追踪/到点判定。越界拒绝失效 → 查 `set_target` 是否用 `is_in_field()` 拒绝（旧行为=静默夹紧，见 §9.2） |
| `transport` | 转运状态机全流程（Mock 夹爪）：APPROACHING→CAPTURING→TRANSPORTING→PLACING→COMPLETE | 流程走完 + 显式停车回调被调用 + 无 VIOLATION | 卡在中间阶段：看到点阈值/推入步骤；VIOLATION：载规则或投放判定 |
| `accounting` | **S-40 载荷台账一致性**：计划≠实装必须被区分（台账/位姿复核/持有不抬爪，见 §10.2） | 台账数=实装数、只记真正套住的那个、隔 >150mm 不记账 | 台账虚高 → 分**虚高**；查 `CAPTURING` 分支的 `_capture_index`/`CAPTURE_RADIUS_MM`/`_captured` |
| `serial` | 串口能否打开 + `PING→PONG` + `START→ACK,START` | 两者都收到 | 见 §4 串口四分类；能开但无 PONG → 下位机没跑/只接单向/未共地 |
| `telemetry` | ODOM **8** 字段@20Hz、IMU **10** 字段@50Hz、TEL **8** 字段、数值合理性（az≈1000mg）、解析器一致性 | 帧前缀/字段数/频率/数值都对 | 见 §4；0 行：下位机没发数据或波特率不符 |
| `start_button` | 一键启动：软件层 `read_button` 是否认得 `EVENT,START_BUTTON`；硬件层需 `HW_SELFTEST_INTERACTIVE=1` 人工按键 | 软件层识别成功 | 识别不了 → 现场按按钮不会进 AUTONOMOUS（查 `SerialChassis.read_button` 匹配前缀） |
| `servo` | RAISE/LOWER/HOLD 各自 ACK；ANGLE 0/45/85 全部接受；86/180/-1 必须回 `ERR,SERVO_ANGLE` | 动作 ACK 正确 + 越界被拒 | 无 ACK：固件未实现或未 `START`；越界没被拒 → 有把舵机顶到限位的风险。动作角度语义：**0°=下压套住、85°=抬起释放** |
| `odometry` | 坐标换算（前进 +Y / 左移 −X / theta 累加）+ 实际运动符号 | 换算正确；`--yes-motion` 时前进 Δx>0、左转 Δθ>0、右转 Δθ<0 | 符号反 → 左右编码器 A/B 相接反或左右轮定义互换 |
| `motors` | `TESTPWM` 正反转（PWM=30，2s）+ `STOP` 后残余轮速 | 正转两轮 Δenc/轮速为正、反转全为负、STOP 后 <0.05m/s | 某方向不动：电机极性/TB6612 接线/左右轮接反；STOP 停不住：停车链路或速度环 |
| `velocity` | **50Hz 持续**下发 `VEL,200,0` 3s，实测位移速度 vs 目标 | 偏差 ≤±25% 且期间无 `EVENT,WATCHDOG*` | ⚠️ 下位机看门狗 300ms 保持/800ms 停：**单发 VEL 必被停**。出现 WATCHDOG → 主循环有阻塞（视觉/串口读等待）导致断流 |
| `camera` | 打开 / 首帧 / 分辨率 / 实测出帧率（≥`CAMERA_MIN_FPS`，默认 10） | 出帧且 fps 达标 | 打开失败：换 `CAM_INDEX`、查占用与 video 组；能开不出帧：USB 供电/带宽/排线。**已知**：自检用 video1、主循环用 video0（CODE_AUDIT S-10） |
| `vision` | ① `detect(None)` 必须返回 `[]`；② **空帧必须 0 检出**（防背景被当目标）；③ 初赛四类目标逐色检出 + 分类器映射（浅蓝必须→DANGEROUS）；④ `--image` 离线图 | 四条全过 | 空帧有检出 → HSV 的 BLACK 阈值 `V≤60` 把深灰背景吃进去了（历史误报，实测整幅画面被当 1 个黑色目标）；某色检不出 → 现场光照下标定 HSV；离线图 0 检出 → 阈值与光照不符 |

---

## 4. 常见 FAIL 的现场处置

### 4.1 串口（`serial`/`telemetry`/`servo`/`odometry`/`motors`/`velocity` 同时 FAIL）
自检会先把原因翻译成**四类**之一，再给处置：

| 结论里的话 | 真实含义 | 处置 |
|---|---|---|
| 设备 **不存在** | 节点都没有 | 下位机上电了吗、USB/排线插好了吗；RDK 板载 40PIN UART1 = `/dev/ttyS1`，USB-TTL 一般是 `/dev/ttyUSB0` |
| 存在但**无权限** | 节点在，open 被拒（Errno 13） | `sudo usermod -aG dialout $USER` 后**重新登录**（组在登录时确定；`sudo` 跑不解决）；或临时 `sudo chmod 666 /dev/ttyS1` |
| **被其他进程占用** | 别人拿着 | 先停 `rescue_robot.main` / VSCode 调试会话，再自检 |
| 未知原因 | 其余 | 看证据里的原始异常 |

**能打开但 PING 没有 PONG**：①下位机是否上电 ②固件是否烧录并复位
③TX/RX 是否交叉 ④GND 是否共地 ⑤是否被别的进程占用。

**`telemetry` 收到 0 行**：下位机在发数据吗（正常应持续 ODOM/IMU/TEL）；两端是否都 115200。
**帧格式不对**：字段数不匹配 → 固件版本与 `chassis_serial_protocol.md` 不一致。

### 4.2 运动类
| 现象 | 处置 |
|---|---|
| `odometry`/`motors`/`velocity` 报 SKIP | 正常：动电机需 `--yes-motion`。**先把轮子架起离地空转**再加参数 |
| 只有一侧轮子不动 | 该侧电机接线/驱动（TB6612 IN1/IN2） |
| 前进时航向漂移 >15° | 左右轮速不一致、编码器分辨率或轮径参数不符 |
| 速度闭环偏差 >25% | 速度环 PI 是 12V 架空标定值，落地需复测；轮子打滑也计入 |
| 出现 `EVENT,WATCHDOG_STOP` | 上位机断流了（某处阻塞 >800ms），查主循环里有没有卡住的分支 |

### 4.3 视觉 / 摄像头
| 现象 | 处置 |
|---|---|
| `camera` FAIL：打不开 | `ls /dev/video*` 确认识别；`CAM_INDEX=0/1` 试；确认没别的程序占用；用户在 `video` 组 |
| `camera` FAIL：出帧率低 | 换 USB3 口、缩短线、降分辨率、关掉其他占用摄像头的程序 |
| `camera` SKIP（`--mock`） | 开发机没摄像头，正常 |
| `vision` FAIL：空帧有检出 | 光照/阴影把背景吃进颜色阈值；重跑 `tools/vision_calibration.py` 标定 |
| `vision` FAIL：某类检不出 | 同色标定问题；确认目标确实在画面里 |
| `vision` FAIL：离线图 0 检出 | 用现场光照重拍一张再试；逆光/过曝会直接检不出 |

### 4.4 决策 / 导航 / 转运（纯算法，与硬件无关）
这三项 FAIL **一定是软件问题**，不是接线问题：
- `decision` FAIL → 载规则理解/实现错，**直接丢分**，优先修。
- `navigation` FAIL → 看是否 `BLOCKED`（规划无解）、是否原地打转（到点判定）。
- `transport` FAIL → 看卡在哪个阶段，对照 `TransportPhase` 的转移条件。

---

## 4.5 ⚠️ 自检**自己也会错** —— 报 FAIL 时先怀疑自检

2026-09-17 真机自检报了 3 个 FAIL，其中 **2 个是自检脚本自己的 bug**，固件没问题。
两次都差点让人去追一个不存在的固件缺陷：

| 报错 | 真相 | 根因 |
|---|---|---|
| `TEL 字段数 8（期望 10）` | **8 才是对的** | `EXPECT_FIELDS["TEL"]` 从 `IMU` **抄错**成 10。协议与固件都是 `TEL`+7 值=8 段 |
| `越界角度 71 未被拒绝（实际 None）` | **固件正确回了 `ERR,SERVO_ANGLE`** | 助手写成 `wait_for("ACK,SERVO",1.0) or wait_for("ERR,",0.3)`，而 `wait_for` **边读边丢弃**不匹配行 → 第一个 wait_for 把 ERR 行读掉丢了 → None |

**识别信号**：报的"实际值"是 **None / 空 / 0** 而不是"收到了错误的值" ——
这通常意味着**读取逻辑**出了问题，而不是对端没做对。

**已加的护栏**（`tools/fix_verifiers/verify_selftest_truthfulness.py`，已并入 `run_all`）：
- 遥测期望字段数**直接从固件 `telemetry.c` 的格式串推导**再比对（不再靠手抄）；
- 协议文档里的字段数也与固件交叉核对；
- 舵机助手必须"一次遍历、ACK/ERR 都认"，并用**假串口流**做行为验证
  （喂一条只含 `ERR,SERVO_ANGLE` 的流，旧写法必须得到 None、新写法必须读到它）；
- 自检的 `MAX_ANGLE` 必须等于固件 `SERVO_MAX_ANGLE_DEG`。

> 顺带确认：固件的越界分支是**先 `return` 再驱动舵机**，所以「只测越界角度」
> 这一项本身**不会让机构动**（要动机构的是 0/35/70 那几项）。

## 5. 安全注意（动电机之前必看）

1. **默认不动电机**：`odometry`/`motors`/`velocity` 在真机模式下**默认 SKIP**，
   必须显式 `--yes-motion` 才会执行。
2. **加 `--yes-motion` 之前**：把车**架起、轮子离地空转**；清空车周围 1m；手边能随时断电。
3. **不要对着人或易倒物测**：`velocity` 会持续 3s 输出 200mm/s，轮子落地会直接窜出去。
4. `servo` 会真实驱动夹爪：确保夹爪行程内没有手指/线束。
5. `motors`/`velocity` 结束会发 `STOP`；若中途 Ctrl+C，**先给下位机断电**再检查。
6. 自检与 **主程序不能同时跑**（抢串口）。
7. 跑完 `servo` 会自动回 `SERVO,RAISE`，避免夹爪压在机构上。

---

## 6. 实测输出（本机 WSL，无硬件，可直接对照）

### 6.1 `--mock`（14 模块全跑，不崩）
```
  PASS=7  FAIL=0  SKIP=7
  ✅ 结论：未发现故障模块（部分模块因环境受限 SKIP）
```
`SKIP` 的 7 项：`serial/telemetry/servo/odometry/motors/velocity`（无串口权限）+ `camera`（无摄像头）。
`PASS` 的 7 项：`ranging/decision/navigation/transport/start_button/vision/accounting`（不依赖硬件的都真跑了）。

### 6.2 真机模式（不加 `--mock`，本机无权限/无摄像头 → 如实报 FAIL）
```
  PASS=7  FAIL=4  SKIP=3
  ❌ 结论：故障模块 = serial, telemetry, servo, camera
```
- `serial/telemetry/servo` FAIL 原因：`/dev/ttyS1 存在但当前用户**无权限**`（Errno 13）
  → 处置：加入 dialout 组后重新登录。
- `camera` FAIL：`摄像头 index=0 打不开（/dev/video0）`。
- `odometry/motors/velocity` SKIP：需 `--yes-motion`。
- 退出码 `1`（有 FAIL）——脚本/CI 可直接判读。

> 结论：**真机模式下设备打不开就是 FAIL**（那就是故障）；`--mock` 是"我明确知道现在没硬件"，
> 才降级为 SKIP。两者语义不同，别混用。

### 6.3 故障注入（验证工具真能定位到模块）
```bash
PYTHONPATH=src python3 tools/hw_selftest.py --only serial,servo --port /dev/nonexistent
```
```
  serial        FAIL    设备 /dev/nonexistent **不存在**
  servo         FAIL    设备 /dev/nonexistent **不存在**
  ❌ 结论：故障模块 = serial, servo            （退出码 1）
```
说明：错误的串口路径被正确归因到"设备不存在"，并**把结论挂到调用它的模块**上
（`serial`/`servo` 各报一次），而不是笼统报"跑不起来"。

---

## 7. 现场三分钟流程

```bash
# 0) 停掉主程序，架起轮子（要测运动的话）
# 1) 先看整体：哪个模块红了
PYTHONPATH=src python3 tools/hw_selftest.py
# 2) 红了就单点复现
PYTHONPATH=src python3 tools/hw_selftest.py --only <故障模块> --verbose
# 3) 按输出的「↳ 处置」做；修完再单点跑一次确认转 PASS
```

---

## 8. 维护须知（改本套自检时）

- 新增模块：在 `tools/hw_selftest/` 加 `m_<name>.py`，用 `@register("name", "标题")` 注册，
  把名字加进 `framework.ORDER`（不加以也会被跑到，但顺序靠后）。
  骨架：
  ```python
  from .framework import register, ok, bad, skip
  MODULE, TITLE = "xxx", "中文标题"
  @register(MODULE, TITLE)
  def run(ctx):                      # ctx: framework.Ctx
      ...                            # 失败用 bad(...)，不要 raise
  ```
- **不允许**在模块里写新的串口协议：解析一律用
  `SerialChassis.parse_frame / parse_imu`，命令一律用它的 `send_*`；
  `ctx.require_cli(module)` 取通道，`ctx.need_motion(module)` 作为动电机闸门。
- 模块内**绝不抛异常**：一切失败都变成 FAIL/SKIP + 人话结论。
- 已改动过的文件：`m_vision`（补"空帧防误报 + 四类目标 + 分类映射"）；
  `m_navigation` 维持队长版本（喂外部位姿），其文件头部留有 REVIEW 提议注释。

---

## 9. 已知问题（不是自检本身的 bug，但会影响读结果的人）

### 9.1 ⚠️ 自检/仿真**必须喂外部位姿**，不要用 `nav.update(None)`

- **现象**：`MockLocalizer`（Mock 定位器）初始位姿固定为 **(1500, 300)**，该点落在
  **BLUE 安全区内部**。若用 `NavigationPipeline(..., my_color=RED, use_mock=True)` 且
  调 `nav.update(None)` 让导航走**自己的内部积分器**，机器人从第一帧就被判定"身处对方禁区"
  → 避障以 200mm/s 倒退 → 实测**第 75 步驶出场地边界**、4000 步停在 `y=-104` 并永久
  `BLOCKED`（此后所有 A* 因起点不可通行而失败）。
- **影响面**：**仅 mock 路径**。真机位姿由 `set_start_pose` / 下位机里程计决定，不受影响；
  但任何自检脚本、仿真、单元测试若用 `update(None)` 都会被它坑成"导航坏了"的假象。
- **正确写法**（与真机 `states/autonomous_state._run_once` 一致）：显式喂位姿并自己积分
  ```python
  x, y, th = 150.0, 150.0, math.pi/2          # 出发区
  cmd = nav.update((x, y, th), dt=0.02)
  x += cmd.linear * math.cos(th) * dt
  y += cmd.linear * math.sin(th) * dt
  th += cmd.angular * dt
  ```
  `tools/hw_selftest/m_navigation.py` 用的就是这种写法（实测 954 步到达 (2694,2694)、
  残距 79mm、[PASS]）。

### 9.2 ~~场外坐标被静默夹紧（A* 报成功，其实到不了）~~ —— **已修复（2026-09-15，fixer）**

- **原现象**：`CostMap._to_grid` 会把超界坐标 clamp 进网格：`(9000, 9000)` → 网格 `(59,59)`
  ≈ 场内 `(2975, 2975)`，于是 `AStarPlanner.plan` 返回 **`success=True`**，而
  `ForbiddenZoneManager.is_in_field(9000,9000)` 为 `False` → 上层以为"已规划到目标点"，
  实际永远走不到，且没有任何报错。
- **现状**：`NavigationPipeline.set_target` 已改为**返回 bool + 越界拒绝 + 禁区内钳制**
  （契约存档：`docs/audit/S_NEW_NAV_TARGET_CONTRACT.md`）。自检 `navigation` 模块已按新契约
  断言：场外坐标 → `set_target` 返回 `False` 且**不改动原目标**；禁区内坐标 → 返回 `True`
  但目标被钳制到最近合法点。实测证据行 `[S-NEW]`：
  ```
  场外坐标 (9000,9000)：set_target=False、target=None、400 步后停在 (150,150) ✓
  对方安全区内部 (1500,100)：set_target=True、target=(1500.0, 1500.0) → 钳制到合法点 ✓
  ```
- **如果这里又出现 FAIL** → 说明静默夹紧的旧行为被改回来了：查 `set_target` 是否在赋值前
  用 `is_in_field()` 拒绝。

### 9.3 已修复：`LoadManager` 伤员混装漏判（顺序相关）——**已确认修复**

- **曾现象**：`can_load_batch([普通, 伤员])` → `NONE`（放行），而
  `can_load_batch([伤员, 普通])` → `INJURED_MULTI`。即"伤员+普通混装"能否被抓到
  **取决于列表顺序** → 直接违规丢分（赛项硬规则：伤员必须单独转运）。
- **现状**：已修复。复测四种组合（首趟/普通趟 × 两种顺序）均为 `INJURED_MULTI`，
  `decision` 模块由 FAIL 转 **PASS**。
- **现场动作**：无需动作。若你看到 `decision` 报 FAIL 的"伤员混装"项，说明这个修复被回退了
  —— 那是**直接丢分项**，优先处理。

---

## 10. 静默故障专项自查（S-40 / S-01 / B8）——**不报错、只会悄悄丢分或悄悄卡死**

这三类缺陷**不会抛异常、不会停车、日志也看不出**，只会让分数悄悄变高、或让机器人悄悄卡在
原地。自检把它们做成了可判 FAIL 的断言，且**纯逻辑、无硬件也必须真跑真判（绝不 SKIP）**。

### 10.1 快速对照表

| 现场症状 | 对应缺陷 | 跑哪个模块 | 判 FAIL 说明什么 |
|---|---|---|---|
| 自评/日志说"这趟送了 3 个"，实际场上只少了 1 个 | **S-40 多目标假装载** | `--only accounting` | 车没去过的目标被记进了货舱 → 分数虚高（赛项只认真送达） |
| 车停在离目标很远的地方就开始"套取"、然后自称送到了 | **S-40 位姿复核** | `--only accounting` | 套取前没复核"车确实在该目标处"，隔着距离记账 |
| 已经套住目标、后退重试后货物却掉在场上 | **S-40 抬爪丢货** | `--only accounting` | 持有目标时后退抬了爪（抬爪=释放） |
| 比赛时间到后车还在朝场地里冲、或在安全区里打转 | **S-01 终场不停车** | `--only decision` | DONE 后没清导航目标 / 没发 STOP / 没退出主循环 |
| 一趟送不到头、之后再也开不出新趟（一直在一个目标附近来回） | **B8 场心误判** | `--only decision` | 场心附近的**无关**目标被判成"投放无效"，把运送途中的导航目标抢走了 |

一条命令同时看这三类：

```bash
PYTHONPATH=src python3 tools/hw_selftest.py --only accounting,decision
```

### 10.2 `accounting` 模块（新）——载荷台账一致性

三条检查与判定：

| 检查 | 构造 | PASS 判据 |
|---|---|---|
| A 台账 = 实装 | 本趟计划 3 个目标，车只开到第 1 个 | `load_manager.count == 1`、`_captured == 1`、货舱里**只有**那 1 个 id（旧 S-40 会记 3 个） |
| B 位姿复核 | 车在 (2400,2500)、目标在 (400,400)（相距 **2900mm**），强制进入 CAPTURING | **不记入装载**（count=0）且被打回 `APPROACHING` 重新对位 |
| C 持有不抬爪 | 已套住 1 个后触发后退重试 | `sleeve.raise_up()` 调用次数 **0**（抬爪=释放，会把货丢回场地） |

实测证据行（本机 `--mock`，纯逻辑所以无硬件也真跑）：
```
[A] 车只到过第 1 个目标 (2400,2500)：load_manager.count=1、_captured=1、
    _captured_ids=[1000]、target_ids={1000}、phase=TRANSPORTING
[B] 车在 (2400,2500)、目标在 (400,400)（相距 2900mm）、强制 CAPTURING → count=0、phase=APPROACHING
[C] 已持有 1 个目标后触发后退重试 → raise_up 被调用 0 次（应为 0）
```

**FAIL 时怎么处置**：
1. 打开 `src/rescue_robot/transport/transport_pipeline.py` 的 `CAPTURING` 分支，确认：
   ①`_capture_index` 是否**逐个**推进（不是永远取 `[0]`）；
   ②`_load_mgr.load()` 是否只在 `sleeve.lower()` **成功之后**才调用；③记账对象是
   `_captured`（实装）而**不是** `_current_targets`（整趟计划）。
2. 位姿复核看 `TransportPipeline.CAPTURE_RADIUS_MM`（默认 `150.0`）：套取前
   `dist > CAPTURE_RADIUS_MM` 必须打回 `APPROACHING`。现场若该值被调大（比如为了"好套一点"），
   A/B 两条检查就会 FAIL —— 这正是要抓的情况。
3. C 检查看 `_begin_retreat`：必须判 `self._captured` 非空时**只后退、不抬爪**。
4. **现场交叉核对**（最直接）：跑完一趟，数一遍场上少了几个目标，与日志里"已套取并记入装载
   N 个"对比；对不上就是台账在虚记。

### 10.3 `decision` 模块（增补）——终场停车 + 场心误判

| 检查 | 判据 | FAIL 含义 |
|---|---|---|
| B8-1/B8-2 无关目标靠近场心 | `_check_invalid_transport()` 返回 **False** | 判据太宽 → 运送途中导航目标被抢走，那一趟永远送不到 |
| B8-3 已送达目标回到场心 | 返回 **True** | 漏判"投放无效" → 会一直以为已送达、不再补送 |
| S-01-1 源码护栏 | `_run_once` 源码里含 `StrategyState.DONE` / `clear_target` / `_stop_chassis` / `_stop_event.set()` | 终场分支被改回去了 |
| S-01-2 行为验证 | DONE 后：`clear_target` 被调 +1 次、`send_stop` ≥1 次、`send_velocity(0,0)` ≥1 次、停止事件已置位 | 比赛时间到后车还在冲 |

实测证据行：
```
[B8-1] 没有已送达目标时、场心附近有无关目标 → 判无效=False（应 False）
[B8-2] 无关目标就在场心附近 → 判无效=False（应 False）
[B8-3] 我们送达过的目标回到场心 → 判无效=True（应 True）
[S-01-1] _run_once 源码护栏：缺少 无（应全都有）
[S-01-2] DONE 后：clear_target +1 次、send_stop 1 次、send_velocity(0,0) 1 次、退出主循环=True
```

**FAIL 时怎么处置**：
- **S-01**：`AutonomousState._run_once` 的 DONE 分支必须依次做
  `navigation.clear_target()` → `_stop_chassis()`（内部 `send_velocity(0,0)` + `send_stop`）
  → `_stop_event.set()` → `return`。少任何一步，比赛结束时车都会继续以 50Hz 下发上一帧速度。
  真机复现：手动把 `decision.strategy_state` 弄成 DONE（或等时间耗尽），看车是否立刻停。
- **B8**：`DecisionEngine._check_invalid_transport` 必须**只遍历 `_delivered_ids`**
  （我们送达过、又变回 ACTIVE 且出现在场心），不能遍历所有 ACTIVE 目标。旧实现就是遍历全场
  → 决赛 25 个目标时常态误触发。

### 10.4 故障注入验证（证明这两条真能判 FAIL）

改坏代码后重跑，必须变 FAIL（本机实测）：

```
# ① 让记账成批写入（复现旧 S-40）
_captured = list(targets)  →  accounting: [FAIL] 台账数与实装数不一致：
                              load_manager.count=1 但实际套住 4 个 → 分数会虚高
# ② 关掉位姿复核
CAPTURE_RADIUS_MM = 1e9    →  accounting: [FAIL] 车距目标 2900mm 却把目标记入了装载
# ③ 摘掉终场分支
_run_once = lambda self, dt: None
                           →  decision:   [FAIL] 终场不停车：决策引擎已 DONE 但导航目标没清
```
每条 FAIL 都带「↳ 处置」，直接告诉现场下一步查哪个文件/哪个常量。
