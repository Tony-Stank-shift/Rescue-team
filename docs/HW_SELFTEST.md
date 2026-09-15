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
| `--list` | 列出 13 个模块名与覆盖内容 | 忘了模块名时 |
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
▶ servo — 夹爪舵机（RAISE/LOWER/HOLD/ANGLE 0~70 + 越界保护）
   ✅ [PASS] 夹爪舵机正常（动作命令、0~70° 全行程、越界保护均正确）
        · SERVO,RAISE → ACK,SERVO,RAISE
        · SERVO,ANGLE,70 → ACK,SERVO,ANGLE,70
        · SERVO,ANGLE,71 → ERR,SERVO_ANGLE
        ↳ 处置：...
```

| 标记 | 含义 | 现场动作 |
|---|---|---|
| ✅ **PASS** | 该模块实测通过 | 不用管 |
| ❌ **FAIL** | 该模块真的有问题（真机模式下设备打不开也算 FAIL，因为那就是故障） | 照该模块的「处置」逐条做 |
| ⏭️ **SKIP** | 因**环境/参数**未测（`--mock` 没硬件、动电机没给 `--yes-motion`） | 看是不是正常跳过；该测的没测就补参数重跑 |

**结论行**（汇总表最后两行）：

```
  PASS=6  FAIL=4  SKIP=3
  ❌ 结论：故障模块 = serial, telemetry, servo, camera
```

- 有 FAIL → 先修 FAIL 的模块，**不要**先看 SKIP。
- 全 SKIP 且 `--mock` → 正常（本机没硬件），代码链路已自检。
- `SKIP=2` 里若是 `odometry/motors/velocity` → 那是"动电机需要 `--yes-motion`"，
  架好轮子重跑即可。

---

## 3. 13 个模块：测什么 / 判定 / 失败怎么办

排障顺序即模块执行顺序（从纯算法 → 链路 → 硬件）：
`ranging → decision → navigation → transport → serial → telemetry → start_button →
servo → odometry → motors → velocity → camera → vision`

| 模块 | 测什么 | PASS 判定 | FAIL 典型原因与处置 |
|---|---|---|---|
| `ranging` | 视觉测距（底边+倾角地平面法）公式回环 + 倾角敏感性 | 200~3000mm 回环误差 <1% | 公式/参数接线错；`config.Camera` 与 `CVDetector` 参数不一致。**注意**：这只是公式自检，真实距离仍必须标定 `TILT_DEG` |
| `decision` | 载规则：首次单独 1 个普通物资 / 单次 ≤3 / 伤员必须单独 / 危险目标必须拒绝 / 空批次不崩 | 全部规则判定正确 | 违规项**直接丢分**：查 `LoadManager.can_load_batch` 的判定顺序（历史 bug：`[普通,伤员]` 会漏判成 NONE）。另：`start_trip` 的 `if not self.is_idle:` 恒真，见 `docs/audit/CODE_AUDIT.md` S-02 |
| `navigation` | 从出发区走到对角目标 / 异常目标不崩 | 能到达（残距 <80mm）且不抛异常 | 未到达：看 A*/纯追踪/到点判定；`BLOCKED` 看避障。**已知缺口**：目标落在对方安全区时会被静默改到 (1500,1500) 并报 ARRIVED（残距可达 1372mm）——该项判定待补 |
| `transport` | 转运状态机全流程（Mock 夹爪）：APPROACHING→CAPTURING→TRANSPORTING→PLACING→COMPLETE | 流程走完 + 显式停车回调被调用 + 无 VIOLATION | 卡在中间阶段：看到点阈值/推入步骤；VIOLATION：载规则或投放判定。**已知**：S-40 多目标只到 1 个就全记账（见 CODE_AUDIT） |
| `serial` | 串口能否打开 + `PING→PONG` + `START→ACK,START` | 两者都收到 | 见 §4 串口四分类；能开但无 PONG → 下位机没跑/只接单向/未共地 |
| `telemetry` | ODOM 8 字段@20Hz、IMU 10 字段@50Hz、TEL 字段数、数值合理性（az≈1000mg）、解析器一致性 | 帧前缀/字段数/频率/数值都对 | 见 §4；0 行：下位机没发数据或波特率不符 |
| `start_button` | 一键启动：软件层 `read_button` 是否认得 `EVENT,START_BUTTON`；硬件层需 `HW_SELFTEST_INTERACTIVE=1` 人工按键 | 软件层识别成功 | 识别不了 → 现场按按钮不会进 AUTONOMOUS（查 `SerialChassis.read_button` 匹配前缀） |
| `servo` | RAISE/LOWER/HOLD 各自 ACK；ANGLE 0/35/70 全部接受；71/180/-1 必须回 `ERR,SERVO_ANGLE` | 动作 ACK 正确 + 越界被拒 | 无 ACK：固件未实现或未 `START`；越界没被拒 → 有把舵机顶到限位的风险。角度语义：**0°=下压套住、70°=抬起释放** |
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

### 6.1 `--mock`（13 模块全跑，不崩）
```
  PASS=6  FAIL=0  SKIP=7
  ✅ 结论：未发现故障模块（部分模块因环境受限 SKIP）
```
`SKIP` 的 7 项：`serial/telemetry/servo/odometry/motors/velocity`（无串口权限）+ `camera`（无摄像头）。
`PASS` 的 6 项：`ranging/decision/navigation/transport/start_button/vision`（不依赖硬件的都真跑了）。

### 6.2 真机模式（不加 `--mock`，本机无权限/无摄像头 → 如实报 FAIL）
```
  PASS=6  FAIL=4  SKIP=3
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
- **相关**：目标落在**对方安全区内部**时，`nav.target` 会被静默改写成 (1500,1500)，
  570 步后报 **ARRIVED**，而实际位置 (1472,1472) 距真实目标 **1372mm**。
  现场表现为"车停在一个不是目标的地方且不再动"。用 `--only navigation --verbose` 看
  `nav.target` 是否被改过。

### 9.2 场外坐标被静默夹紧（A* 报成功，其实到不了）

- **现象**：`CostMap._to_grid` 会把超界坐标 clamp 进网格：`(9000, 9000)` → 网格 `(59,59)`
  ≈ 场内 `(2975, 2975)`。于是 `AStarPlanner.plan` 返回 **`success=True`**，而
  `ForbiddenZoneManager.is_in_field(9000,9000)` 明确返回 `False`，
  `is_safe()` 返回 `True`、`clamp_to_safe()` 也不做夹紧。
- **影响**：调用方以为"已规划到目标点"，实际车会跑到场内某个角落就停；**没有任何报错**。
  现场表现是"规划成功但车没到位/停在奇怪的地方"。
- **临时规避**（在 fixer 修掉 `_to_grid`/`set_target` 之前）：**调用方自己判**
  ```python
  if not nav.forbidden.is_in_field(tx, ty):
      ...  # 视为不可达：上报并停住，不要 set_target
  ```
- **状态**：已确认属实并转给 fixer（低优先级小改动）。自检的 `navigation` 模块目前
  **不判定这一条**（只判"不崩"），所以它现在会显示 PASS —— 别把它当"场外坐标没问题"。

### 9.3 已修复：`LoadManager` 伤员混装漏判（顺序相关）——**已确认修复**

- **曾现象**：`can_load_batch([普通, 伤员])` → `NONE`（放行），而
  `can_load_batch([伤员, 普通])` → `INJURED_MULTI`。即"伤员+普通混装"能否被抓到
  **取决于列表顺序** → 直接违规丢分（赛项硬规则：伤员必须单独转运）。
- **现状**：已修复。复测四种组合（首趟/普通趟 × 两种顺序）均为 `INJURED_MULTI`，
  `decision` 模块由 FAIL 转 **PASS**。
- **现场动作**：无需动作。若你看到 `decision` 报 FAIL 的"伤员混装"项，说明这个修复被回退了
  —— 那是**直接丢分项**，优先处理。
