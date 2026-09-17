# STM32 下位机固件（rescue_f103c8）

> **本目录是什么**：STM32F103 下位机固件的**工程自有代码**（队员们自己写的那部分），
> 2026-09-16 从开发机的 SolidWorks/CubeMX 工程目录原样收录进仓库，**纳入版本控制**。
>
> 之所以要收录：此前固件只存在于开发机的 Windows 目录
> （`C:\Users\Tony Huang\rescue_f103c8`），**没有版本控制**。而 2026-09-16 真机联调
> 在固件里修的两个问题都是**"整机变砖"级**的 —— 那份目录一旦丢失或被旧版本覆盖，
> 症状（**串口一个字节都没有 / 车完全不受控**）会原样复发且极难重新定位。

---

## 1. 目录内容

```
firmware/
├── Core/                      ← 工程自有代码（全部）
│   ├── Src/                   15 个 .c
│   │   ├── main.c             初始化 + 主循环
│   │   ├── command.c          串口命令解析 / 一键启动按钮 / **RX 健康看门狗**
│   │   ├── telemetry.c        ODOM 20Hz + IMU 50Hz + TEL 5~10Hz
│   │   ├── odometry.c         轮式里程计（中点航向积分，x=前向 y=左向）
│   │   ├── chassis.c  motor.c  encoder.c  speed_control.c  servo.c  mpu6050.c
│   │   └── stm32f1xx_hal_msp.c  stm32f1xx_it.c  syscalls.c  sysmem.c  system_stm32f1xx.c
│   └── Inc/                   13 个 .h
├── CMakeLists.txt            构建定义
├── CMakePresets.json         Debug / Release 预设
├── cmake/                   工具链文件
├── STM32F103XX_FLASH.ld     链接脚本（**假定 FLASH 64KB**）
├── startup_stm32f103xb.s    启动文件（xB = 128KB 型号）
└── rescue_f103c8.ioc        CubeMX 配置（引脚复用、外设、时钟的唯一来源）
```

### ⚠️ 本目录**不包含**、需要重新取得的两个目录

| 目录 | 体积 | 为什么不在仓库里 | 怎么取得 |
|---|---|---|---|
| `Drivers/` | 68M（CMSIS 62M + HAL 5.9M） | 第三方文件，体积大且可完全重新生成 | 用 **CubeMX** 打开 `rescue_f103c8.ioc` 重新生成一次 |
| `build/` | 25M | 编译产物 | 编译时自动生成 |

**其它缺失的文件（都是纯第三方/生成物，不影响源码完整性）**：
`.mxproject`、`.settings/`、`.cproject`/`.project`（CubeIDE 工程元数据，CubeMX 重新生成即有）。

---

## 2. 关键外设与引脚（改代码前必看）

| 外设 | 引脚 | 用途 |
|---|---|---|
| **USART1** | **PA9=TX, PA10=RX**, 115200 8N1 | 与 RDK 的**唯一**通信口（命令 + 遥测） |
| I2C2 | PB10=SCL, PB11=SDA | MPU6050（`WHO_AM_I` 必须 `0x68`） |
| TIM1 CH1/CH4 | PA8=LPWM, PA11=RPWM | 左右电机 PWM |
| TIM2 / TIM3 | — | 左右编码器 |
| TIM4 CH1 | PB6 | 舵机 PWM（0°=1000µs 套住 / 70°=1778µs 释放） |
| **START_BTN** | **PB9**（上拉，拨到 ON 拉低） | 一键启动**自锁开关** |
| STATUS_LED | PC13 | 状态灯（**高电平点亮**，注意不是常见的低电平） |

---

## 3. 2026-09-16 真机联调对固件的两处修复 ⚠️

> **这两条都是"整机变砖"级别**，症状看起来都像硬件坏。
> 完整排查记录（现象/证据/根因）见 `docs/audit/REAL_MACHINE_DEBUG_20260916.md`。

### 3.1 `main.c` —— IMU 初始化失败不再致命

**问题**：
```c
if (MPU6050_Init(&hi2c2) != HAL_OK) { Error_Handler(); }   // ← 旧代码
```
`Error_Handler()` 是 `__disable_irq(); while(1){}` 的**永久死循环**，而
**`Telemetry_Init()` 排在它后面** → 只要 IMU 没接好或上电时序不巧
（`WHO_AM_I != 0x68`），固件就卡在初始化，**遥测一个字节都发不出来**，
上位机侧表现为"串口能打开但完全没数据"，极易被误判为 RDK 串口或接线故障。

**修复**：改为 `(void)MPU6050_Init(&hi2c2);`

**为什么降级是安全的**（下游本来就做好守卫）：
- `MPU6050_Update()` 有 `if (!initialized) return HAL_ERROR;`
- `MPU6050_GetLatestSample()` 在 `sample_valid = false` 时返回 `false`
- `Telemetry_Update()` 的 IMU 段仅在 `GetLatestSample` 为真时才发 → IMU 行自动跳过

即：没有 IMU 时定位退化为纯里程计（少了陀螺仪修正），但机器人不再变砖。

### 3.2 `command.c` —— 新增 RX 健康看门狗（修"接收静默死亡"）

**问题**：遥测 TX 一路正常，但对 `PING`/`START`/`VEL`/`SERVO` **一律不回**
（连 `ERR,FORMAT` 都没有），持续可达 **6 秒以上**。
成因：STM32 HAL 在 ORE（溢出）处理里走 `UART_EndRxTransfer()`，
它会清掉 `CR1` 的 `RXNEIE/PEIE` 与 `CR3` 的 `EIE`；
**`EIE` 一清，后续错误连 `HAL_UART_ErrorCallback` 都不再触发**，
于是 `RxState` 停在 `BUSY_RX` 且无任何中断推动状态机 → 接收永久失效，
**而 TX 完全不受影响，所以极难察觉**。

**修复**：`Command_Update()` 内每 `RX_HEALTH_PERIOD_MS = 500ms`
**无条件**强制 ①拉回 `READY` ②清 `ErrorCode` 与 ORE ③`HAL_UART_Receive_IT` 重新武装。

> ⚠️ **必须"无条件"**：第一版写成 `if (RxState != HAL_UART_STATE_BUSY_RX)` 才动手，
> 而故障态**恰恰就是 `BUSY_RX`** → 判断直接跳过、等于没修。
> 代价是极小概率丢掉"恰好在这一刻到达的 1 个字节"——
> VEL 以 50Hz 下发丢 1 帧无影响，PING/START 等有重试，远小于"整机收不到指令"的代价。

**实测效果**：6 秒死窗口消失；18 次探测成功数从 7 升到 13，最长连续失败 3 次 → 2 次。

---

## 4. 编译与烧录

### 4.1 编译（用 STM32CubeCLT 的工具链）

**上次真机联调用的就是下面这条**（绝对路径，原样可用）：

```bash
cd "C:/Users/Tony Huang/rescue_f103c8/build/Debug"
"C:/Users/Tony Huang/AppData/Local/stm32cube/bundles/ninja/1.13.2+st.1/bin/ninja.exe"
```
期望输出末尾：`FLASH 约 57%`，产物 `rescue_f103c8.elf`。

> 换了机器/用户名的话，工具链路径以 `build/Debug/CMakeCache.txt` 里的
> `CMAKE_C_COMPILER` / `CMAKE_MAKE_PROGRAM` 为准。

### 4.2 烧录（ST-Link 接在 PC 上）

```bash
"C:/Users/Tony Huang/AppData/Local/stm32cube/bundles/programmer/2.23.0/bin/STM32_Programmer_CLI.exe" \
    -c port=SWD -w 'C:\Users\Tony Huang\rescue_f103c8\build\Debug\rescue_f103c8.elf' -v -rst
```
期望末尾：`Download verified successfully` + `MCU Reset`。

> `-v` 会逐字节校验写进去的镜像与 `.elf` 一致 —— 这是"**单片机已同步**"的证据。

> ⚠️ 若 ST-Link 曾被 `usbipd attach` 直通给 WSL，Windows 侧会看不到它 →
> 需先 `usbipd detach --busid <id>`（**需要管理员权限**）。

### 4.3 验证固件真的在跑（在 RDK 上）

```bash
# 应当持续刷出 ODOM/IMU/TEL（约 84 行/秒）
timeout 3 head -c 400 /dev/ttyS1 | wc -c        # 远大于 0
```
或直接跑上位机自检：
```bash
cd ~/rescue && PYTHONPATH=src python3 tools/hw_selftest.py --only serial,telemetry,servo
```

---

## 5. 协议契约（固件必须遵守）

完整协议见仓库根的 `chassis_serial_protocol.md`。上位机依赖、**已被现场验证有问题**的两点：

| 项 | 协议说 | 固件实际 | 上位机怎么应对 |
|---|---|---|---|
| `START` 清零里程计 | 收到 START 后清零局部里程计为 (0,0,0) | ⚠️ **实测并不会**（发 START 前后 ODOM 与编码器累计值**逐位相同**） | `ChassisInterface.set_odom_baseline()` 自己记基线并减掉 |
| 一键启动事件 | 拨开关发 `EVENT,START_BUTTON` | 下位机**已 `RUNNING`** 时只发 `EVENT,BUTTON_LED_ON` | 上位机**两个事件都认**（`read_start_request()`） |

> 若将来决定在固件侧修正这两点（让 `START` 真的清零、让事件语义统一），
> 上位机的应对逻辑可以保留（幂等），但**必须回归真机验证**。
