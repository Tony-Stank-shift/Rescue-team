# 固件改动留底（2026-09-16 真机联调）

> **为什么有这份副本**：STM32 固件工程位于**开发机的 Windows 目录**
> （`C:\Users\Tony Huang\rescue_f103c8`），**不在本仓库内、也没有版本控制**。
> 而本轮在固件里修的两个问题都是**"整机变砖"级别**的：
> 一旦那份工程丢失或被旧版本覆盖，症状（**串口一个字节都没有 / 车完全不受控**）
> 会原样复发，且极难重新定位。
>
> 因此把**改过的两个源文件**原样存档在此，作为留底与对照物。

---

## 这两个文件改了什么

### `command.c` —— RX 健康看门狗（修「接收静默死亡」）

**问题**：遥测 TX 一路正常，但对 `PING`/`START`/`VEL`/`SERVO` **一律不回**，
持续可达 6 秒以上。成因是 STM32 HAL 在 ORE（溢出）处理里走 `UART_EndRxTransfer()`，
它会清掉 `CR1` 的 `RXNEIE/PEIE` 与 `CR3` 的 `EIE` ——
**`EIE` 一清，后续错误连 `HAL_UART_ErrorCallback` 都不再触发**，
于是 `RxState` 停在 `BUSY_RX` 且无任何中断推动状态机，接收永久失效。

**改动**：
1. 新增 `#define RX_HEALTH_PERIOD_MS 500U`
2. 新增 `static uint32_t rx_health_tick;`（含大段注释说明来龙去脉）
3. `Command_Update()` 内新增**无条件**重新武装块：
   ```c
   if ((HAL_GetTick() - rx_health_tick) >= RX_HEALTH_PERIOD_MS) {
       rx_health_tick = HAL_GetTick();
       command_uart->RxState = HAL_UART_STATE_READY;
       command_uart->ErrorCode = HAL_UART_ERROR_NONE;
       __HAL_UART_CLEAR_PEFLAG(command_uart);
       if (HAL_UART_Receive_IT(command_uart, &rx_byte, 1U) == HAL_OK)
           rx_restart_pending = false;
       else
           rx_restart_pending = true;
   }
   ```
   > ⚠️ **必须"无条件"**：第一版写成 `if (RxState != HAL_UART_STATE_BUSY_RX)` 才动手，
   > 而故障态**恰恰就是 `BUSY_RX`** → 判断直接跳过 → 等于没修。
4. `HAL_UART_ErrorCallback()` 内补：显式 `__HAL_UART_CLEAR_PEFLAG()` +
   清 `ErrorCode` + 强制 `RxState = HAL_UART_STATE_READY`。
5. `Command_Init()` 内补 `rx_health_tick = HAL_GetTick();`

### `main.c` —— IMU 失败不再致命（修「遥测一字节不发」）

**问题**：`if (MPU6050_Init(&hi2c2) != HAL_OK) Error_Handler();`
而 `Error_Handler()` 是 `__disable_irq(); while(1){}` 的**永久死循环**，
且 **`Telemetry_Init()` 排在它后面** → 只要 IMU 没接好 / 上电时序不巧
（`WHO_AM_I != 0x68`），固件就卡在初始化，**遥测永远发不出来**，
上位机侧表现为"串口能打开但完全没数据"，极易被误判为 RDK 串口或接线故障。

**改动**：把该分支换成
```c
(void)MPU6050_Init(&hi2c2);
```
并附大段说明。**降级是安全的**，因为下游本来就做好守卫：
- `MPU6050_Update()` 有 `if (!initialized) return HAL_ERROR;`
- `MPU6050_GetLatestSample()` 在 `sample_valid=false` 时返回 false
- `Telemetry_Update()` 的 IMU 段仅在 `GetLatestSample` 为真时才发 → IMU 行自动跳过

即：没有 IMU 时定位退化为纯里程计（无陀螺仪修正），但机器人不再变砖。

---

## 怎么应用 / 重新编译烧录

1. 把本目录的 `command.c` / `main.c` 覆盖回
   `C:\Users\Tony Huang\rescue_f103c8\Core\Src\` 下同名文件
   （或只把上面描述的两处改动手工合入）。
2. 编译（工程用 STM32CubeCLT 的 ninja）：
   ```bash
   cd "C:/Users/Tony Huang/rescue_f103c8/build/Debug"
   "C:/Users/Tony Huang/AppData/Local/stm32cube/bundles/ninja/1.13.2+st.1/bin/ninja.exe"
   ```
   期望：`FLASH 约 57%`，产物 `rescue_f103c8.elf`
3. 烧录（ST-Link 接在 PC 上、且未被 `usbipd` 直通给 WSL）：
   ```bash
   "C:/Users/Tony Huang/AppData/Local/stm32cube/bundles/programmer/2.23.0/bin/STM32_Programmer_CLI.exe" \
       -c port=SWD -w 'C:\Users\Tony Huang\rescue_f103c8\build\Debug\rescue_f103c8.elf' -v -rst
   ```
   期望末尾出现 `Download verified successfully` + `MCU Reset`。
4. 复位后验证串口恢复发数据：
   ```bash
   # 在 RDK 上
   timeout 3 head -c 400 /dev/ttyS1 | wc -c     # 应远大于 0（约 84 行/秒遥测）
   ```

> **注意**：本目录只存了改动的两个 `.c` 文件，**不能单独编译**（依赖工程其余部分）。
> 完整排查记录见 `docs/audit/REAL_MACHINE_DEBUG_20260916.md`。
