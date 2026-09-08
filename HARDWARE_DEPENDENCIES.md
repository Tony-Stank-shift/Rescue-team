# 待定硬件依赖清单

> 软件负责人用。记录「硬件/传感器」确定状态及其对软件的影响。
> 状态标记：✅ 已确定 · 🔶 部分确定 · ❌ 未确定/未购买 · ❓ 待确认

---

## 0. 主控平台 ✅（已确定：地平线 RDK X5）

**结论**：主控用**地平线 RDK X5**（旭日 X5，10 TOPS，8 核 A55，2 路 MIPI + 4 路 USB3.0）。

**完整链路**：
```
RDK X5(上位机: Python+OpenCV+串口)
   ├─ USB 摄像头（800W 77°，cv2.VideoCapture）
   └─ TTL 串口(UART) ── STM32(F103: USART1 PA9/PA10) 底盘
```

**对软件的影响**：
- 软件栈 Python + OpenCV + pyserial，RDK X5 可运行（地平线支持）。
- 串口设备：RDK X5 的 UART 设备文件（`/dev/ttyS0`？待确认对应 UART 号）。
- 摄像头：USB 免驱，插 RDK X5 USB 口，`cv2.VideoCapture` 读。
- ⚠️ 部署差异：GPIO 用 RDK 驱动（非 RPi.GPIO）、串口/摄像头 index、系统依赖（opencv-python、pyserial、pyyaml）。

**待确认**：RDK X5 上接 STM32 用的 UART 号 / 设备文件；USB 摄像头在 RDK 上的 index。

---

## 1. 转运机构方案 ✅（已确定）

**结论**：套筒**套住目标后贴着地面推**（不是抓取放上车）；驱动用**舵机 SG90**。

**舵机参数**：SG90 舵机，行程 **0°~90°**（平行地面 ⇄ 垂直于地面），夹爪臂长约 **90mm**；机构采用「从上往下套住目标再贴着地面推」。

**对软件的影响**：✅ 合规。`transport/sleeve_lift.py` 的 RAISED/LOWERED/HOLD 抽象可适配（需按 SG90 角度控制实现）。

---

## 2. 摄像头 ✅（已采购）

**结论**：**800 万像素，77°（无畸形），自动对焦，USB 接口**。安装高度约 **21cm**（俯视前方地面）。

**对软件的影响**：
- OpenCV `cv2.VideoCapture(0)` 直接读，无需特殊驱动。
- 77° 水平无畸变，适合小目标形状识别。
- ⚠️ 建议关闭自动对焦（固定距离场景，定焦更稳）。
- 定位估算使用 `camera_height_mm = 210`（已更新 `detection.py`）。

**待提供**：内参标定数据。

---

## 3. 定位方案 ✅（已确定：编码器里程计 + IMU yaw 校正）

**结论**：
- **位置 (x, y)**：编码器里程计（两差速 + 万向轮），由底盘层负责，精度够用。
- **朝向 (theta)**：用 **IMU 陀螺仪做 yaw 校正**（修正轮式里程计的朝向漂移；打滑/碰撞时也能提供真实角速度）。
- 里程计由底盘层实现，上位机只拿位姿并做融合。

**IMU 选型**：**MPU6050 模块**（沉金工艺、焊直针向下），**I2C** 接口，3.3V 供电。

**接线（4 线 I2C）**：
```
MPU6050 VCC → 3.3V   （⚠️ 勿接 5V）
MPU6050 GND → GND
MPU6050 INT  → STM32 PB5 (MPU6050_INT)
MPU6050 SCL  → STM32 PB10 (I2C2 SCL)
MPU6050 SDA  → STM32 PB11 (I2C2 SDA)
```
> ✅ **确认接 STM32**（下位机负责底盘 + 传感器）；上位机做融合时只需通过串口拿到带 yaw 校正的里程计。

**对软件的影响**：融合逻辑用 `localization.py` 现有 `OdometryLocalizer` 的「里程计 + IMU 互补滤波」思路；上位机负责最终融合。

---

## 4. 电机 + 轮子 ✅（已确定）

**已确定**：
- 电机：**mg370**（带 GMR 编码器减速电机），减速比 **1:34**，额定 **12V**，**2 个**（两差速 + 一万向轮）。
- 轮子：驱动轮 **φ65mm**，万向轮 **φ25mm**（铜柱垫高保证底面水平）。
- 轮距：**209mm**（两驱动轮中心线）。

**对软件的影响**：已更新到代码（`sim_2d.py` 轮距 209 / 减速比 34，`motion_control.py`、`config_loader.py`、`hardware_profile.py`、`robot.default.yaml` 的 wheel_base_mm=209）。

> 编码器线数、速度闭环、里程计积分均由底盘层负责，上层不涉及。

---

## 5. 一键启动按钮 / LED / 蜂鸣器 ⚠️

**状态**：目前都没有。

**⚠️ 赛规提醒**：规则要求「机器人上须有明确标识的一键启动按钮」。硬性要求，建议补买（至少一键按钮 + 一个状态灯）。

**对软件的影响**：`button.py`/`indicator.py` 已就绪（Mock 可用），接真实元件时需改 RDK 的 GPIO 接口并填引脚。

---

## 6. 通信方案 ✅（已确定：TTL 串口，上位机↔STM32，协议 v1）

**协议**：依据 `chassis_serial_protocol.md`（v1）。

**结论**：
- 下位机底盘板：**STM32F103**（丝印 Rescue-F103；具体型号/封装待确认），串口用 **USART1**（TX=PA9，RX=PA10）。
- 波特率：**115200**，8/无/1/无流控，ASCII，逗号分隔，`\r\n` 结尾。
- 上行（上位机→下位机）：`PING`(→PONG)、`START`(→ACK,START，清里程计)、`VEL,v_mm_s,w_mrad_s`、`STOP`(→ACK,STOP)、`ESTOP`(→ACK,ESTOP)。
- 下行（下位机→上位机）：`ODOM,x_m,y_m,theta_rad,encL,encR,vL_m_s,vR_m_s`（**严格 8 字段**）；`EVENT`/`ERR`/`ACK`/`TEL` 只记录不解析。

**编码器/轮速**：编码器每轮 **68028 count/rev**（已含减速比+四倍频）；最大轮速 **250 RPM**（1 RPM = 3.403392 mm/s）。

**对软件的影响**：✅ 已写 `hardware/serial_chassis.py`（严格 ODOM 解析 + PING/START/VEL/STOP/ESTOP + `start_match` 启动流程）+ `hardware/chassis_interface.py`（坐标转换）。`sim_2d.py` 编码器/轮速参数已更新为 68028 / 250 RPM。

**❓ 还差**：RDK 端的 UART 号/设备文件（`/dev/ttyS0`？）。

**调试链路**：电脑(USB) → USB-TTL 模块 → STM32(USART1: PA9=TX, PA10=RX)。

---

## 6b. 下位机底盘引脚（STM32F103，参考）

- 左电机 PWM：PA6（LPWM）；右电机 PWM：PB5（RPWM）
- 左编码器 A/B：PE9/PE11（LGMRA/LGMRB）；右编码器 A/B：PD12/PD13（RGMRA/RGMRB）
- 电流/方向采样：PA1/PA2（LAIN1/LAIN2）、PE0/PE1（RAIN1/RAIN2）
- SWD 调试：PA13/PA14

---

## 7. 电池 ✅（已确定）

**结论**：**12V / 2500mAh**。

**对软件的影响**：现有阈值 11.0~12.6V 基本适配。

**❓ 可选确认**：电池类型（3S 锂电 / 其他）以精确校准低电量阈值。

---

## 8. 辅助传感器 ✅（已确定：无）

**结论**：没有超声波/温度/振动传感器。自检温度项为非关键，可跳过。

---

## 9. 上下层接口约定 ✅（已确定）

**下位机 → 上位机（提供）**：编码器原始计数、左右轮速度、纯轮式里程计 `(x, y, theta)`。
**上位机 → 下位机（下发）**：
- 速度指令：串口 `VEL,v_mm_s,w_mrad_s`（v 整数 mm/s，w 整数 mrad/s；上位机内部 v 用 m/s、w 用 rad/s，下发时换算）。
- 启动：串口 `START`（比赛启动发一次，下位机清零里程计为 (0,0,0)）。

**坐标约定**：x 向前、y 向左、theta 逆时针为正。

**分工**：下位机负责差速换算、限速、左右轮 PID；上位机负责传感器融合 + 全局位姿 + 地图初始偏移。

**⚠️ 与上层代码的差异（已由 `hardware/chassis_interface.py` 适配）**：
- 单位：m ↔ mm（×1000）
- 前方：+X ↔ +Y（坐标旋转 90°）
- 适配层 `ChassisInterface` 负责：`odom_to_upper()`（里程计→上层 mm 位姿）、`velocity_to_command()`（速度→VEL 命令）、初始偏移管理。

---

## 当前结论

- ✅ 已落地：转运（推）、定位（编码器里程计）、电机/轮子参数（轮距 209/减速比 34 已写入代码）、通信方案（串口，波特率 115200）、摄像头（已购，USB IMX219）、主控（RDK）。
- ❓ **还差的**（等你晚点给）：
  1. 套取舵机行程/参数
  2. RDK 端 UART 号（`/dev/ttyXXX`）
  3. 主控具体型号（X3 还是 X5）
