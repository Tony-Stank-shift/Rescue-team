# 全量问题清单（软件负责人视角，结项前最后一遍通读）

> **本文档只记录问题，不含任何修复动作。** 所有条目都来自**逐行读代码 + 只读实跑**，
> 每条给 `文件:行` 与证据。凡未证实的明确标注"未证实"。
>
> 来源：队长自读（核心路径：main / states / transport / load_manager / safe_zone_placer /
> deploy.sh / tools）+ 4 个并行深读（感知层 P-xx、决策层 D-xx、导航层 N-xx、硬件通信层 H-xx）。
>
> **分级口径**
> - **T0 整场级**：真机必现 → 整场 0 分或撞车/退赛；**且集成仿真测不出来**
> - **T1 丢大量分**：单趟到十几分钟的分数损失，或让机器人卡在一个目标上
> - **T2 中等**：特定条件下丢分/低效/现场排障困难
> - **T3 低/死代码**：不直接丢分，但是陷阱或误导
>
> ⚠️ **T0 里前 4 条里有 3 条是本轮修复"激活"的潜伏缺陷，不是原有缺陷**
> （D-02 由 S-11 修复激活、D-03 由队长 N-1 修复引入、N-01 由队长 U4 修复激活）。
> 这说明一件重要的事：**"仿真全绿"不能作为真机可用性的依据**——仿真与真机的喂参差异
> 掩盖了整场级缺陷（详见每条的"为什么仿真测不到"）。

---

# T0 整场级（真机必现，仿真测不到）

## T0-1 ✅【已修改】空地图提前 WAIT 打断"已套住目标"的运送相 → 一趟都送不到
- **状态：已修改**（空地图判定加 `grip_done/release_done` 门控；实测：携带目标 → `TRANSPORT_TO`，空手空地图 → 仍 WAIT 不误终场）
- 位置：`decision/decision_engine.py:257-283`（空地图判定在状态机分派**之前**无条件 return）+ `states/autonomous_state.py:416-420`（套住后调 `mark_being_transported`）+ `perception/world_map.py:127-130`（`active_targets` 只算 ACTIVE）
- 机制：目标被套住 → 标 `BEING_TRANSPORTED` → 不再计入 `active_targets`。若它是场上**最后一个** ACTIVE 目标（开局只识别到 1 个，或清场时最后一趟），`update()` 在分派前就 return WAIT，**永远到不了 `TRANSPORT_TO`**。
- 证据（队长实跑，真机流程）：
  ```
  帧1 有目标: action=NAVIGATE_TO   active=1
  帧2 到达:   action=GRIP
  套住后      active_targets = 0  （该目标状态=BEING_TRANSPORTED）
  帧3/4/5:    action=WAIT  detail=场上暂无目标：保持运行，等待重新检测
  ```
  期望 `TRANSPORT_TO`（去安全区投放），实际 WAIT → 车带着目标停住。
- **为什么仿真测不到**：`integrated_sim` 从不调用 `world_map.mark_being_transported`（全仓只有 `autonomous_state.py:363` 调用），仿真里 `active_targets` 在运送期间始终非空。
- 影响：**真机最坏 0 分**（开局只建 1 个目标 → 首趟永远完不成 → 按赛规后续全部无效）；一般情况是"每趟都送不进去"。
- 建议方向（不实施）：把"空地图"判定移进各状态处理内部；或门控为 `not grip_done and not release_done` 时才 WAIT。

## T0-2 ✅【已修改】卡死判据把同一个量当两个量比 → 正常行驶 5 秒被判"卡死"
- **状态：已修改**（改用两路独立信息：`commanded_velocity` 下发速度 vs 里程计单帧位移；新增 `STUCK_CMD_MIN_MM_S=50` / `STUCK_STEP_MIN_MM=1.0`；`commanded_velocity=None` 时跳过检测。实测：正常行驶 8s 不误判 ✅ / 真卡死能判出 ✅ / 不给下发速度时跳过 ✅。调用链已接线：`autonomous_state._last_cmd` → `decision.update(commanded_velocity=…)` → `anomaly.check`)
- 位置：`decision/anomaly_handler.py:189-205`（`if speed > 100 and dist < STUCK_DISTANCE_MM`）+ `states/autonomous_state.py:525-531`（`_last_velocity = step_mm/dt`，`step_mm` 就是上一帧位姿差）
- 机制：`speed` 用上一帧位移算，`dist` 算当前帧位移——**同一个量的相邻两帧**。于是 `speed>100 且 dist<30` ⟺ 每帧位移落在 (2,30)mm，而 `dt=0.02` 时这正好是 **100~1500mm/s 的正常行驶区间**。
- 证据（队长实跑，真实墙钟，按 autonomous_state 的喂法传真实速度）：
  ```
  t=5.03s 判为 STUCK，detail='卡死 5.0s'，recovery=ESCAPE_MANEUVER
  车实际一直前进：5.03s 走了 1878mm（=373mm/s）
  ```
- **为什么仿真测不到**：`integrated_sim` 不传 `velocity`（`decision_engine.py:242` 只在 `velocity is None` 时 `notify_action()`）→ 整个速度分支被跳过 → 仿真里看门狗**永不触发**。
- 影响：开车约 5 秒触发一次误判 → 与 T0-3 级联 → **整场报废**。（修复前调用方传字面量 `(0,0)`，`speed>100` 永不成立，所以这是"修 S-11 时激活的潜伏缺陷"。）
- 建议方向：用两路独立信息（下发的 `cmd` 非零 / 底盘轮速）对比里程计位移，而不是位姿差比位姿差；`dist` 改滑动窗口累计位移。

## T0-3 ✅【已修改】ANOMALY 状态进得去出不来 → 异常过后决策层永久返回 WAIT
- **状态：已修改**（① 进入 ANOMALY 前记住原策略状态，异常消失后归还——首趟未完成则回首趟，不绕闸门；② `ANOMALY_MAX_S=8s` 超时强制恢复；③ 分派 `else` 兜底不再永久 WAIT，改为恢复并打 ERROR。实测：异常消失 50 帧后 state=FIRST_TRIP/action=NAVIGATE_TO ✅、首趟未完成恢复到 FIRST_TRIP ✅、持续异常 10s 后强制恢复 ✅）
- 位置：`decision/decision_engine.py:236-238`（置 ANOMALY）、`:286-300`（分派）、`:299-300`（`else: WAIT "未知状态"`）
- 机制：任何异常置 `_strategy_state = ANOMALY`，恢复函数**从不还原**原状态；异常消失后分派链全不匹配 → 落到 `else` 永久 WAIT。全仓没有能把状态写回 FREE_RUN 的分支（`_handle_forced_reset` 无调用方）。
- 证据（队长实跑）：
  ```
  异常前:        state=FIRST_TRIP  action=NAVIGATE_TO
  异常帧:        state=ANOMALY     action=NAVIGATE_TO
  异常消失 50 帧后: state=ANOMALY  action=WAIT  detail=未知状态   （剩余 180s）
  ```
- 影响：与 T0-2 级联 = 约 5 秒后整场只剩"15s 一次的保命绕圈"，再也开不出新趟、再也不会投放 → 0 分。
- 建议方向：进入 ANOMALY 前存 `_prev_strategy_state`，异常消失后归还（**不能**统一回 FREE_RUN，否则绕过首趟闸门）；另加 ANOMALY 超时兜底。

## T0-4 ✅【已修改】减速带模式反复重入 → 每轮 3 秒直线冲、全程不看目标
- **状态：已修改**（① 退出条件由**墙钟**改为**里程** `BUMP_TRAVEL_MM=450mm` + `BUMP_MAX_S=8s` 兜底；② 加"本次已完成"闩锁，离开减速带区才解除，同一次越障不再重入；③ 越障期间保留**限幅**朝向修正 `BUMP_MAX_ANGULAR_RAD_S=0.6`，朝当前目标温和纠偏。实测：进入 bump 次数 反复重入 → **1 次**；越障 452mm 后恢复正常导航，距目标 1442mm → **122mm**，随后收敛到 39mm；关掉 `near_speed_bump` 后闩锁自动解除 ✅)
- 位置：`navigation/navigation_pipeline.py:206-216`、`navigation/motion_control.py:235-257`
- 机制：退出条件 `_bump_count >= 3` 是**墙钟计时**，而 `enter_bump_mode()` 会把计时器清零 → 只要 `near_speed_bump` 仍为 True，退出后**下一帧立刻重入**、计数归零 → 又一个 3s 恒速直冲。bump 模式下 `angular=0`、不看目标、不看朝向。
- 证据（队长实跑，50Hz，near_speed_bump 恒 True，目标设在 (1500,1500)）：
  ```
  10 秒内进入 bump 模式 1 次后持续重入
  最终位姿=(300,2300)  距目标 1442mm
  → 一路直冲 2000mm，全程 angular=0、不看目标
  ```
- **为什么修复前是潜伏**：U4 之前 `near_speed_bump` 恒 False（没有调用方传值），这条路径永不进入。U4 把标志接上后它立刻变成可达。
- 影响：出发区前 3 条减速带区域会直线冲 600mm/轮（200mm/s×3s），可能撞围栏/骑上减速带/直入对方安全区（=比赛结束）；且计时与真实越障进度无关。
- 建议方向：按里程/位姿推进量判定退出 + "已通过"闩锁 + 最大时长兜底。

## T0-5 ✅【已修改】摄像头失败 → 感知降级为 MockDetector（伪造 20 个随机目标）→ 整场追幻影
- **状态：已修改**（`main.py` 不再重建为 `use_mock=True`；保持真实感知管线 → 无帧即空地图；`PerceptionPipeline` 新增 `set_vision_available(False)` / `vision_available` 显式故障标志 + 每 5 秒一次显式告警。实测：视觉不可用+无帧 → 活跃目标 0 个、检测器仍是 CVDetector（非 MockDetector）✅)
- 位置：`main.py:277-280`（`perception = PerceptionPipeline(use_mock=True, ...)`）+ `perception/detection.py:444-541`（`MockDetector` 固定生成 20 个目标、每帧 ±2px 抖动）+ `perception/perception_pipeline.py:196-211`
- 机制：`MockDetector` 没有 `estimate_ground_position` → 走兜底 `estimate_position`，返回 `rel_y = 200+(1-cy/img_h)*800` → **永远在车前 200~1000mm、横向 ±500mm 摆动**的幻影目标。
- 影响：真机摄像头一掉线（或预热 3s 超时），机器人会一路导航+套取幻影目标，整场拿不到真实目标，且可能撞紫边/撞对手（判罚）。
- 建议方向：摄像头不可用应输出**空世界地图 + 显式故障标志**（进安全停车/重连），Mock 只允许出现在 CI/仿真入口。

## T0-6 ✅【已修改】`read_pose()` 每帧只读一行 → 接收缓冲持续积压，位姿越来越旧
- **状态：已修改**（新增共享接收缓冲 `_rx_buf` + 非阻塞 `_pump_rx()`/`_pop_line_buffered()`；`read_pose()` 改为排空缓冲、只保留**最后一条** ODOM；`_read_line()` 也走同一缓冲，保证 `wait_for`/`read_imu` 不丢数据；新增 `rx_lines_dropped` 诊断计数。实测：单次 `read_pose()` 取到最新 9900mm（而非旧的 100/200/300）✅、336/336 行全部消费、缓冲最终清空 ✅)
- 位置：`hardware/serial_chassis.py:209-225`（`read_pose`）、`:171-183`（`_read_line` 只 `readline()` 一次）、调用点 `states/autonomous_state.py:325`
- 机制：下位机按协议 ODOM **20Hz** + IMU **50Hz** + TEL **5~10Hz** = **75~80 行/秒**（`chassis_serial_protocol.md:471,300,461`），而上位机每帧只消费 **1 行**（50Hz → 50 行/秒）→ 每秒净积压 25~30 行。非 ODOM 行被直接丢弃且**不补读**。全仓 `reset_input_buffer` **0 处**。
- 证据：协议频率 + 代码单行读取（机制确证）。**滞后秒数取决于驱动缓冲大小（未实测）**，深读方用 4KiB 缓冲建模得 ~1.5s。
- 影响：控制回路用的是过期位姿（500mm/s 时误差量级 0.1~0.75m）→ 进安全区/贴围栏/套取对准全部按过期坐标执行。
- 建议方向：`read_pose()` 内循环读到 ODOM 或缓冲空（drain），并统计丢弃行数。

## T0-7 ✅【已修改】DEBUG 阶段 0.5s 轮询只读一行 → 一键启动事件极可能被丢
- **状态：已修改**（`read_button()` 同样改为排空缓冲，并在整批行里查找 `EVENT,START_BUTTON`。实测：模拟 75 行/秒混合流 + 中途注入一次事件 → 第 4 次轮询即找到 ✅)
- 位置：`main.py:359-363`（0.5s 轮询 `chassis.read_button()`）+ `serial_chassis.py:254-263`（同样只读一行）
- 机制：DEBUG 期间同样有 75~80 行/秒遥测，而轮询每 0.5s 只消费 1 行（=2 行/秒）→ 积压比 T0-6 严重 25 倍；`EVENT,START_BUTTON` 是**一次性事件**（协议 §5.1.1），被丢或被截断混进相邻行就永不匹配。
- 证据：代码行 + 与 T0-6 同一条量化路径。**"事件实际被丢"未直接实测**（深读方的注入实验被中断）。
- 影响：按了开关进不了 AUTONOMOUS → 0 分；即使成功也有 1.5~3s 延迟。
- 建议方向：等待期间排空缓冲后再匹配事件前缀；轮询周期降到 20~50ms。

## T0-8 ✅【已修改】`CAM_INDEX` 两处默认值不一致 + 摄像头是关键自检项 → 上电即 ERROR 退出
- **状态：已修改**（新增唯一入口 `get_camera_index()`，自检与采集共用，默认 0，非数字回退并告警。实测：CAM_INDEX 未设→0 / '1'→1 / 'abc'→回退 0 ✅)
- 位置：`main.py:76`（`RealHardwareChecker` 用默认 `"1"`）vs `main.py:257`（`CameraReader` 用默认 `"0"`）；`system_check.py:269`（摄像头 `critical=True`）→ `boot_state.py:52-61`（失败转 ERROR）→ `main.py:355-357`（break 退出）
- 证据（队长核对）：
  ```
  main.py:76   cam_idx = int(os.environ.get("CAM_INDEX", "1"))
  main.py:257  cam_idx = int(os.environ.get("CAM_INDEX", "0"))
  ```
- 影响：不设 `CAM_INDEX` 时自检开 index 1、采集开 index 0。单摄枚举在 0 的话 → **自检失败 → 上电即 ERROR → 程序退出**，表现为"启动不了"。唯一逃生口 `SKIP_CAMERA_CHECK=1`。
- 建议方向：统一默认值（抽 `get_camera_index()`），自检回退遍历 0..N。

## T0-9 ✅【已修改】`RUN_MODE` 默认 `mock` → 漏设环境变量就是"跑得很好但车不动"
- **状态：已修改**（默认改为 `RunMode.REAL`；当显式 `RUN_MODE=mock` 时打醒目 ERROR 横幅"不会驱动真实底盘、也不会读摄像头"。原则：一个不肯启动的机器人 远比 一个静静不动的机器人 好）
- 位置：`main.py:172`（`os.environ.get("RUN_MODE", RunMode.MOCK)`）、`config.py:112-115`
- 证据：mock 下 `chassis=None`、感知 mock → 一条 `VEL` 都不会发，而日志首行是正常的"🔧 运行模式: MOCK"。
- 影响：现场漏设 = 全程 0 分，且现象是"程序完全正常、有日志、有决策输出"，极难定位。

## T0-10 ✅【已修改】串口 open 失败 / START 握手失败不 fail-fast → 车不动且只剩一行日志
- **状态：已修改**（`main.py`：真机串口打不开 → CRITICAL + `return 1` **拒绝启动**（含排查命令提示）；`autonomous_state.on_enter`：底盘打不开或 `start_match()`（PONG/ACK 超时）失败 → `emergency_stop` 拒绝进入自主模式。实测：本机真机模式（无 dialout 权限）→ `main()` 返回 1 ✅)
- 位置：`main.py:249-252`（open 失败只 `logger.error`）、`autonomous_state.py:195-202`（`start_match()` 失败只打 `⚠️ 失败`）
- 机制：`chassis` 仍非 None → 舵机被装进转运管线（命令静默无效）；`main.py:359` 的串口一键启动判断带 `chassis.is_open` → 永远不成立 → DEBUG 下再也进不了 AUTONOMOUS（除键盘 `l`）；协议明确未 START 时运动命令回 `ERR,NOT_STARTED` → 后续 `VEL` 全被拒 → **车一动不动**。
- 影响：典型现场故障（线序反、下位机没上电、dialout 权限）会表现为"程序在跑但车不动"，只有一行日志。

---

# T1 丢大量分

| # | 问题 | 位置 | 证据/影响 |
|---|---|---|---|
| T1-1 ✅**【已修改】** | **边界安全带 50mm 环**：`set_target` 接受但 A* 永远到不了 → 永久 BLOCKED、零速、每帧重规划；`NAV_TIMEOUT_S` 是**死常量**（定义+赋值，零引用）→ 没有超时放弃 | `navigation_pipeline.py:141-156`、`forbidden_zones.py:96-131`、`path_planner.py:105-111` | 队长复现：`set_target(2999,1500)→接受`，60 帧后 `state=BLOCKED cmd=(0,0)` |
| T1-2 ✅**【已修改】** | **`clamp_to_safe` 把目标甩到场地中心**（最远 1491mm），且仍返回 True | `forbidden_zones.py:158-192` | 深读实跑：`(1500,100) → (1500,1500)` 位移 1400mm；对方安全区外扩 50mm 与边界安全带重叠 20mm → 来回弹后走兜底 |
| T1-3 ⏸**【保留·待真机】** | **巡航速度只有 ~163mm/s（能力 19%）**：路径跟踪给前瞻点（100~250mm），线速度是 `kp(0.8)×distance`，要 1062mm 误差才饱和到 850 | `motion_control.py:201-208,218-231` | 深读实跑端到端：2.85m 花 **22.3s**，全程最大 163mm/s |
| T1-4 ✅**【已修改】** | **`is_at_target` 只看位置不看朝向**，且 `distance<40mm` 无条件返回零速 → 以任意朝向"到达" | `motion_control.py:194-197,269-275` | 下游 `drop_position((rx,ry,rtheta))` 用 rtheta 算落点方向 → 朝向错则落点错（-10 分/个） |
| T1-5 ✅**【已修改】** | **目标状态单向锁存**：`BEING_TRANSPORTED` / `IN_SAFE_ZONE` 都没有回 `ACTIVE` 的边（全仓只有 3 处 status 赋值）→ 投放无效/途中掉落/裁判重放的目标**永久不可选** | `world_map.py:159,266-276,321,418-427` | 深读实跑：套取后 `get_injured()=[]`；关联匹配不看 status 且 `track_lost_count=0`，目标原地被看见就永不删也永不选 |
| T1-6 ✅**【已修改】** | **`_check_invalid_transport` 判据自相矛盾恒 False**：要求 `status==ACTIVE`，但 `_delivered_ids` 里的一定是 `IN_SAFE_ZONE` | `decision_engine.py:415-421` | 深读实跑确认；"裁判把无效投放重放场心"这条恢复路径是死代码 |
| T1-7 ✅**【已修改】** | **首趟投歪后无法重做首趟**：刚运过的那 1 个普通物资是 `BEING_TRANSPORTED`，被 `_selectable` 过滤 → 只能去找"另一个" | `decision_engine.py:370-376`、`target_selector.py:169-180` | 深读实跑：投歪后 `get_regular_supplies()=[]`；若场上只有它 → 首趟永远完不成 → 按赛规整场无效 |
| T1-8 ✅**【已修改】** | **不可达目标 + 无超时**：`NAV_TIMEOUT_S/GRIP_TIMEOUT_S/TRANSPORT_TIMEOUT_S` 三个常量零引用；目标选择不校验可达性 | `decision_engine.py:102-104`、`config.py:222-224` | 深读实跑：300 帧全是 `NAVIGATE_TO (3200,1500)` |
| T1-9 ✅**【已修改】** | **急停不下发 STOP/ESTOP**：`send_estop()` 全仓 0 调用；`on_exit` 只置 stop_event；`main.finally` 关串口前不发停止 | `serial_chassis.py:136-138`、`autonomous_state.py:248-258,394-396`、`main.py:330-336` | 队长核对：`send_stop()` 仅 1 处（`_stop_chassis`，只被套取前/终场用）→ 急停后靠下位机 800ms 看门狗，850mm/s 下最多再冲 0.68m |
| T1-10 ✅**【已修改】** | **保活链被决策引擎逐帧覆盖** → 探索/绕圈只存活 1 帧，实际不解卡；且只排除了 CAPTURING/PLACING | `autonomous_state.py:576-586`、`navigation_pipeline.py:370-393` | 队长读码确认：决策引擎在未套取时每帧发 `NAVIGATE_TO`、已套取时每帧发 `TRANSPORT_TO`，`_set_nav_target` 会把它设回去 |
| T1-11 ✅**【已修改】** | **S-41 裁判强制分离无响应**：`handle_forced_separation` 0 调用；`contact_duration_s` 由 `decision_engine.update()` 默认 0 传入，`autonomous_state` **不传** → "接触>10s"分支永不触发；`OpponentTracker` 结果只进 `get_stats()` | `decision_engine.py:190,234,527`、`anomaly_handler.py:210`、`perception_pipeline.py:240,251` | 队长 grep + 读码确认 |
| T1-12 ✅**【已修改】** | **对手避让整链断裂**：`set_opponent_target()` 0 调用 → `opponent_factor` 恒 1.0；`opponent_strategy.py`（592 行）仅被 re-export = 死代码 | `target_selector.py:65,98`、`opponent_strategy.py` | 队长 grep 确认 |
| T1-13 ✅**【已修改】** | **兜底测距恒为硬编码 500mm**：兜底调用未传 `contour_area` → `sqrt(0)=0` → `distance=500.0`；且 y 再减 105mm，与主路径语义不同 | `perception_pipeline.py:203-211`、`detection.py:378-399` | 深读实跑：任何走兜底的目标都被放到车前 0.5m；结合 3 帧确认会生成假目标 → 空跑一趟 |
| T1-14 ✅**【已修改】** | **套取视觉确认判据过松**：`min_confidence=0.25` 挡不住任何过面积门槛的轮廓（门槛值给 0.30）；ROI 内任意检测都算"套住"；**无帧判"槽内空"**；上游异常按成功（fail-open） | `perception_pipeline.py:93-120`、`transport_pipeline.py:428-431` | 深读读码 + 置信度公式核对；假阳性=空手跑一趟，假阴性=反复抬爪后退 |
| T1-15 ✅**【已修改】** | **硬禁区只有事后反应**：`check_violation` 只在已进入时触发（罚分已成立）；倒车是固定 `linear=-200, angular=0` 的开环，无落点校验/无超时；150mm 预警接口 `get_violation_warning` 零调用 | `navigation_pipeline.py:189-199`、`forbidden_zones.py:194-224` | 深读 grep + 读码；边界安全带超出 ±100mm 后不再属于任何禁区 → 与 T1-1 同源卡死 |

### T1-10 的修改说明（队长，已实测）
`AutonomousState.KEEPALIVE_HOLD_S=4.0`：看门狗触发探索/绕圈后开一个"保活优先窗口"，窗口内 `_set_nav_target()` **不接受**决策引擎的新目标
（旧实现每帧被覆盖 → 保活点只存活 1 帧 → 机器人继续朝原目标硬顶，保活链等于没有）。
实测：窗口内导航目标仍是保活点 `(1000,1000)`；窗口结束后正确切回决策目标 `(2000,2000)`。

### T1-4 的修改说明（队长，已实测）
`MotionController.compute_velocity(..., align_heading=None)`：位置到位但朝向超差时**只给角速度原地对准**，不再无条件零速；
`NavigationPipeline.require_final_heading()/clear_final_heading()` + `_aligned()`，三处"到达"判定都要求朝向满足；
`TransportPipeline` 在距投放点 <`DROP_ALIGN_DIST_MM=500mm` 时置位朝向要求、进入 PLACING 前解除。
实测：位置误差 0 / 朝向差 90° 且要求对准 → `cmd=(0,-3.00)`（只转向）；不要求时 `cmd=(0,0)`（旧行为保留）；导航管线 85 帧后对准到 8.5°（容差 8.6°）才判到达。

### T1-9 的修改说明（队长，已实测）
三处补上停车（`autonomous_state._run_once` 的 EMERGENCY_STOP 分支、`on_exit`、`SerialChassis.close()`），
实测 `close()` 现在会先发 `VEL,0,0` + `STOP`。
**故意的取舍**：一律用 `VEL,0,0`+`STOP`，**不用 `ESTOP`** —— 协议 §4.4 明确
"`ESTOP` 锁定后 `START` 不能解除锁定"（`chassis_serial_protocol.md:101`），
用它会**锁死下位机板子、当天后续场次必须断电重启**。`ESTOP` 仅保留为人工兜底手段。

### T1-1 / T1-2 / T1-8 的修改说明（队长，已实测）
- **T1-1**：`set_target` 新增**第三道闸门**"必须落在可通行格"（`_cost_map.is_free`）；不可通行时用 `_pull_to_traversable()` 朝场地中心逐步拉回（步长 25mm、上限 300mm），拉不回则**拒绝**。实测：`set_target(2999,1500)` 实际目标变 `(2949,1500)`、可通行=True，60 帧后 `state=MOVING cmd=(59,-0.41)`（修复前 `BLOCKED cmd=(0,0)` 永久零速）；合法目标 `(1345,2820)` 不受影响。
- **T1-2**：`clamp_to_safe` 改为"候选点集合取最近合法点"（各 hard 禁区外扩边+四角），**不再**逐矩形弹跳后兜底返回场地中心；位移 >200mm 打 WARNING；找不到合法点返回 `None` → 调用方明确拒绝。实测：`(1500,100)` → `(1500,381)` 位移 281mm（修复前 → `(1500,1500)` 位移 **1400mm** 且静默返回 True）。
- **T1-8**：接入 `NAV_TIMEOUT_S`，但做成**进展感知**（不是绝对超时——实测巡航只有 163mm/s，绝对超时会误弃正常目标）：离目标连续 `NAV_TIMEOUT_S` 秒没有 ≥`NAV_PROGRESS_MM=20mm` 的推进就放弃该目标，记入 `_abandoned_ids` 冷却 `ABANDON_COOLDOWN_S=30s`，冷却期内改去**稳定的**探索点。实测：不可达目标第 13 秒被放弃→转探索；第二个目标被正确选中；冷却期 195 帧只有 1 个探索点（修复前每帧随机=195 个点，会让导航目标每帧跳变）。

---

# T2 中等

| # | 问题 | 位置 | 要点 |
|---|---|---|---|
| T2-1 ✅**【已修改】** | 动态避障整条失效：`COST_OPPONENT=200 < COST_OBSTACLE=255`（阈值判定用后者）→ 对手永不触发局部避障；生产从不传 `opponent_position`；costmap 除 5 个硬禁区外**无任何静态障碍**（减速带/隔板/围栏都没写入） | `navigation_pipeline.py:220-224,286-296`、`path_planner.py:33` | 深读实跑：对手在正前方 200mm 60 帧，`_is_near_obstacle=False`，照直开 |
| T2-2 ✅**【已修改】** | `BLOCKED` 每帧全空间重规划：实测最坏 **15.3ms/帧**（50Hz 预算 20ms 的 76%）+ 每帧 WARNING | `navigation_pipeline.py:243-262`、`path_planner.py:190-240` | RDK 上叠加感知后可能断流 → 下位机看门狗接管（车会停）。RDK 实测未做 |
| T2-3 ✅**【已修改】** | `LocalPlanner` 硬编码 850/3.0，DWA 输出不再限幅 → YAML 限速被旁路 | `path_planner.py:285-291`、`config.py:242-251` | 现场调低限速时**恰恰在贴近障碍的场合下发 850mm/s** |
| T2-4 🟡**【部分已修改】** | 7 类异常只有 2 类可能触发：`imu_data`、`contact_duration_s`、`sensor_status` 生产调用方都不传 → IMU 失控 / 传感器故障 / 碰撞卡死**永不触发** | `decision_engine.py:232-235`、`autonomous_state.py:366-376` | 摄像头掉线、被顶住 10s 在决策层完全"无人知晓" |
| T2-5 ✅**【已修改】** | 降级阶梯 `_check_fallback_needed` 死代码（且 `if idle>10` 提前 return 使 `>13` 不可达） | `decision_engine.py:603-620` | 设计中的 10s 探索/13s 保命从未运行 |
| T2-6 ✅**【已修改】** | 对抗层与 `FORCED_RESET` 死代码，且 `_handle_forced_reset` **无条件** `= FREE_RUN` → 将来接上会绕过首趟闸门（赛规整场无效） | `decision_engine.py:527-547` | 潜在炸弹 |
| T2-7 ⏸**【保留】** | `robustness/` 三件套（fault_tolerance 868 / stability 632 / logging_system 665 行）**零生产调用方**，只在 `deploy.py` 的 import 冒烟里出现 | `robustness/*`、`innovation/deploy.py:224-226` | 传感器/电机/电源/通信故障无可见性、无复盘时间线 |
| T2-8 ⏸**【保留·待真机】** | 套取动作**同步阻塞 50Hz 主循环**：单次 0.9s，`lower_with_retry` 3 次可达 6.3s；期间不发 `VEL` → 下位机按最后一帧速度继续走 300ms 才停 | `sleeve_lift.py:358-362,380-389,429-434` | 叠加 T0-6 的积压，`wait_for` 的 0.5s 大概率白等（读到的是旧数据） |
| T2-9 ✅**【已修改】** | `validate_field_config` 用 **CWD 相对路径** → 换目录启动即**拒绝启动**，且报错把人指向"改配置/TEAM_COLOR"（真正原因是 cwd） | `main.py:123,140,186-194` | 深读实跑：从 `/tmp` 调用 → `['找不到配置文件 config/robot.default.yaml']`；`main.py:179` 那处被 except 吞成 warning → YAML 调参静默失效 |
| T2-10 ✅**【已修改】** | 配置层"改了不生效且不报错"：`ConfigLoader.validate`/`_ROBOT_SCHEMA` 从不被 `from_yaml` 调用；`from_dict` 全是 `.get(key, base)` → 键名/类型写错静默退回默认 | `config_loader.py:36-49,200-234,470-502` | 现场把 `max_speed_mm_s`/`pid.kp`/`watchdog_*` 拼错 → 不生效也不报错 |
| T2-11 ✅**【已修改】** | 环境变量覆盖链路不通：`merge_with_env("RESCUE_")` 只在 `__main__` 自测里被调 | `config_loader.py:181-196,687` | `RESCUE_*` 临时覆盖无效 |
| T2-12 ✅**【已修改】** | `config/field.default.yaml` 在 src 里**零消费点**（仅 `deploy.py:191` 的文件清单提到） | grep | 现场按文档改出发区/安全区/禁区边距**完全不生效** |
| T2-13 ✅**【已修改】** | `config/strategy/{default,aggressive,conservative}.yaml` 三个文件只被 `hot_reloader`（自身无调用）引用 | `hot_reloader.py:9` | 三个策略配置是死的 |
| T2-14 ✅**【已修改】** | `robot.strategy_weights.*`（4 个权重）**未被任何代码读取**：schema 里校验了，`target_selector` 里 0 引用，权重写死在公式 | `config_loader.py:44-46,377-380`、`target_selector.py:102` | 现场改权重无效 |
| T2-15 ✅**【已修改】** | `_extract_opponent` 把**车体系**坐标当**场地系**喂给跟踪器（`estimate_position` 返回车体偏移，`opponent_tracker` 直接与 `robot_position` 比距离） | `perception_pipeline.py:276-278`、`opponent_tracker.py:163-166` | 目前无消费者故不影响得分；**一旦接 T1-11 就立刻误判**（车前 0.3~0.5m 的任意杂色检测被当成"对方贴着我"） |
| T2-16 ⏸**【保留·待真机】** | 面积门槛 `_min_contour_area=200px²` + 5×5 开运算 → 40mm 目标约 **1.2m 外基本检不到** | `detection.py:164,238-251` | 全场 3000×3000 里开局只能看到 1.2m 内 → "开局建图/挑最高分"物理上不成立，必须移动式扫描。像素尺寸为模型推算，**未用真机图实测** |
| T2-17 ✅**【已修改】** | 颜色重叠**不去重**：`LIGHT_BLUE`/`BLUE` 掩码重叠区同一物体产出**两个 Detection**（顺序只决定谁先命中，不阻止后者再命中） | `detection.py:184-198,234-275` | 深读实跑：同一物体同时判成"危险目标"和（覆盖后的）"核心物资" → 最坏把危险目标当救援目标搬 |
| T2-18 ✅**【已修改】** | 颜色覆盖**冲突**只 `logger.error`，不进返回值 → 一类目标静默不可检、启动不报错 | `target_types.py:334-343` | 与文件顶部"现场配置错误必须显式喊出来"的自我要求矛盾 |
| T2-19 ✅**【已修改】** | `deploy.sh` **不同步 `tools/`**（rsync 只同步 `src/`，tar 兜底打 `src/ config/ scripts/`）→ **现场自检程序上不了车** | `scripts/deploy.sh:82-89` | 我们为现场写的 `hw_selftest` 恰恰传不过去 |
| T2-20 ✅**【已修改】** | `deploy.sh` 的 rsync 路径**不同步 `config/`** → 现场改的 YAML 不会到车上（兜底路径才带 config） | `scripts/deploy.sh:82` | 表现为"参数改了没反应" |
| T2-21 ✅**【已修改】** | 部署路径**三处不一致**：`deploy.sh:13` `/home/pi/rescue-robot`、`RUNBOOK.md:146` `/home/sunrise/rescue-robot`、实际用过 `~/rescue` | 三处 | 容易 deploy 到不是"正在运行"的目录 |
| T2-22 ✅**【已修改】** | RUNBOOK 与代码矛盾：`RUNBOOK.md:107` 说"`CHASSIS_PORT` 默认 `/dev/ttyUSB0`，必须显式导出"，而代码**已改为默认 `/dev/ttyS1`**（`main.py:244`）；`serial_chassis.py:28` 注释仍写"RDK 部署 `/dev/ttyS0`（待确认）"，而真机已实测 `/dev/ttyS1` 跑通 | | 会让人怀疑默认值、白花时间 |
| T2-23 ✅**【已修改】** | 主循环 `except Exception` 每帧吞异常继续跑（`autonomous_state.py:271-276`）→ "决策退化/空转"不会让进程退出（本会话实锤过一次：0 分但进程健康） | | 真机排障的盲区 |

### T1-5 / T1-6 / T1-7 的修改说明（队长，已实测）
- 新增 `WorldMap.return_to_field(target_id)`：把目标状态**显式放回 ACTIVE**（旧实现全仓只有 →`IN_SAFE_ZONE` 与
  →`BEING_TRANSPORTED` 两条赋值，**没有任何回写 ACTIVE 的路径**）。
- 新增 `LoadManager.discard_load()`：作废当前装载（清空货舱但**不计趟次、不计分**，与 `release_all()` 区分）。
- `TransportPipeline` 新增 `set_release_failed_callback()`：**投放被判无效**、**VIOLATION 本趟作废**两条路径都回调上层；
  `AutonomousState._on_release_failed()` 把目标逐个 `return_to_field()`。
- 实测（端到端）：投放后 `release_valid=False` → 回调恢复目标 `[1000]` → 该目标**重新可选**（`get_regular_supplies()` 由 0 变 1，状态 `ACTIVE`）
  → **首趟可以重做**（修复前它会永久停在 `BEING_TRANSPORTED`：既不可选、又占着货舱，首趟永远完不成 → 按赛规整场无效）。
- `discard_load()` 实测：清空 `ids=[1000]`、装载数归 0、**趟次 1 → 1（不 +1）**。

### T2-3 / T2-4 / T2-5 的修改说明（队长，已实测）
- **T2-3**：`LocalPlanner` 的 850/3.0 由 `__init__` 默认参数改为类属性 `DEFAULT_MAX_*`，并由 `config.apply_robot_config()`
  与 `MotionController` **同源注入**。实测：把 YAML 改成 `max_speed_mm_s=500 / max_angular_speed_rad_s=1.5` →
  `MotionController=500/1.5` 且 `LocalPlanner=500/1.5`（修复前 LocalPlanner 恒 850，YAML 限速被旁路）。
- **T2-4 🟡 部分**：已接 `contact_duration_s`（见 T1-11）与 `sensor_status`（新增 `decision.update(sensor_status=…)` →
  `anomaly.check`；`autonomous_state` 仅在 `perception.vision_available is False` 时报 `{"camera": False}`，
  所以 Mock/仿真不受影响）。**未接**：`imu_data`（需每帧读串口 IMU，且有干扰位姿 drain 的风险，留待真机调）与
  `robustness/SensorHealthMonitor` 的完整降级链（见 T2-7）。
- **T2-5**：修掉两处 `if` 的**顺序错误**（`>10` 在前直接 return → `>13` 分支永不可达），并**明确标注本方法未接线、
  `_fallback_level` 只写不读**，指向真正生效的保活链（`autonomous_state._check_watchdog` + T1-10 的保活优先窗口）。
  不改写为"已接线"是因为：接了也没有消费者，属"接线也无效"。

### T1-15 / T2-17 的修改说明（队长，已实测）
- **T1-15**：`NavigationPipeline._apply_caution()` —— 在硬禁区**预警带**（`CAUTION_DIST_MM=150`）内把线速度压到
  `CAUTION_SPEED_RATIO=0.5`，日志限频 1 条/秒。用的是现成但**零生产调用**的 `get_violation_warning()`。
  实测：`(1500,400)` 在预警带内 → `400 → 200` mm/s；带外 `(1500,360)` → 不变。
- **T2-17**：`CVDetector._suppress_overlapping()` —— 同一物理物体在多颜色掩码重叠区（`LIGHT_BLUE`/`BLUE` 的 H∈[95,110]）
  会被检出多次，现按 bbox IoU≥0.45 或中心距≤12px 抑制后继检测，保留**高优先级颜色**。
  实测：3 个检测（前两个几乎完全重叠）→ 输出 2 个、保留 `LIGHT_BLUE`+`GREEN`。
  **顺带修掉一个被掩盖的 bug**：我第一版把类名写成 `CvDetector`（真名 `CVDetector`），
  而该 NameError **只在真有重复检测时才触发** —— 第一次单检测测试恰好掩盖了它，已修正并用"真有重复"的场景复验。

### T1-13 / T1-14 的修改说明（队长，已实测）
- **T1-13**：`perception_pipeline` 的位置估算改为"只有检测器**本身没有**地平面测距（Mock）时才走面积法兜底"；
  若检测器有地平面测距但**解算失败**（视线俯角 ≤1°，图像上部/远目标/杂光），**丢弃该检测**并计数告警，
  不再伪造坐标。实测：构造必然解算失败的帧 → 6 帧内活跃目标 **0** 个、丢弃计数 **6**（修复前会得到恒 500mm 的假坐标，
  经 3 帧确认后变成"车前 0.5m 的假目标"）。同时把兜底构造的 `Detection` 补上真实 `contour_area`（旧实现漏传 → 恒 500mm 的直接原因）。
- **T1-14**：`check_sleeve_occupied()` 现在返回 `Optional[bool]`：**本帧没有图像 → `None`（未知）**，调用方（`transport_pipeline`）
  对 `None` **不否定**（按成功继续）；并把置信度门槛由 0.25 提到 0.35（旧门槛挡不住任何过面积门的轮廓，因为门槛值本身就是 0.30）。
  实测：从未 update → `None`；本帧无图 → `None`；有图但槽内无目标 → `False`。

### T2-12 / T2-13 / T2-18 的修改说明（子代理执行 + 队长补齐 T2-13）
- **T2-12**：采用"明确标注"方案 —— `config/field.default.yaml` 头部加 ⛔ 块写明**未被任何生产代码读取、改它无效**，
  并点名真值来源 `perception/field_elements.py::StandardFieldLayout`。**额外发现**：该文件数值与 `FieldConfig` 默认值、
  与 `StandardFieldLayout` **三套互不一致**（文件写 `safe_zones.red x=1200,y=2670`，`FieldConfig` 默认 `x=50,y=2550`）→ 连"参考值"都不能当。
- **T2-13**：三个 `config/strategy/*.yaml` 由**队长**补上醒目告警头（子代理白名单不含这些文件）："未被任何生产代码读取、改它无效且不报错；
  生效的权重改 `config/robot.default.yaml` 的 `robot.strategy_weights`"。
- **T2-18**：`set_color_override` 现在把"**合法但冲突**"（两类目标撞同一个 (颜色,形状)）也并入返回值 →
  `main.py` 的"problems 非空即拒绝启动"自动生效（实测：`{"regular":"light_blue"}` → 报"LIGHT_BLUE/CUBE 同时对应 REGULAR_SUPPLY 与 DANGEROUS"）；
  默认映射无冲突、problems 仍为空。

### T2-2 / T2-6 / T2-9 / T2-23 的修改说明（队长，已实测）
- **T2-2**：`BLOCKED_REPLAN_S=0.5` 退避；BLOCKED 期间返回零速且**不重复规划**，失败日志限频 1 条/秒。
  实测：BLOCKED 下 50 帧只调用 1 次 A*（修复前约 50 次，最坏单次 15.3ms 占 50Hz 预算 76%）。
- **T2-6**：`handle_forced_separation()` 记住分离前的策略状态（`_forced_prev_state`），`_handle_forced_reset()` **恢复它**而不是无条件 `FREE_RUN`
  —— 旧写法在"首趟未完成"时会绕过"必须先单独送 1 个普通物资"的硬规则（按赛规整场无效）。
- **T2-9**：新增 `default_config_path()`：优先当前目录（兼容老用法），否则按 `__file__` 上溯三层定位仓库根的 `config/robot.default.yaml`；
  "找不到配置文件"的报错也补上"已尝试当前目录与仓库根"的说明。旧实现从别的 cwd 启动会**直接拒绝启动**（且日志把人指向配置/TEAM_COLOR）。
- **T2-23**：主循环异常累计计数 + 每 250 帧（约 5s）重报一次 + 保留 `_last_loop_error`。吞异常本身是正确的（单帧异常不该终止整场），
  但必须可见 —— 本会话曾被它掩盖过一次"决策崩了、进程健康、0 分"。

### T1-11 / T1-12 / T2-1 / T2-4 / T2-15 的修改说明（队长，已实测）
- **T1-11**：① `autonomous_state` 每帧读 `perception.opponent_tracker.contact_duration_s` 并喂给 `decision.update(contact_duration_s=…)`
  → 异常链"接触 >10s"分支终于可达（被顶住时会主动脱离，而不是等裁判分开）；② 新增**位姿跳变兜底** `POSE_JUMP_MM=400`：
  用上一帧位姿（`_last_motion_pose`）比较，单帧位移过大即判定"被搬动/强制分离"→ 打 CRITICAL + `_apply_start_pose()` 重置坐标系回出发区，
  并复位本趟进度与保活窗口（旧实现分离后仍按旧坐标跑，最坏朝错误方向冲出场）。
- **T2-15**：`_extract_opponent()` 改为返回**场地坐标**：优先地平面测距（面积法在混装形状下不可用）→ 按 θ 旋转（与目标同一套公式）；
  拿不到 `robot_position/robot_theta` 时返回 None（宁可不报，也不报错坐标）。顺带把"浮点元组相等"匹配改为容差匹配。
- **T1-12**：新增 `PerceptionPipeline.opponent_position`（置信度>0 且见过面才返回），`autonomous_state` 每帧传给 `navigation.update(opponent_position=…)`
  → 生产路径上动态避障参数终于不再是恒 None。
- **T2-1**：`_is_near_obstacle()` 的档位判定由 `>= COST_OBSTACLE(255)` 改为 `>= COST_OPPONENT(200)` —— 旧实现里对手代价 200 **永远**达不到阈值，局部避障恒不触发。
- **T2-4（前置，必须先修否则 T1-12 一接线就会穿对方安全区）**：`CostMap` 新增动态层原值快照 `_dynamic_saved`，`clear_dynamic()` 只恢复**自己改过的**格子；
  旧实现无条件写回 `COST_FREE`，会把对方机器人经过的**静态禁区格子永久抹成可通行** → A* 从对方安全区中间穿过去（=比赛结束）。
  实测：对手经过并 `clear_dynamic()` 后，对方安全区内该点代价仍是 255（修复前变 0）。

### T2-10 / T2-11 / T2-14 的修改说明（并行子代理执行，队长已复核）
`from_yaml` 内已接 `ConfigLoader.validate(_ROBOT_SCHEMA, data)`（键名/类型写错会报出来，不再静默退回默认）；
`from_yaml` 内已串 `ConfigLoader.merge_with_env(data)`（`RESCUE_*` 环境变量覆盖链路打通，无该前缀时是恒等变换）；
`TargetSelector` 读取 `robot.strategy_weights.*`（公式改为 `distance_factor=(1-dist/4242)**(distance_weight/0.3)` 等，
并保留原默认值语义），由 `config.apply_robot_config` 注入。
**队长复核**：`compileall` OK、`run_all` 7/7、`hw_selftest --mock` PASS=7/SKIP=7、
**集成仿真基线未变**——口径见文末「回归基线三要素」（仓库脚本 `snapshot_sim.py`，
`SEEDS=[1,7,42,99,123]`、`start_zone=1`：`score 80/80/80/85/80`、`delivered 7/7/7/8/7`、
`valid==delivered`、`ERROR=0`）。

### T2-19 ~ T2-22 的修改说明（并行子代理执行，队长已复核）
统一清单 `SYNC_DIRS=(src config scripts tools)` / `DELETE_DIRS=(src tools)`，**rsync 与 tar 兜底两条路径同源**；
默认目标改为 RDK 实际约定 `sunrise@192.168.50.2:/home/sunrise/rescue`（新增 `--user/--path`、远端 `mkdir -p`）；
RUNBOOK §1.3 按真机实测改为"`/dev/ttyS1` 已确认"、`RUN_MODE` 默认 `real`、**新增 §1.5.1「真机模式拒绝启动（返回码 1）」**、
附录 C 由行号清单改为可 grep 的符号索引。
**顺带修掉一个队长漏列的真错**：systemctl 兜底提示原写 `python3 -m rescue_robot.main`，**缺 `PYTHONPATH=src`**
（仓库无 setup.py/pyproject → 照抄会 `ModuleNotFoundError`），已修（`.bat` 同步）。
**残留（已记录，不阻塞）**：① RUNBOOK 里仍有成片旧式 `文件:行`（抽样确认已过期：`autonomous_state.py:217` 实际 `:285` 等），
现由附录 C 的符号索引兜底；② `deploy.bat` 无 Windows 环境验证，仅逐行审阅；③ 首次真机部署仍需现场跑一次真脚本确认。
另：`hardware/serial_chassis.py` 的 docstring 里 `/dev/ttyS0（待确认）` 已由队长按实测改为 `/dev/ttyS1`。

---

# T3 低 / 死代码 / 陷阱

| # | 问题 | 位置 |
|---|---|---|
| T3-1 | `compute_approach` 被 `@property` 装饰却带 2 个必填参数 → **访问必 TypeError**（队长实测），0 调用点 | `transport_pipeline.py:176-177` |
| T3-2 | `TransportStatus.violation` 恒为 `None`（`self._violation` 其实有记录）→ VIOLATION 原因不暴露给状态查询 | `transport_pipeline.py:628` |
| T3-3 | `perception/field_detector.py` 死代码（仅 `perception/__init__.py` 导出）；`sensor_fusion` 从没被喂过数据（`_position` 恒 (0,0)）；`imu_fusion.py` 全仓零引用 | |
| T3-4 | `TargetSelector.select_best()` 0 调用（死代码，且它正是含"伤员绝对优先"那份逻辑） | `decision_engine.py` 只用另两个方法 |
| T3-5 | `check_sleeve_occupied` 无法区分"**摄像头无帧**"与"槽内为空" → 无帧时连续判套取失败（靠连续 5 次后自动关闭确认自愈，白扔几趟） | `perception_pipeline.py:110-111` |
| T3-6 | 主循环吞异常（同 T2-23）；终场后 `main()` 不自动退出，需操作员 Ctrl+C | `main.py:347-369` |
| T3-7 | 导航目标被拒时**每帧一条 WARNING** → 50Hz = 50 条/秒，180s 约 **9000 条**（队长实测 500 次→500 条），会淹没真错误 | `autonomous_state.py:436-440` |
| T3-8 | DWA 轨迹评估**忽略角速度**（`theta` 从不更新）→ 0.5s 视界内 ~86° 旋转对碰撞检测不可见；w 采样也没以当前 w 为中心 | `path_planner.py:315-331,343-346` |
| T3-9 | `set_my_color` 重建禁区但**不重写 costmap** → 颜色切换后 A* 仍用旧禁区（目前无生产调用） | `forbidden_zones.py:228-233` |
| T3-10 | `localization.py:31` 注释写 `theta: 0 = 正前方 (+Y)`，实现用的是 `x += ds*cos θ`（= +X 约定）→ **危险注释**（后人"照注释修正"会全盘错位）。实跑确认实现自洽 | |
| T3-11 | `MockLocalizer.reset_pose` 不清 `_odom_distance`（与 `OdometryLocalizer` 语义不一致）→ Mock 路径下置信度诊断失真 | `localization.py:139-149` |
| T3-12 | `set_target` 返回值在两个生产点未检查：`transport_pipeline.py:466`（S-40 下一个目标）、`:532`（**推入点**= 落点再推 `push_dist_mm=100`，红方安全区 y 上限 2970 → 很容易越过 3000 被静默拒绝 → 不推、落点无效 -10 分/个且无日志） | **具体数值路径未证实** |
| T3-13 | `D-11` 自由趟 `release_valid is None` 分支**不可达**（`not None` 先命中）→ 日志把"漏传参数"误报成"投歪" | `decision_engine.py:492-505` |
| T3-14 | `_handle_anomaly` 兜底返回 WAIT；`CONTINUE_REDUCED/ROUND_END` 两个 RecoveryAction 无消费方（今天不会发生，接新异常类型就静默退化） | `decision_engine.py:599` |
| T3-15 | 仿真 `except → WAIT` 把决策异常吞成"静默不动"（生产路径反而没有这层 try） | `integrated_sim.py:289-291` |
| T3-16 | 形状判据 `SHAPE_ASPECT_RATIOS` 按"俯视≈2"设计，与 30° 斜视投影不符：80×40 在朝向 45°~90° 时 ratio 掉到 1.03~1.41 → 被判 CUBE（默认表下被同色兜底救回）；`SHAPE_VERTEX_RANGES` 里圆柱/圆锥台/球区间完全重叠 | `detection.py:62-91,297-340` |
| T3-17 | `_fuzzy_match` 的 `RED↔ORANGE` 是**跨类型**容差（只排除 DANGEROUS，不校验分值）→ 若现场把普通物资覆盖成 red 且形状判错，5 分物资会被判成 15 分伤员 | `classification.py:112-129` |
| T3-18 | 感知每帧 13 次全图算子（4 色 × (inRange+OPEN+CLOSE+findContours)），`getStructuringElement` 在颜色循环内重建；`HuMoments` 与 `estimate_orientation`（PCA）算了从不使用 | `detection.py:231-246,264,317-319`。x86 实测中位 1.5~2.2ms；**RDK 未测** |
| T3-19 | `FOV_DEG=77°` 被当**水平**视场用，若模组标称是对角视场则横向距离系统性偏小 ~19% | `config.py:143`、`detection.py:427`。**必须现场标定** |
| T3-20 | 串口 `_send` 无锁、无 `flush`、未设 `write_timeout` → 无锁（确认，但当前单写者风险低）；UART 卡住时 `write()` 会无限阻塞整个 50Hz 循环（**未证实**） | `serial_chassis.py:140-147` |
| T3-21 | `one_key_start()` check-then-act 竞态（无锁）→ 两线程同时启动时 `transition` 抛 ValueError → 冒泡到 `main` → `return 1` 并关串口（此时循环线程已起） | `state_machine.py:193-203` |
| T3-22 | REAL 模式下 `CommManager` 起 `MockCommServer` → `input()` 线程与 `MockButton` **抢 stdin**（键盘 `l` 可能被吃掉）；退出 DEBUG 时 `join(timeout=1.0)` 拖住约 1s | `comm_server.py:71-78,106-113`、`main.py:235` |
| T3-23 | `WebSocketServer`（`comm_server.py:152-253`）是未接线遗留（`handler(websocket, path)` 与新版 websockets 签名不兼容） | 不影响真机（只用 Mock 服务器） |
| T3-24 | `tools/vision_calibration.py:123` 把 `argv[1]` 直接 `int()` → 传 `--help` 抛 ValueError 崩 | |
| T3-25 | `_delivered_ids` 无法识别"目标被删后重建为新 id"（`world_map` 3s 删目标 + `_next_id` 单调递增）→ "重放场心"可能检不出 | `decision_engine.py:415-421`、`world_map.py:292-298` |

---

# 刻意保留的条目（需求方 2026-09-15 决定）

以下条目**未修改、故意保留**，等待真机阶段处理。**不要把它们当作遗漏**：

| 编号 | 状态 | 保留原因（摘要） | 解锁条件（真机测什么） |
|---|---|---|---|
| **T1-3** | ⏸ 待真机 | 路径跟踪的线速度由"到前瞻点的位置误差"决定（`kp=0.8×距离`，要 1062mm 误差才饱和）→ 实测仅 163mm/s。提速是**行为大改**：套取半径 150mm、投放容差 80mm 都是厘米级，850mm/s 时一帧走 17mm。本会话已有一次盲调提速**过冲致仿真 0 分**的前车之鉴 | 从 850mm/s 起刹的**实际制动距离**、里程计延迟、路径跟踪横向误差 → 再定巡航速度并留余量 |
| **T2-7** | ⏸ 保留 | `robustness/` 三件套共 **2216 行**从未接线。接它 = **新增功能**（非修 bug），且在结项前一次性打开未跑过的代码正是"修 A 引入 B"的标准做法（本会话已三次踩此坑）。且 `PowerMonitor` 需电压传感器而**本车未接**（`check_battery_voltage()` 恒 −1）。其中"摄像头健康"已由 T0-5/T2-4 覆盖，"电机转但不动"已由 T0-2 的两路独立判据覆盖 | 队内决策：是否投入。**低成本高价值切片** = `MotorFaultDetector`（用 ODOM 的 `vL/vR` 做堵转二次确认）与 `EventRecorder`（赛后复盘时间线） |
| **T2-8** | ⏸ 待真机 | 舵机动作同步阻塞主循环（单次 0.9s、重试可达 6.3s）。**最危险的后果已被 S-40 的显式停车挡住**（套取前先发 `VEL,0,0`+`STOP`），剩余代价是"套取期间感知/决策停摆 ~1s"——而套取期间车本来就该静止。真修需把舵机**异步化**，属对 `sleeve_lift`+转运阶段机的重构，且 0.4s 是照 SG90 行程定的 | 舵机**实际行程时间**、`ACK,SERVO,LOWER` 是否在 0.5s 内到达（T0-6 修好接收缓冲后**必须重测**，因为它已改变了 ACK 的到达时序） |
| **T2-16** | ⏸ 待真机 | 面积门槛 200px² → 40mm 目标 1.2m 外即被丢弃。门槛是"看得远 vs 误检多"的权衡，另一侧数据只能从真机图像来（HSV 实际命中率、`FOV_DEG=77°` 究竟是水平还是对角——若是对角则 f 差 19%、镜头畸变、安装倾角全是未标定项） | 真机把目标放 1m/2m/3m，用 `tools/vision_quick.py` 读**实际轮廓面积** → 定门槛（或改成随距离自适应）。纯标定、不需改架构 |
| **T2-4** | 🟡 部分 | 已接 `contact_duration_s`（T1-11）与 `sensor_status`（摄像头故障）；**未接** `imu_data`（需每帧读串口 IMU，有干扰位姿 drain 的风险） | 真机确认真机上每帧读 IMU 是否影响位姿更新 |
| **T3-1 ~ T3-25** | ⏸ 保留 | 本轮范围按需求方指定为 **T0/T1/T2**；T3 为"低危/死代码/陷阱"（如 `compute_approach` 的 `@property` 误用、`TransportStatus.violation` 恒 None、`field_detector`/`sensor_fusion`/`imu_fusion` 死代码、仿真吞异常等），**不丢分不卡死** | 视时间与优先级，可作为后续清理批次 |

---

# 本轮修改总结（T0 / T1 / T2，按需求方指令执行）

**口径**：按优先级逐条修；**涉及真机标定/需要真机验证的先留着**；每修完一条在本文档标注"已修改"。

| 级别 | 已修改 | 故意保留（原因） |
|---|---|---|
| **T0** | **10 / 10 全部** | — |
| **T1** | **14 / 15 ✅** | T1-3 ⏸（巡航速度只 163mm/s，提速必须真机验证制动距离，**留待真机**） |
| **T2** | **19 / 23 ✅**（另 T2-4 🟡 部分） | T2-7 ⏸（`robustness/` 三件套 ~2216 行未接线：属"接线即新功能"，改动面大、非丢分项）<br>T2-8 ⏸（舵机动作同步阻塞主循环 0.9–6s：需改成异步舵机 + 速度保活帧，属真机联调范围）<br>T2-16 ⏸（面积门槛导致 1.2m 外检不到：需真机图像标定门槛）<br>T2-4 🟡 剩余（`imu_data` 每帧读串口、`SensorHealthMonitor` 完整降级链） |

**合计**：✅ **43 条**（T0 10 + T1 14 + T2 19）、🟡 **1 条**（T2-4）、⏸ **4 条**（T1-3 / T2-7 / T2-8 / T2-16），
T0–T2 共 48 条**无一条漏标**；T3 共 25 条按范围整体保留（见上表）。

**每条修改的验证方式**：定向断言（构造触发场景 → 断言修复前会错、修复后正确）+ 回归四件套
（`compileall` / `run_all.py` 7/7 / `hw_selftest --mock` PASS=7 FAIL=0 SKIP=7 / 决策引擎自测 exit 0）+ **集成仿真 5 种子**。
**最终基线（保持未变）**：口径见下节「回归基线三要素」——仓库脚本 `snapshot_sim.py`
（`SEEDS=[1,7,42,99,123]`、`start_zone=1`）得 `score 80/80/80/85/80`、`delivered 7/7/7/8/7`、
`valid == delivered`、`ERROR=0`；且**与修复前逐位一致（零回退，见下节对照证据）**。

---

# 回归基线三要素（口径统一，防下游复现不上）

> 历史遗留问题：本仓库先后用过**多套仿真 harness**，种子集合与 `start_zone` 不同，
> 导致各文档记的"5 种子序列"互不相同（`REVIEW_REPORT.md` §6 曾专门指出该差异：
> 评审跑仓库脚本得 `80/80/80/85/80`，而文档写 `80/80/85/80/80`）。
> 现统一如下：**引用基线必须同时写明「脚本 + 种子集合 + start_zone」三要素。**

| 口径 | 脚本 | 种子集合 | start_zone | score | delivered |
|---|---|---|---|---|---|
| **✅ 基准口径（唯一对外口径）** | `tools/fix_verifiers/snapshot_sim.py` | `[1, 7, 42, 99, 123]` | `1` | **80/80/80/85/80** | **7/7/7/8/7** |
| 历史口径（仅存档，勿再引用） | 临时脚本 `/tmp/reg5.py` | `(1, 42, 123, 7, 2024)` | `3` | 80/80/85/80/80 | 7/7/8/7/7 |
| 队长复查口径 | 自建 harness，`IntegratedSim(seed=s)` 用默认 zone | `1..5` | 默认 | 80/80/85/85/80 | 7/7/8/8/7 |

**共同不变量（三种口径均成立）**：`valid == delivered`（零投错区）、`ERROR == 0`（零异常）、
危险目标从未被转运、无一场出现 `运危险物=True`。

**复核命令（任何人可原样复现）**：
```bash
cd <仓库根>
PYTHONPATH=src python3 tools/fix_verifiers/snapshot_sim.py
# 期望：5 行全 ✅；score 80~85；valid == delivered；无 ERROR
```

## 零回退对照证据（本轮修复前 vs 修复后）

为排除"本轮 T0/T1/T2 修复把仿真跑坏"的可能，用 **`git worktree` 拉出修复前的提交 `ba4df56`**，
在同一台机器、同一份（已修好解析 bug 的）`snapshot_sim.py`、同一组种子下各跑一遍：

| 代码状态 | commit | score 序列 | delivered 序列 |
|---|---|---|---|
| 修复前 | `ba4df56` | 80/80/80/85/80 | 7/7/7/8/7 |
| 修复后 | `c80d3e5` | 80/80/80/85/80 | 7/7/7/8/7 |

→ **逐位完全一致：本轮修复未改变仿真行为（零回退）**。这是"默认行为不变"最直接的证据——
比"基线数字没变"更强，因为后者无法区分"没改坏"与"改好一处又改坏一处"。

复现方式：
```bash
git worktree add /tmp/pre ba4df56
cd /tmp/pre && PYTHONPATH=$PWD/src python3 tools/fix_verifiers/snapshot_sim.py
```

> ⚠️ 注意：`ba4df56` 自带的 `snapshot_sim.py` 有解析 bug（`line.split(" ", 5)` 解包失败），
> 该 bug 已在本轮 tools 提交中修掉；对照时须使用**修复后的**同名脚本指向旧 `src`。

**证据**：每条"已修改"都带实测输出（见各级的修改说明）。

---

# 附：本轮"看起来已修但仍有残留"的项（避免误判为已完成）

| 项 | 残留 |
|---|---|
| S-40 计划≠实装 | 记账正确；但 T1-5/T1-6 让"投放无效后重选"这条恢复链实际是死的 |
| S-01 终场停车 | "时间到"分支正确；但**引入了 T0-1**（空地图 WAIT 打断运送相） |
| S-11 保活链 | 看门狗按实际位移判定正确；但**激活了 T0-2**（假卡死）与 T1-10（保活被覆盖） |
| U4 减速带 | 标志已接线；但**激活了 T0-4**（bump 模式重入） |
| U5 颜色配置 | 接线正确（队长实跑确认改 YAML 真生效）；但 T2-18 冲突检测不进返回值 |
| N-6/N-7 落点 | 判定已回到真实落点（队长实测 5 个坏落点全 INVALID）；但**朝向仍是自由变量**（T1-4） |
| B1 出发区坐标系 | `main.py` 下发 + `_apply_start_pose()` 三处同步，均已接线（深读方"未证实"的疑点已由队长确认解决） |

---

# 建议的处理优先级（**仅供参考，本文档不含修复动作**）

1. **先修 T0-1、T0-2、T0-3**（三条独立成因，合起来会让真机约 5 秒后整场报废；且都是本轮修复引入/激活的）
2. **T0-4**（U4 激活的 bump 重入，可能撞围栏/直入对方安全区）
3. **T0-6 / T0-7**（位姿滞后与一键启动丢事件：真机"车不动/启动不了"的两大现场症状）
4. **T0-8 / T0-9 / T0-10**（三个"上电就废"的启动问题，改动量都很小）
5. **T0-5**（摄像头掉线后整场追幻影，属于"宁可停车也别乱跑"）
6. 再处理 T1 里对分数影响最大的：T1-3（巡航速度 19%）、T1-1/T1-2（幽灵目标与目标被甩走）、T1-4（朝向）、T1-5/T1-6/T1-7（状态锁存与首趟重做）

**方法论提醒**：本轮所有"仿真 80/7/7 全绿"的结论**都不能证明真机可用**——
T0-1、T0-2、T0-4 三条都是仿真与真机喂参差异造成的盲区。真机验证前，
建议先在 `integrated_sim` 里补上"传 velocity + 调 `mark_being_transported` + 传 `near_speed_bump`"
这三件事，让仿真能复现真机路径（这属于验证手段，不属于修复）。

---

# 追加：夹爪 V2 机构变更（2026-09-16 机构组）

**变更**：夹爪由旧「U 型槽 + 舵机带单块后方板渐进上调」改为
「**150×100 方形套框 + 后方三块水平阶梯板**（157×40 / 150×35 / 150×20，自上而下递减）」。
套取方式、舵机行程（0°/70°）、单趟容量（1）**均未变**。
完整实测数据、坐标系判定依据与标定清单见 **`docs/GRIPPER_V2_GEOMETRY.md`**。

## 本次因机构变更而修的问题

| # | 问题 | 严重度 | 说明 |
|---|---|---|---|
| **V2-1** | **`CAPTURE_RADIUS_MM = 150` 对新夹爪必然套空** | **高危** | 套框中心在车心前方 `DROP_FORWARD_MM=70`、开口半深 50 → 目标必须落在车心前方 **[20,120]mm** 才是真的在框正下方。150 在区间外 → 软件记账"已套住"、实车套空，且**反向废掉首趟"必须且仅 1 个普通物资"闸门**（与 S-40 同族的幽灵捕获）。已改为 **100**，并升级为 YAML 可调的 `placement.capture_radius_mm`。**实测：仿真对该值在 80~150 区间完全不敏感（四种取值结果逐位一致），故改动不影响回归基线。** |
| **V2-2** | **`PlacementConfig.drop_forward_mm` 数据类默认值是 150.0**（N-6 证明的坏值） | 中 | `robot.default.yaml` 写了 70.0 所以生产路径没事，但**任何漏写该项的自定义 robot YAML 都会静默拿到 150**。已改为 70.0，并修掉 `drop_position()` 里同样写死 150.0 的误导性兜底。 |
| **V2-3** | **推升来源变了，`place_ramp()` 的语义已失效** | 中 | 旧机构靠舵机带**单块**后方板渐进上调；V2 是三块**固定**阶梯板。旧代码"推入中 0°→70° 分 4 步渐进抬"的**原有升力作用已消失**，只剩渐进卸力。已把 `_place_steps` 提为 YAML 可调 `placement.progressive_raise_steps`（4=旧行为 / 0=保持套住到到位后一次释放），属真机标定项。 |
| **V2-4** | 全仓 7 处 **"U 型槽"** 描述已与实物不符 | 低 | 含日志文案 `视觉确认：U 型槽内未见目标` → 已改为"套取框"，并同步 RUNBOOK 故障表 F12 的引用（否则按图索骥搜不到日志）。 |
| **V2-5** | **`SLEEVE_ROI` 旧值是按旧夹爪标的，V2 换框后大概率失效** | **高危·待真机** | 未标定会导致"视觉确认连续失败 → 自动关闭 → 套取全废"。已升级为 RUNBOOK §5.3 最高优先级标定项；标定前建议先设 `sleeve_confirm: false`。 |

## 新增的工具与文档

- **`tools/stl_gripper_probe.py`** —— 从 STL 实测夹爪几何（开口、阶梯板分层、包围盒）。
  机构再改时重跑即可，避免"靠口述/卡尺量错"（本次即发生：口头为"10cm 正方形"，实测为 **15cm×10cm**）。
- **`docs/GRIPPER_V2_GEOMETRY.md`** —— 实测数据 + 坐标系判定依据 + 7 项真机标定清单。

## 回归结论

改动后四件套 + 规范仿真全部通过，且**规范仿真与基线逐位一致**：
`compileall` OK、`run_all` 7/7、`hw_selftest --mock` PASS=7 FAIL=0 SKIP=7、
决策引擎自测 exit 0、单元测试 core 12/12 + 创新集成 15/15、
`snapshot_sim.py` → **80/80/80/85/80**，`valid == delivered`，无 ERROR。

> ⚠️ 仿真**无法**验证 V2 的两件要命事：① 套框是否真的套住（仿真不建夹爪几何）；
> ② 目标能否爬上紫边斜坡（仿真不建斜坡）。这两条只能真机测。
