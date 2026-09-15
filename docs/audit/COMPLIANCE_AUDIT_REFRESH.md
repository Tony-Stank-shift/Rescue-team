# 合规审计刷新：blocker / high 项在当前代码下的复核

- 任务：t11（req-auditor，attempt 2，attempt_id `a9791035-54e7-428b-8001-cdc38695e91d`）
- 刷新时间基准：当前工作区（含未提交改动）`git diff --stat 0e8a5ae` 显示已改动 6 个文件
  （`decision/anomaly_handler.py`、`decision/decision_engine.py`、`navigation/forbidden_zones.py`、`navigation/navigation_pipeline.py`、`states/autonomous_state.py`、`transport/load_manager.py`）
- 判定口径：**仍成立 / 已修复 / 已变化（给新行号）**。所有行号均为**当前工作区**行号（非基线）。
- 复核方法：读当前文件 + `git status` 确认该文件是否被改 + 实跑关键常量/几何；不采信注释与任务描述。

## 一行一条

| 原编号 | 判定 | 证据（文件:行，当前工作区） | 备注 |
|---|---|---|---|
| **B1** 出发区坐标系写死 3 号区 | **仍成立（未改动）** | `hardware/serial_chassis.py:57-59`（默认 `start_x=150.0/start_y=150.0/start_theta=1.5707`）；`states/autonomous_state.py:138`（`self._pose=(150.0,150.0,1.5707)`）；`main.py` 仍只读 `TEAM_COLOR`（`grep -n "environ.get" main.py` 命中 98/129/142/152/156 五处，**无出发区编号、无朝向**）；全仓无运行时 `set_start_pose()` 调用（仅 `serial_chassis.py:100-102` 定义 + 各文件 `__main__` 自测） | `serial_chassis.py`/`main.py` **未被 fixer 改动**；`autonomous_state.py` 虽被改（新增 velocity 等），但 `:138` 与 on_enter 仍无坐标系初始化 |
| **B3** 首趟只校验装载、不校验"围栏内侧送达" + VIOLATION 永久卡死 | **仍成立（未改动）** | ① 后果表仍写"本轮结束"：`transport/load_manager.py:41-48`（`VIOLATION_CONSEQUENCES`）；② 装载侧仍只看类型/数量：`load_manager.py` 的 `can_load_batch` 首趟分支（`is_first_trip` 判定）；③ 释放侧仍 `release_all(placement_ok=all_valid)` 后即 `_total_trips += 1`：`load_manager.py:294`；④ **VIOLATION 仍不在 idle 集合**：`transport/transport_pipeline.py:177-178` 与 `:441-442` 仍为 `IDLE, COMPLETE`，而 `:266`、`:337` 仍置 `VIOLATION`；⑤ 投送点仍是旧坐标：`decision/decision_engine.py:550-554`（`(1345,2820)`/`(1655,180)`） | fixer 只把 `can_load_batch` 改成"顺序无关"（修掉了 `[伤员,普通]` 被放行这一**新缺陷**），B3 的**首趟闭环与卡死**本体未动 |
| **B4** 投放判定用车身坐标 | **仍成立（未改动）** | `transport/transport_pipeline.py:404-406`：`positions = [(rx, ry)] * len(self._current_targets)` → `self._placer.classify_batch(positions, infos)` | `transport_pipeline.py` **未被 fixer 改动**（`git status` 无该文件）；目标实际落在车头 140~200mm 的 U 槽外 |
| **B5** 蓝色被误判为危险目标 | **仍成立（未改动）** | `perception/classification.py:110-111`（`LIGHT_BLUE: [BLUE, WHITE]`、`BLUE: [LIGHT_BLUE]` 双向映射）+ `:123-128`（同色任意形状兜底）；`perception/detection.py:37,40`（`BLUE ((95,80,60),(125,255,255))` 与 `LIGHT_BLUE ((85,50,110),(108,255,255))` 在 H 95~108/S 80~255 重叠） | `perception/` 整个目录**未被改动**（`git status src/rescue_robot/perception/` 为空） |
| **B6** CUBE/CUBOID 判据重叠 → 伤员必误判 | **仍成立（未改动）** | `perception/detection.py:55`（`CUBE: (4,8)`）与 `:57`（`CUBOID: (4,8)`）区间完全相同；`:66`（`CUBE` 面积比 `(0.5,1.0)`）覆盖 `CUBOID` 的默认 `(0.2,1.0)`；`:285-301` 按 dict 顺序取首个匹配 → CUBE 必胜 | 同 B5，`perception/` 未改 |
| **B7** 一趟选 3 个只去第 1 个 | **仍成立（未改动）** | `decision/decision_engine.py:349-350`（`select_targets_for_trip(..., max_count=3)`）、`:362`（`self._current_target = self._trip_targets[0]`）、`:381`（`target_ids=[t.id for t in self._trip_targets]`，**3 个一起下发**）；`decision/target_selector.py:184,223`（仍 `max_count=3` + `all_supplies[:max_count]`）；`transport/transport_pipeline.py:309`（用 `t.position` 建所有目标位置）、`:334`（`for t in self._current_targets: load(...)` **一次性全部登记**）；`transport_pipeline.py:404`（3 个目标共用同一坐标判定投放） | `decision_engine.py` 被改过但**多目标选择/装载逻辑一字未动**；`target_selector.py`、`transport_pipeline.py` 未改 |
| **B8** `_check_invalid_transport` 常态打断 | **仍成立（未改动）** | `decision/decision_engine.py:325-337`（只要**任意** ACTIVE 目标距 (1500,1500) < 500mm 即 `return True`）→ `:344-345`（每帧 `self._current_target = None`） | 判据与调用位置都与基线一致 |
| **B9** 上位机侧失控/碰撞保护未落地 + 容错模块 0 引用 | **仍成立（未改动）** | 死模块引用数（排除自身/`__init__.py`/`deploy.py`，实测）：`opponent_strategy 0`、`fault_tolerance 0`、`stability 0`、`imu_fusion 0`、`field_detector 0`、`hot_reloader 0`、`model_switcher 0`、`debug_dashboard 0`、`hardware_profile 0`、`logging_system 0`（**10 个全部为 0**）；接触时长仍未传：`states/autonomous_state.py:280-286` 只新增了 `velocity=self._last_velocity`，**仍无 `contact_duration_s`**；`decision/decision_engine.py:185` 仍是默认 `0.0`；`decision/anomaly_handler.py:206` 判据仍是硬编码 `if contact_duration_s > 10.0` | **部分已变化**：fixer 已重构 `anomaly_handler.check()`（`velocity` 变可选、无里程计信息时不再臆测"无动作"，见 `git diff decision/anomaly_handler.py`），**但**接触链路（谁产生 `contact_duration_s`、谁把它传进来）仍断；看门狗仍刻意不停车（`autonomous_state.py:437` 注释「不再因为 15 秒无动作而 emergency_stop」保留） |
| **B10** 比赛时长硬编码 | **仍成立（未改动）** | `decision/decision_engine.py:98-99`（`MATCH_DURATION_S=180.0`、`TIME_PRESSURE_S=30.0`）、`decision/target_selector.py:59`（`TIME_PRESSURE_S=30.0`，**第二处重复硬编码**）；`:153,157` 消费 | YAML `robot.match.duration_s` 依旧无人读 |
| **B12** 减速带越障模式未接线 | **仍成立（未改动）** | 全仓 `near_speed_bump` 仅 3 处命中：`navigation/navigation_pipeline.py:149`（形参默认 `False`）、`:157`（docstring）、`:187`（消费点）；`states/autonomous_state.py` 调用 `self._navigation.update(...)` 时**未传该参数**；`field_elements` 的 24 个 `SPEED_BUMP` 元素在 `states/`、`navigation/` 中**无任何消费** | 无新增接线 |
| **B13** 决赛颜色无运行时配置 | **仍成立（未改动）** | `perception/target_types.py:199`（`FINAL_TARGETS` 颜色写死 绿/黑/橘/浅蓝）、`:255`（`return FINAL_TARGETS`）；全仓无 `target_color_map` 消费点（`grep target_color_map src/rescue_robot/main.py src/rescue_robot/perception/target_types.py` 无命中） | 与 B24 同源；现场公布颜色后仍需改 `.py` |
| **B24** YAML 字段写了但没用（现场改参数不生效） | **仍成立（未改动）** | `src/rescue_robot/config.py:126-169`：`apply_robot_config` 仍只写 `t = cfg.timing`（`:159`）与 `th = cfg.thresholds`（`:169`）；`motors`/`strategy_weights`/`match`/`fallback`/`perception`/`communication`/`logging` 段**仍无引用**；`main.py:102-109` 仍未调用 `ConfigLoader.merge_with_env(prefix="RESCUE_")`；`config/field.default.yaml` **整文件仍 0 消费点** | 队长已派给 fixer（编号 P1.5），截至本次复核**尚未落地** |
| **B2** 安全区位置偏 300mm + 紫围栏环形建模 | **已驳回（不再论证）** | （本轮实测当前代码几何未变：`field_elements.py` 未改动，红区 y[2670,2970]、蓝区 y[30,330]、紫框 y[2640,3000]/[0,360]） | **队长已用像素实测图 7 驳回本项**，本节仅标注，不重复论证；原报告 §A4/A5/B2 的判定以队长结论为准 |
| **补充核验：`comm_server` 在 AUTONOMOUS 期间是否仍接受入站连接**（"不可遥控"合规关键） | **仍接受连接，但无任何"入站 → 运动"通道 → 判定「符合（软件层不可遥控）」** | ① 服务端**无任何锁判断**：`communication/comm_server.py` 全文无 `is_locked` / `is_autonomous`，`WebSocketCommServer.start(host="0.0.0.0", port=8765)`（`:171`）+ `_run_event_loop`（`:223`）+ `handler`（`:229`）→ AUTONOMOUS 期间端口仍监听、仍接受新连接；② 唯一入站入口有门禁：`comm_manager.py:61` `self._server.on_message(self._handle_message)` → `:131` `_handle_message` → `:141-146` 命中 `BLOCKED_IN_AUTONOMOUS` 即 `return make_error(...)`（`comm_protocol.py:39-42` ＝ `{CONFIG_SET, COMMAND}`）；③ **决定性证据：通信层根本没有运动指令通道** —— `communication/` 目录 `grep -rn '"VEL\|VEL,\|velocity_command\|motion_command\|drive'` **零命中**；`_handle_message` 的 handler 表只有 `CONFIG_GET / CONFIG_SET / PING`（`comm_manager.py:148-153`）；`DebugState` 只负责起停服务（`states/debug_state.py:59-69`）。运动唯一路径是上位机→下位机串口（`hardware/serial_chassis.py:send_velocity`），不经局域网 | **结论：不可遥控这一条在软件层成立**（原因不是"锁得严"，而是通信层没有运动通道）。残留建议：`0.0.0.0:8765` 在比赛中仍开门，建议 AUTONOMOUS 期间 `comm_server.stop()` 或改为只发不收（`DebugState.on_exit` 会 stop，但需确认 AUTONOMOUS 期间是否复用同一实例） |

## 汇总

- **仍成立（未改动）**：B1、B3、B4、B5、B6、B7、B8、B10、B12、B13、B24 —— 共 **11 条**
- **仍成立（部分已变化，需按新行号修）**：B9 —— 1 条（`anomaly_handler.check` 已重构，接触链路与看门狗未接）
- **已修复**：无
- **已驳回**：B2（队长结论）
- **本轮新增核验**：`comm_server` AUTONOMOUS 入站行为 → 符合（软件层不可遥控），见上表末行


### 对修复者的两条提示（仅基于本轮证据）

1. **B3 与 B9 是"改了一半"的两条**，最容易在后续验收中被误判为已完成：
   - B3：`load_manager.can_load_batch` 的顺序无关修复**只关掉了"同一趟 `[伤员,普通]` 被放行"这个衍生缺陷**，首趟"送达围栏内侧"的闭环与 `VIOLATION` 不进 `is_idle` 导致的**永久卡死**（`transport_pipeline.py:177,441-442`）都还在。
   - B9：`anomaly_handler` 的"无动作"判据已被改好，但**没有任何模块产生 `contact_duration_s` 并传给 `DecisionEngine`**（`autonomous_state.py:280-286` 无该实参），所以碰撞保护依然等于没有。
2. **B1 尚未开工**（`serial_chassis.py`/`main.py` 零改动），而它决定"抽到非 3 号出发区时全场错位"，仍是最高优先级的未动项。

### 判定完整性说明

- 本轮只复核 blocker/high（B1~B9）+ 队长点名的 B10/B12/B13/B24 + 被驳回的 B2；medium/low 其余项（B11、B14~B23）**未在本轮重新核验**，沿用原报告结论。
- 未复核项中有一项与 B9 同源需注意：B21（`load_manager` 投错区域"先扣分再全额加回"）证据行 `load_manager.py:283` 仍在，未受 fixer 改动影响。
- **工作树状态声明**：本轮复核使用当前工作区（含 fixer 的**未提交**改动）。复核时工作树**已知存在一处回归**——集成仿真得分从 105 分掉到 0 分（队长已派修）；该回归**不影响**本报告的判定（本报告不依赖集成仿真结果，全部证据来自静态读取与常量实跑），但引用本报告时请知悉工作树非稳定态。

---

## 附录：需求侧硬约束结论（软件负责人视角）

> 来源：req-auditor 提交给队长的六条结论，原样收录（对应 COMPLIANCE_AUDIT.md §五「软件负责人视角：赛项要求对软件的硬约束」，该文档已随 t1 结项冻结，故此处原样保留）。
> 每条格式：**要求原文关键句 → 代码现状（文件:行）→ 判定 → 后果**。

### ① 决赛创新实践环节要现场编程/改参数/调试 → 软件必须支持现场改参数而不重编译

- **要求原文关键句**：「在规定时间内，按照决赛现场发布的决赛命题将自带的零部件更换在参赛作品上，并完成该环节的**编程、装配、调试**等任务」。
- **核查结论（已实测计数）**：**现场可改参数只有 13 项** —— `Timing.*` 8 项 + `Thresholds.*` 5 项（`src/rescue_robot/config.py:166-200` 实数为 13 条赋值）。其余关键参数**必须改代码**：
  - `decision/decision_engine.py:98-99` `MATCH_DURATION_S = 180.0` / `TIME_PRESSURE_S = 30.0`（比赛时长、时间压力）
  - `decision/target_selector.py:59` `TIME_PRESSURE_S = 30.0`（第二处重复硬编码）
  - `perception/target_types.py:199-244` `FINAL_TARGETS` 颜色写死（决赛颜色现场公布 → **改不了**）
  - `perception/classification.py:110-128` + `detection.py:34-51` 检测置信度/颜色阈值写死
  - `states/autonomous_state.py:51-53` `WATCHDOG_*` 看门狗阈值写死
  - `transport/transport_pipeline.py:106` `_push_dist_mm = 100.0`（注释写"真机标定"却不可配置）
- **环境变量覆盖链路实际不通**：`innovation/config_loader.py:181-193` 有 `ConfigLoader.merge_with_env(prefix="RESCUE_")`，但 **`main.py` 从不调用**（`main.py:102-109` 只做 `from_yaml` + `apply_robot_config`）；现场可用的环境变量仅 `RUN_MODE / TEAM_COLOR / CHASSIS_PORT / CAM_INDEX / CAM_WARMUP_S`（`main.py:98,129,142,152,156`）。
- **另**：`config/field.default.yaml` **整文件 0 消费点**（`grep -rn "FieldConfig" src/rescue_robot` 除 `config_loader.py`/`__init__.py` 外无命中），且其中 `safe_zones.*.y`、`speed_bumps.width_mm/height_mm` 数值本身与本项目其它部分不一致 → 想靠它"现场改场地"也无效。
- **判定**：**不符合**（对应 B24）
- **后果**：创新实践环节现场改这几个值不生效、必须改源码 → 该环节扣分／耽误调试时间；用户已确认"运行时长""投放扣分细则"PDF 未给数值，却正落在未接线字段上 → 现场公布后无法快速适配。

### ② 一键启动 + 「规定启动时间内必须离开出发区」

- **要求原文关键句**：「参赛队**一键启动**救援机器人，计时开始，各参赛队救援机器人在**规定启动时间内必须离开出发区**，否则本轮比赛结束」。
- **核查结论（延迟链梳理）**：

  | 环节 | 位置 | 时长 |
  |---|---|---|
  | 长按判定 | `config.py:45` `BUTTON_LONG_PRESS_MS = 500` | 500ms（**需人确认**裁判是否接受长按） |
  | BOOT 自检 | `states/boot_state.py:44-56` → `system_check.py:225-238` | 每传感器 `SENSOR_CHECK_TIMEOUT_MS=3000`、每电机 `MOTOR_CHECK_DURATION_MS=500`、整机 `SELF_CHECK_TIMEOUT_S=10`（`config.py:48-51`） |
  | 摄像头首帧预热（建管线时阻塞） | `main.py:150-164` `CAM_WARMUP_S=3.0` | 最多 3s |
  | 一键启动后固定延迟 | `states/autonomous_state.py:162-164` `time.sleep(POST_START_DELAY_MS)` | 固定 1000ms |
  | 起步 | 主循环 50Hz（`autonomous_state.py:252-256`） | 数十 ms |
- **判定**：**部分不符合** —— 延迟链可控但**未被约束**；自检最坏可吃掉十几秒，且 `POST_START_DELAY_MS` 设计初衷是"等裁判离开"，**与"是否已离开出发区"无联动**。
- **后果**：现场"启动时间"较短时可能**还没动就被判本轮结束（0 分）**。
- **建议修法**：自检并行/可裁剪（`system_check.py:225-238`）；`POST_START_DELAY_MS` 降到 ≤300ms；增加"低速直行 300~500mm 离区"的起步动作并用 `field_elements` 的 `SPEED_BUMP`/出发区区域判离区完成。

### ③ 全自主、不可遥控

- **要求原文关键句**：「救援机器人必须采用**自主运行模式**…允许与笔记本电脑进行通讯，运行过程中不能触碰笔记本电脑，**不能用其他任何方式对救援机器人进行遥控**」。
- **核查结论**：
  - `states/autonomous_state.py:165-180` 进入即锁定（`on_enter` 调 `_lock_external_inputs`）；`:519+` `_lock_external_inputs` 本身只记日志
  - `state_machine.py:184-203` `one_key_start()` 置 `_external_inputs_locked=True`，且 `:129-131` 显式禁止 `AUTONOMOUS → DEBUG`
  - `communication/comm_manager.py:140-146` `is_locked` 时拦截 `BLOCKED_IN_AUTONOMOUS`（`comm_protocol.py:39-42` ＝ `CONFIG_SET`/`COMMAND`），自测 `:273-281` 验证拦截计数
  - **`comm_server.py` 补充核验（本轮完成，见正文表格末行）**：服务端无锁判断、AUTONOMOUS 期间**仍接受入站连接**（`comm_server.py:171,223,229`）；但通信层**根本不存在"入站 → 运动"通道**（`communication/` 目录 `grep '"VEL\|velocity_command\|motion_command\|drive'` 零命中；`_handle_message` handler 表仅 `CONFIG_GET/CONFIG_SET/PING`，`comm_manager.py:148-153`），运动唯一路径是串口 `serial_chassis.send_velocity`。
- **判定**：**符合（软件层不可遥控）** —— 原因不是"锁得严"，而是**通信层没有运动通道**。残留建议：`0.0.0.0:8765` 仍开门，比赛中建议 `comm_server.stop()` 或只发不收。
- **后果**：软件层不构成遥控违规；但若被裁判质疑"端口开放"，需要有"只发不收"的证据配合说明。

### ④ 失控保护 / 碰撞保护

- **要求原文关键句**：「应具备高速移动、避障、越障、救援目标的搜索与转运、对象的识别和信息获取…**并具有碰撞保护、失控保护等功能**」。
- **核查结论（上位机侧未实现）**：
  - `decision/anomaly_handler.py:206` 判据 `if contact_duration_s > 10.0` → `COLLISION_STUCK`（`:34` 定义）**存在但入参恒为 0**：`decision/decision_engine.py:185` 默认 `contact_duration_s: float = 0.0`，`states/autonomous_state.py:280-286` 调用 `self._decision.update(...)` **不传该参数**
  - `decision/opponent_strategy.py:284-299`（7s 预警 / 9s 强制脱离）**全仓 0 引用**（`DecisionEngine.__init__` 只建 `TargetSelector` + `AnomalyHandler`）
  - 上位机看门狗**刻意不停车**：`states/autonomous_state.py:437` 注释「不再因为 15 秒无动作而 emergency_stop」保留；`decision_engine` 把 `EMERGENCY_STOP` 降级为绕圈
  - 现有保护仅下位机速度看门狗（`chassis_serial_protocol.md`：300ms 保持 / 800ms 停）
- **判定**：**不符合**（上位机侧未实现）→ 对应正文 B9
- **后果**：① 接触时软件不知道在接触 → 不主动脱离、白丢时间；② 被裁判强制分离放回出发区后**无任何重置位姿并继续运行的接口调用**（`handle_forced_separation` 无人调用）→ 定位仍是旧位姿，**放回后动作全错、该轮报废**；③ 功能要求项在资格审查时可能被质疑。
- **建议修法**：`DecisionEngine.__init__` 实例化 `OpponentStrategy`；`autonomous_state` 把 `contact_duration_s` 传入 `decision.update()`；`handle_forced_separation` 接到分离事件并同步 `chassis.set_start_pose()` + `localizer.set_pose()`。

### ⑤ 识别能力（含初赛/决赛形状参数）

- **要求原文关键句**：「对象的识别和信息获取（**二维码、条码、文字、图像、形状、颜色、温度、振动**等）」。
- **核查结论**：
  - **已实现**：颜色（`perception/detection.py:34-51` HSV 阈值表）+ 形状（`:54-73` 顶点数/面积比、`:270-301` `_classify_shape`）
  - **未实现**：二维码、条码、文字、图像、温度、振动（`system_check.py:101` 仅有"温度传感器存在性检查"接口，检测链未使用；全仓无二维码/条码/文字识别代码）
  - **初赛/决赛形状参数已配置**：`detection.py:54-61` 覆盖 正方体(4-8)/三棱锥(3-5)/长方体(4-8)/圆柱(8-20)/圆锥台(8-20)/球(8-30)；面积比表 `:64-73`
- **判定**：**部分符合**。缺失项对本赛项**不直接丢分**（救援赛项不靠二维码下发任务）；但已实现两项有硬伤：
  - `detection.py:55` `CUBE (4,8)` 与 `:57` `CUBOID (4,8)` **区间完全相同**，`:66` `CUBE (0.5,1.0)` 覆盖 `CUBOID` 默认 `(0.2,1.0)`，`:285-301` 按 dict 顺序取首个匹配 → **长方体（伤员，15 分）必判成正方体**
  - `detection.py:37` `BLUE ((95,80,60),(125,255,255))` 与 `:40` `LIGHT_BLUE ((85,50,110),(108,255,255))` 在 H 95~108、S 80~255 **完全重叠**，叠加 `classification.py:110-111` 双向映射与 `:123-128` 同色兜底 → **蓝色目标可能被判成危险目标（浅蓝）并被永久排除**
- **后果**：伤员（最高分）漏检/错类，每漏一个丢 15 分；蓝色目标被当危险目标 → 永不转运。对应正文 B5/B6。
- **建议修法**：用长宽比区分 CUBE/CUBOID；删除同色兜底、收窄 LIGHT_BLUE 阈值并优先判 LIGHT_BLUE。

### ⑥ 不得损坏场地设施

- **要求原文关键句**：「比赛过程中（含调试），救援机器人**不得损坏场地等赛场设施**…**若出现场地等被破坏，取消比赛资格**」。
- **核查结论**：
  - **有保护**：硬禁区（对方安全区 + 场边 100mm 边距）写入代价地图并实时校验 —— `navigation/forbidden_zones.py:60-106`、`navigation/navigation_pipeline.py:80,96,264`
  - **无保护**：放置动作是"底盘向斜坡前推 `_push_dist_mm=100.0` + 舵机 4 步渐进 0→70°"（`transport/transport_pipeline.py:373-401`），**没有任何力/电流/堵转/位移上限或超时保护**；`Thresholds.MOTOR_MAX_CURRENT_MA`（`config.py:66`）只在自检阶段使用，运行期无堵转判定
  - 减速带也无"撞到就停"判定（`near_speed_bump` 未接线，见正文 B12）
- **判定**：**部分不符合** → 路径层不会主动撞设施，但**一旦顶到紫色斜坡/围栏/减速带就是持续堵转推挤，无软件级熔断**。
- **后果**：**最严重等级——一旦被判定"破坏场地"直接取消比赛资格**（不只是丢分）。
- **建议修法**：给 `TransportPipeline.PLACING` 加"超时 + 位移增量"双重熔断（推入 ≤2s 或位移 <5mm 即中止抬爪）；把 `MOTOR_MAX_CURRENT_MA`/下位机电流遥测接入运行期堵转检测，超阈即 `chassis.send_stop()`；接上 `near_speed_bump` 并按 `motion_control.BUMP_CROSS_TIME_S` 限速通过。

---

*报告：req-auditor ｜ 复核方式：当前工作区逐文件读取 + `git status` 变更判定 + 关键常量实跑 ｜ 附录来源：软件负责人视角六条结论（原样收录）*

