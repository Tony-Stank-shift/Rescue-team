# 修复记录（FIXES）

> 维护：fixer｜原则：一处问题一处修；每处都写「改了哪个文件、为什么、怎么验证」。
> 验证基线命令（每条修复后必跑）：
> ```bash
> python3 -m compileall -q src
> PYTHONPATH=src python3 -c "import pkgutil,importlib,rescue_robot; ..."   # 全模块导入（59 个）
> PYTHONPATH=src python3 -c "import sys;sys.path.insert(0,'tests');import test_core_units as t; ..."  # 单测 12
> PYTHONPATH=src python3 -m rescue_robot.transport.transport_pipeline      # 转运自测
> PYTHONPATH=src python3 tools/hw_selftest.py --mock                     # 分部自检
> PYTHONPATH=src python3 -c "IntegratedSim 5 种子"                        # 集成仿真
> ```

## 回归基准的变更说明（重要）

| 阶段 | score | delivered | valid | 说明 |
|---|---|---|---|---|
| 初始基线 | 105 | 12/14 | 12 | 其中相当一部分是"一趟带 3 个但只去过第 1 个"的**虚高** |
| 当前 | 70 | 7/14 | 7 | 由 `Placement.SLEEVE_MAX_HOLD=1`（一趟只带 1 个）修正后，**7 个全部有效** |

`SLEEVE_MAX_HOLD` 由另一名成员接线（`config.py` / `transport_pipeline.py:112`），
不是本文件修复项；本节只记录"为什么数字变了"，避免被误判为回归。

---

## 一、已修复（本次）

### 1. P1-5 对方安全区禁入未覆盖全部分支 —— 已修（合规硬规则）
- **文件**：`navigation/navigation_pipeline.py`、`navigation/forbidden_zones.py`
- **为什么**：禁区检查只在"路径跟踪"分支里执行，"接近段(dist<150)"与"到达"分支完全绕过 →
  只要目标点落在对方安全区附近，车会**直接开进去**（规则：进入对方安全区 → 比赛结束）。
- **改法**：① 检查上移到 `update()` 开头，覆盖越障/接近/到达/路径跟踪所有分支；
  ② `set_target()` 先把落在 hard 禁区内的目标点钳制到最近合法点；
  ③ 新增 `ForbiddenZoneManager.clamp_to_safe()`（多禁区迭代推出 + 场心兜底，EPS=1mm）。
- **验证**：`/tmp/p1_5_verify.py` → 接近段/到达/路径三分支均返回 -200；`set_target(1500,200)`→`(1500,381)`；
  2000 次随机采样中所有合法点 100% 保持原值。
- **队长复核**：其导航自检中"目标落在对方安全区内"时车不再冲进禁区（400 步后停在 (1143,1143)），PASS。

### 2. P1-2 载规则对顺序敏感（伤员+普通被放行）—— 已修
- **文件**：`transport/load_manager.py::can_load_batch`
- **为什么**：按列表顺序累加判定 → `[伤员,普通]` 被放行（先看到伤员时车还是空的），
  `[普通,伤员]` 才拦下；同一趟货因顺序不同结论相反，违反"伤员必须单独且一次 1 个"。
  同源漏洞还有 `[伤员,核心]` 与"车上已装伤员后再 `can_load_batch` 装普通"。
- **改法**：改为**与顺序无关**的整体构成判定（先统计本批构成，再判伤员规则/首趟规则/上限）。
- **验证**：`/tmp/p1_2_verify.py` → 15/15（含顺序对称性、伤员单独、≤3、危险拒绝、空批次不崩溃）；
  队长自检器 `--only decision` 由 FAIL 转 **PASS**。

### 3. P2-1 `can_load_batch([])` 崩溃 —— 已修（同一处）
- **为什么**：首趟且已装 1 个时，空批次会走到 `targets[0]` → IndexError；主循环 catch-all 吞掉后表现为"静默不动作"。
- **改法**：空批次早返回（视为"没有装载动作"，不计违规）。

### 4. P1-8 保活链失效（空转/卡围栏时整场卡死）—— 已修
- **文件**：`states/autonomous_state.py`、`decision/decision_engine.py`、`decision/anomaly_handler.py`
- **为什么**：① 看门狗判据是**下发速度** `abs(cmd.linear)>10`，轮子卡住打滑空转时指令一直几百 mm/s
  → 保活计时被无限刷新 → 原地耗到时间结束；② `decision.update` 每帧无条件 `notify_action()` 且传字面量
  `velocity=(0,0)` → 内置"15s 无动作/卡死"判定永不触发；③ 异常分支返回 `WAIT` → 无导航目标 → 车真的停住；
  ④ `_get_survival_target` 半径乘了 0.001，返回点距当前位置仅 ~1.6mm → "保命绕圈"原地不动。
- **改法**：看门狗改按**里程计实际位移**判定（累计位移 ≥40mm 才算在动；下发速度只用于"打滑/堵转"告警）；
  套取/投放阶段（CAPTURING/PLACING）的合法静止不打断；异常各分支改为返回可达点的 `NAVIGATE_TO`；
  `_get_survival_target` 改为绕当前位置 400mm 的真绕圈（靠边角时取多角度中最远点）。
- **验证**：`/tmp/p1_8_verify.py` → 11/11（"下发 400mm/s 但零位移"10s 触发探索、13s 触发保命；
  "有真实位移"永不降级；CAPTURING 不打断；保命点距当前位置 ≈400mm）。

### 5. B1 出发区坐标系写死为 3 号区 —— 已修（0 分级）
- **文件**：`perception/field_elements.py`、`states/autonomous_state.py`、`main.py`
- **为什么**：定位原点硬编码 `(150,150,90°)`（= 3 号区），现场抽签抽到 1/2/4 号区 → 全场地图平移
  ≥2700mm（2/4 号区还多一次镜像），第一步就走错；主流程**从未调用** `set_start_pose()`。
- **改法**：`StandardFieldLayout.get_start_zone()/get_start_pose(zone_id, mode)` 作为出发区中心与朝向的
  **唯一来源**（1/2 号区朝 -Y，3/4 号区朝 +Y）；`AutonomousState` 新增 `start_zone` 参数与
  `_apply_start_pose()`，在 `on_enter()` 里同时下发 ① 下位机 `set_start_pose` ② 导航 `reset_pose`
  ③ 内部 `_pose`；`main.py` 读 `START_ZONE`（默认 3、非法回退）并注入；串口默认端口改 `/dev/ttyS1`
  （RDK 板载 UART1，已实测跑通；电脑 USB-TTL 用 `CHASSIS_PORT` 覆盖）。
- **验证**：`/tmp/b1_verify.py` → 9/9（1~4 号区 → (150,2850,-90°)/(2850,2850,-90°)/(150,150,+90°)/(2850,150,+90°)；
  应用后导航定位器位姿 == 内部位姿 == 出发区中心）。

### 6. B4 投放有效性用车身位置判定 —— 已修
- **文件**：`transport/transport_pipeline.py`、`config.py`
- **为什么**：目标在车头 U 型槽内（前伸 140~200mm），释放瞬间它落在车前方；
  用 `(rx,ry)` 判会系统性偏移一个 L —— 投对了被判无效（丢分/首趟失败），投错了被判有效（首趟假成功 → 后续全无效）。
- **改法**：新增 `drop_position(pose) = (x + L·cosθ, y + L·sinθ)`，`L = config.Placement.DROP_FORWARD_MM`
  （YAML `robot.placement.drop_forward_mm`，**真机标定项**）；`PLACING` 判定改用落点。
- **验证**：车在 (1345,2670) 朝 +Y → 落点 (1345,2820) = 红物资区中心（`/tmp/p15_verify.py` 与手工核对）。

### 7. B3 首趟规则未闭环 + VIOLATION 永久卡死 —— 已修
- **文件**：`transport/load_manager.py`、`transport/transport_pipeline.py`、`decision/decision_engine.py`、
  `states/autonomous_state.py`、`simulation/integrated_sim.py`
- **为什么**：① `release_all()` 空装载也 `+1` 趟次、投歪照样计趟 → 首趟状态被清掉，硬规则被绕过；
  ② `release_all(placement_ok=False)` 先扣 10 分又把目标分值全部加回 → 惩罚被抵消；
  ③ `DecisionEngine` 一见 `release_done` 就 `mark_in_safe_zone + 记 5 分 + 切 FREE_RUN`，从不校验是否
  真的落在"物资区围栏内侧"；④ 后果表把 `OVER_LIMIT/FIRST_TRIP_*` 标成"本轮结束"（规则并无此说法）；
  ⑤ 进入 `VIOLATION` 后既不在 `is_idle()` 集合内也无出口 → **永久卡死**，之后所有 GRIP 被静默丢弃。
- **改法**：① 新增 `_first_trip_done` 作为首趟完成的**唯一判据**，只有"恰好 1 个普通物资 + 有效投进本队
  物资区围栏内侧"才置位；空装载不计趟次；② 投错区**不计分、只扣分**（扣分值可配置，见 §8）；
  ③ `DecisionEngine` 新增 `release_valid` 入参：首趟无效则保持 FIRST_TRIP 重做，FREE_RUN 无效则不
  `mark_in_safe_zone` 不计分（否则世界地图把没运到的目标当已运走 → 永久丢分）；④ 后果表改为"本次转运
  无效"类描述；⑤ `VIOLATION` 状态在 `update()` 里自恢复（作废本趟 → 清目标 → 回 IDLE）；
  ⑥ 顺手修死代码守卫 `if not self.is_idle:`（漏括号，恒 False）与 `start_trip/reset` 未复位
  `_place_started/_place_step`（真机第 2 趟会跳过推入+分步上调）。
- **验证**：空装载 → `trips=0 / is_first_trip=True`；首趟投歪 → 仍 `is_first_trip=True`、score=0；
  首趟投对 → `is_first_trip=False`、score=5；transport 自测通过；5 种子仿真无异常事件。

### 8. P1.5 写死在代码里、YAML 里写了却不生效的参数 —— 已接线
- **文件**：`config.py`、`innovation/config_loader.py`、`config/robot.default.yaml`
- **为什么**：`apply_robot_config()` 只接了 timing/thresholds；`match.duration_s / time_pressure_s /
  nav_timeout_s / grip_timeout_s / transport_timeout_s / fallback.watchdog_* / stuck_* / motors.pid* /
  max_speed_mm_s / wheel_base_mm` 全部写了不生效（决赛"现场改参数不重编译"的硬要求落空）。
- **改法**：`apply_robot_config()` 真接线到 `DecisionEngine` / `TargetSelector` / `AnomalyHandler` /
  `AutonomousState` / `MotionController`（新增 `DEFAULT_MAX_LINEAR_SPEED` 等类级默认值供注入）；
  新增 `PlacementConfig` 与 YAML `robot.placement` 段（`drop_forward_mm / push_dist_mm /
  penalty_per_target`），并加注释「⚠️ 现场公布后可直接改，无需重编译」。
- **验证**：`/tmp/p15_verify.py` → 13/13。改前/改后实测：
  | 项目 | 默认 YAML | 改成 | 生效值 |
  |---|---|---|---|
  | `match.duration_s` | 180 | 60 | `DecisionEngine.MATCH_DURATION_S = 60.0` |
  | `match.time_pressure_s` | 30 | 45 | `DecisionEngine.TIME_PRESSURE_S = 45.0` |
  | `placement.penalty_per_target` | 10 | 7 | `Thresholds.PLACEMENT_PENALTY_PER_TARGET = 7` |
  | `placement.drop_forward_mm` | 150 | 220 | `Placement.DROP_FORWARD_MM = 220.0` |
  （改回默认 YAML 后全部还原。）

### 9. B5/B6 目标识别误判 —— 已修
- **文件**：`perception/detection.py`、`perception/classification.py`
- **为什么**：① `BLUE` 与 `LIGHT_BLUE` 的 HSV 区间严重重叠（H 95~108 / S 80~255 全重叠）且 BLUE 先命中
  → 蓝色救援目标会被判成浅蓝"危险目标"→ 永不被搬（丢分）；② 分类器的"同色任意形状"兜底会把
  蓝色检测直接映射成危险目标；③ `CUBE` 与 `CUBOID` 顶点区间相同、面积比区间又重叠且 CUBE 在前
  → **所有方体都判 CUBE**，伤员（80×40×40，15 分，全场最高）必误判（每个丢 15 分）。
- **改法**：① 检测顺序改为**浅蓝优先**，BLUE 饱和度下界 80→120；② 删除跨分值颜色映射
  （ORANGE→RED/YELLOW、BLACK↔BROWN）；③ 删除无条件"同色任意形状"兜底，改为**受限同色兜底**
  （该颜色在配置表唯一 + 结果不是 DANGEROUS + 颜色本身不是 LIGHT_BLUE）；危险目标只认精确匹配；
  ④ 新增长宽比判据 `SHAPE_ASPECT_RATIOS`（CUBE 0.60~1.45 / CUBOID 1.45~3.60）作方体主判据，
  CUBE 面积比收紧到 0.75~1.0 并显式给出 CUBOID 0.45~0.9。
- **验证**：`/tmp/b56_verify.py` → 11/11（合成图 40×40→CUBE、**80×40→CUBOID**、浅蓝→LIGHT_BLUE、
  饱和蓝绝不被判危险、BLUE+CUBE 不再命中危险目标、ORANGE+CUBOID 仍判伤员）；
  `hw_selftest --only vision` PASS。
- **残留风险（需现场标定）**：真机"浅蓝 vs 蓝"的 H/S 区间本质重叠，只靠"检测顺序 + 分类器闸门"
  区分；若决赛现场公布的蓝/浅蓝无法区分，需现场标定 `HSV_RANGES`（与 B13"颜色运行时配置"同源）。

### 10. B2 安全区几何审计结论 —— **驳回，不改**
- 审计报告 B2 称"安全区整体偏 ~300mm、紫围栏四周全包围、上安全区贴顶边"。队长用 PDF 图 7
  （`docs/scene/pdf_field_page16_hi.png`）做像素级实测（以出发区四角反推场地边界，0.3473 px/mm）：
  上安全区红色内框 596×294mm（≈600×300）、距场地上边 29mm（≈30mm）、水平中心 x=795≈场地中心 794.5，
  紫色围栏正好占场地上边 0~20mm 带，下安全区完全对称。
- **结论**：与现有 `field_elements.py` 几何（红 y[2670,2970]、蓝 y[30,330]、x[1200,1800]、紫边 30mm、
  分区 290 + 隔板 20）**一致**，B2 的"偏 300mm"是像素标定误差（很可能把紫围栏当成红区边界）。
- **处置**：不改几何，也不加"几何存疑"的 TODO（会误导后续审查）。

---

## 二、未修复清单（含原因与建议）

> 依据队长指令："只做会丢分/会卡死的项"。以下按影响度排列，均**未动手**。

| # | 项 | 影响 | 原因（为何本次不修） | 建议 |
|---|---|---|---|---|
| U1 | B7 多目标趟次（已由另一成员以 `SLEEVE_MAX_HOLD=1` 收口） | 曾致自评虚高 3 倍 | 已由他人修（本文件仅记录仿真基准为何从 105/12 变 70/7） | 由该成员或其审查者确认语义与回归基线 |
| U2 | B8 `_check_invalid_transport` 按"任意目标距场心<500mm"清当前目标 | 常态触发 → 目标锁定被打断、车来回摇摆 | 需与 B7/位姿改动一起回归（仿真当前 0 命中，属潜伏） | 改为按**目标身份**判定，且转运中不得复位 |
| U3 | B10/B11 比赛时长与自评分数依据 | 现场非 180s 会提前停手/超时仍在动；自评分数与官方评分规则无对应 | 数值来源用户明确"待现场公布" | 已做成 YAML 可配置（§8）；建议现场按公布值改 YAML 并用官方评分表核对 |
| U4 | B12 减速带越障未接线（`near_speed_bump` 恒 False） | 出发区前 3 根减速带全速直冲，易丢定位/卡住 | 属"起步阶段风险"，非本次 P1 清单 | `_run_once` 里按 `SPEED_BUMP` 元素判距 <250mm 后传参 |
| U5 | B13/B15 决赛颜色运行时配置、`field_detector` 未接线 | 颜色一变则全链路返回 None → 进 DONE（0 分）；`TEAM_COLOR` 写错 = 全运到对方区 | 需现场颜色/硬件 | YAML 增加 `targets:` 段；用摄像头自检本队安全区颜色并在不一致时拒绝启动 |
| U6 | B9 失控/碰撞保护为死代码、看门狗"永不停止" | 真机异常时只能低速绕圈到时间耗尽 | 需硬件联调才能定"何时必须停车" | 把 `fault_tolerance/stability` 的降级链接入 `AutonomousState`；碰撞>10s 交给裁判流程 |
| U7 | B16/B17/B19/B20/B21/B22 决策与世界的细节缺陷 | 保命点/探索覆盖/超时常量未用/世界地图缺子区归属 | 非"丢分/卡死"级，且部分已顺手修（`_get_survival_target`、探索 y 上限、扣分抵消） | 见各条审计原文 |
| U8 | B18 仿真与 `field_elements` 的出发区编号相反 | 仿真结论不能直接迁移真机 | 改编号会影响全部历史仿真结论 | 统一以 `field_elements` 为准，并重跑仿真 |
| U9 | B23 仿真不校验转运规则 | 仿真成绩不能当合规证据 | 属验证工具链（t4/t5 范围） | 仿真内引入 `LoadManager` 同一套规则 |
| U10 | P2 其余：`@property compute_approach`（访问必 TypeError）、`perception.update` 相加 `robot_position` 时忽略 θ、`_extract_opponent` 浮点集合匹配+坐标系错用、串口 `_send` 无 flush / `read_pose` 每帧只读一行 | 真机定位/避障偏移、串口丢帧滞后 | 需要真机/摄像头标定才能验证效果 | 见 `CODE_AUDIT.md` 对应条目 |
| U11 | P3 整洁：`or True` 死条件、`_check_fallback_needed` 死方法、函数内重复 import、多处未用 import | 无直接丢分 | 优先级最低 | 统一清理 |
| U12 | 并发写冲突防护 | `config.py` 曾因并发编辑出现全角冒号 + 重复块 → 全仓 import 阻断约 1 分钟 | 流程问题，非代码问题 | 同文件并发编辑前先通报；提交前统一跑 `compileall` |

---

## 三、待现场标定 / 需人确认（代码已留口）

| 项 | 位置 | 现场做什么 |
|---|---|---|
| 投放落点前伸量 `L` | `config.Placement.DROP_FORWARD_MM` / YAML `robot.placement.drop_forward_mm` | 把目标放进槽里量车心到目标的前伸距离，改 YAML |
| 推入斜坡距离 | `Placement.PUSH_DIST_MM` | 推式放置时实测 |
| 投放错误扣分 | `Placement.PENALTY_PER_TARGET`（默认 10，**来源不明**） | 现场公布后改 YAML |
| 比赛时长 | `match.duration_s`（默认 180，**PDF 未给**） | 按裁判公布值改 YAML |
| 出发区 | `START_ZONE`（1~4，默认 3） | 按抽签结果设置环境变量，`on_enter` 日志会打印起点与朝向供核对 |
| 串口 | `CHASSIS_PORT`（默认 `/dev/ttyS1`） | 电脑 USB-TTL 调试时设为 `/dev/ttyUSB0` |
| 浅蓝/蓝 HSV | `perception/detection.py::HSV_RANGES` | 现场用 `tools/vision_calibration.py` 标定 |
| 相机倾角/套取 ROI | `config.Camera.TILT_DEG / SLEEVE_ROI` | 现场标定（未标定时测距与套取确认不可信） |
| 套取机构物理容量 | `config.Placement.SLEEVE_MAX_HOLD`（默认 1） | 由机构组确认；调大前必须重跑集成仿真 |

---

## 四、验证证据索引

验证脚本已**固化进仓库**（不放 /tmp，避免被环境清理导致无法复现）：

```bash
# 一次跑完全部修复验证（6 个脚本，共 66 项断言）
PYTHONPATH=src python3 tools/fix_verifiers/run_all.py
```

| 脚本（`tools/fix_verifiers/`） | 覆盖 | 结果 |
|---|---|---|
| `verify_p1_2_load_rules.py` | 载规则顺序无关 / 伤员单独 / ≤3 / 危险拒绝 / 空批次不崩溃 | 15/15 |
| `verify_p1_5_forbidden_zones.py` | 禁区检查覆盖接近段·到达·路径三分支 + 目标点钳制 | 7/7 |
| `verify_p1_8_watchdog.py` | 看门狗按实际位移判定 + 异常链真实运动 + 保命点距离 | 11/11 |
| `verify_p15_config.py` | YAML 配置真生效（`duration_s` 180→60 等，改前/改后） | 13/13 |
| `verify_b1_start_zone.py` | 出发区 1~4 坐标系初始化（含导航定位器同步） | 9/9 |
| `verify_b5b6_perception.py` | 40×40→CUBE / **80×40→CUBOID** / 浅蓝 vs 蓝防误判 | 11/11 |

| 命令 | 结果 |
|---|---|
| `python3 -m compileall -q src` | OK |
| 全模块导入（`pkgutil.walk_packages`） | 59 ok / 0 failed |
| `tests/test_core_units.py`（直接调用，pytest 不可用） | 12 passed / 0 failed |
| `python3 -m rescue_robot.transport.transport_pipeline` | 全部通过 |
| `python3 tools/hw_selftest.py --mock` | **PASS=6 FAIL=0 SKIP=7** |
| 集成仿真 5 种子 | score 70~80 / delivered 7/14 / **valid 7**（见文件开头基准变更说明） |
