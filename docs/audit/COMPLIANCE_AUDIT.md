# 智能救援赛项 · 竞赛要求合规审计报告

- 审计对象：`/home/ony_uang/Rescue-team`（Python 上位机，commit `0e8a5ae`）
- 依据：`附件2-1-2026第十五届上海工创大赛暨2027第十届国赛-智能+工程创新赛道命题与运行（发布稿）.pdf`「三、智能救援赛项」（第 14~20 页，文本第 324~473 行）
- 要求原文文本：`/tmp/req_topic.txt`（第 324~473 行）；场地/减速带/安全区几何另用 pdfplumber 提取图 7/8/9 实测校核
- 审计范围（用户 2025 收窄）：**纯软件**。硬件（尺寸重量、串口电气、摄像头成像、机械强度）只列「需人确认」，不作结论。
- 口径：凡能靠读代码判定的，给**确定结论**；每条不符合项都回答「比赛时丢什么分 / 卡在哪一步」。
- 纪律：只报有证据的问题；无证据的标「无法判定」并说明缺什么。
- **审计基线**：本次结论基于 `git show 0e8a5ae` 的工作树快照。审计期间修复者已并发改动 3 个文件（`navigation/forbidden_zones.py`、`navigation/navigation_pipeline.py`、`transport/load_manager.py`），因此这些文件的行号以基线为准；核对差异用
  `git diff 0e8a5ae -- <文件>`。其中 `load_manager.can_load_batch` 已被改为"与顺序无关"的判定（修复了 `[伤员, 普通]` 顺序被放行的缺陷），B3 中引用的 `can_load_batch` 行号需按改动后代码重定位；B3/§7 引用的 `load_manager.py:283 penalty = 10 * len(released)` 在队长复核时仍存在。
- **章节地图**：一、逐条对照表｜二、不符合项清单（B1~B24）｜三、需人确认｜四、当前软件达成度判断（含风险 Top3 与修复顺序）｜**五、软件负责人视角：赛项要求对软件的硬约束（6 条）**｜**六、「写了但没用」配置清单（现场改参数不生效，B24）**｜**七、待现场公布项（运行时长 / 投放扣分细则）**

---

## 一、逐条对照表

| # | 要求（官方原文要点） | 代码现状（文件:行） | 判定 | 比赛影响 |
|---|---|---|---|---|
| A1 | 场地 3000×3000mm；四周围栏厚度<20mm、高度<100mm | `perception/field_elements.py:94-96`（`FIELD_SIZE=3000`/`FENCE_THICKNESS=20`/`FENCE_HEIGHT=100`） | 符合 | — |
| A2 | 出发区 1/2/3/4，洋红色，现场抽签确定出发位置 | 几何 4 个 300×300 角落区：`field_elements.py:162-176`；**但实际起点写死 (150,150,90°) = 3 号区**：`hardware/chassis_interface.py:58`、`hardware/serial_chassis.py:31-34`、`states/autonomous_state.py:130`；`main.py` 只读 `TEAM_COLOR`，**全程无出发区编号参数、无 `set_start_pose()` 调用** | **不符合** | **抽到 1/2/4 号区 → 全场坐标系整体错位（≥2700mm），第一趟就到不了物资区，几乎 0 分**（详见 B1） |
| A3 | 安全区红/蓝各一个，物资区与伤员区各占二分之一，中间隔板 300×20×20mm | 隔板常量 `field_elements.py:108`；分区 290+20+290 居中切分 `:180-181,199-208,225-234`；图 8 实测隔板偏向伤员侧（物资区≈300、伤员区≈280） | 部分符合 | 分区宽度偏差 ≤10mm，对投放判定影响小；但**安全区整体位置错 300mm**（见 B2） |
| A4 | 安全区面向场地一侧为紫色直角三角形围栏，便于从外面把目标运进去 | 紫色围栏按**四周全包围**建模：`field_elements.py:186-193, 212-219`（红 x[1170,1830] y[2640,3000]；蓝 x[1170,1830] y[0,360]） | **不符合** | 代码认为安全区**四周都有 30mm 紫色坎**，而实际只有面向场地一侧有斜坡入口 → 放置路径与判定都按"全包围"算（详见 B2） |
| A5 | 安全区距场地边沿的实测位置（图 7 标定为：上安全区 y 0~300 贴顶边；下安全区 y 2800~3081） | 代码红区 y 2670~2970、蓝区 y 30~330（`field_elements.py:184, 211`） | **不符合** | 投送点/判定区整体偏 300mm，与 A4 同源（详见 B2） |
| A6 | 出发区前有长方形减速带 3 根、间隔 50mm（图 9：300×60×10mm） | 挂边两组共 6 块/区、共 24 块：`field_elements.py:111-115, 236-265`；实测与图 7 完全一致（与出发区内边平齐、300×60、净距 50mm）；图 9 截面 300×60×10 与常量一致 | 符合 | 图 7 与文字"3 根"存在差异，按图建模是对的（仅需人确认裁判口径） |
| B1 | 初赛目标：8 绿正方体40 + 4 黑正三棱锥40 + 4 橘长方体80×40×40(PLA) + 4 浅蓝正方体40 | `perception/target_types.py:147-192`（尺寸/颜色/形状/分值齐全）；`perception/detection.py:433-438` Mock 数量 8/4/4/4 | 符合 | — |
| B2 | 决赛目标：10 φ40×60 圆柱 + 5 φ40→φ30×60 圆锥台 + 5 伤员长方体 + 5 φ40 球；**颜色现场公布** | 形状/尺寸表正确：`target_types.py:199-244`；但颜色写死为 绿/黑/橘/浅蓝 | 部分符合 | **决赛颜色现场公布时，分类查表 + 模糊匹配都找不到目标 → 场上"看不到目标"，决赛几乎 0 分**（详见 B6） |
| C1 | 出发后**必须先单独**把 1 个普通物资送到本队安全区**物资区围栏内侧**（多于一个无效），之后才可转运其它目标 | `transport/load_manager.py:155-162`（首趟必须 REGULAR_SUPPLY 且 count==1）；但**只校验"装的是 1 个普通物资"，不校验投放位置是否在物资区围栏内侧**；`decision/decision_engine.py:300-310` 只要 `release_done` 就无条件标记成功 | **不符合** | 首趟投在围栏外/伤员区也算通过 → 整轮比赛在规则上仍处于"步骤 1 未完成"，**后续运的 15 个目标全部无效（等于 0 分）**，而软件自己以为在得分（详见 B3） |
| C2 | 之后一次转运不超过 3 个，超过无效；普通+核心可混装 | `load_manager.py:89`（`MAX_LOAD=3`）、`:164-167`；`target_selector.py:217-223`（普通+核心混装取最近 3 个） | 符合（规则层） | 但**执行层一趟只抓得到第 1 个**（见 C3/ B7） |
| C3 | 伤员必须单独转运、一次只能 1 个 | `load_manager.py:172-179`、`:191-195`（伤员不能与任何目标混装）；`target_selector.py:209-214`（伤员单独成趟） | 符合 | — |
| C4 | 不得把危险目标转运进安全区 | `load_manager.py:151-153` 拒绝 DANGEROUS 装载；`target_selector.py:83-84, 153` 分值 0 被过滤 | 符合 | — |
| C5 | 禁止抓取救援目标、禁止把目标放在机器人上（只能推/套） | 全部行走/装载路径都是"套取"：`transport/sleeve_lift.py`（U 型槽 + 舵机 RAISE/LOWER/HOLD）+ `transport/transport_pipeline.py:303-341`；**代码里没有夹持/抓取动作**，也没有"把目标背在车上"的路径 | 符合（软件层） | 放置用"U 槽 + 斜坡推入"（`transport_pipeline.py:373-412`），符合"推/套"；**没有任何判据或日志能证明没"抓取"** → 完全靠机械设计，需人确认 |
| C6 | 不能进入对方安全区 | 对方安全区作为硬禁区写入代价地图并实时校验：`navigation/forbidden_zones.py:60-84`、`navigation/navigation_pipeline.py:80,96,264` | 符合（软件层） | 判据来自本机定位；**定位错位（B1/B2）时禁区位置也跟着错**，等于没有保护 |
| C7 | 不能把对方安全区内的目标以任何方式移出 | 无实现、无判定、无规避 | **未实现** | 不主动产生违规动作（代码不会去对方区），但**无法保证**（定位漂移进入对方区或推挤到对方目标时无任何检测/回避） |
| C8 | 双方不得主动恶意进攻；接触超过 10s 强制分离并放回出发区、计时不中断 | `decision/opponent_strategy.py:284-299`（7s 预警/9s 强制脱离）**但 `decision_engine.py` 从未构造/调用该模块**（全仓无引用）；`decision_engine.update(contact_duration_s=0.0)` 默认 0，`states/autonomous_state.py:260-265` **调用时未传 `contact_duration_s`**，`anomaly_handler.check(..., contact_duration_s)` 恒为 0 | **未实现** | 发生接触时**软件永远不知道在接触**：不会主动脱离（不违规但白丢时间），也**没有"被裁判放回出发区后重置位姿/继续运行"的接口调用**（`handle_forced_separation` 无人调用）→ 放回后定位仍是旧位姿，后续动作全部错乱，**该轮基本报废** |
| C9 | 转运至安全区的无效救援目标将被取出重新随机放置在场地中央 | 撤除后的重投放只由 `decision_engine.py:314-326` 的"目标离场地中心<500mm 就重选目标"隐式兜底 | 部分符合 | `_check_invalid_transport` 只要发现**任意**活跃目标在中心 500mm 内就返回 True（场地中心本来就是目标常置区）→ 每帧把 `_current_target` 清空重选，**持续打断当前趟次目标锁定**（详见 B8） |
| C10 | 规定运行时间到 / 目标全部移入安全区（不含围栏上）→ 比赛结束 | `decision_engine.py:226-234`（时间到/无活跃目标 → DONE） | 符合 | 但 `MATCH_DURATION_S=180.0`（`decision_engine.py:98`）**是硬编码假设**，本要求原文未规定分钟数 → 若现场不是 3 分钟，会提前 3min 停手或超时后仍在动（详见 B10） |
| D1 | 全程自主运行、不可遥控 | `state_machine.py:184-203` 一键启动后 `is_locked=True` 且不可逆；`communication/comm_manager.py:140-144` 锁定后拦截所有下行指令 | 符合 | — |
| D2 | 一键启动 | `state_machine.py:184-203`（`one_key_start` → AUTONOMOUS） | 符合 | 触发路径依赖下位机 `EVENT,START_BUTTON` → `hardware/button.py` → `one_key_start()`；链路是否真通需人确认 |
| D3 | 规定启动时间内必须离开出发区 | 无"出发区计时/离开判定" | **未实现** | 软件不会判"超时未离区"（这本来由裁判判），但**也没有任何"起步失败就重试/脱困"的兜底**：起步卡住 → 按 `autonomous_state.py:309-314` 的 50Hz 循环 + 10s 看门狗，只会进入"探索/保命绕圈"，**该轮 0 分** |
| D4 | 只能使用一个随机器人装载的电源，比赛全程不能更换 | 软件无关 | 无法判定（需人确认） | — |
| E1 | 识别能力：二维码/条码/文字/图像/形状/颜色/温度/振动 | 仅实现颜色+形状：`perception/detection.py:34-51`（HSV）、`:270-301`（顶点数/面积比）；`system_check.py:101` 有温度传感器检查接口但检测管线未用；无二维码/条码/文字/图像/振动识别 | 部分符合 | 本赛项现场不靠二维码分发任务，**缺这些能力不直接丢分**；但颜色/形状识别本身有 3 处硬伤（B5/B6），那才是丢分点 |
| E2 | 具有碰撞保护、失控保护 | 指令超时保护在下位机（`chassis_serial_protocol.md` 速度看门狗 300ms/800ms）；上位机看门狗**刻意不停止**（`states/autonomous_state.py:343-362`，15s 仍保命运动）；`robustness/fault_tolerance.py`、`decision/opponent_strategy.py` 定义了保护但**全仓 0 引用** | **未实现（上位机侧）** | 失控时上位机不接管、不急停；且**降级/容错模块是死代码**，真机进入异常只能靠"绕圈"（详见 B9） |
| F1 | 尺寸 ≤300×300mm、高 ≤200mm、重量 ≤1.5kg | 代码无法体现（`Camera.HEIGHT_MM=210.0` 超 200mm，是相机光心高度，非车高） | 需人确认 | — |
| F2 | 外表面禁止误导性图案及救援目标颜色 | 代码无法体现 | 需人确认 | — |
| F3 | 安全：禁危险机构/干扰设备；非金属高速旋转件 | 代码无法体现 | 需人确认 | — |
| G1 | 任务命题文档（策划决赛场景/目标/转运方式） | 文档相关：`README.md:100-120` 有内容要求摘要；无文档模板产出 | 无法判定（非代码） | — |
| G2 | 创新实践环节：现场改零部件/编程/装配/调试 | 宣称支持现场改配置：`innovation/config_loader.py`（YAML 载入，`main.py:102-109`）；但 `innovation/hot_reloader.py`、`model_switcher.py`、`debug_dashboard.py`、`hardware_profile.py`、`deploy.py` **全仓 0 引用**，`config_loader` 只套用 timing/thresholds（`config.py:126-148`），**策略权重并未生效** | 部分符合 | 现场只能改 YAML 里的时序/阈值；策略参数改了不生效 → 创新实践环节少一份可展示能力（不直接丢转运分） |
| G3 | 评分规则（分值） | `target_types.py:258-266` 写死 普通5/核心10/伤员15 分；本次提供的附件**不含评分规则表**（第 19 页仅引用"评分规则"） | 无法判定 | `LoadManager.total_score`、`DecisionEngine._score` 全是**自造分数**，与官方规则无任何一致性保证；只能当内部排序用（详见 B11） |
| G4 | 现场比赛 2-3 场，取平均值；同分比总分/目标数 | 无多场次/成绩记录 | 未实现（非上位机职责） | 不丢分（裁判侧统计） |

---

## 二、不符合项清单（按严重度）

> 共 **24 条**：blocker 4（B1~B4）／high 5（B5~B9）／medium 7（B10~B15、B24）／low 8（B16~B23）。

> 每条格式：**问题 / 证据 / 影响（丢什么分·卡在哪一步）/ 建议修法**

### 🔴 blocker

#### B1. 出发区坐标系写死为 3 号区，现场抽签抽到别的区 = 全场错位
- **问题**：机器人定位原点硬编码为 (150,150)、朝向 90°，等价于"永远从 3 号出发区（左下）出发"。现场是**抽签决定 1/2/3/4 号出发区**的，且 4 个区都在角落、朝向各不相同。`main.py` 只提供了 `TEAM_COLOR`，**没有任何出发区/朝向参数，也从未调用 `set_start_pose()`**。
- **证据**：
  - `src/rescue_robot/hardware/serial_chassis.py:28-34`
    ```python
    def __init__(self, port='/dev/ttyUSB0', baudrate=115200, timeout=0.02,
                 start_x_mm: float = 150.0,
                 start_y_mm: float = 150.0,
                 start_theta_rad: float = 1.5707963267948966):
    ```
  - `src/rescue_robot/states/autonomous_state.py:130` → `self._pose: tuple = (150.0, 150.0, 1.5707963267948966)`
  - `src/rescue_robot/main.py:128-130, 142` → 只读 `TEAM_COLOR`；`grep -rn "set_start_pose" src/rescue_robot` 命中全部是各模块 `__main__` 自测段，主流程 0 调用
  - 反证：仿真侧是**对**的，`simulation/sim_2d.py:182-187` 明确接 `start_zone` 并用 `START_CENTERS`
- **影响（丢什么分/卡在哪一步）**：抽到 1 号区时真实起点 (150,2850)，软件认为 (150,150) → **全场地图平移 2700mm**；抽到 2/4 号区还多一个镜像错误。第一步"导航到普通物资"的目标坐标就完全错位，车会直奔错误方向 → **最坏情况第一趟就卡死/撞围栏，整轮 0 分**；就算不撞，也拿不到任何合法转运。
- **建议修法**：
  1. `main.py` 增加 `START_ZONE`（1~4，默认 3）与 `START_HEADING`（默认"朝场地中心"）环境变量/YAML 读取；
  2. 用 `FieldLayout.standard().get_start_zones()`（`field_elements.py:276-277`）取 `region.center` 作为原点，按区号决定朝向（1 号→朝 +X/-Y 中指向场地中心的一侧，以此类推）；
  3. 在 `AutonomousState.on_enter()`（`autonomous_state.py:137`）里、`start_match()` 之前调用：`self._chassis.set_start_pose(x, y, theta)`（`serial_chassis.py:100`）+ `self._navigation.localizer.set_pose(...)`（`navigation/localization.py`）；
  4. 比赛前打印一条 `起点 = 出发区 N 中心 (x,y) 朝向 θ` 的日志用于现场核对。

#### B2. 安全区整体位置与紫色围栏形状建模错误（偏 300mm + 四周全包围）
- **问题**：代码把安全区放在"离场地边 30mm"处并四周包 30mm 紫边。官方图 7 实测标定：**上安全区紧贴顶边（y 0~300，紫色围栏与场地边沿重合）**、下安全区 y 2800~3081；且紫边只在**面向场地的一侧**是直角三角形斜坡。
- **证据**：
  - 代码：`perception/field_elements.py:184` → `red_safe_y = FIELD_SIZE - SAFE_ZONE_HEIGHT - FENCE_THICKNESS_SAFE = 2670`；`:211` → `blue_safe_y = FENCE_THICKNESS_SAFE = 30`；`:186-193`/`:212-219` 把紫围栏建成 `SAFE_ZONE_WIDTH+2*30 × SAFE_ZONE_HEIGHT+2*30` 的**环形包围盒**
  - 运行态实测（`PYTHONPATH=src python3 -c ... StandardFieldLayout()`）：
    ```
    紫色围栏 (红) x[1170,1830] y[2640,3000]
    安全区 (红)   x[1200,1800] y[2670,2970]
    紫色围栏 (蓝) x[1170,1830] y[0,360]
    安全区 (蓝)   x[1200,1800] y[30,330]
    ```
  - 图 7 实测（pdfplumber 600dpi，标尺 300mm≈441px）：上安全区紫框 y 29~400、红区 y 0~300；下安全区紫框 y 2776~3147、蓝区 y 2800~3081；紫框 x 1341~2021（居中，与代码 x[1170,1830] 一致）
  - 消费方：投送点 `decision/decision_engine.py:493-503`（红物资区 `(1345, 2820)`）、投送判定 `transport/safe_zone_placer.py:64-65`（`FULLY_INSIDE_MARGIN_MM=10`）
- **影响（丢什么分/卡在哪一步）**：投送点比真实物资区偏 300mm，车会把目标**推到围栏外/围栏上**（规则：围栏上不计分、围栏外该趟无效）；而且首趟要求"物资区围栏内侧"，投歪 → 触发 B3，**整轮 15 个目标全部作废 = 0 分**。卡点在第 2 趟"运送至物资区"这一步。
- **建议修法**：
  1. `field_elements.py`：`red_safe_y = FIELD_SIZE - SAFE_ZONE_HEIGHT`（=2700），`blue_safe_y = 0`；把 `PURPLE_FENCE` 改成非环形——仍建 660×300 外框，但 `metadata` 标注 `open_side`（红区朝下 / 蓝区朝上），并只在**面向场地一侧**生成挡边；
  2. 同步 `decision_engine._get_supply_area_position()/_get_injured_area_position()`（物资区 y 中心 = 2850 红 / 150 蓝）；
  3. 同步 `navigation/path_planner.py` 的安全区进入点与 `forbidden_zones.py:48-50` 的 `SAFE_MARGIN_MM`（避免把紫边斜坡也算成禁区）。

#### B3. 首趟只校验"装的是 1 个普通物资"，不校验是否送达**物资区围栏内侧**；首趟失败还会硬退出 FIRST_TRIP
- **问题**：转运规则最核心的"步骤 1 未完成则后续全部无效"没有落地判定；`DecisionEngine` 只看 `release_done` 就 `mark_in_safe_zone` 并加 5 分，然后**永久切到 FREE_RUN**。即使首趟投在伤员区/围栏外（`SafeZonePlacer` 已判 invalid），也照样进 FREE_RUN。
- **证据**：
  - `decision/decision_engine.py:291-310`
    ```python
    safe_region = self._get_supply_area_position()
    if not release_done:
        return Action(type=ActionType.TRANSPORT_TO, ...)
    # 投放完成 → 进入 FREE_RUN
    self._world_map.mark_in_safe_zone(self._current_target.id)
    self._targets_delivered += 1
    self._score += get_point_value(TargetType.REGULAR_SUPPLY)
    self._strategy_state = StrategyState.FREE_RUN
    ```
  - `transport/load_manager.py:43-48`：`FIRST_TRIP_MULTI`/`FIRST_TRIP_WRONG_TYPE` 的后果被写成 **"本轮结束"**（规则只写"多于一个无效"，没有本轮结束）
  - `transport/load_manager.py:161-162, 199-203`：首趟不合格是**拒绝装载**；而 `transport_pipeline.py:262-267` 在拒绝时把 `_phase = TransportPhase.VIOLATION`，`AutonomousState` 只在 `is_idle()` 时再 `start_trip`（`autonomous_state.py:271-277`），`VIOLATION` 不在 `is_idle` 集合里（`transport_pipeline.py:441-442`）→ **转运管线永久卡死，之后所有 GRIP 动作被静默丢弃**
  - 投放判定用的是**机器人位置**而不是目标落点：`transport_pipeline.py:404-406` `positions = [(rx, ry)] * len(...)`；且 `safe_zone_placer.py:64-65` 的 `FULLY_INSIDE_MARGIN_MM=10` 对 40~80mm 的目标过严
- **影响（丢什么分/卡在哪一步）**：两种丢法：①首趟投歪但自认成功 → 程序继续运，**规则上整轮 0 分**；②首趟判定/装载被拒 → `VIOLATION` 卡死转运管线 → **一次都运不进去，0 分**。卡点：第一趟投放。
- **建议修法**：
  1. `LoadManager` 增设 `first_delivery_valid: bool`，由 `TransportPipeline` 在释放时用**目标落点**（见 B4）判定的 `PlacementResult.zone == SUPPLY_AREA` 回填；`_total_trips` 只在**首趟有效**后才递增，`is_first_trip_done` 以它为唯一来源；
  2. `VIOLATION_CONSEQUENCES` 按规则改：`OVER_LIMIT`→"本次超出的目标无效"、`FIRST_TRIP_MULTI`→"该次无效，需重新单独送 1 个"（**都不得设成本轮结束**）；`is_fatal` 仅保留"进入对方安全区/移出对方目标"这类真·结束条件；
  3. `AutonomousState`/`TransportPipeline`：`VIOLATION` 必须可恢复——在 `update()` 里把 `VIOLATION` 视为"本趟作废"回 `IDLE`（并在 `_get_status` 上报 violation），否则永久卡死；
  4. `_handle_first_trip` 里只在**有效入区**时切 FREE_RUN，失败则保持 FIRST_TRIP 并重新选最近的普通物资。

#### B4. 投放有效性用"机器人位置"代替"目标落点"
- **问题**：释放瞬间用底盘位置 (rx,ry) 代替目标实际坐标做区域判定。
- **证据**：`transport/transport_pipeline.py:403-415`
  ```python
  positions = [(rx, ry)] * len(self._current_targets)
  infos = [t.info for t in self._current_targets]
  results = self._placer.classify_batch(positions, infos)
  all_valid = all(r.is_valid for r in results)
  ...
  self._load_mgr.release_all(placement_ok=all_valid)
  ```
  靶标在车头 140~200mm 的 U 型槽内（`sleeve_lift.py` U 槽几何），**释放时它落在车前方远处**，用车身坐标必然系统性偏差。
- **影响（丢什么分/卡在哪一步）**：本趟实际投对了却被判 invalid（扣 10 分/个 + 不认首趟）、或投错了被判 valid（首趟假成功 → 整轮作废）。**直接决定首趟成败，也就是 0 分与满分的分界**。
- **建议修法**：在 `TransportPipeline.PLACING` 里按机器人位姿 `(rx, ry, rtheta)` + U 槽前伸距离 `L≈150~200mm` 推算落点：`pos = (rx + L*cos(rtheta), ry + L*sin(rtheta))`，用于 `classify_batch`；把 `L` 作为标定参数暴露到 `config`/YAML；同时把 `FULLY_INSIDE_MARGIN_MM` 改为**按目标尺寸**判定（例如 `margin = max(10, max(size_mm)/2)`）。

### 🟠 high

#### B5. 目标分类器会把蓝色目标误判成"危险目标"（同色兜底逻辑）
- **问题**：`BLUE → [LIGHT_BLUE]` 的容差映射让**任何"蓝色 + 当前形状不匹配"的检测**都被归为浅蓝，而浅蓝在初赛=`危险目标`。蓝色又是复赛/决赛现场公布的候选目标颜色，也是场地/出发区元素颜色。
- **证据**：`src/rescue_robot/perception/classification.py:105-128`
  ```python
  color_similarity = {
      TargetColor.LIGHT_BLUE: [TargetColor.BLUE, TargetColor.WHITE],
      TargetColor.BLUE: [TargetColor.LIGHT_BLUE],
      ...
  }
  for sim_color in similar_colors:          # 1. 同形状 + 相近颜色
      if (sim_color, shape) in self._config: return info
  for (c, s), info in self._config.items(): # 2. 同颜色 + 任意形状（宽松兜底）
      if c == color: return info
  ```
  且 HSV 阈值本身重叠：`detection.py:37-40`
  ```python
  TargetColor.BLUE:       ((95, 80, 60), (125, 255, 255)),
  TargetColor.LIGHT_BLUE: ((85, 50, 110), (108, 255, 255)),   # 与 BLUE 在 H 95~108、S 80~255 完全重叠
  ```
- **影响（丢什么分/卡在哪一步）**：真实的蓝色救援目标被当成危险目标 → `TargetSelector.score_target` 剔除（`target_selector.py:83-84`）→ **该目标永远不会被运，白丢 5/10/15 分**；更糟的是"蓝色 + 未匹配形状"的场地元素被持续当作危险目标，污染世界地图。卡点：目标选择阶段。
- **建议修法**：
  1. 删除"同色任意形状"兜底（`classification.py:123-128`），改为一律返回 `None`（宁可漏检不误分类）；
  2. 提高 BLUE 下界、收窄 LIGHT_BLUE：`LIGHT_BLUE = ((90, 20, 160), (105, 140, 255))` 一类"低饱和 + 高亮度"判据，并**先判 LIGHT_BLUE 再判 BLUE**；
  3. 给 `TargetInfo` 增加 `is_dangerous` 白名单式确认：只有 (LIGHT_BLUE, CUBE/SPHERE) 且置信度 ≥ 阈值才允许判危险，其余一律 UNKNOWN。

#### B6. 形状分类器无法区分"正方体"与"长方体"→ 伤员（15 分，最高分目标）必被误判
- **问题**：`CUBE` 与 `CUBOID` 的顶点区间完全相同（4~8），面积比区间又重叠（CUBE 0.5~1.0 覆盖 CUBOID 的默认 0.2~1.0），遍历顺序里 `CUBE` 在前 → **所有方体都会返回 CUBE**。而初赛/决赛的伤员都是长方体（80×40×40）：初赛橘色 → `(ORANGE, CUBE)` 查表失败 → 模糊匹配落到 `ORANGE` 的 `CUBOID`（碰巧对）；一旦橘色被检测成红/黄（`classification.py:112-115` 把 ORANGE 的相近色映射到 RED/YELLOW），就会命中 `(RED, CUBE)` 等**错误目标**。决赛"颜色现场公布"时问题更严重（见 B13）。
- **证据**：
  - `detection.py:54-73`
    ```python
    SHAPE_VERTEX_RANGES = { TargetShape.CUBE: (4, 8), TargetShape.CUBOID: (4, 8), ... }
    SHAPE_AREA_RATIOS  = { TargetShape.CUBE: (0.5, 1.0), ... }   # CUBOID 无条目 → 默认 (0.2,1.0)
    ```
  - `detection.py:285-301`：按 dict 顺序取第一个"面积比也匹配"的候选 → CUBE 必胜
  - `classification.py:112-120`：ORANGE↔RED/YELLOW、BLACK↔BROWN 的双向相似映射会**跨界命中不同分值的类型**
- **影响（丢什么分/卡在哪一步）**：伤员（15 分，全场最高分）被误判成普通物资（5 分）或"根本不存在的组合"而被丢弃 → **每漏一个伤员丢 15 分，最多丢 60 分（初赛 4 个）/75 分（决赛 5 个）**；同时可能把它们送进物资区，被规则判"伤员放物资区"。卡点：识别阶段（还没开始运就错了）。
- **建议修法**：
  1. 用**长宽比 + 实心度**区分 CUBE/CUBOID（80×40 的长方体俯视长宽比≈2:1）：新增 `aspect_ratio` 判据，`ratio<1.3` → CUBE，`>1.5` → CUBOID；
  2. 把 CUBE 的 `SHAPE_AREA_RATIOS` 收紧到 `(0.75, 1.0)`，给 CUBOID 显式加 `(0.45, 0.8)`；
  3. 删除跨分值颜色的模糊映射（ORANGE→RED/YELLOW、BLACK→BROWN），改为返回 `None` + 计数告警。

#### B7. 多目标趟次是"假装载"：一趟选 3 个，只去第 1 个，却把 3 个都记为已转运
- **问题**：`TargetSelector` 会一次返回最多 3 个目标（普通+核心混装），`DecisionEngine` 把它们全部放进 `target_ids`，`AutonomousState` 把 3 个 id 一起交给 `start_trip`；`TransportPipeline` 导航到 `_current_targets[0]` 后就对**所有** id 执行 `load()`。第 2、3 个目标**车根本没去过**。
- **证据**：
  - `decision/target_selector.py:216-223`（`all_supplies[:max_count]`，`max_count=3`）
  - `decision/decision_engine.py:337-371`：`self._trip_targets = select_targets_for_trip(..., max_count=3)`；`GRIP` 动作带 `target_ids=[t.id for t in self._trip_targets]`；导航目标只有 `self._current_target.position`（= `_trip_targets[0]`）
  - `states/autonomous_state.py:271-277`：`tracks = [world_map.targets[tid] for tid in action.target_ids ...]` → 全部 3 个一起 `start_trip`
  - `transport/transport_pipeline.py:333-341`：套取成功后 `for t in self._current_targets: self._load_mgr.load(t.info, t.id)`
  - `transport/transport_pipeline.py:404`：`positions = [(rx, ry)] * len(self._current_targets)`（3 个目标用同一个坐标判投放，必然"全部有效"）
  - `decision/decision_engine.py:386-390`：全部 `mark_in_safe_zone` + 加分
- **影响（丢什么分/卡在哪一步）**：软件自报"运了 3 个"，场上实际只搬 1 个 → **自评分数虚高 3 倍，实际得分只有 1/3**；而且世界地图把没搬的目标标记为已在安全区 → 它们永远不会再被选 → **永久丢分（初赛最多丢 14 个目标）**。卡点：第二趟之后每一趟。
- **建议修法**（在"一趟只进 1 个"的现行机构下）：
  1. `TransportPipeline.start_trip()` 增加硬约束：`if len(targets) > 1: return (False, ...)`（或按 `sleeve` 实际槽位数的属性判断）；
  2. `TargetSelector.select_targets_for_trip(max_count=...)` 由上层传入**实际槽位数**（默认 1）；`LoadManager.MAX_LOAD=3` 保留作为规则上限，但装载入口按机构能力收口；
  3. 若后续真要做到"一趟 3 个"，必须补**逐个目标导航 + 逐个套取 + 目标落点轨迹记录**，否则维持 1 个。

#### B8. `_check_invalid_transport` 会被场地中心的目标常态触发，持续打断目标锁定
- **问题**：判据是"场上**任意**活跃目标距场地中心 <500mm"。目标被搬到场地中央本来就是比赛的常态（规则里无效目标就放中央），因此该函数几乎每帧返回 True。
- **证据**：`decision/decision_engine.py:314-326`
  ```python
  for tid, t in self._world_map.targets.items():
      if t.status == TargetStatus.ACTIVE:
          dist_to_center = math.sqrt((t.position[0]-1500)**2 + (t.position[1]-1500)**2)
          if dist_to_center < 500:
              logger.info("检测到目标在场地中央(无效转运恢复): ID=%d", tid)
              return True
  ```
  调用点 `:332-336`：每帧 `_current_target = None` → 重新选目标。
- **影响（丢什么分/卡在哪一步）**：正在执行的一趟在 `update()` 后目标被反复重选 → 导航目标频繁跳变（`AutonomousState._set_nav_target` 每帧可能改目标），**车在场上来回摇摆、到不了目标也运不进去**；同时日志被"检测到目标在场地中央"淹没。卡点：FREE_RUN 全阶段。
- **建议修法**：改为**按目标身份**判定：记录本趟提交的目标 id，只有当"该目标 id 上次已知位置在安全区，现在又出现在场地中"时才认为被裁判取回；或直接用 `LoadManager` 的差集（`_total_delivered` 与实际入区数不一致）触发。至少要加"仅在 `self._current_target is None` 时检查"的前置条件，避免打断进行中的趟次。

#### B9. 上位机侧"失控保护/碰撞保护/容错"模块是死代码，且看门狗刻意不停车
- **问题**：`opponent_strategy`、`stability`、`fault_tolerance`、`logging_system`、`imu_fusion`、`field_detector`、`hot_reloader`、`model_switcher`、`debug_dashboard`、`hardware_profile`、`deploy` 共 11 个模块**全仓 0 引用**。上位机看门狗明确选择"永不停止"。
- **证据**：
  - 引用计数（`grep -rl` 排除自身/`__init__`）：上述模块均 `=0`
  - `states/autonomous_state.py:50-53, 343-362`
    ```python
    WATCHDOG_HARD_LIMIT_S = 15.0  # 最后防线：仍不淘汰，仅持续保命运动
    ...
    if idle_duration > self.WATCHDOG_SURVIVAL_S and not self._survival_triggered:
        self._navigation.survival_circle()
    ```
  - `decision/decision_engine.py:422-440`：`EMERGENCY_STOP` 被降级为"保命绕圈"
- **影响（丢什么分/卡在哪一步）**：功能要求明确写"应具备…碰撞保护、失控保护"。当前上位机没有任何异常时接管/停车路径；真机一进异常（摄像头丢帧、串口丢帧、定位发散）只会"低速绕圈"直到时间耗尽 → **该轮 0 分**，且"是否满足功能要求"这一项在资格审查时也可能被质疑。
- **建议修法**：
  1. 至少把 `decision/anomaly_handler.py` 的 `is_fatal` 接到一个**分级策略**：连续无速度反馈/连续定位跳变 >N 次 → `chassis.send_stop()`（而不是继续绕圈）；
  2. 把 `fault_tolerance`/`stability` 里已有的传感器降级链（摄像头失效 → 纯里程计占位；IMU 失效 → 停转）接进 `AutonomousState`；
  3. `opponent_strategy` 必须在 `DecisionEngine` 里实例化并接收 `contact_duration_s`（同时修 C8 的传参）。

### 🟡 medium

#### B10. 比赛时长/启动窗口硬编码，且无"离开出发区"判定
- **证据**：`decision_engine.py:98` `MATCH_DURATION_S = 180.0`；README 也写"运行过程（3 分钟）"，但**本次要求原文没有规定分钟数**；全仓无"启动时间"判定（`grep 启动时间` 仅命中 field_elements 无关处）。
- **影响**：现场不是 3 分钟时，会提前停手（丢分）或超时仍在动（可能被判损坏器材）。
- **修法**：`MATCH_DURATION_S` 移到 `config/robot.default.yaml` + `innovation/config_loader`（并在 `apply_robot_config` 里生效）；赛后打印自评统计；给"起步 10s 内位移 <200mm"打一条 ERROR 日志便于现场定位（判定本身归裁判）。

#### B11. 自评分数与官方"评分规则"无对应关系，却被当成绩展示
- **证据**：`perception/target_types.py:258-266`（5/10/15 硬编码）、`load_manager.py:264-276`（`total_score`）、`decision_engine.py:389`（`_score += get_point_value(...)`）；本次附件**未包含评分规则表**（第 19 页仅引用"评分规则"）。
- **影响**：不会直接丢分，但**现场会用错误的自评分数做取舍决策**（例如"时间不够时先运谁"），可能导致策略选错。
- **修法**：把分值抽到 YAML（`targets.points.*`），默认值标注"假设值，待评分规则"；日志里显式打印 `分值来源=假设`。

#### B12. 减速带越障模式未接线
- **证据**：`navigation/navigation_pipeline.py:141, 167` 需要 `near_speed_bump=True` 才进入越障；`states/autonomous_state.py:286` 调用 `self._navigation.update((x, y, theta), dt=dt)` **未传该参数**（默认 False）；`grep "near_speed_bump"` 仅命中定义与自测。
- **影响**：出发区前 3 根 10mm 高减速带 + 上坡段全速直冲，易丢定位/卡住 → **起步阶段卡住 = 0 分**。
- **修法**：`AutonomousState._run_once` 里用 `FieldLayout` 的 `SPEED_BUMP` 元素（`field_elements.py:236-265`）判"距减速带 <250mm"并把 `near_speed_bump=True` 传下去；`motion_control.BUMP_CROSS_TIME_S` 与实际车速匹配。

#### B13. 决赛"颜色现场公布"无运行时配置；初赛颜色也写死
- **证据**：`target_types.py:199-244` 决赛表颜色固定为 绿/黑/橘/浅蓝；`classification.py:49-72` 用 `(color, shape)` 查表，缺颜色即 `None`；无 YAML/环境变量可改目标颜色（`config/*.yaml` 里无 targets 段）。
- **影响**：决赛颜色一变，**检测→分类全链路返回 None → 世界地图无目标 → `active_targets` 为空 → 决策直接进 DONE（`decision_engine.py:231-234`），0 分**。
- **修法**：`config/robot.default.yaml` 增加 `targets:` 段（初始化器/颜色/形状 → 类型 + 分值），`RobotConfig` 生成 `PRELIMINARY_TARGETS/FINAL_TARGETS`；并保留"只有形状时的保守降级"（按形状优先级：伤员长方体 > 核心圆锥台 > 普通圆柱/球，颜色仅用于区分危险目标）。

#### B14. 图 7 与文字"减速带 3 根"存在差异（按图建模，正确但需备案）
- **证据**：代码每出发区 6 块（挂 2 条边各 3 根，`field_elements.py:236-265`）；实测图 7 每区同样是两组各 3 块（垂直组 x 543~819mm、水平组 y 388~681mm，与出发区内边平齐，块间净距 ≈50mm，块 300×60mm）；文字仅写"出发区前有长方形减速带 3 根，间隔 50mm"。
- **影响**：若裁判按"每区 3 根"理解，只是场地差异；**代码按图建模是对的**，不需要改，但需要现场确认减速带实际布置，决定"起步往哪条边走"。
- **修法**：无需改代码；在 `docs/` 记录两种布置，现场开赛前目视确认出发区前是"一条边 3 根"还是"两条边各 3 根"。

#### B15. 场地元素/场地检测模块未接线
- **证据**：`perception/field_detector.py` 全仓 0 引用（尽管定义了 `PURPLE_FENCE`/`SPEED_BUMP` 等识别项）；`SafeZoneColor` 只能靠 `TEAM_COLOR` 环境变量人工设置，没有"用摄像头识别自己是红方还是蓝方"的能力。
- **影响**：现场抽签结果必须人工敲环境变量；写错 = 全部运到对方区（严重违规）。
- **修法**：把 `field_detector` 接进 `PerceptionPipeline`，用"本车所在安全区颜色"自检 `TEAM_COLOR` 是否与实际一致，不一致时拒绝一键启动（在 `DebugState` 里拦截）。

#### B24. 「现场可改参数」链路未接通：YAML 字段写了但没有任何代码读取（现场改不生效，必须改代码）

- **问题**：赛项要求决赛创新实践环节能在现场**编程/装配/调试**，因此关键参数必须"改配置即生效、不重编译"。仓库里 `config/robot.default.yaml` + `config/field.default.yaml` 已经写了几乎所有该有的字段，但 `apply_robot_config()` **只接通了 `timing` 与 `thresholds` 两组**，其余字段**全项目 0 消费点**；真正驱动行为的是散落在各模块里的硬编码常量。更严重的是 `config/field.default.yaml` **整个文件 0 消费点**（`grep -rn "FieldConfig" src/rescue_robot` 除 `config_loader.py`/`__init__.py` 外无命中），场地几何实际来自 `field_elements.py` 的硬编码 `StandardFieldLayout`。
- **证据**：
  - `src/rescue_robot/config.py:126-148` —— `apply_robot_config` 的函数体只到 `Thresholds.CAMERA_MIN_FPS` 为止，`motors`/`strategy_weights`/`match`/`fallback`/`perception`/`communication`/`logging` 段落**一行都没引用**
  - `src/rescue_robot/main.py:102-109` —— 只 `RobotConfig.from_yaml(...)` + `apply_robot_config(...)`；**未调用** `ConfigLoader.merge_with_env(prefix="RESCUE_")`（该函数存在于 `innovation/config_loader.py:181-193`）→ 环境变量覆盖链路也不通
  - 硬编码替代物（真正生效的值）：
    ```python
    # decision/decision_engine.py:98-104
    MATCH_DURATION_S = 180.0 ; TIME_PRESSURE_S = 30.0
    NAV_TIMEOUT_S = 10.0 ; GRIP_TIMEOUT_S = 3.0 ; TRANSPORT_TIMEOUT_S = 15.0
    # decision/target_selector.py:59
    TIME_PRESSURE_S = 30.0        # ← 与上处重复，改一处不生效
    # transport/load_manager.py:283
    penalty = 10 * len(released)  # ← 投放扣分硬编码，PDF 未给依据
    # states/autonomous_state.py:51-53
    WATCHDOG_EXPLORE_S = 10.0 ; WATCHDOG_SURVIVAL_S = 13.0 ; WATCHDOG_HARD_LIMIT_S = 15.0
    # transport/transport_pipeline.py:106
    self._push_dist_mm = 100.0    # 推入斜坡距离，注释写"真机标定"却不可配置
    ```
  - 逐字段清单见本报告 **§六「写了但没用」配置清单**
- **影响（丢什么分/卡在哪一步）**：① 创新实践环节现场**改这几个值不生效**，必须改源码（甚至重编译/重启），既耽误调试时间也可能被扣该环节分；② 更致命的是**它把一个已知错误锁死了**——`config/field.default.yaml` 里 `safe_zones.red.y: 2670`（应为 2700）和 `speed_bumps.width_mm: 30 / height_mm: 5`（图 9 实测 60×10）**都是错的**，现场想临时纠正也没有入口；③ 用户已确认"运行时长""投放扣分细则"PDF 未给数值，却正好落在未接线的字段上 → 现场公布后**无法快速适配**（见 §七）。
- **建议修法**（见 §6.3 详版）：
  1. `config.apply_robot_config()` 增补写回：`match.duration_s/time_pressure_s/*_timeout_s` → 模块级可变常量（`decision_engine` 改为读 config，去掉类属性硬编码）；`fallback.*` → `AutonomousState.WATCHDOG_*`；`strategy_weights.*` → `TargetSelector` 的评分权重参数；
  2. `perception.target_color_map` → 由它生成 `PRELIMINARY_TARGETS/FINAL_TARGETS`（默认值不变），一举解决 B13；
  3. `transport_pipeline._push_dist_mm`/`_place_steps`、`load_manager` 扣分、目标分值 → 分别做成可配置参数；
  4. `FieldLayout.standard()` 改走 `FieldConfig.from_yaml("config/field.default.yaml")`，**并先按 B2 修正 YAML 内的 safe_zones.y 与 speed_bumps 尺寸**（否则接线即放大错误）；
  5. `main.py:105` 改用 `ConfigLoader.merge_with_env(prefix="RESCUE_")`，支持环境变量现场覆盖；
  6. 验收：改 YAML 后打印 `DecisionEngine.time_remaining_s` 初值、目标颜色表、安全区 y，与 YAML 一致即通过（**不修改任何 .py**）。

### 🟢 low

| 编号 | 问题 | 证据 | 影响 | 建议修法 |
|---|---|---|---|---|
| B16 | `_get_survival_target` 形状/类型错误：被赋给 `pos` 后当坐标用 | `decision_engine.py:485-491` 返回 `(tx, ty)` 二元组，`:428-432` / `:436-440` `pos = self._get_survival_target(...)` → `Action(target_position=pos)`，而 `AutonomousState._set_nav_target` 取 `pos[0], pos[1]`；参数名 `rx, ry` 实为 **1500, 1500**，函数体把它当"当前位置" | 该分支一旦命中，保命绕圈目标点恒定 ≈(1500,1500)，车会直奔场地中心（撞/抢对方目标） | 改签名为 `_get_survival_target(center_x, center_y)`，内部用 `self._pose` 取当前位置；补 return type `Tuple[float,float]` |
| B17 | `_get_explore_target` 的 y 上限 clamp 到 2200 | `decision_engine.py:481-482` `cy = max(200, min(2800, cy))` 之后又 `cy = max(200, min(2200, cy))` | 探索永远搜不到 y>2200（即 1/2 号出发区一侧、上安全区附近）的目标 → 探索阶段效率低 | 统一为 `min(2800, ...)`；并排除安全区内的点 |
| B18 | 仿真/文档中的出发区编号映射与 `field_elements` 相反 | `simulation/sim_2d.py:47` `1:(150,150)` vs `field_elements.py:163-167` 1=左上(150,2850) | 仿真里验证过的策略，在真机上对应的是**另一个区**，验证结论不可直接迁移 | 统一编号：以 `field_elements.STANDARD` 为准，修 `sim_2d.START_CENTERS` 与 `integrated_sim.START_CENTERS` |
| B19 | 决策引擎 5 个 `*_TIMEOUT_S` 常量定义了但从未使用 | `decision_engine.py:102-104`（`NAV_TIMEOUT_S/GRIP_TIMEOUT_S/TRANSPORT_TIMEOUT_S`） | 单步超时无人管，一个动作卡住只能靠 10s 通用看门狗 | 在 `update()` 里按 `_action_phase` 计时并在超时后切目标/重规划 |
| B20 | `_check_fallback_needed` 从未被调用；`_fallback_retries` 未使用 | `decision_engine.py:458-475`、`:132`（`grep` 无调用点） | 13s 保命逻辑实际由 `AutonomousState` 的看门狗接管，决策引擎内的降级链条形同虚设 | 删除或接入 `update()` |
| B21 | `LoadManager` 扣分逻辑自相矛盾 | `load_manager.py:256-262` 先 `-10*len`，`:264-269` 又加回全部目标分值 | 投错区域净收益仍为正，惩罚不生效 | 投错区域的目标不计分（`p=0`）再扣分，或按规则"目标放回场地中心"直接清零该趟 |
| B22 | 世界地图缺少"伤员区/物资区"归属信息 | `world_map.mark_in_safe_zone()`（`world_map.py:418-422`）只标记状态，不记录落在哪个子区 | 无法做"伤员必须进伤员区"的事后校验与重试 | 增加 `safe_zone_subarea` 字段并持久化 |
| B23 | 模拟器不校验转运规则 | `simulation/sim_2d.py:369-375` 只判"车在安全区内且载物"即视为投放 | 用仿真跑出的成绩不能作为合规性证据 | 在仿真里引入 `LoadManager` 做同一套规则校验 |

---

## 三、需人确认（仅列，不占篇幅）

1. **尺寸/重量/颜色**：整车 ≤300×300×200mm、≤1.5kg；`Camera.HEIGHT_MM=210` 说明相机光心 ≥210mm，**整车高度是否 ≤200mm 需实测**（含软质线路）。
2. **一键启动**：下位机 `EVENT,START_BUTTON` → `hardware/button.py` 触发链路是否真通；是否需要"长按 500ms"（`config.py:45`）才启动。
3. **电源**：单一随车电源、全程不更换（软件无关）。
4. **机械**：只推/套不抓取、抗碰撞强度、非金属高速旋转件、无干扰设备/激光。
5. **摄像头成像**：`TILT_DEG=30`、`SLEEVE_ROI` 需现场标定（`config.py:95-107`），否则测距与套取确认都不可信。
6. **串口电气**：RDK UART/波特率/接线（`chassis_serial_protocol.md`）。

---

## 四、当前软件的达成度判断

### 结论：**现在这套软件装上硬件后，拿不到有效分（大概率 0 分），且失败原因不是硬件，而是软件。**

理由链（按发生顺序）：

1. **起步就可能全错**（B1）：出发区写死 3 号区，抽到别的区 → 坐标系整体错位，第一步就偏。
2. **就算坐标系对了，第一趟也守不住规则**（B2/B3/B4）：安全区位置偏 300mm + 投送判定用车身坐标 → 首趟"1 个普通物资送进物资区围栏内侧"极可能不成立；不成立按规则**后续全部无效 = 0 分**，而软件自己会显示"得分"。
3. **就算前三步侥幸成立，也搬不多**（B7/B8）：一趟选 3 个只去 1 个（自评虚高）、`_check_invalid_transport` 常态打断目标锁定 → 实际入区数量远低于软件自报。
4. **识别环节本身不可靠**（B5/B6/B6b）：伤员（最高分）分类错、蓝/浅蓝混淆、决赛颜色无法适配 → 目标要么漏、要么错类。

因此：**不能算"达到比赛目的"**。软件目前是"能跑完全流程的骨架 + 若干关键规则未闭环"。

### 最大风险点（Top 3）

| 排名 | 风险 | 位置 | 为什么最危险 |
|---|---|---|---|
| 1 | **安全区位置/围栏形状 + 只校验装载不校验"围栏内侧送达"** | `field_elements.py:184,211,186-193`；`decision_engine.py:300-310`；`load_manager.py:155-162` | 它同时决定"首趟是否有效"。**一条错 → 整轮 0 分**，且软件不会告警（假成功）。 |
| 2 | **出发区坐标系未按抽签初始化** | `serial_chassis.py:28-34`；`autonomous_state.py:130`；`main.py:128` | 25% 概率抽到非 3 号区就是全场错位；**不依赖任何硬件精度，纯初始化缺失**。 |
| 3 | **多目标趟次假装载 + 中心目标误判打断** | `decision_engine.py:337-371, 314-326`；`transport_pipeline.py:333-341, 404` | 让"能拿到的分"被系统性丢弃或虚报，赛后无法用日志复盘真实入区数。 |

### 建议的修复顺序（给修复者）

1. B1 → B2 → B4 → B3（**顺序不能换**：坐标系 → 场地几何 → 落点判定 → 首趟规则闭环）
2. B7 → B8（真实入区数量）
3. B5 → B6 → B13（识别链）
4. B9 → C8/B12（异常与对抗兜底）
5. 其余 medium/low

---

## 五、软件负责人视角：赛项要求对软件的硬约束

> 本节把要求里**直接约束软件**的条款单独抽出，逐条给「要求原文关键句 → 代码现状（文件:行）→ 判定 → 后果」。

### 5.1 决赛创新实践环节：现场编程 / 改参数 / 调试 → 必须"不重编译就能改"

- **要求原文关键句**：「在规定时间内，按照决赛现场发布的决赛命题将自带的零部件更换在参赛作品上，并完成该环节的**编程、装配、调试**等任务」；初赛任务命题文档要求「策划决赛场景和规划决赛场地…保证在创新实践环节中必须进行救援机器人相关零部件的设计及制造」。
- **代码现状**：配置链路**存在但只接通了 1/4**。
  - `main.py:102-109`：`RobotConfig.from_yaml("config/robot.default.yaml")` → `config.apply_robot_config(robot_cfg)`
  - `config.py:126-148`：`apply_robot_config` **只写回 `Timing.*`（8 项）与 `Thresholds.*`（5 项）**，其余段（`motors`/`strategy_weights`/`match`/`fallback`/`perception`/`communication`/`logging`）**一个都没用**
  - `innovation/config_loader.py:181-193` 提供了 `merge_with_env(prefix="RESCUE_")`，**`main.py` 从不调用** → 环境变量覆盖链路实际不通
- **判定**：**不符合**（对应新增不符合项 **B24**）
- **后果**：创新实践环节现场改这几个值**不会生效**，必须改源码 → 该环节扣分 + 白白耗掉调试时间（调试总时长是硬限制）。

### 5.2 一键启动 + "规定启动时间内必须离开出发区"

- **要求原文关键句**：「调试时间结束，现场裁判发出统一开始指令，参赛队**一键启动**救援机器人，计时开始，各参赛队救援机器人在**规定启动时间内必须离开出发区**，否则本轮比赛结束」。
- **代码现状（延迟链，全部可查）**：
  | 环节 | 位置 | 时长 |
  |---|---|---|
  | 长按判定 | `config.py:45` `BUTTON_LONG_PRESS_MS=500` | 500ms（**需人确认**裁判是否接受长按） |
  | 自检（BOOT→DEBUG） | `states/boot_state.py:44-56` → `system_check.py:225-238` | 每传感器 `SENSOR_CHECK_TIMEOUT_MS=3000`、每电机 `MOTOR_CHECK_DURATION_MS=500`、整机 `SELF_CHECK_TIMEOUT_S=10`（`config.py:48-51`） |
  | 摄像头首帧预热（建管线时，阻塞） | `main.py:150-164` `CAM_WARMUP_S=3.0` | 最多 3s |
  | 按下一键启动后的固定延迟 | `states/autonomous_state.py:162-164` `time.sleep(POST_START_DELAY_MS=1000)` | 固定 1000ms |
  | 起步 | 主循环 50Hz + 导航首帧 | ~数十 ms |
- **判定**：**部分不符合**——延迟链**可控但未被约束**：单传感器超时 3s × 多路传感器串行 + 电机 0.5s × 2 + 摄像头 3s，**极端情况自检可吃掉十几秒**，再加上固定 1s 延迟，与"规定启动时间内离开出发区"存在冲突风险；且 `POST_START_DELAY_MS` 原本是"等裁判离开"用，**没有与出发区离开判定联动**。
- **后果**：若现场"启动时间"较短，可能**还没动就被判本轮结束（0 分）**。
- **建议修法**：① 把自检做成**并行/可裁剪**（`SystemChecker.run` 里非关键项设为非阻塞，`system_check.py:225-238`）；② `POST_START_DELAY_MS` 从 1000 降到 ≤300 并与"是否已离开出发区"解耦（`autonomous_state.py:162-164`）；③ 起步优先走"低速直行 300~500mm 离区"再进入正常导航（新增 `AutonomousState` 起步动作，用 `field_elements` 的 `SPEED_BUMP`/出发区区域判离区完成）。

### 5.3 全自主、不可遥控

- **要求原文关键句**：「救援机器人必须采用**自主运行模式**…允许与笔记本电脑进行通讯，**运行过程中不能触碰笔记本电脑**，**不能用其他任何方式对救援机器人进行遥控**」。
- **代码现状**：
  - `states/autonomous_state.py:137-147`：进入即 `_lock_external_inputs()`；`:364-375` 只记日志
  - `state_machine.py:184-203`：`one_key_start()` 置 `_external_inputs_locked=True`，且 `AUTONOMOUS → DEBUG` 被显式禁止（`:129-131`）
  - `communication/comm_manager.py:140-144`：`is_locked` 时拦截所有入站指令并计数（自测 `:273-281` 验证拦截生效）
- **判定**：**符合（软件层）**。唯一未逐行核验点：`communication/comm_server.py`（WebSocket 服务端）在 AUTONOMOUS 期间是否仍接受**新连接**并存在绕过 `CommManager` 门禁的路径 → 标「**需复核**」，不影响结论（运动指令最终都经 `CommManager`/`chassis`）。
- **后果**：若 `comm_server` 有旁路，会被判"遥控"→ 取消成绩；建议复核并把 `comm_server` 在锁定后改为**只发不收**。

### 5.4 碰撞保护 / 失控保护

- **要求原文关键句**：「应具备高速移动、避障、越障、救援目标的搜索与转运、对象的识别和信息获取（二维码、条码、文字、图像、形状、颜色、温度、振动等），**并具有碰撞保护、失控保护等功能**」。
- **代码现状**：
  - `decision/anomaly_handler.py:34` 定义 `COLLISION_STUCK`（接触 >10s），`decision/opponent_strategy.py:284-299` 实现 7s 预警 / 9s 强制脱离 —— **两者都未被 `DecisionEngine` 实例化或调用**（`opponent_strategy` 全仓 0 引用；`grep` 命中的 `robustness/*`、`innovation/hot_reloader` 也只在 `__init__.py` 的名字导出与 `deploy.py` 的模块字符串里，**无运行时调用点**）
  - `decision/decision_engine.py:184, 208-211`：`contact_duration_s: float = 0.0` 是**入参默认值**，`states/autonomous_state.py:260-265` 调用时**不传** → 接触时长恒为 0，`anomaly_handler.check(..., contact_duration_s)` 永远看不到接触
  - 上位机看门狗**刻意不停车**：`states/autonomous_state.py:50-53, 343-362`（15s 仍"保命绕圈"）；`decision_engine.py:422-440` 把 `EMERGENCY_STOP` **降级**为绕圈
- **判定**：**不符合**（上位机侧未实现）。现有保护只有下位机速度看门狗（`chassis_serial_protocol.md`：300ms 保持 / 800ms 停）。
- **后果**：① 发生接触时软件不知道在接触 → 不会主动脱离，**白丢时间**；② 被裁判强制分离放回出发区后**没有任何重置位姿/继续运行的接口调用**（`handle_forced_separation` 无人调用）→ 定位仍是旧位姿，**放回后动作全错，该轮基本报废**；③ 功能要求项在资格审查时可能被质疑。
- **建议修法**：`DecisionEngine.__init__` 里实例化 `OpponentStrategy`；`AutonomousState._run_once` 把 `contact_duration_s`（来自 `OpponentTracker`/下位机）传进 `decision.update()`；把 `handle_forced_separation(new_pose)` 接到"触发分离"事件并同时调用 `chassis.set_start_pose()` + `localizer.set_pose()`。

### 5.5 识别能力（含初赛/决赛形状参数）

- **要求原文关键句**：「对象的识别和信息获取（**二维码、条码、文字、图像、形状、颜色、温度、振动**等）」。
- **代码现状**：
  - **已实现**：颜色（`perception/detection.py:34-51` HSV 阈值表）+ 形状（`:54-73` 顶点数/面积比 + `:270-301` `_classify_shape`）
  - **未实现**：二维码、条码、文字、图像、温度、振动（`system_check.py:101` 只有"温度传感器存在性检查"接口，检测链未使用；全仓无二维码/条码/文字识别代码）
  - **初赛/决赛形状参数已配置**：`detection.py:54-61` 覆盖 正方体(4-8)/三棱锥(3-5)/长方体(4-8)/圆柱(8-20)/圆锥台(8-20)/球(8-30)；面积比 `:64-73`
- **判定**：**部分符合**。缺失项对本赛项**不直接丢分**（救援赛项不靠二维码下发任务，温度/振动只在原要求里作为"识别能力"举例），但已实现的两项**有硬伤**：
  - `CUBE (4,8)` 与 `CUBOID (4,8)` 顶点区间完全重叠，`SHAPE_AREA_RATIOS` 里 CUBE `(0.5,1.0)` 覆盖 CUBOID 的默认 `(0.2,1.0)`，`_classify_shape`（`:285-301`）按 dict 顺序取第一个匹配 → **长方体（伤员，15 分）必被判成正方体**；
  - `detection.py:37-40` 的 `BLUE ((95,80,60),(125,255,255))` 与 `LIGHT_BLUE ((85,50,110),(108,255,255))` 在 H 95~108、S 80~255 完全重叠，叠加 `classification.py:105-128` 的"同色兜底" → **蓝色目标可能被判成危险目标（浅蓝）并被永久排除**。
- **后果**：伤员（最高分）漏检/错类，**每漏一个丢 15 分**；蓝色目标被当危险目标 → 永不转运。
- **建议修法**：见 B5/B6（用长宽比区分 CUBE/CUBOID；删除同色兜底、收窄 LIGHT_BLUE 阈值）。

### 5.6 不得损坏场地设施

- **要求原文关键句**：「比赛过程中（含调试），救援机器人**不得损坏场地等赛场设施**，为了避免损坏比赛相关设施，裁判员有权终止比赛。**若出现场地等被破坏，取消比赛资格**」。
- **代码现状**：
  - **有保护**：硬禁区（对方安全区 + 场边 100mm 边距）写入代价地图并实时校验 —— `navigation/forbidden_zones.py:60-106`、`navigation/navigation_pipeline.py:80, 96, 264`
  - **无保护**：放置动作是"底盘向斜坡方向前推 `_push_dist_mm=100.0` + 舵机 4 步渐进 0→70°"（`transport/transport_pipeline.py:373-401`），**没有任何力/电流/堵转/位移上限或超时保护**；`Thresholds.MOTOR_MAX_CURRENT_MA`（`config.py:63`）只在自检阶段使用，运行中不做堵转判定
  - 场地内障碍物（减速带）也未见"撞到就停"的判定（`near_speed_bump` 未接线，见 B12）
- **判定**：**部分不符合**。路径规划层不会主动撞设施，但**一旦顶到紫色斜坡/围栏/减速带就是持续堵转推挤**，没有任何软件级熔断。
- **后果**：**最严重等级——一旦被判定"破坏场地"直接取消比赛资格**（不只是丢分）。
- **建议修法**：① 给 `TransportPipeline.PLACING` 加**超时 + 位移增量双重熔断**（推入 ≤2s 或位移 <5mm 即中止并抬爪）；② 把 `MOTOR_MAX_CURRENT_MA`/下位机电流遥测接进运行期堵转检测，超阈值立即 `chassis.send_stop()`；③ 把 `near_speed_bump`（`navigation_pipeline.py:141,167`）接上并按 `motion_control.BUMP_CROSS_TIME_S` 限速通过。

---

## 六、「写了但没用」配置清单（YAML 字段存在 → 行为不生效）

> 来源：`config/robot.default.yaml`、`config/field.default.yaml`；判定方法＝字段是否被 `config.apply_robot_config`（`config.py:126-148`）写回，或在 `config_loader.py` 之外的运行代码里被消费。
> **结论：现场改这些字段完全不会改变机器人行为。**

### 6.1 `config/robot.default.yaml`

| YAML 字段 | 代码里有谁读 | 真正生效的是什么 | 判定 |
|---|---|---|---|
| `robot.match.duration_s: 180` | 无人读（`grep` 全项目 0 消费点） | `decision/decision_engine.py:98 MATCH_DURATION_S = 180.0`（硬编码） | **写了但没用** |
| `robot.match.time_pressure_s: 30` | 无人读 | `decision_engine.py:99 TIME_PRESSURE_S = 30.0`、`target_selector.py:59 TIME_PRESSURE_S = 30.0`（两处硬编码，需同步改） | **写了但没用** |
| `robot.match.nav_timeout_s: 10` | 无人读 | `decision_engine.py:102 NAV_TIMEOUT_S = 10.0`（且**该常量本身也未使用**，见 B19） | **写了但没用** |
| `robot.match.grip_timeout_s: 3` | 无人读 | `decision_engine.py:103 GRIP_TIMEOUT_S = 3.0`（同样未使用） | **写了但没用** |
| `robot.match.transport_timeout_s: 15` | 无人读 | `decision_engine.py:104 TRANSPORT_TIMEOUT_S = 15.0`（同样未使用） | **写了但没用** |
| `robot.fallback.max_retries: 3` | 无人读 | `decision_engine.py:133 self._max_retries = 3`（且未使用） | **写了但没用** |
| `robot.fallback.watchdog_warn_s: 10` | 无人读 | `states/autonomous_state.py:51 WATCHDOG_EXPLORE_S = 10.0` | **写了但没用** |
| `robot.fallback.watchdog_critical_s: 13` | 无人读 | `autonomous_state.py:52 WATCHDOG_SURVIVAL_S = 13.0` | **写了但没用** |
| `robot.fallback.watchdog_timeout_s: 15` | 无人读 | `autonomous_state.py:53 WATCHDOG_HARD_LIMIT_S = 15.0` | **写了但没用** |
| `robot.fallback.stuck_time_s: 5` / `stuck_distance_mm: 30` | 无人读 | 无对应实现（卡死判定缺失） | **写了但没用** |
| `robot.strategy_weights.distance_weight / points_weight / time_weight` | 无人读 | `target_selector.py:88-102` 用硬编码公式（`MAX_FIELD_DISTANCE_MM`、`points ** 1.3`、`urgency*2.0`） | **写了但没用** |
| `robot.strategy_weights.opponent_factor: 0.5` | 无人读 | `target_selector.py:98 opponent_factor = 0.5 if ... else 1.0`（硬编码） | **写了但没用** |
| `robot.motors.max_speed_mm_s: 850` / `max_angular_speed_rad_s` / `wheel_base_mm` / `pid.*` / `pid_angle.*` | 无人读 | 速度上限在 `motion_control.py`（`MAX_LINEAR_SPEED` 等常量）；PID 在下位机固件 | **写了但没用** |
| `perception.target_color_map.{regular,core,injured,dangerous}` | 无人读 | `perception/target_types.py:147-192, 199-244` 把颜色写死在目标表里 | **写了但没用**（直接导致 B13：决赛现场公布颜色时无法适配） |
| `perception.detection.association_threshold_mm / stale_frames / remove_frames` | 无人读 | `perception/world_map.py` 里用自己的常量 | **写了但没用** |
| `communication.*`、`logging.*` | 无人读（`robustness/logging_system.py` 全仓 0 运行时调用） | — | **写了但没用** |
| `robot.timing.*`（8 项）、`robot.thresholds.*`（5 项） | ✅ `config.py:133-148` 写回 | 真正生效 | **已接线** |

### 6.2 `config/field.default.yaml` —— 整文件 0 消费点（更严重的同一类问题）

`grep -rn "FieldConfig" src/rescue_robot` 除 `config_loader.py` / `__init__.py` 外**无命中**；场地几何实际全部来自 `perception/field_elements.py` 的硬编码 `StandardFieldLayout`。后果是：

| YAML 字段 | YAML 写的是 | 真正生效的是 | 差异 |
|---|---|---|---|
| `field.safe_zones.red.y: 2670`（`blue.y: 30`） | 2670 / 30 | `field_elements.py:184, 211` 同样 2670 / 30（**两边一致地错**） | 图 7 实测应为 **2700 / 0**（贴顶/贴底边）→ 见 B2 |
| `field.safe_zones.*.supply_area` / `injured_area` | 300 宽子区（物资区 x1200~1500） | `field_elements.py:199-208` 实际切 290 + 隔板 20 + 290 | 与图 8 的"物资区≈300/伤员区≈280"不一致 |
| `field.speed_bumps.width_mm: 30` / `height_mm: 5` | 30×5 | `field_elements.py:113-114` `SPEED_BUMP_DEPTH=60`（图 9 实测 60，图 9 高度 10） | **YAML 数值本身是错的**，若接线会把减速带改成 30×5 |
| `field.start_zones.{red_left,blue_left...}` | 命名与 `field_elements` 的 1/2/3/4 编号无映射 | 代码侧 `start_positions`（`field_elements.py:163-167`） | 命名体系不统一，接线时必须先统一（见 B18） |
| `field.opponent.contact_warning_s: 7 / contact_force_s: 9 / contact_limit_s: 10` | 7/9/10 | `opponent_strategy.py:88` 附近常量，但**该模块 0 引用** | 规则值对了，功能没接（见 5.4 / C8） |
| `field.my_color: "red"` | red | `main.py:128-130` 读 **`TEAM_COLOR` 环境变量** | 两套入口不一致 |

### 6.3 建议修法（配合 fixer 的 P1.5）

1. **`config.py:126` `apply_robot_config`**：至少把 `match.*` 写回为模块级可变常量，并让 `DecisionEngine`/`TargetSelector` 从 `config` 读取（`MATCH_DURATION_S`/`TIME_PRESSURE_S`/`NAV_TIMEOUT_S`/`GRIP_TIMEOUT_S`/`TRANSPORT_TIMEOUT_S` → 改为 `@property` 读 config）；`fallback.*` 写回 `AutonomousState.WATCHDOG_*` 类属性。
2. **目标颜色**：`target_types.py` 的 `PRELIMINARY_TARGETS/FINAL_TARGETS` 改为由 `perception.target_color_map` 生成（默认值保持与现状一致），这样决赛现场公布颜色后**改 YAML 即可**（同时解决 B13）。
3. **场地几何**：`FieldLayout.standard()` 改为 `FieldLayout.from_config(FieldConfig.from_yaml("config/field.default.yaml"))`，并**先按 B2 修正 YAML 里的 safe_zones.y / speed_bumps 尺寸**（否则接线即放大错误）。
4. **环境变量**：`main.py` 改用 `ConfigLoader.merge_with_env(prefix="RESCUE_")`（`config_loader.py:181-193`），让现场可在**不改文件**的情况下覆盖（对创新实践环节有利）。
5. **验收方式**：改 YAML → 打印 `DecisionEngine.MATCH_DURATION_S` / `TargetSelector.TIME_PRESSURE_S` / 目标颜色表 / 安全区 y 与 `field.default.yaml` 一致，无需改任何 `.py`。

---

## 七、待现场公布项（软件侧须先做成可配置项）

> 以下两项**赛项 PDF 未给出具体数值**（用户已确认，可能现场公布）。软件侧**不能靠猜**，正确做法是：**默认值保持不变，但必须走配置**，现场公布后改 YAML 即可生效。

| 序号 | 待公布项 | PDF 出处与缺失情况 | 代码现状 | 软件侧应先做成 |
|---|---|---|---|---|
| **待定-1** | **规定运行时间（秒）** | 原文只有「规定运行时间到…比赛结束」「在规定运行时间内」（第 440 行附近），**未给出分钟/秒数**；`README.md` 自行假设"3 分钟" | 硬编码 `decision/decision_engine.py:98 MATCH_DURATION_S = 180.0`；YAML `robot.match.duration_s: 180` **写了不用**（见 §6.1）；`simulation/{sim_2d,integrated_sim}.py` 各自又硬编码 180 | ① `MatchConfig.duration_s` 真正驱动 `DecisionEngine`（含 `time_remaining_s`/TIME_PRESSURE 切换）；② 现场公布后只改 `config/robot.default.yaml`；③ `TIME_PRESSURE_S` 同理（`decision_engine.py:99` + `target_selector.py:59` **两处**都要接） |
| **待定-2** | **投放位置错误的扣分细则** | PDF 只在初赛段写「转运至安全区的无效救援目标将被取出重新随机放置在场地中央」（第 436-437 行），**未给扣分数值**；第 19 页仅引用"评分规则"（该表不在本附件内） | 硬编码 `transport/load_manager.py:283 penalty = 10 * len(released)`；且 `:264-269` 随后又把该趟全部分值加回（见 B21），**扣分实际不生效** | ① 扣分规则做成配置（`scoring.wrong_zone_penalty_per_target`，默认 10，标注"假设值"）；② 修掉"先扣后全额加回"的矛盾：投错区域的目标**按 0 分计**再扣分；③ 目标分值与 `target_types.py:258-266` 的 5/10/15 同样移入配置（见 B11） |

> 这两项在本报告的「逐条对照表」里对应 G3（评分规则无法判定）与 C10（运行时间），此处给出**可执行的软件侧动作**：**可配置 + 默认值不猜**。

---

*报告生成：req-auditor（requirements-auditor）｜依据：要求原文 + 代码静态核验 + 图 7/8/9 像素级标定*
