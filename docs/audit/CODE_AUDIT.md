# 全项目代码审计报告（CODE_AUDIT）

- 审计对象：`/home/ony_uang/Rescue-team`（智能救援机器人上位机，Python）
- 审计范围：`src/rescue_robot/**`（65 个 .py，18871 行）、`tools/**`、`scripts/**`、`config/**`
- 审计方式：逐模块精读主链路 + grep 定位可疑模式 + **纯软件逻辑复现**（下文标 `[复现]` 的结论均用可重跑脚本在本机跑过；脚本已入库：`docs/audit/repro/logic_repro.py`（M1~M7 用例，含说明 `docs/audit/repro/README.md`），**不依赖任何硬件**）
- 审计基线：`chassis_serial_protocol.md`（v1.1）、`config/robot.default.yaml`、`HARDWARE_DEPENDENCIES.md`、`MEMORY.md`
- **口径（本次任务约定）**：无 RDK、硬件全关 → 只做**纯软件**检查，聚焦 ①软件实现逻辑是否正确 ②**端到端能否真的完成比赛任务**。凡能靠读代码/逻辑仿真判定的**一律给确定结论**；"需真机确认"只保留确实必须真机的少数项（集中见第八节，仅 6 条）
- 下位机固件仓库 `Tang29Zx/rescue_f103c8` 本机不存在（`/tmp/rescue_f103c8` NOT PRESENT），协议一致性只能核对**上位机侧实现**，固件侧列入第八节
- 已知环境坑（不重复报告）：pytest 因 anyio 插件损坏不可用；本报告用 `PYTHONPATH=src python3 ...` 与 `python3 -m rescue_robot.<模块>` 验证
- **审计所依据的代码版本**：`git` HEAD = `f2e4795`（"docs(audit): 智能救援赛项要求合规审计"）。审计期间 `fixer` 已在**同一工作区**开始修改源码（`git status` 显示 `anomaly_handler.py`/`decision_engine.py`/`forbidden_zones.py`/`navigation_pipeline.py`/`autonomous_state.py`/`load_manager.py` 为已修改），因此：
  - 本报告逐条 `文件:行` 与代码片段**对应的是上述 HEAD 版本**；若某条已被 fixer 修掉，行号可能漂移，请以"证据片段 + 语义"为准复核。
  - 已观察到 fixer 的改动方向与本报告多处一致（例如 `AnomalyHandler.check()` 的 `velocity` 已改为 `Optional` 并标注"绝不凭猜测判无动作"）——**这是好事，但审计结论不因此改写**，请按"优先级表 + 每条的建议修法"核对是否真正闭环。
  - 复现脚本 `docs/audit/repro/logic_repro.py` 在**当前工作区**可跑通（exit=0）；其中 M1 依赖 `DecisionEngine`/`TransportPipeline` 的公开行为，若这两处被重构，用例断言需要同步更新。
- 本报告共 8 个审查维度 + 1 个工程可交付性专节（第十节）：阻塞/实时性、异常与恢复、并发、协议一致性、坐标系与单位、状态机、死代码与不一致、边界条件、配置来源，以及**可维护性/可调试性/可配置性/可部署性/性能预算/文档一致性**。

## 结论摘要

| 严重度 | 数量 | 说明 |
| --- | --- | --- |
| blocker | 6 | 直接决定"拿不到分 / 违规 / 不停车" |
| high | 12 | 关键能力静默失效（看着在跑，实际不产生分数） |
| medium | 13+ | 逻辑与工程缺陷（含第十节工程可交付性问题） |
| low | 9 | 死代码、注释不符、边界 |

### 端到端能否达成目的？—— 确定结论

**主链路能跑通，但拿不到应有的分。** 用纯软件仿真跑完整任务流（`/tmp/mission5.py`，无硬件）：

```
===== M1 完整任务流 =====
 得分=35 送达=4 趟数=3 状态=FREE_RUN
 选择序列=[REGULAR_SUPPLY, INJURED, CORE_SUPPLY]     # 选择顺序正确
 转运阶段=COMPLETE   VIOLATION=False
```

**能确定做对的部分**（均有复现证据）：
- 目标选择顺序正确：首趟只选 1 个普通物资 → 伤员单独成趟 → 核心/普通混合；
- **危险目标被有效排除**：`[复现]` 场上只剩危险目标时进入"探索"而不是去抓（`/tmp/mission6.py` T1）；
- **安全区内的目标能正确排除**：`[复现]` 投放后即使目标被超时删除又重新检测（新 id），仍判为不可选（T5）；无目标时正确进入 `DONE`（T2）；
- 计分链路（`LoadManager.release_all` → `_score`/`_targets_delivered`）数值计算正确。

**致命结论**：**"一趟装多个"在实现上不存在**。`TransportPipeline` 只要到达**第 1 个**目标，就把本趟计划里的**所有**目标一次性记入货舱（S-40 `[复现]`：计划 3 个、车只到第 1 个 → `装载=3`）。车没去过另外两个目标，软件却认为三个都在车上 → **每趟实际只能带 1 个，而计分按 3 个算**。一趟的得分能力只有设计值的 1/3，另外 2 个目标留在原地被下一轮重复选中、重复跑空趟，180 s 内送达总数远低于预期（**丢分**，见 S-40）。



---

# 一、blocker（阻断级）

## S-01 比赛结束/目标清空时机器人不停车，会继续按过期导航目标行驶
- **位置**：`src/rescue_robot/decision/decision_engine.py:226-234,244-245` + `src/rescue_robot/states/autonomous_state.py:279-280,286-292`
- **证据**：
  ```python
  # decision_engine.py:226
  if self.time_remaining_s <= 0:
      self._strategy_state = StrategyState.DONE
      logger.info("比赛时间到!")
      return Action(type=ActionType.WAIT, detail="比赛结束")     # 不产生任何停车动作
  ...
  elif self._strategy_state == StrategyState.DONE:
      return Action(type=ActionType.WAIT, detail="比赛完成")
  ```
  ```python
  # autonomous_state.py:279  WAIT 不在 _run_once 的任何分支里 -> 导航目标保持过期值
  elif action.type == ActionType.TRANSPORT_TO:
      self._set_nav_target(action.target_position)
  ...
  cmd = self._navigation.update((x, y, theta), dt=dt)          # 仍按旧 target 计算速度
  if self._chassis is not None:
      self._chassis.send_velocity(cmd.linear, cmd.angular)     # 继续下发 VEL
  ```
- **影响**：180 s 时间到（或场上目标被清空）后，`NavigationPipeline` 的 `_target` 不会被清除，`nav.update()` 依旧输出速度，底盘持续收到 `VEL`。机器人会在终场后继续冲向下一个目标/墙角，直到人工断电。协议第 8 节要求"正常结束发送 `STOP`"，**这条命令从未被发出**（见 S-08）。
- **严重度**：blocker
- **建议修法**：在 `DecisionEngine.update()` 的 `time_remaining_s <= 0` 与 `not self._world_map.active_targets` 两处返回 `Action(type=ActionType.EMERGENCY_STOP, detail=...)`；并在 `AutonomousState._run_once()` 顶部加终场兜底：`if self._decision.strategy_state in (StrategyState.DONE,) : self._navigation.clear_target(); self._stop_chassis(); return`。

## S-02 首次转运规则被绕过：`start_trip` 的"忙碌检查"写成了方法对象
- **位置**：`src/rescue_robot/transport/transport_pipeline.py:256`
- **证据**：
  ```python
  if not self.is_idle:              # ← 少了 ()，永远拿到 bound method（真值）
      logger.warning(f"无法开始转运：当前阶段={self._phase.name}")
      return (False, Violation.NONE)
  ```
  `[复现]`
  ```
  guard line: ['if not self.is_idle:']
  start_trip while CAPTURING -> (True, <Violation.NONE: 1>)   # 本应 (False, NONE)
  start_trip again          -> (True, <Violation.NONE: 1>)
  ```
- **影响**：**规则校验（首次 ≤1 个普通、≤3 个、危险目标、伤员单独）在实现上没有被"当前趟次"保护**。`TransportPipeline.update()` 的 `CAPTURING` 帧会调用 `start_trip()`（`autonomous_state.py:270-278`，`is_idle()` 在车**未停稳**时因 `_halt_for_capture` 造成 phase 变化而返回 True），于是 `_phase` 被反复重置为 `APPROACHING`、`_current_targets` 被替换，`LoadManager` 里已装载的货物与新的 `_current_targets` 错位 —— 会出现"抓着 1 个物体却报了 3 个"或"已装够 3 个又叠加一批"的清零/叠载故障。
- **严重度**：blocker
- **建议修法**：改为 `if not self.is_idle():`；并**双重保险**：在 `start_trip()` 里增加真实守卫 `if self._load_mgr.state.count > 0 or self._phase not in (TransportPhase.IDLE, TransportPhase.COMPLETE): return (False, Violation.NONE)`；调用侧 `autonomous_state.py:272` 保持 `is_idle()` 判断并把 `start_trip` 的返回值和 violation 打日志。

## S-03 比赛冲刺期（剩余 <30 s）目标选择可返回伤员，直接触发违规
- **位置**：`src/rescue_robot/decision/target_selector.py:198-214`
- **证据**：
  ```python
  def select_targets_for_trip(self, ..., include_injured: bool = False,
                              time_remaining_s: float = 180.0):
      # 时间紧迫时优先伤员
      if time_remaining_s < self.TIME_PRESSURE_S:        # 198
          injured = world_map.get_injured()
          if injured:
              return [min(injured, ...)]                 # 205-206 直接返回伤员
      ...
      if include_injured:                                # 209 先于"首次必须普通"的判断
          injured = world_map.get_injured()
          if injured:
              return [nearest]                           # 214
  ```
  调用方 `decision_engine.py:337-341` 传入 `include_injured=True`，且**没有任何 `is_first_trip` 前置判断**。
- **影响**：剩余时间 <30 s 或场上有伤员时，即便 `LoadManager.is_first_trip` 仍为 True，`_trip_targets` 也会是伤员；`start_trip()` → `can_load_batch()` 返回 `FIRST_TRIP_WRONG_TYPE` / `INJURED_MULTI` → `_phase = TransportPhase.VIOLATION`（`transport_pipeline.py:266`）。按 `VIOLATION_CONSEQUENCES`，**本轮结束且成绩无效**。
- **严重度**：blocker
- **建议修法**：在 `select_targets_for_trip()` 开头加 `if world_map ... is_first_trip`（或由 `decision_engine` 传入 `first_trip=True` 时直接走"只选 1 个普通物资"分支）；`DecisionEngine._handle_free_run()` 里若 `self._trip_targets` 含 `TargetType.INJURED` 则改为单元素列表。

## S-04 违规状态 `TransportPhase.VIOLATION` 无恢复路径，机器人永久卡死
- **位置**：`src/rescue_robot/transport/transport_pipeline.py:266`（置位）、`177-178/441-442`（`is_idle()` 不含 VIOLATION）、`src/rescue_robot/states/autonomous_state.py:272`（唯一守卫）
- **证据**：
  ```python
  # transport_pipeline.py
  self._phase = TransportPhase.VIOLATION          # 266
  ...
  def is_idle(self) -> bool:                       # 441
      return self._phase in (TransportPhase.IDLE, TransportPhase.COMPLETE)
  # autonomous_state.py
  elif action.type == ActionType.GRIP:
      if self._transport.is_idle():                # 272 永远 False
          ...
  ```
  `TransportPipeline.reset()`（`:455`）在 `src/` 中**没有任何调用方**。
- **影响**：一旦任何规则校验失败（S-02/S-03 都会触发），`_phase` 永久停在 VIOLATION：`is_idle()` 恒 False → 决策引擎每帧发 `GRIP` 都被忽略 → `loaded` 恒空 → `_trip_gripped` 恒 False → `transport.update()` 在非 IDLE 分支也不做事 → 决策永远卡在"已到达、发 GRIP"。机器人变成**原地不动等超时**，且 10 s/13 s 看门狗只会让它转小圈（S-11），不会自救。
- **严重度**：blocker
- **建议修法**：`is_idle()` 加入 `TransportPhase.VIOLATION`；在 `autonomous_state._run_once()` 加 `if self._transport.is_violation(): self._transport.reset(); self._decision.reset_after_violation()`；`decision_engine` 增加 `reset_after_violation()`（清 `_current_target/_trip_targets/_fallback_level`，并把 `_strategy_state` 拉回 `FREE_RUN`）。

## S-05 感知坐标系缺少"机器人朝向"旋转，目标定位随车头方向整体错位
- **位置**：`src/rescue_robot/perception/perception_pipeline.py:189-211`（换算处）、`src/rescue_robot/perception/detection.py:356-360,383,398`（相对坐标定义）、`src/rescue_robot/states/autonomous_state.py:257`（传参处）
- **证据**：
  ```python
  # detection.py:398  estimate_ground_position 明确返回"机器人坐标系"
  return (x_off, d)      # (横向偏移 mm, 前方距离 mm)  —— 车体系
  # perception_pipeline.py:207-211
  # 转换到场地坐标（机器人位置 + 相对位置）
  target.position = (robot_position[0] + pos[0],
                     robot_position[1] + pos[1])      # ← 只做平移，没有按 theta 旋转
  ```
  调用侧 `autonomous_state.py:257` 传 `robot_position=(x, y)`（`self._pose` 三元组里的 x/y），**theta 被丢弃**。
- **影响**：车体系 +x（右）、+y（前）必须按 `theta` 旋转才能落到场地系。机器人绕场一周（`theta` 变化 2π）时，同一个目标会被写成**绕机器人一圈**的若干不同场地坐标；`WorldMap` 的 100 mm 最近邻关联会把它们当成多个目标（或对不上而新建 id），于是"目标 id 抖动 + 导航目标点离实物几十厘米到几百厘米"。这是"看起来在感知、就是抓不到"的根因级缺陷。
- **严重度**：blocker
- **建议修法**：`PerceptionPipeline.update()` 增加 `robot_theta` 参数（或在 `detection.estimate_ground_position` 出口做旋转），换算改为
  `wx = x + pos[1]*cos(theta) - pos[0]*sin(theta)`，`wy = y + pos[1]*sin(theta) + pos[0]*cos(theta)`；`autonomous_state.py:257` 传 `robot_pose=(x, y, theta)`。同时把 `CAMERA_*` 焦距/主点改为按真机标定值注入。

---

## S-40 一趟装载"假装抓到全部"：到达第 1 个目标就把它余下的目标全部记入货舱（多目标转运实际不成立）
- **位置**：`src/rescue_robot/transport/transport_pipeline.py:303-341`（CAPTURING 分支）、`:333-338`（一次性 `load` 全部）、`src/rescue_robot/states/autonomous_state.py:270-278`（把整批 id 交给 `start_trip`）
- **证据**（`/tmp/mission9.py`，纯软件）：
  ```python
  # 计划装载 3 个分散目标（核心 2400,2500 / 普通 400,400 / 普通 2600,400）
  # 车只开到了第 1 个目标 (2400, 2500)
  tp.update(car, wm, N()); tp.update(car, wm, N())
  print(tp.load_manager.state.count, tp.load_manager.state.target_ids)
  ```
  ```
  计划装载: [(1000,'CORE_SUPPLY',(2400,2500)), (1001,'REGULAR_SUPPLY',(400,400)), (1002,'REGULAR_SUPPLY',(2600,400))]
  start_trip -> (True, <Violation.NONE: 1>)
  车只到达第 1 个目标 (2400, 2500)，实际记入装载: 3 个 ids={1000, 1001, 1002}
  载具状态: TRANSPORTING
  ```
  代码本身（`:333-338`）：
  ```python
  if success:
      for t in self._current_targets:          # ← 遍历"本趟计划"的全部目标
          ok, v = self._load_mgr.load(t.info, t.id)   # ← 逐个记入，没有任何"车-目标一致性"校验
  ```
  而 `CAPTURING` 的进入条件是"距**第 1 个**目标 <150mm"（`:293-300`），并且**没有任何"逐个目标前往并分别夹取"的状态转移**：`_phase` 从 `CAPTURING` 直达 `TRANSPORTING`（`:339`）。
- **影响（在比赛里损失什么）**：
  1. **丢分**：软件认为满载（3 个），实际只带 1 个到安全区；`LoadManager.release_all()` 按 3 个计分、`_targets_delivered += 3`，但场上另外 2 个目标仍躺在原地 → 实际得分只有名义值的 1/3。
  2. **额外丢分/违规风险**：`SafeZonePlacer.classify_batch()` 用**机器人位置**判定（S-19），会把这"3 个"全判为 valid（不漏报），于是**不会触发 -10 分/个的告警**，问题被完全掩盖；同时视觉确认（`_sleeve_confirm`，只查 ROI 里"有没有 1 个"检测框，`perception_pipeline.py:113-118`）也会返回 True，于是"套取成功"。
  3. **浪费时间**：剩下 2 个目标下一轮又被优先选中（核心物资分最高），车再跑一趟同一片区域，180 s 内趟次被空耗。
  4. 真机上还会退化成"每趟只搬 1 个"：因为只有第 1 个目标真的在槽里，`SLEEVE_ROI` 里只会有 1 个检测框，但代码判"有检测框 → 成功"，所以不会重试、也不会发现少了 2 个。
- **严重度**：blocker
- **建议修法**：在 `TransportPipeline` 增加"多目标逐个套取"的显式状态机：`APPROACHING(target_i) → CAPTURING(target_i) → 校验该目标在槽（视觉/位置） → 若 i < len(targets): 重新 `nav.set_target(targets[i+1].position)` 并回到 APPROACHING`；只有全部到达且逐个确认后才 `load()` 该目标并进入 `TRANSPORTING`。最小改动版（可当天完成）：把 `TransportPipeline.update()` 的 `CAPTURING` 分支改为
  ```python
  reached = [t for t in self._current_targets if self._distance((rx, ry), t.position) < 150]
  if len(reached) < len(self._current_targets):
      self._phase = TransportPhase.APPROACHING        # 还有目标没到，继续开
      if nav: nav.set_target(*self._current_targets[len(reached)].position)
      return self._get_status()
  # 全部到位后才 lower_with_retry + 逐个 load
  ```
  并在 `load()` 循环里用 `self._distance(...)` 复核位置（不满足即抛 violation 而不是静默记入）。
- **一句话**：不改这一条，"一趟 3 个"的核心得分策略在比赛中不成立。

## S-41 裁判强制分离规则完全无响应：接触超 10 s 不会回出发区继续
- **位置**：`src/rescue_robot/perception/opponent_tracker.py:156-179`（接触计时）→ `src/rescue_robot/perception/perception_pipeline.py:220`（喂数据）→ **无人读取**；`src/rescue_robot/decision/decision_engine.py:398-418`（`handle_forced_separation` 定义，零调用方）
- **证据**：
  ```
  grep -rn "handle_forced_separation|FORCED_RESET" src/  →
    decision_engine.py:398 def handle_forced_separation(...)   # 定义
    decision_engine.py:409 _handle_forced_reset(...)           # 定义
    target_selector.py:28 FORCED_RESET = auto()                # 枚举
    → 没有任何调用方（主循环/通信/异常处理都不调用）
  grep -rn "contact_duration\|in_contact\|contact_warning" src/ →
    只在 opponent_tracker.py 内部 + perception_pipeline 的 get_stats()
  ```
  `opponent_tracker` 的输出唯一的"出口"是 `perception_pipeline.get_stats()`（`:282`），**不进入决策**。
- **影响（在比赛里损失什么）**：README 规则「两台机器人接触超过 10 秒 → 强制分离，放回各自出发区继续运行（计时不中断）」。上位机**不知道发生了强制分离**：`_pose` 仍来自下位机里程计（分离后位置跳变，里程计不会自己重定位，除非重新 `START`），于是分离后机器人会**朝着错误的坐标继续跑**（导航目标按旧坐标算），最坏情况撞紫边/撞墙/冲出场地；`handle_forced_separation` 这个"恢复后继续"的实现连同 `FORCED_RESET` 状态一起是死代码。同时 `AnomalyHandler` 里"接触 >10 s → ESCAPE_MANEUVER"也永不触发（S-11：`contact_duration_s` 调用侧恒 0）。
- **严重度**：high
- **建议修法**：① `autonomous_state._run_once()` 读取 `self._perception.opponent_tracker.contact_duration_s`，`>10 s` 时调用 `self._decision.handle_forced_separation(new_pose)` 并把位姿重定位到出发区（`chassis.set_start_pose(150,150,π/2)` + `navigation.reset_pose(...)`）；② 增加"位姿跳变检测"兜底：单帧位移 >400 mm 或 `theta` 跳变 >1.5 rad 时判定被搬动/分离，自动重定位；③ 把 `handle_forced_separation` 接上后 `strategy_state=FORCED_RESET` 才会真正起作用（否则该分支永远走不到）。

---

# 二、high（高）

## S-06 config/*.yaml 基本没接线：改 YAML 对比赛行为几乎无效
- **位置**：`src/rescue_robot/config.py:126-148`（唯一的应用函数）、`src/rescue_robot/main.py:102-109`（唯一调用点）、`config/robot.default.yaml`
- **证据**：
  ```python
  # config.py:133-148 只应用 timing / thresholds 两个段
  t = cfg.timing
  Timing.BUTTON_DEBOUNCE_MS = t.button_debounce_ms
  ...
  th = cfg.thresholds
  Thresholds.BATTERY_MIN_VOLTAGE = th.battery_min_voltage
  ```
  `[复现]`（true dataclass 语义本身没问题，能生效）：
  ```
  AFTER apply(debounce=999, min_fps=77)
  Timing.BUTTON_DEBOUNCE_MS = 999      # ✓ 生效（类属性赋值，实例读同名类属性也变）
  thresholds.CAMERA_MIN_FPS = 77       # ✓ 生效
  ```
  **但 YAML 的其余段落没有任何消费者**（grep 全仓 `src/`）：
  | YAML 键 | 实际实现 | 结论 |
  | --- | --- | --- |
  | `robot.motors.*`（pid/max_speed/wheel_base） | `motion_control.py:108-126` 全部硬编码 | 未接线 |
  | `robot.match.*`（duration/time_pressure/nav/grip/transport timeout） | `decision_engine.py:98-104` 硬编码 | 未接线 |
  | `robot.fallback.*` | `anomaly_handler.py:70-76` 硬编码 | 未接线 |
  | `robot.strategy_weights.*` | `target_selector.py:102` 用固定 `points**1.3` 公式，权重无处读取 | 未接线 |
  | `perception.detection.remove_frames=30` | `world_map.py:97 MAX_LOST_COUNT = 150` | 不一致且未接线 |
  | `perception.detection.stale_frames=3` | `world_map.py:89 MIN_SEEN_COUNT = 3` | 巧合一致，未接线 |
  | `perception.detection.association_threshold_mm=100` | `world_map.py:85 ASSOCIATION_DISTANCE_MM=100.0` | 巧合一致，未接线 |
  | `perception.detection.min_confidence=0.6` | 无人读（`perception_pipeline.py:92` 的 0.25 是另一回事） | 未接线 |
  | `communication.*`、`logging.*` | 无人读 | 未接线 |
- **影响**：决赛"创新实践环节现场改 YAML 免重编译"的能力**基本不存在**；`MATCH_DURATION_S=180` 尤其危险 —— 若官方赛时为 180 s 以外，机器人会提前/滞后停（配合 S-01 更糟）。`config_loader.py` 自带的 `__main__` 自测（`:601-694`）只验证"解析进了 dataclass"，从不调用 `apply_robot_config`，所以这个问题永远不会被它发现。
- **严重度**：high
- **建议修法**：在 `config.apply_robot_config()` 里补 `MotionController.PID_DISTANCE/PID_ANGLE/max_linear_speed/max_angular_speed/wheel_base`、`DecisionEngine.MATCH_DURATION_S/TIME_PRESSURE_S/*_TIMEOUT_S`、`AnomalyHandler.WATCHDOG_*`、`WorldMap.MAX_LOST_COUNT/MIN_SEEN_COUNT/ASSOCIATION_DISTANCE_MM`、`TargetSelector` 权重；或改用"实例注入"（给各管线传入 cfg）而不是改类属性。并在 `config_loader.__main__` 自测里加一条"apply 后断言 `MotionController.PID_DISTANCE[0]` 等被改到"的用例。

## S-07 看门狗"15 s 大限"是死代码；探索档位永不触发；保命模式是与当前位姿重合的 0 向量（原地不动）
- **位置**：`src/rescue_robot/states/autonomous_state.py:343-362`、`src/rescue_robot/decision/decision_engine.py:485-491`
- **证据**：
  ```python
  # autonomous_state.py:353-362
  idle_duration = time.time() - self._last_action_time
  if idle_duration > self.WATCHDOG_SURVIVAL_S and not self._survival_triggered:   # 13s 先判
      self._survival_triggered = True
      self._navigation.survival_circle()
  elif idle_duration > self.WATCHDOG_EXPLORE_S and not self._explore_triggered:   # 10s 后判
      self._explore_triggered = True
      self._navigation.explore()
  ```
  ```python
  # decision_engine.py:485-491
  def _get_survival_target(self, rx, ry):
      radius = 500
      angle = time.time() % (2 * 3.14159)
      tx = rx + radius * 3.14159 * 0.001  # 微小移动  → 0.00157*500 = 0.785 mm
      ty = ry + radius * 0.001            #             → 0.5 mm
      return (tx, ty)                      # ≈ 当前位姿本身
  ```
- **影响**：
  1. `idle > 13` 的分支先命中并置 `_survival_triggered`，10 s 的"探索模式"在 10–13 s 窗口内会因为 `_explore_triggered` 未置位而在下一帧被跳过 → `explore()` 实际上**永远不执行**（只有 0–13 s 之间恰好跨过 10 且未跨过 13 的那 1 帧能触发一次）。
  2. `survival_circle()` 把目标设在距当前位姿 <1 mm 处（`navigation_pipeline.py:348-360` 还额外加了 400 mm 半径却仍用 `rx + 400*cos` 的写法，实际因 `tx/ty` 直接取自 `rx/ry` 而变成同一点附近），`MotionController.is_at_target()` 容差 40 mm 立即判"已到达"→ 速度为 0 → `_last_action_time` 不更新 → `_survival_triggered` 已 True 不再重设目标 → **永久原地不动**。
  3. `WATCHDOG_HARD_LIMIT_S = 15.0`（`:53`）在本文件中从未被引用 → 死常量；`MEMORY.md` 声称的"四层降级保活链"实际只有两层且第二层失效。
- **严重度**：high
- **建议修法**：改成 `if idle > EXPLORE and not explore: explore(); elif idle > SURVIVAL and not survival: survival_circle()`（先探索后保命，两档独立重置）；`_get_survival_target()` 改为 `angle = (time.time()*0.5) % 2π; tx = rx + 500*cos(angle); ty = ry + 500*sin(angle)` 并按场地边界 clamp；`survival_circle()` 里删掉多余/错误的 `radius*cos` 重复计算。

## S-08 紧急停止链路完全不通知下位机：`send_estop()` 零调用
- **位置**：`src/rescue_robot/hardware/serial_chassis.py:136-138`（定义）、全仓无调用；`src/rescue_robot/state_machine.py:205-214`（`emergency_stop` 只改内部状态）、`autonomous_state.py:281-283`（决策急停只调 `sm.emergency_stop`）
- **证据**：
  ```
  grep -rn "send_estop" src/  → 只有 def 行
  # state_machine.py:205
  def emergency_stop(self, reason=""):
      logger.critical(...)
      self.transition(RobotState.ERROR, ...)     # 仅状态机内部，无底层动作
  ```
  且 `RobotState.ERROR` **没有注册 handler**（`main.py:211-213` 只注册 BOOT/DEBUG/AUTONOMOUS）→ 进入 ERROR 时 `new_handler is None`，没有任何 on_enter 停底盘。
- **影响**：Ctrl+C、决策急停、任何非法转移后的"急停"都只让软件进入 ERROR，**底盘仍会按最后一帧 `VEL` 行驶**，直到下位机通信看门狗 300 ms 保持 / 800 ms 平滑停车（协议第 6 节）才停。机器人会带着错误状态前冲最多 ~0.8 s（850 mm/s 时约 0.4 m）。人工按急停开关也只管硬件，软件急停无效。
- **严重度**：high
- **建议修法**：给 `StateMachine` 增加 `on_emergency_stop` 回调注入点；在 `main.py` 构造后注册 `sm.set_emergency_callback(lambda: chassis and (chassis.send_velocity(0,0), chassis.send_estop()))`；`AutonomousState` 的 `ActionType.EMERGENCY_STOP` 分支增加 `self._stop_chassis(); self._chassis.send_estop()`；为 `RobotState.ERROR` 注册一个 `ErrorState` handler（发送 `STOP`/`ESTOP`，LED 常亮红）。

> **更正说明（自审后修订）**：S-09 初稿判为 high，复核后实际不是"回调绑错对象"，已下调为 medium 并移入第三节；S-28 初稿的"10 mm 中心偏差"经复核**不成立**（1345/1655 与 `half_w=290` 一致），真实问题是"到达容差 80 mm vs 放置边界 10 mm 的裕度"，两处均已在下文修订后重述。

## S-09 `RealHardwareChecker` 靠私有属性名注入底盘，一旦改名会静默导致自检失败、机器人进不了 AUTONOMOUS
- **位置**：`src/rescue_robot/main.py:76`（构造时传 `chassis=None`）、`main.py:179-180`（私有属性注入）、`system_check.py:156-158,186,199`
- **证据**：
  ```python
  # main.py:76
  hw_checker = RealHardwareChecker(chassis=None, camera_index=cam_idx)
  # main.py:179-180  ← 用"私有属性名"做依赖注入
  if hasattr(hw_checker, "_chassis"):
      hw_checker._chassis = chassis
  ```
  ```python
  # system_check.py:186 / :199
  if self._chassis is not None and self._chassis.is_open: ... else: return False
  ```
- **影响**：`RealHardwareChecker.__init__` 的公开参数叫 `chassis`，但 `main.py` 传的是 `None`，真正生效靠的是 `hasattr(_chassis)`。**重构改名私有字段 → `hasattr` 静默 False → `self._chassis` 永久 None → IMU 检查与两个电机检查全部 `return False`**（`_is_critical` 语义下 `overall_pass=False`）→ `BootState.on_enter` 转 `ERROR`（`boot_state.py:55-61`）→ 机器人无法进入 AUTONOMOUS。现场只会看到"自检失败 3 项"，排查成本极高。
- **严重度**：medium
- **建议修法**：给 `RealHardwareChecker` 增加公开方法 `set_chassis(self, chassis) -> None` 并在 `main.py` 里显式调用 + 断言；`__init__` 增加 `if chassis is not None: self._chassis = chassis` 以支持直接注入。另见 S-10（自检与主循环相机索引不一致）。


## S-10 相机自检与主循环用的不是同一个设备；且套取视觉确认在"无检测"时直接判失败
- **位置**：`src/rescue_robot/main.py:75`（自检索引默认 1）、`main.py:152`（主循环索引默认 0）、`src/rescue_robot/system_check.py:160-177`（`check_camera`）、`src/rescue_robot/perception/perception_pipeline.py:108-118`（`check_sleeve_occupied`）
- **证据**：
  ```python
  # main.py:75
  cam_idx = int(os.environ.get("CAM_INDEX", "1"))
  hw_checker = RealHardwareChecker(chassis=None, camera_index=cam_idx)
  # main.py:152
  cam_idx = int(os.environ.get("CAM_INDEX", "0"))     # ← 同一个 env，不同的默认值
  ```
  ```python
  # perception_pipeline.py:108-109
  if not self._last_detections:
      return False        # 有帧但没有任何检测 → 槽内空
  ```
  且 `RealHardwareChecker.check_camera()` 通过 `CameraReader.wait_first_frame()` 判断"能开+能出帧"，**不验证画面内容**。
- **影响**：① 自检通过 `/dev/video1`、实际读 `/dev/video0`：自检绿灯但主循环拿不到帧，或反之（`CAM_INDEX` 未设时必然一个是 1 一个是 0）。② 真机若 `frame is None`（相机掉线）→ `_read_line()` 之外的 `CVDetector.detect(None)` 返回 `[]` → `world_map.update([])` 每帧给所有目标 `track_lost_count += 1`，**3 秒后场上所有目标被删除**（`MAX_LOST_COUNT=150`）；重新出帧后全部换新 id（`world_map.py:376 _next_id` 单调递增），`autonomous_state.py:230-235` 的"新一趟"检测被反复触发。③ `check_sleeve_occupied` 用"ROI 内有没有任何低于 0.25 置信度的检测框"判断套住，在真机光照/ROI 未标定下极易持续判 False → `transport_pipeline.py:320-331` 抬爪后退重试，5 次后自动关闭确认（`:324-328`）——"失效保护"实际是把整个确认能力关掉。
- **严重度**：high
- **建议修法**：`main.py` 两个分支统一 `cam_idx = int(os.environ.get("CAM_INDEX", "0"))`，自检与主循环共用同一个 `CameraReader` 实例（自检复用后交给 AutonomousState，避免开两次设备）；`WorldMap.update([])` 在"连续 N 帧无任何检测"时**冻结** `track_lost_count`（或加 `if not detected_targets and self._update_count % 50: return`）；`check_sleeve_occupied` 在 `_last_detections` 为空时返回 `None`（未知），由 `transport_pipeline` 把"未知"按成功处理（与异常路径一致）。

## S-11 `DecisionEngine` 的异常检测每帧被无条件清除，整个"卡死/失控/脱困"子系统是死代码
- **位置**：`src/rescue_robot/decision/decision_engine.py:207-217`（调用与清除）、`anomaly_handler.py:120-142`（无动作检测）、`:175-203`（卡死/碰撞检测）、`:209-256`（脱困）
- **证据**：
  ```python
  # decision_engine.py:208-217
  anomaly = self._anomaly.check(robot_pose, (0, 0),   # ← 速度恒为 (0,0)
                                imu_data, contact_duration_s)   # ← 两者调用侧都是 None/0
  if anomaly.type != AnomalyType.NONE:
      self._strategy_state = StrategyState.ANOMALY
      return self._handle_anomaly(anomaly)
  self._anomaly.notify_action()      # ← 每帧都执行，把 _last_action_time 重置为 now
  ```
  ```python
  # anomaly_handler.py:23-24
  if speed > 10:  self._last_action_time = timestamp     # speed=0 恒不成立
  elif idle_s > self.WATCHDOG_TIMEOUT_S: return ...      # idle_s 恒 ≈0 → 永不触发
  ```
  `[复现]`（grep 全仓）`get_escape_command / is_escaping / keep_moving_fallback / notify_sensor_ok / get_anomaly_count` **只有定义处命中，零调用方**。
- **影响**：`AnomalyHandler.check()` 实际上永远返回 `AnomalyType.NONE`（`speed` 恒 0、`imu_data` 恒 None、`sensor_status` 恒 None）。于是：15 s 无动作检测 ✗、失控检测 ✗、传感器故障降级 ✗、卡死检测（需要 `speed>100`）✗、碰撞卡死（`contact_duration_s` 调用侧恒 0）✗；`_handle_anomaly()` 的 `SURVIVAL`/`EXPLORE`/`ESCAPE` 分支全是死分支；`ANOMALY` 状态一旦被置位也**没有出口**（`update()` 只在 `check()` 返回非 NONE 时才进 `_handle_anomaly`，而 `_handle_anomaly` 对 `EMERGENCY_STOP/ESCAPE/DEGRADE` 都返回 `WAIT`，`_strategy_state` 保持 `ANOMALY`，下一帧 `check()` 返回 NONE 后又从 `_strategy_state` 分支走 `else: return WAIT("未知状态")` —— **永久 WAIT**）。
- **严重度**：high
- **补充（该版本上的一个潜在崩溃，审计期间已被 fixer 触碰到）**：`anomaly_handler.py:121-142` 的 `speed` 只在 `if velocity is not None:` 分支内赋值（旧版直接在函数体赋值），而 `:182` 的卡死检测 `if speed > 100 and dist < ...` 依赖它；当 `idle_s` 落在 `(0, WATCHDOG_WARN_S]` 且 `imu_data is None` 时，`speed` 在 `dist < STUCK_DISTANCE_MM` 分支上是**未定义局部变量** → `UnboundLocalError`。当前之所以不崩，唯一原因是 `decision_engine.py:217` 每帧调用 `notify_action()` 把 `idle_s` 压在 0 附近；一旦按 S-11 的正解"只在动作推进时 notify"（或主循环某帧抛异常跳过了 `update()`），这个崩溃路径就会暴露。修 S-11 时**必须同时**把 `speed` 的初始化提到函数开头（或在 `if imu_data`/卡死检测前统一计算），否则"修好保活"的当天就会崩。
- **建议修法**：①`notify_action()` 只在 `action` 真的推进时调用（把 `decision_engine.py:217` 移到各 `_handle_*` 的成功分支）；②`update()` 签名补 `imu_frame: Optional[dict]`、`sensor_status`、`contact_duration_s`，由 `autonomous_state._run_once()` 从 `chassis.read_imu()`（或 `last_imu_frame` 缓存）+ `_transport.phase` + `opponent_tracker.contact_duration_s` 传入；③`AnomalyHandler.check()` 的 `imu_data` 键名要与 `serial_chassis.parse_imu()` 输出对齐（现为 `'velocity': ...` 取 `imu_data.get("accel_magnitude")`，而解析结果是 `ax_mg/ay_mg/az_mg/gx_mrad_s...`，**键名根本不匹配**）；④`ANOMALY` 状态加 1–2 s 超时回退 `FREE_RUN`。

## S-12 IMU 融合从未接入：`ImuGyroFusion` 零引用，`sensor_fusion` 只是摆设
- **位置**：`src/rescue_robot/perception/imu_fusion.py:22`（类）、全仓无调用；`perception_pipeline.py:68,166`（`SensorFusion` 仅被构造和读取位置）
- **证据**：
  ```
  grep -rn "ImuGyroFusion|corrected_gyro_z|add_calibration_sample" src/ tests/ tools/
  → 全部命中都在 imu_fusion.py 自身（含 __main__ 自测）
  ```
  `SerialChassis.read_imu()` 也没有任何主链路调用方（`grep read_imu` → 只有 `system_check.py:183` 自检）。
- **影响**：协议 5.2.2「由上位机完成陀螺仪零偏校准并把 `gz_corrected_rad_s` 交给编码器+IMU 航向融合」**完全没做**。实际航向 100% 来自下位机纯轮式里程计：打滑/碰撞/轮子卡住后 `theta` 会永久漂移，而 `_is_near_obstacle`、`A*` 起点、坐标变换全都依赖它 → 一次剧烈碰撞即可让全局定位报废且无法恢复（对手机器人推挤是本赛项的常态）。同时 `NavigationPipeline._localizer` 在真机模式下是 `MockLocalizer`（`navigation_pipeline.py:70-74`，`main.py:133` 传 `use_mock=use_mock`），`update()` 里的 `self._localizer.update(cmd...)` 更新的是一个**从未被使用的假位姿**（`autonomous_state.py:220-227` 走 chassis 分支），属"接了线但不影响任何判断"的假动作。
- **严重度**：high
- **建议修法**：在 `AutonomousState.on_enter()` 的 `START` 之后、延迟期内做一次 `ImuGyroFusion.start_calibration()` + 收满 100–200 帧（`read_imu()` 现在是阻塞循环，注意别拖慢 50 Hz，建议放后台线程或在 1 s 延迟里做）；主循环每帧 `chassis.read_imu()`（非阻塞版本：加 `read_imu_now()` 只读一行，命中 `IMU` 才解析）喂给融合器，把 `theta` 用 `w_kalman = 0.95*gyro + 0.05*odom` 融合后写回 `self._pose`；或至少把 `theta` 漂移率用于"重新对齐出发区+减速带"的视觉重定位。
- **备注**：若时间不够，最小可用修法是"IMU 只用于碰撞/打滑检测（`gz` 突变 + 里程计 `theta` 不变）→ 触发重定位"，比完整融合便宜得多。

## S-13 动态障碍清除会连禁区区一起抹掉，A* 可规划出穿进对方安全区的路径
- **位置**：`src/rescue_robot/navigation/path_planner.py:68-72`（`clear_dynamic`）、`navigation_pipeline.py:180-185`（每帧调用）、`forbidden_zones.py:194-199`（只写一次）
- **证据**：
  ```python
  # path_planner.py:68
  def clear_dynamic(self) -> None:
      for x, y, r in self._dynamic_obstacles:
          self._clear_circle(x, y, r)      # 无条件把圆内格子置 COST_FREE
      self._dynamic_obstacles.clear()
  ```
  `[复现]`
  ```
  forbidden cells: 341
  after clear_dynamic: forbidden cells: 328
  LOST forbidden cells: 13 [(1225,2975) ... (1675,2975)]   # 对方安全区顶部条带被抹成自由
  ```
  且 `ForbiddenZoneManager.write_to_cost_map()` 只在 `NavigationPipeline.__init__`（`:96`）调用过一次，**没有任何地方重写**。
- **影响**：对方机器人出现在对方安全区附近（本赛项常见）时，`add_obstacle_circle(..., radius_mm=350)` 覆盖到的禁区区在**同一帧**就被清零；此后 A* 认为该处可通行 → 机器人可能主动开进对方安全区（-5 分/次），或在安全区内部规划出路径后卡在紫边上。`_check_violation` 只在 `navigation_pipeline.py:264` 的事后回退分支里生效（且回退是 `linear=-200`，见 S-17）。
- **严重度**：high
- **建议修法**：`CostMap` 记录"静态层/动态层"两张网格（`_static_grid` / `_dynamic_grid`），`clear_dynamic()` 只清动态层，`get_cost/is_free` 取两层最大值；或在 `NavigationPipeline.update()` 里每帧先 `self._forbidden.write_to_cost_map(self._cost_map)` 再 `add_obstacle_circle()`（顺序反过来，且不要再调 `clear_dynamic`）。

## S-14 主循环单帧异常会整体跳过 `VEL` 下发，下位机看门狗随即停车（可反复出现）
- **位置**：`src/rescue_robot/states/autonomous_state.py:203-213`（粗粒度 try）、`perception_pipeline.py:172-177`（`frame.shape` 的静默 `except`）、`serial_chassis.py:176-179`（`readline` 异常只 warning）
- **证据**：
  ```python
  # autonomous_state.py:203-213
  try:
      self._run_once(dt)                 # 感知+决策+导航+转运+串口下发 全在这一句里
  except Exception as e:
      logger.error(f"主循环异常: {e}", exc_info=True)    # 只打日志
  ```
  `_run_once()` 内部任何一步抛异常（例如 `autonomous_state.py:291-292` 的 `controller.execute(cmd)` —— **`execute` 方法在整个 src/ 里不存在**（grep `def execute` 无命中），一旦有人传了 controller 就是 `AttributeError`；或 `transport_pipeline.py:311 lower_with_retry` 里 `KeyError`），当帧的 `send_velocity` / `transport.update` / 看门狗更新**全部被跳过**。
- **影响**：异常帧不发 `VEL`；连续异常帧则下位机进入 300 ms 保持 → 800 ms 平滑停车（协议第 6 节），车"一瘸一拐"。同时因为异常被吞，`_stop_chassis()`（`:323-339`）也不会被调用，最坏情况是**车停在目标上方不动而软件以为在跑**。日志有 `exc_info=True`，但只在本地，比赛现场没人看。
- **严重度**：high
- **建议修法**：① 把 `_run_once()` 拆成"安全段（发速度/看门狗/停车）"和"可失败段（感知/决策/转运）"，用 `try/except` 分别包裹，保证**无论哪一段抛异常，末尾一定有一次 `send_velocity` 或 `send_stop`**；② 加连续异常计数器，≥3 次则 `self._stop_chassis()` + 降级到固定保命速度；③ 删掉 `controller` 分支或补上 `ChassisInterface.execute()`。

## S-15 `main.py` 的按钮轮询与 50 Hz 主循环抢同一个串口，一键启动可能永远收不到
- **位置**：`src/rescue_robot/main.py:247-262`（每 0.5 s 读一行）、`serial_chassis.py:254-263`（`read_button` 只读一行）
- **证据**：
  ```python
  # main.py:253-257
  if current_state == RobotState.DEBUG and chassis is not None and chassis.is_open:
      if chassis.read_button():        # 0.5s 一次，只读一行
          sm.one_key_start()
  ...
  time.sleep(0.5)
  ```
  ```python
  # serial_chassis.py:260-263
  def read_button(self) -> Optional[str]:
      text = self._read_line()          # 只读一行，非 EVENT,START_BUTTON 就丢
      if text and text.upper().startswith("EVENT,START_BUTTON"): return text
      return None
  ```
- **影响**：下位机以 20 Hz(ODOM)/50 Hz(IMU) 混流 + `EVENT,*` 单次上报（协议 5.1.1）。每 0.5 s 只取一行 → 命中 `EVENT,START_BUTTON` 的窗口极窄，**大概率漏掉**；`wait_for_button()`（`serial_chassis.py:250-252`，带 5 s 超时能连贯读流）**从未被调用**。结果是"按了启动键没反应"，而现场只能靠笔记本键盘（Mock 按钮）启动 —— 真机 `main.py:73` 用的是 `MockButton`。
- **严重度**：high
- **建议修法**：DEBUG 状态改用 `chassis.wait_for_button(timeout=0.5)` 循环（它内部是 `wait_for` 连续读行，不会漏事件）；或把 `read_button()` 改为"在一个有界循环里读到 `EVENT,START_BUTTON` 或超时为止"，并让 `wait_for()` 顺带缓存 ODOM/IMU 帧供后续使用（避免事件帧被丢掉）。

## S-16 套取机构没有任何自检，"电机检查"只是 PING
- **位置**：`src/rescue_robot/system_check.py:192-199`（`check_motor` 实际是 `send_ping()`）、`transport/sleeve_lift.py:353-361`（`SERVO,LOWER` 无 ACK 校验）
- **证据**：
  ```python
  # system_check.py:192-199
  def check_motor(self, motor_id: int) -> bool:
      # 电机正反转测试需真机；框架先用串口 PING 探测底盘连通
      if self._chassis is not None and self._chassis.is_open:
          return self._chassis.send_ping()
      return False
  ```
  ```python
  # sleeve_lift.py:358-361
  if not self._chassis.send_servo("LOWER"):
      logger.warning("SERVO,LOWER 发送失败")
      return False
  self._chassis.wait_for("ACK,SERVO,LOWER", 0.5)     # ← 返回值被丢弃，超时/ERR,LOCKED 都当成功
  ```
- **影响**：① 关键执行机构（U 型套取舵机，唯一的"手"）**完全没有自检**，舵机线松/舵机卡死/PB6 PWM 无输出，BOOT 一律绿灯放行。② 两个"电机检查"其实检查了三遍同一条串口链路（IMU 检查也是 `read_imu`），`MOTOR_CHECK_DURATION_MS=500`（`config.py:50`）从未被使用。③ `SERVO,LOWER` 的 ACK 不校验：下位机回 `ERR,LOCKED`（协议 4.4：ESTOP 后拒绝舵机动作）时上位机仍把 `_state.action` 置 LOWERED/HOLD 并继续"套住搬运"，实际什么都没夹住 → 空手跑到安全区放下空气，整趟白费且软件显示成功。
- **严重度**：high
- **建议修法**：`RealHardwareChecker` 增加 `check_sleeve()`：`send_servo("RAISE")` → `wait_for("ACK,SERVO,RAISE", 0.5)` → `send_servo("LOWER")` → 等 ACK → `SERVO,ANGLE,35` → 等 ACK，任一失败即 FAIL（列为 critical）；`SerialServoLift.lower()/raise_up()/place_ramp()` 把 `wait_for` 返回值取出判断，`None`（超时）或含 `ERR,` 时返回 False 并让上层重试；`check_motor` 改为发送 `VEL,120,0` 0.5 s → 读 ODOM 的 `encL/encR` 增量 → 判"电机是否真的动"。

---

# 三、medium（中）

## S-17 禁区违规时用固定 `linear=-200` 倒车脱困，无避障、无复规划、不看后方
- **位置**：`src/rescue_robot/navigation/navigation_pipeline.py:263-269`
- **证据**：`if violation: return VelocityCommand(linear=-200.0, angular=0.0, ...)`
- **影响**：`MotionController.compute_velocity()` 明确只允许前进（`motion_control.py:190` `linear = max(0, linear)`），所以这 -200 是"绕过整个避障体系"的后门；后方若有围墙/紫边/对方机器人，会直接撞上去。而且它是**每帧都判**，一旦进入禁区就会持续倒车直到退出判定（`check_violation` 用硬边界，可能一路倒到场外）。
- **严重度**：medium
- **建议修法**：改为 `self._state = NavState.BLOCKED; self._astar.plan(...)` 重新规划并返回 0 速度；或调用 `LocalPlanner` 时把"倒车"作为一个采样方向（而非硬编码），并限制连续倒车时间 ≤1 s。

## S-18 `LocalPlanner` 全采样碰撞时返回 (0,0)，机器人原地僵死
- **位置**：`src/rescue_robot/navigation/path_planner.py:309-331`（初始 `best_v, best_w = 0.0, 0.0`）、`navigation_pipeline.py:247-256`（覆盖 `cmd`）
- **证据**：`[复现]`
  ```
  LocalPlanner result when start is next to obstacle: 540.0 0.735
  LocalPlanner result when robot inside obstacle:      0.0   0.0
  LocalPlanner.plan avg 0.6 ms
  ```
  （`:315-316` 的动态窗口在 `w_now=0, dt=0.5` 时 `w_window=(0,0)`，`:321` 只生成 `w=0`；若 `v` 采样全部落在障碍格，`_evaluate_trajectory` 全返回 `-inf`，就保持初值 0,0）
- **影响**：`navigation_pipeline.py:247` 的 `_is_near_obstacle()` 把"周围 ±2 格（±100 mm）内有 `COST_OBSTACLE`"判为接近障碍，而 A* 的路径走廊距禁区区常常就在 1 格内 → **正常沿路行驶时也会进入避障分支**。一旦起点被判为障碍格（例如位姿漂移导致车"落在"禁区区格上，S-12 的漂移会促成此事），输出恒为 (0,0)：车停、看门狗更新不了 `_last_action_time`、13 s 后进保命圈（S-07 又是 0 向量）→ **彻底死锁在场地中间**。
- **严重度**：medium
- **建议修法**：`plan()` 里若 `best_score == -inf` 则返回传入的 `current_vel`（保持原速）或一个"停车可用"的保守值，并在日志中区分"无可行采样"；`_is_near_obstacle()` 的半径 2 → 1，并排除当前格子自身。

## S-19 目标"投放位置"判定用的是**机器人**坐标，不是目标落点
- **位置**：`src/rescue_robot/transport/transport_pipeline.py:404-407`
- **证据**：
  ```python
  positions = [(rx, ry)] * len(self._current_targets)     # 全部用机器人当前位置
  infos = [t.info for t in self._current_targets]
  results = self._placer.classify_batch(positions, infos)
  all_valid = all(r.is_valid for r in results)
  ```
- **影响**：`SafeZonePlacer.classify()` 的语义是"目标落在哪"，这里传的是"车在哪"。因为投放点就是安全区中心，通常两者足够接近而"撞对"；但 `_is_fully_inside` 要求距边界 ≥10 mm（`safe_zone_placer.py:216-221`），车停在安全区**中心**必然通过 —— 也就是说**投放判定几乎恒为 valid，永不发现"放到伤员区/物资区搞反了"**（-10 分/个的规则形同虚设）。同理 `PlacementZone.ON_FENCE` 分支也几乎不可能命中。
- **严重度**：medium
- **建议修法**：用"目标被释放时的位姿"判定：`positions = [(rx + math.cos(rtheta)*d, ry + math.sin(rtheta)*d) for _ in ...]`（d = 套取臂伸出方向投影距离，真机标定），或在 `place_ramp()` 前记录 `nav.target`（投放点中心）作为落点估计；同时把判定前移到 `classify()` 之前的 DEBUG 日志里打出实际坐标，便于真机校准。

## S-20 `LoadManager` 批量装载中途失败会留下"半批"状态
- **位置**：`src/rescue_robot/transport/transport_pipeline.py:333-338`
- **证据**：
  ```python
  if success:
      for t in self._current_targets:
          ok, v = self._load_mgr.load(t.info, t.id)
          if not ok:
              self._phase = TransportPhase.VIOLATION
              return self._get_status()        # ← 前面已经 load 成功的没回滚
  ```
- **影响**：批内第 2 个失败时，第 1 个已经在 `LoadState.targets/target_ids/count` 里；`_phase` 又进 VIOLATION（S-04 无出口）。`world_map.mark_being_transported` 也已把两个目标置 `BEING_TRANSPORTED`（`autonomous_state.py:298-302`），而该状态**永远不会回到 ACTIVE**（`world_map.py:424-427` 只设不回）→ 这两个目标永久退出候选集。
- **严重度**：medium
- **建议修法**：装载前先 `can_load_batch()`（已在上游做过，但套取成功后要重做一遍，因为 `success` 可能返回多于规划的数量）；失败时调用新增的 `LoadManager.rollback_last_batch(ids)`；`world_map.mark_being_transported` 配套增加 `mark_transport_failed(tid)` 把状态改回 ACTIVE（并在 `MAX_LOST_COUNT` 到期删除时保证不会残留）。

## S-21 `_check_invalid_transport` 误判：场地中央任何 ACTIVE 目标都会导致中途换目标
- **位置**：`src/rescue_robot/decision/decision_engine.py:314-334`
- **证据**：
  ```python
  for tid, t in self._world_map.targets.items():
      if t.status == TargetStatus.ACTIVE:
          dist_to_center = math.sqrt((t.position[0]-1500)**2 + (t.position[1]-1500)**2)
          if dist_to_center < 500:
              return True                     # 只要有目标在中央 500mm 内
  ...
  if self._check_invalid_transport(rx, ry):
      self._current_target = None             # 直接把当前目标丢掉
  ```
- **影响**：设计意图是"检测裁判把无效目标放回中心"，实现却是"场上任何一个目标靠近中心就重置"。本赛项中央区域本来就是目标密集区 → 每次进入 `_handle_free_run` 都可能把 `_current_target` 清成 None，于是**正在搬运途中（`grip_done=True`）也会重新选目标**，与 `TransportPipeline` 的 `_current_targets` 失去同步：车把 A 送回去，决策以为在搬 B。
- **严重度**：medium
- **建议修法**：只在"当前目标 `track_lost_count` 突然归零且位置跳到中心"时判无效，或由 `TransportPipeline` 上报"本次投放被判无效"事件来驱动，而不是扫描全场。

## S-22 世界地图 3 s 无帧即删目标，安全区内的目标会以新 id 复活
- **位置**：`src/rescue_robot/perception/world_map.py:255-256,292-298`、`world_map.py:418-422`（`mark_in_safe_zone`）
- **证据**：
  ```python
  for target in self._targets.values():
      target.track_lost_count += 1          # 每帧无条件 +1
  ...
  stale_ids = [tid for tid, t in self._targets.items()
               if t.track_lost_count > self.MAX_LOST_COUNT]   # 150 帧 = 3 s
  for tid in stale_ids: self._targets.pop(tid)
  ```
- **影响**：`mark_in_safe_zone()` 只是把 `status` 改成 `IN_SAFE_ZONE`，`_targets` 里仍在、且**仍会被 3 s 超时删除**。删除后如果视觉又看到它（安全区就在场内，镜头会扫到），`_offer_pending/_create_new_target` 会创建一个**新 id、状态 ACTIVE** 的目标（id 从 `_next_id` 单调取，见 `:376`）→ 决策把已经投放的目标当成新目标去抓，`world_map.is_in_safe_zone()`（`:139-155`）能挡住这一层（`_selectable`），**但前提是位置估计正确**——而 S-05 的坐标错位会让它"看起来不在安全区"。文件里 `MAX_LOST_COUNT` 的注释本身就在描述这个旧的重复扑空问题。
- **严重度**：medium
- **建议修法**：`IN_SAFE_ZONE` 的目标不参与 `track_lost_count` 递增、也永不被超时删除（单独存 `_delivered` 字典）；`_create_new_target` 时先查"同类型 + 位置落在任一安全区内"则直接丢弃该候选。

## S-23 主循环首帧就阻塞：`read_pose()` 串口超时 + `A*` 单次 26 ms，都超出 20 ms 预算
- **位置**：`src/rescue_robot/states/autonomous_state.py:220-221,286-290`、`serial_chassis.py:80`（`timeout=0.02`）、`path_planner.py:190-240`（A*）
- **证据**：`[复现]`
  ```
  A* ok=True len=60  25.9 ms         # 60×60 网格单次规划
  LocalPlanner.plan avg 0.6 ms
  PerceptionPipeline.update avg 2.1 ms （640×480 含 20 个色块）
  CVDetector.detect avg 2.4 ms
  ```
  `SerialChassis.__init__(timeout=0.02)` → `read_pose()` → `self._ser.readline()` 最多阻塞 **20 ms**，占满整个 50 Hz 周期。
- **影响**：`while` 循环的节拍是 `sleep(max(0, dt - elapsed))`（`:210-213`），所以不会累积漂移；但**每次 `readline()` 吃到非 ODOM 行（IMU/TEL/ACK）就返回 None，`self._pose` 保持上一帧**（`:222-224`），而阻塞的 20 ms 已经花掉。实测 ODOM 20 Hz + IMU 50 Hz 混流时，约 **5/6 的帧拿不到新位姿**，实际控制用的位置刷新率只有 20 Hz。叠加 A* 重规划帧（26 ms）会出现单帧 40+ ms 的停顿，下位机速度环会出现一次"保持最后一帧"的抖动。
- **严重度**：medium
- **建议修法**：① `SerialChassis` 改成"后台读线程 + `latest_odom()` 非阻塞取最新帧"（复用 `CameraReader` 的模式），主循环不再直接 `readline()`；② `A*` 改 `heapq` 优先队列并把重规划放到独立线程/低频（例如每 10 帧一次，或仅在 `state==PLANNING/BLOCKED` 时同步规划），其余帧复用旧路径；③ `_run_once` 用 `time.perf_counter()` 统计各段耗时并在超过 20 ms 时打 warning，便于现场定位。

## S-24 异常路径下状态机/资源释放不完整（4 处）
- **位置**：`main.py:233,269-291`、`state_machine.py:143-154`、`autonomous_state.py:179-189`、`hardware/camera_reader.py:101-113`
- **证据**：
  ```python
  # main.py:269-274  finally 里的 button 可能未定义
  finally:
      logger.info("正在清理资源...")
      try: button.stop_monitoring()      # 若异常发生在 _create_hardware 之前 -> NameError
      except Exception: pass             # 但被吞掉，日志说"已清理"
  ```
  ```python
  # state_machine.py:205-214  强制进 ERROR 时不走 on_exit
  except ValueError:
      with self._lock: self._state = RobotState.ERROR    # AUTONOMOUS 的 on_exit 不执行
  ```
  ```python
  # autonomous_state.py:282  EMERGENCY_STOP 分支
  self._sm.emergency_stop("决策引擎请求急停")
  return                            # 主循环继续；不清理 _transport/_navigation 状态
  ```
- **影响**：① `KeyboardInterrupt` 由 `except KeyboardInterrupt` 分支处理，**不经过 `finally` 之前的状态退出**，`AutonomousState.on_exit()` 未必执行（`_stop_event` 未置位、`join(timeout=2.0)` 不执行）→ 线程留着、摄像头/串口靠 `finally` 兜底（这两个确实有释放，`main.py:280-291`）。② `emergency_stop()` 的强制覆盖分支绕过 `on_exit`，AUTONOMOUS 的清理（停主循环、`send_stop`）被跳过。③ `CameraReader.stop()` 里 `join(timeout=2.0)` 超时后仍然 `release()`（`:105-112`），此时后台线程可能正在 `cap.read()` → 偶发崩溃/句柄泄漏。④ `TransportPipeline` 在 VIOLATION/中途退出时不动舵机，`finally` 也只 `close()` 串口，不 `send_servo("RAISE")`。
- **严重度**：medium
- **建议修法**：`main.py` 在 try 之前初始化 `button = MockButton()`（或 `button = None` + `if button is not None`）；`StateMachine.emergency_stop` 的强制分支补一次 `old_handler.on_exit()`；`AutonomousState.on_exit()` 增加 `self._stop_chassis()` 与 `self._transport.reset()`（内含 `sleeve.raise_up()`）。

## S-25 `world_map` 的 `field` 属性被赋值但从未使用；`is_stale` 属性零消费
- **位置**：`world_map.py:118-124`（`field` property + setter）、`:55-60`（`age_s/is_stale`）
- **证据**：`grep "\.is_stale"` 仅 `opponent_tracker.py`（不同类）；`world_map.field` 无读取方
- **影响**：`WorldMap.field` 看似是"场地布局已绑定"的接口，实际 `is_in_safe_zone()` 走的是 `self._field` 的**私有**字段（`:147` 直接判 `self._field is None`），setter 只是摆设；`TrackedTarget.is_stale`（2 s 阈值）与实际的 `MAX_LOST_COUNT`（3 s）语义重复且无人用 → 两套"过期"概念并存，改一处不生效。
- **严重度**：medium
- **建议修法**：删除 `field` property 或让 `is_in_safe_zone()` 用它；删除 `is_stale/age_s` 或改用它们替代 `track_lost_count`。

## S-26 `CameraReader` 的时间戳与 `get_frame_with_ts()` 零消费，无法判断帧新鲜度
- **位置**：`camera_reader.py:52,149,172-175`
- **证据**：`get_frame_with_ts` 无调用方；`PerceptionPipeline.update()` 只 `get_frame()`（`autonomous_state.py:248`）
- **影响**：相机线程掉线后 `get_frame()` 会一直返回**最后一帧旧图**（`_loop` 只在读到新帧时覆盖 `self._frame`），主循环无法区分"新帧"和"5 秒前的旧帧" → 感知继续基于旧图打点（配合 S-22 会造成错误的目标重建）。这正是 `SLEEVE_CONFIRM` 与目标关联最容易出错的场景。
- **严重度**：medium
- **建议修法**：`PerceptionPipeline.update()` 用 `get_frame_with_ts()`，`age = now - ts > 0.2` 时按 `frame=None` 处理（并在日志里降频打点）。

## S-27 `config/robot.default.yaml` 的 `perception.detection.remove_frames=30` 与代码 `MAX_LOST_COUNT=150` 相差 5 倍
- **位置**：`config/robot.default.yaml`（`perception.detection.remove_frames: 30`）、`world_map.py:94-97`
- **证据**：见 `world_map.py:94-97` 的注释解释为何从 30 改成 150（"短暂遮挡/转头就删目标"）。参数改了，YAML 没跟着改，且 YAML 本来就没接线（S-06）。
- **影响**：现场工程师读 YAML 会得到错误的心智模型；若有人真的按 YAML 把代码改回 30，会立刻复现"重复扑空"的历史 bug。
- **严重度**：medium
- **建议修法**：YAML 与代码对齐（150），并把该注释同步进 YAML 注释里。

## S-28 `SAFE_ZONE 分区中心` 与 `field_elements` 几何差 10 mm，且投放/判定/禁区三处各自硬编码
- **位置**：`decision_engine.py:493-503`（1345/1655 × 2820/180）、`field_elements.py:179-234`（实数应为 1355/1645）、`transport_pipeline.py:369`（判 `dist < 80`）
- **证据**：
  ```python
  # decision_engine.py:496
  return (1345.0, 2820.0)   # 红物资区中心
  # field_elements.py：safe_x=1200, half_w=290 -> 左区中心 = 1200+145 = 1345 ✓
  #                                         右区中心 = 1200+290+20+145 = 1655 ✓
  ```
  （复核后决策引擎的 1345/1655 与 `half_w=290` 一致；**真正的不一致在投放判定**：`transport_pipeline.py:369` 用 `dist < 80` 判"到达投放点"，而 `safe_zone_placer` 的 `_is_fully_inside` 要求距安全区**外框** ≥10 mm，安全区高仅 300 mm、扣掉后可用 280 mm，80 mm 的到达容差已经吃掉可放置区域的一半。）
- **影响**：真实风险不是 10 mm 而是"到达容差 80 mm 与放置边界 10 mm 之间的裕度太小"：位姿略偏就会落在 `_is_fully_inside` 之外，被判 `ON_FENCE`/`OUTSIDE` → `all_valid=False` → 只打 warning（`:420-423`），**不扣分的分支（PLACEMENT_WRONG_ZONE）反而不会触发**，即"放歪了但没被告警到扣分等级"。
- **严重度**：medium
- **建议修法**：把 1345/1655 改为从 `FieldLayout` 查出（`get_safe_zone(my_color)` + `SUPPLY_AREA/INJURED_AREA` 的 `region.center`），三处共用同一来源；`dist < 80` 收到 `dist < 40` 并在 `PLACING` 阶段增加"对齐安全区中心"的小幅位置闭环。

## S-29 `AnomalyHandler` 的 IMU 键名与 `SerialChassis.parse_imu()` 输出完全不匹配
- **位置**：`anomaly_handler.py:145-159`（`imu_data.get("accel_magnitude")` / `"gyro_magnitude"`）、`serial_chassis.py:320-325`（输出 `ax_mg/ay_mg/az_mg/gx_mrad_s/...`）
- **证据**：解析器返回键为 `{'tick_ms','seq','ax_mg','ay_mg','az_mg','gx_mrad_s','gy_mrad_s','gz_mrad_s','temp_cC'}`；`AnomalyHandler` 取的两个键**不存在于任何生产者**（grep `accel_magnitude` 全仓只有 `anomaly_handler.py` 一处）。
- **影响**：即使按 S-11 把 `read_imu()` 接进决策引擎，失控检测依旧恒不触发（`.get()` 返回 None → `if accel > 30` 抛 `TypeError`，被主循环吞掉（S-14）→ **反而把整帧打掉**）。这是一个"修了一半就崩"的陷阱。
- **严重度**：medium
- **建议修法**：在 `AnomalyHandler.check()` 里改为 `accel = math.sqrt(ax**2+ay**2+az**2)*9.80665/1000`、`gyro = math.sqrt(gx**2+gy**2+gz**2)/1000`，键名用 `ax_mg/...`；或在 `SerialChassis` 侧增加 `imu_to_physics()` 统一出口。

## S-30 `check_fallback_needed()` 是死代码，且内部阈值顺序与 S-07 同样错误
- **位置**：`decision_engine.py:458-475`
- **证据**：
  ```python
  if action.type == ActionType.WAIT:
      idle_time = time.time() - self._last_action_time
      if idle_time > 10: ... return True          # 10 先命中并 return
      if idle_time > 13: ... return True          # 永不执行
  ```
  grep 全仓 `_check_fallback_needed` **零调用方**（`_fallback_level` 也只在被它和 `_handle_anomaly` 赋值）。
- **影响**：与 `autonomous_state._check_watchdog`（S-07）形成**两套独立的、都没接好的看门狗**；`FallbackLevel` 的全部语义实际不起作用。维护者极易误以为"已经有两层保活"。
- **严重度**：medium
- **建议修法**：删掉 `_check_fallback_needed()`（或改为 `idle > 13` 先判、并真的被 `update()` 调用），并把看门狗职责收敛到一处（建议收敛到 `AutonomousState`，因为它是唯一真正知道"有没有下发速度"的地方）。

---

# 四、low（低）

| 编号 | 标题 | 位置 | 证据/影响 | 建议修法 |
| --- | --- | --- | --- | --- |
| S-31 | `compute_approach` 被写成 `@property`，一旦调用即崩 | `transport_pipeline.py:133-134` | `[复现]` `TransportPipeline.compute_approach()` → `TypeError: missing 2 required positional arguments`（property 包装；`__dict__` 里是 `property` 对象）。当前零调用方 | 删掉 `@property`，恢复普通方法；并补一条 `tests` 用例（本项目无任何"公开接口可调用性"检查） |
| S-32 | `is_idle` 在类中被定义两次（后者覆盖前者），语义靠巧合一致 | `transport_pipeline.py:177-178` 与 `441-442` | 两处实现相同，当前无行为差异；但对 `start_trip`（S-02）的读者极易误判"哪个才是 yaml/文档所指" | 删除 `:177-178` 的重复定义，保留 `:441` |
| S-33 | `perception/field_detector.py`（77 行）模块级死代码 | `field_detector.py:41-77` | `MockFieldDetector/CVFieldDetector/FieldDetection` 仅被 `perception/__init__.py:34` re-export，无实例化；`DetectedFieldElement.RED_SAFE_ZONE` 与 `detection.HSV_RANGES` 的红/蓝阈值重复实现 | 删除该文件与 `__init__.py` 的 re-export，或把 HSV 阈值统一到 `detection.HSV_RANGES` |
| S-34 | `OdometryLocalizer`/`VisualSLAMLocalizer` 全靠 `_localizer.update()` 积分的假位姿，真机模式用 `MockLocalizer` | `navigation_pipeline.py:70-74,173,199,240,261` | `main.py:133` 传 `use_mock=use_mock` → 真机 `use_mock=False` 时是 `OdometryLocalizer`，但 `autonomous_state.py:220-227` 走 chassis 分支，本地位姿**不参与任何判断** | 删掉 `_localizer.update()` 调用或明确注释"仅 Mock 仿真用"；`navigation_pipeline.reset_pose()` 在真机下应上位机重定位（写入 chassis 的 start_pose） |
| S-35 | `controller` 注入点会调用不存在的方法 | `autonomous_state.py:291-292`、`main.py`（从不传 controller） | `grep "def execute" src/` 无命中 → 一旦有人传 controller 就 `AttributeError`，且被 S-14 的粗 try 吞掉，表现为"车不动" | 删除 `controller` 参数，或补 `ChassisInterface.execute(cmd)` |
| S-36 | `tools/vision_*.py` 与 `detection.py` 的 HSV 阈值不一致，标定结果不能直接用 | `tools/vision_quick.py:24-26`、`tools/vision_calibration.py:29-31`、`perception/detection.py:33-43` | orange：`(8,100,100)-(20,255,255)` vs `(4,205,35)-(24,255,255)`；light_blue：`(85,40,100)-(105,200,255)` vs `(85,50,110)-(108,255,255)`；green：`(40,80,60)` 一致 | 把阈值抽到 `config` 或一个 `hsv_ranges.py`，三个工具与 `detection.py` 共用 |
| S-37 | 相机索引默认值三处不一致 | `tools/vision_test.py`/`vision_quick.py`（默认 0）、`vision_calibration.py:47`（默认 1）、`main.py:75`(1) / `main.py:152`(0) | 现场按不同工具编号接线会得到互相矛盾的结论 | 统一为 0 并用 `CAM_INDEX` 环境变量覆盖 |
| S-38 | 空/无条目模块的边界 | `path_planner.py:242-252` | `_reconstruct_path()` 不包含 `start_g` 与 `goal_g`（只回溯 `came_from`），所以返回路径**恒少一个起点**；`path[0]` 从起点格之后开始 | 在返回前 `path.insert(0, start_cell_center)`；并在 A* 起终点同一格时返回空路径（当前会返回空列表 → `_current_path` 为空 → 走"接近段"分支，行为偶然正确） |
| S-39 | `AnomalyHandler` 的 `imu_data`/`sensor_status` 分支键名与实现不符 | `anomaly_handler.py:88-91,113-114,162-173` | `_sensor_status` 只在 `check()` 收到 `sensor_status` 时更新，而唯一调用方传 `None` → 6 个传感器的"故障感知"永远为 True | 与 S-11 一并修；或删除该分支 |

---

# 五、死代码与"注释与实现不符"清单（可直接删/改）

**完全未被引用的模块/类（除包内 re-export 与自身 `__main__` 自测外零调用）**

| 模块 | 行数 | 证据 |
| --- | --- | --- |
| `robustness/fault_tolerance.py` `stability.py` `logging_system.py` | 665+632+868 ≈ 2165 行（注释行含） | `grep` 19 个公开类名 → 只命中自身 + `robustness/__init__.py`；`tests/` 零命中；`innovation/deploy.py:224-225` 只做**模块**冒烟导入，不实例化。**整个"健壮性/稳定性/日志复盘"板块（"板块 8"）在运行链路中不存在** |
| `perception/field_detector.py` | 77 | 见 S-33 |
| `perception/imu_fusion.py` | 154 | 见 S-12 |
| `decision/opponent_strategy.py` | 592 | `OpponentStrategy` 仅被 `decision/__init__.py:18-19,30` re-export，零实例化。连带 `TargetSelector.set_opponent_target/clear_opponent_targets`（`:65-70`）零调用 → `_opponent_targets` 恒空 → `opponent_factor` 恒 1.0（`:98`）→ **对手对抗能力整体未接线** |
| `simulation/sim_2d.py` `integrated_sim.py` | 1054 | 仅 `simulation/__init__.py` re-export；`main.py` 无引用（作为独立工具可用，但不参与真机链路） |
| `innovation/hot_reloader.py` `debug_dashboard.py` `hardware_profile.py` `model_switcher.py` `deploy.py` | 约 2400 | 同上，只有 `config_loader` 被 `main.py:103` 使用 |
| `anomaly_handler.get_escape_command/is_escaping/keep_moving_fallback/notify_sensor_ok/get_anomaly_count` | — | grep 零调用（S-11） |
| `TransportPipeline.reset()` | — | grep 零调用（S-04 的修复点） |

**注释/文档与实现不符（已核对）**

1. `autonomous_state.py:5-8` 注释「分级看门狗（10s 探索 / 13s 保命 / 不再淘汰）」——实际 10 s 档永不触发、`WATCHDOG_HARD_LIMIT_S=15.0` 从未使用（S-07）。
2. `chassis_serial_protocol.md:266` 「上位机收到 `EVENT,START_BUTTON` 后进入 `AUTONOMOUS`」——实际实现是 `main.py:254` 每 0.5 s 轮询一行（S-15），极易漏事件。
3. `chassis_serial_protocol.md:469` 启动顺序第 4 步「操作员按下一键启动按钮 → 上位机发送一次 START」——实际 `main.py` 在 `on_enter` 里按 `PING→START`，并在 `POST_START_DELAY_MS` 后才起主循环；文档未体现该 1 s 延迟与"先 START 后按按钮"的可能差异（**需真机确认时序**）。
4. `transport_pipeline.py:71-79` 文档示例 `if transport.is_idle(): transport.start_trip(target)` —— 与 S-02 的守卫 bug 直接相关（文档写了正确用法，代码没实现）。
5. `perception_pipeline.py:219` 注释「使用非目标的检测作为对方机器人候选」+ `_extract_opponent()` 用 `largest.contour_area` 取最大未分类检测 —— 实际 `Detection.contour_area` 来自 HSV 轮廓，场地紫边/减速带面积远大于对方机器人，**对手位置基本恒错**；而该数据唯一的"消费者"是 `OpponentTracker`（再往上无人读），所以现状无害，但一旦按 S-11 接线就会误触发。
6. `serial_chassis.py:107-110` 的 docstring 说 `send_ping` 已修好"未校验 PONG"——实现正确（`:113` `wait_for("PONG")`）；但 `system_check.py:196` 用它当"电机检查"，属于**用正确 API 做错误的事**（S-16）。
7. `motion_control.py:112` 注释「角度容差 (~9°)，容忍网格离散误差」——`ANGLE_TOLERANCE_RAD` **在 `is_at_target()`（`:257-263`）里根本没用到**（`ctheta` 解包后弃用）。即"到达"只看位置不看朝向：车头反着也能算到达，套取动作会在错误朝向下执行。**建议升为 medium**：修法是 `is_at_target()` 增加 `abs(self._normalize_angle(target_theta - ctheta)) < ANGLE_TOLERANCE_RAD`，或在 `TransportPipeline.APPROACHING` 里先对齐目标方向再判 `dist < 150`。
8. `config.py:44-51` 的 `Timing.SELF_CHECK_TIMEOUT_S=10`、`SENSOR_CHECK_TIMEOUT_MS=3000`、`MOTOR_CHECK_DURATION_MS=500`（以及能被 YAML 覆盖的这三个值）在 `system_check.py` 中**没有任何超时实现**（grep 确认），自检最长耗时由硬件调用决定（`check_camera` 2 s + IMU 0.5 s + 2×PING 1 s ≈ 3.5 s，串口不在时更长）。
9. `field_elements.py:122-141` 的 ASCII 场地示意图把安全区画在"上方左右"、出发区画在四角，与 `DecisionEngine._get_supply_area_position()`（红区 y=2820）一致；但 `SAFE_ZONE_CENTER_X=1500` 与 `DecisionEngine` 用的 1345/1655 中心来自不同算法，注释「PDF 图7」需按 `docs/scene/pdf_field_page16_hi.png` 复核（**需人工看图纸**）。
10. `storage/`：`world_map.is_stale`（2 s）与 `MAX_LOST_COUNT`（3 s）两套过期语义并存（S-25）；`OpponentTracker.MAX_TRACKING_LOST_S = 3.0`（`opponent_tracker.py:73`）定义了但未使用（实际用 `OpponentState.is_stale` 的 1 s）。

---

# 六、配载规则（≥3 / 危险 / 伤员 / 首次）逐条实测结论

| 规则（来自 `load_manager.py:6-13` 注释与赛题） | 实现位置 | 是否真的被校验 | 备注 |
| --- | --- | --- | --- |
| 首次必须且仅 1 个普通物资 | `load_manager.py:199-203`（`can_load_batch`）、`:156-162`（`can_load`） | ✅ 逻辑正确 | 但可被 `@property` 守卫 bug（S-02）绕过"趟次边界"；冲刺激活时选择器会返回伤员（S-03） |
| ≤3 个/趟 | `load_manager.py:205-206`（`MAX_LOAD=3`） | ✅ | `target_selector.select_targets_for_trip(max_count=3)` 也做了截断 |
| 危险目标绝对禁止 | `load_manager.py:189-190,152-153`（`points=0` 也让 `score_target` 返回 -1 被 `score>0` 过滤，`target_selector.py:83-84,153`） | ✅ 双保险 | `get_dangerous()` 无 `_selectable` 过滤（`world_map.py:175-176`），但两个入口都被拦住 |
| 伤员必须单独转运 | `load_manager.py:191-195,173-179`（`can_load_batch` + `can_load`） | ✅ | 但**伤员永远优先于"先装普通+核心"**（`target_selector.py:209-214`），实际策略会变成"一趟只搬一个伤员"，与"≤3 混合装载"的得分效率相悖（赛题策略问题，非 bug，需与 req-auditor 结论对齐） |
| 禁止抓取救援目标（对方机器人） | — | ❌ **无实现** | 全场无"救援目标"（裁判机器人/己方已投放目标）识别逻辑；`opponent_tracker` 的输出无人消费（见死代码清单） |
| 目标不得放在机器人上 | `load_manager.py` | ❌ 部分 | 只有"落点在不在安全区"的判定，且用错坐标（S-19） |

---

# 九、附：修复优先级建议（给 fixer）

| 顺序 | 事项 | 关联 |
| --- | --- | --- |
| 1 | 终场/清空停车（S-01）+ 急停发 `ESTOP`（S-08） | 安全与规则，改动小、收益最大 |
| 2 | 多目标逐个套取（**S-40**）+ `start_trip` 守卫 `()`（S-02）+ VIOLATION 出口（S-04） | 三个 blocker 都在 `transport_pipeline.py`，决定"一趟能拿几分" |
| 3 | 感知补 theta 旋转（S-05） | 定位精度，影响全部抓取 |
| 4 | 看门狗修复（S-07）+ 异常检测接线与键名（S-11/S-29）+ 强制分离响应（S-41） | 保活与对抗链路 |
| 5 | 相机索引/自检/帧新鲜度（S-10/S-26）+ 套取机构自检（S-16） | 上机第一天必须能定位"哪个模块坏了" |
| 6 | 禁区区动态清除（S-13）+ LocalPlanner 零速兜底（S-18）+ 禁区倒车（S-17） | 导航安全 |
| 7 | YAML 接线（S-06 = C-01）+ 依赖清单/dialout/systemd（P-01~P-03） | 决赛现场调参与部署能力 |
| 8 | 死代码清理（第五节清单）+ 文档统一（DC-01~DC-04） | 降低后续误改风险 |

---

# 十、软件负责人视角：工程可交付性审计

> 本节按"这套软件作为交付物是否合格"评审，覆盖 可维护性 / 可调试性 / 可配置性 / 可部署性 / 性能预算 / 文档-代码一致性 六个维度。所有结论都是**纯软件判定**（无硬件参与）。

## 10.0 每条问题"会让机器人损失什么"总表（快速索引）

| 编号 | 严重度 | 在比赛里会损失什么 |
| --- | --- | --- |
| S-01 | blocker | 终场后仍行驶：可能撞紫边/对方安全区（违规+扣分），或触发"无人干预停不下来"的现场事故 |
| S-02 | blocker | 首次规则被绕过：装载台账错乱 → 可能"本轮结束、成绩无效"（丢整轮） |
| S-03 | blocker | 冲刺期选到伤员当首趟 → 违规判无效（丢整轮） |
| S-04 | blocker | 一次规则拒绝后永久卡死 → 后续一分不得 |
| S-05 | blocker | 目标坐标随车头方向整体错位 → 反复扑空，几乎抓不到东西，**丢绝大部分分** |
| S-40 | blocker | "一趟装多个"不成立：每趟只带 1 个、计分按 3 个算 → 有效得分只有设计值 1/3 |
| S-06 | high | 现场改 YAML 不生效 → 调不出参数，且 `MATCH_DURATION_S` 与官方赛时可能不符（提前/滞后收工） |
| S-07 | high | 卡住后保命模式输出 0 速度 → 原地不动到终场（丢剩余时间所有分） |
| S-08 | high | 急停不通知底盘 → 最多前冲 ~0.8 s（约 0.4 m）：撞目标、撞围栏、丢目标 |
| S-09 | high | 自检静默失败 → 进不了 AUTONOMOUS（0 分），且现象难定位 |
| S-10 | high | 相机索引不一致 → 自检绿灯但取不到帧；掉线 3 s 后全场目标被删并换 id → 反复扑空 |
| S-11 | high | 卡死/失控/脱困全不触发 → 陷住后无法自救（丢剩余时间所有分） |
| S-12 | high | 无 IMU 航向融合 → 被推撞一次后 `theta` 永久漂移，全局定位报废（后续全废） |
| S-13 | high | 禁区区被抹掉 → A* 规划进对方安全区（违规 + 每轮 -5 分） |
| S-14 | high | 单帧异常整帧跳过 `VEL` → 下位机看门狗 0.8 s 内停车，"一瘸一拐"，动作被截断 |
| S-15 | high | 一键启动事件大概率漏收 → 无法启动（0 分），只能靠笔记本按键 |
| S-16 | high | 套取舵机无自检 + 不校验 ACK → 空手跑完全程却自认成功（整轮白跑） |
| S-17 | medium | 禁区违规时固定倒车且不看后方 → 撞墙/撞围栏，损失目标与时间 |
| S-18 | medium | 避障采样全碰撞时输出 0 速度 → 僵在原地等超时 |
| S-19 | medium | 投放判定过松 → "物资放进伤员区"这类 -10 分/个 的错误永远不会被发现 |
| S-20 | medium | 半批装载失败不回滚 + 目标永久 `BEING_TRANSPORTED` → 该目标整场再也不会被抓（丢目标） |
| S-21 | medium | 场地中央有目标就重置当前目标 → 与转运管线脱节（跑空趟） |
| S-22 | medium | 3 s 无帧即删目标 → 短暂遮挡后目标换新 id，重复扑空 |
| S-23 | medium | 串口阻塞 20 ms/帧 + A* 26 ms → 位姿只有 20 Hz 刷新、偶发 40 ms 长帧 |
| S-24 | medium | 异常退出路径不完整 → 线程/舵机/串口状态残留，二次启动异常 |
| S-41 | high | 强制分离无响应 → 分离后朝旧坐标跑（撞场/违规），10 s 接触规则形同未实现 |

---

## 10.1 可维护性

### M-01【中】同一逻辑两处实现（"过期目标删除"与"探索/保命"各有一套）
- **位置**：`src/rescue_robot/perception/world_map.py:94-97`（`MAX_LOST_COUNT=150`）vs `src/rescue_robot/decision/decision_engine.py:458-475`（`_check_fallback_needed` 的 10 s/13 s）；`src/rescue_robot/states/autonomous_state.py:343-362`（另一套 10 s/13 s/15 s）
- **证据**：两处 10/13 s 阈值、两处"无动作"定义；`_check_fallback_needed` 零调用方；`WATCHDOG_HARD_LIMIT_S` 零引用
- **影响**：改一处不生效；新人会以为"有两层保活"。**比赛中表现为：真卡住时没人知道该信哪套**
- **建议修法**：看门狗收敛到 `AutonomousState` 一处（它是唯一知道"有没有真的下发速度"的地方）；删除 `_check_fallback_needed` 与 `FallbackLevel`

### M-02【中】上帝对象/超长函数：`TransportPipeline.update()` 单函数 5 个状态机分支共 150 行
- **位置**：`transport_pipeline.py:279-431`（`update`）、`decision_engine.py:178-247`（`update`）
- **证据**：`update()` 内嵌 `APPROACHING/CAPTURING/RETREAT/TRANSPORTING/PLACING` 五段 + 放置推入 + 逐步上调，`return` 分散在 4 个位置；`is_idle/is_complete/phase` 状态查询散落全类
- **影响**：S-40（多目标套取缺失）、S-02（守卫写错）都是在这个函数里"看不出来"的缺陷
- **建议修法**：拆成 `_step_approaching/_step_capturing/_step_transporting/_step_placing` 四个方法，每个方法单一状态转移并在返回前统一 `return self._get_status()`

### M-03【低】命名不达意/易误读
- **位置**：`autonomous_state.py:230-235`（`cur_id`/`_prev_current_id` 表"趟次"但名字像"当前 id"）；`autonomous_state.py:366-375`（`_lock_external_inputs` 只打日志）；`transport_pipeline.py:134`（`compute_approach` 是 property 不是方法）
- **影响**：后续维护者极易把"趟次标志"当成"目标 id 同步"，改出 S-02 那一类 bug
- **建议修法**：重命名 `_trip_target_id`；`_lock_external_inputs` 改为 `_verify_inputs_locked()` 并返回 bool

### M-04【中】模块边界与依赖方向不清晰（4 处反向/隐式依赖）
- **位置**：`decision_engine.py:318`（函数内 `from rescue_robot.perception.target_types import ...` 绝对导入，同模块顶部已 import `TargetType`）；`decision_engine.py:37-43`（`class FallbackLevel` 夹在两组 import 之间）；`main.py:179-180`（靠私有属性名 `_chassis` 注入）；`states/autonomous_state.py:230`（直接读 `self._decision._current_target` **私有字段**）
- **证据**：`autonomous_state.py:230` `cur = self._decision._current_target`；`main.py:180` `hw_checker._chassis = chassis`
- **影响**：任何重构（改私有字段名）都会**静默**破坏主链路，且没有单元测试覆盖这些耦合点
- **建议修法**：`DecisionEngine` 暴露只读属性 `current_target`；`RealHardwareChecker.set_chassis()` 公开方法；`TargetStatus` 统一从顶部 import

### M-05【中】死代码占比过高，掩盖真实缺陷（约 6000 行 ≈ 全仓 1/3）
- **位置**：见第五节死代码清单（`robustness/*` 2165 行、`perception/field_detector.py` 77、`perception/imu_fusion.py` 154、`decision/opponent_strategy.py` 592、`simulation/*` 1054、`innovation/*` 除 config_loader 外约 2400）
- **影响**：任何"全仓 grep"式排查都会被噪音淹没；新人无法判断"哪个模块才是真在跑的"（真在跑的只有约 4 千行）
- **建议修法**：把不参与主链路的模块移到 `legacy/` 或 `tools/`，并在 `README` 用一张"运行链路图"明确标注**只有这些文件在真机上跑**

### M-06【低】注释与实现不一致（19 处，详见第五节）
- **位置**：`autonomous_state.py:5-8`、`motion_control.py:112`、`perception_pipeline.py:219`、`serial_chassis.py:14` vs `chassis_serial_protocol.md:151-153` 等
- **影响**：交接时按注释理解代码会得出错误结论（例如以为"到达判定含朝向" —— 实际 `ANGLE_TOLERANCE_RAD` 根本没参与 `is_at_target`，见第五节第 7 条）
- **建议修法**：把注释当契约，逐条修正；关键阈值在注释里写明"谁在用"

---

## 10.2 可调试性

### D-01【高】"静默降级"过多，故障时表现为"什么都没发生"
- **位置**：`main.py:108-109`（配置加载失败只 warning）→ `perception_pipeline.py:173-175`（无摄像头**静默降级为 Mock 感知**）→ `autonomous_state.py:113-114`（无摄像头时"按接住处理"）→ `transport_pipeline.py:314-332`（连续 5 次确认失败**自动关闭视觉确认**）→ `system_check.py:300-302`（读不到电压就"视为未知，跳过"）
- **证据**：
  ```python
  # main.py:173-175
  if camera is None and not use_mock:
      logger.warning("无可用摄像头，感知降级为 Mock（视觉不可用）")
      perception = PerceptionPipeline(use_mock=True, ...)   # ← 机器人会在"假装看到目标"的世界上跑
  ```
- **影响（损失什么）**：相机没插时机器人**不会报错停止**，而是用随机生成的 20 个假目标继续跑 —— 现场表现是"车在动但完全抓不到东西"，而日志只有一行 warning。这是最难现场定位的一类故障。
- **建议修法**：摄像头不可用属于**致命降级**：置 `sm.state=ERROR` 或至少进入"原地等待 + LED 红闪 + 每分钟打印一次"的显式降级模式；`Mock 感知` 只允许在 `RUN_MODE=mock` 下出现（加断言 `if camera is None and not use_mock: raise/transition_error`）

### D-02【中】关键状态可观测性不足（无统一状态快照）
- **位置**：`autonomous_state.py:379-390`（`get_loop_stats` 定义了但**没有任何调用方**）；`main.py:258-261`（AUTONOMOUS 下只 `logger.debug`）
- **证据**：
  ```
  grep -rn "get_loop_stats" src/ → 只有定义处
  # main.py:258-261
  if current_state == RobotState.AUTONOMOUS:
      status = sm.get_status()
      logger.debug(f"自主运行中: {status}")     # DEBUG 级别，默认 INFO 不输出
  ```
- **影响**：比赛时无法从日志判断"当前阶段/当前目标 id/到安全区还有多远"，出问题只能靠猜
- **建议修法**：`main.py` 的等待循环改为每 2 s 用 **INFO** 打印一行结构化快照（`state / strategy / nav_state / transport_phase / 当前目标 id+坐标 / 位姿 / 剩余时间 / 串口 rx-tx 计数 / 加载数`）；数据源现成（`autonomous_state.get_loop_stats()`、`navigation.get_progress()`、`transport._get_status()`、`serial_chassis.get_stats()`）

### D-03【中】错误信息不可操作（只说"失败"，不说"下一步做什么"）
- **位置**：`serial_chassis.py:142`（`"串口未打开，无法发送"`）、`system_check.py:326`（`"检查未通过"`）、`agent`… `transport_pipeline.py:257`（`"无法开始转运：当前阶段=…"`）
- **影响**：`main.py:145` 已经写了正确的排查提示（`请检查接线 / dialout 权限`），但 `SerialChassis` 内部各处没有 —— 自检失败时队员拿到的是"检查未通过"，不知道该插线还是加权限
- **建议修法**：`CheckItem.message` 补可操作建议（例如 `"IMU 检查未通过：串口未打开 → 检查 CHASSIS_PORT / 用户是否在 dialout 组"`）

### D-04【中】串口收发统计已实现但没有任何出口
- **位置**：`serial_chassis.py:331-340`（`get_stats`）、`:67-68`（`_bytes_tx/_frames_rx`）
- **证据**：`get_stats()` 零调用方；`_frames_rx` 只在解析成功时 +1
- **影响**：无法区分"下位机没发数据"和"上位机解析全失败"（协议一致性排查的第一问）
- **建议修法**：`D-02` 的状态行里带上 `bytes_tx / frames_rx`；并增加 `parse_fail_count`（解析失败行数）与 `last_raw_line`（最近一行原文）

### D-05【低】日志量在异常路径可能爆量
- **位置**：`autonomous_state.py:206`（每帧 `logger.error(..., exc_info=True)`，50 Hz）→ 注释里的 `D-02` 未实现，所以异常会**每帧刷一屏堆栈**
- **影响**：异常时日志暴涨，反而看不到第一条（真正的原因）
- **建议修法**：同一异常只在首次/每 N 次打印完整堆栈（`logging` 的 `FatalCounter` 或自己计数），其余打一行摘要

---

## 10.3 可配置性（硬需求：决赛创新实践环节现场改参数，不能重编译）

### C-01【高】现场"改 YAML 即生效"的能力基本不存在（这是会被扣分的项）
- **位置**：`src/rescue_robot/config.py:126-148`（唯一应用函数，只覆盖 `timing`/`thresholds`）、`src/rescue_robot/main.py:102-109`（唯一调用点）、`config/robot.default.yaml`
- **证据**：见 S-06 的对照表（9 个 YAML 段落里 7 段零消费者）
- **`[复现]` 能生效的**（3 项）：
  ```
  Timing.BUTTON_DEBOUNCE_MS = 999   ✓  apply_robot_config 后 timig.X 与 Timing.X 同步变化
  thresholds.CAMERA_MIN_FPS = 77    ✓
  （camera 几何、detection、motors、match、fallback、strategy_weights、communication、logging）
  ```
- ****"目前无法现场改的关键参数"清单**（必须重编译/改代码才能调）：

  | 想改什么 | 现在写在哪 | 现场能否改 |
  | --- | --- | --- |
  | 控制器 PID、最大线/角速度、轮距 | `motion_control.py:108-126` 类常量 | ❌ |
  | 比赛时长、时间紧迫阈值、导航/夹取/运送超时 | `decision_engine.py:98-104` | ❌ |
  | 看门狗 10/13/15 s、卡死 5 s/30 mm | `anomaly_handler.py:70-76`、`autonomous_state.py:51-53` | ❌ |
  | 目标删除帧数、确认帧数、关联距离 | `world_map.py:85-97` | ❌ |
  | 到达判定容差（位置 40 mm/角度 9°）、投放到达容差 80 mm | `motion_control.py:112-113`、`transport_pipeline.py:369` | ❌ |
  | 套取 ROI、相机高度/倾角/FOV | `config.py:99-109`（模块常量，无 YAML 段落） | ⚠️ 改代码（`Camera` 段不在 yaml 里） |
  | 视觉 HSV 阈值 | `detection.py:33-43`（且与 `tools/*` 不一致） | ❌ |
  | 场地尺寸/安全区几何 | `field_elements.py:94-115` | ❌ |
  | 串口设备/波特率 | 环境变量 `CHASSIS_PORT`（`main.py:142`） | ✅ |
  | 摄像头索引 | 环境变量 `CAM_INDEX`（但两处默认值不同，见 S-10） | ⚠️ |
  | 本队颜色 | 环境变量 `TEAM_COLOR`（`main.py:128-130`），而 `config/field.default.yaml` 的 `field.my_color` **从不被读取** | ⚠️ 只有 env 生效 |
- **影响（损失什么）**：创新实践环节要求"现场编程/改参数"，评委看的是"改配置不改代码"。目前**除了 3 个时序参数和一个串口路径，几乎什么都要改 `.py`** —— 这一项直接对应评分表里的创新实践分；同时 `MATCH_DURATION_S=180` 若与官方赛时不同，会提前/滞后收工（丢分或违规）。
- **建议修法**：① 把上表所有 ❌ 项接进 `apply_robot_config()`（或改为给各管线注入 cfg）；② `config/robot.default.yaml` 增加 `camera:` 段（height/tilt/fov/res/sleeve_roi/sleeve_confirm）并接到 `config.Camera`；③ 在 `main.py` 启动日志里**打印最终生效参数表**（便于现场确认"我改的生效了"）；④ 给 `config_loader.__main__` 增加"apply 后断言关键参数已被改到"的用例，防止再次脱线。

### C-02【中】环境变量覆盖能力已实现但没人用，且会静默吞掉未知键
- **位置**：`config_loader.py:180-196`（`merge_with_env`，`RESCUE_*` 双下划线语法）、`:287-296`（`_set_nested` 会无脑创建中间字典）
- **证据**：`merge_with_env`、`merge_configs`、`fill_defaults` 在 `src/` 中**零调用**（只有 `config_loader.__main__` 自测）
- **影响**：现场"用环境变量临时改一个值"的路子也不通（没人调用）；即便接上，拼错 key 会静默新增一个无人读的键
- **建议修法**：在 `main.py` 加载 YAML 后接 `merge_with_env`；`_set_nested` 增加"路径必须已存在"校验（否则报错列出可用键）

### C-03【中】`config/field.default.yaml` 与 `config/strategy/*` 是否被读取未定义
- **位置**：`config/field.default.yaml`（存在）、`config/strategy/`（目录存在）、`config_loader.py:542-560`（`FieldConfig.from_yaml`）
- **证据**：`grep -rn "field.default.yaml\|config/strategy" src/` → 只有 `main.py:105` 读 `config/robot.default.yaml`
- **影响**：场地/策略配置文件看起来是"现场可改项"，实际**没有人读**，改了 0 效果（比"没有这个文件"更糟：给人错觉）
- **建议修法**：要么在 `main.py` 读入并应用到 `FieldLayout`/`TargetSelector`，要么删掉并在文档里写明"场地几何目前硬编码在 `field_elements.py`"

---

## 10.4 可部署性（目标机 RDK X5 / Ubuntu）

### P-01【高】没有依赖清单，环境不可复现
- **位置**：仓库根 **不存在** `requirements.txt` / `setup.py` / `pyproject.toml`（已 `ls` 确认）
- **证据**：代码实际需要 `pyserial`(串口)、`opencv-python`+`numpy`(视觉)、`PyYAML`(配置)、可选 `psutil`/`websockets`/`RPi.GPIO`/`colorama`/`tkinter`（分别见 `serial_chassis.py:75`、`camera_reader.py:64`、`config_loader.py:24`、`robustness/stability.py`、`comm_server.py:176`、`indicator.py:87`、`deploy.py:34`、`debug_dashboard.py`）
- **影响**：换一台 RDK 或重装系统后无法一次装齐；`main.py` 对缺失依赖是**静默降级**（D-01），现场表现为"能启动但视觉/串口全无"
- **建议修法**：补 `requirements.txt`（`pyserial`、`opencv-python`、`numpy`、`PyYAML` 为必需；其余标 optional 并加注释）；`scripts/deploy.sh` 里增加 `pip install -r requirements.txt` 步骤

### P-02【高】串口权限：当前用户不在 `dialout` 组，`/dev/ttyS*` 会直接抛权限错
- **位置**：`main.py:142-145`（`CHASSIS_PORT`，默认 `/dev/ttyUSB0`）、`serial_chassis.py:80-83`
- **证据**（本机实测）：
  ```
  crw-rw---- 1 root dialout 4, 64 /dev/ttyS0   （/dev/ttyS0..S7 均为 root:dialout 660）
  id -nG → ony_uang adm cdrom sudo dip plugdev      ← 没有 dialout
  ```
  `SerialChassis.open()` 会抛 `PermissionError: [Errno 13]` → 被 `except Exception` 捕获 → 日志 `打开串口失败` → `is_open=False` → **自检 IMU/电机失败 → BOOT 直接进 ERROR**（这也是为什么需要 P-03 的 RUNBOOK 步骤）
- **影响（损失什么）**：部署到 RDK 后**无法启动**（0 分），且现象是"自检失败"，容易被误判为硬件坏
- **建议修法**：`scripts/deploy.sh` 增加 `sudo usermod -aG dialout $USER`（并提示重新登录）；`main.py:145` 的错误信息已含 dialout 提示，建议同时**在启动时预检**：`if not os.access(port, os.R_OK|os.W_OK): logger.error("串口无权限：把用户加入 dialout 组后重新登录")` 并在 `RUN_MODE=real` 下直接 `return 1`（快速失败优于静默降级）

### P-03【中】一键启动/异常自恢复/开机自启不完整
- **位置**：`scripts/deploy.sh:103-104`（`systemctl restart rescue-robot` 之后 fallback 到"手动启动"）、`docs/RUNBOOK.md:150`（已记录"该 systemd 服务在本仓库不存在"）
- **证据**：仓库内 **无任何 `*.service`**（`find -name "*.service"` 为空）
- **影响**：比赛现场只能靠人手敲 `RUN_MODE=real python3 -m rescue_robot.main`；进程崩溃/被 Ctrl+C 后不会自恢复（`main.py` 无 supervisor、无重启逻辑）
- **建议修法**：提交一个 `scripts/rescue-robot.service`（`ExecStart=/usr/bin/python3 -m rescue_robot.main`、`Environment=RUN_MODE=real`、`Restart=on-failure`、`SupplementaryGroups=dialout`、`WorkingDirectory=<repo>`），并在 `deploy.sh` 里 `install` 该 unit

### P-04【低】部署脚本的默认目标是树莓派，与 RDK X5 不一致
- **位置**：`scripts/deploy.sh:11-13`（`TARGET=raspberrypi.local`、`TARGET_USER=pi`、`/home/pi/rescue-robot`）；`scripts/deploy.bat` 同类
- **影响**：直接跑会在错误的机器上找目录；需要每次手动传 `--target`
- **建议修法**：默认值改为 RDK 的 `TARGET=/home/ony_uang/...` 或强制要求 `--target`（无参数即报错并打印用法）

### P-05【中】没有"启动到就绪"的端到端 smoke（无法快速回答"哪个模块坏了"）
- **位置**：`system_check.py:225-262`（自检只覆盖 摄像头/IMU/温度/2×电机/电压 共 6 项，且电机= PING、电压=跳过、温度=恒真，见 S-16）
- **影响**：自检全绿 ≠ 能跑；套取舵机、下位机里程计、`VEL` 是否真被接受、串口双向都**没有覆盖**
- **建议修法**：把自检扩成"分级连通性测试"（与 test-author 的分模块测试程序合流）：`PING→PONG`、`START→ACK,START`、`VEL,100,0` 后读 3 帧 ODOM 判断 `encL/encR` 是否增长、`SERVO,RAISE/LOWER` 各等 ACK、`SERVO,ANGLE,35→ACK`；每项独立 PASS/FAIL 并打印可操作建议

---

## 10.5 性能预算（50 Hz → 每帧 20 ms）

**本机实测（WSL，Python 3.10，未做优化）**：

| 环节 | 实测/估算 | 说明 |
| --- | --- | --- |
| `CVDetector.detect`（HSV×4 颜色 + 形态学 + 轮廓） | **2.4 ms**（640×480、20 个色块）；1.7 ms（纯噪声，无目标） | 实测；与 `_colors_to_detect` 颜色数近似线性（初赛/决赛都是 4 色） |
| `PerceptionPipeline.update`（含检测+分类+世界地图+对方跟踪） | **2.1 ms** | 实测 |
| `LocalPlanner.plan`（50 采样 × 2 × 5 步） | **0.6 ms** | 实测 |
| `AStarPlanner.plan`（200,200 → 2800,2800，60×60 网格） | **25.9 ms** | 实测 —— **单次就超过 20 ms 预算** |
| `SerialChassis.read_pose()`（`readline(timeout=0.02)`） | **0～20 ms**（视行到达时机） | 实测推导；ODOM 20 Hz + IMU 50 Hz 混流时约 5/6 的帧拿到非 ODOM 行 → 白等 20 ms 且位姿不更新 |
| `TransportPipeline.update`（含套取时 `time.sleep` 1.3 s） | 常态 ≈0；套取帧 **1300 ms** | `lower_with_retry` 最坏 4×(0.3+0.5) = 3.2 s；`place_ramp` 5×0.1 s |
| 决策 + 导航 + 状态机（纯 Python 算术） | 通常 < 1 ms | 估算 |
| 串口发送 `VEL`（`write` + `flush`） | < 1 ms | 估算 |

**结论**：
1. **稳态帧 ≈ 2–5 ms + 串口 0–20 ms**，在 20 ms 预算内**勉强可行**，但**位姿刷新率实际只有 20 Hz**（S-23），且每帧的串口阻塞把余量吃光。
2. **A* 重规划帧 = 26 ms + 串口 20 ms ≈ 46 ms** → 该帧必超预算（一帧掉两次），下位机速度环会出现"保持最后一帧"的抖动。触发条件：`navigation_pipeline.py:204-208` 的 `need_replan`（`state==PLANNING/BLOCKED` 或每 30 帧且路径短于 5 点时）。注意"目标附近路径被 prune 到 <5 点"是**常态**，所以 30 帧（0.6 s）一次的 46 ms 长帧会反复出现。
3. **套取帧 1.3 s 完全不发 `VEL`**（`time.sleep` 占满）→ 依赖下位机看门狗平滑停车；代码已用 `_stop_chassis()`（`autonomous_state.py:323-339`）补偿（这一步做对了），但**放慢的 1.3 s 内决策/导航全停**，如果目标在斜坡上会滑。
4. `PerceptionPipeline.update` 里 `frame is None` 时 `CVDetector.detect` 返回 `[]`（`detection.py:189-193`）——这是对的（避免异常），但会走 S-22 的目标删除路径。

**建议**：A* 换 `heapq` + 把重规划下沉到独立线程（或限制为 `state==PLANNING` 时同步、`BLOCKED` 时 1 Hz 重试）；`SerialChassis` 改后台读线程；给 `_run_once` 各段加 `time.perf_counter()` 埋点并在 >20 ms 时打 warning（这样"哪一帧为什么慢"现场可查）。

---

## 10.6 文档-代码一致性 + 交接性

### DC-01【高】接口契约不是单点可查：同一件事在 3–4 个地方各写一遍且互相矛盾
- **位置**：`chassis_serial_protocol.md`（v1.1，最权威）、`src/rescue_robot/hardware/serial_chassis.py:7-27`（文件头 docstring）、`MEMORY.md:290-314`、`README.md`、`HARDWARE_DEPENDENCIES.md`
- **已确认的矛盾（逐条）**：
  1. **舵机角度语义**：`serial_chassis.py:14` 写 `SERVO,ANGLE,deg（0~180）`，而 `chassis_serial_protocol.md:151-153` 明确"实际行程 0°~70°（0°=下压套住 / 70°=抬起释放）"。代码里 `sleeve_lift.py:396,431-432` 用的是 0→70（与协议一致），**只有 serial_chassis 的注释是错的** → 交接时极易按注释发 90°+ 导致机构顶死。
  2. **设备的实际路径**：`serial_chassis.py:25-27` 写"电脑调试 /dev/ttyUSB0；RDK 部署 /dev/ttyS0（待确认具体 UART）"，而本机实测 `/dev/ttyS0..S7` 均存在（S1 也在），`main.py:142` 默认却是 `/dev/ttyUSB0` → 现场要记得设 `CHASSIS_PORT`。
  3. **看门狗与"15 秒淘汰"**：README/`MEMORY.md` 的规则体系里"机器人停止运行 15 秒 → 本轮结束"是**官方判定条件**，而 `MEMORY.md:295` 写"15 秒不再导致淘汰，改为四层降级保活链"，`autonomous_state.py:351` 的注释也这么写。**代码是故意不淘汰的**（`_check_watchdog` 不调 `emergency_stop`）。这是"用软件规避官方淘汰规则"的设计选择，但文档与官方赛规对不上，**评委/裁判视角看是"该停不停"** —— 需要软件负责人明确决策并统一文档（至少不要在同仓里同时存在两种说法）。
  4. **一键启动时序**：`chassis_serial_protocol.md:463-473` 的顺序是"2.发 PING → 3.操作员按按钮 → 4.发 START"，而 `serial_chassis.py:151-167 start_match()` 是"PING→PONG→START→ACK"**全自动**，`main.py:254` 之后才轮询按钮事件 → **协议文档与实际时序不一致**（真机上"先 START 后按按钮"是否被固件接受，见第八节）。
- **影响（损失什么）**：交接给下一位队员时，他会按 docstring 实现出错误的舵机角度/错误时序，造成机构损坏或收不到事件，且**很难发现是文档错而不是代码错**
- **建议修法**：确立**唯一权威契约**：把上位机侧的所有协议常量（命令字、字段数、单位、舵机角度范围、事件名、超时）抽到一个 `hardware/protocol_spec.py`，`serial_chassis.py`/`sleeve_lift.py`/`system_check.py` 全部引用它；`chassis_serial_protocol.md` 作为唯一人类可读文档，其余文件只留指针（"详见 chassis_serial_protocol.md 第 N 节"）；`serial_chassis.py:14` 的 0~180 注释立即改正

### DC-02【中】`README.md` 的功能清单与实现不符（宣称能力未实现）
- **位置**：`README.md:43-60`（"完全自主运行，应具备以下功能"）、`MEMORY.md:256,290-314`（策略与保活链）
- **证据**：README 列出的"自主运行必须功能"里，`对手对抗`（`opponent_strategy.py` 零实例化）、`IMU 融合/失控检测`（`imu_fusion` 零引用、`AnomalyHandler` 恒 NONE）、`15秒保活四层链`（S-07 只剩一层且失效）在代码里**都不成立**
- **影响**：评委若按 README 提问"异常如何恢复/如何对抗"，现场无法演示（答"代码里有"会被追问）
- **建议修法**：README 的功能清单逐条标注**实现状态**（已实现/部分实现/Mock 占位/未实现），并把"未实现"的从"应具备功能"挪到"后续工作"

### DC-03【中】`MEMORY.md` 与实现偏差较大（它被当作"权威记忆"使用）
- **位置**：`MEMORY.md:91`（15 秒防停保活分级看门狗）、`:290-297`（四层降级保活链 P0）
- **证据**：`_check_fallback_needed`（第二层）零调用、`WATCHDOG_HARD_LIMIT_S` 零引用、`explore()` 永不触发、`survival_circle()` 输出 0 向量（S-07）→ 四层里**只有第 0 层真的在跑**
- **影响**：队员按 MEMORY 判断"已经有保活链"，于是不再排查卡死问题 → 现场卡死时无人会去查
- **建议修法**：MEMORY 的每条 P0 功能后面加"实现位置 + 验证命令"；未验证的标 `[未接线]`

### DC-04【低】文档入口分散，没有"5 分钟上手"索引
- **位置**：根目录 `README.md` / `MEMORY.md` / `chassis_serial_protocol.md` / `HARDWARE_DEPENDENCIES.md`，`docs/RUNBOOK.md`（55 KB）、`docs/audit/*`（本报告 + COMPLIANCE_AUDIT）
- **影响**：接手者需要读 5 个文件才能拼出运行链路
- **建议修法**：`README.md` 顶部加一张"运行链路图（哪些 .py 真的在真机上跑）"+"启动三步（装依赖 → dialout → RUN_MODE=real）"+"文档导航表"

---

# 八、需真机确认（仅 6 条，其余结论均已在纯软件层面判定）

> 本次硬件全关，以下 6 条是**实在无法靠读代码/逻辑仿真判定**的，其余 40+ 条问题都已给出确定结论。

1. **下位机固件实现**（本机无 `Tang29Zx/rescue_f103c8`，`/tmp/rescue_f103c8` 不存在，无法读固件）：
   ① `SERVO,RAISE/LOWER/HOLD` 的实际脉宽是否与 `servo.h` 的 0°=下压 / 70°=抬起 一致（`chassis_serial_protocol.md:151-153`）；
   ② 速度看门狗 300 ms 保持 / 800 ms 平滑停车 + "连续 3 个合法 `VEL` 恢复"是否真的实现（协议第 6 节）；
   ③ `EVENT,START_BUTTON` 是**单次上报**还是周期上报 —— 这直接决定 S-15 的严重度（若单次则必然漏收，若周期上报则只是延迟）。
2. **串口实际占空比**：`readline(timeout=0.02)` 与 ODOM 20 Hz / IMU 50 Hz 混流下，`read_pose()` 丢弃 IMU 帧的比例、以及实际位姿刷新率（S-23）。*建议先把 `SerialChassis` 改成后台读线程再测，测出的数才有意义。*
3. **相机几何标定**：`Camera.HEIGHT_MM=210 / TILT_DEG=30 / FOV_DEG=77 / SLEEVE_ROI=(0.32,0.55,0.68,0.98)` 全是待标定值（`config.py:99-109`）。S-05 补上朝向旋转后，必须用 500/1000/1500 mm 三个已知距离重标 `TILT_DEG`。
4. **视觉检出率/误检率**：`CVDetector` 只有 HSV + 轮廓近似，黑色目标（`V<60`，`detection.py:39`）在场地阴影/紫边/深色地面下最易出问题，`_classify_shape` 的顶点数判据（`detection.py:54-61`）在 640×480、30 px 目标上不稳。可用 `docs/vision/vision_photos/图1..8.png` 跑离线检测统计——这是"能不能抓对目标"的头号不确定性（软件层面已知的只有 S-05 坐标错位）。
5. **场地几何外部一致性**：`field_elements.py` 的 3000×3000、安全区 600×300、减速带 24 根（`SPEED_BUMP_COUNT=3` × 4 出发区 × 横纵）× 需与官方图纸 `docs/scene/pdf_field_page16_hi.png` 对齐（本报告只做了代码内部一致性检查）；建议直接采用 req-auditor 在 `docs/audit/COMPLIANCE_AUDIT.md` 给出的结论。
6. **机构行程能否真的推进斜坡**：`place_ramp()` 0→70° 分 4 步、每步 0.1 s（`sleeve_lift.py:429-434`）+ 底盘"前推 100 mm"（`transport_pipeline.py:106,376-388`）的时空配合没有任何实机验证，可能推不进紫边斜坡或把目标顶飞。

> 已由**纯软件判定、无需真机**的高风险项，请勿再称作"待验证"：S-01（不停车）、S-02（守卫失效）、S-40（多目标装载不成立）、S-04（VIOLATION 卡死）、S-05（缺朝向旋转）、S-11/S-29（异常子系统键名不匹配）、S-13（禁区被抹）、S-18（LocalPlanner 零速）、S-07（保命 0 向量）、S-06/C-01（YAML 不接线）、P-01/P-02（无依赖清单、无 dialout）—— 都有可重跑证据（`docs/audit/repro/`）。
