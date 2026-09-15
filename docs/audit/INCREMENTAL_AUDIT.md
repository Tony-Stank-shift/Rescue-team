# 增量审计报告（t13）——本轮修复改动的新缺陷对抗性复核

- **复核对象**：`git diff 0e8a5ae..HEAD`（3 个提交：`e4cd2cb` 修复 / `ff0a29b` 自检程序 / `40a8c4f` 文档）+ 当前工作区未提交改动
  - 源码侧：16 个文件、**+1104 / −236 行**；另新增 `tools/hw_selftest/**`（13 模块分部自检）与 `tools/fix_verifiers/**`
  - 参照基线：`f2e4795`（我 t2 全项目审计所依据的版本）
- **方法**：独立对抗性复核——不采信修复说明，直接读 diff，并对**新逻辑**跑实验（全部无硬件：`PYTHONPATH=src python3 ...`；能纯软件判定的一律给确定结论）
- **纪律**：未修改 `src/`、`tools/`、`config/`、`tests/` 任何文件；本报告是唯一新增产物。
- 立场：找"修 A 引入 B"与"看着修好、其实没闭环"的地方。

---

## 结论摘要（对应 6 个指定重点）

| 重点 | 结论 | 关键证据 |
| --- | --- | --- |
| ① 状态机完整性（`_captured`/`_capture_index`/`_last_delivered`） | **无越界、无无出口分支、无跨趟污染**；发现 1 处结构性缝隙 **N-3** | `min()` 防护 + 实测 RETREAT/VIOLATION/COMPLETE 均可退出 |
| ② 记账一致性（`delivered_ids` 三处） | 三处**都传了**、`None` 默认仍是旧行为、两记账点**成对**；但 `start_match()` **未复位** → **N-2** | grep 全部调用点 + 实测残留 |
| ③ 终场路径 | 🔴 **N-1（blocker）**：把"当前帧看不到目标"当成终场 → **误停车退赛**；死锁/提前退出两项已核实无问题 | 实测 DONE 单向不可恢复 |
| ④ B8 判定改造 | ✅ **三项全部正确**（无效投放不污染、场心重放能检出、运送途中不再抢导航目标） | 实测 2 / 4a / 4b |
| ⑤ 并发与"异常被吞" | 本轮新增分支**不会**被 catch-all 吞掉（逐条核实）；但发现 1 处**既存坐标语义不一致被新旋转放大** → **N-4** | `estimate_position` 与 `estimate_ground_position` 返回值语义不同 |
| ⑥ 配置钳制（`SLEEVE_MAX_HOLD`） | 🟡 钳制在**写入侧**而非读取侧，存在旁路 → **N-5** | 实测 `_sleeve_max_hold = 3` |

### 与 t13 初稿的关系（自我修正记录）

初稿曾把「首趟有效投放锁存 + 落点前伸 150mm」判为 blocker。**本次逐行复核后修正**：
- 落点几何结论**成立**且已量化（`DROP_FORWARD_MM=150` vs 区域高 300mm → 有效朝向窗口仅 ±69°；从场地内部朝红安全区直行到达朝向≈90° → 落点偏出，判 `ON_FENCE`）。
- 但"首趟无效 → 之后批量趟被拒"的**直接触发**依赖该几何，且 `can_load_batch` 的拒绝是**可恢复的**（首趟目标是普通物资，仍能重做）。
- 因此把它的严重度从 blocker 下调为 **high（N-6）**，并把当前版本的真正 ship-blocker 让给 **N-1（误终场）**——后者一旦触发是**整场结束、不可恢复**，比"少分"更致命。

---

# 一、重点 ③ 终场路径

## N-1【blocker】把"当前帧目标表为空"当成终场 → 误刹车整场退赛

- **位置**：
  - 触发侧（本轮新增）：`src/rescue_robot/states/autonomous_state.py:283-293`
  - 条件侧（未强化）：`src/rescue_robot/decision/decision_engine.py:249-255`
  - 数据源：`src/rescue_robot/perception/world_map.py:255-256`（每帧 `track_lost_count += 1`）、`:292-298`（>150 帧即删除目标）
- **证据（代码）**：
  ```python
  # autonomous_state.py:283-293（新增）
  if self._decision.strategy_state == StrategyState.DONE:
      if not self._finished_logged:
          self._finished_logged = True
          logger.info("🏁 比赛结束（决策引擎 DONE）→ 清导航目标 + 停车")
          try: self._navigation.clear_target()
          except Exception as e: logger.warning(f"清导航目标失败: {e}")
          self._stop_chassis()
      self._stop_event.set()     # ← 主循环退出：整场结束，不可恢复
      return
  ```
  ```python
  # decision_engine.py:249-255（DONE 的两个来源之一 = 纯"当前帧看不到目标"）
  if not self._world_map.active_targets:
      self._strategy_state = StrategyState.DONE
      logger.info("所有目标已清空!")
      return Action(type=ActionType.WAIT, detail="所有目标已清空")
  ```
- **证据（实测，纯软件）**：
  ```
  场上无目标时 update():
    第一帧 action=WAIT strategy=DONE
    第二帧 action=WAIT strategy=DONE
  grep "_strategy_state = " decision_engine.py
    → 117/176(复位 FIRST_TRIP)、234(ANOMALY)、245(TIME_PRESSURE)、250/255(DONE)、
      355/496(FREE_RUN)、489(FORCED_RESET)
    ⇒ 没有任何分支能从 DONE 逃出 → DONE 单向
  ```
- **影响（在比赛里损失什么）**：
  1. **误停车 = 整场退赛（0 分）**。`active_targets` 为空**不等于**比赛结束。真机最常见的触发路径是
     「摄像头掉线 / 整帧无检测 → `world_map.update([])` 每帧给所有目标 `track_lost_count+1` → 3 秒（`MAX_LOST_COUNT=150`）后**所有目标被删除**」。
     旧行为下这只是"暂时看不见目标"，重出帧后会重建目标继续跑；**新行为下这 3 秒一过立刻 `_stop_event.set()` 停车**。
  2. 赛规的结束条件确实包含"救援目标被全部移至安全区内"（`README.md:174-178`），但判据必须是**账面清空（已投放/已在安全区）**，不能是"这一帧的检测列表为空"——后者受遮挡/掉线/光照影响极大。
  3. 与"掉帧导致重复扑空"是同一类根因的**放大版**：以前丢的是目标，现在丢的是整场。
- **严重度**：**blocker**
- **建议修法**（按优先级）：
  1. 终场判据改为「**时间到** 或 **账面清空**」：
     ```python
     done = (self.time_remaining_s <= 0) or (
         not self._world_map.active_targets
         and self._world_map.pending_count == 0
         and self._world_map.targets_in_safe_zone_count() >= self._initial_target_count)
     ```
     其中 `_initial_target_count` 在首次稳定建图后记一次（`world_map` 需新增该计数）。
  2. DONE 加"**确认窗口**"：连续 N 秒（建议 3~5 s，YAML 可调）持续满足才置 DONE；期间保持探索/保命运动。
  3. `world_map.update([])` 在"连续无检测帧数 > K"时**冻结** `track_lost_count`（t2 的 S-22 已提，本条是其下游后果）。
  4. 保留 DONE 的"可恢复"语义（目标重新出现即可改回 `FREE_RUN`），只把"时间到"当硬终场。

## N-1 附带结论：重点 ③ 的另外两个子问题**已核实无问题**

| 子问题 | 结论 | 证据 |
| --- | --- | --- |
| 是否会在**正常完成一趟**时被误触发？ | **除 N-1 那条路径外不会**。正常完成一趟时 `strategy_state` 仍是 `FIRST_TRIP`/`FREE_RUN`；`DONE` 只由"时间到"或"目标表为空"设置 | `decision_engine.py:249-255`；`transport` 完成后决策返回 `WAIT` 但策略状态不变 |
| 是否与 `on_exit` 的线程 join **死锁或提前退出**？ | **不会死锁**。`_run_once` 先 `return`，主循环下一轮即退出；`on_exit()`（`autonomous_state.py:239-243`）的 `join(timeout=2.0)` 由 `is_alive()` 守护，最坏只是 2 s 警告；且 `_stop_chassis()` 已让下位机停车，不会"带着速度丢主循环" | `autonomous_state.py:283-293`、`239-243` |
| `_finished_logged` 复位时机 | **正确**：`__init__:143` 与 `on_enter:203` 都置 `False`，重进 AUTONOMOUS 可再记录一次 | 同上两行 |

---

# 二、重点 ② 记账一致性

## N-2【medium】`start_match()` 不复位 `_delivered_ids` / `_trips_completed` / `_score`（多场次残留）

- **位置**：`src/rescue_robot/decision/decision_engine.py:171-177`（`start_match`）、`:140-142`（`_delivered_ids` 定义）
- **证据（实测）**：
  ```
  [重点2b] start_match() 复位检查: 前=({1000}, 1, 5) 后=({1000}, 1, 5)
          → _delivered_ids / _trips_completed / _score 均未复位
  ```
  `start_match()` 只复位 `_match_start_time`/`_match_elapsed`/`_strategy_state`/`_action_phase`，未清 `_delivered_ids`、`_trips_completed`、`_targets_delivered`、`_score`、`_current_target`、`_trip_targets`、`_fallback_level`。
- **影响（含定量，避免夸大）**：
  - `_delivered_ids` 残留 **功能上无害**：`world_map._next_id` 单调递增（`world_map.py:376`），新比赛的目标 id 不会与旧集合碰撞，`_check_invalid_transport` 不会误命中。
  - `_score`/`_targets_delivered`/`_trips_completed` 残留 **会让累计分读错**（`get_stats()` 用于日志/自检输出）；"2~3 场比赛"若在同一进程内进行（`README.md:180-182`，`main.py` 无重启逻辑），现场按日志判成绩会误判。
- **严重度**：medium
- **建议修法**：`start_match()` 末尾补
  ```python
  self._delivered_ids.clear()
  self._trips_completed = 0; self._targets_delivered = 0; self._score = 0
  self._current_target = None; self._trip_targets = []
  self._fallback_level = FallbackLevel.NORMAL
  ```

## 重点 ② 的其余三个子问题：**已核实正确**

| 子问题 | 结论 | 证据 |
| --- | --- | --- |
| 三个内部调用点是否都传了 `delivered_ids`？ | ✅ 全部显式传参 | `update()` → `_handle_first_trip(...)`（`decision_engine.py:261-263`）、`update()` → `_handle_free_run(...)`（`:265-267`）；两个 handler 也有 `= None` 默认（`:277-280`、`:409-412`） |
| `None` 默认值下旧行为是否自洽？ | ✅ 自洽。`if delivered_ids is not None and t.id not in delivered_ids: continue` → 传 `None` 时**不跳过任何目标**，等价旧行为；首趟分支同样由 `delivered_ids is not None` 守卫 | `decision_engine.py:445-460`、`:328-336` |
| 上游（真机 / 仿真）是否真的传了？ | ✅ 真机 `autonomous_state.py:344-348` 传 `velocity`/`release_valid`/`delivered_ids=self._transport.delivered_target_ids`；仿真 `integrated_sim.py:286-287` 同样传三者 | 两处 grep 命中 |
| `_trips_completed` 是否双计？ | ✅ **无双计**：无效投放分支 `return`（`:452-455`）不会走到正常累计（`:466`） | `decision_engine.py:445-467` |

---

# 三、重点 ① 状态机完整性

## N-3【low】`_capture_index` 有防护，但"多余目标"在决策层没清账（结构性缝隙）

- **位置**：`src/rescue_robot/transport/transport_pipeline.py:388-470`（逐个套取 / 容量不足跳过）、`decision_engine.py:436-466`（`_trip_targets` 清理）
- **证据（实测）**：
  ```
  [重点1] _capture_index 越界与状态残留
    计划1个: 末态 phase=COMPLETE captured=[] capture_index=0 越界=False
    计划2个: 末态 phase=IDLE     captured=[] capture_index=0 越界=False
    计划3个: 末态 phase=IDLE     captured=[] capture_index=0 越界=False

  [重点1b] RETREAT 出口
    RETREAT + nav=None            -> APPROACHING   ✓ 可退出
    RETREAT + nav.is_arrived=True -> APPROACHING   ✓ 可退出

  [重点1c] COMPLETE 后残留
    delivered_target_ids=[7]  phase=COMPLETE
    _current_targets=[]  _captured=[]  _capture_index=0   ✓ 已清空
  ```
- **结论（确定）**：
  1. **无越界**：`_capture_index` 只在 `CAPTURING` 成功分支自增（自增前已判 `< len(_current_targets)`），所有列表访问都带 `min(_capture_index, len(...)-1)`（`:360-361`、`:424-425`、`_begin_retreat:283`）→ 即使 `_current_targets` 被外部清空也不会 `IndexError`。
  2. **无"没有出口"的分支**：`RETREAT` / `VIOLATION` / `COMPLETE` 均有出口（实测）；`VIOLATION` 的出口是"下一帧自动作废回 IDLE"（`:353-366`）。
  3. **无跨趟污染**：`start_trip()`（`:320-336`）与 `reset()`（`:598-612`）都清了 `_captured/_capture_index/_last_delivered/_place_started/_place_step/_capture_retries/_confirm_fail_streak`；`COMPLETE` 后也清空（实测）。
- **缝隙内容**：`SLEEVE_MAX_HOLD=1` 且计划含 2~3 个目标时，套取第 1 个后直接 `TRANSPORTING`，剩余目标只被"跳过 + warning"（`:462-470`），**而 `DecisionEngine._trip_targets` 仍保留全部计划**（决策层只用 `delivered_ids` 过滤记账，不清列表）。
- **影响（损失什么）**：低。实测路径下不会算错分（`delivered_ids` 过滤生效）；风险在"决策层与转运层的计划集合不同步"——这正是 `SLEEVE_MAX_HOLD` 被强制钳到 1 的原因（`config.py:186-206` 注释已诚实说明）。
- **严重度**：low
- **建议修法**：投放完成后由转运管线回吐"本趟实际计划"给决策层，或让决策引擎在 `release_done` 时用 `delivered_ids` **重建** `_trip_targets`（而非只过滤）；`_handle_free_run` 的"已投放"分支改为"以 `delivered_ids` 为准记账"。

---

# 四、重点 ④ B8 判定改造 —— ✅ **三项实测全通过**

- **位置**：`decision_engine.py:409-434`（`_check_invalid_transport`）、`:436-438`（`not grip_done` 守卫）、`:140-142`（`_delivered_ids`）
- **证据（实测）**：
  ```
  [重点2]  送达后: _delivered_ids={1000} status=IN_SAFE_ZONE          ← 成对写入 ✓
  [重点4a] 无效投放: action=WAIT _delivered_ids=set() status=ACTIVE   → 未污染 ✓
  [重点4b] 场心重放检出 = True                                          ← 能检出 ✓
  ```
- **三条确定结论**：
  1. **无效投放不会污染 `_delivered_ids`**：`_handle_free_run` 在 `release_valid=False` 时**提前 return**（`:445-455`），`_handle_first_trip` 同理（`:312-320`），两处都在 `mark_in_safe_zone` / `_delivered_ids.add` **之前** → 被判无效的目标不会被记成已送达，于是 `_check_invalid_transport` 之后能正常检出它回到场心（实测 4b=True）。✅ 设计正确。
  2. **两个记账点同步**：`mark_in_safe_zone(t.id)` 与 `self._delivered_ids.add(t.id)` 始终相邻成对（`:461-462`；首趟 `:354-355`），不存在"只写一个"的路径。✅
  3. **运送途中不再抢导航目标**：新增的 `if not grip_done and self._check_invalid_transport(rx, ry)`（`:438`）**正是我 t2 报告 S-21 建议的修法**——手上没货时才允许因"目标无效"重选，避免把导航目标从安全区抢走导致整趟送不到。✅
- **唯一残留风险（medium，非本轮引入）**：`_current_target` 的判定依赖"该 id 还在 `world_map` 里"。若目标被 `MAX_LOST_COUNT` 删除后重建为新 id，`_delivered_ids` 里的旧 id 查不到（`t is None → continue`）→ "重放场心"检不出。与 N-1 同源，建议一并修。

---

# 五、重点 ⑤ 并发/线程与"异常被吞"

## N-4【medium】新旧坐标语义不一致，被新旋转逻辑放大到"远距离目标方位错误"

- **位置**：`src/rescue_robot/perception/perception_pipeline.py:204-234`（新旋转）、`src/rescue_robot/perception/detection.py:399`（`estimate_position` 返回值）、`:437`（`estimate_ground_position` 返回值）
- **证据（代码）**：
  ```python
  # detection.py:437  首选路径 —— 返回值就是"车体系"
  return (x_off, d)          # (横向偏移, 前向距离)
  # detection.py:395-399  兜底路径 —— 同名 (x, y) 但语义完全不同，且多减了 H/2
  x_mm = distance_mm * math.sin(angle_x)
  y_mm = distance_mm * math.cos(angle_x) - camera_height_mm * 0.5
  return (x_mm, y_mm)
  ```
  ```python
  # perception_pipeline.py:227-234（新）—— 把两个返回值**统一**当作 (右向偏移, 前向距离)
  right_off, forward = pos[0], pos[1]
  dx = forward * c + right_off * s
  dy = forward * s - right_off * c
  ```
- **先说好的一面**：首选路径的**数学完全正确**。按"θ 从 +X 逆时针、车体前=(cosθ,sinθ)、车体右=(sinθ,−cosθ)"逐项验算，`dx=forward·cosθ+right·sinθ`、`dy=forward·sinθ−right·cosθ` 与实现一致 → **S-05 的核心修复成立**。✅
- **问题**：当 `estimate_ground_position()` 返回 `None` 时（`detection.py:394-395`：视线俯角 `alpha <= 1°` 即拒绝，典型场景就是**目标很远**；或没有有效 bbox），流程退回 `estimate_position()`（`perception_pipeline.py:198-206`），而它的返回 `y` 被当成"前向距离"用 —— 实际是"测距在图像纵轴上的投影 −105mm"。**表现为：远距离目标被放到错误方位，甚至可能落到车后**。决赛场地 3000×3000、远距离 `bottom_y` 靠上 → `alpha` 小 → **首选路径被拒是常态**，所以这不是罕见分支。
- **关于"异常会不会被吞掉"（重点⑤的核心担心）—— 逐条核实结论：本轮新增分支都不构成新的吞异常风险**
  - `perception_pipeline.py:218-223`：`robot_theta is None` 的 fallback **只告警一次**（`_theta_warned`），数学部分只有 `math.cos/sin`，无除零；
  - `perception_pipeline.py:172-177` 的 `except Exception: pass`（**既存**）只包住 `frame.shape` 分辨率读取，不影响定位；
  - `autonomous_state.py:290` 的 `_navigation.clear_target()` 与新终场路径都**显式 try/except + warning**（有日志，非静默）；
  - 转运新分支：`drop_position()`（`transport_pipeline.py:620-634`）纯 `cos/sin`；`_begin_retreat` 的距离归一化有 `norm < 1e-3` 除零保护（`:286-288`）；`_capture_index` 全部带 `min()`；
  - `_run_once` 外层 catch-all 仍在（`autonomous_state.py:228-232`），但**新增分支里没有会抛未捕获异常的点**。
- **严重度**：medium（不崩、不影响 50Hz；但在高频场景给出错误坐标，且"按错语义旋转"比"不旋转的错"更难被发现）
- **建议修法**：让兜底路径**也返回 `(right_off, forward)`**（把 `distance·cos(angle_x) − H/2` 改为 `distance`，`x` 用 `distance·sin(angle_x)` 作右向偏移）；更稳的是**删掉面积法兜底**（`estimate_ground_position` 返回 `None` 时直接丢弃该帧目标）——面积法在 4 种形状混装的赛场上本就不可用（t2 的 S-05 已说明）。

---

# 六、重点 ⑥ 配置钳制

## N-5【low】`SLEEVE_MAX_HOLD` 的钳制在**写入侧**，存在读取侧旁路

- **位置**：钳制点 `src/rescue_robot/config.py:186-206`（`apply_robot_config` 内）；读取点 `src/rescue_robot/transport/transport_pipeline.py:108-115`；字段定义 `config.py:80-92`（`class Placement`，**普通类、非 frozen**）
- **证据（实测）**：
  ```
  [重点6] 配置钳制是否有旁路
    config.Placement.SLEEVE_MAX_HOLD 初始 = 1
    直接改类属性为 3 后，新实例 _sleeve_max_hold = 3   <-- 旁路成功（钳制不在读取侧）
  ```
  读取侧只做 `max(1, min(3, int(getattr(cfg_placement, "SLEEVE_MAX_HOLD", 1))))`——钳的是 **[1,3]**，并不禁止 `>1`；"禁止 >1"这条策略只写在 `apply_robot_config()` 里。
- **影响（损失什么）**：任何在 `apply_robot_config()` **之后**直接赋值 `config.Placement.SLEEVE_MAX_HOLD = 2/3` 的代码（调试脚本、自检脚本、未来的现场调参入口）都能绕过钳制，重新落进注释里明确警告的坑（"决策引擎 `grip_done` 契约假设一趟 1 个 → 套取途中下发 `TRANSPORT_TO` 抢走导航目标 → 实测 180 s 只送 4 个，比容量 1 的 7 个更差"）。
- **严重度**：low（正常路径 YAML → `apply_robot_config` 已被 `tools/fix_verifiers/verify_s40_s01_team.py:201-209` 覆盖并验证回退到 1）
- **建议修法**：把"禁止 >1"下沉到读取侧：
  ```python
  hold = int(getattr(cfg_placement, "SLEEVE_MAX_HOLD", 1) or 1)
  if hold != 1:
      logger.error("仅支持 SLEEVE_MAX_HOLD=1（决策引擎耦合未完成），强制回退 1")
      hold = 1
  self._sleeve_max_hold = hold
  ```
  或把 `Placement` 改为 `dataclass(frozen=True)` 并统一由 `apply_robot_config` 赋值（与 `Timing`/`Thresholds` 的做法一致，消除"随时可改的全局可变状态"）。

## N-6【high】首趟"有效投放"锁存 + 落点前伸 150 mm → 首趟大概率判无效，之后批量趟被拒（初稿 N-1，严重度已下调；成因确定、影响可控但会显著丢分）

- **位置**：`transport_pipeline.py:620-634`（`drop_position`）、`config.py:88-91`（`DROP_FORWARD_MM=150.0`）、`transport_pipeline.py:533-541`（投放判定用落点）、`load_manager.py:348-360`（`first_ok` 才置 `_first_trip_done`）、`load_manager.py:111-121`（`is_first_trip = not _first_trip_done`）、`load_manager.py:250-256`（`can_load_batch` 的拒绝点）
- **证据（两条独立实验）**：
  **(a) 有效朝向窗口只有 ±69°**
  ```
  红物资区 x[1200,1490] y[2670,2970]；完全置入要求 y∈[2680,2960]
  车心停在区域中心 (1345,2820)，L=150：
    θ=  0° drop=(1495.0,2820.0) valid=False OUTSIDE
    θ= 30° drop=(1474.9,2895.0) valid=True  SUPPLY_AREA
    θ= 90° drop=(1345.0,2970.0) valid=False ON_FENCE     （y 超界 10mm）
    θ=115° drop=(1281.6,2955.9) valid=True  SUPPLY_AREA
    θ=180° drop=(1195.0,2820.0) valid=False ON_FENCE
  ⇒ 有效窗口 sinθ∈[-0.933,0.933] → θ∈[-69°,69°]（或镜像段）
  ```
  **(b) 自然接近路径里 5 个有 4 个落在窗口外**（用目标指向律算到达朝向）
  ```
  从(1000,1500)→ 75.4° → 落点(1382.9,2965.1) valid=False ON_FENCE
  从(1200,2000)→ 80.0° → 落点(1371.1,2967.7) valid=False ON_FENCE
  从(1500,2200)→104.0° → 落点(1308.6,2965.5) valid=False ON_FENCE
  从(1345,2000)→ 90.0° → 落点(1345.0,2970.0) valid=False ON_FENCE
  从( 900,2400)→ 43.3° → 落点(1454.1,2923.0) valid=True  SUPPLY_AREA
  ```
  **(c) 首趟判无效后**：第二趟"1 个普通物资"仍可开始（可恢复），但"普通+核心"被拒（`ok=False`），30 次尝试装载"普通+核心+伤员"被拒 30 次。
- **影响（损失什么）**：到达朝向是**自由变量**——`MotionController.is_at_target()`（`motion_control.py:257-263`）只看位置不看朝向，`TRANSPORTING→PLACING` 判据是"距 nav.target < 80mm"（`transport_pipeline.py:492-496`），**没有任何朝向约束**。于是首趟是否"有效"接近抛硬币；一旦判无效，`_first_trip_done` 不置位 → 无法开始核心/多目标趟（核心 10 分/个、伤员 15 分/个拿不到），只能在"送 1 个普通物资"上循环。更糟的是失败**不自愈**（无效目标留在场上被再次选中）。
  **注意**：`GOAL_ANALYSIS.md` 的 121/127 分是在"代码默认路径"下测出的（其仿真里 `nav.target` 使车正好停在区域中心、落点靠 `rtheta`），**本轮改成落点判定后该基准必须重跑**，否则会得出乐观结论。
- **严重度**：**high**（成因确定、几何必然；可通过"朝向约束"或"落点钳制"当天修掉；不修则显著丢分）
- **建议修法**：
  1. **落点钳制（最省事）**：`drop_position()` 算出落点后，钳制到目标子区域 `region` 的内缩范围内（`x∈[r.x+m, r.x+w−m]`、`y∈[r.y+m, r.y+h−m]`）。
  2. **加朝向闭环**：`TRANSPORTING → PLACING` 前要求 `|Δθ| < 0.2 rad`（或在 `PLACING` 前把车头摆正再释放）。
  3. **解除锁存副作用**：首趟未完成时**只约束"含非普通物资的批次"**，并让 `DecisionEngine` 保持"只选普通物资"（它已如此），避免"N 次拒绝"的循环。
  4. **把 `DROP_FORWARD_MM` 默认值降到与区域几何相容**（区域高仅 300mm，150mm 已占 54%），或改为"沿朝向投影到区域内的最近合法点"再判定。

## N-7【low】`load_manager.py` 出现重复 `return`（死代码）

- **位置**：`src/rescue_robot/transport/load_manager.py:367-369`
- **证据**：`367: return released` / `368: 空行` / `369: return released`（369 不可达）
- **影响**：无功能影响，但属"改完没清理"，误导读者以为后面还有逻辑。
- **严重度**：low
- **建议修法**：删除 368-369 两行。

## N-8【low】`TransportStatus.violation` 恒为 `None`，VIOLATION 原因在可观测层丢失

- **位置**：`transport_pipeline.py:626-636`（`_get_status()` 里 `violation=None,  # not tracked at this level`）；本轮新增的 `self._violation`（`:137`）**没有被填进 status**
- **影响**：现场与自检程序读 `status.violation` 永远是 `None`，无法区分"本趟为什么被作废"；`m_transport` 自检项也因此无法断言违规类型。
- **严重度**：low
- **建议修法**：`_get_status()` 改为 `violation=self._violation`。

---

# 七、真修好的部分（正面结论，附证据）

| 原编号 | 修复位置 | 复核结论 |
| --- | --- | --- |
| **S-02** `start_trip` 守卫 | `transport_pipeline.py:305-308`（`if not self.is_idle():`） | ✅ 真修好；实测忙碌时返回 `(False, Violation.NONE)`，不再覆盖趟次 |
| **S-04** VIOLATION 无出口 | `transport_pipeline.py:353-366` + `load_manager.py:40-55`（不再标 `is_fatal`） | ✅ 真修好；VIOLATION 下一帧自动作废回 IDLE |
| **S-13** 禁区区被动态清除抹掉 | `forbidden_zones.py` / `navigation_pipeline.py`（+77 行，静态层/动态层分离） | ✅ 已改（`tools/fix_verifiers/verify_p1_5_forbidden_zones.py` 有断言） |
| **S-19** 投放判定用车身位置 | `transport_pipeline.py:533-541` + `drop_position()` | ✅ 方向正确（改用落点）——**但引入 N-6 的几何约束** |
| **S-40** 多目标假装载（机制侧） | `transport_pipeline.py:386-470`（`_captured`/`_capture_index` + `CAPTURE_RADIUS_MM` 复核）+ 决策层 `delivered_ids` 记账 | ✅ **机制侧真修好且设计正确**：实测车只到第 1 个目标时 `_captured` 只有 1 个；`delivered_ids` 精确过滤，未套上的目标留在场上（`sleeve_max_hold` 默认 1 并显式挡 >1，属诚实降级） |
| **S-11** 异常链被架空 | `decision_engine.py:222-241`（`velocity` 由上游传入，`None` 才保持旧行为）+ `anomaly_handler.py` 签名改 `Optional` | ✅ 方向正确；真机 `autonomous_state.py:344` 已传 `_last_velocity` |
| **S-05** 缺 theta 旋转 | `perception_pipeline.py:211-234` | ✅ 首选路径数学验算通过；未传 `robot_theta` 时**告警一次**而非静默错位（兜底路径另有 N-4） |
| **S-01** 终场不停车 | `autonomous_state.py:283-293` | ⚠️ 修好了"不停车"，但**同时引入 N-1**（判据太软） |

---

# 八、回归建议（给 fixer / verifier）

1. **必须补的断言**（每条对应本报告一个缺陷，可纯软件跑）：
   - **N-1**：`world_map._targets.clear()`（模拟 3 s 丢帧）后连续 10 帧 `update()` → `strategy_state` **不得**变 `DONE`、`_stop_event` **不得**被 set。
   - **N-2**：连续两次 `start_match()` → `_delivered_ids == set()`、`_score == 0`。
   - **N-4**：构造 `bottom_y_px` 使 `alpha < 1°`（远距离）→ 目标坐标不得落到车后（`dy < 0`）。
   - **N-5**：`config.Placement.SLEEVE_MAX_HOLD = 3` 后**新建** `TransportPipeline()` → `sleeve_max_hold == 1`。
   - **N-6**：以 θ = 0/45/90/135/180° 五种到达朝向跑投放 → `release_all` 的 `placement_ok` **应全为 True**（当前 90°/180° 为 False）。
2. **基准必须重跑**：`GOAL_ANALYSIS.md` 的 121/127 分是在"容量 1 + 旧的宽松投放判定"下测出的，本轮把投放判定改成落点后**旧分数不可引用**；请用 `tools/fix_verifiers/run_all.py`、`snapshot_sim.py` 重跑并对比（历史上已发生过一次"105 → 0"的假回归，务必确认是半成品快照还是真实回归）。
3. **朝向闭环**：`TRANSPORTING → PLACING` 前加 `|Δθ| < 0.2 rad` 断言（当前无任何朝向约束）。

---

# 九、需真机确认（本次唯一无法纯软件判定的部分，共 3 条）

1. **`DROP_FORWARD_MM` 的标定值**：几何结论（150mm 对 300mm 深区域 → 有效朝向窗口仅 ±69°）是纯软件可判定的；但"该填多少"必须真机量（把目标放进槽里，量车心到目标中心的水平距离）。
2. **真机到达朝向分布**：N-6(b) 的"5 个接近路径里 4 个无效"是用目标指向律的模型推算，方向确定、具体比例需实车统计。
3. **下位机固件侧**（`Tang29Zx/rescue_f103c8` 本机不存在）：`SERVO,RAISE/LOWER` 实际脉宽、速度看门狗 300/800 ms、`EVENT,START_BUTTON` 单次还是周期上报 —— 与本轮改动无关，仍属未覆盖项。
