# VERIFICATION_REPORT —— 独立验证：修复真实性 + 分部测试程序有效性

> 任务：t5　作者：verifier（独立验证员）　日期：2026-09-15
> 环境：WSL，**无 RDK X5（关机）、无摄像头、无底盘、当前用户不在 dialout 组**
> 方法：所有结论均为我**本机亲自运行**所得，脚本在 `/tmp/goalexp/`（未入库）；
> 未修改 `src/`、`tools/`、`config/` 任何文件；未执行任何 git 写操作。
> 验证对象快照：工作区未提交改动（含 fixer 的 S-40 / S-01 / S-NEW / B3 / B4 / B8 修复）。

---

## 0. 结论摘要

| 维度 | 判定 |
|---|---|
| 编译 / 导入 / 单测 / 模块自测 | ✅ 全部通过（compileall exit 0；59/59 模块导入；单测 12/12；20/21 模块 `__main__` exit 0） |
| 分部实机测试程序（`hw_selftest`） | ✅ **真实可用**：`--mock` PASS=7 FAIL=0 SKIP=7 exit 0；**4 例故障注入全部报出 FAIL 并指明模块名** |
| 诚实集成基准（5 种子） | ✅ 基线成立：**`valid==delivered`、`ERROR=0`、危险物从未转运**（三套 harness 口径下均成立）。<br>⚠️ **种子口径须三要素写明**：仓库脚本 `snapshot_sim.py`（`SEEDS=[1,7,42,99,123]`、`start_zone=1`）为 **80/80/80/85/80，7/7/7/8/7**；本报告早期引用的 `80/80/85/80/80、7/7/8/7/7` 出自临时脚本 `(1,42,123,7,2024)@start_zone=3`。详见 `FULL_PROBLEM_LIST.md §回归基线三要素` |
| S-40 / S-01 / S-02 / B8 / S-NEW 修复 | ✅ 真实有效（我独立复现并尝试推翻，未能推翻） |
| B3 首趟闭环 / B4 投放判定 | ⚠️ **B4 引入恒真判定（新 blocker）**，并**反向废掉 B3 的围栏内侧闸门** |
| 旧基准 105/12/12 | ✅ 确认已作废（S-40 虚高），不得再作基线 |
| 反例（串口/无帧/世界地图/载规则/测距/显式停车） | ✅ 全部通过 |

**最重要的新发现**：`N-7（blocker）` —— 投放判定把"预测落点"先 `clamp_into_area()` 钳进目标区**再**校验，
导致**任何**落点都被判有效（投到场地中央 / 对方安全区 / 错分区也都 valid），B3 的"必须投进围栏内侧"闸门随之失效。
证据见 §5 N-7。

---

## 1. 必跑项（命令 + 真实输出摘要）

### 1.1 compileall

```bash
python3 -m compileall -q src/rescue_robot tools
```
→ `exit=0`（无输出即无语法错误）

### 1.2 全模块导入（pkgutil.walk_packages）

```bash
PYTHONPATH=src python3 -c "<walk_packages 遍历 rescue_robot.*>"
```
→ `扫描 59 个模块，导入失败 0`

### 1.3 单测（pytest 被 anyio 插件污染 → 手动收集调用）

```bash
PYTHONPATH=src python3 -c "import test_core_units as t; ...逐个调 test_*..."
```
→ **`单测: 12/12 通过, 0 失败`**

### 1.4 各模块自带自测（`grep -rl __main__` 得到的 21 个模块）

```bash
for f in $(grep -rl "__main__" src/rescue_robot --include=*.py); do PYTHONPATH=src python3 -m <模块>; done
```

| 结果 | 模块 |
|---|---|
| exit=0（20 个） | comm_manager / decision_engine / opponent_strategy / chassis_interface / serial_chassis / config_loader / debug_dashboard / deploy / hardware_profile / hot_reloader / model_switcher / navigation_pipeline / imu_fusion / **perception_pipeline** / fault_tolerance / logging_system / stability / **integrated_sim** / sim_2d / **transport_pipeline** |
| exit=124（1 个） | `rescue_robot.main` —— **预期行为**（main 是常驻服务，等按钮/信号。实测输出尾部为 `MockCommServer` ping + Ctrl+C 后 `状态转移: DEBUG → ERROR (紧急停止)` + `机器人已安全停止。`，是"被 timeout 杀掉"而非崩溃） |

`integrated_sim` 自测尾部真实输出：
```
投放统计: 7 个投放, 7 个位置正确
危险目标是否被误转运: False
```

### 1.5 诚实集成基准（5 种子，必须调 `setup_match()`）

```bash
PYTHONPATH=src python3 -c "
from rescue_robot.simulation.integrated_sim import IntegratedSim
for sd in (1,42,123,7,2024):
    sim=IntegratedSim(seed=sd, start_zone=3); sim.setup_match()
    for i in range(int(180/0.02)):
        f=sim.step()
        if f['is_terminal']: break
    ..."
```

| seed | score | delivered | valid | trips | ERROR 数 |
|---|---|---|---|---|---|
| 1 | 80 | 7 | 7 | 7 | 0 |
| 42 | 80 | 7 | 7 | 7 | 0 |
| 123 | 85 | 8 | 8 | 8 | 0 |
| 7 | 80 | 7 | 7 | 7 | 0 |
| 2024 | 80 | 7 | 7 | 7 | 0 |
| **合计/均值** | **405 / 81.0** | **36 / 7.2** | **== delivered** | | **0** |

✅ 与队长给的诚实基准逐位一致。**旧基准 105/12/12 确认作废**：那是 S-40「计划即实装」的虚高
（车只到第 1 个目标，却把整趟计划的目标全记为已送达）。

### 1.6 分部实机测试程序

```bash
PYTHONPATH=src python3 tools/hw_selftest.py --mock
```
真实输出（尾部）：
```
ranging       PASS    测距正常（200~3000mm 回环最大误差 0.00%）
decision      PASS    决策与载规则符合赛项要求（首次单独1普通/≤3/伤员单独/危险拒绝；终场停车与场心误判防护正常）
navigation    PASS    导航正常（可规划并到达目标；异常目标显式拒绝/钳制、不崩、不冲进禁区）
transport     PASS    转运状态机正常（全流程走通、显式停车生效、无违规）
accounting    PASS    载荷台账与实际一致（计划≠实装被正确区分、套取位姿复核生效）
start_button  PASS    软件层正常（一键启动事件可被识别）
vision        PASS    视觉识别可用（无帧不崩、空帧不误报、四类目标颜色与类型均正确）
serial/telemetry/servo/odometry/motors/velocity  SKIP（无串口权限 / 需 --yes-motion）
camera        SKIP    摄像头 index=0 打不开（/dev/video0）
PASS=7  FAIL=0  SKIP=7    ✅ 结论：未发现故障模块（部分模块因环境受限 SKIP）
```
→ `exit=0`。注意 PASS 数已从 t4 交付时的 6 升到 **7**（新增 `accounting` 模块），数字变动来源已确认。

---

## 2. 故障注入（4 例，均证明"能报出 FAIL 并指名模块"）

| # | 命令 | 真实结果 | 判定 |
|---|---|---|---|
| ① | `--port /dev/nonexistent` | `serial FAIL 设备 /dev/nonexistent **不存在**`、`telemetry FAIL`、`servo FAIL`；汇总 `PASS=7 FAIL=4 SKIP=3` → **`❌ 结论：故障模块 = serial, telemetry, servo, camera`**，`exit=1` | ✅ |
| ② | `--image /tmp/does_not_exist.png` | `vision FAIL --image 指定的文件不存在：/tmp/does_not_exist.png`；汇总 `PASS=6 FAIL=5 SKIP=3` → **`故障模块 = serial, telemetry, servo, camera, vision`**，`exit=1` | ✅ |
| ③ | `[camera]` 无 `/dev/video0`（本机真实环境） | `camera FAIL 摄像头 index=0 打不开（/dev/video0）`，并被计入汇总的故障模块列表 | ✅ |
| ④ | `--mock --image /tmp/gray150.png`（纯亮灰 V=150 空图） | `vision FAIL 离线图 gray150.png 里一个目标都没检出 → HSV 阈值与现场光照不匹配（或图里确实没目标）`，`PASS=6 FAIL=1 SKIP=7`，`exit=1` | ✅ |

4/4 都**点名到具体模块**并给出人话处置。这一条 t4 的核心承诺成立。

附带观察（非缺陷）：`--image /tmp/black.png`（纯黑）→ 报告 `检出 1 个`。纯黑图 V=0 落在 HSV 黑阈值内，
所以被判为 1 个黑色目标；`vision` 仍 PASS（它只要求"绿色被检出 + 不崩"，黑图检出属于检测器语义而非测试缺陷）。
`--image` 用进程替换 `<(...)` 时 OpenCV 读不出（fifo），脚本正确报 FAIL——这是加固而非缺陷。

---

## 3. 反例验证（全部执行，全部通过）

| 反例 | 命令/脚本 | 真实输出 | 判定 |
|---|---|---|---|
| 串口不存在时优雅降级 | `SerialChassis('/dev/ttyNOPE99').open()` | `open() 返回 False（未抛异常）is_open=False`；`send_velocity→False`、`read_pose→None`、`send_servo→False`，仅打日志 | ✅ 不崩 |
| `frame=None` 感知返回空 | Mock 与 CVDetector 两条路径 | 均 `不抛异常且活跃目标=0` | ✅ |
| 连续确认（单帧误检不成真目标） | WorldMap 连喂同一检测 | 第1帧 `真目标0/待确认1`；第2帧 `真目标0`；第3帧 `真目标1` | ✅ |
| 短暂丢失不删（<3s） | 连喂空检测 | **第 151 帧（3.02s）**才删（`MAX_LOST_COUNT=150`），1.0s 时仍在 | ✅ |
| 安全区目标被排除 | `_selectable()` / `get_regular_supplies()` | 安全区内 `False`、场地上 `True`；`get_regular_supplies()` 只返回场上的 `[1001]` | ✅ |
| 套取失败重试 3 次后放弃 | 注入"永远套不上"的 sleeve，逐帧推进 | `CAPTURING → RETREAT → APPROACHING` ×3，日志 `套取失败(1/3)(2/3)(3/3)` → `套取连续 3 次失败：放弃本趟转运` → `phase=IDLE, retries=0` | ✅ |
| 进套取时显式停车 | 注入 stop 回调 + 可观测 nav | `进入 CAPTURING 后: nav.target=None(已清) stop_calls=1` | ✅ 清导航目标 + 下发停车都真的发生 |
| 视觉测距正反解回环 | h=210mm / tilt=30° / fov=77° / 640×480，正向投影再反解 | 300/500/800/1200/2000mm → 估算 300.0/500.0/800.0/1200.0/2000.0，**相对误差全部 0.00%** | ✅ |
| 载规则 >3 | `can_load_batch([R]*4)` | `(False, OVER_LIMIT)` | ✅ |
| 载规则 危险目标 | `can_load_batch([R,D])` | `(False, DANGEROUS_TARGET)` | ✅ |
| 载规则 伤员混装 | `[I,R]` / `[R,I]` / `[I,I]` / `[I,C]` | **四种组合全部 `(False, INJURED_MULTI)`**（顺序无关，t4 报的"顺序相关"漏判确认已修） | ✅ |
| 载规则 首趟 | `[R,R]`/`[C]`/`[I]`/`[R]` | 违规/违规/违规/**放行** | ✅ |

---

## 4. 抽查审计声称已修复项（≥5 条，给 文件:行 证据）

| 编号 | 声称 | 我的核查证据（文件:行） | 判定 |
|---|---|---|---|
| **S-40 上半** 计划≠实装 | `transport_pipeline.py:126-128` `_captured` 与 `_current_targets` 分离；`:221` `CAPTURE_RADIUS_MM=150`；`:399-409` 记入前位姿复核（`dist > 150 → APPROACHING`）；`:441-447` 只记入当前这一个 | 行为已复现：计划 3 个 → `_captured=[1]`、`count=1`；900mm 外强置 CAPTURING → `count=0` 且打回 APPROACHING；160mm → 拒绝、140mm → 记入（边界两侧都对） | ✅ 真实 |
| **S-40 下半** 决策只给真送达者标记 | `decision_engine.py:143` `_delivered_ids`；`:192,281,306` 三处 `delivered_ids` 参数；`:341-351` 首趟 `id not in delivered_ids → 重做首趟`；`:466-472` FREE_RUN 只标记真送达 | 行为已复现（需同时传 `release_valid=True`，见 N-8） | ✅ 真实 |
| **S-01 终场停车** | `autonomous_state.py` `_run_once` 顶部终场分支（`clear_target` + `_stop_chassis` + `_stop_event.set`）——`inspect.getsource` 实测片段含 `# ── 终场停车（S-01）──` | 反验：正常开局 `state=FIRST_TRIP`、正常 NAVIGATE_TO 后仍 `!= DONE`，**不会误触发** | ✅ 真实 |
| **S-NEW 越界拒绝 + 不变更原目标** | `navigation_pipeline.py:126-155`：`is_in_field` 不通过 → `_rejected_targets += 1` + WARNING + `return False`，**不修改 `_target`** | 实测 `set_target(9000,9000)→False`、`(-500,1500)→False`、`(1500,9000)→False`，三次之后原目标仍是 `(1500,1500)`；场内目标 `→True` | ✅ 真实（但见 N-9 边界可观测性缺陷） |
| **S-02 忙碌时拒 start_trip** | `transport_pipeline.py` `start_trip` 守卫（队长护栏脚本断言 CAPTURING 中被拒） | `run_all.py` 6/6、`verify_s40_s01_team.py` 18/18 全过（我复现其输出）<br>（上述为**评审当时**口径；结项批已扩至 **`run_all.py` 7/7 套、`verify_s40_s01_team.py` 51 项断言**，均实测通过） | ✅ 真实 |
| **B8 场心误判** | 只认"**已送达过**的目标回到场心"（`_delivered_ids` 成员判定） | 实测：无关 ACTIVE 目标在距场心 28mm → `_check_invalid_transport()` 返回 **False**；把它加入 `_delivered_ids` 后 → 返回 **True** | ✅ 真实 |
| **B3 首趟闭环** | `decision_engine.py:354-357` `release_valid is None → True`（兼容旧调用）；`:358-365` `release_valid=False` → 保持 FIRST_TRIP 重做首趟 | 关闭闸门本身有效（不传 `release_valid` 时首趟被判无效并重做）；**但闸门输入被 B4 污染**，见 N-7 | ⚠️ 部分有效 |
| **N-1 加固** | `anomaly_handler.py:125` `speed = 0.0`；`:132` 守卫内重新赋值；`:194` 卡死检测使用 | 当前 5 种子 `ERROR=0`，集成仿真不再每帧 `DECISION_ERROR` | ✅ 真实 |
| **S-NEW 死锁反验**（队长判定"不会死锁"，我质疑后复核） | `transport_pipeline.py:489-492` RETREAT 出口 = `nav is None or nav.is_arrived()` | 用可编程 stub nav 精确控制：`is_arrived()=True` → **一帧内** `RETREAT → APPROACHING`；连续 3 次失败后 `phase=IDLE, _current_targets=0`。**我最初的"卡在 RETREAT"结论是我脚手架没推进 `nav.update()` 造成的假阳性，已撤回** | ✅ 队长判定成立 |

> 补充：我最初用真实 `NavigationPipeline` 但**忘记每帧调 `nav.update()`**，得到"卡在 RETREAT 1500 帧"的假阳性。
> 修正脚手架后（每帧推进导航）该现象消失。已按纪律撤回，不做无证据指控。同理 §5 的 `_begin_retreat` 静默忽略已降级为 low。

---

## 5. 新发现的问题（含严重度）

### N-7【blocker】投放判定被 `clamp_into_area()` 变成恒真，B3 闸门随之失效

- **位置**：`transport_pipeline.py:545-550`（`base_drop = self.drop_position(...)` → `self._placer.clamp_into_area(base_drop, info)` → `classify_batch`）；
  `safe_zone_placer.py:251-268`（`clamp_into_area` 把点钳进 `target_area_region(info)` 的内缩矩形）；
  `safe_zone_placer.py:216-221`（`_is_fully_inside` 只用 `FULLY_INSIDE_MARGIN_MM=10`）。
- **实测（`/tmp/goalexp/t5_clamp.py`）**：

| 真实落点 | 未钳制判定 | 钳制后落点 | **生产实际判定** |
|---|---|---|---|
| 场地中央 (1500,1500) | INVALID | (1480,2680) | **valid** |
| 紫围栏南侧 (1345,2600) | INVALID | (1345,2680) | **valid** |
| **对方安全区 (1500,150)** | INVALID | (1480,2680) | **valid** |
| 伤员区中心放物资 (1655,2820) | INVALID | (1480,2820) | **valid** |
| 场地外西侧 (-800,2820) | INVALID | (1210,2820) | **valid** |
| **伤员投到物资区 (1345,2820)** | INVALID | (1520,2820) | **valid** |

  `clamp_into_area` 按 `target_area_region(info)` 钳制，**天然保证落点落在"该类型应有的区域"内侧**，
  再交给 `classify()` 校验，因此 `classify` 的 `WRONG_*_IN_ZONE` / `ON_FENCE` / `OUTSIDE` 三个失败分支**永远不可能触发**。
- **影响（全部会丢分或谎报）**：
  1. **B3 首趟闸门失效**：`release_valid` 恒为 True，`decision_engine.py:358-365` 的"投歪 → 保持 FIRST_TRIP 重做首趟"永远不会发生 → 首趟投歪会被当成功，按赛规**后续全部转运无效**。
  2. **计分虚高**：目标实际落在安全区外/围栏上，软件仍记 `valid=True` 并加满分（与 S-40 是同一类"谎报"）。
  3. **-10 分场景不可见**：物资入伤员区（赛规 -10 分/个）在日志里永远看不到，无法复盘。
  4. **`clamp_into_area` 的注释与实际语义相反**：注释说"等价于要求车停得更靠内"，但代码并没有改变车的停放，
     只是把**预测落点**搬进了目标区——即"把测量值改成合格值"，而不是"把车停对"。
- **建议返工方向**：判据改回"**未钳制的真实落点**"（`classify(base_drop, info)`），
  钳制只用于**生成投放点/调整车姿**（例如先算 `clamp_into_area` 得到期望落点，反推导航目标并开过去，
  到位后按真实落点判定）；若确实要用钳制，必须改为"钳制前落点与区域的距离"作为判据（例如距区域 > 某阈值即判无效），
  否则该检查没有任何判别力。**该判定不能作为验收通过的依据**——它现在只能返回 True。

### N-8【high】`release_valid` 默认 `None` 把"首趟围栏内侧"闸门在无实参调用点静默旁路

- `decision_engine.py:354-357`：`if release_valid is None: release_valid = True`（兼容旧调用方）。
- 实测：我按"只看 `delivered_ids`"的写法调用时，首趟被判"投放无效"→ 保持 FIRST_TRIP；
  传入 `release_valid=True` 后正常进入 FREE_RUN，说明**闸门完全由这个可选参数驱动**。
- 风险：任何忘记传 `release_valid` 的调用方（真机/仿真/未来新增入口）都会**默认放行**，
  刚好与 B3 想达到的效果相反。建议默认值改为"从 placer 结果推导"或强制显式传参（无默认）。

### N-9【medium】`set_target` 越界拒绝的"可观测性/一致性"缺陷

1. **角点 `(3000,3000)` 返回 True**：它被视为"场内"（`is_in_field` 是闭区间），随后被 `clamp_to_safe` 钳成 `(2999,2999)`，
   而该点仍在边界安全带内 → 之后的导航会打出 `A*: 终点 (2999, 2999) 不可通行` 并 `BLOCKED`。
   即"接受了目标但永远到不了"，与 S-NEW 想消除的"幽灵任务"是同一类现象（只是触发点变了）。
   实测：`set_target(3000.0,3000.0) → True`、`set_target(100.0,100.0) → True`。
   建议：钳制后**再**检查一次 `is_in_field`（或检查钳制结果是否仍在禁区），不满足则同样返回 `False`。
2. **返回值几乎无人检查**：`src/` 内 `set_target(` 生产调用点 **10 处**（`grep -rn "set_target(" --include=*.py src/ | grep -v "def set_target"`），
   其中 **0 处**把返回值赋给变量（`grep -rn "= *[a-z_.]*set_target(" --include=*.py src/ | wc -l` → `0`）；
   唯一会检查返回值的代码在 `tools/fix_verifiers/verify_p1_5_forbidden_zones.py`（测试脚本，非生产路径）。
   `transport_pipeline.py:286`（`_begin_retreat`）把 `set_target` 放在 `try/except` 里只捕异常，
   而新契约是**返回 False 而不抛异常** → 被拒时**静默改为 RETREAT**、`_phase` 照置、`nav.target` 仍是套取点，
   车实际停在原地等 `is_arrived()`。我已复现该路径（stub nav `accept=False` → `phase=RETREAT, nav.target=(700,600)`）。
   **但按现有 RETREAT 出口（`is_arrived()`）它仍能在一帧后回到 APPROACHING**（见 §4 死锁反验），
   所以**不构成死锁**，只构成"静默无日志 + 行为与预期不符"。定级 medium（可观测性 + 契约不一致），非 blocker。

### N-10【medium】`_run_once` 的感知调用与 `perception.update` 新契约必须成对，否则定位整体错位

- `perception_pipeline.py:218-227` 在未收到 `robot_theta` 时打印：
  `perception.update 未收到 robot_theta → 目标定位忽略机器人朝向，会随车头方向整体错位`。
- 生产调用点 `autonomous_state.py:363` **已正确传** `robot_theta=theta`（我核对过），
  所以生产链路无此问题；但任何新增/第三方调用点（含调试脚本、t4 分部测试）漏传就会静默错位，
  且只有一次 WARNING（`_theta_warned` 只告警一次）。建议把该参数改为**必填**或默认从 `sensor_fusion` 取。

### N-11【low】`rescue_robot.main` 无自测退出路径

`python3 -m rescue_robot.main` 是常驻服务（等按钮/信号），我的批量自测里必然 `exit=124`（被 timeout 杀）。
这不是缺陷，但会让"各模块 `__main__` 自测全绿"的验收口径出现一个假红。建议文档里显式写明，
或给 main 加 `--selftest` 一键跑完即退。

### N-12【low】`R1` 刷屏已量化（供队长决策）

- 实测：500 次非法 `set_target` → **500 条 WARNING**，耗时 2.5ms（本地 196786 次/秒）。
- 50Hz 主循环下若上层持续下发非法目标 → **约 50 条/秒、180s 约 9000 条**。
- 判定：会污染日志、也可能拖慢写盘，但**不会丢分、不会卡死**。是否加限流由队长决定（我倾向加计数聚合日志）。

---

## 6. 无法验证 / 不确定项（诚实标注原因）

| 项 | 原因 |
|---|---|
| 真实串口/舵机/电机/里程计行为 | RDK X5 关机、`/dev/ttyS1` 存在但当前用户**不在 dialout 组**（Errno 13）→ `hw_selftest` 全部 SKIP，只能验证"失败时归因正确" |
| 摄像头实拍识别、`SLEEVE_ROI` 套取视觉确认 | 无 `/dev/video0`；只能用合成图与空图验证"不崩、不误报" |
| `drop_forward_mm` / `push_dist_mm` 等真机标定值 | 需实机标定；我只能在几何上验证判据，无法验证数值是否对 |
| 系统级 `_stop_chassis()` 实际是否让底盘停住 | 无底盘；只能验证代码在终场分支被调用（静态 + 分支存在性），无法验证物理效果 |
| S-40 修复在"真机多目标装运"下的表现 | `SLEEVE_MAX_HOLD=1`（机构物理容量），因此多目标路径在生产配置下不会被执行；队长护栏里"容量=3 的对照臂实测更差（4 vs 7~8）"我未能独立复现（该臂依赖尚未定型的容量>1 决策耦合） |

---

## 7. 复核纪律与自纠

1. **我的第一个"卡在 RETREAT"结论是假阳性**（脚手架未推进 `nav.update()`），已撤回并降级为 N-9 medium。
   与本报告同批提交，避免污染后续返工决策。
2. 我**没有**复核已被队长驳回的 B2（安全区几何），按指令不再纳入验证。
3. 全程**未修改** `src/`、`tools/`、`config/`；**未执行**任何 `git commit/push/tag/fetch/pull`；
   仅做只读检查（`git status/diff/log` 未写远端）。
4. 发现有价值的可疑点时，我先尝试**自己推翻**它（换脚手架、换判据、构造边界值），
   只有推翻失败才写入"新问题"；无法复现的写入 §6。
