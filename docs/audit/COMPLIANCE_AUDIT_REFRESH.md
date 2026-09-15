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

## 汇总

- **仍成立（未改动）**：B1、B3、B4、B5、B6、B7、B8、B10、B12、B13、B24 —— 共 **11 条**
- **仍成立（部分已变化，需按新行号修）**：B9 —— 1 条（`anomaly_handler.check` 已重构，接触链路与看门狗未接）
- **已修复**：无
- **已驳回**：B2（队长结论）

### 对修复者的两条提示（仅基于本轮证据）

1. **B3 与 B9 是"改了一半"的两条**，最容易在后续验收中被误判为已完成：
   - B3：`load_manager.can_load_batch` 的顺序无关修复**只关掉了"同一趟 `[伤员,普通]` 被放行"这个衍生缺陷**，首趟"送达围栏内侧"的闭环与 `VIOLATION` 不进 `is_idle` 导致的**永久卡死**（`transport_pipeline.py:177,441-442`）都还在。
   - B9：`anomaly_handler` 的"无动作"判据已被改好，但**没有任何模块产生 `contact_duration_s` 并传给 `DecisionEngine`**（`autonomous_state.py:280-286` 无该实参），所以碰撞保护依然等于没有。
2. **B1 尚未开工**（`serial_chassis.py`/`main.py` 零改动），而它决定"抽到非 3 号出发区时全场错位"，仍是最高优先级的未动项。

### 判定完整性说明

- 本轮只复核 blocker/high（B1~B9）+ 队长点名的 B10/B12/B13/B24 + 被驳回的 B2；medium/low 其余项（B11、B14~B23）**未在本轮重新核验**，沿用原报告结论。
- 未复核项中有一项与 B9 同源需注意：B21（`load_manager` 投错区域"先扣分再全额加回"）证据行 `load_manager.py:283` 仍在，未受 fixer 改动影响。

*报告：req-auditor ｜ 复核方式：当前工作区逐文件读取 + `git status` 变更判定 + 关键常量实跑*
