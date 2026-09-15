# REVIEW_REPORT —— 质量审查：t3 代码修复是否真正解决审计问题

> 审查人：reviewer（质量审查员）　被审对象：**t3**（fixer 的代码修复）
> 审查轮次：round 1　日期：2026-09-15
> 审查方式：**只读审查**，未修改 `src/`、`tools/`、`config/`、`tests/` 下任何文件，未做任何 git 写操作。
> 依据：`docs/audit/COMPLIANCE_AUDIT.md`、`docs/audit/CODE_AUDIT.md`、`docs/audit/FIXES.md`、
> `docs/audit/S_NEW_NAV_TARGET_CONTRACT.md`、`docs/audit/VERIFICATION_REPORT.md`
> 复核基准：**当前工作区**（未提交改动，`git status` 见 §6）——不是 HEAD。

---

## 0. Verdict

# 🔴 verdict = needs_revision

**理由（一句话）**：B1 / B5 / B6 / S-01 / S-02 / S-04 / S-40 / P1-2 / P1-5 / P1-8 等**逐条实证确认真修**，
但本轮新引入的 `clamp_into_area()` 使投放判定**恒为有效**，把 **B3「首趟必须投进物资区围栏内侧」闸门变成空操作**——
这是"**改了个表象但根因没动，反而把根因盖住了**"的典型形态，且后果是**规则级（本轮结束）**，属 blocker。

> ⚠️ 与 VERIFICATION_REPORT 的 N-7 是同一处缺陷。我**独立复现**并额外证明了它的**下游后果**
> （B3 闸门失效 + 首趟违规不可见），这部分是新增证据（§2）。

---

## 1. 审查结论总表（对照两份审计报告）

### 1.1 COMPLIANCE_AUDIT.md 的 blocker（B1~B4）

| 编号 | 声称修复 | 是否真修 | 证据（文件:行） |
|------|---------|---------|----------------|
| **B1** 出发区坐标系写死 3 号区 | ✅ FIXES §5 | ✅ **真修** | `perception/field_elements.py:280` `get_start_zone()` / `get_start_pose()` 作为唯一来源；实测四区：`1→(150,2850,-1.6)`、`2→(2850,2850,-1.6)`、`3→(150,150,+1.6)`、`4→(2850,150,+1.6)`（1/2 朝 -Y、3/4 朝 +Y，与"朝场地中心"一致）；`autonomous_state.py` `_apply_start_pose()` 三处同步；`main.py` 读 `START_ZONE`。**主流程不再是 0 次 `set_start_pose`** |
| **B2** 安全区几何偏 300mm | 已驳回 | ✅ **驳回正确** | 队长像素实测 + `FIXES.md:137-144` 记录；`field_elements.py:184/211` 几何与图 7 一致。**本项不再审**（按队长驳回口径） |
| **B3** 首趟规则未闭环 + VIOLATION 卡死 | ✅ FIXES §7 | ⚠️ **一半真修、一半被架空** | **VIOLATION 卡死 → 已真修**：`transport_pipeline.py:362` 新增自恢复分支（作废本趟→清目标→回 IDLE）。**首趟闭环闸门 → 被本轮新代码架空**：`decision_engine.py:363-369` 闸门本身写得对，但它的输入 `release_valid` **恒为 True**（见 §2），闸门**永不触发** |
| **B4** 投放有效性用车身位置 | ✅ FIXES §6 | ✅ **真修（这半是对的）** | `transport_pipeline.py:543` 改用 `drop_position((rx,ry,rtheta))`（车心 + L·朝向，`config.Placement.DROP_FORWARD_MM`），`transport_pipeline.py:633` 定义。**方向完全正确**——但结果随即被 `clamp_into_area` 覆盖（§2） |

### 1.2 CODE_AUDIT.md 的 blocker（S-01~S-05、S-40）

| 编号 | 是否真修 | 证据（文件:行） |
|------|---------|----------------|
| **S-01** 终场/清空不停车 | ✅ **真修** | `autonomous_state.py:294-318`：DONE 分两类——"时间到"直接终场，其它来源须**连续保持 `DONE_CONFIRM_S=3.0s`** 才执行（`autonomous_state.py:67`），DONE 消失即复位继续比赛。**设计比我在 t10 里写的"只返回 WAIT"更稳**，误停车方向被防住 |
| **S-02** `if not self.is_idle` 漏括号 | ✅ **真修** | `transport_pipeline.py:311` 已改为 `if not self.is_idle():`，且 `:309-310` 留了成因注释。**"忙碌检查"现在真的生效** |
| **S-03** 冲刺期选到伤员触发违规 | ✅ **结构性消除** | `target_selector.py` 改为价值密度比较 + 容量感知（`min(max_count, SLEEVE_MAX_HOLD)`）；且 `SLEEVE_MAX_HOLD` 默认 1 → 一趟 1 个，不再有"选 3 个"的形态 |
| **S-04** VIOLATION 无出口 | ✅ **真修** | 同 B3：`transport_pipeline.py:362` 自恢复 |
| **S-05** 感知缺 theta 旋转 | ✅ **真修** | `perception_pipeline.py:150, 215-227`：新增 `robot_theta` 形参，`c,s = cos/sin(theta)` 做车体系→场地系旋转，且 `robot_theta is None` 时**只告警一次**（`_theta_warned`）而不是静默错位 |
| **S-40** 一趟全记（假装载） | ✅ **真修** | `transport_pipeline.py:127-129` 引入 `_captured`/`_capture_index`；`:396-445` 只对**当前这一个**目标套取与记入；投放判定/计分改用 `_captured`（`:546`）。`config.py:102` `SLEEVE_MAX_HOLD=1` + `:88-92` 写清物理理由（单只 SG90 单自由度，抬爪即释放） |

### 1.3 其它已核对项

| 项 | 是否真修 | 证据 |
|----|---------|------|
| **P1-2** 载规则顺序敏感 | ✅ **真修** | `load_manager.py::can_load_batch` 改为**先统计本批构成再判定**；实测 13 组用例全部正确（§3） |
| **P1-5** 禁区检查未覆盖全分支 | ✅ **真修** | 检查上移到 `update()` 开头（`navigation_pipeline.py:174-180`，在所有分支之前）；`set_target` 加 `clamp_to_safe` |
| **P1-8** 保活链失效 | ✅ **真修** | `autonomous_state.py:59-60, 389-415`：判据从"下发速度"改为**里程计实测累计位移**（≥40mm 才算在动，单帧 <1mm 滤抖动） |
| **P1.5 / S-06** YAML 不接线 | ✅ **真修** | `apply_robot_config()` 扩展到 `Placement` 等；`run_all.py` 的 `verify_p15_config` 13/13 实测改前/改后生效 |
| **B5** 蓝色误判为危险 | ✅ **真修** | `classification.py` 删除跨分值颜色映射 + 受控同色兜底；`detection.py:37-40` 检测顺序改为浅蓝优先、BLUE 饱和下界 80→120 |
| **B6** CUBE/CUBOID 不分 | ✅ **真修** | `detection.py:73` 新增 `SHAPE_ASPECT_RATIOS`，`:307-310` 以长宽比作方体主判据（CUBE 0.60~1.45 / CUBOID 1.45~3.60） |
| **S-NEW** 越界目标静默夹紧 | ✅ **真修（契约变更）** | `navigation_pipeline.py:125-150` 两道闸门，`-> bool`，越界 `return False` + 计数 + WARNING；`autonomous_state.py:439` **确实检查了返回值** |

---

## 2. 🔴 BLOCKER-1【N-7】投放判定被 `clamp_into_area()` 变成恒真 → B3 首趟闸门被架空

| 字段 | 内容 |
|------|------|
| **id** | `BLOCKER-1` |
| **severity** | **blocker** |
| **位置** | `src/rescue_robot/transport/transport_pipeline.py:548-551`（判定）；`src/rescue_robot/transport/safe_zone_placer.py:251-268`（`clamp_into_area`）；下游 `decision_engine.py:363-369` |

### 问题

生产代码在**判有效性之前**，先把预测落点按目标类型**钳进"正确子区域"内缩 10mm 的矩形**，
**再**把这个已被修正过的点拿去做区域校验：

```python
# transport_pipeline.py:543-551（当前工作区）
base_drop = self.drop_position((rx, ry, rtheta))          # 真实落点（B4 修得对）
dropped = self._captured or list(self._current_targets)
infos = [t.info for t in dropped]
positions = [self._placer.clamp_into_area(base_drop, info)  # ← 先按"该去哪"钳进去
             for info in infos]
results = self._placer.classify_batch(positions, infos)     # ← 再校验一个已被修正的点
all_valid = all(r.is_valid for r in results)
```

`clamp_into_area`（`safe_zone_placer.py:266-267`）把坐标夹到
`[region.x+10, region.x+width-10]`，而 `classify()` 的每个"无效"分支都依赖落点**不在**该子区域内
或**贴着边**（`safe_zone_placer.py:119/136/154/172`）——条件**逻辑上不可能成立**。

### 复现尝试（**可复现**，命令与真实输出如下）

```bash
PYTHONPATH=src python3 - <<'EOF'
from rescue_robot.perception.field_elements import FieldLayout, SafeZoneColor
from rescue_robot.perception.target_types import PRELIMINARY_TARGETS, TargetColor, TargetShape
from rescue_robot.transport.safe_zone_placer import SafeZonePlacer
p = SafeZonePlacer(FieldLayout.standard(), SafeZoneColor.RED)
reg = PRELIMINARY_TARGETS[(TargetColor.GREEN, TargetShape.CUBE)]
for name,(x,y) in [("场地中央",(1500,1500)), ("对方安全区",(1500,150)), ("场外",(9000,9000)),
                   ("紫围栏上",(1500,2655)), ("伤员区里放物资",(1650,2820))]:
    c = p.clamp_into_area((x,y), reg); r = p.classify(c, reg)
    print(f"{name:<16}{str((x,y)):<16}-> 钳制后{str((int(c[0]),int(c[1]))):<14}{r.zone.name:<12}valid={r.is_valid}")
EOF
```

**真实输出**（我实际运行）：
```
场景                原始落点            钳制后             classify 结果        valid
场地中央            (1500, 1500)    -> (1480, 2680)    SUPPLY_AREA         True
对方安全区           (1500, 150)     -> (1480, 2680)    SUPPLY_AREA         True
场外               (9000, 9000)    -> (1480, 2960)    SUPPLY_AREA         True
紫围栏上            (1500, 2655)    -> (1480, 2680)    SUPPLY_AREA         True
伤员区里放物资        (1650, 2820)    -> (1480, 2820)    SUPPLY_AREA         True
```

**5 个物理上完全错误的投放，全部被判 `valid=True`**。`WRONG_SUPPLY_IN_INJURED` / `WRONG_INJURED_IN_SUPPLY` /
`ON_FENCE` / `OUTSIDE` 四个分支在生产调用路径上**不可达**。

### 下游后果（我额外证明的部分）

因为 `release_valid = all_valid` 恒 True，**B3 的首趟闭环闸门（`decision_engine.py:363-369`）永不触发**。
端到端证据（真实运行，"车停在伤员区中央、物资实际落进伤员区"）：

```
车停在伤员区中心; 真实落点 = (1650, 2970)   ← 在伤员区内
钳制后落点 = (1480, 2960)                  ← 被搬到物资区
classify -> SUPPLY_AREA, is_valid=True
all_valid = True   ← 首趟闸门的输入
[对照 未钳制] 真实落点 classify -> ON_FENCE, is_valid=False   ← B4 修对了、但判据被覆盖
DecisionEngine._handle_first_trip: release_valid=True -> 闸门放行 → mark_in_safe_zone + 1分 + 切 FREE_RUN
```

### 影响（为什么是 blocker 而不是 high）

按 `README.md` 评分规则，**"首次转运不是 1 个普通物资"→ 本轮结束**；"物资放入伤员区 → -10 分/个"。
第 1 趟交付物投错区时：

1. 软件判"有效"→ 记 1 分 → 切 `FREE_RUN`（`decision_engine.py:381-386`）；
2. 真实比赛里该趟无效/违规，但软件**不知道**，会继续按"首趟已完成"转运其余目标；
3. 同时 -10 分场景**完全不可见**，自评分数系统性虚高。

即：**B4/B3 的修复目标（"投错要被发现"）被本轮的修复手段（"先钳进正确区再判"）直接抹掉**。
`INCREMENTAL_AUDIT` 把 `clamp_into_area` 当 N-6 提出、`S_NEW_NAV_TARGET_CONTRACT.md` 未涉及此路径。
`transport_pipeline.py:552-556` 的注释**已经意识到风险方向**（明确写"放宽判定会把真错误判成有效…比判无效更糟"），
但**实际采用的方案犯了同一个错误**——钳制后判定等价于把"落点在正确区内"设为前提。

### requiredFix（建议，具体可落地）

```python
# 核心：判定用【未钳制的真实落点】，clamp 只用于"生成投放点/调车姿"，不参与判定
base_drop = self.drop_position((rx, ry, rtheta))
dropped = self._captured or list(self._current_targets)
infos = [t.info for t in dropped]

# ① 判定：真实落点，不钳制
results = self._placer.classify_batch([base_drop] * len(infos), infos)
all_valid = all(r.is_valid for r in results)

# ② 仅当判无效时，用钳制点把车"再往里挪一点"重试（或据此收紧投放点），
#    且把"需要钳制多少"作为可观测指标暴露出来（说明车停得不够靠内）
```

配套要求：
1. **必须补一条回归断言**：`clamp_into_area` 的存在**不得**使 `classify` 的
   `WRONG_*` / `ON_FENCE` / `OUTSIDE` 分支失效。建议在 `tools/fix_verifiers/` 增加
   `verify_b3_placement_verdict.py`：以"车停伤员区中心 + 物资"等 5 个场景断言 `all_valid is False`。
2. **5 种种子仿真之所以全绿，是因为这条路径在仿真里走对了**——
   §5 注的原则同样适用：**回归全绿 ≠ 这条被覆盖**。必须在单测/断言层覆盖"投错区"。

---

## 3. 合规关键规则核对（5 条，全部给位置证据）

在 `LoadManager.can_load_batch` 层面 13 组用例实测（**可复现**，真实输出见下）：

| 规则 | 代码位置 | 实测结果 | 判定 |
|------|---------|---------|------|
| ① 首次必须**单独 1 个普通物资** | `load_manager.py:250-265` | 1个普通→放行；2个普通→`FIRST_TRIP_MULTI`；仅核心→`FIRST_TRIP_WRONG_TYPE`；仅伤员→`FIRST_TRIP_WRONG_TYPE` | ✅ 正确 |
| ② 其后每次 **≤3** | `load_manager.py:267-269`、`MAX_LOAD=3`（`:89`） | 3个→放行；4个→`OVER_LIMIT` | ✅ 正确 |
| ③ 普通+核心**可混合** | `load_manager.py:222-234` | 普通+核心→放行；普通+核心×3→放行 | ✅ 正确 |
| ④ 伤员**单独 1 个** | `load_manager.py:236-248` | 伤员单独→放行；伤员+普通 / 普通+伤员 / 伤员+核心 / 2伤员→**全部 `INJURED_MULTI`**（**顺序无关**） | ✅ 正确（P1-2 已修） |
| ⑤ **危险目标禁入** | `load_manager.py:226-228` | 危险单独 / 危险+普通 / 普通+危险→**全部 `DANGEROUS_TARGET`** | ✅ 正确 |
| ⑥ 禁抓取、只许推套 | `transport/sleeve_lift.py:32-36` | 动作为 `RAISE`/`LOWER`/`HOLD` + `place_ramp()` 渐进上调推入，**无夹紧动作** | ✅ 合规 |

> ⚠️ 但**规则①的"送达物资区围栏内侧"这一半不可信**——见 BLOCKER-1：
> 装载侧校验正确，**交付侧校验恒定放行**。

---

## 4. 是否引入新问题（逐项检查，附证据）

| 检查项 | 结论 | 证据 |
|-------|------|------|
| **50Hz 主循环新增阻塞** | ✅ **无新增** | 对 `autonomous_state / navigation_pipeline / decision_engine / target_selector / transport_pipeline / load_manager / forbidden_zones / detection / classification` 全量扫描 `time.sleep`/`join`/`wait_for`：仅 3 处且均为**既有**——`autonomous_state.py:206`（`on_enter` 一次性 1s）、`:254`（`on_exit` join 2s）、`:282`（**帧尾 `max(0, dt-elapsed)` 补足 20ms，是把主循环**保持在 50Hz** 的必要调用，不是阻塞源**）。本轮改动**未新增**任何阻塞点 |
| **吞异常** | ⚠️ **1 处需关注，非本轮新增** | 本轮 diff 新增 `except Exception` 4 处：`autonomous_state.py:84`（属性回退）、`:314`、`:332`、`:476`。`:314` 带注释"场地布局异常不该让主循环挂掉"并**有日志**，可接受。`:332`/`:476` 需在 t7 复核是否静默（**不作为本轮 blocker**，因不改变已有行为） |
| **竞态** | ✅ **未发现新增** | 主循环单线程；`CameraReader` 有独立锁（`camera_reader.py:53,147-150`）；本轮改动均在主循环线程内 |
| **单位 mm** | ✅ **未破坏** | `drop_position` 用 mm（`config.Placement.DROP_FORWARD_MM`）；`set_target`/`clamp_to_safe` 全 mm |
| **theta 逆时针 / x 前 y 左** | ✅ **未破坏** | `field_elements.py:122-123` 场地系约定未动；`config.py:14-27` 下位机 x前/y左/逆时针 约定未动；`chassis_interface.py:65-85` 转换未动。**B1 的出发区朝向与之一致**（1/2 号 → -Y、3/4 号 → +Y） |
| **协议帧格式** | ✅ **未破坏** | `serial_chassis.py` 未在本轮改动清单中；ODOM 8 字段 / IMU 10 字段严格解析保持；`CHASSIS_PORT` 默认改 `/dev/ttyS1`（`main.py`）属**部署默认值**变更，不影响帧格式 |

### 4.1 需要登记但**不构成本轮 blocker** 的次级问题

| id | severity | 问题 | 位置 | requiredFix |
|----|---------|------|------|------------|
| `MED-1` | medium | `release_valid=None → True` 的兼容默认值会**静默放行**首趟闸门；生产调用方（`autonomous_state.py:372`、`integrated_sim.py:286`）当前都显式传值，但任一未来调用点漏传即静默失效 | `decision_engine.py:356-357, 494-495` | 改为默认 `False`（保守），或对 `None` 打 WARNING；与 BLOCKER-1 一并修 |
| `MED-2` | medium | `set_target()` 契约变更后，`src/` 内仍有 3 个生产调用点**不检查返回值**：`transport_pipeline.py:288`（`_begin_retreat`）、`:464`（S-40 下一个目标）、`:530`（推入点） | 同左 | 至少 `_begin_retreat` 应在被拒时打日志；已在 `S_NEW_NAV_TARGET_CONTRACT.md §7 R2` 登记，**接受为已知残余风险**（不死锁，仅可观测性下降） |
| `MED-3` | medium | `set_target(3000,3000)` 返回 True 但被钳到 `(2999,2999)`，而 A* 判终点不可通行 → 仍可能产生"幽灵任务"的**残余形态** | `navigation_pipeline.py:151-156` + `path_planner.py:180-182` | 边界值应纳入"越界拒绝"（`is_in_field` 用闭区间，建议改为 `0 < x < 3000` 或直接拒绝被钳制的目标） |
| `LOW-1` | low | 越界被拒时**每次调用**打 WARNING；`_set_nav_target` 由 50Hz 主循环每帧调用 → 上层持续非法时会刷屏 | `navigation_pipeline.py:142-146` | 首次 + 每 N 次打一次，或只累加 `_rejected_targets` 交由自检暴露（`S_NEW §7 R1` 已登记） |

> 说明：`MED-1` 与 BLOCKER-1 是**同一后果链**上的两处；修 BLOCKER-1 时应一并处理 `MED-1`。

---

## 5. 方法学核查（"改表象 vs 改根因"）

| 形态 | 是否出现 | 证据 |
|------|---------|------|
| 改表象不改根因 | ⚠️ **出现 1 处** | BLOCKER-1：B4 的落点算法改对了（真根因），但判定被 `clamp_into_area` 覆盖 → **根因被盖住** |
| 补注释当修好 | ✅ 未发现 | 逐条核对均有可执行代码 + 断言脚本 |
| 用放宽判据"消除"报错 | ⚠️ **即 BLOCKER-1** | `transport_pipeline.py:552-556` 的注释**正确识别**了"放宽判定更糟"，但实现的实际效果与之等同 |
| 回归全绿当覆盖证明 | ⚠️ **存在** | 5 种子仿真全绿（我实测复现：80/80/80/85/80，见 §6），但"投错区"路径**从未被仿真触发**；`S_NEW §5` 已自陈同类原则，本条未自陈 |
| 拿仿真自评分当官方计分 | ⚠️ **需在交付文档加限定** | `FIXES.md` 已把基准口径改为 80/7/7 并标注"自评"（✅ 已改善）；但 `GOAL_ANALYSIS.md` 的 105/12 基线**仍是伪基准**（含 S-40 虚高），t9 交付时须标注作废 |

---

## 6. 我的复现记录（全部亲自运行）

| 命令 | 结果 | 用途 |
|------|------|------|
| `python3 -m compileall -q src` | exit 0 | 编译 |
| 全模块导入（`pkgutil.walk_packages`） | **59 ok / 0 failed** | 无导入破坏 |
| `tests/test_core_units.py`（手动收集） | **12 passed / 0 failed** | 单测 |
| `PYTHONPATH=src python3 tools/fix_verifiers/run_all.py` | **6/6 通过（66 项断言）** | 修复验证套件 |
| `PYTHONPATH=src python3 tools/hw_selftest.py --mock` | **PASS=6 FAIL=0 SKIP=7** | 分部自检（与 t5 记录的 PASS=7 差异见下） |
| `PYTHONPATH=src python3 tools/fix_verifiers/snapshot_sim.py` | **score 80/80/80/85/80；delivered 7/7/7/8/7；valid==delivered** | 5 种子仿真 |
| 载规则 13 组用例 | 全部符合 5 条规则（§3） | 合规核对 |
| `clamp_into_area` → `classify` 5 场景 | **全部 valid=True** | BLOCKER-1 复现 |
| 伤员区投物资端到端 | `all_valid=True` → 闸门放行 | BLOCKER-1 下游后果 |

**两处与其它文档的口径差异（需在交付前对齐）**：

1. **种子集合不一致**：`S_NEW_NAV_TARGET_CONTRACT.md §5/§10` 与 `VERIFICATION_REPORT` 用 `(1,42,123,7,2024)`
   且 `start_zone=3`；而仓库里的 `tools/fix_verifiers/snapshot_sim.py:33-35` 用 **`start_zone=1`**。
   我跑的是**仓库脚本**，得 `80/80/80/85/80`（均 81）；文档声称 `80/80/85/80/80`（均 81）——**均值一致、逐种子对应关系不同**。
   → **建议**：交付文档统一写明"脚本 + 种子集合 + `start_zone`"三要素，避免下游复现不上。
2. **`hw_selftest --mock` PASS 数（已澄清，非缺陷）**：`FIXES.md:209` 与 `S_NEW §5` 记 6，
   `VERIFICATION_REPORT` 记 7。我首轮跑得 6、复跑得 **7**（含新增 `accounting` 模块，`--list` 确认已注册 14 个模块）。
   成因是**并发写文件造成的瞬时读数**（test-author 当时正在增补 `m_accounting.py`），**不是口径不一致**。
   → **建议**：交付文档把该数字更新为 `PASS=7 FAIL=0 SKIP=7`（14 模块），并注明"读数需在无并发写入时采集"。

---

## 7. 结论与给 fixer 的最小返工清单

**必须修（否则不得进入 t8 交付）**：

1. **BLOCKER-1**：投放判定改用**未钳制的真实落点**；`clamp_into_area` 只用于生成投放点/调车姿，不参与 `classify`。
   并补 `verify_b3_placement_verdict.py` 断言"投错区必须判无效"（5 场景）。
2. **MED-1**（与 1 同批）：`release_valid=None` 的默认值改为保守语义 + WARNING。

**可接受为已知残余（登记即可，不必本轮修）**：`MED-2`、`MED-3`、`LOW-1`（均不丢分、不卡死）。

**已验证通过、无需返工**：B1、B5、B6、S-01、S-02、S-04、S-05、S-40、P1-2、P1-5、P1-8、P1.5/S-06、B4（落点算法本身）、S-NEW（越界拒绝）。

---

## 8. 附：本轮审查未覆盖项（留给 t7 / 后续）

- `tools/` 下自检程序的可用性、故障定位能力、`--yes-motion` 闸门 → **t7**。
- `docs/audit/GOAL_ANALYSIS.md` 中基于 105/12 伪基准的各条推论（如"初赛可达 121 分""决赛 127 分"）
  是否需重算 → 属文档质量审查（原 t16，已被需求方取消），**建议在 t8 集成交付时至少加一条
  "本文策略结论基于已作废的 105/12 基线，绝对分数需重算"的限定**。
