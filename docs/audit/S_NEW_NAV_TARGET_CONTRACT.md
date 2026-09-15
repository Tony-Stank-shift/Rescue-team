# S-NEW：导航目标越界被静默夹紧，造出"永远走不到的幽灵任务"

> **留档原因**：这条不在原审计清单（`COMPLIANCE_AUDIT.md` / `CODE_AUDIT.md`）里，是
> test-author 在评审分部自检程序时**新发现**的（编号 S-NEW）。它的修复**改变了
> `NavigationPipeline.set_target()` 的对外契约**（新增返回值 + 新增"拒绝设置"语义），
> 比其它局部修复的引入风险高，因此单独留档：现象、根因链、影响面、契约变更、
> 决策与理由、残余风险、验证方式、真机待验项。
>
> **决策（软件负责人裁决）**：**保留该修复**。理由见 §6。
> **边界**：除本文件 §8 列出的后续动作外，`src/rescue_robot/navigation/navigation_pipeline.py`
> 的这段改动在结项前**不再改动**。

---

## 1. 现象（真机上会看到什么）

机器人"执行"了一个目标，导航也报告到达，但**车实际停在原地或跑偏**，且日志里
**看不出任何错误**——看起来一切正常，就是不干活。最坏情况是整场时间被这种"幽灵任务"
耗掉，而所有自检项都是绿的。

## 2. 根因链（四步，每一步单独看都"合理"）

```
① 上层给了个场地外的目标点（如调试口误设 (9000,9000)，或某处坐标计算溢出/符号错）
        ↓
② CostMap._to_grid 用 max(0, min(GRID_SIZE-1, int(mm/CELL))) 把越界坐标
   静默夹到边界格  →  (9000,9000) 变成网格 (59,59)
        ↓            （path_planner.py:105-108）
③ _in_bounds_grid(gx,gy) 因此恒为 True（path_planner.py:110-111）
   → A* 规划"成功"，PlanResult.success = True
        ↓
④ 上层据此认为规划正常、导航继续推进并可能判"到达"，
   而真正的目标点在场地外，is_in_field() == False（forbidden_zones.py:142-144）
```

**本质**：一个"输入非法"的问题被 `max/min` 夹紧**悄悄吃掉**了，变成了一个
"看起来成功、但物理上不可达"的任务。这类缺陷的特征是**不报错、只耗时**，
靠看日志和跑自检都发现不了。

## 3. 修复内容

`src/rescue_robot/navigation/navigation_pipeline.py::set_target()` 改为**两道闸门**
（顺序不可交换）：

| 顺序 | 闸门 | 行为 |
|---|---|---|
| ① | **越界拒绝** | `not self._forbidden.is_in_field(x, y)` → 不设置目标、保持原目标、打 WARNING、`return False`，并累加 `self._rejected_targets`（诊断计数） |
| ② | **禁区钳制** | 落在 hard 禁区（对方安全区 / 边界安全带）内的目标点 → 钳制到最近合法点后设置（**不能**改成拒绝：钳制是既有合规行为，直接拒绝会让"去安全区投放"全部失效） |

同时新增诊断计数 `self._rejected_targets`（`navigation_pipeline.py:87`），便于现场
一眼看出"上层一直在给非法目标"。

**为什么不能只靠"钳到边界"解决**：夹到边界格之后，机器人会朝**场地边缘**开，
既到不了真目标，还可能撞边界安全带/紫边 —— 比拒绝更危险。

## 4. 契约变更（这是本条目最需要留档的部分）

```diff
-    def set_target(self, x: float, y: float) -> None:
+    def set_target(self, x: float, y: float) -> bool:
+        """...
+        Returns:
+            True=已设置（可能被钳制）；False=被拒绝（越界），原目标保持不变
+        """
```

**语义变化**：调用方原本可以假定"调用完目标就变了"；现在**必须**容忍"目标没变"。

### 4.1 生产代码调用方逐个核对

| 位置 | 用途 | 被拒绝时的行为 | 是否死锁 |
|---|---|---|---|
| `transport_pipeline.py:286` | `_begin_retreat` 算出的后退点 | 保留原目标；随后仍置 `RETREAT`，而 `RETREAT` 分支判 `nav.is_arrived()`——原目标此刻通常是"已到达的套取点" → 立刻回 `APPROACHING` 重新对位 | **不会**（退化为原地重试，正是预期语义） |
| `transport_pipeline.py:462` | S-40 逐个套取的"下一个目标" | 目标点来自 `world_map`，坐标在场内；越界只可能来自视觉错检 | 不会（同一跳转逻辑；若目标非法，下一次 `APPROACHING` 仍按该点推进 → 由看门狗兜底） |
| `transport_pipeline.py:518` | 推入斜坡的推入点（场内点 + ≤100mm 偏移） | 保留投放点目标 | 不会（`PLACING` 不等导航到位，按帧推进） |
| `autonomous_state.py:405` | `_set_nav_target`（转发决策引擎目标） | 保留旧目标；决策引擎下一帧会再次下发 → 反复被拒 | 不会，但**会刷日志**（见 §7 残余风险 R1） |
| `integrated_sim.py:383` | 仿真导航目标 | 同 `_set_nav_target` | 不会 |
| `docs`/`__main__` 示例（`decision_engine.py:91` 文档、`navigation_pipeline.py:378/393/426` 自测） | 说明与自测 | 不适用 | — |

**结论：不引入死锁。** 被拒绝的路径全部退化为"保持原目标继续"，最坏是原地重试，
由既有的里程计位移看门狗（`autonomous_state` 按实际位移判定）兜底。

### 4.2 工具/自测侧已同步

`tools/fix_verifiers/verify_p1_5_forbidden_zones.py` 已改为**断言新契约**：
`rejected = n.set_target(-500.0, 1500.0)`（期望 False）、`ok_set = n.set_target(9000.0, 9000.0)`
（期望 False、且原目标不变）、`ok_in = n.set_target(-50.0, 1500.0)`（边界内、期望 True）。
该脚本当前 **10/10 通过** —— fixer 已把它从 7 项扩到 10 项，新增的 3 项正是 S-NEW 的
"越界拒绝 + 原目标保持不变"断言。

## 5. 验证证据（可复现）

| 项 | 命令 | 结果 |
|---|---|---|
| 编译 | `python3 -m compileall -q src/rescue_robot tools` | exit 0 |
| 集成仿真 5 种子 | `python3 /tmp/reg5.py`（见 §9 的伪代码） | score 80/80/85/80/80；delivered 7/7/8/7/7；valid == delivered；`sim.events` 中 ERROR = 0 |
| 禁区/越界契约 | `PYTHONPATH=src python3 tools/fix_verifiers/verify_p1_5_forbidden_zones.py` | **10/10** 通过（含 3 项 S-NEW 越界拒绝断言） |
| 修复验证套件 | `PYTHONPATH=src python3 tools/fix_verifiers/run_all.py` | 6/6 通过（66 项断言） |
| S-40/S-01/B8 护栏 | `PYTHONPATH=src python3 tools/fix_verifiers/verify_s40_s01_team.py` | 18/18 通过 |
| 分部自检 | `python3 tools/hw_selftest.py --mock` | PASS=6 FAIL=0 SKIP=7 |

**注**：越界拒绝这条路径在集成仿真里**不会被触发**（仿真给的目标都在场内），
所以仿真的绿色**不能**证明它有效——它的证据是 §4.2 的契约断言（`9000,9000 → False`）。
这正是本条目必须留档的另一个原因：**回归全绿 ≠ 这条被覆盖**。

## 6. 为什么决定保留（而不是回退）

1. **原行为是错的**：静默夹紧把一个非法输入变成"看起来成功的不可达任务"，
   现场表现为"车不动、导航说到了"，是所有故障类型里最难排查的一类。
2. **修复方向与既有设计一致**：本项目已确立"非法目标必须显式暴露、不许静默兜底"
   的原则（同轮修复里的 S-40 位姿复核、`sleeve_max_hold` 强制回退、首趟规则闭环
   都是同一原则）。
3. **影响面已完成核对**：§4.1 逐个调用方分析，无死锁；§5 回归全绿。
4. **风险可控**：唯一实质代价是"调用方需容忍目标未变"，而现状所有调用方都是
   "下一帧重试"结构，天然容忍。

## 7. 残余风险（已知、未处理，留给后续）

| 编号 | 风险 | 影响 | 建议 |
|---|---|---|---|
| **R1** | `set_target` 被拒时**每次调用都打 WARNING**；`_set_nav_target` 由 50Hz 主循环每帧调用 → 若上层持续下发非法目标，会以 50Hz 刷屏 | 日志淹没真正的错误（本会话已发生过同类问题：`⚠️ 进入禁区` 逐帧刷屏，最终把自检默认日志调到 ERROR 才压住） | 改为"首次 + 每 N 次打一次"，或只累加 `_rejected_targets` 计数、由自检/状态接口暴露；**结项前不改**，现场若刷屏按此处置 |
| **R2** | 调用方目前**都不检查返回值**（`_set_nav_target`、`_begin_retreat` 等忽略 `False`） | 无法区分"设置成功"与"被拒绝"，只能靠日志/计数 | 后续在 `_set_nav_target` 里对 `False` 做一次降级处理（例如上报异常链） |
| **R3** | `is_in_field` 只判 `0..3000` 的矩形，不含"边界安全带" | 目标落在边界带上仍会被 `clamp_to_safe` 钳制（第二道闸门），行为正确但日志语义会混 | 可选：把越界判定并入 `clamp_to_safe` 的统一语义 |
| **R4** | 仿真不触发该分支（见 §5 注） | 该修复的长期防回退保护只有 `verify_p1_5_forbidden_zones.py` 一处 | 保持该脚本在 `run_all.py` 里，不得移除 |

## 8. 后续动作（明确的边界）

1. **保留**当前实现，结项前不再改动 `navigation_pipeline.py` 的这段逻辑。
2. 真机上按 §9 做一次**定向验证**（这是唯一还没做的验证）。
3. R1/R2 作为已知残余风险**不修**，记录在案（符合"只修会丢分/会卡死"的口径：
   它们不丢分、不卡死，只在极端情况下影响可观测性）。

## 9. 真机待验证项（现场按此做一次）

```bash
# ① 目标越界必须被拒绝、且原目标保持不变（不依赖任何硬件外设）
PYTHONPATH=src python3 tools/fix_verifiers/verify_p1_5_forbidden_zones.py

# ② 真机上确认"被拒绝"时车不会朝场外开
#    做法：DEBUG 模式下下发一个场外目标，观察
#      - 日志出现 "⚠️ 导航目标越界 (...) 在场地外 → 拒绝设置"
#      - 车保持原地（或继续执行上一个合法目标），不朝场边冲
#      - 串口无持续 VEL 下发朝场外方向

# ③ 观察 _rejected_targets 是否在正常比赛中增长
#    正常比赛应恒为 0（上层给的目标都在场内）；若增长，说明上游坐标计算有问题，
#    这是排查"幽灵任务"的第一线索——比翻日志快得多。
```

## 10. 集成仿真复现命令（§5 表格所用）

```python
import sys; sys.path.insert(0, "src")
from rescue_robot.simulation.integrated_sim import IntegratedSim

for seed in (1, 42, 123, 7, 2024):
    sim = IntegratedSim(seed=seed, start_zone=3)
    sim.setup_match()                      # ← 必须！漏掉会得到全 0 分并误判为回归
    for _ in range(int(sim.MATCH_DURATION_S / sim.DT) + 10):
        f = sim.step()
        if f["is_terminal"]:
            break
    delivered = [t for t in sim.targets if t.delivered]
    valid = [t for t in delivered if t.delivered_valid]
    errs = [e for e in sim.events if "ERROR" in e]
    print(f"seed={seed:5d} score={sim.score:5d} delivered={len(delivered)} "
          f"valid={len(valid)} ERROR={len(errs)}")
# 期望：score 80/80/85/80/80；delivered 7/7/8/7/7；valid == delivered；ERROR=0
```

---

## 附：本文件相关的两条同源留档（避免再次踩坑）

1. **集成仿真基准的更正**：`FIXES.md` 里"当前 70 / 7"是**改到一半时的快照**（当时
   `decision_engine` 的 `delivered_ids` 未穿透到 handler，异常被 `integrated_sim` 的
   `except Exception` 静默吞掉 → 决策退化为 WAIT）。实测诚实值为 **score 80~85、
   delivered 7~8、valid == delivered、ERROR=0**。
   更早的 **105/12/12 是伪基准**（含 S-40 虚高：车只到第 1 个目标却把本趟计划里的
   全部目标记为已送达），**已作废**——保它等于保住一个谎报送达的缺陷。
2. **审计文档的时效性**：`docs/audit/COMPLIANCE_AUDIT_REFRESH.md` 的复核表基于
   "只有 6 个文件被改动"的旧快照，其中至少 7 条（B1/B3/B4/B5/B6/B7/B8/S-01）已被证伪；
   该文件头部已加「⛔ 已过期」横幅，**权威版本是 `COMPLIANCE_AUDIT_REFRESH2.md`**。
