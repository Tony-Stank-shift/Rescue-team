# 审计复现脚本（纯软件，不需要任何硬件）

用于复现 `docs/audit/CODE_AUDIT.md` 中标注 `[复现]` 的结论。**不依赖 RDK、串口、摄像头**，只需 Python 3.10 + 仓库内的 `src/`。

## 运行

```bash
# 全部用例
PYTHONPATH=src python3 docs/audit/repro/logic_repro.py

# 单个用例（M1..M7）
PYTHONPATH=src python3 docs/audit/repro/logic_repro.py M1
```

## 用例与结论对照

| 用例 | 验证内容 | 期望输出（关键行） | 对应报告条目 |
| --- | --- | --- | --- |
| `M1` | 端到端任务流：选目标 → 套取 → 投放 → 计分 | `得分=35 送达=4 趟数=3`，选择顺序 `REGULAR_SUPPLY → INJURED → CORE_SUPPLY` | 摘要「端到端能否达成目的」正面结论 |
| `M2` | 一趟多目标装载 | `车只到达第 1 个目标 (2400.0, 2500.0)，实际记入装载: 3 个 ids={1000,1001,1002}` | **S-40**（blocker） |
| `M3` | `start_trip` 的 `is_idle` 守卫 | `当前阶段=CAPTURING is_idle()=False` 却 `start_trip -> ok=True` | **S-02**（blocker） |
| `M4` | 动态障碍清除抹掉禁区 | `禁区格: 341 -> 328   丢失 13 格` | **S-13**（high） |
| `M5` | `LocalPlanner` 全碰撞兜底 + A* 耗时 | 起点落入障碍格 → `(0.0, 0.0)`；`A* 单次规划耗时 ≈ 24 ms` | **S-18**、**S-23**（medium） |
| `M6` | 决策层边界（危险目标/空场/时间压力） | 只剩危险目标 → 探索（不抓 ✓）；空场 → `WAIT(所有目标已清空)` | 摘要正面结论、S-03 相关 |
| `M7` | 安全区目标重检测 | 位置准确时 `_selectable=False`；偏差 220 mm 时 `_selectable=True` | S-22 / S-05 关联（medium） |

## 注意

- `M1` 里的 `_NavStub` 是 `NavigationPipeline` 的最小替身（真实链路传的就是它）。若把 `nav.target` 留空，
  `TransportPipeline` 会停在 `TRANSPORTING` 不投放 —— 这是桩的用法问题，不是发现的缺陷。
- 全部用例都是**逻辑仿真**：用"直接朝目标走"的一阶运动模型替代真实导航，用 Mock 套取机构替代真舵机。
  因此结论都是**逻辑层面**的（选谁、装几个、判不判得过、禁区区在不在），不涉及控制精度。
- 报速相关数字（A* ≈24 ms、`CVDetector` ≈2.4 ms、`PerceptionPipeline` ≈2.1 ms）随机器而异，量级结论不变。
