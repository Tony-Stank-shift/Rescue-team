"""
model_switcher.py —— 初赛/决赛模型一键切换 (7.1.2)

一键切换初赛和决赛的目标识别模型配置，无需修改代码。
复用 perception/target_types.py 的 CompetitionPhase 和双配置表。

用法:
  # Python API
  sw = ModelSwitcher()
  sw.switch_to(CompetitionPhase.FINAL)
  print(sw.current_phase)  # FINAL

  # CLI
  python -m rescue_robot.innovation.model_switcher preliminary
  python -m rescue_robot.innovation.model_switcher final
"""

import hashlib
import json
import logging
import sys
import threading
from enum import Enum
from typing import Dict, Optional, Set, Tuple

logger = logging.getLogger("model_switcher")


# 复用现有定义
try:
    from ..perception.target_types import (
        CompetitionPhase,
        FINAL_TARGETS,
        PRELIMINARY_TARGETS,
        TargetColor,
        TargetInfo,
        TargetShape,
        get_target_config,
    )
except ImportError:
    # 独立运行时
    CompetitionPhase = Enum("CompetitionPhase", {"PRELIMINARY": "preliminary", "FINAL": "final"})  # type: ignore
    PRELIMINARY_TARGETS = {}
    FINAL_TARGETS = {}
    TargetColor = None  # type: ignore
    TargetShape = None  # type: ignore
    TargetInfo = None  # type: ignore

    def get_target_config(phase):
        return PRELIMINARY_TARGETS if phase == CompetitionPhase.PRELIMINARY else FINAL_TARGETS


# ============================================================
# ModelSwitcher
# ============================================================

class ModelSwitcher:
    """
    初赛/决赛模型一键切换器。

    特性:
    - 线程安全的状态切换
    - 目标数量自动统计（初赛 20 → 决赛 25）
    - 配置指纹验证（确保切换生效）
    - 切换事件通知（可供其他模块订阅）
    """

    def __init__(self, initial_phase: CompetitionPhase = CompetitionPhase.PRELIMINARY):
        self._phase = initial_phase
        self._lock = threading.Lock()
        self._subscribers: Set[callable] = set()
        self._switch_count = 0

        logger.info(f"ModelSwitcher 初始化: phase={self._phase.name}")

    # ---- 属性 ----

    @property
    def current_phase(self) -> CompetitionPhase:
        """当前比赛阶段（线程安全）"""
        with self._lock:
            return self._phase

    @property
    def switch_count(self) -> int:
        """切换次数"""
        return self._switch_count

    # ---- 切换 ----

    def switch_to(self, phase: CompetitionPhase) -> bool:
        """
        一键切换到指定比赛阶段。

        Returns:
            True 如果成功切换，False 如果已是目标阶段
        """
        with self._lock:
            old = self._phase
            if old == phase:
                logger.info(f"已在 {phase.name} 阶段，无需切换")
                return False

            self._phase = phase
            self._switch_count += 1

        # 锁外通知订阅者
        logger.info(f"⚡ 模型切换: {old.name} → {phase.name} "
                     f"(第 {self._switch_count} 次切换)")

        self._notify_subscribers(old, phase)
        return True

    def toggle(self) -> CompetitionPhase:
        """在两个阶段之间切换。"""
        new = (CompetitionPhase.FINAL if self.current_phase == CompetitionPhase.PRELIMINARY
               else CompetitionPhase.PRELIMINARY)
        self.switch_to(new)
        return new

    # ---- 配置查询 ----

    def get_target_config(self) -> Dict[Tuple, "TargetInfo"]:
        """获取当前阶段的完整目标配置表。"""
        return get_target_config(self.current_phase)

    def get_target_count(self) -> int:
        """获取当前阶段的目标总数。"""
        return len(self.get_target_config())

    def get_target_summary(self) -> dict:
        """获取当前阶段目标配置摘要。"""
        config = self.get_target_config()
        summary = {
            "phase": self.current_phase.name,
            "total_types": len(config),
            "targets": [],
        }
        for (color, shape), info in config.items():
            summary["targets"].append({
                "color": color.value if hasattr(color, "value") else str(color),
                "shape": shape.value if hasattr(shape, "value") else str(shape),
                "type": info.type.name if hasattr(info.type, "name") else str(info.type),
                "points": info.points,
                "description": info.description,
            })
        return summary

    def get_model_checksum(self) -> str:
        """
        获取当前配置的 MD5 指纹。

        用于验证切换是否生效。
        """
        config = self.get_target_config()
        # 使用 str key 避免 enum 不可比较的问题
        items = [(str(k), v.points) for k, v in config.items()]
        items.sort(key=lambda x: x[0])
        canonical = json.dumps(dict(items), sort_keys=True)
        return hashlib.md5(canonical.encode()).hexdigest()[:8]

    # ---- 对比 ----

    def diff_phases(self) -> dict:
        """
        返回初赛与决赛的差异。

        Returns:
            {"preliminary_only": [...], "final_only": [...], "shared": [...],
             "preliminary_count": int, "final_count": int}
        """
        prelim = PRELIMINARY_TARGETS
        final = FINAL_TARGETS

        prelim_keys = set(prelim.keys())
        final_keys = set(final.keys())

        def desc(key):
            info = prelim.get(key) or final.get(key)
            return info.description if info else str(key)

        return {
            "preliminary_only": [desc(k) for k in prelim_keys - final_keys],
            "final_only": [desc(k) for k in final_keys - prelim_keys],
            "shared": [desc(k) for k in prelim_keys & final_keys],
            "preliminary_count": len(prelim),
            "final_count": len(final),
        }

    # ---- 订阅 ----

    def subscribe(self, callback: callable) -> None:
        """
        订阅模型切换事件。

        callback(old_phase, new_phase) 在切换时被调用。
        """
        self._subscribers.add(callback)

    def unsubscribe(self, callback: callable) -> None:
        """取消订阅。"""
        self._subscribers.discard(callback)

    def _notify_subscribers(self, old: CompetitionPhase, new: CompetitionPhase) -> None:
        """通知所有订阅者。"""
        for cb in list(self._subscribers):
            try:
                cb(old, new)
            except Exception as e:
                logger.error(f"模型切换订阅者回调异常: {e}")


# ============================================================
# CLI
# ============================================================

def _cli_main():
    """CLI 入口: python -m rescue_robot.innovation.model_switcher [preliminary|final|status|diff]"""
    if len(sys.argv) < 2:
        print("用法: python -m rescue_robot.innovation.model_switcher [preliminary|final|status|diff]")
        print()
        print("  一键切换初赛/决赛目标识别模型")
        print()
        print("  preliminary  切换到初赛模式 (20 个目标)")
        print("  final        切换到决赛模式 (25 个目标)")
        print("  status       查看当前状态")
        print("  diff         查看初赛/决赛目标差异")
        sys.exit(1)

    cmd = sys.argv[1].lower()
    sw = ModelSwitcher()

    if cmd == "preliminary":
        sw.switch_to(CompetitionPhase.PRELIMINARY)
        print(f"✅ 已切换到初赛模式 ({sw.get_target_count()} 个目标)")
    elif cmd == "final":
        sw.switch_to(CompetitionPhase.FINAL)
        print(f"✅ 已切换到决赛模式 ({sw.get_target_count()} 个目标)")
    elif cmd == "status":
        print(f"当前阶段: {sw.current_phase.name}")
        print(f"目标数量: {sw.get_target_count()}")
        print(f"配置指纹: {sw.get_model_checksum()}")
    elif cmd == "diff":
        diff = sw.diff_phases()
        print(f"初赛目标: {diff['preliminary_count']} 种")
        print(f"决赛目标: {diff['final_count']} 种")
        if diff["preliminary_only"]:
            print(f"\n仅在初赛出现:")
            for d in diff["preliminary_only"]:
                print(f"  • {d}")
        if diff["final_only"]:
            print(f"\n仅在决赛出现:")
            for d in diff["final_only"]:
                print(f"  • {d}")
        if diff["shared"]:
            print(f"\n两阶段共有:")
            for d in diff["shared"]:
                print(f"  • {d}")
    else:
        print(f"未知命令: {cmd}")
        sys.exit(1)


# ============================================================
# 独立测试
# ============================================================

if __name__ == "__main__":
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s [%(levelname)-7s] %(name)s: %(message)s",
        datefmt="%H:%M:%S",
    )

    print("=" * 60)
    print("  模型切换器 — 独立测试")
    print("=" * 60)

    # 由于 target_types.py 需要完整导入路径，使用简化测试
    # 创建模拟的 CompetitionPhase
    class _MockPhase(Enum):
        PRELIMINARY = "preliminary"
        FINAL = "final"

    CompetitionPhase = _MockPhase

    # 覆盖 ModelSwitcher 使用的 get_target_config
    _MOCK_PRELIM = {
        ("green", "cube"): type("Info", (), {"type": type("T", (), {"name": "REGULAR_SUPPLY"})(), "points": 5, "description": "普通物资-初赛"})(),
        ("black", "pyramid"): type("Info", (), {"type": type("T", (), {"name": "CORE_SUPPLY"})(), "points": 10, "description": "核心物资-初赛"})(),
    }
    _MOCK_FINAL = {
        ("green", "cylinder"): type("Info", (), {"type": type("T", (), {"name": "REGULAR_SUPPLY"})(), "points": 5, "description": "普通物资-决赛"})(),
        ("black", "cone_frustum"): type("Info", (), {"type": type("T", (), {"name": "CORE_SUPPLY"})(), "points": 10, "description": "核心物资-决赛"})(),
        ("orange", "cuboid"): type("Info", (), {"type": type("T", (), {"name": "INJURED"})(), "points": 15, "description": "伤员-决赛"})(),
    }

    PRELIMINARY_TARGETS = _MOCK_PRELIM
    FINAL_TARGETS = _MOCK_FINAL

    def get_target_config(phase):
        return PRELIMINARY_TARGETS if phase == CompetitionPhase.PRELIMINARY else FINAL_TARGETS

    # ---- 测试 1: 初始状态 ----
    print("\n--- 测试 1: 初始状态 ---")
    sw = ModelSwitcher(initial_phase=CompetitionPhase.PRELIMINARY)
    assert sw.current_phase == CompetitionPhase.PRELIMINARY
    assert sw.get_target_count() == 2
    print(f"  当前: {sw.current_phase.name}, {sw.get_target_count()} 个目标")
    print("  ✅ 通过")

    # ---- 测试 2: 一键切换 ----
    print("\n--- 测试 2: 一键切换到决赛 ---")
    result = sw.switch_to(CompetitionPhase.FINAL)
    assert result is True
    assert sw.current_phase == CompetitionPhase.FINAL
    assert sw.get_target_count() == 3
    assert sw.switch_count == 1
    print(f"  当前: {sw.current_phase.name}, {sw.get_target_count()} 个目标")
    print("  ✅ 通过")

    # ---- 测试 3: 重复切换无操作 ----
    print("\n--- 测试 3: 重复切换无操作 ---")
    result = sw.switch_to(CompetitionPhase.FINAL)
    assert result is False
    assert sw.switch_count == 1  # 不增加
    print("  ✅ 通过")

    # ---- 测试 4: 切换回来 ----
    print("\n--- 测试 4: 切换回初赛 ---")
    result = sw.switch_to(CompetitionPhase.PRELIMINARY)
    assert result is True
    assert sw.get_target_count() == 2
    assert sw.switch_count == 2
    print("  ✅ 通过")

    # ---- 测试 5: 配置指纹 ----
    print("\n--- 测试 5: 配置指纹 ---")
    fp1 = sw.get_model_checksum()
    sw.switch_to(CompetitionPhase.FINAL)
    fp2 = sw.get_model_checksum()
    assert fp1 != fp2, f"初赛和决赛应有不同指纹: {fp1} vs {fp2}"
    print(f"  初赛指纹: {fp1}, 决赛指纹: {fp2}")
    print("  ✅ 通过")

    # ---- 测试 6: 订阅通知 ----
    print("\n--- 测试 6: 订阅通知 ---")
    notifications = []

    def _on_switch(old, new):
        notifications.append((old, new))

    sw.subscribe(_on_switch)
    sw.switch_to(CompetitionPhase.PRELIMINARY)
    assert len(notifications) == 1
    assert notifications[0] == (CompetitionPhase.FINAL, CompetitionPhase.PRELIMINARY)
    print(f"  收到通知: {notifications[0][0].name} → {notifications[0][1].name}")
    print("  ✅ 通过")

    # ---- 测试 7: 摘要 ----
    print("\n--- 测试 7: 配置摘要 ---")
    summary = sw.get_target_summary()
    assert summary["phase"] == "PRELIMINARY"
    assert len(summary["targets"]) == 2
    print(f"  阶段: {summary['phase']}, 类型: {summary['total_types']}")
    for t in summary["targets"]:
        print(f"    • {t['description']} ({t['points']}分)")
    print("  ✅ 通过")

    print(f"\n{'=' * 60}")
    print("  模型切换器 — 全部测试通过 ✅")
    print(f"{'=' * 60}")
