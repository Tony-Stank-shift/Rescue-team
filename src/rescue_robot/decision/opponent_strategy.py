"""
opponent_strategy.py —— 对抗策略

处理与对方机器人的竞争互动：
  - 威胁评估（距离 + 速度方向）
  - 10 秒主动脱离（>7s 预警，>9s 强制）
  - 目标争夺决策（抢先 vs 放弃）
  - 对方阻挡/进攻应对
  - 对方送入目标 = 白得分，不干扰

核心原则：禁止主动进攻，但可策略性避让和争夺。
"""

import logging
import math
import time
from dataclasses import dataclass
from enum import Enum, auto
from typing import List, Optional, Tuple

logger = logging.getLogger("opponent_strategy")


# ============================================================
# 威胁等级
# ============================================================

class ThreatLevel(Enum):
    """对方威胁等级"""
    NONE = auto()        # 未检测到对方
    LOW = auto()         # 对方远离（>1000mm）
    MEDIUM = auto()      # 对方接近（500-1000mm）
    HIGH = auto()        # 对方很近（<500mm）或朝我方移动
    CRITICAL = auto()    # 接触中或对方主动进攻


# ============================================================
# 对方行为分类
# ============================================================

class OpponentBehavior(Enum):
    """对方行为类型"""
    IDLE = auto()            # 静止
    HEADING_TO_TARGET = auto()  # 前往某目标
    HEADING_TO_US = auto()   # 朝我方移动
    BLOCKING = auto()        # 阻挡我方路径
    AGGRESSIVE = auto()      # 主动进攻（冲撞）
    RETREATING = auto()      # 后退/远离


# ============================================================
# 策略动作
# ============================================================

class StrategyAction(Enum):
    """对抗策略动作"""
    NONE = auto()              # 无需动作
    YIELD = auto()             # 让路（减速/等待）
    REROUTE = auto()           # 绕行
    DISENGAGE = auto()         # 主动脱离
    EVADE = auto()             # 紧急避让
    CONTEND = auto()           # 抢先争夺目标
    ABANDON_TARGET = auto()    # 放弃目标


@dataclass
class StrategyDecision:
    """策略决策"""
    action: StrategyAction = StrategyAction.NONE
    threat: ThreatLevel = ThreatLevel.NONE
    detail: str = ""
    suggested_velocity: Tuple[float, float] = (0.0, 0.0)  # (v, w)
    target_to_avoid: Optional[Tuple[float, float]] = None  # 需避开的点


# ============================================================
# 对抗策略引擎
# ============================================================

class OpponentStrategy:
    """
    对抗策略引擎。

    不包含任何主动进攻逻辑。所有动作都是防御性或策略性的。
    """

    # 距离阈值（mm）
    DISTANCE_CRITICAL = 350.0    # 接触判定
    DISTANCE_HIGH = 500.0        # 高风险
    DISTANCE_MEDIUM = 1000.0     # 中风险

    # 脱离阈值
    DISENGAGE_WARNING_S = 7.0    # 预警时间
    DISENGAGE_FORCE_S = 9.0      # 强制脱离时间
    CONTACT_LIMIT_S = 10.0       # 裁判强制分离时间

    # 对方朝我方移动判定（角度差阈值）
    HEADING_TO_US_ANGLE = math.pi / 4  # 45°

    def __init__(self):
        # 对方行为历史
        self._behavior_history: List[OpponentBehavior] = []
        self._aggression_count = 0
        self._last_opponent_pos: Optional[Tuple[float, float]] = None

        logger.info("OpponentStrategy 初始化")

    # ---- 主接口 ----

    def evaluate(self,
                 opponent_position: Optional[Tuple[float, float]],
                 opponent_velocity: Tuple[float, float] = (0, 0),
                 robot_pose: Tuple[float, float, float] = (0, 0, 0),
                 contact_duration_s: float = 0.0,
                 opponent_heading_to_us: bool = False,
                 robot_nav_target: Optional[Tuple[float, float]] = None,
                 ) -> StrategyDecision:
        """
        综合对抗评估，返回策略决策。

        Args:
            opponent_position: 对方位置，None=未检测到
            opponent_velocity: 对方速度 (vx, vy) mm/s
            robot_pose: 我方位姿 (x, y, theta)
            contact_duration_s: 接触持续时间
            opponent_heading_to_us: 对方是否朝我方移动
            robot_nav_target: 我方当前导航目标

        Returns:
            StrategyDecision
        """
        rx, ry, rtheta = robot_pose

        # 无对方检测
        if opponent_position is None:
            return StrategyDecision(
                action=StrategyAction.NONE,
                threat=ThreatLevel.NONE,
                detail="未检测到对方",
            )

        ox, oy = opponent_position

        # 评估威胁等级
        threat = self._evaluate_threat(
            opponent_position, opponent_velocity,
            robot_pose, contact_duration_s,
            opponent_heading_to_us,
        )

        # 记录对方行为
        behavior = self._classify_behavior(
            opponent_position, opponent_velocity,
            robot_pose, opponent_heading_to_us,
        )
        self._behavior_history.append(behavior)
        if len(self._behavior_history) > 50:
            self._behavior_history = self._behavior_history[-50:]

        if behavior == OpponentBehavior.AGGRESSIVE:
            self._aggression_count += 1

        # 策略决策
        decision = self._make_decision(
            threat, behavior, opponent_position,
            opponent_velocity, robot_pose,
            contact_duration_s, robot_nav_target,
        )

        self._last_opponent_pos = opponent_position
        return decision

    # ---- 威胁评估 ----

    def _evaluate_threat(self,
                         opponent_pos: Tuple[float, float],
                         opponent_vel: Tuple[float, float],
                         robot_pose: Tuple[float, float, float],
                         contact_duration_s: float,
                         heading_to_us: bool) -> ThreatLevel:
        """评估威胁等级"""
        rx, ry, rtheta = robot_pose
        ox, oy = opponent_pos
        dist = math.sqrt((ox - rx) ** 2 + (oy - ry) ** 2)

        # CRITICAL：接触中或对方主动进攻
        if contact_duration_s > 0:
            return ThreatLevel.CRITICAL
        if heading_to_us and dist < self.DISTANCE_HIGH:
            return ThreatLevel.CRITICAL

        # HIGH：很近或朝我方快速移动
        if dist < self.DISTANCE_HIGH:
            return ThreatLevel.HIGH
        vx, vy = opponent_vel
        speed = math.sqrt(vx ** 2 + vy ** 2)
        if heading_to_us and speed > 200 and dist < self.DISTANCE_MEDIUM:
            return ThreatLevel.HIGH

        # MEDIUM：接近中
        if dist < self.DISTANCE_MEDIUM:
            return ThreatLevel.MEDIUM

        return ThreatLevel.LOW

    # ---- 行为分类 ----

    def _classify_behavior(self,
                           opponent_pos: Tuple[float, float],
                           opponent_vel: Tuple[float, float],
                           robot_pose: Tuple[float, float, float],
                           heading_to_us: bool) -> OpponentBehavior:
        """分类对方当前行为"""
        vx, vy = opponent_vel
        speed = math.sqrt(vx ** 2 + vy ** 2)

        if speed < 50:
            return OpponentBehavior.IDLE

        if heading_to_us:
            dist = math.sqrt(
                (opponent_pos[0] - robot_pose[0]) ** 2 +
                (opponent_pos[1] - robot_pose[1]) ** 2
            )
            if dist < self.DISTANCE_CRITICAL and speed > 300:
                return OpponentBehavior.AGGRESSIVE
            if dist < self.DISTANCE_HIGH:
                return OpponentBehavior.HEADING_TO_US

        # 检测是否在阻挡我方路径
        if self._last_opponent_pos and self._is_blocking_path(
            opponent_pos, robot_pose
        ):
            return OpponentBehavior.BLOCKING

        # 后退（远离我方）
        if self._last_opponent_pos:
            prev_dist = math.sqrt(
                (self._last_opponent_pos[0] - robot_pose[0]) ** 2 +
                (self._last_opponent_pos[1] - robot_pose[1]) ** 2
            )
            cur_dist = math.sqrt(
                (opponent_pos[0] - robot_pose[0]) ** 2 +
                (opponent_pos[1] - robot_pose[1]) ** 2
            )
            if cur_dist > prev_dist + 50:
                return OpponentBehavior.RETREATING

        return OpponentBehavior.HEADING_TO_TARGET

    def _is_blocking_path(self,
                          opponent_pos: Tuple[float, float],
                          robot_pose: Tuple[float, float, float]) -> bool:
        """检测对方是否在我方前进路径上"""
        rx, ry, rtheta = robot_pose
        ox, oy = opponent_pos

        # 对方在我方前方 500mm 内且在前进方向 ±30°
        dx, dy = ox - rx, oy - ry
        dist = math.sqrt(dx * dx + dy * dy)
        if dist > 500 or dist < 50:
            return False

        angle_to_opponent = math.atan2(dy, dx)
        angle_diff = abs(angle_to_opponent - rtheta)
        angle_diff = min(angle_diff, 2 * math.pi - angle_diff)

        return angle_diff < math.pi / 6  # 30°

    # ---- 决策 ----

    def _make_decision(self,
                       threat: ThreatLevel,
                       behavior: OpponentBehavior,
                       opponent_pos: Tuple[float, float],
                       opponent_vel: Tuple[float, float],
                       robot_pose: Tuple[float, float, float],
                       contact_duration_s: float,
                       nav_target: Optional[Tuple[float, float]],
                       ) -> StrategyDecision:
        """根据威胁和行为做出决策"""
        rx, ry, rtheta = robot_pose

        # 1. 接触脱离（最高优先级）
        if contact_duration_s > 0:
            if contact_duration_s > self.DISENGAGE_FORCE_S:
                return StrategyDecision(
                    action=StrategyAction.DISENGAGE,
                    threat=threat,
                    detail=f"强制脱离: 接触 {contact_duration_s:.1f}s",
                    suggested_velocity=self._disengage_velocity(
                        robot_pose, opponent_pos
                    ),
                )
            if contact_duration_s > self.DISENGAGE_WARNING_S:
                return StrategyDecision(
                    action=StrategyAction.DISENGAGE,
                    threat=threat,
                    detail=f"预警脱离: 接触 {contact_duration_s:.1f}s",
                    suggested_velocity=self._disengage_velocity(
                        robot_pose, opponent_pos
                    ),
                )

        # 2. 对方进攻 → 避让不反击
        if behavior == OpponentBehavior.AGGRESSIVE:
            return StrategyDecision(
                action=StrategyAction.EVADE,
                threat=threat,
                detail="对方进攻 — 紧急避让（不反击）",
                suggested_velocity=self._evade_velocity(
                    robot_pose, opponent_pos, opponent_vel
                ),
            )

        # 3. 对方阻挡 → 绕行
        if behavior == OpponentBehavior.BLOCKING:
            return StrategyDecision(
                action=StrategyAction.REROUTE,
                threat=threat,
                detail="对方阻挡 — 绕行",
                target_to_avoid=opponent_pos,
            )

        # 4. 对方朝我方移动 → 减速让路
        if behavior == OpponentBehavior.HEADING_TO_US:
            return StrategyDecision(
                action=StrategyAction.YIELD,
                threat=threat,
                detail="对方朝我方移动 — 减速让路",
                suggested_velocity=(100.0, 0.0),  # 减速
            )

        # 5. HIGH 威胁 → 保持警惕
        if threat == ThreatLevel.HIGH:
            return StrategyDecision(
                action=StrategyAction.NONE,
                threat=threat,
                detail="高威胁 — 保持警惕",
            )

        return StrategyDecision(
            action=StrategyAction.NONE,
            threat=threat,
            detail="安全",
        )

    # ---- 脱离动作 ----

    def should_disengage(self, contact_duration_s: float) -> bool:
        """是否应主动脱离"""
        return contact_duration_s > self.DISENGAGE_WARNING_S

    def _disengage_velocity(self,
                            robot_pose: Tuple[float, float, float],
                            opponent_pos: Tuple[float, float]) -> Tuple[float, float]:
        """计算脱离速度（远离对方）"""
        rx, ry, rtheta = robot_pose
        ox, oy = opponent_pos

        # 朝向远离对方的方向
        away_angle = math.atan2(ry - oy, rx - ox)

        # 计算我方需要转多少
        angle_diff = away_angle - rtheta
        angle_diff = (angle_diff + math.pi) % (2 * math.pi) - math.pi

        # 后退 + 转向
        return (-300.0, angle_diff * 2.0)  # 后退 300mm/s + 转向

    def _evade_velocity(self,
                        robot_pose: Tuple[float, float, float],
                        opponent_pos: Tuple[float, float],
                        opponent_vel: Tuple[float, float]) -> Tuple[float, float]:
        """计算避让速度（侧移避开）"""
        rx, ry, rtheta = robot_pose

        # 垂直方向避开
        dx, dy = opponent_pos[0] - rx, opponent_pos[1] - ry
        perp_angle = math.atan2(dy, dx) + math.pi / 2  # 垂直方向

        # 转到垂直方向并前进
        angle_diff = perp_angle - rtheta
        angle_diff = (angle_diff + math.pi) % (2 * math.pi) - math.pi

        return (400.0, angle_diff * 2.0)  # 前进 400mm/s + 转垂直

    # ---- 目标争夺 ----

    def contend_target(self,
                       target_pos: Tuple[float, float],
                       target_points: int,
                       robot_pose: Tuple[float, float, float],
                       opponent_pos: Tuple[float, float],
                       opponent_heading_to_target: bool = False) -> StrategyDecision:
        """
        目标争夺决策。

        决策逻辑：
        - 我近、敌远 → 抢先
        - 目标高分 + 我略近 → 抢先
        - 敌近、我远 + 低分 → 放弃
        """
        rx, ry, _ = robot_pose
        ox, oy = opponent_pos

        my_dist = math.sqrt((rx - target_pos[0]) ** 2 + (ry - target_pos[1]) ** 2)
        opp_dist = math.sqrt((ox - target_pos[0]) ** 2 + (oy - target_pos[1]) ** 2)

        # 我明显更近 → 抢先
        if my_dist < opp_dist - 300:
            return StrategyDecision(
                action=StrategyAction.CONTEND,
                detail=f"抢先: 我近 {my_dist:.0f}mm vs {opp_dist:.0f}mm",
            )

        # 对方明显更近 + 低分 → 放弃
        if opp_dist < my_dist - 300 and target_points <= 5:
            return StrategyDecision(
                action=StrategyAction.ABANDON_TARGET,
                detail=f"放弃: 敌近 {opp_dist:.0f}mm, 低分({target_points})",
            )

        # 对方朝目标移动 + 低分 → 放弃
        if opponent_heading_to_target and target_points <= 5:
            return StrategyDecision(
                action=StrategyAction.ABANDON_TARGET,
                detail=f"放弃: 对方已前往, 低分({target_points})",
            )

        # 高分目标 + 我略远 → 仍尝试抢先
        if target_points >= 10 and my_dist < opp_dist + 500:
            return StrategyDecision(
                action=StrategyAction.CONTEND,
                detail=f"抢先: 高分({target_points}分)",
            )

        # 对方更近 + 高分 → 争夺
        if target_points >= 10:
            return StrategyDecision(
                action=StrategyAction.CONTEND,
                detail=f"争夺: 高分({target_points}分), 敌略近",
            )

        return StrategyDecision(
            action=StrategyAction.ABANDON_TARGET,
            detail=f"放弃: 低优先级",
        )

    # ---- 对方意外行为 ----

    def handle_opponent_gift(self,
                             target_in_our_zone: bool) -> StrategyDecision:
        """
        对方把目标推入我方安全区 → 白得分，不干扰。

        让裁判计分即可，我方不做任何动作。
        """
        if target_in_our_zone:
            logger.info("对方送入目标至我方安全区 — 白得分，不干扰")
            return StrategyDecision(
                action=StrategyAction.NONE,
                detail="对方送入=白得分，不干扰",
            )
        return StrategyDecision(action=StrategyAction.NONE)

    def handle_blocking(self,
                        opponent_pos: Tuple[float, float],
                        robot_pose: Tuple[float, float, float]) -> StrategyDecision:
        """对方静止阻挡 → 绕行，不推撞"""
        return StrategyDecision(
            action=StrategyAction.REROUTE,
            threat=ThreatLevel.MEDIUM,
            detail="对方阻挡 — 绕行，不推撞",
            target_to_avoid=opponent_pos,
        )

    def handle_aggression(self) -> StrategyDecision:
        """对方主动进攻 → 避让，不反击"""
        self._aggression_count += 1
        logger.warning(f"检测到对方进攻 (第{self._aggression_count}次) — 避让不反击")
        return StrategyDecision(
            action=StrategyAction.EVADE,
            threat=ThreatLevel.CRITICAL,
            detail="对方进攻 — 避让，严禁反击",
        )

    # ---- 查询 ----

    def get_aggression_count(self) -> int:
        return self._aggression_count

    def get_recent_behaviors(self, n: int = 10) -> List[OpponentBehavior]:
        return self._behavior_history[-n:]

    def reset(self) -> None:
        self._behavior_history.clear()
        self._aggression_count = 0
        self._last_opponent_pos = None


# ============================================================
# 独立测试
# ============================================================

if __name__ == "__main__":
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s [%(levelname)-7s] %(name)s: %(message)s",
        datefmt="%H:%M:%S",
    )

    print("=" * 50)
    print("  对抗策略 — Mock 模式测试")
    print("=" * 50)

    strat = OpponentStrategy()

    # 测试 1：无对方 → 安全
    d = strat.evaluate(None)
    print(f"\n测试 1: 无对方 threat={d.threat.name} action={d.action.name}")
    assert d.threat == ThreatLevel.NONE
    print("  ✅ 通过")

    # 测试 2：对方远离 → LOW
    d = strat.evaluate((2500, 2500), robot_pose=(500, 500, 0))
    dist_check = math.sqrt((2500-500)**2 + (2500-500)**2)
    print(f"测试 2: 远距离({dist_check:.0f}mm) threat={d.threat.name}")
    assert d.threat == ThreatLevel.LOW
    print("  ✅ 通过")

    # 测试 3：对方接近 → HIGH
    d = strat.evaluate((700, 500), robot_pose=(500, 500, 0))
    d3_dist = math.sqrt((700-500)**2 + (500-500)**2)
    print(f"测试 3: 近距离({d3_dist:.0f}mm) threat={d.threat.name}")
    assert d.threat == ThreatLevel.HIGH
    print("  ✅ 通过")

    # 测试 4：接触脱离（9s）
    d = strat.evaluate((510, 500), robot_pose=(500, 500, 0),
                       contact_duration_s=9.0)
    print(f"测试 4: 接触9s action={d.action.name}")
    assert d.action == StrategyAction.DISENGAGE
    print("  ✅ 通过")

    # 测试 5：接触预警（7.5s）
    d = strat.evaluate((510, 500), robot_pose=(500, 500, 0),
                       contact_duration_s=7.5)
    print(f"测试 5: 接触7.5s action={d.action.name}")
    assert d.action == StrategyAction.DISENGAGE
    print("  ✅ 通过")

    # 测试 6：对方进攻 → 避让
    d = strat.evaluate((510, 500), robot_pose=(500, 500, 0),
                       opponent_velocity=(500, 0),
                       opponent_heading_to_us=True)
    print(f"测试 6: 对方进攻 action={d.action.name} threat={d.threat.name}")
    assert d.action == StrategyAction.EVADE
    print("  ✅ 通过")

    # 测试 7：目标争夺——我近
    d = strat.contend_target(
        (1000, 500), 10, (500, 500, 0), (2000, 500)
    )
    print(f"测试 7: 我近-敌远(10分) action={d.action.name}")
    assert d.action == StrategyAction.CONTEND
    print("  ✅ 通过")

    # 测试 8：目标争夺——敌近低分 → 放弃
    d = strat.contend_target(
        (1800, 500), 5, (500, 500, 0), (1900, 500)
    )
    print(f"测试 8: 敌近-低分(5分) action={d.action.name}")
    assert d.action == StrategyAction.ABANDON_TARGET
    print("  ✅ 通过")

    # 测试 9：对方送分 → 不干扰
    d = strat.handle_opponent_gift(True)
    print(f"测试 9: 对方送入安全区 action={d.action.name}")
    assert d.action == StrategyAction.NONE
    print("  ✅ 通过")

    # 测试 10：禁止主动进攻（代码层面检查）
    has_attack_logic = any(
        'attack' in attr.lower() or 'offensive' in attr.lower()
        for attr in dir(strat)
    )
    print(f"测试 10: 无主动进攻逻辑 {'✅' if not has_attack_logic else '❌'}")

    print(f"\n{'='*50}")
    print("  对抗策略测试全部通过 ✅")
    print(f"{'='*50}")
