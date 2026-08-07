"""
path_planner.py —— 路径规划

A* 全局规划 + DWA 局部避障。

场地建模：
  - 3000×3000mm → 60×60 网格（CELL_SIZE = 50mm）
  - 每个网格单元存储代价（0=自由, 255=障碍/禁区）
  - 对方机器人周围 350mm 膨胀为高风险区
"""

import logging
import math
import time
from dataclasses import dataclass, field
from typing import List, Optional, Set, Tuple, Callable

logger = logging.getLogger("path_planner")


# ============================================================
# 常量和类型
# ============================================================

CELL_SIZE_MM = 50          # 网格分辨率
GRID_SIZE = 60             # 3000 / 50 = 60

# 代价常量
COST_FREE = 0
COST_UNKNOWN = 50
COST_NEAR_OBSTACLE = 150
COST_OBSTACLE = 255
COST_OPPONENT = 200        # 对方机器人区域
COST_FORBIDDEN = 255       # 禁区

Point = Tuple[float, float]  # (x_mm, y_mm)


@dataclass
class PlanResult:
    """路径规划结果"""
    success: bool = False
    waypoints: List[Point] = field(default_factory=list)
    total_cost: float = 0.0
    planning_time_ms: float = 0.0
    path_length_mm: float = 0.0


# ============================================================
# 代价地图
# ============================================================

class CostMap:
    """
    代价地图。

    60×60 网格，每个 cell 值 0-255（0=自由，255=不可通行）。
    """

    def __init__(self, field_size_mm: float = 3000.0):
        self._size = field_size_mm
        self._grid: List[List[int]] = [
            [COST_FREE for _ in range(GRID_SIZE)]
            for _ in range(GRID_SIZE)
        ]
        self._dynamic_obstacles: List[Tuple[float, float, float]] = []  # (x, y, radius_mm)

    def clear_dynamic(self) -> None:
        """清除动态障碍（对方机器人等）"""
        for x, y, r in self._dynamic_obstacles:
            self._clear_circle(x, y, r)
        self._dynamic_obstacles.clear()

    def add_obstacle_circle(self, cx_mm: float, cy_mm: float,
                            radius_mm: float, cost: int = COST_OBSTACLE) -> None:
        """添加圆形障碍物"""
        self._dynamic_obstacles.append((cx_mm, cy_mm, radius_mm))
        self._fill_circle(cx_mm, cy_mm, radius_mm, cost)

    def add_forbidden_rect(self, x_mm: float, y_mm: float,
                           w_mm: float, h_mm: float) -> None:
        """添加矩形禁区"""
        self._fill_rect(x_mm, y_mm, w_mm, h_mm, COST_FORBIDDEN)

    def add_obstacle_rect(self, x_mm: float, y_mm: float,
                          w_mm: float, h_mm: float,
                          cost: int = COST_OBSTACLE) -> None:
        """添加矩形障碍"""
        self._fill_rect(x_mm, y_mm, w_mm, h_mm, cost)

    def is_free(self, x_mm: float, y_mm: float) -> bool:
        """检查点是否可通行"""
        gx, gy = self._to_grid(x_mm, y_mm)
        if not self._in_bounds_grid(gx, gy):
            return False
        return self._grid[gy][gx] < COST_OBSTACLE

    def get_cost(self, x_mm: float, y_mm: float) -> int:
        """获取点的代价"""
        gx, gy = self._to_grid(x_mm, y_mm)
        if not self._in_bounds_grid(gx, gy):
            return COST_FORBIDDEN
        return self._grid[gy][gx]

    def _to_grid(self, x_mm: float, y_mm: float) -> Tuple[int, int]:
        gx = max(0, min(GRID_SIZE - 1, int(x_mm / CELL_SIZE_MM)))
        gy = max(0, min(GRID_SIZE - 1, int(y_mm / CELL_SIZE_MM)))
        return (gx, gy)

    def _in_bounds_grid(self, gx: int, gy: int) -> bool:
        return 0 <= gx < GRID_SIZE and 0 <= gy < GRID_SIZE

    def _fill_circle(self, cx_mm: float, cy_mm: float,
                     radius_mm: float, cost: int) -> None:
        cx_g, cy_g = self._to_grid(cx_mm, cy_mm)
        r_g = max(1, int(radius_mm / CELL_SIZE_MM))
        for gy in range(max(0, cy_g - r_g), min(GRID_SIZE, cy_g + r_g + 1)):
            for gx in range(max(0, cx_g - r_g), min(GRID_SIZE, cx_g + r_g + 1)):
                dist = math.sqrt((gx - cx_g) ** 2 + (gy - cy_g) ** 2)
                if dist <= r_g:
                    self._grid[gy][gx] = max(self._grid[gy][gx], cost)

    def _fill_rect(self, x_mm: float, y_mm: float,
                   w_mm: float, h_mm: float, cost: int) -> None:
        gx1, gy1 = self._to_grid(x_mm, y_mm)
        gx2, gy2 = self._to_grid(x_mm + w_mm, y_mm + h_mm)
        for gy in range(gy1, gy2 + 1):
            for gx in range(gx1, gx2 + 1):
                if self._in_bounds_grid(gx, gy):
                    self._grid[gy][gx] = max(self._grid[gy][gx], cost)

    def _clear_circle(self, cx_mm: float, cy_mm: float, radius_mm: float) -> None:
        """清除圆形区域（重置为 FREE）"""
        cx_g, cy_g = self._to_grid(cx_mm, cy_mm)
        r_g = max(1, int(radius_mm / CELL_SIZE_MM))
        for gy in range(max(0, cy_g - r_g), min(GRID_SIZE, cy_g + r_g + 1)):
            for gx in range(max(0, cx_g - r_g), min(GRID_SIZE, cx_g + r_g + 1)):
                dist = math.sqrt((gx - cx_g) ** 2 + (gy - cy_g) ** 2)
                if dist <= r_g:
                    self._grid[gy][gx] = COST_FREE


# ============================================================
# A* 全局规划器
# ============================================================

class AStarPlanner:
    """
    A* 网格路径规划。

    使用 8 邻域搜索，启发函数为欧几里得距离。
    """

    # 8 邻域 (dx, dy, cost_multiplier)
    _NEIGHBORS = [
        (1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
        (1, 1, 1.414), (-1, 1, 1.414), (1, -1, 1.414), (-1, -1, 1.414),
    ]

    def __init__(self, cost_map: CostMap):
        self._cost_map = cost_map

    def plan(self, start_mm: Point, goal_mm: Point) -> PlanResult:
        """
        A* 规划从 start 到 goal 的路径。

        Returns:
            PlanResult: 路径结果
        """
        t0 = time.time()

        start_g = self._cost_map._to_grid(*start_mm)
        goal_g = self._cost_map._to_grid(*goal_mm)

        # 起点或终点不可通行
        if not self._cost_map.is_free(*start_mm):
            logger.warning(f"A*: 起点 ({start_mm[0]:.0f}, {start_mm[1]:.0f}) 不可通行")
            return PlanResult(success=False)

        if not self._cost_map.is_free(*goal_mm):
            logger.warning(f"A*: 终点 ({goal_mm[0]:.0f}, {goal_mm[1]:.0f}) 不可通行")
            return PlanResult(success=False)

        # A* 核心
        open_set: Set[Tuple[int, int]] = {start_g}
        came_from: dict = {}
        g_score = {start_g: 0.0}
        f_score = {start_g: self._heuristic(start_g, goal_g)}

        while open_set:
            # 取 f 值最小的节点
            current = min(open_set, key=lambda n: f_score.get(n, float('inf')))

            if current == goal_g:
                # 找到路径，回溯
                path = self._reconstruct_path(came_from, current)
                length = self._path_length(path)
                dt = (time.time() - t0) * 1000
                logger.debug(f"A* 成功: {len(path)} waypoints, "
                             f"{length:.0f}mm, {dt:.1f}ms")
                return PlanResult(
                    success=True,
                    waypoints=path,
                    total_cost=g_score[current],
                    planning_time_ms=dt,
                    path_length_mm=length,
                )

            open_set.remove(current)

            for dx, dy, cost_mult in self._NEIGHBORS:
                nx, ny = current[0] + dx, current[1] + dy
                neighbor = (nx, ny)

                if not self._cost_map._in_bounds_grid(nx, ny):
                    continue

                # 代价 = 移动代价 + 网格代价
                grid_cost = self._cost_map._grid[ny][nx]
                if grid_cost >= COST_OBSTACLE:
                    continue

                move_cost = CELL_SIZE_MM * cost_mult
                # 高风险区域增加代价
                if grid_cost >= COST_OPPONENT:
                    move_cost *= 5.0
                elif grid_cost >= COST_NEAR_OBSTACLE:
                    move_cost *= 2.0

                tentative_g = g_score[current] + move_cost

                if tentative_g < g_score.get(neighbor, float('inf')):
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f_score[neighbor] = tentative_g + self._heuristic(neighbor, goal_g)
                    open_set.add(neighbor)

        dt = (time.time() - t0) * 1000
        logger.warning(f"A* 失败: 起点→终点无路径 ({dt:.1f}ms)")
        return PlanResult(success=False, planning_time_ms=dt)

    def _reconstruct_path(self, came_from: dict,
                          current: Tuple[int, int]) -> List[Point]:
        """回溯路径"""
        path = []
        while current in came_from:
            x_mm = current[0] * CELL_SIZE_MM + CELL_SIZE_MM / 2
            y_mm = current[1] * CELL_SIZE_MM + CELL_SIZE_MM / 2
            path.append((x_mm, y_mm))
            current = came_from[current]
        path.reverse()
        return path

    def _heuristic(self, a: Tuple[int, int], b: Tuple[int, int]) -> float:
        """启发函数：欧几里得距离 (mm)"""
        dx = (a[0] - b[0]) * CELL_SIZE_MM
        dy = (a[1] - b[1]) * CELL_SIZE_MM
        return math.sqrt(dx * dx + dy * dy)

    def _path_length(self, path: List[Point]) -> float:
        if len(path) < 2:
            return 0.0
        total = 0.0
        for i in range(1, len(path)):
            dx = path[i][0] - path[i - 1][0]
            dy = path[i][1] - path[i - 1][1]
            total += math.sqrt(dx * dx + dy * dy)
        return total


# ============================================================
# 局部避障（简化 DWA）
# ============================================================

class LocalPlanner:
    """
    局部避障规划器。

    在当前速度附近采样，评估每个候选轨迹的代价：
    - 距全局路径的偏差
    - 距障碍物的距离
    - 速度（越快越好）
    """

    def __init__(self,
                 max_linear_speed: float = 1000.0,   # mm/s
                 max_angular_speed: float = 3.0,     # rad/s
                 num_samples: int = 50):
        self._max_v = max_linear_speed
        self._max_w = max_angular_speed
        self._num_samples = num_samples

        # 采样空间
        self._v_samples: List[float] = []
        self._w_samples: List[float] = []

    def plan(self,
             current_pose: Tuple[float, float, float],  # (x, y, theta)
             current_vel: Tuple[float, float],           # (v, w)
             global_path: List[Point],
             cost_map: CostMap,
             dt: float = 0.5) -> Tuple[float, float]:
        """
        选择最优速度指令。

        Returns:
            (linear_velocity_mm_s, angular_velocity_rad_s)
        """
        best_v, best_w = 0.0, 0.0
        best_score = float('-inf')

        v_now, w_now = current_vel

        # 动态窗口
        v_window = max(0.0, v_now - 500 * dt), min(self._max_v, v_now + 500 * dt)
        w_window = max(-self._max_w, w_now - 1.5 * dt), min(self._max_w, w_now + 1.5 * dt)

        for _ in range(self._num_samples):
            v = v_window[0] + (v_window[1] - v_window[0]) * (_ / self._num_samples)
            for sign in [1, -1]:
                w = sign * (w_window[1] - w_window[0]) * (_ / (self._num_samples * 2))

                score = self._evaluate_trajectory(
                    v, w, current_pose, global_path, cost_map, dt
                )
                if score > best_score:
                    best_score = score
                    best_v = v
                    best_w = w

        return (best_v, best_w)

    def _evaluate_trajectory(self, v: float, w: float,
                             pose: Tuple[float, float, float],
                             global_path: List[Point],
                             cost_map: CostMap, dt: float) -> float:
        """评估轨迹代价"""
        x, y, theta = pose
        score = 0.0

        # 模拟前向
        steps = 5
        for i in range(1, steps + 1):
            t = dt * i / steps
            sim_x = x + v * math.cos(theta) * t
            sim_y = y + v * math.sin(theta) * t

            # 检查碰撞
            if not cost_map.is_free(sim_x, sim_y):
                return float('-inf')

            # 距离代价（距障碍物越远越好）
            grid_cost = cost_map.get_cost(sim_x, sim_y)
            score -= grid_cost * 0.1

        # 速度奖励（越快越好）
        score += v / self._max_v * 50.0

        # 朝向全局路径奖励
        if global_path:
            target = global_path[min(2, len(global_path) - 1)]
            dx = target[0] - x
            dy = target[1] - y
            target_angle = math.atan2(dy, dx)
            angle_diff = abs(theta - target_angle)
            angle_diff = min(angle_diff, 2 * math.pi - angle_diff)
            score += (1.0 - angle_diff / math.pi) * 30.0

        return score
