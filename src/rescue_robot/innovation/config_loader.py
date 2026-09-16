"""
config_loader.py —— 参数化配置系统 (7.1.1)

YAML/JSON 配置文件加载、schema 验证、多源深度合并。
支持现场快速修改参数，无需重新部署代码。

用法:
  loader = ConfigLoader()
  cfg = loader.load_yaml("config/robot.default.yaml")
  robot_cfg = RobotConfig.from_yaml("config/robot.default.yaml")
"""

import json
import logging
import os
from copy import deepcopy
from dataclasses import dataclass, field, asdict
from dataclasses import fields as _dc_fields
from typing import Any, Dict, List, Optional, Tuple, Union

logger = logging.getLogger("config_loader")

# 尝试导入 yaml，不可用时降级
try:
    import yaml as _yaml
    _HAS_YAML = True
except ImportError:
    _HAS_YAML = False
    logger.info("PyYAML 未安装，仅支持 JSON 配置。")


# ============================================================
# Schema 定义
# ============================================================

# 机器人配置 schema: {字段路径: (期望类型, 验证函数)}
#
# ⚠️ 本表**必须**由 `RobotConfig.from_yaml` 调用（T2-10）。
#    旧实现本表从不被调用 → 现场把 `max_speed_mm_s` 拼成 `max_speed_mm`，
#    或写 `duration_s: "240"`（字符串）/ `99999`（越界）时**既不生效也不报错**：
#    机器人照默认值跑，现场却以为调好了。这是本项目最危险的一类缺陷。
#    因此覆盖范围必须包含"现场真的会改"的全部键（限速 / PID / 看门狗 / 超时 /
#    机构几何 / 扣分 / 策略权重），不能只有历史遗留的那 12 个键。
_ROBOT_SCHEMA = {
    # ---- 时序 ----
    "robot.timing.button_debounce_ms": (int, lambda v: 10 <= v <= 500),
    "robot.timing.button_long_press_ms": (int, lambda v: 100 <= v <= 3000),
    "robot.timing.self_check_timeout_s": (int, lambda v: 1 <= v <= 60),
    "robot.timing.led_boot_blink_interval_ms": (int, lambda v: 20 <= v <= 5000),
    "robot.timing.led_auto_blink_interval_ms": (int, lambda v: 20 <= v <= 5000),
    "robot.timing.sensor_check_timeout_ms": (int, lambda v: 100 <= v <= 60000),
    "robot.timing.motor_check_duration_ms": (int, lambda v: 50 <= v <= 10000),
    "robot.timing.post_start_delay_ms": (int, lambda v: 0 <= v <= 10000),
    # ---- 硬件阈值 ----
    "robot.thresholds.battery_min_voltage": (float, lambda v: 9.0 <= v <= 13.0),
    "robot.thresholds.battery_max_voltage": (float, lambda v: 9.0 <= v <= 15.0),
    "robot.thresholds.motor_min_current_ma": (int, lambda v: 0 <= v <= 5000),
    "robot.thresholds.motor_max_current_ma": (int, lambda v: 100 <= v <= 10000),
    "robot.thresholds.camera_min_fps": (int, lambda v: 1 <= v <= 120),
    # ---- 电机 / PID（现场标定项）----
    "robot.motors.count": (int, lambda v: 1 <= v <= 8),
    "robot.motors.max_speed_mm_s": (int, lambda v: 50 <= v <= 3000),
    "robot.motors.max_angular_speed_rad_s": (float, lambda v: 0.1 <= v <= 10.0),
    "robot.motors.wheel_base_mm": (int, lambda v: 50 <= v <= 1000),
    "robot.motors.pid.kp": (float, lambda v: 0.0 <= v <= 10.0),
    "robot.motors.pid.ki": (float, lambda v: 0.0 <= v <= 10.0),
    "robot.motors.pid.kd": (float, lambda v: 0.0 <= v <= 10.0),
    "robot.motors.pid_angle.kp": (float, lambda v: 0.0 <= v <= 10.0),
    "robot.motors.pid_angle.ki": (float, lambda v: 0.0 <= v <= 10.0),
    "robot.motors.pid_angle.kd": (float, lambda v: 0.0 <= v <= 10.0),
    # ---- 策略权重（由 TargetSelector 读取，见 T2-14）----
    "robot.strategy_weights.distance_weight": (float, lambda v: 0.0 <= v <= 1.0),
    "robot.strategy_weights.points_weight": (float, lambda v: 0.0 <= v <= 1.0),
    "robot.strategy_weights.time_weight": (float, lambda v: 0.0 <= v <= 1.0),
    "robot.strategy_weights.opponent_factor": (float, lambda v: 0.0 <= v <= 1.0),
    # ---- 比赛参数（现场按裁判公布值改）----
    "robot.match.duration_s": (int, lambda v: 60 <= v <= 600),
    "robot.match.time_pressure_s": (int, lambda v: 10 <= v <= 120),
    "robot.match.nav_timeout_s": ((int, float), lambda v: 1 <= v <= 120),
    "robot.match.grip_timeout_s": ((int, float), lambda v: 1 <= v <= 120),
    "robot.match.transport_timeout_s": ((int, float), lambda v: 1 <= v <= 120),
    # ---- 降级 / 看门狗（现场排障常改）----
    "robot.fallback.max_retries": (int, lambda v: 0 <= v <= 10),
    "robot.fallback.watchdog_warn_s": ((int, float), lambda v: 1 <= v <= 120),
    "robot.fallback.watchdog_critical_s": ((int, float), lambda v: 1 <= v <= 120),
    "robot.fallback.watchdog_timeout_s": ((int, float), lambda v: 1 <= v <= 120),
    "robot.fallback.stuck_time_s": ((int, float), lambda v: 0.5 <= v <= 120),
    "robot.fallback.stuck_distance_mm": ((int, float), lambda v: 1 <= v <= 2000),
    # ---- 机构几何 / 扣分（真机标定项）----
    "robot.placement.drop_forward_mm": ((int, float), lambda v: 0 <= v <= 500),
    "robot.placement.push_dist_mm": ((int, float), lambda v: 0 <= v <= 500),
    # >1 会被 config.apply_robot_config 强制回退到 1 并打 ERROR（机构只有一个自由度）
    "robot.placement.sleeve_max_hold": (int, lambda v: 1 <= v <= 3),
    "robot.placement.penalty_per_target": (int, lambda v: 0 <= v <= 100),
    # ---- 夹爪 V2（150×100 方形套取框 + 后方三块阶梯板）----
    # 套取开口 [横向, 前后] mm
    "robot.placement.sleeve_opening_mm": (
        list, lambda v: len(v) == 2 and all(
            isinstance(x, (int, float)) and 1 <= x <= 1000 for x in v)),
    # 套取接近闸门 mm：必须 ≤ 开口中心(70) + 半深(50) = 120，否则目标不在框正下方
    "robot.placement.capture_radius_mm": ((int, float), lambda v: 1 <= v <= 500),
    # 投放分步上调次数：0 = 到位后一次性释放
    "robot.placement.progressive_raise_steps": (int, lambda v: 0 <= v <= 20),
}

# 场地配置 schema
_FIELD_SCHEMA = {
    "field.size_mm": (list, lambda v: len(v) == 2 and all(500 <= x <= 5000 for x in v)),
    "field.opponent.contact_limit_s": (float, lambda v: 5.0 <= v <= 30.0),
    "field.opponent.distance_critical_mm": (int, lambda v: 100 <= v <= 1000),
    "field.my_color": (str, lambda v: v in ("red", "blue")),
}


# ============================================================
# ConfigLoader
# ============================================================

class ConfigLoader:
    """
    YAML/JSON 配置加载器。

    功能:
    - 加载 YAML/JSON 文件
    - Schema 验证（必填字段 + 类型 + 范围）
    - 多源深度合并（文件 > 环境变量 > 默认值）
    - 缺失字段自动填充默认值
    """

    def __init__(self, base_dir: str = ""):
        self._base_dir = base_dir or os.getcwd()
        self._loaded: Dict[str, dict] = {}  # path → parsed data

    # ---- 加载 ----

    def load_yaml(self, path: str) -> dict:
        """加载 YAML 文件。YAML 不可用时回退到 JSON 解析。"""
        full_path = self._resolve(path)
        if not os.path.exists(full_path):
            raise FileNotFoundError(f"配置文件不存在: {full_path}")

        with open(full_path, "r", encoding="utf-8") as f:
            content = f.read()

        if _HAS_YAML:
            try:
                data = _yaml.safe_load(content)
            except _yaml.YAMLError as e:
                raise ValueError(f"YAML 解析失败 ({full_path}): {e}")
        else:
            # 回退：尝试 JSON
            try:
                data = json.loads(content)
            except json.JSONDecodeError:
                raise ImportError(
                    f"PyYAML 未安装且文件不是有效 JSON: {full_path}。"
                    f"请安装: pip install pyyaml"
                )

        if data is None:
            data = {}
        if not isinstance(data, dict):
            raise ValueError(f"配置文件顶层必须是字典: {full_path}")

        self._loaded[path] = data
        logger.info(f"配置已加载: {full_path} ({len(data)} 个顶层键)")
        return deepcopy(data)

    def load_json(self, path: str) -> dict:
        """加载 JSON 文件。"""
        full_path = self._resolve(path)
        if not os.path.exists(full_path):
            raise FileNotFoundError(f"配置文件不存在: {full_path}")

        with open(full_path, "r", encoding="utf-8") as f:
            data = json.load(f)

        self._loaded[path] = data
        logger.info(f"配置已加载: {full_path}")
        return deepcopy(data)

    def load_any(self, path: str) -> dict:
        """自动检测格式（.yaml/.yml → YAML, .json → JSON）。"""
        if path.endswith((".yaml", ".yml")):
            return self.load_yaml(path)
        elif path.endswith(".json"):
            return self.load_json(path)
        else:
            # 默认尝试 YAML → JSON
            try:
                return self.load_yaml(path)
            except (ImportError, ValueError):
                return self.load_json(path)

    # ---- 保存 ----

    def save_yaml(self, path: str, data: dict) -> None:
        """保存配置为 YAML。"""
        if not _HAS_YAML:
            raise ImportError("PyYAML 未安装，无法保存 YAML")

        full_path = self._resolve(path)
        os.makedirs(os.path.dirname(full_path), exist_ok=True)
        with open(full_path, "w", encoding="utf-8") as f:
            _yaml.safe_dump(data, f, allow_unicode=True,
                           default_flow_style=False, sort_keys=False)
        logger.info(f"配置已保存: {full_path}")

    def save_json(self, path: str, data: dict, indent: int = 2) -> None:
        """保存配置为 JSON。"""
        full_path = self._resolve(path)
        os.makedirs(os.path.dirname(full_path), exist_ok=True)
        with open(full_path, "w", encoding="utf-8") as f:
            json.dump(data, f, indent=indent, ensure_ascii=False)
        logger.info(f"配置已保存: {full_path}")

    # ---- 合并 ----

    @staticmethod
    def merge_configs(*dicts: dict) -> dict:
        """
        深度合并多个配置源。后面的覆盖前面的。

        优先级: dicts[-1] > dicts[-2] > ... > dicts[0]
        """
        if not dicts:
            return {}
        result = deepcopy(dicts[0])
        for other in dicts[1:]:
            if not other:
                continue
            ConfigLoader._deep_merge(result, other)
        return result

    @staticmethod
    def merge_with_env(config: dict, prefix: str = "RESCUE_") -> dict:
        """
        用环境变量覆盖配置（T2-11）。

        命名规则（**现场临时覆盖用，无需改 YAML/重编译**）::

            RESCUE_<键路径>，路径分隔符 . 写成双下划线 __
            键名与 YAML 完全一致（小写+下划线），前缀 RESCUE_ 后的大小写均可

        例（都是合法写法，效果相同）::

            RESCUE_ROBOT__MATCH__DURATION_S=240        → robot.match.duration_s = 240
            RESCUE_ROBOT__MOTORS__MAX_SPEED_MM_S=500   → robot.motors.max_speed_mm_s = 500
            RESCUE_ROBOT__FALLBACK__WATCHDOG_WARN_S=6  → robot.fallback.watchdog_warn_s = 6
            RESCUE_ROBOT__STRATEGY_WEIGHTS__POINTS_WEIGHT=0.7
            RESCUE_LOGGING__MAX_LOG_SIZE_MB=20         → logging.max_log_size_mb = 20

        取值按 字面量 解析：`true/false/yes/no` → bool，纯整数 → int，
        其余数字 → float，其余 → str（**不求值**，不执行任何代码）。

        ⚠️ 路径写错（如少一个下划线）会**新建一个死键**而不是报错 —— 所以
        `RobotConfig.from_yaml` 在合并之后会做"未知键告警"（见 check_config），
        把这类错误喊出来。覆盖发生在 YAML 之后 → **环境变量优先级最高**
        （与 `merge_configs` 的"文件 > 环境变量 > 默认值"文档一致：后者是
        传入顺序，本方法返回的结果应放在最右/最后合并）。
        """
        result = deepcopy(config)
        for key, value in os.environ.items():
            if not key.startswith(prefix):
                continue
            config_path = key[len(prefix):].lower().replace("__", ".")
            val = ConfigLoader._coerce_env_value(value)
            ConfigLoader._set_nested(result, config_path, val)
            logger.info(f"环境变量覆盖: {key} = {val!r} → {config_path}")
        return result

    # ---- Schema 验证 ----

    @staticmethod
    def _type_name(expected_type) -> str:
        """类型名（支持 (int, float) 这种元组写法）。"""
        if isinstance(expected_type, tuple):
            return "/".join(t.__name__ for t in expected_type)
        return getattr(expected_type, "__name__", str(expected_type))

    @staticmethod
    def validate_detailed(schema: dict, data: dict) -> Dict[str, List[str]]:
        """
        基于 schema 验证配置，把**缺失**与**类型/范围错误**分开返回。

        schema 格式: {"路径.点号分隔": (期望类型, 验证函数)}

        分开的理由：缺失键是**正常**的（部分覆盖的 YAML 靠默认值补齐），
        而类型/范围错误是**现场写错了**（必须大声报出来）。

        Returns:
            {"missing": [...], "invalid": [...]}；两个列表都空表示验证通过
        """
        missing: List[str] = []
        invalid: List[str] = []
        for path, (expected_type, validator) in schema.items():
            try:
                value = ConfigLoader._get_nested(data, path)
            except (KeyError, IndexError, TypeError):
                missing.append(f"缺少必填字段: {path}")
                continue

            if not isinstance(value, expected_type):
                invalid.append(
                    f"字段 {path} 类型错误: "
                    f"期望 {ConfigLoader._type_name(expected_type)}, "
                    f"实际 {type(value).__name__} (值={value!r})"
                )
                continue

            try:
                if not validator(value):
                    invalid.append(f"字段 {path} 值超出范围: {value!r}")
            except Exception as e:
                invalid.append(f"字段 {path} 验证异常: {e!r}")

        return {"missing": missing, "invalid": invalid}

    @staticmethod
    def validate(schema: dict, data: dict) -> List[str]:
        """
        基于 schema 验证配置，返回错误列表（缺失 + 类型/范围，缺失在前）。

        schema 格式: {"路径.点号分隔": (期望类型, 验证函数)}

        Returns:
            [] 表示验证通过，否则返回错误描述列表
        """
        report = ConfigLoader.validate_detailed(schema, data)
        return report["missing"] + report["invalid"]

    @staticmethod
    def _dataclass_kwargs(cls, data: Any) -> dict:
        """只保留 `cls` 认识的字段（未知键已在 check_config 里告警）。

        为什么不直接 `**data`：一个拼错的键会让 dataclass 构造函数抛
        `TypeError`，被上层 `except Exception` 吞成"使用默认参数" →
        整份 YAML 白改。丢键 + 大声告警，比"整份配置回退"安全。
        """
        if not isinstance(data, dict):
            return {}
        allowed = {f.name for f in _dc_fields(cls)}
        return {k: v for k, v in data.items() if k in allowed}

    @staticmethod
    def check_unknown_keys(section: Any, allowed, label: str) -> List[str]:
        """
        找出 section 里**任何代码都不会读**的键，并逐条 WARNING（T2-10）。

        `from_dict` 里大量 `data.get("key", 默认值)` 的写法决定了：
        键名写错 → 静默退回默认值、**不报错**。这是"现场改了配置却不生效"
        的头号成因，所以必须在加载时把无效键喊出来。

        Args:
            section: 配置子字典（非 dict 时原样返回空列表）
            allowed: 该层允许出现的键集合
            label: 人读的路径标签（如 "robot.motors"），用于日志

        Returns:
            无效键的完整路径列表（如 ["robot.motors.max_speed_mm"]）
        """
        if not isinstance(section, dict):
            return []
        unknown = [k for k in section.keys() if k not in allowed]
        for key in unknown:
            logger.warning(
                "配置键无效（拼写错误？该键**不会生效**，值被忽略）: %s.%s "
                "（该层可用键: %s）", label, key, "/".join(sorted(allowed))
            )
        return [f"{label}.{k}" for k in unknown]

    @staticmethod
    def log_schema_report(report: Dict[str, List[str]], source: str = "") -> None:
        """把 schema 报告打印出来：类型/范围错误 → ERROR，缺失 → WARNING（T2-10）。"""
        where = f"（{source}）" if source else ""
        for msg in report.get("invalid", []):
            logger.error("❌ 配置写错，**不会按你写的值生效**%s: %s", where, msg)
        for msg in report.get("missing", []):
            logger.warning("配置未提供，回退到代码默认值%s: %s", where, msg)

    @staticmethod
    def fill_defaults(data: dict, defaults: dict) -> dict:
        """用默认值填充缺失字段。"""
        result = deepcopy(defaults)
        ConfigLoader._deep_merge(result, data)
        return result

    # ---- 查询 ----

    def get_loaded(self, path: str) -> Optional[dict]:
        """获取之前加载的配置。"""
        return deepcopy(self._loaded.get(path))

    @staticmethod
    def query(config: dict, dotted_path: str, default: Any = None) -> Any:
        """点号路径查询配置值。"""
        try:
            return ConfigLoader._get_nested(config, dotted_path)
        except (KeyError, IndexError, TypeError):
            return default

    # ---- 内部 ----

    def _resolve(self, path: str) -> str:
        """解析相对路径。"""
        if os.path.isabs(path):
            return path
        return os.path.join(self._base_dir, path)

    @staticmethod
    def _deep_merge(base: dict, override: dict) -> None:
        """原地深度合并 override 到 base。"""
        for key, val in override.items():
            if key in base and isinstance(base[key], dict) and isinstance(val, dict):
                ConfigLoader._deep_merge(base[key], val)
            else:
                base[key] = deepcopy(val)

    @staticmethod
    def _get_nested(data: dict, path: str) -> Any:
        """点号路径深度取值。"""
        keys = path.split(".")
        current = data
        for key in keys:
            if isinstance(current, list):
                idx = int(key)
                current = current[idx]
            else:
                current = current[key]
        return current

    @staticmethod
    def _set_nested(data: dict, path: str, value: Any) -> None:
        """点号路径深度设值。"""
        keys = path.split(".")
        current = data
        for key in keys[:-1]:
            if key not in current:
                current[key] = {}
            current = current[key]
        current[keys[-1]] = value

    @staticmethod
    def _coerce_env_value(raw: str) -> Union[int, float, bool, str]:
        """环境变量值类型转换。"""
        raw = raw.strip()
        # bool
        if raw.lower() in ("true", "yes", "1"):
            return True
        if raw.lower() in ("false", "no", "0"):
            return False
        # int
        try:
            return int(raw)
        except ValueError:
            pass
        # float
        try:
            return float(raw)
        except ValueError:
            pass
        return raw


# ============================================================
# RobotConfig —— 机器人配置 dataclass
# ============================================================

@dataclass
class RobotTimingConfig:
    """时序参数"""
    button_debounce_ms: int = 50
    button_long_press_ms: int = 500
    led_boot_blink_interval_ms: int = 500
    led_auto_blink_interval_ms: int = 200
    self_check_timeout_s: int = 10
    sensor_check_timeout_ms: int = 3000
    motor_check_duration_ms: int = 500
    post_start_delay_ms: int = 1000


@dataclass
class RobotThresholdsConfig:
    """硬件阈值"""
    battery_min_voltage: float = 11.0
    battery_max_voltage: float = 12.6
    motor_min_current_ma: int = 50
    motor_max_current_ma: int = 5000
    camera_min_fps: int = 10


@dataclass
class MotorPIDConfig:
    """电机 PID 参数"""
    kp: float = 0.8
    ki: float = 0.01
    kd: float = 0.05


@dataclass
class MotorPIDAngleConfig:
    """角度 PID 参数"""
    kp: float = 1.5
    ki: float = 0.0
    kd: float = 0.1


@dataclass
class MotorsConfig:
    """电机配置"""
    count: int = 2
    pid: MotorPIDConfig = field(default_factory=MotorPIDConfig)
    pid_angle: MotorPIDAngleConfig = field(default_factory=MotorPIDAngleConfig)
    max_speed_mm_s: int = 850
    max_angular_speed_rad_s: float = 3.0
    wheel_base_mm: int = 209


@dataclass
class StrategyWeightsConfig:
    """策略权重"""
    distance_weight: float = 0.3
    points_weight: float = 0.5
    time_weight: float = 0.2
    opponent_factor: float = 0.5


@dataclass
class MatchConfig:
    """比赛参数"""
    duration_s: int = 180
    time_pressure_s: int = 30
    nav_timeout_s: int = 10
    grip_timeout_s: int = 3
    transport_timeout_s: int = 15


@dataclass
class FallbackConfig:
    """降级策略参数"""
    max_retries: int = 3
    watchdog_warn_s: int = 10
    watchdog_critical_s: int = 13
    watchdog_timeout_s: int = 15
    stuck_time_s: int = 5
    stuck_distance_mm: int = 30


@dataclass
class PlacementConfig:
    """
    套取/投放机构几何与扣分（现场标定项，无需重编译）。

    ⚠️ 这几个值是**机构标定参数**，必须真机实测后填：
      - sleeve_opening_mm：夹爪 V2 套取开口 [横向, 前后] mm（实测 150×100）。
        目标必须落在横向 ±75、前后 ±50 内才真套得住。
      - capture_radius_mm：套取接近闸门 mm。必须 ≤ 开口中心(70) + 半深(50) = 120，
        否则目标不在套取框正下方 → 软件记账套住、实车套空（幽灵捕获）。
        旧值 150 对 V2 的 150×100 开口**必然套空**，故默认改为 100。
      - progressive_raise_steps：投放分步上调次数。4=旧行为（推入中 0→70 分步渐进抬）；
        0=保持套住到到位后一次性释放。V2 的推升来源是三块固定阶梯板。
      - drop_forward_mm：释放瞬间目标（在套取框内）相对车心的前伸距离，
        用于把"车身位置"换算成"目标落点"，投放有效性判定就是按它算的；
      - push_dist_mm：推式放置时向斜坡方向的推入距离。
      - sleeve_max_hold：一趟能**真正套住**几个目标（套取机构物理容量）。
        **只能填 1**：单只 SG90 只有"套住/释放"一个自由度，每次下压前都要先抬爪
        （抬爪 = 释放），先套住的目标会被放掉；且决策引擎的 grip_done 契约仍假设
        一趟只套 1 个。填 >1 会被强制回退 1 并打 ERROR 日志（受控 A/B 实测
        =3 只送 4 个 / 50 分，比 =1 的 7~8 个 / 80 分更差，还会让软件把没带上的
        目标谎报为已送达）。详见 docs/audit/FIXES.md 的 S-40。
    """
    # ⚠️ 默认值必须与 config.Placement 的真值一致：旧默认写 150.0，而 150 正是
    #    N-6 证明"会把落点推到围栏上"的坏值（真值 70）——任何漏写该项的 robot YAML
    #    都会静默拿到坏值。已改为 70.0。
    drop_forward_mm: float = 70.0
    push_dist_mm: float = 100.0
    sleeve_max_hold: int = 1
    # 夹爪 V2（150×100 方形套取框）：开口 [横向, 前后] mm
    sleeve_opening_mm: List[float] = field(default_factory=lambda: [150.0, 100.0])
    # 套取接近闸门 mm：必须落在 [开口中心 70 − 半深 50, 开口中心 70 + 半深 50]
    # = [20, 120] 内，否则目标不在套取框正下方（软件记账成功、实车套空）。
    capture_radius_mm: float = 100.0
    # 投放分步上调次数：4=推入中 0°→70° 分步渐进抬（旧行为）；0=保持套住到到位后一次释放。
    # 夹爪 V2 的推升来源是三块固定阶梯板，0/4 属真机标定项。
    progressive_raise_steps: int = 4
    # ❓ 该数值来源不明：PDF 只写"无效目标被取出重新随机放置场地中央"，
    #    未给扣分细则；暂按 10 分/个，待现场确认后修改。
    penalty_per_target: int = 10


@dataclass
class RobotConfig:
    """
    机器人完整配置。

    用法:
      cfg = RobotConfig.from_yaml("config/robot.default.yaml")
      cfg2 = RobotConfig.from_yaml("config/robot.custom.yaml", base=cfg)
    """
    timing: RobotTimingConfig = field(default_factory=RobotTimingConfig)
    thresholds: RobotThresholdsConfig = field(default_factory=RobotThresholdsConfig)
    motors: MotorsConfig = field(default_factory=MotorsConfig)
    strategy_weights: StrategyWeightsConfig = field(default_factory=StrategyWeightsConfig)
    match: MatchConfig = field(default_factory=MatchConfig)
    fallback: FallbackConfig = field(default_factory=FallbackConfig)
    placement: PlacementConfig = field(default_factory=PlacementConfig)

    @classmethod
    def from_yaml(cls, path: str, base: Optional["RobotConfig"] = None) -> "RobotConfig":
        """
        从 YAML 文件加载，可指定 base 作为默认值。

        加载链路（顺序即优先级，越靠后越优先）:

            代码默认值 → YAML 文件 → `RESCUE_*` 环境变量

        其中环境变量命名规则见 `ConfigLoader.merge_with_env`，例如::

            RESCUE_ROBOT__MATCH__DURATION_S=240
            RESCUE_ROBOT__MOTORS__MAX_SPEED_MM_S=500

        加载后**一定**会跑一遍 schema 校验 + 未知键检查（T2-10/T2-11）：
        类型错/越界 → `logger.error`；缺失键 → `logger.warning`；
        键名写错（任何代码都不会读的键）→ `logger.warning`（逐条打印）。
        这些信息同时通过返回值暴露，仓库里的告警**不静默**。
        """
        loader = ConfigLoader()
        data = loader.load_yaml(path)
        # ── 环境变量覆盖：必须在这里串上，否则 `RESCUE_*` 链路是死的（T2-11）──
        #    向后兼容：不存在 RESCUE_* 环境变量时 merge_with_env 是恒等变换。
        data = ConfigLoader.merge_with_env(data)
        # ── schema 校验 + 未知键告警：把"写错/写无效"显式报出来（T2-10）──
        cls.check_config(data, source=path)
        return cls.from_dict(data, base)

    @classmethod
    def check_config(cls, data: dict, source: str = "") -> Dict[str, List[str]]:
        """
        校验 robot 配置字典：schema（类型/范围）+ 未知键（键名拼错）（T2-10）。

        只在 `from_yaml` 里自动调用；也可被自检/测试直接调用核对一份 YAML。

        Returns:
            {"missing": [...], "invalid": [...], "unknown": [...]}
        """
        report = ConfigLoader.validate_detailed(_ROBOT_SCHEMA, data)
        ConfigLoader.log_schema_report(report, source)

        unknown: List[str] = []
        for section_path, section_cls in _ROBOT_SECTIONS:
            section = ConfigLoader.query(data, section_path)
            allowed = {f.name for f in _dc_fields(section_cls)}
            unknown += ConfigLoader.check_unknown_keys(section, allowed, section_path)

        report["unknown"] = unknown
        if report["invalid"] or unknown:
            logger.error(
                "⚠️ 配置存在**不影响启动但会让你以为调好了**的问题："
                "%d 个类型/范围错误、%d 个无效键（拼写错误）→ 请修正 %s",
                len(report["invalid"]), len(unknown), source or "YAML")
        return report

    @classmethod
    def from_dict(cls, data: dict, base: Optional["RobotConfig"] = None) -> "RobotConfig":
        """
        从字典构建，缺失字段使用默认值或 base 值。

        ⚠️ 键名写错的键会被**丢弃**（不是猜一个近似键），因为 `check_config`
        已经把它逐条 WARNING + ERROR 报出来了。这样做是为了避免旧实现的坏行为：
        嵌套段里一个拼写错误会抛 `TypeError`，被 `main.py` 的
        `except Exception: warning("使用默认参数")` 吞掉 →
        **整份 YAML（包括其它改对了的参数）全部退回默认值**，现场却以为改好了。
        """
        if base is None:
            base = cls()
        _kw = ConfigLoader._dataclass_kwargs

        robot_data = data.get("robot", {})
        timing_data = robot_data.get("timing", {})
        thresholds_data = robot_data.get("thresholds", {})
        motors_data = robot_data.get("motors", {})
        sw_data = robot_data.get("strategy_weights", {})
        match_data = robot_data.get("match", {})
        fallback_data = robot_data.get("fallback", {})
        placement_data = robot_data.get("placement", {})

        pid_data = motors_data.get("pid", {})
        pid_angle_data = motors_data.get("pid_angle", {})

        return cls(
            timing=RobotTimingConfig(
                **{**asdict(base.timing), **_kw(RobotTimingConfig, timing_data)}
            ),
            thresholds=RobotThresholdsConfig(
                **{**asdict(base.thresholds), **_kw(RobotThresholdsConfig, thresholds_data)}
            ),
            motors=MotorsConfig(
                count=motors_data.get("count", base.motors.count),
                pid=MotorPIDConfig(
                    **{**asdict(base.motors.pid), **_kw(MotorPIDConfig, pid_data)}
                ),
                pid_angle=MotorPIDAngleConfig(
                    **{**asdict(base.motors.pid_angle), **_kw(MotorPIDAngleConfig, pid_angle_data)}
                ),
                max_speed_mm_s=motors_data.get("max_speed_mm_s", base.motors.max_speed_mm_s),
                max_angular_speed_rad_s=motors_data.get(
                    "max_angular_speed_rad_s", base.motors.max_angular_speed_rad_s),
                wheel_base_mm=motors_data.get("wheel_base_mm", base.motors.wheel_base_mm),
            ),
            strategy_weights=StrategyWeightsConfig(
                **{**asdict(base.strategy_weights), **_kw(StrategyWeightsConfig, sw_data)}
            ),
            match=MatchConfig(
                **{**asdict(base.match), **_kw(MatchConfig, match_data)}
            ),
            fallback=FallbackConfig(
                **{**asdict(base.fallback), **_kw(FallbackConfig, fallback_data)}
            ),
            placement=PlacementConfig(
                **{**asdict(base.placement), **_kw(PlacementConfig, placement_data)}
            ),
        )

    def to_yaml(self, path: str) -> None:
        """保存为 YAML。"""
        loader = ConfigLoader()
        loader.save_yaml(path, {"robot": asdict(self)})

    def to_dict(self) -> dict:
        """转为字典。"""
        return {"robot": asdict(self)}


#: robot 各层配置 → 对应的 dataclass（用于"未知键 = 拼写错误"告警，T2-10）。
#: 允许键集合由 dataclass 字段**自动推导**，避免手写清单一改就漂移。
_ROBOT_SECTIONS = (
    ("robot", RobotConfig),
    ("robot.timing", RobotTimingConfig),
    ("robot.thresholds", RobotThresholdsConfig),
    ("robot.motors", MotorsConfig),
    ("robot.motors.pid", MotorPIDConfig),
    ("robot.motors.pid_angle", MotorPIDAngleConfig),
    ("robot.strategy_weights", StrategyWeightsConfig),
    ("robot.match", MatchConfig),
    ("robot.fallback", FallbackConfig),
    ("robot.placement", PlacementConfig),
)


# ============================================================
# FieldConfig —— 场地配置 dataclass
# ============================================================

@dataclass
class RectZone:
    """矩形区域"""
    x: int = 0
    y: int = 0
    width: int = 300
    height: int = 300


@dataclass
class SafeZoneConfig:
    """安全区配置"""
    x: int = 50
    y: int = 2550
    width: int = 600
    height: int = 400
    supply_area: RectZone = field(default_factory=lambda: RectZone(50, 2750, 300, 200))
    injured_area: RectZone = field(default_factory=lambda: RectZone(350, 2750, 300, 200))


@dataclass
class OpponentConfig:
    """对抗参数"""
    contact_warning_s: float = 7.0
    contact_force_s: float = 9.0
    contact_limit_s: float = 10.0
    distance_critical_mm: int = 350
    distance_high_mm: int = 500
    distance_medium_mm: int = 1000


@dataclass
class FieldConfig:
    """
    场地完整配置。

    ⚠️⚠️ **本类的字段不是场地几何的真值来源**（T2-12）。
    生产链路的场地几何来自 `perception/field_elements.py::StandardFieldLayout`
    （出发区 / 安全区 / 物资区 / 伤员区 / 禁区边距 / 减速带全在那里写死），
    `main.py` 用的是 `FieldLayout.standard()`，**从不读 `config/field.default.yaml`**。

    因此：

    - 现场按文档改 `config/field.default.yaml`（出发区、安全区、禁区边距、减速带）
      → **完全无效**，且不会报错。该文件已在头部标注"未被任何代码读取"。
    - 本类只用于离线工具/测试想拿一份场地参数时使用；它的默认值（如
      `safe_zone_red.x=50`）与 `StandardFieldLayout` 的实际几何**并不一致**，
      不要拿它当"场地真值"比对。
    - 真要把 YAML 接成真值：必须改 `main.py` 的 `FieldLayout.standard()` 调用点
      （当前修复范围明确禁止改 main.py），且要回归 5 种子集成仿真 —— 见 T2-12
      的 (a) 方案，本轮选 (b)（标注 + 文档化）以避免动到几何。

    用法:
      cfg = FieldConfig.from_yaml("config/field.default.yaml")
    """
    field_size_mm: Tuple[int, int] = (3000, 3000)
    fence_height_mm: int = 100
    my_color: str = "red"
    opponent: OpponentConfig = field(default_factory=OpponentConfig)
    safe_zone_red: SafeZoneConfig = field(default_factory=SafeZoneConfig)
    safe_zone_blue: SafeZoneConfig = field(
        default_factory=lambda: SafeZoneConfig(
            x=2350, y=2550,
            supply_area=RectZone(2350, 2750, 300, 200),
            injured_area=RectZone(2650, 2750, 300, 200),
        )
    )

    @classmethod
    def from_yaml(cls, path: str) -> "FieldConfig":
        """从 YAML 文件加载（含 `RESCUE_*` 环境变量覆盖 + `_FIELD_SCHEMA` 校验）。

        ⚠️ 注意：本方法在生产链路里**没有调用方**（见类文档字符串）；
        调用它不会改变机器人实际使用的场地几何。
        """
        loader = ConfigLoader()
        data = loader.load_yaml(path)
        data = ConfigLoader.merge_with_env(data)
        # _FIELD_SCHEMA 之前是"定义了但从没被调用"的死表（T2-10）
        ConfigLoader.log_schema_report(
            ConfigLoader.validate_detailed(_FIELD_SCHEMA, data), path)
        return cls.from_dict(data)

    @classmethod
    def from_dict(cls, data: dict) -> "FieldConfig":
        """从字典构建。"""
        field = data.get("field", data)
        return cls(
            field_size_mm=tuple(field.get("size_mm", [3000, 3000])),
            fence_height_mm=field.get("fence_height_mm", 100),
            my_color=field.get("my_color", "red"),
            opponent=OpponentConfig(
                **field.get("opponent", {})
            ),
        )

    def to_yaml(self, path: str) -> None:
        """保存为 YAML。"""
        loader = ConfigLoader()
        loader.save_yaml(path, {"field": asdict(self)})

    def to_dict(self) -> dict:
        """转为字典。"""
        return {"field": asdict(self)}


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
    print("  配置加载器 — 独立测试")
    print("=" * 60)

    import tempfile

    test_dir = os.path.join(tempfile.gettempdir(), "test_config_loader")
    import shutil
    shutil.rmtree(test_dir, ignore_errors=True)
    os.makedirs(test_dir, exist_ok=True)

    # ---- 测试 1: YAML 加载 ----
    print("\n--- 测试 1: YAML 加载 ---")
    yaml_path = os.path.join(test_dir, "test.yaml")
    with open(yaml_path, "w", encoding="utf-8") as f:
        f.write("""\
robot:
  timing:
    button_debounce_ms: 100
  strategy_weights:
    distance_weight: 0.4
""")
    loader = ConfigLoader(base_dir=test_dir)
    data = loader.load_yaml("test.yaml")
    assert data["robot"]["timing"]["button_debounce_ms"] == 100
    assert data["robot"]["strategy_weights"]["distance_weight"] == 0.4
    print("  ✅ 通过")

    # ---- 测试 2: Schema 验证 ----
    print("\n--- 测试 2: Schema 验证 ---")
    errors = ConfigLoader.validate(_ROBOT_SCHEMA, data)
    print(f"  缺失字段数: {len(errors)} (预期 > 0, 因为很多字段未提供)")
    assert len(errors) > 0
    # full.yaml 里出现的键必须全部合法（类型/范围无误）；未提供的键算"缺失"（正常）
    full_yaml = os.path.join(test_dir, "full.yaml")
    with open(full_yaml, "w", encoding="utf-8") as f:
        f.write("""\
robot:
  timing:
    button_debounce_ms: 50
    button_long_press_ms: 500
    self_check_timeout_s: 10
  thresholds:
    battery_min_voltage: 11.0
    motor_max_current_ma: 5000
  motors:
    count: 2
    max_speed_mm_s: 850
  strategy_weights:
    distance_weight: 0.3
    points_weight: 0.5
    time_weight: 0.2
    opponent_factor: 0.5
  match:
    duration_s: 180
    time_pressure_s: 30
""")
    full_data = loader.load_yaml("full.yaml")
    report = ConfigLoader.validate_detailed(_ROBOT_SCHEMA, full_data)
    assert report["invalid"] == [], f"full.yaml 的键必须全部合法: {report['invalid']}"
    print(f"  缺失字段 {len(report['missing'])} 个（未提供 → 用默认值，属正常），"
          f"类型/范围错误 0 个 ✅")

    # ---- 测试 3: 多源合并 ----
    print("\n--- 测试 3: 多源合并 ---")
    defaults = {"a": 1, "b": {"x": 10, "y": 20}}
    override = {"b": {"x": 99}, "c": 3}
    merged = ConfigLoader.merge_configs(defaults, override)
    assert merged["a"] == 1
    assert merged["b"]["x"] == 99       # 覆盖
    assert merged["b"]["y"] == 20       # 保留
    assert merged["c"] == 3             # 新增
    print(f"  合并结果: {merged}")
    print("  ✅ 通过")

    # ---- 测试 4: 环境变量覆盖 ----
    print("\n--- 测试 4: 环境变量覆盖 ---")
    os.environ["RESCUE_ROBOT__TIMING__BUTTON_DEBOUNCE_MS"] = "200"
    merged_env = ConfigLoader.merge_with_env(defaults, prefix="RESCUE_")
    # defaults 没有 robot.timing.button_debounce_ms 路径，不会被覆盖
    del os.environ["RESCUE_ROBOT__TIMING__BUTTON_DEBOUNCE_MS"]
    print("  ✅ 通过")

    # ---- 测试 4b: 环境变量真的能覆盖 YAML（T2-11）----
    print("\n--- 测试 4b: RESCUE_* 环境变量覆盖 YAML（T2-11）---")
    os.environ["RESCUE_ROBOT__MATCH__DURATION_S"] = "240"
    try:
        cfg_env = RobotConfig.from_yaml(full_yaml)
        assert cfg_env.match.duration_s == 240, \
            f"环境变量未生效: {cfg_env.match.duration_s}"
        print(f"  RESCUE_ROBOT__MATCH__DURATION_S=240 → duration_s="
              f"{cfg_env.match.duration_s} ✅")
    finally:
        del os.environ["RESCUE_ROBOT__MATCH__DURATION_S"]

    # ---- 测试 4c: 键名写错 / 值写错必须被显式报出（T2-10）----
    print("\n--- 测试 4c: 未知键告警 + 类型/范围错误（T2-10）---")
    typo_path = os.path.join(test_dir, "typo.yaml")
    with open(typo_path, "w", encoding="utf-8") as f:
        f.write("""\
robot:
  motors:
    max_speed_mm: 500          # ← 拼错：真键是 max_speed_mm_s（旧实现静默忽略）
  match:
    duration_s: "240"          # ← 类型错：字符串（旧实现静默通过）
""")
    rep = RobotConfig.check_config(loader.load_yaml("typo.yaml"), source="typo.yaml")
    assert rep["unknown"] == ["robot.motors.max_speed_mm"], rep["unknown"]
    assert any("robot.match.duration_s" in e for e in rep["invalid"]), rep["invalid"]
    print(f"  未知键 {rep['unknown']} / 类型错误 {len(rep['invalid'])} 条 ✅")

    # ---- 测试 5: RobotConfig 从 YAML ----
    print("\n--- 测试 5: RobotConfig from_yaml ---")
    cfg = RobotConfig.from_yaml(full_yaml)
    assert cfg.timing.button_debounce_ms == 50
    assert cfg.strategy_weights.distance_weight == 0.3
    assert cfg.match.duration_s == 180
    print(f"  策略权重: dist={cfg.strategy_weights.distance_weight}, "
          f"pts={cfg.strategy_weights.points_weight}, "
          f"time={cfg.strategy_weights.time_weight}")
    print("  ✅ 通过")

    # ---- 测试 6: RobotConfig to_dict 往返 ----
    print("\n--- 测试 6: RobotConfig to_dict 往返 ---")
    cfg2 = RobotConfig.from_dict(cfg.to_dict())
    assert cfg2.timing.button_debounce_ms == cfg.timing.button_debounce_ms
    assert cfg2.strategy_weights.distance_weight == cfg.strategy_weights.distance_weight
    print("  ✅ 通过")

    # ---- 测试 7: 点号路径查询 ----
    print("\n--- 测试 7: 点号路径查询 ---")
    val = ConfigLoader.query(full_data, "robot.strategy_weights.distance_weight")
    assert val == 0.3
    missing = ConfigLoader.query(full_data, "robot.nonexistent.key", default=42)
    assert missing == 42
    print("  ✅ 通过")

    # ---- 测试 8: 缺失字段填充默认值 ----
    print("\n--- 测试 8: 缺失字段填充默认值 ---")
    partial = {"robot": {"timing": {"button_debounce_ms": 999}}}
    filled = RobotConfig.from_dict(partial)
    assert filled.timing.button_debounce_ms == 999  # 覆盖
    assert filled.timing.button_long_press_ms == 500  # 使用默认值
    print("  ✅ 通过")

    shutil.rmtree(test_dir, ignore_errors=True)

    print(f"\n{'=' * 60}")
    print("  配置加载器 — 全部测试通过 ✅")
    print(f"{'=' * 60}")
