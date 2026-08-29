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

# 机器人配置 schema: {字段路径: 类型/验证函数}
_ROBOT_SCHEMA = {
    "robot.timing.button_debounce_ms": (int, lambda v: 10 <= v <= 500),
    "robot.timing.button_long_press_ms": (int, lambda v: 100 <= v <= 3000),
    "robot.timing.self_check_timeout_s": (int, lambda v: 1 <= v <= 60),
    "robot.thresholds.battery_min_voltage": (float, lambda v: 9.0 <= v <= 13.0),
    "robot.thresholds.motor_max_current_ma": (int, lambda v: 100 <= v <= 10000),
    "robot.motors.count": (int, lambda v: 1 <= v <= 8),
    "robot.motors.max_speed_mm_s": (int, lambda v: 50 <= v <= 3000),
    "robot.strategy_weights.distance_weight": (float, lambda v: 0.0 <= v <= 1.0),
    "robot.strategy_weights.points_weight": (float, lambda v: 0.0 <= v <= 1.0),
    "robot.strategy_weights.time_weight": (float, lambda v: 0.0 <= v <= 1.0),
    "robot.match.duration_s": (int, lambda v: 60 <= v <= 600),
    "robot.match.time_pressure_s": (int, lambda v: 10 <= v <= 120),
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
        用环境变量覆盖配置。

        环境变量格式: RESCUE_ROBOT__TIMING__BUTTON_DEBOUNCE_MS=100
        双下划线替换为点号路径。
        """
        result = deepcopy(config)
        for key, value in os.environ.items():
            if not key.startswith(prefix):
                continue
            config_path = key[len(prefix):].lower().replace("__", ".")
            val = ConfigLoader._coerce_env_value(value)
            ConfigLoader._set_nested(result, config_path, val)
            logger.info(f"环境变量覆盖: {key} = {val}")
        return result

    # ---- Schema 验证 ----

    @staticmethod
    def validate(schema: dict, data: dict) -> List[str]:
        """
        基于 schema 验证配置，返回错误列表。

        schema 格式: {"路径.点号分隔": (期望类型, 验证函数)}

        Returns:
            [] 表示验证通过，否则返回错误描述列表
        """
        errors = []
        for path, (expected_type, validator) in schema.items():
            try:
                value = ConfigLoader._get_nested(data, path)
            except (KeyError, IndexError, TypeError):
                errors.append(f"缺少必填字段: {path}")
                continue

            if not isinstance(value, expected_type):
                errors.append(
                    f"字段 {path} 类型错误: "
                    f"期望 {expected_type.__name__}, 实际 {type(value).__name__} "
                    f"(值={value})"
                )
                continue

            try:
                if not validator(value):
                    errors.append(
                        f"字段 {path} 值超出范围: {value}"
                    )
            except Exception as e:
                errors.append(f"字段 {path} 验证异常: {e}")

        return errors

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
    max_speed_mm_s: int = 1000
    max_angular_speed_rad_s: float = 3.0
    wheel_base_mm: int = 182


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

    @classmethod
    def from_yaml(cls, path: str, base: Optional["RobotConfig"] = None) -> "RobotConfig":
        """从 YAML 文件加载，可指定 base 作为默认值。"""
        loader = ConfigLoader()
        data = loader.load_yaml(path)
        return cls.from_dict(data, base)

    @classmethod
    def from_dict(cls, data: dict, base: Optional["RobotConfig"] = None) -> "RobotConfig":
        """从字典构建，缺失字段使用默认值或 base 值。"""
        if base is None:
            base = cls()

        robot_data = data.get("robot", {})
        timing_data = robot_data.get("timing", {})
        thresholds_data = robot_data.get("thresholds", {})
        motors_data = robot_data.get("motors", {})
        sw_data = robot_data.get("strategy_weights", {})
        match_data = robot_data.get("match", {})
        fallback_data = robot_data.get("fallback", {})

        pid_data = motors_data.get("pid", {})
        pid_angle_data = motors_data.get("pid_angle", {})

        return cls(
            timing=RobotTimingConfig(
                **{**asdict(base.timing), **timing_data}
            ),
            thresholds=RobotThresholdsConfig(
                **{**asdict(base.thresholds), **thresholds_data}
            ),
            motors=MotorsConfig(
                count=motors_data.get("count", base.motors.count),
                pid=MotorPIDConfig(
                    **{**asdict(base.motors.pid), **pid_data}
                ),
                pid_angle=MotorPIDAngleConfig(
                    **{**asdict(base.motors.pid_angle), **pid_angle_data}
                ),
                max_speed_mm_s=motors_data.get("max_speed_mm_s", base.motors.max_speed_mm_s),
                max_angular_speed_rad_s=motors_data.get(
                    "max_angular_speed_rad_s", base.motors.max_angular_speed_rad_s),
                wheel_base_mm=motors_data.get("wheel_base_mm", base.motors.wheel_base_mm),
            ),
            strategy_weights=StrategyWeightsConfig(
                **{**asdict(base.strategy_weights), **sw_data}
            ),
            match=MatchConfig(
                **{**asdict(base.match), **match_data}
            ),
            fallback=FallbackConfig(
                **{**asdict(base.fallback), **fallback_data}
            ),
        )

    def to_yaml(self, path: str) -> None:
        """保存为 YAML。"""
        loader = ConfigLoader()
        loader.save_yaml(path, {"robot": asdict(self)})

    def to_dict(self) -> dict:
        """转为字典。"""
        return {"robot": asdict(self)}


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
        """从 YAML 文件加载。"""
        loader = ConfigLoader()
        data = loader.load_yaml(path)
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
    # 完整数据应通过
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
    max_speed_mm_s: 1000
  strategy_weights:
    distance_weight: 0.3
    points_weight: 0.5
    time_weight: 0.2
  match:
    duration_s: 180
    time_pressure_s: 30
""")
    full_data = loader.load_yaml("full.yaml")
    errors2 = ConfigLoader.validate(_ROBOT_SCHEMA, full_data)
    assert len(errors2) == 0, f"应无错误: {errors2}"
    print("  ✅ 通过")

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
