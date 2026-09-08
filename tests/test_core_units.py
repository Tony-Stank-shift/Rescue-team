"""
test_core_units.py —— 核心逻辑单元测试（不依赖真机硬件）

覆盖：坐标转换、串口帧解析、目标分类、场地布局、系统自检(Mock)。
运行：PYTHONPATH=src python3 -m pytest tests/test_core_units.py -q
"""

import math


# ============================================================
# 1. 坐标转换（ChassisInterface）
# ============================================================

def test_odom_to_upper_forward_plus_y():
    from rescue_robot.hardware.chassis_interface import ChassisInterface
    # 初始朝 +Y（场地坐标 前方=+Y），起始位姿 (0,0)
    ci = ChassisInterface(start_x_mm=0, start_y_mm=0, start_theta_rad=math.pi / 2)
    # 下位机向前 1m（x_f=1, y_f=0）→ 上层 = (0, +1000)
    x, y, theta = ci.odom_to_upper(1.0, 0.0, 0.0)
    assert abs(x - 0) < 1e-6
    assert abs(y - 1000) < 1e-6
    # 下位机向左 0.5m（y_f=0.5）→ 上层 x 增加（theta0=+Y 时左 = +X）
    x2, y2, _ = ci.odom_to_upper(0.0, 0.5, 0.0)
    assert x2 < 0 and abs(y2 - 0) < 1e-6  # 朝 +Y，左侧 = -X


def test_odom_to_upper_with_start_offset():
    from rescue_robot.hardware.chassis_interface import ChassisInterface
    # 初始朝 +X，起点 (150, 150)
    ci = ChassisInterface(start_x_mm=150, start_y_mm=150, start_theta_rad=0.0)
    x, y, theta = ci.odom_to_upper(0.3, 0.0, 0.0)  # 前进 0.3m
    assert abs(x - (150 + 300)) < 1e-6
    assert abs(y - 150) < 1e-6
    assert abs(theta - 0.0) < 1e-9


def test_velocity_and_start_command():
    from rescue_robot.hardware.chassis_interface import ChassisInterface
    assert ChassisInterface.velocity_to_command(500.0, 1.0) == "VEL,500,1000"
    assert ChassisInterface.velocity_to_command(0.0, -2.0) == "VEL,0,-2000"
    assert ChassisInterface.start_command() == "START"


# ============================================================
# 2. 串口帧解析（SerialChassis）
# ============================================================

def test_parse_frame_valid_odom():
    from rescue_robot.hardware.serial_chassis import SerialChassis
    sc = SerialChassis(port="/dev/ttytest")
    frame = sc.parse_frame(
        "ODOM,0.523100,0.012400,0.083000,174321,180745,0.401200,0.419800")
    assert frame is not None
    assert frame['x_m'] == 0.523100
    assert frame['y_m'] == 0.012400
    assert frame['theta_rad'] == 0.083000
    assert frame['encL'] == 174321
    assert frame['encR'] == 180745
    assert frame['vL'] == 0.401200
    assert frame['vR'] == 0.419800


def test_parse_frame_rejects_bad():
    from rescue_robot.hardware.serial_chassis import SerialChassis
    sc = SerialChassis(port="/dev/ttytest")
    # 字段不足
    assert sc.parse_frame("ODOM,1,2,3") is None
    # 非 ODOM 前缀（ACK/EVENT/乱码不得误解析）
    assert sc.parse_frame("ACK,START") is None
    # 数值非法
    assert sc.parse_frame("ODOM,a,b,c,d,e,f,g") is None


def test_parse_imu():
    from rescue_robot.hardware.serial_chassis import SerialChassis
    sc = SerialChassis(port="/dev/ttytest")
    frame = sc.parse_imu(
        "IMU,1000,5,256,512,-128,100,-200,50,320")
    assert frame is not None
    assert frame['ax_mg'] == 256
    assert frame['gz_mrad_s'] == 50
    assert frame['temp_cC'] == 320
    # 字段数非 10 拒绝
    assert sc.parse_imu("IMU,1,2,3") is None


# ============================================================
# 3. 目标分类（TargetClassifier）
# ============================================================

def _det(color, shape):
    from rescue_robot.perception.target_types import Detection
    return Detection(color=color, shape=shape, bbox=(0, 0, 40, 40), confidence=0.95)


def test_classify_four_types():
    from rescue_robot.perception.classification import TargetClassifier
    from rescue_robot.perception.target_types import (
        TargetColor, TargetShape, TargetType, CompetitionPhase)
    clf = TargetClassifier(CompetitionPhase.PRELIMINARY)
    cases = [
        (TargetColor.GREEN, TargetShape.CUBE, TargetType.REGULAR_SUPPLY),
        (TargetColor.BLACK, TargetShape.TRIANGULAR_PYRAMID, TargetType.CORE_SUPPLY),
        (TargetColor.ORANGE, TargetShape.CUBOID, TargetType.INJURED),
        (TargetColor.LIGHT_BLUE, TargetShape.CUBE, TargetType.DANGEROUS),
    ]
    for color, shape, expects in cases:
        det = clf.classify(_det(color, shape))
        assert det is not None, f"{color}x{shape} 未分类"
        assert det.info.type == expects


def test_classify_unknown_returns_none():
    from rescue_robot.perception.classification import TargetClassifier
    from rescue_robot.perception.target_types import (
        TargetColor, TargetShape, CompetitionPhase)
    clf = TargetClassifier(CompetitionPhase.PRELIMINARY)
    # 初赛没有"黄色正方体"
    det = clf.classify(_det(TargetColor.YELLOW, TargetShape.CUBE))
    assert det is None


# ============================================================
# 4. 场地布局（FieldLayout）
# ============================================================

def test_field_layout_new_safe_zone():
    from rescue_robot.perception.field_elements import (
        FieldLayout, FieldElementType, SafeZoneColor)
    f = FieldLayout.standard()

    def _sz(color):
        return [e for e in f.elements
                if e.type == FieldElementType.SAFE_ZONE
                and e.metadata.get("color_enum") == color]
    # 红色安全区（顶）内部 x[1200,1800] y[2670,2970]
    red = _sz(SafeZoneColor.RED)
    assert len(red) == 1
    r = red[0].region
    assert r.x == 1200 and r.x_max == 1800
    assert r.y == 2670 and r.y_max == 2970
    # 分区 300×300
    supplies = [e for e in f.elements
                if e.type == FieldElementType.SUPPLY_AREA
                and e.metadata.get("safe_zone") == SafeZoneColor.RED]
    assert len(supplies) == 1
    assert supplies[0].region.width == 300 and supplies[0].region.height == 300
    # 蓝色安全区（底）y[30,330]
    blue = _sz(SafeZoneColor.BLUE)
    assert blue[0].region.y == 30 and blue[0].region.y_max == 330


def test_field_layout_start_and_bumps():
    from rescue_robot.perception.field_elements import (
        FieldLayout, FieldElementType)
    f = FieldLayout.standard()
    starts = [e for e in f.elements if e.type == FieldElementType.START_ZONE]
    assert len(starts) == 4
    # 出发区 1 左上、4 右下
    assert starts[0].region.x == 0 and starts[0].region.y == 2700
    bumps = [e for e in f.elements if e.type == FieldElementType.SPEED_BUMP]
    # 每个出发区两条内边各 3 根 = 4×2×3 = 24
    assert len(bumps) == 24
    # 单个减速带 300×60
    assert bumps[0].region.width == 300 and bumps[0].region.height == 60


# ============================================================
# 5. 系统自检（SystemChecker + Mock）
# ============================================================

def test_system_check_mock_passes():
    from rescue_robot.system_check import (
        MockHardwareChecker, SystemChecker, CheckStatus)
    checker = SystemChecker(MockHardwareChecker())
    report = checker.run()
    assert report.overall_pass is True, report.summary()
    assert report.failed_count == 0


def test_real_hardware_checker_degrades():
    from rescue_robot.system_check import RealHardwareChecker
    hw = RealHardwareChecker(chassis=None, camera_index=99)
    # 无摄像头 → check_camera 返回 False
    assert hw.check_camera() is False
    # 无串口 → IMU/电机 返回 False
    assert hw.check_imu() is False
    assert hw.check_motor(0) is False
    # 电池未实现 → -1
    assert hw.check_battery_voltage() == -1.0
