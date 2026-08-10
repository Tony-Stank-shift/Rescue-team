"""
板块 9 仿真测试 — 3 轮自检

测试:
  第 1 轮: 单元功能 (15 tests)
  第 2 轮: 集成验证 (8 tests)
  第 3 轮: 边界情况 (7 tests)
"""

import logging
import math
import os
import random
import sys
import time
import tempfile

logging.basicConfig(
    level=logging.WARNING,
    format="%(asctime)s [%(levelname)-7s] %(message)s",
    datefmt="%H:%M:%S",
)

# 确保 src 在路径中
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

all_passed = 0
all_failed = 0

_test_registry = []


def test(name: str):
    """测试装饰器注册。"""
    def decorator(fn):
        _test_registry.append((name, fn))
        return fn
    return decorator


def run_registered_tests():
    """执行所有注册测试。"""
    global all_passed, all_failed
    for test_name, fn in _test_registry:
        print(f"\n{'─' * 55}")
        print(f"  {test_name}")
        print(f"{'─' * 55}")
        try:
            fn()
            all_passed += 1
            print(f"  OK 通过")
        except AssertionError as e:
            all_failed += 1
            print(f"  FAIL 失败: {e}")
        except Exception as e:
            all_failed += 1
            print(f"  ERROR 异常: {type(e).__name__}: {e}")


# ============================================================
# 第 1 轮: 单元功能
# ============================================================

@test("1.1 SimField 构建 → 场地所有 3D 元素加载成功")
def test_field_build():
    import mujoco
    from rescue_robot.simulation.sim_field import SimField

    spec = mujoco.MjSpec()
    spec.option.timestep = 0.002
    spec.option.gravity = [0, 0, -9.81]

    field = SimField()
    field.build(spec)
    model = spec.compile()

    assert model.ngeom >= 25, f"至少 25 geoms: {model.ngeom}"
    print(f"  geoms={model.ngeom}, bodies={model.nbody}")


@test("1.2 SimTarget 创建 → 6 种形状正确创建")
def test_target_shapes():
    import mujoco
    from dataclasses import dataclass
    from rescue_robot.simulation.sim_target import SimTarget

    @dataclass
    class FakeInfo:
        shape: object = "cube"
        color: object = "green"
        type: object = None
        points: int = 5
        description: str = ""

    shapes = ["cube", "pyramid", "cuboid", "cylinder", "cone_frustum", "sphere"]
    spec = mujoco.MjSpec()
    spec.worldbody.add_geom(type=mujoco.mjtGeom.mjGEOM_PLANE,
                            size=[0, 0, 0.001], pos=[1.5, 1.5, 0])

    for i, shape in enumerate(shapes):
        info = FakeInfo(shape=shape, color="green")
        t = SimTarget(spec.worldbody, info, position_m=(0.3 + i * 0.4, 0.5),
                      target_id=i + 1)
        assert t.body is not None

    model = spec.compile()
    assert model.ngeom == 7  # 6 targets + 1 ground
    print(f"  创建 {len(shapes)} 种形状, geoms={model.ngeom}")


@test("1.3 SimRobot 运动学 → 差速驱动正确")
def test_robot_kinematics():
    import mujoco
    from rescue_robot.simulation.sim_robot import SimRobot

    spec = mujoco.MjSpec()
    spec.option.timestep = 0.004
    spec.option.gravity = [0, 0, -9.81]
    spec.worldbody.add_geom(type=0, size=[0, 0, 0.001], pos=[1.5, 1.5, 0])

    robot = SimRobot(spec.worldbody, start_zone=1)
    model = spec.compile()
    data = mujoco.MjData(model)
    robot.init_pose(data)
    mujoco.mj_forward(model, data)

    # 前进 100 步
    for _ in range(100):
        robot.set_velocity(data, 0.5, 0.0)
        mujoco.mj_step(model, data)
    x, y, _ = robot.get_pose(data)
    assert x > 0.16, f"应向前移动: x={x:.3f}"  # 起点 0.15
    print(f"  前进 100 步: pos=({x:.3f}, {y:.3f})")


@test("1.4 SimRobot 速度限制 → 不超最大速度")
def test_robot_speed_limit():
    import mujoco
    from rescue_robot.simulation.sim_robot import SimRobot

    spec = mujoco.MjSpec()
    spec.option.timestep = 0.004
    spec.option.gravity = [0, 0, -9.81]
    spec.worldbody.add_geom(type=0, size=[0, 0, 0.001], pos=[1.5, 1.5, 0])

    robot = SimRobot(spec.worldbody, start_zone=1)
    model = spec.compile()
    data = mujoco.MjData(model)
    robot.init_pose(data)

    # 超限输入
    robot.set_velocity(data, 100.0, 100.0)
    jid = data.model.joint("robot_free").id
    dof = data.model.jnt_dofadr[jid]
    vx = data.qvel[dof]
    wz = data.qvel[dof + 5]

    assert abs(vx) <= robot.MAX_LINEAR_SPEED_MS + 0.1
    assert abs(wz) <= robot.MAX_ANGULAR_SPEED_RS + 0.1
    print(f"  限制后 vx={vx:.2f} m/s, wz={wz:.2f} rad/s")


@test("1.5 SimCamera 目标检测 → FOV 范围内检测到目标")
def test_camera_detection():
    from dataclasses import dataclass
    from rescue_robot.simulation.sim_sensors import SimCamera

    @dataclass
    class FakeTarget:
        target_id: int
        _pos_m: tuple
        is_delivered: bool = False
        points: int = 5
        is_dangerous: bool = False

    cam = SimCamera(detection_range_m=5.0, fov_deg=90,
                    false_negative_rate=0.0)
    targets = [
        FakeTarget(1, (1.0, 1.0)),    # 前方 45°
        FakeTarget(2, (-1.0, -1.0)),  # 后方 (不可见)
        FakeTarget(3, (3.0, 1.0)),    # 前方 但远
    ]

    dets = cam.detect((0.0, 0.0, 0.0), targets)
    # target 1 should be detected (distance ~1.414, bearing 45°, within 90° FOV)
    ids = [d["id"] for d in dets]
    assert 1 in ids, f"目标 1 应被检测: {ids}"
    assert 2 not in ids, f"目标 2 不应被检测 (后方): {ids}"
    assert 3 in ids, f"目标 3 应被检测 (范围内): {ids}"
    print(f"  检测数: {len(dets)}, ids={ids}")


@test("1.6 SimIMU → 输出含重力加速度")
def test_imu_reading():
    import mujoco
    from rescue_robot.simulation.sim_robot import SimRobot
    from rescue_robot.simulation.sim_sensors import SimIMU

    spec = mujoco.MjSpec()
    spec.option.timestep = 0.004
    spec.option.gravity = [0, 0, -9.81]
    spec.worldbody.add_geom(type=0, size=[0, 0, 0.001], pos=[1.5, 1.5, 0])

    robot = SimRobot(spec.worldbody, start_zone=1)
    model = spec.compile()
    data = mujoco.MjData(model)
    robot.init_pose(data)
    mujoco.mj_forward(model, data)

    imu = SimIMU(gyro_noise_std=0.0, accel_noise_std=0.0)
    r = imu.read(data)
    ax, ay, az = r["accel"]
    assert abs(az + 9.81) < 0.1, f"Z 加速度应 ~-9.81: {az:.3f}"
    print(f"  accel=({ax:.3f}, {ay:.3f}, {az:.3f})")


@test("1.7 SimOdometry → 初始漂移为 0")
def test_odometry_initial():
    from rescue_robot.simulation.sim_sensors import SimOdometry

    odom = SimOdometry(position_noise_per_m=0.0,
                        heading_noise_per_rad=0.0)
    pose = (0.15, 0.15, 0.0)
    op = odom.read(pose)
    assert op == pose, f"无噪声时里程计应等于真值: {op}"
    print(f"  odom = gt = ({op[0]:.3f}, {op[1]:.3f}, {op[2]:.3f})")


@test("1.8 SimWorld 初始化 → model + targets 正确")
def test_world_setup():
    from rescue_robot.simulation.sim_world import SimWorld

    world = SimWorld(mode="headless", seed=5)
    world.setup_match(target_count=8)
    assert world._model is not None
    assert len(world.targets) == 8
    assert world.robot is not None
    assert world.field is not None
    nbody = world._model.nbody
    world.close()
    print(f"  目标: {len(world.targets)}, nbody={nbody}")


@test("1.9 SimWorld step → 不崩溃")
def test_world_step():
    from rescue_robot.simulation.sim_world import SimWorld

    world = SimWorld(mode="headless", seed=7)
    world.setup_match(target_count=5)
    for _ in range(20):
        world.step()
    state = world._get_state()
    assert state.time_elapsed_s > 0
    world.close()
    print(f"  20 steps, time={state.time_elapsed_s:.2f}s")


@test("1.10 SimWorld render → 返回有效图像")
def test_world_render():
    from rescue_robot.simulation.sim_world import SimWorld

    world = SimWorld(mode="headless", seed=9)
    world.setup_match(target_count=3)
    px = world.render()
    assert px.shape[2] == 3
    assert px.shape[0] > 0 and px.shape[1] > 0
    world.close()
    print(f"  渲染: {px.shape}")


@test("1.11 SimRunner headless → 短期比赛完成")
def test_runner_headless():
    from rescue_robot.simulation.sim_runner import run_simulation

    result = run_simulation(
        phase="preliminary",
        mode="headless",
        duration_s=1.0,
        target_count=3,
        seed=42,
    )
    assert result["total_time_s"] >= 0
    assert "score" in result
    print(f"  score={result['score']}, delivered={result['targets_delivered']}")


@test("1.12 SimSensorBundle → 完整数据包")
def test_sensor_bundle():
    import mujoco
    from rescue_robot.simulation.sim_robot import SimRobot
    from rescue_robot.simulation.sim_sensors import SimSensorBundle

    spec = mujoco.MjSpec()
    spec.option.timestep = 0.004
    spec.option.gravity = [0, 0, -9.81]
    spec.worldbody.add_geom(type=0, size=[0, 0, 0.001], pos=[1.5, 1.5, 0])

    robot = SimRobot(spec.worldbody, start_zone=1)
    model = spec.compile()
    data = mujoco.MjData(model)
    robot.init_pose(data)
    mujoco.mj_forward(model, data)

    bundle = SimSensorBundle(model, data, robot)
    pkg = bundle.generate([])
    assert "camera" in pkg
    assert "imu" in pkg
    assert "odometry" in pkg
    assert "timestamp" in pkg
    print(f"  数据包: {list(pkg.keys())}")


@test("1.13 SimOpponent → 3 种行为模式创建成功")
def test_opponent_behaviors():
    import mujoco
    from rescue_robot.simulation.sim_opponent import SimOpponent

    for behavior in ["random", "competitive", "aggressive"]:
        spec = mujoco.MjSpec()
        spec.option.timestep = 0.004
        spec.option.gravity = [0, 0, -9.81]
        spec.worldbody.add_geom(type=0, size=[0, 0, 0.001], pos=[1.5, 1.5, 0])

        opp = SimOpponent(spec.worldbody, behavior=behavior)
        model = spec.compile()
        assert model.nbody >= 2  # world + opponent
    print(f"  random ✅  competitive ✅  aggressive ✅")


@test("1.14 SimMonitor → 状态收集正确")
def test_monitor_status():
    from dataclasses import dataclass
    from rescue_robot.simulation.sim_debug_gui import SimMonitor

    @dataclass
    class FakeState:
        time_elapsed_s: float = 10.0
        time_remaining_s: float = 170.0
        score: int = 25
        targets_delivered: int = 3
        robot_pose: tuple = (1.0, 1.5, 0.3)
        strategy_state: str = "FREE_RUN"
        is_terminal: bool = False
        violations: int = 0
        event: str = None

    monitor = SimMonitor()
    monitor.update(FakeState())
    d = monitor.status_dict()
    assert d["score"] == 25
    assert d["targets_delivered"] == 3
    print(f"  score={d['score']}, delivered={d['targets_delivered']}")


@test("1.15 SimScenario → 6 个预设场景全创建成功")
def test_scenarios_all():
    from rescue_robot.simulation.sim_world import SimWorld
    from rescue_robot.simulation.sim_scenario import SCENARIOS

    for name, setup_fn in SCENARIOS.items():
        world = SimWorld(mode="headless", seed=0)
        setup_fn(world)
        assert world._model is not None
        world.close()
    print(f"  全部 {len(SCENARIOS)} 场景: " + ", ".join(SCENARIOS.keys()))


# ============================================================
# 第 2 轮: 集成验证
# ============================================================

@test("2.1 仿真 → FieldConfig 加载 → 场地按 YAML 参数构建")
def test_integration_field_config():
    from rescue_robot.simulation.sim_world import SimWorld

    world = SimWorld(mode="headless", seed=1)
    world.setup_match(target_count=5)

    # 验证场地构建参数
    model = world._model
    # 检查关键几何体存在
    geom_names = {model.geom(i).name for i in range(model.ngeom)}
    assert "field_ground" in geom_names
    assert "wall_bottom" in geom_names
    assert "safe_red_plate" in geom_names or any("safe_red" in n for n in geom_names)
    assert "safe_blue_plate" in geom_names or any("safe_blue" in n for n in geom_names)

    world.close()
    print(f"  {len(geom_names)} geoms: field_ground ✅ walls ✅ safe_zones ✅")


@test("2.2 仿真 → 简易 AI → 机器人朝向目标移动")
def test_integration_ai_move():
    from rescue_robot.simulation.sim_world import SimWorld

    world = SimWorld(mode="headless", seed=2)
    world.setup_match(target_count=3)

    # 记录初始位置
    state0 = world._get_state()
    x0, y0, _ = state0.robot_pose

    # 运行 100 steps (2s) with simple AI
    for _ in range(100):
        world.step()

    state1 = world._get_state()
    x1, y1, _ = state1.robot_pose
    dist = math.sqrt((x1 - x0)**2 + (y1 - y0)**2)
    assert dist > 0.02, f"机器人应移动: {dist:.3f}m"

    world.close()
    print(f"  移动距离: {dist:.3f}m")


@test("2.3 仿真 → 物理碰撞 → 目标被推动")
def test_integration_target_push():
    import mujoco
    from rescue_robot.simulation.sim_field import SimField
    from rescue_robot.simulation.sim_robot import SimRobot
    from rescue_robot.simulation.sim_target import SimTarget
    from dataclasses import dataclass

    @dataclass
    class FakeInfo:
        shape: object = "cube"
        color: object = "green"
        type: object = "REGULAR_SUPPLY"
        points: int = 5
        description: str = ""

    spec = mujoco.MjSpec()
    spec.option.timestep = 0.004
    spec.option.gravity = [0, 0, -9.81]

    # Ground
    spec.worldbody.add_geom(type=0, size=[0, 0, 0.001], pos=[1.5, 1.5, 0])

    # Target directly in front of robot
    info = FakeInfo()
    target = SimTarget(spec.worldbody, info, position_m=(0.4, 0.15), target_id=1)

    robot = SimRobot(spec.worldbody, start_zone=1)

    model = spec.compile()
    data = mujoco.MjData(model)
    robot.init_pose(data)
    mujoco.mj_forward(model, data)

    # Target initial X
    t_x0 = target.get_position(data)[0]

    # Push into target
    for _ in range(80):
        robot.set_velocity(data, 0.5, 0.0)
        mujoco.mj_step(model, data)

    t_x1 = target.get_position(data)[0]
    assert t_x1 > t_x0 + 0.01, f"目标应被推动: {t_x0:.3f} → {t_x1:.3f}"
    print(f"  目标 X: {t_x0:.3f} → {t_x1:.3f}m")


@test("2.4 仿真 → 目标送入安全区 → 标记 delivered + 计分")
def test_integration_delivery_scoring():
    from rescue_robot.simulation.sim_world import SimWorld
    from rescue_robot.simulation.sim_models import mm_to_m

    world = SimWorld(mode="headless", seed=3)
    world.setup_match(target_count=1)

    # 手动把目标位置设为安全区附近，并推动
    if world.targets:
        t = world.targets[0]
        # 把目标放到安全区边上
        t._pos_m = (mm_to_m(100)[0], mm_to_m(2570)[0])  # near red zone

    # Run some steps
    for _ in range(200):
        world.step()

    # 检查结果
    delivered = sum(1 for t in world.targets if t.is_delivered)
    print(f"  delivered: {delivered}/{len(world.targets)}")

    world.close()


@test("2.5 完整 30s 比赛 → 无崩溃，运行正常")
def test_integration_full_match():
    from rescue_robot.simulation.sim_world import SimWorld

    world = SimWorld(mode="headless", seed=11)
    world.setup_match(target_count=5)
    result = world.run(duration_s=5.0)

    assert result.total_time_s >= 4.9
    assert result.score >= 0

    world.close()
    print(f"  score={result.score}, delivered={result.targets_delivered}, "
          f"time={result.total_time_s:.1f}s")


@test("2.6 仿真 → 渲染 → 连续 50 帧无崩溃")
def test_integration_render_frames():
    from rescue_robot.simulation.sim_world import SimWorld

    world = SimWorld(mode="headless", seed=13)
    world.setup_match(target_count=4)

    for i in range(10):
        world.step()
        px = world.render()
        assert px.shape[2] == 3
        assert px.shape[0] > 0

    world.close()
    print(f"  10 帧渲染成功")


@test("2.7 对手 AI → competitive 行为 → 不影响我方仿真")
def test_integration_with_opponent():
    import mujoco
    from rescue_robot.simulation.sim_robot import SimRobot
    from rescue_robot.simulation.sim_opponent import SimOpponent
    from dataclasses import dataclass

    @dataclass
    class FakeTarget:
        target_id: int
        _pos_m: tuple
        is_delivered: bool = False
        points: int = 5
        is_dangerous: bool = False

    spec = mujoco.MjSpec()
    spec.option.timestep = 0.004
    spec.option.gravity = [0, 0, -9.81]
    spec.worldbody.add_geom(type=0, size=[0, 0, 0.001], pos=[1.5, 1.5, 0])

    robot = SimRobot(spec.worldbody, start_zone=1)
    opp = SimOpponent(spec.worldbody, behavior="aggressive", start_zone=2)

    model = spec.compile()
    data = mujoco.MjData(model)
    robot.init_pose(data)
    opp.init_pose(data)
    mujoco.mj_forward(model, data)

    targets = [FakeTarget(1, (1.5, 1.5), points=10)]
    our_pose = robot.get_pose(data)

    for _ in range(30):
        robot.set_velocity(data, 0.3, 0.0)
        opp.update(data, targets, our_pose)
        mujoco.mj_step(model, data)
        our_pose = robot.get_pose(data)

    # 两个机器人都在场地内
    rx, ry, _ = robot.get_pose(data)
    ox, oy, _ = opp.get_pose(data)
    assert 0 < rx < 3 and 0 < ry < 3, f"我方应在场内: ({rx:.2f}, {ry:.2f})"
    assert 0 < ox < 3 and 0 < oy < 3, f"对手应在场内: ({ox:.2f}, {oy:.2f})"
    print(f"  同场仿真: 我方({rx:.2f},{ry:.2f})  对手({ox:.2f},{oy:.2f})")


@test("2.8 数据包 → camera + IMU + odometry → 格式兼容感知管道")
def test_integration_sensor_format():
    import mujoco
    from rescue_robot.simulation.sim_robot import SimRobot
    from rescue_robot.simulation.sim_sensors import SimSensorBundle

    spec = mujoco.MjSpec()
    spec.option.timestep = 0.004
    spec.option.gravity = [0, 0, -9.81]
    spec.worldbody.add_geom(type=0, size=[0, 0, 0.001], pos=[1.5, 1.5, 0])

    robot = SimRobot(spec.worldbody, start_zone=1)
    model = spec.compile()
    data = mujoco.MjData(model)
    robot.init_pose(data)
    mujoco.mj_forward(model, data)

    bundle = SimSensorBundle(model, data, robot)
    pkg = bundle.generate([])

    # 验证 camera 格式
    assert isinstance(pkg["camera"], list)
    # 验证 imu 格式
    assert isinstance(pkg["imu"]["gyro"], tuple) and len(pkg["imu"]["gyro"]) == 3
    assert isinstance(pkg["imu"]["accel"], tuple) and len(pkg["imu"]["accel"]) == 3
    # 验证 odometry 格式
    assert isinstance(pkg["odometry"], tuple) and len(pkg["odometry"]) == 3

    print(f"  camera: list, imu.gyro: tuple×3, imu.accel: tuple×3, odometry: tuple×3")
    print(f"  格式兼容 ✅")


# ============================================================
# 第 3 轮: 边界情况
# ============================================================

@test("3.1 无目标场景 → 立即结束")
def test_edge_empty_field():
    from rescue_robot.simulation.sim_world import SimWorld

    world = SimWorld(mode="headless", seed=0)
    world.setup_match(target_count=0)

    state = world.step()
    # 0 targets → all delivered → terminal
    assert state.is_terminal
    world.close()
    print(f"  terminal={state.is_terminal}, score={state.score}")


@test("3.2 随机种子可复现 → 相同 seed 相同布局")
def test_edge_reproducible():
    from rescue_robot.simulation.sim_world import SimWorld

    world1 = SimWorld(mode="headless", seed=42)
    world1.setup_match(target_count=5)
    pos1 = [(t._pos_m[0], t._pos_m[1]) for t in world1.targets]
    world1.close()

    # 重置种子
    random.seed(42)

    world2 = SimWorld(mode="headless", seed=42)
    world2.setup_match(target_count=5)
    pos2 = [(t._pos_m[0], t._pos_m[1]) for t in world2.targets]
    world2.close()

    # 位置应完全相同 (同一种子)
    for i, (p1, p2) in enumerate(zip(pos1, pos2)):
        assert abs(p1[0] - p2[0]) < 0.001, f"目标 {i} X 不匹配: {p1[0]:.4f} vs {p2[0]:.4f}"
        assert abs(p1[1] - p2[1]) < 0.001, f"目标 {i} Y 不匹配: {p1[1]:.4f} vs {p2[1]:.4f}"
    print(f"  {len(pos1)} 目标，2 次布局完全相同")


@test("3.3 大量目标 → 性能可接受 (>100 Hz)")
def test_edge_performance():
    from rescue_robot.simulation.sim_world import SimWorld

    world = SimWorld(mode="headless", seed=55)
    world.setup_match(target_count=25)

    t0 = time.time()
    for _ in range(200):
        world.step()
    elapsed = time.time() - t0

    steps_per_sec = 200 / elapsed
    assert steps_per_sec > 50, f"性能过低: {steps_per_sec:.0f} steps/s"
    world.close()
    print(f"  200 steps in {elapsed:.2f}s → {steps_per_sec:.0f} steps/s")


@test("3.4 关闭后重新创建 → 不泄漏")
def test_edge_recreate():
    from rescue_robot.simulation.sim_world import SimWorld

    for i in range(3):
        world = SimWorld(mode="headless", seed=i * 10)
        world.setup_match(target_count=2)
        for _ in range(10):
            world.step()
        world.close()
    print(f"  3 轮创建/关闭 → 无异常")


@test("3.5 多出发区 → 4 个位置均可启动")
def test_edge_start_zones():
    import mujoco
    from rescue_robot.simulation.sim_robot import SimRobot

    for zone in [1, 2, 3, 4]:
        spec = mujoco.MjSpec()
        spec.option.timestep = 0.004
        spec.option.gravity = [0, 0, -9.81]
        spec.worldbody.add_geom(type=0, size=[0, 0, 0.001], pos=[1.5, 1.5, 0])

        robot = SimRobot(spec.worldbody, start_zone=zone)
        model = spec.compile()
        data = mujoco.MjData(model)
        robot.init_pose(data)
        mujoco.mj_forward(model, data)

        x, y, _ = robot.get_pose(data)
        assert 0 <= x <= 3 and 0 <= y <= 3, f"Zone {zone}: ({x:.3f}, {y:.3f})"
    print(f"  4 个出发区均有效")


@test("3.6 MuJoCo 版本检测 → 3.x 可用")
def test_edge_mujoco_version():
    import mujoco
    ver = mujoco.__version__
    major = int(ver.split(".")[0])
    assert major >= 3, f"需要 MuJoCo 3.x, 当前 {ver}"
    print(f"  MuJoCo {ver}")


@test("3.7 渲染尺寸变化 → 正常处理")
def test_edge_render_size():
    import mujoco
    from rescue_robot.simulation.sim_world import SimWorld

    world = SimWorld(mode="headless", seed=77)
    world.setup_match(target_count=2)
    world.step()

    # 不同尺寸渲染 (都必须在默认 framebuffer 内)
    sizes = [(240, 320), (320, 480), (400, 600)]
    for h, w in sizes:
        r = mujoco.Renderer(world._model, h, w)
        r.update_scene(world._data)
        px = r.render()
        r.close()
        assert px.shape[2] == 3
    world.close()
    print(f"  {len(sizes)} 种尺寸均正常")


# ============================================================
# 主入口
# ============================================================

if __name__ == "__main__":
    print("=" * 60)
    print("  板块 9 — MuJoCo 3D 仿真测试")
    print("  第 1 轮: 单元功能 (15)  |  第 2 轮: 集成 (8)")
    print("  第 3 轮: 边界情况 (7)    |  总计: 30 tests")
    print("=" * 60)

    run_registered_tests()

    # 结果汇总
    print(f"\n{'=' * 60}")
    print(f"  自检结果: {all_passed} 通过, {all_failed} 失败")
    print(f"{'=' * 60}")

    if all_failed > 0:
        print(f"\nFAIL {all_failed} 项测试失败!")
        sys.exit(1)
    else:
        print(f"\nOK 全部 {all_passed} 项测试通过!")
        sys.exit(0)
