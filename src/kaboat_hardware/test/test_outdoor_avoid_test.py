"""야외 회피 시험: outdoor_avoid_test.yaml 로 띄운 노드 + launch 의 기록 지점 읽기."""

import importlib.util
import math
import os

import pytest
import rclpy
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import LaserScan

from kaboat_hardware.real_obstacle_avoid_test import RealObstacleAvoidTest
from kaboat_mission.mission_points import MissionPoint, PointStore

HERE = os.path.dirname(__file__)
CONFIG = os.path.join(HERE, '..', 'config', 'outdoor_avoid_test.yaml')
LAUNCH = os.path.join(HERE, '..', 'launch', 'outdoor_avoid_test.launch.py')
ORIGIN = (35.18487880, 128.56510100, 30.0)
START = (157.94, -3.43)          # 현장 기록 avoid_entry
GOAL = (95.00, 5.93)             # 현장 기록 avoid_end (63.6 m)
GOAL_BEARING = math.atan2(GOAL[1] - START[1], GOAL[0] - START[0])   # ≈ 171.5°


@pytest.fixture
def node():
    rclpy.init(args=['--ros-args', '--params-file', CONFIG,
                     '-p', f'fixed_start_xy:=[{START[0]}, {START[1]}]',
                     '-p', f'fixed_goal_xy:=[{GOAL[0]}, {GOAL[1]}]'])
    n = RealObstacleAvoidTest()
    n.motor = []
    n.cmd_pub.publish = n.motor.append
    yield n
    n.destroy_node()
    rclpy.shutdown()


def _feed(n, x, y, yaw):
    odom = Odometry()
    odom.header.frame_id = 'odom'
    odom.pose.pose.position.x, odom.pose.pose.position.y = x, y
    odom.pose.pose.orientation.z = math.sin(yaw / 2)
    odom.pose.pose.orientation.w = math.cos(yaw / 2)
    n._on_odom(odom)
    grid = OccupancyGrid()                       # 배 중심 15 m, 장애물 없음
    grid.header.frame_id = 'odom'
    grid.info.resolution = 0.2
    grid.info.width = grid.info.height = 75
    grid.info.origin.position.x, grid.info.origin.position.y = x - 7.5, y - 7.5
    grid.data = [0] * (75 * 75)
    n._on_grid(grid)
    scan = LaserScan()
    scan.range_min, scan.range_max, scan.ranges = 0.1, 40.0, [12.0]
    n._on_scan(scan)


def test_outdoor_config_has_no_tank_and_uses_recorded_pair(node):
    assert not node.tank_enabled and node.fixed_goal_enabled
    assert node.fixed_start_xy == START and node.fixed_goal_xy == GOAL
    assert node.start_tolerance == 3.0 and not node.require_front_obstacle


def test_far_goal_gets_a_preview_path_and_arms_only_at_start_facing_goal(node):
    _feed(node, START[0] - 1.0, START[1] + 0.5, GOAL_BEARING + math.radians(20))
    node._tick()
    assert node.preview_plan is not None              # 63 m 떨어진 목표도 지도 안 구간으로 계획
    assert node.preview_goal == GOAL
    ok, why = node._arm()
    assert ok, why
    assert node.motor == []                            # drive_enabled: false (yaml 기본)


def test_refuses_far_from_start_or_facing_away(node):
    _feed(node, START[0] - 10.0, START[1], GOAL_BEARING)
    node._tick()
    ok, why = node._arm()
    assert not ok and 'from the start point' in why

    _feed(node, START[0], START[1], GOAL_BEARING + math.radians(90))
    node._tick()
    ok, why = node._arm()
    assert not ok and 'heading' in why


def _drive_along(n, steps=30, step_m=0.3):
    """시작점에서 목표 쪽으로 몇 m 움직이며 tick — FSM 이 지나온 궤적을 기록한다."""
    c, s = math.cos(GOAL_BEARING), math.sin(GOAL_BEARING)
    for k in range(steps):
        _feed(n, START[0] + c * step_m * k, START[1] + s * step_m * k, GOAL_BEARING)
        n._tick()


def _far_point():
    """시작점에서 목표 쪽으로 20 m — 출발 검사(3 m)로는 출발 못 하는 자리."""
    return (START[0] + 20.0 * math.cos(GOAL_BEARING), START[1] + 20.0 * math.sin(GOAL_BEARING))


def test_mid_run_disarm_resumes_from_here_and_keeps_the_trail(node):
    """주행 중 풀리면(센서 끊김·경로 이탈) 시작점에 돌아가야만 다시 출발했고, 다시 출발할 때 지나온
    궤적까지 지워 ESCAPE 가 후진하지 못했다 (2026-11-13 야외 회피). 이제 지금 자리에서 이어 간다."""
    assert node.resume_after_disarm                   # outdoor_avoid_test.yaml 에서 켬
    _feed(node, *START, GOAL_BEARING)
    node._tick()
    assert node._arm()[0]
    _drive_along(node)
    trail_m = node.fsm.trail.length()
    assert trail_m > 5.0
    node._disarm('cross-track error 1.80 m')
    assert not node.started and node.resume_pending
    assert node.fsm.trail.length() == pytest.approx(trail_m)     # 풀려도 궤적 유지
    _feed(node, *_far_point(), GOAL_BEARING + math.radians(120))   # 시작점 20 m·목표에서 120° 틀어짐
    node._tick()
    assert node.started and not node.resume_pending                # 시작점·방위 검사 없이 이어 감
    assert node.fsm.trail.length() == pytest.approx(trail_m)


def test_resume_does_not_need_a_preview_path(node):
    """경로를 못 만드는 자리에서 풀렸어도 이어 간다 — FSM 이 찾고, 못 찾으면 ESCAPE 로 후진."""
    _feed(node, *START, GOAL_BEARING)
    node._tick()
    assert node._arm()[0]
    _drive_along(node)
    node._disarm('/scan stale (0.62 s)')
    node._generate_plan = lambda *a, **k: None
    _feed(node, *_far_point(), GOAL_BEARING)
    node._tick()
    assert node.started


def test_resumed_escape_retraces_the_kept_trail(node):
    """이어 간 뒤 경로를 못 찾으면 ESCAPE 가 남긴 궤적을 되짚어 후진한다."""
    _feed(node, *START, GOAL_BEARING)
    node._tick()
    assert node._arm()[0]
    _drive_along(node)
    end = (START[0] + math.cos(GOAL_BEARING) * 0.3 * 29, START[1] + math.sin(GOAL_BEARING) * 0.3 * 29)
    node._disarm('cross-track error 1.80 m')
    _feed(node, *end, GOAL_BEARING)
    node._tick()
    assert node.started
    out = node.fsm._enter_escape(0.0, 'test', end)
    assert out.state == 'ESCAPE' and node.fsm._retrace is not None    # 되짚을 길이 있음


def test_escape_recovery_does_not_disarm_the_test(node):
    """ESCAPE 에서 회복하는 순간 로거 예외('Logger severity cannot be changed between calls')가
    계획기 오류로 잡혀 시험이 풀렸다 (2026-11-13 bag 10:22:34). 진입·복귀를 두 번 오가도 그대로."""
    from kaboat_behaviors.avoid_fsm import StepOut
    _feed(node, *START, GOAL_BEARING)
    node._tick()
    assert node._arm()[0]
    for st, ev in (('ESCAPE', 'escape_panic'), ('FOLLOW', 'escape_recovered'),
                   ('ESCAPE', 'escape_panic'), ('FOLLOW', 'escape_recovered')):
        node.fsm.step = lambda *a, st=st, ev=ev: StepOut(0.0, 0.0, st, None, ev)
        _feed(node, *START, GOAL_BEARING)
        node._tick()
        assert node.started, st


def test_emergency_stop_does_not_resume(node):
    from std_msgs.msg import Bool
    _feed(node, *START, GOAL_BEARING)
    node._tick()
    assert node._arm()[0]
    _drive_along(node)
    node._on_estop(Bool(data=True))
    node._on_estop(Bool(data=False))
    assert not node.resume_pending
    _feed(node, *_far_point(), GOAL_BEARING)
    node._tick()
    ok, why = node._arm()
    assert not ok and 'from the start point' in why


def test_resume_off_keeps_the_old_start_check(node):
    node.resume_after_disarm = False
    _feed(node, *START, GOAL_BEARING)
    node._tick()
    assert node._arm()[0]
    _drive_along(node)
    node._disarm('cross-track error 1.80 m')
    assert not node.resume_pending
    _feed(node, *_far_point(), GOAL_BEARING)
    node._tick()
    ok, why = node._arm()
    assert not ok and 'from the start point' in why


@pytest.fixture
def launch_mod():
    spec = importlib.util.spec_from_file_location('outdoor_avoid_launch', LAUNCH)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def _files(tmp_path, origin=ORIGIN):
    o = tmp_path / 'origin.yaml'
    o.write_text(f'origin_lat: {origin[0]}\norigin_lon: {origin[1]}\norigin_alt: {origin[2]}\n')
    p = tmp_path / 'points.json'
    PointStore(ORIGIN, {'avoid_entry': MissionPoint(*START, 148.4),
                        'avoid_end': MissionPoint(*GOAL, 139.5)}).save(str(p))
    return str(p), str(o)


def _get(mod, **overrides):
    values = {name: default for name, default, _ in mod.ARGS}
    values.update(overrides)
    return values.__getitem__


def test_launch_reads_recorded_pair(launch_mod, tmp_path):
    points, origin = _files(tmp_path)
    p = launch_mod.avoid_parameters(_get(launch_mod, points_file=points, origin_file=origin))
    assert p['fixed_start_xy'] == list(START) and p['fixed_goal_xy'] == list(GOAL)
    assert p['drive_enabled'] is True and p['wait_for_start'] is False
    assert p['tank_bounds'] == [0.0] * 4 and p['exclude_tank_walls'] is False
    assert p['goal_tolerance'] == 2.0 and p['start_tolerance'] == 3.0


def test_launch_refuses_missing_point_or_moved_origin(launch_mod, tmp_path):
    points, origin = _files(tmp_path)
    with pytest.raises(RuntimeError, match='goal_point:=gate_end'):
        launch_mod.avoid_parameters(_get(launch_mod, goal_point='gate_end',
                                         points_file=points, origin_file=origin))
    moved = tmp_path / 'moved'
    moved.mkdir()
    _, origin_moved = _files(moved, (ORIGIN[0] + 0.0001, ORIGIN[1], ORIGIN[2]))
    with pytest.raises(RuntimeError, match='원점이 다름'):
        launch_mod.avoid_parameters(_get(launch_mod, points_file=points,
                                         origin_file=origin_moved))
