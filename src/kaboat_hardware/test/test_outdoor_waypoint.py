"""indoor_waypoint 야외 모드 + rec 기록 지점 목표 + outdoor_waypoint.launch.py 인자."""

import importlib.util
import math
import os

import pytest
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

from kaboat_hardware.indoor_waypoint import IndoorWaypoint, recorded_target
from kaboat_mission.mission_points import MissionPoint, PointStore

ORIGIN = (35.18487880, 128.56510100, 30.0)
HERE = os.path.dirname(__file__)
LAUNCH = os.path.join(HERE, '..', 'launch', 'outdoor_waypoint.launch.py')


@pytest.fixture
def files(tmp_path):
    origin = tmp_path / 'origin.yaml'
    origin.write_text(f'origin_lat: {ORIGIN[0]}\norigin_lon: {ORIGIN[1]}\norigin_alt: {ORIGIN[2]}\n')
    points = tmp_path / 'points.json'
    PointStore(ORIGIN, {'gate_entry': MissionPoint(-12.5, 30.0, 135.0)}).save(str(points))
    return str(points), str(origin), tmp_path


def test_recorded_target_reads_point_and_checks_origin(files):
    points, origin, tmp_path = files
    assert recorded_target('gate_entry', points, origin) == (-12.5, 30.0, 135.0)
    assert recorded_target('gate_entry', points, origin, use_heading=False) == (-12.5, 30.0, None)
    with pytest.raises(ValueError, match='gate_entry'):        # 없는 이름 → 저장된 목록 안내
        recorded_target('dock_entry', points, origin)
    moved = tmp_path / 'moved.yaml'                             # 원점 ~11 m 이동
    moved.write_text(f'origin_lat: {ORIGIN[0] + 0.0001}\norigin_lon: {ORIGIN[1]}\n')
    with pytest.raises(ValueError, match='원점이 다름'):
        recorded_target('gate_entry', points, str(moved))
    with pytest.raises(ValueError, match='원점이 다름'):        # 원점 파일 없음
        recorded_target('gate_entry', points, str(tmp_path / 'none.yaml'))


def _odom(x, y, yaw=0.0):
    msg = Odometry()
    msg.header.frame_id = 'odom'
    msg.pose.pose.position.x = x
    msg.pose.pose.position.y = y
    msg.pose.pose.orientation.z = math.sin(yaw / 2)
    msg.pose.pose.orientation.w = math.cos(yaw / 2)
    return msg


def _node(*params):
    args = ['--ros-args']
    for p in params:
        args += ['-p', p]
    rclpy.init(args=args)
    try:
        return IndoorWaypoint()
    except Exception:
        rclpy.shutdown()
        raise


def _close(node):
    node.destroy_node()
    rclpy.shutdown()


def _drive(node, x, y, yaw=0.0):
    sent = []
    node._send = sent.append
    node.count_publishers = lambda topic: 1
    node._on_odom(_odom(x, y, yaw))
    node._tick()
    return sent[-1]


def test_tank_mode_still_rejects_goal_outside_tank():
    with pytest.raises(ValueError, match='tank bounds'):
        _node('target_x:=-12.5', 'target_y:=30.0')


def test_outdoor_accepts_negative_coordinates_and_stops_when_goal_too_far():
    node = _node('outdoor:=true', 'target_x:=-12.5', 'target_y:=30.0',
                 'target_angle_deg:=135.0', 'max_goal_distance:=50.0')
    try:
        cmd = _drive(node, -20.0, 25.0)                          # 수조 밖이어도 유효
        assert node.last_status.startswith('PREVIEW_')
        assert node.target.angle_deg == 135.0
        far = _drive(node, 100.0, 30.0)                          # 112 m — 원점 틀림 등
        assert node.last_status == 'GOAL_TOO_FAR' and far == Twist()
        _drive(node, -20.0, 25.0)                                 # 위치가 돌아오면 다시 진행
        assert node.last_status.startswith('PREVIEW_')
        assert isinstance(cmd, Twist)
    finally:
        _close(node)


def test_outdoor_default_has_no_distance_limit():
    node = _node('outdoor:=true', 'target_x:=-12.5', 'target_y:=30.0')
    try:
        assert node.max_goal_distance == 0.0
        _drive(node, 100.0, 30.0)                                # 112 m 떨어져도
        assert node.last_status.startswith('PREVIEW_')           # 멈추지 않음
    finally:
        _close(node)


def test_outdoor_goal_from_recorded_point_without_heading(files):
    points, origin, _ = files
    node = _node('outdoor:=true', 'point:=gate_entry', 'align_heading:=false', 'settle_time:=0.0',
                 f'points_file:={points}', f'origin_file:={origin}')
    try:
        assert (node.target.x, node.target.y, node.target.angle_deg) == (-12.5, 30.0, None)
        node._publish_goal()                                      # 선수각 없음도 발행 가능
        _drive(node, -12.4, 30.1)
        assert node.completed
    finally:
        _close(node)


@pytest.fixture
def launch_mod():
    spec = importlib.util.spec_from_file_location('outdoor_waypoint_launch', LAUNCH)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def _args(mod, **overrides):
    values = {name: default for name, default, _ in mod.ARGS}
    values.update(overrides)
    return values.__getitem__


def test_launch_point_or_coordinates(launch_mod):
    p = launch_mod.node_parameters(_args(launch_mod, point='gate_entry'))
    assert p['outdoor'] is True and p['point'] == 'gate_entry' and p['align_heading'] is True
    assert p['motion_enabled'] is True and p['thrust_ratio'] == 0.2
    assert p['max_goal_distance'] == 0.0                         # 기본 거리 제한 없음
    assert 'target_x' not in p

    p = launch_mod.node_parameters(_args(launch_mod, x='-3.5', y='12', angle='90'))
    assert (p['target_x'], p['target_y'], p['target_angle_deg'], p['align_heading']) == (
        -3.5, 12.0, 90.0, True)
    p = launch_mod.node_parameters(_args(launch_mod, x='1', y='2', motion_enabled='false'))
    assert p['align_heading'] is False and p['motion_enabled'] is False

    with pytest.raises(RuntimeError, match='목표 없음'):
        launch_mod.node_parameters(_args(launch_mod))
    with pytest.raises(RuntimeError, match='숫자'):
        launch_mod.node_parameters(_args(launch_mod, x='abc', y='1'))
    with pytest.raises(RuntimeError, match='true 또는 false'):
        launch_mod.node_parameters(_args(launch_mod, point='p1', heading='yes'))
