"""gate_follower — 파라미터 경계(B)와 토픽 입출력(D).

3쌍 통과·실패 시 정지는 test_gate_follower.py 가 확인한다. 같은 가짜 게이트 장면
(x = 3, 5, 7 m 에 좌우 ±1.5 m 기둥, 목표 x = 9 m)을 그대로 쓴다.
"""
import pytest
import rclpy
from std_msgs.msg import String

from kaboat_behaviors.gate_follower import GateFollower
from test_gate_follower import _follow_current_gate, _ready_node, _set_odom


class Spy:
    def __init__(self):
        self.msgs = []

    def publish(self, msg):
        self.msgs.append(msg)


@pytest.fixture
def make_node():
    nodes = []

    def factory(*params, posts=None):
        args = ['--ros-args']
        for p in params:
            args += ['-p', p]
        rclpy.init(args=args)
        node = _ready_node(posts)
        for name in ('status_pub', 'preview_pub', 'path_pub', 'complete_pub'):
            setattr(node, name, Spy())
        nodes.append(node)
        return node
    yield factory
    for node in nodes:
        node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


@pytest.mark.check('B')
@pytest.mark.parametrize('param', [
    'gate.thrust_ratio:=1.5', 'gate.scan_timeout_sec:=0.0', 'gate.grid_timeout_sec:=-1.0'])
def test_invalid_thrust_ratio_or_timeout_is_rejected(param):
    rclpy.init(args=['--ros-args', '-p', param])
    try:
        with pytest.raises(ValueError):
            GateFollower()
    finally:
        rclpy.shutdown()


@pytest.mark.check('B')
def test_required_pairs_parameter_changes_completion_count(make_node):
    node = make_node('gate.required_pairs:=2')
    for _ in range(2):
        _follow_current_gate(node)
    node.compute_cmd()
    assert len(node._passed_midpoints) == 2
    assert node.complete_pub.msgs == []            # 목표(9, 0)까지 2m 밖
    assert node._status_text in ('GOAL_APPROACH', 'GOAL_LEG_UNSAFE_OR_TOO_FAR')
    _set_odom(node, 8.0, 0.0)
    node.compute_cmd()
    assert [m.data for m in node.complete_pub.msgs] == ['gate']


@pytest.mark.check('B')
def test_grid_in_wrong_frame_stops(make_node):
    node = make_node()
    node.occupancy_grid.header.frame_id = 'map'
    cmd = node.compute_cmd()
    assert node._status_text == 'GRID_TIMEOUT_OR_FRAME'
    assert cmd.linear.x == cmd.angular.z == 0.0


@pytest.mark.check('D')
def test_motion_enabled_publishes_same_command_as_preview(make_node):
    node = make_node('gate.motion_enabled:=true')
    node.compute_cmd()                              # 경로 확보
    x, y = node._path.samples[4]
    _set_odom(node, x, y)
    cmd = node.compute_cmd()
    preview = node.preview_pub.msgs[-1]
    assert cmd.linear.x > 0.0
    assert (cmd.linear.x, cmd.angular.z) == (preview.linear.x, preview.angular.z)


@pytest.mark.check('D')
def test_preview_mode_publishes_preview_but_returns_stop(make_node):
    node = make_node()                              # gate.motion_enabled 기본 false
    node.compute_cmd()
    x, y = node._path.samples[4]
    _set_odom(node, x, y)
    cmd = node.compute_cmd()
    assert cmd.linear.x == cmd.angular.z == 0.0
    assert node.preview_pub.msgs[-1].linear.x > 0.0


@pytest.mark.check('D')
def test_status_changes_are_published_once_and_path_is_debugged(make_node):
    node = make_node()
    node.compute_cmd()
    node.compute_cmd()
    statuses = [m.data for m in node.status_pub.msgs]
    assert statuses == sorted(set(statuses), key=statuses.index)   # 같은 상태 중복 발행 없음
    assert len(node.path_pub.msgs[-1].poses) == len(node._path.samples)
    assert node.path_pub.msgs[-1].header.frame_id == 'odom'


@pytest.mark.check('D')
def test_completion_published_once_per_activation(make_node):
    node = make_node()
    for _ in range(3):
        _follow_current_gate(node)
    for _ in range(3):
        node.compute_cmd()
    assert [m.data for m in node.complete_pub.msgs] == ['gate']
    node._on_state(String(data='station'))
    assert GateFollower.CMD_TOPIC == '/cmd/gate'
