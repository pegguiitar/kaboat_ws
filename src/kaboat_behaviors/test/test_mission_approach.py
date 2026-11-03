"""mission_approach — 다음 미션 시작 자세로 접근하고 도달하면 'approach' 완료를 낸다."""
import math
import time

import pytest
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String

from kaboat_behaviors.mission_approach import MissionApproach


class Spy:
    def __init__(self):
        self.msgs = []

    def publish(self, msg):
        self.msgs.append(msg)


def _odom(x, y, yaw=0.0, frame='odom'):
    msg = Odometry()
    msg.header.frame_id = frame
    msg.pose.pose.position.x = float(x)
    msg.pose.pose.position.y = float(y)
    msg.pose.pose.orientation.z = math.sin(yaw / 2)
    msg.pose.pose.orientation.w = math.cos(yaw / 2)
    return msg


def _goal(x, y, yaw_deg=0.0, frame='odom'):
    msg = PoseStamped()
    msg.header.frame_id = frame
    msg.pose.position.x = float(x)
    msg.pose.position.y = float(y)
    msg.pose.orientation.z = math.sin(math.radians(yaw_deg) / 2)
    msg.pose.orientation.w = math.cos(math.radians(yaw_deg) / 2)
    return msg


@pytest.fixture
def make_node(monkeypatch):
    clock = {'t': 500.0}
    monkeypatch.setattr(time, 'monotonic', lambda: clock['t'])
    nodes = []

    def factory(*params):
        args = ['--ros-args']
        for p in params:
            args += ['-p', p]
        rclpy.init(args=args)
        node = MissionApproach()
        node.preview_pub = Spy()
        node.complete_pub = Spy()
        node.clock = clock
        nodes.append(node)
        return node
    yield factory
    for node in nodes:
        node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


def _tick(node, n=1):
    cmd = None
    for _ in range(n):
        node.clock['t'] += 0.1
        node._on_odom(node._test_odom)
        cmd = node.compute_cmd()
    return cmd


def _activate(node, goal, odom):
    node._on_state(String(data='approach'))
    node._on_entry_goal(goal)
    node._test_odom = odom


@pytest.mark.check('A')
def test_drives_toward_entry_pose(make_node):
    node = make_node('approach.motion_enabled:=true')
    _activate(node, _goal(5.0, 0.0), _odom(0.0, 0.0))
    cmd = _tick(node)
    assert cmd.linear.x > 0.0
    assert abs(cmd.angular.z) < 1e-9


@pytest.mark.check('A')
def test_reports_approach_complete_once_after_settling(make_node):
    node = make_node('approach.motion_enabled:=true')
    _activate(node, _goal(1.0, 1.0, 90.0), _odom(1.0, 1.0, math.pi / 2))
    _tick(node, 3)
    assert node.complete_pub.msgs == []        # settle_time 0.5s 전
    cmd = _tick(node, 5)
    assert [m.data for m in node.complete_pub.msgs] == ['approach']
    assert cmd.linear.x == 0.0 and cmd.angular.z == 0.0
    _tick(node, 3)
    assert len(node.complete_pub.msgs) == 1


@pytest.mark.check('B')
def test_preview_only_when_motion_disabled(make_node):
    node = make_node()   # approach.motion_enabled 기본 false
    _activate(node, _goal(5.0, 0.0), _odom(0.0, 0.0))
    cmd = _tick(node)
    assert cmd.linear.x == 0.0
    assert node.preview_pub.msgs[-1].linear.x > 0.0


@pytest.mark.check('B')
@pytest.mark.parametrize('ratio', ['0.0', '1.5'])
def test_invalid_thrust_ratio_rejected(make_node, ratio):
    with pytest.raises(ValueError):
        make_node(f'approach.thrust_ratio:={ratio}')


@pytest.mark.check('C')
def test_wrong_frame_or_missing_goal_gives_zero(make_node):
    node = make_node('approach.motion_enabled:=true')
    _activate(node, _goal(5.0, 0.0, frame='map'), _odom(0.0, 0.0))
    cmd = _tick(node)
    assert cmd.linear.x == 0.0 and cmd.angular.z == 0.0
    node._on_state(String(data='gate'))
    node._on_state(String(data='approach'))   # 재활성화 시 이전 목표는 지워진다
    assert node.entry_goal is None


@pytest.mark.check('C')
def test_goal_received_while_inactive_is_ignored(make_node):
    node = make_node('approach.motion_enabled:=true')
    node._on_entry_goal(_goal(5.0, 0.0))
    node._on_state(String(data='approach'))
    node._test_odom = _odom(0.0, 0.0)
    cmd = _tick(node)
    assert node.entry_goal is None
    assert cmd.linear.x == 0.0


@pytest.mark.check('C')
def test_stale_odom_gives_zero(make_node):
    node = make_node('approach.motion_enabled:=true')
    _activate(node, _goal(5.0, 0.0), _odom(0.0, 0.0))
    node._on_odom(node._test_odom)
    node.clock['t'] += 1.0          # odom 1s 끊김 (기준 0.5s)
    cmd = node.compute_cmd()
    assert cmd.linear.x == 0.0 and cmd.angular.z == 0.0


@pytest.mark.check('D')
def test_publishes_on_approach_command_topic():
    assert MissionApproach.STATE_NAME == 'approach'
    assert MissionApproach.CMD_TOPIC == '/cmd/approach'
