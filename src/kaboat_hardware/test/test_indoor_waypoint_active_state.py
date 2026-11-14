"""goto active_state: /mission/state 가 그 값일 때만 몰고, 아닐 땐 처음 상태로 (토너먼트 한 번에)."""

import math

import pytest
import rclpy
from nav_msgs.msg import Odometry
from rclpy.parameter import Parameter
from std_msgs.msg import String

from kaboat_hardware.indoor_waypoint import IndoorWaypoint


class _Rec:
    def __init__(self):
        self.msgs = []

    def publish(self, msg):
        self.msgs.append(msg)


@pytest.fixture
def make():
    rclpy.init()
    nodes = []

    def build(active_state):
        n = IndoorWaypoint(parameter_overrides=[
            Parameter('outdoor', value=True), Parameter('motion_enabled', value=True),
            Parameter('target_x', value=10.0), Parameter('target_y', value=0.0),
            Parameter('align_heading', value=False), Parameter('thrust_ratio', value=0.4),
            Parameter('active_state', value=active_state)])
        n.cmd_rec = _Rec()
        n.cmd_pub = n.cmd_rec
        n.count_publishers = lambda topic: 1
        nodes.append(n)
        return n
    yield build
    for n in nodes:
        n.destroy_node()
    rclpy.shutdown()


def _odom(n, x, y, yaw=0.0):
    m = Odometry()
    m.header.frame_id = 'odom'
    m.pose.pose.position.x, m.pose.pose.position.y = x, y
    m.pose.pose.orientation.z, m.pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
    n._on_odom(m)


def test_idle_until_state_then_drives_from_where_it_is(make):
    n = make('approach')
    _odom(n, 0.0, 0.0)
    n._tick()
    assert n.cmd_rec.msgs == [] and n.last_status == 'IDLE_UNTIL_APPROACH'   # 명령 없음
    n._on_mission_state(String(data='undock'))
    _odom(n, 10.0, 0.0)                                          # 직진 중 목표에 와 있어도
    for _ in range(40):
        n._tick()
    assert n.cmd_rec.msgs == [] and not n.completed              # 도착으로 치지 않음
    n._on_mission_state(String(data='approach'))
    _odom(n, 4.0, 0.0)                                           # 직진 뒤 다른 자리
    n._tick()
    assert n.cmd_rec.msgs and n.cmd_rec.msgs[-1].linear.x > 0.0  # 지금 자리에서 새로 출발


def test_leaving_the_state_resets_a_finished_goto(make):
    n = make('approach')
    n._on_mission_state(String(data='approach'))
    _odom(n, 10.0, 0.0)
    n._tick()
    n.completed = True                                            # 도착해서 끝난 goto (머묾은 실제 시간이라 생략)
    n._tick()
    assert n.cmd_rec.msgs[-1].linear.x == 0.0                     # 끝나면 0 만 낸다 — 예전엔 이대로 영영
    n._on_mission_state(String(data='avoid'))
    n._tick()
    assert not n.completed and n.last_status == 'IDLE_UNTIL_APPROACH'
    sent = len(n.cmd_rec.msgs)
    n._tick()
    assert len(n.cmd_rec.msgs) == sent                           # avoid 중엔 명령 없음


def test_default_has_no_state_gate(make):
    n = make('')
    _odom(n, 0.0, 0.0)
    n._tick()
    assert n.cmd_rec.msgs and n.cmd_rec.msgs[-1].linear.x > 0.0  # 예전처럼 바로 몬다
