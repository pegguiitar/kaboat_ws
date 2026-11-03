"""mission_manager — 미션 순서, 전환 조건(완료 신호 + 4m), 시작 자세 접근, YOLO 게이팅.

노드를 spin 하지 않고 콜백(on_odom, on_complete, tick)을 직접 호출한다.
파라미터는 rclpy.init 의 --ros-args 로 넣고, 발행은 publisher 를 Spy 로 바꿔 기록한다.
"""
import pytest
import rclpy
from nav_msgs.msg import Odometry
from std_msgs.msg import String

from kaboat_mission.mission_manager import MISSION_ORDER, MissionManager

WAYPOINTS = {
    'gate': (8.0, 0.0), 'station': (8.0, 8.0), 'dock': (0.0, 8.0),
    'search': (-8.0, 8.0), 'avoid': (-8.0, 0.0),
}


class Spy:
    def __init__(self):
        self.msgs = []

    def publish(self, msg):
        self.msgs.append(msg)

    @property
    def data(self):
        return [m.data for m in self.msgs]


def _odom(x, y):
    msg = Odometry()
    msg.header.frame_id = 'odom'
    msg.pose.pose.position.x = float(x)
    msg.pose.pose.position.y = float(y)
    msg.pose.pose.orientation.w = 1.0
    return msg


def _manager(*params):
    """auto_start=false 로 만들고 Spy 를 꽂은 뒤 advance() 로 시작한다."""
    args = ['--ros-args', '-p', 'auto_start:=false']
    for p in params:
        args += ['-p', p]
    rclpy.init(args=args)
    node = MissionManager()
    node.pub_state = Spy()
    node.pub_goal = Spy()
    node.pub_entry_goal = Spy()
    node.pub_detector_enable = Spy()
    node.advance()
    return node


@pytest.fixture
def make_manager():
    nodes = []

    def factory(*params):
        node = _manager(*params)
        nodes.append(node)
        return node
    yield factory
    for node in nodes:
        node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


def _finish(node, state):
    node.on_complete(String(data=state))
    node.on_odom(_odom(*WAYPOINTS[state]))
    node.tick()


@pytest.mark.check('A')
def test_missions_run_in_fixed_order_to_done(make_manager):
    node = make_manager('entry_required:=false')
    assert node.pub_state.data == ['gate']
    for state in MISSION_ORDER:
        assert node.pub_state.data[-1] == state
        _finish(node, state)
    assert node.pub_state.data == MISSION_ORDER + ['done']
    assert node.phase == 'done'


@pytest.mark.check('A')
def test_entry_approach_precedes_each_mission(make_manager):
    ready = [f'entry_ready.{s}:=true' for s in MISSION_ORDER]
    node = make_manager('entry_waypoints.gate:=[1.0, 2.0, 90.0]', *ready)
    assert node.pub_state.data == ['approach']
    goal = node.pub_entry_goal.msgs[-1]
    assert (goal.pose.position.x, goal.pose.position.y) == (1.0, 2.0)
    assert goal.pose.orientation.z == pytest.approx(0.7071, abs=1e-4)
    node.tick()   # 접근 중에는 시작 자세를 계속 다시 발행한다
    assert len(node.pub_entry_goal.msgs) == 2
    node.on_complete(String(data='approach'))
    assert node.pub_state.data == ['approach', 'gate']
    assert node.pub_goal.msgs[-1].pose.position.x == WAYPOINTS['gate'][0]


@pytest.mark.check('B')
def test_transition_needs_both_completion_and_radius(make_manager):
    node = make_manager('entry_required:=false')
    node.on_complete(String(data='gate'))
    node.on_odom(_odom(8.0 - 4.5, 0.0))   # 4.5m — 반경 밖
    node.tick()
    assert node.pub_state.data == ['gate']
    node.on_odom(_odom(8.0 - 3.9, 0.0))   # 완료 신호는 기억된 상태로 반경 진입
    node.tick()
    assert node.pub_state.data == ['gate', 'station']


@pytest.mark.check('B')
def test_radius_alone_does_not_advance(make_manager):
    node = make_manager('entry_required:=false')
    node.on_odom(_odom(*WAYPOINTS['gate']))
    for _ in range(10):
        node.tick()
    assert node.pub_state.data == ['gate']


@pytest.mark.check('B')
def test_transition_radius_parameter(make_manager):
    node = make_manager('entry_required:=false', 'transition_radius:=1.0')
    node.on_complete(String(data='gate'))
    node.on_odom(_odom(8.0 - 1.5, 0.0))
    node.tick()
    assert node.pub_state.data == ['gate']
    node.on_odom(_odom(8.0 - 0.9, 0.0))
    node.tick()
    assert node.pub_state.data[-1] == 'station'


@pytest.mark.check('B')
def test_single_mission_stops_at_done(make_manager):
    node = make_manager('entry_required:=false', 'start_state:=dock', 'single_mission:=true')
    assert node.pub_state.data == ['dock']
    _finish(node, 'dock')
    assert node.pub_state.data == ['dock', 'done']
    node.on_complete(String(data='search'))
    node.tick()
    assert node.pub_state.data == ['dock', 'done']


@pytest.mark.check('B')
def test_unknown_start_state_starts_from_first_mission(make_manager):
    node = make_manager('entry_required:=false', 'start_state:=tournament')
    assert node.pub_state.data == ['gate']


@pytest.mark.check('B')
def test_malformed_entry_waypoint_is_rejected():
    rclpy.init(args=['--ros-args', '-p', 'auto_start:=false',
                     '-p', 'entry_waypoints.dock:=[1.0, 2.0]'])
    try:
        with pytest.raises(ValueError):
            MissionManager()
    finally:
        rclpy.shutdown()


@pytest.mark.check('C')
def test_missing_entry_pose_blocks_in_idle(make_manager):
    node = make_manager()   # entry_ready 기본값 false
    assert node.pub_state.data == ['idle']
    assert node.phase == 'blocked'
    node.on_complete(String(data='approach'))
    node.on_complete(String(data='gate'))
    node.on_odom(_odom(*WAYPOINTS['gate']))
    node.tick()
    assert node.pub_state.data == ['idle']


@pytest.mark.check('C')
def test_foreign_completion_and_missing_odom_do_not_advance(make_manager):
    node = make_manager('entry_required:=false')
    node.on_complete(String(data='dock'))   # 현재 미션이 아닌 완료 신호
    node.tick()                               # odom 없음
    node.on_odom(_odom(*WAYPOINTS['gate']))
    node.tick()
    assert node.pub_state.data == ['gate']
    assert not node.behavior_complete


@pytest.mark.check('C')
def test_completion_does_not_carry_into_next_mission(make_manager):
    node = make_manager('entry_required:=false')
    _finish(node, 'gate')
    assert node.pub_state.data[-1] == 'station'
    node.on_odom(_odom(*WAYPOINTS['station']))
    node.tick()
    assert node.pub_state.data[-1] == 'station'


@pytest.mark.check('D')
def test_detector_enabled_only_during_dock(make_manager):
    node = make_manager('entry_required:=false')
    for state in MISSION_ORDER:
        _finish(node, state)
    states = node.pub_state.data
    enables = node.pub_detector_enable.data
    assert len(states) == len(enables)
    assert all(enable == (state == 'dock') for state, enable in zip(states, enables))
