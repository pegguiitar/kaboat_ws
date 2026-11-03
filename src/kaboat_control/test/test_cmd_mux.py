"""cmd_mux — 현재 state 명령만 통과, 300ms 워치독 정지, idle/done 상시 정지."""
import pytest
import rclpy
from geometry_msgs.msg import Twist
from rclpy.time import Time
from std_msgs.msg import String

from kaboat_control.cmd_mux import STATE_TOPIC_MAP, CmdMux


class Spy:
    def __init__(self):
        self.msgs = []

    def publish(self, msg):
        self.msgs.append(msg)


class Clock:
    def __init__(self):
        self.t = 100.0

    def now(self):
        return Time(nanoseconds=int(self.t * 1e9))


def _twist(linear, angular=0.0):
    msg = Twist()
    msg.linear.x = float(linear)
    msg.angular.z = float(angular)
    return msg


@pytest.fixture
def mux():
    rclpy.init(args=['--ros-args', '-p', 'timeout_ms:=300.0'])
    node = CmdMux()
    node.pub = Spy()
    clock = Clock()
    node.get_clock = lambda: clock
    node.clock = clock
    yield node
    node.destroy_node()
    rclpy.shutdown()


def _is_stop(msg):
    return msg.linear.x == 0.0 and msg.angular.z == 0.0


@pytest.mark.check('A')
def test_only_active_state_command_passes(mux):
    mux.on_state(String(data='gate'))
    mux.on_cmd('/cmd/gate', _twist(0.1, 0.02))
    mux.on_cmd('/cmd/avoid', _twist(0.5))
    assert [m.linear.x for m in mux.pub.msgs] == [0.1]
    assert mux.pub.msgs[0].angular.z == 0.02


@pytest.mark.check('A')
def test_every_mission_state_has_its_own_topic():
    assert set(STATE_TOPIC_MAP) == {'approach', 'gate', 'station', 'dock', 'search', 'avoid'}
    assert all(topic == f'/cmd/{state}' for state, topic in STATE_TOPIC_MAP.items())


@pytest.mark.check('B')
def test_state_switch_changes_passthrough(mux):
    mux.on_state(String(data='gate'))
    mux.on_cmd('/cmd/gate', _twist(0.1))
    mux.on_state(String(data='station'))
    mux.on_cmd('/cmd/gate', _twist(0.2))
    mux.on_cmd('/cmd/station', _twist(0.3))
    assert [m.linear.x for m in mux.pub.msgs] == [0.1, 0.3]


@pytest.mark.check('B')
def test_watchdog_boundary_at_timeout(mux):
    mux.on_state(String(data='avoid'))
    mux.on_cmd('/cmd/avoid', _twist(0.1))
    mux.clock.t += 0.29
    mux.watchdog()
    assert len(mux.pub.msgs) == 1          # 300ms 이내 — 정지 없음
    mux.clock.t += 0.02
    mux.watchdog()
    assert len(mux.pub.msgs) == 2 and _is_stop(mux.pub.msgs[-1])


@pytest.mark.check('C')
def test_watchdog_stops_when_active_behavior_goes_silent(mux):
    mux.on_state(String(data='dock'))
    mux.on_cmd('/cmd/dock', _twist(0.1))
    for _ in range(10):
        mux.clock.t += 0.05
        mux.watchdog()
    assert _is_stop(mux.pub.msgs[-1])
    # 다른 behavior 가 계속 명령을 보내도 정지는 풀리지 않는다
    mux.on_cmd('/cmd/gate', _twist(0.4))
    mux.clock.t += 0.05
    mux.watchdog()
    assert _is_stop(mux.pub.msgs[-1])


@pytest.mark.check('C')
@pytest.mark.parametrize('state', ['idle', 'done', ''])
def test_inactive_states_always_publish_stop(mux, state):
    mux.on_state(String(data=state))
    mux.watchdog()
    mux.watchdog()
    assert len(mux.pub.msgs) == 2 and all(_is_stop(m) for m in mux.pub.msgs)


@pytest.mark.check('D')
def test_no_output_before_first_command_of_active_state(mux):
    mux.on_state(String(data='search'))
    for _ in range(5):
        mux.clock.t += 0.1
        mux.watchdog()
    assert mux.pub.msgs == []
