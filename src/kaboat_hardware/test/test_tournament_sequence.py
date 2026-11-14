"""tournament_sequence: 출발 직진 → goto → 회피 시험 출발, /mission/state 로 cmd_mux 를 넘긴다."""

import math

import pytest
import rclpy
from nav_msgs.msg import Odometry
from rclpy.parameter import Parameter
from std_msgs.msg import String

from kaboat_hardware import tournament_sequence as ts
from kaboat_mission.mission_manager import UNDOCK_LINEAR
from kaboat_mission.mission_points import MissionPoint, PointStore

ORIGIN = (35.18487880, 128.56510100, 30.0)
START = (40.0, 10.0)
GOAL = (58.0, 10.0)               # 시작점에서 +X 쪽 18 m


def test_start_condition_uses_bearing_to_goal():
    ok, dist, err = ts.start_condition((40.0, 12.0), 0.0, START, GOAL, 3.0, 45.0)
    assert ok and dist == pytest.approx(2.0) and err == pytest.approx(6.34, abs=0.01)
    assert not ts.start_condition((44.0, 10.0), 0.0, START, GOAL, 3.0, 45.0)[0]     # 멂
    assert not ts.start_condition((40.0, 10.0), 90.0, START, GOAL, 3.0, 45.0)[0]    # 틀어짐
    assert ts.start_condition((40.0, 10.0), -40.0, START, GOAL, 3.0, 45.0)[0]


def test_undock_clock_counts_only_auto_and_caps_jumps():
    c = ts.UndockClock(1.0, period=0.05)
    assert not c.update(0.0, True)
    assert not c.update(0.1, True)
    assert not c.update(0.5, False)
    assert not c.update(5.0, True)          # 밀려도 0.2 까지만
    assert c.elapsed == pytest.approx(0.3)


class _Rec:
    def __init__(self, echo=None):
        self.msgs = []
        self.echo = echo        # cmd_mux 처럼 /cmd_vel 로 되돌려 줌 (None = 안 줌)

    def publish(self, msg):
        self.msgs.append(msg)
        if self.echo is not None:
            self.echo(msg)


class _Future:
    """result None = 응답이 안 옴 (유실). deliver() 로 나중에 보낼 수 있다."""

    def __init__(self, result):
        self._result = result
        self._cb = None

    def add_done_callback(self, cb):
        self._cb = cb
        if self._result is not None:
            cb(self)

    def deliver(self, ok, message):
        self._result = type('R', (), {'success': ok, 'message': message})()
        self._cb(self)

    def result(self):
        return self._result


class _Client:
    def __init__(self, replies):
        self.replies = list(replies)
        self.calls = 0
        self.futures = []

    def service_is_ready(self):
        return True

    def call_async(self, _req):
        self.calls += 1
        reply = self.replies.pop(0)
        fut = _Future(None if reply is None else
                      type('R', (), {'success': reply[0], 'message': reply[1]})())
        self.futures.append(fut)
        return fut


@pytest.fixture
def node(tmp_path):
    o = tmp_path / 'origin.yaml'
    o.write_text(f'origin_lat: {ORIGIN[0]}\norigin_lon: {ORIGIN[1]}\norigin_alt: {ORIGIN[2]}\n')
    p = tmp_path / 'points.json'
    PointStore(ORIGIN, {'tournament_entry': MissionPoint(*START, 0.0),
                        'tournament_end': MissionPoint(*GOAL, 0.0)}).save(str(p))
    rclpy.init()
    n = ts.TournamentSequence(parameter_overrides=[
        Parameter('points_file', value=str(p)), Parameter('origin_file', value=str(o))])
    clock = {'t': 100.0}
    n._now = lambda: clock['t']
    n.clock = clock
    n.state_rec, n.undock_rec = _Rec(), _Rec(echo=n._on_cmd_vel)
    n._mux_ready = lambda: True
    n.state_pub, n.undock_pub = n.state_rec, n.undock_rec
    yield n
    n.destroy_node()
    rclpy.shutdown()


def _odom(node, x, y, yaw_deg):
    m = Odometry()
    m.pose.pose.position.x, m.pose.pose.position.y = x, y
    m.pose.pose.orientation.z = math.sin(math.radians(yaw_deg) / 2)
    m.pose.pose.orientation.w = math.cos(math.radians(yaw_deg) / 2)
    node._on_odom(m)


def _run(node, seconds, auto=True, step=0.05):
    for _ in range(int(round(seconds / step))):
        node.clock['t'] += step
        if auto:
            node._on_motor(String(data='AUTO'))
        node._tick()


def test_undock_then_approach_then_avoid_without_gaps(node):
    assert node.phase == 'undock'
    _run(node, 1.0, auto=False)                                  # 모터 상태를 아직 못 받음
    assert node.undock_rec.msgs == [] and node.undock_clock.elapsed == 0.0   # 직진 안 냄
    node._on_motor(String(data='TIMEOUT'))                       # 명령이 없어 TIMEOUT — 받았으니 낸다
    _run(node, 0.5, auto=False)
    assert node.undock_rec.msgs and node.undock_clock.elapsed == 0.0         # 내지만 AUTO 전엔 안 셈
    assert all(m.linear.x == UNDOCK_LINEAR for m in node.undock_rec.msgs)
    _run(node, 3.1)
    assert node.phase == 'approach'
    assert [m.data for m in node.state_rec.msgs] == ['approach']   # cmd_mux 가 바로 goto 명령으로

    node.start_client = _Client([(False, 'wait for a valid preview B-spline path'),
                                 (True, 'avoidance test armed')])
    _odom(node, 30.0, 10.0, 0.0)                                 # 10 m 밖
    _run(node, 1.0)
    assert node.start_client.calls == 0
    _odom(node, 38.0, 10.0, 0.0)                                 # 2 m 안, 목표 방향
    _run(node, 0.4)
    _odom(node, 38.0, 10.0, 0.0)
    assert node.start_client.calls == 0                          # 0.5 s 유지 전
    _run(node, 0.2)
    _odom(node, 38.0, 10.0, 0.0)
    assert node.start_client.calls == 1 and node.phase == 'approach'   # 거절 → goto 계속
    _run(node, 0.3)
    _odom(node, 38.0, 10.0, 0.0)
    assert node.start_client.calls == 1                          # 0.5 s 뒤 다시
    _run(node, 0.3)
    assert node.start_client.calls == 2 and node.phase == 'avoid'
    assert [m.data for m in node.state_rec.msgs] == ['approach', 'avoid']
    _run(node, 1.0)
    assert node.start_client.calls == 2                          # 출발 뒤엔 더 안 부름


def test_teleop_pauses_undock_and_stale_odom_blocks_start(node):
    _run(node, 1.0)
    node.clock['t'] += 0.05
    node._on_motor(String(data='TELEOP'))
    for _ in range(20):
        node.clock['t'] += 0.05
        node._tick()
    assert node.undock_clock.elapsed == pytest.approx(1.0, abs=0.06)    # 텔레옵 동안 안 셈
    _run(node, 2.1)
    assert node.phase == 'approach'
    node.start_client = _Client([(True, 'armed')])
    _odom(node, 40.0, 10.0, 0.0)
    node.clock['t'] += 1.5                                       # 그 뒤 /odom 이 1 s 넘게 안 옴
    _run(node, 2.0)
    assert node.start_client.calls == 0
    _odom(node, 40.0, 10.0, 0.0)                                 # 다시 오면 0.5 s 유지 후 출발
    _run(node, 0.6)
    assert node.start_client.calls == 1 and node.phase == 'avoid'


def test_no_undock_starts_in_approach(tmp_path):
    o = tmp_path / 'origin.yaml'
    o.write_text(f'origin_lat: {ORIGIN[0]}\norigin_lon: {ORIGIN[1]}\norigin_alt: {ORIGIN[2]}\n')
    p = tmp_path / 'points.json'
    PointStore(ORIGIN, {'tournament_entry': MissionPoint(*START, 0.0),
                        'tournament_end': MissionPoint(*GOAL, 0.0)}).save(str(p))
    rclpy.init()
    try:
        n = ts.TournamentSequence(parameter_overrides=[
            Parameter('points_file', value=str(p)), Parameter('origin_file', value=str(o)),
            Parameter('undock_sec', value=0.0)])
        assert n.phase == 'approach'
        n.destroy_node()
        with pytest.raises(ValueError):
            ts.TournamentSequence(parameter_overrides=[
                Parameter('points_file', value=str(p)), Parameter('origin_file', value=str(o)),
                Parameter('start_dist', value=0.0)])
    finally:
        rclpy.shutdown()


def _to_approach(node):
    _run(node, 3.2)
    assert node.phase == 'approach'


def test_lost_reply_times_out_and_already_running_counts_as_started(node):
    _to_approach(node)
    node.start_client = _Client([None, (False, 'avoidance test already running')])
    for _ in range(12):                                          # 0.6 s — 유지 0.5 s 지나 호출
        _odom(node, 38.0, 10.0, 0.0)
        _run(node, 0.05)
    assert node.start_client.calls == 1 and node.phase == 'approach'   # 응답 없음
    for _ in range(36):                                          # 1.8 s 더 — 아직 2 s 안
        _odom(node, 38.0, 10.0, 0.0)
        _run(node, 0.05)
    assert node.start_client.calls == 1
    for _ in range(6):                                           # 2 s 넘음 → 버리고 다시
        _odom(node, 38.0, 10.0, 0.0)
        _run(node, 0.05)
    assert node.start_client.calls == 2 and node.phase == 'avoid'      # already running = 출발함
    node.start_client.futures[0].deliver(True, 'avoidance test armed')  # 옛 응답이 늦게 와도
    assert [m.data for m in node.state_rec.msgs] == ['approach', 'avoid']   # 두 번 안 넘김


def test_running_status_switches_to_avoid_without_any_reply(node):
    _to_approach(node)
    node.start_client = _Client([None])
    for _ in range(12):
        _odom(node, 38.0, 10.0, 0.0)
        _run(node, 0.05)
    assert node.start_client.calls == 1 and node.phase == 'approach'
    node._on_test(String(data='FOLLOW: plan_acquired'))         # 시험은 출발했는데 응답 유실
    _run(node, 0.05)
    assert node.phase == 'avoid'


def test_preview_status_does_not_switch(node):
    _to_approach(node)
    node._on_test(String(data='PREVIEW: obstacles=0, front=None'))
    node._on_test(String(data='WAIT: waiting for /scan'))
    _run(node, 0.5)
    assert node.phase == 'approach'


def test_other_cmd_vel_publisher_blocks_the_undock(node):
    counts = {'/cmd_vel': 2, '/mission/state': 1}                # 다른 터미널의 test_tournament 등
    node.count_publishers = lambda topic: counts[topic]
    _run(node, 2.0)
    assert node.undock_rec.msgs == [] and node.undock_clock.elapsed == 0.0
    counts['/cmd_vel'] = 1                                       # 끄면 이어서
    _run(node, 3.2)
    assert node.phase == 'approach' and node.undock_rec.msgs
    counts['/cmd_vel'] = 2                                       # 그 뒤 단계에선 경고만 — 멈추지 않음
    node.start_client = _Client([(True, 'avoidance test armed')])
    for _ in range(12):
        _odom(node, 38.0, 10.0, 0.0)
        _run(node, 0.05)
    assert node.phase == 'avoid'


def test_undock_waits_for_cmd_mux_and_counts_only_what_reaches_cmd_vel(node):
    ready = {'mux': False}
    node._mux_ready = lambda: ready['mux']
    _run(node, 1.0)
    assert node.undock_rec.msgs == [] and node.undock_clock.elapsed == 0.0   # cmd_mux 안 보임 → 안 냄
    ready['mux'] = True
    node.undock_rec.echo = None                                  # cmd_mux 가 아직 undock state 를 못 받음
    _run(node, 1.5)
    assert node.undock_rec.msgs and node.undock_clock.elapsed == 0.0         # 내지만 /cmd_vel 에 안 보여 안 셈
    node.undock_rec.echo = node._on_cmd_vel                      # 이제 통과
    _run(node, 2.0)
    assert node.phase == 'undock' and node.undock_clock.elapsed == pytest.approx(1.95, abs=0.06)
    _run(node, 1.1)
    assert node.phase == 'approach'


def test_echo_of_another_command_does_not_count(node):
    node.undock_rec.echo = None
    for _ in range(40):
        node.clock['t'] += 0.05
        node._on_motor(String(data='AUTO'))
        from geometry_msgs.msg import Twist
        other = Twist()
        other.linear.x = 0.4                                     # goto 명령 등 — 직진 아님
        node._on_cmd_vel(other)
        node._tick()
    assert node.phase == 'undock' and node.undock_clock.elapsed == 0.0
