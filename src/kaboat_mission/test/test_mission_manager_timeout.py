"""미션 제한 시간 (2026-11-13, 경기 15 분): 실패한 미션에서 무한정 기다리거나 재시도하지 않고 넘어간다.

gate 180·station 120·dock 240·search 180·avoid 180 s (합 900 s). 마감 = 전체 시작 + 앞 미션 몫의 합
(일찍 끝나면 남은 시간 이월). 접근만 90 s 넘어도 건너뜀. dock 은 먼저 직진 후진 3 m (30 s 상한).
"""

import pytest

rclpy = pytest.importorskip('rclpy')
from geometry_msgs.msg import Twist  # noqa: E402
from nav_msgs.msg import Odometry  # noqa: E402
from rclpy.parameter import Parameter  # noqa: E402
from std_msgs.msg import String  # noqa: E402

from kaboat_mission.mission_manager import (  # noqa: E402
    APPROACH_TIMEOUT, MISSION_ORDER, MISSION_TIME_LIMITS, UNDOCK_PERIOD, MissionManager)

T0 = 100.0


@pytest.fixture
def ros():
    rclpy.init()
    yield
    rclpy.shutdown()


class Rig:
    def __init__(self, ready=tuple(MISSION_ORDER), **params):
        overrides = [Parameter('points_file', value='none'),
                     Parameter('undock_sec', value=params.pop('undock_sec', 0.0))]
        for k, name in enumerate(MISSION_ORDER):
            overrides += [Parameter(f'entry_ready.{name}', value=name in ready),
                          Parameter(f'entry_waypoints.{name}', value=[float(k), 1.0, 0.0])]
        overrides += [Parameter(k, value=v) for k, v in params.items()]
        self.clock = [T0]
        self.states, self.retreats = [], []
        node = MissionManager.__new__(MissionManager)
        node._now = lambda: self.clock[0]
        MissionManager.__init__(node, parameter_overrides=overrides)
        self.node = node
        node.pub_state.publish = lambda m: self.states.append(m.data)
        node.pub_retreat.publish = self.retreats.append

    @property
    def state(self):
        n = self.node
        return None if not 0 <= n.idx < len(MISSION_ORDER) else (n.phase, MISSION_ORDER[n.idx])

    def at(self, t):
        self.clock[0] = T0 + t
        self.node.tick()

    def complete(self, name):
        self.node.on_complete(String(data=name))

    def odom(self, x, y):
        msg = Odometry()
        msg.pose.pose.position.x, msg.pose.pose.position.y = float(x), float(y)
        self.node.on_odom(msg)

    def finish(self, name, t):
        """name 미션을 t 에 정상 완료 (완료 신호 + 끝점 2 m 안)."""
        self.odom(*self.node.waypoints[name])
        self.complete(name)
        self.at(t)

    def destroy(self):
        self.node.destroy_node()


def test_budget_is_15_minutes():
    assert sum(MISSION_TIME_LIMITS.values()) == 900.0
    assert MISSION_TIME_LIMITS == {'gate': 180.0, 'station': 120.0, 'dock': 240.0,
                                   'search': 180.0, 'avoid': 180.0}
    assert APPROACH_TIMEOUT == 90.0


def test_approach_over_90s_skips_that_mission(ros):
    rig = Rig()
    try:
        assert rig.state == ('approach', 'gate')
        rig.at(89.9)
        assert rig.state == ('approach', 'gate')
        rig.at(90.0)                                   # 접근 90 s — gate 건너뛰고 station 접근
        assert rig.state == ('approach', 'station')
        assert rig.node.skipped[0][0] == 'gate' and '접근' in rig.node.skipped[0][1]
        # gate 몫의 남은 90 s 는 station 으로 이월: 마감 = 전체 시작 + 180 + 120
        assert rig.node.deadline == pytest.approx(T0 + 300.0)
    finally:
        rig.destroy()


def test_mission_over_its_share_moves_on(ros):
    rig = Rig()
    try:
        rig.at(10.0)
        rig.complete('approach')                       # gate 시작
        assert rig.state == ('mission', 'gate')
        rig.at(179.9)                                  # 게이트를 못 찾아 계속 기다리는 중
        assert rig.state == ('mission', 'gate')
        rig.at(180.0)
        assert rig.state == ('approach', 'station')
        assert rig.states[-1] == 'approach'
    finally:
        rig.destroy()


def test_early_finish_carries_time_over(ros):
    rig = Rig()
    try:
        rig.complete('approach')
        rig.finish('gate', 60.0)                       # gate 1 분에 끝
        assert rig.state == ('approach', 'station')
        assert rig.node.deadline == pytest.approx(T0 + 300.0)   # station 은 남은 2 분 + 2 분
        rig.complete('approach')
        rig.at(299.0)
        assert rig.state == ('mission', 'station')
        rig.at(300.0)
        assert rig.state == ('approach', 'dock')
        assert rig.node.deadline == pytest.approx(T0 + 540.0)
    finally:
        rig.destroy()


def test_dock_timeout_reverses_out_first(ros):
    rig = Rig(start_state='dock')                      # 이어 도는 실행 (dock → search → avoid)
    try:
        rig.complete('approach')
        rig.odom(10.0, 0.0)
        rig.at(240.0)                                  # dock 4 분 초과 — 슬롯 근처일 수 있음
        assert rig.node.phase == 'retreat' and rig.states[-1] == 'retreat'
        rig.clock[0] += UNDOCK_PERIOD
        rig.node.retreat_step()
        cmd = rig.retreats[-1]
        assert cmd.linear.x == pytest.approx(-0.05) and cmd.angular.z == 0.0
        rig.odom(8.5, 0.0)
        rig.node.retreat_step()
        assert rig.node.phase == 'retreat'
        rig.odom(7.0, 0.0)                             # 3 m 빠져나옴 → search 접근
        rig.node.retreat_step()
        assert rig.state == ('approach', 'search')
        assert rig.node._retreat_timer is None
    finally:
        rig.destroy()


def test_dock_retreat_gives_up_after_30s(ros):
    rig = Rig(start_state='dock')
    try:
        rig.complete('approach')
        rig.odom(10.0, 0.0)
        rig.at(240.0)
        rig.clock[0] += 29.9
        rig.node.retreat_step()                        # 못 움직여도 (걸림·텔레옵)
        assert rig.node.phase == 'retreat'
        rig.clock[0] += 0.2
        rig.node.retreat_step()
        assert rig.state == ('approach', 'search')
    finally:
        rig.destroy()


def test_dock_approach_timeout_does_not_reverse(ros):
    rig = Rig(start_state='dock')
    try:
        rig.at(90.0)                                   # 아직 도크에 안 감 — 후진 없이 넘김
        assert rig.state == ('approach', 'search')
        assert rig.retreats == [] and 'retreat' not in rig.states
    finally:
        rig.destroy()


def test_last_mission_timeout_finishes(ros):
    rig = Rig(start_state='avoid')
    try:
        rig.complete('approach')
        rig.at(180.0)
        assert rig.node.phase == 'done' and rig.states[-1] == 'done'
        assert [n for n, _ in rig.node.skipped] == ['avoid']
    finally:
        rig.destroy()


def test_missing_entry_is_skipped_not_stuck(ros):
    rig = Rig(ready=('gate', 'dock', 'search', 'avoid'))   # station_entry 기록 안 함
    try:
        rig.complete('approach')
        rig.finish('gate', 50.0)
        assert rig.state == ('approach', 'dock')
        assert rig.node.skipped == [('station', '시작 자세 미기록')]
        # station 몫(120 s)은 dock 으로 이월: 마감 = 180 + 120 + 240
        assert rig.node.deadline == pytest.approx(T0 + 540.0)
    finally:
        rig.destroy()


def test_single_mission_tests_have_no_timeouts(ros):
    rig = Rig(start_state='gate', single_mission=True)
    try:
        rig.at(1000.0)
        assert rig.state == ('approach', 'gate')
    finally:
        rig.destroy()
    rig = Rig(ready=('gate',), start_state='station', single_mission=True)
    try:
        assert rig.node.phase == 'blocked'                # 단일 시험은 예전처럼 멈춤
    finally:
        rig.destroy()


def test_timeouts_can_be_switched_off(ros):
    rig = Rig(mission_timeouts=False)
    try:
        rig.at(1000.0)
        assert rig.state == ('approach', 'gate')
    finally:
        rig.destroy()


def test_zero_share_means_no_limit_and_next_starts_its_own_clock(ros):
    rig = Rig(**{'time_limit.gate': 0.0})
    try:
        rig.complete('approach')
        rig.at(1000.0)
        assert rig.state == ('mission', 'gate')
        rig.finish('gate', 1000.0)
        assert rig.node.deadline == pytest.approx(T0 + 1000.0 + 120.0)
    finally:
        rig.destroy()


def test_run_clock_starts_when_undock_actually_drives(ros):
    rig = Rig(undock_sec=3.0)
    node = rig.node
    try:
        assert node.phase == 'undock' and node.run_started_at is None
        rig.clock[0] = T0 + 20.0                       # 텔레옵으로 잡고 있는 동안은 경기 시간 아님
        node.on_thruster_status(String(data='AUTO'))
        for _ in range(int(3.2 / UNDOCK_PERIOD)):
            rig.clock[0] += UNDOCK_PERIOD
            node.on_thruster_status(String(data='AUTO'))
            node.undock_step()
        assert rig.state == ('approach', 'gate')
        assert node.run_started_at == pytest.approx(T0 + 20.0 + UNDOCK_PERIOD)
        assert node.deadline == pytest.approx(node.run_started_at + 180.0)
    finally:
        rig.destroy()


@pytest.mark.parametrize('params', [{'time_limit.dock': -1.0}, {'approach_timeout_sec': -5.0},
                                    {'timeout_retreat_cmd': 0.0}])
def test_bad_timeout_params_refused(ros, params):
    with pytest.raises(ValueError):
        Rig(**params)


def test_cmd_mux_passes_retreat_only_in_retreat_state(ros):
    from kaboat_control.cmd_mux import STATE_TOPIC_MAP, CmdMux
    assert STATE_TOPIC_MAP['retreat'] == '/cmd/retreat'
    mux = CmdMux()
    out = []
    mux.pub.publish = out.append
    try:
        back = Twist()
        back.linear.x = -0.05
        mux.on_state(String(data='retreat'))
        mux.on_cmd('/cmd/retreat', back)
        mux.on_cmd('/cmd/dock', Twist())
        assert out == [back]
    finally:
        mux.destroy_node()
