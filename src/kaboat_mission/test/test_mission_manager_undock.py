"""출발 직진 (슬롯 탈출, 2026-11-13): 대회 출발은 슬롯 안에서 한다 — 처음부터 도는 전체 실행은
3 s 동안 텔레옵 4단계(좌우 1700 µs) 직진 뒤 gate_entry 로 접근. 3 s 는 모터가 AUTO 인 시간만 센다."""

import importlib.util
import os

import pytest

rclpy = pytest.importorskip('rclpy')
from geometry_msgs.msg import Twist  # noqa: E402
from launch import LaunchContext  # noqa: E402
from launch.actions import DeclareLaunchArgument, LogInfo  # noqa: E402
from rclpy.parameter import Parameter  # noqa: E402
from std_msgs.msg import String  # noqa: E402

from kaboat_mission.mission_manager import (  # noqa: E402
    UNDOCK_LINEAR, UNDOCK_PERIOD, UNDOCK_SEC, MissionManager)

HERE = os.path.dirname(__file__)
LAUNCH = os.path.join(HERE, '..', '..', 'kaboat_bringup', 'launch', 'autonomy.launch.py')


@pytest.fixture
def ros():
    rclpy.init()
    yield
    rclpy.shutdown()


class Rig:
    """mission_manager + 가짜 시계·발행 기록."""

    def __init__(self, **params):
        overrides = [Parameter('points_file', value='none'),
                     Parameter('entry_ready.gate', value=True),
                     Parameter('entry_waypoints.gate', value=[5.0, 1.0, 90.0])]
        overrides += [Parameter(k, value=v) for k, v in params.items()]
        self.clock = [100.0]
        self.cmds, self.states = [], []
        node = MissionManager.__new__(MissionManager)
        node._now = lambda: self.clock[0]           # 생성자 안 auto_start 부터 가짜 시계
        MissionManager.__init__(node, parameter_overrides=overrides)
        self.node = node
        node.pub_undock.publish = self.cmds.append
        node.pub_state.publish = lambda m: self.states.append(m.data)

    def status(self, mode):
        self.node.on_thruster_status(String(data=mode))

    def run(self, seconds, mode=None):
        """seconds 동안 UNDOCK_PERIOD 마다 (모터 상태 mode 를 5 Hz 로 받으며) undock_step."""
        n = int(round(seconds / UNDOCK_PERIOD))
        for k in range(n):
            self.clock[0] += UNDOCK_PERIOD
            if mode is not None and k % 4 == 0:
                self.status(mode)
            self.node.undock_step()


def test_teleop_level4_pwm():
    # thruster_driver: 4단계 = 1500 + 200 µs, 자율 명령은 1525 + ratio × (1900 − 1525)
    assert round(1525 + UNDOCK_LINEAR * (1900 - 1525)) == 1700
    assert UNDOCK_SEC == 3.0


def test_full_run_drives_straight_3s_on_auto_then_approaches_gate(ros):
    rig = Rig()                                   # auto_start 기본 true, start_state 없음
    node = rig.node
    try:
        assert node.phase == 'undock' and node.idx == -1
        assert node.pub_state is not None
        rig.run(1.0)                              # 모터 상태 아직 없음 (기동 중) — 세지 않음
        assert node.phase == 'undock' and node.undock_elapsed == 0.0
        assert rig.cmds and all(c.linear.x == pytest.approx(UNDOCK_LINEAR)
                                and c.angular.z == 0.0 for c in rig.cmds)
        rig.run(2.0, mode='AUTO')
        assert node.phase == 'undock'
        assert node.undock_elapsed == pytest.approx(2.0, abs=0.06)
        rig.run(1.2, mode='AUTO')                 # 3 s 채움 → gate 접근 (접근 노드가 이어받음)
        assert node.phase == 'approach' and node.idx == 0
        assert rig.states[-1] == 'approach'
        assert all(c.linear.x == pytest.approx(UNDOCK_LINEAR) for c in rig.cmds)
        assert node._undock_timer is None
        n = len(rig.cmds)
        node.undock_step()                        # 끝난 뒤엔 아무것도 안 냄
        assert len(rig.cmds) == n
    finally:
        node.destroy_node()


def test_pauses_while_teleop_or_estop_and_resumes(ros):
    rig = Rig()
    node = rig.node
    try:
        rig.run(1.0, mode='AUTO')
        done = node.undock_elapsed
        rig.run(2.0, mode='TELEOP')               # 텔레옵 중엔 직진이 안 들어감 — 안 셈
        rig.run(2.0, mode='ESTOP')
        assert node.undock_elapsed == pytest.approx(done, abs=0.06)
        assert node.phase == 'undock'
        rig.run(2.2, mode='AUTO')                 # 놓으면 남은 만큼 이어서
        assert node.phase == 'approach'
    finally:
        node.destroy_node()


def test_stale_motor_status_is_not_counted(ros):
    rig = Rig()
    node = rig.node
    try:
        rig.status('AUTO')
        rig.run(3.5)                              # 상태가 1 s 넘게 안 옴 (드라이버 죽음)
        assert node.phase == 'undock'
        assert 0.8 <= node.undock_elapsed <= 1.05  # 마지막 상태 뒤 1 s 까지만 셈
    finally:
        node.destroy_node()


def test_wait_auto_off_counts_plain_time(ros):
    rig = Rig(undock_wait_auto=False)
    try:
        rig.run(3.1)
        assert rig.node.phase == 'approach'
    finally:
        rig.node.destroy_node()


@pytest.mark.parametrize('params', [
    {'start_state': 'gate'},                      # 미션부터 시작 (go_*·test_mission 계열)
    {'start_state': 'station', 'single_mission': True},
    {'single_mission': True},
    {'undock_sec': 0.0},                          # 끔
])
def test_no_undock_unless_full_run_from_start(ros, params):
    rig = Rig(**params)
    try:
        assert rig.node.phase in ('approach', 'blocked')
        assert 'undock' not in rig.states
        assert rig.cmds == []
    finally:
        rig.node.destroy_node()


@pytest.mark.parametrize('params', [{'undock_sec': -1.0}, {'undock_linear': 0.0},
                                    {'undock_linear': 1.5}])
def test_bad_undock_params_refused(ros, params):
    with pytest.raises(ValueError):
        Rig(**params)


def test_cmd_mux_passes_undock_only_in_undock_state(ros):
    from kaboat_control.cmd_mux import STATE_TOPIC_MAP, CmdMux
    assert STATE_TOPIC_MAP['undock'] == '/cmd/undock'
    mux = CmdMux()
    out = []
    mux.pub.publish = out.append
    try:
        go = Twist()
        go.linear.x = UNDOCK_LINEAR
        mux.on_state(String(data='undock'))
        mux.on_cmd('/cmd/undock', go)
        mux.on_cmd('/cmd/approach', Twist())       # 다른 단계 명령은 버림
        assert out == [go]
        mux.on_state(String(data='approach'))
        mux.on_cmd('/cmd/undock', go)              # 접근으로 넘어가면 직진은 버림
        assert out == [go]
    finally:
        mux.destroy_node()


# ── launch ────────────────────────────────────────────

@pytest.fixture(scope='module')
def mod():
    spec = importlib.util.spec_from_file_location('autonomy_launch_undock', LAUNCH)
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m


def _context(mod, **args):
    context = LaunchContext()
    for action in mod.generate_launch_description().entities:
        if isinstance(action, DeclareLaunchArgument):
            context.launch_configurations[action.name] = ''.join(
                s.perform(context) for s in action.default_value)
    context.launch_configurations.update(args)
    return context


def _get(context):
    return lambda name: context.launch_configurations.get(name, '')


def _summary(mod, context):
    log = next(a for a in mod._stack(context) if isinstance(a, LogInfo))
    return ''.join(s.perform(context) for s in log.msg)


def test_launch_undock_only_when_driving(mod):
    # run_autonomy (접근 주행 켬) → 3 s, 기본 미리보기 → 0 (배를 안 움직임)
    run = _context(mod, approach_motion_enabled='true')
    assert mod.undock_seconds(_get(run)) == 3.0
    assert '출발 직진 3.0 s' in _summary(mod, run)
    preview = _context(mod)
    assert mod.undock_seconds(_get(preview)) == 0.0
    assert '출발 직진' not in _summary(mod, preview)
    off = _context(mod, approach_motion_enabled='true', undock_sec='0')
    assert mod.undock_seconds(_get(off)) == 0.0
    # test_mission 은 mission_manager 가 안 씀 — 요약에도 안 나옴
    single = _context(mod, approach_motion_enabled='true', start_state='station',
                      single_mission='true')
    assert '출발 직진' not in _summary(mod, single)
    with pytest.raises(RuntimeError, match='undock_sec'):
        mod.undock_seconds(_get(_context(mod, undock_sec='abc')))
