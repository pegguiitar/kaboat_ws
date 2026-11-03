"""L3 전체 흐름 — mission_manager + 6개 behavior(approach 포함) + cmd_mux 를 한 버스에 연결.

각 behavior 의 내부 판단은 L1·L2 에서 확인했으므로, 여기서는 미션 전환 순서와
명령 선택만 본다. 미션 완료는 /mission/complete 를 직접 넣고, 배 위치는 선체 모델 대신
목표 근처로 옮겨 놓는다(순간 이동). 시작 자세 접근(approach)만은 실제 behavior 가
도달을 판정해 완료 신호를 낸다.
"""
import math

import pytest
import rclpy
from std_msgs.msg import String

from kaboat_behaviors.docking_ctrl import DockingCtrl
from kaboat_behaviors.gate_follower import GateFollower
from kaboat_behaviors.mission_approach import MissionApproach
from kaboat_behaviors.obstacle_planner import ObstaclePlanner
from kaboat_behaviors.search_circler import SearchCircler
from kaboat_behaviors.station_keeper import StationKeeper
from kaboat_control.cmd_mux import STATE_TOPIC_MAP, CmdMux
from kaboat_mission.mission_manager import DEFAULT_WAYPOINTS, MISSION_ORDER, MissionManager
from tests.sim.boat_model import BoatState
from tests.sim.ros_adapters import FakeClock, odom_msg
from tests.system.bus import Bus, wire_behavior, wire_cmd_mux, wire_mission_manager

ENTRY = {   # 미션별 시작 자세 [x, y, 선수각 deg]
    'gate': (0.0, 0.0, 0.0), 'station': (8.0, 3.0, 90.0), 'dock': (3.0, 8.0, 180.0),
    'search': (-4.0, 8.0, 180.0), 'avoid': (-8.0, 4.0, -90.0),
}
BEHAVIORS = (MissionApproach, GateFollower, StationKeeper, DockingCtrl, SearchCircler,
             ObstaclePlanner)
EXPECTED = []
for _m in MISSION_ORDER:
    EXPECTED += ['approach', _m]
EXPECTED.append('done')


class System:
    def __init__(self, monkeypatch, extra_params=()):
        params = ['auto_start:=false', 'approach.motion_enabled:=true']
        for name, (x, y, a) in ENTRY.items():
            params += [f'entry_waypoints.{name}:=[{x}, {y}, {a}]', f'entry_ready.{name}:=true']
        params += list(extra_params)
        args = ['--ros-args']
        for p in params:
            args += ['-p', p]
        rclpy.init(args=args)
        self.clock = FakeClock()
        self.bus = Bus()
        self.manager = MissionManager()
        self.behaviors = {cls.STATE_NAME: cls() for cls in BEHAVIORS}
        self.mux = CmdMux()
        self.clock.install(monkeypatch, self.manager, self.mux, *self.behaviors.values())
        wire_mission_manager(self.bus, self.manager)
        for node in self.behaviors.values():
            wire_behavior(self.bus, node)
        wire_cmd_mux(self.bus, self.mux, STATE_TOPIC_MAP.values())
        self.pose = BoatState()
        self.silenced = set()
        self.steps = 0
        self.cmd_log = []     # [(state, {발행된 /cmd/* 토픽}, /cmd_vel 메시지 목록)]

    @property
    def state(self):
        msgs = self.bus.messages('/mission/state')
        return msgs[-1].data if msgs else ''

    def start(self):
        self.manager.advance()

    def teleport(self, x, y, yaw_deg=0.0):
        self.pose = BoatState(x=x, y=y, yaw=math.radians(yaw_deg))

    def step(self):
        """0.1 s: odom → behavior 10 Hz → manager 5 Hz → cmd_mux 워치독."""
        self.clock.advance(0.1)
        self.steps += 1
        msg = odom_msg(self.pose)
        msg.header.stamp = rclpy.time.Time(nanoseconds=int(self.clock.now * 1e9)).to_msg()
        mark = len(self.bus.log)
        state = self.state
        self.bus.publish('/odom', msg)
        for name, node in self.behaviors.items():
            if name not in self.silenced:
                node._tick()
        if self.steps % 2 == 0:
            self.manager.tick()
        self.mux.watchdog()
        new = self.bus.log[mark:]
        topics = {t for t, _ in new if t.startswith('/cmd/')}
        vel = [m for t, m in new if t == '/cmd_vel']
        # 버스가 동기라 한 주기 안에서 state 가 바뀔 수 있다 (approach 완료 → 다음 미션).
        self.cmd_log.append(((state, self.state), topics, vel))

    def run_until_state_changes(self, max_steps=50):
        before = self.state
        for _ in range(max_steps):
            self.step()
            if self.state != before:
                return True
        return False

    def finish_current(self):
        """현재 state 를 끝낸다: approach 는 실제 도달, 미션은 목표 근처 + 완료 신호."""
        state = self.state
        if state == 'approach':
            mission = MISSION_ORDER[self.manager.idx]
            self.teleport(*ENTRY[mission])
        else:
            gx, gy = DEFAULT_WAYPOINTS[state]
            self.teleport(gx + 1.0, gy)                  # 전환 반경 4 m 이내
            self.bus.publish('/mission/complete', String(data=state))
        assert self.run_until_state_changes(), f"'{state}' 에서 전환되지 않음"

    def close(self):
        for node in (self.manager, self.mux, *self.behaviors.values()):
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


@pytest.fixture
def system(monkeypatch):
    sys_ = System(monkeypatch)
    yield sys_
    sys_.close()


def _state_sequence(bus):
    out = []
    for m in bus.messages('/mission/state'):
        if not out or out[-1] != m.data:
            out.append(m.data)
    return out


@pytest.mark.check('A')
def test_full_sequence_with_entry_approach_reaches_done(system):
    system.start()
    while system.state != 'done':
        system.finish_current()
    assert _state_sequence(system.bus) == EXPECTED


@pytest.mark.check('A')
def test_only_active_behavior_commands_and_mux_passes_only_it(system):
    system.start()
    while system.state != 'done':
        system.step()
        system.finish_current()
    for states, topics, vel in system.cmd_log:
        allowed = {STATE_TOPIC_MAP[s] for s in states if s in STATE_TOPIC_MAP}
        assert topics <= allowed, f'{states} 중 다른 behavior 발행 {topics}'
    # /cmd_vel 로 나간 명령은 같은 tick 의 활성 behavior 명령이거나 정지 명령이다
    for states, topics, vel in system.cmd_log:
        for m in vel:
            assert topics or (m.linear.x == 0.0 and m.angular.z == 0.0)


@pytest.mark.check('B')
def test_completion_far_from_goal_waits_until_radius(system):
    system.start()
    system.finish_current()                      # approach → gate
    assert system.state == 'gate'
    system.teleport(0.0, 0.0)                    # gate 목표 (8, 0) 에서 8 m
    system.bus.publish('/mission/complete', String(data='gate'))
    for _ in range(20):
        system.step()
    assert system.state == 'gate'
    system.teleport(5.0, 0.0)                    # 3 m — 기억된 완료 신호로 전환
    assert system.run_until_state_changes()
    assert system.state == 'approach'


@pytest.mark.check('C')
def test_silent_behavior_triggers_mux_stop(system):
    system.start()
    for _ in range(3):
        system.finish_current()                  # approach → gate → approach → station
    assert system.state == 'station'
    system.teleport(0.0, 0.0)                    # station 목표 (8, 8) 에서 멀리 — 추력 명령이 나온다
    for _ in range(3):
        system.step()
    assert any(m.linear.x != 0.0 for m in system.cmd_log[-1][2])
    system.silenced.add('station')               # station behavior 가 죽음
    for _ in range(4):                           # 0.4 s > 워치독 0.3 s
        system.step()
    last = system.cmd_log[-1][2]
    assert last and all(m.linear.x == 0.0 and m.angular.z == 0.0 for m in last)


@pytest.mark.check('C')
def test_inactive_behaviors_cannot_complete_the_mission(system):
    system.start()
    system.finish_current()                      # approach → gate
    system.teleport(*DEFAULT_WAYPOINTS['gate'])
    mark = len(system.bus.messages('/mission/complete'))   # 앞의 approach 완료는 빼고 센다
    for name, node in system.behaviors.items():
        if name != 'gate':
            node.report_complete()               # 비활성 behavior 는 발행 자체를 하지 않는다
    for _ in range(10):
        system.step()
    assert system.state == 'gate'
    assert system.bus.messages('/mission/complete')[mark:] == []


@pytest.mark.check('D')
def test_detector_enabled_only_during_dock_and_stop_after_done(system):
    system.start()
    while system.state != 'done':
        system.finish_current()
    states = [m.data for m in system.bus.messages('/mission/state')]
    enables = [m.data for m in system.bus.messages('/detector/enable')]
    assert all(e == (s == 'dock') for s, e in zip(states, enables))
    for _ in range(5):
        system.step()
    vel = [m for _, _, v in system.cmd_log[-5:] for m in v]
    assert vel and all(m.linear.x == 0.0 and m.angular.z == 0.0 for m in vel)
