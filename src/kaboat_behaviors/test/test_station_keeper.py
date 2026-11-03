"""station_keeper — 목표점 5m 이내 5초 유지 시 완료, 유지 중에도 목표로 복귀 명령."""
import math
from types import SimpleNamespace

import pytest
from std_msgs.msg import String

from kaboat_behaviors.station_keeper import HOLD_RADIUS, HOLD_TIME, StationKeeper
from node_helpers import BehaviorHarness, is_stop


@pytest.fixture
def h(monkeypatch):
    harness = BehaviorHarness(StationKeeper, monkeypatch)
    harness.activate()
    harness.set_goal(10.0, 0.0)
    yield harness
    harness.close()


def _hold_ticks():
    return int(round(HOLD_TIME / 0.1))


@pytest.mark.check('A')
def test_approaches_goal_from_outside_hold_radius(h):
    h.set_pose(0.0, 0.0)
    cmd = h.tick()
    assert cmd.linear.x > 0.0
    assert h.completions == []


@pytest.mark.check('A')
def test_reports_complete_after_hold_time_inside_radius(h):
    h.set_pose(7.0, 0.0)            # 3m — 유지 반경 안
    h.tick(_hold_ticks())           # 첫 tick 이 진입 시각 — 아직 5s 미만
    assert h.completions == []
    h.tick(1)
    assert h.completions == ['station']


@pytest.mark.check('A')
def test_keeps_steering_back_to_goal_while_holding(h):
    h.set_pose(8.0, 1.0)
    cmd = h.tick()
    assert cmd.linear.x > 0.0
    assert cmd.angular.z < 0.0      # 목표가 우측 아래 — 우회전


@pytest.mark.check('B')
def test_leaving_radius_resets_hold_timer(h):
    h.set_pose(7.0, 0.0)
    h.tick(30)
    h.set_pose(10.0 - HOLD_RADIUS - 0.5, 0.0)
    h.tick(1)
    h.set_pose(7.0, 0.0)
    h.tick(30)                      # 합치면 6s 지만 연속 5s 가 아니다
    assert h.completions == []


@pytest.mark.check('B')
def test_hold_radius_boundary_is_inclusive(h):
    h.set_pose(10.0 - HOLD_RADIUS, 0.0)
    h.tick(_hold_ticks() + 1)
    assert h.completions == ['station']


@pytest.mark.check('B')
def test_completion_once_per_activation_and_reset_on_reactivation(h):
    h.set_pose(9.0, 0.0)
    h.tick(_hold_ticks() + 10)
    assert h.completions == ['station']
    h.activate('gate')
    h.activate('station')
    h.set_goal(10.0, 0.0)           # 재활성화 시 이전 목표는 지워진다
    h.tick(_hold_ticks() + 1)
    assert h.completions == ['station', 'station']


@pytest.mark.check('C')
@pytest.mark.parametrize('case', ['no_goal', 'wrong_frame', 'stale_odom', 'map_odom'])
def test_invalid_inputs_give_stop(monkeypatch, case):
    h = BehaviorHarness(StationKeeper, monkeypatch)
    try:
        h.activate()
        if case != 'no_goal':
            h.set_goal(10.0, 0.0, frame='map' if case == 'wrong_frame' else 'odom')
        h.set_pose(0.0, 0.0, frame='map' if case == 'map_odom' else 'odom')
        if case == 'stale_odom':
            h.node._on_odom(h.pose)
            h.clock.t += 1.0
            cmd = h.node.compute_cmd()
        else:
            cmd = h.tick()
        assert is_stop(cmd)
    finally:
        h.close()


@pytest.mark.check('C')
def test_obstacle_ahead_adds_repulsion(h):
    h.set_pose(0.0, 0.0)
    free = h.tick()
    res, size = 0.1, 100
    data = [0] * (size * size)
    ix, iy = int((2.0 + 5.0) / res), int((0.5 + 5.0) / res)
    data[iy * size + ix] = 100       # 좌측 전방 2m
    h.node._on_grid(SimpleNamespace(
        info=SimpleNamespace(resolution=res, width=size, height=size,
                             origin=SimpleNamespace(position=SimpleNamespace(x=-5.0, y=-5.0))),
        data=data))
    blocked = h.tick()
    assert blocked.angular.z < free.angular.z


@pytest.mark.check('D')
def test_inactive_node_publishes_nothing(h):
    h.activate('gate')
    h.set_pose(0.0, 0.0)
    h.node._on_odom(h.pose)
    h.node._tick()
    assert h.node.cmd_pub.msgs == []
    h.activate('station')
    h.set_goal(10.0, 0.0)
    h.node._on_odom(h.pose)
    h.node._tick()
    assert len(h.node.cmd_pub.msgs) == 1
    assert StationKeeper.CMD_TOPIC == '/cmd/station'
