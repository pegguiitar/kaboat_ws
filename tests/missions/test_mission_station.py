"""L2 Station — odom → station_keeper → waypoint 제어 → 선체 모델.

목표 (10, 0) m 에서 5 m 이내를 5 초 유지하면 완료 신호를 낸다 (station_keeper 규정).
"""
import math

import pytest

from kaboat_behaviors.station_keeper import HOLD_RADIUS, StationKeeper
from tests.missions.harness import MissionSim, Scenario, close, make_node, save_figure, seeds
from tests.sim.boat_model import BoatState
from tests.sim.fake_sensors import World
from tests.sim.ros_adapters import FakeClock

GOAL = (10.0, 0.0)
WORLD = World(bounds=(-3.0, -8.0, 17.0, 8.0))
TIME_LIMIT = 40.0       # 성능 기준: 시작부터 완료까지 [s]


def _run(monkeypatch, scenario, cut_odom_inside=False):
    clock = FakeClock()
    node = make_node(StationKeeper, monkeypatch, clock)
    try:
        sim = MissionSim(node, BoatState(), scenario, WORLD, clock=clock)
        sim.activate(GOAL)
        if cut_odom_inside:
            sim.run(until=lambda s: math.hypot(s.boat.state.x - GOAL[0],
                                               s.boat.state.y - GOAL[1]) < HOLD_RADIUS - 1.0)
            sim.cut_at = sim.t
            sim.odom_alive = False
            sim.run(max_time=sim.t + 10.0, until=lambda s: s.t >= s.cut_at + 10.0)
        else:
            sim.run(max_time=TIME_LIMIT + 20.0, after_complete=10.0)
        return sim
    finally:
        close(node)


def _check_mission(sim):
    rec = sim.rec
    done = rec.first_event_time('complete')
    assert done is not None, '완료 신호 없음'
    assert done <= TIME_LIMIT, f'완료 {done:.1f}s > {TIME_LIMIT}s'
    after = [s for s in rec.samples if s.t >= done]
    worst = max(math.hypot(s.x - GOAL[0], s.y - GOAL[1]) for s in after)
    assert worst <= HOLD_RADIUS, f'완료 후 10초 동안 최대 {worst:.2f}m 이탈'
    assert rec.always_inside(WORLD)
    return done, worst


@pytest.mark.check('S1')
def test_station_nominal(monkeypatch):
    sim = _run(monkeypatch, Scenario.nominal())
    done, worst = _check_mission(sim)
    save_figure('L2_station_S1', sim.rec, WORLD, GOAL)


@pytest.mark.check('S2')
@pytest.mark.parametrize('seed', seeds())
def test_station_disturbed(monkeypatch, seed):
    sim = _run(monkeypatch, Scenario.disturbed(seed))
    _check_mission(sim)
    save_figure(f'L2_station_S2_seed{seed}', sim.rec, WORLD, GOAL)


@pytest.mark.check('S3')
def test_station_odom_loss_stops(monkeypatch):
    sim = _run(monkeypatch, Scenario.failure(), cut_odom_inside=True)
    assert sim.rec.stopped_after(sim.cut_at + 0.6), 'odom 끊김 0.6초 뒤에도 추력 명령'
    assert sim.rec.first_event_time('complete') is None or \
        sim.rec.first_event_time('complete') < sim.cut_at
