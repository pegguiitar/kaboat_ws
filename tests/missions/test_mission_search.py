"""L2 Search — odom·부표맵 → search_circler → LOS 원 경로 → 선체 모델.

탐색 구역 중심 (6, 0) m 근처 초록 부표를 보고 반시계 원을 돌며, 5초 선회하면 완료한다.
"""
import math

import pytest
from kaboat_msgs.msg import Buoy, BuoyArray

from kaboat_behaviors.search_circler import ARRIVE_RADIUS, SearchCircler
from tests.missions.harness import MissionSim, Scenario, close, make_node, save_run, seeds
from tests.sim.boat_model import BoatState
from tests.sim.fake_sensors import Circle, World
from tests.sim.ros_adapters import FakeClock

GOAL = (6.0, 0.0)
MARKER = Circle(9.0, 1.0, 0.25)      # 초록 표식 부표
WORLD = World(circles=[MARKER], bounds=(-3.0, -8.0, 14.0, 8.0))
TIME_LIMIT = 40.0
HOLD = 2.0                            # search_circler 선회 유지 반경 [m]


def _buoy_map(sim):
    msg = BuoyArray()
    b = Buoy()
    b.color = 'green'
    b.position.x, b.position.y = MARKER.x, MARKER.y
    msg.buoys.append(b)
    sim.node._on_buoys(msg)


def _run(monkeypatch, scenario, cut_odom_in_circle=False):
    clock = FakeClock()
    node = make_node(SearchCircler, monkeypatch, clock)
    try:
        sim = MissionSim(node, BoatState(), scenario, WORLD, hooks=[_buoy_map], clock=clock)
        sim.activate(GOAL)
        if cut_odom_in_circle:
            sim.run(until=lambda s: s.node._circle_since is not None and s.t > 1.0)
            sim.cut_at = sim.t
            sim.odom_alive = False
            sim.run(max_time=sim.t + 10.0, until=lambda s: s.t >= s.cut_at + 10.0)
        else:
            sim.run(max_time=TIME_LIMIT + 20.0, after_complete=5.0)
        return sim
    finally:
        close(node)


def _check_mission(sim):
    rec = sim.rec
    done = rec.first_event_time('complete')
    assert done is not None, '완료 신호 없음'
    assert done <= TIME_LIMIT, f'완료 {done:.1f}s > {TIME_LIMIT}s'
    entered = next(s.t for s in rec.samples
                   if math.hypot(s.x - GOAL[0], s.y - GOAL[1]) <= ARRIVE_RADIUS)
    circling = [s for s in rec.samples if entered <= s.t <= done]
    worst = max(math.hypot(s.x - GOAL[0], s.y - GOAL[1]) for s in circling)
    assert worst <= HOLD, f'선회 중 탐색 중심에서 {worst:.2f}m 이탈'
    assert rec.min_clearance(WORLD) > 0.3, '표식 부표와 0.3m 이내 접근'
    assert rec.always_inside(WORLD)


@pytest.mark.check('S1')
def test_search_nominal(monkeypatch):
    sim = _run(monkeypatch, Scenario.nominal())
    save_run('L2_search_S1', sim.rec, WORLD, GOAL)
    _check_mission(sim)


@pytest.mark.check('S2')
@pytest.mark.parametrize('seed', seeds())
def test_search_disturbed(monkeypatch, seed):
    sim = _run(monkeypatch, Scenario.disturbed(seed))
    save_run(f'L2_search_S2_seed{seed}', sim.rec, WORLD, GOAL)
    _check_mission(sim)


@pytest.mark.check('S3')
def test_search_odom_loss_stops(monkeypatch):
    sim = _run(monkeypatch, Scenario.failure(), cut_odom_in_circle=True)
    assert sim.rec.stopped_after(sim.cut_at + 0.6), 'odom 끊김 0.6초 뒤에도 추력 명령'
