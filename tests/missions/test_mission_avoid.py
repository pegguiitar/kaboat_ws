"""L2 Avoid — 가짜 라이다 → occupancy_grid → DRI → B-spline → 회피 FSM → obstacle_planner → 선체 모델.

수조(−3~16 m × ±6 m) 안 부표 3개를 피해 목표 (12, 0) m 2 m 이내에 들어오면 완료한다.
"""
import pytest

from kaboat_behaviors.obstacle_planner import ObstaclePlanner
from tests.missions.harness import (
    LidarGrid, MissionSim, Scenario, close, make_node, save_figure, seeds, tank_walls)
from tests.sim.boat_model import BoatState
from tests.sim.fake_sensors import Circle, World
from tests.sim.ros_adapters import FakeClock

GOAL = (12.0, 0.0)
BUOYS = [Circle(4.0, 0.2, 0.25), Circle(7.0, -0.6, 0.25), Circle(9.5, 0.8, 0.25)]
BOUNDS = (-3.0, -6.0, 16.0, 6.0)
WORLD = World(circles=BUOYS, segments=tank_walls(*BOUNDS), bounds=BOUNDS)
TIME_LIMIT = 40.0
MIN_CLEARANCE = 0.4      # 부표 표면까지 최소 거리 [m]


def _run(monkeypatch, scenario, cut_odom_at=None):
    clock = FakeClock()
    node = make_node(ObstaclePlanner, monkeypatch, clock)
    lidar = LidarGrid(WORLD)
    try:
        sim = MissionSim(node, BoatState(), scenario, WORLD, hooks=[lidar], clock=clock)
        sim.activate(GOAL)
        if cut_odom_at is not None:
            sim.run(until=lambda s: s.boat.state.x >= cut_odom_at)
            sim.cut_at = sim.t
            sim.odom_alive = False
            sim.run(max_time=sim.t + 5.0, until=lambda s: s.t >= s.cut_at + 5.0)
        else:
            sim.run(max_time=TIME_LIMIT + 10.0, after_complete=1.0)
        return sim
    finally:
        lidar.close()
        close(node)


def _check_mission(sim):
    rec = sim.rec
    done = rec.first_event_time('complete')
    assert done is not None, f'완료 신호 없음 (마지막 위치 {rec.samples[-1].x:.1f}, {rec.samples[-1].y:.1f})'
    assert done <= TIME_LIMIT, f'완료 {done:.1f}s > {TIME_LIMIT}s'
    clearance = min(min(((s.x - b.x) ** 2 + (s.y - b.y) ** 2) ** 0.5 - b.radius for b in BUOYS)
                    for s in rec.samples)
    assert clearance >= MIN_CLEARANCE, f'부표와 {clearance:.2f}m 까지 접근'
    assert rec.min_clearance(World(segments=WORLD.segments)) >= MIN_CLEARANCE, '수조 벽 접근'


@pytest.mark.check('S1')
def test_avoid_nominal(monkeypatch):
    sim = _run(monkeypatch, Scenario.nominal())
    save_figure('L2_avoid_S1', sim.rec, WORLD, GOAL)
    _check_mission(sim)


@pytest.mark.check('S2')
@pytest.mark.parametrize('seed', seeds())
def test_avoid_disturbed(monkeypatch, seed):
    sim = _run(monkeypatch, Scenario.disturbed(seed))
    save_figure(f'L2_avoid_S2_seed{seed}', sim.rec, WORLD, GOAL)
    _check_mission(sim)


@pytest.mark.check('S3')
def test_avoid_odom_loss_stops(monkeypatch):
    sim = _run(monkeypatch, Scenario.failure(), cut_odom_at=5.0)
    assert sim.rec.stopped_after(sim.cut_at + 0.6), 'odom 끊김 0.6초 뒤에도 추력 명령'
