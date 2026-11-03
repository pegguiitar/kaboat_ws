"""L2 Gate — 가짜 라이다 → occupancy_grid → gate_corridor → gate_follower → 선체 모델.

수조(−3~14 m × ±6 m) 안 x = 3, 5, 7 m 에 좌우 ±1.5 m 기둥 3쌍, 목표 (9, 0) m.
3쌍 중앙을 차례로 통과하고 목표 2 m 이내에 들어오면 완료한다.
"""
import math

import pytest

from kaboat_behaviors.gate_follower import GateFollower
from tests.missions.harness import (
    LidarGrid, MissionSim, Scenario, close, make_node, save_figure, seeds, tank_walls)
from tests.sim.boat_model import BoatState
from tests.sim.fake_sensors import Circle, World
from tests.sim.ros_adapters import FakeClock

GOAL = (9.0, 0.0)
POSTS = [Circle(x, y, 0.15) for x in (3.0, 5.0, 7.0) for y in (-1.5, 1.5)]
BOUNDS = (-3.0, -6.0, 14.0, 6.0)
WORLD = World(circles=POSTS, segments=tank_walls(*BOUNDS), bounds=BOUNDS)
TIME_LIMIT = 40.0
MIN_CLEARANCE = 0.4      # 기둥 표면까지 최소 거리 [m]
PARAMS = ['gate.motion_enabled:=true']


def _run(monkeypatch, scenario, cut_scan_after_first_gate=False):
    clock = FakeClock()
    node = make_node(GateFollower, monkeypatch, clock, PARAMS)
    lidar = LidarGrid(WORLD, feed_scan_to_behavior=True)
    try:
        sim = MissionSim(node, BoatState(), scenario, WORLD, hooks=[lidar], clock=clock)
        sim.activate(GOAL)
        if cut_scan_after_first_gate:
            sim.run(until=lambda s: len(s.node._passed_midpoints) >= 1)
            sim.cut_at = sim.t
            lidar.scan_alive = False
            sim.run(max_time=sim.t + 5.0, until=lambda s: s.t >= s.cut_at + 5.0)
        else:
            sim.run(max_time=TIME_LIMIT + 10.0, after_complete=1.0)
        sim.passed = list(node._passed_midpoints)
        return sim
    finally:
        lidar.close()
        close(node)


def _check_mission(sim):
    rec = sim.rec
    done = rec.first_event_time('complete')
    assert done is not None, f'완료 신호 없음 (마지막 상태 {rec.samples[-1].status})'
    assert done <= TIME_LIMIT, f'완료 {done:.1f}s > {TIME_LIMIT}s'
    assert len(sim.passed) == 3
    for x_gate in (3.0, 5.0, 7.0):           # 각 게이트 선을 지날 때 중앙에서 벗어난 거리
        k = min(range(len(rec.samples)), key=lambda i: abs(rec.samples[i].x - x_gate))
        assert abs(rec.samples[k].y) < 1.5 - MIN_CLEARANCE, f'x={x_gate} 통과 y={rec.samples[k].y:.2f}'
    clearance = min(min(math.hypot(s.x - c.x, s.y - c.y) - c.radius for c in POSTS)
                    for s in rec.samples)
    assert clearance >= MIN_CLEARANCE, f'기둥과 {clearance:.2f}m 까지 접근'
    assert rec.always_inside(WORLD)


@pytest.mark.check('S1')
def test_gate_nominal(monkeypatch):
    sim = _run(monkeypatch, Scenario.nominal())
    save_figure('L2_gate_S1', sim.rec, WORLD, GOAL)
    _check_mission(sim)


@pytest.mark.check('S2')
@pytest.mark.parametrize('seed', seeds())
def test_gate_disturbed(monkeypatch, seed):
    sim = _run(monkeypatch, Scenario.disturbed(seed))
    save_figure(f'L2_gate_S2_seed{seed}', sim.rec, WORLD, GOAL)
    _check_mission(sim)


@pytest.mark.check('S3')
def test_gate_lidar_loss_stops(monkeypatch):
    sim = _run(monkeypatch, Scenario.failure(), cut_scan_after_first_gate=True)
    # gate.scan_timeout_sec 0.7s + 제어 주기 여유
    assert sim.rec.stopped_after(sim.cut_at + 0.9), '라이다 끊김 0.9초 뒤에도 추력 명령'
    assert sim.rec.samples[-1].status in ('SCAN_TIMEOUT', 'GRID_TIMEOUT_OR_FRAME')
    assert sim.rec.first_event_time('complete') is None
