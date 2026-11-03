"""시험용 선체 모델·가짜 센서 자체 검증.

L2/L3 판정이 이 도구들에 기대므로, 손으로 계산 가능한 값과 맞는지 먼저 확인한다.
"""
import math

import numpy as np
import pytest

from tests.sim.boat_model import (
    MAX_THRUST, BoatModel, BoatState, HullParams, OdomNoise, thrusts_from_command)
from tests.sim.fake_sensors import (
    Circle, ScanConfig, Segment, World, occupancy_grid, scan_ranges)
from tests.sim.recorder import Recorder


def _run(model, linear, angular, seconds):
    for _ in range(int(round(seconds / 0.1))):
        model.step(linear, angular, 0.1)
    return model.state


# ── 선체 모델 ──────────────────────────────────────────

def test_thrust_mapping_matches_twist2thrust_and_saturates():
    assert thrusts_from_command(0.1, 0.0) == pytest.approx((0.1 * MAX_THRUST, 0.1 * MAX_THRUST))
    left, right = thrusts_from_command(0.0, 0.05)
    assert left == pytest.approx(-0.05 * MAX_THRUST) and right == pytest.approx(0.05 * MAX_THRUST)
    assert thrusts_from_command(0.9, 0.9) == pytest.approx((0.0, MAX_THRUST))


def test_straight_thrust_reaches_analytic_terminal_speed():
    p = HullParams()
    linear = 12.0 / MAX_THRUST
    force = 2 * linear * MAX_THRUST
    expected = (-p.x_u + math.sqrt(p.x_u ** 2 + 4 * p.x_uu * force)) / (2 * p.x_uu)
    s = _run(BoatModel(), linear, 0.0, 30.0)
    assert s.u == pytest.approx(expected, rel=1e-3)
    assert s.y == pytest.approx(0.0, abs=1e-9) and s.yaw == pytest.approx(0.0, abs=1e-9)


def test_positive_angular_turns_left_and_reaches_steady_rate():
    p = HullParams()
    angular = 3.0 / MAX_THRUST
    moment = 2 * angular * MAX_THRUST * p.half_beam
    expected = (-p.n_r + math.sqrt(p.n_r ** 2 + 4 * p.n_rr * moment)) / (2 * p.n_rr)
    model = BoatModel()
    s = _run(model, 0.0, angular, 20.0)
    assert s.r == pytest.approx(expected, rel=1e-3)
    assert s.r > 0.0   # 좌회전 = yaw 증가 (REP-103)


def test_zero_command_drifts_with_current():
    model = BoatModel(current=(0.1, -0.05))
    s = _run(model, 0.0, 0.0, 60.0)
    vx, vy = model.world_velocity
    assert (vx, vy) == pytest.approx((0.1, -0.05), abs=1e-3)
    assert s.x > 0.0 and s.y < 0.0


def test_odom_noise_is_reproducible_with_seed():
    state = BoatState(x=1.0, y=2.0, yaw=0.3)
    a = [OdomNoise(0.05, 0.01, seed=3).measure(state) for _ in range(1)]
    b = [OdomNoise(0.05, 0.01, seed=3).measure(state) for _ in range(1)]
    assert vars(a[0]) == vars(b[0])
    assert vars(OdomNoise().measure(state)) == vars(state)


# ── 가짜 센서 ──────────────────────────────────────────

def test_scan_hits_circle_and_segment_at_expected_range():
    cfg = ScanConfig()
    world = World(circles=[Circle(5.0, 0.0, 0.2)], segments=[Segment(-3.0, -5.0, -3.0, 5.0)])
    ranges = scan_ranges(0.0, 0.0, 0.0, world, cfg)
    front = int(round((0.0 - cfg.angle_min) / cfg.angle_increment))
    back = 0   # angle_min = -π
    left = int(round((math.pi / 2 - cfg.angle_min) / cfg.angle_increment))
    assert ranges[front] == pytest.approx(4.8)
    assert ranges[back] == pytest.approx(3.0)
    assert math.isinf(ranges[left])


def test_scan_rotates_with_boat_yaw():
    cfg = ScanConfig()
    world = World(circles=[Circle(0.0, 5.0, 0.2)])
    ranges = scan_ranges(0.0, 0.0, math.pi / 2, world, cfg)
    front = int(round((0.0 - cfg.angle_min) / cfg.angle_increment))
    assert ranges[front] == pytest.approx(4.8)


def test_grid_marks_obstacle_cells_only():
    world = World(circles=[Circle(2.0, 0.0, 0.3)])
    grid = occupancy_grid(world, origin=(-5.0, -5.0), resolution=0.2, size=50)
    def at(x, y):
        ix = int((x + 5.0) / 0.2)
        iy = int((y + 5.0) / 0.2)
        return grid.data[iy * 50 + ix]
    assert at(2.0, 0.0) == 100
    assert at(0.0, 0.0) == 0
    assert at(2.0, 1.0) == 0


def test_world_clearance_and_bounds():
    world = World(circles=[Circle(3.0, 0.0, 0.5)], bounds=(0.0, -2.0, 10.0, 2.0))
    assert world.min_clearance(0.0, 0.0) == pytest.approx(2.5)
    assert world.inside_bounds(5.0, 0.0)
    assert not world.inside_bounds(5.0, 3.0)


def test_recorder_metrics():
    rec = Recorder()
    for i, cmd in enumerate([0.1, 0.1, 0.0, 0.0]):
        rec.record(i * 0.1, BoatState(x=i * 1.0), cmd, 0.0)
    rec.event(0.2, 'complete')
    assert rec.first_event_time('complete') == pytest.approx(0.2)
    assert rec.stopped_after(0.2)
    assert not rec.stopped_after(0.0)
    assert rec.final_distance_to(3.0, 0.0) == pytest.approx(0.0)


@pytest.mark.check('C')
def test_out_of_range_returns_are_reported_as_no_return():
    cfg = ScanConfig(range_max=10.0)
    world = World(circles=[Circle(15.0, 0.0, 0.2)])
    ranges = scan_ranges(0.0, 0.0, 0.0, world, cfg)
    assert np.all(np.isinf(ranges))          # 최대 거리 밖 반사는 무반사(inf)로 처리


@pytest.mark.check('C')
def test_stop_judgement_fails_closed_without_samples():
    rec = Recorder()
    rec.record(0.0, BoatState(), 0.0, 0.0)
    assert not rec.stopped_after(1.0)        # 판정 구간에 기록이 없으면 '정지함'으로 보지 않는다
    assert rec.first_event_time('complete') is None


# ── ROS 메시지 변환 (rclpy 가 있는 Docker 에서만) ─────────────

def test_ros_adapters_roundtrip_pose_and_clock(monkeypatch):
    pytest.importorskip('rclpy')
    import time
    from tests.sim.ros_adapters import FakeClock, odom_msg, scan_msg
    msg = odom_msg(BoatState(x=1.0, y=-2.0, yaw=0.5, u=0.3, r=0.1))
    q = msg.pose.pose.orientation
    yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y ** 2 + q.z ** 2))
    assert yaw == pytest.approx(0.5)
    assert msg.header.frame_id == 'odom'
    assert len(scan_msg([1.0] * 720).ranges) == 720

    clock = FakeClock(start=10.0)
    clock.install(monkeypatch)
    clock.advance(0.5)
    assert time.monotonic() == pytest.approx(10.5)
