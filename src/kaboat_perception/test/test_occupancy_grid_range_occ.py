"""히트 log-odds 를 거리에 비례해 올린다 (range_l_occ, 2026-11-12 선수 V 자 물보라).

현장 영상(2026-11-12 16:50, 2:10~3:15) 분석: 물보라가 라이다 정면 0.4~1.6 m 에 찍혀 그 칸
(직전 값 ≈1, L_MIN 근처)을 스캔 60~80 % 로 맞혀 1.2~1.8 s 만에 95 → ESCAPE.
기본값 0.4 → 1.8 m (처음 0.1 → 3.0 m 는 출렁여 가끔 보이는 가까운 부표를 충돌까지 놓쳤다).
"""

import math

import pytest
import rclpy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

from kaboat_perception.occupancy_grid import (
    L_FREE, L_MAX, L_MIN, L_OCC, OccupancyGridNode, range_l_occ)

R_MIN = 0.4                         # TG-50 range_min
TH = math.log(0.945 / 0.055)        # 발행값(반올림) 95 이상 = log-odds ≥ 2.844
NEAR, FULL = 0.4, 1.8               # 노드 기본값 (l_occ_near, l_occ_full_range)


def break_even(l_occ):
    """칸이 쌓이기 시작하는 최소 가시 비율 (히트 l_occ, 안 보이면 L_FREE)."""
    return -L_FREE / (l_occ - L_FREE)


def seconds_to_95(l_occ, hit_ratio, start=L_MIN):
    """start 에서 히트 비율 hit_ratio 로 맞을 때 95 까지 [s] (10 Hz)."""
    drift = l_occ * hit_ratio + L_FREE * (1.0 - hit_ratio)
    return math.inf if drift <= 0 else (TH - start) / drift / 10.0


def test_ramp_from_min_range_to_full_range():
    near, full = 0.1, 3.0
    assert range_l_occ(0.4, R_MIN, near, full) == pytest.approx(0.1)
    assert range_l_occ(1.7, R_MIN, near, full) == pytest.approx(0.4)
    assert range_l_occ(3.0, R_MIN, near, full) == pytest.approx(L_OCC)
    assert range_l_occ(8.0, R_MIN, near, full) == pytest.approx(L_OCC)
    assert range_l_occ(0.2, R_MIN, near, full) == pytest.approx(0.1)   # 최소 거리 안쪽도 near
    # near = L_OCC 면 거리 무관 (예전 동작)
    assert range_l_occ([0.5, 1.0, 5.0], R_MIN, L_OCC, full).tolist() == [L_OCC] * 3


def test_spray_band_takes_longer_than_flat():
    # 물보라가 찍힌 0.8~1.6 m 칸: 히트 70 % 에서 95 까지 거리 무관(1.84 s)보다 늦게
    # (0.8/1.2/1.6 m → 2.9/2.4/2.0 s)
    flat = seconds_to_95(L_OCC, 0.7)
    for r, at_least in ((0.8, 2.9), (1.2, 2.3), (1.6, 1.95)):
        assert seconds_to_95(float(range_l_occ(r, R_MIN, NEAR, FULL)), 0.7) >= at_least > flat, r
    # 1.8 m 밖 장애물은 그대로 (처음 보는 칸 5 히트)
    assert math.ceil(TH / float(range_l_occ(2.0, R_MIN, NEAR, FULL))) == 5


def test_intermittent_near_buoy_still_accumulates():
    """출렁여 가끔 보이는 가까운 부표: 절반 미만(0.6 m 40 %, 1 m 36 %)만 보여도 칸이 쌓인다.
    처음 넣은 0.1 → 3.0 m 는 0.6 m 67 %·1 m 56 % 가 필요해, 70 % 가시 부표를 충돌까지 놓쳤다
    (감사 시뮬레이션: 2.5 m 안에서만 70 % 보이는 부표 → 0.45 m 까지 지도에 없음)."""
    for r in (0.6, 1.0, 1.4):
        assert break_even(float(range_l_occ(r, R_MIN, NEAR, FULL))) <= 0.41, r
    assert break_even(float(range_l_occ(1.0, R_MIN, 0.1, 3.0))) > 0.55     # 예전 값 (회귀 기록)
    # 70 % 가시 부표는 하한에서 3 s 안에 95 (1 m)
    assert seconds_to_95(float(range_l_occ(1.0, R_MIN, NEAR, FULL)), 0.7) < 3.0


@pytest.fixture
def node():
    rclpy.init(args=['--ros-args', '-p', 'enable_tilt_gate:=false'])
    n = OccupancyGridNode()
    odom = Odometry()
    odom.header.frame_id = 'odom'
    odom.pose.pose.orientation.w = 1.0              # 원점, 선수 = +x
    n._on_odom(odom)
    yield n
    n.destroy_node()
    rclpy.shutdown()


def _logodds_at(n, x, y):
    ax, ay = n.anchor
    return float(n.grid[math.floor(y / n.resolution) - ay, math.floor(x / n.resolution) - ax])


def _scan():
    s = LaserScan()
    s.angle_min, s.angle_increment = 0.0, 0.5       # 빔 0: 정면, 빔 1: 왼쪽 0.5 rad
    s.range_min, s.range_max = R_MIN, 40.0
    s.ranges = [1.1, 5.0]
    return s


def test_scan_hit_increment_depends_on_range(node):
    node._on_scan(_scan())
    # 라이다는 선수 쪽 0.4 m — 정면 1.1 m 히트 = (1.5, 0), 5 m 히트 = (0.4+5cos0.5, 5sin0.5)
    near = _logodds_at(node, 1.5, 0.0)
    far = _logodds_at(node, 0.4 + 5.0 * math.cos(0.5), 5.0 * math.sin(0.5))
    assert near == pytest.approx(float(range_l_occ(1.1, R_MIN, NEAR, FULL)))   # ≈ 0.55
    assert far == pytest.approx(L_OCC)
    assert L_MIN <= near < far <= L_MAX


def test_near_hits_do_not_erase_obstacle_seen_from_afar(node):
    # 멀리서 이미 포화된 장애물 칸은 가까이서 계속 맞으면(작은 증가량) 그대로 포화
    node._recenter()
    ax, ay = node.anchor
    iy, ix = math.floor(0.0 / node.resolution) - ay, math.floor(1.5 / node.resolution) - ax
    node.grid[iy, ix] = L_MAX
    for _ in range(20):
        node._on_scan(_scan())
    assert node.grid[iy, ix] == pytest.approx(L_MAX)


def test_flat_when_near_equals_l_occ():
    rclpy.init(args=['--ros-args', '-p', 'enable_tilt_gate:=false', '-p', 'l_occ_near:=0.7'])
    n = OccupancyGridNode()
    try:
        odom = Odometry()
        odom.pose.pose.orientation.w = 1.0
        n._on_odom(odom)
        n._on_scan(_scan())
        assert _logodds_at(n, 1.5, 0.0) == pytest.approx(L_OCC)
    finally:
        n.destroy_node()
        rclpy.shutdown()


def test_rejects_bad_near_value():
    rclpy.init(args=['--ros-args', '-p', 'l_occ_near:=0.9'])
    try:
        with pytest.raises(ValueError, match='l_occ_near'):
            OccupancyGridNode()
    finally:
        rclpy.shutdown()
