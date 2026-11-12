"""히트 log-odds 를 거리에 비례해 올린다 (range_l_occ, 2026-11-12 선수 V 자 물보라).

현장 영상(2026-11-12 16:50, 2:10~3:15) 분석: 물보라가 라이다 정면 0.4~1.6 m 에 찍혀 그 칸
(직전 값 ≈1, L_MIN 근처)을 스캔 60~80 % 로 맞혀 1.2~1.8 s 만에 95 → ESCAPE.
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


def test_spray_band_now_outlasts_observed_spray():
    # 물보라가 찍힌 0.8~1.6 m 칸: 관측된 히트 비율 80 % 에서도 95 까지 2.5 s 넘게 (관측 1.2~1.8 s)
    for r in (0.8, 1.2, 1.6):
        l_occ = float(range_l_occ(r, R_MIN, 0.1, 3.0))
        assert seconds_to_95(l_occ, 0.8) > 2.5, r
        assert seconds_to_95(L_OCC, 0.8) < 1.8          # 거리 무관이면 관측대로 걸림
    # 3 m 밖 장애물은 그대로 (처음 보는 칸 5 히트)
    assert math.ceil(TH / float(range_l_occ(3.5, R_MIN, 0.1, 3.0))) == 5


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
    assert near == pytest.approx(float(range_l_occ(1.1, R_MIN, 0.1, 3.0)))   # ≈ 0.26
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
