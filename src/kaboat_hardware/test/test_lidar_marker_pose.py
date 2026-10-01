import math

import numpy as np

from kaboat_hardware.lidar_marker_pose import (
    MarkerCandidate, estimate_base_pose, laser_points_to_pool,
    select_marker_pair,
)
from kaboat_hardware.pose_velocity import normalize_angle


THIN_BODY = np.array([0.0, 0.3])
THICK_BODY = np.array([0.0, -0.3])


def _candidate(x, y, diameter):
    return MarkerCandidate(np.array([x, y], dtype=float), diameter, 5)


def test_right_wall_laser_points_into_pool_coordinates():
    points = laser_points_to_pool(
        ranges=[5.0, math.hypot(5.0, 1.0)],
        angles=[0.0, -math.atan2(1.0, 5.0)],
        lidar_x=10.0, lidar_y=2.5, lidar_yaw=math.pi)
    np.testing.assert_allclose(points, [[5.0, 2.5], [5.0, 3.5]], atol=1e-12)


def test_estimate_base_pose_for_axis_aligned_markers():
    x, y, yaw = estimate_base_pose(
        [3.0, 2.3], [3.0, 1.7], THIN_BODY, THICK_BODY)
    assert math.isclose(x, 3.0)
    assert math.isclose(y, 2.0)
    assert math.isclose(yaw, 0.0)


def test_estimate_base_pose_for_rotated_markers():
    x, y, yaw = estimate_base_pose(
        [2.7, 2.0], [3.3, 2.0], THIN_BODY, THICK_BODY)
    assert math.isclose(x, 3.0)
    assert math.isclose(y, 2.0)
    assert math.isclose(yaw, math.pi / 2)


def test_selects_port_thin_and_starboard_thick_at_sixty_centimeters():
    pair = select_marker_pair(
        [_candidate(4.0, 2.3, 0.06), _candidate(4.0, 1.7, 0.18),
         _candidate(8.0, 4.0, 0.20)],
        (0.02, 0.10), (0.12, 0.30), THIN_BODY, THICK_BODY, 0.12)
    assert pair is not None
    assert math.isclose(pair.x, 4.0)
    assert math.isclose(pair.y, 2.0)
    assert math.isclose(pair.yaw, 0.0)
    assert math.isclose(pair.separation, 0.6)


def test_rejects_pair_outside_separation_tolerance():
    pair = select_marker_pair(
        [_candidate(4.0, 2.3, 0.06), _candidate(4.0, 1.5, 0.18)],
        (0.02, 0.10), (0.12, 0.30), THIN_BODY, THICK_BODY, 0.12)
    assert pair is None


def test_rejects_implausible_jump_from_previous_pose():
    pair = select_marker_pair(
        [_candidate(8.0, 4.3, 0.06), _candidate(8.0, 3.7, 0.18)],
        (0.02, 0.10), (0.12, 0.30), THIN_BODY, THICK_BODY, 0.12,
        previous_pose=(2.0, 2.0, 0.0), max_position_jump=0.75,
        max_yaw_jump=math.radians(60.0))
    assert pair is None


def test_yaw_gate_handles_pi_wraparound():
    yaw = math.radians(-179.0)
    left = 0.3 * np.array([-math.sin(yaw), math.cos(yaw)])
    thin = np.array([2.0, 2.0]) + left
    thick = np.array([2.0, 2.0]) - left
    pair = select_marker_pair(
        [MarkerCandidate(thin, 0.06, 5), MarkerCandidate(thick, 0.18, 8)],
        (0.02, 0.10), (0.12, 0.30), THIN_BODY, THICK_BODY, 0.12,
        previous_pose=(2.0, 2.0, math.radians(179.0)),
        max_position_jump=0.75, max_yaw_jump=math.radians(5.0))
    assert pair is not None
    assert abs(normalize_angle(pair.yaw - yaw)) < 1e-9
