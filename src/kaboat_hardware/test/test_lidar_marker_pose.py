import math

import numpy as np

from kaboat_hardware.lidar_marker_pose import (
    MarkerCandidate, estimate_base_pose, select_marker_pair,
)
from kaboat_hardware.pose_velocity import normalize_angle


BOW_BODY = np.array([0.5, 0.0])
STERN_BODY = np.array([-0.5, 0.0])


def _candidate(x, y, diameter):
    return MarkerCandidate(np.array([x, y], dtype=float), diameter, 5)


def test_estimate_base_pose_for_axis_aligned_markers():
    x, y, yaw = estimate_base_pose(
        [3.5, 2.0], [2.5, 2.0], BOW_BODY, STERN_BODY)
    assert math.isclose(x, 3.0)
    assert math.isclose(y, 2.0)
    assert math.isclose(yaw, 0.0)


def test_estimate_base_pose_for_rotated_markers():
    x, y, yaw = estimate_base_pose(
        [3.0, 2.5], [3.0, 1.5], BOW_BODY, STERN_BODY)
    assert math.isclose(x, 3.0)
    assert math.isclose(y, 2.0)
    assert math.isclose(yaw, math.pi / 2)


def test_selects_thin_bow_and_thick_stern_at_one_meter():
    pair = select_marker_pair(
        [_candidate(4.5, 2.0, 0.06), _candidate(3.5, 2.0, 0.18),
         _candidate(8.0, 4.0, 0.20)],
        (0.02, 0.10), (0.12, 0.30), BOW_BODY, STERN_BODY, 0.20)
    assert pair is not None
    assert math.isclose(pair.x, 4.0)
    assert math.isclose(pair.y, 2.0)
    assert math.isclose(pair.yaw, 0.0)
    assert math.isclose(pair.separation, 1.0)


def test_rejects_pair_outside_separation_tolerance():
    pair = select_marker_pair(
        [_candidate(4.5, 2.0, 0.06), _candidate(3.0, 2.0, 0.18)],
        (0.02, 0.10), (0.12, 0.30), BOW_BODY, STERN_BODY, 0.20)
    assert pair is None


def test_rejects_implausible_jump_from_previous_pose():
    pair = select_marker_pair(
        [_candidate(8.5, 4.0, 0.06), _candidate(7.5, 4.0, 0.18)],
        (0.02, 0.10), (0.12, 0.30), BOW_BODY, STERN_BODY, 0.20,
        previous_pose=(2.0, 2.0, 0.0), max_position_jump=0.75,
        max_yaw_jump=math.radians(60.0))
    assert pair is None


def test_yaw_gate_handles_pi_wraparound():
    yaw = math.radians(-179.0)
    bow = np.array([2.0, 2.0]) + 0.5 * np.array([math.cos(yaw), math.sin(yaw)])
    stern = np.array([2.0, 2.0]) - 0.5 * np.array([math.cos(yaw), math.sin(yaw)])
    pair = select_marker_pair(
        [MarkerCandidate(bow, 0.06, 5), MarkerCandidate(stern, 0.18, 8)],
        (0.02, 0.10), (0.12, 0.30), BOW_BODY, STERN_BODY, 0.20,
        previous_pose=(2.0, 2.0, math.radians(179.0)),
        max_position_jump=0.75, max_yaw_jump=math.radians(5.0))
    assert pair is not None
    assert abs(normalize_angle(pair.yaw - yaw)) < 1e-9

