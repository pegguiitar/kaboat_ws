"""Depth encoding and metric sampling tests (ROS-independent)."""

import numpy as np
import pytest

from kaboat_perception.depth_utils import depth_to_meters, distance_at


def test_realsense_16uc1_millimetres_to_metres():
    raw = np.array([[0, 1000, 5000]], dtype=np.uint16)
    out = depth_to_meters(raw, '16UC1')
    assert out.dtype == np.float32
    assert np.allclose(out, [[0.0, 1.0, 5.0]])


def test_gazebo_32fc1_is_already_metres():
    raw = np.array([[0.0, 1.25, 6.0]], dtype=np.float32)
    out = depth_to_meters(raw, '32FC1')
    assert np.allclose(out, [[0.0, 1.25, 6.0]])


def test_unknown_encoding_is_rejected():
    with pytest.raises(ValueError, match='unsupported depth encoding'):
        depth_to_meters(np.zeros((2, 2), dtype=np.uint8), 'mono8')


def test_distance_ignores_zero_and_non_finite_values():
    depth = np.array([
        [0.0, np.nan, 4.0],
        [np.inf, 5.0, 6.0],
        [0.0, 7.0, 8.0],
    ], dtype=np.float32)
    assert distance_at(depth, 1, 1, win=1) == pytest.approx(6.0)


def test_distance_outside_image_is_invalid():
    assert distance_at(np.ones((2, 2), dtype=np.float32), 3, 0) == -1.0
