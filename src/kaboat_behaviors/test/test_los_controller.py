"""Open B-spline and closed-circle LOS share one follower and speed law."""

import math

import numpy as np
import pytest

from kaboat_behaviors.bspline_planner import curvature_ahead
from kaboat_behaviors.los_controller import (
    LosFollower, bspline_path, circle_path, sampled_path)


def test_bspline_path_has_analytic_curvature_and_reaches_end():
    path = bspline_path([(0.0, 0.0), (2.0, 0.0),
                         (4.0, 1.0), (6.0, 1.0)], spacing=0.1)
    assert not path.closed
    assert len(path.samples) == len(path.curvatures)
    assert np.all(np.isfinite(path.curvatures))
    follower = LosFollower()
    first = follower.step(path, path.samples[0], 0.0, 0.0)
    assert first.phase == 'FOLLOW' and first.linear > 0
    for point in path.samples[::10]:
        follower.step(path, point, 0.0, 0.0)
    end = follower.step(path, path.samples[-1], 0.0, 0.0)
    assert end.reached and end.linear == 0 and end.angular == 0
    assert follower.step(path, path.samples[0], 0.0, 0.0).reached


def test_curvature_inverse_speed_reduces_thrust():
    samples = np.column_stack((np.arange(0.0, 10.1, 0.1),
                               np.zeros(101)))
    straight = sampled_path(samples, spacing=0.1,
                            curvatures=np.zeros(101))
    curved = sampled_path(samples, spacing=0.1,
                          curvatures=np.ones(101))
    full = LosFollower().step(straight, (0.0, 0.0), 0.0, 0.0)
    slow = LosFollower().step(curved, (0.0, 0.0), 0.0, 0.0)
    assert slow.curvature == pytest.approx(1.0)
    assert 0 < slow.linear < full.linear


def test_circle_lookahead_and_curvature_wrap_seam():
    path = circle_path(5.0, 2.5, 1.2, spacing=0.1)
    assert path.closed and path.target_laps == 1.0
    assert np.max(np.abs(path.curvatures - 1 / 1.2)) < 1e-12
    path.curvatures[0] = 2.0
    path.progress_idx = len(path.samples) - 2
    assert curvature_ahead(path, 0.5) == pytest.approx(2.0)


def test_circle_counts_one_lap_and_stops():
    path = circle_path(0.0, 0.0, 1.2, spacing=0.2)
    follower = LosFollower()
    n = len(path.samples)
    for i in range(n):
        theta = 2 * math.pi * i / n
        step = follower.step(path, path.samples[i], theta + math.pi / 2, 0.0)
        assert not step.reached
    end = follower.step(path, path.samples[0], math.pi / 2, 0.0)
    assert end.reached and end.progress_idx >= n
    assert (end.linear, end.angular) == (0.0, 0.0)
    assert follower.step(path, path.samples[1], 0.0, 0.0).reached


def test_circle_can_run_continuously_and_reverse_direction():
    path = circle_path(0.0, 0.0, 1.2, direction='cw',
                       spacing=0.2, target_laps=None)
    assert np.all(path.curvatures < 0)
    follower = LosFollower()
    n = len(path.samples)
    for i in range(n + 2):
        point = path.samples[i % n]
        step = follower.step(path, point, -math.pi / 2, 0.0)
        assert not step.reached
    assert step.progress_idx >= n


def test_circle_supports_fractional_laps():
    path = circle_path(0.0, 0.0, 1.2, spacing=0.2, target_laps=0.5)
    follower = LosFollower()
    n = len(path.samples)
    for i in range(n // 2):
        assert not follower.step(path, path.samples[i], 0.0, 0.0).reached
    end = follower.step(path, path.samples[n // 2], 0.0, 0.0)
    assert end.reached


def test_thrust_ratio_scales_los_and_rejects_invalid_values():
    path = circle_path(0.0, 0.0, 1.2, spacing=0.2)
    full = LosFollower().step(path, path.samples[0], 0.0, 0.0, 1.0)
    half = LosFollower().step(path, path.samples[0], 0.0, 0.0, 0.5)
    assert half.linear == pytest.approx(full.linear * 0.5)
    assert half.angular == pytest.approx(full.angular * 0.5)
    zero = LosFollower().step(path, path.samples[0], 0.0, 0.0, 0.0)
    assert (zero.linear, zero.angular) == (0.0, 0.0)
    for ratio in (-0.1, 1.1, float('nan')):
        with pytest.raises(ValueError):
            LosFollower().step(path, path.samples[0], 0.0, 0.0, ratio)


def test_invalid_path_rejected():
    with pytest.raises(ValueError):
        sampled_path([(0.0, 0.0), (1.0, 0.0)], spacing=0.0)
    with pytest.raises(ValueError):
        circle_path(0.0, 0.0, 0.0)
