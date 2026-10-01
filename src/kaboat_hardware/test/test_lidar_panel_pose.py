import math

import numpy as np

from kaboat_hardware.lidar_panel_pose import fit_panel_line, select_panel_line
from kaboat_hardware.pose_velocity import normalize_angle


def _panel(x=5.0, y=2.5, yaw=0.0, span=0.60, count=25):
    center = np.array([x, y])
    tangent = np.array([-math.sin(yaw), math.cos(yaw)])
    return np.array([center + d * tangent
                     for d in np.linspace(-span / 2, span / 2, count)])


def _fit(points):
    return fit_panel_line(points, min_points=6, min_span=0.35,
                          max_span=0.80, max_line_rms=0.03)


def test_fits_rotated_panel_and_midpoint():
    result = _fit(_panel(x=4.0, y=3.0, yaw=math.radians(35)))
    assert result is not None
    np.testing.assert_allclose(result.center, [4.0, 3.0], atol=1e-12)
    assert abs(normalize_angle(result.yaw - math.radians(35))) < 1e-12
    assert math.isclose(result.span, 0.60)


def test_fits_dotted_panel_with_small_lidar_noise():
    points = _panel(x=3.0, y=1.5, yaw=math.radians(-25))
    normal = np.array([math.cos(math.radians(-25)),
                       math.sin(math.radians(-25))])
    points += (0.01 * np.sin(np.arange(len(points)) * 2.4))[:, None] * normal
    result = _fit(points)
    assert result is not None
    np.testing.assert_allclose(result.center, [3.0, 1.5], atol=0.02)
    assert abs(normalize_angle(result.yaw - math.radians(-25))) < 0.03


def test_rejects_short_or_curved_reflections():
    assert _fit(_panel(span=0.20)) is None
    curved = _panel()
    curved[:, 0] += 0.12 * (1.0 - ((curved[:, 1] - 2.5) / 0.3) ** 2)
    assert _fit(curved) is None


def test_maintains_previous_heading_branch_at_180_degrees():
    result = select_panel_line(
        clusters=[_panel()], expected_span=0.60, min_points=6,
        min_span=0.35, max_span=0.80, max_line_rms=0.03,
        previous_pose=(5.0, 2.5, math.pi),
        max_position_jump=0.75, max_yaw_jump=math.radians(60))
    assert result is not None
    assert abs(normalize_angle(result.yaw - math.pi)) < 1e-12


def test_prefers_expected_panel_length_over_other_line():
    result = select_panel_line(
        clusters=[_panel(x=2.0, y=2.0, span=0.38), _panel(x=5.0, y=2.5)],
        expected_span=0.60, min_points=6,
        min_span=0.35, max_span=0.80, max_line_rms=0.03)
    assert result is not None
    np.testing.assert_allclose(result.center, [5.0, 2.5], atol=1e-12)
