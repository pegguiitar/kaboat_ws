import numpy as np

from kaboat_perception.dock_geometry import estimate_marker
from kaboat_perception.depth_utils import depth_to_meters


def test_mask_median_depth_and_centre_ray():
    depth = depth_to_meters(np.full((100, 120), 3000, np.uint16), '16UC1')
    mask = np.zeros_like(depth, dtype=bool)
    mask[25:75, 30:90] = True
    depth[26, 31] = 0.0
    depth[27, 32] = 7.0
    estimate = estimate_marker(mask, depth, 100, 100, 60, 50, min_pixels=100)
    assert estimate is not None
    assert abs(estimate.median_depth_m - 3.0) < 1e-6
    np.testing.assert_allclose(estimate.point_optical, [0.0, 0.0, 3.0], atol=0.02)
    assert estimate.valid_fraction > 0.99


def test_off_axis_marker_is_back_projected_with_intrinsics():
    depth = depth_to_meters(np.full((100, 120), 4000, np.uint16), '16UC1')
    mask = np.zeros_like(depth, dtype=bool)
    mask[40:60, 80:100] = True                     # 중심 (u=89.5→90, v=50)
    estimate = estimate_marker(mask, depth, 100, 100, 60, 50, min_pixels=100)
    np.testing.assert_allclose(estimate.point_optical, [1.2, 0.0, 4.0], atol=0.05)


def test_insufficient_depth_fails_closed():
    mask = np.ones((8, 8), dtype=bool)
    assert estimate_marker(mask, np.zeros((8, 8)), 100, 100, 4, 4) is None
