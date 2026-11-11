import pytest
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


def test_mask_border_background_is_removed():
    mask = np.zeros((40, 40), dtype=bool)
    mask[5:35, 5:35] = True
    depth = np.full((40, 40), 9.0)
    depth[6:34, 6:34] = 3.0
    quality = {}
    result = estimate_marker(mask, depth, 100, 100, 20, 20, diagnostics=quality)
    assert result is not None
    assert result.median_depth_m == 3.0
    assert quality['reason'] == 'ok'


def test_depth_holes_do_not_move_the_centre_ray():
    mask = np.zeros((40, 40), dtype=bool)
    mask[5:35, 5:35] = True
    depth = np.full((40, 40), 3.0)
    full = estimate_marker(mask, depth, 100, 100, 20, 20)
    depth[:, :20] = 0.0
    half = estimate_marker(mask, depth, 100, 100, 20, 20)
    np.testing.assert_allclose(full.point_optical, half.point_optical)


def test_broad_depth_distribution_is_rejected_even_with_all_pixels_valid():
    mask = np.ones((40, 40), dtype=bool)
    depth = np.ones((40, 40)) * 3
    depth[:, 20:] = 5
    quality = {}
    assert estimate_marker(mask, depth, 100, 100, 20, 20, diagnostics=quality) is None
    assert quality['valid_fraction'] == 1.0
    assert quality['reason'] == 'depth_spread_too_large'


def test_large_outlier_fraction_is_rejected():
    mask = np.ones((40, 40), dtype=bool)
    depth = np.ones((40, 40)) * 3
    depth[:, 25:] = 7
    quality = {}
    assert estimate_marker(mask, depth, 100, 100, 20, 20, diagnostics=quality) is None
    assert quality['reason'] == 'too_many_depth_outliers'


def test_temporal_filter_reduces_noise_and_rejects_isolated_jumps():
    from kaboat_perception.dock_geometry import StationaryMarkerFilter
    rng = np.random.default_rng(4)
    observations = rng.normal([3., 2.], 0.08, size=(200, 2))
    observations[80] += 2
    f = StationaryMarkerFilter()
    output = []
    for i, xy in enumerate(observations):
        point, state = f.update(xy, i * .1)
        if i == 80:
            assert point is None and state == 'innovation_rejected'
        if point is not None:
            output.append(point)
    assert np.std(np.array(output)[20:], axis=0).max() < .03
    np.testing.assert_allclose(np.mean(output[20:], axis=0), [3., 2.], atol=.025)
    for i in range(30):
        value, _ = f.update([3., 2.], 20. + i * .1)
    np.testing.assert_allclose(value, [3., 2.], atol=.002)


def test_filter_reacquires_consistent_new_marker_and_resets_after_gap():
    from kaboat_perception.dock_geometry import StationaryMarkerFilter
    f = StationaryMarkerFilter(warmup_samples=1)
    f.update([1, 1], 1.)
    for i in range(4):
        assert f.update([3, 3], 1.1 + i*.1)[0] is None
    value, state = f.update([3, 3], 1.5)
    assert state == 'reacquired'
    np.testing.assert_allclose(value, [3, 3])
    assert f.update([3, 3], 1.5)[1] == 'non_increasing_stamp'
    value, state = f.update([7, 7], 4.)
    assert state == 'reinitialized'
    assert f.epoch == 3
    np.testing.assert_allclose(value, [7, 7])


def test_discrete_depth_bins_with_small_spread_are_not_false_outliers():
    mask = np.ones((40, 40), dtype=bool)
    depth = np.full((40, 40), 8.11)
    depth[:, 25:] = 8.16
    result = estimate_marker(mask, depth, 100, 100, 20, 20)
    assert result is not None
    assert result.inlier_fraction == 1.0
    assert 8.11 < result.median_depth_m < 8.16


def test_depth_independent_sight_ray_reprojects_to_mask_center():
    import cv2
    from kaboat_perception.dock_geometry import mask_sight_ray
    mask=np.zeros((480,640),dtype=bool);mask[220:240,410:425]=True
    k=np.array([[382.,0,317.],[0,382.,245.],[0,0,1.]])
    d=np.array([-.055,.066,-.00037,.00016,-.021])
    ray,uv=mask_sight_ray(mask,382,382,317,245,d)
    pixel,_=cv2.projectPoints(ray.reshape(1,3),np.zeros(3),np.zeros(3),k,d)
    assert pixel.reshape(2)==pytest.approx(uv,abs=1e-4)
    assert ray[2]==1. and ray[0]>0  # right of optical centre


def test_empty_mask_cannot_make_a_sight_ray():
    from kaboat_perception.dock_geometry import mask_sight_ray
    with pytest.raises(ValueError):mask_sight_ray(np.zeros((10,10)),100,100,5,5)
