"""dock_geometry — 경계·파라미터 (기운 면, 거리 범위, 평면 오차, 최소 픽셀, 입구 거리)."""
import numpy as np
import pytest

from kaboat_perception.dock_geometry import estimate_marker, project_entrance_xy

F, CX, CY = 100.0, 60.0, 50.0


def _plane_depth(z0=3.0, slope=0.0, shape=(100, 120)):
    """광학 x 방향으로 기운 면: z = z0 + slope·X. 픽셀마다 정확한 깊이를 푼다."""
    h, w = shape
    u = (np.arange(w) - CX) / F
    z_row = z0 / (1.0 - slope * u)
    return np.tile(z_row.astype(np.float32), (h, 1))


def _mask(shape=(100, 120)):
    m = np.zeros(shape, dtype=bool)
    m[25:75, 30:90] = True
    return m


@pytest.mark.check('B')
def test_tilted_plane_gives_tilted_normal():
    est = estimate_marker(_mask(), _plane_depth(slope=0.5), F, F, CX, CY)
    assert est is not None
    n = est.outward_normal_optical
    expected = np.array([0.5, 0.0, -1.0]) / np.linalg.norm([0.5, 0.0, -1.0])
    np.testing.assert_allclose(n, expected, atol=0.02)


@pytest.mark.check('B')
def test_depth_range_limits():
    assert estimate_marker(_mask(), _plane_depth(z0=13.0), F, F, CX, CY) is None   # 12m 밖
    assert estimate_marker(_mask(), _plane_depth(z0=0.2), F, F, CX, CY) is None    # 0.25m 안
    assert estimate_marker(_mask(), _plane_depth(z0=13.0), F, F, CX, CY, max_depth=14.0) is not None


@pytest.mark.check('B')
def test_plane_rms_threshold():
    rng = np.random.default_rng(0)
    noisy = _plane_depth() + rng.normal(0.0, 0.03, (100, 120)).astype(np.float32)
    assert estimate_marker(_mask(), noisy, F, F, CX, CY, max_plane_rms=0.01) is None
    assert estimate_marker(_mask(), noisy, F, F, CX, CY, max_plane_rms=0.08) is not None


@pytest.mark.check('B')
def test_min_pixels_boundary():
    mask = np.zeros((100, 120), dtype=bool)
    mask[40:50, 40:50] = True                    # 100 픽셀
    depth = _plane_depth()
    assert estimate_marker(mask, depth, F, F, CX, CY, min_pixels=100) is not None
    assert estimate_marker(mask, depth, F, F, CX, CY, min_pixels=101) is None


@pytest.mark.check('C')
@pytest.mark.parametrize('case', ['shape_mismatch', 'bad_focal', 'nan_depth'])
def test_unusable_inputs_fail_closed(case):
    depth = _plane_depth()
    mask = _mask()
    fx = F
    if case == 'shape_mismatch':
        mask = mask[:, :100]
    elif case == 'bad_focal':
        fx = 0.0
    else:
        depth = np.full_like(depth, np.nan)
    assert estimate_marker(mask, depth, fx, F, CX, CY) is None


@pytest.mark.check('B')
def test_entrance_distance_zero_and_negative():
    np.testing.assert_allclose(project_entrance_xy([4, -1, 1.0], [-1, 0, 0], 0.0), [4, -1, 0])
    assert project_entrance_xy([4, -1, 1.0], [-1, 0, 0], -0.5) is None
