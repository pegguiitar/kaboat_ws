"""depth_utils — 경계·파라미터 (encoding 대소문자, 영상 가장자리 창, median, 창 크기)."""
import numpy as np
import pytest

from kaboat_perception.depth_utils import depth_to_meters, distance_at


@pytest.mark.check('B')
def test_encoding_name_is_case_insensitive():
    out = depth_to_meters(np.full((2, 2), 1500, np.uint16), '16uc1')
    assert out.dtype == np.float32
    assert out[0, 0] == pytest.approx(1.5)


@pytest.mark.check('B')
def test_window_is_clipped_at_image_corner():
    depth = np.full((10, 10), 9.0, np.float32)
    depth[:2, :2] = 2.0                           # 모서리 2×2 만 2m
    # (0,0) 주변 7×7 창이 영상 밖으로 잘려 4×4 만 남는다: 2m 4칸 + 9m 12칸
    assert distance_at(depth, 0, 0) == pytest.approx(9.0)
    assert distance_at(depth, 0, 0, win=1) == pytest.approx(2.0)


@pytest.mark.check('B')
def test_median_ignores_single_outlier():
    depth = np.full((20, 20), 4.0, np.float32)
    depth[10, 10] = 50.0                          # 반사 튐
    assert distance_at(depth, 10, 10) == pytest.approx(4.0)


@pytest.mark.check('B')
def test_zero_window_reads_exact_pixel():
    depth = np.arange(25, dtype=np.float32).reshape(5, 5) + 1.0
    assert distance_at(depth, 3, 2, win=0) == pytest.approx(depth[2, 3])
