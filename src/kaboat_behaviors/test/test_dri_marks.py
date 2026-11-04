import math
from types import SimpleNamespace as NS

import numpy as np

from kaboat_behaviors.dri import DriGrid, DriParams, build_dri
from kaboat_behaviors.dri_marks import high_risk_regions, x_segments

RES = 0.2
THRESHOLD = 0.85


def grid_with(cells, n=100, origin=-10.0):
    data = np.full((n, n), -1, np.int16)
    for x, y in cells:
        data[int((y - origin) / RES), int((x - origin) / RES)] = 97
    return NS(info=NS(resolution=RES, width=n, height=n,
                      origin=NS(position=NS(x=origin, y=origin))),
              data=data.ravel().tolist())


def dri_for(cells, boat=(0.0, 0.0), yaw=0.0):
    return build_dri(grid_with(cells), boat, yaw, DriParams())


def test_single_obstacle_gets_one_mark_at_its_centre():
    dri = dri_for([(3.0, 0.0)])
    regions = high_risk_regions(dri, THRESHOLD)
    assert len(regions) == 1
    assert abs(regions[0].x - 3.1) < 0.25 and abs(regions[0].y - 0.1) < 0.25
    assert regions[0].peak > THRESHOLD and regions[0].cells >= 1


def test_two_separate_obstacles_get_two_marks_and_a_cluster_gets_one():
    far_apart = high_risk_regions(dri_for([(3.0, -3.0), (3.0, 3.0)]), THRESHOLD)
    assert len(far_apart) == 2
    assert sorted(round(r.y) for r in far_apart) == [-3, 3]
    wall = [(3.0, y) for y in np.arange(-0.4, 0.5, 0.2)]      # 붙어 있는 벽
    one = high_risk_regions(dri_for(wall), THRESHOLD)
    assert len(one) == 1 and one[0].cells > 3
    assert abs(one[0].y - 0.1) < 0.25                          # 구역 가운데


def test_nothing_above_threshold_gives_no_marks():
    dri = dri_for([(3.0, 0.0)])
    assert high_risk_regions(dri, float(dri.data.max()) + 0.01) == []
    assert high_risk_regions(dri_for([]), THRESHOLD) == []


def test_threshold_is_strictly_greater_than():
    dri = dri_for([(3.0, 0.0)])
    peak = float(dri.data.max())
    assert high_risk_regions(dri, peak) == []                  # 같으면 넘은 게 아님
    assert len(high_risk_regions(dri, peak - 1e-6)) == 1


def test_higher_threshold_shrinks_the_region_but_keeps_its_centre():
    dri = dri_for([(3.0, 0.0)])
    low = high_risk_regions(dri, 0.3)[0]
    high = high_risk_regions(dri, 1.5)[0]
    assert low.cells > high.cells
    assert math.hypot(low.x - high.x, low.y - high.y) < 0.3


def test_world_coordinates_follow_anchor_and_resolution():
    data = np.zeros((10, 10), np.float32)
    data[2:5, 3:6] = 2.0                         # 행 2~4, 열 3~5 → 중심 (행 3, 열 4)
    dri = DriGrid(data=data, anchor=(-20, 30), resolution=0.5)
    (region,) = high_risk_regions(dri, 1.0)
    assert region.cells == 9 and region.peak == 2.0
    assert math.isclose(region.x, (-20 + 4 + 0.5) * 0.5)
    assert math.isclose(region.y, (30 + 3 + 0.5) * 0.5)


def test_diagonal_cells_are_one_region_and_min_cells_filters_specks():
    data = np.zeros((6, 6), np.float32)
    data[1, 1] = data[2, 2] = data[3, 3] = 2.0      # 대각선 = 8-연결
    data[5, 0] = 2.0                                # 외톨이 한 칸
    dri = DriGrid(data=data, anchor=(0, 0), resolution=1.0)
    assert len(high_risk_regions(dri, 1.0)) == 2
    (big,) = high_risk_regions(dri, 1.0, min_cells=2)
    assert big.cells == 3


def test_nan_and_limit():
    data = np.zeros((8, 8), np.float32)
    data[1, 1] = float('nan')
    for i, row in enumerate((0, 2, 4, 6)):
        data[row, 6] = 1.0 + i                       # 서로 떨어진 네 구역, peak 1..4
    dri = DriGrid(data=data, anchor=(0, 0), resolution=1.0)
    regions = high_risk_regions(dri, 0.5, limit=2)
    assert [r.peak for r in regions] == [4.0, 3.0]   # 위험 큰 순서로 자름
    assert all(math.isfinite(r.x) for r in regions)


def test_x_segments_make_two_crossing_diagonals_centred_on_the_region():
    (region,) = high_risk_regions(dri_for([(3.0, 0.0)]), THRESHOLD)
    (a, b), (c, d) = x_segments([region], 0.4)
    assert math.isclose((a[0] + b[0]) / 2, region.x) and math.isclose((a[1] + b[1]) / 2, region.y)
    assert math.isclose((c[0] + d[0]) / 2, region.x) and math.isclose((c[1] + d[1]) / 2, region.y)
    assert math.isclose(math.hypot(b[0] - a[0], b[1] - a[1]), 0.4 * math.sqrt(2))
    dot = (b[0] - a[0]) * (d[0] - c[0]) + (b[1] - a[1]) * (d[1] - c[1])
    assert abs(dot) < 1e-9                                      # 두 대각선은 직교
    assert x_segments([], 0.4) == []
