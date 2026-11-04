import math
from types import SimpleNamespace as NS

import numpy as np

from kaboat_behaviors.dri import DriGrid, DriParams, build_dri
from kaboat_behaviors.dri_marks import high_risk_cells, x_segments

RES = 0.2
THRESHOLD = 0.85


def grid_with(cells, n=75, origin=-7.5):
    data = np.full((n, n), -1, np.int16)
    for x, y in cells:
        data[int((y - origin) / RES), int((x - origin) / RES)] = 97
    return NS(info=NS(resolution=RES, width=n, height=n,
                      origin=NS(position=NS(x=origin, y=origin))),
              data=data.ravel().tolist())


def dri_for(cells, boat=(0.0, -3.0), yaw=math.pi / 2):
    return build_dri(grid_with(cells), boat, yaw, DriParams())


def test_every_cell_over_threshold_gets_exactly_one_mark():
    dri = dri_for([(0.0, 0.0)])
    cells = high_risk_cells(dri, THRESHOLD)
    assert len(cells) == int((dri.data > THRESHOLD).sum()) > 1
    assert len({(round(c.x, 3), round(c.y, 3)) for c in cells}) == len(cells)


def test_marks_sit_on_cell_centres_that_are_over_threshold():
    dri = dri_for([(0.0, 0.0), (1.5, 1.0)])
    for c in high_risk_cells(dri, THRESHOLD):
        assert dri.risk_at(c.x, c.y) > THRESHOLD
        assert c.risk == dri.risk_at(c.x, c.y)
        # 셀 중심: (정수 + 0.5) * 0.2
        assert math.isclose((c.x / RES) % 1.0, 0.5, abs_tol=1e-6)
        assert math.isclose((c.y / RES) % 1.0, 0.5, abs_tol=1e-6)


def test_u_shaped_wall_is_marked_on_the_wall_not_in_the_empty_middle():
    """예전(구역 평균점) 방식은 ㄷ자 벽의 X 하나가 빈 가운데(DRI 0.01)에 찍혔다."""
    wall = ([(-2.0, y) for y in np.arange(-5, 5, 0.1)]
            + [(2.0, y) for y in np.arange(-5, 5, 0.1)]
            + [(x, 5.0) for x in np.arange(-2, 2, 0.1)])
    dri = dri_for(wall)
    cells = high_risk_cells(dri, THRESHOLD)
    assert len(cells) > 50
    assert all(dri.risk_at(c.x, c.y) > THRESHOLD for c in cells)
    assert dri.risk_at(0.0, 0.0) < THRESHOLD                 # 빈 가운데
    assert not any(abs(c.x) < 1.0 and c.y < 4.0 for c in cells)


def test_nothing_or_equal_to_threshold_gives_no_marks():
    dri = dri_for([(0.0, 0.0)])
    peak = float(dri.data.max())
    assert high_risk_cells(dri, peak) == []                  # 같으면 넘은 게 아님
    assert len(high_risk_cells(dri, peak - 1e-6)) >= 1
    assert high_risk_cells(dri_for([]), THRESHOLD) == []


def test_world_coordinates_follow_anchor_and_resolution():
    data = np.zeros((10, 10), np.float32)
    data[3, 4] = 2.0
    dri = DriGrid(data=data, anchor=(-20, 30), resolution=0.5)
    (cell,) = high_risk_cells(dri, 1.0)
    assert math.isclose(cell.x, (-20 + 4 + 0.5) * 0.5)
    assert math.isclose(cell.y, (30 + 3 + 0.5) * 0.5)
    assert cell.risk == 2.0


def test_nan_is_ignored_and_limit_keeps_the_riskiest():
    data = np.zeros((8, 8), np.float32)
    data[1, 1] = float('nan')
    for i, row in enumerate((0, 2, 4, 6)):
        data[row, 6] = 1.0 + i
    dri = DriGrid(data=data, anchor=(0, 0), resolution=1.0)
    assert [c.risk for c in high_risk_cells(dri, 0.5, limit=2)] == [4.0, 3.0]
    assert len(high_risk_cells(dri, 0.5)) == 4


def test_x_fits_inside_one_cell_and_is_centred():
    dri = dri_for([(0.0, 0.0)])
    cell = high_risk_cells(dri, THRESHOLD)[0]
    (a, b), (c, d) = x_segments([cell], 0.15)
    for p, q in ((a, b), (c, d)):
        assert math.isclose((p[0] + q[0]) / 2, cell.x) and math.isclose((p[1] + q[1]) / 2, cell.y)
        assert abs(q[0] - p[0]) <= RES and abs(q[1] - p[1]) <= RES   # 셀(0.2 m) 안
    dot = (b[0] - a[0]) * (d[0] - c[0]) + (b[1] - a[1]) * (d[1] - c[1])
    assert abs(dot) < 1e-12
    assert x_segments([], 0.15) == []
