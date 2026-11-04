import math
from types import SimpleNamespace as NS

import numpy as np

from kaboat_behaviors.slot_clearance import measure_slot

RES = 0.2
N = 100
ORIGIN = -10.0
WIDTH = 3.0      # 슬롯 벽-벽 폭
BACK_X = 6.0     # 뒷벽 x
HULL = 1.0


def slot_grid(width=WIDTH, mouth_x=2.0, back_x=BACK_X, value=97):
    data = np.full((N, N), -1, np.int16)

    def put(x, y):
        ix, iy = int((x - ORIGIN) / RES), int((y - ORIGIN) / RES)
        if 0 <= ix < N and 0 <= iy < N:
            data[iy, ix] = value

    for x in np.arange(mouth_x, back_x + 1e-9, RES / 2):
        put(x, width / 2)
        put(x, -width / 2)
    for y in np.arange(-width / 2, width / 2 + 1e-9, RES / 2):
        put(back_x, y)
    return NS(info=NS(resolution=RES, width=N, height=N,
                      origin=NS(position=NS(x=ORIGIN, y=ORIGIN))),
              data=data.ravel().tolist())


def measure(grid, xy, yaw=0.0, front=0.6, **kw):
    return measure_slot(grid, xy, yaw, hull_width=HULL, back=0.5,
                        front=front, **kw)


def test_centered_hull_sees_one_metre_each_side():
    c = measure(slot_grid(), (3.0, 0.0))
    assert c.both
    assert abs(c.left - 1.0) <= RES and abs(c.right - 1.0) <= RES
    assert abs(c.slot_width - WIDTH) <= 2 * RES


def test_offset_hull_reports_asymmetric_gap():
    c = measure(slot_grid(), (3.0, 0.5))    # 좌측(+y)으로 0.5m 치우침
    assert c.left < c.right
    assert abs(c.left - 0.5) <= RES and abs(c.right - 1.5) <= RES


def test_back_wall_is_not_taken_for_a_side_wall():
    grid = slot_grid()
    # 접안 위치: 뒷벽까지 1.0m. 창을 뒷벽 앞으로 제한하면 옆벽만 잡힌다.
    ok = measure(grid, (5.0, 0.0), front=0.6)
    assert ok.both and ok.min_gap > 0.7
    # 창이 뒷벽까지 닿으면 뒷벽 셀이 간격 ~0 의 옆벽으로 잡힌다(호출자가 막아야 함).
    bad = measure(grid, (5.0, 0.0), front=1.2)
    assert bad.min_gap < 0.3


def test_cells_inside_hull_footprint_are_ignored():
    grid = slot_grid()
    ix = int((3.0 - ORIGIN) / RES)
    iy = int((0.0 - ORIGIN) / RES)
    data = np.asarray(grid.data, np.int16).reshape(N, N)
    data[iy, ix] = 97                      # 선체 중심 위 가짜 점유
    grid.data = data.ravel().tolist()
    c = measure(grid, (3.0, 0.0))
    assert abs(c.left - 1.0) <= RES and abs(c.right - 1.0) <= RES


def test_hull_yaw_is_taken_into_account():
    grid = slot_grid()
    c = measure(grid, (3.0, 0.0), yaw=math.radians(20), front=0.6)
    assert c.both and 0.6 < c.min_gap < 1.1


def test_no_walls_or_weak_cells_give_none():
    empty = NS(info=NS(resolution=RES, width=N, height=N,
                       origin=NS(position=NS(x=ORIGIN, y=ORIGIN))),
               data=[-1] * (N * N))
    c = measure(empty, (3.0, 0.0))
    assert c.left is None and c.right is None and not c.both
    assert c.min_gap is None and c.slot_width is None
    weak = measure(slot_grid(value=70), (3.0, 0.0), occ_threshold=85)
    assert not weak.both


def test_one_wall_only_is_reported_per_side():
    grid = slot_grid()
    data = np.asarray(grid.data, np.int16).reshape(N, N)
    data[int((0.0 - ORIGIN) / RES):, :] = -1   # y>=0 (좌측) 벽 제거
    grid.data = data.ravel().tolist()
    c = measure(grid, (3.0, 0.0))
    assert c.left is None and c.right is not None and not c.both
