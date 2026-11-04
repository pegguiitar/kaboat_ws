import math
from types import SimpleNamespace as NS

import numpy as np
import pytest

from kaboat_behaviors.dri import DriParams, build_dri
from kaboat_hardware.tank_walls import (
    build_tank_dri, edge_distance, mask_wall_cells, split_wall_cells, valid_bounds)

BOUNDS = (0.0, 0.0, 10.0, 5.0)
RES = 0.2


def grid_with(cells, n=75, origin=(-2.0, -5.0)):
    """origin 에서 시작하는 n×n 격자에 (x, y) 점유 셀을 찍는다."""
    data = np.full((n, n), -1, np.int16)
    for x, y in cells:
        data[int((y - origin[1]) / RES), int((x - origin[0]) / RES)] = 97
    return NS(header=NS(frame_id='odom'),
              info=NS(resolution=RES, width=n, height=n,
                      origin=NS(position=NS(x=origin[0], y=origin[1]))),
              data=data.ravel().tolist())


def wall_cells():
    cells = []
    for x in np.arange(0.0, 10.0, 0.1):
        cells += [(x, 0.05), (x, 4.95)]
    for y in np.arange(0.0, 5.0, 0.1):
        cells += [(0.05, y), (9.95, y)]
    return cells


def occupied(grid):
    return {(i % grid.info.width, i // grid.info.width)
            for i, v in enumerate(grid.data) if v >= 0}


def test_edge_distance_is_inside_distance_and_negative_outside():
    assert edge_distance(5.0, 2.5, BOUNDS) == pytest.approx(2.5)
    assert edge_distance(1.0, 2.5, BOUNDS) == pytest.approx(1.0)
    assert edge_distance(9.4, 2.5, BOUNDS) == pytest.approx(0.6)
    assert edge_distance(5.0, 0.3, BOUNDS) == pytest.approx(0.3)
    assert edge_distance(5.0, 4.7, BOUNDS) == pytest.approx(0.3)
    assert edge_distance(-0.5, 2.5, BOUNDS) < 0 and edge_distance(5.0, 6.0, BOUNDS) < 0


def test_valid_bounds_requires_a_positive_finite_area():
    assert valid_bounds(BOUNDS)
    assert not valid_bounds((0.0, 0.0, 0.0, 0.0))          # 기본값 = 꺼짐
    assert not valid_bounds((0.0, 0.0, 10.0))
    assert not valid_bounds((0.0, 0.0, float('nan'), 5.0))
    assert not valid_bounds((10.0, 0.0, 0.0, 5.0))
    assert not valid_bounds(None)


def test_wall_cells_inside_the_band_are_removed_and_the_source_is_untouched():
    grid = grid_with(wall_cells())
    before = list(grid.data)
    masked = mask_wall_cells(grid, BOUNDS, 0.45)
    assert occupied(masked) == set()
    assert grid.data == before                               # 원본 불변


def test_obstacles_beyond_the_band_survive_and_those_in_it_do_not():
    grid = grid_with(wall_cells() + [(5.0, 2.5), (5.0, 0.3), (5.0, 0.65)])
    masked = mask_wall_cells(grid, BOUNDS, 0.45)
    kept = occupied(masked)
    assert len(kept) == 2                                    # 0.7 m, 2.5 m 는 남고 0.3 m 는 사라짐
    res_y = lambda iy: -5.0 + (iy + 0.5) * RES
    assert sorted(round(res_y(iy), 1) for _, iy in kept) == [0.7, 2.5]


def test_cells_outside_the_tank_are_removed_too():
    grid = grid_with([(-1.0, 2.5), (11.0, 2.5), (5.0, 7.0), (5.0, 2.5)], n=100)
    kept = occupied(mask_wall_cells(grid, BOUNDS, 0.45))
    assert len(kept) == 1


def test_masked_grid_drives_build_dri_without_wall_risk():
    boat = (5.0, 2.5)
    cells = wall_cells() + [(6.5, 2.5)]
    raw = build_dri(grid_with(cells), boat, 0.0, DriParams())
    masked = build_dri(mask_wall_cells(grid_with(cells), BOUNDS, 0.45),
                       boat, 0.0, DriParams())
    assert len(raw.occ_xy) > len(masked.occ_xy) == 1         # 부표 하나만 위험원
    assert raw.risk_at(9.5, 2.5) > 0.85                      # 벽을 두면 벽 근처가 위험
    assert masked.risk_at(9.5, 2.5) == 0.0                   # 벽을 빼면 위험 없음
    assert masked.risk_at(6.5, 2.5) > 0.85                   # 부표는 그대로 위험


def test_split_keeps_every_observed_cell_in_exactly_one_grid():
    grid = grid_with(wall_cells() + [(5.0, 2.5)])
    rest, wall = split_wall_cells(grid, BOUNDS, 0.3)
    assert occupied(rest).isdisjoint(occupied(wall))
    assert occupied(rest) | occupied(wall) == occupied(grid)
    assert len(occupied(rest)) == 1 and len(occupied(wall)) > 50


def test_tank_dri_shows_walls_weakly_but_keeps_real_obstacles_at_full_risk():
    boat = (5.0, 2.5)
    grid = grid_with(wall_cells() + [(6.5, 2.5)])
    full = build_dri(grid, boat, 0.0, DriParams())
    soft = build_tank_dri(grid, boat, 0.0, DriParams(), BOUNDS, 0.3, 0.5)
    gone = build_tank_dri(grid, boat, 0.0, DriParams(), BOUNDS, 0.3, 0.0)

    assert full.risk_at(9.5, 2.5) > 0.85           # 벽을 장애물로 보면 문턱 초과
    assert 0.1 < soft.risk_at(9.5, 2.5) <= 0.5     # 상한 처리: 보이지만 문턱 아래
    assert gone.risk_at(9.5, 2.5) == 0.0           # 0 이면 지운 것과 같음
    assert soft.risk_at(6.5, 2.5) == pytest.approx(full.risk_at(6.5, 2.5))   # 부표는 그대로
    # 벽은 기준 장애물/전방 장애물 확인에 안 들어간다
    assert len(soft.occ_xy) == len(gone.occ_xy) == 1
    # 벽 때문에 문턱(0.85)을 넘는 칸이 없다
    wall_band = [(x, y) for x in np.arange(0.0, 10.0, 0.2) for y in np.arange(0.0, 5.0, 0.2)
                 if min(x, 10 - x, y, 5 - y) < 1.0]
    assert max(soft.risk_at(x, y) for x, y in wall_band) <= 0.85
    assert max(soft.risk_at(x, y) for x, y in wall_band if abs(x - 6.5) > 2.0) <= 0.5 + 1e-6


def test_tank_dri_without_any_wall_cells_equals_plain_dri():
    boat = (5.0, 2.5)
    grid = grid_with([(6.5, 2.5)])
    plain = build_dri(grid, boat, 0.0, DriParams())
    soft = build_tank_dri(grid, boat, 0.0, DriParams(), BOUNDS, 0.3, 0.5)
    assert np.array_equal(plain.data, soft.data)
