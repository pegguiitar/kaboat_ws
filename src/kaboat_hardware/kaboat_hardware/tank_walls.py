"""tank_walls — 수조 벽을 장애물에서 빼고, 경계 접근을 판정하는 순수 라이브러리.

ROS 비의존. 수조 시험에서는 `/odom`이 곧 수조 좌표(좌하단 원점, x 0~가로,
y 0~세로)이고, `occupancy_grid`가 이미 선체 스캔을 odom 위치·yaw 로 월드 좌표에
옮겨 셀로 쌓아 두었다. 그래서 격자 셀 좌표가 그대로 수조 좌표이며, 경계 근처
셀을 "벽"으로 보고 지우면 스캔을 따로 변환할 필요가 없다.

벽 셀은 장애물에서 빠지지만 버려지지는 않는다. 벽 셀만 따로 DRI 를 계산해
상한(`wall_dri`, 예: 0.5)으로 눌러 합치므로, 지도에는 약한 위험으로 보이되
플래너의 후보 필터 문턱(safe_threshold × gen_margin = 1.2 × 0.93)보다 낮아 경로를 막지는 않는다. 기준 장애물(`occ_xy`)
과 "전방 장애물 확인"에는 벽이 들어가지 않는다. 플래너는 벽 쪽/반대쪽을 가를 만큼
벽을 의식하지 못하므로, 짝으로 `edge_distance`를 써서 선체 위치가 경계에 가까워지면
정지시킨다(이 노드가 하는 일).

bounds = (x_min, y_min, x_max, y_max) [m]
"""
import math
from types import SimpleNamespace

import numpy as np

from kaboat_behaviors.dri import build_dri


def valid_bounds(bounds) -> bool:
    """네 값이 유한하고 영역이 양의 크기일 때만 켜진 것으로 본다."""
    try:
        x0, y0, x1, y1 = (float(v) for v in bounds)
    except (TypeError, ValueError):
        return False
    return (all(math.isfinite(v) for v in (x0, y0, x1, y1))
            and x1 > x0 and y1 > y0)


def edge_distance(x: float, y: float, bounds) -> float:
    """점에서 가장 가까운 경계까지의 안쪽 거리 [m]. 경계 밖이면 음수."""
    x0, y0, x1, y1 = bounds
    return min(x - x0, x1 - x, y - y0, y1 - y)


def _wall_flags(grid, bounds, band: float):
    """(data, 관측 셀 인덱스 iy, ix, 벽 여부) — 경계 band 안·밖이면 벽."""
    info = grid.info
    res = float(info.resolution)
    width, height = int(info.width), int(info.height)
    data = np.asarray(grid.data, dtype=np.int16).reshape(height, width)
    iy, ix = np.nonzero(data >= 0)
    if ix.size == 0:
        return data, iy, ix, np.zeros(0, dtype=bool)
    wx = info.origin.position.x + (ix + 0.5) * res
    wy = info.origin.position.y + (iy + 0.5) * res
    x0, y0, x1, y1 = bounds
    inner = np.minimum.reduce([wx - x0, x1 - wx, wy - y0, y1 - wy])
    return data, iy, ix, inner < band


def _as_grid(grid, data):
    return SimpleNamespace(info=grid.info, header=getattr(grid, 'header', None),
                           data=data.ravel().tolist())


def split_wall_cells(grid, bounds, band: float):
    """(벽 제외 격자, 벽만 있는 격자). 원본은 바꾸지 않는다.

    경계에서 band 안쪽(그리고 경계 밖)에 있는 관측 셀이 벽이다. 각 격자에서
    해당하지 않는 셀은 미관측(-1)이며, 미관측은 DRI 에서 위험 0(자유)이다.
    """
    data, iy, ix, wall = _wall_flags(grid, bounds, band)
    rest = data.copy()
    only = np.full_like(data, -1)
    if ix.size:
        rest[iy[wall], ix[wall]] = -1
        only[iy[wall], ix[wall]] = data[iy[wall], ix[wall]]
    return _as_grid(grid, rest), _as_grid(grid, only)


def mask_wall_cells(grid, bounds, band: float):
    """벽 셀을 미관측(-1)으로 지운 격자(벽 DRI 를 아예 없앨 때)."""
    return split_wall_cells(grid, bounds, band)[0]


def build_tank_dri(grid, boat_xy, boat_yaw: float, dri_params, bounds,
                   band: float, wall_dri: float):
    """벽 셀은 DRI 상한 wall_dri 로 눌러 합친 DriGrid.

    wall_dri <= 0 이면 벽 셀을 지운 것과 같다. 반환 DriGrid 의 occ_xy 는 벽을
    뺀 실제 장애물 셀이다(플래너 기준 장애물·전방 장애물 확인용).
    """
    rest, wall = split_wall_cells(grid, bounds, band)
    dri = build_dri(rest, boat_xy, boat_yaw, dri_params)
    if wall_dri > 0.0 and np.any(np.asarray(wall.data) >= 0):
        wall_map = build_dri(wall, boat_xy, boat_yaw, dri_params)
        np.maximum(dri.data, np.minimum(wall_map.data, wall_dri), out=dri.data)
    return dri
