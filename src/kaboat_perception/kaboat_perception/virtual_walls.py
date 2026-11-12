"""가상 벽 — 라이다·카메라로 안 보이는 경계(노란 작은 부이 로프 등)를 지도에 장애물로 박는다.

벽은 위도·경도 꺾은선으로 config/virtual_walls.yaml 에 둔다. 원점을 다시 잡아도 그대로
쓸 수 있게, 켤 때(그리고 원점 파일이 바뀔 때) 지금 원점(~/kaboat_geo_origin.yaml) 기준
x(동)·y(북) [m] 로 바꾼다 — gq7_local_odom(/odom)·rec·gps_mark_logger 와 같은 ENU 계산.
(kaboat_hardware.geo_local 과 같은 식. 그 패키지가 이 패키지에 의존해 여기서 가져다 쓸 수 없다.)
"""

import math
import os

import cv2
import numpy as np
import yaml

WGS84_A = 6378137.0
WGS84_E2 = 6.69437999014e-3


def _ecef(lat, lon, height):
    phi, lam = math.radians(lat), math.radians(lon)
    sp, cp = math.sin(phi), math.cos(phi)
    n = WGS84_A / math.sqrt(1.0 - WGS84_E2 * sp * sp)
    return ((n + height) * cp * math.cos(lam), (n + height) * cp * math.sin(lam),
            (n * (1.0 - WGS84_E2) + height) * sp)


def latlon_to_local(lat, lon, origin):
    """위도·경도 → 원점 (lat0, lon0, alt0) 기준 (동, 북) [m]. 높이는 원점 높이(수면)."""
    lat0, lon0, alt0 = origin
    x0, y0, z0 = _ecef(lat0, lon0, alt0)
    x, y, z = _ecef(lat, lon, alt0)
    dx, dy, dz = x - x0, y - y0, z - z0
    sp, cp = math.sin(math.radians(lat0)), math.cos(math.radians(lat0))
    sl, cl = math.sin(math.radians(lon0)), math.cos(math.radians(lon0))
    east = -sl * dx + cl * dy
    north = -sp * cl * dx - sp * sl * dy + cp * dz
    return east, north


def load_walls(path):
    """yaml 의 walls: {이름: [[lat, lon], …]} → {이름: [(lat, lon), …]}. 파일 없으면 {}."""
    path = os.path.expanduser(path)
    if not path or not os.path.exists(path):
        return {}
    with open(path, encoding='utf-8') as f:
        data = yaml.safe_load(f) or {}
    walls = {}
    for name, points in (data.get('walls') or {}).items():
        pts = [(float(p[0]), float(p[1])) for p in points]
        if len(pts) < 2:
            raise ValueError(f'가상 벽 {name!r}: 점이 2개 이상이어야 함')
        for lat, lon in pts:
            if not (math.isfinite(lat) and -90 <= lat <= 90
                    and math.isfinite(lon) and -180 <= lon <= 180):
                raise ValueError(f'가상 벽 {name!r}: 위도·경도가 잘못됨 ({lat}, {lon})')
        walls[str(name)] = pts
    return walls


def walls_to_local(walls, origin):
    """{이름: [(lat, lon)…]} → {이름: N×2 (동, 북)}."""
    return {name: np.array([latlon_to_local(lat, lon, origin) for lat, lon in pts])
            for name, pts in walls.items()}


def wall_mask(walls_xy, anchor, cells, resolution, thickness):
    """rolling window(anchor = 배열 (0,0) 칸의 전역 칸 번호)에 벽 꺾은선을 칠한 [iy, ix] bool."""
    mask = np.zeros((cells, cells), np.uint8)
    width = max(1, int(round(thickness / resolution)))
    for xy in walls_xy.values():
        pts = np.stack([np.floor(xy[:, 0] / resolution) - anchor[0],
                        np.floor(xy[:, 1] / resolution) - anchor[1]], axis=1)
        # cv2 는 int32 좌표 — 창에서 아주 먼 점은 잘라 둔다 (선은 창 경계에서 잘림)
        pts = np.clip(pts, -100000, 100000).astype(np.int32).reshape(-1, 1, 2)
        cv2.polylines(mask, [pts], False, 1, thickness=width)
    return mask.astype(bool)
