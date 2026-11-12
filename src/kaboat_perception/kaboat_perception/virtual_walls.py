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


def _read_yaml(path):
    path = os.path.expanduser(path)
    if not path or not os.path.exists(path):
        return {}
    with open(path, encoding='utf-8') as f:
        return yaml.safe_load(f) or {}


def _latlon_points(name, points, minimum):
    pts = [(float(p[0]), float(p[1])) for p in points]
    if len(pts) < minimum:
        raise ValueError(f'가상 벽 {name!r}: 점이 {minimum}개 이상이어야 함')
    for lat, lon in pts:
        if not (math.isfinite(lat) and -90 <= lat <= 90
                and math.isfinite(lon) and -180 <= lon <= 180):
            raise ValueError(f'가상 벽 {name!r}: 위도·경도가 잘못됨 ({lat}, {lon})')
    return pts


def load_walls(path):
    """yaml 의 walls: {이름: [[lat, lon], …]} → {이름: [(lat, lon), …]}. 파일 없으면 {}."""
    return {str(name): _latlon_points(name, points, 2)
            for name, points in (_read_yaml(path).get('walls') or {}).items()}


def load_arenas(path):
    """yaml 의 arenas: {이름: {corners: [[lat, lon], …], openings: {변 번호: 폭 m}}}.

    corners 는 둘레 순서, 변 i = 모서리 i → i+1 (마지막 → 처음). openings 의 변은 가운데를
    그 폭만큼 비우고 나머지 변은 전부 벽. 파일 없으면 {}.
    """
    arenas = {}
    for name, spec in (_read_yaml(path).get('arenas') or {}).items():
        corners = _latlon_points(name, (spec or {}).get('corners') or [], 3)
        openings = {}
        for side, width in ((spec or {}).get('openings') or {}).items():
            side, width = int(side), float(width)
            if not 0 <= side < len(corners):
                raise ValueError(f'경기장 {name!r}: 변 번호 {side} 는 0~{len(corners) - 1}')
            if not (math.isfinite(width) and width > 0):
                raise ValueError(f'경기장 {name!r}: 변 {side} 비우는 폭이 잘못됨 ({width})')
            openings[side] = width
        arenas[str(name)] = {'corners': corners, 'openings': openings}
    return arenas


def offset_polygon(corners_xy, distance):
    """닫힌 다각형을 바깥쪽으로 distance [m] 넓힌다 (모서리는 두 변의 연장선이 만나는 점)."""
    q = np.asarray(corners_xy, dtype=float)
    if distance == 0.0:
        return q.copy()
    nxt = np.roll(q, -1, axis=0)
    ccw = float(np.sum(q[:, 0] * nxt[:, 1] - nxt[:, 0] * q[:, 1])) > 0.0
    edges = nxt - q
    tangent = edges / np.linalg.norm(edges, axis=1)[:, None]
    normal = np.stack([tangent[:, 1], -tangent[:, 0]], axis=1)    # 반시계면 오른쪽 = 바깥
    if not ccw:
        normal = -normal
    before = np.roll(normal, 1, axis=0)                           # 꼭짓점 i 로 들어오는 변
    miter = (before + normal) / np.maximum(1.0 + np.sum(before * normal, axis=1), 1e-6)[:, None]
    return q + distance * miter


def arena_walls(name, corners_xy, openings, offset=0.0):
    """경기장 둘레를 offset 만큼 바깥으로 넓혀 벽 꺾은선들로. openings 변은 가운데만 비운다.

    {이름_번호: N×2}. 비운 곳이 없으면 닫힌 꺾은선 하나.
    """
    q = offset_polygon(corners_xy, offset)
    n = len(q)

    def cut(i):
        a, b = q[i], q[(i + 1) % n]
        length = float(np.linalg.norm(b - a))
        gap = min(openings[i], length)
        u = (b - a) / max(length, 1e-9)
        return a + u * (length - gap) / 2.0, a + u * (length + gap) / 2.0

    if not openings:
        return {f'{name}_0': np.vstack([q, q[:1]])}
    first = min(openings)
    pieces, current = [], [cut(first)[1]]
    for k in range(1, n + 1):
        i = (first + k) % n
        current.append(q[i])                    # 앞 변의 끝 = 이 변의 시작 모서리
        if i in openings:
            start_gap, end_gap = cut(i)
            current.append(start_gap)
            pieces.append(current)
            current = [end_gap]
    walls = {}
    for piece in pieces:
        pts = [p for j, p in enumerate(piece) if j == 0 or np.linalg.norm(p - piece[j - 1]) > 1e-6]
        if len(pts) >= 2:
            walls[f'{name}_{len(walls)}'] = np.array(pts)
    return walls


def layout_to_local(walls, arenas, origin, offset=0.0):
    """walls + arenas (위도·경도) → 지금 원점 기준 벽 꺾은선 {이름: N×2}, 바깥으로 offset 민 것."""
    out = offset_outward(walls_to_local(walls, origin), offset)
    for name, spec in arenas.items():
        corners = np.array([latlon_to_local(lat, lon, origin) for lat, lon in spec['corners']])
        out.update(arena_walls(name, corners, spec['openings'], offset))
    return out


def walls_to_local(walls, origin):
    """{이름: [(lat, lon)…]} → {이름: N×2 (동, 북)}."""
    return {name: np.array([latlon_to_local(lat, lon, origin) for lat, lon in pts])
            for name, pts in walls.items()}


def offset_outward(walls_xy, distance):
    """각 벽을 경기장 가운데(모든 벽 점의 평균)에서 멀어지는 쪽으로 distance [m] 평행 이동.

    벽 방향(처음 점 → 끝 점)의 수직으로 벽 전체를 민다 — 직선(두 점) 벽 기준. 기록 지점은
    배를 부표 안쪽 옆에 대고 찍어서 벽이 실제 로프보다 안쪽에 잡힌다(2026-11-12 현장 RViz).
    가운데가 벽 선 위라 바깥을 정할 수 없는 벽(벽이 하나뿐 등)은 그대로 둔다.
    """
    if not walls_xy or distance == 0.0:
        return dict(walls_xy)
    center = np.vstack(list(walls_xy.values())).mean(axis=0)
    shifted = {}
    for name, xy in walls_xy.items():
        direction = xy[-1] - xy[0]
        length = float(np.hypot(*direction))
        if length < 1e-9:
            shifted[name] = xy
            continue
        normal = np.array([-direction[1], direction[0]]) / length
        side = float((xy.mean(axis=0) - center) @ normal)
        shifted[name] = xy if abs(side) < 1e-6 else xy + math.copysign(distance, side) * normal
    return shifted


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
