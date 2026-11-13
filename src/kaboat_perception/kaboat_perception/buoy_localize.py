"""buoy_localize — 부표용 라이다 덩어리와 카메라 투영 기하 (ROS 무의존 순수 모듈).

부표 지도는 buoy_lidar_detector 가 만든다: 라이다 = 있음·위치, 카메라 RGB = 색 (buoy_color),
존재 확률 = 부표별 log-odds (buoy_evidence). 여기는 그 노드가 쓰는 기하만 둔다.
  lidar_points    LaserScan → 선체 수평 프레임 점 (라이다 장착 위치 반영)
  lidar_blobs     점 → 덩어리 (거리 기반 군집화) [(점 N×2, 중심, 반폭)] — 원형으로 가정하지 않는다
  level_to_pixel  선체 수평 프레임 3D 점 → 영상 픽셀 (카메라 장착·롤·피치)

덩어리: 두 점 사이가 cluster_eps 안이면 같은 물체 (단일 연결 — 스캔 순서와 무관해서 빔이 중간에
빠져도 한 덩어리로 이어진다). 점 lidar_min_points 개 이상, 폭 min_blob_width ~ max_cluster_width
만 남긴다 — 너무 넓으면 벽·물가·배. 중심은 점 평균, 반폭은 가장 먼 두 점 거리의 절반.
노이즈: 물보라·수면 반사는 몇 빔이 듬성듬성 맞는다. 맞은 빔이 덮는 폭(점 수 × 거리 × 빔 간격)이
min_cover 미만이거나, 덩어리 각도 범위 안 빔 중 맞은 비율이 min_fill 미만이면 버린다
(2026-11-12 bag: 부표 덩어리 채움비 p5 0.6~0.7, 버려진 물 덩어리 p50 0.37).
(예전엔 빔 순서대로 이어 가다 0.3 m 벌어지면 자르고 원의 앞면으로 보고 중심을 뒤로 밀었다.)

2026-11-12 이전의 카메라 프레임 중심 buoy_detector(뎁스로 물체를 따는 경로)는 윤슬·반사에서
뎁스가 크게 튀어 지웠다.

좌표: 선체 기준 수평 프레임 — 원점 = base_link(선체 중심), x 앞, y 왼쪽.
라이다 점은 /scan/level(배가 수평일 때만)이라 회전 없이 옮긴다.
"""
import math
from dataclasses import dataclass

import numpy as np

from kaboat_perception import camera_mount
from kaboat_perception.camera_obstacles import level_rotation


@dataclass
class BuoyGeometry:
    # 카메라 장착 — camera_mount.py 실측값 (occupancy_grid 카메라 층·도킹 TF 와 같은 출처).
    cam_offset_x: float = camera_mount.CAMERA_X
    cam_offset_y: float = camera_mount.CAMERA_Y
    cam_height: float = camera_mount.CAMERA_HEIGHT_ABOVE_WATER   # 흘수선에서 렌즈까지 [m]
    cam_mount_pitch_deg: float = camera_mount.CAMERA_PITCH_DEG   # + = 아래
    # 선체 라이다 — occupancy_grid·gate_follower 의 lidar_offset_x/y 와 같은 값.
    lidar_offset_x: float = camera_mount.LIDAR_X
    lidar_offset_y: float = 0.0
    # 라이다 덩어리
    cluster_eps: float = 0.35         # 두 점이 이 안이면 같은 덩어리 [m]
    lidar_min_points: int = 3         # 덩어리 최소 점 수 (1~2 빔은 물보라·수면)
    min_blob_width: float = 0.1       # 이보다 좁으면 부표 아님 (수면 hit) [m]
    max_cluster_width: float = 1.5    # 이보다 넓으면 부표 아님 (벽·물가·배) [m]
    # 노이즈 거르기 (angle_inc 를 줄 때만) — 물보라·수면 반사는 빔이 듬성듬성 맞는다.
    min_cover: float = 0.05           # 맞은 빔들이 덮는 폭 (점 수 × 거리 × 빔 간격) 하한 [m] — 필요한 점 수가
                                      # 거리에 반비례 (2.4 m 4 점 → 버림, 6.6 m 초록 공 5 점 → 남김. 0.1 이면
                                      # 초록 공 확정이 8 s 늦었다)
    min_fill: float = 0.5             # 덩어리 각도 범위 안 빔 중 맞은 비율 하한 (부표 면은 빔이 이어서 맞음)


def lidar_points(ranges, angle_min, angle_inc, range_min, range_max, g: BuoyGeometry):
    """LaserScan → 수평 프레임 점 N×2 (빔 순서 유지)."""
    r = np.asarray(ranges, dtype=float)
    a = angle_min + angle_inc * np.arange(len(r))
    ok = np.isfinite(r) & (r > range_min) & (r < range_max)
    return np.column_stack((g.lidar_offset_x + r[ok] * np.cos(a[ok]),
                            g.lidar_offset_y + r[ok] * np.sin(a[ok])))


def lidar_blobs(points, g: BuoyGeometry, angle_inc=None, rejected=None):
    """점 N×2 → 부표 크기 덩어리 [(점, 중심, 반폭)]. 두 점이 cluster_eps 안이면 같은 덩어리.

    격자(칸 = cluster_eps)에 점을 나눠 이웃 칸끼리만 거리를 재는 union-find — 점 수에 거의 선형.
    angle_inc(빔 간격 [rad]) 를 주면 듬성듬성한 덩어리(min_cover·min_fill)도 노이즈로 버린다.
    rejected 에 리스트를 주면 버린 덩어리를 (점, 중심, 반폭, 이유) 로 담는다 (시각화용).
    """
    pts = np.asarray(points, dtype=float).reshape(-1, 2)
    n = len(pts)
    if n == 0:
        return []
    eps = g.cluster_eps
    parent = list(range(n))

    def find(i):
        while parent[i] != i:
            parent[i] = parent[parent[i]]
            i = parent[i]
        return i

    cells = {}
    keys = np.floor(pts / eps).astype(np.int64)
    for i, (cx, cy) in enumerate(keys):
        cells.setdefault((int(cx), int(cy)), []).append(i)
    eps2 = eps * eps
    for (cx, cy), idx in cells.items():
        cand = []
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                cand.extend(cells.get((cx + dx, cy + dy), ()))
        cand = np.array(cand)
        for i in idx:
            d2 = np.sum((pts[cand] - pts[i]) ** 2, axis=1)
            for j in cand[d2 <= eps2]:
                ri, rj = find(i), find(int(j))
                if ri != rj:
                    parent[ri] = rj
    groups = {}
    for i in range(n):
        groups.setdefault(find(i), []).append(i)
    out = []
    lidar = np.array([g.lidar_offset_x, g.lidar_offset_y])
    for idx in groups.values():
        b = pts[idx]
        if len(b) >= 2:
            diff = b[:, None, :] - b[None, :, :]
            width = float(np.sqrt(np.max(np.sum(diff ** 2, axis=2))))   # 가장 먼 두 점
        else:
            width = 0.0
        reason = None
        if len(idx) < g.lidar_min_points:
            reason = 'points'
        elif width < g.min_blob_width:
            reason = 'narrow'
        elif width > g.max_cluster_width:
            reason = 'wide'
        elif angle_inc:
            rel = b - lidar
            m = rel.mean(axis=0)
            d = float(np.hypot(*m))
            a = np.arctan2(rel[:, 1], rel[:, 0]) - math.atan2(m[1], m[0])
            a = (a + math.pi) % (2.0 * math.pi) - math.pi        # 덩어리 방향 기준 (뒤쪽 ±π 경계)
            span = (a.max() - a.min()) / angle_inc + 1.0
            if len(b) * d * angle_inc < g.min_cover:
                reason = 'cover'
            elif len(b) / span < g.min_fill:
                reason = 'fill'
        if reason is None:
            out.append((b, b.mean(axis=0), 0.5 * width))
        elif rejected is not None:
            rejected.append((b, b.mean(axis=0), 0.5 * width, reason))
    return out


def level_to_pixel(pts, K, g: BuoyGeometry, roll=0.0, pitch=0.0):
    """수평 프레임 점 N×3 → (u, v, 광학 깊이) — pixels_to_level 의 역."""
    a = math.radians(g.cam_mount_pitch_deg)
    mount = np.array([[math.cos(a), 0.0, math.sin(a)], [0.0, 1.0, 0.0],
                      [-math.sin(a), 0.0, math.cos(a)]])
    level = level_rotation(roll, pitch)
    t = level @ np.array([g.cam_offset_x, g.cam_offset_y, g.cam_height])
    body = (np.asarray(pts, dtype=float) - t) @ (level @ mount)   # 앞, 왼, 위
    fx, fy, cx, cy = K
    return cx - fx * body[:, 1] / body[:, 0], cy - fy * body[:, 2] / body[:, 0], body[:, 0]
