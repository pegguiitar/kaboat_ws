"""buoy_localize — 부표용 라이다 군집과 카메라 투영 기하 (ROS 무의존 순수 모듈).

부표 지도는 buoy_lidar_detector 가 만든다: 라이다 = 있음·위치, 카메라 RGB = 색 (buoy_color),
존재 확률 = 부표별 log-odds (buoy_evidence). 여기는 그 노드가 쓰는 기하만 둔다.
  lidar_points          LaserScan → 선체 수평 프레임 점 (라이다 장착 위치 반영, 빔 순서 유지)
  lidar_clusters        점 → 부표 크기 군집 [(중심, 반지름, 빔 수)] (넓은 벽·물가 제외)
  lidar_cluster_points  같은 군집, 군집 점까지
  surface_center        앞면 점 → 중심·반지름 (한 스캔 추정 — 트래커가 원 맞추기로 다시 잡는다)
  level_to_pixel        선체 수평 프레임 3D 점 → 영상 픽셀 (카메라 장착·롤·피치)

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
    # 라이다 군집
    max_radius: float = 0.5           # 한 스캔 반지름 추정 상한 [m]
    max_cluster_width: float = 1.5    # 이보다 넓은 군집은 부표 아님 (벽·물가·배) [m]
    cluster_gap: float = 0.3          # 이웃 라이다 점 간격이 이보다 크면 다른 물체 [m]
    lidar_min_points: int = 2         # 라이다 군집 최소 빔 수 (1빔은 물보라·노이즈)


def surface_center(pts, origin, g: BuoyGeometry, robust=False, pad=0.0):
    """앞면 점들 (N×2) → (중심 x·y, 추정 반지름). origin = 그 점들을 본 센서 위치.

    반지름 = (시선 수직 폭 + pad) / 2 (robust 면 5~95 백분위 폭 — 뎁스 가장자리 튐 대비),
    중심 = 점 평균 + (π/4)·r̂ · 시선 방향. pad 는 표본 간격 보정 — 실제 가장자리는
    양 끝 표본에서 반 칸씩 바깥이라 라이다는 빔 한 칸(거리·각분해능)을 더한다.
    """
    mean = pts.mean(axis=0)
    los = mean - origin
    los = los / max(float(np.hypot(*los)), 1e-6)
    lateral = (pts - mean) @ np.array([-los[1], los[0]])
    if robust:
        lo, hi = np.percentile(lateral, [5.0, 95.0])
    else:
        lo, hi = lateral.min(), lateral.max()
    r = min(0.5 * (float(hi - lo) + pad), g.max_radius)
    return mean + (math.pi / 4.0) * r * los, r



def lidar_points(ranges, angle_min, angle_inc, range_min, range_max, g: BuoyGeometry):
    """LaserScan → 수평 프레임 점 N×2 (빔 순서 유지)."""
    r = np.asarray(ranges, dtype=float)
    a = angle_min + angle_inc * np.arange(len(r))
    ok = np.isfinite(r) & (r > range_min) & (r < range_max)
    return np.column_stack((g.lidar_offset_x + r[ok] * np.cos(a[ok]),
                            g.lidar_offset_y + r[ok] * np.sin(a[ok])))


def lidar_clusters(points, g: BuoyGeometry, angle_inc=0.0):
    """라이다 점 → 부표 크기 군집들의 [(중심 x·y, 반지름, 빔 수)] (넓은 벽·물가 제외).

    angle_inc = 스캔 각분해능 [rad] — 먼 부표는 빔 몇 개뿐이라 폭을 빔 한 칸만큼 보정.
    """
    return [(center, r, len(cl)) for cl, center, r in lidar_cluster_points(points, g, angle_inc)]


def lidar_cluster_points(points, g: BuoyGeometry, angle_inc=0.0):
    """lidar_clusters 와 같은 군집 — [(군집 점 N×2, 중심, 반지름)] (점까지 필요할 때)."""
    if points is None or len(points) == 0:
        return []
    origin = np.array([g.lidar_offset_x, g.lidar_offset_y])
    gaps = np.hypot(*np.diff(points, axis=0).T) > g.cluster_gap
    out = []
    for cl in np.split(points, np.nonzero(gaps)[0] + 1):
        if len(cl) < g.lidar_min_points or np.hypot(*(cl[-1] - cl[0])) > g.max_cluster_width:
            continue
        pad = float(np.hypot(*(cl.mean(axis=0) - origin))) * abs(angle_inc)
        center, r = surface_center(cl, origin, g, pad=pad)
        out.append((cl, center, r))
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
