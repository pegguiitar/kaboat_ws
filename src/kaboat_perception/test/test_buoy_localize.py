"""buoy_localize.py 단독 테스트 — 부표용 라이다 군집·카메라 투영 기하 (ROS 없이 돈다)."""
import math

import numpy as np
import pytest

from kaboat_perception.buoy_localize import (
    BuoyGeometry, level_to_pixel, lidar_cluster_points, lidar_clusters, lidar_points,
    surface_center)
from kaboat_perception.camera_obstacles import CameraObstacleParams, pixels_to_level

K = (380.0, 380.0, 320.0, 240.0)   # 640×480 D455 근사
G = BuoyGeometry()                 # 현장 기본값 (카메라·라이다 장착)


def _scan(circles, inc_deg=0.5, range_max=40.0, g=G):
    """라이다 원점에서 원(cx, cy, r) 들까지의 첫 교점 거리 — 정면 ±90°."""
    angles = np.radians(np.arange(-90.0, 90.0 + 1e-9, inc_deg))
    ranges = np.full(len(angles), np.inf)
    o = np.array([g.lidar_offset_x, g.lidar_offset_y])
    for i, a in enumerate(angles):
        d = np.array([math.cos(a), math.sin(a)])
        for cx, cy, r in circles:
            oc = o - (cx, cy)
            b = oc @ d
            disc = b * b - (oc @ oc - r * r)
            if disc >= 0.0:
                t = -b - math.sqrt(disc)
                if 0.0 < t < ranges[i]:
                    ranges[i] = t
    return lidar_points(ranges.tolist(), float(angles[0]), math.radians(inc_deg),
                        0.1, range_max, g)



INC = math.radians(0.5)


# ---------- 라이다 군집 ----------

def test_lidar_clusters_skip_wide_wall():
    pts = _scan([(5.0, 0.0, 0.25), (12.0, 0.0, 4.0)])
    clusters = lidar_clusters(pts, G, INC)
    assert len(clusters) == 1
    assert math.hypot(*(clusters[0][0] - (5.0, 0.0))) < 0.05


def test_cluster_points_match_clusters():
    pts = _scan([(5.0, 1.0, 0.25), (8.0, -2.0, 0.3)])
    with_pts = lidar_cluster_points(pts, G, INC)
    plain = lidar_clusters(pts, G, INC)
    assert len(with_pts) == len(plain) == 2
    for (cl, c1, r1), (c2, r2, n) in zip(with_pts, plain):
        assert np.allclose(c1, c2) and r1 == r2 and len(cl) == n


def test_lidar_points_include_mount_offset():
    pts = lidar_points([5.0], 0.0, INC, 0.1, 40.0, G)           # 정면 빔 1개
    assert pts[0] == pytest.approx((G.lidar_offset_x + 5.0, G.lidar_offset_y))


# ---------- 중심 ----------

def test_surface_center_from_front_arc():
    r, c = 0.3, np.array([6.0, 0.0])
    lateral = np.linspace(-r, r, 41)
    pts = np.column_stack((c[0] - np.sqrt(r * r - lateral ** 2), c[1] + lateral))
    center, rad = surface_center(pts, np.zeros(2), G)
    assert rad == pytest.approx(r)
    assert np.hypot(*(center - c)) < 0.01


# ---------- 카메라 기하 ----------

def test_level_to_pixel_inverts_pixels_to_level():
    cam = CameraObstacleParams(offset_x=G.cam_offset_x, offset_y=G.cam_offset_y,
                               height=G.cam_height, mount_pitch_deg=G.cam_mount_pitch_deg)
    u, v = np.array([100.0, 320.0, 500.0]), np.array([300.0, 240.0, 400.0])
    d = np.array([2.0, 5.0, 3.5])
    pts = pixels_to_level(u, v, d, K, cam, roll=0.02, pitch=-0.03)
    uu, vv, dd = level_to_pixel(pts, K, G, roll=0.02, pitch=-0.03)
    assert uu == pytest.approx(u) and vv == pytest.approx(v) and dd == pytest.approx(d)
