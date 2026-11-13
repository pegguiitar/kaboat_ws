"""buoy_localize.py 단독 테스트 — 부표용 라이다 덩어리·카메라 투영 기하 (ROS 없이 돈다)."""
import math

import numpy as np
import pytest

from kaboat_perception.buoy_localize import BuoyGeometry, level_to_pixel, lidar_blobs, lidar_points
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


# ---------- 라이다 덩어리 ----------

def test_blobs_skip_wide_wall():
    pts = _scan([(5.0, 0.0, 0.25), (12.0, 0.0, 4.0)])
    blobs = lidar_blobs(pts, G)
    assert len(blobs) == 1
    _pts, c, half = blobs[0]
    assert math.hypot(*(c - (5.0, 0.0))) < 0.3          # 중심 = 앞면 점 평균 (부표 반지름 안)
    assert half == pytest.approx(0.25, abs=0.03)


def test_two_buoys_two_blobs_regardless_of_point_order():
    pts = _scan([(5.0, 1.0, 0.25), (8.0, -2.0, 0.3)])
    rng = np.random.default_rng(0)
    a = sorted(round(float(c[0]), 3) for _p, c, _h in lidar_blobs(pts, G))
    b = sorted(round(float(c[0]), 3) for _p, c, _h in lidar_blobs(rng.permutation(pts), G))
    assert len(a) == 2 and a == b


def test_missing_beam_in_middle_still_one_blob():
    pts = _scan([(6.0, 0.0, 0.3)])
    pts = np.delete(pts, len(pts) // 2, axis=0)          # 가운데 빔 하나 빠짐
    assert len(lidar_blobs(pts, G)) == 1


def test_adjacent_pair_with_gap_is_two_blobs():
    # 탐색 bag 붙은 쌍 — 중심 1.37 m, 표면 사이 0.5 m 이상 → 덩어리 둘
    pts = _scan([(6.0, 0.0, 0.3), (6.0, 1.37, 0.3)])
    assert len(lidar_blobs(pts, G)) == 2


def test_sparse_and_narrow_hits_dropped():
    pts = np.array([[3.0, 0.0], [3.01, 0.0], [7.0, 2.0]])   # 빔 2개 물보라 + 점 1개
    assert lidar_blobs(pts, G) == []


def test_lidar_points_include_mount_offset():
    pts = lidar_points([5.0], 0.0, INC, 0.1, 40.0, G)           # 정면 빔 1개
    assert pts[0] == pytest.approx((G.lidar_offset_x + 5.0, G.lidar_offset_y))


# ---------- 카메라 기하 ----------

def test_level_to_pixel_inverts_pixels_to_level():
    cam = CameraObstacleParams(offset_x=G.cam_offset_x, offset_y=G.cam_offset_y,
                               height=G.cam_height, mount_pitch_deg=G.cam_mount_pitch_deg)
    u, v = np.array([100.0, 320.0, 500.0]), np.array([300.0, 240.0, 400.0])
    d = np.array([2.0, 5.0, 3.5])
    pts = pixels_to_level(u, v, d, K, cam, roll=0.02, pitch=-0.03)
    uu, vv, dd = level_to_pixel(pts, K, G, roll=0.02, pitch=-0.03)
    assert uu == pytest.approx(u) and vv == pytest.approx(v) and dd == pytest.approx(d)
