"""buoy_localize.py 단독 테스트 — ROS 없이 돈다.

합성 장면: 수면(z=0, 파랑) 위에 원기둥 부표를 세우고, 현장 카메라 기하(뒤 0.33 m ·
높이 0.85 m · 10° 숙임)로 정렬 RGB + 뎁스 영상을 렌더한다. 물에 비친 반사상은 수면
아래 허상(z<0)으로 렌더한다. 라이다 빔은 원과의 첫 교점. 알고리즘은 부표 반지름을 모른다.
"""
import math

import numpy as np
import pytest

from kaboat_perception.buoy_localize import (
    BuoyFix, BuoyGeometry, camera_xy, classify_pixels, depth_buoys, effective_K,
    fuse_with_lidar, level_to_pixel, lidar_buoys, lidar_clusters, lidar_points,
    surface_center)
from kaboat_perception.camera_obstacles import pixels_to_level

W, H = 640, 480
K = (380.0, 380.0, 320.0, 240.0)   # 640×480 D455 근사
G = BuoyGeometry()                 # 현장 기본값: 카메라 -0.33 m · 10° 숙임, 라이다 +0.4 m

RED, GREEN, ORANGE = (30, 30, 220), (40, 200, 40), (0, 140, 255)   # BGR
BLACK, YELLOW, MAGENTA, GRAY = (25, 25, 25), (0, 220, 230), (200, 40, 200), (128, 128, 128)
WATER, SKY = (140, 90, 30), (235, 225, 215)       # 물: 채도 높은 파랑 (H≈104)


def _render(buoys, g=G):
    """buoys: [(x, y, r, 높이, bgr, 'real'|'reflection'|'both')] → (bgr H×W×3, depth H×W [m])."""
    us, vs = np.meshgrid(np.arange(W), np.arange(H))
    u, v = us.ravel().astype(float), vs.ravel().astype(float)
    cam = pixels_to_level([K[2]], [K[3]], [0.0], K, g.camera())[0]
    d = pixels_to_level(u, v, np.ones_like(u), K, g.camera()) - cam   # 광학 깊이 1당 이동
    depth = np.where(d[:, 2] < 0.0, -cam[2] / np.minimum(d[:, 2], -1e-9), np.inf)
    img = np.where((d[:, 2] < 0.0)[:, None], WATER, SKY).astype(np.uint8)
    water = np.isfinite(depth)
    for bx, by, r, h, bgr, kind in buoys:
        ox, oy = cam[0] - bx, cam[1] - by
        a = d[:, 0] ** 2 + d[:, 1] ** 2
        b = 2.0 * (d[:, 0] * ox + d[:, 1] * oy)
        disc = b * b - 4.0 * a * (ox * ox + oy * oy - r * r)
        t = (-b - np.sqrt(np.maximum(disc, 0.0))) / (2.0 * a)
        z = cam[2] + t * d[:, 2]
        if kind in ('real', 'both'):
            hit = (disc >= 0.0) & (t > 0.0) & (z >= 0.0) & (z <= h) & (t < depth)
            depth[hit], img[hit] = t[hit], bgr
            water &= ~hit
        if kind in ('reflection', 'both'):          # 수면 아래 허상 — 물 픽셀만 덮는다
            hit = (disc >= 0.0) & (t > 0.0) & (z < 0.0) & (z >= -h) & water
            depth[hit], img[hit] = t[hit], bgr
    depth[~np.isfinite(depth)] = 0.0                # 하늘: 뎁스 없음
    return img.reshape(H, W, 3), depth.reshape(H, W).astype(np.float32)


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


def _dist(f, xy):
    return math.hypot(f.x - xy[0], f.y - xy[1])


def _depth_fixes(buoys):
    img, dep = _render(buoys)
    obj, label = classify_pixels(img, G)
    return depth_buoys(obj, label, dep, K, G)


# ---------- 뎁스 경로 (camera_obstacles 방식) ----------

@pytest.mark.parametrize('r', [0.2, 0.35])
def test_depth_buoy_center_without_known_radius(r):
    fixes = _depth_fixes([(4.0, 0.5, r, 0.6, RED, 'real')])
    assert len(fixes) == 1
    f = fixes[0]
    assert f.color == 'red' and f.source == 'depth'
    assert _dist(f, (4.0, 0.5)) < 0.05
    assert f.radius == pytest.approx(r, abs=0.05)


def test_red_and_green_are_told_apart():
    fixes = sorted(_depth_fixes([(3.5, 1.0, 0.25, 0.6, GREEN, 'real'),
                                 (3.5, -1.0, 0.25, 0.6, RED, 'real')]), key=lambda f: f.y)
    assert [f.color for f in fixes] == ['red', 'green']
    assert _dist(fixes[0], (3.5, -1.0)) < 0.05 and _dist(fixes[1], (3.5, 1.0)) < 0.05


def test_water_reflection_is_removed_by_height():
    fixes = _depth_fixes([(4.0, 0.0, 0.25, 0.6, RED, 'both')])
    assert len(fixes) == 1 and _dist(fixes[0], (4.0, 0.0)) < 0.05
    assert _depth_fixes([(4.0, 0.0, 0.25, 0.6, RED, 'reflection')]) == []   # 반사상만


@pytest.mark.parametrize('bgr, color', [(RED, 'red'), (ORANGE, 'red'), (GREEN, 'green'),
                                        (BLACK, 'black')])
def test_each_buoy_color_has_its_own_label(bgr, color):
    fixes = _depth_fixes([(4.0, 0.0, 0.25, 0.6, bgr, 'real')])
    assert [f.color for f in fixes] == [color]
    assert _dist(fixes[0], (4.0, 0.0)) < 0.05


@pytest.mark.parametrize('bgr, color', [((20, 90, 230), 'red'),       # 주황빛 빨강 H 10
                                        ((20, 200, 180), 'green'),    # 누르스름한 초록 H 33
                                        ((130, 175, 125), 'green')])  # 옅은 초록 S 73
def test_loose_ranges_take_shifted_reds_and_greens(bgr, color):
    # 햇빛·AWB 로 틀어진 색도 놓치지 않게 범위를 넉넉히
    fixes = _depth_fixes([(4.0, 0.0, 0.25, 0.6, bgr, 'real')])
    assert [f.color for f in fixes] == [color]


@pytest.mark.parametrize('bgr', [YELLOW, MAGENTA, WATER, GRAY])
def test_other_colors_and_gray_are_not_buoys(bgr):
    assert _depth_fixes([(4.0, 0.0, 0.25, 0.6, bgr, 'real')]) == []   # WATER = 파란 물체


def test_water_and_sky_drop_out_by_height_and_range_not_color():
    # 물은 채도 높은 파랑이지만 색으로 거르지 않는다 — 수면 높이(z)·6 m clamp 로 빠진다
    img, dep = _render([])
    obj, label = classify_pixels(img, G)
    assert obj[-1, :].all()                         # 화면 아래 물 픽셀도 물체 픽셀 후보
    assert depth_buoys(obj, label, dep, K, G) == []


def test_depth_beyond_6m_is_not_used():
    assert _depth_fixes([(8.0, 0.0, 0.3, 0.6, GREEN, 'real')]) == []


def test_fallen_buoy_radius_is_capped():
    # 옆으로 누운 부표 — 가로로 긴 실루엣 (근사: 넓은 원기둥 2개를 붙임)
    fixes = _depth_fixes([(4.0, 0.3, 0.35, 0.3, GREEN, 'real'),
                          (4.0, -0.3, 0.35, 0.3, GREEN, 'real')])
    assert len(fixes) == 1 and fixes[0].radius <= G.max_radius


# ---------- 라이다 ----------

def test_lidar_clusters_skip_wide_wall():
    pts = _scan([(5.0, 0.0, 0.25), (12.0, 0.0, 4.0)])
    clusters = lidar_clusters(pts, G, INC)
    assert len(clusters) == 1
    assert math.hypot(*(clusters[0][0] - (5.0, 0.0))) < 0.05


def test_fuse_pulls_depth_fix_toward_lidar():
    depth_fix = BuoyFix('red', 5.15, 0.5, 0.25, 'depth', 300)   # 뎁스 15 cm 오차
    clusters = lidar_clusters(_scan([(5.0, 0.5, 0.25)]), G, INC)
    cam = camera_xy(K, G)
    f = fuse_with_lidar(depth_fix, clusters, cam, G)
    assert f.source == 'fused' and f.color == 'red'
    assert _dist(f, (5.0, 0.5)) < 0.05
    far = lidar_clusters(_scan([(5.0, 2.0, 0.25)]), G, INC)       # 다른 물체 — 융합 안 함
    assert fuse_with_lidar(depth_fix, far, cam, G) is depth_fix


def _lidar_fixes(buoys, skip=()):
    """같은 장면을 카메라·라이다 둘 다로 — 라이다 경로 결과."""
    img, dep = _render(buoys)
    obj, label = classify_pixels(img, G)
    clusters = lidar_clusters(_scan([(x, y, r) for x, y, r, *_ in buoys]), G, INC)
    return lidar_buoys(clusters, obj, label, K, G, skip=skip, depth_m=dep)


def test_far_buoys_get_color_from_projected_window():
    fixes = sorted(_lidar_fixes([(12.0, 2.0, 0.25, 0.6, GREEN, 'both'),
                                 (10.0, -1.5, 0.25, 0.6, RED, 'real'),
                                 (11.0, 4.5, 0.25, 0.6, BLACK, 'real'),
                                 (9.0, -4.0, 0.25, 0.6, ORANGE, 'real')]), key=lambda f: f.y)
    assert [(f.color, f.source) for f in fixes] == [   # 주황(9, -4)은 빨강으로
        ('red', 'lidar'), ('red', 'lidar'), ('green', 'lidar'), ('black', 'lidar')]
    assert _dist(fixes[1], (10.0, -1.5)) < 0.08     # 0.5° 빔 간격 ≈ 9~10 cm
    assert _dist(fixes[2], (12.0, 2.0)) < 0.08


def test_lidar_object_without_buoy_color_is_not_a_buoy():
    assert _lidar_fixes([(9.0, 0.0, 0.15, 1.2, GRAY, 'real')]) == []    # 회색 기둥
    assert _lidar_fixes([(9.0, 0.0, 0.25, 0.6, YELLOW, 'real')]) == []  # 노랑 (기타)


def test_far_buoy_behind_near_buoy_keeps_its_own_color():
    # 카메라에서 9 m 부표가 4 m 부표 뒤에 반쯤 가림 — 색 창의 앞 부표 픽셀은 뎁스로 빼고 판정
    near = BuoyFix('orange', 4.0, -1.2, 0.25, 'fused', 300)
    scene = [(4.0, -1.2, 0.25, 0.6, ORANGE, 'real'), (9.0, -2.4, 0.25, 0.6, RED, 'real')]
    assert [f.color for f in _lidar_fixes(scene, skip=[near])] == ['red']
    scene[1] = (9.0, -2.4, 0.25, 0.6, YELLOW, 'real')       # 뒤가 노랑(기타)이면 버림
    assert _lidar_fixes(scene, skip=[near]) == []


def test_lidar_path_skips_buoys_found_by_depth():
    near = BuoyFix('red', 4.0, 0.0, 0.25, 'fused', 300)
    assert _lidar_fixes([(4.0, 0.0, 0.25, 0.6, RED, 'real')], skip=[near]) == []


def test_surface_center_from_front_arc():
    r, c = 0.3, np.array([6.0, 0.0])
    lateral = np.linspace(-r, r, 41)
    pts = np.column_stack((c[0] - np.sqrt(r * r - lateral ** 2), c[1] + lateral))
    center, rad = surface_center(pts, np.zeros(2), G)
    assert rad == pytest.approx(r)
    assert np.hypot(*(center - c)) < 0.01


# ---------- 카메라 기하 ----------

def test_effective_K_rejects_off_center_sim_K():
    # sim: width 1280 인데 K 는 320px 기준 → HFOV 로 대체
    f, _, cx, cy = effective_K((277.0, 277.0, 160.0, 120.0), 1280, 960, math.radians(87.0))
    assert cx == 640.0 and cy == 480.0
    assert f == pytest.approx(640.0 / math.tan(math.radians(43.5)))
    assert effective_K(K, 640, 480, 1.5) == K


def test_level_to_pixel_inverts_pixels_to_level():
    u, v = np.array([100.0, 320.0, 500.0]), np.array([300.0, 240.0, 400.0])
    d = np.array([2.0, 5.0, 3.5])
    pts = pixels_to_level(u, v, d, K, G.camera(), roll=0.02, pitch=-0.03)
    uu, vv, dd = level_to_pixel(pts, K, G, roll=0.02, pitch=-0.03)
    assert uu == pytest.approx(u) and vv == pytest.approx(v) and dd == pytest.approx(d)
