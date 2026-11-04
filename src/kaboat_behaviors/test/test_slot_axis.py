import math

import numpy as np

from kaboat_behaviors.slot_axis import (
    WallFitParams, fit_back_wall, locate_slot, scan_points)

DEG = math.pi / 180.0


def angle_between(a, b):
    a, b = np.asarray(a, float), np.asarray(b, float)
    return abs(math.atan2(a[0] * b[1] - a[1] * b[0], float(a @ b)))


def wall_frame(delta_deg, marker=(5.0, 0.3)):
    """뒷벽이 선체 정면에서 delta 만큼 돌아간 장면. n: 벽→선체 법선, t: 벽 방향."""
    d = delta_deg * DEG
    n = np.array([-math.cos(d), -math.sin(d)])
    t = np.array([-n[1], n[0]])
    return np.array(marker), n, t


def build_scene(delta_deg, noise=0.01, span=4.5, partitions=True, seed=1,
                clutter=True, marker=(5.0, 0.3)):
    rng = np.random.default_rng(seed)
    m, n, t = wall_frame(delta_deg, marker)
    pts = [m + s * t for s in np.arange(-span, span, 0.02)]       # 이웃까지 이어진 뒷벽
    if partitions:                                                # 슬롯 칸막이(±1.5m)
        for s0 in (-1.5, 1.5):
            pts += [m + s0 * t + k * n for k in np.arange(0.0, 4.0, 0.02)]
    if clutter:
        pts += [np.array([x, y]) for x, y in rng.uniform([1, -6], [9, 6], (60, 2))]
    pts = np.array(pts) + rng.normal(0, noise, (len(pts), 2))
    return pts, m, n


def test_recovers_axis_without_any_prior_normal():
    for delta in (0.0, 8.0, -12.0, 30.0):
        pts, m, n = build_scene(delta)
        wall = fit_back_wall(pts, m)
        assert wall is not None, delta
        assert angle_between(wall.outward, n) < 1.0 * DEG
        assert wall.span >= 2.0


def test_marker_position_error_from_depth_is_tolerated():
    pts, m, n = build_scene(5.0)
    t = np.array([-n[1], n[0]])
    for shift in (0.15 * n + 0.2 * t, -0.2 * n - 0.3 * t):    # 거리·좌우 오차
        wall = fit_back_wall(pts, m + shift)
        assert wall is not None and angle_between(wall.outward, n) < 1.5 * DEG


def test_outward_normal_points_toward_the_boat():
    pts, m, n = build_scene(0.0)
    wall = fit_back_wall(pts, m)
    assert wall is not None and np.dot(wall.outward, -m) > 0


def test_noisy_scan_stays_within_two_degrees():
    pts, m, n = build_scene(6.0, noise=0.02, seed=7)
    wall = fit_back_wall(pts, m)
    assert wall is not None and angle_between(wall.outward, n) < 2.0 * DEG


def test_too_few_points_or_short_wall_returns_none():
    pts, m, n = build_scene(0.0, partitions=False, clutter=False)
    assert fit_back_wall(pts[:10], m) is None
    assert fit_back_wall(np.empty((0, 2)), m) is None
    t = np.array([-n[1], n[0]])
    short = np.array([m + s * t for s in np.arange(-0.3, 0.3, 0.02)])
    assert fit_back_wall(short, m) is None                    # 길이 0.6m < 1.2m


def test_line_not_near_the_marker_is_rejected():
    pts, m, n = build_scene(0.0)
    assert fit_back_wall(pts, m + 1.0 * n) is None            # 표식이 벽에서 1m 앞으로 잘못됨
    assert fit_back_wall(pts, np.array([1.0, 5.0])) is None   # 엉뚱한 위치


def test_scattered_points_are_rejected():
    rng = np.random.default_rng(3)
    scatter = np.column_stack((rng.uniform(4.5, 5.5, 300), rng.uniform(-1.3, 1.3, 300)))
    assert fit_back_wall(scatter, (5.0, 0.0)) is None


def test_wall_seen_edge_on_is_rejected():
    # 벽이 시선 방향과 거의 나란함(법선과 시선이 90° 가까움): 뒷벽 앞면이 아니다.
    line = np.array([[4.0 + s, 0.02 * s] for s in np.arange(0.0, 2.6, 0.02)])
    assert fit_back_wall(line, (5.3, 0.0)) is None


def test_scan_points_follow_occupancy_grid_angle_convention():
    pts = scan_points([2.0, 3.0, float('inf'), 0.0, 60.0],
                      angle_min=-math.pi / 2, angle_increment=math.pi / 4,
                      range_min=0.01, range_max=50.0)
    assert pts.shape == (2, 2)                       # inf, 범위 밖 빔 제외
    np.testing.assert_allclose(pts[0], [0.0, -2.0], atol=1e-9)    # -90°: 우측
    np.testing.assert_allclose(pts[1], [3 * math.cos(-math.pi / 4),
                                         3 * math.sin(-math.pi / 4)], atol=1e-9)


def test_locate_slot_puts_mouth_on_the_lidar_axis_from_the_wall():
    pts, m, n = build_scene(9.0)
    m_depth = m + 0.12 * n                                    # depth 거리 오차 12cm
    slot = locate_slot(pts, m_depth, 2.5)
    assert slot is not None
    base, mouth = np.array(slot.marker_xy), np.array(slot.mouth_xy)
    assert np.linalg.norm(base - m) < 0.05                    # 표식이 벽 위로 투영됨
    assert np.linalg.norm(mouth - (m + 2.5 * n)) < 0.08
    assert abs(np.linalg.norm(mouth - base) - 2.5) < 1e-9
    assert angle_between(mouth - base, n) < 1.0 * DEG


def test_locate_slot_rejects_bad_wall_to_mouth_or_missing_wall():
    pts, m, _ = build_scene(0.0)
    for bad in (0.0, -1.0, float('nan'), float('inf')):
        assert locate_slot(pts, m, bad) is None
    assert locate_slot(np.empty((0, 2)), m, 2.5) is None


def test_default_params_are_sane():
    p = WallFitParams()
    assert p.window_radius < 1.5 and p.min_span <= 2 * p.window_radius
