"""camera_obstacles: 색 판별, 기울기 보정, 반사상·흰색·근거리 제외, 시야 판정."""

import math

import numpy as np
import pytest
from sensor_msgs.msg import Image

from kaboat_perception.camera_obstacles import (
    CameraObstacleParams, color_image_rgb, color_masks, depth_image_m, detect_points,
    level_rotation, pixels_to_level, visible_mask)

K = (380.0, 380.0, 320.0, 240.0)       # fx, fy, cx, cy (640×480)
P = CameraObstacleParams(height=1.0)   # 수면 위 1.0 m, 선체 중심 뒤 0.33 m


def _px(rgb):
    return np.array([[rgb]], dtype=np.uint8)


def test_color_classes():
    on = CameraObstacleParams(white=True)
    assert color_masks(_px((255, 100, 0)), P)['orange'][0, 0]       # 주황 붐
    assert color_masks(_px((220, 30, 30)), P)['orange'][0, 0]       # 빨간 부표
    assert not color_masks(_px((230, 200, 40)), P)['orange'][0, 0]  # 노란 경계 부이
    assert not color_masks(_px((60, 90, 120)), P)['orange'][0, 0]   # 물
    assert 'white' not in color_masks(_px((240, 240, 240)), P)      # 흰색 기본 끔
    assert color_masks(_px((240, 240, 240)), on)['white'][0, 0]
    assert not color_masks(_px((150, 160, 175)), on)['white'][0, 0]  # 회색 하늘·수면


def _project(world, p, roll=0.0, pitch=0.0):
    """수평 프레임 점 → 기울어진 카메라의 (u, v, depth). pixels_to_level 의 역."""
    level = level_rotation(roll, pitch)
    t = level @ np.array([p.offset_x, p.offset_y, p.height])
    a = math.radians(p.mount_pitch_deg)
    mount = np.array([[math.cos(a), 0, math.sin(a)], [0, 1, 0], [-math.sin(a), 0, math.cos(a)]])
    body = (level @ mount).T @ (np.asarray(world, float) - t)
    xo, yo, zo = -body[1], -body[2], body[0]
    return K[0] * xo / zo + K[2], K[1] * yo / zo + K[3], zo


def test_back_projection_level_and_tilted():
    # 수평: 영상 가운데 행은 카메라 높이, 수면 점은 z 0
    pts = pixels_to_level([320], [240], [3.0], K, P)
    np.testing.assert_allclose(pts[0], [P.offset_x + 3.0, 0.0, 1.0], atol=1e-9)
    water = (4.67, 0.5, 0.0)
    u, v, d = _project(water, P)
    np.testing.assert_allclose(pixels_to_level([u], [v], [d], K, P)[0], water, atol=1e-9)

    # 선수가 3° 숙고 1.5° 기운 배 + 장착 숙임 5°: IMU 로 보정하면 수면이 z 0
    tilted = CameraObstacleParams(height=1.0, mount_pitch_deg=5.0)
    roll, pitch = math.radians(1.5), math.radians(3.0)
    u, v, d = _project(water, tilted, roll, pitch)
    fixed = pixels_to_level([u], [v], [d], K, tilted, roll, pitch)[0]
    np.testing.assert_allclose(fixed, water, atol=1e-9)
    # 보정 안 하면 수면이 20 cm 넘게 떠서 0.1 m 높이 기준이 무너진다
    raw = pixels_to_level([u], [v], [d], K, tilted, 0.0, 0.0)[0]
    assert abs(raw[2]) > 0.2


def _scene(white=False):
    rgb = np.zeros((480, 640, 3), np.uint8)
    rgb[:] = (60, 90, 120)                         # 물
    depth = np.zeros((480, 640), np.float32)       # 0 = 뎁스 없음
    def paint(world_top, world_bottom, color, half_width=None):
        u0, v_top, d_top = _project(world_top, P)
        _, v_bot, d_bot = _project(world_bottom, P)
        rows = slice(int(math.ceil(v_top)), int(math.floor(v_bot)))
        cols = (slice(0, 640) if half_width is None
                else slice(int(round(u0)) - half_width, int(round(u0)) + half_width))
        rgb[rows, cols] = color
        depth[rows, cols] = (d_top + d_bot) / 2
    paint((3.67, 0.0, 0.15), (3.67, 0.0, 0.03), (255, 90, 0), 20)   # 붐 (0.03~0.15 m)
    paint((3.67, 0.0, -0.03), (3.67, 0.0, -0.15), (255, 90, 0), 20)  # 물에 비친 붐
    paint((2.67, 1.0, 0.30), (2.67, 1.0, 0.12), (245, 245, 245), 20)  # 흰 부표
    paint((0.37, 0.0, 0.5), (0.37, 0.0, 0.3), (255, 90, 0))        # 0.7 m — 배 자체
    return rgb, depth


def test_detect_keeps_boom_drops_reflection_white_and_too_close():
    rgb, depth = _scene()
    pts, labels = detect_points(rgb, depth, K, P)
    assert len(pts) > 20 and set(labels) == {'orange'}
    assert np.all(pts[:, 2] >= 0.02)                              # 반사상(음수 높이) 없음
    np.testing.assert_allclose(np.median(pts[:, 0]), 3.67, atol=0.05)
    assert np.all(np.hypot(pts[:, 0] - P.offset_x, pts[:, 1]) >= P.range_min)

    with_white = CameraObstacleParams(height=1.0, white=True)
    pts, labels = detect_points(rgb, depth, K, with_white)
    assert 'white' in set(labels)
    np.testing.assert_allclose(np.median(pts[labels == 'white', 1]), 1.0, atol=0.1)

    with pytest.raises(ValueError, match='differ'):
        detect_points(rgb, depth[:-1], K, P)


def test_visible_mask_fov_and_range():
    xs = np.array([3.0, -3.0, 9.0, 3.0, 0.2])
    ys = np.array([0.0, 0.0, 0.0, 3.0, 0.0])
    seen = visible_mask(xs, ys, 0.0, 0.0, 0.0, K, 640, P)
    assert seen.tolist() == [True, False, False, False, False]   # 앞 / 뒤 / 너무 멂 / 시야 밖 / 너무 가까움
    turned = visible_mask(xs, ys, 0.0, 0.0, math.pi, K, 640, P)
    assert turned[1] and not turned[0]


def test_image_decoding():
    color = Image(height=1, width=2, encoding='bgr8', step=6, data=bytes([1, 2, 3, 4, 5, 6]))
    np.testing.assert_array_equal(color_image_rgb(color)[0], [[3, 2, 1], [6, 5, 4]])
    raw = np.array([[1500, 0]], dtype='<u2')
    depth = Image(height=1, width=2, encoding='16UC1', step=4, data=raw.tobytes())
    np.testing.assert_allclose(depth_image_m(depth), [[1.5, 0.0]])
