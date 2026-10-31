import math

import numpy as np

from kaboat_hardware.lidar_sail_pose import (
    SailGeometry, closest_on_model, estimate_sail_pose, raycast,
    raycast_with_index,
)
from kaboat_hardware.pose_velocity import normalize_angle

GEOM = SailGeometry(left=0.45, right=0.30, sail=0.45)
LIDAR = np.array([10.0, 2.5])


def _scan(x, y, yaw, noise=0.0, seed=0):
    """판 모델에 A3M1 과 같은 0.5° 빔을 쏜 합성 스캔.

    판 뒤는 12 m 벽, 실측처럼 빔 3개 중 1개는 무효다.
    """
    angles = np.radians(np.arange(-180.0, 180.0, 0.5)) + math.pi
    panel = raycast(LIDAR, angles, GEOM.segments(x, y, yaw))
    rng = np.random.default_rng(seed)
    ranges = np.where(np.isfinite(panel), panel, 12.0)
    ranges = ranges + noise * rng.standard_normal(len(ranges))
    valid = np.arange(len(angles)) % 3 != 2
    pts = LIDAR + ranges[:, None] * np.column_stack(
        (np.cos(angles), np.sin(angles)))
    on_panel = valid & np.isfinite(panel)
    return angles, ranges, valid, [pts[on_panel]]


def _estimate(x, y, yaw, **kwargs):
    angles, ranges, valid, clusters = _scan(x, y, yaw, **kwargs)
    return estimate_sail_pose(clusters, LIDAR, angles, ranges, valid, GEOM)


def test_recovers_full_yaw_when_both_panels_visible():
    for deg in (20, 60, 135, -120):
        yaw = math.radians(deg)
        result = _estimate(7.0, 2.0, yaw)
        assert result is not None, deg
        assert math.hypot(result.x - 7.0, result.y - 2.0) < 0.03, deg
        assert abs(normalize_angle(result.yaw - yaw)) < math.radians(3), deg


def test_tracks_every_heading_at_far_range_with_previous_pose():
    for deg in range(0, 360, 10):
        yaw = math.radians(deg)
        angles, ranges, valid, clusters = _scan(2.0, 2.5, yaw, noise=0.008)
        result = estimate_sail_pose(
            clusters, LIDAR, angles, ranges, valid, GEOM,
            previous_pose=(2.0, 2.5, yaw + math.radians(5)),
            max_position_jump=0.75, reference_yaw=yaw + math.radians(5))
        assert result is not None, deg
        assert math.hypot(result.x - 2.0, result.y - 2.5) < 0.08, deg
        assert abs(normalize_angle(result.yaw - yaw)) < math.radians(8), deg


def test_default_heading_resolves_ambiguity_without_previous_pose():
    # 앞뒤가 모호한 각도에서도 기본 방향이 실제와 같은 쪽이면 맞게 고른다.
    for deg in range(0, 360, 10):
        yaw = math.radians(deg)
        angles, ranges, valid, clusters = _scan(7.0, 2.5, yaw, noise=0.008)
        result = estimate_sail_pose(
            clusters, LIDAR, angles, ranges, valid, GEOM,
            default_yaw=yaw + math.radians(30))
        assert result is not None, deg
        assert abs(normalize_angle(result.yaw - yaw)) < math.radians(8), deg


def test_single_line_follows_reference_yaw_instead_of_flipping():
    # 확실한 L자가 아닌 장면에서 기존 yaw 와 90°/180° 다른 해석은 쓰지 않는다.
    for deg in range(0, 360, 10):
        yaw = math.radians(deg)
        angles, ranges, valid, clusters = _scan(2.0, 2.5, yaw, noise=0.008)
        result = estimate_sail_pose(
            clusters, LIDAR, angles, ranges, valid, GEOM,
            reference_yaw=yaw + math.radians(10))
        if result is not None and not result.yaw_unique:
            assert abs(normalize_angle(result.yaw - yaw)) < math.radians(40), deg


def test_confirmed_l_overrides_wrong_reference_yaw():
    # 두 판이 모두 보이면 기존 yaw 가 반대여도 스캔 쪽으로 재정렬한다.
    yaw = math.radians(135)
    angles, ranges, valid, clusters = _scan(7.0, 2.5, yaw)
    result = estimate_sail_pose(
        clusters, LIDAR, angles, ranges, valid, GEOM,
        reference_yaw=yaw + math.pi)
    assert result is not None and result.yaw_unique
    assert abs(normalize_angle(result.yaw - yaw)) < math.radians(3)


def _arm_points(x, y, yaw, keep_other=0, noise=0.0):
    """한 판의 점은 모두 두고 다른 판 점은 keep_other 개만 남긴 스캔."""
    angles, ranges, valid, _ = _scan(x, y, yaw, noise=noise)
    _, seg = raycast_with_index(LIDAR, angles, GEOM.segments(x, y, yaw))
    on = valid & (seg >= 0)
    counts = [np.count_nonzero(on & (seg == i)) for i in (0, 1)]
    main = int(np.argmax(counts))
    other = np.where(on & (seg == 1 - main))[0]
    drop = other[keep_other:]
    valid = valid.copy()
    valid[drop] = False  # 다른 판 점이 안 잡힌 것처럼 무효 처리
    on = valid & (seg >= 0)
    pts = LIDAR + ranges[on, None] * np.column_stack(
        (np.cos(angles[on]), np.sin(angles[on])))
    return angles, ranges, valid, [pts]


def test_single_panel_puts_corner_at_line_end_and_covers_points():
    # 한 판만 보여도 좌표가 나오고, 모든 점이 L 모델 위(꺾이는 지점 바깥 아님)에 있다.
    for deg in (0, 45, 90, 150, 210, 300):
        yaw = math.radians(deg)
        angles, ranges, valid, clusters = _arm_points(6.0, 2.5, yaw,
                                                      noise=0.008)
        result = estimate_sail_pose(
            clusters, LIDAR, angles, ranges, valid, GEOM,
            reference_yaw=yaw + math.radians(10))
        assert result is not None, deg
        _, d, _, _ = closest_on_model(clusters[0], result.segments)
        assert np.max(d) < 0.05, deg
        assert abs(normalize_angle(result.yaw - yaw)) < math.radians(8), deg


def test_one_or_two_points_on_other_panel_pick_the_right_l():
    # 기존 yaw 없이도 다른 판의 점 1~2개로 올바른 해석을 고른다.
    for keep in (1, 2):
        for deg in (30, 120, 200, 320):
            yaw = math.radians(deg)
            angles, ranges, valid, clusters = _arm_points(
                6.0, 2.5, yaw, keep_other=keep, noise=0.005)
            result = estimate_sail_pose(
                clusters, LIDAR, angles, ranges, valid, GEOM)
            assert result is not None, (keep, deg)
            assert math.hypot(result.x - 6.0, result.y - 2.5) < 0.06, (keep, deg)
            assert abs(normalize_angle(result.yaw - yaw)) < math.radians(6), (keep, deg)


def test_rejects_unrelated_short_object():
    angles = np.radians(np.arange(-180.0, 180.0, 0.5)) + math.pi
    pts = np.array([[6.0, 2.0 + 0.02 * i] for i in range(5)])
    ranges = np.zeros(len(angles))
    valid = np.zeros(len(angles), dtype=bool)
    assert estimate_sail_pose([pts], LIDAR, angles, ranges, valid, GEOM) is None
