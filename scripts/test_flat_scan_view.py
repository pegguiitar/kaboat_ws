"""flat_scan_view.py 계산 (ROS 없이)."""

import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))

import flat_scan_view as fv  # noqa: E402


def test_points_land_on_floor_in_odom():
    # 라이다 정면(0°) 2 m, 좌측(90°) 1 m, 잘못된 값들
    ranges = [2.0, 1.0, float('inf'), 0.0, float('nan')]
    pts = fv.flatten_scan(ranges, 0.0, math.pi / 2, 0.05, 40.0,
                          pose=(10.0, -3.0, math.pi / 2), lidar_offset=(0.4, 0.0, 0.0), z=0.05)
    assert pts.shape == (2, 3) and pts.dtype == np.float32
    # 배는 북쪽(+y)을 봄: 선체 앞 2.4 m → odom (10, -0.6), 선체 왼쪽 1 m(앞 0.4) → (9, -2.6)
    np.testing.assert_allclose(pts[0], [10.0, -0.6, 0.05], atol=1e-5)
    np.testing.assert_allclose(pts[1], [9.0, -2.6, 0.05], atol=1e-5)


def test_lidar_yaw_offset_rotates_scan():
    pts = fv.flatten_scan([1.0], 0.0, 0.1, 0.0, 10.0, pose=(0.0, 0.0, 0.0),
                          lidar_offset=(0.0, 0.0, math.pi), z=0.0)
    np.testing.assert_allclose(pts[0], [-1.0, 0.0, 0.0], atol=1e-6)


def test_yaw_ignores_roll_pitch():
    h = math.radians(123.1) / 2
    assert abs(math.degrees(fv.yaw_of((0.0, 0.0, math.sin(h), math.cos(h)))) - 123.1) < 1e-9
    # 현장 /odom 값 (피치 약 1°) → 선수각만
    assert abs(math.degrees(fv.yaw_of((-0.00089, 0.0084, 0.88229, 0.47063))) - 123.8) < 0.2


def test_pose_history_nearest_within_tolerance():
    h = fv.PoseHistory(keep_sec=1.0)
    for i in range(30):                     # 25 Hz, 1.2 s
        h.add(100.0 + i * 0.04, (i, 0.0, 0.0))
    assert h.nearest(100.5)[0] in (12, 13)
    assert h.nearest(101.16)[0] == 29
    assert h.nearest(101.6) is None          # 0.3 s 넘게 차이
    assert h.stamps[0] >= 100.16 - 1e-9     # 1 s 넘은 것은 버림
    h.add(100.9, (99, 0.0, 0.0))            # 뒤늦게 온 옛 값은 무시
    assert h.nearest(100.9)[0] != 99
    h.add(50.0, (7, 0.0, 0.0))              # 시각이 크게 되돌아감 → 새로 시작
    assert h.stamps == [50.0] and h.nearest(50.1)[0] == 7
