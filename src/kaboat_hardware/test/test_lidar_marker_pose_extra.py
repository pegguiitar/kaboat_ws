"""마커 자세 추정 — 경계·파라미터(B) 보강: 라이다 설치 방향에 따른 좌표 변환."""
import math

import numpy as np
import pytest

from kaboat_hardware.lidar_marker_pose import laser_points_to_pool


@pytest.mark.check('B')
def test_lidar_mount_yaw_rotates_points_into_pool():
    pts = laser_points_to_pool([2.0, 1.0], [0.0, math.pi / 2], 10.0, 2.5, math.pi)
    # 라이다가 -x 를 보고 있으면 전방 2 m 는 x = 8, 좌측 1 m 는 y = 1.5 이다.
    np.testing.assert_allclose(pts, [[8.0, 2.5], [10.0, 1.5]], atol=1e-9)
