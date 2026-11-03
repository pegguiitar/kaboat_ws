"""┛ 판 자세 추정 — 실패 시 안전(C) 보강: 판이 없거나 순간 이동이면 pose 를 내지 않는다."""
import math

import pytest

from kaboat_hardware.lidar_sail_pose import estimate_sail_pose
from test_lidar_sail_pose import GEOM, LIDAR, _scan


@pytest.mark.check('C')
def test_no_clusters_or_implausible_jump_returns_none():
    angles, ranges, valid, clusters = _scan(7.0, 2.0, math.radians(30))
    assert estimate_sail_pose([], LIDAR, angles, ranges, valid, GEOM) is None
    jumped = estimate_sail_pose(clusters, LIDAR, angles, ranges, valid, GEOM,
                                previous_pose=(2.0, 2.0, 0.0), max_position_jump=1.0)
    assert jumped is None
