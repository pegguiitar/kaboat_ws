"""지도 unknown 칸이어도 지금 스캔 빔이 지나 본 점은 관측으로 (게이트 두 부표 사이 빈 물).

2026-11-12 4번 시험 녹화 재생: 지도는 맞은 빔만 free 로 칠해서 뒤에 맞을 게 없는 게이트 사이
물이 unknown 으로 남고, 경로가 그 사이를 지나다 PATH_UNOBSERVED 로 쌍을 버렸다
(15:31:27·15:31:56·15:32:17). 앞 ±60° 빔 676 개 중 565 개가 무반사(inf).
"""

import math
from types import SimpleNamespace

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String

from kaboat_behaviors.gate_corridor import (
    GateParams, ScanRays, points_observed, points_observed_or_seen, seen_through)
from test_gate_follower import _ready_node

P = GateParams()


def _rays(ranges_by_deg=None, yaw=0.0, n=720):
    ranges = np.full(n, np.inf)
    for deg, r in (ranges_by_deg or {}).items():
        i = int(round((math.radians(deg) + math.pi) / (2 * math.pi / n))) % n
        ranges[i - 2:i + 3] = r
    return ScanRays((0.0, 0.0), yaw, -math.pi, 2 * math.pi / n, ranges, 0.1, 25.0)


def test_no_return_ahead_counts_as_seen_but_not_behind_or_far():
    pts = [(3.0, 0.0), (-3.0, 0.0), (8.0, 0.0), (0.05, 0.0), (2.0, 2.0)]
    assert seen_through(_rays(), pts, P).tolist() == [True, False, False, False, True]
    # 선수를 북쪽으로 돌리면 (3, 0) 은 오른쪽 90° — 110° 안이라 그대로, (-3, 0) 은 왼 90°
    assert seen_through(_rays(yaw=math.pi / 2), pts, P)[:2].tolist() == [True, True]


def test_a_return_before_the_point_blocks_it_and_beyond_does_not():
    rays = _rays({0: 2.0, 45: 5.0})
    pts = [(3.0, 0.0), (1.5, 0.0), (2.5, 2.5)]       # 2 m 에서 맞음 → 3 m 는 안 보임
    assert seen_through(rays, pts, P).tolist() == [False, True, True]


def _unknown_grid(known_radius=1.0, posts=()):
    res, size, origin = 0.2, 100, (-5.0, -10.0)
    data = np.full((size, size), -1, dtype=np.int8)
    yy, xx = np.mgrid[0:size, 0:size]
    cx, cy = origin[0] + (xx + 0.5) * res, origin[1] + (yy + 0.5) * res
    data[np.hypot(cx, cy) <= known_radius] = 0
    for x, y in posts:
        ix, iy = int((x - origin[0]) / res), int((y - origin[1]) / res)
        data[iy:iy + 2, ix:ix + 2] = 100
    info = SimpleNamespace(resolution=res, width=size, height=size,
                           origin=SimpleNamespace(position=SimpleNamespace(x=origin[0], y=origin[1])))
    return SimpleNamespace(info=info, data=data.ravel().tolist())


def test_unknown_cells_pass_only_when_the_scan_saw_through_them():
    grid = _unknown_grid()
    pts = [(0.5, 0.0), (3.0, 0.0), (4.0, 0.5)]
    assert not points_observed(grid, pts)
    assert points_observed_or_seen(grid, pts, _rays(), P)
    assert not points_observed_or_seen(grid, pts, _rays({0: 2.0}), P)
    assert not points_observed_or_seen(grid, pts, None, P)


def test_gate_with_open_water_behind_is_followed_not_dropped():
    """지도엔 배 주변과 부표만 있고 게이트 사이·뒤는 unknown, 스캔은 부표 말고 무반사."""
    rclpy.init()
    node = None
    try:
        posts = [(3.0, -1.5), (3.0, 1.5)]
        node = _ready_node(posts)
        fake = _unknown_grid(1.5, posts)
        msg = OccupancyGrid()
        msg.header.frame_id = 'odom'
        msg.info.resolution = fake.info.resolution
        msg.info.width = msg.info.height = fake.info.width
        msg.info.origin.position.x = fake.info.origin.position.x
        msg.info.origin.position.y = fake.info.origin.position.y
        msg.info.origin.orientation.w = 1.0
        msg.data = fake.data
        node._on_grid(msg)
        node.compute_cmd()
        assert node._status_text == 'FOLLOW_GATE_1/7'
        node._scan_rays = None                      # 빔 정보가 없으면 예전처럼 막힘
        node.compute_cmd()
        assert node._status_text == 'PATH_UNOBSERVED'
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()
