"""가상 벽: 위도·경도 → 원점 기준 x·y (현장 Space 기록과 일치), 지도 칠하기, 노드 반영."""

import os

import numpy as np
import pytest

from kaboat_perception.virtual_walls import (
    latlon_to_local, load_walls, wall_mask, walls_to_local)

HERE = os.path.dirname(__file__)
CONFIG = os.path.join(HERE, '..', 'config', 'virtual_walls.yaml')
ORIGIN = (35.18487880, 128.56510100, 30.0)          # 2026-11-11 RTK 원점
# 2026-11-12 gps_mark_logger CSV: 위도, 경도 → 그 원점 기준 x, y
FIELD = [((35.18483600, 128.56659630), (136.197, -4.747)),
         ((35.18490760, 128.56660160), (136.679, 3.196)),
         ((35.18494360, 128.56632130), (111.149, 7.190)),
         ((35.18486010, 128.56631160), (110.265, -2.074))]


def test_matches_field_gps_log_and_geo_local():
    for (lat, lon), xy in FIELD:
        np.testing.assert_allclose(latlon_to_local(lat, lon, ORIGIN), xy, atol=0.01)
    geo_local = pytest.importorskip('kaboat_hardware.geo_local')
    frame = geo_local.LocalFrame(*ORIGIN)
    for (lat, lon), _ in FIELD:
        np.testing.assert_allclose(latlon_to_local(lat, lon, ORIGIN),
                                   frame.to_local(lat, lon), atol=1e-6)


def test_shipped_config_has_the_two_long_sides():
    walls = load_walls(CONFIG)
    assert set(walls) == {'avoid_north', 'avoid_south'}
    xy = walls_to_local(walls, ORIGIN)
    np.testing.assert_allclose(xy['avoid_north'], [[136.679, 3.196], [111.149, 7.190]], atol=0.01)
    np.testing.assert_allclose(xy['avoid_south'], [[136.197, -4.747], [110.265, -2.074]], atol=0.01)
    assert load_walls('/nonexistent/walls.yaml') == {}


def test_bad_wall_is_refused(tmp_path):
    f = tmp_path / 'w.yaml'
    f.write_text('walls:\n  one: [[35.0, 128.0]]\n')
    with pytest.raises(ValueError, match='2개 이상'):
        load_walls(str(f))


def test_mask_draws_only_inside_the_window():
    walls = {'w': np.array([[-20.0, 1.0], [20.0, 1.0]])}         # 창을 가로지르는 동서 벽
    mask = wall_mask(walls, anchor=(-37, -37), cells=75, resolution=0.2, thickness=0.4)
    rows = np.nonzero(mask.any(axis=1))[0]
    assert mask.any(axis=1).sum() in (2, 3)                       # 두께 0.4 m = 2칸 남짓
    assert mask[rows[0]].all()                                    # 창 끝에서 끝까지
    np.testing.assert_allclose((rows.mean() - 37 + 0.5) * 0.2, 1.0, atol=0.2)
    far = {'w': np.array([[500.0, 500.0], [520.0, 500.0]])}
    assert not wall_mask(far, (-37, -37), 75, 0.2, 0.4).any()


def _node_with(tmp_path, origin=ORIGIN, extra=()):
    import rclpy
    from nav_msgs.msg import Odometry
    from kaboat_perception.occupancy_grid import OccupancyGridNode
    origin_file = tmp_path / 'origin.yaml'
    if origin is not None:
        origin_file.write_text(f'origin_lat: {origin[0]}\norigin_lon: {origin[1]}\n'
                               f'origin_alt: {origin[2]}\n')
    rclpy.init(args=['--ros-args', '-p', f'virtual_walls_file:={CONFIG}',
                     '-p', f'origin_file:={origin_file}', *extra])
    node = OccupancyGridNode()
    node.sent = []
    node.pub.publish = node.sent.append
    odom = Odometry()
    odom.header.frame_id = 'odom'
    odom.pose.pose.position.x, odom.pose.pose.position.y = 123.0, 2.0   # 경기장 가운데
    odom.pose.pose.orientation.w = 1.0
    node._on_odom(odom)
    return node, origin_file


def _value(node, x, y):
    node._publish()
    g = node.sent[-1]
    ix = int((x - g.info.origin.position.x) / g.info.resolution)
    iy = int((y - g.info.origin.position.y) / g.info.resolution)
    return g.data[iy * g.info.width + ix]


def test_node_publishes_walls_and_follows_the_origin(tmp_path):
    import rclpy
    node, origin_file = _node_with(tmp_path)
    try:
        # 북쪽 벽은 x=123 에서 y ≈ 3.196 + (123-136.679)/(111.149-136.679)·(7.190-3.196) = 5.34
        assert _value(node, 123.0, 5.34) == 100
        assert _value(node, 123.0, 2.0) == -1                     # 통로 가운데는 그대로 (미관측)
        assert _value(node, 123.0, -3.4) == 100                   # 남쪽 벽
        # 원점을 북쪽으로 10 m 옮겨 다시 잡으면 벽이 지금 원점 기준으로 내려온다
        moved = (ORIGIN[0] + 10.0 / 111195.0, ORIGIN[1], ORIGIN[2])
        origin_file.write_text(f'origin_lat: {moved[0]}\norigin_lon: {moved[1]}\n'
                               f'origin_alt: {moved[2]}\n')
        os.utime(origin_file, ns=(1, 1))                          # mtime 확실히 바뀜
        node._reload_walls()
        assert _value(node, 123.0, -4.66) == 100
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_node_without_origin_uses_no_walls(tmp_path):
    import rclpy
    node, _ = _node_with(tmp_path, origin=None)
    try:
        assert node.walls_xy == {}
        assert _value(node, 123.0, 5.34) == -1
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_walls_can_be_turned_off(tmp_path):
    import rclpy
    node, _ = _node_with(tmp_path, extra=('-p', 'virtual_walls_file:=none'))
    try:
        assert node.walls_latlon == {} and _value(node, 123.0, 5.34) == -1
    finally:
        node.destroy_node()
        rclpy.shutdown()
