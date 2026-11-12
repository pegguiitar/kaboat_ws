"""가상 벽: 위도·경도 → 원점 기준 x·y (현장 Space 기록과 일치), 지도 칠하기, 노드 반영."""

import os

import numpy as np
import pytest

from kaboat_perception.virtual_walls import (
    arena_walls, latlon_to_local, layout_to_local, load_arenas, load_walls, offset_outward,
    offset_polygon, wall_mask)

HERE = os.path.dirname(__file__)
CONFIG = os.path.join(HERE, '..', 'config', 'virtual_walls.yaml')
ORIGIN = (35.18487880, 128.56510100, 30.0)          # 2026-11-11 RTK 원점
# 2026-11-12 gps_mark_logger CSV: 위도, 경도 → 그 원점 기준 x, y
FIELD = [((35.18483600, 128.56659630), (136.197, -4.747)),
         ((35.18490760, 128.56660160), (136.679, 3.196)),
         ((35.18494360, 128.56632130), (111.149, 7.190)),
         ((35.18486010, 128.56631160), (110.265, -2.074))]
# 토너먼트 경기장: 같은 날 16:47~16:49 gps_mark_logger #1~#4 (CSV x, y)
TOURNAMENT = [((35.18467100, 128.56513170), (2.797, -23.054)),
              ((35.18460040, 128.56520860), (9.801, -30.887)),
              ((35.18475700, 128.56544660), (31.479, -13.513)),
              ((35.18483400, 128.56537370), (24.839, -4.970))]


def test_matches_field_gps_log_and_geo_local():
    for (lat, lon), xy in FIELD:
        np.testing.assert_allclose(latlon_to_local(lat, lon, ORIGIN), xy, atol=0.01)
    geo_local = pytest.importorskip('kaboat_hardware.geo_local')
    frame = geo_local.LocalFrame(*ORIGIN)
    for (lat, lon), _ in FIELD:
        np.testing.assert_allclose(latlon_to_local(lat, lon, ORIGIN),
                                   frame.to_local(lat, lon), atol=1e-6)


def test_shipped_config_is_the_avoid_arena_with_two_entrances():
    assert load_walls(CONFIG) == {}
    arenas = load_arenas(CONFIG)
    assert set(arenas) == {'avoid', 'tournament'}
    corners = [latlon_to_local(lat, lon, ORIGIN) for lat, lon in arenas['avoid']['corners']]
    np.testing.assert_allclose(corners, [xy for _, xy in FIELD], atol=0.01)   # #2 #3 #4 #5
    assert arenas['avoid']['openings'] == {0: 3.0, 2: 3.0}                   # 짧은 두 변
    assert load_walls('/nonexistent/walls.yaml') == {}
    assert load_arenas('/nonexistent/walls.yaml') == {}


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


def test_offset_pushes_each_wall_away_from_the_arena_centre():
    # 방향을 서로 반대로 적은 동서 벽 두 개 (y = +4, -4) → 바깥으로 0.6 m
    walls = {'n': np.array([[0.0, 4.0], [20.0, 4.0]]),
             's': np.array([[20.0, -4.0], [0.0, -4.0]])}
    out = offset_outward(walls, 0.6)
    np.testing.assert_allclose(out['n'][:, 1], [4.6, 4.6])
    np.testing.assert_allclose(out['s'][:, 1], [-4.6, -4.6])
    np.testing.assert_allclose(out['n'][:, 0], [0.0, 20.0])          # 벽 방향으로는 안 움직임
    np.testing.assert_allclose(offset_outward(walls, 0.0)['n'], walls['n'])
    single = {'n': walls['n']}                                        # 바깥을 정할 수 없음
    np.testing.assert_allclose(offset_outward(single, 0.6)['n'], walls['n'])


def _shipped(offset=0.6):
    return layout_to_local(load_walls(CONFIG), load_arenas(CONFIG), ORIGIN, offset)


def test_shipped_long_sides_move_outward_by_sixty_centimetres():
    walls = _shipped()
    # 경기장마다 입구 2곳으로 나뉜 둘레 두 줄
    assert set(walls) == {'avoid_0', 'avoid_1', 'tournament_0', 'tournament_1'}

    def y_at(a, b, x):
        return a[1] + (x - a[0]) / (b[0] - a[0]) * (b[1] - a[1])

    # 북쪽: 입구 북쪽 끝 → #3 → #4 → 출구 북쪽 끝, 남쪽: 출구 남쪽 끝 → #5 → #2 → 입구 남쪽 끝
    north, south = walls['avoid_0'], walls['avoid_1']
    assert len(north) == len(south) == 4
    # x = 123 에서 북쪽 벽 5.34 → 약 5.95, 남쪽 벽 -3.39 → 약 -3.99 (수직 0.6 m, 벽이 조금 기울어짐)
    assert y_at(north[1], north[2], 123.0) == pytest.approx(5.947, abs=0.01)
    assert y_at(south[1], south[2], 123.0) == pytest.approx(-3.990, abs=0.01)


def test_short_sides_leave_three_metres_in_the_middle():
    walls = _shipped()
    north, south = walls['avoid_0'], walls['avoid_1']
    east_gap = (north[0], south[-1])                  # 입구 (#2–#3 변)
    west_gap = (north[-1], south[0])                  # 출구 (#4–#5 변)
    for a, b in (east_gap, west_gap):
        assert np.linalg.norm(a - b) == pytest.approx(3.0, abs=1e-6)
    raw = [latlon_to_local(lat, lon, ORIGIN) for lat, lon in load_arenas(CONFIG)['avoid']['corners']]
    corners = offset_polygon(raw, 0.6)
    for (a, b), (i, j) in ((east_gap, (0, 1)), (west_gap, (2, 3))):
        np.testing.assert_allclose((a + b) / 2, (corners[i] + corners[j]) / 2, atol=1e-6)  # 변 가운데
    # 모서리에서 긴 변과 짧은 변이 같은 점으로 이어진다 (벽 사이 틈 없음)
    np.testing.assert_allclose(north[1], corners[1])
    np.testing.assert_allclose(south[2], corners[0])


def test_offset_polygon_moves_every_side_out():
    square = [(0.0, 0.0), (10.0, 0.0), (10.0, 10.0), (0.0, 10.0)]
    for corners in (square, square[::-1]):                     # 반시계·시계 둘 다
        out = offset_polygon(corners, 1.0)
        assert sorted(map(tuple, np.round(out, 9))) == sorted(
            [(-1.0, -1.0), (11.0, -1.0), (11.0, 11.0), (-1.0, 11.0)])
    np.testing.assert_allclose(offset_polygon(square, 0.0), square)


def test_arena_without_openings_is_one_closed_wall():
    square = [(0.0, 0.0), (10.0, 0.0), (10.0, 10.0), (0.0, 10.0)]
    walls = arena_walls('a', square, {}, 0.0)
    assert list(walls) == ['a_0']
    np.testing.assert_allclose(walls['a_0'], square + [square[0]])
    walls = arena_walls('a', square, {1: 20.0}, 0.0)           # 변보다 넓게 비우면 변 전체가 빔
    assert len(walls) == 1
    np.testing.assert_allclose(walls['a_0'], [(10.0, 10.0), (0.0, 10.0), (0.0, 0.0), (10.0, 0.0)])


def test_bad_arena_is_refused(tmp_path):
    f = tmp_path / 'a.yaml'
    f.write_text('arenas:\n  a:\n    corners: [[35.0, 128.0], [35.1, 128.0]]\n')
    with pytest.raises(ValueError, match='3개 이상'):
        load_arenas(str(f))
    f.write_text('arenas:\n  a:\n    corners: [[35.0, 128.0], [35.1, 128.0], [35.1, 128.1]]\n'
                 '    openings: {3: 3.0}\n')
    with pytest.raises(ValueError, match='변 번호'):
        load_arenas(str(f))


def test_thickness_point_three_is_one_cell():
    walls = {'w': np.array([[-20.0, 1.0], [20.0, 1.0]])}
    mask = wall_mask(walls, anchor=(-37, -37), cells=75, resolution=0.2, thickness=0.3)
    assert mask.any(axis=1).sum() == 1                            # 칸 0.2 m → 0.3 은 1칸


def _node_with(tmp_path, origin=ORIGIN, extra=(), at=(123.0, 2.0)):
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
    odom.pose.pose.position.x, odom.pose.pose.position.y = at   # 기본: 경기장 가운데
    odom.pose.pose.orientation.w = 1.0
    node._on_odom(odom)
    return node, origin_file


def _value(node, x, y):
    node._publish()
    g = node.sent[-1]
    ix = int((x - g.info.origin.position.x) / g.info.resolution)
    iy = int((y - g.info.origin.position.y) / g.info.resolution)
    return g.data[iy * g.info.width + ix]


def _wall_near(node, x, y):
    """1칸 두께 벽은 선 위치가 칸 경계에서 한 칸 어긋날 수 있어 ±1칸 안을 본다."""
    return max(_value(node, x, y + d) for d in (-0.2, 0.0, 0.2)) == 100


def test_node_publishes_walls_and_follows_the_origin(tmp_path):
    import rclpy
    node, origin_file = _node_with(tmp_path)
    try:
        assert node.wall_thickness == 0.3 and node.wall_offset == 0.6
        # 북쪽 벽: 기록 위치 x=123 에서 y ≈ 5.34 → 바깥으로 0.6 m 밀어 약 5.95
        assert _wall_near(node, 123.0, 5.95)
        assert _value(node, 123.0, 5.34) == -1                    # 예전(기록) 위치는 이제 벽 아님
        assert _value(node, 123.0, 2.0) == -1                     # 통로 가운데는 그대로 (미관측)
        assert _wall_near(node, 123.0, -3.99)                     # 남쪽 벽 (-3.39 → 약 -3.99)
        # 원점을 북쪽으로 10 m 옮겨 다시 잡으면 벽이 지금 원점 기준으로 내려온다
        moved = (ORIGIN[0] + 10.0 / 111195.0, ORIGIN[1], ORIGIN[2])
        origin_file.write_text(f'origin_lat: {moved[0]}\norigin_lon: {moved[1]}\n'
                               f'origin_alt: {moved[2]}\n')
        os.utime(origin_file, ns=(1, 1))                          # mtime 확실히 바뀜
        node._reload_walls()
        assert _wall_near(node, 123.0, -4.05)
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
        assert node.walls_latlon == {} and node.arenas_latlon == {}
        assert _value(node, 123.0, 5.95) == -1
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_node_leaves_the_entrance_open(tmp_path):
    import rclpy
    walls = _shipped()
    north, south = walls['avoid_0'], walls['avoid_1']
    gap_mid = (north[0] + south[-1]) / 2                     # 동쪽 입구 가운데
    node, _ = _node_with(tmp_path, at=(gap_mid[0] - 3.0, gap_mid[1]))
    try:
        def near(x, y):
            return [_value(node, x + dx, y + dy) for dx in (-0.2, 0.0, 0.2)
                    for dy in (-0.2, 0.0, 0.2)]
        assert 100 not in near(*gap_mid)                     # 입구 가운데는 비어 있음
        stub = (south[-2] + south[-1]) / 2                   # 입구 남쪽 벽 토막 (#2 쪽)
        assert 100 in near(*stub)
        north_stub = (north[0] + north[1]) / 2               # 입구 북쪽 벽 토막 (#3 쪽)
        assert 100 in near(*north_stub)
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_shipped_tournament_arena_is_the_field_rectangle():
    arena = load_arenas(CONFIG)['tournament']
    corners = np.array([latlon_to_local(lat, lon, ORIGIN) for lat, lon in arena['corners']])
    np.testing.assert_allclose(corners, [xy for _, xy in TOURNAMENT], atol=0.01)
    assert arena['openings'] == {0: 3.0, 2: 3.0}                         # 짧은 두 변
    sides = [np.linalg.norm(corners[(i + 1) % 4] - corners[i]) for i in range(4)]
    np.testing.assert_allclose(sides, [10.51, 27.78, 10.82, 28.51], atol=0.01)   # 약 10.7 × 28.1 m


def test_tournament_walls_long_sides_closed_short_sides_open_three_metres():
    walls = _shipped()
    a, b = walls['tournament_0'], walls['tournament_1']
    raw = [latlon_to_local(lat, lon, ORIGIN) for lat, lon in load_arenas(CONFIG)['tournament']['corners']]
    corners = offset_polygon(raw, 0.6)
    # 바깥으로 0.6 m: 넓힌 둘레 변이 원래 변보다 1.2 m 길다 (양 끝 0.6 m 씩)
    grown = [np.linalg.norm(corners[(i + 1) % 4] - corners[i]) for i in range(4)]
    raw_sides = [np.linalg.norm(np.subtract(raw[(i + 1) % 4], raw[i])) for i in range(4)]
    np.testing.assert_allclose(np.subtract(grown, raw_sides), 1.2, atol=.05)
    # 각 줄 = 짧은 변 토막 → 긴 변 → 짧은 변 토막, 두 줄 사이 틈은 짧은 변 가운데 3 m
    assert len(a) == len(b) == 4
    for gap, (i, j) in (((a[0], b[-1]), (0, 1)), ((a[-1], b[0]), (2, 3))):
        assert np.linalg.norm(gap[0] - gap[1]) == pytest.approx(3.0, abs=1e-6)
        np.testing.assert_allclose((gap[0] + gap[1]) / 2, (corners[i] + corners[j]) / 2, atol=1e-6)
