"""Three ordered LiDAR gate pairs and DRI clearance without YOLO colours."""

import math
from types import SimpleNamespace

import numpy as np
import pytest

from kaboat_behaviors.dri import DriParams, build_dri
from kaboat_behaviors.gate_corridor import (
    GateParams, crossed_gate, lookahead_chord_safe, make_gate_path,
    pair_observed, path_safe, points_observed, select_gate_pair)


def gate_grid(posts=None):
    res, size = 0.2, 100
    origin = (-5.0, -10.0)
    data = np.zeros((size, size), dtype=np.int8)
    if posts is None:
        posts = [(x, y) for x in (3.0, 5.0, 7.0) for y in (-1.5, 1.5)]
    for x, y in posts:
        ix = int(math.floor((x - origin[0]) / res))
        iy = int(math.floor((y - origin[1]) / res))
        data[iy:iy + 2, ix:ix + 2] = 100
    info = SimpleNamespace(
        resolution=res, width=size, height=size,
        origin=SimpleNamespace(position=SimpleNamespace(x=origin[0], y=origin[1])))
    return SimpleNamespace(info=info, data=data.ravel().tolist())


def test_three_pairs_selected_in_order_and_beyond_count_is_configurable():
    grid = gate_grid()
    params = GateParams(required_pairs=3)
    passed = []
    boat = (0.0, 0.0)
    goal = (9.0, 0.0)
    for expected_x in (3.2, 5.2, 7.2):
        pair = select_gate_pair(grid, boat, goal, passed, params)
        assert pair is not None
        assert pair.midpoint[0] == pytest.approx(expected_x)
        assert pair.midpoint[1] == pytest.approx(0.1)
        assert pair.width == pytest.approx(3.0, abs=0.1)
        passed.append(pair.midpoint)
        boat = (expected_x + 0.9, 0.1)
    assert select_gate_pair(grid, boat, goal, passed, params) is None
    assert GateParams(required_pairs=5).required_pairs == 5


def test_gate_path_passes_midpoint_and_dri_rejects_obstacle_on_it():
    p = GateParams()
    grid = gate_grid()
    pair = select_gate_pair(grid, (0.0, 0.0), (9.0, 0.0), [], p)
    path = make_gate_path((0.0, 0.0), pair, p)
    assert np.min(np.linalg.norm(path.samples - pair.midpoint, axis=1)) < 0.03
    dri = build_dri(grid, (0.0, 0.0), 0.0, DriParams())
    assert path_safe(dri, path, p)

    blocked = gate_grid(posts=[(3.0, -1.5), (3.0, 1.5), (1.5, 0.0)])
    blocked_dri = build_dri(blocked, (0.0, 0.0), 0.0, DriParams())
    assert not path_safe(blocked_dri, path, p)


def test_crossing_requires_going_between_posts():
    pair = select_gate_pair(gate_grid(), (0.0, 0.0), (9.0, 0.0), [],
                            GateParams())
    assert crossed_gate((2.8, 0.1), (3.4, 0.1), pair, GateParams())
    assert not crossed_gate((2.8, 1.4), (3.4, 1.4), pair, GateParams())
    assert not crossed_gate((3.4, 0.1), (2.8, 0.1), pair, GateParams())


def test_live_scan_must_show_both_posts_even_if_old_map_keeps_them():
    p = GateParams()
    pair = select_gate_pair(gate_grid(), (0.0, 0.0), (9.0, 0.0), [], p)
    assert pair_observed(pair, [pair.left, pair.right], p)
    assert not pair_observed(pair, [pair.left], p)


def test_missing_post_or_wall_is_not_a_gate():
    p = GateParams()
    one_post = gate_grid(posts=[(3.0, -1.5)])
    assert select_gate_pair(one_post, (0.0, 0.0), (9.0, 0.0), [], p) is None
    wall = gate_grid(posts=[(3.0, 1.5)])
    # A long wall cannot substitute for the missing post.
    data = np.asarray(wall.data).reshape(100, 100)
    data[40:65, 40:42] = 100
    wall.data = data.ravel().tolist()
    assert select_gate_pair(wall, (0.0, 0.0), (9.0, 0.0), [], p) is None


def test_unobserved_cells_block_the_gate_path():
    grid = gate_grid()
    pair = select_gate_pair(grid, (0.0, 0.0), (9.0, 0.0), [], GateParams())
    path = make_gate_path((0.0, 0.0), pair, GateParams())
    assert points_observed(grid, path.samples)
    ix = int(math.floor((1.0 - grid.info.origin.position.x) / grid.info.resolution))
    iy = int(math.floor((0.0 - grid.info.origin.position.y) / grid.info.resolution))
    data = np.asarray(grid.data).reshape(100, 100)
    data[iy - 2:iy + 3, ix - 2:ix + 3] = -1
    grid.data = data.ravel().tolist()
    assert not points_observed(grid, path.samples)


def test_lookahead_safety_checks_the_goal_when_follower_targets_it():
    # follow_cmd switches to the endpoint when it lies inside lookahead.
    # A path sample farther ahead would miss this obstacle on the goal chord.
    path = SimpleNamespace(
        samples=np.asarray([(0.0, y * 0.1) for y in range(21)] + [(0.8, 0.0)]),
        spacing=0.1)

    class Risk:
        occ_xy = np.empty((0, 2))

        def risk_at_many(self, xs, ys):
            return np.where((xs > 0.35) & (xs < 0.45) & (abs(ys) < 0.1), 2.0, 0.0)

    assert not lookahead_chord_safe(
        Risk(), (0.0, 0.0), path, 0, 1.0, GateParams())


def test_scan_hits_use_the_onboard_lidar_mount_offset():
    """선체 라이다가 base_link 보다 선수 쪽에 있으면 앞 벽은 그만큼 더 멀리 찍혀야 한다
    (계산 검증용 0.5 m — 실물 장착값과 무관)."""
    import math
    from types import SimpleNamespace
    from kaboat_behaviors.gate_corridor import scan_hits_world
    scan = SimpleNamespace(ranges=[2.0], angle_min=0.0, angle_increment=0.1,
                           range_min=0.1, range_max=20.0)
    # 선수 +x: 라이다는 (5.5, 2.5), 빔 2 m → 벽 (7.5, 2.5)
    (hit,) = scan_hits_world(scan, (5.0, 2.5), 0.0, (0.5, 0.0))
    assert hit[0] == pytest.approx(7.5) and hit[1] == pytest.approx(2.5)
    # 선수 -x: 라이다는 (4.5, 2.5), 빔 2 m → 벽 (2.5, 2.5)
    (hit,) = scan_hits_world(scan, (5.0, 2.5), math.pi, (0.5, 0.0))
    assert hit[0] == pytest.approx(2.5) and hit[1] == pytest.approx(2.5, abs=1e-9)
    # 오프셋 생략 시 예전처럼 base_link 원점
    (hit,) = scan_hits_world(scan, (5.0, 2.5), 0.0)
    assert hit[0] == pytest.approx(7.0)


def test_excluded_midpoints_are_skipped_like_passed_ones():
    grid = gate_grid()
    p = GateParams()
    first = select_gate_pair(grid, (0.0, 0.0), (9.0, 0.0), [], p)
    other = select_gate_pair(grid, (0.0, 0.0), (9.0, 0.0), [], p,
                             excluded_midpoints=[first.midpoint])
    assert other is not None
    assert np.linalg.norm(np.subtract(other.midpoint, first.midpoint)) > 1.0


def _channel_grid(centers, width):
    """게이트 중심 목록 → 각 게이트 기둥을 통로 진행 방향에 수직으로 놓은 격자."""
    posts = []
    for i, c in enumerate(centers):
        c = np.asarray(c, dtype=float)
        t = (np.asarray(centers[min(i + 1, len(centers) - 1)], dtype=float)
             - np.asarray(centers[max(i - 1, 0)], dtype=float))
        t /= np.linalg.norm(t)
        n = np.array([-t[1], t[0]])
        posts += [tuple(c + n * width / 2), tuple(c - n * width / 2)]
    res, size, origin = 0.2, 250, (-25.0, -25.0)
    data = np.zeros((size, size), dtype=np.int8)
    for x, y in posts:
        ix = int(math.floor((x - origin[0]) / res))
        iy = int(math.floor((y - origin[1]) / res))
        data[iy - 1:iy + 1, ix - 1:ix + 1] = 100
    info = SimpleNamespace(
        resolution=res, width=size, height=size,
        origin=SimpleNamespace(position=SimpleNamespace(x=origin[0], y=origin[1])))
    return SimpleNamespace(info=info, data=data.ravel().tolist())


def _follow(grid, centers, goal, p):
    """노드처럼 통과한 게이트 중점·방향을 넘기며 차례로 고른다 → 고른 중점 목록."""
    boat, passed, dirs = (0.0, 0.0), [], []
    for _ in centers:
        pair = select_gate_pair(grid, boat, goal, passed, p,
                                last_direction=dirs[-1] if dirs else None)
        if pair is None:
            break
        passed.append(pair.midpoint)
        dirs.append(pair.direction)
        boat = tuple(np.asarray(pair.midpoint) + 0.6 * np.asarray(pair.direction))
    return passed


@pytest.mark.parametrize('skew_deg', [0.0, 20.0, 35.0])
def test_field_channel_seven_pairs_up_to_35_deg_off_the_goal_axis(skew_deg):
    """현장: 폭 2.5 m · 간격 2.5 m 7쌍, 통로가 배→목표 축에서 35° 까지 틀어져도 순서대로."""
    u = np.array([math.cos(math.radians(skew_deg)), math.sin(math.radians(skew_deg))])
    centers = [tuple(u * (2.0 + 2.5 * k)) for k in range(7)]
    picked = _follow(_channel_grid(centers, 2.5), centers, (40.0, 0.0), GateParams())
    assert len(picked) == 7
    for got, want in zip(picked, centers):
        assert np.linalg.norm(np.subtract(got, want)) < 0.25


def test_field_channel_bending_ten_deg_per_gate():
    centers, pos, ang = [], np.array([2.0, 0.0]), 0.0
    for _ in range(7):
        centers.append(tuple(pos))
        ang += math.radians(10.0)
        pos = pos + 2.5 * np.array([math.cos(ang), math.sin(ang)])
    picked = _follow(_channel_grid(centers, 2.5), centers, tuple(pos), GateParams())
    assert len(picked) == 7
    for got, want in zip(picked, centers):
        assert np.linalg.norm(np.subtract(got, want)) < 0.25


def test_skewed_gate_crossing_uses_the_gate_normal():
    """35° 기운 게이트: 통과 판정이 배→목표 축이 아니라 게이트선 수직 방향으로."""
    u = np.array([math.cos(math.radians(35.0)), math.sin(math.radians(35.0))])
    centers = [tuple(u * (2.0 + 2.5 * k)) for k in range(3)]
    pair = select_gate_pair(_channel_grid(centers, 2.5), (0.0, 0.0), (40.0, 0.0), [],
                            GateParams())
    assert abs(float(np.dot(pair.direction, u))) > 0.99
    mid = np.asarray(pair.midpoint)
    assert crossed_gate(tuple(mid - 0.3 * u), tuple(mid + 0.3 * u), pair, GateParams())
