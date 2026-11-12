"""게이트 사슬: 부표 기억(PostMemory) → 다음 게이트 미리 고르기 → 중점들을 잇는 경로.

2026-11-12 4번 시험(15:31:27): 게이트 2 를 지난 뒤에야 게이트 3 을 골라, 3 m 앞에서 옆으로
1.3 m 를 급히 옮기다 경로 검사에 막혀 2 초 정지 → 쌍 버림 → 10 초 대기. 게이트 3 의 두 부표는
그 2 초 전부터 라이다에 보이고 있었다.
"""

import math

import numpy as np
import pytest
import rclpy
from std_msgs.msg import String

from kaboat_behaviors.gate_corridor import (
    GatePair, GateParams, make_chain_path, next_gate_after, path_ahead,
    select_gate_pair_from_posts)
from kaboat_behaviors.gate_memory import PostMemory
from test_gate_follower import _grid, _ready_node, _set_odom

# 현장처럼 어긋난 게이트 (간격 3.5 m, 폭 2.8 m, 중점이 옆으로 1.3 m 씩 왔다 갔다)
MIDS = [(3.0, 0.0), (6.5, -1.3), (10.0, 0.0)]
STAGGERED = [(x, y + s) for x, y in MIDS for s in (-1.4, 1.4)]


# ── 부표 기억 ────────────────────────────────────────────────────────────

def test_memory_confirms_after_three_sightings_and_averages_position():
    m = PostMemory()
    for t, x in enumerate((5.0, 5.2, 5.1)):
        assert m.confirmed() == []
        m.update([(x, 1.0)], float(t) * 0.1, (0.0, 0.0))
    assert len(m.confirmed()) == 1
    assert m.confirmed()[0][0] == pytest.approx(5.07, abs=0.05)
    m.update([(5.1, 1.0), (9.0, 0.0)], 0.3, (0.0, 0.0))   # 새 부표는 따로
    assert len(m.posts) == 2 and len(m.confirmed()) == 1


def test_memory_keeps_far_confirmed_posts_but_forgets_unseen_near_ones():
    m = PostMemory(forget_sec=1.0, recheck_range=4.0)
    for t in range(3):
        m.update([(8.0, 0.0), (3.0, 1.0)], t * 0.1, (0.0, 0.0))
    m.update([], 0.5, (0.0, 0.0))
    assert len(m.confirmed()) == 2                        # 아직 1 초 안 지남
    m.update([], 2.0, (0.0, 0.0))
    assert m.confirmed() == [(8.0, 0.0)]                  # 4 m 안(3, 1)은 볼 수 있는데 안 보임 → 지움
    m.update([], 5.0, (6.0, 0.0))                         # 다가가서도 안 보이면 지움
    assert m.confirmed() == [(8.0, 0.0)]
    m.update([], 6.5, (6.0, 0.0))
    assert m.confirmed() == []


def test_unconfirmed_noise_is_dropped_and_far_behind_posts_pruned():
    m = PostMemory()
    m.update([(20.0, 0.0)], 0.0, (10.0, 0.0))             # 한 번만 잡힌 것(물보라·반사)
    m.update([], 1.5, (10.0, 0.0))
    assert m.posts == []
    for t in range(3):
        m.update([(0.0, 0.0)], t * 0.1, (0.0, 0.0))
    m.update([(0.0, 0.0)], 0.4, (16.0, 0.0))              # 배가 16 m 지나감 → 버림
    m.update([], 0.5, (16.0, 0.0))
    assert m.posts == []


def test_with_current_adds_only_remembered_posts_the_grid_lost():
    m = PostMemory()
    for t in range(3):
        m.update([(3.0, 1.0), (7.0, -1.0)], t * 0.1, (0.0, 0.0))
    posts = m.with_current([(3.1, 1.0)])
    assert posts[0] == (3.1, 1.0) and len(posts) == 2     # 지금 잡힌 건 지도 위치 그대로
    assert posts[1] == pytest.approx((7.0, -1.0))         # 지도에서 빠진 건 기억으로


# ── 사슬 고르기·경로 ─────────────────────────────────────────────────────

def test_next_gate_after_follows_the_staggered_course():
    p = GateParams()
    first = select_gate_pair_from_posts(STAGGERED, (0.0, 0.0), (13.0, 0.0), [], p)
    assert first.midpoint == pytest.approx(MIDS[0])
    second = next_gate_after(STAGGERED, first, (13.0, 0.0), [], p)
    assert second.midpoint == pytest.approx(MIDS[1])
    third = next_gate_after(STAGGERED, second, (13.0, 0.0), [first.midpoint], p)
    assert third.midpoint == pytest.approx(MIDS[2])
    assert next_gate_after(STAGGERED, third, (13.0, 0.0),
                           [first.midpoint, second.midpoint], p) is None


def test_chain_path_crosses_every_midpoint_square_to_the_gate():
    p = GateParams()
    pairs = []
    for x, y in MIDS:
        pairs.append(GatePair((x, y + 1.4), (x, y - 1.4), (x, y), 2.8, (1.0, 0.0)))
    path = make_chain_path((0.0, 0.5), pairs, p)
    s = path.samples
    for x, y in MIDS:
        i = int(np.argmin(np.linalg.norm(s - (x, y), axis=1)))
        assert np.linalg.norm(s[i] - (x, y)) < 0.03          # 중점을 지난다
        heading = math.degrees(math.atan2(*(s[i + 2] - s[i - 2])[::-1]))
        assert abs(heading) < 5.0                             # 게이트선에 수직으로
    assert s[-1] == pytest.approx((MIDS[-1][0] + p.exit_distance, 0.0), abs=1e-6)
    steps = np.linalg.norm(np.diff(s, axis=0), axis=1)
    assert steps.max() == pytest.approx(p.sample_spacing, abs=1e-3)
    assert make_chain_path((0.0, 0.0), pairs[:1], p).samples[-1] == pytest.approx((4.0, 0.0))


def test_path_ahead_limits_checks_to_the_horizon():
    p = GateParams()
    pair = GatePair((10.0, 1.4), (10.0, -1.4), (10.0, 0.0), 2.8, (1.0, 0.0))
    path = make_chain_path((0.0, 0.0), [pair, GatePair((14.0, 1.4), (14.0, -1.4),
                                                         (14.0, 0.0), 2.8, (1.0, 0.0))], p)
    ahead = path_ahead(path, 40, 6.0)
    assert np.linalg.norm(ahead[0] - path.samples[38]) < 1e-9
    assert np.sum(np.linalg.norm(np.diff(ahead, axis=0), axis=1)) == pytest.approx(6.1, abs=0.1)
    assert len(path_ahead(path, 40)) == len(path.samples) - 38


# ── 노드 ─────────────────────────────────────────────────────────────────

def _near(points, expected, tol=0.3):
    """게이트 중점 목록 비교 — 지도 칸(0.2 m)에 칠한 부표라 중심이 칸만큼 밀린다."""
    a, b = np.asarray(points, dtype=float), np.asarray(expected, dtype=float)
    return a.shape == b.shape and bool(np.all(np.abs(a - b) <= tol))


def _record_statuses(node):
    seen = []
    original = node._status

    def status(value):
        seen.append(value)
        original(value)
    node._status = status
    return seen


def test_staggered_course_is_driven_without_stopping():
    rclpy.init(args=['--ros-args', '-p', 'gate.required_pairs:=3'])
    node = None
    try:
        node = _ready_node(STAGGERED)
        node._on_goal(_goal(13.0, 0.0))
        seen = _record_statuses(node)
        node.compute_cmd()
        assert _near([q.midpoint for q in node._chain], MIDS)   # 처음부터 셋 다 (지도 칸 0.2 m)
        boat = np.zeros(2)
        for _ in range(1000):
            if len(node._passed_midpoints) == 3:
                break
            s = node._path.samples
            boat = s[min(int(np.argmin(np.linalg.norm(s - boat, axis=1))) + 4, len(s) - 1)]
            _set_odom(node, *boat)
            node.compute_cmd()
        assert _near(node._passed_midpoints, MIDS)
        stops = {v for v in seen if not v.startswith(('FOLLOW_GATE', 'GATE_'))}
        assert stops == set(), stops                     # 멈춤(WAIT·UNSAFE·…) 없이
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


def _goal(x, y):
    from geometry_msgs.msg import PoseStamped
    goal = PoseStamped()
    goal.header.frame_id = 'odom'
    goal.pose.position.x, goal.pose.position.y = x, y
    return goal


def test_next_gate_comes_from_memory_when_the_grid_lost_it():
    rclpy.init()
    node = None
    try:
        node = _ready_node(STAGGERED)
        node._on_goal(_goal(13.0, 0.0))
        for _ in range(3):                                  # 지도 3장 → 부표 확정
            node._on_grid(_grid(STAGGERED))
            node.compute_cmd()
        assert len(node._memory.confirmed()) == 6
        node._drop_pair()
        node._on_grid(_grid(STAGGERED[:2]))                 # 지도엔 이제 게이트 1 만
        node.compute_cmd()
        assert _near([q.midpoint for q in node._chain], MIDS)   # 2·3 은 기억으로
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


def test_single_gate_mode_still_switches_right_after_crossing():
    rclpy.init(args=['--ros-args', '-p', 'gate.max_chain_gates:=1'])
    node = None
    try:
        node = _ready_node(STAGGERED)
        node._on_goal(_goal(13.0, 0.0))
        node.compute_cmd()
        assert len(node._chain) == 1                        # 미리 잇지 않음
        _set_odom(node, 3.3, 0.0)                           # 게이트 1 을 막 지남
        node.compute_cmd()
        assert _near(node._passed_midpoints, MIDS[:1])
        assert _near(node._pair.midpoint, MIDS[1])   # 경로 끝(1 m)까지 안 가고 바로
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


def test_rebuilt_chain_path_restarts_progress():
    """이은 경로로 바꾸면 LOS 진행 위치를 새로 — 옛 경로 번호로 새 경로를 검사하면 엉뚱한
    구간(지도 밖)을 봐서 PATH_UNOBSERVED 로 멀쩡한 게이트를 버렸다 (녹화 재생 15:31:53)."""
    rclpy.init()
    node = None
    try:
        node = _ready_node(STAGGERED)
        node._on_goal(_goal(13.0, 0.0))
        node.compute_cmd()
        from kaboat_behaviors.gate_corridor import make_gate_path
        node._chain = node._chain[:1]
        node._path = make_gate_path((0.0, 0.0), node._pair, node.gate_params)
        node._los_follower.set_path(node._path)
        node._los_follower.progress_idx = 400             # 옛(긴) 경로 번호 — 새 경로 끝 너머
        node.compute_cmd()
        assert len(node._chain) == 3
        assert node._los_follower.progress_idx < 20       # 새 경로의 배 위치(시작)부터
        assert node._status_text == 'FOLLOW_GATE_1/7'
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


def test_blocked_extension_falls_back_to_the_current_gate_instead_of_dropping_it():
    rclpy.init()
    node = None
    try:
        node = _ready_node(STAGGERED)
        node._on_goal(_goal(13.0, 0.0))
        node.compute_cmd()
        assert len(node._chain) == 3
        node._on_grid(_grid([*STAGGERED, (5.0, -0.6)]))   # 게이트 1→2 사이 경로 위 장애물
        node.compute_cmd()
        assert len(node._chain) == 1 and node._rejected == []
        assert _near(node._pair.midpoint, MIDS[0])
        assert node._status_text == 'FOLLOW_GATE_1/7'
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()
