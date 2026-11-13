"""ROS gate behavior uses LiDAR/DRI and requires three actual crossings."""

import math
import time

import numpy as np
import pytest
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

from kaboat_behaviors.gate_follower import GateFollower
from test_gate_corridor import gate_grid


def _odom(x, y):
    msg = Odometry()
    msg.header.frame_id = 'odom'
    msg.pose.pose.position.x = float(x)
    msg.pose.pose.position.y = float(y)
    msg.pose.pose.orientation.w = 1.0
    return msg


def _grid(posts=None):
    fake = gate_grid(posts)
    msg = OccupancyGrid()
    msg.header.frame_id = 'odom'
    msg.info.resolution = fake.info.resolution
    msg.info.width = fake.info.width
    msg.info.height = fake.info.height
    msg.info.origin.position.x = fake.info.origin.position.x
    msg.info.origin.position.y = fake.info.origin.position.y
    msg.info.origin.orientation.w = 1.0
    msg.data = fake.data
    return msg


def _scan_for_posts(boat_xy, posts):
    scan = LaserScan()
    scan.range_min, scan.range_max = 0.1, 25.0
    scan.angle_min = -math.pi
    scan.angle_increment = 2 * math.pi / 720
    ranges = [math.inf] * 720
    for x, y in posts:
        dx, dy = x - boat_xy[0], y - boat_xy[1]
        angle = math.atan2(dy, dx)
        index = round((angle - scan.angle_min) / scan.angle_increment) % 720
        ranges[index] = math.hypot(dx, dy)
    scan.ranges = ranges
    return scan


def _set_odom(node, x, y):
    node._on_odom(_odom(x, y))
    node._on_scan(_scan_for_posts((x, y), node._test_posts))


def _ready_node(posts=None):
    node = GateFollower()
    node._on_state(String(data='gate'))
    goal = PoseStamped()
    goal.header.frame_id = 'odom'
    goal.pose.position.x = 9.0
    node._on_goal(goal)
    node._test_posts = (posts if posts is not None else
                        [(x, y) for x in (3.0, 5.0, 7.0)
                         for y in (-1.5, 1.5)])
    node._on_grid(_grid(posts))
    _set_odom(node, 0.0, 0.0)
    return node


def _follow_current_gate(node, step=4):
    """지금 경로를 따라 배를 옮기다 통과 개수가 하나 늘면 멈춘다.

    게이트 사슬은 경로가 다음 게이트까지 이어져 있어(통과해도 경로가 안 끊김) 경로 끝이
    아니라 통과 개수로 본다. 경로가 중간에 바뀌면(다음 게이트를 이어 붙임) 새 경로를 따른다.
    """
    start = len(node._passed_midpoints)
    cmd = node.compute_cmd()  # acquire path
    assert cmd.linear.x == 0.0  # preview does not drive /cmd/gate
    assert node._path is not None
    for _ in range(2000):
        path = node._path
        if path is None or len(node._passed_midpoints) > start:
            return
        boat = np.array([node.odom.pose.pose.position.x, node.odom.pose.pose.position.y])
        nearest = int(np.argmin(np.linalg.norm(path.samples - boat, axis=1)))
        _set_odom(node, *path.samples[min(nearest + step, len(path.samples) - 1)])
        node.compute_cmd()
    raise AssertionError('게이트를 통과하지 못함')


def test_three_pairs_pass_before_completion_and_output_stays_preview_only():
    rclpy.init(args=['--ros-args', '-p', 'gate.required_pairs:=3'])   # 기본 7 — 3쌍 배치
    node = None
    try:
        node = _ready_node()
        assert not node.motion_enabled
        for count in range(1, 4):
            _follow_current_gate(node)
            assert len(node._passed_midpoints) == count
            assert not node._completion_sent
        node.compute_cmd()
        assert node._completion_sent
        assert node._status_text == 'COMPLETE'
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


def test_missing_third_pair_and_stale_scan_fail_closed():
    rclpy.init()
    node = None
    try:
        posts = [(x, y) for x in (3.0, 5.0) for y in (-1.5, 1.5)]
        node = _ready_node(posts)
        for count in range(1, 3):
            _follow_current_gate(node)
            assert len(node._passed_midpoints) == count
        cmd = node.compute_cmd()
        assert node._status_text == 'WAIT_GATE_PAIR'
        assert not node._completion_sent
        assert cmd.linear.x == cmd.angular.z == 0.0

        node._scan_at = time.monotonic() - 2.0
        cmd = node.compute_cmd()
        assert node._status_text == 'SCAN_TIMEOUT'
        assert cmd.linear.x == cmd.angular.z == 0.0
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


def test_cached_gate_stops_when_one_post_disappears_from_live_scan():
    """한 기둥이 스캔에서 빠져도 post_visible_timeout(1 초) 안이면 계속 — 부표 검출은 스캔당
    83~92 % 라 한 스캔 빠짐마다 서면 4번에 1번 추진 0. 1 초 넘게 안 보이면 멈춘다."""
    rclpy.init()
    node = None
    try:
        node = _ready_node()
        node.compute_cmd()
        assert node._pair is not None
        node._scan_hits = np.asarray([node._pair.left])
        node.compute_cmd()
        assert node._status_text == 'FOLLOW_GATE_1/7'
        node._post_seen_at[1] = time.monotonic() - 1.5      # 오른 기둥 1.5 초째 안 보임
        cmd = node.compute_cmd()
        assert node._status_text == 'GATE_POSTS_NOT_VISIBLE'
        assert cmd.linear.x == 0.0                          # 미리보기 — 명령은 안 냄
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


def test_posts_need_not_be_visible_close_to_the_gate_line():
    """선수 라이다 뒤쪽 가림 때문에 게이트선 직전엔 기둥이 안 보일 수 있다 — 계속 진행."""
    rclpy.init()
    node = None
    try:
        node = _ready_node()
        node.compute_cmd()
        mid = node._pair.midpoint
        _set_odom(node, mid[0] - 0.8, mid[1])           # 게이트선 0.8 m 앞 (해제 거리 1.0 m)
        node._scan_hits = np.asarray([node._pair.left])  # 한쪽 기둥만 보임
        node.compute_cmd()
        assert node._status_text == 'FOLLOW_GATE_1/7'
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


def _blocked_gate_node():
    # 게이트 (x=3) 중점 경로 위에 장애물 → PATH_UNSAFE
    node = _ready_node([(3.0, -1.5), (3.0, 1.5), (1.5, 0.0)])
    node.compute_cmd()
    assert node._status_text == 'PATH_UNSAFE'
    return node


def test_persistent_path_unsafe_drops_the_pair_and_holds_it_out():
    rclpy.init()
    node = None
    try:
        node = _blocked_gate_node()
        mid = node._pair.midpoint
        node._stall_since = time.monotonic() - node.replan_timeout - 0.1
        node.compute_cmd()
        assert node._pair is None and node._path is None
        assert [m for m, _ in node._rejected] == [mid]
        assert node._passed_midpoints == []
        node.compute_cmd()                               # 버린 쌍은 다시 고르지 않음
        assert node._status_text == 'WAIT_GATE_PAIR'

        node._rejected = [(m, time.monotonic() - 1.0) for m, _ in node._rejected]
        node.compute_cmd()                               # 보류 시간이 지나면 다시 후보
        assert node._pair is not None
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


def test_stall_after_crossing_counts_the_gate_as_passed():
    rclpy.init()
    node = None
    try:
        node = _blocked_gate_node()
        mid = node._pair.midpoint
        node._crossed = True
        node._stall_since = time.monotonic() - node.replan_timeout - 0.1
        node.compute_cmd()
        assert node._passed_midpoints == [mid]
        assert node._rejected == []
        assert node._status_text == 'GATE_1/7_PASSED'
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


# ── 첫 쌍 찾아 다가가기 (SEEK_FIRST_PAIR) ─────────────────────────────

class _Clock:
    def __init__(self, t=1000.0):
        self.t = t

    def __call__(self):
        return self.t


def _seek_node(monkeypatch, posts, goal_x=14.0):
    clock = _Clock()
    monkeypatch.setattr(time, 'monotonic', clock)     # gate_follower·behavior_base 가 쓰는 시계
    node = _ready_node(posts)
    goal = PoseStamped()
    goal.header.frame_id = 'odom'
    goal.pose.position.x = goal_x
    node._on_goal(goal)
    previews = []
    node.preview_pub.publish = previews.append
    return node, clock, previews


def _tick_at(node, clock, t, x=0.0, y=0.0):
    clock.t = t
    _set_odom(node, x, y)
    node._on_grid(_grid(node._test_posts))
    return node.compute_cmd()


FAR_GATES = [(x, y) for x in (7.5, 9.5, 11.5) for y in (-1.8, 1.8)]   # 첫 게이트 7.5 m (앞 창 6 m 밖)


def test_seek_first_pair_creeps_toward_far_gate_then_follows_it(monkeypatch):
    rclpy.init()
    node = None
    try:
        node, clock, previews = _seek_node(monkeypatch, FAR_GATES)
        _tick_at(node, clock, 1000.0)
        assert node._status_text == 'WAIT_GATE_PAIR'          # 처음 2 s 는 그대로 기다림
        _tick_at(node, clock, 1001.0)
        assert node._status_text == 'WAIT_GATE_PAIR' and previews[-1].linear.x == 0.0
        cmd = _tick_at(node, clock, 1002.5)
        assert node._status_text == 'SEEK_FIRST_PAIR'
        assert previews[-1].linear.x == pytest.approx(node.seek_thrust)   # 천천히 앞으로
        assert abs(previews[-1].angular.z) < 0.05      # 대칭 게이트 — 거의 곧장 (격자 반 칸 치우침)
        assert cmd.linear.x == 0.0                                        # 미리보기 — 실제 명령 0
        # 2 m 다가가면 첫 게이트가 앞 창(6 m) 안 → 평소처럼 쌍을 골라 따라감
        _tick_at(node, clock, 1003.0, x=2.0)
        assert node._status_text.startswith('FOLLOW_GATE_1/')
        assert node._pair.midpoint[0] == pytest.approx(7.7, abs=0.2)      # 첫 게이트 (7.5 줄)
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


def test_seek_first_pair_stops_at_distance_limit(monkeypatch):
    rclpy.init()
    node = None
    try:
        far = [(x, y) for x in (12.0, 14.0) for y in (-1.8, 1.8)]      # 3 m 가도 앞 창 밖
        node, clock, previews = _seek_node(monkeypatch, far, goal_x=16.0)
        _tick_at(node, clock, 1000.0)
        _tick_at(node, clock, 1002.5)
        assert node._status_text == 'SEEK_FIRST_PAIR'
        _tick_at(node, clock, 1003.0, x=3.1)                           # 시작 자리에서 3.1 m
        assert node._status_text == 'WAIT_GATE_PAIR' and previews[-1].linear.x == 0.0
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


def test_seek_first_pair_waits_when_a_post_is_just_ahead(monkeypatch):
    # 짝 없는 부표가 앞 1.2 m — 더 가면 첫 게이트선을 쌍 없이 넘을 수 있어 서서 기다림
    rclpy.init()
    node = None
    try:
        node, clock, previews = _seek_node(monkeypatch, [(1.2, 0.8), *FAR_GATES])
        _tick_at(node, clock, 1000.0)
        _tick_at(node, clock, 1002.5)
        assert node._status_text == 'WAIT_GATE_PAIR' and previews[-1].linear.x == 0.0
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


def test_seek_is_only_for_the_first_gate(monkeypatch):
    rclpy.init()
    node = None
    try:
        node, clock, previews = _seek_node(monkeypatch, FAR_GATES)
        node._passed_midpoints = [(-1.0, 0.0)]                          # 이미 한 게이트 통과
        node._passed_directions = [(1.0, 0.0)]
        _tick_at(node, clock, 1000.0)
        _tick_at(node, clock, 1003.0)
        assert node._status_text == 'WAIT_GATE_PAIR' and previews[-1].linear.x == 0.0
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


def test_seek_waits_when_posts_are_in_the_pair_window(monkeypatch):
    # 앞 창(6 m) 안에 부표가 보이는데 쌍이 안 됨(왼쪽 둘, 간격 1 m) → 다가가지 않고 기다림.
    # 보이는데 다가가면 게이트 1 이 쌍이 되기 전에 게이트 2 를 먼저 골라 첫 게이트를 건너뛰었다.
    rclpy.init()
    node = None
    try:
        node, clock, previews = _seek_node(monkeypatch, [(4.0, 2.5), (5.0, 2.6), *FAR_GATES])
        _tick_at(node, clock, 1000.0)
        _tick_at(node, clock, 1002.5)
        assert node._status_text == 'WAIT_GATE_PAIR' and previews[-1].linear.x == 0.0
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


def test_seek_leans_toward_posts_seen_beyond_the_window(monkeypatch):
    # 앞 창 너머(7.5 m) 왼쪽으로 치우친 게이트 → 그쪽으로 꺾어 다가감 (±30° 안)
    rclpy.init()
    node = None
    try:
        left_gates = [(x, y + 2.5) for x, y in FAR_GATES]
        node, clock, previews = _seek_node(monkeypatch, left_gates)
        _tick_at(node, clock, 1000.0)
        _tick_at(node, clock, 1002.5)
        assert node._status_text == 'SEEK_FIRST_PAIR'
        assert previews[-1].angular.z > 0.0                              # 왼쪽(부표 쪽)으로
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()
