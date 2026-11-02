"""ROS gate behavior uses LiDAR/DRI and requires three actual crossings."""

import math
import time

import numpy as np
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


def _follow_current_gate(node):
    cmd = node.compute_cmd()  # acquire path
    assert cmd.linear.x == 0.0  # preview does not drive /cmd/gate
    assert node._path is not None
    samples = node._path.samples.copy()
    for x, y in samples[::4]:
        _set_odom(node, x, y)
        node.compute_cmd()
        if node._path is None:
            return
    x, y = samples[-1]
    _set_odom(node, x, y)
    node.compute_cmd()


def test_three_pairs_pass_before_completion_and_output_stays_preview_only():
    rclpy.init()
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
    rclpy.init()
    node = None
    try:
        node = _ready_node()
        node.compute_cmd()
        assert node._pair is not None
        node._scan_hits = np.asarray([node._pair.left])
        cmd = node.compute_cmd()
        assert node._status_text == 'GATE_POSTS_NOT_VISIBLE'
        assert cmd.linear.x == cmd.angular.z == 0.0
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()
