"""occupancy_grid 카메라 층: 라이다가 지우지 못하고, 보이면 감쇠, 시야 밖이면 유지."""

import math

import numpy as np
import pytest
import rclpy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

from kaboat_perception.occupancy_grid import OccupancyGridNode


@pytest.fixture
def node():
    rclpy.init(args=['--ros-args', '-p', 'cam_enabled:=true', '-p', 'enable_tilt_gate:=false'])
    n = OccupancyGridNode()
    n.sent = []
    n.pub.publish = n.sent.append
    n.cam_K, n.cam_width, n.roll_pitch = (380.0, 380.0, 320.0, 240.0), 640, (0.0, 0.0)
    yield n
    n.destroy_node()
    rclpy.shutdown()


def _set_pose(n, yaw=0.0):
    odom = Odometry()
    odom.header.frame_id = 'odom'
    odom.pose.pose.orientation.z = math.sin(yaw / 2)
    odom.pose.pose.orientation.w = math.cos(yaw / 2)
    n._on_odom(odom)


def _value_at(n, x, y):
    n._publish()
    g = n.sent[-1]
    ix = int((x - g.info.origin.position.x) / g.info.resolution)
    iy = int((y - g.info.origin.position.y) / g.info.resolution)
    return g.data[iy * g.info.width + ix]


BOOM = np.array([[3.1, 0.1, 0.1]] * 5)          # 한 칸에 색 점 5개 (붐)


def test_camera_boom_survives_lidar_and_decays_only_when_seen(node):
    _set_pose(node)
    node._add_camera_points(BOOM)
    assert _value_at(node, 3.1, 0.1) < 95                    # 한 프레임은 회피 문턱 아래
    for _ in range(4):
        node._add_camera_points(BOOM)
    assert _value_at(node, 3.1, 0.1) >= 95

    scan = LaserScan()                                       # 붐 위를 지나 7 m 앞까지 가는 빔
    scan.angle_min, scan.angle_increment = 0.0, 0.01
    scan.range_min, scan.range_max, scan.ranges = 0.1, 40.0, [7.0]
    for _ in range(10):
        node._on_scan(scan)
    assert node.grid[node.cam_grid > 0].max() < 0            # 라이다 층은 free 로 봤지만
    assert _value_at(node, 3.1, 0.1) >= 95                   # 발행값은 카메라 층이 지킴

    _set_pose(node, math.pi)                                 # 돌아서 붐이 시야 밖
    for _ in range(20):
        node._add_camera_points(np.empty((0, 3)))
    assert _value_at(node, 3.1, 0.1) >= 95                   # 안 보이면 잊지 않음

    _set_pose(node)                                          # 다시 보는데 붐이 없음
    for _ in range(12):
        node._add_camera_points(np.empty((0, 3)))
    assert _value_at(node, 3.1, 0.1) < 95


def test_camera_defaults_ten_hz_and_three_frames_make_an_obstacle(node):
    # 2026-11-12 현장: 붐이 가끔만 잡혀 5 Hz → 10 Hz, 히트 0.8 → 1.0 (감쇠 -0.4·칸당 3점은 그대로)
    assert node.cam_period == pytest.approx(0.1)
    assert (node.cam_l_occ, node.cam_l_free, node.cam_min_points) == (1.0, -0.4, 3)
    _set_pose(node)
    for _ in range(2):
        node._add_camera_points(BOOM)
    assert _value_at(node, 3.1, 0.1) < 95                    # 2프레임은 아직 문턱 아래
    node._add_camera_points(BOOM)
    assert _value_at(node, 3.1, 0.1) >= 95                   # 3프레임(10 Hz 에서 0.3 s)이면 장애물


def test_camera_topics_use_best_effort_so_realsense_connects(node):
    from rclpy.qos import ReliabilityPolicy
    subs = {s.topic_name: s.qos_profile.reliability for s in node.subscriptions}
    assert subs['/camera/camera_info'] == ReliabilityPolicy.BEST_EFFORT
    for topic in ('/camera/color/image_raw', '/camera/depth/image_raw'):
        assert subs[topic] == ReliabilityPolicy.BEST_EFFORT


def test_sparse_points_below_min_count_are_ignored(node):
    _set_pose(node)
    for _ in range(10):
        node._add_camera_points(BOOM[:2])                    # 칸당 2점 < cam_min_points 3
    assert not (node.cam_grid > 0).any()


def test_camera_layer_off_by_default():
    rclpy.init()
    try:
        n = OccupancyGridNode()
        assert not n.cam_enabled and n.cam_points_pub is None
        n.destroy_node()
    finally:
        rclpy.shutdown()
