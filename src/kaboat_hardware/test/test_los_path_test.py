"""The executable tank test can select both shared LOS path generators."""

import math

import numpy as np
import pytest
import rclpy
from nav_msgs.msg import Odometry

from kaboat_hardware.los_path_test import LosPathTest, make_path
from kaboat_hardware.test_coordinates import BSPLINE_TRACK


@pytest.mark.parametrize('kind', ['bspline', 'circle'])
def test_both_test_modes_build_safe_tank_paths(kind):
    path = make_path(kind, BSPLINE_TRACK['control_points'],
                     5.0, 2.5, 1.2, 'ccw', 2.0, 0.05)
    assert len(path.samples) > 50
    assert np.all((path.samples[:, 0] >= 0.25) & (path.samples[:, 0] <= 9.75))
    assert np.all((path.samples[:, 1] >= 0.25) & (path.samples[:, 1] <= 4.75))
    assert path.closed == (kind == 'circle')


def test_continuous_circle_and_invalid_mode():
    path = make_path('circle', [], 5.0, 2.5, 1.2, 'cw', 0.0, 0.05)
    assert path.closed and path.target_laps is None
    assert np.all(path.curvatures < 0)
    with pytest.raises(ValueError):
        make_path('unknown', [], 5.0, 2.5, 1.2, 'ccw', 2.0, 0.05)


@pytest.mark.parametrize('kind,x,y,yaw', [
    ('bspline', 8.5, 2.0, math.pi),
    ('circle', 6.2, 2.5, math.pi / 2),
])
def test_ros_node_previews_both_paths_without_motor_topic(kind, x, y, yaw):
    rclpy.init(args=['--ros-args', '-p', f'path_kind:={kind}'])
    node = None
    try:
        node = LosPathTest()
        sent = []
        node._send = sent.append
        node.count_publishers = lambda topic: 1
        odom = Odometry()
        odom.header.frame_id = 'odom'
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.orientation.z = math.sin(yaw / 2)
        odom.pose.pose.orientation.w = math.cos(yaw / 2)
        node._on_odom(odom)
        node.started = True
        node._tick()
        assert node.cmd_pub is None
        assert node.last_status == 'PREVIEW_FOLLOW'
        assert len(sent) == 1 and sent[0].linear.x > 0
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()
