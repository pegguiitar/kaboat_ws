"""Dock completion uses the route that was actually fixed by LiDAR/GPS."""

import pytest

rclpy = pytest.importorskip('rclpy')
from rclpy.parameter import Parameter
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String

from kaboat_mission.mission_manager import MissionManager


def test_dock_completion_waits_for_fixed_goal():
    rclpy.init()
    node = MissionManager(parameter_overrides=[
        Parameter('auto_start', value=False),
        Parameter('start_state', value='dock'),
        Parameter('single_mission', value=True),
        Parameter('entry_required', value=False),
        Parameter('points_file', value='none'),
    ])
    try:
        node.advance()
        assert node.phase == 'mission'
        odom = Odometry()
        odom.pose.pose.position.x = 69.0
        odom.pose.pose.position.y = 66.0
        node.on_odom(odom)
        node.on_complete(String(data='dock'))
        node.tick()
        assert node.phase == 'mission'

        path = Path()
        path.header.frame_id = 'odom'
        pose = PoseStamped()
        pose.pose.position.x = 69.0
        pose.pose.position.y = 66.0
        path.poses.append(pose)
        node.on_dock_path(path)
        empty = Path()
        empty.header.frame_id = 'odom'
        node.on_dock_path(empty)
        assert not node.behavior_complete
        node.on_dock_path(path)
        node.on_complete(String(data='dock'))
        node.tick()
        assert node.phase == 'done'
    finally:
        node.destroy_node()
        rclpy.shutdown()
