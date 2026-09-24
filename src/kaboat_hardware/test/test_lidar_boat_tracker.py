import math
import unittest

import numpy as np
import rclpy
from sensor_msgs.msg import LaserScan

from kaboat_hardware.indoor_lidar_odom import _yaw_from_quat
from kaboat_hardware.lidar_boat_tracker import LidarBoatTracker


class TestLidarBoatTrackerPipeline(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        if not rclpy.ok():
            rclpy.init()

    @classmethod
    def tearDownClass(cls):
        if rclpy.ok():
            rclpy.shutdown()

    def setUp(self):
        self.node = LidarBoatTracker()
        self.poses = []
        self.node.pose_cov_pub.publish = lambda msg: self.poses.append(msg)

    def tearDown(self):
        self.node.destroy_node()

    @staticmethod
    def _scan_with_world_points(points):
        """기본 LiDAR pose (5, 0, +90deg)에 맞춘 합성 LaserScan을 만든다."""
        msg = LaserScan()
        msg.header.frame_id = 'laser_frame'
        msg.header.stamp.sec = 1
        msg.angle_min = -math.pi
        msg.angle_increment = 0.0005
        msg.angle_max = math.pi
        msg.range_min = 0.08
        msg.range_max = 15.0
        beam_count = int(round(
            (msg.angle_max - msg.angle_min) / msg.angle_increment)) + 1
        ranges = np.full(beam_count, np.inf, dtype=float)

        for world_x, world_y in points:
            # world_delta = R(+90deg) @ lidar_point
            lidar_x = world_y
            lidar_y = -(world_x - 5.0)
            angle = math.atan2(lidar_y, lidar_x)
            distance = math.hypot(lidar_x, lidar_y)
            index = int(round((angle - msg.angle_min) / msg.angle_increment))
            ranges[index] = distance

        msg.ranges = ranges.tolist()
        return msg

    def test_scan_to_pose_detects_one_meter_bow_stern_pair(self):
        # 선수 얇은 봉: 6cm 폭, 선미 두꺼운 봉: 18cm 폭.
        points = [
            (5.5, 2.47), (5.5, 2.50), (5.5, 2.53),
            (4.5, 2.41), (4.5, 2.50), (4.5, 2.59),
        ]
        self.node._on_scan(self._scan_with_world_points(points))

        self.assertEqual(len(self.poses), 1)
        pose = self.poses[0].pose.pose
        self.assertAlmostEqual(pose.position.x, 5.0, delta=0.01)
        self.assertAlmostEqual(pose.position.y, 2.5, delta=0.01)
        self.assertAlmostEqual(_yaw_from_quat(pose.orientation), 0.0, delta=0.01)
