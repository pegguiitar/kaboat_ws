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
        """오른쪽 벽 LiDAR pose (10, 2.5, 180deg)의 합성 LaserScan."""
        msg = LaserScan()
        msg.header.frame_id = 'shore_laser_frame'
        msg.header.stamp.sec = 1
        msg.angle_min = -math.pi
        msg.angle_max = math.pi
        # 실측 TG-50 스캔과 같은 약 2030개 빔으로 원거리 분해능도 확인한다.
        msg.angle_increment = 2.0 * math.pi / 2029
        msg.range_min = 0.08
        msg.range_max = 15.0
        beam_count = int(round(
            (msg.angle_max - msg.angle_min) / msg.angle_increment)) + 1
        ranges = np.full(beam_count, np.inf, dtype=float)

        for world_x, world_y in points:
            # R(180deg)^T @ (world - lidar_origin)
            lidar_x = 10.0 - world_x
            lidar_y = 2.5 - world_y
            angle = math.atan2(lidar_y, lidar_x)
            distance = math.hypot(lidar_x, lidar_y)
            index = int(round((angle - msg.angle_min) / msg.angle_increment))
            ranges[index] = distance

        msg.ranges = ranges.tolist()
        return msg

    @staticmethod
    def _panel_points(x=5.0, y=2.5, yaw=0.0, span=0.60):
        tangent = np.array([-math.sin(yaw), math.cos(yaw)])
        center = np.array([x, y])
        return [center + distance * tangent
                for distance in np.linspace(-span / 2, span / 2, 25)]

    def test_scan_to_pose_detects_panel_center_and_zero_heading(self):
        points = self._panel_points()
        self.node._on_scan(self._scan_with_world_points(points))

        self.assertEqual(len(self.poses), 1)
        pose = self.poses[0].pose.pose
        self.assertAlmostEqual(pose.position.x, 5.0, delta=0.01)
        self.assertAlmostEqual(pose.position.y, 2.5, delta=0.01)
        self.assertAlmostEqual(
            _yaw_from_quat(pose.orientation), 0.0, delta=0.01)

    def test_scan_to_pose_uses_line_slope_for_positive_heading(self):
        points = self._panel_points(yaw=math.pi / 4)
        self.node._on_scan(self._scan_with_world_points(points))

        self.assertEqual(len(self.poses), 1)
        pose = self.poses[0].pose.pose
        self.assertAlmostEqual(pose.position.x, 5.0, delta=0.01)
        self.assertAlmostEqual(pose.position.y, 2.5, delta=0.01)
        self.assertAlmostEqual(
            _yaw_from_quat(pose.orientation), math.pi / 4, delta=0.01)

    def test_short_reflection_is_not_a_panel(self):
        self.node._on_scan(self._scan_with_world_points(
            self._panel_points(span=0.20)))
        self.assertEqual(self.poses, [])

    def test_panel_is_detected_eight_meters_from_lidar(self):
        self.node._on_scan(self._scan_with_world_points(
            self._panel_points(x=2.0)))
        self.assertEqual(len(self.poses), 1)
        pose = self.poses[0].pose.pose
        self.assertAlmostEqual(pose.position.x, 2.0, delta=0.03)
        self.assertAlmostEqual(pose.position.y, 2.5, delta=0.03)
        self.assertAlmostEqual(_yaw_from_quat(pose.orientation), 0.0, delta=0.03)
