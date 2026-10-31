import math
import unittest

import numpy as np
import rclpy
from sensor_msgs.msg import LaserScan

from kaboat_hardware.indoor_lidar_odom import _yaw_from_quat
from kaboat_hardware.lidar_boat_tracker import LidarBoatTracker
from kaboat_hardware.lidar_sail_pose import SailGeometry, raycast
from kaboat_hardware.pose_velocity import normalize_angle


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
    def _scan_of_boat(x=5.0, y=2.5, yaw=0.0, beam_count=2030, geom=None):
        """오른쪽 벽 LiDAR pose (10, 2.5, 180deg)에서 ┛ 판을 본 합성 LaserScan.

        판에 닿지 않는 빔은 12 m 뒤 벽에 닿는다.
        """
        geom = geom or SailGeometry()
        msg = LaserScan()
        msg.header.frame_id = 'shore_laser_frame'
        msg.header.stamp.sec = 1
        msg.angle_min = -math.pi
        msg.angle_max = math.pi
        # TG-50은 약 2030빔, 수조 외벽 RPLIDAR A3M1은 실측 720빔이다.
        msg.angle_increment = 2.0 * math.pi / (beam_count - 1)
        msg.range_min = 0.08
        msg.range_max = 15.0
        angles = msg.angle_min + np.arange(beam_count) * msg.angle_increment
        ranges = raycast(np.array([10.0, 2.5]), angles + math.pi,
                         geom.segments(x, y, yaw))
        msg.ranges = np.where(np.isfinite(ranges), ranges, 12.0).tolist()
        return msg

    def _assert_pose(self, x, y, yaw, pos_tol=0.03, yaw_tol=0.03):
        self.assertEqual(len(self.poses), 1)
        pose = self.poses[0].pose.pose
        self.assertAlmostEqual(pose.position.x, x, delta=pos_tol)
        self.assertAlmostEqual(pose.position.y, y, delta=pos_tol)
        self.assertLess(abs(normalize_angle(
            _yaw_from_quat(pose.orientation) - yaw)), yaw_tol)

    def test_scan_to_pose_detects_base_link_facing_default_heading(self):
        # 선수가 기본 방향(-X)일 때. 정면 반대(+X)에서는 세로판이 횡단 판 뒤에
        # 가려져 스캔만으로 앞뒤를 가를 수 없고 기본 방향을 따른다.
        self.node._on_scan(self._scan_of_boat(yaw=math.pi))
        self._assert_pose(5.0, 2.5, math.pi)

    def test_corner_gives_full_heading(self):
        # 두 판이 모두 보이는 각도에서는 앞뒤까지 정해진다.
        for yaw in (math.radians(135), math.radians(-120)):
            self.poses.clear()
            self.node.last_raw_pose = None
            self.node._on_scan(self._scan_of_boat(yaw=yaw))
            self._assert_pose(5.0, 2.5, yaw)

    def test_short_reflection_is_not_a_panel(self):
        self.node._on_scan(self._scan_of_boat(
            geom=SailGeometry(left=0.05, right=0.05, sail=0.05)))
        self.assertEqual(self.poses, [])

    def test_a3_720_beam_scan_detects_boat_eight_meters_away(self):
        # 선수가 기본 방향(-X)일 때: 먼 거리에서 앞뒤가 모호해도 기본값으로 맞게 고른다.
        scan = self._scan_of_boat(x=2.0, yaw=math.pi, beam_count=720)
        scan.range_min = 0.05
        scan.range_max = 25.0
        scan.intensities = [0.0] * 720
        self.node._on_scan(scan)
        self._assert_pose(2.0, 2.5, math.pi, pos_tol=0.07, yaw_tol=0.07)

    def test_reacquire_after_loss_follows_last_yaw(self):
        # 기본 방향(-X)과 반대로 서 있어도, 놓치기 직전 yaw 가 있으면 그쪽을 고른다.
        self.node.last_raw_pose = (2.0, 2.5, math.radians(10))
        self.node.last_seen_time = self.node.get_clock().now()
        self.node.target_lost_count = 10
        self.node._on_scan(self._scan_of_boat(x=2.0, yaw=0.0, beam_count=720))
        self._assert_pose(2.0, 2.5, 0.0, pos_tol=0.07, yaw_tol=0.07)
