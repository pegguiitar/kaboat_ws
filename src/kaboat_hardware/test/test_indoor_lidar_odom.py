import math
import time
import unittest

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import Imu
from std_srvs.srv import Trigger

from kaboat_hardware.indoor_lidar_odom import (
    IndoorLidarOdom, _quat_from_yaw, _resolve_pi_ambiguous_yaw,
    _yaw_from_quat,
)
from kaboat_hardware.pose_velocity import normalize_angle


def _pose(x=5.0, y=2.5, yaw=0.0, stamp_sec=1):
    msg = PoseWithCovarianceStamped()
    msg.header.stamp.sec = stamp_sec
    msg.pose.pose.position.x = x
    msg.pose.pose.position.y = y
    msg.pose.pose.orientation = _quat_from_yaw(yaw)
    msg.pose.covariance[0] = 0.03 ** 2
    msg.pose.covariance[7] = 0.03 ** 2
    msg.pose.covariance[35] = math.radians(2.0) ** 2
    return msg


def _imu(yaw_rate=0.0, stamp_sec=1):
    msg = Imu()
    msg.header.stamp.sec = stamp_sec
    msg.angular_velocity.z = yaw_rate
    return msg


def test_quaternion_yaw_conversion():
    angles = [0.0, 0.5, math.pi / 2, math.pi - 0.01, -math.pi / 2, -2.5]
    for angle in angles:
        recovered = _yaw_from_quat(_quat_from_yaw(angle))
        assert math.isclose(
            normalize_angle(angle - recovered), 0.0, abs_tol=1e-6)


def test_resolve_pi_ambiguous_yaw_follows_imu_reference():
    raw_lidar_yaw = math.radians(2.0)
    imu_reference = math.radians(-179.0)

    resolved = _resolve_pi_ambiguous_yaw(raw_lidar_yaw, imu_reference)

    assert abs(normalize_angle(resolved - math.radians(-178.0))) < 1e-9


def test_resolve_pi_ambiguous_yaw_keeps_matching_branch():
    raw_lidar_yaw = math.radians(178.0)
    imu_reference = math.radians(179.0)

    resolved = _resolve_pi_ambiguous_yaw(raw_lidar_yaw, imu_reference)

    assert abs(normalize_angle(resolved - raw_lidar_yaw)) < 1e-9


class TestIndoorLidarOdomSafety(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        if not rclpy.ok():
            rclpy.init()

    @classmethod
    def tearDownClass(cls):
        if rclpy.ok():
            rclpy.shutdown()

    def setUp(self):
        self.node = IndoorLidarOdom()
        self.published = []
        self.node.odom_pub.publish = lambda msg: self.published.append(msg)
        self.node.publish_tf = False

    def tearDown(self):
        self.node.destroy_node()

    def test_does_not_publish_before_lidar_pose(self):
        self.node._on_imu(_imu())
        self.node._tick()
        self.assertEqual(self.published, [])

    def test_publishes_fused_pose_when_both_inputs_are_fresh(self):
        self.node._on_lidar_pose(_pose(x=4.2, y=1.7, yaw=1.1))
        self.node._on_imu(_imu(yaw_rate=0.02))
        self.node._tick()

        self.assertEqual(len(self.published), 1)
        odom = self.published[0]
        self.assertAlmostEqual(odom.pose.pose.position.x, 4.2)
        self.assertAlmostEqual(odom.pose.pose.position.y, 1.7)
        self.assertAlmostEqual(_yaw_from_quat(odom.pose.pose.orientation), 1.1)
        self.assertGreater(odom.pose.covariance[35], 0.0)

    def test_odom_angular_rate_comes_from_panel_not_imu(self):
        self.node._on_lidar_pose(_pose(yaw=0.0))
        self.node._on_imu(_imu(yaw_rate=0.8))
        self.node._tick()
        self.assertAlmostEqual(
            self.published[-1].twist.twist.angular.z, 0.0)

        self.node.last_lidar_yaw_correction_time = time.monotonic() - 0.1
        self.node._on_lidar_pose(_pose(yaw=0.1))
        self.node._tick()
        self.assertGreater(
            self.published[-1].twist.twist.angular.z, 0.1)

    def test_stale_imu_keeps_lidar_odom_and_previous_branch(self):
        self.node._on_lidar_pose(_pose())
        self.node._on_imu(_imu())
        self.node._tick()
        self.node.last_imu_receive_time = (
            time.monotonic() - self.node.imu_timeout - 0.1)
        self.node._tick()
        self.assertEqual(len(self.published), 2)
        self.assertAlmostEqual(
            _yaw_from_quat(self.published[-1].pose.pose.orientation), 0.0)

    def test_stale_lidar_stops_odom_until_reacquisition(self):
        transforms = []
        self.node.publish_tf = True
        self.node.tf_broadcaster.sendTransform = transforms.append
        self.node._on_lidar_pose(_pose())
        self.node._on_imu(_imu())
        self.node._tick()
        old_correction_time = time.monotonic() - 3.2
        self.node.last_lidar_yaw_correction_time = old_correction_time
        self.node.last_pose_receive_time = (
            time.monotonic() - 3.2)
        self.node._tick()
        self.node._tick()
        self.assertEqual(len(self.published), 1)
        self.assertEqual(len(transforms), 1)
        self.assertEqual(
            self.node.last_lidar_yaw_correction_time, old_correction_time)

        self.node._on_lidar_pose(_pose(x=6.2, yaw=0.2))
        self.node._tick()
        self.assertEqual(len(self.published), 2)
        self.assertEqual(len(transforms), 2)
        self.assertAlmostEqual(
            self.published[-1].pose.pose.position.x, 6.2, places=2)
        self.assertGreater(
            self.node.last_lidar_yaw_correction_time, old_correction_time)
        self.assertGreater(_yaw_from_quat(
            self.published[-1].pose.pose.orientation), 0.0)

    def test_rejected_lidar_yaw_does_not_delay_next_correction(self):
        self.node._on_lidar_pose(_pose(yaw=0.0))
        self.node._on_imu(_imu())
        old_correction_time = time.monotonic() - 3.2
        self.node.last_lidar_yaw_correction_time = old_correction_time

        self.node._on_lidar_pose(_pose(yaw=math.pi / 2.0))
        self.assertEqual(
            self.node.last_lidar_yaw_correction_time, old_correction_time)

        self.node._on_lidar_pose(_pose(yaw=0.1))
        self.assertGreater(
            self.node.last_lidar_yaw_correction_time, old_correction_time)

    def test_invalid_lidar_orientation_is_rejected(self):
        msg = _pose()
        msg.pose.pose.orientation.x = 0.0
        msg.pose.pose.orientation.y = 0.0
        msg.pose.pose.orientation.z = 0.0
        msg.pose.pose.orientation.w = 0.0
        self.node._on_lidar_pose(msg)
        self.assertIsNone(self.node.last_pose_receive_time)

    def test_lidar_180_flip_is_aligned_to_imu_heading(self):
        expected_yaw = math.radians(178.0)
        self.node.yaw_filter.initialize(
            expected_yaw, math.radians(2.0) ** 2)
        self.node.yaw_calibrated = True

        self.node._on_lidar_pose(_pose(yaw=math.radians(-2.0)))

        error = normalize_angle(self.node.yaw_filter.yaw - expected_yaw)
        self.assertAlmostEqual(error, 0.0, places=6)
        self.assertAlmostEqual(
            normalize_angle(
                self.node.last_aligned_lidar_yaw - expected_yaw),
            0.0, places=6)

    def test_each_lidar_panel_measurement_sets_odom_heading(self):
        initial_yaw = math.radians(10.0)
        self.node.yaw_filter.initialize(
            initial_yaw, math.radians(2.0) ** 2)
        self.node.yaw_calibrated = True
        self.node.last_lidar_yaw_correction_time = time.monotonic()

        self.node._on_lidar_pose(_pose(yaw=0.0))
        self.assertAlmostEqual(
            self.node.yaw_filter.yaw, 0.0, places=6)

        measured_yaw = math.radians(12.0)
        self.node._on_lidar_pose(_pose(x=6.0, yaw=measured_yaw))
        self.node._on_imu(_imu(yaw_rate=0.0))
        self.node._tick()

        self.assertAlmostEqual(self.node.yaw_filter.yaw, measured_yaw, places=6)
        self.assertAlmostEqual(
            self.published[-1].pose.pose.position.x, 6.0, places=6)
        self.assertAlmostEqual(
            _yaw_from_quat(self.published[-1].pose.pose.orientation),
            measured_yaw, places=6)

    def test_stale_lidar_reacquisition_does_not_reset_imu_yaw(self):
        imu_yaw = math.radians(35.0)
        self.node.yaw_filter.initialize(imu_yaw, math.radians(2.0) ** 2)
        self.node.yaw_calibrated = True
        self.node.last_pose_receive_time = (
            time.monotonic() - self.node.pos_timeout - 0.1)
        self.node.last_lidar_yaw_correction_time = time.monotonic()

        self.node._on_lidar_pose(_pose(yaw=normalize_angle(
            imu_yaw + math.pi)))

        self.assertAlmostEqual(
            normalize_angle(self.node.yaw_filter.yaw - imu_yaw),
            0.0, places=6)

    def test_calibration_estimates_bias_and_uses_lidar_yaw(self):
        lidar_yaw = 2.4
        self.node._on_lidar_pose(_pose(yaw=lidar_yaw))
        now = time.monotonic()
        duration = self.node.calibration_min_duration_sec + 0.01
        sample_count = self.node.calibration_min_samples + 1
        self.node._gyro_samples.clear()
        for index in range(sample_count):
            stamp = now - duration + index * duration / (sample_count - 1)
            self.node._gyro_samples.append((stamp, 0.012))
        self.node.last_imu_receive_time = now

        response = self.node._on_calibrate_imu_yaw(
            Trigger.Request(), Trigger.Response())

        self.assertTrue(response.success, response.message)
        self.assertTrue(self.node.yaw_calibrated)
        self.assertAlmostEqual(self.node.yaw_filter.bias, 0.012, places=6)
        self.assertAlmostEqual(self.node.yaw_filter.yaw, lidar_yaw, places=6)

    def test_calibration_rejects_rotating_boat(self):
        self.node._on_lidar_pose(_pose())
        now = time.monotonic()
        duration = self.node.calibration_min_duration_sec + 0.01
        sample_count = self.node.calibration_min_samples + 1
        self.node._gyro_samples.clear()
        for index in range(sample_count):
            rate = 0.04 if index % 2 else -0.04
            stamp = now - duration + index * duration / (sample_count - 1)
            self.node._gyro_samples.append((stamp, rate))
        self.node.last_imu_receive_time = now

        response = self.node._on_calibrate_imu_yaw(
            Trigger.Request(), Trigger.Response())

        self.assertFalse(response.success)
        self.assertIn('회전 감지', response.message)

    def test_calibration_uses_180_degree_branch_at_start(self):
        self.node._on_lidar_pose(_pose(yaw=0.0))
        now = time.monotonic()
        duration = self.node.calibration_min_duration_sec + 0.01
        sample_count = self.node.calibration_min_samples + 1
        self.node._gyro_samples.clear()
        for index in range(sample_count):
            stamp = now - duration + index * duration / (sample_count - 1)
            self.node._gyro_samples.append((stamp, 0.0))
        self.node.last_imu_receive_time = now

        response = self.node._on_calibrate_imu_yaw(
            Trigger.Request(), Trigger.Response())

        self.assertTrue(response.success, response.message)
        self.assertAlmostEqual(
            abs(self.node.yaw_filter.yaw), math.pi, places=6)

    def test_calibration_rejects_heading_far_from_180_degree_reference(self):
        self.node._on_lidar_pose(_pose(yaw=math.pi / 2.0))
        now = time.monotonic()
        duration = self.node.calibration_min_duration_sec + 0.01
        sample_count = self.node.calibration_min_samples + 1
        self.node._gyro_samples.clear()
        for index in range(sample_count):
            stamp = now - duration + index * duration / (sample_count - 1)
            self.node._gyro_samples.append((stamp, 0.0))
        self.node.last_imu_receive_time = now

        response = self.node._on_calibrate_imu_yaw(
            Trigger.Request(), Trigger.Response())

        self.assertFalse(response.success)
        self.assertIn('선수를 -X 방향', response.message)
