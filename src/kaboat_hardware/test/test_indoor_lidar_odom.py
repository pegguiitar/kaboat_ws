import math
import time
import unittest

import rclpy
from geometry_msgs.msg import PointStamped, Quaternion
from sensor_msgs.msg import Imu
from std_srvs.srv import Trigger

from kaboat_hardware.indoor_lidar_odom import (
    IndoorLidarOdom, _calibration_offset, _yaw_from_quat, _quat_from_yaw,
    normalize_angle
)


def test_quaternion_yaw_conversion():
    angles = [0.0, 0.5, math.pi / 2, math.pi - 0.01, -math.pi / 2, -2.5]
    for angle in angles:
        q = _quat_from_yaw(angle)
        recovered = _yaw_from_quat(q)
        err = normalize_angle(angle - recovered)
        assert math.isclose(err, 0.0, abs_tol=1e-6)


def test_normalize_angle():
    assert math.isclose(abs(normalize_angle(3 * math.pi)), math.pi, abs_tol=1e-6)
    assert math.isclose(abs(normalize_angle(-3 * math.pi)), math.pi, abs_tol=1e-6)
    assert math.isclose(normalize_angle(math.radians(370)), math.radians(10), abs_tol=1e-6)


def test_calibration_offset_handles_wraparound_and_targets_minus_x():
    raw_yaws = [math.radians(179.0), math.radians(-179.5), math.radians(180.0)]
    mean_yaw, offset, max_deviation = _calibration_offset(raw_yaws, math.pi)

    corrected = normalize_angle(mean_yaw + offset)
    assert math.isclose(abs(corrected), math.pi, abs_tol=1e-6)
    assert max_deviation < math.radians(2.0)


def test_resume_behavior():
    """위치 신호 유실 후 재수신 시 초기화 및 복구 검증."""
    from kaboat_hardware.pose_velocity import VelocityEstimator, VelocityParams
    est = VelocityEstimator(VelocityParams(filter_tau=0.15))

    # 1. 초기 10개 정상 수신 (dt=0.1s, vx=1.0m/s)
    for i in range(10):
        t = i * 0.1
        vx, vy, _ = est.update(t, t * 1.0, 0.0, 0.0)

    assert math.isclose(vx, 1.0, abs_tol=0.1)

    # 2. 3초간 신호 단절 (gap)
    # 복구 시 estimator.reset()을 호출하지 않으면 3초 동안의 위치 차분으로 비정상적인 거대 속도가 계산됨
    est.reset()

    # 3. 신호 복구 후 첫 샘플 (t=4.0s)
    vx_rec, vy_rec, _ = est.update(4.0, 4.0, 0.0, 0.0)
    assert vx_rec == 0.0 and vy_rec == 0.0  # reset 덕분에 거대 스파이크 없이 안전하게 0부터 시작

    # 4. 몇 샘플 후 정상 속도로 EMA 수렴 (스파이크 없이 부드러운 복구)
    for i in range(1, 6):
        vx_rec, vy_rec, _ = est.update(4.0 + i * 0.1, 4.0 + i * 0.1, 0.0, 0.0)
    assert math.isclose(vx_rec, 1.0, abs_tol=0.1)


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

    @staticmethod
    def _imu(stamp_sec=1):
        msg = Imu()
        msg.header.stamp.sec = stamp_sec
        msg.orientation.w = 1.0
        return msg

    @staticmethod
    def _position(x=5.0, y=2.5, stamp_sec=1):
        msg = PointStamped()
        msg.header.stamp.sec = stamp_sec
        msg.point.x = x
        msg.point.y = y
        return msg

    def test_does_not_publish_fake_origin_before_position(self):
        self.node._on_imu(self._imu())
        self.node._tick()
        self.assertEqual(self.published, [])

    def test_publishes_only_when_position_and_imu_are_fresh(self):
        self.node._on_imu(self._imu())
        self.node._on_position(self._position())
        self.node._tick()
        self.assertEqual(len(self.published), 1)
        self.assertAlmostEqual(self.published[0].pose.pose.position.x, 5.0)
        self.assertAlmostEqual(self.published[0].pose.pose.position.y, 2.5)

    def test_stale_imu_stops_odom(self):
        self.node._on_imu(self._imu())
        self.node._on_position(self._position())
        self.node._tick()
        self.node._on_position(self._position(x=4.9, stamp_sec=2))
        self.node.last_imu_receive_time = time.monotonic() - self.node.imu_timeout - 0.1
        self.node._tick()
        self.assertEqual(len(self.published), 1)

    def test_stale_position_stops_odom(self):
        self.node._on_imu(self._imu())
        self.node._on_position(self._position())
        self.node._tick()
        self.node.last_pos_receive_time = time.monotonic() - self.node.pos_timeout - 0.1
        self.node._tick()
        self.assertEqual(len(self.published), 1)

    def test_invalid_orientation_is_not_marked_fresh(self):
        msg = self._imu()
        msg.orientation_covariance[0] = -1.0
        self.node._on_imu(msg)
        self.assertIsNone(self.node.last_imu_receive_time)

    def test_calibration_sets_minus_x_heading(self):
        now = time.monotonic()
        raw_yaw = math.radians(37.0)
        self.node._imu_yaw_samples.clear()
        for index in range(self.node.calibration_min_samples):
            stamp = now - self.node.calibration_min_duration_sec + (
                index * self.node.calibration_min_duration_sec /
                (self.node.calibration_min_samples - 1))
            self.node._imu_yaw_samples.append((stamp, raw_yaw, 0.0))
        self.node.raw_imu_yaw = raw_yaw
        self.node.last_imu_receive_time = now

        response = self.node._on_calibrate_imu_yaw(
            Trigger.Request(), Trigger.Response())

        self.assertTrue(response.success, response.message)
        self.assertTrue(self.node.yaw_calibrated)
        self.assertAlmostEqual(abs(self.node.imu_yaw), math.pi, places=6)

    def test_calibration_rejects_motion(self):
        now = time.monotonic()
        self.node._imu_yaw_samples.clear()
        self.node.last_imu_receive_time = now
        for index in range(self.node.calibration_min_samples):
            stamp = now - self.node.calibration_min_duration_sec + (
                index * self.node.calibration_min_duration_sec /
                (self.node.calibration_min_samples - 1))
            self.node._imu_yaw_samples.append((stamp, 0.5, 0.2))

        response = self.node._on_calibrate_imu_yaw(
            Trigger.Request(), Trigger.Response())

        self.assertFalse(response.success)
        self.assertIn('움직임', response.message)

    def test_calibration_rejects_stale_imu(self):
        now = time.monotonic()
        self.node.last_imu_receive_time = now - self.node.imu_timeout - 0.1
        self.node._imu_yaw_samples.append((now - 0.1, 0.5, 0.0))

        response = self.node._on_calibrate_imu_yaw(
            Trigger.Request(), Trigger.Response())

        self.assertFalse(response.success)
        self.assertIn('최신 IMU', response.message)
