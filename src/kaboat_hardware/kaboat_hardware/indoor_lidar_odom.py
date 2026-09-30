"""외부 LiDAR 위치와 선체 IMU gyro를 융합해 실내 `/odom`을 발행한다.

IMU gyro-z 적분값이 연속적인 yaw와 회전 방향을 유지한다. LiDAR 두 봉의 구분이
뒤집혀도 180도 점프하지 않도록 LiDAR yaw와 그 반대 방향 중 IMU 예측값에 가까운
쪽을 선택하고, 정해진 주기마다 절대 yaw와 gyro bias를 보정한다. GQ7 quaternion
yaw는 절대 heading으로 사용하지 않는다.
"""

import math
import time
from collections import deque
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import PoseWithCovarianceStamped, Quaternion, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_srvs.srv import Trigger
from tf2_ros import TransformBroadcaster

from kaboat_hardware.pose_velocity import (
    VelocityEstimator, VelocityParams, normalize_angle,
)
from kaboat_hardware.yaw_bias_ekf import YawBiasEkf


def _yaw_from_quat(q: Quaternion) -> float:
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def _quat_from_yaw(yaw: float) -> Quaternion:
    q = Quaternion()
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


def _stamp_sec(stamp) -> float:
    return stamp.sec + stamp.nanosec * 1e-9


def _resolve_pi_ambiguous_yaw(measured_yaw: float, reference_yaw: float) -> float:
    """두 봉 순서가 뒤집힌 yaw를 reference에 가장 가까운 방향으로 정렬한다."""
    first = normalize_angle(float(measured_yaw))
    opposite = normalize_angle(first + math.pi)
    first_error = abs(normalize_angle(first - reference_yaw))
    opposite_error = abs(normalize_angle(opposite - reference_yaw))
    return first if first_error <= opposite_error else opposite


class IndoorLidarOdom(Node):
    def __init__(self):
        super().__init__('indoor_lidar_odom')

        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('pose_topic', '/boat_pose')
        self.declare_parameter('imu_topic', '/imu/data')
        self.declare_parameter('publish_tf', True)

        self.declare_parameter('pos_timeout_sec', 1.0)
        self.declare_parameter('imu_timeout_sec', 0.5)
        self.declare_parameter('publish_rate', 30.0)
        self.declare_parameter('yaw_rate_sign', 1.0)
        self.declare_parameter('gyro_bias_z', 0.0)

        # LiDAR yaw + gyro bias EKF
        self.declare_parameter('gyro_noise_stddev', 0.01)
        self.declare_parameter('gyro_bias_walk_stddev', 0.001)
        self.declare_parameter('initial_gyro_bias_stddev', 0.05)
        self.declare_parameter('max_gyro_bias_abs', 0.20)
        self.declare_parameter('max_imu_predict_dt', 0.20)
        self.declare_parameter('lidar_yaw_stddev_deg', 2.0)
        self.declare_parameter('lidar_yaw_gate_deg', 45.0)
        self.declare_parameter('lidar_yaw_correction_interval_sec', 3.0)

        # 자이로만으로는 최초 앞뒤를 알 수 없으므로 시작 때 -X 근처에서 분기를 확정한다.
        self.declare_parameter('require_yaw_calibration', False)
        self.declare_parameter('calibration_reference_yaw_deg', 180.0)
        self.declare_parameter('calibration_reference_tolerance_deg', 45.0)
        self.declare_parameter('calibration_window_sec', 2.0)
        self.declare_parameter('calibration_min_duration_sec', 1.5)
        self.declare_parameter('calibration_min_samples', 30)
        self.declare_parameter('calibration_max_rate_deviation', 0.02)
        self.declare_parameter('calibration_max_abs_bias', 0.10)

        self.declare_parameter('vel_window_sec', 0.15)
        self.declare_parameter('vel_filter_tau', 0.15)
        self.declare_parameter('vel_max_speed', 3.0)
        self.declare_parameter('pose_stddev_xy', 0.03)
        self.declare_parameter('twist_stddev', 0.05)

        self.odom_frame = str(self.get_parameter('odom_frame').value)
        self.base_frame = str(self.get_parameter('base_frame').value)
        self.pose_topic = str(self.get_parameter('pose_topic').value)
        self.imu_topic = str(self.get_parameter('imu_topic').value)
        self.publish_tf = bool(self.get_parameter('publish_tf').value)
        self.pos_timeout = float(self.get_parameter('pos_timeout_sec').value)
        self.imu_timeout = float(self.get_parameter('imu_timeout_sec').value)
        self.yaw_rate_sign = float(self.get_parameter('yaw_rate_sign').value)
        self.max_predict_dt = float(
            self.get_parameter('max_imu_predict_dt').value)
        self.default_lidar_yaw_variance = math.radians(float(
            self.get_parameter('lidar_yaw_stddev_deg').value)) ** 2
        self.lidar_yaw_gate = math.radians(float(
            self.get_parameter('lidar_yaw_gate_deg').value))
        self.lidar_yaw_correction_interval = max(0.0, float(
            self.get_parameter('lidar_yaw_correction_interval_sec').value))

        initial_bias = float(self.get_parameter('gyro_bias_z').value)
        self.yaw_filter = YawBiasEkf(
            gyro_noise_stddev=float(
                self.get_parameter('gyro_noise_stddev').value),
            bias_walk_stddev=float(
                self.get_parameter('gyro_bias_walk_stddev').value),
            initial_bias=initial_bias,
            initial_bias_stddev=float(
                self.get_parameter('initial_gyro_bias_stddev').value),
            max_bias_abs=float(
                self.get_parameter('max_gyro_bias_abs').value))

        self.require_yaw_calibration = bool(
            self.get_parameter('require_yaw_calibration').value)
        self.yaw_calibrated = not self.require_yaw_calibration
        self.calibration_reference_yaw = math.radians(float(
            self.get_parameter('calibration_reference_yaw_deg').value))
        self.calibration_reference_tolerance = math.radians(max(0.0, float(
            self.get_parameter('calibration_reference_tolerance_deg').value)))
        self.calibration_window_sec = max(0.1, float(
            self.get_parameter('calibration_window_sec').value))
        self.calibration_min_duration_sec = max(0.0, float(
            self.get_parameter('calibration_min_duration_sec').value))
        self.calibration_min_samples = max(2, int(
            self.get_parameter('calibration_min_samples').value))
        self.calibration_max_rate_deviation = max(0.0, float(
            self.get_parameter('calibration_max_rate_deviation').value))
        self.calibration_max_abs_bias = max(0.0, float(
            self.get_parameter('calibration_max_abs_bias').value))

        self.estimator = VelocityEstimator(VelocityParams(
            window_sec=float(self.get_parameter('vel_window_sec').value),
            filter_tau=float(self.get_parameter('vel_filter_tau').value),
            max_speed=float(self.get_parameter('vel_max_speed').value)))

        self.last_pose_stamp: Optional[float] = None
        self.last_pose_receive_time: Optional[float] = None
        self.last_pose_x = 0.0
        self.last_pose_y = 0.0
        self.last_lidar_yaw: Optional[float] = None
        self.last_aligned_lidar_yaw: Optional[float] = None
        self.last_lidar_yaw_variance = self.default_lidar_yaw_variance
        self.last_lidar_yaw_correction_time: Optional[float] = None
        default_position_variance = float(
            self.get_parameter('pose_stddev_xy').value) ** 2
        self.position_variance_x = default_position_variance
        self.position_variance_y = default_position_variance
        self.vx_body = 0.0
        self.vy_body = 0.0

        self.last_imu_receive_time: Optional[float] = None
        self.latest_raw_yaw_rate = 0.0
        self._gyro_samples = deque()  # (monotonic receive time, signed raw gyro-z)

        self._pose_ok = False
        self._imu_ok = False

        self.tf_broadcaster = TransformBroadcaster(self)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.create_service(
            Trigger, '/calibrate_imu_yaw', self._on_calibrate_imu_yaw)
        self.create_subscription(
            PoseWithCovarianceStamped, self.pose_topic,
            self._on_lidar_pose, qos_profile_sensor_data)
        self.create_subscription(
            Imu, self.imu_topic, self._on_imu, qos_profile_sensor_data)

        publish_rate = max(1.0, float(
            self.get_parameter('publish_rate').value))
        self.create_timer(1.0 / publish_rate, self._tick)

        self.get_logger().info(
            '🚀 [indoor_lidar_odom] LiDAR yaw + IMU gyro EKF 시작\n'
            f"   - LiDAR pose: '{self.pose_topic}'\n"
            f"   - IMU gyro: '{self.imu_topic}'\n"
            f'   - 초기 gyro bias: {initial_bias:.5f} rad/s\n'
            f'   - LiDAR yaw 보정 주기: '
            f'{self.lidar_yaw_correction_interval:.2f}s\n'
            f'   - 시작 정지 보정 필수: {self.require_yaw_calibration}\n'
            f"   - 발행: /odom, TF '{self.odom_frame}' -> '{self.base_frame}'")

    def _on_lidar_pose(self, msg: PoseWithCovarianceStamped):
        q = msg.pose.pose.orientation
        values = (
            msg.pose.pose.position.x, msg.pose.pose.position.y,
            q.x, q.y, q.z, q.w)
        norm_sq = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w
        if not all(math.isfinite(value) for value in values) or norm_sq <= 0.25:
            self.get_logger().warn(
                '유효하지 않은 /boat_pose를 거부했습니다.',
                throttle_duration_sec=2.0)
            return

        now = time.monotonic()
        was_stale = (
            self.last_pose_receive_time is None
            or now - self.last_pose_receive_time > self.pos_timeout)
        x = float(msg.pose.pose.position.x)
        y = float(msg.pose.pose.position.y)
        lidar_yaw = _yaw_from_quat(q)
        yaw_variance = float(msg.pose.covariance[35])
        if not math.isfinite(yaw_variance) or yaw_variance <= 0.0:
            yaw_variance = self.default_lidar_yaw_variance

        if not self.yaw_filter.initialized:
            self.yaw_filter.initialize(lidar_yaw, yaw_variance)
            if self.yaw_calibrated:
                self.last_aligned_lidar_yaw = lidar_yaw
                self.last_lidar_yaw_correction_time = now
        elif self.yaw_calibrated:
            aligned_lidar_yaw = _resolve_pi_ambiguous_yaw(
                lidar_yaw, self.yaw_filter.yaw)
            correction_due = (
                self.last_lidar_yaw_correction_time is None
                or now - self.last_lidar_yaw_correction_time
                >= self.lidar_yaw_correction_interval)
            if correction_due:
                self.last_lidar_yaw_correction_time = now
                if self.yaw_filter.update_yaw(
                        aligned_lidar_yaw, yaw_variance,
                        self.lidar_yaw_gate):
                    self.last_aligned_lidar_yaw = aligned_lidar_yaw
                else:
                    innovation_deg = math.degrees(normalize_angle(
                        aligned_lidar_yaw - self.yaw_filter.yaw))
                    self.get_logger().warn(
                        'IMU 기준으로 정렬한 LiDAR yaw innovation이 gate를 '
                        f'넘어 갱신을 거부했습니다 ({innovation_deg:.1f}°).',
                        throttle_duration_sec=2.0)

        source_stamp = _stamp_sec(msg.header.stamp)
        self.last_pose_stamp = source_stamp if source_stamp > 0.0 else now
        self.last_pose_receive_time = now
        self.last_pose_x = x
        self.last_pose_y = y
        self.last_lidar_yaw = lidar_yaw
        self.last_lidar_yaw_variance = yaw_variance
        covariance_x = float(msg.pose.covariance[0])
        covariance_y = float(msg.pose.covariance[7])
        if math.isfinite(covariance_x) and covariance_x > 0.0:
            self.position_variance_x = covariance_x
        if math.isfinite(covariance_y) and covariance_y > 0.0:
            self.position_variance_y = covariance_y

        if was_stale:
            self.estimator.reset()
        self.vx_body, self.vy_body, _ = self.estimator.update(
            self.last_pose_stamp, x, y, self.yaw_filter.yaw)

    def _on_imu(self, msg: Imu):
        raw_yaw_rate = self.yaw_rate_sign * float(msg.angular_velocity.z)
        if not math.isfinite(raw_yaw_rate):
            return

        now = time.monotonic()
        self.latest_raw_yaw_rate = raw_yaw_rate
        self.last_imu_receive_time = now
        self.yaw_filter.predict(raw_yaw_rate, now, self.max_predict_dt)

        self._gyro_samples.append((now, raw_yaw_rate))
        cutoff = now - self.calibration_window_sec
        while self._gyro_samples and self._gyro_samples[0][0] < cutoff:
            self._gyro_samples.popleft()

    def _on_calibrate_imu_yaw(self, _request, response):
        """선체를 정지한 상태에서 초기 gyro bias를 평균하고 LiDAR yaw로 재정렬."""
        now = time.monotonic()
        if (self.last_imu_receive_time is None
                or now - self.last_imu_receive_time > self.imu_timeout):
            response.success = False
            response.message = '최신 IMU gyro 데이터가 없습니다.'
            return response
        if (self.last_pose_receive_time is None
                or now - self.last_pose_receive_time > self.pos_timeout
                or self.last_lidar_yaw is None):
            response.success = False
            response.message = '최신 LiDAR 좌현/우현 표식 pose가 없습니다.'
            return response

        cutoff = now - self.calibration_window_sec
        samples = [sample for sample in self._gyro_samples if sample[0] >= cutoff]
        if len(samples) < self.calibration_min_samples:
            response.success = False
            response.message = (
                f'IMU 표본 수집 중: {len(samples)}/'
                f'{self.calibration_min_samples}. 배를 정지하세요.')
            return response
        duration = samples[-1][0] - samples[0][0]
        if duration < self.calibration_min_duration_sec:
            response.success = False
            response.message = (
                f'정지 표본 시간 부족: {duration:.2f}/'
                f'{self.calibration_min_duration_sec:.2f}s.')
            return response

        rates = [sample[1] for sample in samples]
        mean_bias = sum(rates) / len(rates)
        max_deviation = max(abs(rate - mean_bias) for rate in rates)
        if max_deviation > self.calibration_max_rate_deviation:
            response.success = False
            response.message = (
                f'선체 회전 감지: gyro 최대 편차 {max_deviation:.4f} rad/s '
                f'(허용 {self.calibration_max_rate_deviation:.4f}).')
            return response
        if abs(mean_bias) > self.calibration_max_abs_bias:
            response.success = False
            response.message = (
                f'추정 bias {mean_bias:.4f} rad/s가 허용 범위를 넘습니다.')
            return response

        sample_variance = sum(
            (rate - mean_bias) ** 2 for rate in rates) / max(1, len(rates) - 1)
        aligned_lidar_yaw = _resolve_pi_ambiguous_yaw(
            self.last_lidar_yaw, self.calibration_reference_yaw)
        reference_error = abs(normalize_angle(
            aligned_lidar_yaw - self.calibration_reference_yaw))
        if reference_error > self.calibration_reference_tolerance:
            response.success = False
            response.message = (
                '초기 방향 확인 실패: 선수를 -X 방향(180°)에 맞추세요. '
                f'가까운 LiDAR 후보 오차 {math.degrees(reference_error):.1f}° '
                f'(허용 {math.degrees(self.calibration_reference_tolerance):.1f}°).')
            return response

        self.yaw_filter.set_bias(
            mean_bias, variance=max(sample_variance / len(rates), 1e-8))
        self.yaw_filter.reset_yaw(
            aligned_lidar_yaw, self.last_lidar_yaw_variance)
        self.yaw_calibrated = True
        self.last_aligned_lidar_yaw = aligned_lidar_yaw
        self.last_lidar_yaw_correction_time = now
        self.estimator.reset()
        response.success = True
        response.message = (
            f'gyro bias 보정 완료: {mean_bias:.5f} rad/s, '
            f'LiDAR yaw {math.degrees(aligned_lidar_yaw):.2f}°로 정렬')
        self.get_logger().info(response.message)
        return response

    def _tick(self):
        now = time.monotonic()
        if self.last_pose_receive_time is None:
            return
        pose_age = now - self.last_pose_receive_time
        if pose_age > self.pos_timeout:
            if self._pose_ok:
                self.get_logger().warn(
                    f'LiDAR pose 유실 ({pose_age:.2f}s) — /odom 발행 중단',
                    throttle_duration_sec=2.0)
                self._pose_ok = False
                self.estimator.reset()
            return

        if self.last_imu_receive_time is None:
            return
        imu_age = now - self.last_imu_receive_time
        if imu_age > self.imu_timeout:
            if self._imu_ok:
                self.get_logger().warn(
                    f'IMU gyro 유실 ({imu_age:.2f}s) — /odom 발행 중단',
                    throttle_duration_sec=2.0)
                self._imu_ok = False
                self.estimator.reset()
            return

        if not self.yaw_filter.initialized:
            return
        if not self.yaw_calibrated:
            self.get_logger().warn(
                '시작 gyro bias 보정 전입니다. '
                '`ros2 run kaboat_hardware calibrate_indoor_imu`를 실행하세요.',
                throttle_duration_sec=5.0)
            return

        if not self._pose_ok or not self._imu_ok:
            self._pose_ok = True
            self._imu_ok = True
            self.get_logger().info(
                '✅ LiDAR pose와 IMU gyro 정상 — EKF /odom 발행')

        yaw = self.yaw_filter.yaw
        yaw_rate = self.latest_raw_yaw_rate - self.yaw_filter.bias
        stamp = self.get_clock().now().to_msg()
        odom = self._build_odometry(
            stamp, self.last_pose_x, self.last_pose_y, yaw,
            self.vx_body, self.vy_body, yaw_rate)
        self.odom_pub.publish(odom)

        if self.publish_tf:
            transform = TransformStamped()
            transform.header.stamp = stamp
            transform.header.frame_id = self.odom_frame
            transform.child_frame_id = self.base_frame
            transform.transform.translation.x = self.last_pose_x
            transform.transform.translation.y = self.last_pose_y
            transform.transform.rotation = _quat_from_yaw(yaw)
            self.tf_broadcaster.sendTransform(transform)

    def _build_odometry(self, stamp, x, y, yaw, vx, vy, yaw_rate):
        msg = Odometry()
        msg.header.stamp = stamp
        msg.header.frame_id = self.odom_frame
        msg.child_frame_id = self.base_frame
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.orientation = _quat_from_yaw(yaw)
        msg.twist.twist.linear.x = vx
        msg.twist.twist.linear.y = vy
        msg.twist.twist.angular.z = yaw_rate
        msg.pose.covariance[0] = self.position_variance_x
        msg.pose.covariance[7] = self.position_variance_y
        msg.pose.covariance[35] = self.yaw_filter.yaw_variance
        twist_variance = float(self.get_parameter('twist_stddev').value) ** 2
        msg.twist.covariance[0] = twist_variance
        msg.twist.covariance[7] = twist_variance
        msg.twist.covariance[35] = twist_variance
        return msg


def main(args=None):
    rclpy.init(args=args)
    node = IndoorLidarOdom()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
