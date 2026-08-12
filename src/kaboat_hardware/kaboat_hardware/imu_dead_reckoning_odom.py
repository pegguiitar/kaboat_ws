"""GQ7 IMU만으로 짧은 파이프라인 시험용 /odom을 발행한다.

GPS/AprilTag가 준비되기 전 TG-50→occupancy_grid 갱신 여부를 확인하기 위한
임시 노드다. 가속도 이중 적분 오차 때문에 항법·모터 주행에는 사용하면 안 된다.
"""

import math

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from std_srvs.srv import Trigger
from tf2_ros import TransformBroadcaster

from kaboat_hardware.imu_dead_reckoning_core import (
    DeadReckoningParams,
    ImuDeadReckoner,
)


def _stamp_sec(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


class ImuDeadReckoningOdom(Node):
    def __init__(self):
        super().__init__('imu_dead_reckoning_odom')
        self.declare_parameter('imu_topic', '/imu/data')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('publish_tf', True)

        defaults = DeadReckoningParams()
        for name in defaults.__dataclass_fields__:
            self.declare_parameter(name, getattr(defaults, name))
        params = DeadReckoningParams(**{
            name: self.get_parameter(name).value
            for name in defaults.__dataclass_fields__
        })
        self.estimator = ImuDeadReckoner(params)
        self.odom_frame = str(self.get_parameter('odom_frame').value)
        self.base_frame = str(self.get_parameter('base_frame').value)
        self.publish_tf = bool(self.get_parameter('publish_tf').value)
        self.pub = self.create_publisher(
            Odometry, str(self.get_parameter('odom_topic').value), 10)
        self.tf_pub = TransformBroadcaster(self) if self.publish_tf else None
        self.create_subscription(
            Imu, str(self.get_parameter('imu_topic').value),
            self._on_imu, qos_profile_sensor_data)
        self.create_service(Trigger, '~/reset', self._reset)
        self._last_calibration_log = -1
        self._announced_ready = False
        self.get_logger().warning(
            'IMU 단독 dead-reckoning 시험 모드 — 장기 항법/모터 주행 금지. '
            f'{params.calibration_duration_sec:.1f}초 동안 센서를 움직이지 마세요.')

    def _on_imu(self, msg):
        if msg.orientation_covariance[0] == -1.0:
            self.get_logger().error(
                'GQ7 orientation이 없어 중력을 제거할 수 없습니다.',
                throttle_duration_sec=5.0)
            return
        q = msg.orientation
        gyro = msg.angular_velocity
        accel = msg.linear_acceleration
        try:
            state = self.estimator.update(
                _stamp_sec(msg.header.stamp),
                (q.x, q.y, q.z, q.w),
                (gyro.x, gyro.y, gyro.z),
                (accel.x, accel.y, accel.z),
            )
        except ValueError as exc:
            self.get_logger().error(str(exc), throttle_duration_sec=5.0)
            return
        if state is None:
            percent = int(self.estimator.calibration_progress * 10.0) * 10
            if percent != self._last_calibration_log:
                self._last_calibration_log = percent
                self.get_logger().info(f'정지 바이어스 보정 중 {percent}%')
            return
        if not self._announced_ready:
            self._announced_ready = True
            bias = self.estimator.accel_bias_body
            self.get_logger().warning(
                '보정 완료 — /odom 발행 시작. 시험 후 반드시 GPS/AprilTag odom으로 '
                f'교체하세요. accel_bias={bias}, gyro_z_bias='
                f'{self.estimator.gyro_bias_z:.6f}')
        odom = self._odometry(msg.header.stamp, state)
        self.pub.publish(odom)
        if self.tf_pub is not None:
            self._publish_tf(odom)

    def _odometry(self, stamp, state):
        msg = Odometry()
        msg.header.stamp = stamp
        msg.header.frame_id = self.odom_frame
        msg.child_frame_id = self.base_frame
        msg.pose.pose.position.x = state.x
        msg.pose.pose.position.y = state.y
        msg.pose.pose.orientation.z = math.sin(state.yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(state.yaw / 2.0)

        c, s = math.cos(state.yaw), math.sin(state.yaw)
        msg.twist.twist.linear.x = c * state.vx_world + s * state.vy_world
        msg.twist.twist.linear.y = -s * state.vx_world + c * state.vy_world
        msg.twist.twist.angular.z = state.yaw_rate

        # IMU 이중 적분 위치는 불확실하므로 의도적으로 큰 공분산을 표시한다.
        msg.pose.covariance[0] = 4.0
        msg.pose.covariance[7] = 4.0
        msg.pose.covariance[35] = 0.25
        msg.twist.covariance[0] = 1.0
        msg.twist.covariance[7] = 1.0
        msg.twist.covariance[35] = 0.04
        return msg

    def _publish_tf(self, odom):
        tf = TransformStamped()
        tf.header = odom.header
        tf.child_frame_id = odom.child_frame_id
        tf.transform.translation.x = odom.pose.pose.position.x
        tf.transform.translation.y = odom.pose.pose.position.y
        tf.transform.rotation = odom.pose.pose.orientation
        self.tf_pub.sendTransform(tf)

    def _reset(self, _request, response):
        self.estimator.reset_pose()
        response.success = True
        response.message = 'IMU dead-reckoning 위치·속도·상대 yaw를 원점으로 초기화함'
        self.get_logger().warning(response.message)
        return response


def main(args=None):
    rclpy.init(args=args)
    node = ImuDeadReckoningOdom()
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
