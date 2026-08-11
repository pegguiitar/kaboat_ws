"""apriltag_odom — 천장 AprilTag + IMU 자이로 → /odom (실내 수조 시험용).

실외 GNSS/INS(GQ7 EKF) 대신 실내에서 `/odom` 을 만드는 소스다. 스택은
`/odom` 의 header.frame_id 를 아무도 읽지 않고 내용만 쓰므로, sim 의 Gazebo
ground-truth / 실물 EKF 에 이어 세 번째 소스로 그대로 갈아끼울 수 있다
(SKELETON §1 "노드는 표준 토픽만 알고 출처는 모른다").

필드별 출처 — 각 센서가 제일 잘하는 것만 취한다:
  pose.position / orientation  ← AprilTag (절대 측정, 드리프트 없음)
  twist.angular.z              ← IMU 자이로 (직접 측정, 미분 없음)
  twist.linear.x/y             ← AprilTag 위치 미분 (pose_velocity.py)

  ⚠️ 자이로를 쓰는 이유는 yaw 미분의 노이즈 때문이다 — 각도 오차 1° 가
  30fps 에서 0.52 rad/s(=실측 ω_max)로 증폭돼 D항을 포화시킨다.
  자이로가 안 오면 미분 폴백으로 내려가되 경고를 낸다.

입력
  TF  <odom_frame> → <tag_frame>   apriltag_ros 가 발행하는 태그 pose
                                    (christianrauch/apriltag_ros 는 pose 를
                                     /detections 가 아니라 TF 로만 낸다)
  /imu/data (sensor_msgs/Imu)      angular_velocity.z 만 사용
출력
  /odom (nav_msgs/Odometry)
  TF  <odom_frame> → <camera_frame>  천장 카메라 설치 자세 (정적, 옵션)

태그 유실 시 **발행을 멈춘다** — 마지막 위치를 계속 재발행하면 배가 옛
좌표를 믿고 달린다. 끊으면 cmd_mux 의 300ms 워치독이 배를 정지시킨다.

⚠️ GQ7 드라이버의 /odom remap 과 동시에 켜면 발행자가 둘이 된다.
   real_sensors.launch.py 의 enable_odom_remap:=false 로 끄고 쓸 것.
"""
import math

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from tf2_ros import Buffer, TransformException, TransformListener
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

from kaboat_hardware.pose_velocity import (
    VelocityEstimator, VelocityParams, normalize_angle)


def _yaw_from_quat(q) -> float:
    """스택 전체와 동일한 yaw 추출 (behavior_base.yaw_from_quaternion)."""
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def _quat_from_yaw(yaw: float):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def _stamp_sec(stamp) -> float:
    return stamp.sec + stamp.nanosec * 1e-9


class AprilTagOdom(Node):
    def __init__(self):
        super().__init__('apriltag_odom')

        # ── 프레임 ─────────────────────────────────────
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        # apriltag_ros 는 'tag<family>:<id>' 형태로 TF 를 낸다. 실제 이름은
        # `ros2 run tf2_tools view_frames` 로 확인해 맞출 것.
        self.declare_parameter('tag_frame', 'tag36h11:0')
        self.declare_parameter('camera_frame', 'ceiling_camera')

        # ── 천장 카메라 설치 자세 (odom → camera) ───────
        # 수조 원점을 어디로 잡을지 정해지면 여기만 채우면 된다. tf2 가
        # odom→camera→tag 합성을 대신 해주므로 노드는 행렬 계산을 안 한다.
        self.declare_parameter('publish_camera_tf', True)
        self.declare_parameter('camera_xyz', [0.0, 0.0, 3.0])
        self.declare_parameter('camera_rpy', [math.pi, 0.0, 0.0])  # 바닥을 내려다봄

        # ── 배에 붙인 태그의 장착 오차 ──────────────────
        self.declare_parameter('tag_yaw_offset', 0.0)      # 태그 정면 vs 선수 [rad]
        self.declare_parameter('tag_offset_xy', [0.0, 0.0])  # 선체 중심 기준 [m]

        # ── 타이밍 ─────────────────────────────────────
        self.declare_parameter('publish_rate', 30.0)       # TF 폴링 주기 [Hz]
        self.declare_parameter('tag_timeout_sec', 0.3)     # 이보다 오래되면 유실
        self.declare_parameter('imu_timeout_sec', 0.5)
        self.declare_parameter('imu_topic', '/imu/data')

        # ── 속도 추정 ──────────────────────────────────
        self.declare_parameter('vel_window_sec', 0.15)
        self.declare_parameter('vel_filter_tau', 0.15)
        self.declare_parameter('vel_max_speed', 3.0)
        self.declare_parameter('use_imu_yaw_rate', True)
        # ⚠️ 반시계(좌회전)에서 angular.z 가 양수여야 한다. 장착이 뒤집혀
        # 부호가 반대면 D항이 감쇠가 아니라 발진 방향으로 작용한다 —
        # 모터 붙이기 전에 손으로 돌려 확인하고, 반대면 -1.0 을 넣을 것.
        self.declare_parameter('yaw_rate_sign', 1.0)
        self.declare_parameter('gyro_bias_z', 0.0)         # 정지 실측 바이어스 [rad/s]

        # ── 공분산 (robot_localization 등 하위 소비자용) ──
        self.declare_parameter('pose_stddev_xy', 0.02)
        self.declare_parameter('pose_stddev_yaw', 0.02)
        self.declare_parameter('twist_stddev', 0.05)

        self.odom_frame = str(self.get_parameter('odom_frame').value)
        self.base_frame = str(self.get_parameter('base_frame').value)
        self.tag_frame = str(self.get_parameter('tag_frame').value)
        self.tag_yaw_offset = float(self.get_parameter('tag_yaw_offset').value)
        self.tag_offset_xy = [float(v) for v in
                              self.get_parameter('tag_offset_xy').value]
        self.tag_timeout = float(self.get_parameter('tag_timeout_sec').value)
        self.imu_timeout = float(self.get_parameter('imu_timeout_sec').value)
        self.use_imu_yaw_rate = bool(
            self.get_parameter('use_imu_yaw_rate').value)
        self.yaw_rate_sign = float(self.get_parameter('yaw_rate_sign').value)
        self.gyro_bias_z = float(self.get_parameter('gyro_bias_z').value)

        self.estimator = VelocityEstimator(VelocityParams(
            window_sec=float(self.get_parameter('vel_window_sec').value),
            filter_tau=float(self.get_parameter('vel_filter_tau').value),
            max_speed=float(self.get_parameter('vel_max_speed').value),
        ))

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        if bool(self.get_parameter('publish_camera_tf').value):
            self._publish_camera_tf()

        self.imu_yaw_rate = None
        self.imu_stamp = None
        self.create_subscription(
            Imu, str(self.get_parameter('imu_topic').value),
            self._on_imu, qos_profile_sensor_data)

        self.pub = self.create_publisher(Odometry, '/odom', 10)
        rate = float(self.get_parameter('publish_rate').value)
        self.create_timer(1.0 / rate, self._tick)

        self._last_stamp = None    # 발행한 마지막 검출 시각 — 단조성 보장용
        self._tag_ok = False       # 유실/재획득 로그 1회만
        self.get_logger().info(
            f"apriltag_odom 시작 — TF '{self.odom_frame}' → '{self.tag_frame}' "
            f'→ /odom, 각속도 출처 '
            f"{'IMU 자이로' if self.use_imu_yaw_rate else '태그 미분'}")

    # ── 정적 TF: 천장 카메라 설치 자세 ────────────────
    def _publish_camera_tf(self):
        xyz = [float(v) for v in self.get_parameter('camera_xyz').value]
        roll, pitch, yaw = (float(v) for v in
                            self.get_parameter('camera_rpy').value)
        cr, sr = math.cos(roll / 2.0), math.sin(roll / 2.0)
        cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
        cy, sy = math.cos(yaw / 2.0), math.sin(yaw / 2.0)

        tf = TransformStamped()
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = self.odom_frame
        tf.child_frame_id = str(self.get_parameter('camera_frame').value)
        tf.transform.translation.x = xyz[0]
        tf.transform.translation.y = xyz[1]
        tf.transform.translation.z = xyz[2]
        tf.transform.rotation.w = cr * cp * cy + sr * sp * sy
        tf.transform.rotation.x = sr * cp * cy - cr * sp * sy
        tf.transform.rotation.y = cr * sp * cy + sr * cp * sy
        tf.transform.rotation.z = cr * cp * sy - sr * sp * cy

        self._static_tf = StaticTransformBroadcaster(self)
        self._static_tf.sendTransform(tf)
        self.get_logger().info(
            f'천장 카메라 정적 TF 발행 — {self.odom_frame} → {tf.child_frame_id} '
            f'@ {xyz}')

    def _on_imu(self, msg: Imu):
        self.imu_yaw_rate = (self.yaw_rate_sign * msg.angular_velocity.z
                             - self.gyro_bias_z)
        self.imu_stamp = _stamp_sec(msg.header.stamp)

    # ── 주기 실행 ─────────────────────────────────────
    def _tick(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                self.odom_frame, self.tag_frame, rclpy.time.Time())
        except TransformException:
            self._mark_lost('태그 TF 없음')
            return

        stamp = _stamp_sec(tf.header.stamp)
        now = self.get_clock().now().nanoseconds * 1e-9
        if now - stamp > self.tag_timeout:
            self._mark_lost(f'태그 유실 — 마지막 검출 {now - stamp:.2f}s 전')
            return
        # 같은 검출을 두 번 내보내지 않는다. 스탬프가 뒤로 가는 경우도 여기서
        # 걸린다 — avoid FSM 이 이 스탬프를 시계로 쓰므로 단조성이 필수다.
        if self._last_stamp is not None and stamp <= self._last_stamp:
            return

        if not self._tag_ok:
            self._tag_ok = True
            self.estimator.reset()   # 공백을 건너뛰어 미분하지 않는다
            self.get_logger().info('태그 획득 — /odom 발행 시작')

        t = tf.transform.translation
        yaw = normalize_angle(
            _yaw_from_quat(tf.transform.rotation) - self.tag_yaw_offset)
        # 태그가 선체 중심에서 떨어져 있으면 그만큼 되돌린다
        c, s = math.cos(yaw), math.sin(yaw)
        x = t.x - (c * self.tag_offset_xy[0] - s * self.tag_offset_xy[1])
        y = t.y - (s * self.tag_offset_xy[0] + c * self.tag_offset_xy[1])

        vx_body, vy_body, tag_yaw_rate = self.estimator.update(stamp, x, y, yaw)
        yaw_rate = self._yaw_rate(stamp, tag_yaw_rate)

        self.pub.publish(self._odometry(tf.header.stamp, x, y, yaw,
                                        vx_body, vy_body, yaw_rate))
        self._last_stamp = stamp

    def _yaw_rate(self, stamp: float, tag_yaw_rate: float) -> float:
        """자이로 우선, 없거나 오래되면 태그 미분으로 폴백."""
        if not self.use_imu_yaw_rate:
            return tag_yaw_rate
        if (self.imu_stamp is None
                or abs(stamp - self.imu_stamp) > self.imu_timeout):
            self.get_logger().warning(
                'IMU 각속도 없음/지연 — 태그 미분으로 폴백 (노이즈 증가, '
                'D항 포화 주의)', throttle_duration_sec=5.0)
            return tag_yaw_rate
        return self.imu_yaw_rate

    def _odometry(self, stamp, x, y, yaw, vx, vy, yaw_rate) -> Odometry:
        msg = Odometry()
        msg.header.stamp = stamp          # 검출 시각 — 수신 시각이 아니다
        msg.header.frame_id = self.odom_frame
        msg.child_frame_id = self.base_frame

        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        qx, qy, qz, qw = _quat_from_yaw(yaw)
        msg.pose.pose.orientation.x = qx
        msg.pose.pose.orientation.y = qy
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw

        msg.twist.twist.linear.x = vx     # body frame (pose_velocity 규약)
        msg.twist.twist.linear.y = vy
        msg.twist.twist.angular.z = yaw_rate

        var_xy = float(self.get_parameter('pose_stddev_xy').value) ** 2
        var_yaw = float(self.get_parameter('pose_stddev_yaw').value) ** 2
        var_tw = float(self.get_parameter('twist_stddev').value) ** 2
        msg.pose.covariance[0] = var_xy
        msg.pose.covariance[7] = var_xy
        msg.pose.covariance[35] = var_yaw
        msg.twist.covariance[0] = var_tw
        msg.twist.covariance[7] = var_tw
        msg.twist.covariance[35] = var_tw
        return msg

    def _mark_lost(self, reason: str):
        """유실 시 발행을 멈춘다 — cmd_mux 300ms 워치독이 배를 정지시킨다."""
        if self._tag_ok:
            self._tag_ok = False
            self.get_logger().warning(f'{reason} — /odom 발행 중단')
        else:
            self.get_logger().warning(reason, throttle_duration_sec=5.0)


def main(args=None):
    rclpy.init(args=args)
    node = AprilTagOdom()
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
