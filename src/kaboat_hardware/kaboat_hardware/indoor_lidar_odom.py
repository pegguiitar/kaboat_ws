"""indoor_lidar_odom — 외부 라이다(수조 GPS) + 선체 IMU 자이로/자세 → /odom (실내 수조 시험용).

실외 GNSS/INS(GQ7 EKF) 대신 실내에서 `/odom`을 만드는 소스입니다.
스택은 `/odom`의 데이터 내용만 소비하므로 실물 EKF 대신 갈아끼워 동작합니다.

필드별 출처 — 각 센서의 강점 결합:
  pose.position       ← 외부 고정 라이다 (/boat_position, 절대 위치 추적, 실내 GPS 역할)
  pose.orientation    ← 선체 GQ7 IMU (/imu/data) + 수조 +X축 편차 보정
  twist.angular.z     ← 선체 GQ7 IMU 자이로 (직접 측정, 지연 및 미분 노이즈 없음)
  twist.linear.x/y    ← 라이다 위치 미분 (body frame 전진/횡방향 속도, pose_velocity)

안전 기능:
  외부 라이다 또는 IMU 신호가 각 timeout 이상 끊기면 /odom 발행을 즉시 중단합니다.
  하위 제어 노드는 자체 odom freshness watchdog으로 이를 감지해 정지해야 합니다.
"""

import math
import time
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import PointStamped, TransformStamped, Quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from tf2_ros import TransformBroadcaster

from kaboat_hardware.pose_velocity import (
    VelocityEstimator, VelocityParams, normalize_angle
)


def _yaw_from_quat(q: Quaternion) -> float:
    """쿼터니언 메시지에서 Yaw(rad) 추출."""
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def _quat_from_yaw(yaw: float) -> Quaternion:
    """Yaw(rad) 각도를 Quaternion 메시지로 변환."""
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


def _stamp_sec(stamp) -> float:
    return stamp.sec + stamp.nanosec * 1e-9


class IndoorLidarOdom(Node):
    def __init__(self):
        super().__init__('indoor_lidar_odom')

        # ── 프레임 및 토픽 파라미터 ───────────────────────
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('pos_topic', '/boat_position')
        self.declare_parameter('imu_topic', '/imu/data')
        self.declare_parameter('publish_tf', True)

        # ── 센서 타이밍 및 타임아웃 ───────────────────────
        self.declare_parameter('pos_timeout_sec', 1.0)      # 라이다 위치 유실 판정 시간 [s] (일시 단절 허용)
        self.declare_parameter('imu_timeout_sec', 0.5)      # IMU 타임아웃 [s]
        self.declare_parameter('publish_rate', 30.0)        # /odom 발행 주기 [Hz]

        # ── IMU 보정 파라미터 ─────────────────────────────
        self.declare_parameter('imu_yaw_offset_deg', -51.27)   # 수조 +X축 기준 IMU 설치 편차 각도 [deg] (+X 정렬 시 -51.27° 보정)
        self.declare_parameter('yaw_rate_sign', 1.0)        # 반시계(좌회전) 양수 부호 보정 (+1.0 또는 -1.0)
        self.declare_parameter('gyro_bias_z', 0.0)          # 자이로 Z축 정지 바이어스 [rad/s]

        # ── 속도 추정 필터 파라미터 (pose_velocity) ───────
        self.declare_parameter('vel_window_sec', 0.15)
        self.declare_parameter('vel_filter_tau', 0.15)
        self.declare_parameter('vel_max_speed', 3.0)

        # ── 공분산 ─────────────────────────────────────────
        self.declare_parameter('pose_stddev_xy', 0.02)
        self.declare_parameter('pose_stddev_yaw', 0.02)
        self.declare_parameter('twist_stddev', 0.05)

        # 파라미터 로드
        self.odom_frame = str(self.get_parameter('odom_frame').value)
        self.base_frame = str(self.get_parameter('base_frame').value)
        self.pos_topic = str(self.get_parameter('pos_topic').value)
        self.imu_topic = str(self.get_parameter('imu_topic').value)
        self.publish_tf = bool(self.get_parameter('publish_tf').value)

        self.pos_timeout = float(self.get_parameter('pos_timeout_sec').value)
        self.imu_timeout = float(self.get_parameter('imu_timeout_sec').value)
        self.imu_yaw_offset_rad = math.radians(float(self.get_parameter('imu_yaw_offset_deg').value))
        self.yaw_rate_sign = float(self.get_parameter('yaw_rate_sign').value)
        self.gyro_bias_z = float(self.get_parameter('gyro_bias_z').value)

        self.estimator = VelocityEstimator(VelocityParams(
            window_sec=float(self.get_parameter('vel_window_sec').value),
            filter_tau=float(self.get_parameter('vel_filter_tau').value),
            max_speed=float(self.get_parameter('vel_max_speed').value),
        ))

        # 상태 변수
        self.last_pos_stamp: Optional[float] = None
        self.last_pos_receive_time: Optional[float] = None
        self.last_pos_x: float = 0.0
        self.last_pos_y: float = 0.0
        self._position_seq: int = 0

        self.imu_stamp: Optional[float] = None
        self.last_imu_receive_time: Optional[float] = None
        self.imu_yaw: float = 0.0
        self.imu_yaw_rate: float = 0.0

        self._pos_ok: bool = False
        self._imu_ok: bool = False
        self._last_published_position_seq: int = -1

        # 통신 인터페이스
        self.tf_broadcaster = TransformBroadcaster(self)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)

        # 구독자
        self.create_subscription(
            PointStamped, self.pos_topic, self._on_position, qos_profile_sensor_data)
        self.create_subscription(
            Imu, self.imu_topic, self._on_imu, qos_profile_sensor_data)

        # 주기 실행 (타이머)
        rate = float(self.get_parameter('publish_rate').value)
        self.create_timer(1.0 / rate, self._tick)

        self.get_logger().info(
            f"🚀 [indoor_lidar_odom] 시작 (실내 GPS + 선체 IMU 융합)!\n"
            f"   - 위치 소스 (실내 GPS): '{self.pos_topic}' (타임아웃: {self.pos_timeout}s)\n"
            f"   - 자세/각속도 소스: '{self.imu_topic}' (오프셋: {math.degrees(self.imu_yaw_offset_rad):.1f}°)\n"
            f"   - 발행: /odom, TF: '{self.odom_frame}' -> '{self.base_frame}' ({self.publish_tf})"
        )

    def _on_position(self, msg: PointStamped):
        """외부 라이다(실내 GPS)로부터 배 위치 수신."""
        self.last_pos_x = msg.point.x
        self.last_pos_y = msg.point.y
        source_stamp = _stamp_sec(msg.header.stamp)
        receive_time = time.monotonic()
        # 속도 미분에는 검출 시각을 쓰되, 드라이버가 0 stamp를 보내는 경우에만
        # 수신 시각으로 폴백한다. timeout 판정에는 아래 수신 시각을 별도로 쓴다.
        self.last_pos_stamp = source_stamp if source_stamp > 0.0 else receive_time
        self.last_pos_receive_time = receive_time
        self._position_seq += 1

    def _on_imu(self, msg: Imu):
        """선체 GQ7 IMU로부터 자세 및 각속도 수신."""
        q = msg.orientation
        norm_sq = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w
        if norm_sq > 0.25:
            raw_yaw = _yaw_from_quat(q)
            self.imu_yaw = normalize_angle(raw_yaw + self.imu_yaw_offset_rad)

        self.imu_yaw_rate = (self.yaw_rate_sign * msg.angular_velocity.z) - self.gyro_bias_z
        self.imu_stamp = _stamp_sec(msg.header.stamp)
        self.last_imu_receive_time = time.monotonic()

    def _tick(self):
        now_monotonic = time.monotonic()

        # 1. 위치 신호 유효성 검사
        if self.last_pos_receive_time is None:
            # 실제 위치를 받기 전에 (0, 0) 가짜 odom을 내보내면 제어기가 이를
            # 유효 위치로 오인할 수 있다. 첫 LiDAR 위치까지는 발행하지 않는다.
            return

        pos_age = now_monotonic - self.last_pos_receive_time
        if pos_age > self.pos_timeout:
            if self._pos_ok:
                self.get_logger().warn(
                    f"⚠️ [indoor_lidar_odom] 외부 라이다 위치 신호 유실 ({pos_age:.2f}s 전) — /odom 발행 중단 (워치독 정지)",
                    throttle_duration_sec=2.0)
                self._pos_ok = False
                self.estimator.reset()
            return

        # 2. IMU도 fresh해야 유효한 pose를 만들 수 있다. 이전 yaw를 계속 쓰면
        # 제어기는 센서 유실을 알 수 없으므로, IMU timeout 시에도 odom을 멈춘다.
        if self.last_imu_receive_time is None:
            return
        imu_age = now_monotonic - self.last_imu_receive_time
        if imu_age > self.imu_timeout:
            if self._imu_ok:
                self.get_logger().warn(
                    f"⚠️ [indoor_lidar_odom] 선체 IMU 신호 유실 ({imu_age:.2f}s 전) "
                    "— /odom 발행 중단",
                    throttle_duration_sec=2.0)
                self._imu_ok = False
                self.estimator.reset()
            return

        if not self._pos_ok or not self._imu_ok:
            self._pos_ok = True
            self._imu_ok = True
            self.estimator.reset()
            self.get_logger().info(
                "✅ [indoor_lidar_odom] LiDAR 위치와 IMU 모두 정상 — /odom 발행 재개")

        # 3. 새 위치 표본마다 odom을 한 번만 발행한다. source stamp가 같거나 0이어도
        # 콜백 순번으로 구분하므로 드라이버 timestamp 품질에 안전하게 대응한다.
        if self._last_published_position_seq == self._position_seq:
            return

        x = self.last_pos_x
        y = self.last_pos_y
        stamp_sec = self.last_pos_stamp

        yaw = self.imu_yaw
        yaw_rate = self.imu_yaw_rate

        # 4. 위치 미분 기반 선속도 (body frame) 추정
        vx_body, vy_body, _ = self.estimator.update(stamp_sec, x, y, yaw)

        # 5. Odometry 메시지 생성 및 발행
        stamp_msg = self.get_clock().now().to_msg()
        odom_msg = self._build_odometry(stamp_msg, x, y, yaw, vx_body, vy_body, yaw_rate)
        self.odom_pub.publish(odom_msg)

        # 6. TF 발행 (odom -> base_link)
        if self.publish_tf:
            tf_msg = TransformStamped()
            tf_msg.header.stamp = stamp_msg
            tf_msg.header.frame_id = self.odom_frame
            tf_msg.child_frame_id = self.base_frame
            tf_msg.transform.translation.x = x
            tf_msg.transform.translation.y = y
            tf_msg.transform.translation.z = 0.0
            tf_msg.transform.rotation = _quat_from_yaw(yaw)
            self.tf_broadcaster.sendTransform(tf_msg)

        self._last_published_position_seq = self._position_seq

    def _build_odometry(self, stamp_msg, x, y, yaw, vx, vy, yaw_rate) -> Odometry:
        msg = Odometry()
        msg.header.stamp = stamp_msg
        msg.header.frame_id = self.odom_frame
        msg.child_frame_id = self.base_frame

        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation = _quat_from_yaw(yaw)

        # Body frame 선속도 및 각속도
        msg.twist.twist.linear.x = vx
        msg.twist.twist.linear.y = vy
        msg.twist.twist.linear.z = 0.0
        msg.twist.twist.angular.z = yaw_rate

        var_xy = float(self.get_parameter('pose_stddev_xy').value) ** 2
        var_yaw = float(self.get_parameter('pose_stddev_yaw').value) ** 2
        var_tw = float(self.get_parameter('twist_stddev').value) ** 2
        msg.pose.covariance[0] = var_xy
        msg.pose.covariance[7] = var_xy
        msg.pose.covariance[35] = var_yaw
        msg.twist.covariance[0] = var_tw
        msg.twist.covariance[7] = var_tw
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
