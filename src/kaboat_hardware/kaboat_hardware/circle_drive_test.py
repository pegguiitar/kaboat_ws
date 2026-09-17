"""circle_drive_test — 실내 수조 고정 반경 원(Circle/Orbit) 추종 주행 테스트.

수조(10m x 5m) 중앙을 중심으로 지정된 반경(기본 1.2m)의 원 궤도를
벡터 필드(Vector Field) 알고리즘으로 추종하며 선회 주행을 수행합니다.

특징:
  - 수조 안 어느 위치에서 시작해도 원 궤도로 부드럽게 진입하여 안착
  - 시계방향(CW) / 반시계방향(CCW) 회전 방향 선택 지원
  - 목표 바퀴 수(기본 2바퀴, 0 설정 시 무한 회전) 완료 후 자동 안전 정지
  - RViz2 시각화 마커(/circle_drive/markers) 발행
  - /emergency_stop 토픽 수신 시 즉시 비상 제동
"""

import math
import time
from typing import Optional

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Quaternion, Point
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker, MarkerArray

from kaboat_hardware.test_coordinates import CIRCLE_DRIVE
from kaboat_hardware.trajectory_visualization import (
    BoundedTrajectoryHistory,
    create_trail_marker,
)


def normalize_angle(angle_rad: float) -> float:
    """각도를 [-pi, pi] 범위로 정규화."""
    while angle_rad > math.pi:
        angle_rad -= 2.0 * math.pi
    while angle_rad < -math.pi:
        angle_rad += 2.0 * math.pi
    return angle_rad


def yaw_to_quaternion(yaw_rad: float) -> Quaternion:
    """Yaw(rad) 각도를 Quaternion 메시지로 변환."""
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw_rad / 2.0)
    q.w = math.cos(yaw_rad / 2.0)
    return q


def yaw_from_quaternion(q: Quaternion) -> float:
    """쿼터니언 메시지에서 Yaw(rad) 추출."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class CircleDriveTest(Node):
    def __init__(self):
        super().__init__('circle_drive_test')

        # ── 주행 및 궤도 파라미터 (test_coordinates.py 기본값 참조) ──
        self.declare_parameter('center_x', float(CIRCLE_DRIVE['center_x']))
        self.declare_parameter('center_y', float(CIRCLE_DRIVE['center_y']))
        self.declare_parameter('radius', float(CIRCLE_DRIVE['radius']))
        self.declare_parameter('direction', str(CIRCLE_DRIVE.get('direction', 'ccw')))
        self.declare_parameter('target_laps', float(CIRCLE_DRIVE.get('target_laps', 2.0)))

        # ── 제어 및 출력 파라미터 ─────────────────────────
        self.declare_parameter('cruise_speed', 0.50)     # 기본 전진 출력 비 (0.0 ~ 1.0, 50%)
        self.declare_parameter('max_angular', 0.60)      # 최대 회전 출력 비 (0.0 ~ 1.0, 60%)
        self.declare_parameter('kp_yaw', 1.0)            # 헤딩 비례(P) 게인
        self.declare_parameter('kd_yaw', 0.15)           # 요레이트 감쇠(D) 게인
        self.declare_parameter('k_converge', 1.5)        # 궤도 진입 수렴 게인

        # ── 안전 파라미터 ─────────────────────────────────
        self.declare_parameter('odom_timeout_sec', 0.5)  # /odom 타임아웃 [s]
        self.declare_parameter('wait_for_start', True)   # 외부 시작 신호(/start_mission) 대기 여부

        # 파라미터 로드
        self.center_x = float(self.get_parameter('center_x').value)
        self.center_y = float(self.get_parameter('center_y').value)
        self.radius = float(self.get_parameter('radius').value)
        dir_str = str(self.get_parameter('direction').value).strip().lower()
        self.dir_sign = -1.0 if dir_str in ('cw', 'clockwise', '-1') else 1.0

        self.target_laps = float(self.get_parameter('target_laps').value)
        self.cruise_speed = float(self.get_parameter('cruise_speed').value)
        self.max_angular = float(self.get_parameter('max_angular').value)
        self.kp_yaw = float(self.get_parameter('kp_yaw').value)
        self.kd_yaw = float(self.get_parameter('kd_yaw').value)
        self.k_converge = float(self.get_parameter('k_converge').value)
        self.odom_timeout = float(self.get_parameter('odom_timeout_sec').value)
        self.wait_for_start = bool(self.get_parameter('wait_for_start').value)

        # 상태 변수
        self.current_x: Optional[float] = None
        self.current_y: Optional[float] = None
        self.current_yaw: Optional[float] = None
        self.current_yaw_rate: float = 0.0
        self.last_odom_time: Optional[float] = None

        self.last_polar_angle: Optional[float] = None
        self.accumulated_angle: float = 0.0   # 누적 회전 각도 [rad]
        self.current_laps: float = 0.0

        self.started = not self.wait_for_start
        self.mission_finished = False
        self.emergency_stopped = False
        self.stop_cmd_sent = False
        self.trajectory_history = BoundedTrajectoryHistory()

        # 통신 인터페이스
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/circle_drive/markers', 10)

        self.create_subscription(Odometry, '/odom', self._on_odom, 10)
        self.create_subscription(Bool, '/emergency_stop', self._on_estop, 10)
        self.create_subscription(Bool, '/start_mission', self._on_start_mission, 10)
        self.create_service(Trigger, '/start_test', self._on_start_service)
        self.create_service(Trigger, '/clear_trajectory', self._on_clear_trajectory)

        # 20Hz 제어 타이머 & 2Hz 마커 시각화 타이머
        self.create_timer(0.05, self._control_loop)
        self.create_timer(0.5, self._publish_markers)

        dir_desc = "반시계방향(CCW)" if self.dir_sign > 0 else "시계방향(CW)"
        lap_desc = f"{self.target_laps:.1f}바퀴" if self.target_laps > 0 else "무한 연속 주행"
        wait_desc = "외부 시작 신호(/start_mission) 대기 모드" if self.wait_for_start else "자동 즉시 시작 모드"

        self.get_logger().info(
            f"🌊 [CircleDriveTest] 원형 선회 주행 테스트 준비 완료!\n"
            f"   - 중심: ({self.center_x:.2f}, {self.center_y:.2f})m, 반경: {self.radius:.2f}m\n"
            f"   - 회전 방향: {dir_desc}, 목표: {lap_desc}\n"
            f"   - 기본 순항 출력: {self.cruise_speed*100:.1f}%, 최대 회전 출력: {self.max_angular*100:.1f}%\n"
            f"   - 상태: {wait_desc}\n"
            f"   - /odom 수신 대기 중..."
        )

    def _on_start_mission(self, msg: Bool):
        if msg.data:
            if not self.started:
                self.get_logger().info("🚀 [출발 신호 수신] /start_mission(True) 수신 — 원형 주행을 시작합니다!")
                self.started = True
        else:
            if self.started and not self.mission_finished:
                self.get_logger().warn("⏸️ [일시 정지] /start_mission(False) 수신 — 주행 일시 정지")
                self.started = False
                self._send_stop()

    def _on_start_service(self, request, response):
        self.get_logger().info("🚀 [출발 서비스 호출] /start_test 호출됨 — 원형 주행을 시작합니다!")
        self.started = True
        response.success = True
        response.message = "Circle drive test started successfully!"
        return response

    def _on_clear_trajectory(self, request, response):
        self.trajectory_history.clear()
        self._publish_markers()
        self.get_logger().info("🧹 [원형 주행 궤적 초기화] /clear_trajectory 호출됨 — 주행 궤적 이력을 초기화하고 마커를 갱신했습니다.")
        response.success = True
        response.message = "Trajectory history cleared and markers updated successfully."
        return response

    def _on_odom(self, msg: Odometry):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        self.current_yaw = yaw_from_quaternion(msg.pose.pose.orientation)
        self.current_yaw_rate = msg.twist.twist.angular.z
        self.last_odom_time = time.monotonic()

        if self.started and not self.mission_finished:
            self.trajectory_history.add_point(self.current_x, self.current_y)

    def _on_estop(self, msg: Bool):
        if msg.data and not self.emergency_stopped:
            self.get_logger().error("🛑 [비상 정지] /emergency_stop 수신! 즉각 제동.")
            self._send_stop()
        self.emergency_stopped = msg.data

    def _control_loop(self):
        # 1. 완료 및 비상정지 확인
        if self.mission_finished:
            if not self.stop_cmd_sent:
                self._send_stop()
                self.stop_cmd_sent = True
            return

        if self.emergency_stopped:
            self._send_stop()
            return

        # 2. 오도메트리 수신 확인
        if self.current_x is None or self.last_odom_time is None:
            return

        # 타임아웃 검사
        if time.monotonic() - self.last_odom_time > self.odom_timeout:
            self.get_logger().warn(
                f"⚠️ [CircleDriveTest] /odom 수신 지연 (> {self.odom_timeout}s) — 안전 정지",
                throttle_duration_sec=2.0
            )
            self._send_stop()
            return

        x, y, yaw = self.current_x, self.current_y, self.current_yaw

        # 3. 시작 신호 대기 확인
        if not self.started:
            self._send_stop()
            dist_to_center = math.hypot(x - self.center_x, y - self.center_y)
            self.get_logger().info(
                f"⏳ [시작 신호 대기 중] 현재 위치: ({x:.2f}, {y:.2f})m (중심 이격: {dist_to_center:.2f}m) | "
                f"노트북에서 출발 명령 대기: ros2 topic pub --once /start_mission std_msgs/msg/Bool \"{{data: true}}\"",
                throttle_duration_sec=3.0
            )
            return

        # 4. 바퀴 수 (Lap) 카운트 갱신
        dx = x - self.center_x
        dy = y - self.center_y
        current_polar_angle = math.atan2(dy, dx)

        if self.last_polar_angle is not None:
            d_theta = normalize_angle(current_polar_angle - self.last_polar_angle)
            # 진행 방향에 따른 회전각 누적
            self.accumulated_angle += d_theta * self.dir_sign
            self.current_laps = max(0.0, self.accumulated_angle / (2.0 * math.pi))

        self.last_polar_angle = current_polar_angle

        # 5. 목표 바퀴 수 도달 검사
        if self.target_laps > 0.0 and self.current_laps >= self.target_laps:
            self.get_logger().info(
                f"🏁 [원형 주행 완료!] 총 {self.current_laps:.2f}바퀴 완주 (목표: {self.target_laps}바퀴) — 안전 제동!")
            self._send_stop()
            self.mission_finished = True
            return

        # 6. 벡터 필드(Vector Field) 기반 원형 추종 제어
        # 현재 중심으로부터의 거리 및 반지름 오차
        dist_to_center = math.hypot(dx, dy)
        radial_error = dist_to_center - self.radius  # 양수: 원 바깥, 음수: 원 안쪽

        # 접선 방향 각도 (CCW: polar + pi/2, CW: polar - pi/2)
        tangent_angle = current_polar_angle + self.dir_sign * (math.pi / 2.0)

        # 원 궤도로 진입하기 위한 수렴 보정각 (atan 기반 부드러운 수렴)
        # 원 바깥이면 원 중심 방향(수렴)으로 선회, 원 안쪽이면 바깥 방향(확장)으로 선회
        converge_angle = self.dir_sign * math.atan2(self.k_converge * radial_error, self.radius)
        # 보정각 최대 +/- 60도 제한
        converge_angle = max(-math.pi / 3.0, min(math.pi / 3.0, converge_angle))

        desired_heading = normalize_angle(tangent_angle + converge_angle)
        heading_err = normalize_angle(desired_heading - yaw)

        # 7. PD 출력 계산
        raw_angular = self.kp_yaw * heading_err - self.kd_yaw * self.current_yaw_rate
        angular_cmd = max(-self.max_angular, min(self.max_angular, raw_angular))

        # 헤딩 오차가 클 때는 감속하여 안정적으로 방향 전환
        heading_alignment = max(0.2, math.cos(heading_err))
        linear_cmd = self.cruise_speed * heading_alignment

        # 8. 모터 명령 발행
        twist = Twist()
        twist.linear.x = float(linear_cmd)
        twist.angular.z = float(angular_cmd)
        self.cmd_pub.publish(twist)

        # 9. 터미널 로깅 (1초 주기)
        dir_label = "CCW" if self.dir_sign > 0 else "CW"
        self.get_logger().info(
            f"🔄 [{dir_label} 원형 주행] 위치: ({x:.2f}, {y:.2f})m | "
            f"반경오차: {radial_error*100:+.1f}cm | 헤딩오차: {math.degrees(heading_err):.1f}° | "
            f"진행: {self.current_laps:.2f}/{self.target_laps:.1f}바퀴 | "
            f"출력: [전진 {linear_cmd*100:.0f}%, 회전 {angular_cmd*100:.0f}%]",
            throttle_duration_sec=1.0
        )

    def _send_stop(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_pub.publish(twist)

    def _publish_markers(self):
        """RViz2 시각화 마커: 원 궤적, 중심점, 목표 헤딩 방향 표시."""
        stamp = self.get_clock().now().to_msg()
        ma = MarkerArray()

        # 1. 목표 원형 궤적 (초록색 링)
        circle_marker = Marker()
        circle_marker.header.stamp = stamp
        circle_marker.header.frame_id = 'odom'
        circle_marker.ns = 'orbit'
        circle_marker.id = 0
        circle_marker.type = Marker.LINE_STRIP
        circle_marker.action = Marker.ADD
        circle_marker.scale.x = 0.04
        circle_marker.color.r = 0.1
        circle_marker.color.g = 0.9
        circle_marker.color.b = 0.2
        circle_marker.color.a = 0.85

        num_segments = 60
        for i in range(num_segments + 1):
            theta = 2.0 * math.pi * (i % num_segments) / num_segments
            p = Point()
            p.x = self.center_x + self.radius * math.cos(theta)
            p.y = self.center_y + self.radius * math.sin(theta)
            p.z = 0.02
            circle_marker.points.append(p)
        ma.markers.append(circle_marker)

        # 2. 중심점 마커 (노란색 구)
        center_marker = Marker()
        center_marker.header.stamp = stamp
        center_marker.header.frame_id = 'odom'
        center_marker.ns = 'center'
        center_marker.id = 1
        center_marker.type = Marker.SPHERE
        center_marker.action = Marker.ADD
        center_marker.pose.position.x = self.center_x
        center_marker.pose.position.y = self.center_y
        center_marker.pose.position.z = 0.05
        center_marker.pose.orientation.w = 1.0
        center_marker.scale.x = 0.18
        center_marker.scale.y = 0.18
        center_marker.scale.z = 0.18
        center_marker.color.r = 1.0
        center_marker.color.g = 0.8
        center_marker.color.b = 0.0
        center_marker.color.a = 0.9
        ma.markers.append(center_marker)

        # 3. 실제 주행 궤적 마커 (오렌지색 LINE_STRIP)
        trail_marker = create_trail_marker(
            points=self.trajectory_history.points,
            frame_id='odom',
            stamp=stamp,
            ns='actual_trajectory',
            marker_id=0,
        )
        trail_marker.lifetime.sec = 1
        trail_marker.lifetime.nanosec = 0
        ma.markers.append(trail_marker)

        self.marker_pub.publish(ma)


def main(args=None):
    rclpy.init(args=args)
    node = CircleDriveTest()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node._send_stop()
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

