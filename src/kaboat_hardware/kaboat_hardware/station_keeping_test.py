"""station_keeping_test — 실내 수조 정해진 웨이포인트 정점 유지(Station Keeping/DP) 테스트.

수조(10m x 5m) 내 지정된 목표 좌표(X, Y) 및 선수각(Yaw)에 배를 고정시키는
가상 앵커(Virtual Anchor / Dynamic Positioning) 제어를 수행합니다.

특징:
  - 수류나 관성에 의해 목표 지점에서 밀려나면 차동 모터를 자동 구동하여 복귀
  - 모터 떨림(Chattering) 및 배터리 낭비 방지를 위한 불감대(Deadband) 제어 내장
    (거리 오차 < 15cm, 각도 오차 < 8° 이내면 모터 정지)
  - 목표 유지 시간(기본 무한 유지, 초 단위 설정 가능) 지정 기능
  - RViz2 시각화 마커(/station_keeping/markers) 발행 (목표점, 허용반경, 목표 방위 화살표)
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


class StationKeepingTest(Node):
    def __init__(self):
        super().__init__('station_keeping_test')

        # ── 목표 위치 및 방위 파라미터 ───────────────────────
        self.declare_parameter('target_x', 5.0)          # 목표 X 위치 [m] (수조 중앙)
        self.declare_parameter('target_y', 2.5)          # 목표 Y 위치 [m] (수조 중앙)
        self.declare_parameter('target_yaw_deg', 180.0)  # 목표 선수각 [deg] (-999.0: 위치만 유지)
        self.declare_parameter('hold_duration_sec', 0.0) # 유지 시간 [s] (0.0: 무한 유지)

        # ── 제어 및 불감대(Deadband) 파라미터 ────────────────
        self.declare_parameter('pos_deadband', 0.15)     # 위치 유지 허용 불감대 반경 [m] (15cm)
        self.declare_parameter('yaw_deadband_deg', 8.0)  # 헤딩 유지 허용 불감대 각도 [deg] (8°)
        self.declare_parameter('max_fwd_speed', 0.12)    # 최대 전진 출력 비 (12%)
        self.declare_parameter('max_rev_speed', 0.08)    # 최대 후진 출력 비 (8%)
        self.declare_parameter('max_angular', 0.35)      # 최대 회전 출력 비 (35%)
        self.declare_parameter('kp_pos', 0.25)           # 위치 오차 비례(P) 게인
        self.declare_parameter('kp_yaw', 0.40)           # 헤딩 오차 비례(P) 게인
        self.declare_parameter('kd_yaw', 0.12)           # 요레이트 감쇠(D) 게인

        # ── 안전 파라미터 ─────────────────────────────────
        self.declare_parameter('odom_timeout_sec', 0.5)  # /odom 타임아웃 [s]
        self.declare_parameter('wait_for_start', True)   # 외부 시작 신호(/start_mission) 대기 여부

        # 파라미터 로드
        self.target_x = float(self.get_parameter('target_x').value)
        self.target_y = float(self.get_parameter('target_y').value)
        yaw_deg = float(self.get_parameter('target_yaw_deg').value)
        self.hold_heading = (yaw_deg > -900.0)
        self.target_yaw = math.radians(yaw_deg) if self.hold_heading else 0.0

        self.hold_duration = float(self.get_parameter('hold_duration_sec').value)
        self.pos_deadband = float(self.get_parameter('pos_deadband').value)
        self.yaw_deadband = math.radians(float(self.get_parameter('yaw_deadband_deg').value))

        self.max_fwd_speed = float(self.get_parameter('max_fwd_speed').value)
        self.max_rev_speed = float(self.get_parameter('max_rev_speed').value)
        self.max_angular = float(self.get_parameter('max_angular').value)
        self.kp_pos = float(self.get_parameter('kp_pos').value)
        self.kp_yaw = float(self.get_parameter('kp_yaw').value)
        self.kd_yaw = float(self.get_parameter('kd_yaw').value)
        self.odom_timeout = float(self.get_parameter('odom_timeout_sec').value)
        self.wait_for_start = bool(self.get_parameter('wait_for_start').value)

        # 상태 변수
        self.current_x: Optional[float] = None
        self.current_y: Optional[float] = None
        self.current_yaw: Optional[float] = None
        self.current_yaw_rate: float = 0.0
        self.last_odom_time: Optional[float] = None

        self.hold_start_time: Optional[float] = None
        self.started = not self.wait_for_start
        self.mission_finished = False
        self.emergency_stopped = False
        self.stop_cmd_sent = False

        # 통신 인터페이스
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/station_keeping/markers', 10)

        self.create_subscription(Odometry, '/odom', self._on_odom, 10)
        self.create_subscription(Bool, '/emergency_stop', self._on_estop, 10)
        self.create_subscription(Bool, '/start_mission', self._on_start_mission, 10)
        self.create_service(Trigger, '/start_test', self._on_start_service)

        # 20Hz 제어 루프 & 2Hz 마커 시각화 타이머
        self.create_timer(0.05, self._control_loop)
        self.create_timer(0.5, self._publish_markers)

        heading_desc = f"{yaw_deg:.1f}°" if self.hold_heading else "자유 헤딩 (위치만 고정)"
        duration_desc = f"{self.hold_duration:.1f}초" if self.hold_duration > 0 else "무한 정점 유지"
        wait_desc = "외부 시작 신호(/start_mission) 대기 모드" if self.wait_for_start else "자동 즉시 시작 모드"

        self.get_logger().info(
            f"⚓ [StationKeepingTest] 웨이포인트 정점 유지(DP) 테스트 준비 완료!\n"
            f"   - 목표 좌표: ({self.target_x:.2f}, {self.target_y:.2f})m, 목표 헤딩: {heading_desc}\n"
            f"   - 불감대(Deadband): 위치 {self.pos_deadband*100:.0f}cm, 각도 {math.degrees(self.yaw_deadband):.1f}°\n"
            f"   - 유지 기간: {duration_desc}\n"
            f"   - 상태: {wait_desc}\n"
            f"   - /odom 수신 대기 중..."
        )

    def _on_start_mission(self, msg: Bool):
        if msg.data:
            if not self.started:
                self.get_logger().info("🚀 [출발 신호 수신] /start_mission(True) 수신 — 웨이포인트 유지를 시작합니다!")
                self.started = True
        else:
            if self.started and not self.mission_finished:
                self.get_logger().warn("⏸️ [일시 정지] /start_mission(False) 수신 — 정점 유지 일시 정지")
                self.started = False
                self._send_stop()

    def _on_start_service(self, request, response):
        self.get_logger().info("🚀 [출발 서비스 호출] /start_test 호출됨 — 웨이포인트 유지를 시작합니다!")
        self.started = True
        response.success = True
        response.message = "Station keeping test started successfully!"
        return response

    def _on_odom(self, msg: Odometry):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        self.current_yaw = yaw_from_quaternion(msg.pose.pose.orientation)
        self.current_yaw_rate = msg.twist.twist.angular.z
        self.last_odom_time = time.monotonic()

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
                f"⚠️ [StationKeepingTest] /odom 수신 지연 (> {self.odom_timeout}s) — 안전 정지",
                throttle_duration_sec=2.0
            )
            self._send_stop()
            return

        x, y, yaw = self.current_x, self.current_y, self.current_yaw

        # 3. 시작 신호 대기 확인
        if not self.started:
            self._send_stop()
            dist_to_target = math.hypot(self.target_x - x, self.target_y - y)
            self.get_logger().info(
                f"⏳ [시작 신호 대기 중] 현재 위치: ({x:.2f}, {y:.2f})m (목표점 이격: {dist_to_target:.2f}m) | "
                f"노트북에서 출발 명령 대기: ros2 topic pub --once /start_mission std_msgs/msg/Bool \"{{data: true}}\"",
                throttle_duration_sec=3.0
            )
            return

        # 4. 목표점 오차 계산
        dx = self.target_x - x
        dy = self.target_y - y
        dist_err = math.hypot(dx, dy)
        bearing_to_target = math.atan2(dy, dx)
        rel_bearing = normalize_angle(bearing_to_target - yaw)

        # 5. 불감대(Deadband) 판정
        in_pos_deadband = (dist_err <= self.pos_deadband)

        if self.hold_heading:
            yaw_err = normalize_angle(self.target_yaw - yaw)
            in_yaw_deadband = (abs(yaw_err) <= self.yaw_deadband)
        else:
            yaw_err = 0.0
            in_yaw_deadband = True

        # 6. 유지 시간 검사
        if in_pos_deadband and in_yaw_deadband:
            if self.hold_start_time is None:
                self.hold_start_time = time.monotonic()
                self.get_logger().info("⚓ [정점 안착!] 목표 웨이포인트 불감대 진입 — 가상 앵커 고정 시작")

            # 목표 유지 시간 설정 시 자동 종료 판정
            if self.hold_duration > 0.0:
                elapsed = time.monotonic() - self.hold_start_time
                if elapsed >= self.hold_duration:
                    self.get_logger().info(
                        f"🏁 [웨이포인트 유지 완료!] {elapsed:.1f}초 동안 정점 유지 완료 — 시험 종료!")
                    self._send_stop()
                    self.mission_finished = True
                    return

            # 완벽히 안착한 상태: 불필요한 모터 채터링 방지 (출력 0)
            self._send_stop()
            self.get_logger().info(
                f"⚓ [정점 유지 중] 위치오차: {dist_err*100:.1f}cm | 헤딩오차: {math.degrees(yaw_err):.1f}° (데드밴드 내 모터 휴지)",
                throttle_duration_sec=2.0
            )
            return
        else:
            self.hold_start_time = None  # 불감대 벗어나면 카운트 리셋

        # 7. 차동 추진 제어 명령 산출
        linear_cmd = 0.0
        angular_cmd = 0.0

        if not in_pos_deadband:
            # 1) 위치 오차가 큰 경우: 목표점을 향해 전진 또는 후진 복귀
            if abs(rel_bearing) <= math.pi / 2.0:
                # 전방 영역: 전진 추진 및 헤딩 지향
                desired_speed = min(self.max_fwd_speed, max(0.04, self.kp_pos * dist_err))
                linear_cmd = desired_speed * max(0.0, math.cos(rel_bearing))

                raw_angular = self.kp_yaw * rel_bearing - self.kd_yaw * self.current_yaw_rate
                angular_cmd = max(-self.max_angular, min(self.max_angular, raw_angular))
            else:
                # 후방 영역: 근거리(35cm 이내)면 후진, 원거리면 선회 후 전진
                rev_bearing = normalize_angle(rel_bearing - math.pi)
                if dist_err < 0.35:
                    desired_speed = min(self.max_rev_speed, max(0.04, self.kp_pos * dist_err))
                    linear_cmd = -desired_speed * max(0.0, math.cos(rev_bearing))

                    raw_angular = self.kp_yaw * rev_bearing - self.kd_yaw * self.current_yaw_rate
                    angular_cmd = max(-self.max_angular, min(self.max_angular, raw_angular))
                else:
                    # 원거리 후방이면 먼저 뱃머리를 목표 방향으로 회전
                    linear_cmd = 0.0
                    raw_angular = self.kp_yaw * rel_bearing - self.kd_yaw * self.current_yaw_rate
                    angular_cmd = max(-self.max_angular, min(self.max_angular, raw_angular))
        else:
            # 2) 위치는 데드밴드 안쪽인데 헤딩 각도만 벗어난 경우: 제자리 회전 정렬
            linear_cmd = 0.0
            raw_angular = self.kp_yaw * yaw_err - self.kd_yaw * self.current_yaw_rate
            angular_cmd = max(-self.max_angular, min(self.max_angular, raw_angular))

        # 8. 모터 명령 발행
        twist = Twist()
        twist.linear.x = float(linear_cmd)
        twist.angular.z = float(angular_cmd)
        self.cmd_pub.publish(twist)

        # 9. 터미널 로깅 (1초 주기)
        self.get_logger().info(
            f"⚓ [정점 복귀 중] 위치: ({x:.2f}, {y:.2f})m | "
            f"이격거리: {dist_err*100:.1f}cm | 헤딩오차: {math.degrees(yaw_err):.1f}° | "
            f"출력: [전진 {linear_cmd*100:.0f}%, 회전 {angular_cmd*100:.0f}%]",
            throttle_duration_sec=1.0
        )

    def _send_stop(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_pub.publish(twist)

    def _publish_markers(self):
        """RViz2 시각화 마커: 목표 위치, 데드밴드 원, 목표 방위 화살표."""
        stamp = self.get_clock().now().to_msg()
        ma = MarkerArray()

        # 1. 목표 위치 구 (보라색 구)
        target_marker = Marker()
        target_marker.header.stamp = stamp
        target_marker.header.frame_id = 'odom'
        target_marker.ns = 'waypoint'
        target_marker.id = 0
        target_marker.type = Marker.SPHERE
        target_marker.action = Marker.ADD
        target_marker.pose.position.x = self.target_x
        target_marker.pose.position.y = self.target_y
        target_marker.pose.position.z = 0.05
        target_marker.pose.orientation.w = 1.0
        target_marker.scale.x = 0.15
        target_marker.scale.y = 0.15
        target_marker.scale.z = 0.15
        target_marker.color.r = 0.7
        target_marker.color.g = 0.2
        target_marker.color.b = 0.9
        target_marker.color.a = 0.9
        ma.markers.append(target_marker)

        # 2. 허용 불감대 링 (노란색 원)
        ring_marker = Marker()
        ring_marker.header.stamp = stamp
        ring_marker.header.frame_id = 'odom'
        ring_marker.ns = 'deadband'
        ring_marker.id = 1
        ring_marker.type = Marker.LINE_STRIP
        ring_marker.action = Marker.ADD
        ring_marker.scale.x = 0.03
        ring_marker.color.r = 1.0
        ring_marker.color.g = 0.85
        ring_marker.color.b = 0.1
        ring_marker.color.a = 0.8

        num_segments = 36
        for i in range(num_segments + 1):
            theta = 2.0 * math.pi * (i % num_segments) / num_segments
            p = Point()
            p.x = self.target_x + self.pos_deadband * math.cos(theta)
            p.y = self.target_y + self.pos_deadband * math.sin(theta)
            p.z = 0.02
            ring_marker.points.append(p)
        ma.markers.append(ring_marker)

        # 3. 목표 선수각 화살표 (청록색 화살표)
        if self.hold_heading:
            arrow_marker = Marker()
            arrow_marker.header.stamp = stamp
            arrow_marker.header.frame_id = 'odom'
            arrow_marker.ns = 'heading'
            arrow_marker.id = 2
            arrow_marker.type = Marker.ARROW
            arrow_marker.action = Marker.ADD
            arrow_marker.pose.position.x = self.target_x
            arrow_marker.pose.position.y = self.target_y
            arrow_marker.pose.position.z = 0.08
            arrow_marker.pose.orientation = yaw_to_quaternion(self.target_yaw)
            arrow_marker.scale.x = 0.5   # 화살표 길이
            arrow_marker.scale.y = 0.06  # 화살표 두께
            arrow_marker.scale.z = 0.06
            arrow_marker.color.r = 0.1
            arrow_marker.color.g = 0.9
            arrow_marker.color.b = 0.9
            arrow_marker.color.a = 0.95
            ma.markers.append(arrow_marker)

        self.marker_pub.publish(ma)


def main(args=None):
    rclpy.init(args=args)
    node = StationKeepingTest()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._send_stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
