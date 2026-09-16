"""straight_line_test — 실내 수조 (9.0, 3.0) -> (1.0, 3.0) 직선 경로 주행 테스트 노드.

주행 개요:
  - 시작점: (X = 9.0m, Y = 3.0m)
  - 목표점: (X = 1.0m, Y = 3.0m)
  - 총 이동거리: 8.0m (수조 우측에서 좌측으로 -X 방향 직진)
  - 목표 선수각(Heading): 180° (π rad)

제어 방식:
  - 경로 추종: Line of Sight (LOS) 가이던스 (Cross-track 오차 기반 조향각 산출)
  - 헤딩 제어: P-D 조향 제어 (Heading 오차 P + IMU 자이로 각속도 D)
  - 속도 제어: 목표점 접근 시(1.5m 이내) 감속, 도착 시(0.35m 이내) 즉시 정지

안전 기능:
  - 수조 벽면 가드: X < 0.5m, X > 9.5m, Y < 0.5m, Y > 4.5m 벗어날 시 비상 정지
  - 오도메트리 워치독: /odom 수신이 0.5초 이상 끊기면 즉각 정지 명령 송출
  - 비상 정지 토픽 연동: /emergency_stop 수신 시 즉각 모터 정지
  - 안전 저속 기본값: cruise_speed = 0.12 (최대 추력의 약 12%)
"""

import math
import time
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy

from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker, MarkerArray

from kaboat_hardware.test_coordinates import STRAIGHT_LINE


def yaw_from_quaternion(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def normalize_angle(a: float) -> float:
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


class StraightLineTest(Node):
    def __init__(self):
        super().__init__('straight_line_test')

        # ── 경로 좌표 파라미터 (test_coordinates.py 기본값 참조) ──
        self.declare_parameter('start_x', float(STRAIGHT_LINE['start_x']))
        self.declare_parameter('start_y', float(STRAIGHT_LINE['start_y']))
        self.declare_parameter('goal_x', float(STRAIGHT_LINE['goal_x']))
        self.declare_parameter('goal_y', float(STRAIGHT_LINE['goal_y']))

        # ── 주행 및 제어 파라미터 ─────────────────────────
        self.declare_parameter('cruise_speed', 0.12)    # 전진 출력 비율 (0.0 ~ 1.0, 기본 12%)
        self.declare_parameter('max_angular', 0.80)     # 최대 회전 출력 비율 (80%)
        self.declare_parameter('lookahead_dist', 1.2)   # LOS 경로 추종 전방 주시 거리 [m]
        self.declare_parameter('goal_tolerance', 0.35)  # 도착 판정 반경 [m]
        self.declare_parameter('slow_radius', 1.5)      # 목표점 접근 감속 반경 [m]
        self.declare_parameter('kp_yaw', 1.0)           # 헤딩 P 게인
        self.declare_parameter('kd_yaw', 0.15)          # 요레이트 D 게인 (감쇠)

        # ── 안전 파라미터 ─────────────────────────────────
        self.declare_parameter('odom_timeout_sec', 0.5) # /odom 타임아웃 [s]
        self.declare_parameter('wait_for_start', True)  # 외부 시작 신호(/start_mission) 대기 여부

        # 파라미터 로드
        self.start_x = float(self.get_parameter('start_x').value)
        self.start_y = float(self.get_parameter('start_y').value)
        self.goal_x = float(self.get_parameter('goal_x').value)
        self.goal_y = float(self.get_parameter('goal_y').value)

        self.cruise_speed = float(self.get_parameter('cruise_speed').value)
        self.max_angular = float(self.get_parameter('max_angular').value)
        self.lookahead = float(self.get_parameter('lookahead_dist').value)
        self.goal_tol = float(self.get_parameter('goal_tolerance').value)
        self.slow_radius = float(self.get_parameter('slow_radius').value)
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

        self.started = not self.wait_for_start
        self.emergency_stopped = False
        self.mission_finished = False
        self.stop_cmd_sent = False

        # 통신 인터페이스
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/straight_drive/markers', 10)

        self.create_subscription(Odometry, '/odom', self._on_odom, 10)
        self.create_subscription(Bool, '/emergency_stop', self._on_estop, 10)
        self.create_subscription(Bool, '/start_mission', self._on_start_mission, 10)
        self.create_service(Trigger, '/start_test', self._on_start_service)

        # 20Hz 제어 루프
        self.create_timer(0.05, self._control_loop)
        # 2Hz RViz 경로 마커 발행
        self.create_timer(0.5, self._publish_path_markers)

        total_dist = math.hypot(self.goal_x - self.start_x, self.goal_y - self.start_y)
        path_angle = math.degrees(math.atan2(self.goal_y - self.start_y, self.goal_x - self.start_x))
        wait_msg = "외부 시작 신호(/start_mission) 대기 모드 활성화됨" if self.wait_for_start else "자동 즉시 시작 모드"
        self.get_logger().info(
            f"🚀 [StraightLineTest] 직선 주행 테스트 준비 완료!\n"
            f"   - 구간: ({self.start_x:.1f}, {self.start_y:.1f})m -> ({self.goal_x:.1f}, {self.goal_y:.1f})m\n"
            f"   - 총 주행거리: {total_dist:.2f}m (방위각: {path_angle:.1f}°)\n"
            f"   - 기본 출력: {self.cruise_speed*100:.1f}%, 도착 오차: {self.goal_tol}m\n"
            f"   - 상태: {wait_msg}\n"
            f"   - /odom 수신 대기 중..."
        )

    def _on_start_mission(self, msg: Bool):
        if msg.data:
            if not self.started:
                self.get_logger().info("🚀 [출발 신호 수신] /start_mission(True) 수신 — 직선 주행을 시작합니다!")
                self.started = True
        else:
            if self.started and not self.mission_finished:
                self.get_logger().warn("⏸️ [일시 정지] /start_mission(False) 수신 — 주행 일시 정지")
                self.started = False
                self._send_stop()

    def _on_start_service(self, request, response):
        self.get_logger().info("🚀 [출발 서비스 호출] /start_test 호출됨 — 직선 주행을 시작합니다!")
        self.started = True
        response.success = True
        response.message = "Straight line test started successfully!"
        return response

    def _on_odom(self, msg: Odometry):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        self.current_yaw = yaw_from_quaternion(msg.pose.pose.orientation)
        self.current_yaw_rate = msg.twist.twist.angular.z
        self.last_odom_time = time.monotonic()

    def _on_estop(self, msg: Bool):
        if msg.data and not self.emergency_stopped:
            self.get_logger().error("🛑 [비상 정지] /emergency_stop 수신! 모터 즉각 정지.")
            self._send_stop()
        self.emergency_stopped = msg.data

    def _control_loop(self):
        # 1. 종료 또는 비상 정지 상태 확인
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
            return  # 첫 odom 대기

        now = time.monotonic()
        if (now - self.last_odom_time) > self.odom_timeout:
            self.get_logger().warn(
                f"⚠️ [StraightLineTest] /odom 신호 {now - self.last_odom_time:.2f}s 지연 — 안전 정지 유지",
                throttle_duration_sec=2.0)
            self._send_stop()
            return

        x, y, yaw = self.current_x, self.current_y, self.current_yaw

        # 3. 외부 시작 신호 대기 확인
        if not self.started:
            self._send_stop()
            dist_to_start = math.hypot(self.start_x - x, self.start_y - y)
            self.get_logger().info(
                f"⏳ [시작 신호 대기 중] 현재 위치: ({x:.2f}, {y:.2f})m (시작점 이격: {dist_to_start:.2f}m) | "
                f"노트북에서 출발 명령 대기: ros2 topic pub --once /start_mission std_msgs/msg/Bool \"{{data: true}}\"",
                throttle_duration_sec=3.0
            )
            return

        # 4. 목표점 도달 검사
        dist_to_goal = math.hypot(self.goal_x - x, self.goal_y - y)
        if dist_to_goal <= self.goal_tol or (x <= self.goal_x and abs(y - self.goal_y) <= 0.6):
            self.get_logger().info(
                f"🏁 [목표 도달!] 최종 위치 -> X: {x:.2f}m, Y: {y:.2f}m (남은거리: {dist_to_goal:.2f}m) — 주행 완료 및 정지!")
            self._send_stop()
            self.mission_finished = True
            return

        # 5. Line of Sight (LOS) 경로 가이던스 계산
        # 직선 벡터 A -> B
        dx_line = self.goal_x - self.start_x  # -8.0
        dy_line = self.goal_y - self.start_y  # 0.0
        line_len = math.hypot(dx_line, dy_line)
        path_heading = math.atan2(dy_line, dx_line)  # π (180도)

        # 경로 단위 벡터 및 법선 벡터
        ux = dx_line / line_len
        uy = dy_line / line_len

        # Cross-track 오차 (경로 좌/우 이탈 거리)
        # start -> current 벡터와 법선(-uy, ux)의 내적
        dx_p = x - self.start_x
        dy_p = y - self.start_y
        cross_track_error = -uy * dx_p + ux * dy_p  # y - 3.0 (y=3.0 기준 위로 벗어나면 음수)

        # LOS 보정각: 경로로 복귀하기 위한 진입각
        correction_angle = math.atan2(-cross_track_error, self.lookahead)
        desired_yaw = normalize_angle(path_heading + correction_angle)

        # 6. P-D 조향 제어기
        heading_error = normalize_angle(desired_yaw - yaw)
        angular_cmd = (self.kp_yaw * heading_error) - (self.kd_yaw * self.current_yaw_rate)
        angular_cmd = max(-self.max_angular, min(self.max_angular, angular_cmd))

        # 7. 전진 속도 계산 (감속 로직)
        speed = self.cruise_speed
        # 헤딩 오차가 크면 회전 우선 감속
        if abs(heading_error) > math.radians(40):
            speed *= 0.4
        elif abs(heading_error) > math.radians(20):
            speed *= 0.7

        # 목표점 1.5m 이내 접근 시 점진적 감속
        if dist_to_goal < self.slow_radius:
            slowdown = max(0.35, dist_to_goal / self.slow_radius)
            speed *= slowdown

        # 8. 모터 명령 발행 (/cmd_vel)
        cmd = Twist()
        cmd.linear.x = float(speed)
        cmd.angular.z = float(angular_cmd)
        self.cmd_pub.publish(cmd)

        self.get_logger().info(
            f"주행 중 -> 현재위치: ({x:.2f}, {y:.2f})m | 목표거리: {dist_to_goal:.2f}m | "
            f"선수각: {math.degrees(yaw):.1f}° (오차: {math.degrees(heading_error):.1f}°) | "
            f"출력: [전진 {cmd.linear.x*100:.0f}%, 회전 {cmd.angular.z*100:.0f}%]",
            throttle_duration_sec=1.0
        )

    def _send_stop(self):
        """정지 명령(/cmd_vel = 0) 송출."""
        if not rclpy.ok():
            return
        try:
            stop_cmd = Twist()
            self.cmd_pub.publish(stop_cmd)
        except Exception:
            pass

    def _publish_path_markers(self):
        """RViz 시각화용 목표 직선 경로 및 시작/도착점 마커."""
        ma = MarkerArray()
        now = self.get_clock().now().to_msg()

        # 1. 목표 직선 경로 라인
        m_line = Marker()
        m_line.header.stamp = now
        m_line.header.frame_id = "odom"
        m_line.ns = "test_path"
        m_line.id = 10
        m_line.type = Marker.LINE_STRIP
        m_line.action = Marker.ADD
        m_line.scale.x = 0.06  # 선 두께
        m_line.color.r = 0.0
        m_line.color.g = 1.0
        m_line.color.b = 0.5
        m_line.color.a = 0.9
        p_start = Point(x=self.start_x, y=self.start_y, z=0.05)
        p_goal = Point(x=self.goal_x, y=self.goal_y, z=0.05)
        m_line.points = [p_start, p_goal]
        ma.markers.append(m_line)

        # 2. 시작점 마커 (초록 원통)
        m_start = Marker()
        m_start.header.stamp = now
        m_start.header.frame_id = "odom"
        m_start.ns = "test_start"
        m_start.id = 11
        m_start.type = Marker.CYLINDER
        m_start.action = Marker.ADD
        m_start.pose.position.x = self.start_x
        m_start.pose.position.y = self.start_y
        m_start.pose.position.z = 0.1
        m_start.scale.x = 0.3
        m_start.scale.y = 0.3
        m_start.scale.z = 0.1
        m_start.color.r = 0.0
        m_start.color.g = 1.0
        m_start.color.b = 0.0
        m_start.color.a = 0.8
        ma.markers.append(m_start)

        # 3. 목표점 마커 (빨강 체커/원통)
        m_goal = Marker()
        m_goal.header.stamp = now
        m_goal.header.frame_id = "odom"
        m_goal.ns = "test_goal"
        m_goal.id = 12
        m_goal.type = Marker.CYLINDER
        m_goal.action = Marker.ADD
        m_goal.pose.position.x = self.goal_x
        m_goal.pose.position.y = self.goal_y
        m_goal.pose.position.z = 0.1
        m_goal.scale.x = 0.4
        m_goal.scale.y = 0.4
        m_goal.scale.z = 0.1
        m_goal.color.r = 1.0
        m_goal.color.g = 0.1
        m_goal.color.b = 0.1
        m_goal.color.a = 0.8
        ma.markers.append(m_goal)

        self.marker_pub.publish(ma)


def main(args=None):
    rclpy.init(args=args)
    node = StraightLineTest()
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

