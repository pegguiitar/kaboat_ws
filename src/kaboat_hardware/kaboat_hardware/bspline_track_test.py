"""bspline_track_test — 실내 수조 (10m x 5m) B-Spline 곡선 경로 생성 및 추종 테스트 노드.

주요 기능:
  - 수조 안전 경계(10m x 5m 내벽 여유폭) 내에서 매끄러운 B-Spline 곡선 생성
  - 기본 제공 프리셋: S자 슬라럼 궤적 (수조를 대각/곡선으로 가로지르는 부드러운 코스)
  - 임의의 제어점(Control Points) 파라미터 입력 지원
  - 전방 주시(Lookahead) 기반 Pure Pursuit 경로 추종 및 P-D 헤딩 조향 제어
  - 곡률 및 헤딩 오차 적응형 감속, 목표 종점 정밀 접근 및 안전 제동
  - 수조 벽면 비상 정지 가드 및 /odom 타임아웃 워치독
  - RViz 실시간 시각화 (/bspline_test/path, /bspline_test/markers)
"""

import math
import time
from typing import List, Optional, Tuple

import numpy as np
from scipy.interpolate import BSpline

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Point, PoseStamped, Quaternion, Twist
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Bool
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker, MarkerArray

from kaboat_hardware.test_coordinates import BSPLINE_TRACK, get_bspline_control_points_xy
from kaboat_hardware.trajectory_visualization import (
    BoundedTrajectoryHistory,
    create_trail_marker,
    create_goal_tolerance_marker,
)


def normalize_angle(angle: float) -> float:
    """각도를 [-pi, pi] 범위로 정규화."""
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def yaw_from_quaternion(q: Quaternion) -> float:
    """쿼터니언에서 2D 평면 yaw 각도 추출."""
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def generate_bspline_path(
    control_points: List[Tuple[float, float]],
    spacing: float = 0.05,
    degree: int = 3
) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, float]:
    """제어점 목록으로부터 호길이 등간격 샘플링된 clamped B-spline을 생성.

    첫 점과 끝 점은 통과하지만, 중간 제어점은 곡선의 형상만 결정하며 반드시
    통과하지 않는다. knot 구성은 시뮬레이션의 B-spline 플래너와 동일한
    open-uniform(clamped) 방식이다.

    반환값:
      xs, ys: 샘플링된 곡선 좌표 [m]
      headings: 각 샘플 지점에서의 곡선 접선 방위각 [rad]
      curvatures: 각 샘플 지점에서의 곡률 kappa [1/m]
      total_length: 곡선 총 길이 [m]
    """
    pts = np.asarray(control_points, dtype=np.float64)
    if len(pts) < 2:
        raise ValueError("제어점은 최소 2개 이상이어야 합니다.")

    k = min(degree, len(pts) - 1)
    if k < 1:
        raise ValueError("B-spline 차수는 1 이상이어야 합니다.")

    # Open-uniform knot vector. 예: 제어점 6개, 3차이면
    # [0, 0, 0, 0, 1/3, 2/3, 1, 1, 1, 1]로 시뮬레이션과 동일하다.
    num_internal_knots = len(pts) - k - 1
    internal_knots = np.linspace(0.0, 1.0, num_internal_knots + 2)[1:-1]
    knots = np.concatenate((
        np.zeros(k + 1, dtype=np.float64),
        internal_knots,
        np.ones(k + 1, dtype=np.float64),
    ))
    spline = BSpline(knots, pts, k, axis=0)

    # 1. 조밀한 u 평가로 호길이 누적 계산
    dense_u = np.linspace(0.0, 1.0, 1000)
    dense_points = spline(dense_u)
    dense_x = dense_points[:, 0]
    dense_y = dense_points[:, 1]
    dx_dense = np.diff(dense_x)
    dy_dense = np.diff(dense_y)
    seg_lengths = np.hypot(dx_dense, dy_dense)
    cum_dist = np.concatenate([[0.0], np.cumsum(seg_lengths)])
    total_length = float(cum_dist[-1])

    # 2. 호길이 등간격(spacing) 샘플링
    if total_length <= spacing:
        target_s = np.array([0.0, total_length])
    else:
        target_s = np.arange(0.0, total_length, spacing)
        if target_s[-1] < total_length:
            target_s = np.append(target_s, total_length)

    u_sampled = np.interp(target_s, cum_dist, dense_u)

    sampled_points = spline(u_sampled)
    xs = sampled_points[:, 0]
    ys = sampled_points[:, 1]

    first_derivative = spline(u_sampled, nu=1)
    dx_du = first_derivative[:, 0]
    dy_du = first_derivative[:, 1]
    if k >= 2:
        second_derivative = spline(u_sampled, nu=2)
        d2x_du2 = second_derivative[:, 0]
        d2y_du2 = second_derivative[:, 1]
    else:
        # 선형 B-spline의 2차 미분은 전체 구간에서 0이다. SciPy는 차수보다
        # 높은 미분 요청을 거부하므로 명시적으로 0을 사용한다.
        d2x_du2 = np.zeros_like(dx_du)
        d2y_du2 = np.zeros_like(dy_du)

    headings = np.arctan2(dy_du, dx_du)
    # 곡률: kappa = (x' y'' - y' x'') / (x'^2 + y'^2)^(3/2)
    denom = np.power(dx_du**2 + dy_du**2, 1.5)
    denom = np.maximum(denom, 1e-6)
    curvatures = (dx_du * d2y_du2 - dy_du * d2x_du2) / denom

    return np.asarray(xs), np.asarray(ys), headings, curvatures, total_length


class BSplineTrackTest(Node):
    """실내 수조 B-Spline 곡선 경로 추종 테스트 노드."""

    def __init__(self):
        super().__init__('bspline_track_test')

        # ── 경로 파라미터 (test_coordinates.py 기본값 참조) ──
        default_cps_x, default_cps_y = get_bspline_control_points_xy()
        default_degree = int(BSPLINE_TRACK.get('spline_degree', 3))
        default_spacing = float(BSPLINE_TRACK.get('sample_spacing', 0.05))

        self.declare_parameter('control_points_x', default_cps_x)
        self.declare_parameter('control_points_y', default_cps_y)
        self.declare_parameter('spline_degree', default_degree)
        self.declare_parameter('sample_spacing', default_spacing)

        # ── 주행 및 제어 파라미터 ─────────────────────────
        self.declare_parameter('cruise_speed', 0.30)    # 전진 기본 출력 비율 (30%)
        self.declare_parameter('max_angular', 0.60)     # 최대 회전 출력 비율 (60%)
        self.declare_parameter('lookahead_dist', 0.6)   # Lookahead 전방 주시 거리 [m]
        self.declare_parameter('goal_tolerance', 0.40)  # 도착 판정 반경 [m]
        self.declare_parameter('slow_radius', 1.5)      # 종점 접근 감속 시작 반경 [m]
        self.declare_parameter('kp_yaw', 1.5)           # 헤딩 비례 게인 P
        self.declare_parameter('kd_yaw', 0.15)          # 요레이트 감쇠 게인 D
        self.declare_parameter('curvature_slowdown', 0.25)  # 곡률 기반 감속 가중치

        # ── 안전 파라미터 ─────────────────────────────────
        self.declare_parameter('odom_timeout_sec', 0.5) # /odom 타임아웃 [s]
        self.declare_parameter('wait_for_start', True)  # 외부 시작 신호(/start_mission) 대기 여부

        # 파라미터 취득
        cps_x = list(self.get_parameter('control_points_x').value)
        cps_y = list(self.get_parameter('control_points_y').value)
        self.degree = int(self.get_parameter('spline_degree').value)
        self.spacing = float(self.get_parameter('sample_spacing').value)

        self.cruise_speed = float(self.get_parameter('cruise_speed').value)
        self.max_angular = float(self.get_parameter('max_angular').value)
        self.lookahead = float(self.get_parameter('lookahead_dist').value)
        self.goal_tol = float(self.get_parameter('goal_tolerance').value)
        self.slow_radius = float(self.get_parameter('slow_radius').value)
        self.kp_yaw = float(self.get_parameter('kp_yaw').value)
        self.kd_yaw = float(self.get_parameter('kd_yaw').value)
        self.curvature_slowdown = float(self.get_parameter('curvature_slowdown').value)
        self.odom_timeout = float(self.get_parameter('odom_timeout_sec').value)
        self.wait_for_start = bool(self.get_parameter('wait_for_start').value)

        if len(cps_x) != len(cps_y):
            raise ValueError(f"control_points_x({len(cps_x)})와 control_points_y({len(cps_y)}) 길이가 다릅니다.")

        self.control_points = list(zip(cps_x, cps_y))

        # B-spline 경로 생성
        self.path_x, self.path_y, self.path_headings, self.path_curvatures, self.total_path_len = (
            generate_bspline_path(self.control_points, spacing=self.spacing, degree=self.degree)
        )
        self.num_points = len(self.path_x)

        # 상태 변수
        self.current_x: Optional[float] = None
        self.current_y: Optional[float] = None
        self.current_yaw: Optional[float] = None
        self.current_yaw_rate: float = 0.0
        self.last_odom_time: Optional[float] = None

        self.started = not self.wait_for_start
        self.progress_idx: int = 0
        self.emergency_stopped: bool = False
        self.mission_finished: bool = False
        self.stop_cmd_sent: bool = False
        self.trajectory_history = BoundedTrajectoryHistory()

        # 통신 인터페이스
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.path_pub = self.create_publisher(Path, '/bspline_test/path', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/bspline_test/markers', 10)

        self.create_subscription(Odometry, '/odom', self._on_odom, 10)
        self.create_subscription(Bool, '/emergency_stop', self._on_estop, 10)
        self.create_subscription(Bool, '/start_mission', self._on_start_mission, 10)
        self.create_service(Trigger, '/start_test', self._on_start_service)
        self.create_service(Trigger, '/clear_trajectory', self._on_clear_trajectory)

        # 20Hz 제어 루프
        self.create_timer(0.05, self._control_loop)
        # 2Hz RViz 시각화 경로/마커 발행
        self.create_timer(0.5, self._publish_rviz_vis)

        wait_msg = "외부 시작 신호(/start_mission) 대기 모드 활성화됨" if self.wait_for_start else "자동 즉시 시작 모드"
        self.get_logger().info(
            f"🌊 [BSplineTrackTest] 수조 B-Spline 추종 테스트 노드 준비 완료!\n"
            f"   - 제어점 개수: {len(self.control_points)}개, 생성된 곡선 샘플: {self.num_points}개\n"
            f"   - 총 경로 길이: {self.total_path_len:.2f}m (샘플 간격: {self.spacing*100:.1f}cm)\n"
            f"   - 시작점: ({self.path_x[0]:.2f}, {self.path_y[0]:.2f})m -> 종점: ({self.path_x[-1]:.2f}, {self.path_y[-1]:.2f})m\n"
            f"   - 기본 순항 출력: {self.cruise_speed*100:.1f}%, 전방 주시거리: {self.lookahead:.1f}m\n"
            f"   - 상태: {wait_msg}\n"
            f"   - /odom 수신 대기 중..."
        )

    def _on_start_mission(self, msg: Bool):
        if msg.data:
            if not self.started:
                self.get_logger().info("🚀 [출발 신호 수신] /start_mission(True) 수신 — B-Spline 곡선 추종을 시작합니다!")
                self.started = True
        else:
            if self.started and not self.mission_finished:
                self.get_logger().warn("⏸️ [일시 정지] /start_mission(False) 수신 — 곡선 추종 일시 정지")
                self.started = False
                self._send_stop()

    def _on_start_service(self, request, response):
        self.get_logger().info("🚀 [출발 서비스 호출] /start_test 호출됨 — B-Spline 곡선 추종을 시작합니다!")
        self.started = True
        response.success = True
        response.message = "B-Spline track test started successfully!"
        return response

    def _on_clear_trajectory(self, request, response):
        self.trajectory_history.clear()
        self._publish_rviz_vis()
        self.get_logger().info("🧹 [B-Spline 궤적 초기화] /clear_trajectory 호출됨 — 주행 궤적 이력을 초기화하고 마커를 갱신했습니다.")
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
            self.get_logger().error("🛑 [비상 정지] /emergency_stop 수신! 즉각 정지.")
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
                f"⚠️ [BSplineTrackTest] /odom 신호 {now - self.last_odom_time:.2f}s 지연 — 안전 정지",
                throttle_duration_sec=2.0)
            self._send_stop()
            return

        x, y, yaw = self.current_x, self.current_y, self.current_yaw

        # 3. 외부 시작 신호 대기 확인
        if not self.started:
            self._send_stop()
            start_dist = math.hypot(self.path_x[0] - x, self.path_y[0] - y)
            self.get_logger().info(
                f"⏳ [시작 신호 대기 중] 현재 위치: ({x:.2f}, {y:.2f})m (시작점 이격: {start_dist:.2f}m) | "
                f"노트북에서 출발 명령 대기: ros2 topic pub --once /start_mission std_msgs/msg/Bool \"{{data: true}}\"",
                throttle_duration_sec=3.0
            )
            return

        # 4. 곡선 위 최근접 진행 인덱스 탐색 (역행 방지: progress_idx 전방 윈도우 탐색)
        search_window = int(3.0 / self.spacing)  # 전방 3m 이내 탐색
        idx_start = max(0, self.progress_idx - 5)
        idx_end = min(self.num_points, self.progress_idx + search_window)

        sub_xs = self.path_x[idx_start:idx_end]
        sub_ys = self.path_y[idx_start:idx_end]
        dists = np.hypot(sub_xs - x, sub_ys - y)
        min_local_idx = int(np.argmin(dists))
        closest_idx = idx_start + min_local_idx

        # 진행 인덱스 단조 갱신
        self.progress_idx = max(self.progress_idx, closest_idx)

        # 5. 종점 도달 검사
        goal_x = self.path_x[-1]
        goal_y = self.path_y[-1]
        dist_to_goal = math.hypot(goal_x - x, goal_y - y)
        remaining_path_dist = (self.num_points - 1 - self.progress_idx) * self.spacing

        if dist_to_goal <= self.goal_tol or (remaining_path_dist <= self.goal_tol * 0.8):
            self.get_logger().info(
                f"🏁 [B-Spline 주행 완료!] 최종 위치: ({x:.2f}, {y:.2f})m, 남은 종점거리: {dist_to_goal:.2f}m — 안전 제동!")
            self._send_stop()
            self.mission_finished = True
            return

        # 6. Lookahead 목표점 산출
        lookahead_steps = int(self.lookahead / self.spacing)
        target_idx = min(self.num_points - 1, self.progress_idx + lookahead_steps)
        target_x = self.path_x[target_idx]
        target_y = self.path_y[target_idx]

        # 7. 목표 선수각 및 P-D 조향 제어
        desired_yaw = math.atan2(target_y - y, target_x - x)
        heading_error = normalize_angle(desired_yaw - yaw)
        angular_cmd = (self.kp_yaw * heading_error) - (self.kd_yaw * self.current_yaw_rate)
        angular_cmd = max(-self.max_angular, min(self.max_angular, angular_cmd))

        # 8. 속도 제어기 (곡률 및 헤딩 오차 적응형 감속 + 종점 접근 감속)
        speed = self.cruise_speed

        # (a) 헤딩 오차 감속
        if abs(heading_error) > math.radians(40):
            speed *= 0.4
        elif abs(heading_error) > math.radians(20):
            speed *= 0.7

        # (b) 곡선 곡률에 따른 감속
        local_curv = abs(self.path_curvatures[closest_idx])
        curv_factor = 1.0 / (1.0 + self.curvature_slowdown * local_curv)
        speed *= curv_factor

        # (c) 종점 접근 점진적 감속
        if remaining_path_dist < self.slow_radius:
            slow_ratio = max(0.35, remaining_path_dist / self.slow_radius)
            speed *= slow_ratio

        # 9. 명령 송출 (/cmd_vel)
        cmd = Twist()
        cmd.linear.x = float(speed)
        cmd.angular.z = float(angular_cmd)
        self.cmd_pub.publish(cmd)

        self.get_logger().info(
            f"곡선 추종 중 -> 현재: ({x:.2f}, {y:.2f})m | 진행: {self.progress_idx}/{self.num_points} | "
            f"목표점: ({target_x:.2f}, {target_y:.2f})m | 헤딩오차: {math.degrees(heading_error):.1f}° | "
            f"출력: [전진 {cmd.linear.x*100:.0f}%, 회전 {cmd.angular.z*100:.0f}%]",
            throttle_duration_sec=1.0
        )

    def _send_stop(self):
        """정지 명령 송출."""
        if not rclpy.ok():
            return
        try:
            stop_cmd = Twist()
            self.cmd_pub.publish(stop_cmd)
        except Exception:
            pass

    def _publish_rviz_vis(self):
        """RViz 시각화: B-Spline Path 및 제어점/타겟 MarkerArray 발행."""
        now = self.get_clock().now().to_msg()

        # 1. nav_msgs/Path 발행
        path_msg = Path()
        path_msg.header.stamp = now
        path_msg.header.frame_id = "odom"

        # 시각화 경량화를 위해 2점마다 1점씩 추가
        for px, py in zip(self.path_x[::2], self.path_y[::2]):
            pose = PoseStamped()
            pose.header.stamp = now
            pose.header.frame_id = "odom"
            pose.pose.position.x = float(px)
            pose.pose.position.y = float(py)
            pose.pose.position.z = 0.05
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)

        self.path_pub.publish(path_msg)

        # 2. 제어점 및 Lookahead 타겟 마커 발행
        ma = MarkerArray()

        # 제어점 마커 (노란색 구체들)
        for i, (cpx, cpy) in enumerate(self.control_points):
            m = Marker()
            m.header.stamp = now
            m.header.frame_id = "odom"
            m.ns = "bspline_control_points"
            m.id = i
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = float(cpx)
            m.pose.position.y = float(cpy)
            m.pose.position.z = 0.1
            m.scale.x = 0.25
            m.scale.y = 0.25
            m.scale.z = 0.25
            m.color.r = 1.0
            m.color.g = 0.8
            m.color.b = 0.0
            m.color.a = 0.7
            ma.markers.append(m)

        # 현재 Lookahead 타겟 마커 (청록색 구체)
        if self.current_x is not None and not self.mission_finished:
            lookahead_steps = int(self.lookahead / self.spacing)
            target_idx = min(self.num_points - 1, self.progress_idx + lookahead_steps)
            tx = float(self.path_x[target_idx])
            ty = float(self.path_y[target_idx])

            m_target = Marker()
            m_target.header.stamp = now
            m_target.header.frame_id = "odom"
            m_target.ns = "bspline_lookahead_target"
            m_target.id = 100
            m_target.type = Marker.SPHERE
            m_target.action = Marker.ADD
            m_target.pose.position.x = tx
            m_target.pose.position.y = ty
            m_target.pose.position.z = 0.15
            m_target.scale.x = 0.35
            m_target.scale.y = 0.35
            m_target.scale.z = 0.35
            m_target.color.r = 0.0
            m_target.color.g = 1.0
            m_target.color.b = 1.0
            m_target.color.a = 0.9
            ma.markers.append(m_target)

        # 3. 목표 종점 허용오차 링 마커 (노란색 원)
        goal_x = float(self.path_x[-1])
        goal_y = float(self.path_y[-1])
        m_goal_tol = create_goal_tolerance_marker(
            center_x=goal_x,
            center_y=goal_y,
            radius=self.goal_tol,
            frame_id="odom",
            stamp=now,
            ns="goal_tolerance",
            marker_id=0,
        )
        ma.markers.append(m_goal_tol)

        # 4. 실제 주행 궤적 마커 (오렌지색 LINE_STRIP)
        m_trail = create_trail_marker(
            points=self.trajectory_history.points,
            frame_id="odom",
            stamp=now,
            ns="actual_trajectory",
            marker_id=0,
        )
        m_trail.lifetime.sec = 1
        m_trail.lifetime.nanosec = 0
        ma.markers.append(m_trail)

        self.marker_pub.publish(ma)


def main(args=None):
    rclpy.init(args=args)
    node = BSplineTrackTest()
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
