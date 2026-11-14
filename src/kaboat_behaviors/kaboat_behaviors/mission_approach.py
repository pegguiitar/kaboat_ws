"""Move to a configured mission entry pose before its behavior starts.

도착 조건 (2026-11-12 야외 1 m·±15° → 2026-11-13 2 m·±30°): 시작 지점 2 m 안 + 선수각 ±30° 안을
settle_time(0.5 s) 유지. 회전 속도 조건은 없다(approach.settle_yaw_rate 0). 공용 waypoint() 기본값
(0.35 m / 8° / 0.1 rad/s)은 바람·물살이 있는 야외 시작 자세 맞추기에 너무 엄격해
approach 에서 gate 등 미션으로 넘어가지 못했다. 다른 behavior 의 waypoint() 는 그대로.

출력 상한 (2026-11-14 오후): 전진 0.8 (≈1825 µs)·회전 0.4, 목표 6 m 앞부터 감속.
  = max_linear 1.0 × thrust_ratio 0.8, max_angular 0.5 × 0.8. 회전은 오전 값(0.4) 그대로 — 전진 1.0 이면
  회전 명령을 더할 때 한쪽 모터가 잘려 조향 여유가 없고, 선회 상한까지 1.0 이면 제자리 회전이 풀파워라서.
  감속 6 m (공용 1.5 m + 도착 반경 2배 4 m 대신): 브레이크 없이 추력만 줄이므로 일찍 줄여 다음 미션에
  넘겨줄 때 속도를 예전과 비슷하게 (모델: gate 0.44 → 0.50 m/s).
  근거 — run2_20261114_145941 bag 이동 5구간 120 s 를 이 노드 제어 코드 + 믹서 + 11-14 배 모델로 다시 돌려
  (예전 값 계산 120.0 s = bag 120.0 s) 새 값 89.7 s. 0.8 의 속도(모델 1.59 m/s)는 실측(0.47, 1.2 m/s)
  밖이라 실제가 1.35 m/s 면 100 s. 단독 시험 goto(go_*·outdoor_waypoint)는 예전 0.4 그대로.
  오전 (0.4·0.4): 예전 공용 behavior 스케일(전진 0.101·회전 0.31) × 0.2 = 전진 0.0202·회전 0.062 라
  야외에서 물살에 밀려 시작 자세에 못 갔다(2026-11-13 dock_test1 bag: search_entry 에서 2.6 → 9.3 m).
"""

import dataclasses
import math

import rclpy
from geometry_msgs.msg import PoseStamped, Twist

from .behavior_base import BehaviorBase, yaw_from_quaternion
from .waypoint_controller import WaypointController, WaypointParams

APPROACH_POSITION_TOLERANCE = 2.0      # [m] (1.0 → 2.0, 2026-11-13)
APPROACH_HEADING_TOLERANCE_DEG = 30.0  # [deg] 좌우 (15 → 30, 2026-11-13)
APPROACH_SETTLE_YAW_RATE = 0.0         # [rad/s] 0 = 회전 속도 조건 없음
# 출력 상한 = 상한 × thrust_ratio → 전진 1.0 × 0.8 = 0.8, 회전 0.5 × 0.8 = 0.4 (2026-11-14 오후)
APPROACH_MAX_LINEAR = 1.0
APPROACH_MAX_ANGULAR = 0.5              # (1.0 → 0.5) 회전 상한은 0.4 그대로 두려고
APPROACH_THRUST_RATIO = 0.8             # (0.2 → 0.4 → 0.8, 2026-11-14) 전진 ≈1825 µs
APPROACH_SLOW_RADIUS = 6.0              # [m] 이 거리부터 전진을 거리에 비례해 줄인다 (공용 1.5 m)


def approach_params(base: WaypointParams, position_tolerance: float,
                    heading_tolerance_deg: float,
                    settle_yaw_rate: float, *, max_linear: float | None = None,
                    max_angular: float | None = None) -> WaypointParams:
    """공용 waypoint 제어 값(게인)은 그대로, 도착 조건과 (주면) 출력 상한만 approach 값으로."""
    caps = {}
    if max_linear is not None:
        caps['max_linear'] = float(max_linear)
    if max_angular is not None:
        caps['max_angular'] = float(max_angular)
    return dataclasses.replace(
        base, position_tolerance=float(position_tolerance),
        heading_tolerance_deg=float(heading_tolerance_deg),
        settle_yaw_rate=float(settle_yaw_rate), **caps)


class MissionApproach(BehaviorBase):
    STATE_NAME = 'approach'
    CMD_TOPIC = '/cmd/approach'

    def __init__(self):
        super().__init__()
        self.declare_parameter('approach.motion_enabled', False)
        self.declare_parameter('approach.thrust_ratio', APPROACH_THRUST_RATIO)
        self.declare_parameter('approach.max_linear', APPROACH_MAX_LINEAR)
        self.declare_parameter('approach.max_angular', APPROACH_MAX_ANGULAR)
        self.declare_parameter('approach.position_tolerance',
                               APPROACH_POSITION_TOLERANCE)
        self.declare_parameter('approach.heading_tolerance_deg',
                               APPROACH_HEADING_TOLERANCE_DEG)
        self.declare_parameter('approach.settle_yaw_rate', APPROACH_SETTLE_YAW_RATE)
        self.declare_parameter('approach.slow_radius', APPROACH_SLOW_RADIUS)
        self.motion_enabled = bool(
            self.get_parameter('approach.motion_enabled').value)
        self.thrust_ratio = float(
            self.get_parameter('approach.thrust_ratio').value)
        if not math.isfinite(self.thrust_ratio) or not 0 < self.thrust_ratio <= 1:
            raise ValueError('approach.thrust_ratio must be in (0, 1]')
        self.slow_radius = float(self.get_parameter('approach.slow_radius').value)
        if not math.isfinite(self.slow_radius) or self.slow_radius <= 0:
            raise ValueError('approach.slow_radius must be > 0')
        params = approach_params(
            self._waypoint_controller.params,
            self.get_parameter('approach.position_tolerance').value,
            self.get_parameter('approach.heading_tolerance_deg').value,
            self.get_parameter('approach.settle_yaw_rate').value,
            max_linear=self.get_parameter('approach.max_linear').value,
            max_angular=self.get_parameter('approach.max_angular').value)
        self._waypoint_controller = WaypointController(params)
        # _tick 이 자르는 전체 스케일 — 공용 behavior 스케일(전진 0.101·회전 0.31) 대신 approach 상한
        self._full_scale = (params.max_linear, params.max_angular)
        yaw_rate = (f'{params.settle_yaw_rate:.2f} rad/s 이하' if params.settle_yaw_rate > 0
                    else '조건 없음')
        self.get_logger().info(
            f'approach 도착 조건: {params.position_tolerance:.2f} m 안, 선수각 '
            f'±{params.heading_tolerance_deg:.0f}° 안, 회전 속도 {yaw_rate}, '
            f'{params.settle_time:.1f} s 유지 | 출력 상한 전진 '
            f'{params.max_linear * self.thrust_ratio:.3f}·회전 '
            f'{params.max_angular * self.thrust_ratio:.3f}, {self.slow_radius:.1f} m 앞부터 감속')
        self.entry_goal = None
        self.preview_pub = self.create_publisher(
            Twist, '/mission/approach_cmd_preview', 10)
        # Volatile goal, repeated by mission_manager while approach is active.
        # This prevents a latched goal from an earlier mission being reused.
        self.create_subscription(
            PoseStamped, '/mission/entry_goal', self._on_entry_goal, 10)
        if not self.motion_enabled:
            self.get_logger().warning(
                'approach.motion_enabled=false — 시작 자세 접근 명령은 미리보기만 발행')

    def on_activate(self):
        self.entry_goal = None

    def on_deactivate(self):
        self.entry_goal = None

    def _on_entry_goal(self, msg: PoseStamped):
        if self.active:
            self.entry_goal = msg

    def compute_cmd(self):
        cmd = Twist()
        goal = self.entry_goal
        if goal is None or goal.header.frame_id != 'odom':
            self.preview_pub.publish(cmd)
            return cmd
        point = goal.pose.position
        q = goal.pose.orientation
        if not all(math.isfinite(v) for v in
                   (point.x, point.y, q.x, q.y, q.z, q.w)):
            self.preview_pub.publish(cmd)
            return cmd
        angle_deg = math.degrees(yaw_from_quaternion(q))
        cmd, reached = self.waypoint(
            point.x, point.y, angle_deg, thrust_ratio=self.thrust_ratio,
            slow_radius=self.slow_radius)
        self._command_full_scale = self._full_scale
        self.preview_pub.publish(cmd)
        if not self.motion_enabled:
            return Twist()
        if reached:
            self.report_complete()
            return Twist()
        return cmd


def main(args=None):
    rclpy.init(args=args)
    node = MissionApproach()
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
