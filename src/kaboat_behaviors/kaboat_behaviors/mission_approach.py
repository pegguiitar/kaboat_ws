"""Move to a configured mission entry pose before its behavior starts.

도착 조건 (2026-11-12 야외 1 m·±15° → 2026-11-13 2 m·±30°): 시작 지점 2 m 안 + 선수각 ±30° 안을
settle_time(0.5 s) 유지. 회전 속도 조건은 없다(approach.settle_yaw_rate 0). 공용 waypoint() 기본값
(0.35 m / 8° / 0.1 rad/s)은 바람·물살이 있는 야외 시작 자세 맞추기에 너무 엄격해
approach 에서 gate 등 미션으로 넘어가지 못했다. 다른 behavior 의 waypoint() 는 그대로.

출력 상한 (2026-11-14): goto(outdoor_waypoint)와 같은 값 — 상한 1.0 × thrust_ratio 0.4 = 전진 0.4·회전 0.4.
예전엔 공용 behavior 스케일(전진 0.101·회전 0.31) × 0.2 = 전진 0.0202·회전 0.062 라 야외에서 물살에
밀려 시작 자세에 못 갔다(2026-11-13 dock_test1 bag: search_entry 에서 2.6 → 9.3 m 멀어짐, 시뮬에서
접근 90 s 초과로 search 를 15번 중 9번 건너뜀).
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
# 출력 상한 — goto(outdoor_waypoint.launch.py 기본 max_linear·max_angular 1.0, thrust_ratio 0.4)와 같게
APPROACH_MAX_LINEAR = 1.0
APPROACH_MAX_ANGULAR = 1.0
APPROACH_THRUST_RATIO = 0.4             # (0.2 → 0.4, 2026-11-14) → 전진·회전 상한 0.4


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
        self.motion_enabled = bool(
            self.get_parameter('approach.motion_enabled').value)
        self.thrust_ratio = float(
            self.get_parameter('approach.thrust_ratio').value)
        if not math.isfinite(self.thrust_ratio) or not 0 < self.thrust_ratio <= 1:
            raise ValueError('approach.thrust_ratio must be in (0, 1]')
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
            f'{params.max_angular * self.thrust_ratio:.3f} (goto 와 같음)')
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
            point.x, point.y, angle_deg, thrust_ratio=self.thrust_ratio)
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
