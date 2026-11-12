"""Move to a configured mission entry pose before its behavior starts.

도착 조건 (2026-11-12 야외): 시작 지점 1 m 안 + 선수각 ±15° 안을 settle_time(0.5 s)
유지. 회전 속도 조건은 없다(approach.settle_yaw_rate 0). 공용 waypoint() 기본값
(0.35 m / 8° / 0.1 rad/s)은 바람·물살이 있는 야외 시작 자세 맞추기에 너무 엄격해
approach 에서 gate 등 미션으로 넘어가지 못했다. 다른 behavior 의 waypoint() 는 그대로.
"""

import dataclasses
import math

import rclpy
from geometry_msgs.msg import PoseStamped, Twist

from .behavior_base import BehaviorBase, yaw_from_quaternion
from .waypoint_controller import WaypointController, WaypointParams

APPROACH_POSITION_TOLERANCE = 1.0      # [m]
APPROACH_HEADING_TOLERANCE_DEG = 15.0  # [deg] 좌우
APPROACH_SETTLE_YAW_RATE = 0.0         # [rad/s] 0 = 회전 속도 조건 없음


def approach_params(base: WaypointParams, position_tolerance: float,
                    heading_tolerance_deg: float,
                    settle_yaw_rate: float) -> WaypointParams:
    """공용 waypoint 제어 값(게인·상한)은 그대로, 도착 조건만 approach 값으로."""
    return dataclasses.replace(
        base, position_tolerance=float(position_tolerance),
        heading_tolerance_deg=float(heading_tolerance_deg),
        settle_yaw_rate=float(settle_yaw_rate))


class MissionApproach(BehaviorBase):
    STATE_NAME = 'approach'
    CMD_TOPIC = '/cmd/approach'

    def __init__(self):
        super().__init__()
        self.declare_parameter('approach.motion_enabled', False)
        self.declare_parameter('approach.thrust_ratio', 0.2)
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
            self.get_parameter('approach.settle_yaw_rate').value)
        self._waypoint_controller = WaypointController(params)
        yaw_rate = (f'{params.settle_yaw_rate:.2f} rad/s 이하' if params.settle_yaw_rate > 0
                    else '조건 없음')
        self.get_logger().info(
            f'approach 도착 조건: {params.position_tolerance:.2f} m 안, 선수각 '
            f'±{params.heading_tolerance_deg:.0f}° 안, 회전 속도 {yaw_rate}, '
            f'{params.settle_time:.1f} s 유지')
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
