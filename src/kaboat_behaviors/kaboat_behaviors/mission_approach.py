"""Move to a configured mission entry pose before its behavior starts."""

import math

import rclpy
from geometry_msgs.msg import PoseStamped, Twist

from .behavior_base import BehaviorBase, yaw_from_quaternion


class MissionApproach(BehaviorBase):
    STATE_NAME = 'approach'
    CMD_TOPIC = '/cmd/approach'

    def __init__(self):
        super().__init__()
        self.declare_parameter('approach.motion_enabled', False)
        self.declare_parameter('approach.thrust_ratio', 0.2)
        self.motion_enabled = bool(
            self.get_parameter('approach.motion_enabled').value)
        self.thrust_ratio = float(
            self.get_parameter('approach.thrust_ratio').value)
        if not math.isfinite(self.thrust_ratio) or not 0 < self.thrust_ratio <= 1:
            raise ValueError('approach.thrust_ratio must be in (0, 1]')
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
