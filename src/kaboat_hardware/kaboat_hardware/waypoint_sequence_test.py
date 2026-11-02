"""Follow five tank-frame waypoints using the shared waypoint controller.

Preview is the default. Physical output requires motion_enabled and an
explicit start service call. Sensors and /odom are started separately.
"""

import math
import time

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger

from kaboat_behaviors.waypoint_controller import (
    WaypointController, WaypointParams, waypoint)


DEFAULT_X = [8.5, 7.0, 5.0, 3.0, 1.5]
DEFAULT_Y = [2.5, 3.5, 2.5, 1.5, 2.5]
DEFAULT_ANGLES = [0.0, 0.0, 0.0, 0.0, 0.0]
DEFAULT_USE_HEADING = [False, False, False, False, False]


def build_waypoints(xs, ys, angles, use_heading, thrust_ratio,
                    size_x, size_y, margin):
    """Validate exactly five odom-frame tank targets and create Waypoints."""
    if not all(len(values) == 5 for values in (xs, ys, angles, use_heading)):
        raise ValueError('exactly five X, Y, angle and heading-enabled values are required')
    limits = (float(size_x), float(size_y), float(margin))
    if (not all(math.isfinite(v) for v in limits)
            or limits[0] <= 0 or limits[1] <= 0
            or not 0 <= limits[2] < min(limits[0], limits[1]) / 2):
        raise ValueError('invalid tank bounds or margin')
    size_x, size_y, margin = limits

    targets = []
    for x, y, angle, enabled in zip(xs, ys, angles, use_heading):
        if not isinstance(enabled, bool) or not math.isfinite(float(angle)):
            raise ValueError('heading flags must be booleans and angles finite')
        target = waypoint(x, y, angle if enabled else None, thrust_ratio)
        if not (margin <= target.x <= size_x - margin
                and margin <= target.y <= size_y - margin):
            raise ValueError('waypoint lies outside the measured tank bounds')
        targets.append(target)
    return tuple(targets)


class WaypointSequenceTest(Node):
    def __init__(self):
        super().__init__('waypoint_sequence_test')
        defaults = {
            'waypoints_x': DEFAULT_X,
            'waypoints_y': DEFAULT_Y,
            'waypoints_angle_deg': DEFAULT_ANGLES,
            'waypoints_use_heading': DEFAULT_USE_HEADING,
            'thrust_ratio': 0.2,
            'position_tolerance': 0.9,
            'heading_tolerance_deg': 10.0,
            'motion_enabled': False,
            'max_linear': 12.0 / 118.6,
            'max_angular': 3.0 / 118.6,
            'yaw_kp': 18.0 / 118.6,
            'yaw_kd': 9.0 / 118.6,
            'slow_radius': 1.5,
            'wait_for_start': True,
            'tank_size_x': 10.0,
            'tank_size_y': 5.0,
            'tank_margin': 0.25,
            'odom_timeout_sec': 0.5,
            'hold_duration_sec': 3.0,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        p = lambda name: self.get_parameter(name).value

        self.size_x = float(p('tank_size_x'))
        self.size_y = float(p('tank_size_y'))
        self.margin = float(p('tank_margin'))
        self.odom_timeout = float(p('odom_timeout_sec'))
        self.hold_duration = float(p('hold_duration_sec'))
        if not math.isfinite(self.odom_timeout) or self.odom_timeout <= 0:
            raise ValueError('odom_timeout_sec must be positive and finite')
        if not math.isfinite(self.hold_duration) or self.hold_duration < 0:
            raise ValueError('hold_duration_sec must be nonnegative and finite')
        self.targets = build_waypoints(
            p('waypoints_x'), p('waypoints_y'), p('waypoints_angle_deg'),
            p('waypoints_use_heading'), p('thrust_ratio'),
            self.size_x, self.size_y, self.margin)
        self.motion_enabled = bool(p('motion_enabled'))
        tuning = WaypointParams(
            max_linear=float(p('max_linear')),
            max_angular=float(p('max_angular')),
            yaw_kp=float(p('yaw_kp')),
            yaw_kd=float(p('yaw_kd')),
            slow_radius=float(p('slow_radius')),
            position_tolerance=float(p('position_tolerance')),
            heading_tolerance_deg=float(p('heading_tolerance_deg')),
            settle_time=0.5,
        )
        self.controller = WaypointController(tuning)
        self.index = 0
        self.controller.set_goal(self.targets[self.index])
        self.odom = None
        self.last_odom_time = None
        self.estop = False
        self.wait_for_start = bool(p('wait_for_start'))
        self.started = not self.wait_for_start
        self.completed = False
        self._hold_since = None
        self._now = time.monotonic
        self.last_status = None

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.path_pub = self.create_publisher(Path, '/waypoint_sequence/path', latched)
        self.goal_pub = self.create_publisher(
            PoseStamped, '/waypoint_sequence/goal', latched)
        self.status_pub = self.create_publisher(
            String, '/waypoint_sequence/status', latched)
        self.preview_pub = self.create_publisher(
            Twist, '/waypoint_sequence/cmd_preview', 10)
        self.cmd_pub = (self.create_publisher(Twist, '/cmd_vel', 10)
                        if self.motion_enabled else None)
        self.create_subscription(Odometry, '/odom', self._on_odom, 10)
        self.create_subscription(Bool, '/emergency_stop', self._on_estop, 10)
        self.create_service(Trigger, '/waypoint_sequence/start', self._on_start)
        self.create_service(Trigger, '/waypoint_sequence/pause', self._on_pause)
        self.create_timer(0.1, self._tick)
        self.create_timer(0.5, self._publish_path)
        self._publish_path()
        self._publish_goal()
        if self.started:
            self._status('WAYPOINT_1/5_APPROACH')
        else:
            self._status('WAIT_START')
        self.get_logger().info(
            f'five-waypoint test ready; ratio={self.targets[0].thrust_ratio:.2f}; '
            f'hold={self.hold_duration:.1f}s; motion_enabled={self.motion_enabled}')

    def _pose(self, target):
        pose = PoseStamped()
        pose.header.frame_id = 'odom'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = target.x
        pose.pose.position.y = target.y
        yaw = target.yaw if target.yaw is not None else 0.0
        pose.pose.orientation.z = math.sin(yaw / 2)
        pose.pose.orientation.w = math.cos(yaw / 2)
        return pose

    def _publish_path(self):
        msg = Path()
        msg.header.frame_id = 'odom'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.poses = [self._pose(target) for target in self.targets]
        self.path_pub.publish(msg)

    def _publish_goal(self):
        self.goal_pub.publish(self._pose(self.targets[self.index]))

    def _status(self, value):
        if value != self.last_status:
            self.last_status = value
            self.status_pub.publish(String(data=value))
            self.get_logger().info(f'waypoint sequence status: {value}')

    def _on_odom(self, msg):
        self.odom = msg
        self.last_odom_time = self._now()

    def _on_estop(self, msg):
        self.estop = msg.data

    def _on_start(self, request, response):
        response.success = not self.estop
        if response.success:
            self.started = True
            response.message = (f'waypoint {self.index + 1}/5 hold resumed'
                                if self.completed else
                                f'waypoint {self.index + 1}/5 started')
        else:
            response.message = 'clear emergency stop first'
        return response

    def _on_pause(self, request, response):
        self.started = False
        self.controller.reset_progress()
        self._hold_since = None
        self._send(Twist())
        self._status('WAIT_START')
        response.success = True
        response.message = f'paused at waypoint {self.index + 1}/5'
        return response

    def _send(self, cmd):
        self.preview_pub.publish(cmd)
        if self.cmd_pub is not None:
            self.cmd_pub.publish(cmd)

    def _halt(self, reason):
        self.controller.reset_progress()
        self._hold_since = None
        self._send(Twist())
        self._status(reason)

    def _valid_odom(self):
        if self.odom is None or self.odom.header.frame_id != 'odom':
            return False
        pose = self.odom.pose.pose
        q = pose.orientation
        values = (pose.position.x, pose.position.y, q.x, q.y, q.z, q.w,
                  self.odom.twist.twist.angular.z)
        norm = q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w
        return (all(math.isfinite(v) for v in values)
                and abs(norm - 1.0) <= 0.1
                and 0.0 <= pose.position.x <= self.size_x
                and 0.0 <= pose.position.y <= self.size_y)

    def _tick(self):
        now = self._now()
        if self.estop:
            self._halt('EMERGENCY_STOP')
            return
        if not self.started:
            self._halt('WAIT_START')
            return
        if self.motion_enabled and self.count_publishers('/cmd_vel') != 1:
            self._halt('CMD_VEL_PUBLISHER_CONFLICT')
            return
        if self.count_publishers('/odom') != 1:
            self._halt('ODOM_PUBLISHER_COUNT_NOT_ONE')
            return
        if (self.last_odom_time is None
                or now - self.last_odom_time > self.odom_timeout):
            self._halt('ODOM_TIMEOUT')
            return
        if not self._valid_odom():
            self._halt('INVALID_ODOM')
            return

        pose = self.odom.pose.pose
        q = pose.orientation
        yaw = math.atan2(2.0 * (q.w*q.z + q.x*q.y),
                         1.0 - 2.0 * (q.y*q.y + q.z*q.z))
        step = self.controller.step(
            pose.position.x, pose.position.y, yaw,
            self.odom.twist.twist.angular.z, now)
        cmd = Twist()
        if step.reached:
            if self._hold_since is None:
                self._hold_since = now
            held_long_enough = now - self._hold_since >= self.hold_duration
            if self.index == len(self.targets) - 1:
                if held_long_enough:
                    self.completed = True
                self._status('COMPLETE_HOLD' if self.completed else
                             'WAYPOINT_5/5_HOLD')
            elif held_long_enough:
                previous = self.index + 1
                self.index += 1
                self._hold_since = None
                self.controller.set_goal(self.targets[self.index])
                self._publish_goal()
                self._status(f'WAYPOINT_{previous}/5_HELD')
            else:
                self._status(f'WAYPOINT_{self.index + 1}/5_HOLD')
        else:
            self._hold_since = None
            cmd.linear.x, cmd.angular.z = step.linear, step.angular
            prefix = '' if self.motion_enabled else 'PREVIEW_'
            phase = ('REACQUIRE' if self.completed else step.phase)
            self._status(f'{prefix}WAYPOINT_{self.index + 1}/5_{phase}')
        self._send(cmd)

    def stop(self):
        if self.cmd_pub is not None:
            self.cmd_pub.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = WaypointSequenceTest()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
