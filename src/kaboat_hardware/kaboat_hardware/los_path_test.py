"""Preview or drive the shared B-spline/circle LOS controller in the tank.

Sensors and odometry are started separately. The default publishes only a
preview command; physical motion needs an explicit start and motion_enabled.
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

from kaboat_behaviors.behavior_base import DEFAULT_MAX_ANGULAR, DEFAULT_MAX_LINEAR
from kaboat_behaviors.los_controller import LosFollower, bspline_path, circle_path
from kaboat_hardware.test_coordinates import BSPLINE_TRACK, CIRCLE_DRIVE


def make_path(kind, control_points, center_x, center_y, radius, direction,
              target_laps, spacing):
    if kind == 'bspline':
        return bspline_path(control_points, spacing=spacing)
    if kind == 'circle':
        return circle_path(center_x, center_y, radius, direction=direction,
                           target_laps=None if target_laps == 0 else target_laps,
                           spacing=spacing)
    raise ValueError("path_kind must be 'bspline' or 'circle'")


class LosPathTest(Node):
    def __init__(self):
        super().__init__('los_path_test')
        defaults = {
            'path_kind': 'bspline',
            'control_points_x': [float(x) for x, _ in BSPLINE_TRACK['control_points']],
            'control_points_y': [float(y) for _, y in BSPLINE_TRACK['control_points']],
            'center_x': float(CIRCLE_DRIVE['center_x']),
            'center_y': float(CIRCLE_DRIVE['center_y']),
            'radius': float(CIRCLE_DRIVE['radius']),
            'direction': str(CIRCLE_DRIVE['direction']),
            'target_laps': float(CIRCLE_DRIVE['target_laps']),
            'sample_spacing': 0.05,
            'thrust_ratio': 0.3,
            'motion_enabled': False,
            'wait_for_start': False,
            'max_linear': DEFAULT_MAX_LINEAR,
            'max_angular': DEFAULT_MAX_ANGULAR,
            'yaw_kp': 18.0 / 118.6,
            'yaw_kd': 9.0 / 118.6,
            'goal_tolerance': 0.4,
            'tank_size_x': 10.0,
            'tank_size_y': 5.0,
            'tank_margin': 0.25,
            'odom_timeout_sec': 0.5,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        p = lambda name: self.get_parameter(name).value

        xs, ys = list(p('control_points_x')), list(p('control_points_y'))
        if len(xs) != len(ys):
            raise ValueError('control_points_x and control_points_y must have equal lengths')
        self.path = make_path(
            str(p('path_kind')), list(zip(xs, ys)), float(p('center_x')),
            float(p('center_y')), float(p('radius')), str(p('direction')),
            float(p('target_laps')), float(p('sample_spacing')))
        self.ratio = float(p('thrust_ratio'))
        self.motion_enabled = bool(p('motion_enabled'))
        self.size_x = float(p('tank_size_x'))
        self.size_y = float(p('tank_size_y'))
        self.margin = float(p('tank_margin'))
        self.odom_timeout = float(p('odom_timeout_sec'))
        if (not all(math.isfinite(v) for v in (
                self.ratio, self.size_x, self.size_y, self.margin, self.odom_timeout))
                or not 0.0 <= self.ratio <= 1.0
                or self.odom_timeout <= 0.0
                or self.size_x <= 0.0 or self.size_y <= 0.0
                or not 0.0 <= self.margin < min(self.size_x, self.size_y) / 2.0
                or not all(self.margin <= x <= self.size_x - self.margin and
                           self.margin <= y <= self.size_y - self.margin
                           for x, y in self.path.samples)):
            raise ValueError('LOS path or settings are outside the measured tank bounds')

        self.follower = LosFollower(
            max_linear=float(p('max_linear')),
            max_angular=float(p('max_angular')),
            yaw_kp=float(p('yaw_kp')),
            yaw_kd=float(p('yaw_kd')),
            goal_tolerance=float(p('goal_tolerance')),
        )
        self.odom = None
        self.last_odom_time = None
        self.wait_for_start = bool(p('wait_for_start'))
        self.started = not self.wait_for_start
        self.estop = False
        self.completed = False
        self.last_status = None

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.path_pub = self.create_publisher(Path, '/los_test/path', latched)
        self.status_pub = self.create_publisher(String, '/los_test/status', latched)
        self.preview_pub = self.create_publisher(Twist, '/los_test/cmd_preview', 10)
        self.cmd_pub = (self.create_publisher(Twist, '/cmd_vel', 10)
                        if self.motion_enabled else None)
        self.create_subscription(Odometry, '/odom', self._on_odom, 10)
        self.create_subscription(Bool, '/emergency_stop', self._on_estop, 10)
        self.create_subscription(Bool, '/start_mission', self._on_start, 10)
        self.create_service(Trigger, '/los_test/start', self._on_start_service)
        self.create_timer(0.1, self._tick)
        self.create_timer(0.5, self._publish_path)
        self._publish_path()
        if self.started:
            self._status(('' if self.motion_enabled else 'PREVIEW_') + 'WAIT_ODOM')
        else:
            self._status('WAIT_START')
        self.get_logger().info(
            f'LOS {p("path_kind")}: {len(self.path.samples)} samples; '
            f'thrust_ratio={self.ratio:.2f}; motion_enabled={self.motion_enabled}')

    def _publish_path(self):
        msg = Path()
        msg.header.frame_id = 'odom'
        msg.header.stamp = self.get_clock().now().to_msg()
        points = self.path.samples
        if self.path.closed:
            points = list(points) + [points[0]]
        for x, y in points:
            pose = PoseStamped()
            pose.header = msg.header
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            pose.pose.orientation.w = 1.0
            msg.poses.append(pose)
        self.path_pub.publish(msg)

    def _status(self, value):
        if value != self.last_status:
            self.last_status = value
            self.status_pub.publish(String(data=value))
            self.get_logger().info(f'LOS test status: {value}')

    def _on_odom(self, msg):
        self.odom = msg
        self.last_odom_time = time.monotonic()

    def _on_estop(self, msg):
        self.estop = msg.data

    def _on_start(self, msg):
        self.started = msg.data

    def _on_start_service(self, request, response):
        self.started = True
        response.success = True
        response.message = 'LOS test started'
        return response

    def _send(self, cmd):
        self.preview_pub.publish(cmd)
        if self.cmd_pub is not None:
            self.cmd_pub.publish(cmd)

    def _halt(self, reason):
        self.follower.reset()
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
        if self.estop:
            self._halt('EMERGENCY_STOP')
            return
        if not self.started:
            self._halt('WAIT_START')
            return
        if self.completed:
            self._send(Twist())
            return
        if self.motion_enabled and self.count_publishers('/cmd_vel') != 1:
            self._halt('CMD_VEL_PUBLISHER_CONFLICT')
            return
        if self.count_publishers('/odom') != 1:
            self._halt('ODOM_PUBLISHER_COUNT_NOT_ONE')
            return
        if (self.last_odom_time is None or
                time.monotonic() - self.last_odom_time > self.odom_timeout):
            self._halt('ODOM_TIMEOUT')
            return
        if not self._valid_odom():
            self._halt('INVALID_ODOM')
            return

        pose = self.odom.pose.pose
        q = pose.orientation
        yaw = math.atan2(2.0 * (q.w*q.z + q.x*q.y),
                         1.0 - 2.0 * (q.y*q.y + q.z*q.z))
        step = self.follower.step(
            self.path, (pose.position.x, pose.position.y), yaw,
            self.odom.twist.twist.angular.z, self.ratio)
        cmd = Twist()
        if not step.reached:
            cmd.linear.x, cmd.angular.z = step.linear, step.angular
        else:
            self.completed = True
        self._send(cmd)
        self._status(('' if self.motion_enabled else 'PREVIEW_') + step.phase)

    def stop(self):
        if self.cmd_pub is not None:
            self.cmd_pub.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = LosPathTest()
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
