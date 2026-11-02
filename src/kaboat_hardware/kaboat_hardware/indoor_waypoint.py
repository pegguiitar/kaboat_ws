"""Sensor-gated indoor tank waypoint trial using the shared controller.

The goal is fixed at startup.  Preview mode is the default and never
publishes /cmd_vel.  Motion mode requires one fresh tank-frame /odom publisher
and exclusive ownership of /cmd_vel; no motor driver is launched here.
"""

import math
import time

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Bool, String

from kaboat_behaviors.waypoint_controller import (
    WaypointController, WaypointParams, waypoint)


def _yaw(q) -> float:
    return math.atan2(2.0 * (q.w*q.z + q.x*q.y),
                      1.0 - 2.0 * (q.y*q.y + q.z*q.z))


class IndoorWaypoint(Node):
    def __init__(self):
        super().__init__('indoor_waypoint')
        defaults = {
            'target_x': 5.0, 'target_y': 2.5, 'target_angle_deg': 0.0,
            'thrust_ratio': 0.2,
            'position_tolerance': 0.9,
            'heading_tolerance_deg': 10.0,
            'motion_enabled': False,
            'tank_size_x': 10.0, 'tank_size_y': 5.0,
            'tank_margin': 0.25,
            'odom_timeout_sec': 0.5,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        tuning = WaypointParams()
        for name in vars(tuning):
            if not self.has_parameter(name):
                self.declare_parameter(name, getattr(tuning, name))
        p = lambda name: self.get_parameter(name).value

        self.target = waypoint(p('target_x'), p('target_y'),
                               p('target_angle_deg'), p('thrust_ratio'))
        self.motion_enabled = bool(p('motion_enabled'))
        self.size_x = float(p('tank_size_x'))
        self.size_y = float(p('tank_size_y'))
        self.margin = float(p('tank_margin'))
        self.odom_timeout = float(p('odom_timeout_sec'))
        if (not all(math.isfinite(v) for v in
                    (self.size_x, self.size_y, self.margin,
                     self.odom_timeout))
                or not 0 <= self.margin < min(self.size_x, self.size_y) / 2
                or self.odom_timeout <= 0
                or not self.margin <= self.target.x <= self.size_x - self.margin
                or not self.margin <= self.target.y <= self.size_y - self.margin):
            raise ValueError('waypoint must lie inside the measured tank bounds')

        params = WaypointParams(**{
            name: float(p(name)) for name in vars(tuning)})
        self.controller = WaypointController(params)
        self.controller.set_goal(self.target)
        self.odom = None
        self.last_odom_time = None
        self.estop = False
        self.completed = False
        self.last_status = None

        latched = QoSProfile(depth=1,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.goal_pub = self.create_publisher(PoseStamped, '/waypoint/goal',
                                              latched)
        self.status_pub = self.create_publisher(String, '/waypoint/status',
                                                latched)
        self.preview_pub = self.create_publisher(
            Twist, '/waypoint/cmd_preview', 10)
        self.cmd_pub = (self.create_publisher(Twist, '/cmd_vel', 10)
                        if self.motion_enabled else None)
        self.create_subscription(Odometry, '/odom', self._on_odom, 10)
        self.create_subscription(Bool, '/emergency_stop', self._on_estop, 10)
        self.create_timer(0.05, self._tick)
        self._publish_goal()
        self._status('WAIT_ODOM')
        self.get_logger().info(
            f'waypoint({self.target.x:.2f}, {self.target.y:.2f}, '
            f'{self.target.angle_deg:.1f}, {self.target.thrust_ratio:.2f}) '
            'in odom; '
            f'motion_enabled={self.motion_enabled}')

    def _publish_goal(self):
        msg = PoseStamped()
        msg.header.frame_id = 'odom'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = self.target.x
        msg.pose.position.y = self.target.y
        msg.pose.orientation.z = math.sin(self.target.yaw / 2.0)
        msg.pose.orientation.w = math.cos(self.target.yaw / 2.0)
        self.goal_pub.publish(msg)

    def _status(self, value):
        if value != self.last_status:
            self.last_status = value
            self.status_pub.publish(String(data=value))
            self.get_logger().info(f'waypoint status: {value}')

    def _on_odom(self, msg):
        self.odom = msg
        self.last_odom_time = time.monotonic()

    def _on_estop(self, msg):
        self.estop = msg.data

    def _send(self, cmd):
        self.preview_pub.publish(cmd)
        if self.cmd_pub is not None:
            self.cmd_pub.publish(cmd)

    def _halt(self, reason):
        self.controller.reset_progress()
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
                and 0 <= pose.position.x <= self.size_x
                and 0 <= pose.position.y <= self.size_y)

    def _tick(self):
        if self.completed:
            self._send(Twist())
            return
        if self.estop:
            self._halt('EMERGENCY_STOP')
            return
        if self.motion_enabled and self.count_publishers('/cmd_vel') != 1:
            self._halt('CMD_VEL_PUBLISHER_CONFLICT')
            return
        if self.count_publishers('/odom') != 1:
            self._halt('ODOM_PUBLISHER_COUNT_NOT_ONE')
            return
        if (self.last_odom_time is None
                or time.monotonic() - self.last_odom_time > self.odom_timeout):
            self._halt('ODOM_TIMEOUT')
            return
        if not self._valid_odom():
            self._halt('INVALID_ODOM')
            return

        pose = self.odom.pose.pose
        step = self.controller.step(
            pose.position.x, pose.position.y, _yaw(pose.orientation),
            self.odom.twist.twist.angular.z, time.monotonic())
        cmd = Twist()
        cmd.linear.x, cmd.angular.z = step.linear, step.angular
        if step.reached:
            self.completed = True
            cmd = Twist()
        self._send(cmd)
        self._status(step.phase if self.motion_enabled
                     else f'PREVIEW_{step.phase}')
        cur_yaw_deg = math.degrees(_yaw(pose.orientation))
        heading_err_deg = math.degrees(step.heading_error)
        self.get_logger().info(
            f'[{step.phase}] 현재: ({pose.position.x:.2f}, {pose.position.y:.2f})m '
            f'| 목표거리: {step.distance:.2f}m (허용: {self.controller.params.position_tolerance:.2f}m) '
            f'| 선수각: {cur_yaw_deg:.1f}° (오차: {heading_err_deg:.1f}°) '
            f'| 출력: [전진 {cmd.linear.x*100:.0f}%, 회전 {cmd.angular.z*100:.0f}%]',
            throttle_duration_sec=1.0)

    def stop(self):
        if self.cmd_pub is not None:
            self.cmd_pub.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = IndoorWaypoint()
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
