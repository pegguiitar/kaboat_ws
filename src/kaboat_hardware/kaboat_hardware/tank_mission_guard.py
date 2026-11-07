"""Guard a single tank mission's cmd_mux output before publishing /cmd_vel.

The production behavior and cmd_mux run in preview mode without a motor-output
publisher. Drive mode additionally requires an explicit start service and fresh
odom, occupancy grid and raw command. Any lost input disarms the test.
The onboard /scan is deliberately not checked here: the operator watches it
in RViz over the Zenoh link during tank tests.
"""

import math
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger


class TankMissionGuard(Node):
    def __init__(self):
        super().__init__('tank_mission_guard')
        defaults = {
            'drive_enabled': False,
            'allow_reverse': False,
            'mission': 'gate',
            'max_linear_cmd': 0.05,
            'max_angular_cmd': 0.02,
            'odom_timeout_sec': 0.5,
            'grid_timeout_sec': 0.5,
            'cmd_timeout_sec': 0.3,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        p = lambda name: self.get_parameter(name).value
        self.drive_enabled = bool(p('drive_enabled'))
        self.allow_reverse = bool(p('allow_reverse'))
        self.mission = str(p('mission'))
        if self.mission not in ('gate', 'avoid'):
            raise ValueError('mission must be gate or avoid')
        self.max_linear = float(p('max_linear_cmd'))
        self.max_angular = float(p('max_angular_cmd'))
        self.timeouts = {
            'odom': float(p('odom_timeout_sec')),
            'grid': float(p('grid_timeout_sec')),
            'cmd': float(p('cmd_timeout_sec')),
        }
        if (not all(math.isfinite(v) and v > 0 for v in
                    (self.max_linear, self.max_angular, *self.timeouts.values()))
                or self.max_linear > 1 or self.max_angular > 1):
            raise ValueError('command limits and timeouts must be positive and finite')

        self.odom = None
        self.grid = None
        self.cmd = None
        self.received = {}
        self.state = ''
        self.state_changed_at = time.monotonic()
        self.estopped = False
        self.armed = False
        self.last_status = None

        self.cmd_pub = (self.create_publisher(Twist, '/cmd_vel', 10)
                        if self.drive_enabled else None)
        self.status_pub = self.create_publisher(
            String, '/tank_mission/status', 10)
        self.create_subscription(Twist, '/tank_mission/cmd_raw',
                                 self._on_cmd, 10)
        self.create_subscription(Odometry, '/odom', self._on_odom, 10)
        self.create_subscription(OccupancyGrid, '/occupancy_grid',
                                 self._on_grid, 1)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, '/mission/state', self._on_state,
                                 latched)
        self.create_subscription(Bool, '/emergency_stop', self._on_estop, 10)
        self.create_service(Trigger, '/tank_mission/start', self._on_start)
        self.create_service(Trigger, '/tank_mission/stop', self._on_stop)
        self.create_timer(0.05, self._tick)
        self._status('PREVIEW' if not self.drive_enabled else 'WAIT_START')

    def _record(self, name, msg):
        setattr(self, name, msg)
        self.received[name] = time.monotonic()

    def _on_cmd(self, msg):
        self._record('cmd', msg)

    def _on_odom(self, msg):
        self._record('odom', msg)

    def _on_grid(self, msg):
        self._record('grid', msg)

    def _on_state(self, msg):
        if msg.data != self.state:
            # The mux output has no source tag. A command from the previous
            # state must not be reused while the new behavior starts up.
            self.cmd = None
            self.received.pop('cmd', None)
            self.state_changed_at = time.monotonic()
            if self.armed and self.cmd_pub is not None:
                self.cmd_pub.publish(Twist())
        self.state = msg.data
        if self.armed and self.state not in ('approach', self.mission):
            self._disarm('mission ended')

    def _on_estop(self, msg):
        self.estopped = bool(msg.data)
        if self.estopped:
            self._disarm('emergency_stop')

    def _input_error(self):
        if self.estopped:
            return 'emergency_stop active'
        if self.state not in ('approach', self.mission):
            return f'mission state {self.state or "unset"}'
        now = time.monotonic()
        for name, timeout in self.timeouts.items():
            if name not in self.received:
                return f'waiting for {name}'
            if now - self.received[name] > timeout:
                return f'{name} stale'
        if self.odom.header.frame_id != 'odom' or self.grid.header.frame_id != 'odom':
            return 'odom/grid frame mismatch'
        pose = self.odom.pose.pose
        q = pose.orientation
        values = (pose.position.x, pose.position.y, q.x, q.y, q.z, q.w,
                  self.odom.twist.twist.angular.z, self.cmd.linear.x,
                  self.cmd.angular.z)
        if not all(math.isfinite(v) for v in values):
            return 'nonfinite odom/command'
        if self.cmd.linear.x < 0.0 and not self.allow_reverse:
            return 'reverse command blocked'
        norm = q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w
        if abs(norm - 1.0) > 0.1:
            return 'invalid odom quaternion'
        if (self.grid.info.resolution <= 0
                or len(self.grid.data) != self.grid.info.width * self.grid.info.height
                or not any(v >= 0 for v in self.grid.data)):
            return 'occupancy grid empty/unobserved'
        return None

    def _on_start(self, _request, response):
        if not self.drive_enabled:
            response.success, response.message = False, 'preview mode has no motor output'
            return response
        reason = self._input_error()
        response.success = reason is None
        response.message = reason or 'tank mission armed'
        if response.success:
            self.armed = True
            self._status('DRIVE')
        return response

    def _on_stop(self, _request, response):
        self._disarm('stop service')
        response.success, response.message = True, 'tank mission stopped'
        return response

    def _disarm(self, reason):
        self.armed = False
        if self.cmd_pub is not None:
            self.cmd_pub.publish(Twist())
        self._status(f'STOP: {reason}')

    def _status(self, value):
        if value != self.last_status:
            self.last_status = value
            self.status_pub.publish(String(data=value))
            self.get_logger().info(value)

    def _tick(self):
        if self.cmd_pub is None:
            return
        if not self.armed:
            self.cmd_pub.publish(Twist())
            return
        reason = self._input_error()
        if (reason == 'waiting for cmd'
                and time.monotonic() - self.state_changed_at <= self.timeouts['cmd']):
            self.cmd_pub.publish(Twist())
            return
        if reason is not None:
            self._disarm(reason)
            return
        cmd = Twist()
        cmd.linear.x = max(-self.max_linear,
                           min(self.max_linear, self.cmd.linear.x))
        cmd.angular.z = max(-self.max_angular,
                            min(self.max_angular, self.cmd.angular.z))
        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = TankMissionGuard()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.cmd_pub is not None:
            node.cmd_pub.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
