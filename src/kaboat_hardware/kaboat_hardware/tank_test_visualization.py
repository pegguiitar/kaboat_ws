"""Show tank test reference paths and shore-LiDAR measured boat trajectory.

This runs on the shore laptop. It never publishes propulsion commands and does
not require the Jetson's MarkerArray topics to cross the Zenoh bridge.
"""

import math
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Path
from rclpy.node import Node
from std_srvs.srv import Trigger

from kaboat_behaviors.los_controller import bspline_path, circle_path
from kaboat_hardware.test_coordinates import (
    BSPLINE_TRACK, CIRCLE_DRIVE, STATION_KEEPING, STRAIGHT_LINE)
from kaboat_hardware.trajectory_visualization import (
    BoundedTrajectoryHistory, create_trajectory_path)
from kaboat_hardware.waypoint_sequence_test import DEFAULT_X, DEFAULT_Y


TEST_KINDS = ('none', 'straight', 'bspline', 'circle', 'los',
              'los_circle', 'waypoint', 'station')


def reference_points(kind):
    """Return the default test route in the same tank/odom coordinates."""
    if kind == 'none':
        return []
    if kind == 'straight':
        return [(STRAIGHT_LINE['start_x'], STRAIGHT_LINE['start_y']),
                (STRAIGHT_LINE['goal_x'], STRAIGHT_LINE['goal_y'])]
    if kind in ('bspline', 'los'):
        return list(bspline_path(
            BSPLINE_TRACK['control_points'],
            spacing=BSPLINE_TRACK['sample_spacing']).samples)
    if kind in ('circle', 'los_circle'):
        plan = circle_path(
            CIRCLE_DRIVE['center_x'], CIRCLE_DRIVE['center_y'],
            CIRCLE_DRIVE['radius'], direction=CIRCLE_DRIVE['direction'],
            target_laps=None, spacing=0.05)
        return list(plan.samples) + [plan.samples[0]]
    if kind == 'waypoint':
        return list(zip(DEFAULT_X, DEFAULT_Y))
    if kind == 'station':
        # A Path needs two distinct points to render a target location.
        x, y = STATION_KEEPING['target_x'], STATION_KEEPING['target_y']
        return [(x - 0.1, y), (x + 0.1, y)]
    raise ValueError(f'unknown tank test kind: {kind}')


class TankTestVisualization(Node):
    def __init__(self):
        super().__init__('tank_test_visualization')
        self.declare_parameter('test_kind', 'none')
        self.kind = str(self.get_parameter('test_kind').value)
        if self.kind not in TEST_KINDS:
            raise ValueError(f'test_kind must be one of {TEST_KINDS}')

        self.reference = [(float(x), float(y), 0.05)
                          for x, y in reference_points(self.kind)]
        self.history = BoundedTrajectoryHistory()
        self.last_pose_time = None

        self.actual_pub = self.create_publisher(
            Path, '/tank_test/actual_path', 10)
        self.target_pub = self.create_publisher(
            Path, '/tank_test/target_path', 10)
        self.create_subscription(
            PoseWithCovarianceStamped, '/boat_pose', self._on_pose, 10)
        self.create_service(
            Trigger, '/tank_test/clear_trajectory', self._clear_trajectory)
        self.create_timer(0.5, self._publish_paths)
        self.create_timer(5.0, self._check_pose)
        self._publish_paths()
        self.get_logger().info(
            f'tank test visualization: {self.kind}; actual=/boat_pose, '
            'target=local test defaults; no motor commands')

    def _on_pose(self, msg):
        if msg.header.frame_id != 'odom':
            self.get_logger().warn(
                f'/boat_pose frame {msg.header.frame_id!r} is not odom',
                throttle_duration_sec=5.0)
            return
        x = float(msg.pose.pose.position.x)
        y = float(msg.pose.pose.position.y)
        if not (math.isfinite(x) and math.isfinite(y)
                and 0.0 <= x <= 10.0 and 0.0 <= y <= 5.0):
            return
        self.history.add_point(x, y)
        self.last_pose_time = time.monotonic()

    def _publish_paths(self):
        stamp = self.get_clock().now().to_msg()
        self.target_pub.publish(create_trajectory_path(
            self.reference, stamp=stamp))
        self.actual_pub.publish(create_trajectory_path(
            self.history.points, stamp=stamp, z_offset=0.06))

    def _clear_trajectory(self, request, response):
        self.history.clear()
        self._publish_paths()
        response.success = True
        response.message = 'shore-LiDAR trajectory cleared'
        return response

    def _check_pose(self):
        if (self.last_pose_time is None or
                time.monotonic() - self.last_pose_time > 5.0):
            self.get_logger().warn(
                'No recent /boat_pose; actual path cannot grow',
                throttle_duration_sec=10.0)


def main(args=None):
    rclpy.init(args=args)
    node = TankTestVisualization()
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
