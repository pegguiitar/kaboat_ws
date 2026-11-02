"""Publish the existing DRI calculation while the boat is driven manually.

This node is read-only: /scan, /odom and /occupancy_grid are its only inputs.
It never publishes a motor command or starts an autonomy behavior.
"""

import dataclasses
import math
import time

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

from kaboat_behaviors.behavior_base import yaw_from_quaternion
from kaboat_behaviors.dri import DriParams, build_dri


def dri_visualization(dri, source: OccupancyGrid, scale: float, stamp):
    """Encode DRI as a 0..100 OccupancyGrid for RViz, using the source map."""
    output = OccupancyGrid()
    output.header.frame_id = source.header.frame_id
    output.header.stamp = stamp
    output.info = source.info
    output.data = np.rint(np.clip(dri.data / scale * 100.0, 0.0, 100.0)).astype(
        np.int8).ravel().tolist()
    return output


class ManualDriMonitor(Node):
    def __init__(self):
        super().__init__('manual_dri_monitor')
        self.declare_parameter('viz_dri_scale', 4.0)
        self.declare_parameter('odom_timeout_sec', 0.5)
        self.declare_parameter('scan_timeout_sec', 0.7)
        self.declare_parameter('grid_timeout_sec', 0.7)
        self.viz_scale = float(self.get_parameter('viz_dri_scale').value)
        self.odom_timeout = float(self.get_parameter('odom_timeout_sec').value)
        self.scan_timeout = float(self.get_parameter('scan_timeout_sec').value)
        self.grid_timeout = float(self.get_parameter('grid_timeout_sec').value)
        for name, value in (
            ('viz_dri_scale', self.viz_scale),
            ('odom_timeout_sec', self.odom_timeout),
            ('scan_timeout_sec', self.scan_timeout),
            ('grid_timeout_sec', self.grid_timeout),
        ):
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f'{name} must be positive and finite')

        defaults = DriParams()
        dri_values = {}
        for field in dataclasses.fields(DriParams):
            name = f'dri.{field.name}'
            self.declare_parameter(name, getattr(defaults, field.name))
            dri_values[field.name] = self.get_parameter(name).value
        self.dri_params = DriParams(**dri_values)
        if (not 0 <= self.dri_params.occ_threshold <= 100
                or not all(math.isfinite(value) for value in (
                    self.dri_params.sigma, self.dri_params.dist_falloff,
                    self.dri_params.k_dist, self.dri_params.k_head,
                    self.dri_params.a_dist, self.dri_params.a_head))
                or self.dri_params.sigma <= 0
                or self.dri_params.dist_falloff <= 0
                or min(self.dri_params.k_dist, self.dri_params.k_head,
                       self.dri_params.a_dist, self.dri_params.a_head) < 0):
            raise ValueError('invalid DRI parameters')

        self.odom = None
        self.grid = None
        self.odom_at = None
        self.scan_at = None
        self.grid_at = None
        self._state = None
        self._status_at = 0.0
        self._last_valid_info = None

        self.dri_pub = self.create_publisher(OccupancyGrid, '/manual_dri/dri', 1)
        self.status_pub = self.create_publisher(String, '/manual_dri/status', 10)
        self.create_subscription(Odometry, '/odom', self._on_odom, 10)
        self.create_subscription(OccupancyGrid, '/occupancy_grid', self._on_grid, 1)
        self.create_subscription(LaserScan, '/scan', self._on_scan,
                                 qos_profile_sensor_data)
        self.create_timer(0.1, self._tick)
        self.get_logger().info('Manual DRI monitor ready; no drive commands are published')

    def _on_odom(self, msg):
        self.odom = msg
        self.odom_at = time.monotonic()

    def _on_grid(self, msg):
        self.grid = msg
        self.grid_at = time.monotonic()

    def _on_scan(self, msg):
        ranges = np.asarray(msg.ranges, dtype=float)
        if np.any(np.isfinite(ranges) & (ranges > msg.range_min)
                  & (ranges < msg.range_max)):
            self.scan_at = time.monotonic()

    def _input_error(self, now):
        if self.odom_at is None or now - self.odom_at > self.odom_timeout:
            return 'WAIT_ODOM'
        pose = self.odom.pose.pose
        q = pose.orientation
        values = (pose.position.x, pose.position.y, q.x, q.y, q.z, q.w)
        if (self.odom.header.frame_id != 'odom'
                or not all(math.isfinite(value) for value in values)
                or abs(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w - 1.0) > 0.1):
            return 'INVALID_ODOM_OR_FRAME'
        if self.scan_at is None or now - self.scan_at > self.scan_timeout:
            return 'WAIT_SCAN'
        if self.grid_at is None or now - self.grid_at > self.grid_timeout:
            return 'WAIT_GRID'
        info = self.grid.info
        origin = info.origin.orientation
        if (self.grid.header.frame_id != 'odom'
                or not math.isfinite(info.resolution) or info.resolution <= 0
                or info.width <= 0 or info.height <= 0
                or len(self.grid.data) != info.width * info.height
                or not all(math.isfinite(value) for value in (
                    info.origin.position.x, info.origin.position.y))
                or max(abs(origin.x), abs(origin.y), abs(origin.z),
                       abs(origin.w - 1.0)) > 1e-3):
            return 'INVALID_GRID_OR_FRAME'
        if not any(value >= 0 for value in self.grid.data):
            return 'WAIT_GRID_OBSERVATIONS'
        return None

    def _status(self, state, detail=''):
        now = time.monotonic()
        changed = state != self._state
        if changed or now - self._status_at >= 1.0:
            self.status_pub.publish(String(data=(
                f'{state}: {detail}' if detail else state)))
            self._status_at = now
        if changed:
            self._state = state
            self.get_logger().info(f'manual DRI: {state}')

    def _clear_dri(self):
        info = self._last_valid_info
        if info is None and self.grid is not None:
            info = self.grid.info
        if info is None:
            return
        size = info.width * info.height
        if (info.width <= 0 or info.height <= 0
                or (self._last_valid_info is None
                    and size != len(self.grid.data))):
            return
        output = OccupancyGrid()
        output.header.frame_id = 'odom'
        output.header.stamp = self.get_clock().now().to_msg()
        output.info = info
        output.data = [-1] * size
        self.dri_pub.publish(output)

    def _tick(self):
        now = time.monotonic()
        error = self._input_error(now)
        if error:
            if error != self._state:
                self._clear_dri()  # remove stale RViz risk on input loss
            self._status(error)
            return
        pose = self.odom.pose.pose
        try:
            dri = build_dri(
                self.grid, (pose.position.x, pose.position.y),
                yaw_from_quaternion(pose.orientation), self.dri_params)
            output = dri_visualization(
                dri, self.grid, self.viz_scale, self.get_clock().now().to_msg())
        except (ValueError, ArithmeticError, OverflowError) as exc:
            self._clear_dri()
            self._status('DRI_ERROR', str(exc))
            return
        self.dri_pub.publish(output)
        self._last_valid_info = self.grid.info
        self._status('READY', f'occupied_cells={len(dri.occ_xy)}')


def main(args=None):
    rclpy.init(args=args)
    node = ManualDriMonitor()
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
