"""Gate behavior: LiDAR grid -> DRI -> paired posts -> shared LOS follower.

Each aperture supplies a mandatory centre crossing. The free-form obstacle
avoidance planner is not used because it can route around a gate.
"""

import dataclasses
import math
import time

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

from .behavior_base import BehaviorBase, yaw_from_quaternion
from .dri import DriParams, build_dri
from .gate_corridor import (
    GateParams, crossed_gate, lookahead_chord_safe, make_gate_path,
    pair_observed, path_safe, points_observed, points_safe, scan_hits_world,
    select_gate_pair)


class GateFollower(BehaviorBase):
    STATE_NAME = 'gate'
    CMD_TOPIC = '/cmd/gate'

    def __init__(self):
        super().__init__()
        self.gate_params = self._params_from('gate', GateParams)
        self.dri_params = self._params_from('dri', DriParams)
        self.declare_parameter('gate.thrust_ratio', 0.5)       # waypoint(접근) 구간
        # LOS 경로 추종 구간: los_path_test 에서 검증된 값 0.3 (전진·회전 출력 상한).
        self.declare_parameter('gate.los_thrust_ratio', 0.3)
        self.declare_parameter('gate.motion_enabled', False)
        self.declare_parameter('gate.scan_timeout_sec', 0.7)
        self.declare_parameter('gate.grid_timeout_sec', 0.7)
        self.thrust_ratio = float(self.get_parameter('gate.thrust_ratio').value)
        self.los_thrust_ratio = float(
            self.get_parameter('gate.los_thrust_ratio').value)
        self.motion_enabled = bool(self.get_parameter('gate.motion_enabled').value)
        self.scan_timeout = float(self.get_parameter('gate.scan_timeout_sec').value)
        self.grid_timeout = float(self.get_parameter('gate.grid_timeout_sec').value)
        if (not math.isfinite(self.thrust_ratio)
                or not 0.0 <= self.thrust_ratio <= 1.0
                or not math.isfinite(self.los_thrust_ratio)
                or not 0.0 <= self.los_thrust_ratio <= 1.0
                or not math.isfinite(self.scan_timeout) or self.scan_timeout <= 0
                or not math.isfinite(self.grid_timeout) or self.grid_timeout <= 0):
            raise ValueError('invalid gate thrust ratio or sensor timeout')

        self._scan_at = None
        self._scan_hits = None
        self._grid_at = None
        self._pair = None
        self._path = None
        self._previous_xy = None
        self._crossed = False
        self._passed_midpoints = []
        self._status_text = None

        self.path_pub = self.create_publisher(Path, '/gate/path', 1)
        self.pair_pub = self.create_publisher(MarkerArray, '/gate/pairs', 1)
        self.status_pub = self.create_publisher(String, '/gate/status', 10)
        self.preview_pub = self.create_publisher(Twist, '/gate/cmd_preview', 10)
        self.create_subscription(LaserScan, '/scan', self._on_scan,
                                 qos_profile_sensor_data)
        self.get_logger().info(
            f'gate LiDAR/DRI: required_pairs={self.gate_params.required_pairs}, '
            f'thrust_ratio={self.thrust_ratio:.2f}, '
            f'los_thrust_ratio={self.los_thrust_ratio:.2f}, '
            f'motion_enabled={self.motion_enabled}')

    def _params_from(self, prefix, cls):
        defaults = cls()
        values = {}
        for field in dataclasses.fields(cls):
            name = f'{prefix}.{field.name}'
            self.declare_parameter(name, getattr(defaults, field.name))
            values[field.name] = self.get_parameter(name).value
        return cls(**values)

    def on_activate(self):
        self._pair = None
        self._path = None
        self._previous_xy = None
        self._crossed = False
        self._passed_midpoints = []
        self._status_text = None

    def on_deactivate(self):
        self._pair = None
        self._path = None
        self._previous_xy = None
        self._crossed = False

    def _on_grid(self, msg: OccupancyGrid):
        super()._on_grid(msg)
        self._grid_at = time.monotonic()

    def _on_scan(self, msg: LaserScan):
        if self.waypoint_odom_ready():
            pose = self.odom.pose.pose
            hits = scan_hits_world(
                msg, (pose.position.x, pose.position.y),
                yaw_from_quaternion(pose.orientation))
            if len(hits) == 0:
                return
            self._scan_hits = hits
            self._scan_at = time.monotonic()

    def _status(self, value):
        if value != self._status_text:
            self._status_text = value
            self.status_pub.publish(String(data=value))
            self.get_logger().info(f'gate: {value}')

    def _stop(self, reason):
        self._previous_xy = None  # no crossing is inferred across sensor gaps
        self._status(reason)
        self._publish_debug()
        self.preview_pub.publish(Twist())
        return Twist()

    def _output(self, cmd):
        self.preview_pub.publish(cmd)
        return cmd if self.motion_enabled else Twist()

    def _publish_debug(self):
        path = Path()
        path.header.frame_id = 'odom'
        path.header.stamp = self.get_clock().now().to_msg()
        if self._path is not None:
            for x, y in self._path.samples:
                pose = PoseStamped()
                pose.header = path.header
                pose.pose.position.x = float(x)
                pose.pose.position.y = float(y)
                pose.pose.position.z = 0.2
                pose.pose.orientation.w = 1.0
                path.poses.append(pose)
        self.path_pub.publish(path)

        marks = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        marks.markers.append(clear)
        if self._pair is not None:
            for i, point in enumerate((self._pair.left, self._pair.right,
                                       self._pair.midpoint)):
                marker = Marker()
                marker.header = path.header
                marker.ns, marker.id = 'gate_pair', i
                marker.type, marker.action = Marker.SPHERE, Marker.ADD
                marker.pose.position.x = point[0]
                marker.pose.position.y = point[1]
                marker.pose.position.z = 0.3
                marker.pose.orientation.w = 1.0
                marker.scale.x = marker.scale.y = marker.scale.z = 0.3
                marker.color.a = 1.0
                marker.color.g = 1.0 if i == 2 else 0.3
                marker.color.r = 0.1 if i == 2 else 1.0
                marks.markers.append(marker)
        self.pair_pub.publish(marks)

    def _goal_leg_safe(self, dri, grid, boat_xy, goal_xy):
        dx = goal_xy[0] - boat_xy[0]
        dy = goal_xy[1] - boat_xy[1]
        distance = math.hypot(dx, dy)
        if distance > self.gate_params.max_goal_after_gates:
            return False
        length = min(2.0, distance)
        n = max(2, int(math.ceil(length / self.gate_params.sample_spacing)) + 1)
        points = [(boat_xy[0] + t * dx / max(distance, 1e-9),
                   boat_xy[1] + t * dy / max(distance, 1e-9))
                  for t in (length * k / (n - 1) for k in range(n))]
        return (points_safe(dri, points, self.gate_params)
                and points_observed(grid, points))

    def compute_cmd(self):
        now = time.monotonic()
        if (self.goal is None or self.goal.header.frame_id != 'odom'
                or not self.waypoint_odom_ready()):
            return self._stop('WAIT_ODOM_OR_GOAL')
        if (self.occupancy_grid is None
                or self.occupancy_grid.header.frame_id != 'odom'
                or self._grid_at is None
                or now - self._grid_at > self.grid_timeout):
            return self._stop('GRID_TIMEOUT_OR_FRAME')
        if self._scan_at is None or now - self._scan_at > self.scan_timeout:
            return self._stop('SCAN_TIMEOUT')

        grid = self.occupancy_grid
        if (grid.info.resolution <= 0
                or len(grid.data) != grid.info.width * grid.info.height):
            return self._stop('INVALID_GRID')
        pose = self.odom.pose.pose
        boat = (pose.position.x, pose.position.y)
        goal = (self.goal.pose.position.x, self.goal.pose.position.y)
        yaw = yaw_from_quaternion(pose.orientation)
        dri = build_dri(grid, boat, yaw, self.dri_params)

        if len(self._passed_midpoints) >= self.gate_params.required_pairs:
            if self.distance_to_goal() <= 2.0:
                self.report_complete()
                self._status('COMPLETE')
                self._publish_debug()
                return self._output(Twist())
            if not self._goal_leg_safe(dri, grid, boat, goal):
                return self._stop('GOAL_LEG_UNSAFE_OR_TOO_FAR')
            cmd, _ = self.waypoint(*goal, thrust_ratio=self.thrust_ratio,
                                   slow_radius=3.0)
            self._status('GOAL_APPROACH')
            self._publish_debug()
            return self._output(cmd)

        if self._path is None:
            pair = select_gate_pair(
                grid, boat, goal, self._passed_midpoints, self.gate_params)
            if pair is None or not pair_observed(
                    pair, self._scan_hits, self.gate_params):
                return self._stop('WAIT_GATE_PAIR')
            self._pair = pair
            self._path = make_gate_path(boat, pair, self.gate_params)
            self._previous_xy = boat
            self._crossed = False
            self._los_follower.reset()

        if not self._crossed and not pair_observed(
                self._pair, self._scan_hits, self.gate_params):
            return self._stop('GATE_POSTS_NOT_VISIBLE')

        progress = self._los_follower.progress_idx or 0
        if not path_safe(dri, self._path, self.gate_params, progress):
            return self._stop('PATH_UNSAFE')
        if not points_observed(
                grid, self._path.samples[max(0, progress - 2):]):
            return self._stop('PATH_UNOBSERVED')
        cmd, reached = self.los(self._path, self.los_thrust_ratio)
        progress = self._los_follower.progress_idx or 0
        if not lookahead_chord_safe(
                dri, boat, self._path, progress,
                self._los_follower.params.lookahead, self.gate_params, grid):
            return self._stop('LOOKAHEAD_UNSAFE')

        if (not self._crossed and self._previous_xy is not None
                and crossed_gate(self._previous_xy, boat,
                                 self._pair, self.gate_params)):
            self._crossed = True
        self._previous_xy = boat

        if reached:
            if not self._crossed:
                return self._stop('GATE_NOT_CROSSED')
            self._passed_midpoints.append(self._pair.midpoint)
            passed = len(self._passed_midpoints)
            self._pair = None
            self._path = None
            self._previous_xy = None
            self._crossed = False
            self._los_follower.reset()
            self._status(f'GATE_{passed}/{self.gate_params.required_pairs}_PASSED')
            self._publish_debug()
            return self._output(Twist())

        self._status(f'FOLLOW_GATE_{len(self._passed_midpoints) + 1}/'
                     f'{self.gate_params.required_pairs}')
        self._publish_debug()
        return self._output(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = GateFollower()
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
