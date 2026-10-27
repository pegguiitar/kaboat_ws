"""Real LiDAR avoidance test using the mission's DRI, B-spline planner and FSM.

The node consumes the existing /scan -> /occupancy_grid pipeline and /odom.
It previews a short goal straight ahead of the boat until /start_mission is true
or /real_avoid_test/start is called. Motor output is disabled by default; when enabled it
publishes directly to /cmd_vel, so no other /cmd_vel source may run alongside it.
"""

import dataclasses
import math
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker, MarkerArray

from kaboat_behaviors.avoid_fsm import AvoidFsm, FsmParams
from kaboat_behaviors.behavior_base import YAW_KD, YAW_KP, yaw_from_quaternion
from kaboat_behaviors.bspline_planner import (
    PlannerParams, check, generate,
)
from kaboat_behaviors.dri import DriParams, build_dri
from kaboat_hardware.trajectory_visualization import (
    BoundedTrajectoryHistory, create_trail_marker,
)


class RealObstacleAvoidTest(Node):
    """Standalone tank test; it does not depend on mission_manager or cmd_mux."""

    def __init__(self):
        super().__init__('real_obstacle_avoid_test')

        self.declare_parameter('drive_enabled', False)
        self.declare_parameter('allow_escape', False)
        self.declare_parameter('goal_distance', 3.0)
        self.declare_parameter('goal_tolerance', 0.5)
        self.declare_parameter('max_cross_track_error', 0.7)
        self.declare_parameter('max_linear_cmd', 0.10)
        self.declare_parameter('max_angular_cmd', 0.06)
        self.declare_parameter('odom_timeout_sec', 0.5)
        self.declare_parameter('scan_timeout_sec', 0.5)
        self.declare_parameter('grid_timeout_sec', 0.5)
        self.declare_parameter('require_front_obstacle', True)
        self.declare_parameter('front_corridor_half_width', 1.0)
        self.declare_parameter('viz_dri_scale', 4.0)

        self.drive_enabled = bool(self.get_parameter('drive_enabled').value)
        self.allow_escape = bool(self.get_parameter('allow_escape').value)
        self.goal_distance = float(self.get_parameter('goal_distance').value)
        self.goal_tolerance = float(self.get_parameter('goal_tolerance').value)
        self.max_cross_track_error = float(
            self.get_parameter('max_cross_track_error').value)
        self.max_linear_cmd = float(self.get_parameter('max_linear_cmd').value)
        self.max_angular_cmd = float(self.get_parameter('max_angular_cmd').value)
        self.odom_timeout = float(self.get_parameter('odom_timeout_sec').value)
        self.scan_timeout = float(self.get_parameter('scan_timeout_sec').value)
        self.grid_timeout = float(self.get_parameter('grid_timeout_sec').value)
        self.require_front_obstacle = bool(
            self.get_parameter('require_front_obstacle').value)
        self.front_corridor_half_width = float(
            self.get_parameter('front_corridor_half_width').value)
        self.viz_dri_scale = float(self.get_parameter('viz_dri_scale').value)

        for name, value in (
            ('goal_distance', self.goal_distance),
            ('goal_tolerance', self.goal_tolerance),
            ('max_cross_track_error', self.max_cross_track_error),
            ('odom_timeout_sec', self.odom_timeout),
            ('scan_timeout_sec', self.scan_timeout),
            ('grid_timeout_sec', self.grid_timeout),
            ('viz_dri_scale', self.viz_dri_scale),
            ('front_corridor_half_width', self.front_corridor_half_width),
        ):
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f'{name} must be positive and finite')
        for name, value in (
            ('max_linear_cmd', self.max_linear_cmd),
            ('max_angular_cmd', self.max_angular_cmd),
        ):
            if not math.isfinite(value) or not 0.0 < value <= 1.0:
                raise ValueError(f'{name} must be in (0, 1]')

        self.dri_params = self._params_from('dri', DriParams)
        self.planner_params = self._params_from('planner', PlannerParams)
        self.fsm_params = self._params_from('fsm', FsmParams)
        self.fsm = AvoidFsm(self.planner_params, self.fsm_params,
                            yaw_kp=YAW_KP, yaw_kd=YAW_KD)

        self.odom = None
        self.grid = None
        self.scan = None
        self.odom_received = None
        self.grid_received = None
        self.scan_received = None
        self.started = False
        self.finished = False
        self.estopped = False
        self.goal = None
        self.preview_goal = None
        self.preview_plan = None
        self.preview_received = None
        self.preview_obstacle_distance = None
        self.trajectory_history = BoundedTrajectoryHistory()

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.preview_cmd_pub = self.create_publisher(
            Twist, '/real_avoid_test/cmd_preview', 10)
        self.path_pub = self.create_publisher(Path, '/real_avoid_test/path', 10)
        self.dri_pub = self.create_publisher(
            OccupancyGrid, '/real_avoid_test/dri', 10)
        self.marker_pub = self.create_publisher(
            MarkerArray, '/real_avoid_test/markers', 10)
        self.status_pub = self.create_publisher(
            String, '/real_avoid_test/status', 10)

        self.create_subscription(Odometry, '/odom', self._on_odom, 10)
        self.create_subscription(OccupancyGrid, '/occupancy_grid',
                                 self._on_grid, 1)
        self.create_subscription(LaserScan, '/scan', self._on_scan,
                                 qos_profile_sensor_data)
        self.create_subscription(Bool, '/start_mission',
                                 self._on_start_mission, 10)
        self.create_subscription(Bool, '/emergency_stop', self._on_estop, 10)
        self.create_service(Trigger, '/real_avoid_test/start',
                            self._on_start_service)
        self.create_timer(0.1, self._tick)

        mode = 'DRIVE' if self.drive_enabled else 'PREVIEW ONLY'
        self.get_logger().info(
            f'Real avoidance test ready ({mode}). Goal: {self.goal_distance:.1f} m '
            'ahead of the start pose. Waiting for /scan, /odom and /occupancy_grid.')

    def _params_from(self, prefix, cls):
        defaults = cls()
        values = {}
        for field in dataclasses.fields(cls):
            name = f'{prefix}.{field.name}'
            self.declare_parameter(name, getattr(defaults, field.name))
            values[field.name] = self.get_parameter(name).value
        return cls(**values)

    def _on_odom(self, msg):
        self.odom = msg
        self.odom_received = time.monotonic()
        if self.started and not self.finished:
            point = msg.pose.pose.position
            self.trajectory_history.add_point(point.x, point.y)

    def _on_grid(self, msg):
        self.grid = msg
        self.grid_received = time.monotonic()

    def _on_scan(self, msg):
        self.scan = msg
        self.scan_received = time.monotonic()

    def _input_error(self, now):
        for name, msg, received, timeout in (
            ('/odom', self.odom, self.odom_received, self.odom_timeout),
            ('/scan', self.scan, self.scan_received, self.scan_timeout),
            ('/occupancy_grid', self.grid, self.grid_received,
             self.grid_timeout),
        ):
            if msg is None or received is None:
                return f'waiting for {name}'
            if now - received > timeout:
                return f'{name} stale ({now - received:.2f} s)'
        if not self.scan.ranges:
            return '/scan has no ranges'
        if not any(self.scan.range_min < r < self.scan.range_max
                   and math.isfinite(r) for r in self.scan.ranges):
            return '/scan has no valid obstacle returns'
        if (self.grid.info.resolution <= 0.0 or not self.grid.data
                or len(self.grid.data) != self.grid.info.width * self.grid.info.height):
            return '/occupancy_grid is empty'
        if (self.odom.header.frame_id and self.grid.header.frame_id
                and self.odom.header.frame_id != self.grid.header.frame_id):
            return 'odom and grid frame IDs differ'
        pose = self.odom.pose.pose
        twist = self.odom.twist.twist
        values = (pose.position.x, pose.position.y,
                  pose.orientation.x, pose.orientation.y,
                  pose.orientation.z, pose.orientation.w,
                  twist.linear.x, twist.linear.y, twist.angular.z)
        if not all(math.isfinite(v) for v in values):
            return '/odom contains nonfinite values'
        q = pose.orientation
        qnorm = q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w
        if not 0.8 <= qnorm <= 1.2:
            return '/odom orientation quaternion is invalid'
        return None

    def _boat_state(self):
        pose = self.odom.pose.pose
        yaw = yaw_from_quaternion(pose.orientation)
        c, s = math.cos(yaw), math.sin(yaw)
        twist = self.odom.twist.twist
        vel = (c * twist.linear.x - s * twist.linear.y,
               s * twist.linear.x + c * twist.linear.y)
        return (pose.position.x, pose.position.y), yaw, vel, twist.angular.z

    def _relative_goal(self, boat, yaw):
        return (boat[0] + self.goal_distance * math.cos(yaw),
                boat[1] + self.goal_distance * math.sin(yaw))

    def _front_obstacle_distance(self, occ_xy, boat, yaw):
        """Require an obstacle on the short test lane, not just a side wall."""
        if len(occ_xy) == 0:
            return None
        delta = occ_xy - np.asarray(boat)
        c, s = math.cos(yaw), math.sin(yaw)
        forward = delta[:, 0] * c + delta[:, 1] * s
        lateral = -delta[:, 0] * s + delta[:, 1] * c
        on_lane = ((forward > 0.0) & (forward <= self.goal_distance)
                   & (np.abs(lateral) <= self.front_corridor_half_width))
        if not np.any(on_lane):
            return None
        return float(np.min(np.hypot(delta[on_lane, 0], delta[on_lane, 1])))

    def _on_start_mission(self, msg):
        if msg.data:
            self._arm()
        else:
            self._disarm('start_mission false')

    def _on_start_service(self, _request, response):
        response.success, response.message = self._arm()
        return response

    def _arm(self):
        now = time.monotonic()
        if self.started:
            return False, 'avoidance test already running'
        if self.finished:
            return False, 'test finished; send /start_mission false to reset'
        if self.estopped:
            return False, 'E-stop is active'
        reason = self._input_error(now)
        if reason:
            return False, reason
        if (self.preview_plan is None or self.preview_goal is None
                or self.preview_received is None
                or now - self.preview_received > 0.3):
            return False, 'wait for a valid preview B-spline path'
        if (self.require_front_obstacle
                and self.preview_obstacle_distance is None):
            return False, 'no confirmed obstacle ahead within goal distance'
        self.goal = self.preview_goal
        self.fsm.reset()
        self.started = True
        self.finished = False
        self.trajectory_history.clear()
        self.get_logger().info(
            f'Test armed: goal=({self.goal[0]:.2f}, {self.goal[1]:.2f}), '
            f'drive_enabled={self.drive_enabled}')
        return True, 'avoidance test armed'

    def _on_estop(self, msg):
        if msg.data:
            self.estopped = True
            self._disarm('emergency_stop')
        else:
            self.estopped = False

    def _disarm(self, reason):
        if self.started:
            self.get_logger().warn(f'avoidance test disarmed: {reason}')
        self.started = False
        self.finished = False
        self.goal = None
        self.preview_plan = None
        self.preview_received = None
        self.fsm.reset()
        self._publish_command(Twist())

    def _publish_command(self, cmd):
        self.preview_cmd_pub.publish(cmd)
        if self.drive_enabled:
            self.cmd_pub.publish(cmd)

    def _publish_status(self, state, detail):
        msg = String()
        msg.data = f'{state}: {detail}'
        self.status_pub.publish(msg)

        marker = Marker()
        marker.header.frame_id = (
            self.grid.header.frame_id if self.grid is not None
            and self.grid.header.frame_id else 'odom')
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns, marker.id = 'test_status', 0
        marker.type, marker.action = Marker.TEXT_VIEW_FACING, Marker.ADD
        if self.odom is not None:
            marker.pose.position.x = self.odom.pose.pose.position.x
            marker.pose.position.y = self.odom.pose.pose.position.y
        marker.pose.position.z = 0.7
        marker.pose.orientation.w = 1.0
        marker.scale.z = 0.28
        marker.color.a = 1.0
        if state in ('STOP', 'ERROR', 'ESTOP'):
            marker.color.r = 1.0
        elif state in ('PREVIEW', 'FOLLOW', 'FINISHED'):
            marker.color.g = 1.0
        else:
            marker.color.r = marker.color.g = 1.0
        marker.text = state
        marker.lifetime.sec = 1
        self.marker_pub.publish(MarkerArray(markers=[marker]))

    def _clear_visualization(self):
        path = Path()
        path.header.frame_id = (
            self.grid.header.frame_id if self.grid is not None
            and self.grid.header.frame_id else 'odom')
        path.header.stamp = self.get_clock().now().to_msg()
        self.path_pub.publish(path)
        marker = Marker()
        marker.action = Marker.DELETEALL
        self.marker_pub.publish(MarkerArray(markers=[marker]))

    def _tick(self):
        now = time.monotonic()
        reason = self._input_error(now)
        if self.estopped:
            self._publish_command(Twist())
            self._clear_visualization()
            self._publish_status('ESTOP', 'emergency_stop active')
            return
        if reason:
            if self.started:
                self._disarm(reason)
            else:
                self._publish_command(Twist())
            self.preview_plan = None
            self.preview_received = None
            self._clear_visualization()
            self._publish_status('WAIT', reason)
            return
        if self.finished:
            self._publish_command(Twist())
            self._publish_status('FINISHED', 'goal tolerance reached')
            return

        boat, yaw, vel, yaw_rate = self._boat_state()
        goal = self.goal if self.started else self._relative_goal(boat, yaw)
        try:
            dri = build_dri(self.grid, boat, yaw, self.dri_params)
            now_t = (self.odom.header.stamp.sec
                     + self.odom.header.stamp.nanosec * 1e-9)
            if now_t <= 0.0:
                now_t = now

            if not self.started:
                plan = generate(dri, boat, yaw, vel, goal,
                                self.planner_params, now_t)
                if plan is not None and check(
                        plan, dri, boat, self.planner_params) is not None:
                    plan = None
                self.preview_plan = plan
                self.preview_goal = goal
                self.preview_received = now
                self.preview_obstacle_distance = self._front_obstacle_distance(
                    dri.occ_xy, boat, yaw)
                self._publish_command(Twist())
                self._publish_debug(dri, plan, goal)
                self._publish_status(
                    'PREVIEW' if plan is not None else 'NO PLAN',
                    f'obstacles={len(dri.occ_xy)}, '
                    f'front={self.preview_obstacle_distance}, '
                    f'drive_enabled={self.drive_enabled}')
                return

            if math.hypot(goal[0] - boat[0], goal[1] - boat[1]) <= self.goal_tolerance:
                self.started = False
                self.finished = True
                self._publish_command(Twist())
                self._publish_debug(dri, self.fsm.plan, goal)
                self._publish_status('FINISHED', 'goal tolerance reached')
                return

            out = self.fsm.step(dri, boat, yaw, vel, yaw_rate, goal, now_t)
            if out.event:
                self.get_logger().info(f'[{out.state}] {out.event}',
                                       throttle_duration_sec=1.0)
            if out.state == 'ESCAPE' and not self.allow_escape:
                self._disarm('ESCAPE requested; reverse motion disabled')
                self._publish_debug(dri, None, goal)
                self._publish_status('STOP', 'ESCAPE requested')
                return
            if not (math.isfinite(out.linear_x) and math.isfinite(out.angular_z)):
                self._disarm('FSM produced a nonfinite command')
                self._publish_status('ERROR', 'nonfinite command')
                return

            if out.plan is not None:
                ahead = out.plan.samples[max(0, out.plan.progress_idx - 3):]
                cross_track = float(np.min(np.hypot(
                    ahead[:, 0] - boat[0], ahead[:, 1] - boat[1])))
                if cross_track > self.max_cross_track_error:
                    self._disarm(f'cross-track error {cross_track:.2f} m')
                    self._publish_debug(dri, None, goal)
                    self._publish_status('STOP', 'cross-track limit')
                    return

            cmd = Twist()
            cmd.linear.x = float(np.clip(out.linear_x,
                                         -self.max_linear_cmd,
                                         self.max_linear_cmd))
            cmd.angular.z = float(np.clip(out.angular_z,
                                          -self.max_angular_cmd,
                                          self.max_angular_cmd))
            self._publish_command(cmd)
            self._publish_debug(dri, out.plan, goal)
            self._publish_status(out.state, out.event or 'following')
        except (ValueError, ArithmeticError) as exc:
            self._disarm(f'planner error: {exc}')
            self._publish_status('ERROR', str(exc))

    def _publish_debug(self, dri, plan, goal):
        stamp = self.odom.header.stamp
        frame = self.grid.header.frame_id or 'odom'
        path = Path()
        path.header.stamp = stamp
        path.header.frame_id = frame
        if plan is not None:
            for x, y in plan.samples:
                point = PoseStamped()
                point.header = path.header
                point.pose.position.x = float(x)
                point.pose.position.y = float(y)
                point.pose.position.z = 0.1
                point.pose.orientation.w = 1.0
                path.poses.append(point)
        self.path_pub.publish(path)

        if self.count_subscribers('/real_avoid_test/dri') > 0:
            grid = OccupancyGrid()
            grid.header = self.grid.header
            grid.info = self.grid.info
            values = np.clip(dri.data / self.viz_dri_scale * 100.0,
                             0.0, 100.0)
            grid.data = np.rint(values).astype(np.int8).ravel().tolist()
            self.dri_pub.publish(grid)

        markers = MarkerArray()
        wp = Marker()
        wp.header = path.header
        wp.ns, wp.id = 'goal', 0
        wp.type, wp.action = Marker.SPHERE, Marker.ADD
        wp.pose.position.x, wp.pose.position.y = goal
        wp.pose.position.z = 0.15
        wp.pose.orientation.w = 1.0
        wp.scale.x = wp.scale.y = wp.scale.z = 0.3
        wp.color.g, wp.color.a = 1.0, 1.0
        markers.markers.append(wp)

        for i in range(6):
            marker = Marker()
            marker.header = path.header
            marker.ns, marker.id = 'control_points', i
            if plan is None or i >= len(plan.cps):
                marker.action = Marker.DELETE
            else:
                marker.type, marker.action = Marker.SPHERE, Marker.ADD
                marker.pose.position.x = float(plan.cps[i, 0])
                marker.pose.position.y = float(plan.cps[i, 1])
                marker.pose.position.z = 0.1
                marker.pose.orientation.w = 1.0
                marker.scale.x = marker.scale.y = marker.scale.z = 0.15
                marker.color.r, marker.color.a = 1.0, 1.0
            markers.markers.append(marker)

        trail = create_trail_marker(
            points=self.trajectory_history.points,
            frame_id=frame, stamp=stamp,
            ns='actual_trajectory', marker_id=0)
        trail.lifetime.sec = 1
        markers.markers.append(trail)
        self.marker_pub.publish(markers)


def main(args=None):
    rclpy.init(args=args)
    node = RealObstacleAvoidTest()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._publish_command(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
