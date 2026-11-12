"""GPS-positioned, single-plan docking with straight LOS segments."""
import json
import math
import time
from collections import deque

import numpy as np
import rclpy
from geometry_msgs.msg import PointStamped, PoseStamped, Twist
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data as sensor_qos
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String
from std_srvs.srv import Trigger

from kaboat_behaviors.behavior_base import yaw_from_quaternion
from kaboat_behaviors.los_controller import LosFollower, sampled_path
from .dock_one_shot import LocalGeo, aligned_normals, fixed_route, symmetric_markers, wrap


def stamp(msg):
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


class DockCtrlV2(Node):
    def __init__(self, **node_kwargs):
        super().__init__('dock_ctrl_v2', **node_kwargs)
        defaults = {'drive_enabled': False, 'manual_handoff_enabled': False,
                    'mission_mode': False,
                    'stage_latitude': -999.0,
                    'stage_longitude': -999.0, 'sim_stage_odom_xy': [-999.0, -999.0],
                    'stage_radius_m': 3.0, 'gps_timeout_s': 1.5,
                    'odom_timeout_s': 1.0, 'perception_timeout_s': 1.5,
                    'stable_samples': 5, 'stable_duration_s': 0.8,
                    'max_turn_cmd': 0.20,
                    'approach_cmd': 0.10, 'final_cmd': 0.05,
                    'via_tolerance_m': 0.30, 'goal_tolerance_m': 0.25,
                    'yaw_tolerance_deg': 8.0, 'target_class': 'triangle'}
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        p = lambda name: self.get_parameter(name).value
        self.drive = bool(p('drive_enabled'))
        self.manual_handoff = bool(p('manual_handoff_enabled'))
        self.mission_mode = bool(p('mission_mode'))
        self.mission_state = ''
        self.complete_sent = False
        self.stage_ll = (float(p('stage_latitude')), float(p('stage_longitude')))
        self.sim_stage = np.asarray(p('sim_stage_odom_xy'), float)
        if self.sim_stage.shape != (2,):
            raise ValueError('sim_stage_odom_xy must be XY')
        self.stage_radius = float(p('stage_radius_m'))
        self.gps_timeout = float(p('gps_timeout_s'))
        self.odom_timeout = float(p('odom_timeout_s'))
        self.perception_timeout = float(p('perception_timeout_s'))
        self.stable_samples = int(p('stable_samples'))
        self.stable_duration = float(p('stable_duration_s'))
        self.max_turn = float(p('max_turn_cmd'))
        self.approach_cmd = float(p('approach_cmd'))
        self.final_cmd = float(p('final_cmd'))
        self.via_tol = float(p('via_tolerance_m'))
        self.goal_tol = float(p('goal_tolerance_m'))
        self.yaw_tol = math.radians(float(p('yaw_tolerance_deg')))
        self.target = str(p('target_class'))
        if self.target not in ('triangle', 'circle', 'square'):
            raise ValueError('invalid target_class')
        if not all(math.isfinite(v) and v > 0 for v in
                   (self.stage_radius, self.gps_timeout, self.odom_timeout,
                    self.perception_timeout, self.stable_duration,
                    self.max_turn, self.approach_cmd, self.final_cmd,
                    self.via_tol, self.goal_tol, self.yaw_tol)) or self.stable_samples < 2:
            raise ValueError('invalid docking controller limits')
        if not self.mission_mode and self.stage_ll[0] == -999 and self.sim_stage[0] == -999:
            self.get_logger().warn('No staging waypoint: set stage_latitude/longitude before start')
        self.geo = None
        self.gps = None
        self.gps_received = 0.0
        self.gps_history = deque(maxlen=5)
        self.odom = None
        self.odom_received = 0.0
        self.det = self.lidar = self.goal = None
        self.det_received = self.lidar_received = self.goal_received = 0.0
        self.waypoints = None
        self.waypoints_received = 0.0
        self.phase = 'IDLE'
        self.ready_since = None
        self.ready_count = 0
        self.last_ready_scan_stamp = None
        self.last_scan_stamp = None
        self.first_plan = self.final_plan = None
        self.route_enu = None
        self.route_origin_odom = None
        self.route_origin_enu = None
        self.sim_stage_enu = None
        self.fixed_path_msg = self.fixed_gps_msg = None
        self.last_path_pub = 0.0
        self.follower = LosFollower(yaw_kp=1.4, yaw_kd=.16,
                                    max_linear=1.0, max_angular=1.0,
                                    goal_tolerance=self.via_tol,
                                    ratio_limits_only=True)
        self.cmd = self.create_publisher(Twist, '/cmd/dock' if self.mission_mode else '/cmd_vel', 10)
        self.complete_pub = self.create_publisher(String, '/mission/complete', 10)
        self.preview = self.create_publisher(Twist, '/dock/cmd_preview', 10)
        self.status = self.create_publisher(String, '/dock/status', 10)
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.path_pub = self.create_publisher(Path, '/dock/one_shot_path', latched)
        self.gps_path_pub = self.create_publisher(String, '/dock/one_shot_gps', latched)
        self.create_subscription(NavSatFix, '/gps/fix', self._gps, sensor_qos)
        self.create_subscription(Odometry, '/odom', self._odom, sensor_qos)
        self.create_subscription(String, '/dock/detection_status', self._det, 10)
        self.create_subscription(String, '/dock/lidar/status', self._lidar, 10)
        self.create_subscription(String, '/dock/avoid/goal_status', self._goal, 10)
        self.create_subscription(Path, '/dock/waypoints', self._waypoints, 10)
        if self.mission_mode:
            self.create_subscription(String, '/mission/state', self._on_mission_state, latched)
        self.create_service(Trigger, '/dock/start', self._start)
        self.create_service(Trigger, '/dock/stop', self._stop)
        self.create_timer(.1, self._tick)

    def _gps(self, msg):
        if (msg.status.status < 0 or not math.isfinite(msg.latitude)
                or not math.isfinite(msg.longitude) or not -90 <= msg.latitude <= 90
                or not -180 <= msg.longitude <= 180):
            return
        if self.geo is None:
            self.geo = LocalGeo(msg.latitude, msg.longitude)
        self.gps, self.gps_received = msg, time.monotonic()
        self.gps_history.append(self.geo.xy(msg.latitude, msg.longitude))

    def _odom(self, msg):
        self.odom, self.odom_received = msg, time.monotonic()

    def _det(self, msg):
        try:
            self.det = json.loads(msg.data)
            self.det_received = time.monotonic()
        except ValueError:
            self.det = None

    def _lidar(self, msg):
        try:
            data = json.loads(msg.data)
            if data.get('scan_stamp_s') != self.last_scan_stamp:
                self.lidar = data
                self.last_scan_stamp = data.get('scan_stamp_s')
                self.lidar_received = time.monotonic()
        except ValueError:
            self.lidar = None

    def _goal(self, msg):
        try:
            self.goal = json.loads(msg.data)
            self.goal_received = time.monotonic()
        except ValueError:
            self.goal = None

    def _waypoints(self, msg):
        if msg.header.frame_id == 'odom' and len(msg.poses) == 3:
            self.waypoints, self.waypoints_received = msg, time.monotonic()

    def _position(self):
        return np.median(np.asarray(self.gps_history), axis=0)

    def _odom_xy(self):
        p = self.odom.pose.pose.position
        return np.array([p.x, p.y])

    def _stage(self):
        if self.stage_ll[0] != -999 and self.stage_ll[1] != -999:
            return self.geo.xy(*self.stage_ll)
        if self.sim_stage_enu is not None:
            return self.sim_stage_enu
        if self.sim_stage[0] != -999 and self.odom is not None:
            return self._position() + self.sim_stage - self._odom_xy()
        return None

    def _inputs(self):
        now = time.monotonic()
        if not self.gps_history or now - self.gps_received > self.gps_timeout:
            return 'GPS fix stale or missing'
        if (self.odom is None or now - self.odom_received > self.odom_timeout
                or self.odom.header.frame_id != 'odom'):
            return 'odom yaw stale or frame invalid'
        return None

    def _reset_search(self, phase):
        self.phase = phase
        self.complete_sent = False
        self.ready_count = 0
        self.ready_since = None
        self.last_ready_scan_stamp = None
        self.route_enu = None
        self.route_origin_odom = self.route_origin_enu = None
        self.fixed_path_msg = self.fixed_gps_msg = None
        self.first_plan = self.final_plan = None
        self.det = self.lidar = self.goal = self.waypoints = None
        self.det_received = self.lidar_received = self.goal_received = 0.0
        self.waypoints_received = 0.0
        self.last_scan_stamp = None
        self.follower.reset()
        empty_path = Path()
        empty_path.header.frame_id = 'odom'
        empty_path.header.stamp = self.get_clock().now().to_msg()
        self.path_pub.publish(empty_path)
        self.gps_path_pub.publish(String(data=json.dumps({'one_shot': False, 'route': None})))

    def _on_mission_state(self, msg):
        old = self.mission_state
        self.mission_state = msg.data
        if msg.data == 'dock' and old != 'dock':
            # mission_approach reached the entry waypoint; only a measured
            # wall-normal yaw correction is needed before planning.
            self._reset_search('SCAN')
        elif msg.data != 'dock' and old == 'dock':
            self.phase = 'IDLE'
            self._command(0, 0)

    def _start(self, _request, response):
        error = self._inputs()
        if self.mission_mode and self.mission_state != 'dock':
            error = 'mission state is not dock'
        if error is None and not self.mission_mode and self._stage() is None:
            error = 'staging GPS waypoint has not been set'
        response.success = error is None
        response.message = error or 'GPS docking mission started'
        if error is None:
            if not self.mission_mode and self.sim_stage[0] != -999:
                self.sim_stage_enu = self._position() + self.sim_stage - self._odom_xy()
            self._reset_search('SCAN' if self.mission_mode else 'STAGE')
        return response

    def _stop(self, _request, response):
        self.phase = 'IDLE'
        self._command(0, 0)
        if self.drive and self.manual_handoff:
            self.cmd.publish(Twist())
        response.success = True
        response.message = 'docking stopped'
        return response

    def _command(self, linear, angular):
        msg = Twist()
        msg.linear.x = float(linear)
        msg.angular.z = float(angular)
        self.preview.publish(msg)
        if self.drive and (not self.manual_handoff or self.phase != 'IDLE'):
            self.cmd.publish(msg)

    def _report(self, detail):
        data = {'phase': self.phase, 'detail': detail, 'drive_enabled': self.drive,
                'manual_handoff_enabled': self.manual_handoff,
                'gps_xy': self._position().tolist() if self.gps_history else None,
                'route_fixed': self.route_enu is not None,
                'route_xy': [xy.tolist() for xy in self.route_enu] if self.route_enu else None}
        self.status.publish(String(data=json.dumps(data)))

    def _yaw(self):
        return yaw_from_quaternion(self.odom.pose.pose.orientation)

    def _turn(self, heading, yaw):
        rate = self.odom.twist.twist.angular.z
        return float(np.clip(1.4 * wrap(heading - yaw) - .16 * rate,
                             -self.max_turn, self.max_turn))

    def _route(self):
        if (self.goal is None or not self.goal.get('valid') or self.waypoints is None
                or time.monotonic() - self.goal_received > self.perception_timeout
                or time.monotonic() - self.waypoints_received > self.perception_timeout):
            return False
        if self.goal.get('geometry_source') != 'lidar_ransac':
            return False
        origin_odom = self._odom_xy()
        origin_enu = self._position()
        via_odom = np.array([self.waypoints.poses[0].pose.position.x,
                             self.waypoints.poses[0].pose.position.y])
        goal_odom = np.array([self.waypoints.poses[2].pose.position.x,
                              self.waypoints.poses[2].pose.position.y])
        if np.linalg.norm(np.asarray(self.goal.get('goal_xy', [math.inf, math.inf]))
                          - goal_odom) > .25:
            return False
        via = origin_enu + via_odom - origin_odom
        stop = origin_enu + goal_odom - origin_odom
        try:
            first, second, heading = fixed_route(origin_enu, via, stop)
        except ValueError:
            return False
        self.route_enu = (origin_enu.copy(), via, stop)
        self.route_origin_odom = origin_odom.copy()
        self.route_origin_enu = origin_enu.copy()
        self.first_plan = sampled_path(first, spacing=.1, curvatures=np.zeros(len(first)), frame_id='dock_enu')
        self.final_plan = sampled_path(second, spacing=.1, curvatures=np.zeros(len(second)), frame_id='dock_enu')
        self.final_heading = heading
        self.follower.reset()
        path = Path()
        path.header.frame_id = 'odom'
        path.header.stamp = self.get_clock().now().to_msg()
        for xy in np.vstack((first, second[1:])):
            p = PoseStamped()
            p.header = path.header
            p.pose.position.x, p.pose.position.y = map(float, xy - origin_enu + origin_odom)
            p.pose.position.z = 0.18  # keep the RViz line above the grid and goal markers
            p.pose.orientation.w = 1.0
            path.poses.append(p)
        self.fixed_path_msg = path
        gps_points = [dict(latitude=lat, longitude=lon)
                      for lat, lon in (self.geo.latlon(xy) for xy in self.route_enu)]
        self.fixed_gps_msg = String(data=json.dumps({'start': gps_points[0],
            'via': gps_points[1], 'goal': gps_points[2], 'frame': 'WGS84',
            'one_shot': True}))
        self.path_pub.publish(self.fixed_path_msg)
        self.gps_path_pub.publish(self.fixed_gps_msg)
        self.last_path_pub = time.monotonic()
        return True

    def _tick(self):
        if self.fixed_path_msg is not None and time.monotonic()-self.last_path_pub >= 1.0:
            self.path_pub.publish(self.fixed_path_msg)
            self.gps_path_pub.publish(self.fixed_gps_msg)
            self.last_path_pub = time.monotonic()
        error = self._inputs()
        if self.phase == 'IDLE':
            self._command(0, 0)
            self._report(error or 'waiting for /dock/start')
            return
        if error:
            self._command(0, 0)
            self._report(error)
            return
        xy, yaw = self._position(), self._yaw()
        if self.phase == 'STAGE':
            stage = self._stage()
            if stage is None:
                self._command(0, 0); self._report('staging waypoint unavailable'); return
            distance = np.linalg.norm(xy - stage)
            if distance <= self.stage_radius:
                self.phase = 'SCAN'
                self.ready_count = 0
                self.ready_since = None
                self.last_ready_scan_stamp = None
                self._command(0, 0)
            else:
                heading = math.atan2(stage[1]-xy[1], stage[0]-xy[0])
                turn = self._turn(heading, yaw)
                self._command(self.approach_cmd if abs(wrap(heading-yaw)) < .5 else 0, turn)
            self._report(f'staging distance {distance:.2f}m')
            return
        if self.phase == 'SCAN':
            now = time.monotonic()
            detection_fresh = self.det is not None and now-self.det_received < self.perception_timeout
            lidar_fresh = self.lidar is not None and now-self.lidar_received < self.perception_timeout
            symmetric, why = symmetric_markers(self.det or {}) if detection_fresh else (False, 'YOLO stale')
            aligned, heading, normal_why = aligned_normals(self.lidar or {}, yaw) if lidar_fresh else (False, None, 'RANSAC stale')
            ready = symmetric and aligned
            if ready:
                scan_stamp = self.lidar.get('scan_stamp_s')
                if scan_stamp is not None and scan_stamp != self.last_ready_scan_stamp:
                    if self.ready_since is None:
                        self.ready_since = now
                    self.ready_count += 1
                    self.last_ready_scan_stamp = scan_stamp
            else:
                self.ready_since = None
                self.ready_count = 0
                self.last_ready_scan_stamp = None
            if (ready and self.ready_count >= self.stable_samples
                    and now-self.ready_since >= self.stable_duration and self._route()):
                self.phase = 'TO_VIA'
                self._command(0, 0)
                self._report('one-shot GPS route fixed')
                return
            # Align only to a measured station normal. Never sweep blindly through 360°.
            turn = (self._turn(heading, yaw)
                    if heading is not None and abs(wrap(heading-yaw)) > self.yaw_tol
                    else 0.0)
            self._command(0, turn)
            self._report(f'{why}; {normal_why}; stable={self.ready_count}')
            return
        if self.phase == 'TO_VIA':
            self.follower.goal_tolerance = self.via_tol
            step = self.follower.step(self.first_plan, xy, yaw,
                                      self.odom.twist.twist.angular.z,
                                      thrust_ratio=self.approach_cmd,
                                      turn_ratio=self.max_turn)
            if step.reached or np.linalg.norm(xy-self.route_enu[1]) <= self.via_tol:
                self.phase = 'ALIGN_VIA'
                self.follower.reset()
                self._command(0, 0)
            else:
                self._command(step.linear, step.angular)
            self._report(f'via error {np.linalg.norm(xy-self.route_enu[1]):.2f}m')
            return
        if self.phase == 'ALIGN_VIA':
            error = abs(wrap(self.final_heading-yaw))
            if error <= self.yaw_tol:
                self.phase = 'TO_GOAL'
                self._command(0, 0)
            else:
                self._command(0, self._turn(self.final_heading, yaw))
            self._report(f'via heading error {math.degrees(error):.1f}deg')
            return
        if self.phase == 'TO_GOAL':
            self.follower.goal_tolerance = self.goal_tol
            step = self.follower.step(self.final_plan, xy, yaw,
                                      self.odom.twist.twist.angular.z,
                                      thrust_ratio=self.final_cmd,
                                      turn_ratio=self.max_turn)
            if step.reached or np.linalg.norm(xy-self.route_enu[2]) <= self.goal_tol:
                self.phase = 'COMPLETE'
                self._command(0, 0)
                if self.mission_mode and not self.complete_sent:
                    self.complete_pub.publish(String(data='dock'))
                    self.complete_sent = True
            else:
                self._command(step.linear, step.angular)
            self._report(f'goal error {np.linalg.norm(xy-self.route_enu[2]):.2f}m')
            return
        # Keep the fixed GPS goal under small wave / residual-thrust drift.
        goal_error = self.route_enu[2] - xy
        distance = float(np.linalg.norm(goal_error))
        direction = np.array([math.cos(self.final_heading), math.sin(self.final_heading)])
        along = float(goal_error @ direction)
        lateral = float(direction[0]*goal_error[1]-direction[1]*goal_error[0])
        speed = self.odom.twist.twist.linear.x
        if distance < .12 and abs(speed) < .03:
            self._command(0, 0)
        else:
            heading = self.final_heading + float(np.clip(.5*lateral, -.2, .2))
            thrust = float(np.clip(.08*along-.12*speed, -.035, .035))
            self._command(thrust, self._turn(heading, yaw))
        self._report(f'holding fixed GPS goal, error {distance:.2f}m')


def main(args=None):
    rclpy.init(args=args)
    node = DockCtrlV2()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._command(0, 0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
