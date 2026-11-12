"""Real LiDAR avoidance test using the mission's DRI, B-spline planner and FSM.

The node consumes the existing /scan -> /occupancy_grid pipeline and /odom.
It previews a short goal straight ahead of the boat until /start_mission is true
or /real_avoid_test/start is called. Motor output is disabled by default; when enabled it
publishes directly to /cmd_vel, so no other /cmd_vel source may run alongside it.
"""

import copy
import dataclasses
import math
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PointStamped, PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker, MarkerArray

from kaboat_behaviors.avoid_fsm import AvoidFsm, FsmParams, follow_cmd
from kaboat_behaviors.behavior_base import yaw_from_quaternion
from kaboat_behaviors.bspline_planner import (
    PlannerParams, check, generate, generate_docking,
)
from kaboat_behaviors.dri import DriParams, build_dri
from kaboat_behaviors.dri_marker import clear_dri_x_markers, dri_x_markers
from kaboat_behaviors.los_controller import (
    BSPLINE_TRACK_YAW_KD, BSPLINE_TRACK_YAW_KP,
)
from kaboat_hardware.tank_walls import build_tank_dri, edge_distance, valid_bounds
from kaboat_hardware.trajectory_visualization import (
    BoundedTrajectoryHistory, create_trail_marker,
)


class RealObstacleAvoidTest(Node):
    """Standalone tank test; it does not depend on mission_manager or cmd_mux."""

    def __init__(self):
        super().__init__('real_obstacle_avoid_test')

        self.declare_parameter('drive_enabled', False)
        self.declare_parameter('wait_for_start', True)
        # 경로를 5초 넘게 못 만들거나 위험이 해소되지 않으면 후진 탈출(ESCAPE).
        # false 면 그때 정지(해제)한다. 후진 중에도 수조 경계 정지는 그대로 동작.
        self.declare_parameter('allow_escape', True)
        self.declare_parameter('final_goal_topic', '')
        self.declare_parameter('goal_topic', '')
        self.declare_parameter('dock_waypoints_topic', '')
        self.declare_parameter('dock_final_max_linear_cmd', .10)
        self.declare_parameter('goal_timeout_sec', 0.7)
        self.declare_parameter('goal_max_shift_m', 0.5)
        self.declare_parameter('require_goal_updates_when_running',True)
        self.declare_parameter('goal_distance', 3.0)
        # 고정 목표(odom = 수조 좌표). 켜면 목표는 선수 앞 goal_distance 가 아니라
        # fixed_goal_xy 이고, 배는 어디서든 선수가 목표를 향해(start_heading_tolerance_deg)
        # 있으면 출발한다. start_tolerance > 0 이면 fixed_start_xy 근처에서만 출발하도록
        # 위치 검사를 켠다(기본 0 = 끔). heading 허용 >= 180 이면 방향 검사도 끈다.
        self.declare_parameter('fixed_goal_enabled', False)
        self.declare_parameter('fixed_goal_xy', [0.0, 0.0])
        self.declare_parameter('fixed_start_xy', [0.0, 0.0])
        self.declare_parameter('start_tolerance', 0.0)
        self.declare_parameter('start_heading_tolerance_deg', 45.0)
        self.declare_parameter('goal_tolerance', 0.5)
        self.declare_parameter('max_cross_track_error', 0.7)
        # 출력 상한: 전진 0.25 (= fsm.cmd_full), 회전 0.31 (2026-11-06 실물 튜닝)
        self.declare_parameter('max_linear_cmd', 0.25)
        self.declare_parameter('max_angular_cmd', 0.31)
        self.declare_parameter('odom_timeout_sec', 0.5)
        self.declare_parameter('scan_timeout_sec', 0.5)
        self.declare_parameter('grid_timeout_sec', 0.5)
        self.declare_parameter('require_front_obstacle', True)
        self.declare_parameter('front_corridor_half_width', 1.0)
        self.declare_parameter('viz_dri_scale', 4.0)
        # DRI 원값이 이 문턱을 넘는 셀마다 셀 중심에 빨간 X (RViz 표시 전용).
        self.declare_parameter('viz_dri_mark_threshold', 1.116)   # = planner 후보 필터 문턱(1.2 × 0.93)
        self.declare_parameter('viz_dri_mark_size', 0.15)   # 셀(0.2 m) 안에 들어가는 크기
        # 수조 시험: /odom 이 수조 좌표(좌하단 원점)이므로 경계를 알 수 있다.
        # tank_bounds=(x_min, y_min, x_max, y_max). 기본값(전부 0)은 꺼짐 = 야외 동작.
        #  - exclude_tank_walls: 경계 띠(tank_wall_band) 안·밖의 점유 셀을 '벽'으로 보고
        #    장애물에서 뺀다. 버리지는 않고 벽만의 DRI 를 tank_wall_dri 상한으로 눌러 합친다
        #    (0 이면 완전히 지움). 상한은 플래너 후보 필터 문턱보다 낮아야 경로를 막지 않는다.
        #  - tank_stop_margin: 선체 중심이 경계에서 이 거리 안이면 정지(해제)한다.
        self.declare_parameter('tank_bounds', [0.0, 0.0, 0.0, 0.0])
        self.declare_parameter('exclude_tank_walls', False)
        self.declare_parameter('tank_wall_band', 0.5)
        self.declare_parameter('tank_wall_dri', 0.5)
        self.declare_parameter('tank_stop_margin', 0.5)

        self.drive_enabled = bool(self.get_parameter('drive_enabled').value)
        self.wait_for_start = bool(self.get_parameter('wait_for_start').value)
        self.allow_escape = bool(self.get_parameter('allow_escape').value)
        final_goal_topic = str(self.get_parameter('final_goal_topic').value)
        self.final_goal_pub = (self.create_publisher(
            PointStamped, final_goal_topic, QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
            if final_goal_topic else None)
        self.goal_topic = str(self.get_parameter('goal_topic').value)
        self.require_goal_updates=bool(self.get_parameter('require_goal_updates_when_running').value)
        self.dock_waypoints_topic=str(self.get_parameter('dock_waypoints_topic').value)
        self.dock_final_cmd=float(self.get_parameter('dock_final_max_linear_cmd').value)
        if not math.isfinite(self.dock_final_cmd) or not 0<self.dock_final_cmd<=1:
            raise ValueError('invalid docking final command limit')
        self.dock_waypoints=None;self.dock_waypoints_received=None;self.latched_dock_waypoints=None
        self.dock_plan_reason=''
        if self.dock_waypoints_topic:
            self.create_subscription(Path,self.dock_waypoints_topic,self._on_dock_waypoints,10)
        self.goal_timeout = float(self.get_parameter('goal_timeout_sec').value)
        self.goal_max_shift = float(self.get_parameter('goal_max_shift_m').value)
        if not all(math.isfinite(v) and v > 0 for v in (self.goal_timeout, self.goal_max_shift)):
            raise ValueError('external goal freshness and shift limits must be positive')
        self.external_goal = None
        self.external_goal_received = None
        self.external_goal_error = None
        self.observed_goal_on_arm=None
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
        self.viz_dri_mark_threshold = float(
            self.get_parameter('viz_dri_mark_threshold').value)
        self.viz_dri_mark_size = float(
            self.get_parameter('viz_dri_mark_size').value)
        self.tank_bounds = tuple(
            float(v) for v in self.get_parameter('tank_bounds').value)
        self.tank_enabled = valid_bounds(self.tank_bounds)
        self.exclude_tank_walls = bool(
            self.get_parameter('exclude_tank_walls').value)
        self.tank_wall_band = float(self.get_parameter('tank_wall_band').value)
        self.tank_wall_dri = float(self.get_parameter('tank_wall_dri').value)
        self.tank_stop_margin = float(
            self.get_parameter('tank_stop_margin').value)
        if self.exclude_tank_walls and not self.tank_enabled:
            raise ValueError('exclude_tank_walls requires a valid tank_bounds '
                             '(x_min, y_min, x_max, y_max)')
        if self.tank_enabled:
            width = self.tank_bounds[2] - self.tank_bounds[0]
            height = self.tank_bounds[3] - self.tank_bounds[1]
            if not (math.isfinite(self.tank_wall_band) and self.tank_wall_band > 0.0
                    and math.isfinite(self.tank_wall_dri) and self.tank_wall_dri >= 0.0
                    and math.isfinite(self.tank_stop_margin)
                    and 0.0 <= self.tank_stop_margin < 0.5 * min(width, height)):
                raise ValueError(
                    'tank_wall_band must be positive, tank_wall_dri >= 0 and '
                    'tank_stop_margin must be in [0, half of the smaller tank side)')

        self.fixed_goal_enabled = bool(
            self.get_parameter('fixed_goal_enabled').value)
        if self.goal_topic and self.fixed_goal_enabled:
            raise ValueError('goal_topic and fixed_goal_enabled are mutually exclusive')
        self.fixed_goal_xy = tuple(
            float(v) for v in self.get_parameter('fixed_goal_xy').value)
        self.fixed_start_xy = tuple(
            float(v) for v in self.get_parameter('fixed_start_xy').value)
        self.start_tolerance = float(self.get_parameter('start_tolerance').value)
        self.start_heading_tolerance = math.radians(float(
            self.get_parameter('start_heading_tolerance_deg').value))
        if self.fixed_goal_enabled:
            if (len(self.fixed_goal_xy) != 2 or len(self.fixed_start_xy) != 2
                    or not all(math.isfinite(v) for v in
                               self.fixed_goal_xy + self.fixed_start_xy)
                    or not math.isfinite(self.start_tolerance)
                    or not math.isfinite(self.start_heading_tolerance)):
                raise ValueError('fixed_goal_xy and fixed_start_xy must be finite '
                                 '[x, y] pairs')
            if self.tank_enabled:
                points = [('fixed_goal_xy', self.fixed_goal_xy)]
                if self.start_tolerance > 0.0:     # 시작점은 위치 검사를 켤 때만 쓴다
                    points.append(('fixed_start_xy', self.fixed_start_xy))
                for label, point in points:
                    edge = edge_distance(point[0], point[1], self.tank_bounds)
                    if edge < self.tank_stop_margin:
                        raise ValueError(
                            f'{label} {point} is {edge:.2f} m from the tank '
                            f'boundary (stop margin {self.tank_stop_margin:.2f} m)')

        for name, value in (
            ('goal_distance', self.goal_distance),
            ('goal_tolerance', self.goal_tolerance),
            ('max_cross_track_error', self.max_cross_track_error),
            ('odom_timeout_sec', self.odom_timeout),
            ('scan_timeout_sec', self.scan_timeout),
            ('grid_timeout_sec', self.grid_timeout),
            ('viz_dri_scale', self.viz_dri_scale),
            ('viz_dri_mark_threshold', self.viz_dri_mark_threshold),
            ('viz_dri_mark_size', self.viz_dri_mark_size),
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
        # 경로 추종 게인은 test_los 와 같은 값 (kp 1.4, kd 0.16)
        self.fsm = AvoidFsm(self.planner_params, self.fsm_params,
                            yaw_kp=BSPLINE_TRACK_YAW_KP,
                            yaw_kd=BSPLINE_TRACK_YAW_KD,generator=self._generate_plan)
        gen_threshold = self.planner_params.safe_threshold * self.planner_params.gen_margin
        if (self.tank_enabled and self.exclude_tank_walls
                and self.tank_wall_dri >= gen_threshold):
            raise ValueError(
                f'tank_wall_dri ({self.tank_wall_dri}) must be below the planner '
                f'candidate filter threshold ({gen_threshold:.2f}); otherwise walls '
                'block paths again')

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
        self.dri_marks_pub = self.create_publisher(
            MarkerArray, '/real_avoid_test/dri_marks', 1)
        self.status_pub = self.create_publisher(
            String, '/real_avoid_test/status', 10)

        if self.goal_topic:
            self.create_subscription(PointStamped, self.goal_topic, self._on_goal, qos_profile_sensor_data)
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
        if self.goal_topic:
            goal_description = f'live marker goal from {self.goal_topic}; latched on start'
        elif self.fixed_goal_enabled:
            goal_description = f'fixed ({self.fixed_goal_xy[0]:.2f}, {self.fixed_goal_xy[1]:.2f})'
        else:
            goal_description = f'{self.goal_distance:.1f} m ahead of the start pose'
        self.get_logger().info(
            f'Real avoidance test ready ({mode}). Goal: {goal_description}. '
            'Waiting for /scan, /odom and /occupancy_grid.')
        if self.tank_enabled:
            self.get_logger().info(
                f'Tank bounds {self.tank_bounds}: walls_excluded='
                f'{self.exclude_tank_walls} (band {self.tank_wall_band:.2f} m, '
                f'wall DRI cap {self.tank_wall_dri:.2f}), '
                f'stop margin {self.tank_stop_margin:.2f} m')

    def _params_from(self, prefix, cls):
        defaults = cls()
        values = {}
        for field in dataclasses.fields(cls):
            name = f'{prefix}.{field.name}'
            self.declare_parameter(name, getattr(defaults, field.name))
            values[field.name] = self.get_parameter(name).value
        return cls(**values)

    def _tank_edge(self, x, y):
        """(x, y) 에서 수조 경계까지의 안쪽 거리 [m]. 수조 설정이 없으면 None."""
        if not self.tank_enabled:
            return None
        return edge_distance(x, y, self.tank_bounds)

    def _build_dri(self, boat, yaw):
        """이 틱의 DRI. 수조 벽 제외가 켜져 있으면 벽 셀은 상한 tank_wall_dri 로 누른다."""
        if self.exclude_tank_walls and self.tank_enabled:
            return build_tank_dri(self.grid, boat, yaw, self.dri_params,
                                  self.tank_bounds, self.tank_wall_band,
                                  self.tank_wall_dri)
        return build_dri(self.grid, boat, yaw, self.dri_params)

    @staticmethod
    def _goal_stamp(msg):
        return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

    def _on_goal(self, msg):
        p = msg.point
        expected = self.odom.header.frame_id if self.odom is not None else 'odom'
        if (msg.header.frame_id != (expected or 'odom')
                or not all(math.isfinite(v) for v in (p.x, p.y, p.z)) or abs(p.z) > 1e-6):
            self.external_goal = self.external_goal_received = None
            self.external_goal_error = 'invalid marker goal frame or coordinates'
            return
        age = self.get_clock().now().nanoseconds * 1e-9 - self._goal_stamp(msg)
        if self._goal_stamp(msg) <= 0 or not -0.1 <= age <= self.goal_timeout:
            return
        if self.external_goal is not None and self._goal_stamp(msg) <= self._goal_stamp(self.external_goal):
            return
        self.external_goal = msg
        self.external_goal_received = time.monotonic()
        self.external_goal_error = None

    def _on_dock_waypoints(self,msg):
        valid=(msg.header.frame_id=='odom' and len(msg.poses)==3)
        if valid:
            for pose in msg.poses:
                p,q=pose.pose.position,pose.pose.orientation
                valid=valid and pose.header.frame_id in ('','odom') and abs(p.z)<1e-6
                valid=valid and all(math.isfinite(v) for v in (p.x,p.y,p.z,q.x,q.y,q.z,q.w))
                valid=valid and abs(q.x)<1e-6 and abs(q.y)<1e-6 and abs(q.z*q.z+q.w*q.w-1)<1e-3
        if not valid:
            self.dock_waypoints=self.dock_waypoints_received=None
            return
        age=self.get_clock().now().nanoseconds*1e-9-self._goal_stamp(msg)
        if self._goal_stamp(msg)<=0 or not -.1<=age<=self.goal_timeout:return
        if self.dock_waypoints is not None and self._goal_stamp(msg)<=self._goal_stamp(self.dock_waypoints):return
        xy=np.array([[p.pose.position.x,p.pose.position.y] for p in msg.poses])
        heading=yaw_from_quaternion(msg.poses[-1].pose.orientation)
        unit=np.array([math.cos(heading),math.sin(heading)]);steps=np.diff(xy,axis=0)
        if np.any(steps@unit<=.05) or np.any(abs(steps[:,0]*unit[1]-steps[:,1]*unit[0])>.02):
            self.dock_waypoints=self.dock_waypoints_received=None
            return
        self.dock_waypoints=msg;self.dock_waypoints_received=time.monotonic()

    def _generate_plan(self,dri,boat,yaw,vel,goal,params,now_t=0.):
        if not self.dock_waypoints_topic:return generate(dri,boat,yaw,vel,goal,params,now_t)
        msg=self.latched_dock_waypoints if self.started else self.dock_waypoints
        if msg is None:return None
        xy=np.array([[p.pose.position.x,p.pose.position.y] for p in msg.poses])
        heading=yaw_from_quaternion(msg.poses[-1].pose.orientation)
        plan=generate_docking(dri,boat,yaw,vel,xy,heading,params,now_t)
        self.dock_plan_reason=''
        if plan is None:
            limit=params.safe_threshold*params.gen_margin
            risks=dri.risk_at_many(xy[:,0],xy[:,1])
            boat_risk=dri.risk_at(float(boat[0]),float(boat[1]))
            if boat_risk>params.safe_threshold:
                self.dock_plan_reason=f'START_OCCUPIED risk={boat_risk:.2f}'
            elif risks[0]>limit:
                self.dock_plan_reason=f'APPROACH_OCCUPIED risk={risks[0]:.2f}'
            elif risks[-1]>limit:
                self.dock_plan_reason=f'STOP_OCCUPIED risk={risks[-1]:.2f}'
            else:
                tail=np.linspace(xy[0],xy[-1],max(3,int(np.linalg.norm(xy[-1]-xy[0])/(.5*dri.resolution))+1))
                tail_risk=float(np.max(dri.risk_at_many(tail[:,0],tail[:,1])))
                self.dock_plan_reason=(f'FINAL_CORRIDOR_BLOCKED risk={tail_risk:.2f}' if tail_risk>limit
                                       else 'NO_SAFE_CURVED_APPROACH')
        if plan is not None and self.tank_enabled:
            if any(self._tank_edge(float(x),float(y))<self.tank_stop_margin for x,y in plan.samples):
                self.dock_plan_reason="PATH_NEAR_TANK_BOUNDARY"
                return None
        return plan

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

    def _input_error(self, now, *, require_goal=True):
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
        if require_goal and self.goal_topic and not self.finished and (not self.started or self.require_goal_updates):
            if self.external_goal is None or self.external_goal_received is None:
                return self.external_goal_error or f'waiting for marker goal {self.goal_topic}'
            age = self.get_clock().now().nanoseconds * 1e-9 - self._goal_stamp(self.external_goal)
            if now - self.external_goal_received > self.goal_timeout or not -0.1 <= age <= self.goal_timeout:
                return 'marker goal stale'
            if self.external_goal.header.frame_id != (self.odom.header.frame_id or 'odom'):
                return 'marker goal and odom frame IDs differ'
            if self.started and self.goal is not None:
                p = self.external_goal.point
                reference=self.observed_goal_on_arm if self.observed_goal_on_arm is not None else self.goal
                if math.hypot(p.x-reference[0],p.y-reference[1])>self.goal_max_shift:
                    return 'marker moved away from the latched goal; reset and reacquire'
        if require_goal and self.dock_waypoints_topic and not self.finished and (not self.started or self.require_goal_updates):
            msg=self.dock_waypoints
            if msg is None or self.dock_waypoints_received is None:return 'waiting for docking waypoints'
            age=self.get_clock().now().nanoseconds*1e-9-self._goal_stamp(msg)
            if now-self.dock_waypoints_received>self.goal_timeout or not -.1<=age<=self.goal_timeout:
                return 'docking waypoints stale'
            if self.external_goal is None:return 'waiting for docking stop goal'
            p=msg.poses[-1].pose.position;g=self.external_goal.point
            if math.hypot(p.x-g.x,p.y-g.y)>.02 or abs(self._goal_stamp(msg)-self._goal_stamp(self.external_goal))>1e-5:
                return 'docking waypoints and stop goal are not synchronized'
            if self.started and self.latched_dock_waypoints is not None:
                old=self.latched_dock_waypoints
                a=yaw_from_quaternion(msg.poses[-1].pose.orientation);b=yaw_from_quaternion(old.poses[-1].pose.orientation)
                if abs(math.atan2(math.sin(a-b),math.cos(a-b)))>math.radians(15):return 'station normal changed; reset and reacquire'
                for current,latched in zip(msg.poses,old.poses):
                    p,q=current.pose.position,latched.pose.position
                    if math.hypot(p.x-q.x,p.y-q.y)>self.goal_max_shift:return 'docking waypoints shifted; reset and reacquire'
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

    def _target_goal(self, boat, yaw):
        """출발 전 미리보기용 목표 — 고정 목표가 켜져 있으면 그 좌표."""
        if self.goal_topic:
            p = self.external_goal.point
            return (p.x, p.y)
        if self.fixed_goal_enabled:
            return self.fixed_goal_xy
        return self._relative_goal(boat, yaw)

    def _front_obstacle_distance(self, occ_xy, boat, yaw, goal=None):
        """Require an obstacle on the test lane, not just a side wall.

        차선은 상대 목표일 때 선수 방향 goal_distance, 고정 목표일 때 배→목표 방향
        목표까지의 거리다(폭은 front_corridor_half_width).
        """
        if len(occ_xy) == 0:
            return None
        lane_yaw, length = yaw, self.goal_distance
        if (self.fixed_goal_enabled or self.goal_topic) and goal is not None:
            dx, dy = goal[0] - boat[0], goal[1] - boat[1]
            length = math.hypot(dx, dy)
            if length < 1e-6:
                return None
            lane_yaw = math.atan2(dy, dx)
        delta = occ_xy - np.asarray(boat)
        c, s = math.cos(lane_yaw), math.sin(lane_yaw)
        forward = delta[:, 0] * c + delta[:, 1] * s
        lateral = -delta[:, 0] * s + delta[:, 1] * c
        on_lane = ((forward > 0.0) & (forward <= length)
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
        if self.tank_enabled:
            pos = self.odom.pose.pose.position
            edge = self._tank_edge(pos.x, pos.y)
            if edge < self.tank_stop_margin:
                return False, (f'boat is {edge:.2f} m from the tank boundary '
                               f'(stop margin {self.tank_stop_margin:.2f} m)')
            if self.preview_goal is not None:
                goal_edge = self._tank_edge(*self.preview_goal)
                if goal_edge < self.tank_stop_margin:
                    return False, (f'goal is {goal_edge:.2f} m from the tank '
                                   f'boundary (stop margin '
                                   f'{self.tank_stop_margin:.2f} m)')
        if self.fixed_goal_enabled or self.goal_topic:
            pos = self.odom.pose.pose.position
            if self.fixed_goal_enabled and self.start_tolerance > 0.0:
                off = math.hypot(pos.x - self.fixed_start_xy[0],
                                 pos.y - self.fixed_start_xy[1])
                if off > self.start_tolerance:
                    return False, (
                        f'boat is {off:.2f} m from the start point '
                        f'({self.fixed_start_xy[0]:.2f}, {self.fixed_start_xy[1]:.2f}); '
                        f'allowed {self.start_tolerance:.2f} m')
            if self.start_heading_tolerance < math.pi:
                yaw = yaw_from_quaternion(self.odom.pose.pose.orientation)
                heading_goal = self._target_goal((pos.x, pos.y), yaw)
                bearing = math.atan2(heading_goal[1] - pos.y, heading_goal[0] - pos.x)
                err = abs(math.atan2(math.sin(bearing - yaw),
                                     math.cos(bearing - yaw)))
                if err > self.start_heading_tolerance:
                    return False, (
                        f'boat heading is {math.degrees(err):.0f} deg off the goal '
                        f'direction; allowed '
                        f'{math.degrees(self.start_heading_tolerance):.0f} deg')
        if (self.preview_plan is None or self.preview_goal is None
                or self.preview_received is None
                or now - self.preview_received > 0.3):
            return False, 'wait for a valid preview B-spline path'
        if (self.require_front_obstacle
                and self.preview_obstacle_distance is None):
            return False, 'no confirmed obstacle ahead within goal distance'
        if self.goal_topic:
            pos=self.odom.pose.pose.position
            observed=self._target_goal((pos.x,pos.y),yaw_from_quaternion(self.odom.pose.pose.orientation))
            if math.hypot(observed[0]-self.preview_goal[0],observed[1]-self.preview_goal[1])>self.goal_max_shift:
                return False, 'marker goal changed; wait for a new preview path'
        if self.dock_waypoints_topic and self.tank_enabled:
            if any(self._tank_edge(p.pose.position.x,p.pose.position.y)<self.tank_stop_margin for p in self.dock_waypoints.poses):
                return False,'docking waypoint violates tank boundary'
        self.observed_goal_on_arm=((self.external_goal.point.x,self.external_goal.point.y) if self.goal_topic else None)
        self.goal = self.preview_goal
        self.latched_dock_waypoints=copy.deepcopy(self.dock_waypoints) if self.dock_waypoints_topic else None
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
        self.latched_dock_waypoints=None
        self.observed_goal_on_arm=None
        self._publish_command(Twist())

    @staticmethod
    def _is_final_segment(plan):
        return (plan is not None and (plan.docking_phase=='FINAL'
                or (plan.tail_samples is not None and plan.progress_idx>=len(plan.samples)-len(plan.tail_samples)-1)))

    def _preview_follow_command(self,plan,boat,yaw,yaw_rate,goal):
        cmd=Twist()
        if plan is None:return cmd
        linear,angular=follow_cmd(plan,boat,yaw,yaw_rate,goal,self.fsm_params,
                                 BSPLINE_TRACK_YAW_KP,BSPLINE_TRACK_YAW_KD,
                                 pivot_angle=self.fsm_params.pivot_angle)   # FSM 과 같은 제자리 정렬
        if not math.isfinite(linear) or not math.isfinite(angular):raise ValueError('nonfinite preview controller output')
        limit=min(self.max_linear_cmd,self.dock_final_cmd) if self._is_final_segment(plan) else self.max_linear_cmd
        cmd.linear.x=float(np.clip(linear,-limit,limit));cmd.angular.z=float(np.clip(angular,-self.max_angular_cmd,self.max_angular_cmd))
        return cmd

    def _publish_command(self, cmd, preview=None):
        self.preview_cmd_pub.publish(cmd if preview is None else preview)
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
        self.dri_marks_pub.publish(clear_dri_x_markers(path.header.frame_id))

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
            # The risk map is sensor data, so keep it available while the
            # docking target is missing. Never label stale sensor data as fresh.
            if self._input_error(now,require_goal=False) is None:
                try:
                    boat,yaw,_,_=self._boat_state()
                    self._publish_dri(self._build_dri(boat,yaw),
                                      self.grid.header.frame_id or 'odom',self.odom.header.stamp)
                except (ValueError,ArithmeticError) as exc:
                    self.get_logger().warn(f'DRI visualization unavailable: {exc}',throttle_duration_sec=2.)
            self._publish_status('WAIT', reason)
            return
        if self.finished:
            self._publish_command(Twist())
            self._publish_status('FINISHED', 'goal tolerance reached')
            return

        boat, yaw, vel, yaw_rate = self._boat_state()
        # 벽을 DRI 에서 뺀 대신, 선체 위치(/odom)가 수조 경계에 가까워지면 정지한다.
        if self.started and self.tank_enabled:
            edge = self._tank_edge(*boat)
            if edge < self.tank_stop_margin:
                self._disarm(f'near tank boundary ({edge:.2f} m < '
                             f'{self.tank_stop_margin:.2f} m)')
                self._publish_status(
                    'STOP', f'tank boundary {edge:.2f} m < '
                            f'{self.tank_stop_margin:.2f} m')
                return
        goal = self.goal if self.started else self._target_goal(boat, yaw)
        try:
            dri = self._build_dri(boat, yaw)
            now_t = (self.odom.header.stamp.sec
                     + self.odom.header.stamp.nanosec * 1e-9)
            if now_t <= 0.0:
                now_t = now

            if not self.started:
                plan = self._generate_plan(dri, boat, yaw, vel, goal,
                                self.planner_params, now_t)
                if plan is not None and check(
                        plan, dri, boat, self.planner_params) is not None:
                    plan = None
                self.preview_plan = plan
                self.preview_goal = goal
                self.preview_received = now
                self.preview_obstacle_distance = self._front_obstacle_distance(
                    dri.occ_xy, boat, yaw, goal)
                start_text = ''
                if not self.wait_for_start:
                    # 즉시 출발 모드에서 출발이 막힌 이유를 상태·로그에 남긴다
                    # (전엔 버려져서 배가 왜 안 움직이는지 알 수 없었다).
                    if plan is None:
                        why = 'no valid preview B-spline path (NO PLAN)'
                    else:
                        ok, why = self._arm()
                        if ok:
                            return
                    start_text = f', start_blocked={why}'
                    self.get_logger().warn(f'auto start blocked: {why}',
                                           throttle_duration_sec=2.0)
                preview=(self._preview_follow_command(plan,boat,yaw,yaw_rate,goal)
                         if self.dock_waypoints_topic else None)
                self._publish_command(Twist(),preview=preview)
                self._publish_debug(dri, plan, goal)
                tank_text = ''
                if self.tank_enabled:
                    tank_text = (f', tank_edge={self._tank_edge(*boat):.2f}'
                                 f', walls_excluded={self.exclude_tank_walls}')
                phase=plan.docking_phase if plan is not None else '-'
                self._publish_status(
                    'PREVIEW' if plan is not None else 'NO PLAN',
                    f'obstacles={len(dri.occ_xy)}, '
                    f'front={self.preview_obstacle_distance}, '
                    f'goal=({goal[0]:.2f}, {goal[1]:.2f}), '
                    f'drive_enabled={self.drive_enabled}{tank_text}{start_text}, '
                    f'dock_phase={phase}, blocked={self.dock_plan_reason}')
                return

            aligned=True
            if self.dock_waypoints_topic and self.latched_dock_waypoints is not None and not getattr(self,'approach_only',False):
                terminal=yaw_from_quaternion(self.latched_dock_waypoints.poses[-1].pose.orientation)
                aligned=abs(math.atan2(math.sin(yaw-terminal),math.cos(yaw-terminal)))<=math.radians(15)
            if aligned and math.hypot(goal[0] - boat[0], goal[1] - boat[1]) <= self.goal_tolerance:
                self.started = False
                self.finished = True
                self._publish_command(Twist())
                self._publish_debug(dri, self.fsm.plan, goal)
                self._publish_status('FINISHED', 'goal tolerance reached')
                return

            out = self.fsm.step(dri, boat, yaw, vel, yaw_rate, goal, now_t)
            if out.state != getattr(self, '_last_fsm_state', 'FOLLOW'):
                # 상태 전환(ESCAPE 진입·복귀)은 거르지 않고 이유와 함께 — 1 s throttle 에 묻혀
                # ESCAPE 진입이 로그에 안 남은 적이 있다 (2026-11-12 영상 분석)
                detail = self.fsm.escape_detail if out.state == 'ESCAPE' else ''
                log = self.get_logger().warn if out.state == 'ESCAPE' else self.get_logger().info
                log(f'[{out.state}] {out.event or ""}' + (f' — {detail}' if detail else ''))
            elif out.event:
                self.get_logger().info(f'[{out.state}] {out.event}',
                                       throttle_duration_sec=1.0)
            self._last_fsm_state = out.state
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
            final_limit=(min(self.max_linear_cmd,self.dock_final_cmd)
                         if self._is_final_segment(out.plan) else self.max_linear_cmd)
            cmd.linear.x = float(np.clip(out.linear_x,
                                         -final_limit,
                                         final_limit))
            cmd.angular.z = float(np.clip(out.angular_z,
                                          -self.max_angular_cmd,
                                          self.max_angular_cmd))
            self._publish_command(cmd)
            self._publish_debug(dri, out.plan, goal)
            self._publish_status(out.state, out.event or 'following')
        except (ValueError, ArithmeticError) as exc:
            self._disarm(f'planner error: {exc}')
            self._publish_status('ERROR', str(exc))

    def _publish_dri(self,dri,frame,stamp):
        if self.dri_pub.get_subscription_count() > 0:
            grid = OccupancyGrid()
            grid.header = self.grid.header
            grid.info = self.grid.info
            values = np.clip(dri.data / self.viz_dri_scale * 100.0,
                             0.0, 100.0)
            grid.data = np.rint(values).astype(np.int8).ravel().tolist()
            self.dri_pub.publish(grid)

        if self.dri_marks_pub.get_subscription_count() > 0:
            self.dri_marks_pub.publish(dri_x_markers(
                dri, self.viz_dri_mark_threshold, self.viz_dri_mark_size,
                frame, stamp))


    def _publish_debug(self, dri, plan, goal):
        stamp = self.odom.header.stamp
        frame = self.grid.header.frame_id or 'odom'
        if self.final_goal_pub is not None:
            final_goal = PointStamped()
            final_goal.header.stamp, final_goal.header.frame_id = stamp, frame
            final_goal.point.x, final_goal.point.y = goal
            self.final_goal_pub.publish(final_goal)
        path = Path()
        path.header.stamp = stamp
        path.header.frame_id = frame
        if plan is not None:
            for index,(x, y) in enumerate(plan.samples):
                point = PoseStamped()
                point.header = path.header
                point.pose.position.x = float(x)
                point.pose.position.y = float(y)
                point.pose.position.z = 0.0
                tangent=plan.samples[min(index+1,len(plan.samples)-1)]-plan.samples[max(0,index-1)]
                heading=math.atan2(float(tangent[1]),float(tangent[0]))
                if index==len(plan.samples)-1 and plan.terminal_heading is not None:heading=plan.terminal_heading
                point.pose.orientation.z=math.sin(heading/2)
                point.pose.orientation.w=math.cos(heading/2)
                path.poses.append(point)
        self.path_pub.publish(path)

        self._publish_dri(dri,frame,stamp)

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
