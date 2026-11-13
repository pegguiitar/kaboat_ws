"""Single-plan docking with straight LOS segments.

순서 (2026-11-13):
  SCAN    제자리에서 YOLO 로 목표 도형만 찾는다 (세 슬롯 중 어디로 갈지 = 목표 도형의 화면 방위).
          선수를 돌려 목표를 화면 가운데(±center_tolerance_deg)에 두고, 회전이 멎은 채 연속
          bearing_samples 프레임의 세계 방위가 bearing_spread_deg 안이면 그 방위를 고정.
  SEEK    고정한 방위의 직선을 seek_cmd 로 서행(LOS). 라이다 스캔면이 수면 위 ~0.14 m 라 슬롯
          벽은 약 6 m 안에서만 잡힌다(2026-11-12 bag). 최근 cloud_window_s 의 /scan/level 을 odom 에
          쌓아 고정 치수(dock_slot.yaml: 폭·깊이) ㄷ자 슬롯을 찾는다 — 목표 방위선에서
          slot_target_offset_m 안의 슬롯만 (YOLO 는 세 슬롯 중 어디인지만 정한다). 같은 ㄷ자가
          lock_min_span_s 이상 같은 자리·법선이면 진입 기하 고정. 직선 하나(RANSAC)로 벽을 정하면
          뒷벽과 평행한 칸막이 끝면(1.94 m 앞)을 잡던 문제를 ㄷ자 전체로 피한다(dock_u_slot.py).
          앞 front_stop_m 안에 점이 있거나 seek_max_travel_m 를 가면 멈춰 기다린다.
  BACKOFF 고정했을 때 이미 경유점보다 벽에 가까우면 선수 유지 후진으로 거리를 벌린다.
  TO_VIA → ALIGN_VIA → TO_GOAL  경유점(입구 밖 pre_entry_offset) → 슬롯 중심선과 평행하게 선수
          정렬 → 정지점(뒷벽 앞 berth_distance). 치수는 모두 dock_slot.yaml (slot_config).
  REVERSE → COMPLETE  선수 유지 후진 retreat_distance_m → /mission/complete=dock, 이탈점 유지.

위치 출처 position_source (기본 odom, 2026-11-12): /odom = GQ7 RTK 필터 위치(25 Hz, 선체 중심).
gps 는 /gps/fix 원시(2 Hz, 좌현 안테나) 최근 5개 중앙값 — 시뮬레이션 원래 방식, 1~2 s 늦다.
marker_order (기본 '' = 순서 안 봄): 화면 왼쪽부터의 도형 순서를 알면 'square,triangle,circle' 처럼.
주면 같이 보이는 마커들의 좌우 순서가 어긋난 프레임(가장자리 오검출)을 목표 방위에서 뺀다.
"""
import json
import math
import os
import time
from collections import deque

import numpy as np
import rclpy
import yaml
from geometry_msgs.msg import Point, PoseStamped, Twist
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data as sensor_qos
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import LaserScan, NavSatFix
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from kaboat_behaviors.behavior_base import yaw_from_quaternion
from kaboat_behaviors.dock_u_slot import USlotParams, find_u_slot
from kaboat_behaviors.los_controller import LosFollower, sampled_path
from kaboat_behaviors.slot_axis import scan_points
from kaboat_behaviors.slot_dimensions import SlotDimensions
from .dock_one_shot import (LocalGeo, fixed_route, parse_marker_order, retreat_error,
                            retreat_route, stable_bearing, stable_wall_lock,
                            straight_samples, target_sight, wall_distance, wrap)

FRONT_MIN_RANGE_M = 0.8      # 이보다 가까운 점은 선체·물보라로 보고 앞 막힘 판정에서 뺀다
FRONT_MIN_POINTS = 3


def stamp(msg):
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


def front_clearance(scan, half_angle, stop_distance):
    """스캔 앞쪽(±half_angle) stop_distance 안 점이 FRONT_MIN_POINTS 개 이상이면 그 최소 거리, 아니면 None."""
    ranges = np.asarray(scan.ranges, float)
    angles = scan.angle_min + scan.angle_increment * np.arange(len(ranges))
    angles = np.arctan2(np.sin(angles), np.cos(angles))
    near = (np.isfinite(ranges) & (ranges >= max(scan.range_min, FRONT_MIN_RANGE_M))
            & (ranges < stop_distance) & (np.abs(angles) <= half_angle))
    if int(near.sum()) < FRONT_MIN_POINTS:
        return None
    return float(ranges[near].min())


def default_slot_config():
    """kaboat_bringup/config/dock_slot.yaml (슬롯·선체 치수를 정하는 한 곳)."""
    from ament_index_python.packages import get_package_share_directory
    return os.path.join(get_package_share_directory('kaboat_bringup'), 'config', 'dock_slot.yaml')


def load_slot_dimensions(path):
    """dock_slot.yaml → SlotDimensions (치수 검사 포함)."""
    with open(path, encoding='utf-8') as f:
        dock = yaml.safe_load(f)['docking_ctrl']['ros__parameters']['dock']
    return SlotDimensions(
        wall_to_mouth=float(dock['wall_to_mouth_m']), slot_width=float(dock['slot_width']),
        slot_width_tolerance=float(dock['slot_width_tolerance']),
        hull_width=float(dock['hull_width']), hull_length=float(dock['hull_length']),
        berth_distance=float(dock['berth_distance']),
        pre_entry_offset=float(dock['pre_entry_offset']),
        exit_clearance=float(dock['exit_clearance_m']))


def transform_xy(points, transform):
    """(N, 2) 스캔 평면 점 → TF(geometry_msgs/Transform) 적용 후 XY."""
    q, t = transform.rotation, transform.translation
    xyz = np.c_[points, np.zeros(len(points))]
    a = np.array([q.x, q.y, q.z])
    rotated = xyz + 2 * np.cross(a, np.cross(a, xyz) + q.w * xyz)
    return rotated[:, :2] + [t.x, t.y]


class DockCtrlV2(Node):
    def __init__(self, **node_kwargs):
        super().__init__('dock_ctrl_v2', **node_kwargs)
        defaults = {'drive_enabled': False, 'manual_handoff_enabled': False,
                    'mission_mode': False,
                    'stage_latitude': -999.0,
                    'stage_longitude': -999.0, 'sim_stage_odom_xy': [-999.0, -999.0],
                    'stage_radius_m': 3.0, 'gps_timeout_s': 1.5,
                    'odom_timeout_s': 1.0, 'perception_timeout_s': 1.5,
                    'stable_samples': 5,
                    'max_turn_cmd': 0.20,
                    'approach_cmd': 0.10, 'final_cmd': 0.05,
                    'retreat_cmd': 0.05, 'retreat_distance_m': 3.25,
                    'via_tolerance_m': 0.30, 'goal_tolerance_m': 0.25,
                    'yaw_tolerance_deg': 8.0, 'target_class': 'triangle',
                    'position_source': 'odom', 'marker_order': '',
                    # SCAN: 목표 도형 방위 고정
                    'min_confidence': 0.55, 'center_tolerance_deg': 6.0,
                    'bearing_samples': 5, 'bearing_spread_deg': 3.0,
                    'lock_yaw_rate_deg': 4.0,
                    # SEEK: 방위 따라 서행, 라이다 벽 고정
                    'seek_cmd': 0.07, 'seek_max_travel_m': 10.0,
                    'front_stop_m': 2.0, 'front_half_angle_deg': 25.0,
                    'scan_topic': '/scan',
                    # ㄷ자 슬롯 (치수는 slot_config = dock_slot.yaml, '' 면 kaboat_bringup 것)
                    'slot_config': '', 'slot_scan_topic': '/scan/level',
                    'cloud_window_s': 1.0, 'slot_target_offset_m': 1.6,
                    'slot_max_range_m': 12.0,
                    'lock_window_s': 2.5, 'lock_min_span_s': 1.0,
                    # 9 m 밖 ㄷ자는 좌우 0.3 m 이상 틀린 게 17 % (2026-11-12 bag) — 이 안에서만 고정
                    'lock_max_distance_m': 8.0,
                    'lock_goal_spread_m': 0.3, 'lock_normal_spread_deg': 5.0,
                    # BACKOFF
                    'via_margin_m': 0.5, 'backoff_max_m': 3.0}
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
        self.max_turn = float(p('max_turn_cmd'))
        self.approach_cmd = float(p('approach_cmd'))
        self.final_cmd = float(p('final_cmd'))
        self.retreat_cmd = float(p('retreat_cmd'))
        self.retreat_distance = float(p('retreat_distance_m'))
        self.via_tol = float(p('via_tolerance_m'))
        self.goal_tol = float(p('goal_tolerance_m'))
        self.yaw_tol = math.radians(float(p('yaw_tolerance_deg')))
        self.target = str(p('target_class'))
        self.position_source = str(p('position_source')).strip().lower()
        if self.position_source not in ('odom', 'gps'):
            raise ValueError("position_source must be 'odom' or 'gps'")
        self.marker_order = parse_marker_order(p('marker_order'))
        if self.target not in ('triangle', 'circle', 'square'):
            raise ValueError('invalid target_class')
        self.min_conf = float(p('min_confidence'))
        self.center_tol = math.radians(float(p('center_tolerance_deg')))
        self.bearing_n = int(p('bearing_samples'))
        self.bearing_spread = float(p('bearing_spread_deg'))
        self.lock_rate = math.radians(float(p('lock_yaw_rate_deg')))
        self.seek_cmd = float(p('seek_cmd'))
        self.seek_max_travel = float(p('seek_max_travel_m'))
        self.front_stop = float(p('front_stop_m'))
        self.front_half_angle = math.radians(float(p('front_half_angle_deg')))
        self.cloud_window = float(p('cloud_window_s'))
        self.lock_window = float(p('lock_window_s'))
        self.lock_min_span = float(p('lock_min_span_s'))
        self.lock_goal_spread = float(p('lock_goal_spread_m'))
        self.lock_normal_spread = float(p('lock_normal_spread_deg'))
        self.lock_max_distance = float(p('lock_max_distance_m'))
        slot_config = str(p('slot_config')).strip() or default_slot_config()
        self.dims = load_slot_dimensions(slot_config)
        self.slot_params = USlotParams(slot_width=self.dims.slot_width,
                                       depth=self.dims.wall_to_mouth,
                                       target_offset=float(p('slot_target_offset_m')),
                                       max_range=float(p('slot_max_range_m')))
        # 정지점: 뒷벽 앞 berth_distance (선체 중심), 경유점: 입구 밖 pre_entry_offset
        self.stop_distance = self.dims.berth_distance
        self.via_distance = self.dims.wall_to_mouth + self.dims.pre_entry_offset
        self.via_margin = float(p('via_margin_m'))
        self.backoff_max = float(p('backoff_max_m'))
        if not all(math.isfinite(v) and v > 0 for v in
                   (self.stage_radius, self.gps_timeout, self.odom_timeout,
                    self.perception_timeout,
                    self.max_turn, self.approach_cmd, self.final_cmd,
                    self.retreat_cmd, self.via_tol, self.goal_tol, self.yaw_tol,
                    self.min_conf, self.center_tol, self.bearing_spread, self.lock_rate,
                    self.seek_cmd, self.seek_max_travel, self.front_stop,
                    self.front_half_angle, self.cloud_window, self.lock_window,
                    self.lock_goal_spread, self.lock_normal_spread, self.lock_max_distance,
                    self.via_margin, self.backoff_max)
                   ) or not math.isfinite(self.retreat_distance) or self.retreat_distance <= 3.0 \
                or self.stable_samples < 2 or self.bearing_n < 2 \
                or not 0 <= self.lock_min_span < self.lock_window:
            raise ValueError('invalid docking controller limits')
        if not self.mission_mode and self.stage_ll[0] == -999 and self.sim_stage[0] == -999:
            self.get_logger().warn('No staging waypoint: set stage_latitude/longitude before start')
        self.geo = None
        self.gps = None
        self.gps_received = 0.0
        self.gps_history = deque(maxlen=5)
        self.odom = None
        self.odom_received = 0.0
        self.det = self.scan = None
        self.det_received = self.scan_received = 0.0
        self.det_seq = self.last_det_seq = 0
        self.bearing_samples = deque(maxlen=max(self.bearing_n, 2))
        self.pending_scans = deque(maxlen=30)       # TF 를 기다리는 /scan/level
        self.cloud = deque()                        # (stamp_s, odom XY 점들), 최근 cloud_window_s
        self.cloud_version = self.detected_version = 0
        self.slot_samples = deque(maxlen=max(self.stable_samples, 2) * 8)
        self.slot_received = 0.0
        self.slot_last = None                       # (USlot 또는 None, 이유)
        self.locked_bearing = None
        self.seek_start = self.seek_start_odom = self.seek_plan = None
        self.lock = None
        self.backoff_start = self.backoff_heading = None
        self.phase = 'IDLE'
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
        self.path_markers_pub = self.create_publisher(MarkerArray, '/dock/one_shot_markers', latched)
        self.slot_markers_pub = self.create_publisher(MarkerArray, '/dock/slot_markers', 1)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_subscription(NavSatFix, '/gps/fix', self._gps, sensor_qos)
        self.create_subscription(Odometry, '/odom', self._odom, sensor_qos)
        self.create_subscription(LaserScan, str(p('scan_topic')), self._scan, sensor_qos)
        self.create_subscription(LaserScan, str(p('slot_scan_topic')), self._slot_scan, sensor_qos)
        self.create_subscription(String, '/dock/detection_status', self._det, 10)
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

    def _scan(self, msg):
        self.scan, self.scan_received = msg, time.monotonic()

    def _det(self, msg):
        try:
            self.det = json.loads(msg.data)
            self.det_received = time.monotonic()
            self.det_seq += 1
        except ValueError:
            self.det = None

    def _slot_scan(self, msg):
        if msg.header.frame_id and stamp(msg) > 0:
            self.pending_scans.append(msg)

    def _add_cloud(self, stamp_s, points):
        """odom XY 점들을 누적 (최근 cloud_window_s 만 남김)."""
        if self.cloud and stamp_s <= self.cloud[-1][0]:
            return
        self.cloud.append((float(stamp_s), np.asarray(points, float).reshape(-1, 2)))
        while self.cloud and self.cloud[0][0] < stamp_s - self.cloud_window:
            self.cloud.popleft()
        self.cloud_version += 1

    def _update_cloud(self):
        """TF 가 준비된 스캔을 odom 점으로 바꿔 누적. 0.5 s 넘게 못 바꾼 스캔은 버린다."""
        now = self.get_clock().now().nanoseconds * 1e-9
        while self.pending_scans:
            scan = self.pending_scans[0]
            try:
                tf = self.tf_buffer.lookup_transform('odom', scan.header.frame_id,
                                                     Time.from_msg(scan.header.stamp)).transform
            except TransformException:
                if now - stamp(scan) > .5:
                    self.pending_scans.popleft()
                    continue
                return
            self.pending_scans.popleft()
            local = scan_points(scan.ranges, scan.angle_min, scan.angle_increment,
                                scan.range_min, scan.range_max)
            self._add_cloud(stamp(scan), transform_xy(local, tf))

    def _detect_slot(self):
        """새 누적 점이 있으면 목표 방위선 근처 ㄷ자를 찾아 샘플로 쌓는다."""
        if self.cloud_version == self.detected_version or not self.cloud:
            return
        self.detected_version = self.cloud_version
        points = np.vstack([pts for _, pts in self.cloud])
        direction = (math.cos(self.locked_bearing), math.sin(self.locked_bearing))
        slot, why = find_u_slot(points, self._odom_xy(), self.seek_start_odom, direction,
                                self.slot_params)
        self.slot_last = (slot, why)
        self._publish_slot_markers(slot)
        if slot is None:
            return
        if slot.wall_distance > self.lock_max_distance:
            self.slot_last = (None, f'U slot {slot.wall_distance:.1f}m away — closer than '
                                    f'{self.lock_max_distance:.1f}m to lock')
            return
        center, outward = np.asarray(slot.center), np.asarray(slot.outward)
        self.slot_samples.append((self.cloud[-1][0], center, outward,
                                  center + self.via_distance * outward,
                                  center + self.stop_distance * outward))
        self.slot_received = time.monotonic()

    def _publish_slot_markers(self, slot):
        marker = Marker()
        marker.header.frame_id = 'odom'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns, marker.id = 'dock_slot', 0
        if slot is None:
            marker.action = Marker.DELETE
        else:
            c, n = np.asarray(slot.center), np.asarray(slot.outward)
            t = np.array([-n[1], n[0]]) * self.dims.slot_width / 2
            depth = n * self.dims.wall_to_mouth
            marker.type, marker.action = Marker.LINE_STRIP, Marker.ADD
            marker.scale.x = .06
            marker.color.g = marker.color.a = 1.0
            marker.pose.orientation.w = 1.0
            marker.lifetime.nanosec = 500_000_000
            marker.points = [Point(x=float(x), y=float(y), z=.1)
                             for x, y in (c + t + depth, c + t, c - t, c - t + depth)]
        self.slot_markers_pub.publish(MarkerArray(markers=[marker]))

    def _gps_xy(self):
        return np.median(np.asarray(self.gps_history), axis=0)

    def _position(self):
        """제어에 쓰는 배 위치 — odom 이면 /odom 그대로, gps 면 LocalGeo 평면의 GPS 중앙값."""
        if self.position_source == 'odom':
            return self._odom_xy()
        return self._gps_xy()

    def _odom_xy(self):
        p = self.odom.pose.pose.position
        return np.array([p.x, p.y])

    def _stage(self):
        if self.stage_ll[0] != -999 and self.stage_ll[1] != -999:
            if self.geo is None or not self.gps_history:
                return None
            if self.position_source == 'odom':   # 위경도 → 지금 GPS 와의 차이로 odom 좌표
                return self._odom_xy() + self.geo.xy(*self.stage_ll) - self._gps_xy()
            return self.geo.xy(*self.stage_ll)
        if self.sim_stage_enu is not None:
            return self.sim_stage_enu
        if self.sim_stage[0] != -999 and self.odom is not None:
            return self._position() + self.sim_stage - self._odom_xy()
        return None

    def _inputs(self):
        now = time.monotonic()
        if self.position_source == 'gps' and (
                not self.gps_history or now - self.gps_received > self.gps_timeout):
            return 'GPS fix stale or missing'
        if (self.odom is None or now - self.odom_received > self.odom_timeout
                or self.odom.header.frame_id != 'odom'):
            return 'odom yaw stale or frame invalid'
        return None

    def _reset_search(self, phase):
        self.phase = phase
        self.complete_sent = False
        self.route_enu = None
        self.route_origin_odom = self.route_origin_enu = None
        self.fixed_path_msg = self.fixed_gps_msg = None
        self.first_plan = self.final_plan = None
        self.det = None
        self.det_received = 0.0
        self.last_det_seq = self.det_seq
        self.bearing_samples.clear()
        self.cloud.clear()
        self.pending_scans.clear()
        self.slot_samples.clear()
        self.slot_received = 0.0
        self.slot_last = None
        self.locked_bearing = None
        self.seek_start = self.seek_start_odom = self.seek_plan = None
        self.lock = None
        self.backoff_start = self.backoff_heading = None
        self.follower.reset()
        empty_path = Path()
        empty_path.header.frame_id = 'odom'
        empty_path.header.stamp = self.get_clock().now().to_msg()
        self.path_pub.publish(empty_path)
        clear = Marker()
        clear.action = Marker.DELETEALL
        self.path_markers_pub.publish(MarkerArray(markers=[clear]))
        self.gps_path_pub.publish(String(data=json.dumps({'one_shot': False, 'route': None})))

    def _on_mission_state(self, msg):
        old = self.mission_state
        self.mission_state = msg.data
        if msg.data == 'dock' and old != 'dock':
            # mission_approach reached the entry waypoint (three shapes in view):
            # pick the target slot by its YOLO bearing.
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
        lock = self.lock
        data = {'phase': self.phase, 'detail': detail, 'drive_enabled': self.drive,
                'manual_handoff_enabled': self.manual_handoff,
                'position_source': self.position_source,
                'target_class': self.target,
                'xy': (self._position().tolist()
                       if (self.odom if self.position_source == 'odom' else self.gps_history)
                       else None),
                'gps_xy': self._gps_xy().tolist() if self.gps_history else None,
                'locked_bearing_deg': (math.degrees(self.locked_bearing)
                                       if self.locked_bearing is not None else None),
                'slot': self._slot_report(),
                'slot_lock': ({'center_xy': lock['wall'].tolist(), 'outward': lock['outward'].tolist(),
                               'heading_deg': math.degrees(lock['heading'])} if lock else None),
                'route_fixed': self.route_enu is not None,
                'route_xy': [xy.tolist() for xy in self.route_enu] if self.route_enu else None}
        self.status.publish(String(data=json.dumps(data)))

    def _slot_report(self):
        if self.slot_last is None:
            return None
        slot, why = self.slot_last
        if slot is None:
            return {'found': False, 'reason': why}
        return {'found': True, 'center_xy': list(slot.center), 'outward': list(slot.outward),
                'wall_distance_m': slot.wall_distance,
                'coverage': {k: round(v, 3) for k, v in slot.coverage.items()}}

    def _yaw(self):
        return yaw_from_quaternion(self.odom.pose.pose.orientation)

    def _turn(self, heading, yaw):
        rate = self.odom.twist.twist.angular.z
        return float(np.clip(1.4 * wrap(heading - yaw) - .16 * rate,
                             -self.max_turn, self.max_turn))

    def _publish_route_markers(self, header):
        markers = MarkerArray()
        colors = ((.1, .8, 1.), (1., .7, 0.), (1., .1, .1), (.1, 1., .3))
        for index, (label, point) in enumerate(zip(('START', 'VIA', 'DOCK', 'EXIT'), self.route_enu)):
            x, y = point - self.route_origin_enu + self.route_origin_odom
            for text in (False, True):
                marker = Marker()
                marker.header = header
                marker.ns = 'dock_one_shot'
                marker.id = index * 2 + int(text)
                marker.action = Marker.ADD
                marker.type = Marker.TEXT_VIEW_FACING if text else Marker.SPHERE
                marker.pose.position.x, marker.pose.position.y = float(x), float(y)
                marker.pose.position.z = .55 if text else .25
                marker.pose.orientation.w = 1.0
                marker.scale.x = marker.scale.y = marker.scale.z = .20 if text else .25
                marker.color.r, marker.color.g, marker.color.b = colors[index]
                marker.color.a = 1.0
                if text:
                    marker.text = label
                markers.markers.append(marker)
        self.path_markers_pub.publish(markers)

    def _route(self, via_odom, stop_odom):
        """고정한 벽의 경유점·정지점(odom)으로 직선 두 구간 + 후진 구간을 한 번만 만든다."""
        origin_odom = self._odom_xy()
        origin_enu = self._position()
        via = origin_enu + np.asarray(via_odom, float) - origin_odom
        stop = origin_enu + np.asarray(stop_odom, float) - origin_odom
        try:
            first, second, heading = fixed_route(origin_enu, via, stop)
            retreat, exit_xy = retreat_route(stop, heading, self.retreat_distance)
        except ValueError:
            return False
        self.route_enu = (origin_enu.copy(), via, stop, exit_xy)
        self.route_origin_odom = origin_odom.copy()
        self.route_origin_enu = origin_enu.copy()
        self.first_plan = sampled_path(first, spacing=.1, curvatures=np.zeros(len(first)), frame_id='dock_enu')
        self.final_plan = sampled_path(second, spacing=.1, curvatures=np.zeros(len(second)), frame_id='dock_enu')
        self.final_heading = heading
        self.follower.reset()
        path = Path()
        path.header.frame_id = 'odom'
        path.header.stamp = self.get_clock().now().to_msg()
        for xy in np.vstack((first, second[1:], retreat[1:])):
            p = PoseStamped()
            p.header = path.header
            p.pose.position.x, p.pose.position.y = map(float, xy - origin_enu + origin_odom)
            p.pose.position.z = 0.18  # keep the RViz line above the grid and goal markers
            p.pose.orientation.w = 1.0
            path.poses.append(p)
        self.fixed_path_msg = path
        self.fixed_gps_msg = String(data=json.dumps(self._route_latlon()))
        self.path_pub.publish(self.fixed_path_msg)
        self._publish_route_markers(path.header)
        self.gps_path_pub.publish(self.fixed_gps_msg)
        self.last_path_pub = time.monotonic()
        return True

    def _route_latlon(self):
        """고정 경로 네 점의 위경도 (기록용). odom 위치면 지금 GPS 와의 차이로 바꾼다."""
        names = ('start', 'via', 'goal', 'exit')
        if self.geo is None or not self.gps_history:
            return {'one_shot': True, 'frame': 'odom',
                    **{n: list(map(float, xy)) for n, xy in zip(names, self.route_enu)}}
        shift = (self._gps_xy() - self._odom_xy() if self.position_source == 'odom'
                 else np.zeros(2))
        points = {}
        for name, xy in zip(names, self.route_enu):
            lat, lon = self.geo.latlon(np.asarray(xy) + shift)
            points[name] = dict(latitude=lat, longitude=lon)
        return {'one_shot': True, 'frame': 'WGS84', **points}

    # ── 목표 도형 방위 고정 / 라이다 벽 고정 ─────────────────────
    def _wall_lock(self):
        if not self.slot_samples or time.monotonic() - self.slot_received > self.perception_timeout:
            reason = self.slot_last[1] if self.slot_last else 'no scan cloud yet'
            return False, None, f'waiting for U slot ({reason})'
        return stable_wall_lock(self.slot_samples, min_samples=self.stable_samples,
                                window_s=self.lock_window, min_span_s=self.lock_min_span,
                                max_goal_spread_m=self.lock_goal_spread,
                                max_normal_spread_deg=self.lock_normal_spread)

    def _via_room(self, lock):
        """(지금 벽까지, 경유점 + 여유까지) 법선 방향 거리 [m]."""
        have = wall_distance(self._odom_xy(), lock['wall'], lock['outward'])
        need = wall_distance(lock['via'], lock['wall'], lock['outward']) + self.via_margin
        return have, need

    def _take_wall_lock(self, yaw):
        """목표 벽이 안정되면 경로를 고정(TO_VIA)하거나 너무 가까우면 BACKOFF. 바뀌었으면 True."""
        ok, lock, why = self._wall_lock()
        if not ok:
            return False, why
        self.lock = lock
        have, need = self._via_room(lock)
        if have < need:
            self.phase = 'BACKOFF'
            self.backoff_start = self._odom_xy()
            self.backoff_heading = yaw
            self._command(0, 0)
            self._report(f'U slot locked {have:.2f}m away; backing off to {need:.2f}m')
            return True, 'ok'
        if not self._route(lock['via'], lock['stop']):
            return False, 'U slot locked but route rejected'
        self.phase = 'TO_VIA'
        self._command(0, 0)
        self._report(f'U slot locked {have:.2f}m away; one-shot route fixed')
        return True, 'ok'

    def _start_seek(self, bearing):
        self.phase = 'SEEK'
        self.locked_bearing = bearing
        self.seek_start = self._position()
        self.seek_start_odom = self._odom_xy()
        self.cloud.clear()
        self.slot_samples.clear()
        end = self.seek_start + self.seek_max_travel * np.array([math.cos(bearing), math.sin(bearing)])
        samples = straight_samples(self.seek_start, end)
        self.seek_plan = sampled_path(samples, spacing=.1, curvatures=np.zeros(len(samples)),
                                      frame_id='dock_enu')
        self.follower.reset()

    def _scan_target(self, yaw):
        """SCAN 한 tick: 목표 도형을 화면 가운데로 돌리고, 멈춘 채 방위가 안정되면 SEEK."""
        if self.det is None or time.monotonic() - self.det_received > self.perception_timeout:
            self.bearing_samples.clear()
            return 0.0, 'YOLO stale'
        ok, bearing, why = target_sight(self.det, self.target, order=self.marker_order,
                                        min_confidence=self.min_conf)
        if not ok:
            self.bearing_samples.clear()
            return 0.0, why            # 목표가 안 보이면 멈춰 기다린다 (눈먼 360° 회전 안 함)
        world = wrap(yaw + bearing)
        if self.det_seq != self.last_det_seq:
            self.last_det_seq = self.det_seq
            still = abs(self.odom.twist.twist.angular.z) <= self.lock_rate
            if abs(bearing) <= self.center_tol and still:
                self.bearing_samples.append(world)
            else:
                self.bearing_samples.clear()
        locked, mean, why = stable_bearing(self.bearing_samples, min_samples=self.bearing_n,
                                           max_spread_deg=self.bearing_spread)
        if locked:
            self._start_seek(mean)
            return 0.0, f'{self.target} bearing locked {math.degrees(mean):.1f} deg'
        turn = self._turn(world, yaw) if abs(bearing) > .5 * self.center_tol else 0.0
        return turn, f'centering {self.target} ({math.degrees(bearing):+.1f} deg); {why}'

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
                self._reset_search('SCAN')
                self._command(0, 0)
            else:
                heading = math.atan2(stage[1]-xy[1], stage[0]-xy[0])
                turn = self._turn(heading, yaw)
                self._command(self.approach_cmd if abs(wrap(heading-yaw)) < .5 else 0, turn)
            self._report(f'staging distance {distance:.2f}m')
            return
        if self.phase == 'SCAN':
            turn, why = self._scan_target(yaw)
            self._command(0, turn)
            self._report(why)
            return
        if self.phase == 'SEEK':
            self._update_cloud()
            self._detect_slot()
            taken, wall_why = self._take_wall_lock(yaw)
            if taken:
                return
            travelled = float(np.linalg.norm(xy - self.seek_start))
            if self.scan is None or time.monotonic() - self.scan_received > self.perception_timeout:
                self._command(0, 0)
                self._report(f'scan stale — holding; {wall_why}')
                return
            blocked = front_clearance(self.scan, self.front_half_angle, self.front_stop)
            if blocked is not None:
                self._command(0, 0)
                self._report(f'something {blocked:.2f}m ahead — holding; {wall_why}')
                return
            if travelled >= self.seek_max_travel - self.via_tol:
                self._command(0, 0)
                self._report(f'no U slot within {self.seek_max_travel:.1f}m — holding; {wall_why}')
                return
            step = self.follower.step(self.seek_plan, xy, yaw,
                                      self.odom.twist.twist.angular.z,
                                      thrust_ratio=self.seek_cmd,
                                      turn_ratio=self.max_turn)
            self._command(step.linear, step.angular)
            self._report(f'seeking along {math.degrees(self.locked_bearing):.1f} deg, '
                         f'{travelled:.1f}m; {wall_why}')
            return
        if self.phase == 'BACKOFF':
            have, need = self._via_room(self.lock)
            backed = float(np.linalg.norm(self._odom_xy() - self.backoff_start))
            if have >= need:
                if self._route(self.lock['via'], self.lock['stop']):
                    self.phase = 'TO_VIA'
                    self._command(0, 0)
                    self._report(f'backed off to {have:.2f}m; one-shot route fixed')
                else:
                    self._command(0, 0)
                    self._report('backed off but route rejected — holding')
                return
            if backed >= self.backoff_max:
                self._command(0, 0)
                self._report(f'backed off {backed:.2f}m, still {have:.2f}/{need:.2f}m — holding')
                return
            self._command(-self.retreat_cmd, self._turn(self.backoff_heading, yaw))
            self._report(f'backing off {have:.2f}/{need:.2f}m from target wall')
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
                self.phase = 'REVERSE'
                self._command(-self.retreat_cmd, self._turn(self.final_heading, yaw))
            else:
                self._command(step.linear, step.angular)
            self._report(f'goal error {np.linalg.norm(xy-self.route_enu[2]):.2f}m')
            return
        if self.phase == 'REVERSE':
            progress, lateral = retreat_error(xy, self.route_enu[2], self.final_heading)
            if progress >= max(3.0, self.retreat_distance - .15) and abs(lateral) <= .75:
                self.phase = 'COMPLETE'
                self._command(0, 0)
                if self.mission_mode and not self.complete_sent:
                    self.complete_pub.publish(String(data='dock'))
                    self.complete_sent = True
            else:
                heading = self.final_heading + float(np.clip(.35*lateral, -.2, .2))
                linear = -self.retreat_cmd if abs(wrap(heading-yaw)) < .4 else 0.0
                self._command(linear, self._turn(heading, yaw))
            self._report(f'retreat {progress:.2f}/{self.retreat_distance:.2f}m, lateral {lateral:.2f}m')
            return
        # Hold the fixed exit point under small drift until the next mission takes over.
        goal_error = self.route_enu[3] - xy
        distance = float(np.linalg.norm(goal_error))
        direction = np.array([math.cos(self.final_heading), math.sin(self.final_heading)])
        along = float(goal_error @ direction)
        lateral = float(direction[0]*goal_error[1]-direction[1]*goal_error[0])
        speed = self.odom.twist.twist.linear.x
        if distance < .12 and abs(speed) < .03:
            self._command(0, 0)
        else:
            thrust = float(np.clip(.08*along-.12*speed, -.035, .035))
            heading = self.final_heading + math.copysign(1.0, thrust) * float(np.clip(.5*lateral, -.2, .2))
            self._command(thrust, self._turn(heading, yaw))
        self._report(f'holding fixed GPS exit, error {distance:.2f}m')


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
