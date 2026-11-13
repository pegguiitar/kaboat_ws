"""Single-plan docking with straight LOS segments.

순서 (2026-11-13):
  SCAN    제자리에서 YOLO 로 목표 도형만 찾는다 (세 슬롯 중 어디로 갈지 = 목표 도형의 화면 방위).
          세계 방위 = 촬영 시각(image_stamp_s, 없으면 받은 시각 - camera_latency_s)의 odom 선수각
          + 화면 방위. 최근 bearing_window_s 안에서 화면 가운데 ±sample_max_offset_deg,
          선회율 ±lock_yaw_rate_deg 인 프레임 bearing_samples 개의 세계 방위가 bearing_spread_deg
          안이면 그 방위를 고정. 그동안 선수는 세계 방위를 odom 으로 따라간다(scan_turn_kp/kd).
          2026-11-13 bag: 예전 방식(받은 시각의 선수각 + 화면 방위로 P 제어, ±3° 데드밴드, 가운데
          ±6°·4°/s 동시 조건)은 카메라 지연 0.21 s 때문에 ±18°, 25°/s 로 계속 흔들려 한 번도
          고정 못 했다 — 같은 기록에 지연 보상만 넣어도 세계 방위 표준편차 3.5° → 1.2°.
          목표가 안 보이면 search_turn_cmd 로 천천히 돌며 찾는다 (선회각 제한 없음, YOLO 를 믿음).
          방향 근거 (좋은 것 우선, 더 좋은 근거가 처음 생기면 한 번 다시 정하고 그 외엔 유지 — 돌면
          보이던 도형이 반대로 밀려나므로 매번 다시 정하면 좌우로 왔다 갔다 한다):
            다른 도형 2개 — 슬롯은 같은 간격이라 목표는 두 도형 화면 간격만큼 바깥 어느 한쪽.
                            화면 안 후보는 빼고(보였을 것) 가운데에 가까운 쪽으로 (가까우면 양쪽에 걸쳐 보여서
                            평균 쪽은 틀린다: 2026-11-13 시뮬 5.5 m, 반대로 28 s 돎).
            다른 도형 1개가 가운데 ±search_centre_deg 밖 — 그 도형 쪽 (목표는 그 너머).
            다른 도형 1개가 가운데 근처 — 방향 단서 없음: 선수 유지하고 retreat_cmd 로 후진해 시야를
                            넓힌다(최대 search_backoff_max_m, 뒤 1.5 m 안 장애물이면 중단).
                            그래도 모르면 그 도형 쪽, 정확히 가운데면 시계방향.
            아무것도 없음 — 시계방향.
          YOLO 상태 자체가 끊기면(stale) 멈춘다. 누적 search_max_turns(2)바퀴를 돌아도 목표가 없으면
          멈추고 상태에 'not found after 2 turns' 를 남긴다(목표가 보이면 다시 시작).
          화면 안/밖 판단의 반화각은 검출 상태의 image_half_fov_deg(좌우), 없으면 camera_half_fov_deg.
  SEEK    목표 도형을 선수 가운데(±seek_center_deg)에 둔 채 seek_cmd 로 서행 — 벗어나면 전진 0, 선수만
          돌린다(선회율 제약 없음). 보이는 동안 지연 보상한 YOLO 방위로 목표 방위·방위선을 갱신하고
          (한 번에 seek_max_update_deg 넘게 튀면 버림), 가까워져 안 보이면 마지막 방위로 간다.
          라이다 스캔면이 수면 위 ~0.14 m 라 슬롯
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
                            target_sight, wall_distance, wrap)

FRONT_MIN_RANGE_M = 0.8      # 이보다 가까운 점은 선체·물보라로 보고 앞 막힘 판정에서 뺀다
FRONT_MIN_POINTS = 3


def stamp(msg):
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


def front_clearance(scan, half_angle, stop_distance, centre=0.0):
    """스캔 centre 방향(기본 앞, math.pi 면 뒤) ±half_angle 안, stop_distance 안 점이
    FRONT_MIN_POINTS 개 이상이면 그 최소 거리, 아니면 None."""
    ranges = np.asarray(scan.ranges, float)
    angles = scan.angle_min + scan.angle_increment * np.arange(len(ranges)) - centre
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
                    'min_confidence': 0.55,
                    # 고정에 쓸 프레임: 화면 가운데 ±8°. 2026-11-13 bag(라이다 ㄷ자 중심 대비): ±8° 면
                    # 방위 오차 ≤0.6°, ±20° 면 ≤2.6° — 가장자리일수록 틀리고, 11/12 에는 화면 끝
                    # 오검출로 옆 슬롯을 고른 적이 있다. 선회율 제한은 기본 끔(0) — 촬영 시각으로
                    # 지연 보상하면 25°/s 로 돌아도 맞다. 0 보다 크면 그 선회율 [deg/s] 이하 프레임만.
                    'sample_max_offset_deg': 8.0, 'lock_yaw_rate_deg': 0.0,
                    'bearing_samples': 5, 'bearing_spread_deg': 3.0,
                    'bearing_window_s': 2.0,
                    # 검출 상태에 image_stamp_s 가 없을 때(예전 검출기) 쓰는 촬영→수신 지연
                    'camera_latency_s': 0.2,
                    # SCAN 제자리 선회 PD (세계 방위 - odom 선수각). 1.4/0.16 은 bag 에서 식별한
                    # 선회 모델(지연 0.2 s, 시정수 0.8 s)에서 덜 감쇠돼 0.8/0.4 로 낮춤.
                    'scan_turn_kp': 0.8, 'scan_turn_kd': 0.4,
                    # 목표 탐색 선회 명령 (식별 모델: 0.06 → 약 8°/s, 바람 편향 2.2°/s 보다 충분히 큼)
                    'search_turn_cmd': 0.06,
                    # 도형 1개가 이 안이면 방향 단서가 없다고 보고 후진해 시야를 넓힌다 (최대 거리)
                    'search_centre_deg': 10.0, 'search_backoff_max_m': 3.0,
                    # 검출 상태에 image_half_fov_deg 가 없을 때의 카메라 수평 반화각. D455 640x480 실측
                    # fx 377·cx 317 → 왼쪽 40.0°, 오른쪽 40.6° (2026-11-13 bag, 검출 광선 1272개 맞춤)
                    'camera_half_fov_deg': 40.0,
                    # 탐색 선회 누적 이만큼(바퀴) 돌아도 목표가 없으면 멈추고 상태에 남긴다
                    'search_max_turns': 2.0,
                    # SEEK: 방위 따라 서행, 라이다 벽 고정
                    'seek_cmd': 0.07, 'seek_max_travel_m': 10.0,
                    'seek_center_deg': 8.0, 'seek_max_update_deg': 10.0,
                    'front_stop_m': 2.0, 'front_half_angle_deg': 25.0,
                    'scan_topic': '/scan',
                    # ㄷ자 슬롯 (치수는 slot_config = dock_slot.yaml, '' 면 kaboat_bringup 것)
                    'slot_config': '', 'slot_scan_topic': '/scan/level',
                    'cloud_window_s': 1.0, 'slot_target_offset_m': 1.6,
                    'slot_max_range_m': 12.0,
                    'lock_window_s': 2.5, 'lock_min_span_s': 1.0,
                    # 9 m 밖 ㄷ자는 좌우 0.3 m 이상 틀린 게 17 % (2026-11-12 bag) — 이 안에서만 고정.
                    # 2026-11-13 bag 재생: 7.7 m 에서 고정하면 슬롯 중심이 법선 방향 0.89 m 틀렸고
                    # 6 m 안 추정은 모두 ±0.15 m (원래 5.05 m 고정과 1 cm 차) → 8 → 6 m.
                    'lock_max_distance_m': 6.0,
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
        self.sample_offset = math.radians(float(p('sample_max_offset_deg')))
        self.bearing_n = int(p('bearing_samples'))
        self.bearing_spread = float(p('bearing_spread_deg'))
        self.bearing_window = float(p('bearing_window_s'))
        self.lock_rate = math.radians(float(p('lock_yaw_rate_deg')))
        self.camera_latency = float(p('camera_latency_s'))
        self.scan_kp = float(p('scan_turn_kp'))
        self.scan_kd = float(p('scan_turn_kd'))
        self.search_turn = float(p('search_turn_cmd'))
        self.search_centre = float(p('search_centre_deg'))
        self.search_backoff_max = float(p('search_backoff_max_m'))
        self.camera_half_fov = float(p('camera_half_fov_deg'))
        self.search_max_turns = float(p('search_max_turns'))
        self.seek_cmd = float(p('seek_cmd'))
        self.seek_max_travel = float(p('seek_max_travel_m'))
        self.seek_center = math.radians(float(p('seek_center_deg')))
        self.seek_max_update = math.radians(float(p('seek_max_update_deg')))
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
                    self.min_conf, self.sample_offset, self.bearing_spread,
                    self.bearing_window, self.scan_kp, self.search_turn,
                    self.search_centre, self.search_backoff_max, self.camera_half_fov,
                    self.search_max_turns,
                    self.seek_cmd, self.seek_max_travel, self.front_stop,
                    self.seek_center, self.seek_max_update,
                    self.front_half_angle, self.cloud_window, self.lock_window,
                    self.lock_goal_spread, self.lock_normal_spread, self.lock_max_distance,
                    self.via_margin, self.backoff_max)
                   ) or not math.isfinite(self.retreat_distance) or self.retreat_distance <= 3.0 \
                or not (math.isfinite(self.camera_latency) and 0 <= self.camera_latency < 2) \
                or not (math.isfinite(self.scan_kd) and self.scan_kd >= 0) \
                or not (math.isfinite(self.lock_rate) and self.lock_rate >= 0) \
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
        self.det_stamp = None                       # 지금 검출의 촬영 시각 [s, ROS 시간]
        self.yaw_history = deque(maxlen=100)        # (odom 헤더 시각, 펼친 선수각) — 약 4 s
        self.bearing_samples = deque(maxlen=max(self.bearing_n, 2) * 4)   # (시각, 세계 방위)
        self.scan_heading = None                    # SCAN 이 따라가는 목표 세계 방위
        self.scan_heading_received = 0.0
        self.search_dir = None                      # 목표 탐색 선회 방향 (+1 왼쪽, -1 오른쪽)
        self.search_rank = 0                        # 방향 근거: 0 없음, 1 도형 1개, 2 도형 2개
        self.widen_start = self.widen_heading = None   # 시야 넓히기 후진 시작 위치·선수각
        self.widen_done = False
        self.search_turned = 0.0                    # 이번 탐색에서 누적 선회 [rad]
        self.search_prev_yaw = None
        self.search_exhausted = False               # search_max_turns 를 다 돌았다
        self.last_sight = None                      # (수신 monotonic, 세계 방위, 화면 방위) — RViz 용
        self.last_cmd = (0.0, 0.0)                  # 마지막 (전진, 선회) 명령 — RViz 글자용
        self.phase_log = deque(maxlen=6)            # (ROS 시각, 단계, 상세) — RViz 단계 전이 기록
        self.pending_scans = deque(maxlen=30)       # TF 를 기다리는 /scan/level
        self.cloud = deque()                        # (stamp_s, odom XY 점들), 최근 cloud_window_s
        self.cloud_version = self.detected_version = 0
        self.slot_samples = deque(maxlen=max(self.stable_samples, 2) * 8)
        self.slot_received = 0.0
        self.slot_last = None                       # (USlot 또는 None, 이유)
        self.locked_bearing = None
        self.seek_start = self.seek_start_odom = self.seek_plan = None
        self.seek_line_point = None                 # 목표 방위선이 지나는 점 (odom, 마지막 시선)
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
        # 단계·전이 기록·목표 시선·고정 방위선·SEEK 가운데 범위 (odom, RViz/bag 재생 확인용)
        self.ctrl_markers_pub = self.create_publisher(MarkerArray, '/dock/ctrl_markers', 1)
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
        yaw = yaw_from_quaternion(msg.pose.pose.orientation)
        t = stamp(msg)
        if self.yaw_history:
            last_t, last_yaw = self.yaw_history[-1]
            if t < last_t:              # 시간이 거꾸로(bag 다시 재생 등) — 이력 버림
                self.yaw_history.clear()
            else:
                yaw = last_yaw + wrap(yaw - last_yaw)
        self.yaw_history.append((t, yaw))

    def _yaw_at(self, t):
        """odom 이력에서 시각 t 의 선수각 (보간). t 가 이력보다 늦으면 마지막 값, 이르면 None."""
        if not self.yaw_history:
            return None
        times = [s for s, _ in self.yaw_history]
        if t is None or t >= times[-1]:
            return wrap(self.yaw_history[-1][1])
        if t < times[0]:
            return None
        return wrap(float(np.interp(t, times, [y for _, y in self.yaw_history])))

    def _scan(self, msg):
        self.scan, self.scan_received = msg, time.monotonic()

    def _det(self, msg):
        try:
            self.det = json.loads(msg.data)
            self.det_received = time.monotonic()
            self.det_seq += 1
        except ValueError:
            self.det = None
            return
        image_stamp = self.det.get('image_stamp_s') if isinstance(self.det, dict) else None
        try:
            image_stamp = float(image_stamp)
        except (TypeError, ValueError):
            image_stamp = None
        if image_stamp is None or not math.isfinite(image_stamp) or image_stamp <= 0:
            image_stamp = self.get_clock().now().nanoseconds * 1e-9 - self.camera_latency
        self.det_stamp = image_stamp

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
        slot, why = find_u_slot(points, self._odom_xy(), self.seek_line_point, direction,
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
        self.scan_heading = None
        self.scan_heading_received = 0.0
        self._clear_search()
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
        self.last_cmd = (msg.linear.x, msg.angular.z)
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
        if not self.phase_log or self.phase_log[-1][1] != self.phase:
            self.phase_log.append((self.get_clock().now().nanoseconds * 1e-9, self.phase, detail))
        self._publish_ctrl_markers(detail)

    PHASE_COLORS = {'IDLE': (.7, .7, .7), 'STAGE': (.8, .8, .3), 'SCAN': (1., .7, .1),
                    'SEEK': (.3, .6, 1.), 'BACKOFF': (1., .3, .3), 'TO_VIA': (.2, .9, .4),
                    'ALIGN_VIA': (.2, .9, .8), 'TO_GOAL': (.1, 1., .2), 'REVERSE': (.8, .4, 1.),
                    'COMPLETE': (1., 1., 1.)}

    def _publish_ctrl_markers(self, detail):
        """RViz: 단계 글자·전이 기록, 지연 보상한 목표 시선, SCAN 목표/고정 방위선, SEEK ±가운데."""
        if self.odom is None:
            return
        header_stamp = self.get_clock().now().to_msg()
        xy, yaw = self._odom_xy(), self._yaw()
        markers = []

        def marker(mid, kind, color, scale, z=.3, alpha=1.0):
            m = Marker()
            m.header.frame_id, m.header.stamp = 'odom', header_stamp
            m.ns, m.id, m.type, m.action = 'dock_ctrl', mid, kind, Marker.ADD
            m.pose.orientation.w = 1.0
            m.pose.position.z = z
            m.scale.x = m.scale.y = m.scale.z = scale
            m.color.r, m.color.g, m.color.b = color
            m.color.a = alpha
            m.lifetime.nanosec = 500_000_000
            markers.append(m)
            return m

        def ray(mid, origin, heading, length, color, width, z=.3):
            m = marker(mid, Marker.LINE_STRIP, color, width, z=0.0)
            end = np.asarray(origin) + length * np.array([math.cos(heading), math.sin(heading)])
            m.points = [Point(x=float(origin[0]), y=float(origin[1]), z=z),
                        Point(x=float(end[0]), y=float(end[1]), z=z)]
            return m

        color = self.PHASE_COLORS.get(self.phase, (1., 1., 1.))
        text = marker(0, Marker.TEXT_VIEW_FACING, color, .7, z=2.2)
        text.pose.position.x, text.pose.position.y = map(float, xy)
        linear, angular = self.last_cmd
        applied = '' if self.drive else ' — preview only (drive_enabled=false)'
        text.text = (f'{self.phase}\n{detail[:70]}\n'
                     f'cmd v={linear:+.2f} w={angular:+.2f}{applied}')
        log = marker(1, Marker.TEXT_VIEW_FACING, (1., 1., 1.), .45, z=4.0, alpha=.9)
        log.pose.position.x, log.pose.position.y = float(xy[0]) - 4.0, float(xy[1]) + 3.0
        log.text = '\n'.join(time.strftime('%H:%M:%S', time.localtime(t)) + f' {ph}'
                             for t, ph, _ in self.phase_log)
        now = time.monotonic()
        if self.last_sight and now - self.last_sight[0] <= self.perception_timeout:
            _, world, screen = self.last_sight          # 노랑 = 고정 표본 범위 밖, 초록 = 안
            usable = abs(screen) <= self.sample_offset
            ray(2, xy, world, 14.0, (.2, 1., .2) if usable else (1., 1., .2), .06, z=.35)
        if self.phase == 'SCAN' and self.scan_heading is not None:
            ray(3, xy, self.scan_heading, 12.0, (1., .55, 0.), .12, z=.25)
        if self.locked_bearing is not None and self.seek_line_point is not None:
            ray(4, self.seek_line_point, self.locked_bearing, self.seek_max_travel + 2.0,
                (1., .2, .1), .15, z=.2)
        if self.phase == 'SEEK':                        # 선수 ±seek_center_deg — 목표 방위가 안이면 전진
            cone = marker(5, Marker.LINE_LIST, (.6, .8, 1.), .04, z=0.0, alpha=.7)
            for side in (-1, 1):
                end = xy + 6.0 * np.array([math.cos(yaw + side * self.seek_center),
                                           math.sin(yaw + side * self.seek_center)])
                cone.points += [Point(x=float(xy[0]), y=float(xy[1]), z=.3),
                                Point(x=float(end[0]), y=float(end[1]), z=.3)]
        self.ctrl_markers_pub.publish(MarkerArray(markers=markers))

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
        self.seek_line_point = self.seek_start_odom.copy()
        self.cloud.clear()
        self.slot_samples.clear()
        self.follower.reset()

    def _target_sample(self):
        """새 YOLO 결과면 (ok, 화면 방위, 촬영 시각 세계 방위 또는 None, 이유). 아니면 ok=False."""
        ok, bearing, why = target_sight(self.det, self.target, order=self.marker_order,
                                        min_confidence=self.min_conf)
        if not ok or self.det_seq == self.last_det_seq:
            return ok, bearing, None, why
        self.last_det_seq = self.det_seq
        seen_yaw = self._yaw_at(self.det_stamp)
        if seen_yaw is None:
            return ok, bearing, None, 'no odom yaw at image time'
        world = wrap(seen_yaw + bearing)
        self.last_sight = (time.monotonic(), world, bearing)
        return ok, bearing, world, why

    def _seek_sight(self):
        """SEEK 중 목표가 보이면 목표 방위(EMA)·방위선을 갱신. 반환 설명."""
        if self.det is None or time.monotonic() - self.det_received > self.perception_timeout:
            return f'{self.target} not in view — keeping {math.degrees(self.locked_bearing):.1f} deg'
        ok, bearing, world, why = self._target_sample()
        if world is None:
            return why if not ok else f'{self.target} screen {math.degrees(bearing):+.1f} deg'
        jump = wrap(world - self.locked_bearing)
        if abs(bearing) > self.sample_offset or abs(jump) > self.seek_max_update:
            return f'{self.target} sight {math.degrees(world):.1f} deg ignored (jump {math.degrees(jump):+.1f})'
        self.locked_bearing = wrap(self.locked_bearing + .3 * jump)
        self.seek_line_point = self._odom_xy()
        return f'{self.target} screen {math.degrees(bearing):+.1f} deg'

    def _scan_turn(self, heading, yaw):
        """SCAN 제자리 선회 PD — 데드밴드 없음(가운데 근처에서도 선회율을 계속 잡는다)."""
        rate = self.odom.twist.twist.angular.z
        return float(np.clip(self.scan_kp * wrap(heading - yaw) - self.scan_kd * rate,
                             -self.max_turn, self.max_turn))

    def _scan_target(self, yaw):
        """SCAN 한 tick: 목표 도형의 세계 방위로 선수를 돌리며, 방위가 안정되면 SEEK.

        세계 방위는 촬영 시각의 선수각으로 만든다 — 받은 시각의 선수각을 쓰면 도는 동안
        (선회율 × 0.2 s) 만큼 밀려 제어가 흔들리고 방위 고정도 안 된다(2026-11-13 bag).
        """
        now = time.monotonic()
        if self.det is None or now - self.det_received > self.perception_timeout:
            self.bearing_samples.clear()
            self.scan_heading = None
            return 0.0, 0.0, 'YOLO stale'
        ok, bearing, world, why = self._target_sample()
        if world is not None:
            self._clear_search()                    # 목표를 봤다 — 다음 탐색은 새로 방향을 정한다
            self.scan_heading = world if self.scan_heading is None else math.atan2(
                math.sin(world) + math.sin(self.scan_heading),
                math.cos(world) + math.cos(self.scan_heading))
            self.scan_heading_received = now
            rate = abs(self.odom.twist.twist.angular.z)
            if abs(bearing) <= self.sample_offset and (self.lock_rate == 0 or rate <= self.lock_rate):
                self.bearing_samples.append((self.det_stamp, world))
        # 한두 프레임 놓쳐도 지우지 않는다 — 창(bearing_window_s) 밖으로 나간 것만 버린다
        while self.bearing_samples and (self.bearing_samples[0][0]
                                        < self.det_stamp - self.bearing_window):
            self.bearing_samples.popleft()
        locked, mean, lock_why = stable_bearing([w for _, w in self.bearing_samples],
                                                min_samples=self.bearing_n,
                                                max_spread_deg=self.bearing_spread)
        if locked:
            self._start_seek(mean)
            return 0.0, 0.0, f'{self.target} bearing locked {math.degrees(mean):.1f} deg'
        if self.scan_heading is None or now - self.scan_heading_received > self.perception_timeout:
            self.scan_heading = None
            return self._search(why, yaw)
        detail = (f'turning to {self.target} {math.degrees(self.scan_heading):.1f} deg'
                  + (f' (screen {math.degrees(bearing):+.1f} deg)' if ok else f' ({why})'))
        return 0.0, self._scan_turn(self.scan_heading, yaw), f'{detail}; {lock_why}'

    def _clear_search(self):
        self.search_dir = None
        self.search_rank = 0
        self.widen_start = self.widen_heading = None
        self.widen_done = False
        self.search_turned = 0.0
        self.search_prev_yaw = None
        self.search_exhausted = False

    def _half_fov(self, side):
        """화면 수평 반화각 [deg], side +1 왼쪽 / -1 오른쪽 — 검출 상태 값, 없으면 파라미터."""
        fov = self.det.get('image_half_fov_deg') if isinstance(self.det, dict) else None
        try:
            value = float(fov['left' if side > 0 else 'right'])
            if 5.0 <= value <= 89.0:
                return value
        except (KeyError, TypeError, ValueError):
            pass
        return self.camera_half_fov

    def _other_shapes(self):
        """지금 검출에서 목표가 아닌 신뢰 도형들의 화면 방위 [deg, + 왼쪽]."""
        rays = self.det.get('sight_rays', {}) if isinstance(self.det, dict) else {}
        seen = {}
        for name, ray in (rays or {}).items():
            try:
                if name != self.target and float(ray.get('confidence', 0)) >= self.min_conf:
                    value = float(ray['bearing_left_deg'])
                    if math.isfinite(value):
                        seen[name] = value
            except (AttributeError, KeyError, TypeError, ValueError):
                continue
        return seen

    def _search(self, why, yaw):
        """목표가 안 보일 때 (전진, 선회, 설명). 근거별 방향은 모듈 설명 참고."""
        seen = self._other_shapes()
        basis = ('other shapes ' + ', '.join(f'{k} {v:+.0f}' for k, v in sorted(seen.items()))
                 if seen else 'no shape in view')
        if self.search_exhausted:
            return 0.0, 0.0, (f'target {self.target} not found after {self.search_max_turns:g} turns '
                              f'— holding ({basis}; {why})')
        if len(seen) >= 2 and self.search_rank < 2:
            low, high = sorted(seen.values())[0], sorted(seen.values())[-1]
            gap = high - low                        # 이웃 도형 사이 화면 간격
            # 화면 안에 들어올 후보는 아니다(보였을 것) — 화면 밖 후보 중 가운데에 가까운 쪽
            left, right = high + gap, low - gap
            outside = [c for c in (left, right)
                       if abs(c) > self._half_fov(1 if c > 0 else -1) - 3.0] or [left, right]
            self.search_dir = 1 if min(outside, key=abs) == left else -1
            self.search_rank = 2
            self.widen_start = None
        elif len(seen) == 1 and self.search_rank < 1:
            (bearing,) = seen.values()
            if abs(bearing) >= self.search_centre or self.widen_done:
                self.search_dir = 1 if bearing > 0 else -1
                self.search_rank = 1
                self.widen_start = None
            else:
                widen = self._widen_view(yaw)
                if widen is not None:
                    self.search_prev_yaw = yaw      # 후진(선수 유지)은 탐색 선회로 안 센다
                    return (*widen[:2], f'{widen[2]} for {self.target} ({basis}; {why})')
                self.search_dir = 1 if bearing > 0 else -1
                self.search_rank = 1
        elif self.widen_start is not None and not seen:
            widen = self._widen_view(yaw)           # 후진 중 잠깐 놓쳐도 계속 넓힌다
            if widen is not None:
                self.search_prev_yaw = yaw
                return (*widen[:2], f'{widen[2]} for {self.target} ({basis}; {why})')
        if self.search_dir is None:
            self.search_dir = -1                    # 아무것도 안 보이면 시계방향부터
        if self.search_prev_yaw is not None:
            self.search_turned += abs(wrap(yaw - self.search_prev_yaw))
        self.search_prev_yaw = yaw
        turned = math.degrees(self.search_turned)
        if self.search_turned >= self.search_max_turns * 2 * math.pi:
            self.search_exhausted = True            # 목표가 보이면 _clear_search 로 다시 시작
            return 0.0, 0.0, (f'target {self.target} not found after {self.search_max_turns:g} turns '
                              f'— holding ({basis}; {why})')
        side = 'left' if self.search_dir > 0 else 'right (clockwise)'
        return 0.0, self.search_dir * self.search_turn, (
            f'searching {side} for {self.target} ({basis}; turned {turned:.0f}/'
            f'{360 * self.search_max_turns:.0f} deg; {why})')

    def _widen_view(self, yaw):
        """선수 유지 후진으로 시야를 넓힌다. 끝났으면(최대 거리·뒤 장애물) None."""
        if self.widen_done:
            return None
        if self.widen_start is None:
            self.widen_start, self.widen_heading = self._odom_xy(), yaw
        backed = float(np.linalg.norm(self._odom_xy() - self.widen_start))
        behind = (front_clearance(self.scan, self.front_half_angle, 1.5, centre=math.pi)
                  if self.scan is not None else None)
        if backed >= self.search_backoff_max or behind is not None:
            self.widen_done = True
            self.widen_start = None
            return None
        return (-self.retreat_cmd, self._scan_turn(self.widen_heading, yaw),
                f'widening view, backing {backed:.1f}/{self.search_backoff_max:.1f}m')

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
            linear, turn, why = self._scan_target(yaw)
            self._command(linear, turn)
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
            sight_why = self._seek_sight()
            error = wrap(self.locked_bearing - yaw)
            centred = abs(error) <= self.seek_center
            self._command(self.seek_cmd if centred else 0.0, self._scan_turn(self.locked_bearing, yaw))
            self._report(f'seeking {math.degrees(self.locked_bearing):.1f} deg '
                         f'({"centred" if centred else f"centring {math.degrees(error):+.1f} deg"}), '
                         f'{travelled:.1f}m; {sight_why}; {wall_why}')
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
