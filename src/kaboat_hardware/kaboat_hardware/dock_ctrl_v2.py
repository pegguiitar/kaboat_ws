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
            다른 도형 1개가 가운데 근처 — 도크에 아주 가까울 때(표지 간격 3.55 m, 반화각 40° →
                            뒷벽 약 4.2 m 안)만 생긴다. 치우친 반대쪽으로 돈다: 가려진 옆 도형 중
                            화면 가장자리에 더 가까운 쪽이라 가장 적게 돌아 단서가 나온다. 정확히 가운데면
                            시계방향. (예전 후진으로 시야 넓히기는 뺐다 — 라이다가 120~240° 를 가려 뒤를 못 봄.)
            정한 방향으로 돌아도 새 도형이 안 나오면 한 번 반대로 — 도형 1개로 정했으면
                            search_reverse_deg(75°), 도형 2개(간격 규칙)로 정했으면 예측 위치가 광축
                            가운데에 올 때까지(예측 각도, 최소 20°). 간격 규칙은 끝 슬롯에서 틀린다
                            (시뮬: 끝 슬롯 앞에서 반대로 330° 돎). 반전한 쪽은 비어 있다고 보고, 그 뒤 간격
                            규칙도 그쪽 후보를 뺀다.
            아무것도 없음 — 시계방향.
          YOLO 상태 자체가 끊기면(stale) 멈춘다. 누적 search_max_turns(2)바퀴를 돌아도 목표가 없으면
          상태에 'not found after 2 turns' 를 남기고(목표가 보이면 다시 시작) — 미션 모드면 SCAN 시작
          선수각(도크 쪽)으로 되돌린 뒤 /mission/give_up 'dock: …' 을 보낸다. 전체 실행이면 mission_manager 가
          시간 초과 때처럼 직진 후진 3 m 뒤 다음 미션으로 넘긴다(선수가 도크 쪽이라 후진은 도크에서
          멀어진다). 단일 시험·미션 모드 아님이면 그대로 멈춘다.
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
          앞 front_stop_m 안에 점이 있으면 멈춰 기다린다(아래 REPOSITION·UNBLOCK). 막힘 없이
          seek_max_travel_m 를 가도 못 잡으면 방위가 틀렸거나 낡았다고 보고 그 자리에서 SCAN 부터 다시
          (seek_rescan_max_attempts 번까지, 다 쓰면 멈춤).
  REPOSITION 비스듬하거나 앞이 reposition_after_s 막히면 보이는 ㄷ자를 slot_pitch_m 만큼 옮긴 임시 목표
          축 위 경유점으로 우회 (한 번만).
  UNBLOCK 앞이 reposition_after_s 막혔는데 우회를 못 짜거나(보이는 ㄷ자 없음 등) 이미 썼거나, 우회 전진이
          막히면: 선수 유지 직진 후진 unblock_distance_m (온 길을 되짚음 — 뒤는 라이다가 못 본다) 뒤
          SCAN 부터 다시. unblock_max_attempts 번 다 쓰면 멈춰 기다린다 (예전엔 처음부터 그대로 멈춤).
  BACKOFF 고정했을 때 이미 경유점보다 벽에 가까우면 선수 유지 후진으로 거리를 벌린다.
  TO_VIA → ALIGN_VIA → TO_GOAL  경유점(입구 밖 pre_entry_offset) → 슬롯 중심선과 평행하게 선수
          정렬 → 정지점(뒷벽 앞 berth_distance). 치수는 모두 dock_slot.yaml (slot_config).
  REVERSE → COMPLETE  선수 유지 후진 retreat_distance_m → /mission/complete=dock, 이탈점 유지.
  ABORT   (미션 모드만, 2026-11-14) 더 할 게 없거나 진전이 없으면 서서 미션 마감(240 s)을 기다리지 않고
          /mission/give_up 'dock: 이유' 를 1 s 마다 — 전체 실행이면 mission_manager 가 직진 후진 3 m 뒤 search.
          바로 포기: SEEK 재탐색·막힘 후진(UNBLOCK) 횟수 소진, BACKOFF 실패. 시간: SCAN 에서 YOLO 가
          give_up_yolo_stale_s(30 s) 끊김, TO_VIA give_up_to_via_s(60 s)·TO_GOAL give_up_to_goal_s(60 s) 안에
          도착 못 함(시뮬 정상 최대 9·37 s, 실패 172·61 s). TO_GOAL 이면 슬롯 안일 수 있어 슬롯 축 선수로
          give_up_reverse_m(3 m) 직진 후진(retreat_cmd) 뒤 포기. 단독 도킹 시험(미션 모드 아님)은 예전처럼 멈춤.

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
from dataclasses import replace

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
GIVE_UP_ALIGN_MAX_S = 20.0   # 목표 못 찾아 포기하기 전 SCAN 시작 선수각으로 되돌리는 상한 [s]


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


def load_slot_pitch(path, default=4.02):
    """dock_slot.yaml slot_pitch_m (이웃 슬롯 중심 간격). 없으면 default — 2026-11-12 라이다 실측 3.99/4.06."""
    with open(path, encoding='utf-8') as f:
        dock = yaml.safe_load(f)['docking_ctrl']['ros__parameters']['dock']
    return float(dock.get('slot_pitch_m', default))


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
                    # 후진 (REVERSE·BACKOFF·UNBLOCK·포기 전 정렬 뒤). 0.20 = 1400 µs = 텔레옵 후진 2단계
                    # (2026-11-14, 예전 0.05 = 1456 µs 는 후진 1단계 1450 µs 보다 약해 도크 쪽 물살
                    # 0.15 m/s 에서 슬롯을 못 빠져나왔다 — 폐루프 시뮬 scripts/e2e_sim)
                    'retreat_cmd': 0.20, 'retreat_distance_m': 3.25,
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
                    # 도형 1개로 정한 방향으로 이만큼 돌아도 새 도형이 없으면 반대로. 시뮬 격자 4070가지에서
                    # 맞는 쪽이면 새 도형이 최대 68°(p99 58°) 안에 보였다 → 75° (예전 90°)
                    'search_centre_deg': 10.0, 'search_reverse_deg': 75.0,
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
                    'via_margin_m': 0.5, 'backoff_max_m': 3.0,
                    # REPOSITION: SEEK 가 앞이 막힌 채 이만큼 ㄷ자를 못 잡으면 우회 (한 번만)
                    'reposition_after_s': 3.0,
                    # 임시 경유점: 임시 목표 ㄷ자 뒷벽에서 법선 방향 거리. ㄷ자 고정 거리(lock_max_distance_m 6)
                    # 안이고 BACKOFF 기준(경유점 3.44 + 여유 0.5 = 3.94 m) 밖이라 다시 잠그면 바로 TO_VIA.
                    'reposition_distance_m': 5.0, 'reposition_tolerance_m': 0.5,
                    # 보이는 ㄷ자 법선과 목표 방위가 이 각 이상이면 막히기 전에 바로 우회 — 목표 표지로 가는
                    # 시선이 칸막이 끝에 닿기 시작하는 각 atan(슬롯 반폭 1.485 / 깊이 1.94) ≈ 37°
                    'reposition_angle_deg': 37.0,
                    # UNBLOCK: 막혔는데 우회를 못 하면 선수 유지 직진 후진(retreat_cmd)으로 이만큼 온 길을
                    # 되짚고 SCAN 부터 다시 (unblock_max_s 넘으면 덜 갔어도 다시). 도킹 한 번에 최대
                    # unblock_max_attempts 번 — 다 쓰면 멈춰 기다린다 (전체 실행은 도킹 제한 240 s 가 넘김).
                    'unblock_distance_m': 2.0, 'unblock_max_s': 30.0,
                    'unblock_max_attempts': 3,
                    # SEEK 가 막힘 없이 seek_max_travel_m 를 가도 ㄷ자를 못 잡으면 그 자리에서 SCAN 부터 다시
                    # (목표 방위를 새로 잡음). 도킹 한 번에 최대 이만큼 — 다 쓰면 멈춰 기다린다.
                    'seek_rescan_max_attempts': 2,
                    # 포기 (미션 모드, 2026-11-14): 더 할 게 없거나 진전이 없으면 서서 마감(240 s)을 기다리지
                    # 않고 /mission/give_up — 전체 실행이면 mission_manager 가 직진 후진 3 m 뒤 search 로.
                    # 0 = 그 조건 끔. 재탐색·막힘 후진·BACKOFF 를 다 써서 멈추면 바로 포기.
                    # 폐루프 시뮬 정상 판 최대: TO_VIA 9 s·TO_GOAL 37 s (실패 판 172·61 s), YOLO 첫 결과 9 s(bag).
                    'give_up_yolo_stale_s': 30.0, 'give_up_to_via_s': 60.0,
                    'give_up_to_goal_s': 60.0,
                    # TO_GOAL 에서 포기하면 슬롯 안일 수 있어 먼저 선수 유지 직진 후진 (retreat_cmd) — 정지점은
                    # 경유점보다 2.44 m 안쪽이라 3 m 면 입구 밖 (mission_manager 후진 0.05 는 조류에 약함)
                    'give_up_reverse_m': 3.0}
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
        self.search_reverse = math.radians(float(p('search_reverse_deg')))
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
        self.slot_pitch = load_slot_pitch(slot_config)
        self.slot_params = USlotParams(slot_width=self.dims.slot_width,
                                       depth=self.dims.wall_to_mouth,
                                       target_offset=float(p('slot_target_offset_m')),
                                       max_range=float(p('slot_max_range_m')))
        # 정지점: 뒷벽 앞 berth_distance (선체 중심), 경유점: 입구 밖 pre_entry_offset
        self.stop_distance = self.dims.berth_distance
        self.via_distance = self.dims.wall_to_mouth + self.dims.pre_entry_offset
        self.via_margin = float(p('via_margin_m'))
        self.backoff_max = float(p('backoff_max_m'))
        self.reposition_after = float(p('reposition_after_s'))
        self.reposition_distance = float(p('reposition_distance_m'))
        self.reposition_tol = float(p('reposition_tolerance_m'))
        self.reposition_angle = math.radians(float(p('reposition_angle_deg')))
        self.unblock_distance = float(p('unblock_distance_m'))
        self.unblock_max_s = float(p('unblock_max_s'))
        self.unblock_max = int(p('unblock_max_attempts'))
        self.seek_rescan_max = int(p('seek_rescan_max_attempts'))
        self.give_up_yolo_stale = float(p('give_up_yolo_stale_s'))
        self.give_up_to_via = float(p('give_up_to_via_s'))
        self.give_up_to_goal = float(p('give_up_to_goal_s'))
        self.give_up_reverse = float(p('give_up_reverse_m'))
        if not all(math.isfinite(v) and v >= 0 for v in (
                self.give_up_yolo_stale, self.give_up_to_via, self.give_up_to_goal,
                self.give_up_reverse)):
            raise ValueError('give_up_* must be finite and >= 0 (0 = off)')
        if not all(math.isfinite(v) and v > 0 for v in
                   (self.stage_radius, self.gps_timeout, self.odom_timeout,
                    self.perception_timeout,
                    self.max_turn, self.approach_cmd, self.final_cmd,
                    self.retreat_cmd, self.via_tol, self.goal_tol, self.yaw_tol,
                    self.min_conf, self.sample_offset, self.bearing_spread,
                    self.bearing_window, self.scan_kp, self.search_turn,
                    self.search_centre, self.search_reverse, self.camera_half_fov,
                    self.search_max_turns,
                    self.seek_cmd, self.seek_max_travel, self.front_stop,
                    self.seek_center, self.seek_max_update,
                    self.front_half_angle, self.cloud_window, self.lock_window,
                    self.lock_goal_spread, self.lock_normal_spread, self.lock_max_distance,
                    self.via_margin, self.backoff_max, self.reposition_after,
                    self.reposition_distance, self.reposition_tol, self.slot_pitch,
                    self.reposition_angle, self.unblock_distance, self.unblock_max_s)
                   ) or not math.isfinite(self.retreat_distance) or self.retreat_distance <= 3.0 \
                or self.unblock_max < 0 or self.seek_rescan_max < 0 \
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
        self.search_since_decision = 0.0            # 도형으로 방향을 정한 뒤 돈 각 [rad]
        self.search_limit = 0.0                     # 이만큼 돌아도 새 도형이 없으면 반대로 [rad]
        self.search_names = set()                   # 그때까지 본 다른 도형 이름
        self.search_reversed = False
        self.search_empty_side = None               # 돌아 봐도 새 도형이 없던 쪽 (+1/−1)
        self.search_turned = 0.0                    # 이번 탐색에서 누적 선회 [rad]
        self.search_prev_yaw = None
        self.search_exhausted = False               # search_max_turns 를 다 돌았다
        self.scan_home_yaw = None                   # 이번 SCAN 을 시작한 선수각 (대개 도크 쪽)
        self.give_up_since = None                   # 다 돌고 시작 선수각으로 되돌리기 시작 (monotonic)
        self.give_up_sent = False                   # /mission/give_up 보냄
        self.give_up_last = 0.0
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
        self.blocked_since = None                   # SEEK 앞 막힘 시작 (monotonic)
        self.reposition = None                      # 우회 계획 (임시 ㄷ자·경유점·단계)
        self.reposition_used = False                # 이번 도킹에서 우회를 이미 했나
        self.reposition_planned = 0.0               # 마지막 막힘 우회 계획 시도 (monotonic)
        self.oblique_checked = 0.0                  # 마지막 비스듬함(37°) 확인 (monotonic)
        self.reposition_why = ''
        self.unblock_count = 0                      # 이번 도킹에서 막혀 후진한 횟수
        self.seek_rescans = 0                       # 이번 도킹에서 SEEK 거리 한도로 다시 SCAN 한 횟수
        self.unblock_start = self.unblock_heading = None
        self.unblock_since = 0.0
        self.unblock_why = ''
        self.abort_reason = ''                      # ABORT: 포기 이유 (/mission/give_up 'dock: …')
        self.abort_from = None                      # ABORT 를 시작한 odom 위치 (후진 거리)
        self.abort_since = 0.0
        self.abort_heading = None                   # ABORT 전 후진 때 지킬 선수각 (None = 후진 없이 포기)
        self.yolo_stale_since = None                # SCAN 에서 YOLO 가 끊긴 때 (monotonic)
        self.phase_seen = 'IDLE'                    # 단계가 바뀐 때를 재려고 (TO_VIA·TO_GOAL 시간 제한)
        self.phase_since = time.monotonic()
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
        # 목표를 끝내 못 찾음 → mission_manager 에 dock 건너뛰기 요청 (전체 실행에서만 받아 줌)
        self.give_up_pub = (self.create_publisher(String, '/mission/give_up', 10)
                            if self.mission_mode else None)
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

    def _reset_search(self, phase, keep_reposition=False):
        self.phase = phase
        self.abort_reason, self.abort_from, self.abort_heading = '', None, None
        self.yolo_stale_since = None
        self.blocked_since = None
        self.reposition = None
        self.unblock_start = None
        if not keep_reposition:                     # 새 도킹 — 우회·막힘 후진·다시 SCAN 횟수도 처음부터
            self.reposition_used = False
            self.reposition_why = ''
            self.unblock_count = 0
            self.seek_rescans = 0
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
        self.scan_home_yaw = None                   # 첫 SCAN tick 의 선수각으로 다시 잡는다
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
                    'COMPLETE': (1., 1., 1.), 'REPOSITION': (1., .2, 1.), 'UNBLOCK': (1., .5, .5),
                    'ABORT': (1., .1, .1)}

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
        if self.reposition is not None:                 # 우회: 임시 목표 ㄷ자(자홍)·경유점
            rp = self.reposition
            way = marker(6, Marker.SPHERE, (1., .2, 1.), .45, z=.3)
            way.pose.position.x, way.pose.position.y = map(float, rp['waypoint'])
            n = np.asarray(rp['outward'])
            t = np.array([-n[1], n[0]]) * self.dims.slot_width / 2
            depth = n * self.dims.wall_to_mouth
            c = np.asarray(rp['target'])
            slot = marker(7, Marker.LINE_STRIP, (1., .2, 1.), .08, z=0.0)
            slot.points = [Point(x=float(x), y=float(y), z=.15)
                           for x, y in (c + t + depth, c + t, c - t, c - t + depth)]
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

    # ── 막히면 우회: 보이는 ㄷ자를 슬롯 간격만큼 옮겨 임시 목표 ㄷ자 → 그 축 위 경유점 ─────────
    def _visible_u_slot(self, points, boat, yaw):
        """목표 방위선 제한 없이, 지금 양쪽 면까지 다 보이는 ㄷ자 하나 (점수 최고). 없으면 None.

        find_u_slot 은 앞방향 ±40° 안에서 도크 방향을 찾으므로 선수 기준 −80~+80° 를 40° 씩 본다.
        """
        params = replace(self.slot_params, target_offset=self.slot_pitch)
        best = None
        for offset in range(-80, 81, 40):
            direction = (math.cos(yaw + math.radians(offset)), math.sin(yaw + math.radians(offset)))
            slot, _ = find_u_slot(points, boat, boat, direction, params)
            if (slot is not None and slot.wall_distance <= self.lock_max_distance
                    and (best is None or slot.score > best.score)):
                best = slot
        return best

    def _plan_reposition(self, xy, yaw, slot=None, require_offset=False):
        """보이는 ㄷ자 + 목표 방위선 → 임시 목표 ㄷ자·경유점. 성공하면 True (reposition_why 에 이유).

        require_offset: 보이는 ㄷ자가 바로 목표 슬롯(0칸)이면 우회하지 않는다 — 그 ㄷ자는 SEEK 가
        그대로 잠근다 (비스듬해도 양쪽 면이 보이면 잠긴다: 시뮬 43° 끝 슬롯).
        """
        if not self.cloud or self.locked_bearing is None or self.seek_line_point is None:
            self.reposition_why = 'no scan cloud or target bearing'
            return False
        boat = self._odom_xy()
        if slot is None:
            slot = self._visible_u_slot(np.vstack([pts for _, pts in self.cloud]), boat, yaw)
        if slot is None:
            self.reposition_why = 'no complete U slot in view to offset from'
            return False
        centre, outward = np.asarray(slot.center), np.asarray(slot.outward)
        along = np.array([-outward[1], outward[0]])          # 뒷벽 따라
        bearing = np.array([math.cos(self.locked_bearing), math.sin(self.locked_bearing)])
        # 목표 방위선(seek_line_point + r·bearing)과 뒷벽 선(centre + q·along)의 교점 → q
        matrix = np.c_[bearing, -along]
        if abs(np.linalg.det(matrix)) < 1e-6:
            self.reposition_why = 'target bearing parallel to the dock wall'
            return False
        r, q = np.linalg.solve(matrix, centre - np.asarray(self.seek_line_point))
        steps = int(round(q / self.slot_pitch))
        if r <= 0 or abs(q - steps * self.slot_pitch) > .4 * self.slot_pitch:
            self.reposition_why = (f'target bearing hits the wall {q:+.1f}m from the visible slot '
                                   f'— not a slot position')
            return False
        if require_offset and steps == 0:
            self.reposition_why = 'the visible U slot is the target itself'
            return False
        target = centre + steps * self.slot_pitch * along
        waypoint = target + self.reposition_distance * outward
        heading = math.atan2(-outward[1], -outward[0])        # 임시 ㄷ자를 정면으로
        start = self._position()
        way_enu = start + waypoint - boat                     # 위치 출처(odom/gps) 좌표로
        try:
            samples = straight_samples(start, way_enu)
        except ValueError:
            self.reposition_why = 'already at the reposition waypoint'
            return False
        self.reposition = dict(slot=centre, target=target, outward=outward, steps=steps,
                               waypoint=waypoint, waypoint_enu=way_enu, heading=heading, stage='turn',
                               plan=sampled_path(samples, spacing=.1, curvatures=np.zeros(len(samples)),
                                                 frame_id='dock_enu'),
                               blocked_since=None)
        self.reposition_used = True
        self.follower.reset()
        self.slot_samples.clear()                   # 우회 중엔 임시 목표 축으로 목표 ㄷ자를 새로 모은다
        self.slot_received = 0.0
        self.reposition_why = (f'visible U slot shifted {steps:+d} x {self.slot_pitch:.2f}m → waypoint '
                               f'{self.reposition_distance:.1f}m out on target axis')
        return True

    def _oblique_reposition(self, xy, yaw):
        """SEEK: 보이는 ㄷ자 법선과 목표 방위가 reposition_angle 이상이면 막히기 전에 우회 계획.

        도크 위치는 모른다 — 지금 양쪽 면까지 보이는 ㄷ자의 법선을 쓴다(세 슬롯은 한 부교라 평행).
        보이는 ㄷ자가 없으면(대개 6 m 밖 — 그 거리에선 비스듬해도 칸막이에 안 닿는다) 판단 안 함.
        """
        now = time.monotonic()
        if (self.reposition_used or not self.cloud or self.locked_bearing is None
                or now - self.oblique_checked < 1.0):
            return False
        self.oblique_checked = now                  # 보이는 ㄷ자 찾기는 무거워 1 s 에 한 번
        slot = self._visible_u_slot(np.vstack([pts for _, pts in self.cloud]), self._odom_xy(), yaw)
        if slot is None:
            return False
        inward = -np.asarray(slot.outward)
        bearing = np.array([math.cos(self.locked_bearing), math.sin(self.locked_bearing)])
        angle = math.acos(float(np.clip(bearing @ inward, -1.0, 1.0)))
        if angle < self.reposition_angle:
            return False
        if not self._plan_reposition(xy, yaw, slot=slot, require_offset=True):
            return False
        self.reposition_why = (f'target {math.degrees(angle):.0f} deg off the visible U slot axis '
                               f'(>= {math.degrees(self.reposition_angle):.0f}) — {self.reposition_why}')
        return True

    def _reposition_lock(self, yaw):
        """REPOSITION 중에도 임시 목표 축으로 목표 ㄷ자를 찾아, 안정되면 바로 TO_VIA. 바뀌었으면 True.

        받는 조건: 잠긴 중심이 임시 목표 중심에서 1 m 안(옆 슬롯 아님), 뒷벽까지 경유점+여유(3.94 m)
        이상(BACKOFF 후진 안 함). 아니면 우회를 그대로 이어 간다.
        """
        rp = self.reposition
        self._update_cloud()
        if self.cloud and self.cloud_version != self.detected_version:
            self.detected_version = self.cloud_version
            target, outward = np.asarray(rp['target']), np.asarray(rp['outward'])
            slot, why = find_u_slot(np.vstack([pts for _, pts in self.cloud]), self._odom_xy(),
                                    target, -outward, self.slot_params)
            self.slot_last = (slot, why)
            self._publish_slot_markers(slot)
            if (slot is not None and slot.wall_distance <= self.lock_max_distance
                    and float(np.linalg.norm(np.asarray(slot.center) - target)) <= 1.0):
                centre, normal = np.asarray(slot.center), np.asarray(slot.outward)
                self.slot_samples.append((self.cloud[-1][0], centre, normal,
                                          centre + self.via_distance * normal,
                                          centre + self.stop_distance * normal))
                self.slot_received = time.monotonic()
        ok, lock, _ = self._wall_lock()
        if not ok:
            return False
        have, need = self._via_room(lock)
        if have < need or not self._route(lock['via'], lock['stop']):
            return False
        self.lock = lock
        self.reposition = None
        self.phase = 'TO_VIA'
        self._command(0, 0)
        self._report(f'target U slot locked during reposition {have:.2f}m away; one-shot route fixed')
        return True

    def _reposition_tick(self, xy, yaw):
        """REPOSITION: 경유점 쪽으로 선회 → 전진(앞 막힘 검사) → 임시 ㄷ자 정면으로 선회 → SCAN."""
        if self._reposition_lock(yaw):
            return
        rp = self.reposition
        way = rp['waypoint_enu']
        distance = float(np.linalg.norm(way - xy))
        if rp['stage'] == 'turn':
            head = math.atan2(way[1] - xy[1], way[0] - xy[0])
            error = wrap(head - yaw)
            if abs(error) <= math.radians(15):
                rp['stage'] = 'drive'
            self._command(0, self._scan_turn(head, yaw))
            self._report(f'reposition: turning to waypoint ({math.degrees(error):+.0f} deg, {distance:.1f}m)')
            return
        if rp['stage'] == 'drive':
            if distance <= self.reposition_tol:
                rp['stage'] = 'face'
                self._command(0, 0)
                self._report('reposition: at waypoint')
                return
            if self.scan is None or time.monotonic() - self.scan_received > self.perception_timeout:
                self._command(0, 0)
                self._report('reposition: scan stale — holding')
                return
            blocked = front_clearance(self.scan, self.front_half_angle, self.front_stop)
            if blocked is not None:
                self._command(0, 0)
                now = time.monotonic()
                if rp['blocked_since'] is None:
                    rp['blocked_since'] = now
                held = now - rp['blocked_since']
                if held >= self.reposition_after and self._start_unblock(
                        yaw, f'reposition blocked {held:.1f}s, {blocked:.2f}m ahead'):
                    return
                if held >= self.reposition_after and self._can_give_up():
                    self._start_abort(f'reposition blocked {blocked:.2f}m ahead after '
                                      f'{self.unblock_count} retreats')
                    return
                self._report(f'reposition: something {blocked:.2f}m ahead — holding ({distance:.1f}m to go)'
                             + self._unblock_note())
                return
            rp['blocked_since'] = None
            step = self.follower.step(rp['plan'], xy, yaw, self.odom.twist.twist.angular.z,
                                      thrust_ratio=self.seek_cmd, turn_ratio=self.max_turn)
            if step.reached:
                rp['stage'] = 'face'
            self._command(step.linear, step.angular)
            self._report(f'reposition: driving to waypoint, {distance:.1f}m to go')
            return
        error = wrap(rp['heading'] - yaw)                     # face: 임시 ㄷ자 정면
        if abs(error) <= self.yaw_tol and abs(self.odom.twist.twist.angular.z) <= math.radians(5):
            self._reset_search('SCAN', keep_reposition=True)  # 목표 표지가 가운데 — 다시 잠그고 SEEK
            self.reposition_why = 'already repositioned once — holding'
            self._command(0, 0)
            self._report('reposition done — facing temporary U slot, re-scanning')
            return
        self._command(0, self._scan_turn(rp['heading'], yaw))
        self._report(f'reposition: facing temporary U slot ({math.degrees(error):+.0f} deg)')

    # ── 막혀서 더 할 게 없을 때: 온 길로 직진 후진 → SCAN 부터 다시 ──────────────────────
    def _start_unblock(self, yaw, why):
        """선수 유지 직진 후진(UNBLOCK) 시작. 횟수를 다 썼으면 False (호출한 쪽이 멈춰 기다린다).

        뒤(120~240°)는 라이다가 가려 못 본다 — 방금 앞으로 온 길을 같은 선수로 되짚는 것이라
        BACKOFF·REVERSE·미션 시간 초과 후진과 같은 가정이다.
        """
        if self.unblock_count >= self.unblock_max:
            return False
        self.unblock_count += 1
        self.phase = 'UNBLOCK'
        self.unblock_start = self._odom_xy()
        self.unblock_heading = yaw
        self.unblock_since = time.monotonic()
        self.unblock_why = why
        self.reposition = None
        self.follower.reset()
        self._command(0, 0)
        self._report(f'{why} — backing {self.unblock_distance:.1f}m to re-scan '
                     f'({self.unblock_count}/{self.unblock_max})')
        return True

    def _unblock_note(self):
        if self.unblock_max and self.unblock_count >= self.unblock_max:
            return f'; gave up after {self.unblock_count} retreats'
        return ''

    def _unblock_tick(self, yaw):
        backed = float(np.linalg.norm(self._odom_xy() - self.unblock_start))
        elapsed = time.monotonic() - self.unblock_since
        if backed >= self.unblock_distance or elapsed >= self.unblock_max_s:
            self._reset_search('SCAN', keep_reposition=True)
            self._command(0, 0)
            self._report(f'backed {backed:.2f}m in {elapsed:.0f}s after block '
                         f'({self.unblock_count}/{self.unblock_max}) — re-scanning')
            return
        self._command(-self.retreat_cmd, self._turn(self.unblock_heading, yaw))
        self._report(f'unblock {self.unblock_count}/{self.unblock_max}: backing straight '
                     f'{backed:.2f}/{self.unblock_distance:.1f}m ({self.unblock_why})')

    # ── 포기 (미션 모드): 서서 마감을 기다리지 않고 mission_manager 에 dock 건너뛰기 요청 ─────────
    def _can_give_up(self):
        """미션 모드만 — 단독 도킹 시험(give_up_pub 없음)은 예전처럼 그 자리에서 멈춰 기다린다."""
        return self.give_up_pub is not None

    def _start_abort(self, reason, reverse_heading=None):
        """ABORT: reverse_heading 을 주면 그 선수를 지키며 give_up_reverse_m 직진 후진(슬롯 밖으로) 뒤 포기."""
        self.phase = 'ABORT'
        self.abort_reason = reason
        self.abort_from = self._odom_xy()
        self.abort_since = time.monotonic()
        self.abort_heading = reverse_heading if self.give_up_reverse > 0 else None
        self.give_up_last = 0.0
        self.reposition = None
        self.follower.reset()
        self._command(0, 0)
        self._report(f'{reason} — giving up dock'
                     + (f' after backing {self.give_up_reverse:.1f}m' if self.abort_heading is not None else ''))

    def _abort_tick(self, yaw):
        backed = float(np.linalg.norm(self._odom_xy() - self.abort_from))
        elapsed = time.monotonic() - self.abort_since
        if (self.abort_heading is not None and backed < self.give_up_reverse
                and elapsed < self.unblock_max_s):
            self._command(-self.retreat_cmd, self._turn(self.abort_heading, yaw))
            self._report(f'{self.abort_reason} — backing out {backed:.2f}/{self.give_up_reverse:.1f}m '
                         'before giving up')
            return
        self._command(0, 0)
        now = time.monotonic()
        if now - self.give_up_last >= 1.0:          # 한 번 잃어도 되게 — state 가 바뀌면 IDLE
            self.give_up_pub.publish(String(data=f'dock: {self.abort_reason}'))
            self.give_up_last = now
            self.give_up_sent = True
        self._report(f'{self.abort_reason} — gave up, asked mission manager to skip dock')

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
        self.search_limit = 0.0
        self.search_since_decision = 0.0
        self.search_names = set()
        self.search_reversed = False
        self.search_empty_side = None               # 돌아 봐도 새 도형이 없던 쪽 (+1/−1)
        self.search_turned = 0.0
        self.search_prev_yaw = None
        self.search_exhausted = False
        self.give_up_since = None
        self.give_up_sent = False
        self.give_up_last = 0.0

    def _search_exhausted(self, yaw, basis, why):
        """누적 search_max_turns 바퀴를 돌아도 목표가 없을 때 (전진, 선회, 설명).

        미션 모드: SCAN 시작 선수각(대개 도크 쪽)으로 되돌린 뒤(최대 GIVE_UP_ALIGN_MAX_S) /mission/give_up 을
        1 s 마다 보낸다. 전체 실행이면 mission_manager 가 받아 직진 후진 3 m → 다음 미션 — 탐색 선회로
        선수가 아무 데나 향한 채 후진하면 도크 쪽으로 갈 수 있다. 단일 시험은 무시돼 그대로 멈춘다.
        """
        lost = f'target {self.target} not found after {self.search_max_turns:g} turns'
        if self.give_up_pub is None:
            return 0.0, 0.0, f'{lost} — holding ({basis}; {why})'
        now = time.monotonic()
        if self.give_up_since is None:
            self.give_up_since = now
        home = yaw if self.scan_home_yaw is None else self.scan_home_yaw
        error = wrap(home - yaw)
        settled = (abs(error) <= self.yaw_tol
                   and abs(self.odom.twist.twist.angular.z) <= math.radians(5))
        if (not self.give_up_sent and not settled
                and now - self.give_up_since < GIVE_UP_ALIGN_MAX_S):
            return 0.0, self._scan_turn(home, yaw), (
                f'{lost} — turning back to scan start heading ({math.degrees(error):+.0f} deg) '
                f'before giving up ({basis})')
        if now - self.give_up_last >= 1.0:          # 한 번 잃어도 되게 — state 가 바뀌면 IDLE
            self.give_up_pub.publish(String(data=f'dock: {lost}'))
            self.give_up_last = now
            self.give_up_sent = True
        return 0.0, 0.0, f'{lost} — gave up, asked mission manager to skip dock ({basis}; {why})'

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
            return self._search_exhausted(yaw, basis, why)
        new_shape = bool(set(seen) - self.search_names)
        self.search_names |= set(seen)
        if len(seen) >= 2 and self.search_rank < 2:
            low, high = sorted(seen.values())[0], sorted(seen.values())[-1]
            gap = high - low                        # 이웃 도형 사이 화면 간격
            # 화면 안에 들어올 후보는 아니다(보였을 것) — 화면 밖 후보 중 가운데에 가까운 쪽
            left, right = high + gap, low - gap
            outside = [c for c in (left, right)
                       if abs(c) > self._half_fov(1 if c > 0 else -1) - 3.0] or [left, right]
            # 이미 훑어 비어 있던 쪽 후보는 뺀다 (끝 슬롯 너머엔 도형이 없다)
            if self.search_empty_side is not None:
                outside = [c for c in outside if (1 if c > 0 else -1) != self.search_empty_side] or outside
            predicted = min(outside, key=abs)
            self.search_dir = 1 if predicted == left else -1
            self.search_rank = 2
            # 예측 위치가 광축 가운데에 올 때까지(= 예측 각도만큼) 돌아도 새 도형이 없으면 반대로.
            # 같은 간격 가정의 예측 오차(시뮬 −30~+40°)가 반화각 40° 안이라 맞는 쪽이면 그 안에 보인다.
            self.search_limit = max(math.radians(abs(predicted)), math.radians(20.0))
            self.search_since_decision = 0.0
            new_shape = False
        elif len(seen) == 1 and self.search_rank < 1:
            (bearing,) = seen.values()
            if abs(bearing) >= self.search_centre:
                self.search_dir = 1 if bearing > 0 else -1      # 목표는 그 도형 너머
            elif bearing == 0.0:
                self.search_dir = -1                            # 정확히 가운데 — 시계방향
            else:
                self.search_dir = -1 if bearing > 0 else 1      # 치우친 반대쪽
            self.search_rank = 1
            self.search_limit = self.search_reverse
            self.search_since_decision = 0.0
            new_shape = False
        if self.search_dir is None:
            self.search_dir = -1                    # 아무것도 안 보이면 시계방향부터
        step = abs(wrap(yaw - self.search_prev_yaw)) if self.search_prev_yaw is not None else 0.0
        self.search_prev_yaw = yaw
        self.search_turned += step
        reversed_now = False
        if self.search_rank >= 1:
            self.search_since_decision = 0.0 if new_shape else self.search_since_decision + step
            if not self.search_reversed and self.search_since_decision >= self.search_limit:
                self.search_empty_side = self.search_dir
                self.search_dir = -self.search_dir   # 그쪽엔 더 없다 (끝 슬롯) — 한 번만 뒤집기
                self.search_reversed = reversed_now = True
        turned = math.degrees(self.search_turned)
        if self.search_turned >= self.search_max_turns * 2 * math.pi:
            self.search_exhausted = True            # 목표가 보이면 _clear_search 로 다시 시작
            return self._search_exhausted(yaw, basis, why)
        side = 'left' if self.search_dir > 0 else 'right (clockwise)'
        note = ' — reversed: nothing new that way' if reversed_now or self.search_reversed else ''
        return 0.0, self.search_dir * self.search_turn, (
            f'searching {side}{note} for {self.target} ({basis}; turned {turned:.0f}/'
            f'{360 * self.search_max_turns:.0f} deg; {why})')

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
        now = time.monotonic()
        if self.phase != self.phase_seen:
            self.phase_seen, self.phase_since = self.phase, now
        in_phase = now - self.phase_since
        if self.phase == 'ABORT':
            self._abort_tick(yaw)
            return
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
            if self.scan_home_yaw is None:
                self.scan_home_yaw = yaw
            linear, turn, why = self._scan_target(yaw)
            if why == 'YOLO stale':
                if self.yolo_stale_since is None:
                    self.yolo_stale_since = now
                stale = now - self.yolo_stale_since
                if self._can_give_up() and self.give_up_yolo_stale and stale >= self.give_up_yolo_stale:
                    self._start_abort(f'YOLO stale for {stale:.0f}s')
                    return
            else:
                self.yolo_stale_since = None
            self._command(linear, turn)
            self._report(why)
            return
        if self.phase == 'SEEK':
            self._update_cloud()
            self._detect_slot()
            taken, wall_why = self._take_wall_lock(yaw)
            if taken:
                return
            if self._oblique_reposition(xy, yaw):
                self.phase = 'REPOSITION'
                self._command(0, 0)
                self._report(self.reposition_why)
                return
            travelled = float(np.linalg.norm(xy - self.seek_start))
            if self.scan is None or time.monotonic() - self.scan_received > self.perception_timeout:
                self._command(0, 0)
                self._report(f'scan stale — holding; {wall_why}')
                return
            blocked = front_clearance(self.scan, self.front_half_angle, self.front_stop)
            if blocked is not None:
                self._command(0, 0)
                now = time.monotonic()
                if self.blocked_since is None:
                    self.blocked_since = now
                held = now - self.blocked_since
                if held >= self.reposition_after:
                    if not self.reposition_used and now - self.reposition_planned >= 1.0:
                        self.reposition_planned = now
                        if self._plan_reposition(xy, yaw):
                            self.phase = 'REPOSITION'
                            self._report(f'blocked {held:.1f}s without U slot — {self.reposition_why}')
                            return
                    # 우회를 못 짰거나 이미 썼다 — 서서 기다리지 않고 온 길로 빠져 다시 찾는다
                    why = (self.reposition_why if not self.reposition_used
                           else 'already repositioned once')
                    if self._start_unblock(yaw, f'blocked {held:.1f}s, {blocked:.2f}m ahead, '
                                                f'no reposition ({why})'):
                        return
                    if self._can_give_up():                 # 우회·막힘 후진을 다 씀 — 더 할 게 없다
                        self._start_abort(f'blocked {blocked:.2f}m ahead after '
                                          f'{self.unblock_count} retreats')
                        return
                note = f'; reposition: {self.reposition_why}' if self.reposition_why else ''
                self._report(f'something {blocked:.2f}m ahead — holding; {wall_why}{note}'
                             + self._unblock_note())
                return
            self.blocked_since = None
            if travelled >= self.seek_max_travel - self.via_tol:
                self._command(0, 0)
                # 앞은 비었는데 ㄷ자를 못 잡음 — 방위가 틀렸거나 낡았다. 서 있지 않고 여기서 목표 방위부터 다시
                if self.seek_rescans < self.seek_rescan_max:
                    self.seek_rescans += 1
                    self._reset_search('SCAN', keep_reposition=True)
                    self._report(f'no U slot within {self.seek_max_travel:.1f}m ({wall_why}) — '
                                 f're-scanning here ({self.seek_rescans}/{self.seek_rescan_max})')
                    return
                if self._can_give_up():
                    self._start_abort(f'no U slot after {self.seek_rescans} re-scans')
                    return
                self._report(f'no U slot within {self.seek_max_travel:.1f}m — holding; {wall_why}; '
                             f'gave up after {self.seek_rescans} re-scans')
                return
            sight_why = self._seek_sight()
            error = wrap(self.locked_bearing - yaw)
            centred = abs(error) <= self.seek_center
            self._command(self.seek_cmd if centred else 0.0, self._scan_turn(self.locked_bearing, yaw))
            self._report(f'seeking {math.degrees(self.locked_bearing):.1f} deg '
                         f'({"centred" if centred else f"centring {math.degrees(error):+.1f} deg"}), '
                         f'{travelled:.1f}m; {sight_why}; {wall_why}')
            return
        if self.phase == 'REPOSITION':
            self._reposition_tick(xy, yaw)
            return
        if self.phase == 'UNBLOCK':
            self._unblock_tick(yaw)
            return
        if self.phase == 'BACKOFF':
            have, need = self._via_room(self.lock)
            backed = float(np.linalg.norm(self._odom_xy() - self.backoff_start))
            if have >= need:
                if self._route(self.lock['via'], self.lock['stop']):
                    self.phase = 'TO_VIA'
                    self._command(0, 0)
                    self._report(f'backed off to {have:.2f}m; one-shot route fixed')
                elif self._can_give_up():
                    self._start_abort('backed off but route rejected')
                else:
                    self._command(0, 0)
                    self._report('backed off but route rejected — holding')
                return
            if backed >= self.backoff_max:
                if self._can_give_up():
                    self._start_abort(f'backed off {backed:.2f}m, still {have:.2f}/{need:.2f}m from wall')
                    return
                self._command(0, 0)
                self._report(f'backed off {backed:.2f}m, still {have:.2f}/{need:.2f}m — holding')
                return
            self._command(-self.retreat_cmd, self._turn(self.backoff_heading, yaw))
            self._report(f'backing off {have:.2f}/{need:.2f}m from target wall')
            return
        if self.phase == 'TO_VIA':
            if self._can_give_up() and self.give_up_to_via and in_phase >= self.give_up_to_via:
                # 경유점은 입구 밖 — 후진 없이 포기 (mission_manager 가 직진 후진 3 m)
                self._start_abort(f'via not reached in {in_phase:.0f}s '
                                  f'({np.linalg.norm(xy-self.route_enu[1]):.2f}m left)')
                return
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
            if self._can_give_up() and self.give_up_to_goal and in_phase >= self.give_up_to_goal:
                # 슬롯 안일 수 있다 — 슬롯 축 선수를 지키며 먼저 빠져나온 뒤 포기
                self._start_abort(f'berth not reached in {in_phase:.0f}s '
                                  f'({np.linalg.norm(xy-self.route_enu[2]):.2f}m left)',
                                  reverse_heading=self.final_heading)
                return
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
