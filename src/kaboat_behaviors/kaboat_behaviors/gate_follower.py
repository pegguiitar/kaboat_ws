"""Gate behavior: LiDAR grid -> DRI -> paired posts -> shared LOS follower.

Each aperture supplies a mandatory centre crossing. The free-form obstacle
avoidance planner is not used because it can route around a gate.

게이트 사슬 (2026-11-12): 지도 부표 덩어리를 기억(gate_memory.PostMemory)해 두고, 지금
게이트(사슬 첫 칸) 다음 게이트를 그 기억으로 미리 골라 경로를 배 → 중점들 → 마지막 게이트
뒤로 이어 둔다(max_chain_gates 까지). 지금 게이트를 기둥 사이로 지나면 경로를 새로 만들지
않고 바로 다음 칸으로 넘어간다. 다음 게이트를 못 찾거나 이은 경로가 검사에 걸리면 예전처럼
게이트 하나씩 간다.

첫 쌍 찾아 다가가기 (2026-11-13): 첫 게이트(통과 0개)에서 쌍이 seek_first_after_sec 넘게 안
잡히면 그 너머 지도 부표 쪽(없으면 배 → gate_end 방향, 진행선에서 ±seek_max_bias_deg 안)으로
seek_thrust_ratio 로 천천히 다가간다(상태 SEEK_FIRST_PAIR). 시작 자리에서 seek_max_distance 까지만,
앞 seek_stop_ahead 안에 부표가 있으면 선다.
  2026-11-14: 앞 seek_wait_within(4.5 m) 안에 부표가 보여도 가장 가까운 것이 seek_close_stop_along
  (2.0 m) 앞에 올 때까지는 다가간다 (예전엔 4.5 m 안에 보이면 바로 서서 물살에 밀렸다). 그 대신
  첫 게이트 지키기를 둔다 — 예전에 보이는데 다가가면 게이트 1 이 쌍이 되기 전에 게이트 2 가 먼저
  쌍이 되어 첫 게이트를 건너뛰었다(시뮬).

첫 게이트 지키기 (2026-11-14): 아직 통과한 게이트가 없을 때, 고른 쌍의 게이트선보다
first_guard_margin(1.2 m) 넘게 배 쪽에 짝 없는 확정 부표(쌍 폭 안쪽)가 있으면 그 쌍은 게이트 2 로
보고 받지 않는다 — 게이트 1 의 한쪽만 잡힌 것. 이미 따라가던 쌍도 그런 부표가 새로 확정되면 버린다.
그동안은 게이트 1 중점을 짐작해(게이트 2 중점을 짝 없는 부표 줄까지 당김) 그 seek_close_stop_along
앞 통로 가운데 대기점으로 가서 게이트 1 을 보고 서고, 물살에 1.2 m 넘게 밀리면 다시 간다. first_guard_max_sec(15 s) 넘게 계속 막히면(짝 부표가
영영 안 잡힘) 지키기를 풀고 고른 쌍으로 간다.
"""

import dataclasses
import math
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

from .behavior_base import BehaviorBase, yaw_from_quaternion
from .los_controller import LOS_TURN_RATIO
from .dri import DriParams, build_dri
from .gate_corridor import (
    GateParams, describe_reasons, gate_crossing, grid_posts, is_diagonal, lookahead_chord,
    lookahead_chord_safe,
    make_chain_path, make_gate_path, next_gate_after, pair_observed, path_ahead, path_safe,
    points_observed_or_seen, points_safe, scan_hits_world, scan_rays,
    select_gate_pair_from_posts)
from .gate_memory import PostMemory


def _xy(point) -> str:
    return f'({point[0]:.2f}, {point[1]:.2f})'

# 서서 기다릴 때 자리 지키기 게인 (추력 비율 / rad, / (rad/s), / m)
HOLD_YAW_KP = 0.8
HOLD_YAW_KD = 0.3
HOLD_ALONG_KP = 0.3
GATE_LOS_THRUST_RATIO = 0.35  # 게이트 통과(LOS) 직진 상한 — config/gate_params.yaml 과 같게 (0.3 → 0.35, 2026-11-13)
SEEK_LOOK_BEYOND = 4.0        # [m] 첫 쌍 다가가기 방향: 앞 창 너머 이 거리까지의 지도 부표를 본다
FIRST_GUARD_LATERAL_EXTRA = 1.0   # [m] 첫 게이트 지키기: 쌍 폭/2 + 이만큼 옆 안의 부표만 (먼 옆 물체 제외)
FIRST_GUARD_MIN_AHEAD = 0.5       # [m] 첫 게이트 지키기: 배 앞 이만큼 넘게 있는 부표만
GUARD_ARRIVE = 0.6                # [m] 지키기 대기점 도착 (밀려서 GUARD_RESUME 넘으면 다시 감)
GUARD_RESUME = 1.2
GUARD_TARGET_MAX = 6.0            # [m] 대기점이 이보다 멀면 짐작이 틀린 것으로 보고 안 감


def nearer_lone_post(pair, boat_xy, posts, margin, own_radius=0.7):
    """고른 첫 쌍의 게이트선보다 margin 넘게 배 쪽에 있는 짝 없는 부표 → (x, y) 또는 None.

    그런 부표가 쌍 폭 안쪽(± 폭/2 + FIRST_GUARD_LATERAL_EXTRA)·배 앞에 있으면 그 쌍은 더 먼 게이트
    (게이트 2)일 가능성이 크다 — 게이트 1 은 한쪽 부표만 잡힌 것. 여럿이면 배에 가장 가까운 것.
    own_radius 안의 부표는 쌍 자신으로 본다. margin <= 0 이면 끔.
    """
    if margin <= 0.0 or not posts:
        return None
    mid = np.asarray(pair.midpoint, dtype=float)
    normal = np.asarray(pair.direction, dtype=float)
    line = np.array([-normal[1], normal[0]])
    boat = np.asarray(boat_xy, dtype=float)
    own = np.asarray([pair.left, pair.right], dtype=float)
    best, best_before = None, -math.inf
    for post in posts:
        q = np.asarray(post, dtype=float)
        if float(np.min(np.linalg.norm(own - q, axis=1))) < own_radius:
            continue
        before = float((mid - q) @ normal)          # + = 게이트선보다 배 쪽
        if before < margin or float((q - boat) @ normal) < FIRST_GUARD_MIN_AHEAD:
            continue
        if abs(float((q - mid) @ line)) > pair.width / 2.0 + FIRST_GUARD_LATERAL_EXTRA:
            continue
        if before > best_before:
            best, best_before = (float(q[0]), float(q[1])), before
    return best


class GateFollower(BehaviorBase):
    STATE_NAME = 'gate'
    CMD_TOPIC = '/cmd/gate'

    def __init__(self):
        super().__init__()
        self.gate_params = self._params_from('gate', GateParams)
        self.dri_params = self._params_from('dri', DriParams)
        self.declare_parameter('gate.thrust_ratio', 0.5)       # waypoint(접근) 구간
        # LOS 경로 추종 구간: 직진 0.3·회전 0.31 상한. 직진은 야외 바람 때문에 공용 LOS
        # SOTA 0.25(약 1619 µs)에서 0.4(약 1675 µs)로 올렸다가(2026-11-12 오전) 0.3(약 1637 µs)
        # 으로 내림(2026-11-12 오후): 0.4 는 실측 약 1.0 m/s 라 간격 3~4 m 게이트 사이가 3~4초 —
        # 옆으로 어긋난 다음 게이트로 S 자로 옮기기에 빠름.
        # 회전 상한·요 게인·lookahead 는 공용 LOS SOTA 값 그대로.
        self.declare_parameter('gate.los_thrust_ratio', GATE_LOS_THRUST_RATIO)
        self.declare_parameter('gate.los_turn_ratio', LOS_TURN_RATIO)
        self.declare_parameter('gate.motion_enabled', False)
        # 선체 라이다 장착 위치(occupancy_grid 의 lidar_offset_x/y 와 같은 값,
        # 실측 선체 중심보다 선수 0.4 m)
        self.declare_parameter('lidar_offset_x', 0.4)
        self.declare_parameter('lidar_offset_y', 0.0)
        self.declare_parameter('gate.scan_timeout_sec', 0.7)
        self.declare_parameter('gate.grid_timeout_sec', 0.7)
        # 배 중심이 게이트선 이 거리 안이면 기둥이 지금 스캔에 보이는지 더 묻지 않는다.
        # 선체 라이다는 선수 0.5 m 에 있고 뒤쪽 120°~240° 를 가려서, 폭 2 m 게이트면
        # 중심이 선을 넘기 직전에 기둥이 가림 구간으로 들어가 통과 판정을 놓쳤다.
        self.declare_parameter('gate.visibility_release_distance', 1.0)
        # 같은 쌍으로 이 시간 넘게 진행을 못 하면 쌍을 버리고 다시 고른다(이미 게이트선을
        # 넘었으면 통과로 인정). 버린 쌍은 reject_hold_sec 동안 후보에서 뺀다.
        self.declare_parameter('gate.replan_timeout_sec', 2.0)
        # 10 → 3 s (2026-11-12): 버린 쌍을 빼 두는 동안 다른 쌍이 없으면 추진 0 으로 서 있어
        # 바람에 밀렸다 (현장 12초 정지, 시뮬 옆바람 실패의 대부분).
        self.declare_parameter('gate.reject_hold_sec', 3.0)
        # 기둥 보임: 두 기둥 각각 이 시간 [s] 안에 한 번이라도 스캔에 잡혔으면 보이는 것으로
        # (0 = 예전처럼 지금 스캔에 둘 다). 부표 검출은 스캔당 83~92 % 라 둘 다 같은 스캔에
        # 잡힐 확률은 ~75 % — 매 스캔 요구하면 4번에 1번 추진 0 (2026-11-12 녹화 기반 시뮬).
        self.declare_parameter('gate.post_visible_timeout_sec', 1.0)
        # 서서 기다릴 때(쌍 없음·경로 막힘·기둥 안 보임 …) 추진 0 대신 멈춘 자리·선수각을
        # 이 추력 상한으로 지킨다 (0 = 예전처럼 추진 0). 추진 0 이면 바람에 밀려 부표 쪽으로
        # 가거나 통로를 벗어났다 (시뮬 북풍 0.1 m/s 4/4 실패). 센서 끊김 정지는 그대로 0.
        # 기본 끔: 시뮬에서 옆바람(북 0.1 m/s)·검출 저하엔 나아졌지만 잔잔할 때 첫 게이트를
        # 놓치는 경우가 생겨 더 손봐야 함 (scripts/gate_sim, 2026-11-12).
        self.declare_parameter('gate.hold_thrust_ratio', 0.0)
        # 밀려서 막히면 서기 전에 지금 자리에서 경로 다시 그리기 + 안전 검사를 '지금보다 더
        # 가까워지지 않기'까지 풀기 (false = 예전처럼 바로 막힘·정지)
        self.declare_parameter('gate.replan_from_here', True)
        # 첫 쌍 찾아 다가가기 (위 모듈 설명): 첫 게이트에서 쌍이 이 시간 [s] 넘게 안 잡히면
        # 천천히 다가간다 (0 = 끔, 예전처럼 서서 기다림)
        self.declare_parameter('gate.seek_first_after_sec', 2.0)
        self.declare_parameter('gate.seek_thrust_ratio', 0.12)     # 직진 (시뮬 약 0.45 m/s)
        self.declare_parameter('gate.seek_max_distance', 3.0)      # [m] 시작 자리에서 이만큼까지만
        self.declare_parameter('gate.seek_stop_ahead', 1.5)        # [m] 앞 이 안에 부표가 있으면 섬
        # [m] 앞 이 거리 안에 부표가 보이면 다가가지 않고 쌍을 기다린다 (검출 83~92 % 거리)
        self.declare_parameter('gate.seek_wait_within', 4.5)
        # [m] 위 거리 안에 부표가 보여도 가장 가까운 것이 이만큼 앞에 올 때까지는 다가간다
        # (2026-11-14, 첫 게이트 지키기와 같이). seek_wait_within 이상이면 예전처럼 보이면 바로 섬
        self.declare_parameter('gate.seek_close_stop_along', 2.0)
        self.declare_parameter('gate.seek_max_bias_deg', 30.0)     # 진행선에서 부표 쪽으로 꺾는 상한
        # 첫 게이트 지키기 (위 모듈 설명): 고른 첫 쌍의 게이트선보다 이만큼 [m] 넘게 배 쪽에 짝 없는
        # 확정 부표가 있으면 그 쌍(게이트 2)을 받지 않음 (0 = 끔)
        self.declare_parameter('gate.first_guard_margin', 1.2)
        self.declare_parameter('gate.first_guard_max_sec', 15.0)   # 이만큼 넘게 막히면 지키기를 풂
        # 포기 (2026-11-14): 마지막 게이트를 지난 뒤(없으면 시작부터) 진행(FOLLOW·GOAL_APPROACH·SEEK_FIRST_PAIR)이
        # 아닌 상태로 보낸 시간이 이만큼 쌓이면 /mission/give_up — 전체 실행이면 mission_manager 가 다음 미션으로.
        # 폐루프 시뮬: 성공 23 판 최대 11.6 s, 실패 판(게이트 1 을 기둥 밖으로 지남·강한 조류) 127~166 s 를 서서
        # 떠밀렸다(16 m). 게이트 수가 대회 배치와 달라 required_pairs 를 못 채울 때도 여기서 끝난다. 0 = 끔.
        self.declare_parameter('gate.give_up_stall_sec', 25.0)
        self.thrust_ratio = float(self.get_parameter('gate.thrust_ratio').value)
        self.los_thrust_ratio = float(
            self.get_parameter('gate.los_thrust_ratio').value)
        self.los_turn_ratio = float(
            self.get_parameter('gate.los_turn_ratio').value)
        self.motion_enabled = bool(self.get_parameter('gate.motion_enabled').value)
        self.lidar_offset = (float(self.get_parameter('lidar_offset_x').value),
                             float(self.get_parameter('lidar_offset_y').value))
        self.scan_timeout = float(self.get_parameter('gate.scan_timeout_sec').value)
        self.grid_timeout = float(self.get_parameter('gate.grid_timeout_sec').value)
        self.visibility_release = float(
            self.get_parameter('gate.visibility_release_distance').value)
        self.replan_timeout = float(self.get_parameter('gate.replan_timeout_sec').value)
        self.reject_hold = float(self.get_parameter('gate.reject_hold_sec').value)
        self.post_visible_timeout = float(
            self.get_parameter('gate.post_visible_timeout_sec').value)
        self.hold_thrust = float(self.get_parameter('gate.hold_thrust_ratio').value)
        self.replan_from_here = bool(self.get_parameter('gate.replan_from_here').value)
        self.seek_after = float(self.get_parameter('gate.seek_first_after_sec').value)
        self.seek_thrust = float(self.get_parameter('gate.seek_thrust_ratio').value)
        self.seek_max_distance = float(self.get_parameter('gate.seek_max_distance').value)
        self.seek_stop_ahead = float(self.get_parameter('gate.seek_stop_ahead').value)
        self.seek_wait_within = float(self.get_parameter('gate.seek_wait_within').value)
        self.seek_close_stop = float(self.get_parameter('gate.seek_close_stop_along').value)
        self.first_guard_margin = float(self.get_parameter('gate.first_guard_margin').value)
        self.first_guard_max_sec = float(self.get_parameter('gate.first_guard_max_sec').value)
        self.give_up_stall = float(self.get_parameter('gate.give_up_stall_sec').value)
        if not (math.isfinite(self.give_up_stall) and self.give_up_stall >= 0.0):
            raise ValueError('gate.give_up_stall_sec must be finite and >= 0 (0 = off)')
        self.seek_max_bias = math.radians(
            float(self.get_parameter('gate.seek_max_bias_deg').value))
        if (not all(math.isfinite(v) for v in (
                self.seek_after, self.seek_thrust, self.seek_max_distance,
                self.seek_stop_ahead, self.seek_wait_within, self.seek_max_bias,
                self.seek_close_stop, self.first_guard_margin, self.first_guard_max_sec))
                or self.seek_after < 0.0 or not 0.0 <= self.seek_thrust <= 1.0
                or self.seek_max_distance < 0.0 or self.seek_stop_ahead < 0.0
                or self.seek_wait_within < self.seek_stop_ahead
                or self.seek_close_stop < self.seek_stop_ahead
                or self.first_guard_margin < 0.0 or self.first_guard_max_sec <= 0.0
                or not 0.0 <= self.seek_max_bias < math.pi / 2):
            raise ValueError('invalid gate first-pair seek parameters')
        if (not all(math.isfinite(v) for v in (
                self.visibility_release, self.replan_timeout, self.reject_hold,
                self.post_visible_timeout, self.hold_thrust))
                or self.visibility_release < 0.0 or self.replan_timeout <= 0.0
                or self.reject_hold < 0.0 or self.post_visible_timeout < 0.0
                or not 0.0 <= self.hold_thrust <= 1.0):
            raise ValueError('invalid gate visibility release or replan timing')
        if (not math.isfinite(self.thrust_ratio)
                or not 0.0 <= self.thrust_ratio <= 1.0
                or not math.isfinite(self.los_thrust_ratio)
                or not 0.0 <= self.los_thrust_ratio <= 1.0
                or not math.isfinite(self.los_turn_ratio)
                or not 0.0 <= self.los_turn_ratio <= 1.0
                or not math.isfinite(self.scan_timeout) or self.scan_timeout <= 0
                or not math.isfinite(self.grid_timeout) or self.grid_timeout <= 0):
            raise ValueError('invalid gate thrust ratio or sensor timeout')

        self._scan_at = None
        self._scan_hits = None
        self._scan_rays = None       # 지금 스캔 빔 — 지도 unknown 칸을 빔이 지나 봤는지
        self._grid_at = None
        self._pair = None
        self._path = None
        self._previous_xy = None
        self._crossed = False
        self._passed_midpoints = []
        self._passed_directions = []   # 통과한 게이트 진행 방향 — 다음 쌍 기울기 기준
        self._status_text = None
        self._stall_since = None
        self._rejected = []          # [(midpoint, 다시 후보가 되는 시각)]
        self._chain = []             # 미리 고른 게이트들 — [0] 이 지금 게이트(_pair)
        self._post_seen_at = [None, None]   # 지금 게이트 왼·오 기둥이 스캔에 마지막으로 잡힌 시각
        self._hold_point = None      # 서서 기다릴 때 지키는 자리 (x, y, 선수각 deg)
        self._memory_grid = None     # 기억에 마지막으로 넣은 지도 메시지
        self._extend_block_until = 0.0   # 이은 경로가 검사에 걸려 줄인 뒤 잠깐 다시 잇지 않음
        self._replan_block_until = 0.0   # 밀려서 다시 그린 뒤 잠깐은 다시 안 그림
        self._wait_since = None      # 첫 쌍을 기다리기 시작한 시각 (첫 쌍 찾아 다가가기)
        self._seek_origin = None     # 다가가기 시작한 자리 — 여기서 seek_max_distance 까지만
        self._seek_note = None       # 다가가다 선 이유 (로그 한 번만)
        self._guard_since = None     # 첫 게이트 지키기가 처음 막은 시각
        self._guard_expired = False  # first_guard_max_sec 넘어 지키기를 푼 뒤
        self._guard_hint = None      # (짐작한 게이트 1 중점, 진행 방향, 시각) — 대기점
        self._guard_arrived = False  # 지키기 대기점에 와서 서 있는 중
        p = self.gate_params
        self._memory = PostMemory(
            match_radius=p.memory_match_radius, confirm_hits=p.memory_confirm_hits,
            forget_sec=p.memory_forget_sec, recheck_range=p.memory_recheck_range,
            prune_range=p.memory_prune_range)

        self.path_pub = self.create_publisher(Path, '/gate/path', 1)
        self.pair_pub = self.create_publisher(MarkerArray, '/gate/pairs', 1)
        self.status_pub = self.create_publisher(String, '/gate/status', 10)
        # 진전 없이 정체 → mission_manager 에 gate 건너뛰기 요청 (전체 실행에서만 받아 줌)
        self.give_up_pub = self.create_publisher(String, '/mission/give_up', 10)
        self.preview_pub = self.create_publisher(Twist, '/gate/cmd_preview', 10)
        # 부표 쌍 찾기는 스캔 점을 바로 쓰므로 배가 수평일 때만 오는 /scan/level 을 받는다
        self.create_subscription(LaserScan, '/scan/level', self._on_scan,
                                 qos_profile_sensor_data)
        self.get_logger().info(
            f'gate LiDAR/DRI: required_pairs={self.gate_params.required_pairs}, '
            f'thrust_ratio={self.thrust_ratio:.2f}, '
            f'los_thrust_ratio={self.los_thrust_ratio:.2f}, '
            f'los_turn_ratio={self.los_turn_ratio:.2f}, '
            f'motion_enabled={self.motion_enabled}, 게이트 사슬 {p.max_chain_gates}개, '
            f'부표 기억 {p.memory_confirm_hits}번 잡히면 확정')

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
        self._passed_directions = []   # 통과한 게이트 진행 방향 — 다음 쌍 기울기 기준
        self._status_text = None
        self._stall_since = None
        self._rejected = []
        self._chain = []
        self._hold_point = None
        self._memory.reset()
        self._memory_grid = None
        self._extend_block_until = 0.0
        self._replan_block_until = 0.0
        self._wait_since = None
        self._seek_origin = None
        self._seek_note = None
        self._guard_since = None
        self._guard_expired = False
        self._guard_hint = None
        self._guard_arrived = False
        self._stall_acc = 0.0        # 마지막 게이트 통과 뒤 진행 아닌 상태로 보낸 시간 [s]
        self._stall_last = None
        self._stall_passed = 0
        self._give_up_last = None

    def on_deactivate(self):
        self._pair = None
        self._path = None
        self._previous_xy = None
        self._crossed = False
        self._stall_since = None
        self._chain = []

    def _on_grid(self, msg: OccupancyGrid):
        super()._on_grid(msg)
        self._grid_at = time.monotonic()

    def _on_scan(self, msg: LaserScan):
        if self.waypoint_odom_ready():
            pose = self.odom.pose.pose
            hits = scan_hits_world(
                msg, (pose.position.x, pose.position.y),
                yaw_from_quaternion(pose.orientation), self.lidar_offset)
            if len(hits) == 0:
                return
            self._scan_hits = hits
            self._scan_rays = scan_rays(
                msg, (pose.position.x, pose.position.y),
                yaw_from_quaternion(pose.orientation), self.lidar_offset)
            self._scan_at = time.monotonic()

    def _status(self, value):
        if value != self._status_text:
            self._status_text = value
            self.status_pub.publish(String(data=value))
            self.get_logger().info(f'gate: {value}')

    def _stop(self, reason, keep_previous=False, hold=False):
        """정지. hold 면 추진 0 대신 멈춘 자리·선수각을 hold_thrust_ratio 로 지킨다(바람 표류)."""
        if not keep_previous:
            self._previous_xy = None  # no crossing is inferred across sensor gaps
        self._status(reason)
        self._publish_debug()
        if hold and self.hold_thrust > 0.0 and self.waypoint_odom_ready():
            return self._output(self._hold_cmd())
        self.preview_pub.publish(Twist())
        return Twist()

    def _hold_cmd(self):
        """멈춘 자리·선수각 지키기 — 선수각 PD + 선수 방향 앞뒤 거리만큼 전진/후진.

        옆으로 밀린 것은 고치지 않는다(돌아서 쫓아가면 부표가 뒤로 가서 다시 막힘 — 시뮬 맞바람).
        단위는 LOS 와 같은 추력 비율 (waypoint() 는 전체 스케일 max_linear 0.1 이라 너무 약함).
        """
        pose = self.odom.pose.pose
        x, y = pose.position.x, pose.position.y
        yaw = yaw_from_quaternion(pose.orientation)
        if self._hold_point is None:
            self._hold_point = (x, y, yaw)
        hx, hy, hyaw = self._hold_point
        along = (hx - x) * math.cos(hyaw) + (hy - y) * math.sin(hyaw)   # + = 지킬 자리가 앞
        err = math.atan2(math.sin(hyaw - yaw), math.cos(hyaw - yaw))
        turn = max(-self.los_turn_ratio, min(self.los_turn_ratio,
                   HOLD_YAW_KP * err - HOLD_YAW_KD * self.odom.twist.twist.angular.z))
        push = max(-self.hold_thrust, min(self.hold_thrust, HOLD_ALONG_KP * along))
        if abs(err) > math.radians(45.0):
            push = 0.0
        self._command_full_scale = (self.hold_thrust, self.los_turn_ratio)
        cmd = Twist()
        cmd.linear.x = push
        cmd.angular.z = turn
        return cmd

    def _wait_first_pair(self, boat, yaw, dri, posts, goal, now):
        """쌍을 못 골랐을 때. 첫 게이트면 seek_first_after_sec 뒤 첫 쌍 쪽으로 천천히 다가가고,
        그 밖(두 번째 게이트부터·한도·막힘)은 예전처럼 서서 기다린다."""
        if self._passed_midpoints or self.seek_after <= 0.0 or self.seek_thrust <= 0.0:
            return self._stop('WAIT_GATE_PAIR', hold=True)
        if self._wait_since is None:
            self._wait_since = now
        if now - self._wait_since < self.seek_after:
            return self._stop('WAIT_GATE_PAIR', hold=True)
        hint = self._guard_hint
        if hint is not None and now - hint[2] <= 1.5:
            cmd, status, why = self._guard_command(boat, yaw, dri, posts, hint)
            if cmd is not None:
                self._seek_note = None
                self._hold_point = None
                self._status(status)
                self._publish_debug()
                return self._output(cmd)
        else:
            self._guard_arrived = False
            cmd, why = self._seek_command(boat, yaw, dri, posts, goal)
        if cmd is None:
            if why != self._seek_note:
                self._seek_note = why
                self.get_logger().info(
                    f'gate: 첫 쌍 찾아 다가가기 멈춤 — {why}; 서서 기다림 (배 {_xy(boat)})')
            return self._stop('WAIT_GATE_PAIR', hold=True)
        self._seek_note = None
        self._hold_point = None
        self._status('SEEK_FIRST_PAIR')
        self._publish_debug()
        return self._output(cmd)

    def _first_guard(self, pair, boat, now):
        """첫 게이트 지키기 — 이 쌍을 받지 말아야 하면 그 이유(짝 없는 부표 위치), 아니면 None."""
        if (self._passed_midpoints or self.first_guard_margin <= 0.0 or self._guard_expired
                or pair is None):
            return None
        lone = nearer_lone_post(pair, boat, self._memory.confirmed(), self.first_guard_margin,
                                self.gate_params.memory_match_radius)
        if lone is None:
            return None
        if self._guard_since is None:
            self._guard_since = now
        if now - self._guard_since > self.first_guard_max_sec:
            self._guard_expired = True
            self.get_logger().warning(
                f'gate: 첫 게이트 지키기 {self.first_guard_max_sec:.0f} s 넘음 — 짝 없는 부표 '
                f'{_xy(lone)} 의 짝이 안 잡혀 지키기를 풀고 쌍 {_xy(pair.midpoint)} 로 감 (배 {_xy(boat)})')
            return None
        normal = np.asarray(pair.direction, dtype=float)
        before = float((np.asarray(pair.midpoint) - np.asarray(lone)) @ normal)
        # 게이트 1 중점 짐작 = 게이트 2 중점을 짝 없는 부표 줄까지 당긴 점 — 배가 한쪽으로 치우쳐
        # 있으면 곧장 다가가도 반대쪽 짝 부표가 계속 멀다(시뮬: 2 m 치우친 출발)
        guess = np.asarray(pair.midpoint, dtype=float) - before * normal
        self._guard_hint = ((float(guess[0]), float(guess[1])),
                            (float(normal[0]), float(normal[1])), now)
        return (f'쌍 {_xy(pair.midpoint)} 보다 {before:.1f} m 앞에 짝 없는 부표 {_xy(lone)} — '
                f'게이트 1 한쪽만 잡힌 것으로 보고 그 쌍(게이트 2)을 안 받음, 게이트 1 중점 짐작 '
                f'{_xy(guess)}')

    def _guard_command(self, boat, yaw, dri, posts, hint):
        """첫 게이트 지키기 중 — 짐작한 게이트 1 중점 seek_close_stop_along 앞(통로 가운데) 대기점으로
        가서 게이트 1 을 보고 선다. 물살에 GUARD_RESUME 넘게 밀리면 다시 간다.

        → (Twist, 상태, '') 또는 (None, '', 못 가는 이유). 곧장 진행선을 따라 다가가면 배가 한쪽으로
        치우친 채 멈춰 반대쪽 짝 부표가 계속 멀었다 — 조류 0.1 m/s 시뮬에서 15판 모두 못 잡음.
        """
        p = self.gate_params
        mid = np.asarray(hint[0], dtype=float)
        normal = np.asarray(hint[1], dtype=float)
        target = mid - self.seek_close_stop * normal
        rel = target - np.asarray(boat, dtype=float)
        distance = float(np.linalg.norm(rel))
        if distance > GUARD_TARGET_MAX:
            return None, '', f'게이트 1 앞 대기점 {_xy(target)} 이 {distance:.1f} m — 너무 멂'
        self._guard_arrived = distance <= (GUARD_RESUME if self._guard_arrived else GUARD_ARRIVE)
        if self._guard_arrived:
            face = mid - np.asarray(boat, dtype=float)       # 서서 게이트 1 을 바라봄
            heading = math.atan2(face[1], face[0])
            forward = 0.0
            status = 'WAIT_GATE_PAIR'
        else:
            heading = math.atan2(rel[1], rel[0])
            direction = rel / max(distance, 1e-6)
            off = np.asarray(posts, dtype=float).reshape(-1, 2) - np.asarray(boat, dtype=float)
            ahead, side = off @ direction, off @ np.array([-direction[1], direction[0]])
            if np.any((ahead > -0.3) & (ahead < min(self.seek_stop_ahead, distance + 0.5))
                      & (np.abs(side) < 1.0)):
                return None, '', '대기점 가는 길 바로 앞에 부표'
            steps = np.arange(0.0, min(2.5, distance) + 1e-9, p.sample_spacing)
            points = np.asarray(boat, dtype=float) + steps[:, None] * direction
            if not points_safe(dri, points, p, start=boat if self.replan_from_here else None):
                return None, '', '대기점 가는 길 안전 검사에 걸림'
            forward = None
            status = 'SEEK_FIRST_PAIR'
        err = math.atan2(math.sin(heading - yaw), math.cos(heading - yaw))
        turn = max(-self.los_turn_ratio, min(self.los_turn_ratio,
                   HOLD_YAW_KP * err - HOLD_YAW_KD * self.odom.twist.twist.angular.z))
        cmd = Twist()
        cmd.linear.x = (forward if forward is not None
                        else self.seek_thrust if abs(err) < math.radians(30.0) else 0.0)
        cmd.angular.z = turn
        self._command_full_scale = (max(self.seek_thrust, 1e-6), self.los_turn_ratio)
        self.get_logger().info(
            f'gate: 첫 게이트 지키기 — 게이트 1 앞 대기점 {_xy(target)} '
            f'{"에서 게이트 1 을 보며 짝 부표 기다림" if self._guard_arrived else f"으로 감 ({distance:.1f} m)"}'
            f' (배 {_xy(boat)})', throttle_duration_sec=1.0)
        return cmd, status, ''

    def _seek_command(self, boat, yaw, dri, posts, goal):
        """첫 쌍이 잡힐 쪽으로 천천히 다가가는 명령 → (Twist, '') 또는 (None, 멈춘 이유).

        앞 seek_wait_within(4.5 m) 안에 부표가 보이면 가장 가까운 것이 seek_close_stop_along(2.0 m)
        앞에 올 때까지만 다가간다 (2026-11-14). 예전엔 4.5 m 안에 보이면 바로 섰다 — 그때 다가가면
        게이트 1 이 쌍으로 잡히기 전에 게이트 2 가 먼저 쌍이 되어 첫 게이트를 건너뛰었다 (녹화 기반
        시뮬, 첫 게이트 3.7 m 시작 50판 중 1 → 5). 지금은 그 건너뛰기를 첫 게이트 지키기
        (nearer_lone_post)가 막는다. 앞 창 전체를 기준으로 하면 비스듬한 게이트 1 의 한쪽 부표만
        창(6 m) 안에 걸려 영영 기다렸다(6 m 시작).
        방향: 배 → gate_end 진행선. 4.5 m 너머 ~ 앞 창 + SEEK_LOOK_BEYOND 의 지도 부표가 둘 이상
        보이면 그 가운데 쪽으로 기울인다(±seek_max_bias). 첫 게이트 지키기 중에는 이것 대신
        _guard_command (게이트 1 앞 대기점).
        멈춤: 가장 가까운 부표가 2.0 m 앞 / 시작 자리에서 seek_max_distance 넘게 옴 / 앞
        seek_stop_ahead 안(옆 포함)에 부표 / 앞 2.5 m 안전 검사(DRI·부표 min_clearance)에 걸림.
        """
        p = self.gate_params
        axis = np.asarray(goal, dtype=float) - np.asarray(boat, dtype=float)
        length = float(np.linalg.norm(axis))
        if length < 1e-6:
            return None, 'gate_end 위'
        axis /= length
        side_axis = np.array([-axis[1], axis[0]])
        if self._seek_origin is None:
            self._seek_origin = boat
        moved = math.dist(boat, self._seek_origin)
        if moved >= self.seek_max_distance:
            return None, f'시작 자리에서 {moved:.1f} m 옴 (한도 {self.seek_max_distance:.1f} m)'
        rel = np.asarray(posts, dtype=float).reshape(-1, 2) - np.asarray(boat, dtype=float)
        along, side = rel @ axis, rel @ side_axis
        near = ((along > -0.5) & (along < self.seek_stop_ahead)
                & (np.abs(side) <= p.max_gate_width / 2.0))
        if near.any():
            k = int(np.argmin(np.where(near, np.hypot(along, side), np.inf)))
            return None, (f'앞 {along[k]:.1f} m·옆 {side[k]:+.1f} m 에 부표 '
                          f'(seek_stop_ahead {self.seek_stop_ahead:.1f} m)')
        lateral_ok = np.abs(side) <= p.max_midpoint_lateral + p.max_gate_width / 2.0
        close = (along >= p.min_gate_forward) & (along <= self.seek_wait_within) & lateral_ok
        if close.any():
            nearest = float(np.min(along[close]))
            if nearest <= self.seek_close_stop:
                return None, (f'앞 {nearest:.1f} m 에 부표 (seek_close_stop_along '
                              f'{self.seek_close_stop:.1f} m) — 쌍이 잡히길 기다림')
        beyond = ((along > self.seek_wait_within)
                  & (along <= p.max_gate_forward + SEEK_LOOK_BEYOND) & lateral_ok)
        bias = 0.0
        if int(beyond.sum()) >= 2:
            bias = math.atan2(float(np.mean(side[beyond])), float(np.mean(along[beyond])))
            bias = max(-self.seek_max_bias, min(self.seek_max_bias, bias))
        base = math.atan2(axis[1], axis[0])
        heading = None
        for candidate in ((base + bias, base) if bias else (base,)):
            direction = np.array([math.cos(candidate), math.sin(candidate)])
            steps = np.arange(0.0, 2.5 + 1e-9, p.sample_spacing)
            points = np.asarray(boat, dtype=float) + steps[:, None] * direction
            if points_safe(dri, points, p, start=boat if self.replan_from_here else None):
                heading = candidate
                break
        if heading is None:
            return None, '앞 2.5 m 안전 검사에 걸림'
        err = math.atan2(math.sin(heading - yaw), math.cos(heading - yaw))
        turn = max(-self.los_turn_ratio, min(self.los_turn_ratio,
                   HOLD_YAW_KP * err - HOLD_YAW_KD * self.odom.twist.twist.angular.z))
        cmd = Twist()
        cmd.linear.x = self.seek_thrust if abs(err) < math.radians(30.0) else 0.0
        cmd.angular.z = turn
        self._command_full_scale = (self.seek_thrust, self.los_turn_ratio)
        self.get_logger().info(
            f'gate: SEEK_FIRST_PAIR — 첫 쌍 쪽으로 다가감: 방향 {math.degrees(heading):.0f}° '
            f'(gate_end 쪽에서 {math.degrees(heading - base):+.0f}°, 먼 부표 '
            f'{int(beyond.sum())}개), '
            f'시작 자리에서 {moved:.1f}/{self.seek_max_distance:.1f} m (배 {_xy(boat)})',
            throttle_duration_sec=1.0)
        return cmd, ''

    def _posts_visible(self, now):
        """지금 게이트 두 기둥이 보이는지 — 각각 post_visible_timeout 안에 한 번 잡혔으면."""
        if self.post_visible_timeout <= 0.0:
            return pair_observed(self._pair, self._scan_hits, self.gate_params)
        for i, post in enumerate((self._pair.left, self._pair.right)):
            if self._post_seen(post):
                self._post_seen_at[i] = now
        return all(t is not None and now - t <= self.post_visible_timeout
                   for t in self._post_seen_at)

    def _gate_ahead(self, boat):
        """배 중심에서 현재 게이트선까지 진행 방향 거리 [m] (지나쳤으면 음수)."""
        mid = np.asarray(self._pair.midpoint)
        return float((mid - np.asarray(boat)) @ np.asarray(self._pair.direction))

    def _drop_pair(self):
        self._pair = None
        self._path = None
        self._chain = []
        self._previous_xy = None
        self._crossed = False
        self._stall_since = None
        self._los_follower.reset()

    def _post_seen(self, post) -> bool:
        """기둥 하나가 지금 스캔(scan_match_radius 안)에 보이는지 — 진단 로그용."""
        hits = np.asarray(self._scan_hits, dtype=float)
        if hits.ndim != 2 or len(hits) == 0:
            return False
        return bool(np.any(np.sum((hits - np.asarray(post)) ** 2, axis=1)
                           <= self.gate_params.scan_match_radius ** 2))

    def _record_pass(self, how):
        n = len(self._passed_midpoints) + 1
        crossing = getattr(self, '_crossing', None)
        self.get_logger().info(
            f'gate: 게이트 {n}/{self.gate_params.required_pairs} 통과 ({how}) — 중점 '
            f'{_xy(self._pair.midpoint)}, 가로지른 점 '
            f'{_xy(crossing) if crossing is not None else "없음"}')
        self._passed_midpoints.append(self._pair.midpoint)
        self._passed_directions.append(self._pair.direction)

    def _mark_passed(self, how='경로 끝 도달'):
        self._record_pass(how)
        self._drop_pair()
        passed = len(self._passed_midpoints)
        self._status(f'GATE_{passed}/{self.gate_params.required_pairs}_PASSED')

    def _set_current(self, pair):
        """사슬의 다음 칸을 지금 게이트로 — 경로·LOS 진행은 그대로 둔다."""
        self._pair = pair
        now = time.monotonic()
        self._post_seen_at = [now, now]
        self._crossed = False
        self._crossing = None
        self._stall_since = None

    def _advance_chain(self, boat):
        """사슬 안에서 지금 게이트를 기둥 사이로 지남 → 통과로 세고 바로 다음 칸으로."""
        self._record_pass('기둥 사이 통과 — 이어 둔 다음 게이트로')
        self._chain.pop(0)
        self._set_current(self._chain[0])
        self._previous_xy = boat

    def _extend_chain(self, boat, dri, grid, posts, goal):
        """기억한 부표로 사슬 끝 게이트의 다음 게이트를 미리 골라 경로를 이어 둔다.

        지금 게이트를 이미 지났으면(경로 끝 1 m 를 가는 중) 그 게이트는 통과로 세고 배에서
        다음 게이트로 바로 잇는다. 이은 경로가 검사(앞 check_horizon m)에 걸리면 잇지 않는다.
        """
        p = self.gate_params
        if (self._pair is None or not self._chain
                or time.monotonic() < self._extend_block_until
                or len(self._chain) - int(self._crossed) >= p.max_chain_gates
                or len(self._passed_midpoints) + len(self._chain) >= p.required_pairs):
            return False
        last = self._chain[-1]
        before = [*self._passed_midpoints, *(q.midpoint for q in self._chain[:-1])]
        # 버린 쌍 보류(_rejected)는 여기 적용하지 않는다. 보류는 "배 자리에서 그 쌍으로 가는
        # 길이 막혔다"는 뜻인데, 사슬은 앞 게이트를 거쳐 가므로 그 이유가 없다. 적용하면 바로
        # 그 게이트를 건너뛴다: 출발 때 게이트 1 이 아직 안 잡혀 게이트 2 를 먼저 골랐다가
        # (게이트 1 부표에 막혀) 버림 → 게이트 1 을 고른 뒤 사슬이 보류 중인 게이트 2 를 빼고
        # 게이트 3 을 "2번"으로 이음 → 7개를 다 지나고도 6/7 로 끝없이 대기 (2026-11-13
        # 녹화 기반 시뮬, 라이다 약함·위치 잡음 3판; 만료된 보류도 경로가 있는 동안 안 지워졌다).
        nxt = next_gate_after(posts, last, goal, before, p)
        if nxt is None:
            return False
        chain = [*self._chain[1:], nxt] if self._crossed else [*self._chain, nxt]
        path = make_chain_path(boat, chain, p)
        if not points_safe(dri, path_ahead(path, 0, p.check_horizon), p):
            self.get_logger().info(
                f'gate: 다음 게이트 {_xy(nxt.midpoint)} 는 찾았지만 이은 경로가 안전 검사에 걸려 '
                f'아직 안 이음 (배 {_xy(boat)})', throttle_duration_sec=1.0)
            return False
        shift = np.asarray(nxt.midpoint) - np.asarray(last.midpoint)
        d = np.asarray(last.direction)
        self.get_logger().info(
            f'gate: 게이트 {len(self._passed_midpoints) + len(self._chain) + 1}/'
            f'{p.required_pairs} 미리 잡음 — 왼 {_xy(nxt.left)}, 오 {_xy(nxt.right)}, '
            f'중점 {_xy(nxt.midpoint)}, 폭 {nxt.width:.2f} m, 앞 게이트에서 앞 '
            f'{float(shift @ d):.1f} m·옆 {float(shift @ np.array([-d[1], d[0]])):+.1f} m; '
            f'배 {_xy(boat)}, 사슬 {len(chain)}개')
        if self._crossed:
            self._record_pass('기둥 사이 통과 — 다음 게이트로 바로 이음')
            self._set_current(chain[0])
            self._previous_xy = boat
        self._chain = chain
        self._path = path
        self._los_follower.reset()   # 새 경로 — 진행 위치를 옛 경로 번호로 검사하지 않게
        return True

    def _stall(self, reason, now):
        """같은 쌍으로 진행 못 함 — replan_timeout 넘게 이어지면 쌍을 놓고 다시 고른다.

        예전엔 경로가 막히거나 기둥이 안 보이면 같은 쌍·경로를 계속 다시 검사해
        영원히 정지했다.
        """
        if self._stall_since is None:
            self._stall_since = now
        elif now - self._stall_since >= self.replan_timeout:
            if self._crossed:
                # 게이트선은 실제로 넘었다 — 통과로 인정하고 다음 쌍으로
                self.get_logger().warn(
                    f'gate: {reason} after crossing; counting the gate as passed')
                self._mark_passed('가로지른 뒤 막혀서 인정')
                self._publish_debug()
                self.preview_pub.publish(Twist())
                return Twist()
            self.get_logger().warn(
                f'gate: {reason} for {now - self._stall_since:.1f} s; '
                f'dropping pair at ({self._pair.midpoint[0]:.2f}, '
                f'{self._pair.midpoint[1]:.2f}) and re-selecting '
                f'(기둥 {_xy(self._pair.left)}·{_xy(self._pair.right)}, 폭 {self._pair.width:.2f} m, '
                f'{self.reject_hold:.0f} s 동안 후보에서 뺌)')
            self._rejected.append((self._pair.midpoint, now + self.reject_hold))
            self._drop_pair()
            return self._stop(reason, hold=True)
        # 센서는 살아 있다(odom 정상) — 정지 중에도 게이트선 통과는 계속 판정한다
        return self._stop(reason, keep_previous=True, hold=True)

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
        # 미리 이어 둔 다음 게이트들 — 노란 공 (중점은 조금 작게)
        for k, pair in enumerate(self._chain[1:]):
            for i, point in enumerate((pair.left, pair.right, pair.midpoint)):
                marks.markers.append(self._sphere(
                    path.header, 'gate_chain', 3 * k + i, point,
                    0.2 if i == 2 else 0.3, (1.0, 0.85, 0.1)))
        # 기억한 부표 — 확정 흰 기둥, 확정 전 회색 (/gate/pairs 에 같이 냄, RViz 설정 그대로)
        for i, post in enumerate(self._memory.posts):
            marker = Marker()
            marker.header = path.header
            marker.ns, marker.id = 'gate_memory', i
            marker.type, marker.action = Marker.CYLINDER, Marker.ADD
            marker.pose.position.x = float(post.xy[0])
            marker.pose.position.y = float(post.xy[1])
            marker.pose.position.z = 0.1
            marker.pose.orientation.w = 1.0
            marker.scale.x = marker.scale.y = 0.25
            marker.scale.z = 0.2
            shade = 1.0 if post.confirmed else 0.5
            marker.color.r = marker.color.g = marker.color.b = shade
            marker.color.a = 0.8
            marks.markers.append(marker)
        self.pair_pub.publish(marks)

    @staticmethod
    def _sphere(header, ns, marker_id, point, size, rgb):
        marker = Marker()
        marker.header = header
        marker.ns, marker.id = ns, marker_id
        marker.type, marker.action = Marker.SPHERE, Marker.ADD
        marker.pose.position.x = float(point[0])
        marker.pose.position.y = float(point[1])
        marker.pose.position.z = 0.3
        marker.pose.orientation.w = 1.0
        marker.scale.x = marker.scale.y = marker.scale.z = size
        marker.color.r, marker.color.g, marker.color.b = rgb
        marker.color.a = 1.0
        return marker

    def _goal_leg(self, boat_xy, goal_xy):
        """배 → gate_end 방향 앞 2 m 의 점들."""
        dx = goal_xy[0] - boat_xy[0]
        dy = goal_xy[1] - boat_xy[1]
        distance = math.hypot(dx, dy)
        length = min(2.0, distance)
        n = max(2, int(math.ceil(length / self.gate_params.sample_spacing)) + 1)
        return [(boat_xy[0] + t * dx / max(distance, 1e-9),
                 boat_xy[1] + t * dy / max(distance, 1e-9))
                for t in (length * k / (n - 1) for k in range(n))]

    def _goal_leg_safe(self, dri, boat_xy, goal_xy):
        if math.dist(boat_xy, goal_xy) > self.gate_params.max_goal_after_gates:
            return False
        return points_safe(dri, self._goal_leg(boat_xy, goal_xy), self.gate_params)

    # 진행으로 보는 상태 — 나머지(WAIT_GATE_PAIR·PATH_UNSAFE·LOOKAHEAD_UNSAFE·GATE_NOT_CROSSED·센서 끊김 …)는 정체
    PROGRESS_STATUSES = ('FOLLOW_GATE', 'GOAL_APPROACH', 'SEEK_FIRST_PAIR', 'COMPLETE')

    def compute_cmd(self):
        cmd = self._compute_cmd()
        self._track_stall(time.monotonic())
        return cmd

    def _track_stall(self, now):
        """정체 시간을 쌓아 give_up_stall_sec 넘으면 /mission/give_up 'gate: …' 을 1 s 마다.

        게이트를 하나 지나면 0 부터 다시. 단일 미션 시험·mission_manager 가 없으면 받는 쪽이 무시하고
        gate 는 계속 시도한다.
        """
        passed = len(self._passed_midpoints)
        if passed != self._stall_passed:
            self._stall_passed, self._stall_acc = passed, 0.0
        dt = 0.0 if self._stall_last is None else min(max(now - self._stall_last, 0.0), 0.5)
        self._stall_last = now
        status = self._status_text or ''
        if status.startswith(self.PROGRESS_STATUSES) or status.endswith('PASSED'):
            return
        self._stall_acc += dt
        if not self.give_up_stall or self._stall_acc < self.give_up_stall:
            return
        if self._give_up_last is None or now - self._give_up_last >= 1.0:
            self._give_up_last = now
            reason = (f'{passed}/{self.gate_params.required_pairs} gates passed, no progress for '
                      f'{self._stall_acc:.0f}s (last {status or "none"})')
            self.give_up_pub.publish(String(data=f'gate: {reason}'))
            self.get_logger().warning(f'gate: 포기 — {reason}', throttle_duration_sec=5.0)

    def _compute_cmd(self):
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
        current = grid_posts(grid, self.gate_params)
        if grid is not self._memory_grid:          # 새 지도가 왔을 때만 한 번 잡힌 것으로 센다
            self._memory.update(current, now, boat)
            self._memory_grid = grid
        posts = self._memory.with_current(current)

        if len(self._passed_midpoints) >= self.gate_params.required_pairs:
            if self.distance_to_goal() <= 2.0:
                self.report_complete()
                self._status('COMPLETE')
                self._publish_debug()
                return self._output(Twist())
            if not self._goal_leg_safe(dri, boat, goal):
                return self._stop('GOAL_LEG_UNSAFE_OR_TOO_FAR', hold=True)
            cmd, _ = self.waypoint(*goal, thrust_ratio=self.thrust_ratio,
                                   slow_radius=3.0)
            if self._slow_if_unobserved(cmd, grid, self._goal_leg(boat, goal), boat):
                self._status('GOAL_APPROACH_SLOW')
                self._hold_point = None
                self._publish_debug()
                return self._output(cmd)
            self._hold_point = None
            self._status('GOAL_APPROACH')
            self._publish_debug()
            return self._output(cmd)

        # 만료된 보류는 경로가 있는 동안에도 지운다 (예전엔 경로 없을 때만 지워 남아 있었다)
        self._rejected = [(m, until) for m, until in self._rejected if until > now]
        if self._path is None:
            stats = {}
            pair = select_gate_pair_from_posts(
                posts, boat, goal, self._passed_midpoints, self.gate_params,
                excluded_midpoints=[m for m, _ in self._rejected],
                last_direction=(self._passed_directions[-1]
                                if self._passed_directions else None),
                stats=stats)
            if pair is None:
                self.get_logger().info(
                    f'gate: WAIT_GATE_PAIR — 쌍 없음: {describe_reasons(stats)}; '
                    f'보류 중 쌍 {len(self._rejected)}, 통과 {len(self._passed_midpoints)}, '
                    f'배 {_xy(boat)}', throttle_duration_sec=1.0)
                return self._wait_first_pair(boat, yaw, dri, posts, goal, now)
            guard = self._first_guard(pair, boat, now)
            if guard is not None:
                self.get_logger().info(f'gate: WAIT_GATE_PAIR — {guard} (배 {_xy(boat)})',
                                       throttle_duration_sec=1.0)
                return self._wait_first_pair(boat, yaw, dri, posts, goal, now)
            if not pair_observed(pair, self._scan_hits, self.gate_params):
                seen = [self._post_seen(post) for post in (pair.left, pair.right)]
                self.get_logger().info(
                    f'gate: WAIT_GATE_PAIR — 쌍 {_xy(pair.midpoint)} 기둥이 지금 스캔에 안 보임 '
                    f'(왼 {_xy(pair.left)} {"보임" if seen[0] else "안 보임"}, '
                    f'오 {_xy(pair.right)} {"보임" if seen[1] else "안 보임"}; 지도에만 남은 것)',
                    throttle_duration_sec=1.0)
                return self._wait_first_pair(boat, yaw, dri, posts, goal, now)
            self._wait_since = None
            d = pair.direction
            self.get_logger().info(
                f'gate: 게이트 {len(self._passed_midpoints) + 1}/'
                f'{self.gate_params.required_pairs} 쌍 선택 — 왼 {_xy(pair.left)}, '
                f'오 {_xy(pair.right)}, 중점 {_xy(pair.midpoint)}, 폭 {pair.width:.2f} m, '
                f'기울기 {stats.get("chosen_skew_deg") or 0.0:.0f}°, 진행 방향 '
                f'{round(math.degrees(math.atan2(d[1], d[0]))) + 0:d}°, 배 {_xy(boat)}; '
                f'{describe_reasons(stats)}')
            self._crossing = None
            self._pair = pair
            self._post_seen_at = [now, now]
            self._chain = [pair]
            self._path = make_gate_path(boat, pair, self.gate_params)
            self._previous_xy = boat
            self._crossed = False
            self._stall_since = None
            self._los_follower.reset()

        # 통과 판정을 기둥 보임 확인보다 먼저 한다 — 중심이 선을 넘는 순간 기둥이
        # 라이다 뒤쪽 가림 구간에 들어가도 통과를 놓치지 않게.
        self._check_crossing(boat)
        # 다음 게이트를 기억한 부표로 미리 골라 경로를 이어 둔다 (게이트 사슬, max_chain_gates 까지)
        while self._extend_chain(boat, dri, grid, posts, goal):
            pass
        if self._crossed and len(self._chain) > 1:
            self._advance_chain(boat)
        if self._drop_diagonal(boat, posts):
            return self._stop('WAIT_GATE_PAIR', hold=True)    # 다음 틱에 다시 고름
        if not self._crossed:
            # 따라가던 첫 쌍 앞에 짝 없는 부표가 새로 확정됨 → 게이트 2 를 고른 것 (고를 때는 아직
            # 확정 전이었음). 버리고 게이트 1 쪽으로 다가가며 쌍을 기다린다.
            guard = self._first_guard(self._pair, boat, now)
            if guard is not None:
                self.get_logger().info(f'gate: 따라가던 쌍 버림 — {guard} (배 {_xy(boat)})')
                self._drop_pair()
                return self._wait_first_pair(boat, yaw, dri, posts, goal, now)

        if (not self._crossed
                and self._gate_ahead(boat) > self.visibility_release
                and not self._posts_visible(now)):
            return self._stall('GATE_POSTS_NOT_VISIBLE', now)

        problem = self._path_problem(dri)
        if problem is not None and len(self._chain) > 1:
            # 미리 이은 부분이 걸림 — 지금 게이트를 버리지 말고 지금 게이트만 가는 경로로 줄인다
            self.get_logger().info(
                f'gate: 이은 경로 {problem} — 다음 게이트 이음 풀고 게이트 '
                f'{len(self._passed_midpoints) + 1} 만 감 (배 {_xy(boat)})')
            self._chain = self._chain[:1]
            self._path = make_gate_path(boat, self._pair, self.gate_params)
            self._los_follower.reset()
            self._extend_block_until = now + 1.0
            problem = self._path_problem(dri)
        if problem is not None and self._replan(boat, dri, now, problem):
            problem = self._path_problem(dri)
        if problem is not None:
            return self._stall(problem, now)
        return self._follow(boat, dri, grid, now)

    def _drop_diagonal(self, boat, posts):
        """고른 뒤 새로 잡힌 부표로 보니 대각선 오쌍인 게이트를 사슬에서 뺀다.

        쌍을 고를 때 네 번째 부표가 아직 안 잡혔으면 대각선 검사를 못 한다 — 검출이 약할 때
        게이트 1 오른 부표와 게이트 2 왼 부표를 한 게이트로 골라 게이트 하나를 건너뛰었다(시뮬).
        지금 게이트는 게이트선 1.5 m 앞까지만 다시 본다(가까우면 배 근처 잡음이 끼어듦).
        버린 쌍은 보류에 넣지 않는다 — 진짜 쌍을 바로 다시 고르게.
        """
        p = self.gate_params
        for i, pair in enumerate(self._chain):
            if i == 0 and (self._crossed or self._gate_ahead(boat) < 1.5):
                continue
            if not is_diagonal(pair.left, pair.right, posts, p, p.memory_match_radius):
                continue
            self.get_logger().info(
                f'gate: 게이트 {len(self._passed_midpoints) + i + 1} {_xy(pair.midpoint)} 는 새로 잡힌 '
                f'부표로 보니 대각선 오쌍 — {"다시 고름" if i == 0 else "사슬에서 뺌"} (배 {_xy(boat)})')
            if i == 0:
                self._drop_pair()
                return True
            self._chain = self._chain[:i]
            self._path = make_chain_path(boat, self._chain, p)
            self._los_follower.reset()
            return False
        return False

    def _replan(self, boat, dri, now, reason):
        """막힘 — 지금 자리에서 지금 게이트(와 이어 둔 게이트)로 경로를 새로 그린다.

        바람에 밀려 경로에서 벗어나면 배→경로 직선이 부표를 스쳐 막혔다. 서면(추진 0) 더 밀리므로
        먼저 지금 자리에서 다시 그려 본다. 게이트선을 이미 넘었으면 안 한다(통과 판정 유지).
        """
        if (not self.replan_from_here or self._crossed or now < self._replan_block_until
                or not self._chain):
            return False
        self._replan_block_until = now + 1.0
        p = self.gate_params
        for chain in (self._chain, self._chain[:1]):
            path = make_chain_path(boat, chain, p)
            if points_safe(dri, path_ahead(path, 0, p.check_horizon), p, start=boat):
                self.get_logger().info(
                    f'gate: {reason} — 지금 자리에서 게이트 {len(self._passed_midpoints) + 1} '
                    f'경로 다시 그림 (배 {_xy(boat)}, 사슬 {len(chain)}개)')
                self._chain = list(chain)
                self._path = path
                self._los_follower.reset()
                self._stall_since = None
                return True
        return False

    def _path_problem(self, dri):
        """경로 앞 check_horizon m 의 안전(장애물 min_clearance·DRI). 관측은 _slow_if_unobserved."""
        progress = self._los_follower.progress_idx or 0
        pose = self.odom.pose.pose
        start = (pose.position.x, pose.position.y) if self.replan_from_here else None
        if not path_safe(dri, self._path, self.gate_params, progress,
                         self.gate_params.check_horizon, start=start):
            return 'PATH_UNSAFE'
        return None

    def _slow_if_unobserved(self, cmd, grid, points, boat):
        """points 중 안 본 칸(지도 unknown 이고 지금 빔도 안 지남)이 있으면 직진을 줄인다.

        안 본 칸은 장애물이 아니다(장애물이면 지도에 점유 → 안전 검사가 거름). 서지 않고
        unobserved_thrust_scale 배로 계속 가면 다가가며 빔이 그 칸을 본다.
        """
        if points_observed_or_seen(grid, points, self._scan_rays, self.gate_params):
            return False
        cmd.linear.x *= self.gate_params.unobserved_thrust_scale
        self.get_logger().info(
            f'gate: 앞 {self.gate_params.observed_horizon:.0f} m 에 안 본 칸 — 직진 '
            f'{self.gate_params.unobserved_thrust_scale:.0%} 로 계속 (배 {_xy(boat)})',
            throttle_duration_sec=1.0)
        return True

    def _check_crossing(self, boat):
        if not self._crossed and self._previous_xy is not None:
            hit = gate_crossing(self._previous_xy, boat, self._pair)
            if hit is not None:
                point, offset = hit
                limit = self._pair.width / 2.0 - self.gate_params.min_clearance
                inside = offset <= limit
                self.get_logger().info(
                    f'gate: 게이트선 가로지름 {_xy(point)} — 중점에서 옆 {offset:.2f} m '
                    f'(허용 {limit:.2f}) → {"기둥 사이 통과" if inside else "기둥 사이 아님"}')
                if inside:
                    self._crossed = True
                    self._crossing = point
        self._previous_xy = boat

    def _follow(self, boat, dri, grid, now):
        cmd, reached = self.los(self._path, self.los_thrust_ratio, self.los_turn_ratio)
        progress = self._los_follower.progress_idx or 0
        lookahead = self._los_follower.params.lookahead
        if not lookahead_chord_safe(dri, boat, self._path, progress, lookahead,
                                    self.gate_params, escape=self.replan_from_here):
            if not (self._replan(boat, dri, now, 'LOOKAHEAD_UNSAFE')
                    and self._path_problem(dri) is None):
                return self._stall('LOOKAHEAD_UNSAFE', now)
            cmd, reached = self.los(self._path, self.los_thrust_ratio, self.los_turn_ratio)
            progress = self._los_follower.progress_idx or 0
            if not lookahead_chord_safe(dri, boat, self._path, progress, lookahead,
                                        self.gate_params, escape=self.replan_from_here):
                return self._stall('LOOKAHEAD_UNSAFE', now)
        near = np.vstack((
            path_ahead(self._path, progress, self.gate_params.observed_horizon),
            lookahead_chord(boat, self._path, progress, lookahead, self.gate_params)))
        slow = self._slow_if_unobserved(cmd, grid, near, boat)

        if reached:
            if not self._crossed:
                # 경로 끝까지 왔는데 기둥 사이를 지나지 않았다. 다시 고르면 이 게이트는
                # 이미 뒤라 다음 게이트로 넘어가 버리므로(게이트 누락) 정지 유지.
                self.get_logger().warn(
                    f'gate: GATE_NOT_CROSSED — 경로 끝인데 쌍 {_xy(self._pair.midpoint)} 기둥 '
                    f'사이를 안 지남, 정지 유지 (배 {_xy(boat)})', throttle_duration_sec=2.0)
                return self._stop('GATE_NOT_CROSSED', hold=True)
            self._mark_passed()
            self._publish_debug()
            return self._output(Twist())
        self._stall_since = None
        self._hold_point = None

        self._status(f'FOLLOW_GATE_{len(self._passed_midpoints) + 1}/'
                     f'{self.gate_params.required_pairs}' + ('_SLOW' if slow else ''))
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
