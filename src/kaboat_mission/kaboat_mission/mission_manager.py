"""mission_manager — 종합임무 state machine.

순서   [undock] → gate(항로추종) → station(위치유지) → dock(도킹) → search(탐색)
       → avoid(장애물 회피) → done

출발 직진 (undock, 2026-11-13): 대회 출발은 슬롯 안에 들어가 있는 상태에서 한다. 처음부터 도는
  전체 실행(start_state 없음, single_mission=false)이면 gate 접근 전에 undock_sec(3 s) 동안
  텔레옵 4단계와 같은 직진(undock_linear 0.4667 = 좌우 1700 µs)을 /cmd/undock 으로 낸 뒤 gate_entry 로
  접근한다. 3 s 는 모터가 실제로 자율 명령을 받는 동안(/teleop/status == AUTO)만 센다 — 텔레옵·비상
  정지·노드 기동 지연 동안 시간이 헛되이 지나가지 않게 (undock_wait_auto:=false 면 그냥 시간).
  undock_sec:=0 이면 끔.

제한 시간 (2026-11-13, 경기 전체 15 분): 여러 미션을 이어 도는 실행(single_mission=false)에서 한
  미션이 실패해 무한정 기다리거나 재시도하지 않게, 미션마다 몫을 두고 넘으면 건너뛴다.
    time_limit.<미션> [s]  gate 180, station 120, dock 240, search 180, avoid 180 (합 900 s = 15 분).
                           미션 몫은 그 미션의 접근(approach)부터 센다. 마감 시각은 '전체 시작 + 앞
                           미션들 몫의 합'으로 고정 — 앞 미션이 일찍 끝나면 남은 시간이 다음 미션으로
                           넘어가고, 늦어도 뒤 미션 몫을 먹지 않는다 (0 = 그 미션 제한 없음).
                           전체 시작 = 출발 직진이 실제로 시작된 때(없으면 첫 접근).
    approach_timeout_sec   접근만 따로 90 s 넘으면 그 미션을 건너뛴다.
  넘으면 다음 미션 접근으로 (avoid 면 done). dock 미션 중에 넘으면 슬롯 근처일 수 있으므로 먼저
  직진 후진(timeout_retreat_cmd 0.05)으로 timeout_retreat_m(3 m) 또는 timeout_retreat_max_sec(30 s)
  만큼 빠져나온 뒤 넘긴다 (state 'retreat', /cmd/retreat). 시작 자세(entry)가 기록 안 된 미션도
  멈추지 않고 건너뛴다. mission_timeouts:=false 면 전부 끔. 단일 미션 시험(test_mission·go_*)은 해당
  없음 — 넘어갈 다음 미션이 없다.

미션 시작: 각 미션의 entry_waypoints.<state> = [x, y, angle_deg] 로
  approach state 에 진입한다. waypoint() 도달 신호를 받은 뒤에만 해당
  behavior 를 활성화한다. 미설정 entry 는 idle 로 정지한다.

전환 규칙 (팀 합의):
  1) 현재 미션 behavior 가 /mission/complete 로 자기 state 이름을 발행하고
  2) |현재위치(/odom) − 현재 미션 목표좌표| ≤ transition_radius(기본 2.0m — RTK, 2026-11-12. 예전 5.0)
  두 조건을 모두 만족하면 다음 state 로 전이한다.
  dock 은 경로 고정 뒤 /dock/one_shot_path 마지막 점을 미션 목표로 사용한다.

발행   /mission/state (String, latched)  — cmd_mux 와 모든 behavior 가 구독
       /mission/goal  (PoseStamped, latched) — 현재 미션의 목표좌표
       /mission/entry_goal (PoseStamped) — 다음 미션 시작 위치/방향
       /mission/search_center (PoseStamped, latched) — 탐색 영역 중심 (search 시작 때).
                                            rec search_center, 없으면 search_entry 선수 방향
                                            search_center_offset(7 m) 앞. search_end 는 출구만.
       /detector/enable (Bool, latched)  — dock_mark_detector YOLO 추론 게이팅
                                            (state == 'dock' 일 때만 true)
구독   /mission/complete (String) — 완료한 behavior 의 state 이름

미션별 종료 목표좌표(waypoints.<state>)와 시작 자세
(entry_waypoints.<state>, entry_ready.<state>)는 파라미터로 주입한다
— kaboat_bringup/config/mission_params.yaml 참고.

현장 기록 지점 (rec): points_file(기본 ~/kaboat_mission_points.json)이 있으면
  <미션>_entry → entry_waypoints.<미션> + entry_ready.<미션>=true
  <미션>_end   → waypoints.<미션>
  search_center → 탐색 영역 중심 (선택)
으로 mission_params 값을 덮어쓴다(kaboat_mission/mission_points.py). 시작 거부(노드 종료):
  - 파일이 깨졌거나, 기록 당시 원점이 지금 원점 파일(origin_file)과 다를 때
  - 한 미션의 _entry 와 _end 중 하나만 있을 때 (dock_entry 단독은 허용)
파일이 없으면 mission_params 값 그대로. points_file 이 '' 또는 none 이면 읽지 않는다(수조 런치).

단일 미션 테스트:
  전체를 순서대로 안 돌리고 특정 미션 하나만 켜서 그 behavior 를 반복
  개발/디버깅하고 싶을 때는 start_state 파라미터를 쓴다.
    ros2 run kaboat_mission mission_manager --ros-args \
        -p start_state:=dock -p single_mission:=true
  start_state 의 접근 단계부터 시작하고, single_mission:=true 면 그
  완료 신호와 목표 2m 조건 달성 후 다음으로 안 넘어가고 바로 'done'(정지)으로 간다 —
  cmd_mux 가 멈추니 배가 목표 근처에서 안전하게 정지한 채로 결과를 볼 수 있다.
  이때도 entry_waypoints.<state>와 entry_ready.<state>=true가 필요하다.
"""
import math
import os

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy

from std_msgs.msg import String, Bool
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry, Path

from kaboat_mission.mission_points import (
    DEFAULT_ORIGIN_FILE, DEFAULT_POINTS_FILE, MISSION_STATES, PointStore,
    is_mission_point, mission_targets, read_origin_file, search_center)

SEARCH_CENTER_OFFSET = 7.0   # [m] search_center 미기록 시 search_entry 선수 방향 이만큼 앞

# 출발 직진 (슬롯 탈출) — 텔레옵 4단계 = 중립 1500 + 200 µs = 1700 µs (thruster_driver
# teleop_level_pwm_offsets[3]). 자율 명령은 불감대(25 µs) 뒤 1525~1900 µs 에 비례하므로
# (1700 − 1525) / (1900 − 1525) = 0.4667 이 좌우 1700 µs 다 (thrusters.yaml 기본값 기준).
UNDOCK_SEC = 3.0
UNDOCK_LINEAR = round((1700 - 1525) / (1900 - 1525), 4)
UNDOCK_PERIOD = 0.05            # [s] /cmd/undock 발행 주기 (cmd_mux 워치독 0.3 s, 모터 0.3 s 보다 짧게)
THRUSTER_STATUS_TIMEOUT = 1.0   # [s] /teleop/status(5 Hz)가 이보다 오래 안 오면 AUTO 로 안 본다

# 미션 제한 시간 [s] — 경기 전체 15 분(900 s) 배분. 접근 시간 포함, 남은 시간은 다음 미션으로 이월.
MISSION_TIME_LIMITS = {'gate': 180.0, 'station': 120.0, 'dock': 240.0,
                       'search': 180.0, 'avoid': 180.0}
APPROACH_TIMEOUT = 90.0         # [s] 시작 자세 접근만의 상한
TIMEOUT_RETREAT_M = 3.0         # [m] dock 시간 초과 시 직진 후진 거리
TIMEOUT_RETREAT_CMD = 0.05      # 그 후진 출력 (dock_retreat_cmd 와 같은 값)
TIMEOUT_RETREAT_MAX_SEC = 30.0  # [s] 후진 상한 (못 움직여도 넘어간다)

MISSION_ORDER = list(MISSION_STATES)

# 파라미터 미지정 시의 스모크 테스트용 기본 웨이포인트 (odom 좌표 [m]).
# 실제 경기장 좌표는 mission_params.yaml 에서 덮어쓴다.
DEFAULT_WAYPOINTS = {
    'gate':    [8.0, 0.0],
    'station': [8.0, 8.0],
    'dock':    [0.0, 8.0],
    'search':  [-8.0, 8.0],
    'avoid':   [-8.0, 0.0],
}


class MissionManager(Node):
    def __init__(self, **node_kwargs):
        super().__init__('mission_manager', **node_kwargs)

        # 전환 판정 반경 [m]. 5.0 → 2.0 (2026-11-12): RTK Fixed 로 위치가 수 cm 라 좁혔다.
        # behavior 완료 거리(gate·avoid 2.0, search 는 선회 뒤 search_end 1.5 m)와 맞춘다.
        # dock 은 기록된 dock_end 대신 한 번 고정한 LiDAR/GPS 정박점을 쓴다.
        self.declare_parameter('transition_radius', 2.0)
        self.declare_parameter('auto_start', True)          # 켜지면 바로 첫 미션 시작
        self.declare_parameter('start_state', '')            # 단일 미션 테스트용 시작 지점
        self.declare_parameter('single_mission', False)      # true 면 start_state 하나만 하고 done
        self.declare_parameter('entry_required', True)      # 단독 미션 preview 전용 우회
        # 현장 기록 지점(rec) — 있으면 아래 waypoints/entry_* 를 덮어씀. '' 또는 none 이면 안 읽음
        self.declare_parameter('points_file', DEFAULT_POINTS_FILE)
        self.declare_parameter('origin_file', DEFAULT_ORIGIN_FILE)
        for name in MISSION_ORDER:
            self.declare_parameter(f'waypoints.{name}', DEFAULT_WAYPOINTS[name])
            self.declare_parameter(f'entry_waypoints.{name}', [0.0, 0.0, 0.0])
            self.declare_parameter(f'entry_ready.{name}', False)
        # 탐색 영역 중심 — rec search_center 가 있으면 덮어씀. 둘 다 없으면 entry 앞 offset
        self.declare_parameter('search_center', [0.0, 0.0])
        self.declare_parameter('search_center_ready', False)
        self.declare_parameter('search_center_offset', SEARCH_CENTER_OFFSET)
        # 출발 직진 (슬롯 탈출) — 처음부터 도는 전체 실행에서만. 0 이면 끔
        self.declare_parameter('undock_sec', UNDOCK_SEC)
        self.declare_parameter('undock_linear', UNDOCK_LINEAR)
        self.declare_parameter('undock_wait_auto', True)
        # 제한 시간 — 여러 미션을 이어 도는 실행에서만 (모듈 설명)
        self.declare_parameter('mission_timeouts', True)
        for name in MISSION_ORDER:
            self.declare_parameter(f'time_limit.{name}', MISSION_TIME_LIMITS[name])
        self.declare_parameter('approach_timeout_sec', APPROACH_TIMEOUT)
        self.declare_parameter('timeout_retreat_m', TIMEOUT_RETREAT_M)
        self.declare_parameter('timeout_retreat_cmd', TIMEOUT_RETREAT_CMD)
        self.declare_parameter('timeout_retreat_max_sec', TIMEOUT_RETREAT_MAX_SEC)

        self.radius = float(self.get_parameter('transition_radius').value)
        self.mission_timeouts = bool(self.get_parameter('mission_timeouts').value)
        self.time_limits = {name: float(self.get_parameter(f'time_limit.{name}').value)
                            for name in MISSION_ORDER}
        self.approach_timeout = float(self.get_parameter('approach_timeout_sec').value)
        self.retreat_m = float(self.get_parameter('timeout_retreat_m').value)
        self.retreat_cmd = float(self.get_parameter('timeout_retreat_cmd').value)
        self.retreat_max_sec = float(self.get_parameter('timeout_retreat_max_sec').value)
        if not all(math.isfinite(v) and v >= 0.0 for v in (
                *self.time_limits.values(), self.approach_timeout, self.retreat_m,
                self.retreat_max_sec)):
            raise ValueError('time_limit.*, approach_timeout_sec, timeout_retreat_m and '
                             'timeout_retreat_max_sec must be finite and >= 0')
        if not (math.isfinite(self.retreat_cmd) and 0.0 < self.retreat_cmd <= 1.0):
            raise ValueError('timeout_retreat_cmd must be in (0, 1]')
        self.undock_sec = float(self.get_parameter('undock_sec').value)
        self.undock_linear = float(self.get_parameter('undock_linear').value)
        self.undock_wait_auto = bool(self.get_parameter('undock_wait_auto').value)
        if not (math.isfinite(self.undock_sec) and self.undock_sec >= 0.0):
            raise ValueError('undock_sec must be finite and >= 0')
        if not (math.isfinite(self.undock_linear) and 0.0 < self.undock_linear <= 1.0):
            raise ValueError('undock_linear must be in (0, 1]')
        self.waypoints = {
            name: list(self.get_parameter(f'waypoints.{name}').value)
            for name in MISSION_ORDER
        }
        self.entry_waypoints = {
            name: list(self.get_parameter(f'entry_waypoints.{name}').value)
            for name in MISSION_ORDER
        }
        self.entry_ready = {
            name: bool(self.get_parameter(f'entry_ready.{name}').value)
            for name in MISSION_ORDER
        }
        self.entry_required = bool(self.get_parameter('entry_required').value)
        self.search_center = (list(self.get_parameter('search_center').value)
                              if bool(self.get_parameter('search_center_ready').value) else None)
        self.search_center_offset = float(self.get_parameter('search_center_offset').value)
        if self.search_center is not None and (
                len(self.search_center) != 2
                or not all(math.isfinite(v) for v in self.search_center)):
            raise ValueError('search_center must be [x, y]')
        if not math.isfinite(self.search_center_offset):
            raise ValueError('search_center_offset must be finite')
        self.apply_recorded_points(str(self.get_parameter('points_file').value),
                                   str(self.get_parameter('origin_file').value))
        for name in MISSION_ORDER:
            pose = self.entry_waypoints[name]
            if len(pose) != 3 or not all(math.isfinite(v) for v in pose):
                raise ValueError(f'entry_waypoints.{name} must be [x, y, angle_deg]')

        self.odom = None
        self.dock_route_goal = None
        # 완료 신호가 웨이포인트 도착보다 먼저 와도 잃지 않고 기억한다.
        self.behavior_complete = False
        self.phase = 'idle'

        start_state = self.get_parameter('start_state').value
        self.start_state = start_state
        self.single_mission = bool(self.get_parameter('single_mission').value)
        if start_state:
            if start_state not in MISSION_ORDER:
                self.get_logger().error(
                    f"start_state='{start_state}' 는 알 수 없는 미션입니다 "
                    f"(가능한 값: {MISSION_ORDER}). 처음부터 시작합니다.")
                self.idx = -1
            else:
                self.idx = MISSION_ORDER.index(start_state) - 1  # advance() 가 +1 해서 착지
        else:
            self.idx = -1   # MISSION_ORDER 인덱스 (-1 = 시작 전, 정상 순서대로)

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub_state = self.create_publisher(String, '/mission/state', latched)
        self.pub_goal = self.create_publisher(PoseStamped, '/mission/goal', latched)
        self.pub_entry_goal = self.create_publisher(
            PoseStamped, '/mission/entry_goal', 10)
        self.pub_search_center = self.create_publisher(
            PoseStamped, '/mission/search_center', latched)
        # v5: YOLO 추론 게이팅 — dock_mark_detector 는 이 신호가 true 인 동안만 추론
        self.pub_detector_enable = self.create_publisher(Bool, '/detector/enable', latched)
        # 출발 직진 명령 — cmd_mux 가 state 'undock' 일 때 /cmd_vel 로 통과
        self.pub_undock = self.create_publisher(Twist, '/cmd/undock', 10)
        self.thruster_mode = None          # thruster_driver /teleop/status (AUTO·TELEOP·ESTOP …)
        self.thruster_mode_at = None
        self.undock_elapsed = 0.0          # 모터가 AUTO 였던 시간 합 [s]
        self._undock_last = None
        self._undock_timer = None
        self._undock_warned_at = None
        # 제한 시간
        self.run_started_at = None         # 전체 시작 (출발 직진이 실제로 시작된 때, 없으면 첫 접근)
        self.deadline = None               # 지금 미션의 마감 시각 (None = 제한 없음)
        self._slot_end = None              # 앞 미션 마감 — 다음 미션 몫을 여기에 잇는다(이월)
        self.approach_started_at = None
        self.skipped = []                  # 건너뛴 미션 [(이름, 이유)]
        # dock 시간 초과 후진 — cmd_mux 가 state 'retreat' 일 때 /cmd_vel 로 통과
        self.pub_retreat = self.create_publisher(Twist, '/cmd/retreat', 10)
        self._retreat_timer = None
        self._retreat_from = None
        self._retreat_started = None

        self.create_subscription(Odometry, '/odom', self.on_odom, 10)
        self.create_subscription(String, '/mission/complete', self.on_complete, 10)
        self.create_subscription(Path, '/dock/one_shot_path', self.on_dock_path, latched)
        self.create_subscription(String, '/teleop/status', self.on_thruster_status, 10)
        self.create_timer(0.2, self.tick)  # 5Hz 판정

        wp_str = ', '.join(f"{n}({w[0]:.0f},{w[1]:.0f})" for n, w in self.waypoints.items())
        self.get_logger().info(
            f'mission_manager 시작 — 전환 반경 {self.radius}m, 웨이포인트 {wp_str}')

        if bool(self.get_parameter('auto_start').value):
            if self.undock_applies():
                self.start_undock()
            else:
                self.advance()

    # ── 출발 직진 (슬롯 탈출) ───────────────────────────
    def undock_applies(self) -> bool:
        """처음부터 도는 전체 실행(start_state 없음, single_mission=false)이고 undock_sec > 0.
        start_state:=gate 는 idx 가 처음과 같지만 'gate 부터 시험' 이라 하지 않는다."""
        return (self.undock_sec > 0.0 and not self.start_state and self.idx == -1
                and not self.single_mission)

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def start_undock(self):
        self.phase = 'undock'
        self.undock_elapsed = 0.0
        self._undock_last = None
        self._undock_warned_at = None
        self.set_state('undock')
        self.get_logger().info(
            f'출발 직진 (슬롯 탈출) — {self.undock_sec:.1f} s 동안 직진 {self.undock_linear:.4f} '
            f'(텔레옵 4단계 ≈ 좌우 1700 µs) 뒤 gate_entry 로 접근'
            + (' (모터가 AUTO 인 시간만 셈)' if self.undock_wait_auto else ''))
        self._undock_timer = self.create_timer(UNDOCK_PERIOD, self.undock_step)

    def on_thruster_status(self, msg: String):
        self.thruster_mode = msg.data.strip().upper()
        self.thruster_mode_at = self._now()

    def _motor_auto(self, now: float) -> bool:
        return (self.thruster_mode == 'AUTO' and self.thruster_mode_at is not None
                and now - self.thruster_mode_at <= THRUSTER_STATUS_TIMEOUT)

    def undock_step(self, now: float | None = None):
        """UNDOCK_PERIOD 마다 — 직진 명령을 내고, 모터가 AUTO 인 시간을 쌓아 undock_sec 이면 gate 접근."""
        if self.phase != 'undock':
            return
        now = self._now() if now is None else now
        counting = (not self.undock_wait_auto) or self._motor_auto(now)
        if counting and self.run_started_at is None:
            self._start_run_clock(now)            # 경기 시간은 실제로 출발한 때부터
        if self._undock_last is not None and counting:
            # 타이머가 밀려도 한 번에 크게 건너뛰지 않게
            self.undock_elapsed += min(max(now - self._undock_last, 0.0), 4 * UNDOCK_PERIOD)
        self._undock_last = now
        if self.undock_elapsed >= self.undock_sec:
            # 정지 명령은 따로 안 낸다 — state 가 approach 로 바뀌면 cmd_mux 가 /cmd/undock 을 버리고
            # mission_approach(10 Hz)가 바로 이어받는다 (안 이어받아도 모터 워치독 0.3 s 에 중립)
            if self._undock_timer is not None:
                self.destroy_timer(self._undock_timer)
                self._undock_timer = None
            self.get_logger().info(
                f'출발 직진 끝 ({self.undock_elapsed:.1f} s) — gate_entry 접근 시작')
            self.advance()
            return
        cmd = Twist()
        cmd.linear.x = self.undock_linear
        self.pub_undock.publish(cmd)
        if not counting and (self._undock_warned_at is None
                             or now - self._undock_warned_at >= 5.0):
            self._undock_warned_at = now
            self.get_logger().warning(
                f'출발 직진 대기 — 모터 모드 {self.thruster_mode or "모름(/teleop/status 없음)"}: '
                f'AUTO 일 때만 셈 ({self.undock_elapsed:.1f}/{self.undock_sec:.1f} s). '
                '텔레옵을 놓거나 비상 정지를 풀면 이어서 직진')

    # ── 현장 기록 지점 ─────────────────────────────────
    def apply_recorded_points(self, points_file: str, origin_file: str):
        """rec 로 기록한 <미션>_entry / <미션>_end 로 미션 좌표를 덮어쓴다.

        원점이 다르거나 파일이 잘못됐으면 ValueError — 엉뚱한 자리로 가지 않도록 시작을 막는다.
        """
        if points_file.strip().lower() in ('', 'none'):
            self.get_logger().info('points_file 비활성 — mission_params 좌표만 사용')
            return
        path = os.path.expanduser(points_file)
        if not os.path.exists(path):
            self.get_logger().info(f'기록 지점 파일 없음 ({path}) — mission_params 좌표 사용')
            return
        store = PointStore.load(path)
        try:
            store.check_origin(read_origin_file(origin_file))
        except ValueError as exc:
            raise ValueError(
                f'{path}: {exc} — 원점을 다시 잡았으면 지점을 다시 기록하거나 '
                'points_file 을 바꾼다') from exc
        targets = mission_targets(store, MISSION_ORDER)
        for state in MISSION_ORDER:
            if state not in targets:
                continue
            entry, end = targets[state]['entry'], targets[state]['end']
            self.entry_waypoints[state] = list(entry)
            self.entry_ready[state] = True
            self.waypoints[state] = list(end)
            self.get_logger().info(
                f'[기록 지점] {state}: 시작 ({entry[0]:+.2f}, {entry[1]:+.2f}, '
                f'{entry[2]:+.1f}°) → 끝 ({end[0]:+.2f}, {end[1]:+.2f})')
        center = search_center(store)
        if center is not None:
            self.search_center = list(center)
            self.get_logger().info(
                f'[기록 지점] 탐색 영역 중심 ({center[0]:+.2f}, {center[1]:+.2f})')
        missing = [s for s in MISSION_ORDER if s not in targets]
        if missing:
            self.get_logger().warning(
                f'기록 지점이 없는 미션 {missing} — mission_params 값 사용 '
                '(entry_ready=false 면 이어 도는 실행은 그 미션을 건너뛰고, 단일 미션 시험은 멈춤)')
        unused = [n for n in store.points if not is_mission_point(n, MISSION_ORDER)]
        if unused:
            self.get_logger().info(f'미션 이름 규칙이 아닌 지점(안 씀): {unused}')

    # ── state 전이 ────────────────────────────────────
    def advance(self):
        now = self._now()
        if self.run_started_at is None:
            self._start_run_clock(now)
        self.idx += 1
        if self.idx >= len(MISSION_ORDER):
            self.phase = 'done'
            self.set_state('done')
            skipped = (f' — 건너뛴 미션 {", ".join(f"{n}({why})" for n, why in self.skipped)}'
                       if self.skipped else '')
            self.get_logger().info(
                f'종합임무 전체 완료 — done (cmd_mux 정지 상태), '
                f'전체 {now - self.run_started_at:.0f} s{skipped}')
            return
        state = MISSION_ORDER[self.idx]
        self._set_deadline(state, now)
        if self.entry_required:
            if not self.entry_ready[state]:
                if self.timeouts_active():
                    # 이어 도는 실행 — 멈추지 않고 다음 미션으로 (남은 몫은 다음으로 이월)
                    self.skipped.append((state, '시작 자세 미기록'))
                    self.get_logger().error(
                        f"'{state}' 시작 자세 미설정 (rec {state}_entry 또는 entry_waypoints."
                        f"{state}·entry_ready.{state}) — '{state}' 건너뜀")
                    self.advance()
                    return
                self.phase = 'blocked'
                self.set_state('idle')
                self.get_logger().error(
                    f"'{state}' 시작 자세 미설정 — entry_waypoints.{state}와 "
                    f'entry_ready.{state}=true 설정 후 재시작 필요')
                return
            self.phase = 'approach'
            self.approach_started_at = now
            self.set_state('approach')
            self.publish_entry_goal()
            pose = self.entry_waypoints[state]
            self.get_logger().info(
                f"미션 [{self.idx+1}/{len(MISSION_ORDER)}] '{state}' 접근 — "
                f'({pose[0]}, {pose[1]}), 선수각 {pose[2]}°{self._deadline_text(now)}')
        else:
            self.start_mission()

    # ── 제한 시간 ──────────────────────────────────────
    def timeouts_active(self) -> bool:
        """이어 도는 실행만 — 단일 미션 시험은 넘어갈 다음 미션이 없다."""
        return self.mission_timeouts and not self.single_mission

    def _start_run_clock(self, now: float):
        self.run_started_at = now
        self._slot_end = now

    def _set_deadline(self, state: str, now: float):
        """이 미션 마감 = 앞 미션 마감(이월) + 이 미션 몫. 몫 0 이면 제한 없음(다음은 그때부터)."""
        limit = self.time_limits[state]
        if not self.timeouts_active() or limit <= 0.0:
            self.deadline = None
            self._slot_end = None
            return
        base = self._slot_end if self._slot_end is not None else now
        self.deadline = base + limit
        self._slot_end = self.deadline

    def _deadline_text(self, now: float) -> str:
        if self.deadline is None:
            return ''
        return (f' [마감 전체 +{self.deadline - self.run_started_at:.0f} s, '
                f'남은 {self.deadline - now:.0f} s]')

    def check_timeout(self, now: float) -> bool:
        """접근 상한·미션 마감을 넘었으면 건너뛰고 True."""
        if not self.timeouts_active() or not (0 <= self.idx < len(MISSION_ORDER)):
            return False
        state = MISSION_ORDER[self.idx]
        if (self.phase == 'approach' and self.approach_timeout > 0.0
                and self.approach_started_at is not None
                and now - self.approach_started_at >= self.approach_timeout):
            why = f'시작 자세 접근 {self.approach_timeout:.0f} s 초과'
        elif self.phase in ('approach', 'mission') and self.deadline is not None \
                and now >= self.deadline:
            why = (f'제한 시간 초과 ({"접근 중" if self.phase == "approach" else "미션 중"}, '
                   f'몫 {self.time_limits[state]:.0f} s, 전체 +{now - self.run_started_at:.0f} s)')
        else:
            return False
        retreat = state == 'dock' and self.phase == 'mission' and self.retreat_m > 0.0
        self.skipped.append((state, why))
        self.get_logger().error(
            f"'{state}' {why} — 건너뜀"
            + (f', 먼저 직진 후진 {self.retreat_m:.1f} m' if retreat else ''))
        if retreat:
            self.start_retreat(now)
        else:
            self.advance()
        return True

    def start_retreat(self, now: float):
        """dock 시간 초과 — 슬롯 근처일 수 있으니 직진 후진으로 빠져나온 뒤 다음 미션."""
        self.phase = 'retreat'
        self.set_state('retreat')
        pos = None if self.odom is None else self.odom.pose.pose.position
        self._retreat_from = None if pos is None else (pos.x, pos.y)
        self._retreat_started = now
        self._retreat_timer = self.create_timer(UNDOCK_PERIOD, self.retreat_step)

    def retreat_step(self, now: float | None = None):
        if self.phase != 'retreat':
            return
        now = self._now() if now is None else now
        moved = 0.0
        if self.odom is not None:
            pos = self.odom.pose.pose.position
            if self._retreat_from is None:
                self._retreat_from = (pos.x, pos.y)
            moved = math.dist((pos.x, pos.y), self._retreat_from)
        elapsed = now - self._retreat_started
        if moved >= self.retreat_m or elapsed >= self.retreat_max_sec:
            if self._retreat_timer is not None:
                self.destroy_timer(self._retreat_timer)
                self._retreat_timer = None
            self.get_logger().info(
                f'dock 시간 초과 후진 끝 ({moved:.1f} m, {elapsed:.0f} s) — 다음 미션으로')
            self.advance()
            return
        cmd = Twist()
        cmd.linear.x = -self.retreat_cmd
        self.pub_retreat.publish(cmd)

    def _pose(self, x, y, angle_deg=0.0):
        goal = PoseStamped()
        goal.header.frame_id = 'odom'
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = float(x)
        goal.pose.position.y = float(y)
        angle = math.radians(angle_deg) / 2.0
        goal.pose.orientation.z = math.sin(angle)
        goal.pose.orientation.w = math.cos(angle)
        return goal

    def publish_entry_goal(self):
        state = MISSION_ORDER[self.idx]
        self.pub_entry_goal.publish(self._pose(*self.entry_waypoints[state]))

    def search_area_center(self):
        """탐색 영역 중심 [x, y] — 기록/파라미터 search_center, 없으면 search_entry 선수 방향
        search_center_offset 앞, 그것도 없으면 None (search_circler 가 자기 활성화 자세 앞을 씀)."""
        if self.search_center is not None:
            return list(self.search_center)
        if self.entry_ready['search']:
            x, y, deg = self.entry_waypoints['search']
            a = math.radians(deg)
            return [x + self.search_center_offset * math.cos(a),
                    y + self.search_center_offset * math.sin(a)]
        return None

    def start_mission(self):
        self.phase = 'mission'
        state = MISSION_ORDER[self.idx]
        if state == 'dock':
            self.dock_route_goal = None
        if state == 'search':
            # behavior 가 활성화되기 전에 latched 로 먼저 둔다
            center = self.search_area_center()
            if center is not None:
                self.pub_search_center.publish(self._pose(*center))
                self.get_logger().info(
                    f'탐색 영역 중심 ({center[0]:.2f}, {center[1]:.2f}) → /mission/search_center')
            else:
                self.get_logger().warning(
                    '탐색 영역 중심 없음 (search_center·search_entry 둘 다 없음) — '
                    'search_circler 가 활성화 자세 앞을 씀')
        self.set_state(state)
        self.publish_mission_goal()
        wp = self.waypoints[state]
        self.get_logger().info(
            f"미션 [{self.idx+1}/{len(MISSION_ORDER)}] '{state}' 시작 — 목표 ({wp[0]}, {wp[1]})"
            f'{self._deadline_text(self._now())}')

    def publish_mission_goal(self):
        state = MISSION_ORDER[self.idx]
        wp = (self.dock_route_goal if state == 'dock' and self.dock_route_goal is not None
              else self.waypoints[state])
        self.pub_goal.publish(self._pose(*wp))

    def set_state(self, state: str):
        self.behavior_complete = False
        msg = String()
        msg.data = state
        self.pub_state.publish(msg)
        # 도킹 구간에서만 YOLO 추론 ON (v5 게이팅 규약)
        enable = Bool()
        enable.data = (state == 'dock')
        self.pub_detector_enable.publish(enable)

    # ── behavior 완료 + 전환 반경 판정 ────────────────
    def on_odom(self, msg: Odometry):
        self.odom = msg

    def on_dock_path(self, msg: Path):
        if msg.header.frame_id == 'odom' and msg.poses:
            point = msg.poses[-1].pose.position
            if math.isfinite(point.x) and math.isfinite(point.y):
                self.dock_route_goal = [point.x, point.y]
        elif msg.header.frame_id == 'odom':
            self.dock_route_goal = None
            if self.phase == 'mission' and MISSION_ORDER[self.idx] == 'dock':
                self.behavior_complete = False

    def on_complete(self, msg: String):
        if not (0 <= self.idx < len(MISSION_ORDER)):
            return
        if self.phase == 'approach':
            if msg.data == 'approach':
                self.get_logger().info(
                    f"'{MISSION_ORDER[self.idx]}' 시작 자세 도달 — behavior 활성화")
                self.start_mission()
            return
        if self.phase != 'mission':
            return
        state = MISSION_ORDER[self.idx]
        if msg.data != state:
            self.get_logger().warning(
                f"현재 미션은 '{state}' — 다른 완료 신호 '{msg.data}' 무시")
            return
        if not self.behavior_complete:
            self.behavior_complete = True
            self.get_logger().info(
                f"behavior '{state}' 완료 신호 수신 — 목표 {self.radius}m 이내 진입 대기")

    def tick(self):
        if not (0 <= self.idx < len(MISSION_ORDER)):
            return
        if self.check_timeout(self._now()):
            return
        if self.phase == 'approach':
            self.publish_entry_goal()
            return
        if self.phase != 'mission':
            return
        self.publish_mission_goal()
        if self.odom is None:
            return
        state = MISSION_ORDER[self.idx]
        if state == 'dock' and self.dock_route_goal is None:
            return
        wp = self.dock_route_goal if state == 'dock' else self.waypoints[state]
        dx = wp[0] - self.odom.pose.pose.position.x
        dy = wp[1] - self.odom.pose.pose.position.y
        inside_waypoint = math.hypot(dx, dy) <= self.radius
        if self.behavior_complete and inside_waypoint:
            self.get_logger().info(
                f"behavior 완료 + 목표 {self.radius}m 이내 진입 — "
                f"'{MISSION_ORDER[self.idx]}' 완료")
            if self.single_mission:
                self.phase = 'done'
                self.get_logger().info(
                    'single_mission=true — 다음 미션으로 안 넘어가고 done 으로 정지')
                self.set_state('done')
                self.idx = len(MISSION_ORDER)  # tick() 이 더 이상 판정 안 하도록
            else:
                self.advance()


def main(args=None):
    rclpy.init(args=args)
    try:
        node = MissionManager()
    except ValueError as exc:
        # 기록 지점·원점이 맞지 않으면 엉뚱한 자리로 가지 않도록 시작하지 않는다
        rclpy.logging.get_logger('mission_manager').fatal(f'시작 거부: {exc}')
        rclpy.shutdown()
        raise SystemExit(1) from None
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
