"""tournament_sequence — 토너먼트 한 번에: 출발 직진 → goto 시작점 → 회피 시험, 사이에 명령 공백 없이.

outdoor_tournament_run.launch.py (go_tournament) 가 처음에 cmd_mux·goto·회피 시험 노드·이 노드를 같이 띄운다.
goto 는 /cmd/approach, 회피 시험은 /cmd/avoid 로 내고(remap), cmd_mux 가 /mission/state 에 맞는 것만
/cmd_vel 로 보낸다. 이 노드가 /mission/state 를 바꾼다:

  undock    /cmd/undock 로 출발 직진 undock_linear (launch 기본 1.0 = 최대, mission_manager 출발 직진과 같은 규칙 — 모터가 AUTO
            (/teleop/status)인 시간만 셈, 텔레옵·비상 정지 중이면 기다렸다가 이어서)
  approach  goto 명령 통과. 시작 조건(start_point 에서 start_dist 안 + 선수가 goal_point 방향
            ±start_heading_deg)을 start_hold_sec 유지하면 회피 시험 시작 서비스를 부른다.
            거절되면(경로 없음 등) 이유를 남기고 0.5 s 뒤 다시 — 그동안 goto 가 계속 몬다.
  avoid     회피 시험 명령 통과 (도착하면 시험 노드가 정지 명령을 낸다)

다음 단계 노드가 이미 떠서 명령을 내고 있으므로 넘어갈 때 기다림이 없다. 예전 go_and_test 는 직진 뒤 goto
를 그때 켜서 약 3 s(뜨기 1 s + /odom 발행자 찾기 2 s), goto 를 끄고 시험을 그때 켜서 또 몇 초 명령이 없었다
(2026-11-14 젯슨 육상 시험).
"""

import math
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from std_msgs.msg import String
from std_srvs.srv import Trigger

from kaboat_mission.mission_manager import (
    THRUSTER_STATUS_TIMEOUT, UNDOCK_LINEAR, UNDOCK_PERIOD, UNDOCK_SEC)
from kaboat_mission.mission_points import (
    DEFAULT_ORIGIN_FILE, DEFAULT_POINTS_FILE, recorded_point)

ODOM_STALE_SEC = 1.0
RETRY_SEC = 0.5                 # 시작 서비스가 거절하면 다시 부를 때까지 [s]
PENDING_TIMEOUT_SEC = 2.0       # 시작 서비스 응답을 이만큼 못 받으면 버리고 다시 [s]
RUNNING_STATES = ('FOLLOW', 'ESCAPE')   # 회피 시험이 달리는 중일 때 /real_avoid_test/status 머리말
CONFLICT_WARN_SEC = 5.0
CMD_ECHO_SEC = 0.3              # /cmd_vel 에 직진이 이만큼 안에 보여야 '나가는 중' (모터 워치독과 같음)
MUX_NODE = 'cmd_mux'


def wrap_deg(angle: float) -> float:
    return (angle + 180.0) % 360.0 - 180.0


def start_condition(boat_xy, yaw_deg, start_xy, goal_xy, dist_tol, heading_tol):
    """(만족?, 시작점까지 [m], 선수각 오차 [deg]) — 선수각 기준은 배 → 목표 방향 (회피 시험 출발 검사와 같음)."""
    dist = math.hypot(boat_xy[0] - start_xy[0], boat_xy[1] - start_xy[1])
    bearing = math.degrees(math.atan2(goal_xy[1] - boat_xy[1], goal_xy[0] - boat_xy[0]))
    err = abs(wrap_deg(yaw_deg - bearing))
    return dist <= dist_tol and err <= heading_tol, dist, err


class UndockClock:
    """출발 직진 시간 — 모터가 AUTO 인 동안만 쌓고, 한 번에 4 주기보다 크게는 안 쌓는다
    (mission_manager.undock_step 과 같은 규칙)."""

    def __init__(self, seconds: float, period: float = UNDOCK_PERIOD):
        self.seconds = float(seconds)
        self.max_step = 4.0 * period
        self.elapsed = 0.0
        self._last = None

    def update(self, now: float, counting: bool) -> bool:
        if self._last is not None and counting:
            self.elapsed += min(max(now - self._last, 0.0), self.max_step)
        self._last = now
        return self.elapsed >= self.seconds


class HoldTimer:
    """조건이 hold 초 동안 끊기지 않고 이어지면 True."""

    def __init__(self, hold_sec: float):
        self.hold = float(hold_sec)
        self.since = None

    def update(self, ok: bool, now: float) -> bool:
        if not ok:
            self.since = None
            return False
        if self.since is None:
            self.since = now
        return now - self.since >= self.hold


class TournamentSequence(Node):
    def __init__(self, **kwargs):
        super().__init__('tournament_sequence', **kwargs)
        self.declare_parameter('start_point', 'tournament_entry')
        self.declare_parameter('goal_point', 'tournament_end')
        self.declare_parameter('points_file', DEFAULT_POINTS_FILE)
        self.declare_parameter('origin_file', DEFAULT_ORIGIN_FILE)
        self.declare_parameter('undock_sec', UNDOCK_SEC)
        self.declare_parameter('undock_linear', UNDOCK_LINEAR)
        self.declare_parameter('start_dist', 3.0)
        self.declare_parameter('start_heading_deg', 45.0)
        self.declare_parameter('start_hold_sec', 0.5)
        self.declare_parameter('start_service', '/real_avoid_test/start')

        p = lambda name: self.get_parameter(name).value  # noqa: E731
        points, origin = str(p('points_file')), str(p('origin_file'))
        self.start_name, self.goal_name = str(p('start_point')), str(p('goal_point'))
        self.start_pt = recorded_point(self.start_name, points, origin)
        self.goal_pt = recorded_point(self.goal_name, points, origin)
        self.undock_sec = float(p('undock_sec'))
        self.undock_linear = float(p('undock_linear'))
        self.start_dist = float(p('start_dist'))
        self.start_heading = float(p('start_heading_deg'))
        hold = float(p('start_hold_sec'))
        values = (self.undock_sec, self.undock_linear, self.start_dist, self.start_heading, hold)
        if (not all(math.isfinite(v) for v in values) or not 0.0 <= self.undock_sec <= 30.0
                or not 0.0 < self.undock_linear <= 1.0 or self.start_dist <= 0.0
                or not 0.0 < self.start_heading <= 180.0 or hold < 0.0):
            raise ValueError('undock_sec 0~30, undock_linear (0,1], start_dist > 0, '
                             'start_heading_deg (0,180], start_hold_sec ≥ 0')

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.state_pub = self.create_publisher(String, '/mission/state', latched)
        self.status_pub = self.create_publisher(String, '/tournament_sequence/status', latched)
        self.undock_pub = self.create_publisher(Twist, '/cmd/undock', 10)
        self.create_subscription(Odometry, '/odom', self._on_odom, qos_profile_sensor_data)
        self.create_subscription(String, '/teleop/status', self._on_motor, 10)
        self.create_subscription(String, '/waypoint/status', self._on_goto, latched)
        self.create_subscription(String, '/real_avoid_test/status', self._on_test, 10)
        # 직진이 실제로 /cmd_vel 로 나가는지 (cmd_mux 가 undock 을 받아 통과시키는지) 확인용
        self.create_subscription(Twist, '/cmd_vel', self._on_cmd_vel, 10)
        self.start_client = self.create_client(Trigger, str(p('start_service')))

        self.odom = None            # (x, y, yaw_deg, 받은 시각)
        self.motor = (None, None)   # (모드, 받은 시각)
        self.goto_status = None
        self.test_status = None
        self.cmd_echo = None        # (linear, angular, 받은 시각) — /cmd_vel
        self.mux_ok = False
        self.undock_clock = UndockClock(self.undock_sec)
        self.hold = HoldTimer(hold)
        self.pending = None         # 시작 서비스 응답 기다리는 future
        self.pending_since = None
        self.last_conflict_warn = None
        self.next_try = 0.0
        self.last_print = 0.0
        self.last_warn = None
        self.finished_logged = False
        self.phase = None

        self.get_logger().info(
            f'토너먼트 한 번에 — 출발 직진 {self.undock_sec:g} s (세기 {self.undock_linear:g}) → goto {self.start_name} '
            f'({self.start_pt.x:.2f}, {self.start_pt.y:.2f}) → 시작 조건 {self.start_dist:g} m 안 + '
            f'{self.goal_name} 방향 ±{self.start_heading:g}° 를 {hold:g} s 유지하면 회피 시험 출발')
        self._enter('undock' if self.undock_sec > 0.0 else 'approach')
        self.create_timer(UNDOCK_PERIOD, self._tick)

    # ── 입력 ──────────────────────────────────────────
    @staticmethod
    def _now() -> float:
        return time.monotonic()

    def _on_odom(self, msg):
        q = msg.pose.pose.orientation
        yaw = math.degrees(math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                      1.0 - 2.0 * (q.y * q.y + q.z * q.z)))
        self.odom = (msg.pose.pose.position.x, msg.pose.pose.position.y, yaw, self._now())

    def _on_motor(self, msg):
        self.motor = (msg.data.strip().upper(), self._now())

    def _on_cmd_vel(self, msg):
        self.cmd_echo = (msg.linear.x, msg.angular.z, self._now())

    def _mux_ready(self) -> bool:
        """cmd_mux 가 /mission/state 와 /cmd/undock 을 구독하는 게 보이나 (한 번 보이면 계속 True)."""
        if not self.mux_ok:
            self.mux_ok = all(
                any(info.node_name == MUX_NODE for info in self.get_subscriptions_info_by_topic(t))
                for t in ('/mission/state', '/cmd/undock'))
        return self.mux_ok

    def _undock_applied(self, now) -> bool:
        """/cmd_vel 에 지금 출발 직진이 나가고 있나."""
        if self.cmd_echo is None:
            return False
        lin, ang, at = self.cmd_echo
        return (now - at <= CMD_ECHO_SEC and abs(lin - self.undock_linear) < 1e-3
                and abs(ang) < 1e-3)

    def _on_goto(self, msg):
        if msg.data != self.goto_status and self.phase == 'approach':
            self.get_logger().info(f'goto: {msg.data}')
        self.goto_status = msg.data

    def _on_test(self, msg):
        self.test_status = msg.data
        if self.phase == 'avoid' and msg.data.startswith('FINISHED') and not self.finished_logged:
            self.finished_logged = True
            self.get_logger().info(f'✓ 토너먼트 도착 — {msg.data}')
            self._status('FINISHED')

    def _motor_auto(self, now) -> bool:
        mode, at = self.motor
        return mode == 'AUTO' and at is not None and now - at <= THRUSTER_STATUS_TIMEOUT

    # ── 단계 ──────────────────────────────────────────
    def _status(self, text):
        self.status_pub.publish(String(data=text))

    def _enter(self, phase):
        self.phase = phase
        self.state_pub.publish(String(data=phase))
        self._status(phase.upper())
        self.get_logger().info(f'▶ {phase}')

    def _tick(self):
        now = self._now()
        conflict = self._conflict(now)
        if self.phase == 'undock':
            self._undock_tick(now, conflict)
        elif self.phase == 'approach':
            self._approach_tick(now)

    def _conflict(self, now) -> bool:
        """/cmd_vel 을 cmd_mux 말고도 내는 노드, /mission/state 를 이 노드 말고도 내는 노드가 있나.

        있으면 명령이 섞인다(예: 다른 터미널에 남은 test_tournament·goto·run_autonomy). 출발 직진은 이게
        풀릴 때까지 안 낸다 — goto 의 CMD_VEL_PUBLISHER_CONFLICT 와 같은 생각. 그 뒤 단계에선 경고만.
        """
        cmd = self.count_publishers('/cmd_vel')
        state = self.count_publishers('/mission/state')
        if cmd <= 1 and state <= 1:
            return False
        if self.last_conflict_warn is None or now - self.last_conflict_warn >= CONFLICT_WARN_SEC:
            self.last_conflict_warn = now
            self.get_logger().error(
                f'/cmd_vel 발행자 {cmd}개·/mission/state 발행자 {state}개 (각각 1개여야 함) — 다른 터미널의 '
                'test_*·goto·run_autonomy 를 끌 것 (ros2 topic info /cmd_vel -v)'
                + (' — 출발 직진 안 냄' if self.phase == 'undock' else ''))
        return True

    def _undock_tick(self, now, conflict=False):
        """출발 직진 — 셈은 '모터 AUTO' 이면서 '/cmd_vel 에 직진이 보일 때'만.

        cmd_mux 가 undock state 를 늦게 받으면(발견 지연) 그동안 /cmd/undock 을 버리는데, 예전엔 그 시간도
        세어 직진이 1.0 s 로 줄었다 (2026-11-14 시뮬 s1). 모터 상태를 못 받았거나 cmd_mux 가 아직 안 보이면
        직진을 내지 않는다 (모터 상태 2.7 s 지연 때 직진이 5.75 s 로 늘었던 것).
        """
        if conflict:
            self.undock_clock.update(now, False)   # 내지 않는 동안은 안 셈
            return
        wait = None
        if self.motor[0] is None:
            wait = '모터 상태(/teleop/status)를 받으면 시작 (run_motor 가 켜져 있는지)'
        elif not self._mux_ready():
            wait = 'cmd_mux 연결을 기다리는 중'
        if wait is not None:
            self.undock_clock.update(now, False)
            if self.last_warn is None or now - self.last_warn >= 5.0:
                self.last_warn = now
                self.get_logger().warn(f'출발 직진 대기 — {wait}')
            return
        cmd = Twist()
        cmd.linear.x = self.undock_linear
        self.undock_pub.publish(cmd)
        auto = self._motor_auto(now)
        applied = self._undock_applied(now)
        if self.undock_clock.update(now, auto and applied):
            # 정지 명령은 따로 안 낸다 — state 가 approach 로 바뀌면 cmd_mux 가 goto 명령을 바로 통과
            self.get_logger().info(
                f'출발 직진 끝 ({self.undock_clock.elapsed:.1f} s) — goto 명령으로 이어감')
            self._enter('approach')
            return
        if not (auto and applied) and (self.last_warn is None or now - self.last_warn >= 5.0):
            self.last_warn = now
            why = (f'모터 모드 {self.motor[0]} (AUTO 일 때만 셈 — 텔레옵을 놓거나 비상 정지를 풀면 이어서)'
                   if not auto else 'cmd_mux 가 아직 직진을 /cmd_vel 로 안 냄')
            self.get_logger().warn(
                f'출발 직진 대기 — {why} ({self.undock_clock.elapsed:.1f}/{self.undock_sec:g} s)')

    def _approach_tick(self, now):
        head = (self.test_status or '').split(':', 1)[0]
        if head in RUNNING_STATES:
            # 서비스 응답을 놓쳐도(유실·늦음) 시험이 달리기 시작했으면 그 명령으로 — 안 넘기면 시험은 달리는데
            # cmd_mux 는 goto 명령을 계속 통과시킨다
            self.get_logger().warn(f'회피 시험이 이미 달리는 중 ({self.test_status[:40]}) — 시험 명령으로')
            self.pending = None
            self._enter('avoid')
            return
        if self.pending is not None:
            if now - self.pending_since <= PENDING_TIMEOUT_SEC:
                return
            self.get_logger().warn(f'시작 서비스 응답이 {PENDING_TIMEOUT_SEC:g} s 넘게 없음 — 다시 부름')
            self.pending = None
        if self.odom is None or now - self.odom[3] > ODOM_STALE_SEC:
            self.hold.update(False, now)
            if now - self.last_print >= 2.0:
                self.last_print = now
                self.get_logger().warn('/odom 기다리는 중')
            return
        x, y, yaw, _ = self.odom
        ok, dist, err = start_condition((x, y), yaw, (self.start_pt.x, self.start_pt.y),
                                        (self.goal_pt.x, self.goal_pt.y),
                                        self.start_dist, self.start_heading)
        held = self.hold.update(ok, now)
        if now - self.last_print >= 1.0:
            self.last_print = now
            self.get_logger().info(
                f'goto 중 — {self.start_name} 까지 {dist:.1f} m (≤ {self.start_dist:g}), 선수각 오차 '
                f'{err:.0f}° (≤ {self.start_heading:g}){"  [유지 중]" if ok else ""}')
        if not held or now < self.next_try:
            return
        if not self.start_client.service_is_ready():
            self.next_try = now + RETRY_SEC
            self.get_logger().warn('회피 시험 노드 기다리는 중 (시작 서비스 없음)',
                                   throttle_duration_sec=2.0)
            return
        self.get_logger().info(f'✓ 시작 조건 만족 — 거리 {dist:.2f} m, 선수각 오차 {err:.0f}° → 회피 시험 출발')
        self.pending_since = now
        self.pending = self.start_client.call_async(Trigger.Request())
        self.pending.add_done_callback(self._on_start_result)

    def _on_start_result(self, future):
        if future is not self.pending or self.phase != 'approach':
            return      # 버린 옛 호출의 늦은 응답 — 시험이 달리면 위 상태 검사가 넘긴다
        self.pending = None
        try:
            res = future.result()
        except Exception as exc:  # noqa: BLE001 — 서비스 실패도 다시 시도
            res = None
            reason = repr(exc)
        if res is not None and (res.success or 'already running' in res.message):
            # already running = 앞 호출 응답을 놓쳤지만 시험은 이미 출발함
            self.get_logger().info(f'회피 시험 출발: {res.message}')
            self._enter('avoid')
            return
        reason = res.message if res is not None else reason
        self.next_try = self._now() + RETRY_SEC
        self.get_logger().warn(f'회피 시험 출발 거절 — {reason} (goto 계속, {RETRY_SEC:g} s 뒤 다시)',
                               throttle_duration_sec=2.0)


def main(args=None):
    rclpy.init(args=args)
    node = TournamentSequence()
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
