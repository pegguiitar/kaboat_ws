"""mission_manager — 종합임무 state machine.

순서   gate(항로추종) → station(위치유지) → dock(도킹) → search(탐색)
       → avoid(장애물 회피) → done

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
       /detector/enable (Bool, latched)  — dock_mark_detector YOLO 추론 게이팅
                                            (state == 'dock' 일 때만 true)
구독   /mission/complete (String) — 완료한 behavior 의 state 이름

미션별 종료 목표좌표(waypoints.<state>)와 시작 자세
(entry_waypoints.<state>, entry_ready.<state>)는 파라미터로 주입한다
— kaboat_bringup/config/mission_params.yaml 참고.

현장 기록 지점 (rec): points_file(기본 ~/kaboat_mission_points.json)이 있으면
  <미션>_entry → entry_waypoints.<미션> + entry_ready.<미션>=true
  <미션>_end   → waypoints.<미션>
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
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path

from kaboat_mission.mission_points import (
    DEFAULT_ORIGIN_FILE, DEFAULT_POINTS_FILE, MISSION_STATES, PointStore,
    mission_slot, mission_targets, read_origin_file)

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
        # behavior 완료 거리(gate·avoid 2.0, search 원 1.2)와 맞춘다.
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

        self.radius = float(self.get_parameter('transition_radius').value)
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
        # v5: YOLO 추론 게이팅 — dock_mark_detector 는 이 신호가 true 인 동안만 추론
        self.pub_detector_enable = self.create_publisher(Bool, '/detector/enable', latched)

        self.create_subscription(Odometry, '/odom', self.on_odom, 10)
        self.create_subscription(String, '/mission/complete', self.on_complete, 10)
        self.create_subscription(Path, '/dock/one_shot_path', self.on_dock_path, latched)
        self.create_timer(0.2, self.tick)  # 5Hz 판정

        wp_str = ', '.join(f"{n}({w[0]:.0f},{w[1]:.0f})" for n, w in self.waypoints.items())
        self.get_logger().info(
            f'mission_manager 시작 — 전환 반경 {self.radius}m, 웨이포인트 {wp_str}')

        if bool(self.get_parameter('auto_start').value):
            self.advance()

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
        missing = [s for s in MISSION_ORDER if s not in targets]
        if missing:
            self.get_logger().warning(
                f'기록 지점이 없는 미션 {missing} — mission_params 값 사용 '
                '(entry_ready=false 면 그 미션에서 멈춤)')
        unused = [n for n in store.points if mission_slot(n, MISSION_ORDER) is None]
        if unused:
            self.get_logger().info(f'미션 이름 규칙이 아닌 지점(안 씀): {unused}')

    # ── state 전이 ────────────────────────────────────
    def advance(self):
        self.idx += 1
        if self.idx >= len(MISSION_ORDER):
            self.phase = 'done'
            self.set_state('done')
            self.get_logger().info('종합임무 전체 완료 — done (cmd_mux 정지 상태)')
            return
        state = MISSION_ORDER[self.idx]
        if self.entry_required:
            if not self.entry_ready[state]:
                self.phase = 'blocked'
                self.set_state('idle')
                self.get_logger().error(
                    f"'{state}' 시작 자세 미설정 — entry_waypoints.{state}와 "
                    f'entry_ready.{state}=true 설정 후 재시작 필요')
                return
            self.phase = 'approach'
            self.set_state('approach')
            self.publish_entry_goal()
            pose = self.entry_waypoints[state]
            self.get_logger().info(
                f"미션 [{self.idx+1}/{len(MISSION_ORDER)}] '{state}' 접근 — "
                f'({pose[0]}, {pose[1]}), 선수각 {pose[2]}°')
        else:
            self.start_mission()

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

    def start_mission(self):
        self.phase = 'mission'
        state = MISSION_ORDER[self.idx]
        if state == 'dock':
            self.dock_route_goal = None
        self.set_state(state)
        self.publish_mission_goal()
        wp = self.waypoints[state]
        self.get_logger().info(
            f"미션 [{self.idx+1}/{len(MISSION_ORDER)}] '{state}' 시작 — 목표 ({wp[0]}, {wp[1]})")

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
