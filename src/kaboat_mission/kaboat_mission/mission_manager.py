"""mission_manager — 종합임무 state machine.

순서   gate(항로추종) → station(위치유지) → dock(도킹) → search(탐색)
       → avoid(장애물 회피) → done

미션 시작: 각 미션의 entry_waypoints.<state> = [x, y, angle_deg] 로
  approach state 에 진입한다. waypoint() 도달 신호를 받은 뒤에만 해당
  behavior 를 활성화한다. 미설정 entry 는 idle 로 정지한다.

전환 규칙 (팀 합의):
  1) 현재 미션 behavior 가 /mission/complete 로 자기 state 이름을 발행하고
  2) |현재위치(/odom) − 현재 미션 목표좌표| ≤ transition_radius(기본 5.0m)
  두 조건을 모두 만족하면 다음 state 로 전이한다.

발행   /mission/state (String, latched)  — cmd_mux 와 모든 behavior 가 구독
       /mission/goal  (PoseStamped, latched) — 현재 미션의 목표좌표
       /mission/entry_goal (PoseStamped) — 다음 미션 시작 위치/방향
       /detector/enable (Bool, latched)  — dock_mark_detector YOLO 추론 게이팅
                                            (state == 'dock' 일 때만 true)
구독   /mission/complete (String) — 완료한 behavior 의 state 이름

미션별 종료 목표좌표(waypoints.<state>)와 시작 자세
(entry_waypoints.<state>, entry_ready.<state>)는 파라미터로 주입한다
— kaboat_bringup/config/mission_params.yaml 참고.

단일 미션 테스트:
  전체를 순서대로 안 돌리고 특정 미션 하나만 켜서 그 behavior 를 반복
  개발/디버깅하고 싶을 때는 start_state 파라미터를 쓴다.
    ros2 run kaboat_mission mission_manager --ros-args \
        -p start_state:=dock -p single_mission:=true
  start_state 의 접근 단계부터 시작하고, single_mission:=true 면 그
  완료 신호와 목표 5m 조건 달성 후 다음으로 안 넘어가고 바로 'done'(정지)으로 간다 —
  cmd_mux 가 멈추니 배가 목표 근처에서 안전하게 정지한 채로 결과를 볼 수 있다.
  이때도 entry_waypoints.<state>와 entry_ready.<state>=true가 필요하다.
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy

from std_msgs.msg import String, Bool
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry

MISSION_ORDER = ['gate', 'station', 'dock', 'search', 'avoid']

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
    def __init__(self):
        super().__init__('mission_manager')

        self.declare_parameter('transition_radius', 5.0)   # 전환 판정 반경 [m]
        self.declare_parameter('auto_start', True)          # 켜지면 바로 첫 미션 시작
        self.declare_parameter('start_state', '')            # 단일 미션 테스트용 시작 지점
        self.declare_parameter('single_mission', False)      # true 면 start_state 하나만 하고 done
        self.declare_parameter('entry_required', True)      # 단독 미션 preview 전용 우회
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
        for name in MISSION_ORDER:
            pose = self.entry_waypoints[name]
            if len(pose) != 3 or not all(math.isfinite(v) for v in pose):
                raise ValueError(f'entry_waypoints.{name} must be [x, y, angle_deg]')

        self.odom = None
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
        self.create_timer(0.2, self.tick)  # 5Hz 판정

        wp_str = ', '.join(f"{n}({w[0]:.0f},{w[1]:.0f})" for n, w in self.waypoints.items())
        self.get_logger().info(
            f'mission_manager 시작 — 전환 반경 {self.radius}m, 웨이포인트 {wp_str}')

        if bool(self.get_parameter('auto_start').value):
            self.advance()

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
        self.set_state(state)
        self.publish_mission_goal()
        wp = self.waypoints[state]
        self.get_logger().info(
            f"미션 [{self.idx+1}/{len(MISSION_ORDER)}] '{state}' 시작 — 목표 ({wp[0]}, {wp[1]})")

    def publish_mission_goal(self):
        wp = self.waypoints[MISSION_ORDER[self.idx]]
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
        wp = self.waypoints[MISSION_ORDER[self.idx]]
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
    node = MissionManager()
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
