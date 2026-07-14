"""mission_manager — 종합임무 state machine.

순서   gate(항로추종) → station(위치유지) → dock(도킹) → search(탐색)
       → avoid(장애물 회피) → done

전환 규칙 (팀 합의):
  |현재위치(/odom) − 현재 미션 목표좌표| < transition_radius(기본 1.0m)
  이면 다음 state 로 전이한다. behavior 는 완료 신호를 낼 필요가 없다.

발행   /mission/state (String, latched)  — cmd_mux 와 모든 behavior 가 구독
       /mission/goal  (PoseStamped, latched) — 현재 미션의 목표좌표
       /detector/enable (Bool, latched)  — dock_mark_detector YOLO 추론 게이팅
                                            (state == 'dock' 일 때만 true)

미션별 목표좌표는 파라미터(waypoints.<state>)로 주입한다
— kaboat_bringup/config/mission_params.yaml 참고.
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

        self.declare_parameter('transition_radius', 1.0)   # 전환 판정 반경 [m]
        self.declare_parameter('auto_start', True)          # 켜지면 바로 첫 미션 시작
        for name in MISSION_ORDER:
            self.declare_parameter(f'waypoints.{name}', DEFAULT_WAYPOINTS[name])

        self.radius = float(self.get_parameter('transition_radius').value)
        self.waypoints = {
            name: list(self.get_parameter(f'waypoints.{name}').value)
            for name in MISSION_ORDER
        }

        self.odom = None
        self.idx = -1   # MISSION_ORDER 인덱스 (-1 = 시작 전)

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub_state = self.create_publisher(String, '/mission/state', latched)
        self.pub_goal = self.create_publisher(PoseStamped, '/mission/goal', latched)
        # v5: YOLO 추론 게이팅 — dock_mark_detector 는 이 신호가 true 인 동안만 추론
        self.pub_detector_enable = self.create_publisher(Bool, '/detector/enable', latched)

        self.create_subscription(Odometry, '/odom', self.on_odom, 10)
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
            self.set_state('done')
            self.get_logger().info('종합임무 전체 완료 — done (cmd_mux 정지 상태)')
            return
        state = MISSION_ORDER[self.idx]
        self.set_state(state)
        wp = self.waypoints[state]

        goal = PoseStamped()
        goal.header.frame_id = 'odom'
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = float(wp[0])
        goal.pose.position.y = float(wp[1])
        goal.pose.orientation.w = 1.0
        self.pub_goal.publish(goal)

        self.get_logger().info(
            f"미션 [{self.idx+1}/{len(MISSION_ORDER)}] '{state}' 시작 — 목표 ({wp[0]}, {wp[1]})")

    def set_state(self, state: str):
        msg = String()
        msg.data = state
        self.pub_state.publish(msg)
        # 도킹 구간에서만 YOLO 추론 ON (v5 게이팅 규약)
        enable = Bool()
        enable.data = (state == 'dock')
        self.pub_detector_enable.publish(enable)

    # ── 1m 전환 판정 ──────────────────────────────────
    def on_odom(self, msg: Odometry):
        self.odom = msg

    def tick(self):
        if self.odom is None or not (0 <= self.idx < len(MISSION_ORDER)):
            return
        wp = self.waypoints[MISSION_ORDER[self.idx]]
        dx = wp[0] - self.odom.pose.pose.position.x
        dy = wp[1] - self.odom.pose.pose.position.y
        if math.hypot(dx, dy) < self.radius:
            self.get_logger().info(
                f"목표 {self.radius}m 이내 진입 — '{MISSION_ORDER[self.idx]}' 완료")
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
