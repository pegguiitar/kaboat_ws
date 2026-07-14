"""cmd_mux — 현재 state 의 behavior 명령만 /cmd_vel 로 통과시키는 선택 스위치.

입력   /cmd/gate /cmd/station /cmd/dock /cmd/search /cmd/avoid (Twist)
       /mission/state (String, latched)
출력   /cmd_vel (Twist) → twist2thrust → 좌우 추력

페일세이프 (워치독):
  활성 behavior 의 명령이 timeout_ms(기본 300ms) 이상 끊기면 /cmd_vel 에
  정지(0)를 발행한다 — behavior 노드가 죽어도 배가 마지막 명령으로 계속
  달리는 것을 방지. state 가 목록에 없으면(idle/done 등) 항상 정지.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy

from std_msgs.msg import String
from geometry_msgs.msg import Twist

# state 이름 → 해당 behavior 의 명령 토픽
STATE_TOPIC_MAP = {
    'gate':    '/cmd/gate',
    'station': '/cmd/station',
    'dock':    '/cmd/dock',
    'search':  '/cmd/search',
    'avoid':   '/cmd/avoid',
}


class CmdMux(Node):
    def __init__(self):
        super().__init__('cmd_mux')

        self.declare_parameter('timeout_ms', 300.0)
        self.timeout = float(self.get_parameter('timeout_ms').value) / 1000.0

        self.state = ''
        self.last_cmd_time = {}   # topic → 마지막 수신 시각

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, '/mission/state', self.on_state, latched)

        for topic in STATE_TOPIC_MAP.values():
            # 클로저 캡처 주의 — topic 을 기본 인자로 고정
            self.create_subscription(
                Twist, topic,
                lambda msg, t=topic: self.on_cmd(t, msg), 10)

        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_timer(0.05, self.watchdog)  # 20Hz 감시

        self._stopped = True   # 마지막에 정지 명령을 냈는지 (중복 로그 방지)
        self.get_logger().info(
            f'cmd_mux 시작 — 워치독 {self.timeout*1000:.0f}ms, 대상 {list(STATE_TOPIC_MAP)}')

    def on_state(self, msg: String):
        if msg.data != self.state:
            self.get_logger().info(f"통과 대상 변경 '{self.state}' → '{msg.data}'")
        self.state = msg.data

    def on_cmd(self, topic: str, msg: Twist):
        if STATE_TOPIC_MAP.get(self.state) != topic:
            return  # 비활성 behavior 의 명령은 버림
        self.last_cmd_time[topic] = self.get_clock().now()
        self.pub.publish(msg)
        self._stopped = False

    def watchdog(self):
        topic = STATE_TOPIC_MAP.get(self.state)
        if topic is None:
            self.stop('비활성 state')   # idle/done 등 — 항상 정지
            return
        last = self.last_cmd_time.get(topic)
        if last is None:
            return  # 아직 첫 명령 전 — behavior 가 뜨기 전 정지 유지 불필요(초기 정지 상태)
        elapsed = (self.get_clock().now() - last).nanoseconds * 1e-9
        if elapsed > self.timeout:
            self.stop(f"'{self.state}' 명령 {elapsed*1000:.0f}ms 끊김")

    def stop(self, reason: str):
        if not self._stopped:
            self.get_logger().warn(f'페일세이프 정지 — {reason}')
            self._stopped = True
        self.pub.publish(Twist())  # 전부 0


def main(args=None):
    rclpy.init(args=args)
    node = CmdMux()
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
