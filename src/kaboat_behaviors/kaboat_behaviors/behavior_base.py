"""behavior 공통 base 클래스.

모든 behavior 노드가 상속한다. 하는 일:
  - /mission/state 구독 → 자기 state 일 때만 active
  - /mission/goal, /odom, /obstacles, /detections/marks 구독 (공유 데이터 버스)
  - active 인 동안 10Hz 로 compute_cmd() 를 호출해 /cmd/<이름> 에 Twist 발행
  - 속도 상한 (twist2thrust 의 scale=60 기준, 추력이 안전 범위를 넘지 않게)

비활성일 때는 아무것도 발행하지 않는다 — cmd_mux 는 활성 behavior 의
명령만 통과시키고, 300ms 워치독으로 끊김을 감지해 정지시킨다.
"""
import math

from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy

from std_msgs.msg import String
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry

from kaboat_msgs.msg import ObstacleArray, MarkArray

# twist2thrust scale=60 기준 속도 한계 (linear 0.2 → 12N, 직진 테스트 검증값).
# ⚠️ 회전은 특히 보수적으로 — 축소 선체(1.1m)는 관성이 s⁵ 로 줄어 원본보다
# 약 20배 민첩해서, 회전 명령이 크면 요 발진 → 선수 처박힘 → 전복한다 (실측 2회).
DEFAULT_MAX_LINEAR = 0.20    # linear.x 상한
DEFAULT_MAX_ANGULAR = 0.05   # angular.z 상한 (차동추력 ±3N)
YAW_KP = 0.3                 # 방위 P 게인
YAW_KD = 0.15                # 요레이트 감쇠 (발진 방지)


def yaw_from_quaternion(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def normalize_angle(a: float) -> float:
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


class BehaviorBase(Node):
    """서브클래스는 STATE_NAME, CMD_TOPIC 을 정의하고 compute_cmd() 를 구현한다."""

    STATE_NAME = ''   # 예: 'gate' — /mission/state 가 이 값일 때만 active
    CMD_TOPIC = ''    # 예: '/cmd/gate'

    def __init__(self):
        super().__init__(f'{self.STATE_NAME}_behavior')

        self.declare_parameter('max_linear', DEFAULT_MAX_LINEAR)
        self.declare_parameter('max_angular', DEFAULT_MAX_ANGULAR)
        self.max_linear = float(self.get_parameter('max_linear').value)
        self.max_angular = float(self.get_parameter('max_angular').value)

        self.state = ''
        self.goal = None          # PoseStamped | None
        self.odom = None          # Odometry | None
        self.obstacles = []       # list[kaboat_msgs.Obstacle]
        self.obstacles_frame = ''
        self.marks = []           # list[kaboat_msgs.Mark]

        # mission_manager 가 latched(transient_local) 로 발행 — 늦게 떠도 수신
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, '/mission/state', self._on_state, latched)
        self.create_subscription(PoseStamped, '/mission/goal', self._on_goal, latched)

        self.create_subscription(Odometry, '/odom', self._on_odom, 10)
        self.create_subscription(ObstacleArray, '/obstacles', self._on_obstacles, 10)
        self.create_subscription(MarkArray, '/detections/marks', self._on_marks, 10)

        self.cmd_pub = self.create_publisher(Twist, self.CMD_TOPIC, 10)
        self.create_timer(0.1, self._tick)  # 10Hz

        self.get_logger().info(
            f"behavior '{self.STATE_NAME}' 대기 — active 조건 /mission/state == '{self.STATE_NAME}'")

    # ── 콜백 ─────────────────────────────────────────
    def _on_state(self, msg: String):
        if msg.data != self.state:
            self.get_logger().info(f"state 변경 '{self.state}' → '{msg.data}' "
                                   f"({'활성' if msg.data == self.STATE_NAME else '대기'})")
        self.state = msg.data

    def _on_goal(self, msg: PoseStamped):
        self.goal = msg

    def _on_odom(self, msg: Odometry):
        self.odom = msg

    def _on_obstacles(self, msg: ObstacleArray):
        self.obstacles = list(msg.obstacles)
        self.obstacles_frame = msg.header.frame_id

    def _on_marks(self, msg: MarkArray):
        self.marks = list(msg.marks)

    # ── 주기 실행 ─────────────────────────────────────
    @property
    def active(self) -> bool:
        return self.state == self.STATE_NAME

    def _tick(self):
        if not self.active:
            return
        cmd = self.compute_cmd()
        if cmd is None:
            return
        cmd.linear.x = max(-self.max_linear, min(self.max_linear, cmd.linear.x))
        cmd.angular.z = max(-self.max_angular, min(self.max_angular, cmd.angular.z))
        self.cmd_pub.publish(cmd)

    def compute_cmd(self):
        """활성 상태에서 10Hz 로 호출. Twist 를 반환 (None 이면 미발행)."""
        raise NotImplementedError

    # ── 위치 제어 공통 헬퍼 ────────────────────────────
    def distance_to_goal(self) -> float:
        if self.odom is None or self.goal is None:
            return float('inf')
        dx = self.goal.pose.position.x - self.odom.pose.pose.position.x
        dy = self.goal.pose.position.y - self.odom.pose.pose.position.y
        return math.hypot(dx, dy)

    def heading_error_to_goal(self) -> float:
        """목표 방향 − 현재 선수각 (좌회전 양수)."""
        if self.odom is None or self.goal is None:
            return 0.0
        dx = self.goal.pose.position.x - self.odom.pose.pose.position.x
        dy = self.goal.pose.position.y - self.odom.pose.pose.position.y
        target_yaw = math.atan2(dy, dx)
        yaw = yaw_from_quaternion(self.odom.pose.pose.orientation)
        return normalize_angle(target_yaw - yaw)

    def seek_goal(self, slow_radius: float = 3.0) -> Twist:
        """goal 을 향한 PD 제어 Twist. 모든 behavior 의 이동 기본기.

        D항(요레이트 감쇠)이 필수 — P 만 쓰면 민첩한 축소 선체가 발진해 전복한다.
        """
        cmd = Twist()
        if self.odom is None or self.goal is None:
            return cmd  # 데이터 없으면 정지
        err = self.heading_error_to_goal()
        dist = self.distance_to_goal()
        yaw_rate = self.odom.twist.twist.angular.z
        cmd.angular.z = YAW_KP * err - YAW_KD * yaw_rate
        # 목표를 크게 벗어난 방향이면 회전 우선, 가까워지면 감속
        if abs(err) < math.pi / 3:
            cmd.linear.x = self.max_linear * min(dist / slow_radius, 1.0)
        else:
            # 회전 중에도 저속 전진 유지 — 제자리 급회전(전복 위험)을 피한다
            cmd.linear.x = self.max_linear * 0.3
        return cmd
