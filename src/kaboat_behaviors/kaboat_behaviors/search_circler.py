"""search_circler — 탐색 behavior. 표식 색에 따라 좌/우 선회.

skeleton:
  goal(탐색 구역 중심) 근처까지 이동한 뒤, 인식된 표식 색에 따라
  좌(green)/우(red) 선회. 표식이 안 보이면 제자리 저속 선회로 탐색.

TODO(팀): 규정의 정확한 선회 방향/횟수 반영, 선회 반경 제어
"""
import rclpy

from .behavior_base import BehaviorBase
from .obstacle_avoidance_utils import apply_repulsion

# 탐색은 목표점 0.8m 이내에서 시작한다. 따라서 완료 신호가 발생할 때
# mission_manager 의 별도 2m 웨이포인트 조건도 함께 만족한다.
ARRIVE_RADIUS = 0.8
SEARCH_TIME = 5.0


class SearchCircler(BehaviorBase):
    STATE_NAME = 'search'
    CMD_TOPIC = '/cmd/search'

    def __init__(self):
        super().__init__()
        self._circle_since = None

    def on_activate(self):
        self._circle_since = None

    def compute_cmd(self):
        # 아직 구역 밖이면 접근부터
        if self.distance_to_goal() > ARRIVE_RADIUS:
            self._circle_since = None
            cmd = self.seek_goal()
            return apply_repulsion(cmd, self.occupancy_grid, self.odom)

        # 구역 안 — 색 따라 선회 (skeleton 규칙: green 좌선회, red 우선회)
        # 부표맵은 뒤 것·먼 것도 보관하므로 전방 시야(visible_buoys)로 걸러
        # "지금 보이는" 색만 본다.
        from geometry_msgs.msg import Twist
        cmd = Twist()
        now = self.get_clock().now()
        if self._circle_since is None:
            self._circle_since = now
        elif ((now - self._circle_since).nanoseconds * 1e-9 >= SEARCH_TIME):
            self.report_complete()

        greens = self.visible_buoys('green')
        reds = self.visible_buoys('red')
        if greens and not reds:
            cmd.angular.z = +self.max_angular
            cmd.linear.x = self.max_linear * 0.5
        elif reds and not greens:
            cmd.angular.z = -self.max_angular
            cmd.linear.x = self.max_linear * 0.5
        else:
            cmd.angular.z = +self.max_angular * 0.5  # 표식 미발견 — 탐색 선회
        return apply_repulsion(cmd, self.occupancy_grid, self.odom)


def main(args=None):
    rclpy.init(args=args)
    node = SearchCircler()
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
