"""search_circler — 탐색 behavior. 표식 색에 따라 좌/우 원 경로를 추종.

skeleton:
  goal(탐색 구역 중심) 근처까지 이동한 뒤, 인식된 표식 색에 따라
  좌(green)/우(red) 원 경로를 LOS로 추종한다. 표식이 안 보이면
  제자리 저속 선회로 탐색한다.

TODO(팀): 규정의 정확한 선회 방향/횟수 반영
"""
import rclpy
from geometry_msgs.msg import Twist

from .behavior_base import BehaviorBase
from .los_controller import circle_path
from .obstacle_avoidance_utils import apply_repulsion

# 탐색은 목표점 0.8m 이내에서 시작한다. 따라서 완료 신호가 발생할 때
# mission_manager 의 별도 2m 웨이포인트 조건도 함께 만족한다.
ARRIVE_RADIUS = 0.8
SEARCH_TIME = 5.0
SEARCH_CIRCLE_RADIUS = 1.2
SEARCH_THRUST_RATIO = 0.5


class SearchCircler(BehaviorBase):
    STATE_NAME = 'search'
    CMD_TOPIC = '/cmd/search'

    def __init__(self):
        super().__init__()
        self._circle_since = None
        self._circle_path = None
        self._circle_key = None

    def on_activate(self):
        self._circle_since = None
        self._circle_path = None
        self._circle_key = None

    def compute_cmd(self):
        if (self.goal is None or self.goal.header.frame_id != 'odom'
                or not self.waypoint_odom_ready()):
            return Twist()
        # 일단 원 주행을 시작하면 목표 반경보다 큰 원호에서도 추종을 유지한다.
        # 미션 완료 판정 반경(2m) 밖으로 벗어난 경우에만 다시 접근한다.
        distance = self.distance_to_goal()
        if ((self._circle_since is None and distance > ARRIVE_RADIUS)
                or distance > 2.0):
            self._circle_since = None
            self._circle_path = None
            self._circle_key = None
            self._los_follower.reset()
            point = self.goal.pose.position
            cmd, _ = self.waypoint(point.x, point.y, slow_radius=3.0)
            return apply_repulsion(cmd, self.occupancy_grid, self.odom)

        # 구역 안 — 색 따라 선회 (skeleton 규칙: green 좌선회, red 우선회)
        # 부표맵은 뒤 것·먼 것도 보관하므로 전방 시야(visible_buoys)로 걸러
        # "지금 보이는" 색만 본다.
        cmd = Twist()
        now = self.get_clock().now()
        if self._circle_since is None:
            self._circle_since = now
        elif ((now - self._circle_since).nanoseconds * 1e-9 >= SEARCH_TIME):
            self.report_complete()

        greens = self.visible_buoys('green')
        reds = self.visible_buoys('red')
        direction = ('ccw' if greens and not reds else
                     'cw' if reds and not greens else None)
        if direction is None:
            self._circle_path = None
            self._circle_key = None
            self._los_follower.reset()
            cmd.angular.z = +self.max_angular * 0.5  # 표식 미발견 — 탐색 선회
        else:
            point = self.goal.pose.position
            key = (point.x, point.y, direction)
            if key != self._circle_key:
                self._circle_path = circle_path(
                    point.x, point.y, SEARCH_CIRCLE_RADIUS,
                    direction=direction, target_laps=None)
                self._circle_key = key
            cmd, _ = self.los(self._circle_path, SEARCH_THRUST_RATIO)
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
