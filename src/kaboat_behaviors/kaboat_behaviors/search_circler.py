"""search_circler — 탐색 behavior. 표식 색에 따라 좌/우 선회.

skeleton:
  goal(탐색 구역 중심) 근처까지 이동한 뒤, 인식된 표식 색에 따라
  좌(green)/우(red) 선회. 표식이 안 보이면 제자리 저속 선회로 탐색.

TODO(팀): 규정의 정확한 선회 방향/횟수 반영, 선회 반경 제어
"""
import rclpy

from .behavior_base import BehaviorBase
from .obstacle_avoidance_utils import apply_repulsion

# 도착 판정 반경 — mission_manager 의 전환 반경(1m)보다 작아야 한다.
# 크게 잡으면 목표점 밖에서 선회만 하다가 미션 전환(1m 진입)이 영영 안 일어난다.
ARRIVE_RADIUS = 0.8


class SearchCircler(BehaviorBase):
    STATE_NAME = 'search'
    CMD_TOPIC = '/cmd/search'

    def compute_cmd(self):
        # 아직 구역 밖이면 접근부터
        if self.distance_to_goal() > ARRIVE_RADIUS:
            cmd = self.seek_goal()
            return apply_repulsion(cmd, self.obstacles, self.odom, self.obstacles_frame)

        # 구역 안 — 색 따라 선회 (skeleton 규칙: green 좌선회, red 우선회)
        from geometry_msgs.msg import Twist
        cmd = Twist()
        greens = [m for m in self.marks if m.color == 'green']
        reds = [m for m in self.marks if m.color == 'red']
        if greens and not reds:
            cmd.angular.z = +self.max_angular
            cmd.linear.x = self.max_linear * 0.5
        elif reds and not greens:
            cmd.angular.z = -self.max_angular
            cmd.linear.x = self.max_linear * 0.5
        else:
            cmd.angular.z = +self.max_angular * 0.5  # 표식 미발견 — 탐색 선회
        return apply_repulsion(cmd, self.obstacles, self.odom, self.obstacles_frame)


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
