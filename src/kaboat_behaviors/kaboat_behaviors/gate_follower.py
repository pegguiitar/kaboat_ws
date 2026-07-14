"""gate_follower — 항로추종 behavior. 게이트(빨강-초록 쌍) 중점 통과.

skeleton — 로직 우선순위:
  1) 카메라에 red + green 표식이 같이 보이면 두 bearing 의 중점으로 조향
  2) 안 보이면 goal 웨이포인트를 향해 주행 (seek_goal)
  항상 obstacle_avoidance_utils 로 회피 보정.

TODO(팀): 게이트 쌍 매칭(가장 가까운 red-green 짝짓기), 통과 판정,
          LiDAR /obstacles 와 융합해 게이트 기둥 거리 확정
"""
import rclpy

from .behavior_base import BehaviorBase
from .obstacle_avoidance_utils import apply_repulsion


class GateFollower(BehaviorBase):
    STATE_NAME = 'gate'
    CMD_TOPIC = '/cmd/gate'

    def compute_cmd(self):
        reds = [m for m in self.buoys if m.color == 'red']
        greens = [m for m in self.buoys if m.color == 'green']

        if reds and greens:
            # 가장 확신도 높은 쌍의 중점 방위로 조향
            r = max(reds, key=lambda m: m.confidence)
            g = max(greens, key=lambda m: m.confidence)
            mid_bearing = (r.bearing + g.bearing) / 2.0
            cmd = self.seek_goal()          # 기본 전진 성분
            cmd.angular.z = 1.0 * mid_bearing  # 게이트 중점 우선 조향
            cmd.linear.x = self.max_linear * 0.8
        else:
            cmd = self.seek_goal()

        return apply_repulsion(cmd, self.occupancy_grid, self.odom)


def main(args=None):
    rclpy.init(args=args)
    node = GateFollower()
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
