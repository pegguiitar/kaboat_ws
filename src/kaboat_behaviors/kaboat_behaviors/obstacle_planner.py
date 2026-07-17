"""obstacle_planner — 장애물 회피 항행 behavior. 장애물 밭 통과.

skeleton:
  goal 을 향해 주행하되 회피 보정을 강하게 적용.
  (다른 behavior 와 같은 apply_repulsion 을 쓰지만, 이 미션은 회피가
   주 임무라서 게인을 높인 전용 파라미터를 쓴다)

TODO(팀): VFH valley 탐색 + DRI 기반 제대로 된 로컬 플래너로 교체
          — 기존 설계(VFH + B-spline)를 여기에 편입
"""
import rclpy

from .behavior_base import BehaviorBase, MAX_THRUST
from . import obstacle_avoidance_utils as oa


class ObstaclePlanner(BehaviorBase):
    STATE_NAME = 'avoid'
    CMD_TOPIC = '/cmd/avoid'

    def compute_cmd(self):
        cmd = self.seek_goal()
        # 장애물 밭 전용 — 회피 게인/범위 강화해서 적용 (54N = 구 게인 0.9 동등)
        saved_gain, saved_range = oa.GAIN, oa.AVOID_RANGE
        oa.GAIN, oa.AVOID_RANGE = 54.0 / MAX_THRUST, 8.0
        try:
            cmd = oa.apply_repulsion(cmd, self.occupancy_grid, self.odom)
        finally:
            oa.GAIN, oa.AVOID_RANGE = saved_gain, saved_range
        return cmd


def main(args=None):
    rclpy.init(args=args)
    node = ObstaclePlanner()
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
