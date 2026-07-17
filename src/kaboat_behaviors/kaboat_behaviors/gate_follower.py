"""gate_follower — 항로추종 behavior. 게이트(빨강-초록 쌍) 중점 통과.

skeleton — 로직 우선순위:
  1) 카메라에 red + green 표식이 같이 보이면, 그중 **거리(distance)가
     가장 비슷한 빨강-초록 조합**을 같은 게이트 쌍으로 간주해 그 둘의
     bearing 중점으로 조향한다.
     distance 가 -1.0(무효, 뎁스 유효범위 밖 등)인 mark 는 애초에 제외한다.
  2) 안 보이면 goal 웨이포인트를 향해 주행 (seek_goal)
  항상 obstacle_avoidance_utils 로 회피 보정.

TODO(팀): 통과 판정, 쌍 매칭에 거리 임계값(너무 다르면 애초에 페어 아님으로
          간주하고 seek_goal 로 폴백) 추가
"""
import rclpy

from .behavior_base import BehaviorBase, MAX_THRUST
from .obstacle_avoidance_utils import apply_repulsion


class GateFollower(BehaviorBase):
    STATE_NAME = 'gate'
    CMD_TOPIC = '/cmd/gate'

    def compute_cmd(self):
        # self.buoys 의 원소(m)는 kaboat_msgs/Mark 하나:
        #   string  color       "red" | "green" | "orange" | "blue" | "unknown"
        #   string  shape       "circle" | "triangle" | "cross" | "unknown"
        #   float32 confidence  0.0 ~ 1.0
        #   float32 bearing     카메라 광축 기준 방위각 [rad], 좌측 +
        #   float32 distance    추정 거리 [m], 미상이면 -1.0
        reds = [m for m in self.buoys if m.color == 'red' and m.distance > 0.0]
        greens = [m for m in self.buoys if m.color == 'green' and m.distance > 0.0]

        if reds and greens:
            # 거리가 가장 비슷한 빨강-초록 조합 = 같은 게이트 쌍 (모듈 docstring 참고)
            r, g = min(((r, g) for r in reds for g in greens),
                       key=lambda rg: abs(rg[0].distance - rg[1].distance))
            mid_bearing = (r.bearing + g.bearing) / 2.0
            cmd = self.seek_goal()          # 기본 전진 성분
            # 게이트 중점 우선 조향 — 오차 1rad 당 차동 60N (구 게인 1.0 동등)
            cmd.angular.z = (60.0 / MAX_THRUST) * mid_bearing
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
