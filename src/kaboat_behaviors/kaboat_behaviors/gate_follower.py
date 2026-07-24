"""gate_follower — 항로추종 behavior. 게이트(빨강-초록 쌍) 중점 통과.

skeleton — 로직 우선순위:
  1) 전방 시야에 red + green 부표가 같이 보이면, 그중 **거리가 가장 비슷한
     빨강-초록 조합**을 같은 게이트 쌍으로 간주해 그 둘의 방위 중점으로 조향.
     부표는 전역(odom) 좌표 부표맵으로 오므로, 거리·방위는 base 의
     visible_buoys() 가 자기 odom 으로 계산한다(전방 ±50°·15m 필터로
     "지금 보이는" 것만 — 부표맵은 뒤 것·먼 것도 보관하기 때문).
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
        # 현재 skeleton 의 gate 완료 정의는 채널 출구 waypoint 2m 이내 진입.
        # mission_manager 도 독립적으로 같은 2m 위치 조건을 확인한다.
        if self.distance_to_goal() <= 2.0:
            self.report_complete()

        # visible_buoys('red') → [(buoy, 거리[m], 방위[rad,좌+]), ...] (가까운 순).
        # buoy 는 kaboat_msgs/Buoy (id·color·position(odom좌표)·confidence).
        reds = self.visible_buoys('red')
        greens = self.visible_buoys('green')

        if reds and greens:
            # 거리가 가장 비슷한 빨강-초록 조합 = 같은 게이트 쌍 (모듈 docstring 참고)
            r, g = min(((r, g) for r in reds for g in greens),
                       key=lambda rg: abs(rg[0][1] - rg[1][1]))   # [1]=거리
            mid_bearing = (r[2] + g[2]) / 2.0                     # [2]=방위
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
