"""station_keeper — 위치유지 behavior. 목표점 5m 이내에서 5초 유지.

skeleton:
  goal 을 향한 P 제어로 접근/유지하고, 5m 이내 체류 시간을 로그로 출력.
  5m/5s 유지 달성 시 완료 신호를 보내고, mission_manager 가 목표점 2m
  이내 조건까지 확인한 뒤 다음 미션으로 전환한다.

TODO(팀): 바람/파도 드리프트 보상 (heading 유지 포함 station keeping)
"""
import rclpy

from .behavior_base import BehaviorBase
from .obstacle_avoidance_utils import apply_repulsion

HOLD_RADIUS = 5.0   # 규정 유지 반경 [m]
HOLD_TIME = 5.0     # 규정 유지 시간 [s]


class StationKeeper(BehaviorBase):
    STATE_NAME = 'station'
    CMD_TOPIC = '/cmd/station'

    def __init__(self):
        super().__init__()
        self._inside_since = None
        self._reported = False

    def on_activate(self):
        self._inside_since = None
        self._reported = False

    def compute_cmd(self):
        dist = self.distance_to_goal()

        # 5m/5s 유지 완료를 보고한다 (최종 전환은 mission_manager 몫).
        now = self.get_clock().now()
        if dist <= HOLD_RADIUS:
            if self._inside_since is None:
                self._inside_since = now
            elif not self._reported and \
                    (now - self._inside_since).nanoseconds * 1e-9 >= HOLD_TIME:
                self.get_logger().info(f'위치유지 달성 — {HOLD_RADIUS}m 이내 {HOLD_TIME}s 유지 완료')
                self._reported = True
                self.report_complete()
        else:
            self._inside_since = None

        cmd = self.seek_goal(slow_radius=5.0)
        return apply_repulsion(cmd, self.occupancy_grid, self.odom)


def main(args=None):
    rclpy.init(args=args)
    node = StationKeeper()
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
