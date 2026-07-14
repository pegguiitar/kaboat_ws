"""docking_ctrl — 도킹 behavior. 표식 인식 → 지정 슬롯 접안.

skeleton:
  목표 표식 색(param target_color)이 보이면 그 bearing 으로 저속 접근,
  안 보이면 goal(도킹 구조물 앞 대기점)으로 이동.

TODO(팀): YOLO 표식 종류(십자/삼각/원) 매칭, 접안 최종 단계 정렬
          (dock tooth 폭 1.5m — 선폭 대비 여유 확인), 후진 이탈
"""
import rclpy

from .behavior_base import BehaviorBase
from .obstacle_avoidance_utils import apply_repulsion


class DockingCtrl(BehaviorBase):
    STATE_NAME = 'dock'
    CMD_TOPIC = '/cmd/dock'

    def __init__(self):
        super().__init__()
        self.declare_parameter('target_color', 'red')  # 당일 지정 표식 색
        self.target_color = self.get_parameter('target_color').value

    def compute_cmd(self):
        targets = [m for m in self.marks if m.color == self.target_color]

        if targets:
            best = max(targets, key=lambda m: m.confidence)
            cmd = self.seek_goal()
            cmd.angular.z = 1.2 * best.bearing
            cmd.linear.x = self.max_linear * 0.5   # 접안은 저속
        else:
            cmd = self.seek_goal()

        return apply_repulsion(cmd, self.obstacles, self.odom, self.obstacles_frame)


def main(args=None):
    rclpy.init(args=args)
    node = DockingCtrl()
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
