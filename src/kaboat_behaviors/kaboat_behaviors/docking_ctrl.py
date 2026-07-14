"""docking_ctrl — 도킹 behavior. 표식 인식 → 지정 슬롯 접안.

v5: 도킹 표식은 공유 버스의 buoys 가 아니라 **/detections/dock_marks**
(dock_mark_detector, YOLO — dock state 에서만 mission_manager 가
/detector/enable 로 추론을 켠다)를 이 behavior 만 단독 구독한다.

skeleton:
  목표 표식 색(param target_color)이 보이면 그 bearing 으로 저속 접근,
  안 보이면 goal(도킹 구조물 앞 대기점)으로 이동.

TODO(팀): 표식 모양(circle/triangle/square) 매칭 — dock_mark_detector 가
          YOLO 로 교체되면 mark.shape 으로 필터, 접안 최종 단계 정렬
          (dock tooth 폭 1.5m — 선폭 대비 여유 확인), 후진 이탈
"""
import rclpy

from kaboat_msgs.msg import MarkArray

from .behavior_base import BehaviorBase
from .obstacle_avoidance_utils import apply_repulsion


class DockingCtrl(BehaviorBase):
    STATE_NAME = 'dock'
    CMD_TOPIC = '/cmd/dock'

    def __init__(self):
        super().__init__()
        self.declare_parameter('target_color', 'red')   # 당일 지정 표식 색
        self.declare_parameter('target_shape', 'unknown')  # TODO(팀): YOLO 후 circle 등
        self.target_color = self.get_parameter('target_color').value
        self.target_shape = self.get_parameter('target_shape').value

        self.dock_marks = []   # 도킹 전용 검출 (공유 버스와 별도)
        self.create_subscription(
            MarkArray, '/detections/dock_marks', self._on_dock_marks, 10)

    def _on_dock_marks(self, msg: MarkArray):
        self.dock_marks = list(msg.marks)

    def compute_cmd(self):
        targets = [m for m in self.dock_marks if m.color == self.target_color]
        # TODO(팀): YOLO 교체 후 shape 조건 AND — m.shape == self.target_shape

        if targets:
            best = max(targets, key=lambda m: m.confidence)
            cmd = self.seek_goal()
            cmd.angular.z = 1.2 * best.bearing
            cmd.linear.x = self.max_linear * 0.5   # 접안은 저속
        else:
            cmd = self.seek_goal()

        return apply_repulsion(cmd, self.occupancy_grid, self.odom)


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
