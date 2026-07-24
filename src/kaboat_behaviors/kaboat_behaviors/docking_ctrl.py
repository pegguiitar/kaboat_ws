"""docking_ctrl — 표식 기반 도킹 FSM의 ROS 배선 노드.

v5: 도킹 표식은 공유 버스의 buoys 가 아니라 **/detections/dock_marks**
(dock_mark_detector, YOLO — dock state 에서만 mission_manager 가
/detector/enable 로 추론을 켠다)를 이 behavior 만 단독 구독한다.

  APPROACH → ACQUIRE → ALIGN → ENTER → HOLD → REVERSE → COMPLETE

판단과 전이는 ROS 비의존 docking_fsm.py에 있고, 이 노드는 파라미터·토픽·
Twist 변환만 담당한다. 표식을 잡은 ALIGN 이후에는 도킹 구조물 자체를
장애물로 밀어내면 슬롯 진입이 불가능하므로 공통 repulsion을 적용하지 않는다.
"""
import dataclasses

import rclpy
from geometry_msgs.msg import Twist

from kaboat_msgs.msg import MarkArray

from .behavior_base import BehaviorBase
from .docking_fsm import DockingFsm, DockParams, DockTarget
from .obstacle_avoidance_utils import apply_repulsion


class DockingCtrl(BehaviorBase):
    STATE_NAME = 'dock'
    CMD_TOPIC = '/cmd/dock'

    def __init__(self):
        super().__init__()
        self.declare_parameter('target_color', 'red')   # 당일 지정 표식 색
        self.declare_parameter('target_shape', 'unknown')
        self.declare_parameter('dock.mark_freshness', 0.5)
        self.target_color = self.get_parameter('target_color').value
        self.target_shape = self.get_parameter('target_shape').value
        self.mark_freshness = float(
            self.get_parameter('dock.mark_freshness').value)

        defaults = DockParams()
        values = {}
        for field in dataclasses.fields(DockParams):
            name = f'dock.{field.name}'
            self.declare_parameter(name, getattr(defaults, field.name))
            values[field.name] = self.get_parameter(name).value
        self.fsm = DockingFsm(DockParams(**values))
        self.dock_marks = []   # 도킹 전용 검출 (공유 버스와 별도)
        self._marks_received_at = None
        self.create_subscription(
            MarkArray, '/detections/dock_marks', self._on_dock_marks, 10)

    def on_activate(self):
        self.fsm.reset()
        self.dock_marks = []
        self._marks_received_at = None

    def on_deactivate(self):
        self.fsm.reset()
        self.dock_marks = []
        self._marks_received_at = None

    def _on_dock_marks(self, msg: MarkArray):
        self.dock_marks = list(msg.marks)
        self._marks_received_at = self.get_clock().now().nanoseconds * 1e-9

    def _best_target(self, now: float):
        if (self._marks_received_at is None
                or now - self._marks_received_at > self.mark_freshness):
            return None
        targets = [
            mark for mark in self.dock_marks
            if mark.color == self.target_color
            and (self.target_shape == 'unknown'
                 or mark.shape == self.target_shape)
            and mark.distance >= 0.0
        ]
        if not targets:
            return None
        best = max(targets, key=lambda mark: mark.confidence)
        return DockTarget(
            bearing=float(best.bearing),
            distance=float(best.distance))

    def compute_cmd(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        target = self._best_target(now)
        out = self.fsm.step(now, self.distance_to_goal(), target)
        if out.event is not None:
            self.get_logger().info(f'도킹 FSM: {out.event}')
        if out.complete:
            self.report_complete()

        if out.seek_goal:
            cmd = self.seek_goal(slow_radius=3.0)
        else:
            cmd = Twist()
            cmd.linear.x = self.max_linear * out.linear
            cmd.angular.z = self.max_angular * out.angular

        if out.use_repulsion:
            return apply_repulsion(cmd, self.occupancy_grid, self.odom)
        return cmd


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
