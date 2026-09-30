"""docking_ctrl — 3D marker에서 계산된 boat-local 입구 XY 추종.

YOLO는 /dock/entrance_base (PointStamped, base_link, z=0)를 발행한다.
MarkArray의 픽셀 중심 거리로 진입하지 않는다. 기본 motion_enabled=false:
카메라 TF·입구 오프셋·현장 안전 검증 후 명시적으로 켜야 추력이 나온다.

  APPROACH → ACQUIRE → ALIGN → ENTER → HOLD → REVERSE → COMPLETE

판단과 전이는 ROS 비의존 docking_fsm.py에 있고, 이 노드는 파라미터·토픽·
Twist 변환만 담당한다. 표식을 잡은 ALIGN 이후에는 도킹 구조물 자체를
장애물로 밀어내면 슬롯 진입이 불가능하므로 공통 repulsion을 적용하지 않는다.
"""
import dataclasses

import rclpy
from geometry_msgs.msg import PointStamped, Twist

from .behavior_base import BehaviorBase, YAW_KD
from .dock_target import target_from_entrance
from .docking_fsm import DockingFsm, DockParams
from .obstacle_avoidance_utils import apply_repulsion


class DockingCtrl(BehaviorBase):
    STATE_NAME = 'dock'
    CMD_TOPIC = '/cmd/dock'

    def __init__(self):
        super().__init__()
        self.declare_parameter('dock.motion_enabled', False)
        self.declare_parameter('dock.entrance_freshness_s', 0.35)
        self.declare_parameter('dock.max_stamp_age_s', 0.5)
        self.declare_parameter('dock.max_entrance_range_m', 12.0)
        self.declare_parameter('dock.base_frame', 'base_link')
        self.motion_enabled = bool(self.get_parameter('dock.motion_enabled').value)
        self.entrance_freshness = float(
            self.get_parameter('dock.entrance_freshness_s').value)
        self.max_stamp_age = float(
            self.get_parameter('dock.max_stamp_age_s').value)
        self.max_entrance_range = float(
            self.get_parameter('dock.max_entrance_range_m').value)
        self.base_frame = str(self.get_parameter('dock.base_frame').value)

        defaults = DockParams()
        values = {}
        for field in dataclasses.fields(DockParams):
            name = f'dock.{field.name}'
            self.declare_parameter(name, getattr(defaults, field.name))
            values[field.name] = self.get_parameter(name).value
        self.fsm = DockingFsm(DockParams(**values))
        self.entrance = None
        self._entrance_received_at = None
        self.create_subscription(
            PointStamped, '/dock/entrance_base', self._on_entrance, 10)
        if not self.motion_enabled:
            self.get_logger().warning(
                'dock.motion_enabled=false — 좌표는 구독하지만 도킹 추력은 차단')

    def on_activate(self):
        self.fsm.reset()
        self.entrance = None
        self._entrance_received_at = None

    def on_deactivate(self):
        self.fsm.reset()
        self.entrance = None
        self._entrance_received_at = None

    def _on_entrance(self, msg: PointStamped):
        if msg.header.frame_id != self.base_frame:
            self.get_logger().warning(
                f'입구 좌표 frame 오류: {msg.header.frame_id} != {self.base_frame}',
                throttle_duration_sec=5.0)
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        age = now - stamp
        if stamp <= 0 or age < -0.1 or age > self.max_stamp_age:
            self.get_logger().warning(
                f'입구 좌표 시각 오류/지연: age={age:.2f}s',
                throttle_duration_sec=5.0)
            return
        self.entrance = msg
        self._entrance_received_at = now

    def _fresh_target(self, now: float):
        if (self.entrance is None or self._entrance_received_at is None
                or now - self._entrance_received_at > self.entrance_freshness):
            return None
        point = self.entrance.point
        return target_from_entrance(
            float(point.x), float(point.y), float(point.z),
            max_range=self.max_entrance_range)

    def compute_cmd(self):
        if not self.motion_enabled:
            return Twist()
        now = self.get_clock().now().nanoseconds * 1e-9
        target = self._fresh_target(now)
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
            if self.odom is not None and out.angular != 0.0:
                cmd.angular.z -= YAW_KD * self.odom.twist.twist.angular.z

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
