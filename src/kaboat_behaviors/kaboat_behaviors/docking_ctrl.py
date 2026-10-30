"""docking_ctrl — 세 도킹 목표점을 순서대로 추종.

YOLO는 /dock/entrance_base 와 /dock/marker_base 를 발행한다.
두 점에서 입구 바깥·입구·접안점의 목표점을 계산한다. 기본 motion_enabled=false:
카메라 TF·입구 오프셋·현장 안전 검증 후 명시적으로 켜야 추력이 나온다.

  APPROACH → ACQUIRE → PRE_ENTRY → ALIGN → TO_MOUTH → TO_BERTH
  → BERTH_VERIFY → HOLD → REVERSE → COMPLETE (관측 상실 시 ABORT)

판단과 전이는 ROS 비의존 docking_fsm.py에 있고, 이 노드는 파라미터·토픽·
Twist 변환만 담당한다. 표식을 잡은 ALIGN 이후에는 도킹 구조물 자체를
장애물로 밀어내면 슬롯 진입이 불가능하므로 공통 repulsion을 적용하지 않는다.
"""
import dataclasses
import json
import math

import rclpy
from geometry_msgs.msg import PointStamped, PoseStamped, Twist
from nav_msgs.msg import Path
from std_msgs.msg import String

from .behavior_base import BehaviorBase, YAW_KD
from .dock_target import plan_from_points
from .docking_fsm import DockingFsm, DockParams, DockState
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
        self.declare_parameter('dock.odom_freshness_s', 0.5)
        self.motion_enabled = bool(self.get_parameter('dock.motion_enabled').value)
        self.entrance_freshness = float(
            self.get_parameter('dock.entrance_freshness_s').value)
        self.max_stamp_age = float(
            self.get_parameter('dock.max_stamp_age_s').value)
        self.max_entrance_range = float(
            self.get_parameter('dock.max_entrance_range_m').value)
        self.base_frame = str(self.get_parameter('dock.base_frame').value)
        self.odom_freshness = float(
            self.get_parameter('dock.odom_freshness_s').value)

        defaults = DockParams()
        values = {}
        for field in dataclasses.fields(DockParams):
            name = f'dock.{field.name}'
            self.declare_parameter(name, getattr(defaults, field.name))
            values[field.name] = self.get_parameter(name).value
        self.fsm = DockingFsm(DockParams(**values))
        self.entrance = None
        self._entrance_received_at = None
        self.marker = None
        self._marker_received_at = None
        self._reverse_origin = None
        self._odom_received_at = None
        self.create_subscription(
            PointStamped, '/dock/entrance_base', self._on_entrance, 10)
        self.create_subscription(
            PointStamped, '/dock/marker_base', self._on_marker, 10)
        self.create_subscription(
            String, '/dock/detection_status', self._on_detection_status, 10)
        self.waypoints_pub = self.create_publisher(Path, '/dock/waypoints', 10)
        if not self.motion_enabled:
            self.get_logger().warning(
                'dock.motion_enabled=false — 좌표는 구독하지만 도킹 추력은 차단')

    def on_activate(self):
        self.fsm.reset()
        self.entrance = None
        self._entrance_received_at = None
        self.marker = None
        self._marker_received_at = None
        self._reverse_origin = None

    def on_deactivate(self):
        self.fsm.reset()
        self.entrance = None
        self._entrance_received_at = None
        self.marker = None
        self._marker_received_at = None
        self._reverse_origin = None

    def _on_entrance(self, msg: PointStamped):
        self._on_point('entrance', msg)

    def _on_odom(self, msg):
        super()._on_odom(msg)
        self._odom_received_at = self.get_clock().now().nanoseconds * 1e-9

    def _on_marker(self, msg: PointStamped):
        self._on_point('marker', msg)

    def _on_detection_status(self, msg: String):
        try:
            valid = json.loads(msg.data).get('valid') is True
        except (ValueError, TypeError):
            valid = False
        if not valid:
            self.entrance = self.marker = None
            self._entrance_received_at = self._marker_received_at = None

    def _on_point(self, kind: str, msg: PointStamped):
        if msg.header.frame_id != self.base_frame:
            self.get_logger().warning(
                f'{kind} 좌표 frame 오류: {msg.header.frame_id} != {self.base_frame}',
                throttle_duration_sec=5.0)
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        age = now - stamp
        if stamp <= 0 or age < -0.1 or age > self.max_stamp_age:
            self.get_logger().warning(
                f'{kind} 좌표 시각 오류/지연: age={age:.2f}s',
                throttle_duration_sec=5.0)
            return
        setattr(self, kind, msg)
        setattr(self, f'_{kind}_received_at', now)

    def _fresh_plan(self, now: float):
        if (self.entrance is None or self.marker is None
                or self._entrance_received_at is None
                or self._marker_received_at is None):
            return None
        if (now - self._entrance_received_at > self.entrance_freshness
                or now - self._marker_received_at > self.entrance_freshness
                or self.entrance.header.stamp != self.marker.header.stamp):
            return None
        stamp = self.marker.header.stamp
        age = now - (stamp.sec + stamp.nanosec * 1e-9)
        if age > self.max_stamp_age or abs(self.entrance.point.z) > 0.05:
            return None
        marker, mouth = self.marker.point, self.entrance.point
        return plan_from_points(
            marker.x, marker.y, mouth.x, mouth.y,
            pre_entry_offset=self.fsm.p.pre_entry_offset,
            berth_distance=self.fsm.p.berth_distance,
            max_range=self.max_entrance_range)

    def _publish_waypoints(self, plan):
        path = Path()
        path.header.frame_id = self.base_frame
        path.header.stamp = (self.marker.header.stamp if plan is not None
                             else self.get_clock().now().to_msg())
        if plan is not None:
            for target in (plan.pre_entry, plan.mouth, plan.berth):
                pose = PoseStamped()
                pose.header = path.header
                pose.pose.position.x = target.distance * math.cos(target.bearing)
                pose.pose.position.y = target.distance * math.sin(target.bearing)
                pose.pose.orientation.w = 1.0
                path.poses.append(pose)
        self.waypoints_pub.publish(path)

    def compute_cmd(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        plan = self._fresh_plan(now)
        self._publish_waypoints(plan)
        if not self.motion_enabled:
            return Twist()
        if (self._odom_received_at is None
                or now - self._odom_received_at > self.odom_freshness):
            return Twist()
        reverse_distance = 0.0
        if self._reverse_origin is not None and self.odom is not None:
            position = self.odom.pose.pose.position
            reverse_distance = math.hypot(
                position.x - self._reverse_origin[0],
                position.y - self._reverse_origin[1])
        out = self.fsm.step(now, self.distance_to_goal(), plan,
                            reverse_distance=reverse_distance)
        if out.event is not None:
            self.get_logger().info(f'도킹 FSM: {out.event}')
        if out.state == DockState.REVERSE and self._reverse_origin is None:
            if self.odom is None:
                return Twist()
            position = self.odom.pose.pose.position
            self._reverse_origin = (position.x, position.y)
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
