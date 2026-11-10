"""docking_ctrl — 세 도킹 목표점을 순서대로 추종.

YOLO+depth 는 어느 슬롯인지와 표식의 대략 위치(/dock/marker_base)만 준다.
슬롯 축과 입구는 /scan/level(배가 수평일 때만 오는 스캔)의 뒷벽 직선에서 구한다(slot_axis.py; depth 법선은 쓰지
않는다). 표식 주변 라이다 점들에 직선을 맞추고, 표식을 그 직선에 투영한 점에서
법선 방향으로 wall_to_mouth 만큼 떨어진 곳을 입구로 삼아 입구 바깥·입구·접안점
목표점을 계산한다. 직선이 안 잡히거나 /scan/level 이 끊기면(기울어 있을 때 포함) 대체 추정 없이 계획을
만들지 않는다. 기본 motion_enabled=false: 카메라 TF·입구 오프셋·현장 안전 검증 후
명시적으로 켜야 추력이 나온다.

  APPROACH → ACQUIRE → PRE_ENTRY → ALIGN → TO_MOUTH → TO_BERTH
  → BERTH_VERIFY → HOLD → REVERSE → COMPLETE (관측 상실 시 ABORT)

TO_MOUTH·TO_BERTH 에서는 점유 격자(/occupancy_grid)로 슬롯 옆벽 간격을 재서
FSM 에 넘긴다(slot_clearance.py). 입구 도착 시 양쪽 벽·벽 폭을 확인하고(실패 시
대기 후 재시도 waypoint 복귀), 진입 중 벽에 가까워지면 감속, 0.3초 연속 근접하면
ABORT 한다. 뒷벽을 옆벽으로 오인하지 않도록 측정 창은
뒷벽까지 거리에서 여유를 뺀 만큼만 앞을 본다.

ACQUIRE에서 표식을 못 찾으면 dock.retry_* 로 지정한 odom 좌표(RELOCATE)로
이동해 한 번 더 탐색한다. dock.retry_enabled=false 이면 기존대로 ABORT.

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
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

from .behavior_base import BehaviorBase, YAW_KD, yaw_from_quaternion
from .dock_target import plan_from_points
from .slot_axis import locate_slot, scan_points
from .slot_clearance import measure_slot
from .slot_dimensions import SlotDimensions
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
        self.declare_parameter('dock.staging_heading_enabled', False)
        self.declare_parameter('dock.staging_angle_deg', 0.0)
        self.declare_parameter('dock.grid_freshness_s', 1.0)
        self.declare_parameter('dock.slot_occ_threshold', 85)
        self.declare_parameter('dock.scan_max_age_s', 0.5)
        # 슬롯·선체 치수는 kaboat_bringup/config/dock_slot.yaml 한 곳에서 정한다.
        self.declare_parameter('dock.wall_to_mouth_m', 2.5)
        self.declare_parameter('dock.exit_clearance_m', 0.2)
        self.declare_parameter('dock.dimensions_measured', False)
        self.declare_parameter('dock.retry_enabled', False)
        self.declare_parameter('dock.retry_x', 0.0)
        self.declare_parameter('dock.retry_y', 0.0)
        self.declare_parameter('dock.retry_angle_deg', 0.0)
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
        self.staging_heading_enabled = bool(
            self.get_parameter('dock.staging_heading_enabled').value)
        self.staging_angle_deg = float(
            self.get_parameter('dock.staging_angle_deg').value)
        if not math.isfinite(self.staging_angle_deg):
            raise ValueError('dock.staging_angle_deg must be finite')

        self.grid_freshness = float(
            self.get_parameter('dock.grid_freshness_s').value)
        self.slot_occ_threshold = int(
            self.get_parameter('dock.slot_occ_threshold').value)
        self.scan_max_age = float(self.get_parameter('dock.scan_max_age_s').value)
        self.wall_to_mouth = float(self.get_parameter('dock.wall_to_mouth_m').value)
        self.retry_enabled = bool(self.get_parameter('dock.retry_enabled').value)
        self.retry_goal = tuple(float(self.get_parameter(f'dock.retry_{key}').value)
                                for key in ('x', 'y', 'angle_deg'))
        if self.retry_enabled and not all(map(math.isfinite, self.retry_goal)):
            raise ValueError('dock.retry_x/y/angle_deg must be finite')

        defaults = DockParams()
        values = {}
        for field in dataclasses.fields(DockParams):
            name = f'dock.{field.name}'
            self.declare_parameter(name, getattr(defaults, field.name))
            values[field.name] = self.get_parameter(name).value
        if not self.retry_enabled:
            values['acquire_retries'] = 0   # 재시도 지점이 없으면 이동하지 않는다
        # 슬롯·선체 치수: 서로 안 맞으면 여기서 시작을 거부하고, 파생값은 한 곳에서 계산.
        self.slot_dims = SlotDimensions(
            wall_to_mouth=self.wall_to_mouth,
            slot_width=float(values['slot_width']),
            slot_width_tolerance=float(values['slot_width_tolerance']),
            hull_width=float(values['hull_width']),
            hull_length=float(values['hull_length']),
            berth_distance=float(values['berth_distance']),
            pre_entry_offset=float(values['pre_entry_offset']),
            exit_clearance=float(self.get_parameter('dock.exit_clearance_m').value))
        if values['reverse_min_distance'] != defaults.reverse_min_distance:
            self.get_logger().warning(
                'dock.reverse_min_distance 는 무시된다 — dock_slot.yaml 치수에서 '
                f'{self.slot_dims.reverse_min_distance:.2f}m 로 계산')
        values.update(self.slot_dims.fsm_overrides())
        self.wall_fit_params = self.slot_dims.wall_fit_params()
        self.fsm = DockingFsm(DockParams(**values))
        d = self.slot_dims
        self.get_logger().info(
            f'슬롯 치수: 뒷벽→입구 {d.wall_to_mouth:.2f}m, 폭 {d.slot_width:.2f}m'
            f'(±{d.slot_width_tolerance:.2f}), 선체 {d.hull_width:.2f}×'
            f'{d.hull_length:.2f}m, 접안 {d.berth_distance:.2f}m, W1 {d.pre_entry_offset:.2f}m, '
            f'후진 최소 {d.reverse_min_distance:.2f}m, 뒷벽 창 반경 {d.wall_fit_radius:.2f}m')
        if not bool(self.get_parameter('dock.dimensions_measured').value):
            self.get_logger().warning(
                '슬롯·선체 치수가 실측 전 가정치입니다(dock_slot.yaml 의 '
                'dimensions_measured=false) — 실측값으로 채운 뒤 추력을 켜세요')
        self.marker = None
        self._marker_received_at = None
        self._reverse_origin = None
        self._odom_received_at = None
        self.create_subscription(
            PointStamped, '/dock/marker_base', self._on_marker, 10)
        self.create_subscription(
            String, '/dock/detection_status', self._on_detection_status, 10)
        self.scan = None
        self._scan_received_at = None
        self.create_subscription(
            LaserScan, '/scan/level', self._on_scan, qos_profile_sensor_data)
        self.waypoints_pub = self.create_publisher(Path, '/dock/waypoints', 10)
        # 모니터링용: 라이다 뒷벽에서 계산한 입구(base_link, z=0). 제어 입력이 아니다.
        self.entrance_pub = self.create_publisher(
            PointStamped, '/dock/entrance_base', 10)
        if not self.motion_enabled:
            self.get_logger().warning(
                'dock.motion_enabled=false — 좌표는 구독하지만 도킹 추력은 차단')

    def on_activate(self):
        self.fsm.reset()
        self.marker = None
        self._marker_received_at = None
        self._reverse_origin = None

    def on_deactivate(self):
        self.fsm.reset()
        self.marker = None
        self._marker_received_at = None
        self._reverse_origin = None

    def _on_odom(self, msg):
        super()._on_odom(msg)
        self._odom_received_at = self.get_clock().now().nanoseconds * 1e-9

    def _on_scan(self, msg: LaserScan):
        self.scan = msg
        self._scan_received_at = self.get_clock().now().nanoseconds * 1e-9

    def _on_marker(self, msg: PointStamped):
        if msg.header.frame_id != self.base_frame:
            self.get_logger().warning(
                f'marker 좌표 frame 오류: {msg.header.frame_id} != {self.base_frame}',
                throttle_duration_sec=5.0)
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        age = now - stamp
        if stamp <= 0 or age < -0.1 or age > self.max_stamp_age:
            self.get_logger().warning(
                f'marker 좌표 시각 오류/지연: age={age:.2f}s',
                throttle_duration_sec=5.0)
            return
        self.marker = msg
        self._marker_received_at = now

    def _on_detection_status(self, msg: String):
        try:
            valid = json.loads(msg.data).get('valid') is True
        except (ValueError, TypeError):
            valid = False
        if not valid:
            self.marker = None
            self._marker_received_at = None

    def _fresh_plan(self, now: float):
        if self.marker is None or self._marker_received_at is None:
            return None
        if now - self._marker_received_at > self.entrance_freshness:
            return None
        stamp = self.marker.header.stamp
        if now - (stamp.sec + stamp.nanosec * 1e-9) > self.max_stamp_age:
            return None
        if (self.scan is None or self._scan_received_at is None
                or now - self._scan_received_at > self.scan_max_age):
            self.get_logger().warning(
                '/scan/level 이 없거나 지연(배 기울어짐 포함) — 도킹 계획 보류',
                throttle_duration_sec=5.0)
            return None
        scan = self.scan
        points = scan_points(scan.ranges, scan.angle_min, scan.angle_increment,
                             scan.range_min, scan.range_max)
        marker = self.marker.point
        slot = locate_slot(points, (marker.x, marker.y), self.wall_to_mouth,
                           self.wall_fit_params)
        if slot is None:
            self.get_logger().warning(
                '표식 주변에서 라이다 뒷벽 직선을 찾지 못함 — 도킹 계획 보류',
                throttle_duration_sec=2.0)
            return None
        wall = slot.wall
        self.get_logger().info(
            f'라이다 뒷벽: 점 {wall.points}, 길이 {wall.span:.1f}m, '
            f'rms {wall.rms * 100:.1f}cm, 표식 오차 {wall.marker_offset * 100:.0f}cm',
            throttle_duration_sec=2.0)
        return plan_from_points(
            slot.marker_xy[0], slot.marker_xy[1],
            slot.mouth_xy[0], slot.mouth_xy[1],
            pre_entry_offset=self.fsm.p.pre_entry_offset,
            berth_distance=self.fsm.p.berth_distance,
            max_range=self.max_entrance_range)

    def _slot_clearance(self, now: float, plan):
        """TO_MOUTH/TO_BERTH 에서 점유 격자로 선체-옆벽 간격을 잰다."""
        p = self.fsm.p
        grid = self.occupancy_grid
        if (not p.slot_check_enabled or plan is None or grid is None
                or self.odom is None or grid.header.frame_id != 'odom'):
            return None
        stamp = grid.header.stamp
        age = now - (stamp.sec + stamp.nanosec * 1e-9)
        if age > self.grid_freshness or age < -0.5:
            return None
        # 뒷벽(표식이 붙은 벽)을 옆벽으로 오인하지 않도록 앞쪽 창을 제한한다.
        front = max(p.hull_length / 2.0,
                    min(p.slot_look_ahead,
                        plan.marker.distance - p.slot_back_wall_margin))
        pose = self.odom.pose.pose
        return measure_slot(
            grid, (pose.position.x, pose.position.y),
            yaw_from_quaternion(pose.orientation),
            hull_width=p.hull_width, back=p.hull_length / 2.0, front=front,
            occ_threshold=self.slot_occ_threshold)

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
            entrance = PointStamped()
            entrance.header = path.header
            entrance.point.x = plan.mouth.distance * math.cos(plan.mouth.bearing)
            entrance.point.y = plan.mouth.distance * math.sin(plan.mouth.bearing)
            self.entrance_pub.publish(entrance)
        self.waypoints_pub.publish(path)

    def compute_cmd(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        plan = self._fresh_plan(now)
        self._publish_waypoints(plan)
        if not self.motion_enabled:
            return Twist()
        if (self._odom_received_at is None
                or now - self._odom_received_at > self.odom_freshness
                or not self.waypoint_odom_ready()):
            return Twist()
        reverse_distance = 0.0
        if self._reverse_origin is not None and self.odom is not None:
            position = self.odom.pose.pose.position
            reverse_distance = math.hypot(
                position.x - self._reverse_origin[0],
                position.y - self._reverse_origin[1])
        staging_distance = self.distance_to_goal()
        staging_cmd = None
        if self.fsm.state == DockState.APPROACH:
            if self.goal is None or self.goal.header.frame_id != 'odom':
                staging_distance = math.inf
                staging_cmd = Twist()
            else:
                point = self.goal.pose.position
                staging_cmd, staging_reached = self.waypoint(
                    point.x, point.y,
                    self.staging_angle_deg if self.staging_heading_enabled else None,
                    slow_radius=(None if self.staging_heading_enabled else 3.0))
                # When heading is requested, wait until both errors settle.
                if self.staging_heading_enabled and not staging_reached:
                    staging_distance = math.inf
        relocate_reached = False
        if self.fsm.state == DockState.RELOCATE:
            staging_cmd, relocate_reached = self.waypoint(*self.retry_goal)
        clearance = None
        if self.fsm.state in (DockState.TO_MOUTH, DockState.TO_BERTH):
            clearance = self._slot_clearance(now, plan)
        out = self.fsm.step(now, staging_distance, plan,
                            reverse_distance=reverse_distance,
                            relocate_reached=relocate_reached,
                            clearance=clearance)
        if out.event is not None:
            self.get_logger().info(f'도킹 FSM: {out.event}')
        if out.state == DockState.REVERSE and self._reverse_origin is None:
            if self.odom is None:
                return Twist()
            position = self.odom.pose.pose.position
            self._reverse_origin = (position.x, position.y)
        if out.complete:
            self.report_complete()

        if out.use_waypoint:
            cmd = staging_cmd if staging_cmd is not None else Twist()
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
