"""Sensor-gated single-waypoint trial using the shared controller (tank or outdoor).

The goal is fixed at startup.  Preview mode is the default and never
publishes /cmd_vel.  Motion mode requires one fresh odom-frame /odom publisher
and exclusive ownership of /cmd_vel; no motor driver is launched here.

Tank (default): goal and /odom must lie inside the measured tank bounds.
Outdoor (outdoor:=true): no tank bounds. Optionally (max_goal_distance > 0) the boat
halts with GOAL_TOO_FAR while the goal is farther than that; 0 (default) = no limit.
The goal can also be a point recorded with `rec` (point:=gate_entry).
"""

import math
import time

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Bool, String

from kaboat_behaviors.waypoint_controller import (
    WaypointController, WaypointParams, waypoint)
from kaboat_mission.mission_points import (
    DEFAULT_ORIGIN_FILE, DEFAULT_POINTS_FILE, recorded_point)


def _yaw(q) -> float:
    return math.atan2(2.0 * (q.w*q.z + q.x*q.y),
                      1.0 - 2.0 * (q.y*q.y + q.z*q.z))


def recorded_target(name, points_file=DEFAULT_POINTS_FILE,
                    origin_file=DEFAULT_ORIGIN_FILE, use_heading=True):
    """rec 로 기록한 지점 → (x, y, 선수각 deg | None).

    지점이 없거나, 파일이 깨졌거나, 기록 당시 원점과 지금 원점이 다르면 ValueError.
    """
    point = recorded_point(name, points_file, origin_file)
    return point.x, point.y, (point.yaw_deg if use_heading else None)


class IndoorWaypoint(Node):
    def __init__(self, **kwargs):
        super().__init__('indoor_waypoint', **kwargs)
        defaults = {
            'target_x': 5.0, 'target_y': 2.5, 'target_angle_deg': 0.0,
            'thrust_ratio': 0.2,
            'position_tolerance': 0.9,
            'heading_tolerance_deg': 10.0,
            'motion_enabled': False,
            'tank_size_x': 10.0, 'tank_size_y': 5.0,
            'tank_margin': 0.25,
            'odom_timeout_sec': 0.5,
            # 야외: 수조 범위 대신 목표 거리 상한
            'outdoor': False,
            'max_goal_distance': 0.0,    # > 0 이면 이보다 먼 목표에서 정지. 0 = 제한 없음
            # 비어 있지 않으면 target_x/y/angle 대신 rec 기록 지점을 목표로
            'point': '',
            'align_heading': True,     # false = 위치만 (선수각 맞추지 않음)
            'points_file': DEFAULT_POINTS_FILE,
            'origin_file': DEFAULT_ORIGIN_FILE,
            # 비어 있지 않으면 /mission/state 가 이 값일 때만 몬다 — 그 밖엔 명령을 안 내고 처음 상태로
            # 돌아가 있다가, 이 state 가 되면 지금 자리에서 새로 출발한다 (토너먼트 한 번에: 'approach').
            # 미리 띄워 둔 goto 가 출발 직진 중에 도착(REACHED)해 버려, 직진 뒤엔 0 만 내던 문제 (2026-11-14 시뮬).
            'active_state': '',
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        tuning = WaypointParams()
        for name in vars(tuning):
            if not self.has_parameter(name):
                self.declare_parameter(name, getattr(tuning, name))
        p = lambda name: self.get_parameter(name).value

        self.point_name = str(p('point')).strip()
        align = bool(p('align_heading'))
        if self.point_name:
            tx, ty, angle = recorded_target(
                self.point_name, str(p('points_file')), str(p('origin_file')), align)
        else:
            tx, ty = p('target_x'), p('target_y')
            angle = p('target_angle_deg') if align else None
        self.target = waypoint(tx, ty, angle, p('thrust_ratio'))
        self.motion_enabled = bool(p('motion_enabled'))
        self.outdoor = bool(p('outdoor'))
        self.max_goal_distance = float(p('max_goal_distance'))
        self.size_x = float(p('tank_size_x'))
        self.size_y = float(p('tank_size_y'))
        self.margin = float(p('tank_margin'))
        self.odom_timeout = float(p('odom_timeout_sec'))
        if (not math.isfinite(self.odom_timeout) or self.odom_timeout <= 0):
            raise ValueError('odom_timeout_sec must be positive')
        if self.outdoor:
            if not (math.isfinite(self.max_goal_distance)
                    and self.max_goal_distance >= 0):
                raise ValueError('max_goal_distance must be >= 0 (0 = no limit)')
        elif (not all(math.isfinite(v) for v in
                      (self.size_x, self.size_y, self.margin))
                or not 0 <= self.margin < min(self.size_x, self.size_y) / 2
                or not self.margin <= self.target.x <= self.size_x - self.margin
                or not self.margin <= self.target.y <= self.size_y - self.margin):
            raise ValueError('waypoint must lie inside the measured tank bounds')

        params = WaypointParams(**{
            name: float(p(name)) for name in vars(tuning)})
        self.controller = WaypointController(params)
        self.controller.set_goal(self.target)
        self.odom = None
        self.last_odom_time = None
        self.estop = False
        self.completed = False
        self.last_status = None
        self.active_state = str(p('active_state')).strip()
        self.mission_state = None

        latched = QoSProfile(depth=1,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.goal_pub = self.create_publisher(PoseStamped, '/waypoint/goal',
                                              latched)
        self.status_pub = self.create_publisher(String, '/waypoint/status',
                                                latched)
        self.preview_pub = self.create_publisher(
            Twist, '/waypoint/cmd_preview', 10)
        self.cmd_pub = (self.create_publisher(Twist, '/cmd_vel', 10)
                        if self.motion_enabled else None)
        self._cmd_topic = (getattr(self.cmd_pub, 'topic_name', '/cmd_vel')
                           if self.cmd_pub is not None else '/cmd_vel')
        self.create_subscription(Odometry, '/odom', self._on_odom, 10)
        self.create_subscription(Bool, '/emergency_stop', self._on_estop, 10)
        if self.active_state:
            self.create_subscription(String, '/mission/state', self._on_mission_state, latched)
        self.create_timer(0.05, self._tick)
        self._publish_goal()
        self._status('WAIT_ODOM')
        heading = ('선수각 맞추지 않음' if self.target.angle_deg is None
                   else f'{self.target.angle_deg:.1f}')
        source = f'기록 지점 {self.point_name!r}' if self.point_name else '좌표 입력'
        limit = (f'목표 거리 최대 {self.max_goal_distance:.0f} m'
                 if self.max_goal_distance > 0 else '거리 제한 없음')
        area = f'야외({limit})' if self.outdoor else '수조'
        self.get_logger().info(
            f'waypoint({self.target.x:.2f}, {self.target.y:.2f}, {heading}, '
            f'{self.target.thrust_ratio:.2f}) in odom ({source}); {area}; '
            f'motion_enabled={self.motion_enabled}')

    def _publish_goal(self):
        msg = PoseStamped()
        msg.header.frame_id = 'odom'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = self.target.x
        msg.pose.position.y = self.target.y
        yaw = self.target.yaw or 0.0          # 선수각 없음 → 표시만 +X
        msg.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.orientation.w = math.cos(yaw / 2.0)
        self.goal_pub.publish(msg)

    def _status(self, value):
        if value != self.last_status:
            self.last_status = value
            self.status_pub.publish(String(data=value))
            self.get_logger().info(f'waypoint status: {value}')

    def _on_odom(self, msg):
        self.odom = msg
        self.last_odom_time = time.monotonic()

    def _on_estop(self, msg):
        self.estop = msg.data

    def _on_mission_state(self, msg):
        self.mission_state = msg.data

    def _idle(self):
        """active_state 가 아닐 때 — 명령 없이 처음 상태로 (도착 표시도 지움)."""
        self.completed = False
        self.controller.reset()                 # 도착 판정(_arrived)·머문 시간까지 새로
        self.controller.set_goal(self.target)
        self._status(f'IDLE_UNTIL_{self.active_state.upper()}')

    def _send(self, cmd):
        self.preview_pub.publish(cmd)
        if self.cmd_pub is not None:
            self.cmd_pub.publish(cmd)

    def _halt(self, reason):
        self.controller.reset_progress()
        self._send(Twist())
        self._status(reason)

    def _valid_odom(self):
        if self.odom is None or self.odom.header.frame_id != 'odom':
            return False
        pose = self.odom.pose.pose
        q = pose.orientation
        values = (pose.position.x, pose.position.y, q.x, q.y, q.z, q.w,
                  self.odom.twist.twist.angular.z)
        norm = q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w
        if not (all(math.isfinite(v) for v in values) and abs(norm - 1.0) <= 0.1):
            return False
        return self.outdoor or (0 <= pose.position.x <= self.size_x
                                and 0 <= pose.position.y <= self.size_y)

    def _tick(self):
        if self.active_state and self.mission_state != self.active_state:
            self._idle()
            return
        if self.completed:
            self._send(Twist())
            return
        if self.estop:
            self._halt('EMERGENCY_STOP')
            return
        # 자기 출력 토픽의 발행자 수 — launch 에서 /cmd_vel 을 /cmd/approach 등으로 돌리면(cmd_mux 뒤,
        # outdoor_tournament_run) 그 토픽을 센다. count_publishers 는 remap 을 안 따르므로 이름을 직접.
        if self.motion_enabled and self.count_publishers(self._cmd_topic) != 1:
            self._halt('CMD_VEL_PUBLISHER_CONFLICT')
            return
        if self.count_publishers('/odom') != 1:
            self._halt('ODOM_PUBLISHER_COUNT_NOT_ONE')
            return
        if (self.last_odom_time is None
                or time.monotonic() - self.last_odom_time > self.odom_timeout):
            self._halt('ODOM_TIMEOUT')
            return
        if not self._valid_odom():
            self._halt('INVALID_ODOM')
            return

        pose = self.odom.pose.pose
        if self.outdoor:
            distance = math.hypot(self.target.x - pose.position.x,
                                  self.target.y - pose.position.y)
            if 0 < self.max_goal_distance < distance:
                self._halt('GOAL_TOO_FAR')
                self.get_logger().warn(
                    f'목표까지 {distance:.1f} m > max_goal_distance '
                    f'{self.max_goal_distance:.0f} m — 정지. 원점·좌표를 확인하거나 '
                    'max_goal_distance 를 올리세요', throttle_duration_sec=2.0)
                return
        step = self.controller.step(
            pose.position.x, pose.position.y, _yaw(pose.orientation),
            self.odom.twist.twist.angular.z, time.monotonic())
        cmd = Twist()
        cmd.linear.x, cmd.angular.z = step.linear, step.angular
        if step.reached:
            self.completed = True
            cmd = Twist()
        self._send(cmd)
        self._status(step.phase if self.motion_enabled
                     else f'PREVIEW_{step.phase}')
        cur_yaw_deg = math.degrees(_yaw(pose.orientation))
        heading_err_deg = math.degrees(step.heading_error)
        self.get_logger().info(
            f'[{step.phase}] 현재: ({pose.position.x:.2f}, {pose.position.y:.2f})m '
            f'| 목표거리: {step.distance:.2f}m (허용: {self.controller.params.position_tolerance:.2f}m) '
            f'| 선수각: {cur_yaw_deg:.1f}° (오차: {heading_err_deg:.1f}°) '
            f'| 출력: [전진 {cmd.linear.x*100:.0f}%, 회전 {cmd.angular.z*100:.0f}%]',
            throttle_duration_sec=1.0)

    def stop(self):
        # Ctrl+C 로 컨텍스트가 먼저 닫히면 publish 가 RCLError 로 죽어 traceback 이 났다 (2026-11-14).
        # 그때는 못 내도 모터 워치독(0.3 s)이 중립으로 돌린다.
        if self.cmd_pub is None or not self.context.ok():
            return
        try:
            self.cmd_pub.publish(Twist())
        except Exception:  # noqa: BLE001 — 종료 중 컨텍스트 경합
            pass


def main(args=None):
    rclpy.init(args=args)
    node = IndoorWaypoint()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
