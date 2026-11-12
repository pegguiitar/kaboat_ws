"""Gate behavior: LiDAR grid -> DRI -> paired posts -> shared LOS follower.

Each aperture supplies a mandatory centre crossing. The free-form obstacle
avoidance planner is not used because it can route around a gate.
"""

import dataclasses
import math
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

from .behavior_base import BehaviorBase, yaw_from_quaternion
from .los_controller import LOS_TURN_RATIO
from .dri import DriParams, build_dri
from .gate_corridor import (
    GateParams, describe_reasons, gate_crossing, lookahead_chord_safe, make_gate_path,
    pair_observed, path_safe, points_observed, points_safe, scan_hits_world,
    select_gate_pair)


def _xy(point) -> str:
    return f'({point[0]:.2f}, {point[1]:.2f})'

GATE_LOS_THRUST_RATIO = 0.4   # 게이트 통과(LOS) 직진 상한 — config/gate_params.yaml 과 같게


class GateFollower(BehaviorBase):
    STATE_NAME = 'gate'
    CMD_TOPIC = '/cmd/gate'

    def __init__(self):
        super().__init__()
        self.gate_params = self._params_from('gate', GateParams)
        self.dri_params = self._params_from('dri', DriParams)
        self.declare_parameter('gate.thrust_ratio', 0.5)       # waypoint(접근) 구간
        # LOS 경로 추종 구간: 직진 0.4·회전 0.31 상한. 직진은 야외 바람 때문에 공용 LOS
        # SOTA 0.25(약 1619 µs)에서 0.4(약 1675 µs, goto 기본과 같음)로 올림(2026-11-12).
        # 회전 상한·요 게인·lookahead 는 공용 LOS SOTA 값 그대로.
        self.declare_parameter('gate.los_thrust_ratio', GATE_LOS_THRUST_RATIO)
        self.declare_parameter('gate.los_turn_ratio', LOS_TURN_RATIO)
        self.declare_parameter('gate.motion_enabled', False)
        # 선체 라이다 장착 위치(occupancy_grid 의 lidar_offset_x/y 와 같은 값,
        # 실측 선체 중심보다 선수 0.4 m)
        self.declare_parameter('lidar_offset_x', 0.4)
        self.declare_parameter('lidar_offset_y', 0.0)
        self.declare_parameter('gate.scan_timeout_sec', 0.7)
        self.declare_parameter('gate.grid_timeout_sec', 0.7)
        # 배 중심이 게이트선 이 거리 안이면 기둥이 지금 스캔에 보이는지 더 묻지 않는다.
        # 선체 라이다는 선수 0.5 m 에 있고 뒤쪽 120°~240° 를 가려서, 폭 2 m 게이트면
        # 중심이 선을 넘기 직전에 기둥이 가림 구간으로 들어가 통과 판정을 놓쳤다.
        self.declare_parameter('gate.visibility_release_distance', 1.0)
        # 같은 쌍으로 이 시간 넘게 진행을 못 하면 쌍을 버리고 다시 고른다(이미 게이트선을
        # 넘었으면 통과로 인정). 버린 쌍은 reject_hold_sec 동안 후보에서 뺀다.
        self.declare_parameter('gate.replan_timeout_sec', 2.0)
        self.declare_parameter('gate.reject_hold_sec', 10.0)
        self.thrust_ratio = float(self.get_parameter('gate.thrust_ratio').value)
        self.los_thrust_ratio = float(
            self.get_parameter('gate.los_thrust_ratio').value)
        self.los_turn_ratio = float(
            self.get_parameter('gate.los_turn_ratio').value)
        self.motion_enabled = bool(self.get_parameter('gate.motion_enabled').value)
        self.lidar_offset = (float(self.get_parameter('lidar_offset_x').value),
                             float(self.get_parameter('lidar_offset_y').value))
        self.scan_timeout = float(self.get_parameter('gate.scan_timeout_sec').value)
        self.grid_timeout = float(self.get_parameter('gate.grid_timeout_sec').value)
        self.visibility_release = float(
            self.get_parameter('gate.visibility_release_distance').value)
        self.replan_timeout = float(self.get_parameter('gate.replan_timeout_sec').value)
        self.reject_hold = float(self.get_parameter('gate.reject_hold_sec').value)
        if (not all(math.isfinite(v) for v in (
                self.visibility_release, self.replan_timeout, self.reject_hold))
                or self.visibility_release < 0.0 or self.replan_timeout <= 0.0
                or self.reject_hold < 0.0):
            raise ValueError('invalid gate visibility release or replan timing')
        if (not math.isfinite(self.thrust_ratio)
                or not 0.0 <= self.thrust_ratio <= 1.0
                or not math.isfinite(self.los_thrust_ratio)
                or not 0.0 <= self.los_thrust_ratio <= 1.0
                or not math.isfinite(self.los_turn_ratio)
                or not 0.0 <= self.los_turn_ratio <= 1.0
                or not math.isfinite(self.scan_timeout) or self.scan_timeout <= 0
                or not math.isfinite(self.grid_timeout) or self.grid_timeout <= 0):
            raise ValueError('invalid gate thrust ratio or sensor timeout')

        self._scan_at = None
        self._scan_hits = None
        self._grid_at = None
        self._pair = None
        self._path = None
        self._previous_xy = None
        self._crossed = False
        self._passed_midpoints = []
        self._passed_directions = []   # 통과한 게이트 진행 방향 — 다음 쌍 기울기 기준
        self._status_text = None
        self._stall_since = None
        self._rejected = []          # [(midpoint, 다시 후보가 되는 시각)]

        self.path_pub = self.create_publisher(Path, '/gate/path', 1)
        self.pair_pub = self.create_publisher(MarkerArray, '/gate/pairs', 1)
        self.status_pub = self.create_publisher(String, '/gate/status', 10)
        self.preview_pub = self.create_publisher(Twist, '/gate/cmd_preview', 10)
        # 부표 쌍 찾기는 스캔 점을 바로 쓰므로 배가 수평일 때만 오는 /scan/level 을 받는다
        self.create_subscription(LaserScan, '/scan/level', self._on_scan,
                                 qos_profile_sensor_data)
        self.get_logger().info(
            f'gate LiDAR/DRI: required_pairs={self.gate_params.required_pairs}, '
            f'thrust_ratio={self.thrust_ratio:.2f}, '
            f'los_thrust_ratio={self.los_thrust_ratio:.2f}, '
            f'los_turn_ratio={self.los_turn_ratio:.2f}, '
            f'motion_enabled={self.motion_enabled}')

    def _params_from(self, prefix, cls):
        defaults = cls()
        values = {}
        for field in dataclasses.fields(cls):
            name = f'{prefix}.{field.name}'
            self.declare_parameter(name, getattr(defaults, field.name))
            values[field.name] = self.get_parameter(name).value
        return cls(**values)

    def on_activate(self):
        self._pair = None
        self._path = None
        self._previous_xy = None
        self._crossed = False
        self._passed_midpoints = []
        self._passed_directions = []   # 통과한 게이트 진행 방향 — 다음 쌍 기울기 기준
        self._status_text = None
        self._stall_since = None
        self._rejected = []

    def on_deactivate(self):
        self._pair = None
        self._path = None
        self._previous_xy = None
        self._crossed = False
        self._stall_since = None

    def _on_grid(self, msg: OccupancyGrid):
        super()._on_grid(msg)
        self._grid_at = time.monotonic()

    def _on_scan(self, msg: LaserScan):
        if self.waypoint_odom_ready():
            pose = self.odom.pose.pose
            hits = scan_hits_world(
                msg, (pose.position.x, pose.position.y),
                yaw_from_quaternion(pose.orientation), self.lidar_offset)
            if len(hits) == 0:
                return
            self._scan_hits = hits
            self._scan_at = time.monotonic()

    def _status(self, value):
        if value != self._status_text:
            self._status_text = value
            self.status_pub.publish(String(data=value))
            self.get_logger().info(f'gate: {value}')

    def _stop(self, reason, keep_previous=False):
        if not keep_previous:
            self._previous_xy = None  # no crossing is inferred across sensor gaps
        self._status(reason)
        self._publish_debug()
        self.preview_pub.publish(Twist())
        return Twist()

    def _gate_ahead(self, boat):
        """배 중심에서 현재 게이트선까지 진행 방향 거리 [m] (지나쳤으면 음수)."""
        mid = np.asarray(self._pair.midpoint)
        return float((mid - np.asarray(boat)) @ np.asarray(self._pair.direction))

    def _drop_pair(self):
        self._pair = None
        self._path = None
        self._previous_xy = None
        self._crossed = False
        self._stall_since = None
        self._los_follower.reset()

    def _post_seen(self, post) -> bool:
        """기둥 하나가 지금 스캔(scan_match_radius 안)에 보이는지 — 진단 로그용."""
        hits = np.asarray(self._scan_hits, dtype=float)
        if hits.ndim != 2 or len(hits) == 0:
            return False
        return bool(np.any(np.sum((hits - np.asarray(post)) ** 2, axis=1)
                           <= self.gate_params.scan_match_radius ** 2))

    def _mark_passed(self, how='경로 끝 도달'):
        n = len(self._passed_midpoints) + 1
        crossing = getattr(self, '_crossing', None)
        self.get_logger().info(
            f'gate: 게이트 {n}/{self.gate_params.required_pairs} 통과 ({how}) — 중점 '
            f'{_xy(self._pair.midpoint)}, 가로지른 점 '
            f'{_xy(crossing) if crossing is not None else "없음"}')
        self._passed_midpoints.append(self._pair.midpoint)
        self._passed_directions.append(self._pair.direction)
        self._drop_pair()
        passed = len(self._passed_midpoints)
        self._status(f'GATE_{passed}/{self.gate_params.required_pairs}_PASSED')

    def _stall(self, reason, now):
        """같은 쌍으로 진행 못 함 — replan_timeout 넘게 이어지면 쌍을 놓고 다시 고른다.

        예전엔 경로가 막히거나 기둥이 안 보이면 같은 쌍·경로를 계속 다시 검사해
        영원히 정지했다.
        """
        if self._stall_since is None:
            self._stall_since = now
        elif now - self._stall_since >= self.replan_timeout:
            if self._crossed:
                # 게이트선은 실제로 넘었다 — 통과로 인정하고 다음 쌍으로
                self.get_logger().warn(
                    f'gate: {reason} after crossing; counting the gate as passed')
                self._mark_passed('가로지른 뒤 막혀서 인정')
                self._publish_debug()
                self.preview_pub.publish(Twist())
                return Twist()
            self.get_logger().warn(
                f'gate: {reason} for {now - self._stall_since:.1f} s; '
                f'dropping pair at ({self._pair.midpoint[0]:.2f}, '
                f'{self._pair.midpoint[1]:.2f}) and re-selecting '
                f'(기둥 {_xy(self._pair.left)}·{_xy(self._pair.right)}, 폭 {self._pair.width:.2f} m, '
                f'{self.reject_hold:.0f} s 동안 후보에서 뺌)')
            self._rejected.append((self._pair.midpoint, now + self.reject_hold))
            self._drop_pair()
            return self._stop(reason)
        # 센서는 살아 있다(odom 정상) — 정지 중에도 게이트선 통과는 계속 판정한다
        return self._stop(reason, keep_previous=True)

    def _output(self, cmd):
        self.preview_pub.publish(cmd)
        return cmd if self.motion_enabled else Twist()

    def _publish_debug(self):
        path = Path()
        path.header.frame_id = 'odom'
        path.header.stamp = self.get_clock().now().to_msg()
        if self._path is not None:
            for x, y in self._path.samples:
                pose = PoseStamped()
                pose.header = path.header
                pose.pose.position.x = float(x)
                pose.pose.position.y = float(y)
                pose.pose.position.z = 0.2
                pose.pose.orientation.w = 1.0
                path.poses.append(pose)
        self.path_pub.publish(path)

        marks = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        marks.markers.append(clear)
        if self._pair is not None:
            for i, point in enumerate((self._pair.left, self._pair.right,
                                       self._pair.midpoint)):
                marker = Marker()
                marker.header = path.header
                marker.ns, marker.id = 'gate_pair', i
                marker.type, marker.action = Marker.SPHERE, Marker.ADD
                marker.pose.position.x = point[0]
                marker.pose.position.y = point[1]
                marker.pose.position.z = 0.3
                marker.pose.orientation.w = 1.0
                marker.scale.x = marker.scale.y = marker.scale.z = 0.3
                marker.color.a = 1.0
                marker.color.g = 1.0 if i == 2 else 0.3
                marker.color.r = 0.1 if i == 2 else 1.0
                marks.markers.append(marker)
        self.pair_pub.publish(marks)

    def _goal_leg_safe(self, dri, grid, boat_xy, goal_xy):
        dx = goal_xy[0] - boat_xy[0]
        dy = goal_xy[1] - boat_xy[1]
        distance = math.hypot(dx, dy)
        if distance > self.gate_params.max_goal_after_gates:
            return False
        length = min(2.0, distance)
        n = max(2, int(math.ceil(length / self.gate_params.sample_spacing)) + 1)
        points = [(boat_xy[0] + t * dx / max(distance, 1e-9),
                   boat_xy[1] + t * dy / max(distance, 1e-9))
                  for t in (length * k / (n - 1) for k in range(n))]
        return (points_safe(dri, points, self.gate_params)
                and points_observed(grid, points))

    def compute_cmd(self):
        now = time.monotonic()
        if (self.goal is None or self.goal.header.frame_id != 'odom'
                or not self.waypoint_odom_ready()):
            return self._stop('WAIT_ODOM_OR_GOAL')
        if (self.occupancy_grid is None
                or self.occupancy_grid.header.frame_id != 'odom'
                or self._grid_at is None
                or now - self._grid_at > self.grid_timeout):
            return self._stop('GRID_TIMEOUT_OR_FRAME')
        if self._scan_at is None or now - self._scan_at > self.scan_timeout:
            return self._stop('SCAN_TIMEOUT')

        grid = self.occupancy_grid
        if (grid.info.resolution <= 0
                or len(grid.data) != grid.info.width * grid.info.height):
            return self._stop('INVALID_GRID')
        pose = self.odom.pose.pose
        boat = (pose.position.x, pose.position.y)
        goal = (self.goal.pose.position.x, self.goal.pose.position.y)
        yaw = yaw_from_quaternion(pose.orientation)
        dri = build_dri(grid, boat, yaw, self.dri_params)

        if len(self._passed_midpoints) >= self.gate_params.required_pairs:
            if self.distance_to_goal() <= 2.0:
                self.report_complete()
                self._status('COMPLETE')
                self._publish_debug()
                return self._output(Twist())
            if not self._goal_leg_safe(dri, grid, boat, goal):
                return self._stop('GOAL_LEG_UNSAFE_OR_TOO_FAR')
            cmd, _ = self.waypoint(*goal, thrust_ratio=self.thrust_ratio,
                                   slow_radius=3.0)
            self._status('GOAL_APPROACH')
            self._publish_debug()
            return self._output(cmd)

        if self._path is None:
            self._rejected = [(m, until) for m, until in self._rejected if until > now]
            stats = {}
            pair = select_gate_pair(
                grid, boat, goal, self._passed_midpoints, self.gate_params,
                excluded_midpoints=[m for m, _ in self._rejected],
                last_direction=(self._passed_directions[-1]
                                if self._passed_directions else None),
                stats=stats)
            if pair is None:
                self.get_logger().info(
                    f'gate: WAIT_GATE_PAIR — 쌍 없음: {describe_reasons(stats)}; '
                    f'보류 중 쌍 {len(self._rejected)}, 통과 {len(self._passed_midpoints)}, '
                    f'배 {_xy(boat)}', throttle_duration_sec=1.0)
                return self._stop('WAIT_GATE_PAIR')
            if not pair_observed(pair, self._scan_hits, self.gate_params):
                seen = [self._post_seen(post) for post in (pair.left, pair.right)]
                self.get_logger().info(
                    f'gate: WAIT_GATE_PAIR — 쌍 {_xy(pair.midpoint)} 기둥이 지금 스캔에 안 보임 '
                    f'(왼 {_xy(pair.left)} {"보임" if seen[0] else "안 보임"}, '
                    f'오 {_xy(pair.right)} {"보임" if seen[1] else "안 보임"}; 지도에만 남은 것)',
                    throttle_duration_sec=1.0)
                return self._stop('WAIT_GATE_PAIR')
            d = pair.direction
            self.get_logger().info(
                f'gate: 게이트 {len(self._passed_midpoints) + 1}/'
                f'{self.gate_params.required_pairs} 쌍 선택 — 왼 {_xy(pair.left)}, '
                f'오 {_xy(pair.right)}, 중점 {_xy(pair.midpoint)}, 폭 {pair.width:.2f} m, '
                f'기울기 {stats.get("chosen_skew_deg") or 0.0:.0f}°, 진행 방향 '
                f'{round(math.degrees(math.atan2(d[1], d[0]))) + 0:d}°, 배 {_xy(boat)}; '
                f'{describe_reasons(stats)}')
            self._crossing = None
            self._pair = pair
            self._path = make_gate_path(boat, pair, self.gate_params)
            self._previous_xy = boat
            self._crossed = False
            self._stall_since = None
            self._los_follower.reset()

        # 통과 판정을 기둥 보임 확인보다 먼저 한다 — 중심이 선을 넘는 순간 기둥이
        # 라이다 뒤쪽 가림 구간에 들어가도 통과를 놓치지 않게.
        if not self._crossed and self._previous_xy is not None:
            hit = gate_crossing(self._previous_xy, boat, self._pair)
            if hit is not None:
                point, offset = hit
                limit = self._pair.width / 2.0 - self.gate_params.min_clearance
                inside = offset <= limit
                self.get_logger().info(
                    f'gate: 게이트선 가로지름 {_xy(point)} — 중점에서 옆 {offset:.2f} m '
                    f'(허용 {limit:.2f}) → {"기둥 사이 통과" if inside else "기둥 사이 아님"}')
                if inside:
                    self._crossed = True
                    self._crossing = point
        self._previous_xy = boat

        if (not self._crossed
                and self._gate_ahead(boat) > self.visibility_release
                and not pair_observed(self._pair, self._scan_hits,
                                     self.gate_params)):
            return self._stall('GATE_POSTS_NOT_VISIBLE', now)

        progress = self._los_follower.progress_idx or 0
        if not path_safe(dri, self._path, self.gate_params, progress):
            return self._stall('PATH_UNSAFE', now)
        if not points_observed(
                grid, self._path.samples[max(0, progress - 2):]):
            return self._stall('PATH_UNOBSERVED', now)
        cmd, reached = self.los(self._path, self.los_thrust_ratio, self.los_turn_ratio)
        progress = self._los_follower.progress_idx or 0
        if not lookahead_chord_safe(
                dri, boat, self._path, progress,
                self._los_follower.params.lookahead, self.gate_params, grid):
            return self._stall('LOOKAHEAD_UNSAFE', now)

        if reached:
            if not self._crossed:
                # 경로 끝까지 왔는데 기둥 사이를 지나지 않았다. 다시 고르면 이 게이트는
                # 이미 뒤라 다음 게이트로 넘어가 버리므로(게이트 누락) 정지 유지.
                self.get_logger().warn(
                    f'gate: GATE_NOT_CROSSED — 경로 끝인데 쌍 {_xy(self._pair.midpoint)} 기둥 '
                    f'사이를 안 지남, 정지 유지 (배 {_xy(boat)})', throttle_duration_sec=2.0)
                return self._stop('GATE_NOT_CROSSED')
            self._mark_passed()
            self._publish_debug()
            return self._output(Twist())
        self._stall_since = None

        self._status(f'FOLLOW_GATE_{len(self._passed_midpoints) + 1}/'
                     f'{self.gate_params.required_pairs}')
        self._publish_debug()
        return self._output(cmd)


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
