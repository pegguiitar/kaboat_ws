"""station_keeper — 위치유지 behavior. 부표 4개 사각형 안에 들어가 hold_time 유지.

경기: 약 3 m × 3 m 사각형 꼭짓점에 부표 4개(흔들림). 그 사이에 들어가 유지하면 된다.
계산은 station_keeping.py(ROS 무의존)에 있고, 이 노드는 입출력만 맡는다.

  SEARCH  부표 2개 전 — /mission/goal(station_end, 사각형 중심에 기록)로 천천히 접근
  ENTER   처음 보인 한 변(가장 가까운 부표 쌍)을 입구로 고정 → 입구 앞 정렬점 → 중심
          (가상 앵커 제어로 주행 — 해류에 밀려 부표에 붙지 않게)
  HOLD    중심에 가상 앵커(거리 비례 목표 속도 + /odom 속도로 후진 제동, 선수각 PD —
          선수각은 HOLD 진입 때 방향 유지).
          배 중심이 부표 사각형 안(각 변에서 inside_margin 0.6 m 안쪽)에 hold_time(5 s)
          연속이면 /mission/complete 1회 (2026-11-13 경기 조건 — 예전엔 중심 1 m 반경).
          사각형은 매 틱 보이는 부표로 다시 잡으므로 부표가 흔들리거나 밀려도 따라간다.
          다음 미션 전환은 mission_manager 가 한다(완료 + station_end 2 m 안). 전환 전까지 계속 유지.

출구 (2026-11-12): rec 기록 파일에 station_center 가 있고 station_end 와 다르면, 사각형
중심은 station_center 를 쓰고 station_end 는 출구 지점으로 본다. HOLD 5 s 가 끝나면 바로
완료하지 않고 출구 쪽 변 가운데를 지나 사각형 밖 경유점 → station_end 로 나가서 완료를
보고한다(그래야 다음 미션 접근이 부표 사각형을 가로지르지 않는다). station_center 가
없으면 예전처럼 station_end = 중심, 유지 끝에 완료.
부표는 공용 /occupancy_grid 가 아니라 station 전용 지도로 찾는다: /scan/level 을 목표 중심
고정 격자에 쌓되, 큰 덩어리(4칸 이상 = 부표)는 빔이 지나간 칸을 깎는 양을 공용 지도의
free_ratio(기본 0.1)배로 줄여 라이다가 앞만 보는 동안 옆·뒤로 간 부표를 잊지 않는다. 작은
덩어리(노이즈)는 공용 지도와 똑같이 깎는다. 공용 지도·다른 behavior 는 그대로.

입력   /scan/level, /odom, /mission/goal, /mission/state
출력   /cmd/station            주행 명령 (station.motion_enabled=false 면 0)
       /station/cmd_preview     계산된 명령 (motion_enabled 와 무관)
       /station/status          단계 변화 (String)
       /station/markers         부표·중심·유지 반경·입구 (RViz, odom)
       /station/map             station 전용 지도 (OccupancyGrid, 구독자 있을 때만)
파라미터 station.<StationParams 필드>, station.motion_enabled, lidar_offset_x/y
"""
import dataclasses
import math
import time

import rclpy
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Point, Twist
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

from kaboat_mission.mission_points import (
    DEFAULT_ORIGIN_FILE, DEFAULT_POINTS_FILE, recorded_point)

from .behavior_base import BehaviorBase, yaw_from_quaternion
from .station_keeping import StationFsm, StationParams, anchor_command, exit_waypoints

GOAL_RESET_DISTANCE = 1.0   # [m] 목표가 이만큼 바뀌면 전용 지도·상태를 새로 시작
SCAN_STALE_SEC = 2.0        # [s] /scan/level 이 이보다 오래 끊기면 경고 (지도는 유지)


class StationKeeper(BehaviorBase):
    STATE_NAME = 'station'
    CMD_TOPIC = '/cmd/station'

    def __init__(self):
        super().__init__()
        self.params = self._params_from('station', StationParams)
        self.declare_parameter('station.motion_enabled', True)
        # 사각형 중심을 따로 기록한 지점 이름 (없으면 station_end 가 중심)
        self.declare_parameter('station.center_point', 'station_center')
        self.declare_parameter('points_file', DEFAULT_POINTS_FILE)
        self.declare_parameter('origin_file', DEFAULT_ORIGIN_FILE)
        # 선체 라이다 장착 위치 (occupancy_grid·gate_follower 의 lidar_offset_x/y 와 같은 값)
        self.declare_parameter('lidar_offset_x', 0.4)
        self.declare_parameter('lidar_offset_y', 0.0)
        self.motion_enabled = bool(self.get_parameter('station.motion_enabled').value)
        self.lidar_offset = (float(self.get_parameter('lidar_offset_x').value),
                             float(self.get_parameter('lidar_offset_y').value))
        if not all(math.isfinite(v) for v in self.lidar_offset):
            raise ValueError('lidar_offset_x/y must be finite')

        self.center_point = str(self.get_parameter('station.center_point').value).strip()
        self.points_file = str(self.get_parameter('points_file').value)
        self.origin_file = str(self.get_parameter('origin_file').value)
        self._center_xy = None      # station_center (활성화 때 읽음)
        self._exit_targets = None   # 유지 후 출구 경유점 [사각형 밖, station_end]
        self._exit_reported = False
        self.fsm = None
        self._fsm_at = 0.0
        self._last_status = None
        self._scan_at = None
        self._last_decision = None

        self.status_pub = self.create_publisher(String, '/station/status', 10)
        self.preview_pub = self.create_publisher(Twist, '/station/cmd_preview', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/station/markers', 1)
        self.map_pub = self.create_publisher(OccupancyGrid, '/station/map', 1)
        # 부표 위치는 스캔 점을 바로 쌓으므로 배가 수평일 때만 오는 /scan/level 을 받는다
        self.create_subscription(LaserScan, '/scan/level', self._on_scan,
                                 qos_profile_sensor_data)
        self.create_timer(0.5, self._publish_debug)
        p = self.params
        self.get_logger().info(
            f'station: 유지 = 부표 사각형 변에서 {p.inside_margin} m 안쪽 {p.hold_time} s, '
            f'목표 속도 ≤ {p.max_speed} m/s (제동 ≤ {p.max_brake}), 부표 간격 {p.side_length} m, '
            f'전용 지도 감쇠 {p.l_free} (큰 덩어리 {p.big_min_cells}칸↑ × {p.free_ratio}), '
            f'motion_enabled={self.motion_enabled}')

    def _params_from(self, prefix, cls):
        defaults = cls()
        values = {}
        for f in dataclasses.fields(cls):
            name = f'{prefix}.{f.name}'
            self.declare_parameter(name, getattr(defaults, f.name))
            values[f.name] = self.get_parameter(name).value
        return cls(**values)

    # ── 활성화 ──────────────────────────────────────────
    def on_activate(self):
        self.fsm = None
        self._last_status = None
        self._last_decision = None
        self._exit_targets = None
        self._exit_reported = False
        self._center_xy = self._load_center()

    def _load_center(self):
        """rec 기록 파일의 station_center → (x, y). 없거나 원점이 다르면 None (station_end 가 중심)."""
        if not self.center_point:
            return None
        try:
            point = recorded_point(self.center_point, self.points_file, self.origin_file)
        except ValueError as exc:
            self.get_logger().info(
                f'station: {self.center_point} 없음 — station_end 를 중심으로 (출구 없음): {exc}')
            return None
        self.get_logger().info(
            f'station: 중심 = {self.center_point} ({point.x:+.2f}, {point.y:+.2f}), '
            f'station_end 는 유지 후 나갈 출구')
        return (point.x, point.y)

    def on_deactivate(self):
        self.fsm = None
        self._last_decision = None

    def _status(self, text):
        if text != self._last_status:
            self._last_status = text
            self.status_pub.publish(String(data=text))
            self.get_logger().info(f'station: {text}')

    # ── 스캔 → 전용 지도 ────────────────────────────────
    def _on_scan(self, msg: LaserScan):
        if not self.active or self.fsm is None or not self.waypoint_odom_ready():
            return
        pose = self.odom.pose.pose
        yaw = yaw_from_quaternion(pose.orientation)
        c, s = math.cos(yaw), math.sin(yaw)
        ox, oy = self.lidar_offset
        lidar_xy = (pose.position.x + ox * c - oy * s, pose.position.y + ox * s + oy * c)
        self.fsm.add_scan(msg.ranges, msg.angle_min, msg.angle_increment,
                          msg.range_min, msg.range_max, lidar_xy, yaw)
        self._scan_at = time.monotonic()

    # ── 제어 ─────────────────────────────────────────────
    def compute_cmd(self):
        if (self.goal is None or self.goal.header.frame_id != 'odom'
                or not self.waypoint_odom_ready()):
            return Twist()
        goal_xy = (self.goal.pose.position.x, self.goal.pose.position.y)
        # 중심 = station_center (station_end 와 1 m 넘게 다를 때), 출구 = station_end
        exit_xy = None
        center_xy = goal_xy
        if self._center_xy is not None and math.dist(self._center_xy, goal_xy) > GOAL_RESET_DISTANCE:
            center_xy, exit_xy = self._center_xy, goal_xy
        if self.fsm is None or math.dist(center_xy, self.fsm.goal) > GOAL_RESET_DISTANCE:
            self.fsm = StationFsm(center_xy, self.params)
            self._fsm_at = time.monotonic()
            self._exit_targets = None
            self._exit_reported = False
            self._status(f'SEARCH — 목표 ({center_xy[0]:+.2f}, {center_xy[1]:+.2f}) 로 접근, 부표 탐색')

        pose = self.odom.pose.pose
        boat_xy = (pose.position.x, pose.position.y)
        yaw = yaw_from_quaternion(pose.orientation)
        now = self.get_clock().now().nanoseconds * 1e-9
        speed = self.odom.twist.twist.linear.x      # 선체 기준 전진 속도 (GQ7) — 앵커 제동
        d = self.fsm.step(boat_xy, yaw, self.odom.twist.twist.angular.z, now, speed=speed)
        self._last_decision = d

        last_scan = self._fsm_at if self._scan_at is None else max(self._scan_at, self._fsm_at)
        if time.monotonic() - last_scan > SCAN_STALE_SEC:
            self.get_logger().warning(
                '/scan/level 이 끊김 — 전용 지도는 마지막 상태로 유지',
                throttle_duration_sec=5.0)

        if self._exit_targets is not None:          # 유지 끝 — 출구로 나가는 중
            return self._exit_cmd(boat_xy, yaw)

        center = ('없음' if d.center is None
                  else f'({d.center[0]:+.2f}, {d.center[1]:+.2f}) [{d.center_kind}]')
        # 좌표는 매 틱 흔들리므로 상태 변화 판정에는 단계·부표 수·중심 종류만 쓴다
        # HOLD 의 앵커 세부 단계(RETURN/ALIGN/BACK)는 자주 바뀌어 아래 1 s 로그로만 본다
        phase = d.phase if d.phase == 'HOLD' else f'{d.phase}/{d.stage}'
        self._status(f'{phase} — 부표 {len(d.buoys)}개, 중심 {d.center_kind or "없음"}')
        p = self.params
        if d.phase == 'HOLD':        # HOLD 에서는 중심이 항상 있다 (SEARCH 는 없을 수 있음)
            if d.clearance is not None:
                rule = f'사각형 변에서 {p.inside_margin:.2f} m 안쪽'
                where = f'변까지 {d.clearance:+.2f} m ({"안" if d.inside else "밖"})'
            else:
                rule = f'중심 {p.hold_radius:.1f} m 안 (부표 사각형 없음)'
                where = f'중심까지 {math.dist(boat_xy, d.center):.2f} m'
            self.get_logger().info(
                f'station HOLD/{d.stage}: {where}, 유지 {d.hold_elapsed:.1f}/{p.hold_time:.1f} s, '
                f'부표 {len(d.buoys)}개', throttle_duration_sec=1.0)
            if d.completed_now:
                self.get_logger().info(
                    f'위치유지 달성 — {rule} {p.hold_time} s 유지 '
                    f'(부표 {len(d.buoys)}개, 중심 {center})')
                if exit_xy is None:
                    self.report_complete()
                else:
                    self._exit_targets = exit_waypoints(
                        d.buoys, d.center or center_xy, exit_xy, self.params)
                    route = ' → '.join(f'({x:+.2f}, {y:+.2f})' for x, y in self._exit_targets)
                    self.get_logger().info(f'station: 출구로 나감 — {route}')

        if self._exit_targets is not None:
            return self._exit_cmd(boat_xy, yaw)

        if d.kind == 'waypoint':
            cmd, _ = self.waypoint(d.target[0], d.target[1], thrust_ratio=d.thrust_ratio,
                                   slow_radius=d.slow_radius)
        else:
            cmd = Twist()
            cmd.linear.x = float(d.linear)
            cmd.angular.z = float(d.angular)
            # 가상 앵커의 상한(직진·회전)이 곧 전체 스케일 — _tick 이 이 값으로 자른다
            self._command_full_scale = (self.params.max_forward, self.params.max_turn)
        self.preview_pub.publish(cmd)
        return cmd if self.motion_enabled else Twist()

    def _exit_cmd(self, boat_xy, yaw):
        """유지 후 사각형 밖 경유점 → station_end. 출구에 닿으면 완료 보고, 그 자리 유지."""
        p = self.params
        target = self._exit_targets[0]
        if len(self._exit_targets) > 1 and math.dist(boat_xy, target) <= p.exit_reach:
            self._exit_targets.pop(0)
            target = self._exit_targets[0]
        if not self._exit_reported and math.dist(boat_xy, target) <= p.exit_reach \
                and len(self._exit_targets) == 1:
            self._exit_reported = True
            self._status('EXIT — 출구 도착, 완료 보고')
            self.report_complete()
        elif not self._exit_reported:
            self._status(f'EXIT — ({target[0]:+.2f}, {target[1]:+.2f}) 로 '
                         f'({len(self._exit_targets)}개 남음)')
        linear, angular, _ = anchor_command(
            boat_xy, yaw, self.odom.twist.twist.angular.z, target, None, p,
            speed=self.odom.twist.twist.linear.x)
        cmd = Twist()
        cmd.linear.x, cmd.angular.z = float(linear), float(angular)
        self._command_full_scale = (p.max_forward, p.max_turn)
        self.preview_pub.publish(cmd)
        return cmd if self.motion_enabled else Twist()

    # ── RViz ─────────────────────────────────────────────
    def _publish_debug(self):
        if not self.active or self.fsm is None:
            return
        if self.map_pub.get_subscription_count() > 0:
            self.map_pub.publish(self._map_msg())
        if self._last_decision is not None:
            self.marker_pub.publish(self._markers(self._last_decision))

    def _map_msg(self):
        m = self.fsm.map
        grid = OccupancyGrid()
        grid.header.stamp = self.get_clock().now().to_msg()
        grid.header.frame_id = 'odom'
        grid.info.resolution = float(self.params.resolution)
        grid.info.width = m.n
        grid.info.height = m.n
        grid.info.origin.position.x = float(m.origin[0])
        grid.info.origin.position.y = float(m.origin[1])
        grid.info.origin.orientation.w = 1.0
        grid.data = m.probability_percent().ravel().tolist()
        return grid

    def _markers(self, d):
        stamp = self.get_clock().now().to_msg()
        out = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        out.markers.append(clear)

        def marker(ns, mid, mtype, rgba, scale):
            mk = Marker()
            mk.header.frame_id = 'odom'
            mk.header.stamp = stamp
            mk.ns, mk.id, mk.type, mk.action = ns, mid, mtype, Marker.ADD
            mk.pose.orientation.w = 1.0
            mk.scale.x, mk.scale.y, mk.scale.z = scale
            mk.color.r, mk.color.g, mk.color.b, mk.color.a = rgba
            return mk

        for i, b in enumerate(d.buoys):
            mk = marker('buoys', i, Marker.CYLINDER, (1.0, 0.3, 0.1, 0.9), (0.4, 0.4, 0.5))
            mk.pose.position.x, mk.pose.position.y, mk.pose.position.z = b.x, b.y, 0.25
            out.markers.append(mk)
        if d.center is not None:
            mk = marker('center', 0, Marker.SPHERE, (0.1, 0.9, 0.3, 0.9), (0.25, 0.25, 0.25))
            mk.pose.position.x, mk.pose.position.y = d.center
            out.markers.append(mk)
        if d.square is not None:
            # 부표 사각형 (노랑) — 완료 판정 영역은 이 변들에서 inside_margin 안쪽
            outline = marker('square', 0, Marker.LINE_STRIP, (1.0, 0.85, 0.1, 0.8),
                             (0.04, 0.0, 0.0))
            outline.points = [Point(x=x, y=y, z=0.02) for x, y in [*d.square, d.square[0]]]
            out.markers.append(outline)
        elif d.center is not None:
            ring = marker('hold_radius', 0, Marker.LINE_STRIP, (1.0, 0.85, 0.1, 0.8),
                          (0.04, 0.0, 0.0))
            r = self.params.hold_radius
            for k in range(37):
                th = 2.0 * math.pi * k / 36
                ring.points.append(Point(x=d.center[0] + r * math.cos(th),
                                         y=d.center[1] + r * math.sin(th), z=0.02))
            out.markers.append(ring)
        if d.gate is not None:
            line = marker('entry_gate', 0, Marker.LINE_LIST, (0.2, 0.6, 1.0, 0.9),
                          (0.06, 0.0, 0.0))
            mid = d.gate.midpoint
            n = d.gate.normal
            pre = self.params.pre_gate_distance
            line.points = [Point(x=d.gate.a[0], y=d.gate.a[1], z=0.05),
                           Point(x=d.gate.b[0], y=d.gate.b[1], z=0.05),
                           Point(x=mid[0] - n[0] * pre, y=mid[1] - n[1] * pre, z=0.05),
                           Point(x=mid[0] + n[0] * pre, y=mid[1] + n[1] * pre, z=0.05)]
            out.markers.append(line)
        if d.target is not None:
            mk = marker('target', 0, Marker.SPHERE, (0.7, 0.2, 0.9, 0.9), (0.18, 0.18, 0.18))
            mk.pose.position.x, mk.pose.position.y = d.target
            out.markers.append(mk)
        return out


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
