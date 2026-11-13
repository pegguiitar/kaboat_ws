"""search_circler — 탐색 behavior. 지정 색 부표를 중심으로 규정 방향 원 선회.

규정 5.2(6) (2026-11): 사전에 주어진 색의 부표를 중심으로 원을 그리며 선회한다.
  빨강·초록 → 우현선회(시계), 검정(노랑에서 바뀜) → 좌현선회(반시계).
  검정은 buoy_lidar_detector 가 'unknown'(빨강·초록 아닌 부표 크기 물체)으로 내므로 그중에서 고른다.
목표 색은 대회 당일 공지 → search.target_color (autonomy.launch.py search_target_color:=).
계산은 search_orbit.py(ROS 무의존)에 있고, 이 노드는 입출력과 주행만 맡는다.

  LOOK     활성화 자세를 search.hold_linear 상한으로 지키며 탐색 영역 중심 쪽을 보고 look_sec 둘러봄
           (부표 트래커는 이 behavior 가 활성이 돼야 켜진다 — 그동안 부표가 쌓인다)
  SEEK     LOOK 뒤에도 못 찾음 — 탐색 영역 중심(/mission/search_center)으로, 도착하면 제자리 둘러보기
  ORBIT    부표 고정 → 원 밖이면 규정 방향 접선 진입 경로를 먼저 LOS 로 따라간 뒤, 반경
           orbit_radius 원을 LOS 로 추종. 원 띠 안에서 laps 바퀴 돌면 DONE
           (회피 반발은 켜 두되 목표 부표 둘레 ignore_radius 안 셀은 뺀다)
  DONE     /mission/complete 1회 → search_end 로 이동 (mission_manager 전환 = 완료 + 2 m 안)
           search_end 가 부표 위에 찍혀 있으면 부표에서 1.5 m 비킨 점으로 (exit_point)
  GIVE_UP  영역 중심에서 give_up_sec 동안 못 찾음 → 완료 신호 내고 search_end 로

탐색 영역 중심: /mission/search_center (mission_manager — rec search_center, 없으면 search_entry
  선수 방향 7 m 앞). 못 받으면(search_entry 도 없는 시뮬·mission_params 흐름) 예전처럼
  search_end 를 영역 중심으로 쓴다 (경고). 받으면 search_end(/mission/goal)는 완료 뒤 출구로만
  쓴다 (2026-11-13: 끝점 10 m 안에서 후보를 찾다 22 m 떨어진 탐색 부표를 끝까지 못 골랐다).

점유지도 필수 (gate 와 같은 규칙): /occupancy_grid 가 search.grid_timeout_sec(0.7 s) 넘게 끊기거나
  frame 이 odom 이 아니면 정지 — run_map 을 안 켜면 반발 회피 없이 도는 걸 막는다.
  search.require_grid:=false 로 끌 수 있다(지도 없는 수조·시험).
반발 조향(apply_repulsion): SEEK 이동·접선 진입·선회·출구 이동, LOOK 은 전진할 때만
  (탐색 부표 쪽을 보고 제자리 회전할 때 걸면 앞 부표 때문에 선수각을 못 맞춘다).

입력   /detections/buoys, /odom, /mission/goal, /mission/search_center, /mission/state,
       /occupancy_grid
출력   /cmd/search          주행 명령
       /search/status        단계 변화 (String)
       /search/markers       목표 부표·선회 원·탐색 영역·진입 경로 (RViz, odom)
파라미터 search.<SearchParams 필드>
"""
import dataclasses
import math
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Point, PoseStamped, Twist
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

from .behavior_base import BehaviorBase, WAYPOINT_MAX_ANGULAR, yaw_from_quaternion
from .los_controller import circle_path, sampled_path
from .obstacle_avoidance_utils import apply_repulsion
from .search_orbit import SearchFsm, SearchParams, tangent_entry
from .waypoint_controller import WaypointController, waypoint as make_waypoint

GOAL_RESET_DISTANCE = 1.0   # [m] 출구·영역 중심이 이만큼 바뀌면 탐색을 새로 시작
ENTRY_SPACING = 0.05        # [m] 진입 경로 샘플 간격
ENTRY_DONE_DISTANCE = 1.0   # [m] 진입 경로 끝점에 이만큼 다가가면 원 추종으로
MARKER_RGB = {'red': (1.0, 0.1, 0.1), 'green': (0.1, 0.9, 0.2), 'black': (0.2, 0.2, 0.2)}


def polyline_path(points, spacing=ENTRY_SPACING):
    """꼭짓점 목록 → 거의 등간격으로 샘플한 열린 경로 (모서리는 LOS 가 lookahead 로 깎는다)."""
    pts = [np.asarray(points[0], dtype=np.float64)]
    for a, b in zip(points[:-1], points[1:]):
        a, b = np.asarray(a, dtype=np.float64), np.asarray(b, dtype=np.float64)
        n = max(1, int(math.ceil(np.linalg.norm(b - a) / spacing)))
        for k in range(1, n + 1):
            pts.append(a + (b - a) * (k / n))
    samples = np.array(pts)
    keep = np.concatenate(([True], np.linalg.norm(np.diff(samples, axis=0), axis=1) > 1e-6))
    return sampled_path(samples[keep], spacing=spacing)


class SearchCircler(BehaviorBase):
    STATE_NAME = 'search'
    CMD_TOPIC = '/cmd/search'
    USES_BUOYS = True    # 활성일 때만 /detections/buoys 구독 → 그동안만 부표 검출 노드가 돈다

    def __init__(self):
        super().__init__()
        self.params = self._params_from('search', SearchParams)
        self.fsm = None
        self.search_center = None     # /mission/search_center (x, y)
        # LOOK 위치 유지 전용 — 공용 waypoint() 는 그대로 두고 전진 상한만 hold_linear 로 다른 것
        self._hold_ctrl = WaypointController(dataclasses.replace(
            self._waypoint_controller.params, max_linear=self.params.hold_linear))
        self._grid_at = None          # 마지막 /occupancy_grid 수신 시각 (monotonic)
        self._reset_run()

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(PoseStamped, '/mission/search_center',
                                 self._on_search_center, latched)
        self.status_pub = self.create_publisher(String, '/search/status', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/search/markers', 1)
        self.create_timer(0.5, self._publish_markers)
        p = self.params
        if p.direction is None:
            self.get_logger().error(
                'search.target_color 미설정 — 대회 당일 공지 색(red/green/black)을 '
                'search_target_color:=<색> 으로 넣어야 탐색 미션이 움직인다')
        else:
            self.get_logger().info(
                f'search: 목표 {p.target_color} → {p.direction} 선회, 반경 {p.orbit_radius} m, '
                f'{p.laps} 바퀴, 영역 반경 {p.target_search_radius} m, 시작 둘러보기 '
                f'{p.look_sec:.0f} s, 접선 진입 {"켬" if p.tangent_entry else "끔"}')

    def _params_from(self, prefix, cls):
        defaults = cls()
        values = {}
        for f in dataclasses.fields(cls):
            name = f'{prefix}.{f.name}'
            self.declare_parameter(name, getattr(defaults, f.name))
            values[f.name] = self.get_parameter(name).value
        return cls(**values)

    def _reset_run(self):
        self.fsm = None
        self._circle_path = None
        self._circle_center = None
        self._entry_path = None
        self._entry_done = False
        self._entry_points = None
        self._hold = None              # LOOK 에서 지킬 (x, y, 선수각 deg)
        self._hold_ctrl.reset()
        self._warned_no_center = False
        self._last_status = None

    # ── 활성화 ──────────────────────────────────────────
    def on_activate(self):
        self._reset_run()

    def on_deactivate(self):
        self._reset_run()

    def _on_grid(self, msg):
        super()._on_grid(msg)
        self._grid_at = time.monotonic()

    def _grid_ok(self, now: float) -> bool:
        grid = self.occupancy_grid
        return (grid is not None and grid.header.frame_id == 'odom'
                and self._grid_at is not None
                and now - self._grid_at <= self.params.grid_timeout_sec)

    def _on_search_center(self, msg: PoseStamped):
        p = msg.pose.position
        if msg.header.frame_id == 'odom' and math.isfinite(p.x) and math.isfinite(p.y):
            self.search_center = (p.x, p.y)

    def _status(self, text):
        if text != self._last_status:
            self._last_status = text
            self.status_pub.publish(String(data=text))
            self.get_logger().info(f'search: {text}')

    def _area_xy(self, goal):
        """탐색 영역 중심 — /mission/search_center, 못 받았으면 예전처럼 search_end(goal)."""
        if self.search_center is not None:
            return self.search_center
        if not self._warned_no_center:
            self._warned_no_center = True
            self.get_logger().warning(
                '/mission/search_center 없음 — search_end 를 탐색 영역 중심으로 씀 '
                '(현장에선 rec search_entry 또는 search_center 를 기록)')
        return goal

    # ── 제어 ─────────────────────────────────────────────
    def compute_cmd(self):
        if (self.goal is None or self.goal.header.frame_id != 'odom'
                or not self.waypoint_odom_ready()):
            return Twist()
        if self.params.direction is None:
            self.get_logger().error('search.target_color 미설정 — 정지',
                                    throttle_duration_sec=5.0)
            return Twist()
        now = time.monotonic()
        if self.params.require_grid and not self._grid_ok(now):
            # gate 와 같은 규칙 — 지도 없이(반발 회피 없이) 움직이지 않는다
            self._status('점유지도 없음·끊김 — 정지 (run_map 확인)')
            self.get_logger().error(
                f'search: /occupancy_grid 가 {self.params.grid_timeout_sec:.1f} s 넘게 없음 — 정지. '
                'run_map 을 켜거나, 지도 없는 시험이면 search.require_grid:=false',
                throttle_duration_sec=5.0)
            return Twist()
        goal = (self.goal.pose.position.x, self.goal.pose.position.y)
        area = self._area_xy(goal)
        if (self.fsm is None or math.dist(goal, self.fsm.goal) > GOAL_RESET_DISTANCE
                or math.dist(area, self.fsm.area) > GOAL_RESET_DISTANCE):
            self.fsm = SearchFsm(self.params, goal, area)
            self._circle_path = None
            self._circle_center = None
            self._entry_path = None
            self._entry_done = False
            self._entry_points = None
            self._hold = None
            self._los_follower.reset()

        pos = self.odom.pose.pose.position
        boat = (pos.x, pos.y)
        fsm = self.fsm
        phase = fsm.update(self.buoys, boat, now)

        if fsm.done:
            self._status('완료 — 탐색 목표점으로' if phase == 'DONE'
                         else f'{self.params.give_up_sec:.0f} s 동안 목표 부표 못 찾음 — 포기')
            self.report_complete()
            return self._go_goal(fsm.exit_point(boat))

        if phase in ('LOOK', 'SEEK'):
            # 고정이 없다(처음이거나 풀림) — 다음 고정 때 진입 경로부터 다시
            self._circle_center = None
            self._entry_path, self._entry_points, self._entry_done = None, None, False

        if phase == 'LOOK':
            self._status(f'시작 자세에서 둘러보기 — {self.params.target_color} 부표 기다림')
            return self._hold_pose(boat, area)

        if phase == 'SEEK':
            if math.dist(boat, fsm.area) > self.params.arrive_radius:
                self._status(f'{self.params.target_color} 부표 찾는 중 — 탐색 영역 중심으로')
                return self._go_goal(fsm.area)
            self._status('탐색 영역 중심 도착 — 제자리 둘러보기')
            turn = self.params.scan_turn
            self._command_full_scale = (self.max_linear, turn)
            cmd = Twist()
            cmd.angular.z = -turn if fsm.direction == 'cw' else turn
            return cmd

        # ORBIT
        cx, cy = fsm.center
        if (self._circle_center is None
                or math.dist(fsm.center, self._circle_center) > self.params.recenter_distance):
            self._circle_path = circle_path(cx, cy, self.params.orbit_radius,
                                            direction=fsm.direction, target_laps=None)
            # 처음 고정, 또는 다른 부표로 바꿔 고정(switch) — 진입 경로를 새로
            if (self._circle_center is None
                    or math.dist(fsm.center, self._circle_center) > self.params.relock_radius):
                self._plan_entry(boat, fsm.center)
            self._circle_center = fsm.center
        ignore = (cx, cy, self.params.ignore_radius)
        if self._entry_path is not None and not self._entry_done:
            self._status(f'{self.params.target_color} 부표 ({cx:+.1f}, {cy:+.1f}) '
                         f'{fsm.direction} 선회 원으로 접선 진입')
            cmd, reached = self.los(self._entry_path, self.params.orbit_thrust,
                                    self.params.orbit_turn)
            if reached or math.dist(boat, self._entry_points[-1]) <= ENTRY_DONE_DISTANCE:
                self._entry_done = True
                self._los_follower.reset()
            return apply_repulsion(cmd, self.occupancy_grid, self.odom, ignore=ignore)
        self._status(f'{self.params.target_color} 부표 ({cx:+.1f}, {cy:+.1f}) '
                     f'{fsm.direction} 선회')
        cmd, _ = self.los(self._circle_path, self.params.orbit_thrust, self.params.orbit_turn)
        self.get_logger().info(
            f'search: 선회 {math.degrees(fsm.swept):.0f}° / {360.0 * self.params.laps:.0f}°',
            throttle_duration_sec=2.0)
        return apply_repulsion(cmd, self.occupancy_grid, self.odom, ignore=ignore)

    def _plan_entry(self, boat, center):
        """고정 직후 한 번 — 규정 방향 접선 진입 경로 (원 띠 안이면 없음)."""
        self._entry_path, self._entry_points, self._entry_done = None, None, True
        if not self.params.tangent_entry:
            return
        others = [(b.position.x, b.position.y) for b in self.buoys
                  if math.hypot(b.position.x - center[0], b.position.y - center[1]) > 1.0]
        pts = tangent_entry(boat, center, self.params.orbit_radius, self.fsm.direction, others,
                            lead=self.params.entry_lead, clearance=self.params.entry_clearance,
                            skip_band=self.params.entry_skip_band)
        if pts is None:
            return
        self._entry_points = pts
        self._entry_path = polyline_path(pts)
        self._entry_done = False
        self._los_follower.reset()
        via = ' (경유점 1개 — 다른 부표 비킴)' if len(pts) == 4 else ''
        self.get_logger().info(
            f'search: 접선 진입 — 접점 ({pts[-2][0]:.1f}, {pts[-2][1]:.1f}){via}')

    def _hold_pose(self, boat, area):
        """LOOK — 활성화 자리를 지키며 탐색 영역 중심 쪽을 본다 (바람 보정용 상한 hold_linear)."""
        if self._hold is None:
            angle = math.degrees(math.atan2(area[1] - boat[1], area[0] - boat[0]))
            self._hold = (boat[0], boat[1], angle)
        x, y, angle = self._hold
        self._hold_ctrl.set_goal(make_waypoint(x, y, angle))
        # _tick 이 이 상한으로 자른다 (공용 waypoint() 의 max_linear 0.101 대신 hold_linear)
        self._command_thrust_ratio = 1.0
        self._command_full_scale = (self.params.hold_linear, WAYPOINT_MAX_ANGULAR)
        pose = self.odom.pose.pose
        step = self._hold_ctrl.step(pose.position.x, pose.position.y,
                                    yaw_from_quaternion(pose.orientation),
                                    self.odom.twist.twist.angular.z, time.monotonic())
        cmd = Twist()
        cmd.linear.x, cmd.angular.z = step.linear, step.angular
        if cmd.linear.x > 0.0:
            # 밀려서 제자리로 돌아갈 때만 — 제자리 회전(선수각 맞추기)엔 걸지 않는다
            cmd = apply_repulsion(cmd, self.occupancy_grid, self.odom)
        return cmd

    def _go_goal(self, target):
        pos = self.odom.pose.pose.position
        if math.hypot(target[0] - pos.x, target[1] - pos.y) <= self.params.arrive_radius:
            self._los_follower.reset()
            return Twist()
        cmd, _ = self.waypoint(target[0], target[1], slow_radius=3.0)
        return apply_repulsion(cmd, self.occupancy_grid, self.odom)

    # ── RViz ─────────────────────────────────────────────
    def _ring(self, ns, stamp, cx, cy, rad, rgba, width=0.05):
        m = Marker()
        m.header.frame_id = 'odom'
        m.header.stamp = stamp
        m.ns, m.id = ns, 0
        m.type, m.action = Marker.LINE_STRIP, Marker.ADD
        m.pose.orientation.w = 1.0
        m.scale.x = width
        m.color.r, m.color.g, m.color.b, m.color.a = rgba
        for k in range(73):
            a = 2.0 * math.pi * k / 72
            m.points.append(Point(x=cx + rad * math.cos(a), y=cy + rad * math.sin(a)))
        return m

    def _publish_markers(self):
        if self.marker_pub.get_subscription_count() == 0:
            return
        marks = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        marks.markers.append(clear)
        if self.active and self.fsm is not None:
            stamp = self.get_clock().now().to_msg()
            ax, ay = self.fsm.area
            marks.markers.append(self._ring('search_area', stamp, ax, ay,
                                            self.params.target_search_radius,
                                            (0.6, 0.6, 0.9, 0.6), 0.08))
            if self._hold is not None and self.fsm.phase == 'LOOK':
                hold = Marker()
                hold.header.frame_id = 'odom'
                hold.header.stamp = stamp
                hold.ns, hold.id = 'search_hold', 0
                hold.type, hold.action = Marker.ARROW, Marker.ADD
                hold.pose.position.x, hold.pose.position.y = self._hold[0], self._hold[1]
                a = math.radians(self._hold[2])
                hold.pose.orientation.z, hold.pose.orientation.w = math.sin(a / 2), math.cos(a / 2)
                hold.scale.x, hold.scale.y, hold.scale.z = 1.2, 0.15, 0.15
                hold.color.r, hold.color.g, hold.color.b, hold.color.a = 0.3, 0.8, 1.0, 1.0
                marks.markers.append(hold)
            if self._entry_points is not None and not self._entry_done:
                line = Marker()
                line.header.frame_id = 'odom'
                line.header.stamp = stamp
                line.ns, line.id = 'search_entry', 0
                line.type, line.action = Marker.LINE_STRIP, Marker.ADD
                line.pose.orientation.w = 1.0
                line.scale.x = 0.06
                line.color.r, line.color.g, line.color.b, line.color.a = 0.0, 1.0, 1.0, 0.9
                line.points = [Point(x=float(px), y=float(py)) for px, py in self._entry_points]
                marks.markers.append(line)
        if self.active and self.fsm is not None and self.fsm.center is not None:
            cx, cy = self.fsm.center
            stamp = self.get_clock().now().to_msg()
            r, g, b = MARKER_RGB.get(self.params.target_color, (0.6, 0.6, 0.6))
            sph = Marker()
            sph.header.frame_id = 'odom'
            sph.header.stamp = stamp
            sph.ns, sph.id = 'search_target', 0
            sph.type, sph.action = Marker.SPHERE, Marker.ADD
            sph.pose.position.x, sph.pose.position.y, sph.pose.position.z = cx, cy, 0.3
            sph.pose.orientation.w = 1.0
            sph.scale.x = sph.scale.y = sph.scale.z = 0.6
            sph.color.r, sph.color.g, sph.color.b, sph.color.a = r, g, b, 1.0
            marks.markers.append(sph)
            marks.markers.append(self._ring('search_orbit', stamp, cx, cy,
                                            self.params.orbit_radius, (1.0, 1.0, 0.0, 0.8)))
        self.marker_pub.publish(marks)


def main(args=None):
    rclpy.init(args=args)
    node = SearchCircler()
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
