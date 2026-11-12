"""search_circler — 탐색 behavior. 지정 색 부표를 중심으로 규정 방향 원 선회.

규정 5.2(6) (2026-11): 사전에 주어진 색의 부표를 중심으로 원을 그리며 선회한다.
  빨강·초록 → 우현선회(시계), 검정(노랑에서 바뀜) → 좌현선회(반시계).
  검정은 buoy_detector 가 'unknown'(빨강·초록 아닌 부표 크기 물체)으로 내므로 그중에서 고른다.
목표 색은 대회 당일 공지 → search.target_color (autonomy.launch.py search_target_color:=).
계산은 search_orbit.py(ROS 무의존)에 있고, 이 노드는 입출력과 주행만 맡는다.

  SEEK     목표 색 부표 전 — /mission/goal(search_end)로 접근, 도착하면 제자리 둘러보기
  ORBIT    부표 고정 → 반경 orbit_radius 원을 LOS 로 추종. 원 띠 안에서 laps 바퀴 돌면 DONE
           (회피 반발은 켜 두되 목표 부표 둘레 ignore_radius 안 셀은 뺀다)
  DONE     /mission/complete 1회 → search_end 로 이동 (mission_manager 전환 = 완료 + 2 m 안)
           search_end 가 부표 위에 찍혀 있으면 부표에서 1.5 m 비킨 점으로 (exit_point)
  GIVE_UP  목표점에서 give_up_sec 동안 못 찾음 → 완료 신호 내고 search_end 에서 대기

입력   /detections/buoys, /odom, /mission/goal, /mission/state, /occupancy_grid
출력   /cmd/search          주행 명령
       /search/status        단계 변화 (String)
       /search/markers       목표 부표·선회 원 (RViz, odom)
파라미터 search.<SearchParams 필드>
"""
import dataclasses
import math
import time

import rclpy
from geometry_msgs.msg import Point, Twist
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

from .behavior_base import BehaviorBase
from .los_controller import circle_path
from .obstacle_avoidance_utils import apply_repulsion
from .search_orbit import SearchFsm, SearchParams

GOAL_RESET_DISTANCE = 1.0   # [m] 목표가 이만큼 바뀌면 탐색을 새로 시작
MARKER_RGB = {'red': (1.0, 0.1, 0.1), 'green': (0.1, 0.9, 0.2), 'black': (0.2, 0.2, 0.2)}


class SearchCircler(BehaviorBase):
    STATE_NAME = 'search'
    CMD_TOPIC = '/cmd/search'

    def __init__(self):
        super().__init__()
        self.params = self._params_from('search', SearchParams)
        self.fsm = None
        self._circle_path = None
        self._circle_center = None
        self._last_status = None

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
                f'{p.laps} 바퀴, 후보 반경 {p.target_search_radius} m')

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
        self._circle_path = None
        self._circle_center = None
        self._last_status = None

    def on_deactivate(self):
        self.fsm = None
        self._circle_path = None
        self._circle_center = None

    def _status(self, text):
        if text != self._last_status:
            self._last_status = text
            self.status_pub.publish(String(data=text))
            self.get_logger().info(f'search: {text}')

    # ── 제어 ─────────────────────────────────────────────
    def compute_cmd(self):
        if (self.goal is None or self.goal.header.frame_id != 'odom'
                or not self.waypoint_odom_ready()):
            return Twist()
        if self.params.direction is None:
            self.get_logger().error('search.target_color 미설정 — 정지',
                                    throttle_duration_sec=5.0)
            return Twist()
        goal = (self.goal.pose.position.x, self.goal.pose.position.y)
        if self.fsm is None or math.dist(goal, self.fsm.goal) > GOAL_RESET_DISTANCE:
            self.fsm = SearchFsm(self.params, goal)
            self._circle_path = None
            self._circle_center = None
            self._los_follower.reset()

        pos = self.odom.pose.pose.position
        boat = (pos.x, pos.y)
        fsm = self.fsm
        phase = fsm.update(self.buoys, boat, time.monotonic())

        if fsm.done:
            self._status('완료 — 탐색 목표점으로' if phase == 'DONE'
                         else f'{self.params.give_up_sec:.0f} s 동안 목표 부표 못 찾음 — 포기')
            self.report_complete()
            return self._go_goal(fsm.exit_point(boat))

        if phase == 'SEEK':
            if self.distance_to_goal() > self.params.arrive_radius:
                self._status(f'{self.params.target_color} 부표 찾는 중 — 탐색 목표점으로')
                return self._go_goal(goal)
            self._status('탐색 목표점 도착 — 제자리 둘러보기')
            turn = self.params.scan_turn
            self._command_full_scale = (self.max_linear, turn)
            cmd = Twist()
            cmd.angular.z = -turn if fsm.direction == 'cw' else turn
            return cmd

        # ORBIT
        cx, cy = fsm.center
        self._status(f'{self.params.target_color} 부표 ({cx:+.1f}, {cy:+.1f}) '
                     f'{fsm.direction} 선회')
        if (self._circle_center is None
                or math.dist(fsm.center, self._circle_center) > self.params.recenter_distance):
            self._circle_path = circle_path(cx, cy, self.params.orbit_radius,
                                            direction=fsm.direction, target_laps=None)
            self._circle_center = fsm.center
        cmd, _ = self.los(self._circle_path, self.params.orbit_thrust, self.params.orbit_turn)
        self.get_logger().info(
            f'search: 선회 {math.degrees(fsm.swept):.0f}° / {360.0 * self.params.laps:.0f}°',
            throttle_duration_sec=2.0)
        return apply_repulsion(cmd, self.occupancy_grid, self.odom,
                               ignore=(cx, cy, self.params.ignore_radius))

    def _go_goal(self, target):
        pos = self.odom.pose.pose.position
        if math.hypot(target[0] - pos.x, target[1] - pos.y) <= self.params.arrive_radius:
            self._los_follower.reset()
            return Twist()
        cmd, _ = self.waypoint(target[0], target[1], slow_radius=3.0)
        return apply_repulsion(cmd, self.occupancy_grid, self.odom)

    # ── RViz ─────────────────────────────────────────────
    def _publish_markers(self):
        if self.marker_pub.get_subscription_count() == 0:
            return
        marks = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        marks.markers.append(clear)
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
            ring = Marker()
            ring.header = sph.header
            ring.ns, ring.id = 'search_orbit', 0
            ring.type, ring.action = Marker.LINE_STRIP, Marker.ADD
            ring.pose.orientation.w = 1.0
            ring.scale.x = 0.05
            ring.color.r, ring.color.g, ring.color.b, ring.color.a = 1.0, 1.0, 0.0, 0.8
            rad = self.params.orbit_radius
            for k in range(73):
                a = 2.0 * math.pi * k / 72
                ring.points.append(Point(x=cx + rad * math.cos(a), y=cy + rad * math.sin(a)))
            marks.markers.append(ring)
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
