"""point_recorder — 서비스를 부르면 배의 지금 위치(/odom x, y, 선수각)를 이름 붙여 JSON 에 저장.

나중에 mission_manager 가 이 파일(kaboat_mission/mission_points.py 형식)을 읽어 미션
시작·목표 지점으로 쓴다. 젯슨 터미널(ssh)에서:

  ros2 run kaboat_hardware point_recorder              # 단축어 run_points
  ros2 service call /mission_points/record kaboat_msgs/srv/RecordPoint "{name: gate_entry}"
                                                       # 단축어 rec_point gate_entry
  ros2 service call /mission_points/list std_srvs/srv/Trigger '{}'   # 단축어 list_points

- 이름을 비우면 p1, p2, … 로 자동. 같은 이름으로 다시 부르면 덮어쓴다(이전 값은 로그·응답에).
- 저장 파일: ~/kaboat_mission_points.json (points_file 파라미터). 한 번 기록할 때마다 저장한다.
- RViz: /mission_points/markers (MarkerArray, latched, frame odom) — 화살표 = 선수각.

⚠ 서비스는 Zenoh 브리지로 넘어가지 않는다 — 노트북에서는 젯슨에 ssh 로 들어가서 부른다.
⚠ x/y 는 GQ7 /odom 기준이다. run_gq7_outdoor 가 ~/kaboat_geo_origin.yaml 로 원점을 고정한
  상태에서 기록해야 다음에 다시 켜도 같은 자리다. 기록할 때 그 원점을 파일에 같이 남기고,
  원점이 다른 파일에는 섞어 기록하지 않는다(거부 — points_file 을 바꾸거나 파일을 옮긴다).
  원점 파일을 새로 잡은 뒤 센서 런치를 다시 안 켰으면 GQ7 은 아직 예전 원점이라 여기서
  알아챌 수 없다 — 원점을 바꿨으면 run_gq7_outdoor 를 다시 켠 뒤 기록한다.
"""

from datetime import datetime
import math
import os
import time

from geometry_msgs.msg import Point, Pose
from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker, MarkerArray
import yaml

from kaboat_behaviors.behavior_base import yaw_from_quaternion
from kaboat_hardware.geo_local import LocalFrame
from kaboat_hardware.gq7_origin import load_origin_file
from kaboat_mission.mission_points import (
    DEFAULT_POINTS_FILE, FRAME_ID, MissionPoint, PointStore, auto_name)
from kaboat_msgs.srv import RecordPoint


class PointRecorder(Node):
    def __init__(self, **node_kwargs):
        super().__init__('point_recorder', **node_kwargs)
        self.declare_parameter('points_file', DEFAULT_POINTS_FILE)
        self.declare_parameter('origin_file', '~/kaboat_geo_origin.yaml')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('max_odom_age_sec', 0.5)   # 이보다 오래된 /odom 으로는 기록 안 함
        p = lambda name: self.get_parameter(name).value
        self.points_file = os.path.expanduser(str(p('points_file')))
        self.origin_file = str(p('origin_file'))
        self.max_odom_age = float(p('max_odom_age_sec'))
        if not (math.isfinite(self.max_odom_age) and self.max_odom_age > 0):
            raise ValueError('max_odom_age_sec must be > 0')

        self.odom = None
        self.odom_at = None
        self.load_error = None
        try:
            self.store = PointStore.load(self.points_file)
        except (OSError, ValueError) as exc:
            # 깨진 파일을 빈 내용으로 덮어쓰지 않도록 기록을 막는다
            self.store = PointStore()
            self.load_error = f'지점 파일을 읽지 못함 ({self.points_file}): {exc}'
            self.get_logger().error(self.load_error + ' — 고치거나 옮긴 뒤 다시 켠다')

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.markers_pub = self.create_publisher(MarkerArray, '/mission_points/markers', latched)
        self.create_subscription(Odometry, str(p('odom_topic')), self._on_odom, 10)
        self.create_service(RecordPoint, '/mission_points/record', self._on_record)
        self.create_service(Trigger, '/mission_points/list', self._on_list)

        self._publish_markers()
        if self.load_error is None:
            self.get_logger().info(
                f'지점 {len(self.store.points)}개 불러옴 ({self.points_file}) — '
                'rec_point <이름> 으로 지금 위치 기록')

    def _on_odom(self, msg: Odometry):
        self.odom = msg
        self.odom_at = time.monotonic()

    # ── 기록 ─────────────────────────────────────────────
    def _current_pose(self):
        """(x, y, yaw_deg) 또는 기록할 수 없는 이유(str)."""
        if self.odom is None:
            return '/odom 을 아직 못 받음 (run_gq7_outdoor 가 켜져 있나?)'
        age = time.monotonic() - self.odom_at
        if age > self.max_odom_age:
            return f'/odom 이 {age:.1f}초 동안 안 옴 (허용 {self.max_odom_age:.1f}초)'
        if self.odom.header.frame_id != FRAME_ID:
            return (f"/odom frame_id 가 '{self.odom.header.frame_id}' — "
                    f"'{FRAME_ID}' 여야 함 (gq7_outdoor.yaml 의 map_frame_id)")
        pos = self.odom.pose.pose.position
        yaw = math.degrees(yaw_from_quaternion(self.odom.pose.pose.orientation))
        if not all(math.isfinite(v) for v in (pos.x, pos.y, yaw)):
            return '/odom 위치가 유한한 수가 아님'
        return pos.x, pos.y, yaw

    def _on_record(self, request, response):
        response.success = False
        if self.load_error:
            response.message = self.load_error
            return response
        pose = self._current_pose()
        if isinstance(pose, str):
            response.message = pose
            return response
        x, y, yaw_deg = round(pose[0], 3), round(pose[1], 3), round(pose[2], 2)

        try:
            origin = load_origin_file(self.origin_file)
        except (OSError, KeyError, TypeError, ValueError, yaml.YAMLError) as exc:
            response.message = f'원점 파일을 읽지 못함 ({self.origin_file}): {exc}'
            return response
        lat = lon = None
        if origin is not None:
            lat, lon, _ = LocalFrame(*origin).to_llh(x, y)
            lat, lon = round(lat, 9), round(lon, 9)

        name = request.name.strip() or auto_name(self.store.points)
        point = MissionPoint(x, y, yaw_deg, lat, lon,
                             datetime.now().isoformat(timespec='seconds'))
        try:
            store, previous = self.store.with_point(name, point, origin)
            store.save(self.points_file)
        except (OSError, ValueError) as exc:
            response.message = f'기록 실패: {exc}'
            self.get_logger().error(response.message)
            return response
        self.store = store

        response.success = True
        response.name, response.x, response.y, response.yaw_deg = name, x, y, yaw_deg
        response.message = f'{name}: x={x:+.2f} m, y={y:+.2f} m, 선수각 {yaw_deg:+.1f}°'
        if previous is not None:
            response.message += (f' (덮어씀 — 이전 x={previous.x:+.2f}, y={previous.y:+.2f}, '
                                 f'{previous.yaw_deg:+.1f}°)')
        if origin is None:
            response.message += (' ⚠ 원점 파일 없음 — GQ7 원점이 켤 때마다 바뀌어 '
                                 '다음 실행에서는 다른 자리다')
        self.get_logger().info(response.message)
        self._publish_markers()
        return response

    def _on_list(self, _request, response):
        response.success = self.load_error is None
        if self.load_error:
            response.message = self.load_error
            return response
        lines = [f'{len(self.store.points)}개 ({self.points_file})']
        lines += [f'{name}: x={p.x:+.2f} y={p.y:+.2f} 선수각 {p.yaw_deg:+.1f}° [{p.recorded_at}]'
                  for name, p in self.store.points.items()]
        response.message = '\n'.join(lines)
        return response

    # ── RViz ─────────────────────────────────────────────
    def _publish_markers(self):
        markers = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)
        stamp = self.get_clock().now().to_msg()
        for i, (name, p) in enumerate(self.store.points.items()):
            half = math.radians(p.yaw_deg) / 2.0
            for kind in (Marker.ARROW, Marker.TEXT_VIEW_FACING):
                m = Marker()
                m.header.stamp, m.header.frame_id = stamp, FRAME_ID
                m.ns = 'mission_points'
                m.id = 2 * i + (kind == Marker.TEXT_VIEW_FACING)
                m.type, m.action = kind, Marker.ADD
                m.pose = Pose(position=Point(x=p.x, y=p.y, z=0.0))
                m.color.a = 1.0
                if kind == Marker.ARROW:
                    m.pose.orientation.z, m.pose.orientation.w = math.sin(half), math.cos(half)
                    m.scale.x, m.scale.y, m.scale.z = 1.0, 0.15, 0.15
                    m.color.r, m.color.g, m.color.b = 0.1, 0.9, 1.0
                else:
                    m.pose.position.z = 0.6
                    m.pose.orientation.w = 1.0
                    m.scale.z = 0.5
                    m.text = name
                    m.color.r = m.color.g = m.color.b = 1.0
                markers.markers.append(m)
        self.markers_pub.publish(markers)


def main(args=None):
    rclpy.init(args=args)
    node = PointRecorder()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
