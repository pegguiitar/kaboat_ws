"""point_recorder — 요청 토픽을 받으면 배의 지금 위치(/odom x, y, 선수각)를 이름 붙여 JSON 에 저장.

나중에 mission_manager 가 이 파일(kaboat_mission/mission_points.py 형식)을 읽어 미션
시작·목표 지점으로 쓴다.

  젯슨:   ros2 run kaboat_hardware point_recorder          # 단축어 run_points
  노트북: rec gate_entry  (= python3 scripts/rec_point.py gate_entry)  # 지금 위치 기록
          python3 scripts/rec_point.py                     # 이름 생략 → p1, p2 …
          python3 scripts/rec_point.py --list              # 저장된 지점 보기
  (젯슨에서도 같은 스크립트가 된다 — 단축어 rec / list_points)

서비스는 Zenoh 브리지로 안 넘어가서 짧은 String 토픽 두 개로 주고받는다(둘 다 브리지 허용 목록에 있음).
  /mission_points/request (String, 노트북 → 젯슨)
      JSON {"id": "a1b2c3d4", "cmd": "record" | "list", "name": "gate_entry"}
      JSON 이 아닌 글자를 보내면 그 글자를 이름으로 기록한다(ros2 topic pub 으로 직접 보낼 때).
  /mission_points/result (String, 젯슨 → 노트북)
      JSON {"id", "cmd", "success", "message", "name", "x", "y", "yaw_deg"}  (list 면 "points")
  같은 id 가 다시 오면 다시 기록하지 않고 앞서 낸 결과를 다시 보낸다 — 노트북 스크립트가
  결과를 못 받았을 때 같은 id 로 재전송해도 두 번 기록되지 않는다.

- 같은 이름으로 다시 기록하면 덮어쓴다(이전 값은 결과 메시지에). 기록할 때마다 파일에 저장.
- 저장 파일: ~/kaboat_mission_points.json (points_file 파라미터).
- RViz: /mission_points/markers (MarkerArray, latched, frame odom) — 화살표 = 선수각.

⚠ x/y 는 GQ7 /odom 기준이다. run_gq7_outdoor 가 ~/kaboat_geo_origin.yaml 로 원점을 고정한
  상태에서 기록해야 다음에 다시 켜도 같은 자리다. 기록할 때 그 원점을 파일에 같이 남기고,
  원점이 다른 파일에는 섞어 기록하지 않는다(거부 — points_file 을 바꾸거나 파일을 옮긴다).
  원점 파일을 새로 잡은 뒤 센서 런치를 다시 안 켰으면 GQ7 은 아직 예전 원점이라 여기서
  알아챌 수 없다 — 원점을 바꿨으면 run_gq7_outdoor 를 다시 켠 뒤 기록한다.
"""

from collections import OrderedDict
from datetime import datetime
import json
import math
import os
import time

from geometry_msgs.msg import Point, Pose
from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray
import yaml

from kaboat_behaviors.behavior_base import yaw_from_quaternion
from kaboat_hardware.geo_local import LocalFrame
from kaboat_hardware.gq7_origin import load_origin_file
from kaboat_mission.mission_points import (
    DEFAULT_POINTS_FILE, FRAME_ID, MissionPoint, PointStore, auto_name)

REQUEST_TOPIC = '/mission_points/request'
RESULT_TOPIC = '/mission_points/result'
REMEMBERED_IDS = 32          # 재전송 판별용으로 기억하는 최근 요청 수


def parse_request(data: str) -> dict:
    """요청 문자열 → {'id', 'cmd', 'name'}. JSON 이 아니면 글자 전체를 이름으로 본다."""
    text = data.strip()
    try:
        req = json.loads(text)
    except ValueError:
        req = None
    if not isinstance(req, dict):
        return {'id': '', 'cmd': 'record', 'name': text}
    return {'id': str(req.get('id', '')), 'cmd': str(req.get('cmd', 'record')),
            'name': str(req.get('name', '') or '').strip()}


class PointRecorder(Node):
    def __init__(self, **node_kwargs):
        super().__init__('point_recorder', **node_kwargs)
        self.declare_parameter('points_file', DEFAULT_POINTS_FILE)
        self.declare_parameter('origin_file', '~/kaboat_geo_origin.yaml')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('max_odom_age_sec', 0.5)   # 이보다 오래된 /odom 으로는 기록 안 함
        self.points_file = os.path.expanduser(str(self.get_parameter('points_file').value))
        self.origin_file = str(self.get_parameter('origin_file').value)
        self.max_odom_age = float(self.get_parameter('max_odom_age_sec').value)
        if not (math.isfinite(self.max_odom_age) and self.max_odom_age > 0):
            raise ValueError('max_odom_age_sec must be > 0')

        self.odom = None
        self.odom_at = None
        self.answered = OrderedDict()      # 요청 id → 결과 JSON
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
        self.result_pub = self.create_publisher(String, RESULT_TOPIC, 10)
        self.create_subscription(Odometry, str(self.get_parameter('odom_topic').value),
                                 self._on_odom, 10)
        self.create_subscription(String, REQUEST_TOPIC, self._on_request, 10)

        self._publish_markers()
        if self.load_error is None:
            self.get_logger().info(
                f'지점 {len(self.store.points)}개 불러옴 ({self.points_file}) — '
                '노트북·젯슨에서 rec <이름> 으로 지금 위치 기록')

    def _on_odom(self, msg: Odometry):
        self.odom = msg
        self.odom_at = time.monotonic()

    # ── 요청 ─────────────────────────────────────────────
    def _on_request(self, msg: String):
        req = parse_request(msg.data)
        if req['id'] and req['id'] in self.answered:      # 재전송: 다시 기록하지 않음
            self.result_pub.publish(String(data=self.answered[req['id']]))
            return
        if req['cmd'] == 'list':
            result = self.list_points()
        elif req['cmd'] == 'record':
            result = self.record(req['name'])
        else:
            result = {'success': False, 'message': f"모르는 cmd: {req['cmd']!r}"}
        result = {'id': req['id'], 'cmd': req['cmd'], **result}
        text = json.dumps(result, ensure_ascii=False)
        if req['id']:
            self.answered[req['id']] = text
            while len(self.answered) > REMEMBERED_IDS:
                self.answered.popitem(last=False)
        self.result_pub.publish(String(data=text))

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

    def record(self, name: str) -> dict:
        """지금 위치를 name 으로 기록. 결과 dict (success, message, 성공 시 name/x/y/yaw_deg)."""
        if self.load_error:
            return {'success': False, 'message': self.load_error}
        pose = self._current_pose()
        if isinstance(pose, str):
            return {'success': False, 'message': pose}
        x, y, yaw_deg = round(pose[0], 3), round(pose[1], 3), round(pose[2], 2)

        try:
            origin = load_origin_file(self.origin_file)
        except (OSError, KeyError, TypeError, ValueError, yaml.YAMLError) as exc:
            return {'success': False,
                    'message': f'원점 파일을 읽지 못함 ({self.origin_file}): {exc}'}
        lat = lon = None
        if origin is not None:
            lat, lon, _ = LocalFrame(*origin).to_llh(x, y)
            lat, lon = round(lat, 9), round(lon, 9)

        name = name or auto_name(self.store.points)
        point = MissionPoint(x, y, yaw_deg, lat, lon,
                             datetime.now().isoformat(timespec='seconds'))
        try:
            store, previous = self.store.with_point(name, point, origin)
            store.save(self.points_file)
        except (OSError, ValueError) as exc:
            message = f'기록 실패: {exc}'
            self.get_logger().error(message)
            return {'success': False, 'message': message}
        self.store = store

        message = f'{name}: x={x:+.2f} m, y={y:+.2f} m, 선수각 {yaw_deg:+.1f}°'
        if previous is not None:
            message += (f' (덮어씀 — 이전 x={previous.x:+.2f}, y={previous.y:+.2f}, '
                        f'{previous.yaw_deg:+.1f}°)')
        if origin is None:
            message += ' ⚠ 원점 파일 없음 — GQ7 원점이 켤 때마다 바뀌어 다음 실행에서는 다른 자리다'
        self.get_logger().info(message)
        self._publish_markers()
        return {'success': True, 'message': message, 'name': name,
                'x': x, 'y': y, 'yaw_deg': yaw_deg}

    def list_points(self) -> dict:
        if self.load_error:
            return {'success': False, 'message': self.load_error}
        lines = [f'{len(self.store.points)}개 ({self.points_file})']
        lines += [f'  {name}: x={p.x:+.2f} y={p.y:+.2f} 선수각 {p.yaw_deg:+.1f}° '
                  f'[{p.recorded_at}]' for name, p in self.store.points.items()]
        return {'success': True, 'message': '\n'.join(lines),
                'points': {name: p.pose() for name, p in self.store.points.items()}}

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
