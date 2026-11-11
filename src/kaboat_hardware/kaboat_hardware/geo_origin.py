"""geo_origin — GQ7 위도·경도로 고정 원점을 정하고, 위도·경도를 원점 기준 x/y [m]로 발행.

원점 정하기 (우선순위)
  1. 파라미터 origin_lat / origin_lon 이 주어지면 그 값
  2. 없으면 origin_file(기본 ~/kaboat_geo_origin.yaml)에 저장된 값
  3. 둘 다 없으면 대기 — 원점에 배를 두고 서비스 호출:
       ros2 service call /geo/set_origin std_srvs/srv/Trigger '{}'
     최근 origin_samples 개(기본 20개, GQ7 2 Hz 면 약 10초) GNSS 위치 평균을 원점으로
     잡고 origin_file 에 저장한다. 다음 실행부터는 그 파일 값을 쓴다.

입력
  /gps/fix (NavSatFix)          GQ7 GNSS 위치 (real_sensors.launch 가 /gnss_1/llh_position 을 remap)
  /geo/target_llh (NavSatFix)   변환할 임의 위도·경도 (latitude/longitude 만 사용)
출력
  /geo/position (PointStamped)       배(GNSS 안테나) 위치, 원점 기준 x/y [m]
  /geo/target_local (PointStamped)   /geo/target_llh 를 변환한 x/y
  /geo/targets (PoseArray, latched)  파라미터 targets 의 x/y (순서 = targets 순서)
  /geo/target_markers (MarkerArray, latched)  RViz 표시용 (구 + 이름)
  /geo/origin (NavSatFix, latched)   현재 원점 위도·경도
  /geo/status (String)               WAIT_ORIGIN / WAIT_FIX / OK

좌표: x = 동쪽, y = 북쪽 (axis_yaw_deg 로 x 축을 돌릴 수 있음, geo_local.py 참고).
⚠ frame_id 는 기본 'geo_local' 이다. 야외 센서 런치(real_sensors outdoor:=true,
  단축어 run_gq7_outdoor)의 gq7_local_odom 이 같은 원점 파일로 /odom 을 만들므로,
  /geo/position 과 /odom 의 x/y 는 같은 좌표계다(안테나·선체 중심 위치 차이만큼 다름).
  원점을 새로 정하면 gq7_local_odom 이 1초 안에 다시 읽는다 — GQ7 재시작 불필요.
  단 axis_yaw_deg = 0 일 때만이다. GQ7 /odom 은 항상 x = 동쪽, y = 북쪽이라 축을 돌리면
  /geo/position·/geo/targets 와 /odom 의 x/y 가 어긋난다 (0 이 아니면 시작 시 경고).
⚠ /gps/fix 는 GNSS 1 안테나 위치다(선체 중심이 아님). 센서 원점과 안테나 거리만큼 차이가 난다.
"""

from collections import deque
from datetime import datetime
import math
import os
import time

import rclpy
from geometry_msgs.msg import Point, PointStamped, Pose, PoseArray
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import String
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker, MarkerArray
import yaml

from kaboat_hardware.geo_local import (
    LocalFrame, average_llh, check_lat_lon, horizontal_std, parse_targets)
from kaboat_hardware.gq7_origin import load_origin_file


def fix_problem(fix, max_std_m: float):
    """NavSatFix 를 원점·위치 계산에 쓸 수 없는 이유, 쓸 수 있으면 None."""
    if fix.status.status < NavSatStatus.STATUS_FIX:
        return 'GNSS no fix'
    try:
        check_lat_lon(fix.latitude, fix.longitude)
    except ValueError as exc:
        return str(exc)
    if max_std_m > 0.0:
        if fix.position_covariance_type == NavSatFix.COVARIANCE_TYPE_UNKNOWN:
            return 'GNSS covariance unknown'
        std = horizontal_std(list(fix.position_covariance))
        if std is None:
            return 'invalid GNSS covariance'
        if std > max_std_m:
            return f'horizontal 1-sigma {std:.2f} m > {max_std_m:.2f} m'
    return None


class GeoOrigin(Node):
    def __init__(self, **node_kwargs):
        super().__init__('geo_origin', **node_kwargs)
        defaults = {
            'gps_topic': '/gps/fix',
            'frame_id': 'geo_local',
            'origin_lat': float('nan'),
            'origin_lon': float('nan'),
            'origin_alt': 0.0,
            'origin_file': '~/kaboat_geo_origin.yaml',
            'axis_yaw_deg': 0.0,          # 로컬 x 축 방향 (동쪽에서 반시계 [deg])
            'origin_samples': 20,         # 원점 평균에 쓸 GNSS 개수
            'origin_window_sec': 30.0,    # 이보다 오래된 GNSS 는 평균에서 뺀다
            # 평균에 쓴 GNSS 가 평균점에서 이보다 멀리 흩어져 있으면 원점 설정 거부 [m]
            # (배가 움직이는 중이거나 GNSS 가 불안정)
            'origin_max_spread_m': 2.0,
            'max_fix_age_sec': 2.0,
            'max_horizontal_std_m': 0.0,  # 0 이면 공분산 검사 안 함
            'targets': [''],              # ['이름,위도,경도', ...]
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        p = lambda name: self.get_parameter(name).value

        self.frame_id = str(p('frame_id'))
        self.origin_file = os.path.expanduser(str(p('origin_file')))
        self.axis_yaw_deg = float(p('axis_yaw_deg'))
        self.origin_samples = int(p('origin_samples'))
        self.origin_window = float(p('origin_window_sec'))
        self.origin_max_spread = float(p('origin_max_spread_m'))
        self.max_fix_age = float(p('max_fix_age_sec'))
        self.max_std = float(p('max_horizontal_std_m'))
        if (self.origin_samples < 1
                or not all(math.isfinite(v) and v > 0 for v in (
                    self.origin_window, self.max_fix_age, self.origin_max_spread))
                or not math.isfinite(self.max_std) or self.max_std < 0
                or not math.isfinite(self.axis_yaw_deg)):
            raise ValueError('invalid geo_origin sampling/age/axis parameters')
        self.targets = parse_targets(p('targets'))
        if self.axis_yaw_deg != 0.0:
            self.get_logger().warn(
                f'axis_yaw_deg = {self.axis_yaw_deg:.1f}° — GQ7 /odom 은 x = 동쪽, y = 북쪽 '
                '그대로라 /geo/position·/geo/targets 의 x/y 를 /odom 목표로 쓰면 안 된다. '
                '/odom 과 맞추려면 axis_yaw_deg: 0.0')

        self.frame: LocalFrame | None = None
        self.origin_source = ''
        self.recent = deque()           # (monotonic, lat, lon, alt)
        self.last_fix_at = None
        self.last_problem = 'no GNSS message yet'
        self.last_local = None

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.position_pub = self.create_publisher(PointStamped, '/geo/position', 10)
        self.target_local_pub = self.create_publisher(PointStamped, '/geo/target_local', 10)
        self.targets_pub = self.create_publisher(PoseArray, '/geo/targets', latched)
        self.markers_pub = self.create_publisher(MarkerArray, '/geo/target_markers', latched)
        self.origin_pub = self.create_publisher(NavSatFix, '/geo/origin', latched)
        self.status_pub = self.create_publisher(String, '/geo/status', 10)
        self.create_subscription(NavSatFix, str(p('gps_topic')), self._on_fix,
                                 qos_profile_sensor_data)
        self.create_subscription(NavSatFix, '/geo/target_llh', self._on_target_llh, 10)
        self.create_service(Trigger, '/geo/set_origin', self._on_set_origin)
        self.create_timer(1.0, self._publish_status)

        lat, lon, alt = float(p('origin_lat')), float(p('origin_lon')), float(p('origin_alt'))
        if math.isfinite(lat) and math.isfinite(lon):
            self._set_frame(lat, lon, alt, 'parameter')
        else:
            loaded = self._load_origin_file()
            if loaded is not None:
                self._set_frame(*loaded, f'file {self.origin_file}')
        if self.frame is None:
            self.get_logger().warn(
                '원점 없음 — 원점 위치에 배를 두고 '
                "`ros2 service call /geo/set_origin std_srvs/srv/Trigger '{}'`")

    # ── 원점 ─────────────────────────────────────────────
    def _load_origin_file(self):
        try:
            return load_origin_file(self.origin_file)
        except (OSError, KeyError, TypeError, ValueError, yaml.YAMLError) as exc:
            self.get_logger().error(f'원점 파일을 읽지 못함 ({self.origin_file}): {exc}')
            return None

    def _save_origin_file(self, lat, lon, alt, samples, spread):
        data = {
            'origin_lat': float(lat), 'origin_lon': float(lon), 'origin_alt': float(alt),
            'samples': int(samples), 'spread_m': round(float(spread), 3),
            'saved_at': datetime.now().isoformat(timespec='seconds'),
        }
        directory = os.path.dirname(self.origin_file)
        if directory:
            os.makedirs(directory, exist_ok=True)
        tmp = self.origin_file + '.tmp'
        with open(tmp, 'w', encoding='utf-8') as f:
            yaml.safe_dump(data, f, allow_unicode=True, sort_keys=False)
            f.flush()
            os.fsync(f.fileno())
        os.replace(tmp, self.origin_file)

    def _set_frame(self, lat, lon, alt, source):
        self.frame = LocalFrame(lat, lon, alt, self.axis_yaw_deg)
        self.origin_source = source
        msg = NavSatFix()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.status.status = NavSatStatus.STATUS_FIX
        msg.latitude, msg.longitude, msg.altitude = lat, lon, alt
        self.origin_pub.publish(msg)
        self.get_logger().info(
            f'원점 = ({lat:.8f}, {lon:.8f}, {alt:.2f} m) [{source}], '
            f'x 축 = 동쪽에서 반시계 {self.axis_yaw_deg:.1f}°')
        if source.startswith('set_origin'):
            self.get_logger().warn(
                '원점 파일이 바뀌었다 — 야외 센서 런치(run_gq7_outdoor)의 gq7_local_odom 이 '
                '1초 안에 /odom 원점을 바꾼다. 그 전에 rec 로 기록한 지점은 이 원점과 안 맞는다')
        # 다음 GNSS 를 기다리지 않고 마지막 위치를 새 원점 기준으로 바로 계산
        self.last_local = (self.frame.to_local(*self.recent[-1][1:3])
                           if self.recent else None)
        self._publish_targets()

    def _on_set_origin(self, _request, response):
        now = time.monotonic()
        self._drop_old(now)
        if len(self.recent) < self.origin_samples:
            response.success = False
            response.message = (
                f'유효한 GNSS {self.origin_samples}개 필요, 최근 {self.origin_window:.0f}초 '
                f'안에 {len(self.recent)}개 ({self.last_problem})')
            return response
        samples = [s[1:] for s in list(self.recent)[-self.origin_samples:]]
        lat, lon, alt = average_llh(samples)
        probe = LocalFrame(lat, lon, alt)
        spread = max(math.hypot(*probe.to_local(s[0], s[1])) for s in samples)
        if spread > self.origin_max_spread:
            response.success = False
            response.message = (
                f'GNSS 가 평균점에서 최대 {spread:.2f} m 흩어짐 (허용 '
                f'{self.origin_max_spread:.2f} m) — 배를 멈추고 다시 시도')
            return response
        try:
            self._save_origin_file(lat, lon, alt, len(samples), spread)
            saved = f', 저장 {self.origin_file}'
        except OSError as exc:
            saved = f', 파일 저장 실패: {exc}'
        self._set_frame(lat, lon, alt, f'set_origin 평균 {len(samples)}개')
        response.success = True
        response.message = (f'원점 = ({lat:.8f}, {lon:.8f}), 평균 {len(samples)}개, '
                            f'최대 흩어짐 {spread:.2f} m{saved}')
        self.get_logger().info(response.message)
        return response

    # ── GNSS ─────────────────────────────────────────────
    def _drop_old(self, now):
        while self.recent and now - self.recent[0][0] > self.origin_window:
            self.recent.popleft()

    def _on_fix(self, msg: NavSatFix):
        problem = fix_problem(msg, self.max_std)
        if problem:
            self.last_problem = problem
            return
        now = time.monotonic()
        alt = msg.altitude if math.isfinite(msg.altitude) else 0.0
        self.recent.append((now, msg.latitude, msg.longitude, alt))
        while len(self.recent) > max(self.origin_samples, 1) * 4:
            self.recent.popleft()
        self._drop_old(now)
        self.last_fix_at = now
        self.last_problem = None
        if self.frame is None:
            return
        x, y = self.frame.to_local(msg.latitude, msg.longitude)
        self.last_local = (x, y)
        out = PointStamped()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self.frame_id
        out.point.x, out.point.y = x, y
        self.position_pub.publish(out)

    def _on_target_llh(self, msg: NavSatFix):
        if self.frame is None:
            self.get_logger().warn('원점이 없어 /geo/target_llh 를 변환하지 못함',
                                   throttle_duration_sec=2.0)
            return
        try:
            x, y = self.frame.to_local(msg.latitude, msg.longitude)
        except ValueError as exc:
            self.get_logger().warn(f'/geo/target_llh 거부: {exc}')
            return
        out = PointStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = self.frame_id
        out.point.x, out.point.y = x, y
        self.target_local_pub.publish(out)

    # ── 발행 ─────────────────────────────────────────────
    def local_targets(self):
        """[(이름, x, y)] — 원점이 없으면 빈 목록."""
        if self.frame is None:
            return []
        return [(t.name, *self.frame.to_local(t.lat, t.lon)) for t in self.targets]

    def _publish_targets(self):
        stamp = self.get_clock().now().to_msg()
        poses = PoseArray()
        poses.header.stamp = stamp
        poses.header.frame_id = self.frame_id
        markers = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)
        for i, (name, x, y) in enumerate(self.local_targets()):
            pose = Pose()
            pose.position = Point(x=x, y=y, z=0.0)
            pose.orientation.w = 1.0
            poses.poses.append(pose)
            for kind in (Marker.SPHERE, Marker.TEXT_VIEW_FACING):
                m = Marker()
                m.header = poses.header
                m.ns = 'geo_targets'
                m.id = 2 * i + (kind == Marker.TEXT_VIEW_FACING)
                m.type, m.action = kind, Marker.ADD
                m.pose = Pose(position=Point(x=x, y=y, z=0.5 if kind != Marker.SPHERE else 0.0))
                m.pose.orientation.w = 1.0
                m.scale.x = m.scale.y = m.scale.z = 0.6 if kind == Marker.SPHERE else 0.5
                m.color.a, m.color.r, m.color.g, m.color.b = 1.0, 1.0, 0.8, 0.1
                if kind == Marker.TEXT_VIEW_FACING:
                    m.text = name
                    m.color.r = m.color.g = m.color.b = 1.0
                markers.markers.append(m)
            self.get_logger().info(f'목표 {name}: x={x:+.2f} m, y={y:+.2f} m')
        self.targets_pub.publish(poses)
        self.markers_pub.publish(markers)

    def status_text(self):
        now = time.monotonic()
        if self.frame is None:
            return 'WAIT_ORIGIN: /geo/set_origin 호출 또는 origin_lat/lon 지정 필요'
        if (self.last_fix_at is None or now - self.last_fix_at > self.max_fix_age
                or self.last_local is None):
            return f'WAIT_FIX: {self.last_problem or "GNSS stale"}'
        x, y = self.last_local
        return (f'OK: origin=({self.frame.origin[0]:.7f}, {self.frame.origin[1]:.7f}) '
                f'[{self.origin_source}] boat x={x:+.2f} y={y:+.2f}')

    def _publish_status(self):
        self.status_pub.publish(String(data=self.status_text()))


def main(args=None):
    rclpy.init(args=args)
    node = GeoOrigin()
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
