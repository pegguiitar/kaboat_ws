"""buoy_lidar_detector — 라이다 중심 부표 검출 (라이다 = 있음·위치, 카메라 RGB = 색).

buoy_detector(카메라 프레임 중심, 뎁스 경로)를 대신할 후보. 윤슬·반사에서 뎁스가 크게 튀어
(2026-11-12 현장) 뎁스는 쓰지 않는다. 계산은 순수 모듈에 있다:
  buoy_localize   lidar_points / lidar_clusters — 원시 스캔 → 부표 크기 군집 (선체 프레임)
  buoy_color      군집 → 영상 창 → 색 표 (가림·판정 불가는 표 없음)
  buoy_evidence   부표별 존재 log-odds — 라이다 연결로 올리고, 빔이 자리를 지나가면 깎음

박자는 스캔이다: 스캔마다 ① 군집 ② 최신 영상(image_max_age 안)으로 색 ③ 스캔 시각 odom 으로
전역 ④ 트래커. 영상 시각과 스캔 시각 사이 배 이동은 odom 으로 보정해 창을 잡는다.

입력   /scan/level, /odom, 컬러 영상(color_topic, color_compressed), /camera/camera_info,
       /imu/data (창 투영 롤·피치)
출력   /detections/buoys (BuoyArray, 확정 + 카메라에 찍힌 것만 — unseen 은 publish_unseen),
       ~/markers (전체, 확률·색·마지막 근거), ~/debug_image (창·표)
"""
import collections
import dataclasses
import math

import cv2
import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, CompressedImage, Image, Imu, LaserScan
from visualization_msgs.msg import Marker, MarkerArray

from kaboat_msgs.msg import Buoy, BuoyArray
from kaboat_perception.buoy_color import ColorParams, classify_colors, vote_colors
from kaboat_perception.buoy_evidence import EvidenceParams, EvidenceTracker, LidarView
from kaboat_perception.buoy_localize import BuoyGeometry, lidar_cluster_points, lidar_points

RGB = {'red': (1.0, 0.15, 0.15), 'green': (0.1, 0.9, 0.2), 'unknown': (0.15, 0.15, 0.15),
       'unseen': (0.55, 0.55, 0.85)}
BGR_VOTE = {'red': (40, 40, 255), 'green': (40, 220, 40), 'unknown': (200, 200, 200),
            None: (0, 220, 255)}


def _stamp(h):
    return h.stamp.sec + h.stamp.nanosec * 1e-9


def _yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class BuoyLidarDetector(Node):
    def __init__(self):
        super().__init__('buoy_lidar_detector')
        self.declare_parameter('scan_topic', '/scan/level')
        self.declare_parameter('color_topic', '/camera/color/image_raw')
        self.declare_parameter('color_compressed', False)
        self.declare_parameter('info_topic', '/camera/camera_info')
        self.declare_parameter('output_topic', '/detections/buoys')
        self.declare_parameter('image_max_age', 0.2)   # [s] 스캔과 이만큼 넘게 차이 나면 색 표 안 넣음
        # 카메라에 한 번도 안 찍힌 확정 부표(unseen)도 내보낼지 — 기본은 안 냄 (검정=unknown 오염 방지)
        self.declare_parameter('publish_unseen', False)
        # RViz 마커에 확정 전 후보도 그릴지 — 기본은 확정만 (후보가 부표처럼 보여 헷갈림)
        self.declare_parameter('show_tentative', False)
        # 새 부표는 이 거리 안 라이다 군집에서만 만든다. 시야 제한(spawn_half_fov_deg)은 기본 끔(180) —
        # 배가 부표를 옆으로 지나가는 일이 많아, 카메라 정면에서만 만들면 점유지도보다 10~20 s
        # 늦었다 (2026-11-12 bag). 카메라에 안 찍힌 건 unseen 이라 /detections/buoys 로 안 나간다.
        self.declare_parameter('spawn_half_fov_deg', 180.0)
        self.declare_parameter('spawn_max_range', 15.0)
        # 부표 후보가 될 라이다 군집 — 수면·물보라·선수파 hit(빔 2개, 폭 몇 cm, 배 옆 1~1.5 m)를 거른다.
        # 2026-11-12 bag: 진짜 부표는 빔 8~21개·폭 0.2~0.33 m, 가짜는 빔 2개·폭 1~6 cm.
        self.declare_parameter('cand_min_beams', 3)
        self.declare_parameter('cand_min_width', 0.1)
        self.declare_parameter('cand_min_range', 2.0)
        # 보는 거리 — 이보다 먼 군집은 안 씀 (먼 부표는 빔이 적고 물가·배가 섞임). evidence.max_range 와 같게.
        self.declare_parameter('cand_max_range', 15.0)
        self.geom = BuoyGeometry()
        self.cp = self._params_from('color', ColorParams)
        self.ep = self._params_from('evidence', EvidenceParams)
        self.tracker = EvidenceTracker(self.ep)
        self.image_max_age = float(self.get_parameter('image_max_age').value)
        self.publish_unseen = bool(self.get_parameter('publish_unseen').value)
        self.show_tentative = bool(self.get_parameter('show_tentative').value)
        self.spawn_half_fov = math.radians(float(self.get_parameter('spawn_half_fov_deg').value))
        self.spawn_max_range = float(self.get_parameter('spawn_max_range').value)
        self.cand_min_beams = int(self.get_parameter('cand_min_beams').value)
        self.cand_min_width = float(self.get_parameter('cand_min_width').value)
        self.cand_min_range = float(self.get_parameter('cand_min_range').value)
        self.cand_max_range = float(self.get_parameter('cand_max_range').value)

        self.poses = collections.deque(maxlen=400)   # (t, x, y, yaw)
        self.K = None
        self.image = None       # (t, bgr)
        self.label = None       # (t, label)
        self.roll = self.pitch = 0.0
        self.last_votes = []

        g = lambda n: self.get_parameter(n).value  # noqa: E731
        self.create_subscription(LaserScan, g('scan_topic'), self._on_scan, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odom', self._on_odom, 50)
        self.create_subscription(Imu, '/imu/data', self._on_imu, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, g('info_topic'), self._on_info, qos_profile_sensor_data)
        if bool(g('color_compressed')):
            self.create_subscription(CompressedImage, g('color_topic'), self._on_compressed,
                                     qos_profile_sensor_data)
        else:
            self.create_subscription(Image, g('color_topic'), self._on_image, qos_profile_sensor_data)
        self.pub = self.create_publisher(BuoyArray, g('output_topic'), 10)
        self.marker_pub = self.create_publisher(MarkerArray, '~/markers', 1)
        self.debug_pub = self.create_publisher(Image, '~/debug_image', 1)
        self.get_logger().info(
            f"buoy_lidar_detector: {g('scan_topic')} + {g('color_topic')} → {g('output_topic')} "
            f'(확정 L ≥ {self.ep.l_confirm}, 삭제 L ≤ {self.ep.l_delete})')

    def _params_from(self, prefix, cls):
        defaults = cls()
        values = {}
        for f in dataclasses.fields(cls):
            name = f'{prefix}.{f.name}'
            self.declare_parameter(name, getattr(defaults, f.name))
            values[f.name] = type(getattr(defaults, f.name))(self.get_parameter(name).value)
        return cls(**values)

    # ── 입력 ─────────────────────────────────────────────
    def _on_odom(self, m: Odometry):
        p = m.pose.pose
        self.poses.append((_stamp(m.header), p.position.x, p.position.y, _yaw(p.orientation)))

    def _on_imu(self, m: Imu):
        q = m.orientation
        self.roll = math.atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y))
        self.pitch = math.asin(max(-1.0, min(1.0, 2 * (q.w * q.y - q.z * q.x))))

    def _on_info(self, m: CameraInfo):
        self.K = (m.k[0], m.k[4], m.k[2], m.k[5])

    def _on_compressed(self, m: CompressedImage):
        img = cv2.imdecode(np.frombuffer(m.data, np.uint8), cv2.IMREAD_COLOR)
        if img is not None:
            self.image = (_stamp(m.header), img)

    def _on_image(self, m: Image):
        if m.encoding not in ('bgr8', 'rgb8'):
            self.get_logger().error(f'지원 안 하는 encoding {m.encoding}', throttle_duration_sec=5.0)
            return
        img = np.frombuffer(m.data, np.uint8).reshape(m.height, m.step)[:, :m.width * 3]
        img = img.reshape(m.height, m.width, 3)
        self.image = (_stamp(m.header), img[..., ::-1].copy() if m.encoding == 'rgb8' else img)

    def _pose_at(self, t):
        if not self.poses:
            return None
        ts = [q[0] for q in self.poses]
        i = int(np.searchsorted(ts, t))
        if i <= 0 or i >= len(ts):
            q = self.poses[0 if i <= 0 else -1]
            return None if abs(q[0] - t) > 0.3 else q[1:]
        a, b = self.poses[i - 1], self.poses[i]
        w = (t - a[0]) / max(b[0] - a[0], 1e-9)
        dyaw = math.atan2(math.sin(b[3] - a[3]), math.cos(b[3] - a[3]))
        return (a[1] + w * (b[1] - a[1]), a[2] + w * (b[2] - a[2]), a[3] + w * dyaw)

    # ── 메인 (스캔마다) ───────────────────────────────────
    def _on_scan(self, s: LaserScan):
        t = _stamp(s.header)
        pose = self._pose_at(t)
        if pose is None:
            return
        g = self.geom
        found = [(xy, r, cl) for cl, xy, r in lidar_cluster_points(
                     lidar_points(s.ranges, s.angle_min, s.angle_increment,
                                  s.range_min, s.range_max, g), g, s.angle_increment)
                 if len(cl) >= self.cand_min_beams and 2.0 * r >= self.cand_min_width
                 and self.cand_min_range
                 <= math.hypot(xy[0] - g.lidar_offset_x, xy[1] - g.lidar_offset_y)
                 <= self.cand_max_range]
        clusters = [(xy, r, len(cl)) for xy, r, cl in found]
        px, py, yaw = pose
        c, sn = math.cos(yaw), math.sin(yaw)
        world = [(px + c * x - sn * y, py + sn * x + c * y) for (x, y), _r, _n in clusters]

        votes = [None] * len(clusters)
        self.last_votes = []
        if self.image is not None and self.K is not None and abs(self.image[0] - t) <= self.image_max_age:
            ipose = self._pose_at(self.image[0]) or pose
            if self.label is None or self.label[0] != self.image[0]:
                self.label = (self.image[0], classify_colors(self.image[1], self.cp))
            ic, isn = math.cos(ipose[2]), math.sin(ipose[2])
            cands = []
            for (wx, wy), (_xy, r, _n) in zip(world, clusters):
                dx, dy = wx - ipose[0], wy - ipose[1]
                cands.append(((ic * dx + isn * dy, -isn * dx + ic * dy), r))
            res = vote_colors(self.label[1], cands, self.K, g, self.cp, self.roll, self.pitch)
            votes = list(res)   # ColorVote — 트래커가 픽셀 수를 누적
            self.last_votes = res

        ox, oy = g.lidar_offset_x, g.lidar_offset_y
        view = LidarView(px + c * ox - sn * oy, py + sn * ox + c * oy, yaw,
                         s.angle_min, s.angle_increment, np.asarray(s.ranges, dtype=float),
                         s.range_min, s.range_max)
        cam_x = g.cam_offset_x
        spawn = [abs(math.atan2(y, x - cam_x)) <= self.spawn_half_fov
                 and math.hypot(x - cam_x, y) <= self.spawn_max_range
                 for (x, y), _r, _n in clusters]
        rot = np.array([[c, -sn], [sn, c]])
        world_pts = [cl @ rot.T + np.array([px, py]) for _xy, _r, cl in found]
        conf = self.tracker.update([(x, y, v, ok, wp) for (x, y), v, ok, wp
                                    in zip(world, votes, spawn, world_pts)], t, view)
        self._publish(s.header.stamp, conf)
        self._publish_markers(s.header.stamp)
        if self.debug_pub.get_subscription_count() > 0 and self.image is not None:
            self._publish_debug()

    # ── 출력 ─────────────────────────────────────────────
    def _publish(self, stamp, conf):
        out = BuoyArray()
        out.header.frame_id = 'odom'
        out.header.stamp = stamp
        for tr in conf:
            if tr.color == 'unseen' and not self.publish_unseen:
                continue
            b = Buoy()
            b.id, b.color = tr.id, tr.color
            b.position.x, b.position.y = tr.x, tr.y
            b.confidence = float(tr.probability)
            out.buoys.append(b)
        self.pub.publish(out)

    def _publish_markers(self, stamp):
        if self.marker_pub.get_subscription_count() == 0:
            return
        arr = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        arr.markers.append(clear)
        for tr in self.tracker.tracks:
            if not tr.confirmed and not self.show_tentative:
                continue
            r, gg, b = RGB.get(tr.color, RGB['unknown'])
            m = Marker()
            m.header.frame_id, m.header.stamp = 'odom', stamp
            m.ns, m.id = 'buoy', tr.id
            m.type, m.action = Marker.CYLINDER, Marker.ADD
            m.pose.position.x, m.pose.position.y, m.pose.position.z = tr.x, tr.y, 0.3
            m.pose.orientation.w = 1.0
            # 지름 = 원 맞추기 반지름 × 2 (사전값에서 시작, 점이 원 밖에 찍히면 커짐)
            m.scale.x = m.scale.y = 2.0 * tr.radius
            m.scale.z = 0.6
            m.color.r, m.color.g, m.color.b = r, gg, b
            m.color.a = 1.0 if tr.confirmed else 0.35
            arr.markers.append(m)
            txt = Marker()
            txt.header = m.header
            txt.ns, txt.id = 'label', tr.id
            txt.type, txt.action = Marker.TEXT_VIEW_FACING, Marker.ADD
            txt.pose.position.x, txt.pose.position.y, txt.pose.position.z = tr.x, tr.y, 1.1
            txt.pose.orientation.w = 1.0
            txt.scale.z = 0.35
            txt.color.r = txt.color.g = txt.color.b = txt.color.a = 1.0
            txt.text = f'{tr.probability:.2f}'   # 존재 확률만
            arr.markers.append(txt)
        self.marker_pub.publish(arr)

    def _publish_debug(self):
        t, img = self.image
        img = img.copy()
        for v in self.last_votes:
            if v.rect is None:
                continue
            u0, v0, u1, v1 = v.rect
            cv2.rectangle(img, (u0, v0), (u1, v1), BGR_VOTE[v.color], 2)
            cv2.putText(img, f'{v.color or "-"} r{v.red} g{v.green} /{v.visible}',
                        (u0, max(v0 - 4, 10)), cv2.FONT_HERSHEY_SIMPLEX, 0.4, BGR_VOTE[v.color], 1)
        conf = self.tracker.confirmed()
        cv2.putText(img, f'tracks {len(self.tracker.tracks)} confirmed {len(conf)}: '
                    + ' '.join(f'#{c.id}{c.color[0]}' for c in conf),
                    (5, 15), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)
        msg = Image()
        msg.header.stamp = Time(seconds=t).to_msg()
        msg.header.frame_id = 'camera_color_optical_frame'
        msg.height, msg.width = img.shape[:2]
        msg.encoding, msg.step = 'bgr8', img.shape[1] * 3
        msg.data = img.tobytes()
        self.debug_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = BuoyLidarDetector()
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
