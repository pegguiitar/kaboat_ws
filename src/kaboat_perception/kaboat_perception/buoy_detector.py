"""buoy_detector — 부표 검출 + 색 판정(빨강/초록/검정) → 전역 부표맵 (탐색·게이트용, 학습 불필요)

입력   /camera/color/image_raw (sensor_msgs/Image)   — RGB 원본, 색 판정
       /camera/depth/image_raw (sensor_msgs/Image)    — 정렬 뎁스 (6 m 이내만 사용)
       /camera/camera_info (sensor_msgs/CameraInfo)    — K (픽셀 ↔ 시선)
       /scan/level (sensor_msgs/LaserScan)            — 위치 (배가 수평일 때만 오는 스캔)
       /imu/data (sensor_msgs/Imu)                    — 롤·피치 (카메라 시선 수평 보정)
       /odom (nav_msgs/Odometry)                      — ④ 전역 투영 · ⑤ 추적 시간축
출력   /detections/buoys (kaboat_msgs/BuoyArray, frame_id="odom")

**출력은 점유지도(cell)와 다른 좌표 랜드마크다** — 색(red|green|black)·정체성(id)·전역(odom)
**중심** 좌표를 갖는 점 목록. 그래서 behavior 는 "빨간 부표는 월드 (x,y)에 있다"를
직접 알고, 자기 odom 으로 상대 기하를 다시 계산한다.

파이프라인 — 매 프레임 (계산은 buoy_localize.py 순수 모듈):
  ① classify_pixels()  물체 픽셀(검정, 또는 채도 있고 물·하늘색 아님) + 픽셀 색 라벨
  ② depth_buoys()      6 m 이내 — 물체 픽셀 → 3D → 수면 위만(반사상 제거) → 수평 덩어리
       fuse_with_lidar()   → 색 다수결, 근처 라이다 군집과 위치 융합 (camera_obstacles 방식)
  ③ lidar_buoys()      ② 로 안 잡힌 라이다 군집 (6 m 너머 등) → 영상에 투영한 창의 색 다수결
  ④ project_to_world() odom 으로 선체 좌표 → 전역 (x,y)
  ⑤ BuoyTracker        프레임 간 연결·N-of-M 확정·prune (순수 모듈)

반지름은 고정값이 아니라 매 관측 덩어리 폭으로 추정한다 (넘어진 부표 대비 상한만 둠).
카메라·라이다 장착값은 TF 가 아니라 파라미터(cam_*, lidar_offset_*)다 — occupancy_grid 와 같게.
dock 표식(YOLO·도킹 state 만)은 dock_mark_detector 가 따로 맡는다.
"""
import dataclasses
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, LaserScan, CameraInfo, Imu
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray

from kaboat_msgs.msg import Buoy, BuoyArray
from kaboat_perception.buoy_localize import (
    BuoyGeometry, camera_xy, classify_pixels, depth_buoys, effective_K,
    fuse_with_lidar, lidar_buoys, lidar_clusters, lidar_points)
from kaboat_perception.buoy_tracker import (
    BuoyTracker, TrackerParams, Detection, project_to_world)
from kaboat_perception.depth_utils import depth_to_meters

try:
    from cv_bridge import CvBridge
    CV_AVAILABLE = True
except ImportError:
    CV_AVAILABLE = False


class BuoyDetector(Node):
    def __init__(self):
        super().__init__('buoy_detector')

        # 카메라 수평 화각 [rad] — camera_info 를 못 받은 초기 프레임의 폴백.
        # xacro depth_camera(D455: 87°=1.5184)와 일치해야 방위각이 맞는다.
        self.declare_parameter('hfov_rad', 1.5184)
        self.hfov = float(self.get_parameter('hfov_rad').value)
        # ②③ 위치에 쓸 스캔의 최대 나이 [s]. /scan/level 은 배가 기울면 끊기므로,
        # 그동안 오래된 스캔(배가 그새 돌았을 수 있음)은 안 쓰고 depth(6 m 이내)만 쓴다.
        self.declare_parameter('lidar_max_age_sec', 0.3)
        self.lidar_max_age = float(self.get_parameter('lidar_max_age_sec').value)
        # 센서 장착·색 판정·뎁스 물체·융합·반지름 상한 (buoy_localize.BuoyGeometry 필드 이름 그대로).
        for f in dataclasses.fields(BuoyGeometry):
            self.declare_parameter(f.name, f.default)
        self.geom = BuoyGeometry(**{
            f.name: type(f.default)(self.get_parameter(f.name).value)
            for f in dataclasses.fields(BuoyGeometry)})

        # ⑥ 트래커 파라미터 (ROS param 로 노출 — 부표밭 실측 튜닝용)
        self.declare_parameter('gate_radius', 1.5)
        self.declare_parameter('confirm_hits', 3)
        self.declare_parameter('drop_time', 3.0)
        self.tracker = BuoyTracker(TrackerParams(
            gate_radius=float(self.get_parameter('gate_radius').value),
            confirm_hits=int(self.get_parameter('confirm_hits').value),
            drop_time=float(self.get_parameter('drop_time').value)))

        self.pub = self.create_publisher(BuoyArray, '/detections/buoys', 10)
        # RViz 디버그 — BuoyArray 는 커스텀 msg 라 RViz 가 못 읽어, 같은 부표를
        # MarkerArray(색 구 + id 텍스트)로도 그린다. 구독자 있을 때만 발행.
        self.viz_pub = self.create_publisher(MarkerArray, '/detections/buoys_viz', 10)

        if not CV_AVAILABLE:
            self.get_logger().error(
                'cv2/cv_bridge 를 찾을 수 없습니다 — 인식 비활성. '
                '컨테이너에 ros-humble-cv-bridge, python3-opencv 설치 필요.')
            return

        self.bridge = CvBridge()
        self.depth = None         # 최신 depth 프레임 (numpy, meters)
        self.scan = None          # ②③ 위치용 최신 LiDAR (/scan/level)
        self.scan_at = None       # 그 스캔을 받은 시각 (노드 시계 — sim 시간 대응)
        self.odom = None          # ④⑤ 전역 투영·시간축용 최신 odom
        self.K = None             # camera_info (fx, fy, cx, cy) — effective_K 로 검사 후 사용
        self.roll = 0.0           # IMU 롤·피치 [rad] (ROS 규약: pitch + = 선수 아래)
        self.pitch = 0.0
        # 실제 센서 드라이버는 대역폭/지연 때문에 Best Effort(SensorDataQoS)를
        # 쓰는 경우가 많다. Reliable 기본 QoS로 구독하면 DDS QoS 불일치로
        # 토픽 이름은 보여도 콜백이 한 번도 실행되지 않을 수 있다.
        self.create_subscription(
            Image, '/camera/color/image_raw', self.on_image, qos_profile_sensor_data)
        self.create_subscription(
            Image, '/camera/depth/image_raw', self._on_depth, qos_profile_sensor_data)
        self.create_subscription(
            CameraInfo, '/camera/camera_info', self._on_info, qos_profile_sensor_data)
        self.create_subscription(
            LaserScan, '/scan/level', self._on_scan, qos_profile_sensor_data)
        self.create_subscription(
            Imu, '/imu/data', self._on_imu, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odom', self._on_odom, 10)

        self.get_logger().info(
            'buoy_detector 시작 (빨강/초록/검정, 뎁스 ≤ '
            f'{self.geom.depth_max:.1f} m + 라이다 융합, 반지름 ≤ {self.geom.max_radius:.2f} m) '
            '→ /detections/buoys')

    def _on_depth(self, msg: Image):
        # 내부 표현은 항상 float32 미터. Gazebo는 32FC1[m], RealSense ROS의
        # aligned depth는 기본 16UC1[mm]라 encoding에 따라 여기서 한 번 통일한다.
        raw = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        try:
            self.depth = depth_to_meters(raw, msg.encoding)
        except ValueError as exc:
            self.depth = None
            self.get_logger().error(str(exc), throttle_duration_sec=5.0)

    def _on_info(self, msg: CameraInfo):
        k = msg.k                 # K = [fx 0 cx; 0 fy cy; 0 0 1]
        self.K = (k[0], k[4], k[2], k[5])

    def _on_scan(self, msg: LaserScan):
        self.scan = msg
        self.scan_at = self.get_clock().now()

    def _on_imu(self, msg: Imu):
        q = msg.orientation
        # 롤·피치 — 픽셀 시선을 수평으로 돌리고 수평선 행을 구하는 데 쓴다.
        self.roll = math.atan2(2.0 * (q.w * q.x + q.y * q.z),
                               1.0 - 2.0 * (q.x * q.x + q.y * q.y))
        sinp = 2.0 * (q.w * q.y - q.z * q.x)
        self.pitch = math.asin(max(-1.0, min(1.0, sinp)))

    def _on_odom(self, msg: Odometry):
        self.odom = msg

    # ── 메인 콜백 ────────────────────────────────────────────

    def on_image(self, msg: Image):
        if self.odom is None:
            return   # 전역 투영에 배 pose 필수 — odom 오기 전엔 대기

        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        height, width = frame.shape[:2]
        depth = self.depth
        if depth is not None and depth.shape[:2] != frame.shape[:2]:
            self.get_logger().error(
                'RGB/depth 해상도가 다릅니다 — RGB에 정렬된 depth 토픽을 사용하세요 '
                f'(RGB={frame.shape[:2]}, depth={depth.shape[:2]})',
                throttle_duration_sec=5.0)
            depth = None

        K = effective_K(self.K, width, height, self.hfov)
        roll, pitch = self.roll, self.pitch

        # 배 pose (④ 투영) + 시간축 (⑤ 트래커) — odom 스탬프(sim RTF<1 대응)
        pos = self.odom.pose.pose.position
        yaw = _yaw_from_quaternion(self.odom.pose.pose.orientation)
        now_t = (self.odom.header.stamp.sec
                 + self.odom.header.stamp.nanosec * 1e-9)

        scan = self._fresh_scan()
        clusters = [] if scan is None else lidar_clusters(lidar_points(
            scan.ranges, scan.angle_min, scan.angle_increment,
            scan.range_min, scan.range_max, self.geom), self.geom, scan.angle_increment)
        cam = camera_xy(K, self.geom, roll, pitch)

        obj, label = classify_pixels(frame, self.geom)                      # ①
        fixes = []
        if depth is not None:                                               # ② 6 m 이내
            fixes = [fuse_with_lidar(f, clusters, cam, self.geom)
                     for f in depth_buoys(obj, label, depth, K, self.geom, roll, pitch)]
        fixes += lidar_buoys(clusters, obj, label, K, self.geom, roll, pitch,  # ③ 나머지
                             skip=fixes, depth_m=depth)

        detections = []
        for f in fixes:
            wx, wy = project_to_world(math.atan2(f.y, f.x), math.hypot(f.x, f.y),
                                      pos.x, pos.y, yaw)      # ④
            detections.append(Detection(color=f.color, x=wx, y=wy,
                                        confidence=min(f.points / 100.0, 1.0)))

        # ⑤ 시간 융합 → 확정 부표만
        tracks = self.tracker.update(detections, now_t)
        self._publish(tracks)

    def _fresh_scan(self):
        """쓸 수 있는 /scan/level — 없거나 오래됐으면(배가 기울어 끊김) None → 뎁스만 쓴다."""
        if self.scan is None or self.scan_at is None:
            return None
        age = (self.get_clock().now() - self.scan_at).nanoseconds * 1e-9
        if age > self.lidar_max_age:
            return None
        return self.scan

    def _publish(self, tracks):
        out = BuoyArray()
        out.header.frame_id = 'odom'
        out.header.stamp = self.odom.header.stamp
        for tr in tracks:
            b = Buoy()
            b.id = tr.id
            b.color = tr.color
            b.position.x = tr.x
            b.position.y = tr.y
            b.position.z = 0.0
            b.confidence = float(tr.confidence)
            out.buoys.append(b)
        self.pub.publish(out)

        if self.count_subscribers('/detections/buoys_viz') > 0:
            self._publish_viz(tracks)

    def _publish_viz(self, tracks):
        """부표맵을 RViz MarkerArray 로 — 색 구 + id 텍스트 (frame=odom)."""
        rgb = {'red': (1.0, 0.1, 0.1), 'green': (0.1, 0.9, 0.2),
               'black': (0.15, 0.15, 0.15),
               'unknown': (0.6, 0.6, 0.6)}
        marks = MarkerArray()
        clear = Marker()             # 이전 프레임 잔상 제거 (prune 된 부표 지우기)
        clear.action = Marker.DELETEALL
        marks.markers.append(clear)
        for tr in tracks:
            r, g, b = rgb.get(tr.color, rgb['unknown'])
            sph = Marker()
            sph.header.frame_id = 'odom'
            sph.header.stamp = self.odom.header.stamp
            sph.ns, sph.id = 'buoy', tr.id
            sph.type, sph.action = Marker.SPHERE, Marker.ADD
            sph.pose.position.x, sph.pose.position.y = tr.x, tr.y
            sph.pose.position.z = 0.2
            sph.pose.orientation.w = 1.0
            sph.scale.x = sph.scale.y = sph.scale.z = 0.5
            sph.color.r, sph.color.g, sph.color.b, sph.color.a = r, g, b, 1.0
            marks.markers.append(sph)
            txt = Marker()
            txt.header = sph.header
            txt.ns, txt.id = 'buoy_id', tr.id
            txt.type, txt.action = Marker.TEXT_VIEW_FACING, Marker.ADD
            txt.pose.position.x, txt.pose.position.y = tr.x, tr.y
            txt.pose.position.z = 0.9
            txt.pose.orientation.w = 1.0
            txt.scale.z = 0.5
            txt.color.r = txt.color.g = txt.color.b = txt.color.a = 1.0
            txt.text = f'{tr.id}:{tr.color}'
            marks.markers.append(txt)
        self.viz_pub.publish(marks)


def _yaw_from_quaternion(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def main(args=None):
    rclpy.init(args=args)
    node = BuoyDetector()
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
