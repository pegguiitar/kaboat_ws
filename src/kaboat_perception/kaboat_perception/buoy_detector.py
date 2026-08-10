"""buoy_detector — 부표·게이트 색 검출 → 전역 부표맵 (상시 가동, ① 레이어, 학습 불필요)

입력   /camera/color/image_raw (sensor_msgs/Image)   — RGB 원본, 블롭 검출용
       /camera/depth/image_raw (sensor_msgs/Image)    — 거리 채움 (RGB와 픽셀 정렬됨)
       /camera/camera_info (sensor_msgs/CameraInfo)    — fx·cx 로 방위각 정밀화
       /scan (sensor_msgs/LaserScan)                  — ⑤ 오검출 제거용 (LiDAR 교차확인)
       /imu/data (sensor_msgs/Imu)                    — ③ 수평선 보정용 pitch
       /odom (nav_msgs/Odometry)                      — ⑥ 전역 투영 · ⑦ 추적 시간축
출력   /detections/buoys (kaboat_msgs/BuoyArray, frame_id="odom")

**출력은 점유지도(cell)와 다른 좌표 랜드마크다** — 색·정체성(id)·전역(odom)
좌표를 갖는 점 목록. 그래서 behavior 는 "빨간 부표는 월드 (x,y)에 있다"를
직접 알고, 자기 odom 으로 상대 기하를 다시 계산한다.

파이프라인(buoy_detector_pipeline 확장판) — 매 프레임:
  ① hsv_blobs()        HSV 색 마스크 → 색 덩어리 후보               [구현]
  ② filter_geometry()  둥글고 꽉 찬 것만 (눈부심 줄무늬 제거)        [구현]
  ③ filter_waterline() 수평선 아래(물 위)만 (하늘·물가 제거)         [구현]
  ④ 거리·방위          depth 3×3 median + camera_info fx 로 bearing  [구현]
  ⑤ lidar_confirms()   그 방위에 라이다 반사 있나 (광학 유령 제거)   [구현]
  ⑥ project_to_world() odom 으로 배기준 극좌표 → 전역 (x,y)          [구현]
  ⑦ BuoyTracker        프레임 간 연결·N-of-M 확정·prune (순수 모듈)  [구현]

⑦ 시간 융합은 ROS 무의존 순수 모듈 buoy_tracker.py 에 있고, 이 노드는
토픽을 받아 ①~⑥ 을 돌려 월드 검출을 만든 뒤 트래커에 투입하는 배선판이다.
dock 표식(YOLO·도킹 state 만)은 dock_mark_detector 가 따로 맡는다.
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, LaserScan, CameraInfo, Imu
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray

from kaboat_msgs.msg import Buoy, BuoyArray
from kaboat_perception.buoy_tracker import (
    BuoyTracker, TrackerParams, Detection, project_to_world)
from kaboat_perception.depth_utils import depth_to_meters, distance_at

try:
    import cv2
    import numpy as np
    from cv_bridge import CvBridge
    CV_AVAILABLE = True
except ImportError:
    CV_AVAILABLE = False

# HSV 색 범위 (OpenCV H 범위 0~180) — 시뮬레이션 부표 원색 기준.
# 실제 카메라/조명에서는 반드시 재캘리브레이션 필요 (셔터·AWB 고정 후 현장 캘리브).
HSV_RANGES = {
    'red':    [((0, 120, 80), (8, 255, 255)), ((172, 120, 80), (180, 255, 255))],
    'orange': [((9, 120, 80), (25, 255, 255))],
    'green':  [((40, 80, 60), (85, 255, 255))],
}

MIN_BLOB_AREA = 200      # 픽셀 — 이보다 작은 블롭은 노이즈로 무시

# ② 기하 필터 임계 — 부표는 둥글고(원형도↑) 꽉 찬(solidity↑) 덩어리.
MIN_CIRCULARITY = 0.55   # 4πA/P² : 1=완전한 원, 가늘고 긴 줄무늬는 낮다
MIN_SOLIDITY    = 0.80   # A/볼록껍질 : 요철 많은 반사 얼룩은 낮다
MAX_ASPECT      = 3.0    # 외접 사각형 종횡비 상한 (길쭉한 것 제거)


def hsv_blobs(frame, hsv_ranges=HSV_RANGES, min_area=MIN_BLOB_AREA):
    """① HSV 마스크 → (color, contour, area, cx, cy) 후보 목록.

    cx,cy 는 블롭 중심의 픽셀 좌표 — cx 는 방위각(bearing) 계산에,
    cy 는 depth 이미지에서 같은 픽셀의 거리를 읽을 때 같이 쓰인다.
    """
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    candidates = []
    for color, ranges in hsv_ranges.items():
        mask = None
        for lo, hi in ranges:
            m = cv2.inRange(hsv, np.array(lo), np.array(hi))
            mask = m if mask is None else cv2.bitwise_or(mask, m)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                       cv2.CHAIN_APPROX_SIMPLE)
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < min_area:
                continue
            m = cv2.moments(cnt)
            candidates.append((color, cnt, area, m['m10'] / m['m00'], m['m01'] / m['m00']))
    return candidates


def filter_geometry(candidates):
    """② 블롭 기하 필터 — 둥글고 꽉 찬 덩어리만 통과.

    수면 반사 눈부심은 가늘고 길쭉하거나(낮은 원형도) 요철투성이(낮은
    solidity)라 여기서 걸러진다. 부표는 거의 원반이라 셋 다 넉넉히 통과한다.
    """
    kept = []
    for cand in candidates:
        _color, cnt, area, _cx, _cy = cand
        perim = cv2.arcLength(cnt, True)
        if perim <= 0:
            continue
        circularity = 4.0 * math.pi * area / (perim * perim)
        hull_area = cv2.contourArea(cv2.convexHull(cnt))
        solidity = area / hull_area if hull_area > 0 else 0.0
        _, _, w, h = cv2.boundingRect(cnt)
        aspect = max(w, h) / max(min(w, h), 1)
        if (circularity >= MIN_CIRCULARITY and solidity >= MIN_SOLIDITY
                and aspect <= MAX_ASPECT):
            kept.append(cand)
    return kept


def filter_waterline(candidates, sky_row):
    """③ 수면 컨텍스트 — 하늘 컷선(sky_row) 아래만 통과, 명백한 하늘만 제거.

    ⚠️ 부표는 수평선 "아래"에 있지 않다 — 키가 있어 수평선 위로 솟고,
    가까울수록 콘 높이가 크게 잡혀 중심(cy)이 더 위로 올라간다(sim 실측:
    5m 부표 cy≈0.43H, 20m 부표 cy≈0.50H, 수평선도 그 근처). 그래서
    "수평선 아래만" 으로 자르면 부표가 통째로 잘린다. sky_row 는 참 수평선보다
    한참 위(작은 cy)의 컷선으로, 그보다 위에 있는 건 부표일 수 없는 하늘·
    구름·먼 물가만 — 그것만 버린다. 노드가 IMU pitch 로 보정해 넘긴다.
    """
    return [c for c in candidates if c[4] >= sky_row]


def bearing_from_pixel(cx: float, width: float, fx: float, ppx: float,
                       hfov: float) -> float:
    """픽셀 x → 방위각[rad, 좌측 +] (핀홀 atan 모델).

    ⚠️ camera_info 의 K 는 신뢰할 수 있을 때만 쓴다 — 이 sim 의 gz rgbd
    camera_info 는 width=1280 이라 하면서 K 는 320px 기준(ppx≈160, fx≈277)
    으로 어긋나 있다. 그대로 쓰면 주점이 중심(640)에서 480px 벗어나 모든
    방위가 ~60° 오른쪽으로 쏠린다. 그래서 주점이 이미지 중심 근처일 때만
    (실물 D455 는 정상) camera_info 를 쓰고, 아니면 xacro HFOV 와 실제 이미지
    폭으로 초점거리를 유도해 쓴다(자기 일관적).
    """
    if fx is not None and fx > 0.0 and abs(ppx - width / 2.0) < 0.1 * width:
        return math.atan2(ppx - cx, fx)                 # 신뢰 가능한 K
    f = (width / 2.0) / math.tan(hfov / 2.0)            # HFOV·폭에서 유도
    return math.atan2(width / 2.0 - cx, f)              # 좌측(cx<중심) → +


def lidar_confirms(ranges, angle_min, angle_inc, bearing, distance,
                   beam_window=2, range_tol=1.5):
    """⑤ 그 방위에 라이다 반사가 있나 — 광학(카메라) 오검출의 마지막 관문.

    수면 반사·눈부심은 카메라엔 물체처럼 보여도 라이다엔 반환값이 없다.
    bearing 방향의 빔 몇 개(beam_window)를 보고, 유효 반사가 있으면 통과.
    depth 거리를 알면(≥0) 그 거리와 range_tol 안에서 일치할 때만 통과한다.
    스캔이 없으면(None) 게이팅하지 않고 통과(라이다 미가동 시 비전만).
    """
    if ranges is None or angle_inc == 0.0:
        return True
    idx = int(round((bearing - angle_min) / angle_inc))
    n = len(ranges)
    for j in range(idx - beam_window, idx + beam_window + 1):
        if not (0 <= j < n):
            continue
        r = ranges[j]
        if not math.isfinite(r) or r <= 0.0:
            continue
        if distance < 0.0 or abs(r - distance) <= range_tol:
            return True
    return False


class BuoyDetector(Node):
    def __init__(self):
        super().__init__('buoy_detector')

        # 카메라 수평 화각 [rad] — camera_info 를 못 받은 초기 프레임의 폴백.
        # xacro depth_camera(D455: 87°=1.5184)와 일치해야 방위각이 맞는다.
        self.declare_parameter('hfov_rad', 1.5184)
        self.hfov = float(self.get_parameter('hfov_rad').value)
        # ③ 하늘 컷선 위치 (이미지 높이 비율) — 이 행보다 위(작은 cy)만 하늘로
        # 보고 버린다. 참 수평선보다 위여야 부표(수평선 위로 솟음)를 안 자른다
        # (filter_waterline docstring 의 sim 실측 근거). 실물은 카메라 장착
        # 각도·높이에 맞춰 재캘리브(§7).
        self.declare_parameter('horizon_frac', 0.30)
        self.horizon_frac = float(self.get_parameter('horizon_frac').value)

        # ⑦ 트래커 파라미터 (ROS param 로 노출 — 부표밭 실측 튜닝용)
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
        self.scan = None          # ⑤ 교차확인용 최신 LiDAR
        self.odom = None          # ⑥⑦ 전역 투영·시간축용 최신 odom
        self.fx = None            # camera_info 초점거리 px (방위각 정밀화)
        self.ppx = None           # camera_info 주점 x px
        self.pitch = 0.0          # ③ 수평선 보정용 IMU pitch [rad]
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
            LaserScan, '/scan', self._on_scan, qos_profile_sensor_data)
        self.create_subscription(
            Imu, '/imu/data', self._on_imu, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odom', self._on_odom, 10)

        self.get_logger().info(
            'buoy_detector 시작 (HSV→기하→수면→depth→lidar→전역추적) → /detections/buoys')

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
        self.fx = msg.k[0]        # K = [fx 0 cx; 0 fy cy; 0 0 1]
        self.ppx = msg.k[2]

    def _on_scan(self, msg: LaserScan):
        self.scan = msg

    def _on_imu(self, msg: Imu):
        q = msg.orientation
        # pitch (전후 기울기) — 수평선 행 보정에 사용.
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

        # ① 색 → ② 기하 → ③ 수면
        candidates = hsv_blobs(frame)
        candidates = filter_geometry(candidates)
        candidates = filter_waterline(candidates, self._sky_cut_row(height))

        # 배 pose (⑥ 투영) + 시간축 (⑦ 트래커) — odom 스탬프(sim RTF<1 대응)
        pos = self.odom.pose.pose.position
        yaw = _yaw_from_quaternion(self.odom.pose.pose.orientation)
        now_t = (self.odom.header.stamp.sec
                 + self.odom.header.stamp.nanosec * 1e-9)

        detections = []
        for color, _cnt, area, cx, cy in candidates:
            bearing = bearing_from_pixel(cx, width, self.fx, self.ppx, self.hfov)
            distance = distance_at(depth, cx, cy)          # ④ depth median
            if distance < 0.0:
                continue                                   # 거리 미상 부표는 투영 불가
            if not self._lidar_ok(bearing, distance):      # ⑤ 라이다 교차확인
                continue
            wx, wy = project_to_world(bearing, distance, pos.x, pos.y, yaw)  # ⑥
            detections.append(Detection(color=color, x=wx, y=wy,
                                        confidence=min(area / 5000.0, 1.0)))

        # ⑦ 시간 융합 → 확정 부표만
        tracks = self.tracker.update(detections, now_t)
        self._publish(tracks)

    def _sky_cut_row(self, height: int) -> float:
        """③ 하늘 컷선 픽셀 행 — 이미지 비율 기준선에 IMU pitch 보정을 더한다.

        이 행보다 위(작은 cy)는 부표일 수 없는 하늘로 본다. pitch>0(뱃머리 위)
        이면 수평선이 이미지 아래로 내려가므로(행 증가) 컷선도 같이 내린다. 세로
        초점거리는 fy≈fx 로 근사(D455 정사각 픽셀). camera_info 전이면 비율만.
        """
        base = height * self.horizon_frac
        if self.fx is not None and self.fx > 0.0:
            return base + math.tan(self.pitch) * self.fx
        return base

    def _lidar_ok(self, bearing: float, distance: float) -> bool:
        if self.scan is None:
            return True
        return lidar_confirms(self.scan.ranges, self.scan.angle_min,
                              self.scan.angle_increment, bearing, distance)

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
               'orange': (1.0, 0.5, 0.0), 'unknown': (0.6, 0.6, 0.6)}
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
            sph.scale.x = sph.scale.y = sph.scale.z = 0.6
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
