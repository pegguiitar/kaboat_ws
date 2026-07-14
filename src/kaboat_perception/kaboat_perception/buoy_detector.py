"""buoy_detector — 부표·게이트 색 검출 (상시 가동, ① 레이어, 학습 불필요)

입력   /camera/color/image_raw (sensor_msgs/Image)   — RGB 원본
       /scan (sensor_msgs/LaserScan)                  — ④ LiDAR 교차확인용
       /camera/depth/points (sensor_msgs/PointCloud2) — ④ 거리 보강용 (선택)
       /odom (nav_msgs/Odometry)                      — ⑤ 시간 추적용
출력   /detections/buoys (kaboat_msgs/MarkArray)

node_structure v5 의 buoy_detector. HSV 는 1단계 앞단 마스크일 뿐이고,
아래 5단계를 AND 로 겹쳐 "빨간 점"이 아니라 "부표"로 확정하는 구조다
(buoy_detector_pipeline.svg 참고). YOLO 와 무관 — 항로추종·위치유지·탐색이
쓰는 이 노드는 학습 없이 항상 켜져 있고, 도킹 표식만 dock_mark_detector
(YOLO, 도킹 state 에서만 추론)가 따로 맡는다.

skeleton 구현 — 현재는 ① HSV 마스크 + 최소 면적만 실동작.
②~⑤ 는 인터페이스(메서드 훅)만 잡아둔 통과(pass-through) 상태다:
  ② _filter_geometry()      블롭 기하 (circularity/solidity/종횡비)
  ③ _filter_waterline()     수면 컨텍스트 (수평선 위 하늘 마스킹)
  ④ _crosscheck_lidar()     LiDAR/depth 교차확인 + 거리 채움  ← 오검출 제거 핵심
  ⑤ _track_temporal()       N-of-M 프레임 지속성 + odom 추적

TODO(팀): ②~⑤ 순서대로 채우기. ④가 가장 효과 큼 (수면 반사·물가 물체 제거
          + distance 필드가 여기서 채워져 behavior 들이 거리 기반 판단 가능).
"""
import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan, PointCloud2
from nav_msgs.msg import Odometry

from kaboat_msgs.msg import Mark, MarkArray

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


def hsv_blobs(frame, hsv_ranges=HSV_RANGES, min_area=MIN_BLOB_AREA):
    """① HSV 마스크 → (color, contour, area, cx) 후보 목록.

    dock_mark_detector 의 placeholder 도 이 함수를 재사용한다
    (YOLO 교체 전까지 두 검출기가 같은 앞단을 공유).
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
            candidates.append((color, cnt, area, m['m10'] / m['m00']))
    return candidates


class BuoyDetector(Node):
    def __init__(self):
        super().__init__('buoy_detector')

        # 카메라 수평 화각 [rad] — xacro depth_camera(D455: 87°=1.5184)와 일치해야
        # 방위각이 맞는다. 카메라 스펙을 바꾸면 이 파라미터도 같이 갱신할 것.
        self.declare_parameter('hfov_rad', 1.5184)
        self.hfov = float(self.get_parameter('hfov_rad').value)

        self.pub = self.create_publisher(MarkArray, '/detections/buoys', 10)

        if not CV_AVAILABLE:
            self.get_logger().error(
                'cv2/cv_bridge 를 찾을 수 없습니다 — 인식 비활성. '
                '컨테이너에 ros-humble-cv-bridge, python3-opencv 설치 필요.')
            return

        self.bridge = CvBridge()
        self.scan = None          # ④ 교차확인용 최신 LiDAR
        self.odom = None          # ⑤ 추적용 최신 odom
        self.create_subscription(Image, '/camera/color/image_raw', self.on_image, 10)
        self.create_subscription(LaserScan, '/scan', self._on_scan, 10)
        self.create_subscription(Odometry, '/odom', self._on_odom, 10)
        # TODO(팀): ④에서 depth 보강이 필요해지면 /camera/depth/points 구독 추가
        #           (PointCloud2 파싱 비용이 있으니 /scan 매칭으로 부족할 때만).

        self.get_logger().info(
            'buoy_detector 시작 (HSV+①만 실동작, ②~⑤ TODO) — → /detections/buoys')

    def _on_scan(self, msg: LaserScan):
        self.scan = msg

    def _on_odom(self, msg: Odometry):
        self.odom = msg

    # ── 파이프라인 ②~⑤ 훅 (지금은 전부 통과) ──────────────────

    def _filter_geometry(self, candidates):
        """② 블롭 기하 필터 — 둥글고 꽉 찬 덩어리만.

        TODO(팀): circularity(4πA/P²)·solidity(A/convexhull)·종횡비 임계로
                  수면 반사 눈부심 줄무늬·가늘고 긴 형상 제거.
        """
        return candidates

    def _filter_waterline(self, candidates, frame):
        """③ 수면 컨텍스트 — waterline 밴드 아래만 통과.

        TODO(팀): 수평선 추정(또는 고정 상단 비율 마스킹)으로 하늘·구름·
                  물가 지형지물 후보 제거. IMU 롤/피치로 수평선 보정 가능.
        """
        return candidates

    def _crosscheck_lidar(self, marks):
        """④ LiDAR/depth 교차확인 — 가장 강력한 오검출 제거 + 거리 채움.

        TODO(팀): mark.bearing 방향의 /scan range 를 조회해
                  (1) 실제 물체가 있는지 확인 (없으면 수면 반사 → 탈락)
                  (2) mark.distance 를 그 range 로 채움 (지금은 -1.0 고정)
                  (3) 거리 대비 블롭 픽셀 크기가 부표 실물 크기와 맞는지 검증.
        """
        return marks

    def _track_temporal(self, marks):
        """⑤ 시간 지속성 — N-of-M 프레임 만족한 안정 검출만 확정.

        TODO(팀): odom 기반 전역좌표 트래킹으로 프레임 간 동일 부표 연결,
                  깜빡이는 단발 검출(물결 반짝임 등) 제거.
        """
        return marks

    # ── 메인 콜백 ────────────────────────────────────────────

    def on_image(self, msg: Image):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        width = frame.shape[1]

        # ① HSV 후보 → ② 기하 → ③ 수면 컨텍스트
        candidates = hsv_blobs(frame)
        candidates = self._filter_geometry(candidates)
        candidates = self._filter_waterline(candidates, frame)

        out = MarkArray()
        out.header = msg.header
        for color, _cnt, area, cx in candidates:
            # 픽셀 오프셋 → 방위각 (이미지 좌측 = +bearing, REP-103 좌회전 양수)
            bearing = -(cx - width / 2.0) / (width / 2.0) * (self.hfov / 2.0)
            mark = Mark()
            mark.color = color
            mark.shape = 'buoy'
            mark.confidence = min(area / 5000.0, 1.0)  # 면적 기반 임시 값
            mark.bearing = float(bearing)
            mark.distance = -1.0     # ④에서 채워질 예정
            out.marks.append(mark)

        # ④ LiDAR 교차확인 → ⑤ 시간 추적
        out.marks = self._track_temporal(self._crosscheck_lidar(out.marks))
        self.pub.publish(out)


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
