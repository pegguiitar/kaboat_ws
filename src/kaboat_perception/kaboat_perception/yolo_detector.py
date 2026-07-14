"""yolo_detector — 표식/부표 색·모양 인식 (상시 가동, ① 레이어)

입력   /camera/color/image_raw (sensor_msgs/Image)  — sim은 gz bridge, 실물은 D455 드라이버
출력   /detections/marks (kaboat_msgs/MarkArray)

skeleton 구현 — HSV 색 블롭 검출:
  YOLO 모델이 준비되기 전까지의 기능형 플레이스홀더.
  시뮬레이션 부표(orange/green/red)가 뚜렷한 원색이라 HSV 마스크만으로
  색/방위(bearing)까지는 검증 가능하다. shape 은 "unknown" 으로 발행.

TODO(팀): YOLO 추론으로 교체 (shape/confidence 채우기),
          /camera/depth/image_raw 로 distance 채우기
"""
import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

from kaboat_msgs.msg import Mark, MarkArray

try:
    import cv2
    import numpy as np
    from cv_bridge import CvBridge
    CV_AVAILABLE = True
except ImportError:
    CV_AVAILABLE = False

# HSV 색 범위 (OpenCV H 범위 0~180) — 시뮬레이션 부표 원색 기준.
# 실제 카메라/조명에서는 반드시 재캘리브레이션 필요.
HSV_RANGES = {
    'red':    [((0, 120, 80), (8, 255, 255)), ((172, 120, 80), (180, 255, 255))],
    'orange': [((9, 120, 80), (25, 255, 255))],
    'green':  [((40, 80, 60), (85, 255, 255))],
}

MIN_BLOB_AREA = 200      # 픽셀 — 이보다 작은 블롭은 노이즈로 무시
HFOV_RAD = 1.047         # 카메라 수평 화각 [rad] (xacro depth_camera 와 일치)


class YoloDetector(Node):
    def __init__(self):
        super().__init__('yolo_detector')

        self.pub = self.create_publisher(MarkArray, '/detections/marks', 10)

        if not CV_AVAILABLE:
            self.get_logger().error(
                'cv2/cv_bridge 를 찾을 수 없습니다 — 인식 비활성. '
                '컨테이너에 ros-humble-cv-bridge, python3-opencv 설치 필요.')
            return

        self.bridge = CvBridge()
        self.sub = self.create_subscription(
            Image, '/camera/color/image_raw', self.on_image, 10)
        self.get_logger().info(
            'yolo_detector 시작 (HSV placeholder) — /camera/color/image_raw → /detections/marks')

    def on_image(self, msg: Image):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        width = frame.shape[1]

        out = MarkArray()
        out.header = msg.header

        for color, ranges in HSV_RANGES.items():
            mask = None
            for lo, hi in ranges:
                m = cv2.inRange(hsv, np.array(lo), np.array(hi))
                mask = m if mask is None else cv2.bitwise_or(mask, m)

            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                           cv2.CHAIN_APPROX_SIMPLE)
            for cnt in contours:
                area = cv2.contourArea(cnt)
                if area < MIN_BLOB_AREA:
                    continue
                m = cv2.moments(cnt)
                cx = m['m10'] / m['m00']
                # 픽셀 오프셋 → 방위각 (이미지 좌측 = +bearing, REP-103 좌회전 양수)
                bearing = -(cx - width / 2.0) / (width / 2.0) * (HFOV_RAD / 2.0)

                mark = Mark()
                mark.color = color
                mark.shape = 'unknown'   # TODO(팀) YOLO 로 교체 시 채움
                mark.confidence = min(area / 5000.0, 1.0)  # 면적 기반 임시 값
                mark.bearing = float(bearing)
                mark.distance = -1.0     # TODO(팀) depth 이미지로 채움
                out.marks.append(mark)

        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = YoloDetector()
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
