"""dock_mark_detector — 도킹 표식 인식 (YOLO 자리, 도킹 state 에서만 추론)

입력   /camera/color/image_raw (sensor_msgs/Image)
       /camera/depth/image_raw (sensor_msgs/Image)    — 거리 채움 (buoy_detector 와 동일)
       /detector/enable (std_msgs/Bool, latched) — mission_manager 가 게이팅
출력   /detections/dock_marks (kaboat_msgs/MarkArray)

node_structure v5 의 dock_mark_detector. YOLO 추론(모양 3클래스: 원·삼각·
사각)은 연산이 비싸서 **도킹 state 에서만 ON** 한다 — mission_manager 가
state == 'dock' 일 때만 /detector/enable=true 를 latched 로 발행하고,
이 노드는 disabled 동안 이미지 콜백을 즉시 리턴해 추론 비용을 0 으로 만든다.
부표·게이트는 buoy_detector(HSV, 상시)가 따로 맡는다 — 이 분리가 v5 의 핵심.

skeleton 구현 — 아직 YOLO 아님:
  buoy_detector.hsv_blobs() 를 재사용한 색 블롭 placeholder.
  shape 은 "unknown", distance 는 buoy_detector.distance_at() 재사용해
  뎁스 이미지로 채움 (도킹 최종 접근 정렬에 거리가 중요해서 먼저 반영).

TODO(팀): YOLOv8 추론으로 교체 —
  1) 모양 3클래스(circle/triangle/square) 모델 로드 (ultralytics)
  2) bbox 내부 HSV 로 색 확정 (v5: "색은 bbox 내부 HSV 확정")
  3) enable=false 전환 시 모델을 GPU 에서 내리는(또는 유지하는) 정책 결정
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Bool

from kaboat_msgs.msg import Mark, MarkArray

try:
    from cv_bridge import CvBridge
    from kaboat_perception.buoy_detector import hsv_blobs, distance_at, CV_AVAILABLE
except ImportError:
    CV_AVAILABLE = False


class DockMarkDetector(Node):
    def __init__(self):
        super().__init__('dock_mark_detector')

        self.declare_parameter('hfov_rad', 1.5184)   # buoy_detector 와 동일 근거
        self.hfov = float(self.get_parameter('hfov_rad').value)

        self.enabled = False
        self.pub = self.create_publisher(MarkArray, '/detections/dock_marks', 10)

        # mission_manager 가 latched 로 발행 — 이 노드가 늦게 떠도 최신 상태 수신
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Bool, '/detector/enable', self._on_enable, latched)

        if not CV_AVAILABLE:
            self.get_logger().error(
                'cv2/cv_bridge 를 찾을 수 없습니다 — 인식 비활성.')
            return

        self.bridge = CvBridge()
        self.depth = None   # buoy_detector 와 동일한 뎁스 프레임 캐시 패턴
        self.create_subscription(Image, '/camera/color/image_raw', self.on_image, 10)
        self.create_subscription(Image, '/camera/depth/image_raw', self._on_depth, 10)
        self.get_logger().info(
            'dock_mark_detector 대기 (YOLO placeholder=HSV) — '
            '/detector/enable=true 인 동안만 추론 → /detections/dock_marks')

    def _on_depth(self, msg: Image):
        self.depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

    def _on_enable(self, msg: Bool):
        if msg.data != self.enabled:
            self.get_logger().info(
                f"추론 {'ON (도킹 state 진입)' if msg.data else 'OFF (도킹 state 이탈)'}")
        self.enabled = msg.data
        # TODO(팀): YOLO 교체 후 — ON 전환 시 모델 로드/워밍업, OFF 시 해제 정책

    def on_image(self, msg: Image):
        if not self.enabled:
            return   # 게이팅 — disabled 동안 추론 비용 0

        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        width = frame.shape[1]

        out = MarkArray()
        out.header = msg.header
        # TODO(팀): 여기를 YOLO 추론으로 교체 (아래는 HSV placeholder)
        for color, _cnt, area, cx, cy in hsv_blobs(frame):
            bearing = -(cx - width / 2.0) / (width / 2.0) * (self.hfov / 2.0)
            mark = Mark()
            mark.color = color
            mark.shape = 'unknown'   # TODO(팀): YOLO 모양 클래스 (circle/triangle/square)
            mark.confidence = min(area / 5000.0, 1.0)
            mark.bearing = float(bearing)
            mark.distance = distance_at(self.depth, cx, cy)
            out.marks.append(mark)
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = DockMarkDetector()
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
