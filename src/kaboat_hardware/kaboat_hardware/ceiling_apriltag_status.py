"""천장 카메라 보정·검출·PC 시각 상태를 사람이 읽기 쉽게 진단한다."""

import math

import rclpy
from apriltag_msgs.msg import AprilTagDetectionArray
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo


def stamp_sec(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def camera_info_is_calibrated(msg):
    """PnP에 필요한 fx/fy와 homogeneous K[2,2]가 유효한지 검사한다."""
    return (
        len(msg.k) == 9
        and all(math.isfinite(value) for value in msg.k)
        and msg.k[0] > 0.0
        and msg.k[4] > 0.0
        and msg.k[8] != 0.0
        and msg.width > 0
        and msg.height > 0
    )


class CeilingAprilTagStatus(Node):
    def __init__(self):
        super().__init__('ceiling_apriltag_status')
        self.declare_parameter(
            'camera_info_topic', '/ceiling_cam/camera/camera_info')
        self.declare_parameter('detections_topic', '/apriltag/detections')
        self.declare_parameter('detection_timeout_sec', 0.5)
        self.declare_parameter('capture_latency_tolerance_sec', 0.2)

        self.timeout = float(
            self.get_parameter('detection_timeout_sec').value)
        self.latency_tolerance = float(
            self.get_parameter('capture_latency_tolerance_sec').value)
        self.camera_info = None
        self.last_array_time = None
        self.last_tag_time = None
        self.last_ids = []
        self.last_latency = None
        self._last_state = None

        self.create_subscription(
            CameraInfo,
            str(self.get_parameter('camera_info_topic').value),
            self._on_camera_info,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            AprilTagDetectionArray,
            str(self.get_parameter('detections_topic').value),
            self._on_detections,
            qos_profile_sensor_data,
        )
        self.create_timer(1.0, self._report)
        self.get_logger().info(
            '천장 AprilTag 상태 감시 시작 — 카메라 보정과 검출 지연을 확인합니다.')

    def _now_sec(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_camera_info(self, msg):
        self.camera_info = msg

    def _on_detections(self, msg):
        now = self._now_sec()
        self.last_array_time = now
        if not msg.detections:
            return
        self.last_tag_time = now
        self.last_ids = [detection.id for detection in msg.detections]
        self.last_latency = now - stamp_sec(msg.header.stamp)

    def _report(self):
        now = self._now_sec()
        if self.camera_info is None:
            self._transition('no_camera_info', 'CameraInfo 없음 — 웹캠/토픽 확인')
            return
        if not camera_info_is_calibrated(self.camera_info):
            self._transition(
                'uncalibrated',
                '카메라 미보정 — AprilTag 좌표를 사용하지 마세요. '
                'cameracalibrator로 보정 YAML을 먼저 만드세요.')
            return
        if self.last_array_time is None or now - self.last_array_time > self.timeout:
            self._transition(
                'no_detector', '검출 메시지 없음 — apriltag 노드/토픽 확인')
            return
        if self.last_tag_time is None or now - self.last_tag_time > self.timeout:
            self._transition(
                'tag_lost', '태그 미검출 — ID·크기·조명·화면 내 태그 크기 확인')
            return
        if (self.last_latency is not None
                and abs(self.last_latency) > self.latency_tolerance):
            self._transition(
                'capture_latency',
                f'카메라 stamp 지연/역행 이상: {self.last_latency:+.3f}s — '
                'use_node_time과 카메라 부하 확인')
            return
        self._transition(
            'ok', f'정상 — tag IDs={self.last_ids}, '
            f'검출 지연={self.last_latency:.3f}s')

    def _transition(self, state, message):
        if state == self._last_state:
            return
        self._last_state = state
        if state == 'ok':
            self.get_logger().info(message)
        else:
            self.get_logger().warning(message)


def main(args=None):
    rclpy.init(args=args)
    node = CeilingAprilTagStatus()
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
