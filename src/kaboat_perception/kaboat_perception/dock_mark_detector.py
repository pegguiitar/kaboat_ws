"""Gated D455 YOLO26-seg detector and depth-based dock marker locator.

Inputs: aligned color/depth/camera_info and latched /detector/enable.
Outputs: /detections/dock_marks, /dock/marker_base (3D, base_link),
         /dock/detection_status.
Only the marker position is estimated here. The dock axis / entrance comes
from the lidar back-wall fit in docking_ctrl, not from a depth plane normal.
No marker is published without valid depth and TF.
"""

import json
import math
from pathlib import Path

import cv2
import numpy as np
if int(np.__version__.split('.')[0]) >= 2:
    raise RuntimeError('ROS Humble cv_bridge requires a NumPy 1.x runtime; '
                       'use a compatible ROS/YOLO environment')
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped
from kaboat_msgs.msg import Mark, MarkArray
from message_filters import ApproximateTimeSynchronizer, Subscriber
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Bool, String
from tf2_geometry_msgs import do_transform_point
from tf2_ros import Buffer, TransformException, TransformListener

from .depth_utils import depth_to_meters
from .dock_geometry import estimate_marker
from .dock_segmentation import CLASS_NAMES, CLASS_PARTS, DockSegmenter


class DockMarkDetector(Node):
    def __init__(self):
        super().__init__('dock_mark_detector')
        for name, value in {
            'weights': '', 'target_class': 'red_triangle', 'base_frame': 'base_link',
            'confidence': 0.45, 'image_size': 640,
            'device': 'cpu', 'min_depth_m': 0.25, 'max_depth_m': 12.0,
            'min_mask_pixels': 40, 'min_valid_fraction': 0.5,
            'sync_slop_s': 0.08,
        }.items():
            self.declare_parameter(name, value)
        self.weights = str(self.get_parameter('weights').value)
        self.target_class = str(self.get_parameter('target_class').value)
        if self.target_class not in CLASS_NAMES:
            raise ValueError(f'target_class must be one of {CLASS_NAMES}')
        self.base_frame = str(self.get_parameter('base_frame').value)
        self.enabled = False
        self.segmenter = None
        self.model_error = ''
        self.bridge = CvBridge()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.marks_pub = self.create_publisher(MarkArray, '/detections/dock_marks', 10)
        self.marker_pub = self.create_publisher(PointStamped, '/dock/marker_base', 10)
        self.status_pub = self.create_publisher(String, '/dock/detection_status', 10)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Bool, '/detector/enable', self._on_enable, latched)
        image = Subscriber(self, Image, '/camera/color/image_raw',
                           qos_profile=qos_profile_sensor_data)
        depth = Subscriber(self, Image, '/camera/depth/image_raw',
                           qos_profile=qos_profile_sensor_data)
        info = Subscriber(self, CameraInfo, '/camera/camera_info',
                          qos_profile=qos_profile_sensor_data)
        self.sync = ApproximateTimeSynchronizer(
            [image, depth, info], queue_size=12,
            slop=float(self.get_parameter('sync_slop_s').value))
        self.sync.registerCallback(self.on_frame)
        if not self.weights or not Path(self.weights).is_file():
            self.get_logger().error('Dock YOLO weights unset or missing; detector fails closed.')

    def _on_enable(self, msg: Bool):
        self.enabled = bool(msg.data)
        if self.enabled and self.segmenter is None and not self.model_error:
            try:
                if not self.weights or not Path(self.weights).is_file():
                    raise FileNotFoundError(self.weights or 'weights parameter unset')
                self.segmenter = DockSegmenter(
                    self.weights, float(self.get_parameter('confidence').value),
                    int(self.get_parameter('image_size').value),
                    str(self.get_parameter('device').value))
                self.get_logger().info(f'Dock YOLO ready: {self.weights}')
            except Exception as exc:
                self.model_error = str(exc)
                self.get_logger().error(f'Dock YOLO unavailable: {exc}')

    def _status(self, valid: bool, reason: str, **extra):
        msg = String()
        msg.data = json.dumps({'valid': valid, 'reason': reason,
                               'target_class': self.target_class, **extra})
        self.status_pub.publish(msg)

    @staticmethod
    def _point(stamp, frame, xyz):
        point = PointStamped()
        point.header.stamp = stamp
        point.header.frame_id = frame
        point.point.x, point.point.y, point.point.z = map(float, xyz)
        return point

    def on_frame(self, rgb_msg: Image, depth_msg: Image, info_msg: CameraInfo):
        if not self.enabled:
            return
        if self.segmenter is None:
            self._status(False, 'model_unavailable', detail=self.model_error)
            return
        if (rgb_msg.width != depth_msg.width or rgb_msg.height != depth_msg.height
                or (info_msg.width, info_msg.height) != (rgb_msg.width, rgb_msg.height)):
            self._status(False, 'unaligned_rgb_depth_or_info')
            return
        fx, fy, cx, cy = map(float, (info_msg.k[0], info_msg.k[4],
                                      info_msg.k[2], info_msg.k[5]))
        if min(fx, fy) <= 0 or not all(map(math.isfinite, (fx, fy, cx, cy))):
            self._status(False, 'invalid_intrinsics')
            return
        if (any(abs(value) > 0 for value in info_msg.d)
                and info_msg.distortion_model not in ('plumb_bob', 'rational_polynomial')):
            self._status(False, 'unsupported_distortion_model')
            return
        try:
            rgb = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='bgr8')
            raw_depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
            depth = depth_to_meters(raw_depth, depth_msg.encoding)
            detections = self.segmenter.predict(rgb)
        except (ValueError, cv2.error, RuntimeError) as exc:
            self._status(False, 'image_or_inference_error', detail=str(exc))
            return
        marks = MarkArray()
        marks.header = rgb_msg.header
        estimates = {}
        for detection in detections:
            try:
                estimate = estimate_marker(
                    detection.mask, depth, fx, fy, cx, cy,
                    min_depth=float(self.get_parameter('min_depth_m').value),
                    max_depth=float(self.get_parameter('max_depth_m').value),
                    min_pixels=int(self.get_parameter('min_mask_pixels').value),
                    distortion=info_msg.d)
            except (cv2.error, ValueError) as exc:
                self._status(False, 'marker_geometry_error', detail=str(exc))
                return
            if estimate is None or estimate.valid_fraction < float(
                    self.get_parameter('min_valid_fraction').value):
                continue
            estimates[detection.class_name] = (detection, estimate)
            mark = Mark()
            mark.color, mark.shape = CLASS_PARTS[detection.class_name]
            mark.confidence = detection.confidence
            x, _, z = estimate.point_optical
            mark.bearing = math.atan2(-x, z)
            mark.distance = math.hypot(x, z)
            marks.marks.append(mark)
        self.marks_pub.publish(marks)
        if self.target_class not in estimates:
            self._status(False, 'target_missing_or_invalid_depth')
            return
        detection, estimate = estimates[self.target_class]
        optical_frame = rgb_msg.header.frame_id
        if not optical_frame:
            self._status(False, 'missing_optical_frame')
            return
        stamp = rgb_msg.header.stamp
        try:
            transform = self.tf_buffer.lookup_transform(
                self.base_frame, optical_frame, rclpy.time.Time.from_msg(stamp),
                timeout=Duration(seconds=0.1))
            marker_optical = self._point(stamp, optical_frame, estimate.point_optical)
            marker_base = do_transform_point(marker_optical, transform)
        except TransformException as exc:
            self._status(False, 'camera_to_base_tf_unavailable', detail=str(exc))
            return
        self.marker_pub.publish(marker_base)
        self._status(True, 'ok', confidence=detection.confidence,
                     depth_m=estimate.median_depth_m,
                     valid_fraction=estimate.valid_fraction)


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
