"""Gated D455 YOLO26-seg detector and depth-based dock entrance estimator.

Inputs: aligned color/depth/camera_info and latched /detector/enable.
Outputs: /detections/dock_marks, /dock/marker_base (3D),
         /dock/normal_base (unit vector), /dock/entrance_base (boat-local z=0),
         /dock/detection_status.
No entrance is published without valid depth, a stable marker plane, and TF.
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
from geometry_msgs.msg import Point, PointStamped, Vector3Stamped
from kaboat_msgs.msg import Mark, MarkArray
from message_filters import ApproximateTimeSynchronizer, Subscriber
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Bool, String
from tf2_geometry_msgs import do_transform_point
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker

from .depth_utils import depth_to_meters
from .dock_geometry import (extract_marker_cloud, estimate_common_wall,
                            marker_on_wall, project_entrance_xy)
from .dock_segmentation import CLASS_NAMES, DockSegmenter
from .dock_visualization import annotate_dock_frame


class DockMarkDetector(Node):
    def __init__(self):
        super().__init__('dock_mark_detector')
        for name, value in {
            'weights': '', 'target_class': 'triangle', 'base_frame': 'base_link',
            'wall_to_mouth_m': 2.5, 'confidence': 0.45, 'image_size': 640,
            'device': 'cpu', 'min_depth_m': 0.25, 'max_depth_m': 12.0,
            'min_mask_pixels': 40, 'min_valid_fraction': 0.5,
            'max_plane_rms_m': 0.08, 'sync_slop_s': 0.08,
            'wall_inlier_distance_m': 0.03, 'wall_min_inlier_fraction': 0.6,
            'mask_erosion_pixels': 1,
            'enabled': False, 'max_frame_age_s': 0.5,
            'publish_debug_image': False,
        }.items():
            self.declare_parameter(name, value)
        self.weights = str(self.get_parameter('weights').value)
        self.target_class = str(self.get_parameter('target_class').value)
        if self.target_class not in CLASS_NAMES:
            raise ValueError(f'target_class must be one of {CLASS_NAMES}')
        self.base_frame = str(self.get_parameter('base_frame').value)
        self.wall_to_mouth = float(self.get_parameter('wall_to_mouth_m').value)
        self.publish_debug = bool(self.get_parameter('publish_debug_image').value)
        if not math.isfinite(self.wall_to_mouth) or self.wall_to_mouth < 0:
            raise ValueError('wall_to_mouth_m must be finite and non-negative')
        self.enabled = False
        self.segmenter = None
        self.model_error = ''
        self.bridge = CvBridge()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.marks_pub = self.create_publisher(MarkArray, '/detections/dock_marks', 10)
        self.marker_pub = self.create_publisher(PointStamped, '/dock/marker_base', 10)
        self.normal_pub = self.create_publisher(Vector3Stamped, '/dock/normal_base', 10)
        self.entrance_pub = self.create_publisher(PointStamped, '/dock/entrance_base', 10)
        self.status_pub = self.create_publisher(String, '/dock/detection_status', 10)
        self.debug_pub = (self.create_publisher(
            Image, '/dock/debug_image', qos_profile_sensor_data)
            if self.publish_debug else None)
        self.normal_marker_pub = (self.create_publisher(
            Marker, '/dock/normal_marker', 10)
            if self.publish_debug else None)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Bool, '/detector/enable', self._on_enable, latched)
        latest_image = QoSProfile(
            depth=1, reliability=qos_profile_sensor_data.reliability)
        image = Subscriber(self, Image, '/camera/color/image_raw',
                           qos_profile=latest_image)
        depth = Subscriber(self, Image, '/camera/depth/image_raw',
                           qos_profile=latest_image)
        info = Subscriber(self, CameraInfo, '/camera/camera_info',
                          qos_profile=latest_image)
        self.sync = ApproximateTimeSynchronizer(
            [image, depth, info], queue_size=2,
            slop=float(self.get_parameter('sync_slop_s').value))
        self.sync.registerCallback(self.on_frame)
        if not self.weights or not Path(self.weights).is_file():
            self.get_logger().error('Dock YOLO weights unset or missing; detector fails closed.')
        initial_enable = Bool()
        initial_enable.data = bool(self.get_parameter('enabled').value)
        self._on_enable(initial_enable)

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

    def _publish_debug(self, rgb, rgb_msg, detections, estimates, status,
                       intrinsics=None, normal_optical=None, normal_base=None):
        if self.debug_pub is None:
            return
        try:
            annotated = annotate_dock_frame(
                rgb, detections, estimates, self.target_class, status=status,
                intrinsics=intrinsics, normal_optical=normal_optical,
                normal_base=normal_base)
            message = self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
            message.header = rgb_msg.header
            self.debug_pub.publish(message)
        except (cv2.error, ValueError) as exc:
            self.get_logger().warning(f'Dock debug image failed: {exc}')

    def _publish_normal_marker(self, marker_base, normal_xyz):
        if self.normal_marker_pub is None:
            return
        arrow = Marker()
        arrow.header = marker_base.header
        arrow.ns = 'dock_normal'
        arrow.id = 0
        arrow.type = Marker.ARROW
        arrow.action = Marker.ADD
        arrow.points = [marker_base.point, Point(
            x=float(marker_base.point.x + 0.6 * normal_xyz[0]),
            y=float(marker_base.point.y + 0.6 * normal_xyz[1]),
            z=float(marker_base.point.z + 0.6 * normal_xyz[2]))]
        arrow.scale.x = 0.035
        arrow.scale.y = 0.075
        arrow.scale.z = 0.12
        arrow.color.r = 0.15
        arrow.color.g = 1.0
        arrow.color.b = 0.15
        arrow.color.a = 1.0
        arrow.lifetime = Duration(seconds=1.5).to_msg()
        self.normal_marker_pub.publish(arrow)

    def on_frame(self, rgb_msg: Image, depth_msg: Image, info_msg: CameraInfo):
        if not self.enabled:
            return
        if not self._frame_is_fresh(rgb_msg):
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
        clouds = {}
        detections_by_class = {}
        for detection in detections:
            try:
                estimate = extract_marker_cloud(
                    detection.mask, depth, fx, fy, cx, cy,
                    min_depth=float(self.get_parameter('min_depth_m').value),
                    max_depth=float(self.get_parameter('max_depth_m').value),
                    min_pixels=int(self.get_parameter('min_mask_pixels').value),
                    erosion_pixels=int(self.get_parameter('mask_erosion_pixels').value),
                    distortion=info_msg.d)
            except (np.linalg.LinAlgError, cv2.error, ValueError) as exc:
                self._status(False, 'marker_geometry_error', detail=str(exc))
                return
            if estimate is None or estimate.valid_fraction < float(
                    self.get_parameter('min_valid_fraction').value):
                continue
            clouds[detection.class_name] = estimate
            detections_by_class[detection.class_name] = detection
            mark = Mark()
            mark.color = 'unknown'
            mark.shape = detection.class_name
            mark.confidence = detection.confidence
            x, _, z = estimate.point_optical
            mark.bearing = math.atan2(-x, z)
            mark.distance = math.hypot(x, z)
            marks.marks.append(mark)
        self.marks_pub.publish(marks)
        try:
            wall = estimate_common_wall(
                clouds,
                inlier_distance=float(self.get_parameter('wall_inlier_distance_m').value),
                min_inlier_fraction=float(self.get_parameter('wall_min_inlier_fraction').value),
                max_plane_rms=float(self.get_parameter('max_plane_rms_m').value))
        except (np.linalg.LinAlgError, ValueError) as exc:
            self._status(False, 'wall_geometry_error', detail=str(exc))
            return
        if wall is None:
            self._publish_debug(rgb, rgb_msg, detections, estimates, 'wall plane unavailable')
            self._status(False, 'wall_plane_unavailable', available_markers=list(clouds))
            return
        for name in wall.marker_names:
            estimates[name] = (detections_by_class[name], marker_on_wall(clouds[name], wall))
        if self.target_class not in estimates:
            self._publish_debug(rgb, rgb_msg, detections, estimates,
                                'target missing / invalid depth')
            self._status(False, 'target_missing_or_invalid_depth_or_wall_outlier',
                         wall_markers=list(wall.marker_names))
            return
        detection, estimate = estimates[self.target_class]
        optical_frame = rgb_msg.header.frame_id
        if not optical_frame:
            self._publish_debug(rgb, rgb_msg, detections, estimates,
                                'missing camera frame')
            self._status(False, 'missing_optical_frame')
            return
        stamp = rgb_msg.header.stamp
        try:
            transform = self.tf_buffer.lookup_transform(
                self.base_frame, optical_frame, rclpy.time.Time.from_msg(stamp),
                timeout=Duration(seconds=0.1))
            marker_optical = self._point(stamp, optical_frame, estimate.point_optical)
            tip_optical = self._point(
                stamp, optical_frame,
                estimate.point_optical + estimate.outward_normal_optical)
            marker_base = do_transform_point(marker_optical, transform)
            tip_base = do_transform_point(tip_optical, transform)
        except TransformException as exc:
            self._publish_debug(rgb, rgb_msg, detections, estimates,
                                'camera TF unavailable')
            self._status(False, 'camera_to_base_tf_unavailable', detail=str(exc))
            return
        marker_xyz = np.array([marker_base.point.x, marker_base.point.y,
                               marker_base.point.z])
        normal_xyz = np.array([tip_base.point.x, tip_base.point.y,
                               tip_base.point.z]) - marker_xyz
        if not self._frame_is_fresh(rgb_msg):
            return
        normal_msg = Vector3Stamped()
        normal_msg.header.stamp = stamp
        normal_msg.header.frame_id = self.base_frame
        normal_msg.vector.x, normal_msg.vector.y, normal_msg.vector.z = map(float, normal_xyz)
        self.normal_pub.publish(normal_msg)
        self._publish_normal_marker(marker_base, normal_xyz)
        entrance_xyz = project_entrance_xy(
            marker_xyz, normal_xyz, self.wall_to_mouth)
        if entrance_xyz is None:
            self._publish_debug(
                rgb, rgb_msg, detections, estimates, 'normal valid / entrance invalid',
                (fx, fy, cx, cy), estimate.outward_normal_optical, normal_xyz)
            self._status(False, 'invalid_wall_normal_or_offset')
            return
        if not self._frame_is_fresh(rgb_msg):
            return
        self.marker_pub.publish(marker_base)
        self.entrance_pub.publish(self._point(stamp, self.base_frame, entrance_xyz))
        self._publish_debug(rgb, rgb_msg, detections, estimates,
                            f'wall normal ({len(wall.marker_names)} markers)',
                            (fx, fy, cx, cy), estimate.outward_normal_optical,
                            normal_xyz)
        self._status(True, 'ok', confidence=detection.confidence,
                     depth_m=estimate.median_depth_m,
                     valid_fraction=estimate.valid_fraction,
                     plane_rms_m=estimate.plane_rms_m,
                     wall_markers=list(wall.marker_names),
                     wall_inlier_fractions=wall.inlier_fractions,
                     plane_mode='single_marker' if len(wall.marker_names) == 1 else 'common_wall')

    def _frame_is_fresh(self, msg: Image):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        age = self.get_clock().now().nanoseconds * 1e-9 - stamp
        if stamp <= 0 or age < -0.1 or age > float(
                self.get_parameter('max_frame_age_s').value):
            self._status(False, 'stale_image', age_s=age)
            return False
        return True


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
