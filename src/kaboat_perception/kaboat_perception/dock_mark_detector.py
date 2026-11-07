"""Gated D455 YOLO26-seg detector and depth-based dock marker locator.

Inputs: aligned color/depth/camera_info and latched /detector/enable.
Outputs: /detections/dock_marks, /dock/marker_base (3D, base_link),
         /dock/marker_ground (filtered XY, odom), /dock/marker_ground_raw,
         /dock/debug_image,
         /dock/detection_status.
Only the marker position is estimated here. The dock axis / entrance comes
from the lidar back-wall fit in docking_ctrl, not from a depth plane normal.
3D depth markers require valid depth and TF; sight rays and arrows do not.
"""

import json
import math
import time
from pathlib import Path

import cv2
import numpy as np
if int(np.__version__.split('.')[0]) >= 2:
    raise RuntimeError('ROS Humble cv_bridge requires a NumPy 1.x runtime; '
                       'use a compatible ROS/YOLO environment')
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Point, PointStamped
from kaboat_msgs.msg import Mark, MarkArray
from message_filters import ApproximateTimeSynchronizer, Subscriber
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, ReliabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Bool, String
from tf2_geometry_msgs import do_transform_point
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from .depth_utils import depth_to_meters
from .dock_geometry import estimate_marker, StationaryMarkerFilter, mask_sight_ray
from .dock_segmentation import CLASS_NAMES, DockSegmenter


class DockMarkDetector(Node):
    def __init__(self):
        super().__init__('dock_mark_detector')
        for name, value in {
            'weights': '', 'target_class': 'triangle', 'base_frame': 'base_link',
            'confidence': 0.45, 'image_size': 640,
            'device': '0', 'half': True, 'min_depth_m': 0.25, 'max_depth_m': 12.0,
            'min_mask_pixels': 40, 'min_valid_fraction': 0.5,
            'sync_slop_s': 0.08, 'enabled': False, 'publish_debug_image': False,
            'debug_max_fps': 5.0, 'debug_max_width': 320, 'bearing_arrow_length_m': 1.0,
            'mask_erosion_pixels': 1, 'max_depth_sigma_m': 0.25,
            'min_depth_inlier_fraction': 0.7, 'depth_outlier_floor_m': 0.06, 'marker_filter_tau_s': 0.5,
            'marker_filter_gate_m': 0.35, 'marker_filter_reset_s': 1.5,
        }.items():
            self.declare_parameter(name, value)
        self.weights = str(self.get_parameter('weights').value)
        self.target_class = str(self.get_parameter('target_class').value)
        if self.target_class not in CLASS_NAMES:
            raise ValueError(f'target_class must be one of {CLASS_NAMES}')
        self.base_frame = str(self.get_parameter('base_frame').value)
        self.marker_filters = {name: StationaryMarkerFilter(
            tau_s=float(self.get_parameter('marker_filter_tau_s').value),
            gate_m=float(self.get_parameter('marker_filter_gate_m').value),
            reset_after_s=float(self.get_parameter('marker_filter_reset_s').value))
            for name in CLASS_NAMES}
        self.marker_filter = self.marker_filters[self.target_class]
        self.depth_diagnostics = {}
        self.enabled = False
        self.segmenter = None
        self.model_error = ''
        self.bridge = CvBridge()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.marks_pub = self.create_publisher(MarkArray, '/detections/dock_marks', 10)
        self.marker_pub = self.create_publisher(PointStamped, '/dock/marker_base', 10)
        self.ground_pub = self.create_publisher(
            PointStamped, '/dock/marker_ground', QoSProfile(
                depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.VOLATILE, history=HistoryPolicy.KEEP_LAST))
        self.raw_ground_pub = self.create_publisher(
            PointStamped, '/dock/marker_ground_raw', QoSProfile(
                depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.VOLATILE, history=HistoryPolicy.KEEP_LAST))
        marker_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                                durability=DurabilityPolicy.VOLATILE)
        self.class_ground_pubs = {name: self.create_publisher(
            PointStamped, f'/dock/markers/{name}/ground', marker_qos) for name in CLASS_NAMES}
        self.class_ray_pubs = {name: self.create_publisher(
            PointStamped, f'/dock/markers/{name}/ray', marker_qos) for name in CLASS_NAMES}
        self.class_base_pubs = {name: self.create_publisher(
            PointStamped, f'/dock/markers/{name}/base', marker_qos) for name in CLASS_NAMES}
        self.bearing_viz_pub = self.create_publisher(MarkerArray, '/dock/bearing_markers', marker_qos)
        self.bearing_arrow_length = float(self.get_parameter('bearing_arrow_length_m').value)
        if not math.isfinite(self.bearing_arrow_length) or self.bearing_arrow_length <= 0:
            raise ValueError('bearing_arrow_length_m must be positive')
        self.all_markers_pub = self.create_publisher(MarkerArray, '/dock/markers_ground', marker_qos)
        self.class_base_ground_pubs = {name: self.create_publisher(
            PointStamped, f'/dock/markers/{name}/ground_base', marker_qos) for name in CLASS_NAMES}
        self.base_markers_pub = self.create_publisher(MarkerArray, '/dock/markers_base', marker_qos)
        self.debug_frame = None
        self.debug_detections = []
        self.debug_max_fps = float(self.get_parameter('debug_max_fps').value)
        self.debug_max_width = int(self.get_parameter('debug_max_width').value)
        if self.debug_max_fps < 0 or self.debug_max_width < 0:
            raise ValueError('debug limits must be non-negative (0 means unlimited)')
        self.last_debug_time = -math.inf
        self.debug_count = 0
        self.input_counts = {'color': 0, 'depth': 0, 'camera_info': 0}
        self.input_info = {}
        self.status_pub = self.create_publisher(String, '/dock/detection_status', 10)
        self.debug_pub = (self.create_publisher(Image, '/dock/debug_image', QoSProfile(
                              depth=2, reliability=ReliabilityPolicy.BEST_EFFORT,
                              durability=DurabilityPolicy.VOLATILE, history=HistoryPolicy.KEEP_LAST))
                          if self.get_parameter('publish_debug_image').value else None)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Bool, '/detector/enable', self._on_enable, latched)
        latest_image = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE, history=HistoryPolicy.KEEP_LAST)
        image = Subscriber(self, Image, '/camera/color/image_raw',
                           qos_profile=latest_image)
        depth = Subscriber(self, Image, '/camera/depth/image_raw',
                           qos_profile=latest_image)
        info = Subscriber(self, CameraInfo, '/camera/camera_info',
                          qos_profile=latest_image)
        for stream, subscriber in (('color', image), ('depth', depth), ('camera_info', info)):
            subscriber.registerCallback(self._record_input, stream)
        self.sync = ApproximateTimeSynchronizer(
            [image, depth, info], queue_size=2,
            slop=float(self.get_parameter('sync_slop_s').value))
        self.sync.registerCallback(self.on_frame)
        if not self.weights or not Path(self.weights).is_file():
            self.get_logger().error('Dock YOLO weights unset or missing; detector fails closed.')

        initial = Bool()
        initial.data = bool(self.get_parameter('enabled').value)
        self._on_enable(initial)

    def _on_enable(self, msg: Bool):
        self.enabled = bool(msg.data)
        if not self.enabled:
            for marker_filter in self.marker_filters.values():
                marker_filter.reset()
        if self.enabled and self.segmenter is None and not self.model_error:
            try:
                if not self.weights or not Path(self.weights).is_file():
                    raise FileNotFoundError(self.weights or 'weights parameter unset')
                self.segmenter = DockSegmenter(
                    self.weights, float(self.get_parameter('confidence').value),
                    int(self.get_parameter('image_size').value),
                    str(self.get_parameter('device').value),
                    half=bool(self.get_parameter('half').value))
                self.get_logger().info(f'Dock YOLO ready: {self.weights}')
            except Exception as exc:
                self.model_error = str(exc)
                self.get_logger().error(f'Dock YOLO unavailable: {exc}')

    def _record_input(self, message, stream):
        self.input_counts[stream] += 1
        self.input_info[stream] = {
            'width': message.width, 'height': message.height,
            'frame': message.header.frame_id,
            'encoding': getattr(message, 'encoding', 'camera_info'),
        }

    def _status(self, valid: bool, reason: str, **extra):
        extra['processing_ms'] = round((time.perf_counter() - self.frame_started) * 1000, 1)
        extra['inference_ms'] = self.inference_ms
        self._publish_debug(reason)
        self._publish_bearing_markers()
        extra['input_counts'] = dict(self.input_counts)
        extra['input_info'] = self.input_info
        extra['debug_count'] = self.debug_count
        extra['depth_quality'] = self.depth_diagnostics
        extra['sight_rays'] = getattr(self, 'sight_diagnostics', {})
        msg = String()
        msg.data = json.dumps({'valid': valid, 'reason': reason,
                               'target_class': self.target_class,
                               'ground_valid': valid, **extra})
        self.status_pub.publish(msg)

    def _publish_bearing_markers(self):
        """Fixed-length optical-origin arrows; their length is not target range."""
        if self.debug_frame is None:
            return
        header = self.debug_frame[1]
        diagnostics = getattr(self, 'sight_diagnostics', {})
        colors = {'triangle': (1., .1, .1), 'circle': (.1, 1., .1), 'square': (.1, .3, 1.)}
        array = MarkerArray()
        for i, name in enumerate(CLASS_NAMES):
            ray = diagnostics.get(name, {}).get('optical_ray')
            tip = None
            if ray is not None:
                ray = np.asarray(ray, dtype=float)
                tip = ray / np.linalg.norm(ray) * self.bearing_arrow_length
            for text in (False, True):
                marker = Marker()
                marker.header = header
                marker.ns, marker.id = 'dock_bearings', i * 2 + int(text)
                marker.action = Marker.ADD if tip is not None else Marker.DELETE
                marker.type = Marker.TEXT_VIEW_FACING if text else Marker.ARROW
                marker.pose.orientation.w = 1.
                marker.color.r, marker.color.g, marker.color.b = ((1., 0., 1.)
                    if name == self.target_class else colors[name])
                marker.color.a = 1.
                marker.lifetime.nanosec = 500000000
                if tip is not None:
                    if text:
                        marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = map(float, tip)
                        marker.pose.position.y -= .12
                        marker.scale.z = .10
                        marker.text = name + ' bearing'
                    else:
                        marker.points = [Point(), Point(x=float(tip[0]), y=float(tip[1]), z=float(tip[2]))]
                        marker.scale.x, marker.scale.y, marker.scale.z = .025, .07, .10
                array.markers.append(marker)
        self.bearing_viz_pub.publish(array)

    def _publish_debug(self, status):
        if self.debug_pub is None or self.debug_frame is None:
            return
        now = time.monotonic()
        if self.debug_max_fps and now - self.last_debug_time < 1.0 / self.debug_max_fps:
            return
        rgb, header = self.debug_frame
        annotated = rgb.copy()
        for detection in self.debug_detections:
            selected = detection.class_name == self.target_class
            color = (255, 0, 255) if selected else (0, 255, 0)
            mask = detection.mask
            annotated[mask] = (0.65 * annotated[mask] + 0.35 * np.array(color)).astype(np.uint8)
            contours, _ = cv2.findContours(mask.astype(np.uint8),
                                          cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            cv2.drawContours(annotated, contours, -1, color, 2)
            ys, xs = np.nonzero(mask)
            if xs.size:
                x, y, right, bottom = int(xs.min()), int(ys.min()), int(xs.max()), int(ys.max())
                cv2.rectangle(annotated, (x, y), (right, bottom), color, 2)
                cv2.putText(annotated, f'{detection.class_name} {detection.confidence:.2f}',
                            (x, max(y - 5, 45)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
                if selected:
                    cv2.circle(annotated, (int(xs.mean()), int(ys.mean())), 5, color, -1)
        cv2.putText(annotated, f'{self.target_class}: {status}', (8, 22),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
        height, width = annotated.shape[:2]
        if self.debug_max_width and width > self.debug_max_width:
            annotated = cv2.resize(
                annotated, (self.debug_max_width, round(height * self.debug_max_width / width)),
                interpolation=cv2.INTER_AREA)
        debug = self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
        debug.header = header
        self.debug_pub.publish(debug)
        self.last_debug_time = time.monotonic()
        self.debug_count += 1

    @staticmethod
    def _point(stamp, frame, xyz):
        point = PointStamped()
        point.header.stamp = stamp
        point.header.frame_id = frame
        point.point.x, point.point.y, point.point.z = map(float, xyz)
        return point

    def on_frame(self, rgb_msg: Image, depth_msg: Image, info_msg: CameraInfo):
        self.frame_started = time.perf_counter()
        self.inference_ms = None
        self.depth_diagnostics = {}
        self.sight_diagnostics = {}
        self.debug_frame = None
        self.debug_detections = []
        try:
            rgb = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='bgr8')
            self.debug_frame = (rgb, rgb_msg.header)
        except (ValueError, cv2.error, RuntimeError) as exc:
            self._status(False, 'rgb_conversion_error', detail=str(exc))
            return
        if not self.enabled:
            self._status(False, 'disabled')
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
            raw_depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
            depth = depth_to_meters(raw_depth, depth_msg.encoding)
            inference_started = time.perf_counter()
            detections = self.segmenter.predict(rgb)
            self.inference_ms = (time.perf_counter() - inference_started) * 1000
        except (ValueError, cv2.error, RuntimeError) as exc:
            self._status(False, 'image_or_inference_error', detail=str(exc))
            return
        self.debug_detections = detections
        marks = MarkArray()
        marks.header = rgb_msg.header
        estimates = {}
        for detection in detections:
            # A sight ray is useful even when stereo depth is absent or rejected.
            ys, xs = np.nonzero(detection.mask)
            if len(xs) >= 8:
                ray, uv = mask_sight_ray(detection.mask, fx, fy, cx, cy, info_msg.d)
                self.class_ray_pubs[detection.class_name].publish(self._point(
                    rgb_msg.header.stamp, rgb_msg.header.frame_id, ray))
                self.sight_diagnostics[detection.class_name] = dict(
                    pixel_uv=uv.tolist(), optical_ray=ray.tolist(),
                    bearing_left_deg=math.degrees(math.atan2(-ray[0],ray[2])),
                    confidence=detection.confidence, depth_used=False)
            quality = {}
            self.depth_diagnostics[detection.class_name] = quality
            try:
                estimate = estimate_marker(
                    detection.mask, depth, fx, fy, cx, cy,
                    min_depth=float(self.get_parameter('min_depth_m').value),
                    max_depth=float(self.get_parameter('max_depth_m').value),
                    min_pixels=int(self.get_parameter('min_mask_pixels').value),
                    distortion=info_msg.d,
                    erosion_pixels=int(self.get_parameter('mask_erosion_pixels').value),
                    max_depth_sigma_m=float(self.get_parameter('max_depth_sigma_m').value),
                    min_inlier_fraction=float(self.get_parameter('min_depth_inlier_fraction').value),
                    outlier_floor_m=float(self.get_parameter('depth_outlier_floor_m').value),
                    diagnostics=quality)
            except (cv2.error, ValueError) as exc:
                self._status(False, 'marker_geometry_error', detail=str(exc))
                return
            if estimate is None or estimate.valid_fraction < float(
                    self.get_parameter('min_valid_fraction').value):
                continue
            estimates[detection.class_name] = (detection, estimate)
            mark = Mark()
            mark.color = 'unknown'
            mark.shape = detection.class_name
            mark.confidence = detection.confidence
            x, _, z = estimate.point_optical
            mark.bearing = math.atan2(-x, z)
            mark.distance = math.hypot(x, z)
            marks.marks.append(mark)
        self.marks_pub.publish(marks)
        results = self._project_estimates(estimates, rgb_msg.header)
        selected = results.get(self.target_class)
        if selected is None:
            self._status(False, 'target_missing_or_invalid_depth', markers=results)
        else:
            details = {k: v for k, v in selected.items() if k not in ('valid', 'reason')}
            self._status(selected['valid'], selected['reason'], markers=results, **details)

    def _project_estimates(self, estimates, header):
        """One measured-time TF pair; independent depth/filter state per marker."""
        results = {}
        if not estimates:
            self._publish_all_markers({}, header)
            self._publish_all_markers({}, header, frame=self.base_frame, publisher=self.base_markers_pub)
            return results
        if not header.frame_id:
            results = {name: dict(valid=False, reason='missing_optical_frame') for name in estimates}
            self._publish_all_markers({}, header)
            self._publish_all_markers({}, header, frame=self.base_frame, publisher=self.base_markers_pub)
            return results
        stamp = header.stamp
        measurement_time = rclpy.time.Time.from_msg(stamp)
        try:
            base_tf = self.tf_buffer.lookup_transform(
                self.base_frame, header.frame_id, measurement_time, timeout=Duration(seconds=0.1))
        except TransformException as exc:
            self._publish_all_markers({}, header)
            self._publish_all_markers({}, header, frame=self.base_frame, publisher=self.base_markers_pub)
            return {name: dict(valid=False, reason='camera_to_base_tf_unavailable', detail=str(exc))
                    for name in estimates}
        optical = {}
        base_points = {}
        for name, (detection, estimate) in estimates.items():
            optical[name] = self._point(stamp, header.frame_id, estimate.point_optical)
            base = do_transform_point(optical[name], base_tf)
            self.class_base_pubs[name].publish(base)
            ground_base = self._point(stamp, self.base_frame, [base.point.x, base.point.y, 0.0])
            self.class_base_ground_pubs[name].publish(ground_base)
            base_points[name] = ground_base
            if name == self.target_class:
                self.marker_pub.publish(base)
        self._publish_all_markers(base_points, header, frame=self.base_frame, publisher=self.base_markers_pub)
        try:
            odom_tf = self.tf_buffer.lookup_transform(
                'odom', header.frame_id, measurement_time, timeout=Duration(seconds=0.1))
        except TransformException as exc:
            self._publish_all_markers({}, header)
            return {name: dict(valid=False, reason='camera_to_odom_tf_unavailable', detail=str(exc))
                    for name in estimates}
        points = {}
        for name, (detection, estimate) in estimates.items():
            raw = do_transform_point(optical[name], odom_tf)
            raw.header.stamp, raw.header.frame_id, raw.point.z = stamp, 'odom', 0.0
            if name == self.target_class:
                self.raw_ground_pub.publish(raw)
            xy = [raw.point.x, raw.point.y]
            marker_filter = self.marker_filters[name]
            value, state = marker_filter.update(xy, stamp.sec + stamp.nanosec * 1e-9)
            results[name] = dict(
                valid=value is not None, reason='ok' if value is not None else 'marker_filter_' + state,
                filter_state=state, filter_epoch=marker_filter.epoch,
                measurement_stamp_s=stamp.sec + stamp.nanosec * 1e-9,
                raw_ground_xy=xy, depth_m=estimate.median_depth_m,
                depth_sigma_m=estimate.depth_sigma_m,
                valid_fraction=estimate.valid_fraction, confidence=detection.confidence)
            if value is not None:
                point = self._point(stamp, 'odom', [*value, 0.0])
                self.class_ground_pubs[name].publish(point)
                points[name] = point
                results[name]['filtered_ground_xy'] = value.tolist()
                if name == self.target_class:
                    self.ground_pub.publish(point)
        self._publish_all_markers(points, header)
        return results

    def _publish_all_markers(self, points, header, *, frame='odom', publisher=None):
        array = MarkerArray()
        colors = {'triangle': (1., .1, .1), 'circle': (.1, 1., .1), 'square': (.1, .3, 1.)}
        for i, name in enumerate(CLASS_NAMES):
            for text in (False, True):
                marker = Marker()
                marker.header.stamp, marker.header.frame_id = header.stamp, frame
                marker.ns, marker.id = 'dock_markers', 2 * i + int(text)
                marker.action = Marker.ADD if name in points else Marker.DELETE
                marker.type = Marker.TEXT_VIEW_FACING if text else Marker.SPHERE
                marker.pose.orientation.w = 1.0
                if name in points:
                    marker.pose.position.x = points[name].point.x
                    marker.pose.position.y = points[name].point.y
                    marker.pose.position.z = 0.25 if text else 0.0
                marker.scale.x = marker.scale.y = marker.scale.z = 0.18
                color = (1., 0., 1.) if name == self.target_class else colors[name]
                marker.color.r, marker.color.g, marker.color.b = color
                marker.color.a = 1.0
                marker.text = name
                marker.lifetime = Duration(seconds=0.7).to_msg()
                array.markers.append(marker)
        (self.all_markers_pub if publisher is None else publisher).publish(array)


def main(args=None):
    rclpy.init(args=args)
    node = DockMarkDetector()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
