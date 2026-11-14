"""Remote visualization contract, including transform-before-projection."""
import json
from types import SimpleNamespace
from unittest.mock import Mock

import numpy as np
import pytest

pytest.importorskip('rclpy')
from cv_bridge import CvBridge
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import CameraInfo
from tf2_ros import TransformException
from kaboat_perception.dock_geometry import StationaryMarkerFilter
from kaboat_perception.dock_mark_detector import DockMarkDetector
from kaboat_perception.dock_segmentation import DockDetection


class Harness:
    on_frame = DockMarkDetector.on_frame
    _status = DockMarkDetector._status
    _publish_debug = DockMarkDetector._publish_debug
    _publish_bearing_markers = DockMarkDetector._publish_bearing_markers
    _point = staticmethod(DockMarkDetector._point)
    _project_estimates = DockMarkDetector._project_estimates
    _publish_all_markers = DockMarkDetector._publish_all_markers

    def __init__(self):
        self.marker_filter = StationaryMarkerFilter(warmup_samples=1)
        self.marker_filters = {name: StationaryMarkerFilter(warmup_samples=1)
                               for name in ('triangle', 'circle', 'square')}
        self.marker_filters['triangle'] = self.marker_filter
        self.class_ground_pubs = {name: Mock() for name in self.marker_filters}
        self.class_ray_pubs = {name: Mock() for name in self.marker_filters}
        self.class_base_pubs = {name: Mock() for name in self.marker_filters}
        self.all_markers_pub = Mock()
        self.base_markers_pub = Mock()
        self.bearing_viz_pub = Mock()
        self.bearing_arrow_length = 1.0
        self.class_base_ground_pubs = {name: Mock() for name in self.marker_filters}
        self.depth_diagnostics = {}
        self.frames = 0
        self.raw_ground_pub = Mock()
        self.debug_max_fps = 0
        self.debug_max_width = 0
        self.last_debug_time = -float('inf')
        self.debug_count = 0
        self.input_counts = {}
        self.input_info = {}
        self.enabled = True
        self.target_class = 'triangle'
        self.base_frame = 'base_link'
        self.bridge = CvBridge()
        self.model_error = ''
        self.marks_pub = Mock()
        self.marker_pub = Mock()
        self.ground_pub = Mock()
        self.status_pub = Mock()
        self.debug_pub = Mock()
        mask = np.zeros((80, 80), dtype=bool)
        mask[30:60, 30:60] = True
        self.segmenter = Mock()
        self.segmenter.predict.return_value = [DockDetection('triangle', 0.9, mask)]
        self.tf_buffer = Mock()
        self.tf_buffer.lookup_transform.side_effect = self.transform

    def get_parameter(self, name):
        return SimpleNamespace(value={'min_depth_m': 0.25, 'max_depth_m': 12.0,
                                     'min_mask_pixels': 40, 'min_valid_fraction': 0.5,
                                     'mask_erosion_pixels': 1, 'max_depth_sigma_m': 0.25,
                                     'min_depth_inlier_fraction': 0.7, 'depth_outlier_floor_m': 0.06}[name])

    @staticmethod
    def transform(target, source, stamp, **kwargs):
        tf = TransformStamped()
        tf.header.frame_id = target
        tf.child_frame_id = source
        # Rotate camera optical Z into world X, then translate.
        tf.transform.rotation.y = 2 ** -0.5
        tf.transform.rotation.w = 2 ** -0.5
        tf.transform.translation.x = 10.0
        tf.transform.translation.y = 20.0
        tf.transform.translation.z = 30.0
        return tf


def frame(node, depth_values=None):
    rgb = node.bridge.cv2_to_imgmsg(np.zeros((80, 80, 3), dtype=np.uint8), 'bgr8')
    rgb.header.frame_id = 'camera_color_optical_frame'
    rgb.header.stamp.sec = 123 + node.frames
    node.frames += 1
    rgb.header.stamp.nanosec = 456
    values = np.full((80, 80), 2000, dtype=np.uint16) if depth_values is None else depth_values
    depth = node.bridge.cv2_to_imgmsg(values, '16UC1')
    info = CameraInfo(width=80, height=80, k=[100., 0., 40., 0., 100., 40., 0., 0., 1.])
    node.on_frame(rgb, depth, info)
    return rgb


def test_measurement_transform_then_ground_projection():
    node = Harness()
    rgb = frame(node)
    point = node.ground_pub.publish.call_args.args[0]
    assert point.header.frame_id == 'odom'
    assert point.header.stamp == rgb.header.stamp
    assert point.point.x == pytest.approx(12.0)
    assert point.point.y == pytest.approx(20.09)
    assert point.point.z == 0.0
    call = node.tf_buffer.lookup_transform.call_args
    assert call.args[:2] == ('odom', rgb.header.frame_id)
    assert call.args[2].nanoseconds == 123000000456
    debug = node.debug_pub.publish.call_args.args[0]
    assert debug.encoding == 'bgr8' and debug.header == rgb.header
    assert json.loads(node.status_pub.publish.call_args.args[0].data)['ground_valid']


def test_no_detection_keeps_status_image_without_ground_point():
    node = Harness()
    node.segmenter.predict.return_value = []
    frame(node)
    node.ground_pub.publish.assert_not_called()
    debug = node.debug_pub.publish.call_args.args[0]
    assert np.any(node.bridge.imgmsg_to_cv2(debug))
    assert not json.loads(node.status_pub.publish.call_args.args[0].data)['ground_valid']


def test_missing_odom_tf_has_no_ground_point_but_keeps_overlay():
    node = Harness()
    original = node.transform
    def transform(target, *args, **kwargs):
        if target == 'odom':
            raise TransformException('missing odom')
        return original(target, *args, **kwargs)
    node.tf_buffer.lookup_transform.side_effect = transform
    frame(node)
    node.ground_pub.publish.assert_not_called()
    node.debug_pub.publish.assert_called_once()
    assert json.loads(node.status_pub.publish.call_args.args[0].data)['reason'] == 'camera_to_odom_tf_unavailable'


def test_debug_resize_throttle_preserves_measurement_and_inference():
    node = Harness()
    node.debug_max_width = 40
    node.debug_max_fps = 0.01
    rgb = frame(node)
    debug = node.debug_pub.publish.call_args.args[0]
    assert (debug.width, debug.height) == (40, 40)
    assert debug.header == rgb.header and debug.encoding == 'bgr8'
    frame(node)
    assert node.debug_pub.publish.call_count == 1
    assert node.segmenter.predict.call_count == 2
    assert node.ground_pub.publish.call_count == 2


def test_unreliable_depth_does_not_publish_or_update_temporal_filter():
    node = Harness()
    values = np.full((80, 80), 2000, dtype=np.uint16)
    values[:, 45:] = 5000
    frame(node, values)
    node.ground_pub.publish.assert_not_called()
    node.raw_ground_pub.publish.assert_not_called()
    assert node.marker_filter.value is None
    state = json.loads(node.status_pub.publish.call_args.args[0].data)
    assert state['depth_quality']['triangle']['reason'] == 'depth_spread_too_large'


def test_temporal_rejection_publishes_raw_only_and_marks_state_invalid():
    node = Harness()
    frame(node)
    original = node.transform
    def shifted(target, *args, **kwargs):
        tf = original(target, *args, **kwargs)
        if target == 'odom':
            tf.transform.translation.x += 2.0
        return tf
    node.tf_buffer.lookup_transform.side_effect = shifted
    frame(node)
    assert node.raw_ground_pub.publish.call_count == 2
    assert node.ground_pub.publish.call_count == 1
    state = json.loads(node.status_pub.publish.call_args.args[0].data)
    assert not state['ground_valid']
    assert state['filter_state'] == 'innovation_rejected'
    # Filtering never mutates the measurement retained by the raw publisher.
    assert node.raw_ground_pub.publish.call_args.args[0].point.x == pytest.approx(14.)
    assert node.ground_pub.publish.call_args.args[0].point.x == pytest.approx(12.)


def test_all_three_classes_have_independent_projection_and_filter_state():
    from kaboat_perception.dock_geometry import estimate_marker
    from std_msgs.msg import Header
    node = Harness()
    header = Header(frame_id='camera_color_optical_frame')
    header.stamp.sec = 123
    estimates = {}
    for i, name in enumerate(('triangle', 'circle', 'square')):
        mask = np.zeros((80, 80), dtype=bool)
        mask[10+i*15:30+i*15, 10+i*20:30+i*20] = True
        detection = DockDetection(name, .9, mask)
        estimate = estimate_marker(mask, np.full((80,80), 2.0), 100, 100, 40, 40)
        estimates[name] = (detection, estimate)
    results = node._project_estimates(estimates, header)
    assert all(results[name]['valid'] for name in estimates)
    ys = []
    for name in estimates:
        message = node.class_ground_pubs[name].publish.call_args.args[0]
        assert message.header.frame_id == 'odom' and message.point.z == 0.0
        assert message.header.stamp == header.stamp
        ys.append(message.point.y)
    assert len(set(ys)) == 3
    assert len({id(f) for f in node.marker_filters.values()}) == 3
    assert len(node.all_markers_pub.publish.call_args.args[0].markers) == 6
    node._project_estimates({'square': estimates['square']}, header)
    array = node.all_markers_pub.publish.call_args.args[0]
    assert all(m.action == m.DELETE for m in array.markers if m.id < 4)


def test_other_markers_publish_when_selected_triangle_is_missing():
    node = Harness()
    mask = np.ones((80, 80), dtype=bool)
    node.segmenter.predict.return_value = [DockDetection('circle', .9, mask)]
    frame(node)
    node.class_ground_pubs['circle'].publish.assert_called_once()
    node.ground_pub.publish.assert_not_called()
    state = json.loads(node.status_pub.publish.call_args.args[0].data)
    assert state['markers']['circle']['valid']
    assert not state['valid']


def test_boat_frame_projection_is_independent_of_odom_and_flattens_z():
    node = Harness()
    original = node.transform
    def transform(target, *args, **kwargs):
        if target == 'odom':
            raise TransformException('missing odom')
        return original(target, *args, **kwargs)
    node.tf_buffer.lookup_transform.side_effect = transform
    rgb = frame(node)
    point = node.class_base_ground_pubs['triangle'].publish.call_args.args[0]
    base = node.class_base_pubs['triangle'].publish.call_args.args[0]
    assert point.header.frame_id == 'base_link'
    assert point.header.stamp == rgb.header.stamp
    assert point.point.x == base.point.x and point.point.y == base.point.y
    assert point.point.z == 0.0
    array = node.base_markers_pub.publish.call_args.args[0]
    assert all(m.header.frame_id == 'base_link' for m in array.markers)
    assert any(m.action == m.ADD for m in array.markers)
    node.ground_pub.publish.assert_not_called()


def test_sight_ray_is_published_even_when_all_depth_is_missing():
    node = Harness()
    rgb = frame(node, np.zeros((80,80), dtype=np.uint16))
    msg = node.class_ray_pubs['triangle'].publish.call_args.args[0]
    assert msg.header == rgb.header
    assert msg.point.z == 1.0
    node.class_base_pubs['triangle'].publish.assert_not_called()
    node.ground_pub.publish.assert_not_called()


def test_bearing_arrow_starts_at_optical_origin_has_fixed_length_without_depth():
    node = Harness()
    rgb = frame(node, np.zeros((80,80), dtype=np.uint16))
    array = node.bearing_viz_pub.publish.call_args.args[0]
    arrow = next(m for m in array.markers if m.id==0)
    assert arrow.type==arrow.ARROW and arrow.action==arrow.ADD
    assert arrow.header==rgb.header
    assert [arrow.points[0].x,arrow.points[0].y,arrow.points[0].z]==[0.,0.,0.]
    tip=np.array([arrow.points[1].x,arrow.points[1].y,arrow.points[1].z])
    assert np.linalg.norm(tip)==pytest.approx(1.)
    ray=node.class_ray_pubs['triangle'].publish.call_args.args[0].point
    direction=np.array([ray.x,ray.y,ray.z]);direction/=np.linalg.norm(direction)
    assert tip==pytest.approx(direction)
    node.ground_pub.publish.assert_not_called()


def test_missing_detection_deletes_bearing_arrow_and_missing_tf_does_not_hide_it():
    node = Harness()
    node.tf_buffer.lookup_transform.side_effect=TransformException('no tf')
    frame(node)
    array=node.bearing_viz_pub.publish.call_args.args[0]
    assert array.markers[0].action==array.markers[0].ADD
    node.segmenter.predict.return_value=[]
    frame(node)
    array=node.bearing_viz_pub.publish.call_args.args[0]
    assert all(m.action==m.DELETE for m in array.markers)


def test_status_reports_image_time_and_half_fov():
    """도킹 제어용: 촬영 시각과 이 영상의 좌우 반화각 (cx 40, fx 100, 폭 80 → 각 21.8°)."""
    node = Harness()
    frame(node, np.zeros((80, 80), dtype=np.uint16))
    status = json.loads(node.status_pub.publish.call_args.args[0].data)
    assert status['image_half_fov_deg'] == {'left': 21.8, 'right': 21.8}
    assert 'image_stamp_s' in status


# ── 모델 미리 불러오기 (2026-11-14) ─────────────────────────────────────────────

def test_no_status_while_model_is_preloading():
    """불러오는 중엔 상태를 안 낸다 — dock_ctrl 이 'YOLO stale' 로 서서 기다리게 (탐색 회전 안 함)."""
    node = Harness()
    node.segmenter = None
    node._model_loading = True
    frame(node)
    node.status_pub.publish.assert_not_called()


def test_missing_model_after_loading_still_reports_unavailable():
    node = Harness()
    node.segmenter = None
    node._model_loading = False
    node.model_error = 'no weights'
    frame(node)
    assert json.loads(node.status_pub.publish.call_args.args[0].data)['reason'] == 'model_unavailable'


def test_detector_preloads_and_warms_up_at_startup(tmp_path, monkeypatch):
    """런치 때 백그라운드로 불러오고 빈 영상으로 2번 추론 — 켜질 때 다시 불러오지 않는다."""
    import threading
    import rclpy
    import kaboat_perception.dock_mark_detector as mod

    built, predicted = [], []

    class FakeSegmenter:
        def __init__(self, *args, **kwargs):
            built.append(args)

        def predict(self, bgr):
            predicted.append(bgr.shape)
            return []

    monkeypatch.setattr(mod, 'DockSegmenter', FakeSegmenter)
    weights = tmp_path / 'best.pt'
    weights.write_bytes(b'x')
    rclpy.init(args=['--ros-args', '-p', f'weights:={weights}'])
    try:
        node = mod.DockMarkDetector()
        for t in threading.enumerate():
            if t.name == 'dock_yolo_preload':
                t.join(5.0)
        assert node.segmenter is not None and not node._model_loading
        assert len(built) == 1 and predicted == [(480, 640, 3), (480, 640, 3)]
        node._on_enable(mod.Bool(data=True))
        assert len(built) == 1                          # 켜질 때 다시 안 불러옴
        node.destroy_node()
    finally:
        rclpy.shutdown()


def test_disabled_detector_publishes_nothing():
    """도킹이 아닐 때는 상태·디버그 영상·추론 모두 없음 (예전엔 'disabled' 상태를 매 프레임)."""
    node = Harness()
    node.enabled = False
    frame(node)
    node.status_pub.publish.assert_not_called()
    node.debug_pub.publish.assert_not_called()
    node.segmenter.predict.assert_not_called()
