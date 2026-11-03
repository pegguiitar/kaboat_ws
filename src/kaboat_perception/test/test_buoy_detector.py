"""buoy_detector — HSV 색 검출 ①~⑤ 단계와 노드 파이프라인 (학습 없음).

색 원을 그린 합성 BGR 영상과 합성 depth 영상으로 확인한다.
⑥ 전역 투영과 ⑦ 시간 융합 자체는 test_buoy_tracker.py 가 확인한다.
"""
import math

import cv2
import numpy as np
import pytest
import rclpy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

from kaboat_perception.buoy_detector import (
    MIN_BLOB_AREA, BuoyDetector, bearing_from_pixel, filter_geometry,
    filter_waterline, hsv_blobs, lidar_confirms)

W, H = 640, 480
BGR = {'red': (0, 0, 255), 'green': (0, 255, 0), 'orange': (0, 128, 255)}


def _frame(blobs=(), background=(120, 80, 40)):
    """blobs: [(color, cx, cy, radius)] — 물색 배경에 색 원을 그린 영상."""
    img = np.full((H, W, 3), background, dtype=np.uint8)
    for color, cx, cy, r in blobs:
        cv2.circle(img, (cx, cy), r, BGR[color], -1)
    return img


# ── ①~⑤ 단계 함수 ──────────────────────────────────────

@pytest.mark.check('A')
def test_hsv_blobs_finds_each_color_at_its_centroid():
    img = _frame([('red', 100, 300, 25), ('green', 320, 300, 25), ('orange', 540, 300, 25)])
    found = {c[0]: (c[3], c[4]) for c in hsv_blobs(img)}
    assert set(found) == {'red', 'green', 'orange'}
    assert found['red'] == pytest.approx((100, 300), abs=1.0)
    assert found['green'] == pytest.approx((320, 300), abs=1.0)
    assert found['orange'] == pytest.approx((540, 300), abs=1.0)


@pytest.mark.check('A')
def test_geometry_filter_keeps_round_blob_and_drops_stripe():
    img = _frame([('red', 200, 300, 25)])
    cv2.rectangle(img, (350, 290), (630, 300), BGR['red'], -1)   # 가늘고 긴 반사 줄무늬
    kept = filter_geometry(hsv_blobs(img))
    assert len(kept) == 1
    assert kept[0][3] == pytest.approx(200, abs=1.0)


@pytest.mark.check('A')
def test_bearing_is_zero_at_center_and_positive_to_the_left():
    hfov = math.radians(87.0)
    assert bearing_from_pixel(W / 2, W, None, None, hfov) == pytest.approx(0.0)
    assert bearing_from_pixel(0.0, W, None, None, hfov) == pytest.approx(hfov / 2)
    assert bearing_from_pixel(W, W, None, None, hfov) == pytest.approx(-hfov / 2)


@pytest.mark.check('B')
def test_min_area_boundary_and_red_hue_wraparound():
    small = int(math.sqrt(MIN_BLOB_AREA / math.pi)) - 2
    assert hsv_blobs(_frame([('red', 320, 300, small)])) == []
    img = np.full((H, W, 3), (120, 80, 40), dtype=np.uint8)
    hsv_red_high = np.uint8([[[176, 255, 255]]])                  # H 176 — 빨강 두 번째 구간
    cv2.circle(img, (320, 300), 25, cv2.cvtColor(hsv_red_high, cv2.COLOR_HSV2BGR)[0, 0].tolist(), -1)
    assert [c[0] for c in hsv_blobs(img)] == ['red']


@pytest.mark.check('B')
def test_waterline_cut_is_inclusive_and_drops_only_above():
    cands = [('red', None, 500.0, 320.0, 143.0), ('red', None, 500.0, 320.0, 144.0)]
    assert [c[4] for c in filter_waterline(cands, 144.0)] == [144.0]


@pytest.mark.check('B')
def test_bearing_uses_camera_info_only_when_principal_point_is_sane():
    hfov = math.radians(87.0)
    good = bearing_from_pixel(220.0, W, 400.0, 320.0, hfov)
    assert good == pytest.approx(math.atan2(100.0, 400.0))
    # gz camera_info 처럼 주점이 중심에서 크게 벗어나면 HFOV 로 유도한다
    bad_k = bearing_from_pixel(220.0, W, 277.0, 160.0, hfov)
    assert bad_k == pytest.approx(bearing_from_pixel(220.0, W, None, None, hfov))


@pytest.mark.check('B')
def test_lidar_confirmation_window_and_range_tolerance():
    inc = math.radians(0.5)
    ranges = [math.inf] * 720
    idx = int(round((0.0 - (-math.pi)) / inc))
    ranges[idx + 2] = 5.0                       # 방위에서 2빔 옆 반사
    assert lidar_confirms(ranges, -math.pi, inc, 0.0, 5.0)
    assert lidar_confirms(ranges, -math.pi, inc, 0.0, 6.4)       # 1.5m 허용
    assert not lidar_confirms(ranges, -math.pi, inc, 0.0, 7.0)
    assert lidar_confirms(ranges, -math.pi, inc, 0.0, -1.0)      # 거리 미상이면 반사만 확인


@pytest.mark.check('C')
def test_lidar_without_return_rejects_and_missing_scan_passes():
    assert not lidar_confirms([math.inf] * 720, -math.pi, math.radians(0.5), 0.0, 5.0)
    assert lidar_confirms(None, 0.0, 0.0, 0.0, 5.0)


# ── 노드 파이프라인 ────────────────────────────────────

@pytest.fixture
def node():
    rclpy.init(args=['--ros-args', '-p', 'confirm_hits:=3'])
    n = BuoyDetector()
    n.published = []
    n._publish = lambda tracks: n.published.append(list(tracks))
    yield n
    n.destroy_node()
    rclpy.shutdown()


def _odom(t, x=0.0, y=0.0):
    msg = Odometry()
    msg.header.frame_id = 'odom'
    msg.header.stamp.sec = int(t)
    msg.pose.pose.position.x = x
    msg.pose.pose.position.y = y
    msg.pose.pose.orientation.w = 1.0
    return msg


def _feed(node, img, depth_m, frames=3, encoding='32FC1'):
    for k in range(frames):
        node._on_odom(_odom(100 + k))
        if depth_m is not None:
            node._on_depth(node.bridge.cv2_to_imgmsg(depth_m, encoding=encoding))
        node.on_image(node.bridge.cv2_to_imgmsg(img, encoding='bgr8'))


def _depth(value=5.0, shape=(H, W)):
    return np.full(shape, value, dtype=np.float32)


@pytest.mark.check('A')
def test_pipeline_confirms_buoy_at_projected_world_position(node):
    _feed(node, _frame([('green', W // 2, 300, 25)]), _depth(5.0))
    tracks = node.published[-1]
    assert len(tracks) == 1
    assert tracks[0].color == 'green'
    assert (tracks[0].x, tracks[0].y) == pytest.approx((5.0, 0.0), abs=0.05)


@pytest.mark.check('B')
def test_realsense_millimetre_depth_gives_same_distance(node):
    mm = np.full((H, W), 5000, dtype=np.uint16)
    _feed(node, _frame([('red', W // 2, 300, 25)]), mm, encoding='16UC1')
    assert node.published[-1][0].x == pytest.approx(5.0, abs=0.05)


@pytest.mark.check('C')
def test_no_odom_means_no_processing(node):
    node.on_image(node.bridge.cv2_to_imgmsg(_frame([('red', W // 2, 300, 25)]), encoding='bgr8'))
    assert node.published == []


@pytest.mark.check('C')
@pytest.mark.parametrize('depth', ['mismatched', 'zero'])
def test_unusable_depth_drops_detection(node, depth):
    d = _depth(5.0, shape=(240, 320)) if depth == 'mismatched' else _depth(0.0)
    _feed(node, _frame([('red', W // 2, 300, 25)]), d)
    assert node.published[-1] == []


@pytest.mark.check('C')
def test_sky_blob_and_lidar_ghost_are_rejected(node):
    _feed(node, _frame([('red', W // 2, 60, 25)]), _depth(5.0))      # 하늘 컷선 위
    assert node.published[-1] == []
    scan = LaserScan()
    scan.angle_min, scan.angle_increment = -math.pi, math.radians(0.5)
    scan.ranges = [math.inf] * 720                                      # 라이다 반사 없음
    node._on_scan(scan)
    _feed(node, _frame([('red', W // 2, 300, 25)]), _depth(5.0))
    assert node.published[-1] == []


@pytest.mark.check('D')
def test_published_message_is_odom_frame_buoy_array():
    rclpy.init()
    try:
        n = BuoyDetector()
        sent = []
        n.pub.publish = sent.append
        n._on_odom(_odom(100))
        from kaboat_perception.buoy_tracker import Track
        track = Track(id=7, x=1.0, y=2.0, hits=3, last_seen=100.0, confidence=0.5,
                      color_votes={'red': 3})
        n._publish([track])
        assert sent[0].header.frame_id == 'odom'
        assert (sent[0].buoys[0].id, sent[0].buoys[0].color) == (7, 'red')
        n.destroy_node()
    finally:
        rclpy.shutdown()
