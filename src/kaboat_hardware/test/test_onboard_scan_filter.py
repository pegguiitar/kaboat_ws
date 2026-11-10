"""Check that the boat-structure sector and tilted scans never corrupt mapping."""

import math
import unittest

from sensor_msgs.msg import LaserScan

from kaboat_hardware.onboard_scan_filter import (
    invalidate_all_ranges,
    is_tilted,
    mask_sector,
    quaternion_to_roll_pitch,
)


class TestOnboardScanFilter(unittest.TestCase):
    def test_120_to_240_in_published_scan_frame(self):
        raw = LaserScan()
        raw.header.frame_id = 'onboard_laser_frame'
        raw.angle_min = -math.pi
        raw.angle_increment = math.radians(1.0)
        raw.ranges = [2.0] * 361
        raw.intensities = [9.0] * 361

        filtered = mask_sector(raw, 120.0, 240.0)
        for bearing in (119, 120, 180, 240, 241):
            index = (bearing + 180) % 360
            if 120 <= bearing <= 240:
                self.assertTrue(math.isinf(filtered.ranges[index]))
                self.assertEqual(filtered.intensities[index], 0.0)
            else:
                self.assertEqual(filtered.ranges[index], 2.0)
                self.assertEqual(filtered.intensities[index], 9.0)

        self.assertEqual(raw.ranges[index], 2.0)
        self.assertEqual(raw.intensities[index], 9.0)
        self.assertEqual(filtered.header.frame_id, raw.header.frame_id)
        self.assertEqual(filtered.angle_min, raw.angle_min)
        self.assertEqual(filtered.angle_increment, raw.angle_increment)

    def test_negative_increment_and_empty_intensities(self):
        raw = LaserScan()
        raw.angle_min = math.pi
        raw.angle_increment = -math.radians(1.0)
        raw.ranges = [2.0] * 361

        filtered = mask_sector(raw, 120.0, 240.0)
        self.assertTrue(math.isinf(filtered.ranges[180 - 180]))  # 180 deg (rear) is masked
        self.assertEqual(filtered.ranges[180], 2.0)  # 0 deg remains valid
        self.assertEqual(len(filtered.intensities), 0)

    def test_quaternion_to_roll_pitch(self):
        # Identity quaternion -> 0 roll, 0 pitch
        roll, pitch = quaternion_to_roll_pitch(0.0, 0.0, 0.0, 1.0)
        self.assertAlmostEqual(roll, 0.0)
        self.assertAlmostEqual(pitch, 0.0)

        # 10 deg roll (around X axis)
        angle = math.radians(10.0)
        roll, pitch = quaternion_to_roll_pitch(math.sin(angle / 2), 0.0, 0.0, math.cos(angle / 2))
        self.assertAlmostEqual(math.degrees(roll), 10.0, places=4)
        self.assertAlmostEqual(math.degrees(pitch), 0.0, places=4)

        # 20 deg pitch (around Y axis)
        angle_p = math.radians(20.0)
        roll, pitch = quaternion_to_roll_pitch(0.0, math.sin(angle_p / 2), 0.0, math.cos(angle_p / 2))
        self.assertAlmostEqual(math.degrees(roll), 0.0, places=4)
        self.assertAlmostEqual(math.degrees(pitch), 20.0, places=4)

    def test_is_tilted_threshold_3_deg(self):
        thresh = math.radians(3.0)

        # 2.9 degrees -> not tilted
        self.assertFalse(is_tilted(math.radians(2.9), math.radians(0.0), thresh))
        self.assertFalse(is_tilted(math.radians(0.0), math.radians(-2.9), thresh))

        # 3.1 degrees -> tilted
        self.assertTrue(is_tilted(math.radians(3.1), math.radians(0.0), thresh))
        self.assertTrue(is_tilted(math.radians(0.0), math.radians(-3.1), thresh))
        self.assertTrue(is_tilted(math.radians(4.0), math.radians(4.0), thresh))

    def test_is_tilted_asymmetric_pitch_minus_4_to_plus_1(self):
        p_min = math.radians(-4.0)
        p_max = math.radians(1.0)
        r_min = math.radians(-3.0)
        r_max = math.radians(3.0)

        # Inside allowable pitch range (-2.2 deg like boat rest pose, -3.1 deg wave tilt)
        self.assertFalse(is_tilted(math.radians(0.0), math.radians(-2.2), p_min, p_max, r_min, r_max))
        self.assertFalse(is_tilted(math.radians(0.0), math.radians(-3.1), p_min, p_max, r_min, r_max))
        self.assertFalse(is_tilted(math.radians(0.0), math.radians(0.5), p_min, p_max, r_min, r_max))
        self.assertFalse(is_tilted(math.radians(-2.9), math.radians(-3.9), p_min, p_max, r_min, r_max))

        # Outside allowable pitch range (< -4.0 or > 1.0)
        self.assertTrue(is_tilted(math.radians(0.0), math.radians(-4.1), p_min, p_max, r_min, r_max))
        self.assertTrue(is_tilted(math.radians(0.0), math.radians(1.1), p_min, p_max, r_min, r_max))

        # Outside allowable roll range
        self.assertTrue(is_tilted(math.radians(3.1), math.radians(-1.0), p_min, p_max, r_min, r_max))
        self.assertTrue(is_tilted(math.radians(-3.1), math.radians(-1.0), p_min, p_max, r_min, r_max))

    def test_invalidate_all_ranges(self):
        raw = LaserScan()
        raw.ranges = [1.0, 2.0, 3.0]
        raw.intensities = [10.0, 20.0, 30.0]

        invalidated = invalidate_all_ranges(raw)
        self.assertTrue(all(math.isinf(r) for r in invalidated.ranges))
        self.assertTrue(all(i == 0.0 for i in invalidated.intensities))


if __name__ == '__main__':
    unittest.main()


# ── 노드: /scan 은 항상, /scan/level 은 수평일 때만 ─────────────────────
import pytest  # noqa: E402

rclpy = pytest.importorskip('rclpy')
from rclpy.parameter import Parameter  # noqa: E402
from sensor_msgs.msg import Imu  # noqa: E402

from kaboat_hardware.onboard_scan_filter import OnboardScanFilter  # noqa: E402


def _imu(pitch_deg=0.0, roll_deg=0.0):
    """roll·pitch 만 있는 IMU (yaw 0)."""
    msg = Imu()
    r, p = math.radians(roll_deg) / 2.0, math.radians(pitch_deg) / 2.0
    msg.orientation.w = math.cos(r) * math.cos(p)
    msg.orientation.x = math.sin(r) * math.cos(p)
    msg.orientation.y = math.cos(r) * math.sin(p)
    msg.orientation.z = -math.sin(r) * math.sin(p)
    return msg


def _scan():
    scan = LaserScan()
    scan.angle_min, scan.angle_increment = -math.pi, math.radians(90.0)
    scan.range_min, scan.range_max = 0.1, 50.0
    scan.ranges = [5.0, 5.0, 5.0, 5.0]
    return scan


@pytest.fixture
def scan_filter():
    rclpy.init()
    node = None

    def make(**params):
        nonlocal node
        node = OnboardScanFilter(parameter_overrides=[
            Parameter(k, value=v) for k, v in params.items()])
        node.sent = {'scan': [], 'level': []}
        node.publisher.publish = node.sent['scan'].append
        node.level_publisher.publish = node.sent['level'].append
        return node
    yield make
    if node is not None:
        node.destroy_node()
    rclpy.shutdown()


def test_scan_always_level_only_when_level(scan_filter):
    node = scan_filter()
    node._on_imu(_imu(pitch_deg=1.0))                     # 범위 안 (-4°~+2°)
    node._on_scan(_scan())
    assert (len(node.sent['scan']), len(node.sent['level'])) == (1, 1)
    for pitch in (3.0, -5.0):                             # 범위 밖 → /scan 만
        node._on_imu(_imu(pitch_deg=pitch))
        node._on_scan(_scan())
    node._on_imu(_imu(roll_deg=4.0))
    node._on_scan(_scan())
    assert (len(node.sent['scan']), len(node.sent['level'])) == (4, 1)
    node._on_imu(_imu())                                  # 다시 수평 → 둘 다
    node._on_scan(_scan())
    assert (len(node.sent['scan']), len(node.sent['level'])) == (5, 2)


def test_both_topics_get_the_sector_mask(scan_filter):
    node = scan_filter()
    node._on_imu(_imu())
    node._on_scan(_scan())                                # 각도 -180, -90, 0, 90
    for out in (node.sent['scan'][-1], node.sent['level'][-1]):
        assert math.isinf(out.ranges[0]) and out.ranges[2] == 5.0   # 선미(-180°) 가림


def test_invalidate_mode_sends_empty_level_scan(scan_filter):
    node = scan_filter(tilt_action='invalidate')
    node._on_imu(_imu(pitch_deg=5.0))
    node._on_scan(_scan())
    assert node.sent['scan'][-1].ranges[2] == 5.0         # /scan 은 그대로
    assert all(math.isinf(r) for r in node.sent['level'][-1].ranges)


def test_no_imu_or_stale_imu_passes_level(scan_filter):
    node = scan_filter()                                  # IMU 아직 없음 → 판정 불가, 통과
    node._on_scan(_scan())
    assert len(node.sent['level']) == 1
    node._on_imu(_imu(pitch_deg=10.0))
    node.latest_imu_time -= 5.0                           # IMU 지연 → 통과 (경고)
    node._on_scan(_scan())
    assert len(node.sent['level']) == 2


def test_filter_off_passes_level_even_when_tilted(scan_filter):
    node = scan_filter(enable_imu_filter=False)
    node._on_imu(_imu(pitch_deg=10.0))
    node._on_scan(_scan())
    assert (len(node.sent['scan']), len(node.sent['level'])) == (1, 1)
