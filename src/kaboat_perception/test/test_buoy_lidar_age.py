"""buoy_detector ④ 라이다 중심: 오래된 /scan/level 은 안 쓰고 depth 로만 위치를 잡는다."""

import math
from types import SimpleNamespace

import pytest

pytest.importorskip('rclpy')
from rclpy.time import Time  # noqa: E402
from sensor_msgs.msg import LaserScan  # noqa: E402

from kaboat_perception.buoy_detector import BuoyDetector  # noqa: E402


def _scan_with_hit_ahead(distance=5.0):
    """1° 간격 360빔, 정면(0 rad)에만 반사."""
    scan = LaserScan()
    scan.angle_min, scan.angle_increment = -math.pi, math.radians(1.0)
    scan.range_min, scan.range_max = 0.1, 50.0
    scan.ranges = [float('inf')] * 360
    scan.ranges[180] = distance
    return scan


def _detector(scan, age_sec, max_age=0.3):
    now = Time(nanoseconds=int(100e9))
    return SimpleNamespace(
        scan=scan,
        scan_at=None if scan is None else Time(nanoseconds=int((100.0 - age_sec) * 1e9)),
        lidar_max_age=max_age,
        get_clock=lambda: SimpleNamespace(now=lambda: now))


def test_fresh_scan_is_used():
    scan = _scan_with_hit_ahead()
    assert BuoyDetector._fresh_scan(_detector(scan, age_sec=0.05)) is scan


def test_stale_or_missing_scan_is_skipped():
    stale = _detector(_scan_with_hit_ahead(), age_sec=1.0)  # 배가 기울어 /scan/level 끊김
    assert BuoyDetector._fresh_scan(stale) is None
    assert BuoyDetector._fresh_scan(_detector(None, 0.0)) is None
