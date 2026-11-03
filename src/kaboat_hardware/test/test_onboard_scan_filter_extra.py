"""선체 라이다 스캔 필터 — 실패 시 안전(C) 보강: 빈 스캔도 깨지지 않고 그대로 통과."""
import math

import pytest
from sensor_msgs.msg import LaserScan

from kaboat_hardware.onboard_scan_filter import invalidate_all_ranges, mask_sector


@pytest.mark.check('C')
def test_empty_scan_is_handled_without_error():
    raw = LaserScan()
    raw.header.frame_id = 'onboard_laser_frame'
    raw.angle_min = -math.pi
    raw.angle_increment = math.radians(1.0)
    masked = mask_sector(raw, 120.0, 240.0)
    cleared = invalidate_all_ranges(raw)
    assert list(masked.ranges) == [] and list(cleared.ranges) == []
    assert masked.header.frame_id == cleared.header.frame_id == 'onboard_laser_frame'
