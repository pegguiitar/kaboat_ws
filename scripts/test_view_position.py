"""view_position.py 표시 계산 (ROS 없이)."""

import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

import view_position as vp  # noqa: E402


def _q(yaw_deg):
    h = math.radians(yaw_deg) / 2.0
    return (0.0, 0.0, math.sin(h), math.cos(h))


def test_heading_and_compass():
    assert vp.heading_deg(_q(0.0)) == 0.0 or abs(vp.heading_deg(_q(0.0))) < 1e-9
    assert abs(vp.heading_deg(_q(119.5)) - 119.5) < 1e-9
    assert abs(vp.compass_deg(119.5) - 330.5) < 1e-9      # 현장에서 본 값
    assert abs(vp.compass_deg(-60.8) - 150.8) < 1e-9
    assert vp.compass_deg(90.0) == 0.0                     # 북쪽
    assert vp.compass_deg(0.0) == 90.0                     # 동쪽


def test_line_shows_values_gnss_and_stale_warning():
    line = vp.format_line(-0.34, -0.24, -54.6, 0.12, 0.05, 'RTK Fixed')
    assert '-0.34' in line and '-0.24' in line and '144.6' in line and 'RTK Fixed' in line
    assert vp.GREEN in line and '끊김' not in line
    assert 'GNSS' not in vp.format_line(0, 0, 0, 0, 0.0)  # 상태 메시지 없는 환경
    assert '2.5초 끊김' in vp.format_line(0, 0, 0, 0, 2.5)
    assert vp.gnss_color('SBAS') == vp.RED and vp.gnss_color('RTK Float') == vp.YELLOW
