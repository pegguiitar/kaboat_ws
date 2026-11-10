"""위도·경도 ↔ 원점 기준 x/y 변환 정확도."""

import math

import pytest

from kaboat_hardware.geo_local import (
    LocalFrame, average_llh, horizontal_std, parse_targets)

LAT0, LON0 = 35.1234567, 129.0456789


def test_origin_is_zero_and_axes_are_east_north():
    f = LocalFrame(LAT0, LON0)
    assert f.to_local(LAT0, LON0) == pytest.approx((0.0, 0.0), abs=1e-6)
    x, y = f.to_local(LAT0 + 0.0001, LON0)          # 북쪽 약 11.09 m
    assert abs(x) < 1e-3 and y == pytest.approx(11.09, abs=0.02)
    x, y = f.to_local(LAT0, LON0 + 0.0001)          # 동쪽 약 9.11 m (위도 35°)
    assert x == pytest.approx(9.11, abs=0.02) and abs(y) < 1e-3


def test_round_trip_within_a_millimetre():
    f = LocalFrame(LAT0, LON0, 10.0, axis_yaw_deg=33.0)
    for x, y in ((0.0, 0.0), (70.0, 20.0), (-35.5, 12.25), (150.0, -80.0)):
        lat, lon, _ = f.to_llh(x, y)
        assert f.to_local(lat, lon) == pytest.approx((x, y), abs=1e-3)


def test_axis_rotation_puts_the_field_direction_on_x():
    east = LocalFrame(LAT0, LON0)
    turned = LocalFrame(LAT0, LON0, axis_yaw_deg=90.0)   # x 축 = 북쪽
    lat, lon, _ = east.to_llh(0.0, 10.0)                 # 북쪽 10 m 지점
    assert turned.to_local(lat, lon) == pytest.approx((10.0, 0.0), abs=1e-3)
    assert turned.yaw_to_local(math.pi / 2) == pytest.approx(0.0)


def test_parse_targets_and_average():
    targets = parse_targets(['', 'start, 35.1, 129.0', 'corner,35.2,129.1'])
    assert [t.name for t in targets] == ['start', 'corner']
    for bad in (['nolatlon'], ['a,95,129'], ['a,35,129', 'a,35,129']):
        with pytest.raises(ValueError):
            parse_targets(bad)
    assert average_llh([(1.0, 2.0, 3.0), (3.0, 4.0, 5.0)]) == (2.0, 3.0, 4.0)


def test_horizontal_std():
    assert horizontal_std([4.0, 0, 0, 0, 1.0, 0, 0, 0, 9.0]) == pytest.approx(2.0)
    assert horizontal_std([0.0] * 9) is None
