import math

import numpy as np
import pytest

from kaboat_hardware.dock_one_shot import (
    LocalGeo, aligned_normals, fixed_route, parse_marker_order, retreat_error,
    retreat_route, symmetric_markers)


def test_gps_route_roundtrip_and_fixed_straight_segments():
    geo = LocalGeo(-33.724223, 150.679736)
    start = geo.xy(-33.724223, 150.679736)
    via = np.array([5.0, 0.0])
    stop = np.array([6.0, 0.0])
    latitude, longitude = geo.latlon(stop)
    np.testing.assert_allclose(geo.xy(latitude, longitude), stop, atol=1e-8)
    first, second, heading = fixed_route(start, via, stop)
    np.testing.assert_allclose(first[0], start)
    np.testing.assert_allclose(first[-1], second[0])
    np.testing.assert_allclose(second[-1], stop)
    assert len(first) == 51 and len(second) == 11
    assert heading == 0.0


def test_three_masks_and_independent_normals_required():
    status = {'input_info': {'color': {'width': 640}}, 'sight_rays': {
        'square': {'confidence': .9, 'pixel_uv': [150, 60]},
        'triangle': {'confidence': .9, 'pixel_uv': [320, 60]},
        'circle': {'confidence': .9, 'pixel_uv': [490, 60]},
    }}
    assert symmetric_markers(status)[0]
    status['sight_rays']['triangle']['pixel_uv'] = [400, 60]
    assert not symmetric_markers(status)[0]
    status['sight_rays']['triangle']['pixel_uv'] = [320, 60]
    status['direct_normals'] = {name: {'valid': True, 'normal_odom': [-1.0, 0.0],
                                      'wall_center_odom': [70.0, y]}
                                for name, y in [('circle', 61.05),
                                                ('triangle', 66.0),
                                                ('square', 70.95)]}
    assert aligned_normals(status, 0.0)[0]
    assert not aligned_normals(status, math.radians(20))[0]
    status['direct_normals']['square']['wall_center_odom'][0] = 67.6
    assert not aligned_normals(status, 0.0)[0]
    status['direct_normals']['square']['wall_center_odom'][0] = 70.0
    status['direct_normals']['square']['valid'] = False
    assert not aligned_normals(status, 0.0)[0]


def test_retreat_leg_keeps_bow_heading_and_exits_over_three_metres():
    stop = np.array([69.0, 66.0])
    samples, exit_xy = retreat_route(stop, 0.0)
    np.testing.assert_allclose(samples[0], stop)
    np.testing.assert_allclose(samples[-1], [65.75, 66.0])
    np.testing.assert_allclose(exit_xy, samples[-1])
    assert retreat_error(exit_xy, stop, 0.0) == (3.25, 0.0)
    assert retreat_error(np.array([66.0, 66.2]), stop, 0.0)[0] == 3.0


def _rays(**pixels):
    return {'input_info': {'color': {'width': 640}}, 'sight_rays': {
        name: {'confidence': .9, 'pixel_uv': [u, 60]} for name, u in pixels.items()}}


def test_marker_order_is_not_assumed_by_default():
    # 실제 대회 도크 배치를 모르므로 기본은 화면 위치로 가운데 마커를 정한다
    for left, middle, right in (('square', 'triangle', 'circle'),
                                ('circle', 'square', 'triangle'),
                                ('triangle', 'circle', 'square')):
        ok, why = symmetric_markers(_rays(**{left: 150, middle: 320, right: 490}))
        assert ok, why
    ok, why = symmetric_markers(_rays(square=150, triangle=420, circle=490))
    assert not ok and 'middle' in why                          # 가운데 마커가 화면 중앙이 아님
    ok, why = symmetric_markers(_rays(square=10, triangle=320, circle=490))   # 양쪽 중점 250 → 중앙에서 70 px > 64
    assert not ok and 'symmetric' in why


def test_explicit_marker_order_is_enforced():
    order = parse_marker_order('square, triangle, circle')
    assert order == ('square', 'triangle', 'circle')
    assert symmetric_markers(_rays(square=150, triangle=320, circle=490), order=order)[0]
    ok, why = symmetric_markers(_rays(circle=150, triangle=320, square=490), order=order)
    assert not ok and 'square-triangle-circle' in why
    assert parse_marker_order('') is None and parse_marker_order(None) is None
    for bad in ('square,triangle', 'square,triangle,triangle', 'square,triangle,star'):
        with pytest.raises(ValueError):
            parse_marker_order(bad)
