import math

import numpy as np

from kaboat_hardware.dock_one_shot import (
    LocalGeo, aligned_normals, fixed_route, retreat_error, retreat_route,
    symmetric_markers)


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
