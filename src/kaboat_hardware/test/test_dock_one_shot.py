import math

import numpy as np
import pytest

from kaboat_hardware.dock_one_shot import (
    LocalGeo, fixed_route, parse_marker_order, retreat_error, retreat_route,
    stable_bearing, stable_wall_lock, target_sight, wall_distance)


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


def test_retreat_leg_keeps_bow_heading_and_exits_over_three_metres():
    stop = np.array([69.0, 66.0])
    samples, exit_xy = retreat_route(stop, 0.0)
    np.testing.assert_allclose(samples[0], stop)
    np.testing.assert_allclose(samples[-1], [65.75, 66.0])
    np.testing.assert_allclose(exit_xy, samples[-1])
    assert retreat_error(exit_xy, stop, 0.0) == (3.25, 0.0)
    assert retreat_error(np.array([66.0, 66.2]), stop, 0.0)[0] == 3.0


def _rays(**markers):
    """도형 이름 → (픽셀 u, 화면 방위 deg[+왼쪽]) 또는 (u, 방위, 신뢰도)."""
    rays = {}
    for name, value in markers.items():
        u, bearing, *conf = value
        rays[name] = {'confidence': conf[0] if conf else .9, 'pixel_uv': [u, 60],
                      'bearing_left_deg': bearing}
    return {'input_info': {'color': {'width': 640}}, 'sight_rays': rays}


def test_target_sight_needs_only_the_target_marker():
    # 세 슬롯 중 어디로 갈지는 목표 도형의 화면 방위 하나로 정한다 — 나머지 둘은 안 보여도 된다
    ok, bearing, _ = target_sight(_rays(circle=(200, 12.0)), 'circle')
    assert ok and bearing == pytest.approx(math.radians(12.0))
    ok, bearing, why = target_sight(_rays(square=(200, 12.0)), 'circle')
    assert not ok and bearing is None and 'circle' in why
    ok, _, why = target_sight(_rays(circle=(200, 12.0, .4)), 'circle')   # 신뢰도 낮음
    assert not ok and 'confident' in why


def test_target_sight_marker_order_rejects_edge_false_positive():
    order = parse_marker_order('square,circle,triangle')
    good = _rays(square=(150, 20.0), circle=(320, 0.0), triangle=(490, -20.0))
    assert target_sight(good, 'circle', order=order)[0]
    # 2026-11-12 bag: 화면 오른쪽 끝에 엉뚱한 사각형 → 순서가 circle-triangle-square 로 어긋남
    bad = _rays(square=(630, -38.0), circle=(150, 20.0), triangle=(320, 0.0))
    ok, _, why = target_sight(bad, 'circle', order=order)
    assert not ok and 'square-circle-triangle' in why
    # 두 개만 보여도 그 둘의 순서만 본다
    assert target_sight(_rays(circle=(320, 0.0), triangle=(490, -20.0)), 'circle', order=order)[0]
    # 순서를 안 주면 보지 않는다
    assert target_sight(bad, 'circle')[0]


def test_stable_bearing_needs_consistent_samples():
    base = math.radians(-96.0)
    ok, mean, _ = stable_bearing([base + math.radians(d) for d in (-1, 0, 1, .5, -.5)])
    assert ok and mean == pytest.approx(base, abs=1e-9)
    ok, mean, why = stable_bearing([base] * 4)
    assert not ok and mean is None and '4/5' in why
    ok, _, why = stable_bearing([base + math.radians(d) for d in (-4, 0, 4, 0, 0)])
    assert not ok and 'unstable' in why
    # ±180° 경계를 넘어도 원형 평균
    ok, mean, _ = stable_bearing([math.pi - .01, -math.pi + .01] * 3)
    assert ok and abs(abs(mean) - math.pi) < .01             # 0° 쪽으로 평균되지 않음


def _wall(stamp, wall=(0.0, 0.0), normal_deg=90.0, jitter=(0.0, 0.0)):
    n = np.array([math.cos(math.radians(normal_deg)), math.sin(math.radians(normal_deg))])
    w = np.asarray(wall, float) + jitter
    return (stamp, w, n, w + 3.0 * n, w + 1.0 * n)


def test_stable_wall_lock_fixes_geometry_parallel_to_normal():
    samples = [_wall(10.0 + .1 * k, jitter=(.02 * (-1) ** k, 0.0)) for k in range(5)]
    ok, lock, why = stable_wall_lock(samples)
    assert ok, why
    np.testing.assert_allclose(lock['outward'], [0.0, 1.0], atol=1e-9)
    np.testing.assert_allclose(lock['stop'], [0.004, 1.0], atol=1e-9)
    np.testing.assert_allclose(lock['via'], [0.004, 3.0], atol=1e-9)
    assert lock['heading'] == pytest.approx(-math.pi / 2)      # 법선과 평행하게 벽 쪽으로
    assert wall_distance([0.0, 5.0], lock['wall'], lock['outward']) == pytest.approx(5.0)


def test_stable_wall_lock_rejects_few_old_or_jumping_walls():
    assert not stable_wall_lock([_wall(10.0 + .1 * k) for k in range(4)])[0]
    # 첫 샘플이 오래됨 — 창(1.5 s) 밖이라 빼고 세면 4 개
    old = [_wall(5.0)] + [_wall(10.0 + .1 * k) for k in range(4)]
    ok, _, why = stable_wall_lock(old)
    assert not ok and '4/5' in why
    # 10 Hz 로 끊김 없이 들어와도 min_span 을 채우면 고정 (마지막 5 개만 보면 0.4 s 라 영영 안 됨)
    steady = [_wall(10.0 + .1 * k) for k in range(12)]
    assert not stable_wall_lock(steady[:8], window_s=2.5, min_span_s=1.0)[0]
    assert stable_wall_lock(steady, window_s=2.5, min_span_s=1.0)[0]
    # 한 스캔이 슬롯 옆면(법선 90° 다름)을 잡음
    side = [_wall(10.0 + .1 * k) for k in range(4)] + [_wall(10.4, normal_deg=0.0)]
    assert not stable_wall_lock(side)[0]
    # 정지점이 0.5 m 튐
    jump = [_wall(10.0 + .1 * k) for k in range(4)] + [_wall(10.4, wall=(.5, 0.0))]
    ok, _, why = stable_wall_lock(jump)
    assert not ok and 'stop point' in why


def test_marker_order_parsing():
    order = parse_marker_order('square, triangle, circle')
    assert order == ('square', 'triangle', 'circle')
    assert parse_marker_order('') is None and parse_marker_order(None) is None
    for bad in ('square,triangle', 'square,triangle,triangle', 'square,triangle,star'):
        with pytest.raises(ValueError):
            parse_marker_order(bad)
