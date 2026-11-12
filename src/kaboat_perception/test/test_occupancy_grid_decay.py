"""라이다 log-odds 값(0.7 / -0.3 / 8.0, 2026-11-12): 장애물이 생기고 사라지는 스캔 수."""

import math

from kaboat_perception.occupancy_grid import L_FREE, L_MAX, L_MIN, L_OCC

DRI_THRESHOLD = 95      # dri.DriParams.occ_threshold — 발행값(0~100 반올림) 이 이상이면 장애물


def published(logodds):
    return round(100.0 / (1.0 + math.exp(-logodds)))


def scans_until(start, step, done):
    value, n = start, 0
    while not done(value):
        value = min(L_MAX, max(L_MIN, value + step))
        n += 1
    return n


def test_values():
    assert (L_OCC, L_FREE, L_MIN, L_MAX) == (0.7, -0.3, -4.5, 8.0)


def test_new_obstacle_needs_five_hits():
    # 선수 V 자 물보라(2026-11-12)가 4 히트에 장애물이 되던 것을 5 히트로
    assert scans_until(0.0, L_OCC, lambda v: published(v) >= DRI_THRESHOLD) == 5
    # free 로 여러 번 확인된 칸(L_MIN)은 11 히트
    assert scans_until(L_MIN, L_OCC, lambda v: published(v) >= DRI_THRESHOLD) == 11


def test_saturated_obstacle_survives_under_two_seconds_of_free_rays():
    assert scans_until(0.0, L_OCC, lambda v: v >= L_MAX - 1e-6) == 12   # 12 히트에 포화
    assert scans_until(L_MAX, L_FREE, lambda v: published(v) < DRI_THRESHOLD) == 18   # 10 Hz → 1.8 s


def test_cell_grows_only_when_hit_in_more_than_thirty_percent_of_scans():
    # 유지되는 최소 명중률 = |L_FREE| / (L_OCC + |L_FREE|) = 30 %
    assert math.isclose(abs(L_FREE) / (L_OCC + abs(L_FREE)), 0.3)
    assert 1 * L_OCC + 2 * L_FREE > 0.0       # 3 스캔 중 1 히트(33 %)면 쌓임
    assert 1 * L_OCC + 3 * L_FREE < 0.0       # 4 스캔 중 1 히트(25 %)면 줄어듦 — 가끔 튀는 물보라
