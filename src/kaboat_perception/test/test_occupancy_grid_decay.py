"""라이다 log-odds 값(0.8 / -0.2 / 8.0, 2026-11-12): 장애물이 생기고 사라지는 스캔 수."""

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
    assert (L_OCC, L_FREE, L_MIN, L_MAX) == (0.8, -0.2, -4.5, 8.0)


def test_new_obstacle_needs_four_hits():
    assert scans_until(0.0, L_OCC, lambda v: published(v) >= DRI_THRESHOLD) == 4


def test_saturated_obstacle_survives_two_and_a_half_seconds_of_free_rays():
    assert scans_until(0.0, L_OCC, lambda v: v >= L_MAX - 1e-6) == 10   # 10 히트에 포화
    assert scans_until(L_MAX, L_FREE, lambda v: published(v) < DRI_THRESHOLD) == 26   # 10 Hz → 2.6 s


def test_cell_holds_when_hit_in_one_scan_out_of_five():
    # 유지되는 최소 명중률 = |L_FREE| / (L_OCC + |L_FREE|) = 20 %
    assert abs(L_FREE) / (L_OCC + abs(L_FREE)) == 0.2
    assert 1 * L_OCC + 4 * L_FREE >= 0.0                                  # 5 스캔 중 1 히트면 안 줄어듦
