"""ㄷ자 슬롯 검출 — 빗살 도크(슬롯 3개)를 라이다 빔으로 쏴서 만든 점(가림 포함)으로 검사.

도크 로컬: 뒷벽 y = 0, 물 쪽 +y, 슬롯 중심 x = -4, 0, 4 (간격 4.0 m), 폭 2.97, 깊이 1.94 (2026-11-12 실측).
"""
import math

import numpy as np
import pytest

from kaboat_behaviors.dock_u_slot import USlotParams, dock_orientation, find_u_slot

WIDTH, DEPTH, PITCH, FINGER = 2.97, 1.94, 4.0, 1.03
PARAMS = USlotParams(slot_width=WIDTH, depth=DEPTH)


def comb_segments():
    half = WIDTH / 2
    centres = [-PITCH, 0.0, PITCH]
    segs = [((c - half, 0.0), (c + half, 0.0)) for c in centres]          # 뒷벽
    fingers = [(centres[0] - half - FINGER, centres[0] - half)]
    fingers += [(a + half, b - half) for a, b in zip(centres, centres[1:])]
    fingers += [(centres[-1] + half, centres[-1] + half + FINGER)]
    for a, b in fingers:
        segs += [((a, 0.0), (a, DEPTH)), ((b, 0.0), (b, DEPTH)), ((a, DEPTH), (b, DEPTH))]
    return segs


def raycast(boat, segments, max_range=12.0, step_deg=.2, noise=.01, seed=0):
    """boat 에서 360° 빔 — 가장 가까운 면에 맞은 점 (가림 재현)."""
    rng = np.random.default_rng(seed)
    angles = np.radians(np.arange(-180, 180, step_deg))
    d = np.c_[np.cos(angles), np.sin(angles)]
    best = np.full(len(angles), np.inf)
    o = np.asarray(boat, float)
    for (ax, ay), (bx, by) in segments:
        e = np.array([bx - ax, by - ay])
        denom = d[:, 0] * e[1] - d[:, 1] * e[0]
        ok = np.abs(denom) > 1e-12
        rel = np.array([ax, ay]) - o
        t = np.where(ok, (rel[0] * e[1] - rel[1] * e[0]) / np.where(ok, denom, 1), np.inf)
        s = np.where(ok, (rel[0] * d[:, 1] - rel[1] * d[:, 0]) / np.where(ok, denom, 1), -1)
        hit = ok & (t > .05) & (s >= 0) & (s <= 1)
        best = np.where(hit & (t < best), t, best)
    keep = best < max_range
    r = best[keep] + rng.normal(0, noise, keep.sum())
    return o + d[keep] * r[:, None]


def to_odom(xy, yaw=math.radians(37.0), shift=(150.0, -30.0)):
    c, s = math.cos(yaw), math.sin(yaw)
    xy = np.asarray(xy, float)
    return np.c_[c * xy[..., 0] - s * xy[..., 1], s * xy[..., 0] + c * xy[..., 1]] + shift


def rotate(v, yaw=math.radians(37.0)):
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([c * v[0] - s * v[1], s * v[0] + c * v[1]])


def detect(boat_local, aim_local, points_local=None, params=PARAMS):
    boat = np.asarray(boat_local, float)
    pts = raycast(boat, comb_segments()) if points_local is None else points_local
    direction = rotate(np.asarray(aim_local, float) - boat)
    return find_u_slot(to_odom(pts), to_odom(boat[None])[0], to_odom(boat[None])[0],
                       direction, params)


def local_of(xy):
    c, s = math.cos(math.radians(-37.0)), math.sin(math.radians(-37.0))
    v = np.asarray(xy, float) - (150.0, -30.0)
    return np.array([c * v[0] - s * v[1], s * v[0] + c * v[1]])


def test_head_on_slot_centre_and_normal():
    # 방위선이 0.3 m 어긋나 있어도(YOLO 방위 오차) 중심은 양쪽 안쪽 면 가운데
    slot, why = detect((0.3, 6.5), (0.3, 0.0))
    assert slot is not None, why
    np.testing.assert_allclose(local_of(slot.center), [0.0, 0.0], atol=.05)
    outward = rotate(np.array(slot.outward), -math.radians(37.0))
    assert math.degrees(math.acos(np.clip(outward @ [0.0, 1.0], -1, 1))) < 1.0
    assert slot.wall_distance == pytest.approx(6.5, abs=.06)
    assert slot.coverage['interior'] == 0.0


def test_oblique_view_picks_the_slot_on_the_target_bearing():
    # 가운데 슬롯 앞(7 m)에서 왼쪽 슬롯(-4, 0) 쪽을 봄 — 비스듬히 30°, 한쪽 안쪽 면은 가려짐
    slot, why = detect((0.0, 7.0), (-4.0, 0.0))
    assert slot is not None, why
    np.testing.assert_allclose(local_of(slot.center), [-4.0, 0.0], atol=.06)
    slot, why = detect((0.0, 7.0), (4.0, 0.0))
    assert slot is not None, why
    np.testing.assert_allclose(local_of(slot.center), [4.0, 0.0], atol=.06)


def test_bearing_over_a_finger_finds_no_slot():
    # 방위선이 칸막이(x = 2) 위를 지나면 양쪽 슬롯 중심이 1.9 m 밖 → 고르지 않는다
    slot, why = detect((0.0, 7.0), (2.0, 0.0))
    assert slot is None, (slot, why)


def test_finger_tips_alone_are_not_a_back_wall():
    # 멀어서 칸막이 끝면만 보이는 경우 (2026-11-12 bag 의 RANSAC 오고정 원인) — ㄷ자가 아니다
    boat = np.array([0.2, 8.0])
    pts = raycast(boat, comb_segments())
    tips_only = pts[pts[:, 1] > DEPTH - .05]
    slot, why = detect(boat, (0.2, 0.0), tips_only)
    assert slot is None, (slot, why)


def test_sparse_back_wall_still_locks_on_the_back_wall_not_the_tips():
    # 뒷벽 점이 70 % 빠져도(멀고 선수가 숙여짐) 칸막이 끝 직선이 아니라 진짜 뒷벽
    boat = np.array([0.0, 6.0])
    pts = raycast(boat, comb_segments())
    rng = np.random.default_rng(1)
    back = np.abs(pts[:, 1]) < .05
    keep = ~back | (rng.random(len(pts)) < .3)
    slot, why = detect(boat, (0.0, 0.0), pts[keep])
    assert slot is not None, why
    np.testing.assert_allclose(local_of(slot.center), [0.0, 0.0], atol=.06)


def test_object_inside_the_slot_rejects_it():
    boat = np.array([0.0, 6.0])
    segs = comb_segments() + [((-.3, .8), (.3, .8)), ((-.3, .8), (-.3, 1.2)), ((.3, .8), (.3, 1.2))]
    slot, why = detect(boat, (0.0, 0.0), raycast(boat, segs))
    assert slot is None and 'interior' in why


def test_too_few_points_and_orientation_search():
    slot, why = detect((0.0, 6.0), (0.0, 0.0), np.zeros((5, 2)) + [0.0, 3.0])
    assert slot is None and 'few points' in why
    pts = raycast(np.array([0.0, 6.0]), comb_segments()) - [0.0, 6.0]
    forward = dock_orientation(pts[pts[:, 1] < 0], np.array([math.sin(.3), -math.cos(.3)]), 40.0)
    assert math.degrees(math.acos(np.clip(forward @ [0.0, -1.0], -1, 1))) < .5
