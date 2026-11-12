"""buoy_evidence 단독 테스트 — 부표별 log-odds (라이다 근거만)."""
import math

import numpy as np
import pytest

from kaboat_perception.buoy_evidence import (
    EvidenceParams, EvidenceTracker, LidarView, beam_evidence)

INC = math.radians(0.2)


def view(buoy_ranges=None, default=np.inf, x=0.0, y=0.0, yaw=0.0):
    """라이다 원점 (x, y), 선수 yaw. buoy_ranges = {방위[deg]: 거리} — 그 방향 ±1° 빔."""
    a = np.arange(-math.pi, math.pi, INC)
    r = np.full(len(a), default)
    for deg, rng in (buoy_ranges or {}).items():
        r[np.abs(np.degrees(a) - deg) <= 1.0] = rng
    return LidarView(x, y, yaw, -math.pi, INC, r, 0.4, 50.0)


P = EvidenceParams()


# ---------- 빔 근거 ----------

@pytest.mark.parametrize('ranges, expect', [
    ({0: 12.0}, 'free'),          # 부표(8 m) 자리를 지나 12 m 에서 맞음
    ({0: 4.0}, 'occluded'),       # 앞에서 맞음 — 가림
    ({0: 8.1}, 'consistent'),     # 부표 거리에서 맞음
    ({}, 'none'),                 # 반사 없음 — 검정 흡수일 수 있음
])
def test_beam_evidence(ranges, expect):
    assert beam_evidence(view(ranges), 8.0, 0.0, P) == expect


def test_beam_evidence_respects_boat_yaw():
    v = view({0: 12.0}, yaw=math.pi / 2)      # 선수 +y, 정면 빔 12 m
    assert beam_evidence(v, 0.0, 8.0, P) == 'free'
    assert beam_evidence(v, 8.0, 0.0, P) == 'none'   # 오른쪽 90° 빔은 반사 없음


# ---------- 확정 ----------

def test_confirm_after_three_hits():
    tk = EvidenceTracker(P)
    assert tk.update([(8.0, 0.0, None)], 0.0) == []
    assert tk.update([(8.0, 0.0, None)], 0.1) == []   # 1.6 < 2.0
    assert len(tk.update([(8.0, 0.0, None)], 0.2)) == 1


def test_single_noise_hit_removed_after_timeout():
    tk = EvidenceTracker(P)
    tk.update([(8.0, 0.0, None)], 0.0)
    tk.update([], 2.9)
    assert len(tk.tracks) == 1
    tk.update([], 3.2)
    assert tk.tracks == []


# ---------- 삭제 (보였어야 하는데 안 보일 때만) ----------

def confirmed_at(x=8.0):
    tk = EvidenceTracker(P)
    for k in range(6):
        tk.update([(x, 0.0, None)], 0.1 * k)          # L = 4.0 (상한)
    return tk


def run(tk, v, seconds, t0=0.6):
    for k in range(int(seconds * 10)):
        tk.update([], t0 + 0.1 * (k + 1), v)


def test_beam_passing_through_deletes_in_about_two_and_half_seconds():
    tk = confirmed_at()
    run(tk, view({0: 12.0}), 2.3)                    # 4 + 1 = 5 → 2.5 s 필요
    assert len(tk.confirmed()) == 1
    run(tk, view({0: 12.0}), 0.4, t0=2.9)
    assert tk.tracks == []


@pytest.mark.parametrize('ranges', [{0: 4.0}, {}, {0: 8.1}])
def test_occluded_no_return_or_consistent_never_deletes(ranges):
    tk = confirmed_at()
    run(tk, view(ranges), 60.0)
    assert len(tk.confirmed()) == 1


def test_no_scan_view_never_deletes():
    tk = confirmed_at()
    for k in range(600):
        tk.update([], 0.7 + 0.1 * k, None)
    assert len(tk.confirmed()) == 1


def test_reobserved_after_partial_free_recovers():
    tk = confirmed_at()
    run(tk, view({0: 12.0}), 1.5)                    # L 4 → 0.8 (첫 dt 0.2 + 0.1 × 14)
    tk.update([(8.0, 0.0, None)], 2.2, view({0: 8.0}))
    assert tk.confirmed()[0].L == pytest.approx(1.6, abs=0.05)


# ---------- 색 표 ----------

def test_colour_from_votes_none_is_not_a_vote():
    tk = EvidenceTracker(P)
    seq = ['green', None, None, None, None, None, None, None, 'green', None]
    for k, c in enumerate(seq):
        tk.update([(8.0, 0.0, c)], 0.1 * k)
    assert tk.confirmed()[0].color == 'green'        # 가려진 동안 표 없음 → 초록 유지


def test_unknown_votes_make_unknown():
    tk = EvidenceTracker(P)
    for k in range(5):
        tk.update([(8.0, 0.0, 'unknown')], 0.1 * k)
    assert tk.confirmed()[0].color == 'unknown'


# ---------- 병합 ----------

def test_close_confirmed_tracks_merge():
    tk = EvidenceTracker(EvidenceParams(gate_radius=0.3))
    for k in range(4):
        tk.update([(8.0, 0.0, 'red'), (8.5, 0.0, 'red')], 0.1 * k)
    assert len(tk.confirmed()) == 1


def test_never_seen_by_camera_is_unseen_not_unknown():
    tk = EvidenceTracker(P)
    for k in range(4):
        tk.update([(8.0, 0.0, None)], 0.1 * k)
    assert tk.confirmed()[0].color == 'unseen'


# ---------- 새 부표 제한·시간 점프 ----------

def test_detection_not_allowed_to_spawn_creates_nothing():
    tk = EvidenceTracker(P)
    for k in range(5):
        tk.update([(8.0, 0.0, None, False)], 0.1 * k)
    assert tk.tracks == []


def test_existing_track_updated_by_non_spawning_detection():
    tk = confirmed_at()
    run(tk, view({0: 12.0}), 1.5)                    # L 0.8
    tk.update([(8.0, 0.0, None, False)], 2.2)        # 시야 밖에서 다시 잡힘
    assert tk.confirmed()[0].L == pytest.approx(1.6, abs=0.05)


def test_time_jump_back_resets():
    tk = confirmed_at()
    tk.update([], 0.0)                               # bag 이 처음으로 돌아감 (0.5 → 0.0, 1 s 미만)
    assert len(tk.confirmed()) == 1
    tk.update([], 100.0)
    tk.update([], 5.0)                               # 95 s 거꾸로
    assert tk.tracks == []
