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


def EP(**kw):
    """확정 히트·나이 조건을 끈 파라미터 — L 규칙만 보는 테스트용 (조건 자체는 아래 따로 테스트)."""
    return EvidenceParams(**{'confirm_min_hits': 1, 'confirm_min_age': 0.0, **kw})


P = EP()


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


def test_short_burst_of_hits_is_not_confirmed():
    # 배 옆 물보라 — 3 연속 히트(0.2 s)로 L 은 2.4 지만 히트 4 개·0.5 s 가 안 돼 확정 안 됨
    tk = EvidenceTracker(EvidenceParams())
    for k in range(3):
        assert tk.update([(3.0, 0.0, None)], 0.1 * k) == []
    for k in range(3, 6):
        tk.update([], 0.1 * k)
    assert tk.confirmed() == []


def test_steady_hits_confirm_after_half_second():
    tk = EvidenceTracker(EvidenceParams())
    got = [len(tk.update([(8.0, 0.0, None)], 0.1 * k)) for k in range(7)]
    assert got == [0, 0, 0, 0, 0, 1, 1]               # 히트 4 는 0.3 s — 0.5 s 지나서 확정


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

def test_overlapping_confirmed_tracks_merge():
    tk = EvidenceTracker(EP(gate_radius=0.1))
    for k in range(4):
        tk.update([(8.0, 0.0, 'red'), (8.4, 0.0, 'red')], 0.1 * k)   # 0.4 ≤ 0.2 + 0.2 + 0.1
    assert len(tk.confirmed()) == 1


def test_adjacent_pair_never_merges():
    tk = EvidenceTracker(EP(gate_radius=0.1, spawn_min_sep=0.8))
    for k in range(4):
        tk.update([(8.0, 0.0, 'green'), (8.9, 0.0, None)], 0.1 * k)   # 0.9 m — 원이 안 겹침
    assert len(tk.confirmed()) == 2


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


# ---------- 위치·크기 (덩어리) ----------

def blob(center, half, n=9, along=(0.0, 1.0)):
    """중심 center, 반폭 half 의 직선 덩어리 점 (along 방향)."""
    c, u = np.asarray(center, float), np.asarray(along, float)
    return c + np.outer(np.linspace(-half, half, n), u)


def test_position_is_median_of_blob_centres():
    # 스캔마다 0.15 m 흔들리고, 가끔 한 스캔이 크게 튀어도 중심은 거의 그대로
    rng = np.random.default_rng(2)
    tk = EvidenceTracker(P)
    for k in range(30):
        c = np.array([8.0, 0.0]) + rng.normal(0, 0.15, 2)
        if k % 10 == 5:
            c = c + (0.6, 0.0)                           # 튄 스캔
        tk.update([(c[0], c[1], None, True, blob(c, 0.2))], 0.1 * k)
    t = tk.tracks[0]
    assert math.hypot(t.x - 8.0, t.y) < 0.1


def test_size_is_upper_percentile_of_blob_half_widths():
    tk = EvidenceTracker(P)
    for k in range(20):
        half = 0.25 if k % 4 else 0.1                    # 가끔 일부만 맞은 좁은 덩어리
        tk.update([(8.0, 0.0, None, True, blob((8.0, 0.0), half))], 0.1 * k)
    assert tk.tracks[0].radius == pytest.approx(0.25, abs=0.01)


def test_size_capped_and_floored():
    big, small = EvidenceTracker(P), EvidenceTracker(P)
    for k in range(5):
        big.update([(8.0, 0.0, None, True, blob((8.0, 0.0), 1.2))], 0.1 * k)
        small.update([(8.0, 0.0, None, True, blob((8.0, 0.0), 0.02, n=3))], 0.1 * k)
    assert big.tracks[0].radius == pytest.approx(P.radius_max)
    assert small.tracks[0].radius == pytest.approx(P.radius_min)


def test_non_round_blob_is_still_a_buoy():
    # L 자·삼각 덩어리(캡슐·옆에서 본 원뿔) — 원이 아니어도 하나의 부표, 크기는 덩어리 폭
    pts = np.array([[8.0, -0.3], [8.0, -0.1], [8.0, 0.1], [8.2, 0.1], [8.4, 0.1]])
    tk = EvidenceTracker(P)
    for k in range(4):
        tk.update([(8.1, 0.0, None, True, pts)], 0.1 * k)
    assert len(tk.confirmed()) == 1
    assert tk.confirmed()[0].radius == pytest.approx(0.5 * math.hypot(0.4, 0.4), abs=0.01)


def test_wide_buoy_blob_associates_by_its_size():
    # 반폭 0.5 m 덩어리가 한 스캔에서 0.8 m 옆(한쪽 끝만 맞음)으로 와도 같은 부표 (반폭 + gate)
    tk = EvidenceTracker(P)
    for k in range(5):
        tk.update([(8.0, 0.0, None, True, blob((8.0, 0.0), 0.5))], 0.1 * k)
    tk.update([(8.0, 0.8, None, True, blob((8.0, 0.8), 0.15))], 0.6)
    assert len(tk.tracks) == 1 and tk.tracks[0].hits == 6


def test_far_track_not_freed_beyond_max_range():
    tk = EvidenceTracker(EP(max_range=10.0, forget_range=30.0))
    for k in range(6):
        tk.update([(12.0, 0.0, None)], 0.1 * k)
    run(tk, view({0: 20.0}), 10.0)                   # 12 m 부표 자리를 빔이 지나가도
    assert len(tk.confirmed()) == 1                  # clamp 밖이라 안 깎음


# ---------- 색: 픽셀 수 누적 ----------

def px(red, green, visible, usable=True, color=None):
    from types import SimpleNamespace
    return SimpleNamespace(red=red, green=green, visible=visible, usable=usable, color=color)


def test_pixel_accumulation_separates_adjacent_pair():
    # 초록 공: 창이 공 위 4 프레임 (초록 30~58 %). 검정 캡슐: 옆 공이 2 프레임 섞이고(초록 33 %)
    # 나머지 6 프레임은 초록 0 — 표 한 장씩이면 둘 다 초록이 될 수 있지만 픽셀 비율로 갈린다.
    green, black = EvidenceTracker(P), EvidenceTracker(P)
    for k, f in enumerate([0.58, 0.46, 0.38, 0.30]):
        green.update([(8.0, 0.0, px(0, int(1000 * f), 1000))], 0.1 * k)
    for k, f in enumerate([0.33, 0.33, 0, 0, 0, 0, 0, 0]):
        black.update([(8.0, 0.0, px(0, int(1000 * f), 1000))], 0.1 * k)
    assert green.confirmed()[0].color == 'green'
    assert black.confirmed()[0].color == 'unknown'


def test_unusable_frames_do_not_count():
    tk = EvidenceTracker(P)
    for k in range(4):
        tk.update([(8.0, 0.0, px(0, 900, 1000, usable=False))], 0.1 * k)
    assert tk.confirmed()[0].px_visible == 0
    assert tk.confirmed()[0].color == 'unseen'


def test_red_needs_dominance_over_green():
    tk = EvidenceTracker(P)
    for k in range(4):
        tk.update([(8.0, 0.0, px(300, 200, 1000))], 0.1 * k)
    assert tk.confirmed()[0].color == 'unknown'           # 섞였으면 판정하지 않음


def test_buoy_forgotten_when_boat_far_away():
    tk = confirmed_at(8.0)
    tk.update([], 1.0, view({}, x=-10.0, y=0.0))     # 배(라이다)가 18 m 떨어짐 (> 17)
    assert tk.tracks == []


# ---------- 중복 생성 막기 ----------

def test_no_new_buoy_right_next_to_existing():
    tk = EvidenceTracker(EP(gate_radius=0.1))
    for k in range(4):
        tk.update([(8.0, 0.0, None)], 0.1 * k)
    tk.update([(8.9, 0.0, None)], 0.5)                 # 0.9 m 옆 — 같은 부표의 다른 면
    assert len(tk.tracks) == 1


def test_adjacent_pair_at_1_2m_gets_two_buoys():
    tk = EvidenceTracker(EP(gate_radius=0.1))
    for k in range(4):
        tk.update([(8.0, 0.0, 'green'), (9.3, 0.0, None)], 0.1 * k)   # 탐색 bag 쌍 중심 간격 ≥ 1.37 m
    assert len(tk.confirmed()) == 2


def test_split_cluster_in_one_scan_makes_one_buoy():
    # 한 스캔에서 한 부표가 두 군집(0.5 m 간격)으로 쪼개져도 새 부표는 하나
    tk = EvidenceTracker(EP(gate_radius=0.1))
    tk.update([(8.0, 0.0, None), (8.0, 0.5, None)], 0.0)
    assert len(tk.tracks) == 1


def test_confirmed_tracks_drifting_within_spawn_sep_merge():
    tk = EvidenceTracker(EP(gate_radius=0.1, spawn_min_sep=0.8))
    for k in range(4):
        tk.update([(8.0, 0.0, None), (9.0, 0.0, None)], 0.1 * k)     # 1.0 m — 둘 다 생김
    assert len(tk.confirmed()) == 2
    tk.tracks[1].x = 8.7                                              # 0.7 m 로 다가옴
    tk.update([], 0.5)
    assert len(tk.confirmed()) == 1


def test_color_frac_from_pixels_and_votes():
    from kaboat_perception.buoy_evidence import EvTrack
    t = EvTrack(id=1, x=0.0, y=0.0, L=3.0, last_seen=0.0, px_red=10, px_green=340, px_visible=1000,
                px_params=(0.15, 2.0, 150))
    assert t.color == 'green' and t.color_frac == pytest.approx(0.34)
    t = EvTrack(id=2, x=0.0, y=0.0, L=3.0, last_seen=0.0, px_red=20, px_green=50, px_visible=1000,
                px_params=(0.15, 2.0, 150))
    assert t.color == 'unknown' and t.color_frac == pytest.approx(0.05)   # 빨강·초록 중 큰 쪽
    t = EvTrack(id=3, x=0.0, y=0.0, L=3.0, last_seen=0.0, color_votes={'red': 3, 'unknown': 1})
    assert t.color == 'red' and t.color_frac == pytest.approx(0.75)       # 픽셀 적으면 표 비율
    t = EvTrack(id=4, x=0.0, y=0.0, L=3.0, last_seen=0.0)
    assert t.color == 'unseen' and t.color_frac == 0.0
