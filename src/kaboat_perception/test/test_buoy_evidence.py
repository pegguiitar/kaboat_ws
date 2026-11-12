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

def test_overlapping_confirmed_tracks_merge():
    tk = EvidenceTracker(EvidenceParams(gate_radius=0.1))
    for k in range(4):
        tk.update([(8.0, 0.0, 'red'), (8.4, 0.0, 'red')], 0.1 * k)   # 0.4 ≤ 0.2 + 0.2 + 0.1
    assert len(tk.confirmed()) == 1


def test_adjacent_pair_never_merges():
    tk = EvidenceTracker(EvidenceParams(gate_radius=0.1, spawn_min_sep=0.8))
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


# ---------- 원 맞추기 (반지름 고정 이동 → 안 맞으면 키움) ----------

def arc(center, r, viewer, half_deg=60.0, noise=0.01, rng=None):
    """viewer 에서 본 원의 앞면 호 점 (가장자리 half_deg 까지만 보임)."""
    rng = rng or np.random.default_rng(0)
    c = np.asarray(center, float)
    los = math.atan2(viewer[1] - c[1], viewer[0] - c[0])
    a = los + np.radians(np.linspace(-half_deg, half_deg, 15))
    pts = np.column_stack((c[0] + r * np.cos(a), c[1] + r * np.sin(a)))
    return pts + rng.normal(0, noise, pts.shape)


def run_arcs(true_r, viewers, center=(8.0, 0.0)):
    tk = EvidenceTracker(P)
    rng = np.random.default_rng(1)
    for k, vw in enumerate(viewers):
        pts = arc(center, true_r, vw, rng=rng)
        m = pts.mean(axis=0)
        tk.update([(m[0], m[1], None, True, pts)], 0.1 * k, view({}, x=vw[0], y=vw[1]))
    return tk.tracks[0]


PASS_BY = [(x, -6.0) for x in np.linspace(0.0, 16.0, 40)]   # 옆을 지나가며 여러 방향


def test_radius_grows_to_true_size_when_seen_from_many_directions():
    t = run_arcs(0.3, PASS_BY)
    assert t.grown and t.radius == pytest.approx(0.3, abs=0.04)
    assert math.hypot(t.x - 8.0, t.y) < 0.05


def test_radius_never_shrinks_below_prior():
    t = run_arcs(0.12, PASS_BY)
    assert not t.grown and t.radius == pytest.approx(P.radius_prior)
    assert math.hypot(t.x - 8.0, t.y) < 0.1


def test_one_direction_grows_only_to_chord_lower_bound():
    # 한 방향 ±60° 호 — 호 폭 2·0.3·sin60 = 0.52 → 반지름 하한 0.26. 원 맞추기로 더 키우지는 않는다
    t = run_arcs(0.3, [(2.0, 0.0)] * 40)
    assert t.radius == pytest.approx(0.26, abs=0.02)


def test_far_track_not_freed_beyond_max_range():
    tk = EvidenceTracker(EvidenceParams(max_range=10.0, forget_range=30.0))
    for k in range(6):
        tk.update([(12.0, 0.0, None)], 0.1 * k)
    run(tk, view({0: 20.0}), 10.0)                   # 12 m 부표 자리를 빔이 지나가도
    assert len(tk.confirmed()) == 1                  # clamp 밖이라 안 깎음


def test_surface_points_associate_to_fitted_centre():
    # 부표 중심 (8, 0) r 0.3 — 앞면 점 평균은 중심에서 ~0.25 m 앞. 중심끼리 재면 gate 0.2 밖이라
    # 새 부표가 됐지만, 원 둘레 기준이면 같은 부표.
    tk = EvidenceTracker(EvidenceParams(gate_radius=0.2, radius_prior=0.3))
    pts = arc((8.0, 0.0), 0.3, (2.0, 0.0), noise=0.0)
    for k in range(5):
        tk.update([(8.0, 0.0, None, True, pts)], 0.1 * k)
    assert len(tk.tracks) == 1


def test_centre_stays_behind_surface_even_if_points_curve_toward_lidar():
    # 번짐으로 점 무더기가 라이다(서쪽 (2, 0)) 쪽으로 오목 — 원 (7, 0) r 0.6 의 동쪽 호처럼 생김.
    # 그대로 맞추면 중심이 점보다 라이다 쪽에 놓인다. 중심은 점들보다 라이다에서 멀어야 한다.
    a = np.radians(np.linspace(-40, 40, 15))
    pts = np.column_stack((7.0 + 0.6 * np.cos(a), 0.6 * np.sin(a)))
    tk = EvidenceTracker(P)
    for k in range(20):
        tk.update([(7.6, 0.0, None, True, pts)], 0.1 * k, view({}, x=2.0, y=0.0))
    t = tk.tracks[0]
    assert t.x > pts[:, 0].mean()                      # 면 뒤


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


def test_smeared_blob_does_not_grow():
    # 스캔 번짐으로 0.4 m 두께 덩어리가 된 점 — 원호가 아니니 키우지 않는다
    rng = np.random.default_rng(3)
    tk = EvidenceTracker(P)
    for k, vw in enumerate(PASS_BY):
        pts = np.array([8.0, 0.0]) + rng.uniform(-0.2, 0.2, (15, 2))
        tk.update([(8.0, 0.0, None, True, pts)], 0.1 * k, view({}, x=vw[0], y=vw[1]))
    assert tk.tracks[0].radius <= 0.22                # 호 폭 하한(≈0.2) 이상으로 안 커짐


# ---------- 중복 생성 막기·호 폭 하한 ----------

def test_no_new_buoy_right_next_to_existing():
    tk = EvidenceTracker(EvidenceParams(gate_radius=0.1))
    for k in range(4):
        tk.update([(8.0, 0.0, None)], 0.1 * k)
    tk.update([(8.9, 0.0, None)], 0.5)                 # 0.9 m 옆 — 같은 부표의 다른 면
    assert len(tk.tracks) == 1


def test_adjacent_pair_at_1_2m_gets_two_buoys():
    tk = EvidenceTracker(EvidenceParams(gate_radius=0.1))
    for k in range(4):
        tk.update([(8.0, 0.0, 'green'), (9.3, 0.0, None)], 0.1 * k)   # 탐색 bag 쌍 중심 간격 ≥ 1.37 m
    assert len(tk.confirmed()) == 2


def test_radius_grows_to_chord_lower_bound():
    tk = EvidenceTracker(P)
    pts = np.column_stack((np.full(9, 8.0), np.linspace(-0.25, 0.25, 9)))   # 호 폭 0.5 m
    for k in range(12):
        tk.update([(8.0, 0.0, None, True, pts)], 0.1 * k, view({}, x=0.0, y=0.0))
    assert tk.tracks[0].radius == pytest.approx(0.25, abs=0.01)


def test_split_cluster_in_one_scan_makes_one_buoy():
    # 한 스캔에서 한 부표가 두 군집(0.5 m 간격)으로 쪼개져도 새 부표는 하나
    tk = EvidenceTracker(EvidenceParams(gate_radius=0.1))
    tk.update([(8.0, 0.0, None), (8.0, 0.5, None)], 0.0)
    assert len(tk.tracks) == 1


def test_confirmed_tracks_drifting_within_spawn_sep_merge():
    tk = EvidenceTracker(EvidenceParams(gate_radius=0.1, spawn_min_sep=0.8))
    for k in range(4):
        tk.update([(8.0, 0.0, None), (9.0, 0.0, None)], 0.1 * k)     # 1.0 m — 둘 다 생김
    assert len(tk.confirmed()) == 2
    tk.tracks[1].x = 8.7                                              # 0.7 m 로 다가옴
    tk.update([], 0.5)
    assert len(tk.confirmed()) == 1
