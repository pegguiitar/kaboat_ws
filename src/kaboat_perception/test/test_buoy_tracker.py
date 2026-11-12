"""buoy_tracker.py 단독 테스트 — ROS 없이 돈다.

가짜 시계(now 직접 주입) + 합성 검출 스트림. 트래커를 순수 모듈로 분리한
이유가 이 테스트다 — 결합·확정·prune·색투표는 전이/타이머 로직이라
카메라 없이 여기서만 제대로 검증된다. 좌표는 월드(odom)프레임.
"""
import math

import pytest

from kaboat_perception.buoy_tracker import (
    BuoyTracker, TrackerParams, Detection, Track, project_to_world)


# ---------- project_to_world ----------

def test_project_forward_at_origin():
    x, y = project_to_world(0.0, 5.0, 0.0, 0.0, 0.0)
    assert x == pytest.approx(5.0) and y == pytest.approx(0.0)


def test_project_left_is_positive_y():
    # bearing +90° (좌측), 배 원점·yaw0 → 월드 +y
    x, y = project_to_world(math.pi / 2, 3.0, 0.0, 0.0, 0.0)
    assert x == pytest.approx(0.0, abs=1e-9) and y == pytest.approx(3.0)


def test_project_yaw_rotates_body():
    # yaw +90°(뱃머리 +y), 정면 2m → 월드 (0, 2)
    x, y = project_to_world(0.0, 2.0, 0.0, 0.0, math.pi / 2)
    assert x == pytest.approx(0.0, abs=1e-9) and y == pytest.approx(2.0)


def test_project_translates_by_boat_position():
    x, y = project_to_world(0.0, 2.0, 5.0, 3.0, 0.0)
    assert x == pytest.approx(7.0) and y == pytest.approx(3.0)


# ---------- 확정 (N-of-M) ----------

def det(color='red', x=10.0, y=0.0, conf=1.0):
    return Detection(color=color, x=x, y=y, confidence=conf)


def test_single_hit_not_confirmed():
    tr = BuoyTracker(TrackerParams(confirm_hits=3))
    out = tr.update([det()], now=0.0)
    assert out == []                 # 아직 승격 전
    assert len(tr.tracks) == 1       # tentative 트랙은 존재


def test_confirmed_after_n_hits():
    tk = BuoyTracker(TrackerParams(confirm_hits=3))
    tk.update([det()], now=0.0)
    tk.update([det()], now=0.1)
    out = tk.update([det()], now=0.2)
    assert len(out) == 1
    assert out[0].color == 'red'


def test_confirmed_only_returns_confirmed():
    tk = BuoyTracker(TrackerParams(confirm_hits=3))
    tk.update([det(x=10.0)], now=0.0)          # tentative A
    tk.update([det(x=10.0)], now=0.1)
    out = tk.update([det(x=10.0), det(x=30.0)], now=0.2)   # A 확정, B 신규
    assert len(out) == 1 and out[0].x == pytest.approx(10.0, abs=0.5)


# ---------- 데이터 결합 ----------

def test_nearby_detections_merge_one_track():
    tk = BuoyTracker(TrackerParams(gate_radius=1.5))
    for i in range(4):
        tk.update([det(x=10.0 + 0.2 * i, y=0.1 * i)], now=float(i))
    assert len(tk.tracks) == 1        # 살짝 흔들려도 한 부표


def test_distant_detections_are_separate_tracks():
    tk = BuoyTracker(TrackerParams(gate_radius=1.5))
    tk.update([det(x=10.0), det(x=20.0)], now=0.0)
    assert len(tk.tracks) == 2


def test_gate_radius_boundary_inside_merges():
    tk = BuoyTracker(TrackerParams(gate_radius=1.5))
    tk.update([det(x=10.0)], now=0.0)
    tk.update([det(x=11.4)], now=0.1)   # 1.4m < 1.5 → 같은 트랙
    assert len(tk.tracks) == 1


def test_gate_radius_boundary_outside_spawns():
    tk = BuoyTracker(TrackerParams(gate_radius=1.5))
    tk.update([det(x=10.0)], now=0.0)
    tk.update([det(x=11.6)], now=0.1)   # 1.6m > 1.5 → 새 트랙
    assert len(tk.tracks) == 2


def test_two_detections_same_spot_claim_distinct_tracks():
    # 한 프레임에 겹치는 두 검출이 같은 트랙을 동시에 먹으면 안 됨(1:1)
    tk = BuoyTracker(TrackerParams(gate_radius=1.5))
    tk.update([det(x=10.0)], now=0.0)
    tk.update([det(x=10.1), det(x=10.2)], now=0.1)
    assert len(tk.tracks) == 2


# ---------- prune ----------
# 확정 부표는 "안 보임"으로 안 지운다 — 보여야 하는 자리(시야·거리 안, 안 가림)에서
# drop_time 놓쳤을 때만. 미확정(노이즈)은 마지막 관측 후 drop_time.

AHEAD = (0.0, 0.0, 0.0)          # 배 원점, 선수 +x — x=4 부표는 정면 4 m (시야 안)
FACING_AWAY = (0.0, 0.0, math.pi)  # 같은 자리에서 뒤로 돎 — x=4 부표는 등 뒤


def test_unconfirmed_noise_pruned_after_drop_time():
    tk = BuoyTracker(TrackerParams(confirm_hits=3, drop_time=3.0))
    tk.update([det()], now=0.0)
    tk.update([], now=2.9)
    assert len(tk.tracks) == 1                  # 아직 살아있음
    tk.update([], now=3.2)
    assert tk.tracks == []                      # 3s 초과 → 삭제


def test_confirmed_kept_without_pose():
    tk = BuoyTracker(TrackerParams(confirm_hits=1, drop_time=3.0))
    tk.update([det(x=4.0)], now=0.0)
    for k in range(1, 101):
        tk.update([], now=0.1 * k)              # 10 s, pose 없음 → 안 쌓임
    assert len(tk.confirmed()) == 1


def test_confirmed_kept_while_out_of_view():
    tk = BuoyTracker(TrackerParams(confirm_hits=1, drop_time=3.0))
    tk.update([det(x=4.0)], now=0.0, boat_pose=AHEAD)
    for k in range(1, 101):
        tk.update([], now=0.1 * k, boat_pose=FACING_AWAY)   # 10 s 등 뒤
    assert len(tk.confirmed()) == 1
    assert tk.tracks[0].missed == 0.0


def test_confirmed_kept_beyond_view_range():
    tk = BuoyTracker(TrackerParams(confirm_hits=1, drop_time=3.0))
    tk.update([det(x=10.0)], now=0.0, boat_pose=AHEAD)      # 정면이지만 6 m 너머
    for k in range(1, 101):
        tk.update([], now=0.1 * k, boat_pose=AHEAD)
    assert len(tk.confirmed()) == 1


def test_confirmed_pruned_when_missed_in_view():
    tk = BuoyTracker(TrackerParams(confirm_hits=1, drop_time=3.0))
    tk.update([det(x=4.0)], now=0.0, boat_pose=AHEAD)
    for k in range(1, 29):
        tk.update([], now=0.1 * k, boat_pose=AHEAD)         # 2.8 s 놓침
    assert len(tk.confirmed()) == 1
    for k in range(29, 33):
        tk.update([], now=0.1 * k, boat_pose=AHEAD)         # 3.2 s 놓침
    assert tk.tracks == []


def test_missed_time_only_counts_in_view():
    # 시야 안 2 s → 등 뒤 5 s → 다시 시야 안 0.5 s: 놓친 시간은 2.5 s 라 남는다
    tk = BuoyTracker(TrackerParams(confirm_hits=1, drop_time=3.0))
    tk.update([det(x=4.0)], now=0.0, boat_pose=AHEAD)
    t = 0.0
    for pose, n in ((AHEAD, 20), (FACING_AWAY, 50), (AHEAD, 5)):
        for _ in range(n):
            t += 0.1
            tk.update([], now=t, boat_pose=pose)
    assert len(tk.confirmed()) == 1
    assert tk.tracks[0].missed == pytest.approx(2.5)


def test_reobservation_resets_missed():
    tk = BuoyTracker(TrackerParams(confirm_hits=1, drop_time=3.0))
    tk.update([det(x=4.0)], now=0.0, boat_pose=AHEAD)
    for k in range(1, 26):
        tk.update([], now=0.1 * k, boat_pose=AHEAD)         # 2.5 s 놓침
    tk.update([det(x=4.0)], now=2.6, boat_pose=AHEAD)       # 다시 봄
    for k in range(27, 50):
        tk.update([], now=0.1 * k, boat_pose=AHEAD)         # 2.3 s 놓침
    assert len(tk.confirmed()) == 1


def test_occluded_buoy_not_pruned():
    # 정면 3 m 부표 뒤 5 m 부표 — 뒤 것은 가려져 못 보는 게 당연하니 안 지운다
    tk = BuoyTracker(TrackerParams(confirm_hits=1, drop_time=3.0))
    tk.update([det(x=3.0), det(x=5.0)], now=0.0, boat_pose=AHEAD)
    for k in range(1, 51):
        tk.update([det(x=3.0)], now=0.1 * k, boat_pose=AHEAD)
    assert len(tk.confirmed()) == 2


def test_camera_gap_does_not_jump_missed_time():
    tk = BuoyTracker(TrackerParams(confirm_hits=1, drop_time=3.0))
    tk.update([det(x=4.0)], now=0.0, boat_pose=AHEAD)
    tk.update([], now=10.0, boat_pose=AHEAD)    # 카메라 10 s 끊겼다 복귀한 첫 프레임
    assert len(tk.confirmed()) == 1


# ---------- 색 투표 ----------

def test_color_majority_vote():
    tk = BuoyTracker(TrackerParams(confirm_hits=1, gate_radius=1.5))
    tk.update([det(color='red')], now=0.0)
    tk.update([det(color='green')], now=0.1)    # 물결 반짝임 1회
    tk.update([det(color='red')], now=0.2)
    out = tk.update([det(color='red')], now=0.3)
    assert out[0].color == 'red'


def test_track_color_defaults_unknown_when_empty():
    t = Track(id=0, x=0.0, y=0.0, hits=0, last_seen=0.0, confidence=0.0)
    assert t.color == 'unknown'


# ---------- id 안정성 ----------

def test_id_stable_across_frames():
    tk = BuoyTracker(TrackerParams(confirm_hits=1))
    tk.update([det()], now=0.0)
    first_id = tk.tracks[0].id
    tk.update([det(x=10.1)], now=0.1)
    assert tk.tracks[0].id == first_id


def test_new_tracks_get_incrementing_ids():
    tk = BuoyTracker(TrackerParams())
    tk.update([det(x=10.0), det(x=20.0)], now=0.0)
    ids = sorted(t.id for t in tk.tracks)
    assert ids == [0, 1]


def test_id_not_reused_after_prune():
    tk = BuoyTracker(TrackerParams(confirm_hits=3, drop_time=1.0))
    tk.update([det(x=10.0)], now=0.0)
    tk.update([], now=2.0)                       # 첫 트랙(id0, 미확정) prune
    tk.update([det(x=50.0)], now=2.1)            # 새 트랙
    assert tk.tracks[0].id == 1                  # id0 재사용 안 함


# ---------- EMA 위치 평활 ----------

def test_ema_smooths_position():
    tk = BuoyTracker(TrackerParams(confirm_hits=1, ema_alpha=0.3, gate_radius=5.0))
    tk.update([det(x=10.0)], now=0.0)
    tk.update([det(x=12.0)], now=0.1)
    # 10 + 0.3*(12-10) = 10.6 — 최신값(12)으로 확 튀지 않음
    assert tk.tracks[0].x == pytest.approx(10.6)


def test_ema_alpha_one_takes_latest():
    tk = BuoyTracker(TrackerParams(confirm_hits=1, ema_alpha=1.0, gate_radius=5.0))
    tk.update([det(x=10.0)], now=0.0)
    tk.update([det(x=12.0)], now=0.1)
    assert tk.tracks[0].x == pytest.approx(12.0)


def test_empty_update_returns_empty():
    tk = BuoyTracker(TrackerParams())
    assert tk.update([], now=0.0) == []
