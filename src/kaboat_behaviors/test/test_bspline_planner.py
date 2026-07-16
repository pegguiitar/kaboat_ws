"""bspline_planner.py 단독 테스트 (AVOIDANCE_PLAN.MD §2 작업 2) — ROS 없이 돈다.

DriGrid 는 실제 build_dri 로 만든다 (합성 FakeGrid → 진짜 위험장) — 가짜
위험장을 손으로 그리면 "위험장 모양에 대한 가정"이 테스트에 숨어들기 때문.
장면 좌표계: 배 (0,0), heading +x, 격자는 배 중심 20×20m (test_dri.make_grid).
"""
import math

import numpy as np
import pytest

from kaboat_behaviors.dri import DriParams, build_dri
from kaboat_behaviors.bspline_planner import (
    Plan, PlannerParams, check, generate, repair)
from test_dri import make_grid, RES

BOAT = (0.0, 0.0)
YAW = 0.0
V_FULL = (1.48, 0.0)   # §6 실측 전속 (max_linear=0.20 → 1.48 m/s)
V_ZERO = (0.0, 0.0)
WP_FAR = (15.0, 0.0)


def dri_of(obstacles, boat_xy=BOAT, yaw=YAW):
    grid, _ = make_grid(obstacles, boat_xy=boat_xy)
    return build_dri(grid, boat_xy, yaw, DriParams())


def gen(dri, vel=V_FULL, wp=WP_FAR, boat=BOAT, yaw=YAW, **kw):
    return generate(dri, boat, yaw, vel, wp, PlannerParams(**kw))


def p1_len(plan):
    return float(np.hypot(*(plan.cps[1] - plan.cps[0])))


# ---------- (i) radii 단조증가 — p1 포함 ----------

def test_radii_monotone_including_p1():
    """§4-5: p2~p5 만 보면 t_look 류 버그를 못 잡는다 — ‖p1−p0‖ 부터 검사."""
    plan = gen(dri_of([(4.0, 0.2)]))
    seq = [p1_len(plan)] + list(plan.radii)
    assert all(a < b for a, b in zip(seq, seq[1:])), seq


def test_radii_monotone_in_r7_fallback():
    plan = gen(dri_of([]))
    seq = [p1_len(plan)] + list(plan.radii)
    assert all(a < b for a, b in zip(seq, seq[1:])), seq


# ---------- (ii) 정지 상태 p1 퇴화 방지 ----------

def test_stationary_p1_uses_heading_and_min_dist():
    """v≈0 이면 p1 = p0 + p1_min_dist·heading — 스플라인 접선이 살아있어야 한다."""
    plan = gen(dri_of([(4.0, 0.0)]), vel=V_ZERO)
    p = PlannerParams()
    assert np.allclose(plan.cps[1], [p.p1_min_dist, 0.0], atol=1e-9)
    assert np.all(np.isfinite(plan.samples))


def test_p1_direction_is_heading_not_velocity():
    """p1 방향은 v̂ 가 아니라 heading (§7-7) — 순수 측면 표류는 관성이 아니다."""
    plan = gen(dri_of([]), vel=(0.0, 0.9))    # 옆으로만 미끄러지는 중
    d1 = plan.cps[1] - plan.cps[0]
    assert d1[0] > 0 and abs(d1[1]) < 1e-9    # heading(+x) 방향
    p = PlannerParams()
    assert np.hypot(*d1) == pytest.approx(p.p1_min_dist)  # surge=0 → 하한


def test_p1_length_uses_surge_component_only():
    """사선 활주: 길이는 ‖v‖ 가 아니라 heading 성분(surge)만 (§7-7)."""
    plan = gen(dri_of([]), vel=(1.2, 0.9))    # ‖v‖=1.5, surge=1.2
    d1 = plan.cps[1] - plan.cps[0]
    assert abs(d1[1]) < 1e-9
    assert np.hypot(*d1) == pytest.approx(1.2 * PlannerParams().t_look)


def test_p1_reverse_motion_collapses_to_min_dist():
    """후진(ESCAPE) 중엔 surge<0 → p1 이 하한으로 수축, 방향은 여전히 전방
    (R9 자기교정 — 구 규칙은 p1 이 배 뒤로 갈 수 있었다)."""
    plan = gen(dri_of([]), vel=(-0.8, 0.0))
    d1 = plan.cps[1] - plan.cps[0]
    assert d1[0] > 0
    assert np.hypot(*d1) == pytest.approx(PlannerParams().p1_min_dist)


# ---------- (iii) 단순 장면 — 생성 경로가 check 무위반 ----------

def test_simple_scene_generates_clean_path():
    dri = dri_of([(4.0, 0.2)])
    plan = gen(dri)
    assert plan is not None
    assert check(plan, dri, BOAT, PlannerParams()) is None


# ---------- (iv) 위반 검출 → repair → 재검사 통과 ----------

def test_check_detects_and_repair_fixes():
    """빈 격자로 직선 plan 을 만들고, 경로 위에 장애물이 '나타난' DRI 로 검사."""
    p = PlannerParams()
    plan = gen(dri_of([]))                       # R7 직선 (waypoint 방향)
    dri_new = dri_of([(6.0, 0.0)])               # 경로 한가운데 출현
    idx = check(plan, dri_new, BOAT, p)
    assert idx is not None
    # 노드는 tick 당 repair_max 회씩 여러 tick 에 걸쳐 수리한다 — 여기선
    # 메커니즘 검증이 목적이라 수렴까지 반복 (60 = 필요 이동각/Δθ 의 여유 상한).
    for _ in range(60):
        assert repair(plan, dri_new, idx, p), "repair 가 개선을 못 찾음"
        idx = check(plan, dri_new, BOAT, p)
        if idx is None:
            break
    assert idx is None


# ---------- (v) 장애물 0개 → R7 직선 폴백 ----------

def test_r7_fallback_is_straight_to_waypoint():
    plan = gen(dri_of([]))
    assert plan.r1 is None
    lateral = np.abs(plan.samples[:, 1])         # waypoint 가 +x 축 위
    assert float(lateral.max()) < 0.15


# ---------- (vi) waypoint 가 r_end 안/밖 ----------

def test_waypoint_inside_clamps_r_end():
    plan = gen(dri_of([(3.0, 0.8)]), wp=(5.0, 0.0))
    assert plan.radii[-1] == pytest.approx(5.0, abs=0.05)
    end_dist = float(np.hypot(*(plan.samples[-1] - plan.cps[0])))
    assert end_dist == pytest.approx(5.0, abs=0.05)


def test_waypoint_far_r_end_is_grid_bound():
    """배가 격자 중앙 → r_end = 10 − boundary_margin."""
    plan = gen(dri_of([(3.0, 0.8)]), wp=(30.0, 0.0))
    p = PlannerParams()
    assert plan.radii[-1] == pytest.approx(10.0 - p.boundary_margin, abs=RES)


# ---------- (vii) 기준 장애물 선택 ----------

def test_ref_obstacle_rear_hard_cut():
    """배 뒤 1m + 정면 5m → 뒤는 하드컷, r1 = 5m."""
    plan = gen(dri_of([(-1.0, 0.0), (5.0, 0.0)]))
    assert plan.r1 == pytest.approx(5.0, abs=2 * RES)


def test_ref_obstacle_bearing_weighted_distance():
    """정면 8m vs 측면 60° 3m → ρ_eff 가중해도 측면이 뽑힌다 (r1=3m)."""
    side = (3.0 * math.cos(math.radians(60)), 3.0 * math.sin(math.radians(60)))
    plan = gen(dri_of([(8.0, 0.0), side]))
    assert plan.r1 == pytest.approx(3.0, abs=2 * RES)


# ---------- (viii) CP 사다리는 r1 과 무관 ----------

def _ladder(plan):
    """‖p1‖ ↔ radii[-1] 균등 4분할 기대값."""
    lo, hi = p1_len(plan), plan.radii[-1]
    return [lo + k * (hi - lo) / 4.0 for k in (1, 2, 3, 4)]


def test_ladder_independent_of_near_obstacle():
    """장애물이 p1 보다 가까워도(측면 70° 1.2m) 사다리는 흔들리지 않고,
    r1 은 실거리 그대로 보존된다 (재생성 트리거 전용)."""
    near = (1.2 * math.cos(math.radians(70)), 1.2 * math.sin(math.radians(70)))
    plan = gen(dri_of([near]))
    assert plan.r1 == pytest.approx(1.2, abs=2 * RES)
    assert plan.radii == pytest.approx(_ladder(plan))
    seq = [p1_len(plan)] + list(plan.radii)
    assert all(a < b for a, b in zip(seq, seq[1:])), seq


def test_ladder_spread_when_obstacle_at_horizon():
    """지평선(8m) 장애물이 기준으로 뽑혀도 arc 가 r_end 쪽에 뭉치지 않는다 —
    구 규칙(radii[0]=r1)은 [7.9, 8.2, 8.6, 8.9] 로 퇴화해 경로가 장애물 밭
    입구에서 끝났다 (sim 실측)."""
    plan = gen(dri_of([(8.0, 0.2)]))
    assert plan.r1 == pytest.approx(8.1, abs=2 * RES)   # 기준 장애물은 그놈
    assert plan.radii == pytest.approx(_ladder(plan))   # 사다리는 균등
    assert plan.radii[0] < 4.0                          # 뭉침 아님


# ---------- (ix) r_end 격자 유도 — 배가 창 중심을 벗어남 ----------

def test_r_end_derived_from_actual_grid_geometry():
    """격자 recenter 지연(1Hz) 재현: 배가 창 중심에서 +1.5m. 명목 9m 를 쓰면
    p5 후보 7/31 이 창 밖(inf)으로 죽던 장면 (§7-5) — 유도값은 8.5−margin."""
    grid, _ = make_grid([(6.0, 0.3)], boat_xy=(0.0, 0.0))   # 창은 (0,0) 중심
    boat = (1.5, 0.0)
    dri = build_dri(grid, boat, YAW, DriParams())
    plan = generate(dri, boat, YAW, V_FULL, (15.0, 0.0), PlannerParams())
    p = PlannerParams()
    assert plan.radii[-1] == pytest.approx(8.5 - p.boundary_margin, abs=RES)
    # 모든 샘플이 창 안 (inf 없음)
    assert np.all(np.isfinite(
        dri.risk_at_many(plan.samples[:, 0], plan.samples[:, 1])))


# ---------- (x) spline 양끝 통과 + 시작 접선 ----------

def test_spline_endpoints_and_start_tangent():
    plan = gen(dri_of([(4.0, 0.2)]))
    assert np.allclose(plan.samples[0], plan.cps[0], atol=1e-6)
    assert np.allclose(plan.samples[-1], plan.cps[5], atol=0.05)
    d_first = plan.samples[1] - plan.samples[0]
    d1 = plan.cps[1] - plan.cps[0]
    cos = float(d_first @ d1 / (np.hypot(*d_first) * np.hypot(*d1)))
    # 첫 샘플 간격(0.1m) 동안 곡률 κ≈1 이면 접선이 ~3° 돈다 — 그 여유 포함
    assert cos > 0.995


# ---------- (xi) hard_max — 포위 장면에서 None ----------

def test_generate_returns_none_when_surrounded():
    """전진 반평면이 3m 장애물 벽 — argmin 후보조차 hard_max 초과 → None (R9)."""
    wall = [(3.0 * math.cos(a), 3.0 * math.sin(a))
            for a in np.linspace(-math.pi / 2, math.pi / 2, 25)]
    plan = gen(dri_of(wall), hard_max=1.0)
    assert plan is None


# ---------- (xii) repair 실패 조건 ----------

def test_repair_fails_without_improvement():
    """DRI 가 어디나 0 이면 ±Δθ 어느 쪽도 개선이 아님 → False."""
    dri = dri_of([])
    plan = gen(dri)
    assert repair(plan, dri, len(plan.samples) // 2, PlannerParams()) is False


def test_repair_respects_fov_limit_and_terminates():
    """waypoint 정면의 큰 위험 덩어리 — repair 를 반복해도 CP 방위는 부채꼴
    (wp ±fov/2) 안에 머물고, 유한 횟수 안에 False 로 끝난다."""
    p = PlannerParams()
    plan = gen(dri_of([]), wp=(9.0, 0.0))
    blob = [(6.0, y) for y in np.arange(-4.0, 4.01, 0.4)]
    dri_new = dri_of(blob)
    half = p.search_fov / 2
    for _ in range(200):                      # 발산 방지 상한
        idx = check(plan, dri_new, BOAT, p)
        if idx is None or not repair(plan, dri_new, idx, p):
            break
    else:
        pytest.fail("repair 가 200회 안에 안 끝남")
    for b in plan.bearings:
        assert abs(b - plan.wp_bearing) <= half + 1e-9


# ---------- Plan 메타 무결성 ----------

def test_progress_idx_monotone():
    """배가 경로를 따라 전진하면 progress_idx 는 단조 전진, 후퇴해도 안 줄어든다."""
    p = PlannerParams()
    dri = dri_of([(4.0, 0.2)])
    plan = gen(dri)
    check(plan, dri, BOAT, p)
    i0 = plan.progress_idx
    mid = tuple(plan.samples[len(plan.samples) // 2])
    check(plan, dri, mid, p)
    i1 = plan.progress_idx
    assert i1 >= i0
    check(plan, dri, BOAT, p)                 # 뒤로 텔레포트해도
    assert plan.progress_idx >= i1            # 안 되돌아간다


def test_samples_spacing_is_half_resolution():
    plan = gen(dri_of([(4.0, 0.2)]))
    seg = np.hypot(*np.diff(plan.samples, axis=0).T)
    assert np.all(seg < 0.5 * RES + 1e-6)
    assert np.median(seg) == pytest.approx(0.5 * RES, rel=0.1)
