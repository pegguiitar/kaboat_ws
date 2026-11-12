"""avoid_fsm.py 단독 테스트 (AVOIDANCE.MD §3-4) — ROS 없이 돈다.

가짜 시계(now_t 직접 주입) + 합성 장면 루프 — avoid_fsm 을 순수 모듈로
분리한 이유가 이 테스트다. 장면 좌표계는 test_bspline_planner 와 동일:
배 (0,0), heading +x, 격자는 배 중심 20×20m.
"""
import math

import numpy as np
import pytest

from kaboat_behaviors.dri import DriParams, build_dri
from kaboat_behaviors.bspline_planner import PlannerParams, check, curvature_ahead
from kaboat_behaviors.avoid_fsm import (
    AvoidFsm, FsmParams, follow_cmd, speed_to_cmd)
from test_dri import make_grid

BOAT = (0.0, 0.0)
YAW = 0.0
V_FULL = (1.48, 0.0)
WP_FAR = (15.0, 0.0)
KP, KD = 1.5, 0.15   # 회피 노드 게인 = LOS bspline_track (test_los 실물 튜닝값)


def dri_of(obstacles, boat_xy=BOAT, yaw=YAW):
    grid, _ = make_grid(obstacles, boat_xy=boat_xy)
    return build_dri(grid, boat_xy, yaw, DriParams())


def make_fsm(**fsm_kw):
    return AvoidFsm(PlannerParams(), FsmParams(**fsm_kw), yaw_kp=KP, yaw_kd=KD)


def acquire(fsm, dri, boat=BOAT, yaw=YAW, vel=V_FULL, wp=WP_FAR, t=0.0):
    out = fsm.step(dri, boat, yaw, vel, 0.0, wp, t)
    assert out.plan is not None, 'plan 확보 실패 — 장면 구성 오류'
    return out


# ---------- curvature_ahead ----------

def test_curvature_straight_is_zero():
    fsm = make_fsm()
    plan = acquire(fsm, dri_of([])).plan
    assert curvature_ahead(plan, 2.0) < 1e-6


def test_curvature_avoid_path_positive_and_bounded():
    """회피 경로 κ 는 실측 0.9~1.2 급 — 0 도 아니고 터무니없지도 않아야."""
    fsm = make_fsm()
    plan = acquire(fsm, dri_of([(4.0, 0.2)])).plan
    k = curvature_ahead(plan, 6.0)
    assert 0.05 < k < 5.0, k


# ---------- follow_cmd ----------

def test_follow_straight_full_speed_no_steer():
    fsm = make_fsm()
    plan = acquire(fsm, dri_of([])).plan
    p = FsmParams()
    lin, ang = follow_cmd(plan, BOAT, YAW, 0.0, WP_FAR, p, KP, KD)
    assert abs(ang) < 1e-9
    assert math.isclose(lin, speed_to_cmd(p.v_max, p), rel_tol=1e-9)


def test_follow_curvature_slows_down():
    """기본 창(2m)엔 굽이가 아직 안 들어와 감속이 없는 게 정상 —
    창을 굽이(3~5m 지점)까지 넓혀 감속 경로를 검증한다."""
    fsm = make_fsm()
    # 좁은 후보 FOV와 높은 직진성 가중치에서는 단일 장애물을 완만하게 피한다.
    # 감속 메커니즘 자체는 S자 곡률이 확실히 생기는 두 장애물로 검증한다.
    plan = acquire(fsm, dri_of([(3.0, -0.5), (4.0, 1.0)])).plan
    # 현재 기본 v_max=1.1에서는 이 곡률도 그대로 추종 가능하다. 감속 공식 자체는
    # 기존 교정 전속 1.48m/s를 넣어 곡률 제한이 실제로 작동하는지 검증한다.
    p = FsmParams(kappa_window=6.0, v_max=1.48)
    lin, _ = follow_cmd(plan, BOAT, YAW, 0.0, WP_FAR, p, KP, KD)
    assert lin < speed_to_cmd(p.v_max, p) - 1e-12
    assert lin >= speed_to_cmd(p.v_min, p) * min(15.0 / p.slow_radius, 1.0) - 1e-12


def test_follow_caps_target_to_waypoint_when_close():
    """wp 가 lookahead 안 — 목표점 = wp 자체 (경로가 wp 를 지나쳐도 무시)."""
    fsm = make_fsm()
    wp = (0.6, 0.4)   # wp 거리 0.72 m < lookahead(1 m), min_horizon(2 m) 이라 경로는 wp 너머까지 뻗는다
    plan = acquire(fsm, dri_of([]), wp=wp).plan
    p = FsmParams()
    _, ang = follow_cmd(plan, BOAT, YAW, 0.0, wp, p, KP, KD)
    err_wp = math.atan2(wp[1], wp[0])
    assert math.isclose(ang, KP * err_wp, rel_tol=1e-9)


def test_follow_large_heading_error_slow_forward():
    """오차 >60° — 저속 전진 유지 (제자리 급회전 방지, seek_goal 규칙)."""
    fsm = make_fsm()
    plan = acquire(fsm, dri_of([])).plan
    p = FsmParams()
    lin, ang = follow_cmd(plan, BOAT, math.pi, 0.0, WP_FAR, p, KP, KD)
    assert ang != 0.0
    assert 0.0 < lin <= speed_to_cmd(0.3 * p.v_max, p) + 1e-12


def test_follow_pivots_in_place_beyond_pivot_angle():
    """pivot_angle(회피 FSM, 35°): 다음 목표점이 그보다 벗어나 있으면 전진 0 — 제자리 회전 정렬.
    그 안이면 보통처럼 전진. pivot_angle=None(LOS 등)은 예전 규칙 그대로 (위 테스트)."""
    fsm = make_fsm()
    plan = acquire(fsm, dri_of([])).plan          # +x 직선
    p = FsmParams()
    lin, ang = follow_cmd(plan, BOAT, math.radians(50.0), 0.0, WP_FAR, p, KP, KD,
                          pivot_angle=p.pivot_angle)
    assert lin == 0.0 and ang < 0.0               # 오른쪽(시계)으로 돌아 +x 에 맞춤
    lin, ang = follow_cmd(plan, BOAT, math.radians(20.0), 0.0, WP_FAR, p, KP, KD,
                          pivot_angle=p.pivot_angle)
    assert lin > 0.0 and ang < 0.0
    # 예전 규칙이면 50° 에서도 전진 (60° 미만)
    lin_old, _ = follow_cmd(plan, BOAT, math.radians(50.0), 0.0, WP_FAR, p, KP, KD)
    assert lin_old > 0.0


def test_stationary_boom_scene_pivots_then_goes():
    """2026-11-12 영상 2:22~2:50 재현 — 정지한 배 앞 2.7 m 붐 줄, 왼쪽(+y) 끝만 열림.
    예전: 전진 7 %·회전 31 % (좌 −·우 +) 로 제자리에서 흔들림.
    지금: 첫 명령은 제자리 회전(전진 0), 경로 시작 방향에 맞추면 직선 속도로 전진."""
    wall = [(2.7, -y) for y in np.round(np.arange(-1.4, 6.01, 0.1), 2)]
    dri = dri_of(wall)
    fsm = make_fsm()
    out = fsm.step(dri, BOAT, YAW, (0.0, 0.0), 0.0, WP_FAR, 0.0)   # 정지 상태에서 계획
    assert out.event == 'plan_acquired'
    d1 = fsm.plan.cps[1] - fsm.plan.cps[0]
    start = math.atan2(d1[1], d1[0])
    assert start > math.radians(30.0)             # 열린 왼쪽으로 곧게 시작
    out = fsm.step(dri, BOAT, YAW, (0.0, 0.0), 0.0, WP_FAR, 0.1)
    assert out.state == 'FOLLOW' and out.linear_x == 0.0 and out.angular_z > 0.0
    out = fsm.step(dri, BOAT, start, (0.0, 0.0), 0.0, WP_FAR, 0.2)  # 제자리 회전으로 정렬된 뒤
    p = FsmParams()
    assert out.state == 'FOLLOW'
    assert out.linear_x >= speed_to_cmd(1.0, p)   # 곡률 감속에 묶이지 않음 (예전 0.07)


def test_follow_slows_near_waypoint():
    """slow_radius 안 — 도착 감속 (mission_manager 5m 전환 전 과속 방지)."""
    fsm = make_fsm()
    wp = (1.0, 0.0)   # slow_radius(1.5 m) 안
    plan = acquire(fsm, dri_of([]), wp=wp).plan
    p = FsmParams()
    lin, _ = follow_cmd(plan, BOAT, YAW, 0.0, wp, p, KP, KD)
    assert 0.0 < lin < speed_to_cmd(p.v_max * (1.0 / p.slow_radius), p) + 1e-12


def test_follow_combines_large_heading_and_waypoint_slowdown_in_old_order():
    """공통 조종기 분리 전과 동일하게 회전 속도캡 뒤 거리감속을 적용한다."""
    fsm = make_fsm()
    wp = (-0.7, 0.0)   # 뒤쪽(오차 >60°) + lookahead(0.8 m)·slow_radius(1.5 m) 안
    plan = acquire(fsm, dri_of([]), wp=wp).plan
    p = FsmParams()
    lin, _ = follow_cmd(plan, BOAT, YAW, 0.0, wp, p, KP, KD)
    expected_v = 0.3 * p.v_max * (0.7 / p.slow_radius)
    assert lin == pytest.approx(speed_to_cmd(expected_v, p))


# ---------- AvoidFsm.step — 기본 흐름 ----------

def test_first_tick_stops_and_acquires():
    fsm = make_fsm()
    out = fsm.step(dri_of([]), BOAT, YAW, V_FULL, 0.0, WP_FAR, 0.0)
    assert (out.linear_x, out.angular_z) == (0.0, 0.0)
    assert out.event == 'plan_acquired' and out.plan is not None


def test_second_tick_follows():
    fsm = make_fsm()
    dri = dri_of([])
    fsm.step(dri, BOAT, YAW, V_FULL, 0.0, WP_FAR, 0.0)
    out = fsm.step(dri, BOAT, YAW, V_FULL, 0.0, WP_FAR, 0.1)
    assert out.linear_x > 0.0 and out.event is None


def test_backstop_regenerates():
    fsm = make_fsm(t_backstop=3.0)
    dri = dri_of([])
    fsm.step(dri, BOAT, YAW, V_FULL, 0.0, WP_FAR, 0.0)
    out = fsm.step(dri, (2.0, 0.0), YAW, V_FULL, 0.0, WP_FAR, 3.5)
    assert out.event == 'backstop_regen'
    assert out.plan.created_t == 3.5 and out.plan.origin == (2.0, 0.0)


def test_violation_resolved_by_ladder():
    """빈 격자로 직선 plan 확보 후 경로 위 장애물 등장 → 사다리가 tick 안에
    해소 (수리가 먼저, 안 되면 재생성)."""
    fsm = make_fsm()
    fsm.step(dri_of([]), BOAT, YAW, V_FULL, 0.0, WP_FAR, 0.0)
    dri2 = dri_of([(4.0, 0.2)])
    out = fsm.step(dri2, BOAT, YAW, V_FULL, 0.0, WP_FAR, 0.1)
    assert out.event in ('violation_repaired', 'violation_regen')
    assert check(out.plan, dri2, BOAT, fsm.p) is None   # tick 종료 시 무위반
    assert np.all(np.isfinite(out.plan.samples))


def test_reset_clears_plan():
    fsm = make_fsm()
    fsm.step(dri_of([]), BOAT, YAW, V_FULL, 0.0, WP_FAR, 0.0)
    fsm.reset()
    assert fsm.plan is None and fsm.state == 'FOLLOW'


# ---------- FOLLOW 사다리 ----------

WALL = [(3.0 * math.cos(a), 3.0 * math.sin(a))
        for a in np.linspace(-math.pi / 2, math.pi / 2, 25)]  # 전진 반평면 전멸


def test_trigger_a_fires_after_passing_r1():
    """기준 장애물(r1) 통과 — 진행 반경 > 저장 r1 → 재생성."""
    fsm = make_fsm(t_backstop=1e9)          # (c) 차단 — (a)만 격리
    dri = dri_of([(4.0, 2.5)])              # 측방 장애물: r1 유한, 경로 위반 없음
    out = acquire(fsm, dri)
    assert out.plan.r1 is not None
    r1 = out.plan.r1
    boat2 = (r1 + 0.5, 0.0)                 # r1 반경 밖으로 진행한 배
    dri2 = dri_of([(4.0, 2.5)], boat_xy=boat2)
    out = fsm.step(dri2, boat2, YAW, V_FULL, 0.0, WP_FAR, 0.1)
    assert out.event == 'trigger_a_regen'
    assert out.plan.origin == boat2


def test_trigger_a2_fires_on_none_to_finite():
    """저장 r1=None(빈 밭 직선 plan) ∧ 현재 r1 유한 → (a') 발동 (hold 0 — 판정만 격리)."""
    fsm = make_fsm(t_backstop=1e9, trigger_a2_hold=0.0)
    out = acquire(fsm, dri_of([]))
    assert out.plan.r1 is None
    dri2 = dri_of([(6.0, 3.0)])             # 전방 부채꼴 등장, 경로는 안 건드림
    out = fsm.step(dri2, BOAT, YAW, V_FULL, 0.0, WP_FAR, 0.1)
    assert out.event == 'trigger_a2_regen'
    assert out.plan.r1 is not None


def test_trigger_a2_fires_on_r1_drop():
    """(a') 급락: 더 가까운 위협 등장 → 저장 r1 − δ 아래로 → 기준 교체 (hold 0)."""
    fsm = make_fsm(t_backstop=1e9, delta_r1=1.0, trigger_a2_hold=0.0)
    out = acquire(fsm, dri_of([(8.0, 3.0)]))
    r1_far = out.plan.r1
    near = [(8.0, 3.0), (4.0, -2.5)]        # 4.7m 측방 — r1_far(8.5)보다 급락
    out = fsm.step(dri_of(near), BOAT, YAW, V_FULL, 0.0, WP_FAR, 0.1)
    assert out.event == 'trigger_a2_regen'
    assert out.plan.r1 < r1_far - 1.0


def test_trigger_a2_waits_for_persistent_new_obstacle():
    """기본 trigger_a2_hold 0.5 s: 한 tick 깜빡인 가까운 칸(물보라)으로는 재계획 안 함,
    0.5 s 이상 계속 보이면 재계획 (2026-11-12 영상: 재계획 평균 2.8 s 마다)."""
    fsm = make_fsm(t_backstop=1e9, delta_r1=1.0)
    acquire(fsm, dri_of([(8.0, 3.0)]))
    near = dri_of([(8.0, 3.0), (4.0, -2.5)])
    far = dri_of([(8.0, 3.0)])
    assert fsm.step(near, BOAT, YAW, V_FULL, 0.0, WP_FAR, 0.1).event is None   # 깜빡
    assert fsm.step(far, BOAT, YAW, V_FULL, 0.0, WP_FAR, 0.2).event is None    # 사라짐
    assert fsm.step(near, BOAT, YAW, V_FULL, 0.0, WP_FAR, 0.3).event is None   # 다시 시작
    assert fsm.step(near, BOAT, YAW, V_FULL, 0.0, WP_FAR, 0.6).event is None   # 0.3 s
    out = fsm.step(near, BOAT, YAW, V_FULL, 0.0, WP_FAR, 0.85)                 # 0.55 s
    assert out.event == 'trigger_a2_regen'


def test_trigger_a_skips_near_reference_obstacle():
    """기준 장애물이 trigger_a_min_r1(1.5 m)보다 가까우면 지나쳐도 재계획 안 함 — 코앞
    장애물 옆에서 새로 만들다 ESCAPE 로 가던 것(영상 trigger_a 1 s 안 ESCAPE ×5). 백스톱이 맡음."""
    for min_r1, expect in ((1.5, None), (0.0, 'trigger_a_regen')):
        fsm = make_fsm(t_backstop=1e9, trigger_a_min_r1=min_r1)
        acquire(fsm, dri_of([(4.0, 2.5)]))
        fsm.plan.r1 = 1.2                         # 가까운 기준 장애물이었다고 둠
        boat2 = (1.6, 0.0)                        # 진행 반경 1.6 > r1
        out = fsm.step(dri_of([(4.0, 2.5)], boat_xy=boat2), boat2, YAW, V_FULL, 0.0,
                       WP_FAR, 0.1)
        assert out.event == expect, min_r1


def test_backstop_replan_keeps_side_of_current_plan():
    """백스톱 재계획이 지금 경로(왼쪽 우회)를 prev_plan 으로 넘겨, 좌우 비용이 같아진 장면에서도
    왼쪽을 유지한다 (예전: 처음부터 만들어 오른쪽으로 뒤집힘)."""
    bar = lambda y: [(4.0, y - 0.2), (4.0, y), (4.0, y + 0.2)]  # noqa: E731
    fsm = make_fsm()
    acquire(fsm, dri_of(bar(-0.4)), vel=(1.0, 0.0))
    side = lambda plan: plan.samples[np.argmin(np.abs(plan.samples[:, 0] - 4.0)), 1]  # noqa: E731
    assert side(fsm.plan) > 0.5
    out = fsm.step(dri_of(bar(0.0)), BOAT, YAW, (1.0, 0.0), 0.0, WP_FAR, FsmParams().t_backstop + 0.1)
    assert out.event == 'backstop_regen'
    assert side(out.plan) > 0.5


def test_trigger_a2_ignores_small_drop():
    """δ 이내 잔물결로는 재생성 안 함 — 트리거 소음 억제."""
    fsm = make_fsm(t_backstop=1e9, delta_r1=1.0)
    acquire(fsm, dri_of([(8.0, 3.0)]))
    out = fsm.step(dri_of([(7.6, 3.0)]), BOAT, YAW, V_FULL, 0.0, WP_FAR, 0.1)
    assert out.event is None


def test_violation_repaired_without_regen():
    """살짝 걸치는 장애물 — repair 가 CP 방위 이동으로 해소, 재생성 없음."""
    fsm = make_fsm(t_backstop=1e9)
    out = acquire(fsm, dri_of([]))
    created = out.plan.created_t
    # 경로(y=0) 살짝 옆 장애물 — check 위반은 나되 ±6° 수리로 빠질 수 있는 배치
    out = fsm.step(dri_of([(5.0, 0.6)]), BOAT, YAW, V_FULL, 0.0, WP_FAR, 0.1)
    assert out.event in ('violation_repaired', 'violation_regen')
    if out.event == 'violation_repaired':
        assert out.plan.created_t == created   # 같은 plan (재생성 아님)


def test_violation_regen_when_repair_cannot_fix():
    """정면 덩어리 — 수리 한도로 못 내리면 즉시 재생성으로 우회."""
    fsm = make_fsm(t_backstop=1e9)
    acquire(fsm, dri_of([]), wp=(9.0, 0.0))
    # ±30° 후보 범위 안에서 새 경로를 확보할 수 있는 폭의 정면 덩어리.
    blob = [(6.0, y) for y in np.arange(-1.0, 1.01, 0.4)]
    out = fsm.step(dri_of(blob), BOAT, YAW, V_FULL, 0.0, (9.0, 0.0), 0.1)
    assert out.event == 'violation_regen'
    assert check(out.plan, dri_of(blob), BOAT, fsm.p) is None


def test_violation_wait_slows_when_margin_left():
    """수리·재생성 전멸 + 위반까지 여유 ≥ d_panic → 감속 유지 (ESCAPE 아직)."""
    fsm = make_fsm(t_backstop=1e9, d_panic=2.0, t_retry=1e9)
    out = acquire(fsm, dri_of([]))
    p = FsmParams()
    # 전방 6m 링 — 위반은 멀리(≥ d_panic) 있고 generate 는 반평면 전멸로 None
    wall6 = [(6.0 * math.cos(a), 6.0 * math.sin(a))
             for a in np.linspace(-math.pi / 2, math.pi / 2, 40)]
    out = fsm.step(dri_of(wall6), BOAT, YAW, V_FULL, 0.0, WP_FAR, 0.1)
    if out.event == 'violation_wait':
        assert out.state == 'FOLLOW'
        assert 0.0 < out.linear_x <= speed_to_cmd(p.v_min, p) + 1e-12
    else:
        # 6m 링에서도 generate 가 답을 찾으면 이 장면은 규격 미달 — 결과만 검증
        assert out.event in ('violation_regen', 'escape_panic')


def test_escape_on_close_violation():
    """위반이 d_panic 안 — 즉시 ESCAPE (여유 없음)."""
    fsm = make_fsm(t_backstop=1e9, d_panic=3.0)
    acquire(fsm, dri_of([]))
    out = fsm.step(dri_of(WALL), BOAT, YAW, V_FULL, 0.0, WP_FAR, 0.1)
    assert out.state == 'ESCAPE' and out.event == 'escape_panic'
    assert (out.linear_x, out.angular_z) == (0.0, 0.0)  # ③ 전 stub — 정지
    # 노드가 로그에 남길 진입 이유: 경로상 위반 거리와 d_panic
    assert '위반까지' in fsm.escape_detail and 'd_panic 3.0 m' in fsm.escape_detail


def test_escape_after_no_plan_retry_timeout():
    """활성화부터 포위 — plan 확보 t_retry 초과 → ESCAPE."""
    fsm = make_fsm(t_retry=2.0)
    dri = dri_of(WALL)
    t, out = 0.0, None
    for _ in range(30):
        out = fsm.step(dri, BOAT, YAW, V_FULL, 0.0, WP_FAR, t)
        if out.state == 'ESCAPE':
            break
        assert out.event == 'generate_none'
        t += 0.1
    assert out.state == 'ESCAPE' and out.event == 'escape_no_plan'
    assert t <= 2.5
    assert fsm.escape_detail.startswith('경로 없음') and 't_retry 2.0 s' in fsm.escape_detail
    fsm.reset()
    assert fsm.escape_detail == ''


# ---------- ESCAPE ----------

# 배가 -x 쪽에서 +x 로 직진해 BOAT(0,0) 에 왔다 — ESCAPE 는 이 길을 거꾸로 되짚는다.
STRAIGHT_TRAIL = [(x, 0.0) for x in np.arange(-3.0, 0.0001, 0.1)]


def lay_trail(fsm, points):
    for point in points:
        fsm.trail.record(point)


def enter_escape(fsm, trail=STRAIGHT_TRAIL):
    """지나온 궤적 + 빈 밭에서 plan 확보 → 포위 벽으로 ESCAPE 진입시키는 공통 준비."""
    lay_trail(fsm, trail)
    acquire(fsm, dri_of([]))
    out = fsm.step(dri_of(WALL), BOAT, YAW, V_FULL, 0.0, WP_FAR, 0.1)
    assert fsm.state == 'ESCAPE', out.event
    return out


def test_escape_backs_up():
    """ESCAPE tick — 후진 명령(lin<0). 진입 tick(전이)만 정지."""
    fsm = make_fsm(t_backstop=1e9, d_panic=3.0)
    enter_escape(fsm)
    out = fsm.step(dri_of(WALL), BOAT, YAW, V_FULL, 0.0, WP_FAR, 0.2)
    assert out.state == 'ESCAPE' and out.linear_x < 0.0


def test_escape_reverse_output_is_escape_thrust():
    """후진 출력은 속도 곡선이 아니라 escape_thrust 그대로 (실물에서 0.04 로 안 움직이던 문제)."""
    fsm = make_fsm(t_backstop=1e9, d_panic=3.0)
    enter_escape(fsm)
    out = fsm.step(dri_of(WALL), BOAT, YAW, V_FULL, 0.0, WP_FAR, 0.2)
    assert out.state == 'ESCAPE'
    assert out.linear_x == pytest.approx(-FsmParams().escape_thrust)
    assert FsmParams().escape_thrust == pytest.approx(0.1)


def test_escape_straight_trail_backs_straight():
    """직선으로 왔으면 선미가 이미 지나온 길을 향한다 — 조향 0."""
    fsm = make_fsm(t_backstop=1e9, d_panic=3.0)
    enter_escape(fsm)
    out = fsm.step(dri_of(WALL), BOAT, YAW, V_FULL, 0.0, WP_FAR, 0.2)
    assert out.angular_z == pytest.approx(0.0, abs=1e-9)


def test_escape_follows_trail_not_dri():
    """좌후방에서 대각선으로 왔다 — 뒤쪽 DRI 와 무관하게 선미를 그 길(-135°)로 돌린다.
    yaw=0 → 선미 π, 오차 = wrap(-3π/4 − π) = +π/4 → ang > 0.
    정후방(π)에 장애물을 둬도(예전 argmin 이면 피했을 방향과 무관) 같은 답이다."""
    fsm = make_fsm(t_backstop=1e9, d_panic=3.0)
    diagonal = [(-d / math.sqrt(2.0), -d / math.sqrt(2.0))
                for d in np.arange(3.0, -0.0001, -0.1)]
    enter_escape(fsm, trail=diagonal)
    out = fsm.step(dri_of(WALL + [(-2.0, 0.0)]), BOAT, YAW, V_FULL, 0.0, WP_FAR, 0.2)
    assert out.linear_x < 0.0
    assert out.angular_z > 0.0


def test_escape_without_trail_holds_still():
    """궤적이 없으면(활성화 직후 포위 등) 선미 사각지대로 후진하지 않고 정지."""
    fsm = make_fsm(t_backstop=1e9, d_panic=3.0)
    enter_escape(fsm, trail=[])
    out = fsm.step(dri_of(WALL), BOAT, YAW, V_FULL, 0.0, WP_FAR, 0.2)
    assert out.state == 'ESCAPE' and out.event == 'escape_no_trail'
    assert (out.linear_x, out.angular_z) == (0.0, 0.0)
    out = fsm.step(dri_of(WALL), BOAT, YAW, V_FULL, 0.0, WP_FAR, 0.3)
    assert out.event is None                          # 이벤트는 처음 1회만
    assert (out.linear_x, out.angular_z) == (0.0, 0.0)


def test_escape_stops_at_trail_end():
    """궤적 가장 오래된 점까지 되짚었으면 정지한 채로 generate 만 재시도."""
    fsm = make_fsm(t_backstop=1e9, d_panic=3.0)
    enter_escape(fsm)
    fsm.step(dri_of(WALL), BOAT, YAW, V_FULL, 0.0, WP_FAR, 0.2)   # 되짚기 시작
    t, out, events = 0.3, None, []
    for x in np.arange(-0.2, -3.0001, -0.2):          # 배가 길을 따라 뒤로 물러남
        out = fsm.step(dri_of(WALL, boat_xy=(x, 0.0)), (x, 0.0), YAW,
                       (-0.3, 0.0), 0.0, WP_FAR, t)
        events.append(out.event)
        t += 0.1
    assert out.state == 'ESCAPE'
    assert events.count('escape_trail_end') == 1      # 이벤트는 처음 1회만
    assert (out.linear_x, out.angular_z) == (0.0, 0.0)


def test_trail_records_only_in_follow():
    """FOLLOW tick 은 위치를 기록하고, ESCAPE 중(되짚는 중)엔 기록하지 않는다."""
    fsm = make_fsm(t_backstop=1e9, d_panic=3.0)
    for i, x in enumerate(np.arange(-1.0, 0.0001, 0.2)):
        fsm.step(dri_of([], boat_xy=(x, 0.0)), (x, 0.0), YAW, V_FULL, 0.0,
                 WP_FAR, 0.1 * i)
    assert len(fsm.trail.points) == 6
    fsm.step(dri_of(WALL), BOAT, YAW, V_FULL, 0.0, WP_FAR, 1.0)    # → ESCAPE
    assert fsm.state == 'ESCAPE'
    n = len(fsm.trail.points)
    fsm.step(dri_of(WALL, boat_xy=(-0.5, 0.0)), (-0.5, 0.0), YAW,
             (-0.3, 0.0), 0.0, WP_FAR, 1.1)
    assert len(fsm.trail.points) == n


def test_trail_spacing_and_max_length():
    """간격 미만 이동은 무시, 최대 길이를 넘으면 오래된 점부터 버린다."""
    from kaboat_behaviors.avoid_fsm import Trail
    trail = Trail(spacing=0.1, max_length=1.0)
    trail.record((0.0, 0.0))
    trail.record((0.05, 0.0))                         # 0.1 m 미만 — 무시
    assert len(trail.points) == 1
    for x in np.arange(0.1, 3.0001, 0.1):
        trail.record((x, 0.0))
    assert trail.length() <= 1.0 + 1e-9
    assert trail.points[-1][0] == pytest.approx(3.0)
    assert trail.points[0][0] == pytest.approx(2.0)


def test_reset_clears_trail():
    fsm = make_fsm()
    lay_trail(fsm, STRAIGHT_TRAIL)
    fsm.reset()
    assert len(fsm.trail.points) == 0


def test_escape_recovery_drops_retraced_part():
    """복귀하면 이미 되짚은(최신) 구간은 버리고 남은 오래된 길만 둔다."""
    fsm = make_fsm(t_backstop=1e9, d_panic=3.0, escape_min_t=0.0, escape_n_ok=1)
    enter_escape(fsm)
    boat = (-1.0, 0.0)                                # 1 m 되짚은 지점
    out = fsm.step(dri_of([], boat_xy=boat), boat, YAW, V_FULL, 0.0, WP_FAR, 0.2)
    assert out.state == 'FOLLOW' and out.event == 'escape_recovered'
    xs = [p[0] for p in fsm.trail.points]
    assert max(xs) <= -1.0 + 1e-6                     # (0,0)~(-1,0) 구간은 사라짐
    assert min(xs) == pytest.approx(-3.0)


def test_escape_no_chatter_before_min_time():
    """진입 직후 전방이 열려도 최소 이탈 시간 전엔 복귀하지 않는다."""
    fsm = make_fsm(t_backstop=1e9, d_panic=3.0, escape_min_t=3.0, escape_n_ok=5)
    enter_escape(fsm)                                  # t=0.1 진입
    out = fsm.step(dri_of([]), BOAT, YAW, V_FULL, 0.0, WP_FAR, 0.2)
    assert out.state == 'ESCAPE' and out.event is None


def test_escape_recovers_after_min_time_and_streak():
    """최소 이탈 + 연속 성공 충족 → FOLLOW 복귀 (새 plan 보유)."""
    fsm = make_fsm(t_backstop=1e9, d_panic=3.0, escape_min_t=1.0, escape_n_ok=3)
    enter_escape(fsm)                                  # t=0.1 진입
    open_dri = dri_of([])
    t, out = 0.2, None
    for _ in range(40):
        out = fsm.step(open_dri, BOAT, YAW, V_FULL, 0.0, WP_FAR, t)
        if out.state == 'FOLLOW':
            break
        t += 0.1
    assert out.state == 'FOLLOW' and out.event == 'escape_recovered'
    assert out.plan is not None and t >= 1.0


def test_escape_streak_resets_on_failure():
    """성공-실패-성공 은 연속이 아니다 — 실패가 카운터를 리셋한다."""
    fsm = make_fsm(t_backstop=1e9, d_panic=3.0, escape_min_t=0.0, escape_n_ok=2)
    enter_escape(fsm)
    open_dri, wall_dri = dri_of([]), dri_of(WALL)
    fsm.step(open_dri, BOAT, YAW, V_FULL, 0.0, WP_FAR, 0.2)   # 성공 1
    fsm.step(wall_dri, BOAT, YAW, V_FULL, 0.0, WP_FAR, 0.3)   # 실패 → 리셋
    out = fsm.step(open_dri, BOAT, YAW, V_FULL, 0.0, WP_FAR, 0.4)  # 성공 1
    assert out.state == 'ESCAPE'                       # 아직 2연속 아님
    out = fsm.step(open_dri, BOAT, YAW, V_FULL, 0.0, WP_FAR, 0.5)  # 성공 2
    assert out.state == 'FOLLOW'


@pytest.mark.parametrize('radius', [3.0, 6.0])
def test_closed_loop_retrace_follows_curved_trail(radius):
    """곡선으로 왔으면 후진도 그 곡선을 따라간다 — 이상화 운동학(dock 테스트와 같은 모델).

    배가 원호(반경 radius, 90°)를 반시계로 돌아 왔고, generate 가 계속 실패해
    ESCAPE 로 들어간다. 후진 중 궤적과의 거리가 계속 작아야 한다.
    """
    f = FsmParams(t_retry=0.0)
    fsm = AvoidFsm(PlannerParams(), f, yaw_kp=KP, yaw_kd=KD,
                   generator=lambda *a: None)
    angles = np.linspace(-math.pi / 2, 0.0, 200)       # (0,-r) 에서 (r,0) 까지
    trail = np.stack([radius * np.cos(angles), radius * np.sin(angles) + radius], axis=1)
    trail -= trail[-1]                                   # 끝점 = 배 위치 (0,0)
    lay_trail(fsm, trail)
    xy, yaw, rate = np.array([0.0, 0.0]), math.pi / 2, 0.0   # 원호 끝 접선 = +y
    dense = fsm.trail.retrace_samples(xy, 0.01)
    worst, out = 0.0, None
    for tick in range(600):
        out = fsm.step(None, xy, yaw, (0.0, 0.0), rate, WP_FAR, tick * 0.05)
        if out.event == 'escape_trail_end':
            break
        cmd = min(0.0, out.linear_x)
        speed = -f.v_full * (abs(cmd) / f.cmd_full) ** (1 / f.cmd_exp)
        rate = float(np.clip(out.angular_z, -0.31, 0.31)) * 0.52 / 0.31
        yaw += rate * 0.05
        xy = xy + speed * np.array([math.cos(yaw), math.sin(yaw)]) * 0.05
        worst = max(worst, float(np.min(np.hypot(*(dense - xy).T))))
    assert out.event == 'escape_trail_end'
    assert np.hypot(*(xy - trail[0])) < 0.5          # 지나온 길의 시작점까지 되짚음
    assert worst < 0.3, worst                          # 궤적에서 30 cm 안
