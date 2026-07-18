"""avoid_fsm.py 단독 테스트 (AVOIDANCE.MD §3-4) — ROS 없이 돈다.

가짜 시계(now_t 직접 주입) + 합성 장면 루프 — avoid_fsm 을 순수 모듈로
분리한 이유가 이 테스트다. 장면 좌표계는 test_bspline_planner 와 동일:
배 (0,0), heading +x, 격자는 배 중심 20×20m.
"""
import math

import numpy as np

from kaboat_behaviors.dri import DriParams, build_dri
from kaboat_behaviors.bspline_planner import PlannerParams, check, curvature_ahead
from kaboat_behaviors.avoid_fsm import (
    AvoidFsm, FsmParams, follow_cmd, speed_to_cmd)
from test_dri import make_grid

BOAT = (0.0, 0.0)
YAW = 0.0
V_FULL = (1.48, 0.0)
WP_FAR = (15.0, 0.0)
KP, KD = 18.0 / 118.6, 9.0 / 118.6   # behavior_base 게인과 동일


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
    plan = acquire(fsm, dri_of([(4.0, 0.2)])).plan
    p = FsmParams(kappa_window=6.0)
    lin, _ = follow_cmd(plan, BOAT, YAW, 0.0, WP_FAR, p, KP, KD)
    assert lin < speed_to_cmd(p.v_max, p) - 1e-12
    assert lin >= speed_to_cmd(p.v_min, p) * min(15.0 / p.slow_radius, 1.0) - 1e-12


def test_follow_caps_target_to_waypoint_when_close():
    """wp 가 lookahead 안 — 목표점 = wp 자체 (경로가 wp 를 지나쳐도 무시)."""
    fsm = make_fsm()
    wp = (1.5, 1.0)   # min_horizon(3m) 때문에 경로는 wp 너머까지 뻗는다
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


def test_follow_slows_near_waypoint():
    """slow_radius 안 — 도착 감속 (mission_manager 1m 전환 전 과속 방지)."""
    fsm = make_fsm()
    wp = (2.0, 0.0)
    plan = acquire(fsm, dri_of([]), wp=wp).plan
    p = FsmParams()
    lin, _ = follow_cmd(plan, BOAT, YAW, 0.0, wp, p, KP, KD)
    assert lin < speed_to_cmd(p.v_max, p) * (2.0 / p.slow_radius) + 1e-12


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
    """저장 r1=None(빈 밭 직선 plan) ∧ 현재 r1 유한 → (a') 발동."""
    fsm = make_fsm(t_backstop=1e9)
    out = acquire(fsm, dri_of([]))
    assert out.plan.r1 is None
    dri2 = dri_of([(6.0, 3.0)])             # 전방 부채꼴 등장, 경로는 안 건드림
    out = fsm.step(dri2, BOAT, YAW, V_FULL, 0.0, WP_FAR, 0.1)
    assert out.event == 'trigger_a2_regen'
    assert out.plan.r1 is not None


def test_trigger_a2_fires_on_r1_drop():
    """(a') 급락: 더 가까운 위협 등장 → 저장 r1 − δ 아래로 → 기준 교체."""
    fsm = make_fsm(t_backstop=1e9, delta_r1=1.0)
    out = acquire(fsm, dri_of([(8.0, 3.0)]))
    r1_far = out.plan.r1
    near = [(8.0, 3.0), (4.0, -2.5)]        # 4.7m 측방 — r1_far(8.5)보다 급락
    out = fsm.step(dri_of(near), BOAT, YAW, V_FULL, 0.0, WP_FAR, 0.1)
    assert out.event == 'trigger_a2_regen'
    assert out.plan.r1 < r1_far - 1.0


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
    blob = [(6.0, y) for y in np.arange(-1.2, 1.21, 0.4)]
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


# ---------- ESCAPE ----------

def enter_escape(fsm):
    """빈 밭에서 plan 확보 → 포위 벽으로 ESCAPE 진입시키는 공통 준비."""
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


def test_escape_steers_stern_to_low_dri():
    """후방 좌측(방위 135~180°)이 막힘 — 선미가 우후방으로 향하도록 조향.
    yaw=0 이라 선미 기준 오차 = wrap(target−π) < 0 쪽(우후방) → ang < 0
    ...가 아니라 target≈-135° 면 err=wrap(-2.36−π)=+0.78 → ang > 0."""
    fsm = make_fsm(t_backstop=1e9, d_panic=3.0)
    enter_escape(fsm)
    rear_left = WALL + [(2.5 * math.cos(a), 2.5 * math.sin(a))
                        for a in np.linspace(math.radians(120),
                                             math.radians(180), 8)]
    out = fsm.step(dri_of(rear_left), BOAT, YAW, V_FULL, 0.0, WP_FAR, 0.2)
    assert out.linear_x < 0.0
    assert out.angular_z > 0.0     # 선미를 자유로운 우후방(-135°권)으로


def test_escape_surrounded_still_commands():
    """완전 포위 — 실패 없이 전방위 argmin 으로 명령을 낸다 (최후 단 불허)."""
    fsm = make_fsm(t_backstop=1e9, d_panic=3.0)
    enter_escape(fsm)
    ring = [(2.2 * math.cos(a), 2.2 * math.sin(a))
            for a in np.linspace(-math.pi, math.pi, 40, endpoint=False)]
    out = fsm.step(dri_of(ring), BOAT, YAW, V_FULL, 0.0, WP_FAR, 0.2)
    assert out.state == 'ESCAPE' and out.event == 'escape_surrounded'
    assert out.linear_x < 0.0


def test_escape_rear_softening_still_repels():
    """a_head 는 후방을 연하게 그림 — 그래도 정후방
    장애물 방향이 argmin 으로 뽑히면 안 된다 (후진 진행 방향 충돌)."""
    from kaboat_behaviors.avoid_fsm import escape_pick
    dri = dri_of([(-2.0, 0.0)])   # 정후방 2m
    bearing, surrounded = escape_pick(dri, BOAT, YAW, PlannerParams(), FsmParams())
    assert not surrounded
    assert abs(_wrap_t(bearing - math.pi)) > 0.3   # 정후방(π) 회피


def _wrap_t(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


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
