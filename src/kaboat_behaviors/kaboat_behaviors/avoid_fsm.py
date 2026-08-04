"""avoid_fsm — 회피 조종 정책 순수 라이브러리 (rclpy 무의존, AVOIDANCE.MD §3-4).

경계 규칙: dri = 위험 표현, bspline_planner = 경로 기하, **이 모듈 = 조종
정책** — 시간과 상태(FOLLOW/ESCAPE, plan 보유)를 유일하게 갖고, 스플라인
내부는 모른다. 노드(obstacle_planner)는 배선만. rclpy 무의존인 이유는
전이·타이머 로직을 가짜 시계 + 합성 장면 루프로 검증하기 위해서다
(위험도·경로 라이브러리의 설계 오류를 전부 잡은 방식) — 실물에선
rosbag tick 재생 디버깅이 된다.

구성:
  follow_cmd  — 경로 추종 (lookahead 점 PD 조향 + waypoint 캡 + 곡률 감속)
  escape_pick/escape_cmd — 후진 탈출 (후방 선분 최소 위험 방향, 선미 기준 조향)
  AvoidFsm    — 상태기계. 따라가기 사다리(검사→수리→재생성→긴급도 분기)와
                탈출 진입·복귀(최소 이탈 + 연속 성공) 판정을 갖는다.
"""
from dataclasses import dataclass
import math

import numpy as np

from .bspline_planner import (
    Plan, PlannerParams, check, curvature_ahead, generate, pick_ref_obstacle,
    repair)
from .motion_controller import (
    DesiredVelocity, MotionControllerParams, control_velocity,
    speed_to_command)


@dataclass
class FsmParams:
    """조종 정책 상수 — 값의 근거는 AVOIDANCE.MD §4 표.

    v↔cmd 캘리브레이션(cmd_full·v_full)은 물리 결합층(SKELETON §6) — 실물 전환 시
    이 두 값만 재실측하면 speed_to_cmd 가 통째로 교체 지점이다.
    """
    lookahead: float = 2.0        # 📝 follow 목표점 전방 호길이 [m]
    kappa_window: float = 2.0     # 📝 곡률 감속 전방 창 [m]
    v_min: float = 0.3            # 📝 곡률 감속 하한 [m/s]
    v_max: float = 1.4            # 📝 현재 회피 추종 상한 [m/s] (교정 전속은 v_full)
    omega_max: float = 0.52       # ✅ 실측 최대 요레이트 [rad/s] — v = ω_max/κ
    slow_radius: float = 3.0      # seek_goal 과 동일 — waypoint 근접 감속 반경 [m]
    t_backstop: float = 3.0       # 📝 재생성 트리거 (c) 주기 [s]
    d_panic: float = 3.0          # 📝 위반까지 경로거리 이 값 미만이면 즉시 ESCAPE [m]
    #   근거: 전속 1.48 에서 타력 감쇠 실측(3s 에 0.11 로) → 제동거리 ~2m + 여유
    t_retry: float = 5.0          # 📝 plan 재확보/위반 교착 누적 상한 [s] → ESCAPE
    delta_r1: float = 1.0         # 📝 트리거 (a') r1 급락 판정 폭 [m]
    escape_radius: float = 2.5    # 📝 ESCAPE 방위 후보 반경 [m] — 단거리 선분 시험
    escape_n_bearings: int = 31   # 반평면 후보 수 (생성기 n_candidates 와 동급)
    v_escape: float = 0.5         # 📝 후진 속도 [m/s] — 대칭 실측(-1.48)의 보수적 1/3
    escape_min_t: float = 3.0     # 📝 최소 이탈 시간 [s] — 즉시 복귀 채터링 차단
    escape_n_ok: int = 5          # 📝 복귀 조건: generate 연속 성공 tick 수
    cmd_full: float = 12.0 / 118.6  # ✅ v_full 을 내는 추력 명령 (SKELETON §6 직진 검증값)
    v_full: float = 1.48          # ✅ cmd_full 에서의 실측 속도 [m/s]
    cmd_exp: float = 1.64         # ✅ v↔cmd 멱지수 — sim 2점 fit (12N→1.480,
    #   6N→0.970 m/s, 2026-07-18). 정상상태 추력=항력 균형이 순수 2차 항력이면
    #   2.0, 실측은 선형 항력 섞여 1.64. 실물 전환 시 cmd_full·v_full 과 함께 재실측.


@dataclass
class StepOut:
    linear_x: float               # 추력 명령 비율 [-1,1] — 노드가 Twist 로 배선
    angular_z: float
    state: str                    # 'FOLLOW' | 'ESCAPE' — 로깅/viz 용
    plan: Plan | None             # 현재 추종 중인 plan — viz 용
    event: str | None = None      # tick 에서 일어난 일 (로깅) — 없으면 None


def _controller_params(p: FsmParams, yaw_kp: float,
                       yaw_kd: float) -> MotionControllerParams:
    return MotionControllerParams(
        yaw_kp=yaw_kp,
        yaw_kd=yaw_kd,
        cmd_full=p.cmd_full,
        v_full=p.v_full,
        cmd_exp=p.cmd_exp,
    )


def speed_to_cmd(v: float, p: FsmParams) -> float:
    """물리 속도[m/s] → 추력 명령 비율. cmd = cmd_full·(v/v_full)^cmd_exp.

    2026-07-18 sim 2점 실측 fit (비례 근사 → 멱곡선 교체): 비례로 두면
    중속 명령이 실제보다 큰 속도를 내 감속 의도(v=ω_max/κ)가 안 먹는다
    (cmd 절반 = 6N 이 0.74 아닌 0.97 m/s). 변환을 이 함수 하나로 격리해
    두는 이유: 실물 전환 시 교정 곡선만 여기서 교체 (SKELETON §6 지도).
    """
    return speed_to_command(v, _controller_params(p, 0.0, 0.0))


def follow_cmd(plan: Plan, boat_xy, yaw: float, yaw_rate: float, wp_xy,
               fsm: FsmParams, yaw_kp: float, yaw_kd: float
               ) -> tuple[float, float]:
    """경로 추종 명령 (linear_x, angular_z) — PD 조향 + 곡률 feedforward 감속.

    조향은 seek_goal() 과 같은 PD(게인도 노드가 같은 값 주입), 목표점만
    경로 위 lookahead 점. 감속은 전방 창 max κ 로 v = clamp(ω_max/κ, v_min,
    v_max) — κ_avail = ω/v 라 교정 전속(1.48)에선 0.35, 현재 회피 상한
    1.1m/s에서도 0.47뿐인데 경로는 κ 0.9~1.2가 필요해(실측) 조향만으론
    물리적으로 못 따라간다.
    heading 오차 비례 감속 항은 v1 제외 — seek_goal 의 이산 규칙(오차 >60°
    면 저속)만 재사용한다 (제자리 급회전 = 전복 위험 차단, SKELETON §6).
    """
    boat = np.asarray(boat_xy, dtype=np.float64)
    wp = np.asarray(wp_xy, dtype=np.float64)
    wp_dist = float(np.hypot(*(wp - boat)))

    # 목표점: 진행 위치 + lookahead 호길이 앞 샘플. waypoint 근접 시 wp 로
    # 캡 — r_end 가 min_horizon 으로 클램프돼 경로가 wp 를 지나쳐 뻗는 문제
    # 차단 (도착·완료 보고는 obstacle_planner/mission_manager 담당).
    if wp_dist <= fsm.lookahead:
        target = wp
    else:
        idx = min(plan.progress_idx + int(round(fsm.lookahead / plan.spacing)),
                  len(plan.samples) - 1)
        target = plan.samples[idx]
        if float(np.hypot(*(target - boat))) < 0.3:
            # 경로 끝에 도달했는데 재생성이 계속 실패한 퇴화 상황 —
            # 방위가 정의되도록 wp 를 임시 목표로 (안전은 check 가 보장)
            target = wp

    kappa = curvature_ahead(plan, fsm.kappa_window)
    v = fsm.v_max if kappa < 1e-6 else min(max(fsm.omega_max / kappa,
                                               fsm.v_min), fsm.v_max)
    wp_scale = min(wp_dist / fsm.slow_radius, 1.0)
    v *= wp_scale  # waypoint 근접 감속 (seek_goal 규칙)

    direction = target - boat
    norm = float(np.hypot(*direction))
    if norm <= 1e-12:
        desired = DesiredVelocity(0.0, 0.0)
    else:
        desired = DesiredVelocity(
            vx=float(v * direction[0] / norm),
            vy=float(v * direction[1] / norm),
        )
    out = control_velocity(
        desired, yaw, yaw_rate, _controller_params(fsm, yaw_kp, yaw_kd),
        # 기존 순서: 큰 오차 속도캡 후 waypoint 감속. 두 조건이 동시에
        # 걸려도 refactor 전과 같은 명령이 되도록 캡에도 wp_scale을 적용한다.
        turn_speed_limit=0.3 * fsm.v_max * wp_scale)
    return out.linear_x, out.angular_z


def escape_pick(dri, boat_xy, yaw: float, planner: PlannerParams,
                fsm: FsmParams) -> tuple[float, bool]:
    """ESCAPE 조향 방위 선택  — (방위, 포위 여부) 반환.

    후방 반평면(선미 ±90°) 후보를 배→후보점 **선분 max DRI argmin** 으로
    고른다 — 탐색을 후방으로 제한하는 이유: ① 전방은 이미 더 강한 시험
    (스플라인 완주+하드 필터)에서 탈락했는데 선분(단거리)만 통과하는 전방
    주머니로 전진하면 갇힘 심화, ② 후방 = 방금 지나온 물 = 실측 보증 free,
    ③ ESCAPE 의 목적은 항법이 아니라 generate 전제 리셋(surge↓→p1 수축).
    후방 **전원 threshold 초과(포위)일 때만** 전방위로 확장. 방향 선정은
    어느 범위에서든 **무조건 argmin — 후보를 기준치로 거르지 않는다**
    (ESCAPE 는 사다리 최후 단이라 실패 불허: 하드 필터는 실패를 받아줄
    다음 단이 있는 generate 만의 특권).
    """
    boat = np.asarray(boat_xy, dtype=np.float64)

    def seg_risks(bearings):
        pts = boat[None, :] + fsm.escape_radius * np.stack(
            [np.cos(bearings), np.sin(bearings)], axis=1)
        ts = np.linspace(0.125, 1.0, 8)[:, None, None]
        seg = boat[None, None, :] + ts * (pts[None, :, :] - boat[None, None, :])
        r = dri.risk_at_many(seg[..., 0].ravel(), seg[..., 1].ravel())
        return r.reshape(8, -1).max(axis=0)

    stern = yaw + math.pi
    rear = stern + np.linspace(-0.5 * math.pi, 0.5 * math.pi,
                               fsm.escape_n_bearings)
    risks = seg_risks(rear)
    surrounded = bool(np.min(risks) > planner.safe_threshold)
    if not surrounded:
        return float(rear[int(np.argmin(risks))]), False
    full = stern + np.linspace(-math.pi, math.pi, 2 * fsm.escape_n_bearings,
                               endpoint=False)
    risks = seg_risks(full)
    return float(full[int(np.argmin(risks))]), True


def escape_cmd(dri, boat_xy, yaw: float, yaw_rate: float,
               planner: PlannerParams, fsm: FsmParams,
               yaw_kp: float, yaw_kd: float) -> tuple[float, float, bool]:
    """후진 + 저DRI 조향 (linear_x, angular_z, surrounded).

    기준각은 **선미**: 오차 = wrap(target − (yaw+π)) — 후진 중엔
    꽁무니가 진행 방향이므로. 차동추력의 요 모멘트는 속도 부호와 무관해
    조향 부호는 전진과 동일 (2026-07-18 sim 실측: wz +0.44 vs +0.49).
    """
    target, surrounded = escape_pick(dri, boat_xy, yaw, planner, fsm)
    desired = DesiredVelocity(
        vx=fsm.v_escape * math.cos(target),
        vy=fsm.v_escape * math.sin(target),
        reverse=True,
    )
    out = control_velocity(
        desired, yaw, yaw_rate, _controller_params(fsm, yaw_kp, yaw_kd))
    return out.linear_x, out.angular_z, surrounded


class AvoidFsm:
    """조종 정책 상태기계. 노드가 tick 마다 step() 을 부르고 결과를 발행한다.

    모든 경로가 cmd 를 반환한다 — cmd_mux 300ms 워치독에 걸리지 않도록
    tick 당 발행 1회는 노드가 보장하되, 여기서 None 을 내지 않는 게 전제.
    """

    def __init__(self, planner: PlannerParams, fsm: FsmParams,
                 yaw_kp: float, yaw_kd: float):
        self.p = planner
        self.fsm = fsm
        self.yaw_kp = yaw_kp
        self.yaw_kd = yaw_kd
        self.reset()

    def reset(self) -> None:
        """비활성→재활성 전이 시 노드가 호출 — 낡은 plan·상태 이월 금지."""
        self.state = 'FOLLOW'
        self.plan: Plan | None = None
        self._no_plan_t0 = None    # plan 부재가 시작된 시각 — t_retry 판정
        self._stuck_t0 = None      # 위반 미해소(수리·재생성 다 실패) 시작 시각
        self._escape_t0 = None     # ESCAPE 진입 시각 — 최소 이탈 판정
        self._escape_ok = 0        # ESCAPE 중 generate 연속 성공 tick 수

    # ── tick ─────────────────────────────────────────
    def step(self, dri, boat_xy, yaw: float, vel_xy, yaw_rate: float,
             wp_xy, now_t: float) -> StepOut:
        """tick 진입점 (§4). dri 는 노드가 매 tick build_dri 로 재계산해 넘긴다."""
        if self.state == 'ESCAPE':
            return self._escape_tick(dri, boat_xy, yaw, vel_xy, yaw_rate,
                                     wp_xy, now_t)
        return self._follow_tick(dri, boat_xy, yaw, vel_xy, yaw_rate,
                                 wp_xy, now_t)

    def _enter_escape(self, now_t: float, ev: str) -> StepOut:
        self.state = 'ESCAPE'
        self.plan = None           # ESCAPE 는 plan 없이 산다 — 복귀 시 새로
        self._escape_t0 = now_t
        self._escape_ok = 0
        self._no_plan_t0 = self._stuck_t0 = None
        return StepOut(0.0, 0.0, self.state, None, ev)

    def _escape_tick(self, dri, boat_xy, yaw, vel_xy, yaw_rate, wp_xy,
                     now_t: float) -> StepOut:
        """ESCAPE (§4): 후진 + 후방 argmin 조향, 매 tick generate 재시도.
        복귀는 **최소 이탈 시간 + 연속 성공** 둘 다 — 후진→p1 수축→generate
        재성공→전진→재막힘 채터링의 이중 차단 (확정 설계)."""
        f = self.fsm
        new = generate(dri, boat_xy, yaw, vel_xy, wp_xy, self.p, now_t)
        self._escape_ok = self._escape_ok + 1 if new is not None else 0
        if (new is not None
                and now_t - self._escape_t0 >= f.escape_min_t
                and self._escape_ok >= f.escape_n_ok):
            self.state = 'FOLLOW'
            self.plan = new
            return StepOut(0.0, 0.0, self.state, new, 'escape_recovered')
        lin, ang, surrounded = escape_cmd(dri, boat_xy, yaw, yaw_rate,
                                          self.p, f, self.yaw_kp, self.yaw_kd)
        return StepOut(lin, ang, self.state, None,
                       'escape_surrounded' if surrounded else None)

    def _follow_tick(self, dri, boat_xy, yaw, vel_xy, yaw_rate,
                     wp_xy, now_t: float) -> StepOut:
        f = self.fsm

        # ── plan 없음 (활성화 직후 포함): 정지 + 확보 시도, t_retry → ESCAPE.
        # 성공해도 이번 tick 은 정지 — 추종은 다음 tick 부터 (§4 흐름 그대로).
        if self.plan is None:
            self.plan = generate(dri, boat_xy, yaw, vel_xy, wp_xy, self.p, now_t)
            if self.plan is not None:
                self._no_plan_t0 = None
                return StepOut(0.0, 0.0, self.state, self.plan, 'plan_acquired')
            if self._no_plan_t0 is None:
                self._no_plan_t0 = now_t
            if now_t - self._no_plan_t0 > f.t_retry:
                return self._enter_escape(now_t, 'escape_no_plan')
            return StepOut(0.0, 0.0, self.state, None, 'generate_none')

        ev = None
        slow = False
        violation = check(self.plan, dri, boat_xy, self.p)   # progress 갱신 겸용

        if violation is None:
            self._stuck_t0 = None
            # ── 재생성 트리거 (generate ≤ 1회/tick, None 이면 기존 유지 —
            # 아직 check 통과 중이라 안전 구멍 아님):
            #  (a)  진행 반경 > 저장 r1 — 기준 장애물 통과
            #  (a') 현재 r1 < 저장 r1 − δ — 기준 장애물 교체. 저장 None ∧
            #       현재 유한(∞→유한 = 급락)도 발동
            #  (c)  백스톱 주기 경과
            trig = None
            prog_r = math.hypot(boat_xy[0] - self.plan.origin[0],
                                boat_xy[1] - self.plan.origin[1])
            r1_now = pick_ref_obstacle(dri.occ_xy, boat_xy, yaw, self.p)
            if self.plan.r1 is not None and prog_r > self.plan.r1:
                trig = 'trigger_a_regen'
            elif r1_now is not None and (self.plan.r1 is None
                                         or r1_now < self.plan.r1 - f.delta_r1):
                trig = 'trigger_a2_regen'
            elif now_t - self.plan.created_t >= f.t_backstop:
                trig = 'backstop_regen'
            if trig is not None:
                new = generate(dri, boat_xy, yaw, vel_xy, wp_xy, self.p, now_t)
                if new is not None:
                    self.plan, ev = new, trig
        else:
            # ── 위반 사다리: repair(≤repair_max) → generate → 긴급도 분기
            for _ in range(self.p.repair_max):
                if not repair(self.plan, dri, violation, self.p):
                    break                      # 8후보 전수 개선 없음 → 재생성으로
                violation = check(self.plan, dri, boat_xy, self.p)
                if violation is None:
                    ev = 'violation_repaired'
                    self._stuck_t0 = None
                    break
            if violation is not None:
                new = generate(dri, boat_xy, yaw, vel_xy, wp_xy, self.p, now_t)
                if new is not None:
                    self.plan, ev = new, 'violation_regen'
                    self._stuck_t0 = None
                else:
                    # 수리도 재생성도 실패 — 남은 여유(거리+누적 시간)로 분기
                    if self._stuck_t0 is None:
                        self._stuck_t0 = now_t
                    d_viol = (violation - self.plan.progress_idx) * self.plan.spacing
                    if (d_viol < f.d_panic
                            or now_t - self._stuck_t0 > f.t_retry):
                        return self._enter_escape(now_t, 'escape_panic')
                    ev, slow = 'violation_wait', True   # 감속 + 다음 tick 재시도

        lin, ang = follow_cmd(self.plan, boat_xy, yaw, yaw_rate, wp_xy,
                              f, self.yaw_kp, self.yaw_kd)
        if slow:
            # 위반 지점까지는 안전(check 가 보장) — 기어가며 수리 기회를 번다
            lin = min(lin, speed_to_cmd(f.v_min, f))
        return StepOut(lin, ang, self.state, self.plan, ev)
