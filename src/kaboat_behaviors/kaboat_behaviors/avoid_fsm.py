"""avoid_fsm — 회피 조종 정책 순수 라이브러리 (rclpy 무의존, AVOIDANCE.MD §3-4).

경계 규칙: dri = 위험 표현, bspline_planner = 경로 기하, **이 모듈 = 조종
정책** — 시간과 상태(FOLLOW/ESCAPE, plan 보유)를 유일하게 갖고, 스플라인
내부는 모른다. 노드(obstacle_planner)는 배선만. rclpy 무의존인 이유는
전이·타이머 로직을 가짜 시계 + 합성 장면 루프로 검증하기 위해서다
(위험도·경로 라이브러리의 설계 오류를 전부 잡은 방식) — 실물에선
rosbag tick 재생 디버깅이 된다.

구성:
  follow_cmd  — 경로 추종 (lookahead 점 PD 조향 + waypoint 캡 + 곡률 감속)
  Trail       — 지나온 궤적 기록 (FOLLOW 중 odom 위치를 일정 간격으로 저장)
  retrace_cmd — 후진 탈출: 기록한 궤적을 거꾸로 되짚는다 (선미 기준 조향)
  AvoidFsm    — 상태기계. 따라가기 사다리(검사→수리→재생성→긴급도 분기)와
                탈출 진입·복귀(최소 이탈 + 연속 성공) 판정을 갖는다.

후진 방향을 궤적으로 정하는 이유 (2026-11-10): 선체 라이다는 선미 120°
(onboard_scan_filter 의 120~240°)를 못 본다. occupancy_grid 는 안 보이는 칸을
갱신하지 않고, DRI 는 unknown 을 빈 물로 본다. 예전 escape_pick(후방 선분 DRI
argmin)은 그 사각지대를 근거로 방향을 골라, 제자리 회전 뒤처럼 선미 쪽이
한 번도 안 본 칸이면 unknown(=위험 0) 쪽으로 후진할 수 있었다. 배가 실제로
지나온 길은 그 자리가 비어 있었다는 물리적 증거이고 경기장 부표는 움직이지
않으므로, 그 길만 거꾸로 되짚는다.
"""
from collections import deque
from dataclasses import dataclass, replace
import inspect
import math

import numpy as np

from .bspline_planner import (
    Plan, PlannerParams, SampledPlan, check, curvature_ahead, generate, pick_ref_obstacle,
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
    # 경로 추종 상수 slow_radius 는 수조에서 검증한 LOS(LosFollower.bspline_track) 와 같은
    # 값 (2026-11-04 실물 튜닝). lookahead 는 LOS 의 1.0 에서 0.8 로 줄임 (2026-11-06,
    # 회피 경로를 더 바짝 따라가게). LOS 는 자기 lookahead(1.0)를 따로 지정해 영향 없음.
    # 이전 sim 값: lookahead 2.0, slow_radius 3.0, cmd_full 12/118.6.
    lookahead: float = 0.8        # follow 목표점 전방 호길이 [m]
    kappa_window: float = 2.0     # 📝 곡률 감속 전방 창 [m]
    v_min: float = 0.3            # 📝 곡률 감속 하한 [m/s]
    v_max: float = 1.4            # 📝 현재 회피 추종 상한 [m/s] (교정 전속은 v_full)
    omega_max: float = 0.52       # ✅ 실측 최대 요레이트 [rad/s] — v = ω_max/κ
    slow_radius: float = 1.5      # 목표 근접 감속 반경 [m]
    # 📝 재생성 트리거 (c) 주기 [s]. 3.0 → 5.0 (2026-11-12 영상): FOLLOW 중 재계획·수리가 평균
    # 2.8 s(2:21~2:50 은 2.4 s)마다라 경로가 자주 갈아엎혔다. 멀쩡한 경로는 더 오래 따라간다.
    t_backstop: float = 5.0
    d_panic: float = 2.0          # 📝 위반까지 경로거리 이 값 미만이면 즉시 ESCAPE [m]
    #   근거: 전속 1.48 에서 타력 감쇠 실측(3s 에 0.11 로) → 제동거리 ~2m + 여유 (예전 3.0).
    #   3.0 → 2.0 (2026-11-12): 붐이 추가된 회피 코스가 촘촘해 3 m 안에 장애물이 늘 있어
    #   수리·재생성을 시도할 틈 없이 후진 탈출이 잦을 것으로 봄. 회피 추종 출력 상한(0.25)
    #   에서는 전속보다 느려 제동거리가 짧다. 빠르게 달리는 구간에서는 여유가 없음에 주의.
    t_retry: float = 5.0          # 📝 plan 재확보/위반 교착 누적 상한 [s] → ESCAPE
    delta_r1: float = 1.0         # 📝 트리거 (a') r1 급락 판정 폭 [m]
    # 트리거 (a) 는 기준 장애물이 이 거리 [m] 이상일 때만 — 선체 옆 물보라·바로 옆 부표(r1 < 1.5 m)를
    # 지나는 순간 재계획하면 그 장애물이 코앞인 채로 새 경로를 만들어, 영상(2026-11-12)에서
    # trigger_a 1 s 안에 ESCAPE 가 다섯 번 났다. 가까운 기준은 백스톱이 맡는다.
    trigger_a_min_r1: float = 1.5
    # 트리거 (a') 는 더 가까운 장애물이 이 시간 [s] 이상 계속 보여야 — 물보라 칸 하나의 깜빡임으로
    # 재계획하지 않게. 0 = 즉시 (예전)
    trigger_a2_hold: float = 0.5
    # ESCAPE 는 FOLLOW 중 기록한 궤적을 거꾸로 되짚는다 (모듈 docstring 참고).
    trail_spacing: float = 0.1       # 궤적 기록 간격 [m] — 이만큼 움직일 때마다 1점
    trail_max_length: float = 10.0   # 보관할 궤적 길이 [m] — 넘으면 오래된 점부터 버림
    retrace_lookahead: float = 0.8   # 되짚기 조향 목표점 거리 [m] (follow lookahead 와 같음)
    retrace_end_tolerance: float = 0.2   # 궤적 끝까지 이 거리 남으면 정지 [m]
    # ESCAPE 후진 출력 비율 (직접 지정). 예전엔 v_escape 0.5 m/s 를 속도 곡선으로 바꿨는데
    # 실물 cmd_full(0.23) 기준 0.04 밖에 안 나와 수조에서 배가 움직이지 않았다
    # (2026-11-06 실물 튜닝, 수조 테스트에 ESCAPE 를 켜면서 0.1 로 정함).
    escape_thrust: float = 0.1
    # 제자리 정렬: 선수가 다음 목표점(lookahead) 방향에서 이 각도 [rad] 넘게 벗어나면 전진 0 —
    # 먼저 제자리 회전으로 정렬한 뒤 나간다. 예전 규칙(>60° 에서 0.3·v_max 저속 전진)은 12~60°
    # 오차에서 전진하며 도는 동안 벽 쪽으로 밀렸다. 정지한 배는 경로가 첫 경로점 쪽으로 곧게
    # 시작하므로(PlannerParams.stationary_surge) 출발 때 이 각도를 자주 넘는다 (2026-11-12).
    # 회피 FSM 만 쓴다 — LOS·B-spline 시험의 follow_cmd 는 예전 규칙 그대로.
    pivot_angle: float = math.radians(35.0)
    # 멈춤 확인 [s]: d_panic 위반(수리·재생성 실패)이 걸리면 바로 후진하지 않고 전진·회전 0 으로
    # 멈춘 채 이만큼 매 tick 다시 본다 — 풀리면 FOLLOW, 계속되면 ESCAPE. 2026-11-12 영상 분석:
    # 선수 물보라는 짧은 것 0.1~0.4 s(지도에서 걸러짐), ESCAPE 를 낸 것은 시작부터 1.0~1.7 s 였고
    # 새로 보인 실제 부유물·부표도 1.7~2.7 s 찍혀 "찍힌 시간"만으론 못 가른다. 그래서 장애물 판단을
    # 늦추는 대신 멈춘 채 확인 — 실제 장애물이어도 전진하지 않아 안전 (0.7 m/s 에서 타력 ≈0.4 m,
    # d_panic 2 m 안). 물보라 칸은 빔이 지나가면 스캔마다 0.3 씩 깎여 보통 0.3~1 s 에 95 아래.
    # 1.5 s: 감속하는 동안(배가 미끄러지는 동안 물보라가 계속)까지 덮어야 한다 — 감사 시뮬레이션
    # (물보라 모델 180 s): 0 s ESCAPE 38~45 번·못 나감, 1.0 s 25~36 번·못 나감, 1.5·2.0 s 0 번·도착.
    # 물보라 없는 붐·벽 장면은 0~2 s 모두 같은 결과. 0 = 즉시 ESCAPE (예전).
    # 예전 "ESCAPE 전 1.5 s 지속 확인"은 그동안 계속 달려서 뺐었다(AVOIDANCE.MD) — 이건 멈춘 채 확인
    escape_confirm_t: float = 1.5
    escape_min_t: float = 3.0     # 📝 최소 이탈 시간 [s] — 즉시 복귀 채터링 차단
    escape_n_ok: int = 5          # 📝 복귀 조건: generate 연속 성공 tick 수
    # v_full 을 내는 추력 명령 = 회피 **전진** 출력 비율. 회피 노드는 이 값을 전진
    # 출력 상한으로도 쓴다. 회전 상한은 노드 쪽 별도 값(0.31)이다.
    # 2026-11-06 실물 튜닝: 0.3(직선 전진 0.27)이면 꺾이는 회피 경로에서 관성으로
    # 바깥으로 밀려 추종을 못 했다. 0.15·0.2·0.23 을 거쳐 0.25(직선 전진 0.23, 수조 SOTA).
    cmd_full: float = 0.25
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


def follow_cmd(plan: Plan | SampledPlan, boat_xy, yaw: float, yaw_rate: float, wp_xy,
               fsm: FsmParams, yaw_kp: float, yaw_kd: float,
               pivot_angle: float | None = None) -> tuple[float, float]:
    """경로 추종 명령 (linear_x, angular_z) — PD 조향 + 곡률 feedforward 감속.

    조향은 공통 waypoint 와 같은 PD(게인도 노드가 같은 값 주입), 목표점만
    경로 위 lookahead 점. 감속은 전방 창 max κ 로 v = clamp(ω_max/κ, v_min,
    v_max) — κ_avail = ω/v 라 교정 전속(1.48)에선 0.35, 현재 회피 상한
    1.1m/s에서도 0.47뿐인데 경로는 κ 0.9~1.2가 필요해(실측) 조향만으론
    물리적으로 못 따라간다.
    heading 오차 비례 감속 항은 v1 제외 — 위치 waypoint 의 이산 규칙(오차 >60°
    면 저속)만 재사용한다 (제자리 급회전 = 전복 위험 차단, SKELETON §6 — sim 소형선 기준).
    pivot_angle 을 주면(회피 FSM, 2026-11-12) 오차가 그 각도를 넘을 때 전진 0 — 실물은
    텔레옵에서도 제자리 회전(좌 −·우 +)을 쓴다. None 이면 예전 규칙.
    """
    boat = np.asarray(boat_xy, dtype=np.float64)
    closed = isinstance(plan, SampledPlan) and plan.closed
    if closed:
        wp_scale = 1.0
    else:
        wp = np.asarray(wp_xy, dtype=np.float64)
        wp_dist = float(np.hypot(*(wp - boat)))

    # 목표점: 진행 위치 + lookahead 호길이 앞 샘플. waypoint 근접 시 wp 로
    # 캡 — r_end 가 min_horizon 으로 클램프돼 경로가 wp 를 지나쳐 뻗는 문제
    # 차단 (도착·완료 보고는 obstacle_planner/mission_manager 담당).
    if closed:
        steps = min(max(1, int(round(fsm.lookahead / plan.spacing))),
                    max(1, len(plan.samples) // 4))
        idx = (plan.progress_idx + steps) \
              % len(plan.samples)
        target = plan.samples[idx]
    elif wp_dist <= fsm.lookahead:
        target = wp
    else:
        idx = min(plan.progress_idx + max(1, int(round(fsm.lookahead / plan.spacing))),
                  len(plan.samples) - 1)
        target = plan.samples[idx]
        if float(np.hypot(*(target - boat))) < 0.3:
            # 경로 끝에 도달했는데 재생성이 계속 실패한 퇴화 상황 —
            # 방위가 정의되도록 wp 를 임시 목표로 (안전은 check 가 보장)
            target = wp

    kappa = curvature_ahead(plan, fsm.kappa_window)
    v = fsm.v_max if kappa < 1e-6 else min(max(fsm.omega_max / kappa,
                                               fsm.v_min), fsm.v_max)
    if not closed:
        wp_scale = min(wp_dist / fsm.slow_radius, 1.0)
    v *= wp_scale  # waypoint 근접 감속

    direction = target - boat
    norm = float(np.hypot(*direction))
    if norm <= 1e-12:
        desired = DesiredVelocity(0.0, 0.0)
    else:
        desired = DesiredVelocity(
            vx=float(v * direction[0] / norm),
            vy=float(v * direction[1] / norm),
        )
    params = _controller_params(fsm, yaw_kp, yaw_kd)
    # 기존 순서: 큰 오차 속도캡 후 waypoint 감속. 두 조건이 동시에
    # 걸려도 refactor 전과 같은 명령이 되도록 캡에도 wp_scale을 적용한다.
    turn_limit = 0.3 * fsm.v_max * wp_scale
    if pivot_angle is not None:
        params = replace(params, turn_slow_angle=pivot_angle)
        turn_limit = 0.0              # 오차 > pivot_angle → 전진 0, 제자리 회전으로 정렬
    out = control_velocity(desired, yaw, yaw_rate, params, turn_speed_limit=turn_limit)
    return out.linear_x, out.angular_z


RETRACE_SPACING = 0.05   # 되짚기 경로 샘플 간격 [m] (LOS·회피 경로와 같은 0.05)


class Trail:
    """FOLLOW 중 지나온 odom 위치 — ESCAPE 가 거꾸로 되짚을 길.

    spacing 이상 움직였을 때만 한 점을 더하므로 제자리 회전·정지 중엔 늘지 않는다.
    누적 길이가 max_length 를 넘으면 가장 오래된 점부터 버린다.
    """

    def __init__(self, spacing: float, max_length: float):
        if not (spacing > 0.0 and max_length > spacing):
            raise ValueError('trail spacing must be positive and below max_length')
        self.spacing = float(spacing)
        self.max_length = float(max_length)
        self.points: deque = deque()   # 오래된 점 → 최신 점, np.ndarray(2,)
        self._length = 0.0

    def clear(self) -> None:
        self.points.clear()
        self._length = 0.0

    def record(self, boat_xy) -> None:
        point = np.asarray(boat_xy, dtype=np.float64)
        if point.shape != (2,) or not np.all(np.isfinite(point)):
            return
        if self.points:
            step = float(np.hypot(*(point - self.points[-1])))
            if step < self.spacing - 1e-9:
                return
            self._length += step
        self.points.append(point)
        while self._length > self.max_length + 1e-9 and len(self.points) > 2:
            dropped = self.points.popleft()
            self._length -= float(np.hypot(*(self.points[0] - dropped)))

    def length(self) -> float:
        return self._length

    def retrace_samples(self, boat_xy, spacing: float) -> np.ndarray | None:
        """현재 위치 → 최신 점 → … → 가장 오래된 점을 spacing 간격으로 고른 샘플.

        되짚을 길이 spacing 보다 짧으면 None.
        """
        if not self.points:
            return None
        boat = np.asarray(boat_xy, dtype=np.float64)
        pts = np.vstack([boat[None, :]] + list(reversed(self.points)))
        seg = np.hypot(*np.diff(pts, axis=0).T)
        keep = np.concatenate([[True], seg > 1e-9])   # 겹친 점 제거 (보간 분모 0 방지)
        pts = pts[keep]
        if len(pts) < 2:
            return None
        arc = np.concatenate([[0.0], np.cumsum(np.hypot(*np.diff(pts, axis=0).T))])
        if arc[-1] < spacing:
            return None
        s = np.append(np.arange(0.0, arc[-1], spacing), arc[-1])
        return np.stack([np.interp(s, arc, pts[:, 0]),
                         np.interp(s, arc, pts[:, 1])], axis=1)

    def keep_older_than(self, samples: np.ndarray, idx: int) -> None:
        """되짚기를 samples[idx] 까지 마쳤다 — 그 앞(이미 되짚은 최신 쪽)을 버린다."""
        older = samples[idx:][::-1]        # 오래된 점 → 지금 위치 (시간순)
        self.clear()
        for point in older:
            self.record(point)


def retrace_progress(samples: np.ndarray, idx: int, boat_xy,
                     spacing: float) -> int:
    """되짚기 진행 인덱스 — 앞쪽 창에서 가장 가까운 샘플, 뒤로는 안 간다."""
    boat = np.asarray(boat_xy, dtype=np.float64)
    back = max(1, int(math.ceil(0.25 / spacing)))
    ahead = max(1, int(math.ceil(1.5 / spacing)))
    lo = max(0, idx - back)
    hi = min(len(samples) - 1, idx + ahead)
    window = samples[lo:hi + 1]
    nearest = lo + int(np.argmin(np.hypot(*(window - boat).T)))
    return max(idx, nearest)


def retrace_cmd(samples: np.ndarray, idx: int, boat_xy, yaw: float,
                yaw_rate: float, spacing: float, fsm: FsmParams,
                yaw_kp: float, yaw_kd: float) -> tuple[float, float]:
    """후진 + 궤적 위 lookahead 점으로 선미를 돌리는 조향 (linear_x, angular_z).

    기준각은 **선미**: 오차 = wrap(target − (yaw+π)) — 후진 중엔
    꽁무니가 진행 방향이므로. 차동추력의 요 모멘트는 속도 부호와 무관해
    조향 부호는 전진과 동일 (2026-07-18 sim 실측: wz +0.44 vs +0.49).
    """
    boat = np.asarray(boat_xy, dtype=np.float64)
    steps = max(1, int(round(fsm.retrace_lookahead / spacing)))
    target = samples[min(idx + steps, len(samples) - 1)]
    direction = target - boat
    if float(np.hypot(*direction)) < 1e-6:
        return 0.0, 0.0
    desired = DesiredVelocity(
        vx=float(direction[0]), vy=float(direction[1]), reverse=True)
    out = control_velocity(
        desired, yaw, yaw_rate, _controller_params(fsm, yaw_kp, yaw_kd))
    # 조향은 선미 기준 PD 그대로, 후진 출력은 escape_thrust 를 직접 쓴다
    return -fsm.escape_thrust, out.angular_z


class AvoidFsm:
    """조종 정책 상태기계. 노드가 tick 마다 step() 을 부르고 결과를 발행한다.

    모든 경로가 cmd 를 반환한다 — cmd_mux 300ms 워치독에 걸리지 않도록
    tick 당 발행 1회는 노드가 보장하되, 여기서 None 을 내지 않는 게 전제.
    """

    def __init__(self, planner: PlannerParams, fsm: FsmParams,
                 yaw_kp: float, yaw_kd: float, generator=None):
        self.p = planner
        self.generate = generate if generator is None else generator
        # 재계획 때 지금 경로를 prev_plan 으로 넘길 수 있는 생성 함수인가 (w_hist — 같은 쪽 유지)
        try:
            params = inspect.signature(self.generate).parameters.values()
            self._gen_takes_prev = any(
                q.name == 'prev_plan' or q.kind is inspect.Parameter.VAR_KEYWORD for q in params)
        except (TypeError, ValueError):
            self._gen_takes_prev = False
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
        self._panic_t0 = None      # 멈춤 확인(escape_confirm_t) 시작 시각
        self._escape_t0 = None     # ESCAPE 진입 시각 — 최소 이탈 판정
        self._escape_ok = 0        # ESCAPE 중 generate 연속 성공 tick 수
        self.escape_detail = ''    # 마지막 ESCAPE 진입 이유 (노드 로그용 — 위반 거리·막힌 시간)
        self._a2_since = None      # 트리거 (a') 조건이 처음 참이 된 시각 — trigger_a2_hold 판정
        # 지나온 궤적 — 재활성화 때 지운다 (이전 activation 의 길은 근거가 아님)
        self.trail = Trail(self.fsm.trail_spacing, self.fsm.trail_max_length)
        self._retrace = None       # ESCAPE 되짚기 샘플 (지금 위치 → 오래된 점)
        self._retrace_idx = 0
        self._retrace_stopped = False   # 궤적 없음/끝 — 정지 이벤트 1회만

    # ── tick ─────────────────────────────────────────
    def step(self, dri, boat_xy, yaw: float, vel_xy, yaw_rate: float,
             wp_xy, now_t: float) -> StepOut:
        """tick 진입점 (§4). dri 는 노드가 매 tick build_dri 로 재계산해 넘긴다."""
        if self.state == 'ESCAPE':
            # 되짚는 중엔 기록하지 않는다 — 되짚은 만큼은 복귀 때 잘라낸다
            return self._escape_tick(dri, boat_xy, yaw, vel_xy, yaw_rate,
                                     wp_xy, now_t)
        self.trail.record(boat_xy)
        return self._follow_tick(dri, boat_xy, yaw, vel_xy, yaw_rate,
                                 wp_xy, now_t)

    def _enter_escape(self, now_t: float, ev: str, boat_xy, detail: str = '') -> StepOut:
        self.state = 'ESCAPE'
        self.escape_detail = detail
        self.plan = None           # ESCAPE 는 plan 없이 산다 — 복귀 시 새로
        self._escape_t0 = now_t
        self._panic_t0 = None
        self._escape_ok = 0
        self._no_plan_t0 = self._stuck_t0 = None
        # 진입 순간 위치 → 최신 점 → … → 가장 오래된 점. 없으면 None (정지)
        self._retrace = self.trail.retrace_samples(boat_xy, RETRACE_SPACING)
        self._retrace_idx = 0
        self._retrace_stopped = False
        return StepOut(0.0, 0.0, self.state, None, ev)

    def _escape_tick(self, dri, boat_xy, yaw, vel_xy, yaw_rate, wp_xy,
                     now_t: float) -> StepOut:
        """ESCAPE (§4): 지나온 궤적을 거꾸로 후진, 매 tick generate 재시도.
        복귀는 **최소 이탈 시간 + 연속 성공** 둘 다 — 후진→p1 수축→generate
        재성공→전진→재막힘 채터링의 이중 차단 (확정 설계).

        궤적이 없거나(활성화 직후 포위 등) 끝까지 되짚었으면 정지한 채로
        generate 만 재시도한다 — 선미 사각지대(라이다 120°)로 근거 없이
        후진하지 않는다."""
        f = self.fsm
        new = self._new_plan(dri, boat_xy, yaw, vel_xy, wp_xy, now_t)
        self._escape_ok = self._escape_ok + 1 if new is not None else 0
        if self._retrace is not None:
            self._retrace_idx = retrace_progress(
                self._retrace, self._retrace_idx, boat_xy, RETRACE_SPACING)
        if (new is not None
                and now_t - self._escape_t0 >= f.escape_min_t
                and self._escape_ok >= f.escape_n_ok):
            if self._retrace is not None:
                # 되짚은 구간은 버리고, 남은(더 오래된) 길만 다음 ESCAPE 용으로 둔다
                self.trail.keep_older_than(self._retrace, self._retrace_idx)
                self.trail.record(boat_xy)
            self.state = 'FOLLOW'
            self.plan = new
            return StepOut(0.0, 0.0, self.state, new, 'escape_recovered')

        if self._retrace is None:
            return self._escape_hold('escape_no_trail')
        remaining = (len(self._retrace) - 1 - self._retrace_idx) * RETRACE_SPACING
        if remaining <= f.retrace_end_tolerance:
            return self._escape_hold('escape_trail_end')
        lin, ang = retrace_cmd(self._retrace, self._retrace_idx, boat_xy, yaw,
                               yaw_rate, RETRACE_SPACING, f,
                               self.yaw_kp, self.yaw_kd)
        return StepOut(lin, ang, self.state, None, None)

    def _regenerate(self, dri, boat_xy, yaw, vel_xy, wp_xy, now_t):
        """FOLLOW 중 재계획 — 지금 경로를 prev_plan 으로 넘겨 같은 쪽을 유지 (w_hist).
        생성 함수가 prev_plan 을 못 받으면(테스트용 대체 함수 등) 예전처럼."""
        if self._gen_takes_prev:
            return self._new_plan(dri, boat_xy, yaw, vel_xy, wp_xy, now_t, prev_plan=self.plan)
        return self._new_plan(dri, boat_xy, yaw, vel_xy, wp_xy, now_t)

    def _new_plan(self, dri, boat_xy, yaw, vel_xy, wp_xy, now_t, **kw):
        """generate 결과를 같은 tick check() 에 통과할 때만 쓴다 — 아니면 None (생성 실패와 같음).
        2026-11-12 감사: 위반 재생성·트리거·백스톱·ESCAPE 복귀가 새 경로를 검사 없이 채택해,
        생성기가 낸 위반 경로(무작위 장면 6 %)를 배가 그대로 따라갔다. bspline_planner.generate 는
        이제 위반 경로를 내지 않지만, 대체 생성 함수(도킹 등)까지 같은 선으로 막는다."""
        new = self.generate(dri, boat_xy, yaw, vel_xy, wp_xy, self.p, now_t, **kw)
        if new is not None and check(new, dri, boat_xy, self.p) is not None:
            return None
        return new

    def _escape_hold(self, ev: str) -> StepOut:
        """되짚을 길이 없음 — 정지하고 generate 재시도만 (이벤트는 처음 1회)."""
        first = not self._retrace_stopped
        self._retrace_stopped = True
        return StepOut(0.0, 0.0, self.state, None, ev if first else None)

    def _follow_tick(self, dri, boat_xy, yaw, vel_xy, yaw_rate,
                     wp_xy, now_t: float) -> StepOut:
        f = self.fsm

        # ── plan 없음 (활성화 직후 포함): 정지 + 확보 시도, t_retry → ESCAPE.
        # 성공해도 이번 tick 은 정지 — 추종은 다음 tick 부터 (§4 흐름 그대로).
        if self.plan is None:
            self.plan = self._new_plan(dri, boat_xy, yaw, vel_xy, wp_xy, now_t)
            if self.plan is not None:
                self._no_plan_t0 = None
                return StepOut(0.0, 0.0, self.state, self.plan, 'plan_acquired')
            if self._no_plan_t0 is None:
                self._no_plan_t0 = now_t
            if now_t - self._no_plan_t0 > f.t_retry:
                return self._enter_escape(
                    now_t, 'escape_no_plan', boat_xy,
                    f'경로 없음 {now_t - self._no_plan_t0:.1f} s (t_retry {f.t_retry:.1f} s)')
            return StepOut(0.0, 0.0, self.state, None, 'generate_none')

        ev = None
        slow = False
        violation = check(self.plan, dri, boat_xy, self.p)   # progress 갱신 겸용

        if violation is None:
            self._stuck_t0 = self._panic_t0 = None
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
            a2 = r1_now is not None and (self.plan.r1 is None
                                         or r1_now < self.plan.r1 - f.delta_r1)
            if not a2:
                self._a2_since = None
            elif self._a2_since is None:
                self._a2_since = now_t
            if (self.plan.r1 is not None and self.plan.r1 >= f.trigger_a_min_r1
                    and prog_r > self.plan.r1):
                trig = 'trigger_a_regen'
            elif a2 and now_t - self._a2_since >= f.trigger_a2_hold:
                trig = 'trigger_a2_regen'
            elif now_t - self.plan.created_t >= f.t_backstop:
                trig = 'backstop_regen'
            if trig is not None:
                new = self._regenerate(dri, boat_xy, yaw, vel_xy, wp_xy, now_t)
                if new is not None:
                    self.plan, ev = new, trig
                    self._a2_since = None
        else:
            # ── 위반 사다리: repair(≤repair_max) → generate → 긴급도 분기
            for _ in range(self.p.repair_max):
                if not repair(self.plan, dri, violation, self.p):
                    break                      # 8후보 전수 개선 없음 → 재생성으로
                violation = check(self.plan, dri, boat_xy, self.p)
                if violation is None:
                    ev = 'violation_repaired'
                    self._stuck_t0 = self._panic_t0 = None
                    break
            if violation is not None:
                new = self._regenerate(dri, boat_xy, yaw, vel_xy, wp_xy, now_t)
                if new is not None:
                    self.plan, ev = new, 'violation_regen'
                    self._stuck_t0 = self._panic_t0 = None
                else:
                    # 수리도 재생성도 실패 — 남은 여유(거리+누적 시간)로 분기
                    if self._stuck_t0 is None:
                        self._stuck_t0 = now_t
                    d_viol = (violation - self.plan.progress_idx) * self.plan.spacing
                    stuck = now_t - self._stuck_t0
                    if d_viol < f.d_panic or stuck > f.t_retry:
                        held = 0.0
                        if d_viol < f.d_panic and stuck <= f.t_retry and f.escape_confirm_t > 0.0:
                            # 멈춤 확인: 바로 후진하지 않고 멈춘 채 escape_confirm_t 동안 매 tick
                            # 다시 본다 — 그 사이 위반이 풀리면(물보라 칸이 깎임) FOLLOW 계속
                            first = self._panic_t0 is None
                            if first:
                                self._panic_t0 = now_t
                            held = now_t - self._panic_t0
                            if held < f.escape_confirm_t:
                                return StepOut(0.0, 0.0, self.state, self.plan,
                                               'panic_hold' if first else None)
                        return self._enter_escape(
                            now_t, 'escape_panic', boat_xy,
                            f'경로상 위반까지 {d_viol:.2f} m (d_panic {f.d_panic:.1f} m), '
                            f'멈춘 채 {held:.1f} s 확인, 수리·재생성 실패 {stuck:.1f} s')
                    self._panic_t0 = None
                    ev, slow = 'violation_wait', True   # 감속 + 다음 tick 재시도

        lin, ang = follow_cmd(self.plan, boat_xy, yaw, yaw_rate, wp_xy,
                              f, self.yaw_kp, self.yaw_kd, pivot_angle=f.pivot_angle)
        if slow:
            # 위반 지점까지는 안전(check 가 보장) — 기어가며 수리 기회를 번다
            lin = min(lin, speed_to_cmd(f.v_min, f))
        return StepOut(lin, ang, self.state, self.plan, ev)
