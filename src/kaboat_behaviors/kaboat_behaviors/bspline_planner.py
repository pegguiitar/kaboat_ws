"""bspline_planner — DRI 격자 위에서 B-spline 로컬 경로를 생성/검사/수리하는
순수 라이브러리 (AVOIDANCE.MD §3-2/§3-3 의 구현, ROS 의존 없음).

설계 요지:
  경로는 제어점 6개짜리 clamped cubic B-spline. p0(배)·p1(관성점)이 시작
  기하를 고정하고, p2~p5 는 배 중심 **동심원(arc) 위에서 방위만 고르는** 점
  이다 — 반지름이 단조증가라 "항상 바깥으로 전진"이 구조적으로 보장되고,
  수리(repair)는 반지름을 안 건드리므로 그 보장을 깨지 못한다.

  생성(generate)과 검사(check)는 분리 — 생성은 가끔, 검사는 매 tick
  최신 DRI 로. 후보 탐색은 waypoint 방위 ±search_fov/2 부채꼴 — 이론적
  최대치인 180°는 "waypoint 쪽 전진 성분 ≥ 0 인 반평면"(§3-2)이지만, 기본값은
  90°로 좁혀뒀다. wp_bearing 이 헤딩과 크게 벌어진 상황(급회전 직후 등)에서
  넓은 부채꼴이면 p2 가 헤딩 기준 거의 정반대까지 뽑혀 배가 옆/뒤로 도는
  것처럼 보이는 사례가 실측됐기 때문 — 부채꼴 중심은 여전히 wp_bearing 이라
  헤딩 자체는 탐색에 관여 안 하고, 폭만 좁혀 최악의 이탈각을 줄인 것.
  후보 고르기는 **하드 필터**: 직전 제어점→후보 선분의 DRI > safe_threshold·gen_margin
  후보는 제거한다. 생존자 중 먼저 방위 비용 최적점 주변(dri_angle_slack)으로 탐색
  범위를 제한한 뒤, 그 안에서 선분 최대 DRI 최저값과 risk_straight_slack 이내인 후보를
  추린다. 거의 같은 위험이면 그중 방위 비용이 가장 낮은 후보를 골라 직진성을
  회복한다. 전역 최소 DRI 를 바로 고르지 않으므로 빈 바다로 과도하게
  우회하지 않으면서 경계에 붙는 경로도 피한다.
  안전 판정은 **완성 곡선**으로 한다 (2026-11-12): 선분은 곡선의 근사라 곡선이 모서리를
  질러가거나 배→p1 구간이 비어, 예전엔 무작위 장면 6 % 가 check() 위반 경로였다. 이제
  다 고른 곡선을 check() 와 같은 선으로 검사하고, 위반이면 그 지점을 움직이는 arc 로
  되돌아가 옆 후보로 바꾼다. 최저 비용 곡선이 안전하면 예전과 같은 경로다. 안전한
  곡선이 없으면 None — 전진 반평면에 답이 없다는 뜻이고, 그 답은 ESCAPE(후진)다.

  t_look=1.0s 는 전속 1.48 m/s(Gazebo 시뮬 맞춤 — 야외 실측은 명령 0.40 → 1.06 m/s,
  los_controller.MEASURED_SPEED_*) 기준 재산출값(§3-2) — 5.0s 초안은
  p1 이 7.4m 로 튀어 스플라인이 나갔다 되돌아오는 첨점을 만들었다.
"""
from dataclasses import dataclass, field, replace
from functools import lru_cache
import math

import numpy as np
from scipy.interpolate import BSpline


@dataclass
class PlannerParams:
    """§3 파라미터 표의 planner.* — 노드가 ROS param 에서 채워 넘긴다."""
    t_look: float = 1.0            # p1 = surge·t_look [s] — §3-2 재산출값
    p1_min_dist: float = 1.0       # 정지/후진 시 접선 방향을 주기 위한 p1 하한 [m]
    bearing_max: float = math.radians(100.0)  # r1 후보 후방 하드컷
    # r1 방위 가중 — ρ_eff = ρ(1+k(1−cosΔθ)). 0.5 는 방금 지나친 측면(~85°)
    # 부표가 계속 뽑혀 첫 arc 가 협상 끝난 장애물에 묶였다(§3-2) — 1.5 면
    # 측면이 전방보다 2.4배 가까워야 뽑히고, 코앞 측면 위협은 여전히 이긴다.
    k_bear: float = 1.5
    # r_end = 격자 내접 최대 반지름 − 이 값 [m]. 1.0 → 0.5 (2026-07-18 사용자
    # 채택): 개활수역 전속 직선·밭 관통 sim 에서 가짜 경계 위반 0 실측 —
    # 동급 안전에 계획 지평선 +0.5m. 격자 recenter 지연이 커지면(실물 SLAM
    # 등) 재검토.
    boundary_margin: float = 0.5
    # waypoint 가 코앞이어도 r_end 하한 [m]. 3.0 → 2.0 (2026-11-04 실물 튜닝): 3 m 로 두면
    # 벽 앞 목표에서 마지막 제어점이 목표를 지나 벽 위에 놓여 NO PLAN 이 났다.
    # 퇴화 방지 하한(p1_len + 2·cp_min_gap = 2.0 m)이 따로 있어 2.0 보다 낮출 의미는 없다.
    finite_goal: bool = False     # marker mission: terminate at the goal, including near goals
    # 이 전진 속도 [m/s] 보다 느리면(정지·후진 직후) 배가 제자리 회전으로 방향을 바꿀 수 있다고
    # 보고 p1 을 선수가 아니라 첫 경로점(p2) 쪽에 둔다 — 경로가 "선수로 1 m 직진 → 급히 꺾기"
    # 대신 곧게 시작하고, 꺾임은 배 위치의 제자리 회전(FsmParams.pivot_angle)이 맡는다.
    # 2026-11-12 현장 영상 2:22~2:50: 정지한 배 앞 2.3 m 붐 — 매 경로가 선수로 1 m 간 뒤 40°+
    # 꺾여(반지름 1.4 m) 전진 8 %·회전 31 % → 제자리에서 흔들리며 28 s 못 나감. 0 = 끔(예전처럼 항상 선수)
    stationary_surge: float = 0.3
    min_horizon: float = 2.0
    cp_min_gap: float = 0.5        # r_end 퇴화 방지: r_end ≥ ‖p1‖ + 2·이 값 [m]
    # 후보 부채꼴 = wp 중심 ±search_fov/2. 원래 180°(wp 전진 반평면 전체, §3-2)
    # 였는데, wp_bearing 이 헤딩과 크게 벌어진 상황(급회전 직후 등)에서 p2 가
    # 헤딩 기준 거의 정반대까지 뽑혀 배가 옆/뒤로 도는 것처럼 보이는 사례가
    # 실측돼 120°→90°로 좁힘. 부채꼴 폭만 좁아지고 중심은 여전히
    # wp_bearing — 헤딩은 이 탐색에 전혀 관여하지 않는다.
    search_fov: float = math.radians(90.0)
    # 막힐 때만 넓히기 (2026-11-12): search_fov 안에서 안전 후보가 없으면 같은 tick 에
    # 이 폭으로 한 번 더 찾는다. 붐이 비스듬히 통로를 막아 목표 방향 ±45° 밖으로 돌아가야
    # 하는 회피 코스용. 평소(±45° 에 답이 있을 때)는 동작이 같다. ≤ search_fov 면 끔.
    search_fov_fallback: float = math.radians(180.0)
    n_candidates: int = 31
    # 후보 기하 비용 가중. 먼저 이 비용의 최적 안전 후보를 찾고, 그 후보의
    # 방위 ±dri_angle_slack 안에서만 DRI 최소 후보를 찾는다. 전역 DRI argmin
    # 시절처럼 CP 가 밭 밖 DRI=0 지대로 도망가는 것은 막되, 위험 경계에
    # 간신히 걸친 직선 후보를 무조건 고르는 것도 피한다.
    # w_prev 는 직전 CP 방향을 유지해 후보 방위 양자화가 만드는 arc 간
    # S 잔물결을 억제한다. 목표 방향과 동등하게 반영하도록 1.0.
    w_wp: float = 1.0
    w_prev: float = 1.0
    # 재계획 때 직전 경로 쪽 유지: 후보 비용 += w_hist·|후보 방위 − 직전 경로가 같은 반지름에서
    # 지나던 방위|. 재계획(트리거·백스톱·위반 재생성)은 매번 처음부터 만들어, 장애물 좌우
    # 비용이 비슷하면 작은 변화(선수·물보라 칸)에도 반대쪽으로 넘어갔다 — 2026-11-12 영상
    # 2:23.4 붐 아래 오른쪽 → 2:23.8 왼쪽 S 자. 안전 필터는 그대로라 그쪽이 막히면 넘어간다.
    w_hist: float = 1.0
    dri_angle_slack: float = math.radians(12.0)  # 기하 최적 후보 주변 DRI 탐색 반폭
    # 제한 범위의 최소 max DRI 와 이 값 이내면 위험이 사실상 같다고 보고
    # geom_cost 로 직진성을 고른다. 안전 기준선(1.0)의 3%에 해당한다.
    risk_straight_slack: float = 0.03
    # 유일한 안전 기준선 — 생성(후보 하드 필터)과 검사(check 위반)가 같은
    # 선을 쓴다. 후보는 선분 max DRI > 이 값이면 제거하고 제한 각도 안에서
    # 최소 DRI 근처 후보 중 가장 직선적인 것을 고른다. 어떤 arc 든 생존자
    # 0 이면 generate → None (구 hard_max 역할 흡수 — 생성이 "괜찮다"고
    # 낸 경로를 같은 tick check 가 위반 판정하던 자기모순 제거).
    # berth 관계: 거품 반경 = σ·√(2·ln(A/threshold)).
    # 1.0 → 1.2 (2026-11-04 실물 튜닝): 장애물 간격 ~2 m 에서 후보가 너무 적었다
    # (빈 틈 2 m 기준 후보 7/31 → 12/31, sigma 0.4 와 함께). 이전 값은 sim 튜닝값.
    safe_threshold: float = 1.2
    # 생성 필터는 threshold×이 값 에서 자른다 — 필터는 현(chord)으로 판정
    # 하는데 스플라인은 굽이에서 현보다 수 cm 안쪽으로 붙어, 1.0 그대로면
    # 경계 스침(실측 1.013)이 남는다. 0.85 = 위험 경사(≈2.7/m) 기준 ~5cm
    # 편차 흡수. check 는 margin 없이 threshold 그대로 본다.
    # 0.85 → 0.93 (2026-11-04): 장애물 간격 ~2 m 수조 배치. 무작위 400장면 합성 조사에서
    # 만들자마자 check 위반 수(1/396)와 틈 통과 간격이 0.85 와 같았다.
    gen_margin: float = 0.93
    repair_max: int = 3            # tick 당 수리 한도 (노드가 쓰는 값)
    repair_dtheta: float = math.radians(6.0)  # 수리 1회 방위 이동각
    progress_window: float = 2.0   # check 진행 갱신 전방 탐색 창 [m]


@dataclass
class Plan:
    cps: np.ndarray          # (6, 2) 제어점 world 좌표 [p0..p5]
    samples: np.ndarray      # (M, 2) spline 샘플 — 호길이 순, 간격 = spacing
    origin: tuple            # 계획 시점 배 위치 (= p0) — arc 중심이자 트리거 (a) 기준
    r1: float | None         # 기준 장애물 실거리 — 재생성 트리거 (a) 전용. 없으면 None
    radii: list              # p2~p5 arc 반지름 — ‖p1‖↔r_end 균등 사다리, 수리 불변
    bearings: list           # p2~p5 의 origin 기준 방위각 — 수리는 이 값만 이동
    created_t: float         # 생성 시각 [s] — 재생성 트리거 (c) 기준
    wp_bearing: float        # waypoint 방위 — 후보 부채꼴 중심 (수리 한계 판정용)
    spacing: float           # 샘플 간격 [m] (= 0.5·res)
    goal_locked: bool = False
    terminal_heading: float | None = None
    tail_samples: np.ndarray | None = None
    docking_phase: str = ''
    progress_idx: int = 0    # 배가 지나온 samples 진행 인덱스 — check 가 갱신(단조)
    search_fov: float | None = None  # 이 계획을 만든 부채꼴 폭 (넓혀 만들었으면 수리도 그 폭)


@dataclass
class SampledPlan:
    """미리 생성한 경로를 공통 follow_cmd로 추종할 때 쓰는 샘플 표현."""
    samples: np.ndarray
    curvatures: np.ndarray
    spacing: float
    progress_idx: int = 0
    closed: bool = False
    target_laps: float | None = None
    frame_id: str = 'odom'


def _wrap(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


# clamped cubic, 제어점 6개 — 양 끝점 통과, 시작 접선 = p0→p1 방향
_KNOTS = np.array([0, 0, 0, 0, 1 / 3, 2 / 3, 1, 1, 1, 1], dtype=np.float64)


_U_DENSE = np.linspace(0.0, 1.0, 300)
_BASIS = BSpline(_KNOTS, np.eye(6), 3)   # 매개변수 u → 제어점 6개의 가중치


def _dense_spline(cps: np.ndarray):
    """(조밀 곡선 300점, 누적 호길이)."""
    dense = np.asarray(BSpline(_KNOTS, cps, 3)(_U_DENSE))
    cum = np.concatenate([[0.0], np.cumsum(np.hypot(*np.diff(dense, axis=0).T))])
    return dense, cum


def _resample(dense, cum, spacing):
    """조밀 곡선 → 호길이 등간격 샘플 (끝점 포함). 샘플 i 의 호길이 = i·spacing."""
    targets = np.arange(0.0, cum[-1], spacing)
    xs = np.interp(targets, cum, dense[:, 0])
    ys = np.interp(targets, cum, dense[:, 1])
    return np.vstack([np.stack([xs, ys], axis=1), dense[-1:]])


def _sample_spline(cps: np.ndarray, spacing: float) -> np.ndarray:
    """spline 을 조밀 평가 후 호길이 등간격으로 재샘플 (끝점 포함)."""
    return _resample(*_dense_spline(cps), spacing)


def pick_ref_obstacle(occ_xy, boat, yaw, p: PlannerParams):
    """§3-2 기준 장애물 — 후방 하드컷 + 방위 가중 유효거리. 실거리 ρ 또는 None.

    공개 함수 — generate 내부용이면서, FSM 재생성 트리거 (a')(현재 r1 급락
    = 기준 장애물 교체 감지)가 generate 없이 매 tick 호출한다.
    """
    if occ_xy is None or len(occ_xy) == 0:
        return None
    d = occ_xy - boat
    rho = np.hypot(d[:, 0], d[:, 1])
    dth = np.abs(np.vectorize(_wrap)(np.arctan2(d[:, 1], d[:, 0]) - yaw))
    ok = (dth <= p.bearing_max) & (rho > 1e-6)
    if not np.any(ok):
        return None
    eff = rho[ok] * (1.0 + p.k_bear * (1.0 - np.cos(dth[ok])))
    return float(rho[ok][np.argmin(eff)])


def _argmin_toward_wp(cost, bearings, wp_bearing) -> int:
    """cost 최소 후보. 동점이면 wp 방위에 가장 가까운 후보.

    w_wp = w_prev 라 직전 CP 방위와 wp 방위 사이 후보는 기하 비용이 전부 같다.
    np.argmin 은 동점에서 배열 앞쪽(−search_fov/2, 오른쪽)을 고르므로, 오른쪽으로
    피한 경로는 직전 방위를 유지해 목표 옆으로 멀어지고 왼쪽 회피만 곧바로 wp 로
    복귀하는 비대칭이 있었다 (2026-11-04 실측: 목표 6 m 에서 끝점이 옆으로 3 m).
    동점을 wp 쪽으로 깨서 좌우 대칭으로 만든다.
    """
    best = float(np.min(cost))
    tied = np.nonzero(cost <= best + 1e-9)[0]
    return int(tied[np.argmin(np.abs(bearings[tied] - wp_bearing))])


def _plan_bearing_at(plan, boat, radius):
    """직전 경로가 지금 배 중심 반지름 radius 원을 처음 넘는 점의 방위 — 없으면 None."""
    if plan is None:
        return None
    ahead = plan.samples[max(0, int(getattr(plan, 'progress_idx', 0))):]
    if len(ahead) == 0:
        return None
    d = ahead - np.asarray(boat, dtype=np.float64)
    out = np.nonzero(np.hypot(d[:, 0], d[:, 1]) >= radius)[0]
    if out.size == 0:
        return None
    q = d[out[0]]
    return math.atan2(q[1], q[0])


# ── 되돌아가기 (2026-11-12) ── 무작위 1500 장면 감사에서 예전 생성은 6 %(82 개)가 check()
# 위반 경로였다 (현만 보고 곡선은 안 봄). 지금은 완성 곡선을 검사하고 위반이면 되돌아간다.
RANKED_PER_ARC = 5      # arc 마다 시도하는 후보 수 (1위 + 양옆으로 비킨 4개)
RANK_GAP = math.radians(6.0)   # 다음 순위 후보는 앞 순위들과 이 각도 이상 떨어진 것
# _generate_once 1회에 그려 검사하는 곡선 수 상한. 성공한 시도의 99 % 가 곡선 5개 이하, 최대 12개.
MAX_CURVES = 12
# generate() 1회의 작업 상한 = arc 후보 순위 계산 + 곡선 그리기 횟수 (각 노트북 ~0.2 ms).
# 정지한 배가 사방이 막히면 시작 방향 13개 × 부채꼴 2개를 다 훑느라 270 ms 까지 걸렸다 —
# FSM 은 경로 없음·ESCAPE 동안 매 tick(0.1 s) generate 하므로 최악 시간을 묶는다.
# 무작위 1500 장면 (노트북): 80 → 경로 없음 34·최대 23~27 ms, 120 → 같은 34·최대 33 ms
# (예전 생성: 경로 없음 55 + 위반 82, 최대 22 ms).
WORK_BUDGET = 80


class _Budget:
    """generate() 1회가 나눠 쓰는 작업 횟수."""

    def __init__(self, n):
        self.left = n

    def spend(self):
        self.left -= 1


_CHORD_TS = np.linspace(0.125, 1.0, 8)[:, None, None]


def _chord_risk(dri, start, ends):
    """start → 각 끝점 직선 위 최대 위험 (8점, start 자신 제외).

    후보 고르기용 근사일 뿐 — 안전 판정은 완성 곡선 검사가 한다. 0.1 m 간격으로 촘촘히
    보면 무작위 1500 장면 중 55 % 경로가 바뀌고(현장에서 맞춘 gen_margin 0.93 은 8점 기준),
    p1 이 위험 칸 안이라 현 시작이 막혀도 곡선은 안전한 장면 15 개를 잃었다 (2026-11-12)."""
    ends = np.atleast_2d(ends)
    seg = start[None, None, :] + _CHORD_TS * (ends[None, :, :] - start[None, None, :])
    return dri.risk_at_many(seg[..., 0].ravel(), seg[..., 1].ravel()).reshape(8, -1).max(axis=0)


@lru_cache(maxsize=8)
def _fan(fov: float, n: int) -> np.ndarray:
    """후보 방위 오프셋 −fov/2 … +fov/2 (n 개, 등간격)."""
    out = np.linspace(-0.5 * fov, 0.5 * fov, n)
    out.flags.writeable = False
    return out


def _arc_ranked(dri, origin, radius, wp_bearing, prev_pt, prev_bearing,
                p: PlannerParams, hist_bearing: float | None = None, k: int = 1):
    """arc 위 후보 순위 — 하드 필터 + 제한 범위 최소 DRI 근처 직진 우선. [(방위, 점)] 최대 k 개.

    ① **직전 CP → 후보를 잇는 선분**(8점, _chord_risk) 위 max DRI > safe_threshold·gen_margin
       인 후보 제거 (창 밖 inf 도 자연 탈락). 후보 점 하나만 보면 점은 거품 밖인데 거기까지
       가는 곡선이 거품을 가로지르는 구멍이 남는다 (정면 4 m 장애물 1개에서 위험 1.55 스침).
       선분도 곡선의 근사라 곡선이 모서리를 질러갈 수 있다 — 그건 완성 곡선 검사가 잡는다.
    ② 생존자 중 w_wp·|wp방위차| + w_prev·|직전CP방위차| (+ w_hist·|직전 경로 방위차|)
       가 최소인 기준 후보를 찾고, 그 방위 ±dri_angle_slack 로 탐색 범위를 제한
    ③ 제한 범위의 선분 max DRI 최저값 + risk_straight_slack 이내 후보 중
       기하 비용 argmin = 1위
    ②③ 모두 비용 동점이면 wp 방위에 가까운 후보 (_argmin_toward_wp)
    2위 이하 = 1위 양옆으로 RANK_GAP 이상 비킨 안전 후보 (가까운 것부터) — generate 가
    완성 곡선이 위반일 때 되돌아가 쓴다. 생존자 없으면 [] → 호출측이 한 arc 앞으로.
    """
    bearings = wp_bearing + _fan(p.search_fov, p.n_candidates)
    cands = np.stack([origin[0] + radius * np.cos(bearings),
                      origin[1] + radius * np.sin(bearings)], axis=1)
    seg_risk = _chord_risk(dri, np.asarray(prev_pt, dtype=np.float64), cands)
    avail = seg_risk <= p.safe_threshold * p.gen_margin
    geom = (
        p.w_wp * np.abs(bearings - wp_bearing)  # 부채꼴 폭 ≤ π 라 wrap 불필요
        + p.w_prev * np.abs(_wrap(bearings - prev_bearing))
    )
    if hist_bearing is not None and p.w_hist > 0.0:
        geom = geom + p.w_hist * np.abs(_wrap(bearings - hist_bearing))
    if not np.any(avail):
        return []
    geom_cost = np.where(avail, geom, np.inf)
    geom_best = _argmin_toward_wp(geom_cost, bearings, wp_bearing)
    # 전역 DRI 최소를 택하면 밭 밖의 0 지대로 과도하게 우회할 수 있으므로,
    # 기하 최적 안전 후보 주변으로만 범위를 제한한다.
    allowed = avail & (np.abs(_wrap(bearings - bearings[geom_best]))
                       <= p.dri_angle_slack + 1e-12)
    min_risk = float(np.min(seg_risk[allowed]))
    near_min_risk = allowed & (seg_risk <= min_risk + p.risk_straight_slack + 1e-12)
    first = _argmin_toward_wp(np.where(near_min_risk, geom_cost, np.inf), bearings, wp_bearing)
    picked = [first]
    if k > 1:
        # 2위 이하 = 1위 옆으로 RANK_GAP 씩 비킨 안전 후보 (가까운 것부터, 같은 거리면 현 위험
        # 낮은 쪽). 완성 곡선 위반은 대개 곡선이 꺾이는 곳에서 모서리를 질러가서다 — 제어점을
        # 조금 비키면 풀린다. 부채꼴 아무 데서나 다음 후보를 고르면 붐 반대쪽으로 갔다 돌아오는
        # U 자 경로가 났다. 후보가 등간격이라 각도 대신 인덱스 차로 센다.
        gap = max(1, math.ceil(RANK_GAP / (p.search_fov / (p.n_candidates - 1)) - 1e-9))
        ok, risk = avail.tolist(), seg_risk.tolist()
        for i in sorted(range(len(ok)), key=lambda i: (abs(i - first), risk[i])):
            if len(picked) >= k:
                break
            if ok[i] and all(abs(i - q) >= gap for q in picked):
                picked.append(i)
    return [(float(bearings[i]), cands[i]) for i in picked]


def _arc_pick(dri, origin, radius, wp_bearing, prev_pt, prev_bearing,
              p: PlannerParams, hist_bearing: float | None = None):
    """_arc_ranked 의 1위 (방위, 점) — 생존자 없으면 None."""
    ranked = _arc_ranked(dri, origin, radius, wp_bearing, prev_pt, prev_bearing, p,
                         hist_bearing, k=1)
    return ranked[0] if ranked else None


PIVOT_START_MIN_ANGLE = math.radians(5.0)   # 첫 경로점이 선수와 이보다 가까우면 p1 그대로
PIVOT_SCAN_STEP = math.radians(15.0)        # 정지 배 1차 생성 실패 시 p1 방향을 훑는 간격
# 훑을 때 이 거리 [m] 까지 시작 방향 직선이 위험선 위면 그 방향은 건너뛴다 — 곡선의 첫 부분은
# 시작 접선을 거의 따라가므로(0.3 m 에서 벗어남 ~수 cm) 거기가 막힌 방향은 곡선도 막힌다
PIVOT_CLEAR = 0.3


def generate(dri, boat_xy, boat_yaw: float, boat_vel_xy, waypoint_xy,
             p: PlannerParams, now_t: float = 0.0, *, goal_heading: float | None = None,
             terminal_handle_m: float = .75, prev_plan: Plan | None = None) -> Plan | None:
    """제어점 6개 생성 + spline 샘플링 (§3-2). search_fov 에 답이 없으면 search_fov_fallback
    으로 한 번 더 (막힐 때만 넓히기). 그래도 없으면 None.

    내보내는 경로는 곡선 샘플 전체가 safe_threshold 이하 — check() 를 통과한다 (_generate_once).

    배가 거의 서 있으면(전진 < stationary_surge) 제자리 회전으로 시작 방향을 바꿀 수 있다:
      · 경로가 있으면 p1 을 첫 경로점 쪽으로 돌린 경로를 한 번 더 만들어 쓴다
        (PlannerParams.stationary_surge 주석)
      · 선수 방향으로는 경로가 없으면(배 → p1 이 막힘 등) p1 방향을 waypoint 방위 ±90° 안에서
        PIVOT_SCAN_STEP 간격으로 waypoint 에 가까운 순서로 훑는다 — 2026-11-12 감사: 붐 바로
        앞에 선 배가 선수 쪽 1 m 만 보고 경로 없음 → ESCAPE 를 되풀이했다.
    prev_plan(재계획 때 지금 경로)을 주면 후보가 그 경로 쪽을 유지하려 한다 (w_hist)."""
    kw = dict(goal_heading=goal_heading, terminal_handle_m=terminal_handle_m,
              prev_plan=prev_plan, budget=_Budget(WORK_BUDGET))
    args = (dri, boat_xy, boat_yaw, boat_vel_xy, waypoint_xy, p, now_t)
    plan = _generate_wide(*args, **kw)
    if p.stationary_surge <= 0.0:
        return plan
    hdg = np.array([math.cos(boat_yaw), math.sin(boat_yaw)])
    if float(np.asarray(boat_vel_xy, dtype=np.float64) @ hdg) >= p.stationary_surge:
        return plan
    if plan is not None:
        return _pivot_start(plan, args, kw)
    wp_bearing = math.atan2(waypoint_xy[1] - boat_xy[1], waypoint_xy[0] - boat_xy[0])
    n = int(round(0.5 * p.search_fov_fallback / PIVOT_SCAN_STEP))
    dirs = [_wrap(wp_bearing + s * k * PIVOT_SCAN_STEP) for k in range(n + 1) for s in (1, -1)
            if k > 0 or s > 0]
    dirs.sort(key=lambda d: (round(abs(_wrap(d - wp_bearing)), 9), abs(_wrap(d - boat_yaw))))
    boat = np.asarray(boat_xy, dtype=np.float64)
    ss = np.linspace(0.1, PIVOT_CLEAR, 3)
    for d in dirs:
        if abs(_wrap(d - boat_yaw)) < PIVOT_START_MIN_ANGLE:
            continue                       # 선수 방향은 위에서 이미 해 봤다
        if np.any(dri.risk_at_many(boat[0] + ss * math.cos(d), boat[1] + ss * math.sin(d))
                  > p.safe_threshold):
            continue                       # 시작부터 막힌 방향 — 작업 상한을 안 쓴다
        # 훑기는 방향마다 최저 비용 곡선 하나만 (되돌아가기 없음) — 실패한 방향마다
        # 되돌아가기를 하면 방향 하나에 상한의 절반(~40)을 써서 2~3 방향 만에 상한이 찼다
        alt = _generate_wide(*args, p1_dir=d, ranked=1, **kw)
        if alt is not None:
            return _pivot_start(alt, args, kw, d)
        if kw['budget'].left <= 0:
            break                          # 작업 상한 — 다음 tick 에 다시
    return None


def _pivot_start(plan, args, kw, p1_dir=None):
    """p1 을 첫 경로점(p2) 쪽으로 돌린 경로 — 안 되면 받은 plan 그대로."""
    boat_xy, boat_yaw = args[1], args[2]
    first = plan.cps[2] - np.asarray(boat_xy, dtype=np.float64)
    start_dir = math.atan2(first[1], first[0])
    now_dir = boat_yaw if p1_dir is None else p1_dir
    if abs(_wrap(start_dir - now_dir)) < PIVOT_START_MIN_ANGLE:
        return plan
    alt = _generate_wide(*args, p1_dir=start_dir, **kw)
    return plan if alt is None else alt


def _generate_wide(dri, boat_xy, boat_yaw, boat_vel_xy, waypoint_xy, p, now_t,
                   p1_dir=None, budget=None, ranked=RANKED_PER_ARC, **kw):
    """search_fov 로 생성, 없으면 search_fov_fallback 으로 한 번 더.
    넓은 부채꼴은 제어점이 크게 꺾여 곡선이 장애물 모서리를 질러가기 쉽지만, 완성 곡선
    검사·되돌아가기(_generate_once)가 그런 곡선을 내보내지 않는다.
    기본 부채꼴은 남은 작업 상한의 절반까지만 — 막다른 arc 를 되돌아가며 상한을 다 써서
    넓은 부채꼴은 돌아 보지도 못한 장면이 있었다 (1500 장면 중 예전엔 안전 경로를 냈는데
    지금 못 내는 장면 3 → 1, 정지 배 훑기를 가볍게 한 뒤 0)."""
    budget = _Budget(WORK_BUDGET) if budget is None else budget
    wide = p.search_fov_fallback > p.search_fov
    share = _Budget(budget.left // 2 if wide else budget.left)
    given = share.left
    plan = _generate_once(dri, boat_xy, boat_yaw, boat_vel_xy, waypoint_xy, p, now_t,
                          p1_dir=p1_dir, budget=share, ranked=ranked, **kw)
    budget.left -= given - share.left
    if plan is None and wide:
        plan = _generate_once(dri, boat_xy, boat_yaw, boat_vel_xy, waypoint_xy,
                              replace(p, search_fov=p.search_fov_fallback), now_t,
                              p1_dir=p1_dir, budget=budget, ranked=ranked, **kw)
    return plan


def _generate_once(dri, boat_xy, boat_yaw: float, boat_vel_xy, waypoint_xy,
                   p: PlannerParams, now_t: float = 0.0, *, goal_heading: float | None = None,
                   terminal_handle_m: float = .75, p1_dir: float | None = None,
                   prev_plan: Plan | None = None, budget: _Budget | None = None,
                   ranked: int = RANKED_PER_ARC) -> Plan | None:
    """generate 의 한 번 시도 — 부채꼴 폭은 p.search_fov.

    p1_dir: p1 방향 [rad]. None 이면 선수(boat_yaw). 정지한 배는 generate 가 첫 경로점 쪽을 준다.
    budget: generate() 1회가 나눠 쓰는 작업 상한 (WORK_BUDGET). 다 쓰면 None.
    ranked: arc 마다 시도할 후보 수 (1 = 되돌아가기 없이 최저 비용 곡선만 검사).
    """
    budget = _Budget(WORK_BUDGET) if budget is None else budget
    boat = np.asarray(boat_xy, dtype=np.float64)
    vel = np.asarray(boat_vel_xy, dtype=np.float64)
    # p1 방향 = heading, 길이 = surge(전진 성분)·t_look (§3-2).
    # 배의 진짜 구속은 yaw(≈31°/s 상한)다 — 전후는 역추력으로 제동 가능하고
    # sway 는 무액추에이션이라 항력이 죽인다. 그래서 접선은 v̂(표류·노이즈
    # 포함)가 아니라 heading 에 묶고, 관성 길이도 heading 성분만 센다.
    # 후진 중엔 surge<0 → 하한으로 즉시 수축 (ESCAPE 자기교정이 더 빨라짐).
    hdg = np.array([math.cos(boat_yaw), math.sin(boat_yaw)])
    surge = max(0.0, float(vel @ hdg))
    p1_len = max(surge * p.t_look, p.p1_min_dist)
    p1_yaw = boat_yaw if p1_dir is None else float(p1_dir)
    p1_hdg = np.array([math.cos(p1_yaw), math.sin(p1_yaw)])
    p1 = boat + p1_len * p1_hdg

    wp = np.asarray(waypoint_xy, dtype=np.float64)
    wp_bearing = math.atan2(wp[1] - boat[1], wp[0] - boat[0])
    wp_dist = float(np.hypot(*(wp - boat)))

    # r_end = 배 중심 최대 내접원 − margin (§3-2: 격자 recenter 지연에도
    # arc 전체가 창 안임을 보장하는 최대 반지름) — waypoint 가 가까우면 거기까지만.
    res = dri.resolution
    hgt, wid = dri.data.shape
    x0, y0 = dri.anchor[0] * res, dri.anchor[1] * res
    inscribed = min(boat[0] - x0, x0 + wid * res - boat[0],
                    boat[1] - y0, y0 + hgt * res - boat[1])
    available = inscribed - p.boundary_margin
    goal_locked = False
    if p.finite_goal:
        if wp_dist <= 1e-6 or available <= 1e-6:
            return None
        r_end = min(available, wp_dist)
        # Keep all control points inside the remaining distance to the goal.
        p1_len = min(p1_len, 0.25 * r_end)
        p1 = boat + p1_len * p1_hdg
        goal_locked = wp_dist <= available
    else:
        r_end = min(available, max(wp_dist, p.min_horizon))
        r_end = max(r_end, p1_len + 2.0 * p.cp_min_gap)

    if goal_heading is not None and (not math.isfinite(goal_heading) or terminal_handle_m<=0):
        raise ValueError('invalid terminal heading constraint')
    heading_locked=goal_heading is not None and goal_locked
    terminal_control=(wp-terminal_handle_m*np.array([math.cos(goal_heading),math.sin(goal_heading)])
                      if heading_locked else None)
    r1 = pick_ref_obstacle(dri.occ_xy, boat, boat_yaw, p)

    # CP 사다리는 r1 과 무관 — 항상 ‖p1‖ ↔ r_end 균등 4분할.
    # 초안은 첫 arc 를 r1(기준 장애물 거리)에 앉혔는데, 장애물이 지평선
    # 근처면 radii 가 r_end 쪽에 뭉쳐(실측 [7.9,8.2,8.6,8.9]) 경로가 장애물
    # 밭 "입구에서 끝나는" 퇴화가 났다. berth 는 arc 가 장애물 링 위에
    # 앉아서가 아니라 arc 후보의 DRI 비용이 정한다 — 사다리 간격(≈0.9m)
    # < 거품 유효 반경이라 어느 arc 든 이웃 장애물을 느낀다. ⚠️ 격자를
    # 키우면 이 부등식이 깨질 수 있음(arc 4개 고정) — 그때 재검토.
    # r1 은 Plan.r1 로 저장만 하고 재생성 트리거 (a) 전용으로 쓴다.
    radii = [p1_len + k * (r_end - p1_len) / 4.0 for k in (1, 2, 3, 4)]
    spacing = 0.5 * res
    gen_limit = p.safe_threshold * p.gen_margin
    # 완성 곡선 합격선 = check() 와 같은 safe_threshold (도킹 마지막 정렬은 예전대로 gen_margin)
    curve_limit = gen_limit if heading_locked else p.safe_threshold

    # 배 → p1 구간은 직선이 아니라 완성 곡선 검사(finish)가 본다 — 예전엔 선분 사슬이 p1 에서
    # 시작해 이 구간을 아무도 안 봤다 (2026-11-12 감사: 정지한 배 정면 0.7 m 장애물 칸을 지나는
    # 경로를 그대로 냈다). 배 → p1 직선으로 거르면 안 된다: 곡선은 p1 을 지나지 않고 p2 쪽으로
    # 일찍 휘므로, p1 이 위험 칸 안이어도 곡선은 안전한 경우가 무작위 1500 장면 중 21 개였다.
    # 배 자리부터 위험선 위면(장애물 ~0.6 m 안) 어떤 곡선도 첫 샘플에서 걸린다 — 바로 None.
    if dri.risk_at(float(boat[0]), float(boat[1])) > curve_limit:
        return None

    def finish(choice):
        """p2~p5 [(방위, 점)] → (Plan, None, None). 곡선 위반이면 (None, 첫 위반 샘플, 호길이 표)."""
        cps = np.vstack([boat, p1] + [pt for _, pt in choice])
        dense, cum = _dense_spline(cps)
        samples = _resample(dense, cum, spacing)
        bad = np.nonzero(dri.risk_at_many(samples[:, 0], samples[:, 1]) > curve_limit)[0]
        if bad.size:
            return None, int(bad[0]), cum
        rr = list(radii)
        if heading_locked:
            rr[2] = float(np.linalg.norm(terminal_control - boat))
        return Plan(cps=cps, samples=samples,
                    origin=(float(boat[0]), float(boat[1])), r1=r1,
                    radii=rr, bearings=[b for b, _ in choice], created_t=now_t,
                    wp_bearing=wp_bearing, spacing=spacing, goal_locked=goal_locked,
                    terminal_heading=goal_heading if heading_locked else None,
                    search_fov=p.search_fov), None, None

    def fixed_choice(pt):
        return (math.atan2(pt[1] - boat[1], pt[0] - boat[0]), pt)

    if r1 is None:
        # R7 — 전방 부채꼴(±bearing_max)에 장애물 없음: waypoint 쪽 직선. 곡선 검사를 통과할
        # 때만 — 선수 뒤쪽 waypoint 로 돌아가는 곡선은 부채꼴 밖 장애물을 지날 수 있다
        straight = [(wp_bearing, boat + r * np.array([math.cos(wp_bearing), math.sin(wp_bearing)]))
                    for r in radii]
        if goal_locked:
            straight[3] = (wp_bearing, wp)
        if heading_locked:
            straight[2] = fixed_choice(terminal_control)
        budget.spend()
        plan, _, _ = finish(straight)
        if plan is not None:
            return plan

    # 제어점 고르기 + 되돌아가기. 각 arc 는 _arc_ranked 순위대로 시도하고, 완성 곡선이
    # 위반이면 그 위반 지점의 곡선을 가장 크게 움직이는 arc(기저 가중치 최대)의 다음 순위로
    # 바꿔 뒤 arc 를 다시 고른다. 다음 순위가 없으면 한 arc 앞으로. 최저 비용 곡선이 안전하면
    # 예전과 같은 경로다. 곡선 MAX_CURVES 개 안에 없으면 None (→ 호출측이 넓히기/ESCAPE).
    hist = [_plan_bearing_at(prev_plan, boat, r) for r in radii]
    opts = [None] * 4                  # arc j 의 [(방위, 점)] 순위 — 앞 arc 가 바뀌면 다시 계산
    idx = [0] * 4

    def options(j):
        prev_pt, prev_b = (p1, p1_yaw) if j == 0 else (opts[j - 1][idx[j - 1]][1],
                                                       opts[j - 1][idx[j - 1]][0])
        if goal_locked and j == 3:     # 마커 목표 — 고르는 것 없이 직선 검사만
            return [(wp_bearing, wp)] if _chord_risk(dri, prev_pt, wp)[0] <= gen_limit else []
        if heading_locked and j == 2:  # 도킹 정렬점 — 같은 식
            ok = _chord_risk(dri, prev_pt, terminal_control)[0] <= gen_limit
            return [fixed_choice(terminal_control)] if ok else []
        budget.spend()
        return _arc_ranked(dri, boat, radii[j], wp_bearing, prev_pt, prev_b, p, hist[j],
                           k=ranked)

    j = curves = 0
    while True:
        while j < 4:                   # 앞으로 채우기
            if budget.left <= 0:
                return None
            if opts[j] is None:
                opts[j] = options(j)
            if idx[j] < len(opts[j]):
                j += 1
                continue
            opts[j], idx[j] = None, 0  # 이 arc 후보 소진 → 한 arc 앞의 다음 순위
            j -= 1
            if j < 0:
                return None            # 안전 후보 전멸 → 호출측이 넓히기/ESCAPE
            idx[j] += 1
        budget.spend()
        plan, bad, cum = finish([opts[q][idx[q]] for q in range(4)])
        if plan is not None:
            return plan
        curves += 1
        if curves >= MAX_CURVES:
            return None
        u = float(np.interp(bad * spacing, cum, _U_DENSE))
        j = int(np.argmax(_BASIS(u)[2:]))          # 위반 지점을 가장 크게 움직이는 arc
        for q in range(j + 1, 4):
            opts[q], idx[q] = None, 0
        idx[j] += 1


def generate_docking(dri, boat_xy, boat_yaw, boat_vel_xy, waypoints_xy, goal_heading,
                     p, now_t=0.):
    """B-spline to normal-aligned approach, then a checked straight docking tail."""
    waypoints=np.asarray(waypoints_xy,float)
    if waypoints.shape!=(3,2) or not np.isfinite(waypoints).all() or not math.isfinite(goal_heading):
        raise ValueError('docking requires three finite waypoints and heading')
    approach,alignment,stop=waypoints
    heading=np.array([math.cos(goal_heading),math.sin(goal_heading)])
    steps=np.diff(waypoints,axis=0)
    if np.any(steps@heading<=.05) or np.any(abs(steps[:,0]*heading[1]-steps[:,1]*heading[0])>.02):
        raise ValueError('docking waypoints must advance along terminal heading')
    boat=np.asarray(boat_xy,float)
    along=float((boat-approach)@heading)
    cross=abs(float((boat-approach)@np.array([-heading[1],heading[0]])))
    final_phase=(along>=-.15 and cross<=.35 and abs(_wrap(boat_yaw-goal_heading))<=math.radians(20))
    # 도킹 접근은 출발 선수를 유지한다(test_dock_bspline) — 정지 시 p1 을 첫 경로점 쪽으로
    # 돌리는 규칙(stationary_surge)은 쓰지 않는다
    finite=replace(p,finite_goal=True,stationary_surge=0.0)
    if final_phase:
        remaining=float((stop-boat)@heading)
        if remaining<=1e-4:return None
        # Respect the observed starting heading while converging onto the axis.
        distance=float(np.linalg.norm(stop-boat));handle=min(.4,.2*distance)
        cps=np.vstack([boat,boat+handle*np.array([math.cos(boat_yaw),math.sin(boat_yaw)]),
                       boat+(stop-boat)*.4,boat+(stop-boat)*.65,stop-handle*heading,stop])
        spacing=.5*dri.resolution;samples=_sample_spline(cps,spacing)
        if np.any(dri.risk_at_many(samples[:,0],samples[:,1])>p.safe_threshold*p.gen_margin):return None
        delta=cps[2:]-boat
        return Plan(cps,samples,tuple(boat),pick_ref_obstacle(dri.occ_xy,boat,boat_yaw,p),
                    np.linalg.norm(delta,axis=1).tolist(),np.arctan2(delta[:,1],delta[:,0]).tolist(),
                    now_t,math.atan2(stop[1]-boat[1],stop[0]-boat[0]),spacing,
                    goal_locked=True,terminal_heading=goal_heading,docking_phase='FINAL')
    distance=float(np.linalg.norm(approach-boat))
    plan=generate(dri,boat,boat_yaw,boat_vel_xy,approach,finite,now_t,
                  goal_heading=goal_heading,terminal_handle_m=min(.75,max(.15,.3*distance)))
    if plan is None:return None
    plan.docking_phase='APPROACH'
    if not plan.goal_locked:return plan   # local horizon; attach tail when approach is reached
    length=float(np.linalg.norm(stop-approach))
    ts=np.r_[np.arange(plan.spacing,length,plan.spacing),length]
    tail=approach+ts[:,None]*heading
    # Include the explicitly published alignment waypoint exactly.
    tail=np.vstack([tail,alignment]);order=np.argsort((tail-approach)@heading)
    tail=tail[order];keep=np.r_[True,np.linalg.norm(np.diff(tail,axis=0),axis=1)>1e-8];tail=tail[keep]
    tail[-1]=stop
    if np.any(dri.risk_at_many(tail[:,0],tail[:,1])>p.safe_threshold*p.gen_margin):return None
    plan.tail_samples=tail;plan.samples=np.vstack([plan.samples,tail])
    return plan


def check(plan: Plan, dri, boat_xy, p: PlannerParams) -> int | None:
    """전방 구간 DRI 검사. 첫 위반 샘플 인덱스, 없으면 None.

    진행 갱신이 부수효과: progress_idx 는 [현재, +progress_window] 창에서
    배와 최근접한 샘플로 단조 전진 (뒤로 안 돌아감). 검사 구간은 거기부터
    끝까지 전부 — 먼 위반도 일찍 발견해 일찍 수리하는 게 이득 (§3-3).
    창 밖으로 나간 샘플은 risk=inf 라 자동으로 위반 취급된다.
    """
    n_win = max(1, int(round(p.progress_window / plan.spacing)))
    lo = plan.progress_idx
    seg = plan.samples[lo:lo + n_win + 1]
    d = np.hypot(seg[:, 0] - boat_xy[0], seg[:, 1] - boat_xy[1])
    plan.progress_idx = lo + int(np.argmin(d))

    ahead = plan.samples[plan.progress_idx:]
    risks = dri.risk_at_many(ahead[:, 0], ahead[:, 1])
    bad = np.nonzero(risks > p.safe_threshold)[0]
    return None if bad.size == 0 else plan.progress_idx + int(bad[0])


def repair(plan: Plan, dri, violation_idx: int, p: PlannerParams) -> bool:
    """arc CP(p2~p5) 하나를 자기 arc 위에서 ±Δθ 이동해 전방 위험을 낮춘다.

    후보 = 4개 CP × ±Δθ = **8개 전수 평가** (각각 재샘플 → 전방 구간 max
    DRI). "위반/봉우리 샘플의 최근접 CP 하나"류 타겟팅은 안 쓴다 — 봉우리가
    두 CP 사이 구간이면 최근접 CP 하나로는 전역 max 를 못 내려 교착하는
    배치가 실재한다 (게이트+측방 장면 실측: 최근접-CP 방식 1.62 교착 →
    전수 방식은 계속 하강). 재샘플이 ms 단위라 8후보는 공짜다.

    수락 기준 = 전방 max DRI 의 **엄격한 감소** — 단조 감소라 반복 수리가
    순환하지 않는다. 8후보 모두 개선 없음 / 부채꼴(wp_bearing ± search_fov/2)
    한계면 False → 호출측이 재생성/ESCAPE 로 간다. 반지름은 불변이므로
    radii 단조(= 전진 보장)는 수리로 깨지지 않는다. violation_idx 는 위반
    존재 확인용(호출 계약)으로만 받는다.
    """
    if plan.docking_phase=='FINAL':return False
    origin = np.asarray(plan.origin)
    cur = dri.risk_at_many(plan.samples[plan.progress_idx:, 0],
                           plan.samples[plan.progress_idx:, 1])
    baseline = float(np.max(cur))
    ref = plan.samples[plan.progress_idx]       # 재샘플 후 진행 재정합 기준점

    best = None
    for j in range(4):                          # radii/bearings 인덱스 (CP j+2)
        if (plan.goal_locked and j == 3) or (plan.terminal_heading is not None and j==2):
            continue                           # preserve the final marker mission goal
        for sgn in (1.0, -1.0):
            nb = plan.bearings[j] + sgn * p.repair_dtheta
            fov = max(p.search_fov, plan.search_fov or 0.0)  # 넓혀 만든 계획은 그 폭까지
            if abs(_wrap(nb - plan.wp_bearing)) > 0.5 * fov + 1e-12:
                continue                        # arc 부채꼴 한계 (§3-3)
            cps = plan.cps.copy()
            cps[j + 2] = origin + plan.radii[j] * np.array([math.cos(nb),
                                                            math.sin(nb)])
            samples = _sample_spline(cps, plan.spacing)
            if plan.tail_samples is not None:samples=np.vstack([samples,plan.tail_samples])
            prog = int(np.argmin(np.hypot(samples[:, 0] - ref[0],
                                          samples[:, 1] - ref[1])))
            worst = float(np.max(dri.risk_at_many(samples[prog:, 0],
                                                  samples[prog:, 1])))
            if worst < baseline - 1e-12 and (best is None or worst < best[0]):
                best = (worst, j, nb, cps, samples, prog)

    if best is None:
        return False
    _, j, nb, cps, samples, prog = best
    plan.bearings[j] = nb
    plan.cps, plan.samples, plan.progress_idx = cps, samples, prog
    return True


def curvature_ahead(plan: Plan | SampledPlan, window: float) -> float:
    """진행 위치부터 전방 window[m] 구간의 max |κ| [1/m].

    follow 감속 v = ω_max/κ_max 의 입력 (§4). κ 는 B-spline 도함수로 해석
    평가 — 샘플 유한차분은 spacing(0.1m) 에서 노이즈가 κ 봉우리를 만들어
    감속이 덜컹거린다. 호길이↔파라미터 대응만 조밀 평가로 만든다.
    samples 가 호길이 등간격이라 진행 호길이 = progress_idx·spacing.
    """
    if isinstance(plan, SampledPlan):
        # 고정 시험 코스는 제어점 개수/차수가 가변이므로 생성 때 계산한
        # 해석적 곡률 샘플을 사용한다. 현재 점만이 아니라 전방 창의 최대값.
        if plan.closed:
            count = max(1, int(math.ceil(window / plan.spacing))) + 1
            count = min(count, len(plan.curvatures))
            indices = ((plan.progress_idx + np.arange(count))
                       % len(plan.curvatures))
            return float(np.max(np.abs(plan.curvatures[indices])))
        start = min(plan.progress_idx, len(plan.curvatures) - 1)
        end = min(len(plan.curvatures),
                  start + max(1, int(math.ceil(window / plan.spacing))) + 1)
        return float(np.max(np.abs(plan.curvatures[start:end])))

    spl = BSpline(_KNOTS, plan.cps, 3)
    u = np.linspace(0.0, 1.0, 200)
    pts = np.asarray(spl(u))
    s = np.concatenate([[0.0], np.cumsum(np.hypot(*np.diff(pts, axis=0).T))])
    s0 = plan.progress_idx * plan.spacing
    if plan.tail_samples is not None and s0>=s[-1]:return 0.
    mask = (s >= s0) & (s <= s0 + window)
    if not np.any(mask):
        mask[-1] = True                 # 경로 끝 도달 — 끝점 곡률이라도
    d1 = np.asarray(spl.derivative(1)(u[mask]))
    d2 = np.asarray(spl.derivative(2)(u[mask]))
    num = np.abs(d1[:, 0] * d2[:, 1] - d1[:, 1] * d2[:, 0])
    den = np.maximum((d1[:, 0] ** 2 + d1[:, 1] ** 2) ** 1.5, 1e-12)
    return float(np.max(num / den))
