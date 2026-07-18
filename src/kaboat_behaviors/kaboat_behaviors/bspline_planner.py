"""bspline_planner — DRI 격자 위에서 B-spline 로컬 경로를 생성/검사/수리하는
순수 라이브러리 (AVOIDANCE.MD §3-2/§3-3 의 구현, ROS 의존 없음).

설계 요지:
  경로는 제어점 6개짜리 clamped cubic B-spline. p0(배)·p1(관성점)이 시작
  기하를 고정하고, p2~p5 는 배 중심 **동심원(arc) 위에서 방위만 고르는** 점
  이다 — 반지름이 단조증가라 "항상 바깥으로 전진"이 구조적으로 보장되고,
  수리(repair)는 반지름을 안 건드리므로 그 보장을 깨지 못한다.

  생성(generate)과 검사(check)는 분리 — 생성은 가끔, 검사는 매 tick
  최신 DRI 로. 후보 탐색은 waypoint 방위 ±search_fov/2 부채꼴인데, 기본
  180° 는 임의값이 아니라 "waypoint 쪽 전진 성분 ≥ 0 인 반평면"이다(§3-2).
  안전은 **하드 필터**: DRI > safe_threshold 후보는 제거하고, 생존자 중
  방위 비용만으로 고른다 — 거품(σ) 이 이미 berth 를 인코딩하므로 비용에
  DRI 를 또 넣는 건 이중 계산이다. 어떤 arc 든 생존자가 없으면 None —
  전진 반평면에 답이 없다는 뜻이고, 그 답은 ESCAPE(후진)다.

  t_look=1.0s 는 실측 전속 1.48 m/s 기준 재산출값(§3-2) — 5.0s 초안은
  p1 이 7.4m 로 튀어 스플라인이 나갔다 되돌아오는 첨점을 만들었다.
"""
from dataclasses import dataclass, field
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
    min_horizon: float = 3.0       # waypoint 가 코앞이어도 r_end 하한 [m]
    cp_min_gap: float = 0.5        # r_end 퇴화 방지: r_end ≥ ‖p1‖ + 2·이 값 [m]
    search_fov: float = math.radians(180.0)  # 후보 부채꼴 = wp 전진 반평면
    n_candidates: int = 31
    # 후보 비용 가중. DRI 항은 없다 — 안전은 비용이 아니라 **하드 필터**
    # (아래 safe_threshold)다. 소프트 DRI 벌점 시절엔 threshold 이하의 안전한
    # 꼬리 위험(0.4~0.6)도 벌점으로 작동해, CP 가 틈 대신 밭 밖 DRI=0 지대로
    # 도망갔다(sim 실측 — 안전을 거품 크기와 비용에서 이중 계산한 탓).
    # w_prev 는 후보 6° 양자화가 만드는 arc 간 S 잔물결 억제.
    w_wp: float = 1.0
    w_prev: float = 0.5
    # 유일한 안전 기준선 — 생성(후보 하드 필터)과 검사(check 위반)가 같은
    # 선을 쓴다. 후보는 선분 max DRI > 이 값이면 제거, 생존자 중 방위 비용
    # argmin, 어떤 arc 든 생존자 0 이면 generate → None (구 hard_max 역할
    # 흡수 — 생성이 "괜찮다"고 낸 경로를 같은 tick check 가 위반 판정하던
    # 자기모순 제거). berth 관계: 거품 반경 = σ·√(2·ln(A/threshold)).
    safe_threshold: float = 1.0
    # 생성 필터는 threshold×이 값 에서 자른다 — 필터는 현(chord)으로 판정
    # 하는데 스플라인은 굽이에서 현보다 수 cm 안쪽으로 붙어, 1.0 그대로면
    # 경계 스침(실측 1.013)이 남는다. 0.85 = 위험 경사(≈2.7/m) 기준 ~5cm
    # 편차 흡수. check 는 margin 없이 threshold 그대로 본다.
    gen_margin: float = 0.85
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
    progress_idx: int = 0    # 배가 지나온 samples 진행 인덱스 — check 가 갱신(단조)


def _wrap(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


# clamped cubic, 제어점 6개 — 양 끝점 통과, 시작 접선 = p0→p1 방향
_KNOTS = np.array([0, 0, 0, 0, 1 / 3, 2 / 3, 1, 1, 1, 1], dtype=np.float64)


def _sample_spline(cps: np.ndarray, spacing: float) -> np.ndarray:
    """spline 을 조밀 평가 후 호길이 등간격으로 재샘플 (끝점 포함)."""
    dense = np.asarray(BSpline(_KNOTS, cps, 3)(np.linspace(0.0, 1.0, 300)))
    seg = np.hypot(*np.diff(dense, axis=0).T)
    cum = np.concatenate([[0.0], np.cumsum(seg)])
    targets = np.arange(0.0, cum[-1], spacing)
    xs = np.interp(targets, cum, dense[:, 0])
    ys = np.interp(targets, cum, dense[:, 1])
    return np.vstack([np.stack([xs, ys], axis=1), dense[-1:]])


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


def _arc_pick(dri, origin, radius, wp_bearing, prev_pt, prev_bearing,
              p: PlannerParams):
    """arc 위 후보 선택 — 안전은 하드 필터, 선호는 방위 비용.

    ① **직전 CP → 후보를 잇는 선분**(샘플 8점) 위 max DRI > safe_threshold 인
       후보 제거 (창 밖 inf 도 자연 탈락). 후보 점 하나만 보면 점은 거품
       밖인데 거기까지 가는 스플라인이 거품을 가로지르는 구멍이 남는다 —
       가장 단순한 장면(정면 4m 장애물 1개)에서 위험 1.55 스침으로 실측.
       선분은 스플라인 현(chord)의 근사지만 점 하나보다 훨씬 정직하다.
    ② 생존자 중 w_wp·|wp방위차| + w_prev·|직전CP방위차| argmin
    생존자 없으면 None — 호출측(generate)이 전체를 None 으로 (→ ESCAPE).
    """
    half = 0.5 * p.search_fov
    bearings = wp_bearing + np.linspace(-half, half, p.n_candidates)
    cands = np.stack([origin[0] + radius * np.cos(bearings),
                      origin[1] + radius * np.sin(bearings)], axis=1)
    # t=0(직전 CP 자신)은 제외 — 이미 직전 단계에서 판정된 점이다
    ts = np.linspace(0.125, 1.0, 8)[:, None, None]
    seg = prev_pt[None, None, :] + ts * (cands[None, :, :] - prev_pt[None, None, :])
    risks = dri.risk_at_many(seg[..., 0].ravel(),
                             seg[..., 1].ravel()).reshape(8, -1)
    safe = risks.max(axis=0) <= p.safe_threshold * p.gen_margin
    if not np.any(safe):
        return None
    cost = (p.w_wp * np.abs(bearings - wp_bearing)   # 부채꼴 폭 ≤ π 라 wrap 불필요
            + p.w_prev * np.abs([_wrap(b - prev_bearing) for b in bearings]))
    cost[~safe] = np.inf
    i = int(np.argmin(cost))
    return float(bearings[i]), cands[i]


def generate(dri, boat_xy, boat_yaw: float, boat_vel_xy, waypoint_xy,
             p: PlannerParams, now_t: float = 0.0) -> Plan | None:
    """제어점 6개 생성 + spline 샘플링 (§3-2). 전진 반평면에 답이 없으면 None."""
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
    p1 = boat + p1_len * hdg

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
    r_end = inscribed - p.boundary_margin
    r_end = min(r_end, max(wp_dist, p.min_horizon))
    r_end = max(r_end, p1_len + 2.0 * p.cp_min_gap)   # 퇴화 방지

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
    if r1 is None:
        bearings = [wp_bearing] * 4   # R7 — 전방 부채꼴에 장애물 없음, 직선
    else:
        bearings = []
        prev_pt = p1                  # 선분 필터의 시작점 사슬: p1 → p2 → …
        prev_b = boat_yaw             # 첫 arc 의 연속성 기준 = p1 방향 = heading
        for r in radii:
            picked = _arc_pick(dri, boat, r, wp_bearing, prev_pt, prev_b, p)
            if picked is None:
                return None            # 안전 후보 전멸 → 호출측이 ESCAPE)
            b, prev_pt = picked
            bearings.append(b)
            prev_b = b

    cps = np.vstack([boat, p1]
                    + [boat + r * np.array([math.cos(b), math.sin(b)])
                       for r, b in zip(radii, bearings)])
    spacing = 0.5 * res
    return Plan(cps=cps, samples=_sample_spline(cps, spacing),
                origin=(float(boat[0]), float(boat[1])), r1=r1,
                radii=radii, bearings=bearings, created_t=now_t,
                wp_bearing=wp_bearing, spacing=spacing)


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
    origin = np.asarray(plan.origin)
    cur = dri.risk_at_many(plan.samples[plan.progress_idx:, 0],
                           plan.samples[plan.progress_idx:, 1])
    baseline = float(np.max(cur))
    ref = plan.samples[plan.progress_idx]       # 재샘플 후 진행 재정합 기준점

    best = None
    for j in range(4):                          # radii/bearings 인덱스 (CP j+2)
        for sgn in (1.0, -1.0):
            nb = plan.bearings[j] + sgn * p.repair_dtheta
            if abs(_wrap(nb - plan.wp_bearing)) > 0.5 * p.search_fov + 1e-12:
                continue                        # arc 부채꼴 한계 (§3-3)
            cps = plan.cps.copy()
            cps[j + 2] = origin + plan.radii[j] * np.array([math.cos(nb),
                                                            math.sin(nb)])
            samples = _sample_spline(cps, plan.spacing)
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


def curvature_ahead(plan: Plan, window: float) -> float:
    """진행 위치부터 전방 window[m] 구간의 max |κ| [1/m].

    follow 감속 v = ω_max/κ_max 의 입력 (§4). κ 는 B-spline 도함수로 해석
    평가 — 샘플 유한차분은 spacing(0.1m) 에서 노이즈가 κ 봉우리를 만들어
    감속이 덜컹거린다. 호길이↔파라미터 대응만 조밀 평가로 만든다.
    samples 가 호길이 등간격이라 진행 호길이 = progress_idx·spacing.
    """
    spl = BSpline(_KNOTS, plan.cps, 3)
    u = np.linspace(0.0, 1.0, 200)
    pts = np.asarray(spl(u))
    s = np.concatenate([[0.0], np.cumsum(np.hypot(*np.diff(pts, axis=0).T))])
    s0 = plan.progress_idx * plan.spacing
    mask = (s >= s0) & (s <= s0 + window)
    if not np.any(mask):
        mask[-1] = True                 # 경로 끝 도달 — 끝점 곡률이라도
    d1 = np.asarray(spl.derivative(1)(u[mask]))
    d2 = np.asarray(spl.derivative(2)(u[mask]))
    num = np.abs(d1[:, 0] * d2[:, 1] - d1[:, 1] * d2[:, 0])
    den = np.maximum((d1[:, 0] ** 2 + d1[:, 1] ** 2) ** 1.5, 1e-12)
    return float(np.max(num / den))
