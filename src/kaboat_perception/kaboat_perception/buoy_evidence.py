"""buoy_evidence — 부표별 존재 log-odds 트래커 (ROS 무의존 순수 모듈).

점유지도가 칸마다 하는 log-odds 를 부표 하나마다 한다. 위치는 원시 라이다 군집 중심
(실수 좌표, 칸으로 뭉개지 않음), 존재는 Track.L 이 쌓는다. 근거는 라이다만 쓴다 —
카메라는 색 표만 넣는다 (buoy_color).

존재 근거 (스캔마다):
  올림  라이다 군집이 이 부표와 연결됨                      L += l_hit
  깎음  부표 방향 빔이 부표 자리를 지나 더 멀리서 맞음       L += l_free_rate · dt
  유지  부표보다 앞에서 맞음(가림), 부표 거리에서 맞았지만 군집이 안 됨,
        반사 없음(검정 무광은 빛을 흡수할 수 있다), 라이다 거리 범위 밖, 스캔 없음
  → "보였어야 하는데 안 보였을 때"만 지운다. 시야 밖·가림·센서 없음은 근거가 아니다.

상태 (히스테리시스):
  후보 —(L ≥ l_confirm)→ 확정 —(L ≤ l_delete)→ 삭제
  후보는 L < 0 이 되거나 tentative_timeout 동안 다시 안 잡히면 삭제 (물보라·반짝임).
  새 부표는 호출자가 허락한 검출(can_spawn — 노드는 20 m 안)에서만 만든다.
  옆·뒤의 물가·다리 기둥·정박 배가 부표가 되지 않게. 이미 있는 부표는 어느 방향 검출로든 갱신.
  시간이 1 s 넘게 거꾸로 가면(bag 반복 재생·시계 재설정) 처음부터 다시 시작한다.
  배에서 forget_range(15 m) 넘게 멀어진 부표는 지운다 — 점유지도가 배 주변 창 밖을 버리듯.
  확정 부표 둘이 merge_radius 안이면 L 큰 쪽으로 합친다.

색: 프레임별 표 ('red' | 'green' | 'unknown', None 은 표 없음). 빨강·초록 중 많은 쪽이
  2표 이상이고 전체 표의 25 % 이상이면 그 색, 아니면 unknown (buoy_tracker 와 같은 규칙).
  표가 하나도 없으면 'unseen' — 카메라에 한 번도 안 찍힌 라이다 물체(옆·뒤의 물가, 다리 기둥).
  unknown(= 검정 후보)과 섞이지 않게 따로 둔다.

위치·크기 (원 맞추기): 한 스캔의 호 폭으로는 지름을 알 수 없다 — 호가 몇 도 찍혔는지
  모르니(가장자리는 빔이 비스듬해 반사가 끊김) 폭은 늘 지름보다 작다. 그래서 부표마다
  연결된 군집의 점을 전역(odom) 좌표로 모은다 (부표는 고정, 위치는 RTK). 새로 잡힐 때마다
    ① 이동  반지름 r 을 고정하고 중심만 움직여 점들에 맞춘다 (반지름 고정 원 맞추기).
    ② 키움  그래도 원 바깥으로 fit_out_tol 넘게 벗어난 점이 fit_out_frac 이상이고, 그 점들이
           서로 다른 시선 방향(fit_sector_deg 단위) fit_min_sectors 곳 이상에서 왔으면 r 이
           작은 것 — 그때는 여러 방향 점이 모인 것이라 중심·반지름을 함께 맞춰 r 을 정하고 ① 로. 줄이지는 않는다(점이 원 안쪽이면 호가 일부만 보인
           것일 수 있다). 상한 radius_max.
  처음 r = radius_prior. 작은 원은 실제 호보다 빨리 휘어서 호 양 끝이 원 밖으로 삐져나오므로
  중심을 옮겨도 안 맞는다 → 키우는 신호. 크게 틀리는 쪽(회피에 안전)으로만 틀린다.
  단, 함께 맞춘 원이 점에 잘 맞을 때(fit_max_rms)·중심이 크게 안 움직일 때만 키운다.
  2026-11-12 bag 은 스캔을 겹치면 부표 중심이 RMS ≈ 0.15 m 흔들리고(odom·라이다 시각 맞춤,
  한쪽 호 중심 추정) 부표를 한쪽 30° 범위에서만 봐서, 키우지 않고 사전값 그대로가 맞다.
  원 중심은 늘 보이는 면 뒤(라이다 반대편)에 둔다 (fit_min_facing) — 번진 점 무더기가 라이다
  쪽으로 오목하게 휘면 맞추기가 원을 면 앞에 놓아, 점이 없는 쪽으로 커졌다 (초록 부표).

숫자는 처음 기준값이다 — bag 으로 센서별 검출률·오검출률을 재서 맞춘다.
"""
import math
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

KNOWN_COLORS = ('red', 'green')
KNOWN_COLOR_MIN_VOTES = 2
KNOWN_COLOR_FRAC = 0.25
MAX_STEP = 0.5   # [s] 프레임 간격 상한 — 스캔이 한동안 끊겼다 돌아온 첫 스캔에 깎임이 몰리지 않게
TIME_JUMP_BACK = 1.0   # [s] 이보다 거꾸로 가면 리셋


@dataclass(frozen=True)
class EvidenceParams:
    gate_radius: float = 0.7        # 군집 ↔ 부표 연결 최대 거리 [m] (부표는 고정 — 1.5 는 배를 따라
                                    # 움직이는 수면 hit 를 한 부표로 끌고 다녔고, 1.0 은 1.2 m 붙은
                                    # 초록·검정 쌍을 섞었다. 스캔 간 중심 흔들림 RMS ≈ 0.15 m)
    ema_alpha: float = 0.3          # 위치 평활 (새 관측 가중치)
    l_hit: float = 0.8              # 라이다 군집 연결 1회
    l_free_rate: float = -2.0       # 빔이 부표 자리를 지나감 [/s]
    l_confirm: float = 2.0          # 후보 → 확정 (≈ 88 %)
    l_delete: float = -1.0          # 확정 → 삭제 (≈ 27 %)
    l_min: float = -2.0
    l_max: float = 4.0
    tentative_timeout: float = 3.0  # 후보가 이만큼 안 잡히면 삭제 [s]
    merge_radius: float = 1.0       # 확정 부표 병합 거리 [m] — 스캔 번짐으로 한 부표가 둘로 갈린 것 합침
                                    # (1.2 m 붙은 초록·검정 쌍보다는 작게)
    pass_margin: float = 0.5        # 빔이 부표보다 이만큼 멀리/가까이서 맞아야 지나감/가림 [m]
    beam_half_width: float = 0.15   # 부표 방향 빔 창 반폭 — 부표 반폭 [m]
    min_beam_half_angle: float = math.radians(0.5)
    max_range: float = 10.0         # 라이다 원점에서 이보다 먼 부표는 깎지 않음 (노드는 군집도 이 안만) [m]
    forget_range: float = 15.0      # 라이다 원점에서 이보다 멀어진 부표는 지움 — 점유지도 창처럼 배 주변만 [m]
    # 원 맞추기
    radius_prior: float = 0.2       # 처음 반지름 [m] — 작게 시작 (줄이지 않으므로)
    radius_max: float = 0.75        # 반지름 상한 [m] (지름 1.5 m)
    fit_min_points: int = 10        # 이만큼 점이 모이면 원 맞추기로 중심을 잡는다
    fit_max_points: int = 600       # 부표마다 보관하는 점 (오래된 것부터 버림)
    fit_out_tol: float = 0.05       # 원 바깥으로 이만큼 넘게 벗어나야 '안 맞음' [m] (라이다 오차 이상)
    fit_out_frac: float = 0.2       # 벗어난 점 비율이 이 이상이면 키움
    fit_sector_deg: float = 30.0    # 시선 방향 구간 [deg]
    fit_min_sectors: int = 3        # 벗어난 점이 이만큼 다른 시선 구간에서 와야 키움 (3 = 60° 넘게 돌아가며 봄)
    fit_max_rms: float = 0.08       # 키울 때 함께 맞춘 원의 잔차 RMS 상한 [m] — 넘으면 번짐이라 안 키움
    fit_max_shift: float = 0.3      # 키울 때 중심이 이보다 많이 움직이면 안 키움 [m]
    fit_max_jump: float = 1.0       # 맞춘 중심이 이전보다 이만큼 넘게 튀면 그 결과는 버림 [m]
    # 색 — 부표마다 창 픽셀 수 누적 (빨강·초록·보이는). 붙은 쌍에서 옆 부표가 한두 프레임 섞여도
    # 자기 색 픽셀이 훨씬 많아 갈린다 (프레임마다 표 한 장씩 세면 섞인 표가 같은 무게였다).
    px_min_frac: float = 0.15       # 빨강 또는 초록이 보이는 픽셀의 이 비율 이상이어야 그 색
    px_dominance: float = 2.0       # 다른 색 픽셀의 이 배 이상이어야 그 색
    px_min_visible: int = 150       # 누적 보이는 픽셀이 이보다 적으면 표 규칙으로 (그것도 없으면 unseen)
                                    # (빨강 부표가 쓸 만한 프레임 1장·176 px 로 지나간 경우도 판정되게)
    fit_min_facing: float = 0.7     # 맞춘 원에서 라이다 쪽을 향한 점 비율이 이보다 작으면 버림 —
                                    # 원 중심은 보이는 면 뒤(라이다 반대편)에 있어야 한다
    fit_outlier: float = 0.3        # 원에서 이보다 먼 점은 맞추기에서 뺌 (물보라·옆 부표) [m]


@dataclass
class LidarView:
    """한 스캔 — 전역(odom) 라이다 원점·선수각과 원시 거리."""
    x: float
    y: float
    yaw: float
    angle_min: float
    angle_increment: float
    ranges: np.ndarray
    range_min: float
    range_max: float


@dataclass
class EvTrack:
    id: int
    x: float
    y: float
    L: float
    last_seen: float
    confirmed: bool = False
    hits: int = 0
    color_votes: Dict[str, int] = field(default_factory=dict)
    last_evidence: str = 'hit'
    radius: float = 0.2           # 원 맞추기 반지름 [m] (radius_prior 에서 시작, 키우기만)
    px_red: int = 0               # 누적 창 픽셀 (usable 프레임만)
    px_green: int = 0
    px_visible: int = 0
    px_params: Optional[Tuple[float, float, int]] = None   # (min_frac, dominance, min_visible)
    grown: bool = False           # 반지름이 사전값에서 커졌나 (점이 원 밖에 찍혀서)
    pts: Optional[np.ndarray] = None    # 모은 점 N×2 (전역)
    views: Optional[np.ndarray] = None  # 점마다 시선 방향 [rad] (라이다 원점 → 군집), NaN = 모름

    @property
    def color(self) -> str:
        min_frac, dominance, min_visible = self.px_params or (0.15, 2.0, 150)
        if self.px_visible >= min_visible:
            for c, n, other in (('red', self.px_red, self.px_green),
                                ('green', self.px_green, self.px_red)):
                if n >= min_frac * self.px_visible and n >= dominance * other:
                    return c
            return 'unknown'
        total = sum(self.color_votes.values())
        if total == 0:
            return 'unseen'
        known = [(n, c) for c, n in self.color_votes.items() if c in KNOWN_COLORS]
        if known:
            n, c = max(known)
            if n >= KNOWN_COLOR_MIN_VOTES and n >= KNOWN_COLOR_FRAC * total:
                return c
        return 'unknown'

    @property
    def probability(self) -> float:
        return 1.0 / (1.0 + math.exp(-self.L))


def fit_center_fixed_radius(pts, center, r, outlier, iters=10):
    """반지름 r 고정 원 맞추기 — Σ(|p − c| − r)² 를 c 에 대해 가우스-뉴턴. outlier 넘게 먼 점은 뺌."""
    c = np.asarray(center, dtype=float).copy()
    for _ in range(iters):
        d = pts - c
        n = np.hypot(d[:, 0], d[:, 1])
        ok = (n > 1e-6) & (np.abs(n - r) <= outlier)
        if ok.sum() < 3:
            break
        u = d[ok] / n[ok, None]
        f = n[ok] - r
        J = -u
        step = np.linalg.lstsq(J, -f, rcond=None)[0]
        c += step
        if np.hypot(*step) < 1e-4:
            break
    return c


def fit_circle_free(pts, center, r, outlier, iters=15):
    """중심·반지름 함께 맞추기 (가우스-뉴턴). 여러 방향 점이 모였을 때만 쓴다 — 한쪽 호만으로는
    반지름이 크게 튄다."""
    c = np.asarray(center, dtype=float).copy()
    for _ in range(iters):
        d = pts - c
        n = np.hypot(d[:, 0], d[:, 1])
        ok = (n > 1e-6) & (np.abs(n - r) <= outlier)
        if ok.sum() < 4:
            break
        u = d[ok] / n[ok, None]
        J = np.column_stack((-u, -np.ones(ok.sum())))
        step = np.linalg.lstsq(J, -(n[ok] - r), rcond=None)[0]
        c += step[:2]
        r += float(step[2])
        if float(np.linalg.norm(step)) < 1e-4:
            break
    return c, r


def facing_fraction(pts, views, c) -> float:
    """원 중심 c 에서 본 점 방향이 라이다 쪽(시선 반대)을 향하는 비율. 시선 모르면 1."""
    ok = np.isfinite(views)
    if not ok.any():
        return 1.0
    n = pts[ok] - np.asarray(c)
    toward = -np.column_stack((np.cos(views[ok]), np.sin(views[ok])))
    return float(np.mean(np.sum(n * toward, axis=1) > 0.0))


def behind_surface(pts, views, r) -> Optional[np.ndarray]:
    """점 평균에서 시선 방향(라이다 → 부표)으로 0.8 r 뒤 — 보이는 호 평균은 중심보다 약 0.8 r 앞."""
    ok = np.isfinite(views)
    if not ok.any():
        return None
    u = np.array([np.mean(np.cos(views[ok])), np.mean(np.sin(views[ok]))])
    n = np.hypot(*u)
    if n < 1e-6:
        return None
    return pts.mean(axis=0) + 0.8 * r * u / n


def _fit_behind(pts, views, c, r, p: EvidenceParams):
    """반지름 고정 맞추기 — 결과가 보이는 면 앞(라이다 쪽)에 놓이면 버리고 면 뒤 점을 쓴다.
    스캔 번짐으로 점 무더기가 라이다 쪽으로 오목하게 휘면, 맞추기만으로는 원이 앞에 놓였다."""
    start = behind_surface(pts, views, r)
    c0 = c if start is None else start
    c1 = fit_center_fixed_radius(pts, c0, r, p.fit_outlier)
    if start is not None and (facing_fraction(pts, views, c1) < p.fit_min_facing
                              or np.hypot(*(c1 - start)) > r):
        return start
    return c1


def fit_track_circle(t: 'EvTrack', p: EvidenceParams):
    """① 이동 ② 안 맞으면 키움 — t.x, t.y, t.radius, t.grown 갱신."""
    pts, views = t.pts, t.views
    c = np.array([t.x, t.y])
    r = t.radius
    for _ in range(6):
        c = _fit_behind(pts, views, c, r, p)
        dist = np.hypot(pts[:, 0] - c[0], pts[:, 1] - c[1])
        inl = np.abs(dist - r) <= p.fit_outlier
        out = inl & (dist - r > p.fit_out_tol)
        if inl.sum() == 0 or out.sum() < p.fit_out_frac * inl.sum():
            break
        v = views[out]
        v = v[np.isfinite(v)]
        sectors = np.unique(np.floor(np.degrees(v) / p.fit_sector_deg)).size
        if sectors < p.fit_min_sectors:
            break
        # 키움 — 여러 방향 점이 원 밖에 찍혔으니 중심·반지름을 함께 맞춰 반지름을 정한다.
        # 함께 맞춘 원이 점에 잘 맞을 때만(번짐이 아니라 진짜 큰 원일 때만) 받는다.
        c_free, r_free = fit_circle_free(pts, c, r, p.fit_outlier)
        if not (np.all(np.isfinite(c_free)) and np.isfinite(r_free)):
            break
        dfree = np.hypot(pts[:, 0] - c_free[0], pts[:, 1] - c_free[1]) - r_free
        ok = np.abs(dfree) <= p.fit_outlier
        rms = float(np.sqrt(np.mean(dfree[ok] ** 2))) if ok.any() else float('inf')
        r_new = min(p.radius_max, float(r_free))
        if (r_new <= r + 1e-3 or rms > p.fit_max_rms
                or np.hypot(*(c_free - c)) > p.fit_max_shift
                or facing_fraction(pts, views, c_free) < p.fit_min_facing):
            break
        r, t.grown = r_new, True
        c = c_free
    if not np.all(np.isfinite(c)) or np.hypot(c[0] - t.x, c[1] - t.y) > p.fit_max_jump:
        return                       # 맞추기가 튀었다 — 이번 결과는 버림
    t.x, t.y, t.radius = float(c[0]), float(c[1]), r


def beam_evidence(view: LidarView, x: float, y: float, p: EvidenceParams) -> str:
    """부표 자리 (x, y) 에 대한 이 스캔의 근거 → 'free' | 'occluded' | 'consistent' | 'none'."""
    dx, dy = x - view.x, y - view.y
    d = math.hypot(dx, dy)
    if (d < view.range_min + p.pass_margin or d > view.range_max - p.pass_margin
            or d > p.max_range):
        return 'none'
    bearing = math.atan2(dy, dx) - view.yaw
    half = max(p.min_beam_half_angle, math.atan2(p.beam_half_width, d))
    r = np.asarray(view.ranges, dtype=float)
    a = view.angle_min + view.angle_increment * np.arange(len(r))
    near = np.abs(np.arctan2(np.sin(a - bearing), np.cos(a - bearing))) <= half
    rr = r[near]
    rr = rr[np.isfinite(rr) & (rr > view.range_min) & (rr < view.range_max)]
    if rr.size == 0:
        return 'none'            # 반사 없음 — 검정 흡수·하늘·뒤쪽 가림일 수 있어 근거 아님
    if np.any(rr < d - p.pass_margin):
        return 'occluded'
    if np.any(np.abs(rr - d) <= p.pass_margin):
        return 'consistent'
    return 'free'                # 전부 부표보다 멀리서 맞음 — 그 자리는 비었다


class EvidenceTracker:
    def __init__(self, params: EvidenceParams = None):
        self.p = params or EvidenceParams()
        self.tracks: List[EvTrack] = []
        self._next_id = 0
        self._last_now: Optional[float] = None

    def confirmed(self) -> List[EvTrack]:
        return [t for t in self.tracks if t.confirmed]

    def reset(self):
        self.tracks = []
        self._last_now = None

    def update(self, detections: Sequence[Tuple], now: float,
               view: Optional[LidarView] = None) -> List[EvTrack]:
        """detections = [(x, y, 색 표 또는 None[, can_spawn=True[, 군집 점 N×2 (전역)]])],
        view = 이 스캔 (없으면 깎지 않음, 점의 시선 방향도 모름)."""
        p = self.p
        if self._last_now is not None and now < self._last_now - TIME_JUMP_BACK:
            self.reset()
        detections = [(*d, True, None)[:5] if len(d) == 3 else (*d, None)[:5] for d in detections]
        dt = 0.0 if self._last_now is None else min(max(now - self._last_now, 0.0), MAX_STEP)
        self._last_now = now

        # ① 연결 — 가까운 쌍부터 1:1
        pairs = []
        for i, (x, y, _c, _s, pts) in enumerate(detections):
            for t in self.tracks:
                d = self._assoc_distance(t, x, y, pts)
                if d <= p.gate_radius:
                    pairs.append((d, i, t.id))
        pairs.sort()
        used_det, used_trk = set(), set()
        by_id = {t.id: t for t in self.tracks}
        for _d, i, tid in pairs:
            if i in used_det or tid in used_trk:
                continue
            used_det.add(i)
            used_trk.add(tid)
            t = by_id[tid]
            x, y, color, _s, pts = detections[i]
            self._add_points(t, pts, view)
            if t.pts is not None and len(t.pts) >= p.fit_min_points:
                fit_track_circle(t, p)
            else:
                t.x += p.ema_alpha * (x - t.x)
                t.y += p.ema_alpha * (y - t.y)
            t.L = min(p.l_max, t.L + p.l_hit)
            t.hits += 1
            t.last_seen = now
            t.last_evidence = 'hit'
            self._add_color(t, color)
        for i, (x, y, color, can_spawn, pts) in enumerate(detections):
            if i in used_det or not can_spawn:
                continue
            t = EvTrack(self._next_id, float(x), float(y), p.l_hit, now, hits=1,
                        radius=p.radius_prior)
            self._add_points(t, pts, view)
            t.px_params = (p.px_min_frac, p.px_dominance, p.px_min_visible)
            self._add_color(t, color)
            self._next_id += 1
            self.tracks.append(t)
            used_trk.add(t.id)

        # ② 연결 안 된 부표 — 라이다 빔 근거
        if view is not None:
            for t in self.tracks:
                if t.id in used_trk:
                    continue
                ev = beam_evidence(view, t.x, t.y, p)
                t.last_evidence = ev
                if ev == 'free':
                    t.L = max(p.l_min, t.L + p.l_free_rate * dt)

        # ③ 상태 전이 (배에서 forget_range 넘게 멀어진 부표는 지움)
        keep = []
        for t in self.tracks:
            if view is not None and math.hypot(t.x - view.x, t.y - view.y) > p.forget_range:
                continue
            if not t.confirmed:
                if t.L >= p.l_confirm:
                    t.confirmed = True
                elif t.L < 0.0 or now - t.last_seen > p.tentative_timeout:
                    continue
            elif t.L <= p.l_delete:
                continue
            keep.append(t)
        self.tracks = keep
        self._merge()
        return self.confirmed()

    @staticmethod
    def _add_color(t: EvTrack, color):
        """색 표 — 문자열('red'|'green'|'unknown') 또는 픽셀 수(red/green/visible/usable 속성)."""
        if color is None:
            return
        if isinstance(color, str):
            t.color_votes[color] = t.color_votes.get(color, 0) + 1
            return
        if not getattr(color, 'usable', False):
            return
        t.px_red += int(color.red)
        t.px_green += int(color.green)
        t.px_visible += int(color.visible)
        if color.color is not None:
            t.color_votes[color.color] = t.color_votes.get(color.color, 0) + 1

    @staticmethod
    def _assoc_distance(t: EvTrack, x, y, pts) -> float:
        """검출 ↔ 부표 거리. 군집 점이 있으면 점 평균(앞면)과 부표 원 둘레 사이 거리 —
        부표 위치는 원 중심인데 군집은 앞면이라, 중심끼리 재면 같은 부표가 새 부표로 갈렸다."""
        if pts is not None and len(pts):
            m = np.asarray(pts, dtype=float).reshape(-1, 2).mean(axis=0)
            return abs(math.hypot(m[0] - t.x, m[1] - t.y) - t.radius)
        return math.hypot(t.x - x, t.y - y)

    def _add_points(self, t: EvTrack, pts, view: Optional[LidarView]):
        if pts is None or len(pts) == 0:
            return
        pts = np.asarray(pts, dtype=float).reshape(-1, 2)
        if view is not None:
            m = pts.mean(axis=0)
            ang = math.atan2(m[1] - view.y, m[0] - view.x)
        else:
            ang = float('nan')
        v = np.full(len(pts), ang)
        t.pts = pts if t.pts is None else np.vstack((t.pts, pts))[-self.p.fit_max_points:]
        t.views = v if t.views is None else np.concatenate((t.views, v))[-self.p.fit_max_points:]

    def _merge(self):
        conf = sorted(self.confirmed(), key=lambda t: (-t.hits, -t.L))   # 많이 잡힌 쪽을 남긴다
        drop = set()
        for i, a in enumerate(conf):
            if a.id in drop:
                continue
            for b in conf[i + 1:]:
                if b.id in drop or math.hypot(a.x - b.x, a.y - b.y) > self.p.merge_radius:
                    continue
                for c, n in b.color_votes.items():
                    a.color_votes[c] = a.color_votes.get(c, 0) + n
                a.px_red += b.px_red
                a.px_green += b.px_green
                a.px_visible += b.px_visible
                if b.pts is not None:
                    a.pts = b.pts if a.pts is None else np.vstack((a.pts, b.pts))[-self.p.fit_max_points:]
                    a.views = (b.views if a.views is None
                               else np.concatenate((a.views, b.views))[-self.p.fit_max_points:])
                a.hits += b.hits
                a.L = max(a.L, b.L)
                drop.add(b.id)
        if drop:
            self.tracks = [t for t in self.tracks if t.id not in drop]
