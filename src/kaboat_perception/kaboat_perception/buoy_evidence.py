"""buoy_evidence — 부표별 존재 log-odds 트래커 (ROS 무의존 순수 모듈).

점유지도가 칸마다 하는 log-odds 를 부표 하나마다 한다. 위치는 원시 라이다 덩어리
(실수 좌표, 칸으로 뭉개지 않음), 존재는 Track.L 이 쌓는다. 근거는 라이다만 쓴다 —
카메라는 색 표만 넣는다 (buoy_color).

존재 근거 (스캔마다):
  올림  라이다 덩어리가 이 부표와 연결됨                    L += l_hit
  깎음  부표 방향 빔이 부표 자리를 지나 더 멀리서 맞음       L += l_free_rate · dt
  유지  부표보다 앞에서 맞음(가림), 부표 거리에서 맞았지만 덩어리가 안 됨,
        반사 없음(검정 무광은 빛을 흡수할 수 있다), 라이다 거리 범위 밖, 스캔 없음
  → "보였어야 하는데 안 보였을 때"만 지운다. 시야 밖·가림·센서 없음은 근거가 아니다.

상태 (히스테리시스):
  후보 —(L ≥ l_confirm, 히트 confirm_min_hits 이상, confirm_min_age 지남)→ 확정 —(L ≤ l_delete)→ 삭제
  후보는 L < 0 이 되거나 tentative_timeout 동안 다시 안 잡히면 삭제 (물보라·반짝임).
  새 부표는 호출자가 허락한 검출(can_spawn)에서만 만든다. 기존 부표 중심 spawn_min_sep 안에서는
  새 부표를 안 만들고, 확정 부표가 그 안으로 다가오면 합친다 — 같은 부표가 다른 방향에서 둘로
  갈리지 않게 (gate bag 0.5~0.9 m 중복 → 0). 탐색 bag 의 붙은 초록·검정 쌍은 1.37 m 라 따로.
  시간이 1 s 넘게 거꾸로 가면(bag 반복 재생·시계 재설정) 처음부터 다시 시작한다.
  배에서 forget_range(17 m) 넘게 멀어진 부표는 지운다 — 점유지도가 배 주변 창 밖을 버리듯.

위치·크기 (덩어리): 부표를 원으로 가정하지 않는다. 스캔마다 연결된 라이다 덩어리의 중심(점 평균)과
  반폭(가장 먼 두 점 거리 / 2)을 부표마다 최근 blob_window 스캔만큼 모아
    위치 = 덩어리 중심들의 중앙값 (x, y 따로) — 번짐·튀는 스캔에 강하다
    크기 = 반폭들의 size_pct 백분위 (radius_min~radius_max)
  덩어리 중심은 보이는 면 쪽으로 조금(부표 반폭 이하) 치우치지만 여러 방향에서 보면 평균된다.
  (예전엔 원 맞추기·면 뒤 제약·반지름 키우기를 했는데 번진 점에서 자꾸 엇나갔다.)

색: 창 픽셀 수 누적 (red/green/visible) — 빨강 또는 초록이 보이는 픽셀의 px_min_frac 이상·다른 색의
  px_dominance 배 이상이면 그 색, 아니면 unknown. 누적 px_min_visible 미만이면 프레임 표 규칙
  (빨강·초록 중 많은 쪽이 2표 이상·25 % 이상). 표도 없으면 'unseen' — 카메라에 한 번도 안 찍힌
  라이다 물체. unknown(= 검정 후보)과 섞이지 않게 따로 둔다.

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
    gate_radius: float = 0.5        # 덩어리 중심 ↔ 부표 중심 연결 거리 = 부표 반폭 + 이만큼 [m]
                                    # (스캔 번짐 RMS ≈ 0.15 m 의 3배)
    l_hit: float = 0.8              # 라이다 덩어리 연결 1회
    l_free_rate: float = -2.0       # 빔이 부표 자리를 지나감 [/s]
    l_confirm: float = 2.0          # 후보 → 확정 (≈ 88 %)
    l_delete: float = -1.0          # 확정 → 삭제 (≈ 27 %)
    l_min: float = -2.0
    l_max: float = 4.0
    tentative_timeout: float = 3.0  # 후보가 이만큼 안 잡히면 삭제 [s]
    confirm_min_hits: int = 4       # 확정에 필요한 최소 히트 수 — L 만으로는 3 연속 히트(0.3 s)면 확정돼
                                    # 배 옆 물보라 덩어리가 부표가 됐다 (2026-11-12 탐색 bag)
    confirm_min_age: float = 0.5    # 처음 잡힌 뒤 이만큼 지나야 확정 [s]
    merge_radius: float = 1.0       # 확정 부표 병합 거리 상한 [m] — 그 안에서도 두 덩어리가 겹치거나
                                    # (중심 거리 ≤ 두 반폭 합 + merge_overlap) spawn_min_sep 안일 때만
    merge_overlap: float = 0.1      # [m]
    spawn_min_sep: float = 1.0      # 기존 부표 중심 이 안에서는 새 부표를 만들지 않음 [m]
    pass_margin: float = 0.5        # 빔이 부표보다 이만큼 멀리/가까이서 맞아야 지나감/가림 [m]
    beam_half_width: float = 0.15   # 부표 방향 빔 창 반폭 하한 [m] (부표 반폭이 더 크면 그것)
    min_beam_half_angle: float = math.radians(0.5)
    max_range: float = 15.0         # 보는 거리 — 라이다 원점에서 이보다 먼 부표는 깎지 않음
                                    # (노드는 덩어리도 이 안만, cand_max_range) [m]
    forget_range: float = 17.0      # 이보다 멀어진 부표는 지움 — 점유지도 창처럼 배 주변만. 보는 거리보다
                                    # 조금 크게: 경계에서 생겼다 지워졌다 깜빡이지 않게 [m]
    # 위치·크기 (덩어리)
    blob_window: int = 30           # 위치·크기에 쓰는 최근 스캔 수
    size_pct: float = 80.0          # 최근 반폭의 이 백분위 = 부표 반폭 (튀는 값 제외)
    radius_min: float = 0.1         # 반폭 하한 [m]
    radius_max: float = 0.75        # 반폭 상한 [m] (지름 1.5 m)
    radius_prior: float = 0.2       # 덩어리 폭을 모를 때 (점 하나·폭 없음) [m]
    # 색 — 부표마다 창 픽셀 수 누적 (빨강·초록·보이는). 붙은 쌍에서 옆 부표가 한두 프레임 섞여도
    # 자기 색 픽셀이 훨씬 많아 갈린다 (프레임마다 표 한 장씩 세면 섞인 표가 같은 무게였다).
    px_min_frac: float = 0.15       # 빨강 또는 초록이 보이는 픽셀의 이 비율 이상이어야 그 색
    px_dominance: float = 2.0       # 다른 색 픽셀의 이 배 이상이어야 그 색
    px_min_visible: int = 150       # 누적 보이는 픽셀이 이보다 적으면 표 규칙으로 (그것도 없으면 unseen)


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
    radius: float = 0.2           # 부표 반폭 [m] — 최근 덩어리 반폭의 size_pct 백분위
    centers: List[Tuple[float, float]] = field(default_factory=list)   # 최근 덩어리 중심 (전역)
    widths: List[float] = field(default_factory=list)                   # 최근 덩어리 반폭 [m]
    px_red: int = 0               # 누적 창 픽셀 (usable 프레임만)
    px_green: int = 0
    px_visible: int = 0
    px_params: Optional[Tuple[float, float, int]] = None   # (min_frac, dominance, min_visible)
    born: float = 0.0             # 처음 잡힌 시각 [s]

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
    def color_frac(self) -> float:
        """색 순도 0–1 — 판정한 색(빨강·초록)의 근거 비율. 누적 픽셀이 충분하면 그 색 픽셀 / 보이는 픽셀,
        아니면 그 색 표 / 전체 표. unknown 은 빨강·초록 중 큰 쪽 비율, unseen 은 0.
        탐색이 검정 목표인데 unknown 이 없고 같은 색이 둘일 때 약한 쪽을 검정으로 고르는 데 쓴다
        (2026-11-14 오전 run1: 흐린 날 검정이 green 으로 굳어 탐색 포기)."""
        color = self.color
        _min_frac, _dominance, min_visible = self.px_params or (0.15, 2.0, 150)
        if self.px_visible >= min_visible:
            n = {'red': self.px_red, 'green': self.px_green}.get(color, max(self.px_red, self.px_green))
            return min(1.0, n / self.px_visible)
        total = sum(self.color_votes.values())
        if total == 0:
            return 0.0
        n = self.color_votes.get(color) if color in KNOWN_COLORS else max(
            (self.color_votes.get(c, 0) for c in KNOWN_COLORS), default=0)
        return min(1.0, (n or 0) / total)

    @property
    def probability(self) -> float:
        return 1.0 / (1.0 + math.exp(-self.L))


def beam_evidence(view: LidarView, x: float, y: float, p: EvidenceParams,
                  half_width: float = 0.0) -> str:
    """부표 자리 (x, y) 에 대한 이 스캔의 근거 → 'free' | 'occluded' | 'consistent' | 'none'.
    half_width = 부표 반폭 — 빔 창을 이만큼 넓힌다 (하한 beam_half_width)."""
    dx, dy = x - view.x, y - view.y
    d = math.hypot(dx, dy)
    if (d < view.range_min + p.pass_margin or d > view.range_max - p.pass_margin
            or d > p.max_range):
        return 'none'
    bearing = math.atan2(dy, dx) - view.yaw
    half = max(p.min_beam_half_angle, math.atan2(max(p.beam_half_width, half_width), d))
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


def blob_stats(pts, x, y):
    """덩어리 점 N×2 → (중심 x, 중심 y, 반폭). 점이 없으면 (x, y, None)."""
    if pts is None or len(pts) == 0:
        return float(x), float(y), None
    b = np.asarray(pts, dtype=float).reshape(-1, 2)
    m = b.mean(axis=0)
    if len(b) < 2:
        return float(m[0]), float(m[1]), None
    d = b[:, None, :] - b[None, :, :]
    return float(m[0]), float(m[1]), 0.5 * float(np.sqrt((d ** 2).sum(-1)).max())


class EvidenceTracker:
    def __init__(self, params: EvidenceParams = None):
        self.p = params or EvidenceParams()
        self.tracks: List[EvTrack] = []
        self._next_id = 0
        self._last_now: Optional[float] = None
        self.last_assoc: Dict[int, int] = {}   # 마지막 update 의 검출 번호 → 부표 id (시각화용)

    def confirmed(self) -> List[EvTrack]:
        return [t for t in self.tracks if t.confirmed]

    def reset(self):
        self.tracks = []
        self._last_now = None

    def update(self, detections: Sequence[Tuple], now: float,
               view: Optional[LidarView] = None) -> List[EvTrack]:
        """detections = [(x, y, 색 표 또는 None[, can_spawn=True[, 덩어리 점 N×2 (전역)]])],
        view = 이 스캔 (없으면 깎지 않음). 점이 있으면 위치·크기는 점에서 잰다."""
        p = self.p
        if self._last_now is not None and now < self._last_now - TIME_JUMP_BACK:
            self.reset()
        detections = [(*d, True, None)[:5] if len(d) == 3 else (*d, None)[:5] for d in detections]
        blobs = [blob_stats(d[4], d[0], d[1]) for d in detections]
        dt = 0.0 if self._last_now is None else min(max(now - self._last_now, 0.0), MAX_STEP)
        self._last_now = now

        # ① 연결 — 가까운 쌍부터 1:1. 덩어리 중심이 부표 중심에서 (부표 반폭 + gate_radius) 안
        pairs = []
        for i, (bx, by, _w) in enumerate(blobs):
            for t in self.tracks:
                d = math.hypot(t.x - bx, t.y - by)
                if d <= t.radius + p.gate_radius:
                    pairs.append((d, i, t.id))
        pairs.sort()
        used_det, used_trk = set(), set()
        self.last_assoc = {}
        by_id = {t.id: t for t in self.tracks}
        for _d, i, tid in pairs:
            if i in used_det or tid in used_trk:
                continue
            used_det.add(i)
            used_trk.add(tid)
            t = by_id[tid]
            self.last_assoc[i] = tid
            self._add_blob(t, *blobs[i])
            t.L = min(p.l_max, t.L + p.l_hit)
            t.hits += 1
            t.last_seen = now
            t.last_evidence = 'hit'
            self._add_color(t, detections[i][2])
        existing = [(t.x, t.y) for t in self.tracks]
        for i, (_x, _y, color, can_spawn, _pts) in enumerate(detections):
            if i in used_det or not can_spawn:
                continue
            bx, by, w = blobs[i]
            if any(math.hypot(bx - ex, by - ey) < p.spawn_min_sep for ex, ey in existing):
                continue                   # 기존 부표 바로 옆 — 같은 부표의 다른 면일 가능성이 크다
            existing.append((bx, by))      # 같은 스캔에서 쪼개진 덩어리 둘이 둘 다 생기지 않게
            t = EvTrack(self._next_id, bx, by, p.l_hit, now, hits=1, radius=p.radius_prior, born=now)
            self.last_assoc[i] = t.id
            self._add_blob(t, bx, by, w)
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
                ev = beam_evidence(view, t.x, t.y, p, t.radius)
                t.last_evidence = ev
                if ev == 'free':
                    t.L = max(p.l_min, t.L + p.l_free_rate * dt)

        # ③ 상태 전이 (배에서 forget_range 넘게 멀어진 부표는 지움)
        keep = []
        for t in self.tracks:
            if view is not None and math.hypot(t.x - view.x, t.y - view.y) > p.forget_range:
                continue
            if not t.confirmed:
                if (t.L >= p.l_confirm and t.hits >= p.confirm_min_hits
                        and now - t.born >= p.confirm_min_age):
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

    def _add_blob(self, t: EvTrack, x, y, half_width):
        """덩어리 하나 반영 — 위치 = 최근 중심들의 중앙값, 크기 = 최근 반폭들의 상위 백분위."""
        p = self.p
        t.centers = (t.centers + [(float(x), float(y))])[-p.blob_window:]
        c = np.asarray(t.centers)
        t.x, t.y = float(np.median(c[:, 0])), float(np.median(c[:, 1]))
        if half_width is not None and math.isfinite(half_width):
            t.widths = (t.widths + [float(half_width)])[-p.blob_window:]
        if t.widths:
            r = float(np.percentile(t.widths, p.size_pct))
            t.radius = min(p.radius_max, max(p.radius_min, r))

    def _merge(self):
        conf = sorted(self.confirmed(), key=lambda t: (-t.hits, -t.L))   # 많이 잡힌 쪽을 남긴다
        drop = set()
        for i, a in enumerate(conf):
            if a.id in drop:
                continue
            for b in conf[i + 1:]:
                d = math.hypot(a.x - b.x, a.y - b.y)
                close = max(a.radius + b.radius + self.p.merge_overlap, self.p.spawn_min_sep)
                if b.id in drop or d > self.p.merge_radius or d > close:
                    continue
                for c, n in b.color_votes.items():
                    a.color_votes[c] = a.color_votes.get(c, 0) + n
                a.px_red += b.px_red
                a.px_green += b.px_green
                a.px_visible += b.px_visible
                a.hits += b.hits
                a.L = max(a.L, b.L)
                drop.add(b.id)
        if drop:
            self.tracks = [t for t in self.tracks if t.id not in drop]
