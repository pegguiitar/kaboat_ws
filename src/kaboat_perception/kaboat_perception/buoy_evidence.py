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
  새 부표는 호출자가 허락한 검출(can_spawn — 노드는 카메라 시야 ±45°·20 m 안)에서만 만든다.
  옆·뒤의 물가·다리 기둥·정박 배가 부표가 되지 않게. 이미 있는 부표는 어느 방향 검출로든 갱신.
  시간이 1 s 넘게 거꾸로 가면(bag 반복 재생·시계 재설정) 처음부터 다시 시작한다.
  확정 부표 둘이 merge_radius 안이면 L 큰 쪽으로 합친다.

색: 프레임별 표 ('red' | 'green' | 'unknown', None 은 표 없음). 빨강·초록 중 많은 쪽이
  2표 이상이고 전체 표의 25 % 이상이면 그 색, 아니면 unknown (buoy_tracker 와 같은 규칙).
  표가 하나도 없으면 'unseen' — 카메라에 한 번도 안 찍힌 라이다 물체(옆·뒤의 물가, 다리 기둥).
  unknown(= 검정 후보)과 섞이지 않게 따로 둔다.

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
    gate_radius: float = 1.0        # 군집 ↔ 부표 연결 최대 거리 [m] (부표는 고정 — 1.5 는 배를 따라
                                    # 움직이는 수면 hit 를 한 부표로 끌고 다녔다)
    ema_alpha: float = 0.3          # 위치 평활 (새 관측 가중치)
    l_hit: float = 0.8              # 라이다 군집 연결 1회
    l_free_rate: float = -2.0       # 빔이 부표 자리를 지나감 [/s]
    l_confirm: float = 2.0          # 후보 → 확정 (≈ 88 %)
    l_delete: float = -1.0          # 확정 → 삭제 (≈ 27 %)
    l_min: float = -2.0
    l_max: float = 4.0
    tentative_timeout: float = 3.0  # 후보가 이만큼 안 잡히면 삭제 [s]
    merge_radius: float = 0.8       # 확정 부표 병합 거리 [m]
    pass_margin: float = 0.5        # 빔이 부표보다 이만큼 멀리/가까이서 맞아야 지나감/가림 [m]
    beam_half_width: float = 0.15   # 부표 방향 빔 창 반폭 — 부표 반폭 [m]
    min_beam_half_angle: float = math.radians(0.5)


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
    radius: float = 0.0           # 라이다 군집 반지름 평균 [m] (0 = 모름)

    @property
    def color(self) -> str:
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


def beam_evidence(view: LidarView, x: float, y: float, p: EvidenceParams) -> str:
    """부표 자리 (x, y) 에 대한 이 스캔의 근거 → 'free' | 'occluded' | 'consistent' | 'none'."""
    dx, dy = x - view.x, y - view.y
    d = math.hypot(dx, dy)
    if d < view.range_min + p.pass_margin or d > view.range_max - p.pass_margin:
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
        """detections = [(x, y, 색 표 또는 None[, can_spawn=True[, 반지름]])] (전역),
        view = 이 스캔 (없으면 깎지 않음)."""
        p = self.p
        if self._last_now is not None and now < self._last_now - TIME_JUMP_BACK:
            self.reset()
        detections = [(*d, True, 0.0)[:5] if len(d) == 3 else (*d, 0.0)[:5] for d in detections]
        dt = 0.0 if self._last_now is None else min(max(now - self._last_now, 0.0), MAX_STEP)
        self._last_now = now

        # ① 연결 — 가까운 쌍부터 1:1
        pairs = []
        for i, (x, y, _c, _s, _r) in enumerate(detections):
            for t in self.tracks:
                d = math.hypot(t.x - x, t.y - y)
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
            x, y, color, _s, r = detections[i]
            t.x += p.ema_alpha * (x - t.x)
            t.y += p.ema_alpha * (y - t.y)
            if r > 0.0:
                t.radius = r if t.radius <= 0.0 else t.radius + p.ema_alpha * (r - t.radius)
            t.L = min(p.l_max, t.L + p.l_hit)
            t.hits += 1
            t.last_seen = now
            t.last_evidence = 'hit'
            if color is not None:
                t.color_votes[color] = t.color_votes.get(color, 0) + 1
        for i, (x, y, color, can_spawn, r) in enumerate(detections):
            if i in used_det or not can_spawn:
                continue
            t = EvTrack(self._next_id, float(x), float(y), p.l_hit, now, hits=1, radius=float(r))
            if color is not None:
                t.color_votes[color] = 1
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

        # ③ 상태 전이
        keep = []
        for t in self.tracks:
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

    def _merge(self):
        conf = sorted(self.confirmed(), key=lambda t: -t.L)
        drop = set()
        for i, a in enumerate(conf):
            if a.id in drop:
                continue
            for b in conf[i + 1:]:
                if b.id in drop or math.hypot(a.x - b.x, a.y - b.y) > self.p.merge_radius:
                    continue
                for c, n in b.color_votes.items():
                    a.color_votes[c] = a.color_votes.get(c, 0) + n
                drop.add(b.id)
        if drop:
            self.tracks = [t for t in self.tracks if t.id not in drop]
