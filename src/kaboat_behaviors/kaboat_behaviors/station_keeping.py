"""station_keeping — 위치유지(station) 미션 전용 계산 (ROS 무의존).

경기: 부표 4개(약 3 m × 3 m, 파도에 흔들려 모양이 일정하지 않음)가 꼭짓점인 사각형
안에 들어가 그 자리를 유지한다. 사각형 주변 6 m(라이다 신뢰 거리) 안에는 다른 장애물이 없다.

1) StationMap — station 전용 log-odds 격자
   공용 /occupancy_grid 는 빔이 지나간 칸을 스캔마다 L_FREE(-0.2) 깎는다. 라이다가 앞만
   보는데 배가 사각형 안에서 흔들리면 빔이 옆·뒤 부표 위로 지나가며 부표를 지운다. 그래서
   station 은 /scan/level 로 자기 지도를 따로 쌓고, **큰 덩어리**(점유 칸이 big_min_cells
   = 4칸 이상 이어진 것 = 부표)만 감쇠에 free_ratio(0.1)를 곱해 오래 기억한다. 작은 덩어리
   (해수면 반사 등 노이즈, 보통 1~2칸)는 공용 지도와 똑같이 깎여 금방 사라진다.
   공용 지도와 다른 behavior 는 그대로다.
   격자는 미션 목표(station_end) 중심에 고정된 정사각형이다(구역이 고정이라 rolling 불필요).
2) find_buoys — 부표 크기 덩어리 중 목표 반경 안의 강한 순 최대 4개.
3) estimate_center — 부표 수별 사각형 중심.
     2개: 한 변이면 중점에서 안쪽으로 간격/2, 대각선이면 중점
     3개: 가장 먼 쌍 = 대각선 → 그 중점
     4개: 평균
4) 입구 — 처음 보이는 한 변(가장 가까운 쌍)을 입구로 고정하고, 입구 앞 정렬점을 거쳐
   중심으로 들어간다. 부표 사이를 지나므로 공용 waypoint()(전진 12 N)가 아니라 가상 앵커
   (전진 상한 max_forward 0.25 = 게이트 LOS 직진 상한)로 가서 해류에 밀리지 않게 한다.
5) anchor_command — 수조 station_keeping_test 의 가상 앵커(위치 P + 선수각 PD + 불감대,
   가까운 오버슈트는 저속 후진)를 순수 함수로 옮긴 것. 선수각은 HOLD 진입 때 방향 유지.
6) StationFsm — SEARCH → ENTER → HOLD. 반경 hold_radius 안에서 hold_time 연속 유지하면
   완료 1회 보고(다음 미션 전환은 mission_manager 몫). 완료 뒤에도 전환 전까지 계속 유지.
"""

from dataclasses import dataclass, field
import math

import numpy as np
from scipy import ndimage


def _wrap(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def _clamp(value: float, limit: float) -> float:
    return max(-limit, min(limit, value))


@dataclass(frozen=True)
class StationParams:
    # ── station 전용 지도 ──
    resolution: float = 0.1         # [m] 셀
    map_half_size: float = 7.0      # [m] 목표 중심 ±7 m (부표 반경 4 m + 여유)
    max_range: float = 6.0          # [m] 라이다 신뢰 거리. 이보다 먼 히트는 안 씀
    l_occ: float = 0.8              # 히트 log-odds (공용 occupancy_grid L_OCC 와 같음)
    l_free: float = -0.2            # free 감쇠 (공용 L_FREE 와 같음) — 작은 덩어리(노이즈)
    free_ratio: float = 0.1         # 큰 덩어리(부표)만 감쇠 × 이 비율 (−0.2 × 0.1 = −0.02/스캔)
    big_min_cells: int = 4          # 점유(≥ occ_logodds) 칸이 이만큼 이어지면 큰 덩어리.
    # 지름 0.4 m 부표는 0.1 m 격자에서 4~9칸(거리 1.5~5.5 m, 흔들림 포함 시뮬 실측),
    # 해수면 반사는 보통 1~2칸.
    l_min: float = -2.0             # 공용(-4.5)보다 높게: 흔들려 옆 칸으로 간 부표를 빨리 다시 잡음
    l_max: float = 8.0              # 공용 L_MAX 와 같음
    # ── 부표 ──
    occ_logodds: float = 2.0        # 이 이상인 칸만 부표 후보 (히트 3스캔)
    min_cluster_cells: int = 2
    max_cluster_extent: float = 1.2  # [m] 흔들림으로 번진 부표까지
    buoy_search_radius: float = 4.0  # [m] 목표(station_end) 기준 부표 후보 반경
    merge_distance: float = 0.8     # [m] 이보다 가까운 덩어리는 같은 부표(약한 쪽 버림)
    side_length: float = 3.0        # [m] 사각형 한 변 (공칭)
    min_side: float = 1.5           # [m] 한 변으로 볼 최소 부표 간격
    # ── 입구 ──
    pre_gate_distance: float = 1.0  # [m] 입구 중점 앞 정렬점
    gate_lateral_tol: float = 0.4   # [m] 입구 수직이등분선에서 이만큼 안이면 바로 중심으로
    track_radius: float = 1.0       # [m] 입구 부표 재연결 반경
    # ── SEARCH 접근 (base waypoint(): 전진 상한 max_linear 12 N × search_ratio) ──
    search_ratio: float = 1.0
    search_slow_radius: float = 2.0
    # ── 유지·완료 ──
    hold_radius: float = 1.0        # [m] 중심에서 이 안이면 '유지 중'
    hold_time: float = 5.0          # [s] 연속 유지 시간 → 완료 보고
    fallback_to_goal: bool = True   # 부표를 못 찾으면 station_end 를 중심으로 유지
    # ── 가상 앵커 (수조 station_keeping_test 값, 상한은 LOS 직진·회전 상한) ──
    pos_deadband: float = 0.5
    yaw_deadband_deg: float = 8.0
    max_forward: float = 0.25
    max_reverse: float = 0.08
    max_turn: float = 0.31
    min_forward: float = 0.04
    kp_pos: float = 0.25
    kp_yaw: float = 1.5
    kd_yaw: float = 0.15
    reverse_band: float = 0.3       # [m] 불감대 + 이 거리 안의 뒤쪽 목표는 후진으로

    def __post_init__(self):
        floats = [v for v in vars(self).values() if not isinstance(v, bool)]
        if not all(math.isfinite(float(v)) for v in floats):
            raise ValueError('station parameters must be finite')
        positive = (self.resolution, self.map_half_size, self.max_range, self.l_occ,
                    self.occ_logodds, self.max_cluster_extent, self.buoy_search_radius,
                    self.side_length, self.min_side, self.pre_gate_distance,
                    self.gate_lateral_tol, self.track_radius, self.search_slow_radius,
                    self.hold_radius, self.pos_deadband, self.yaw_deadband_deg,
                    self.max_forward, self.max_turn, self.kp_pos, self.kp_yaw)
        if (min(positive) <= 0 or self.l_free > 0 or not 0.0 <= self.free_ratio <= 1.0
                or self.l_min >= 0 or self.l_max <= self.occ_logodds
                or self.min_cluster_cells < 1 or self.big_min_cells < 1 or self.hold_time < 0
                or self.min_side >= self.side_length
                or not 0.0 <= self.max_reverse <= 1.0 or self.max_forward > 1.0
                or self.max_turn > 1.0 or self.kd_yaw < 0 or self.reverse_band < 0
                or not 0.0 < self.search_ratio <= 1.0):
            raise ValueError('invalid station parameters')

    @property
    def diagonal_threshold(self) -> float:
        """이보다 먼 부표 쌍은 대각선 — 한 변(s)과 대각선(√2·s)의 중간."""
        return self.side_length * (1.0 + math.sqrt(2.0)) / 2.0


# ── 1) station 전용 지도 ──────────────────────────────

class StationMap:
    """목표 중심에 고정된 log-odds 격자. [iy, ix] 순서."""

    def __init__(self, center_xy, params: StationParams):
        self.p = params
        self.center = (float(center_xy[0]), float(center_xy[1]))
        self.n = int(round(2.0 * params.map_half_size / params.resolution))
        self.origin = (self.center[0] - params.map_half_size,
                       self.center[1] - params.map_half_size)
        self.logodds = np.zeros((self.n, self.n), dtype=np.float32)
        self.observed = np.zeros((self.n, self.n), dtype=bool)

    def _cells(self, xs, ys):
        res = self.p.resolution
        ix = np.floor((xs - self.origin[0]) / res).astype(np.int64)
        iy = np.floor((ys - self.origin[1]) / res).astype(np.int64)
        inside = (ix >= 0) & (ix < self.n) & (iy >= 0) & (iy < self.n)
        return np.unique(iy[inside] * self.n + ix[inside])

    def integrate_scan(self, ranges, angle_min, angle_increment, range_min, range_max,
                       lidar_xy, yaw):
        """스캔 1장을 누적한다. 히트는 셀당 1회 l_occ, 빔 경로는 셀당 1회 l_free.

        빔 경로 중 큰 덩어리(부표)에 속한 칸만 l_free × free_ratio 로 덜 깎는다.
        """
        p = self.p
        ranges = np.asarray(ranges, dtype=np.float64)
        if ranges.size == 0:
            return
        angles = angle_min + angle_increment * np.arange(ranges.size)
        valid = (np.isfinite(ranges) & (ranges > range_min) & (ranges < range_max))
        if not valid.any():
            return
        r = ranges[valid]
        wa = yaw + angles[valid]
        cos_a, sin_a = np.cos(wa), np.sin(wa)
        lx, ly = float(lidar_xy[0]), float(lidar_xy[1])

        hit = r <= p.max_range
        hit_flat = self._cells(lx + r[hit] * cos_a[hit], ly + r[hit] * sin_a[hit])

        # 빔 경로: 반 셀 간격 표본. 히트 빔은 끝 한 셀 앞까지, 먼 빔은 신뢰 거리까지.
        step = 0.5 * p.resolution
        dist = np.arange(0.0, p.max_range, step)
        free_len = np.where(hit, r - p.resolution, p.max_range)
        mask = dist[None, :] < free_len[:, None]
        xs = (lx + dist[None, :] * cos_a[:, None])[mask]
        ys = (ly + dist[None, :] * sin_a[:, None])[mask]
        free_flat = np.setdiff1d(self._cells(xs, ys), hit_flat, assume_unique=True)

        flat = self.logodds.reshape(-1)
        decay = np.full(free_flat.size, p.l_free, dtype=np.float32)
        if free_flat.size and p.free_ratio < 1.0:
            decay[self.big_mask().reshape(-1)[free_flat]] *= p.free_ratio
        flat[free_flat] += decay
        flat[hit_flat] += p.l_occ
        np.clip(self.logodds, p.l_min, p.l_max, out=self.logodds)
        seen = self.observed.reshape(-1)
        seen[free_flat] = True
        seen[hit_flat] = True

    def big_mask(self) -> np.ndarray:
        """점유(≥ occ_logodds) 칸이 8방향으로 big_min_cells 이상 이어진 덩어리의 칸들."""
        p = self.p
        labels, count = ndimage.label(self.logodds >= p.occ_logodds,
                                      structure=np.ones((3, 3), dtype=np.uint8))
        if count == 0:
            return np.zeros_like(self.observed)
        big = np.bincount(labels.ravel()) >= p.big_min_cells
        big[0] = False
        return big[labels]

    def probability_percent(self) -> np.ndarray:
        """디버그 발행용 0~100 (미관측 -1), [iy, ix]."""
        prob = np.rint(100.0 / (1.0 + np.exp(-self.logodds)))
        return np.where(self.observed, prob, -1).astype(np.int8)


# ── 2) 부표 ──────────────────────────────────────────

@dataclass(frozen=True)
class Buoy:
    x: float
    y: float
    strength: float   # 덩어리 log-odds 합


def find_buoys(station_map: StationMap) -> list[Buoy]:
    """목표 반경 안 부표 크기 덩어리, 강한 순 최대 4개 (가까운 덩어리는 하나로)."""
    p = station_map.p
    weights = station_map.logodds
    occupied = weights >= p.occ_logodds
    labels, count = ndimage.label(occupied, structure=np.ones((3, 3), dtype=np.uint8))
    if count == 0:
        return []
    candidates = []
    for label_id, slc in enumerate(ndimage.find_objects(labels), start=1):
        if slc is None:
            continue
        local = labels[slc] == label_id
        cells = int(local.sum())
        if cells < p.min_cluster_cells:
            continue
        extent = p.resolution * max(local.shape)
        if extent > p.max_cluster_extent:
            continue
        iy, ix = np.nonzero(local)
        w = weights[slc][local].astype(np.float64)
        cx = slc[1].start + float(np.average(ix, weights=w)) + 0.5   # 가중 중심 [셀]
        cy = slc[0].start + float(np.average(iy, weights=w)) + 0.5
        x = station_map.origin[0] + cx * p.resolution
        y = station_map.origin[1] + cy * p.resolution
        if math.hypot(x - station_map.center[0], y - station_map.center[1]) > p.buoy_search_radius:
            continue
        candidates.append(Buoy(x, y, float(w.sum())))
    candidates.sort(key=lambda b: b.strength, reverse=True)
    chosen: list[Buoy] = []
    for b in candidates:
        if all(math.hypot(b.x - c.x, b.y - c.y) >= p.merge_distance for c in chosen):
            chosen.append(b)
        if len(chosen) == 4:
            break
    return chosen


# ── 3) 사각형 중심 ────────────────────────────────────

def estimate_center(points, boat_xy, params: StationParams, hint_xy=None):
    """부표 좌표 목록 → (중심 (x, y), 종류) 또는 None.

    2개가 한 변일 때 중심은 그 변의 어느 쪽인지 정해야 한다: hint_xy(직전 중심)가 있으면
    그쪽, 없으면 배 반대쪽(처음 본 변은 배 앞에 있다).
    """
    pts = [np.asarray(pt, dtype=float) for pt in points]
    if len(pts) >= 4:
        return tuple(np.mean(pts[:4], axis=0)), 'four'
    if len(pts) == 3:
        pairs = [(0, 1), (0, 2), (1, 2)]
        i, j = max(pairs, key=lambda ij: np.linalg.norm(pts[ij[0]] - pts[ij[1]]))
        if np.linalg.norm(pts[i] - pts[j]) >= params.diagonal_threshold:
            return tuple((pts[i] + pts[j]) / 2.0), 'three'
        return tuple(np.mean(pts, axis=0)), 'three_mean'
    if len(pts) == 2:
        a, b = pts
        gap = float(np.linalg.norm(b - a))
        mid = (a + b) / 2.0
        if gap >= params.diagonal_threshold:
            return tuple(mid), 'diagonal'
        if gap < params.min_side:
            return None
        normal = np.array([-(b - a)[1], (b - a)[0]]) / gap
        ref = (np.asarray(hint_xy, dtype=float) - mid if hint_xy is not None
               else mid - np.asarray(boat_xy, dtype=float))
        if float(normal @ ref) < 0.0:
            normal = -normal
        return tuple(mid + normal * gap / 2.0), 'side'
    return None


# ── 4) 입구 ──────────────────────────────────────────

@dataclass(frozen=True)
class EntryGate:
    a: tuple
    b: tuple
    normal: tuple     # 사각형 안쪽을 향하는 단위벡터

    @property
    def midpoint(self):
        return ((self.a[0] + self.b[0]) / 2.0, (self.a[1] + self.b[1]) / 2.0)

    @property
    def width(self):
        return math.hypot(self.b[0] - self.a[0], self.b[1] - self.a[1])


def _gate(a, b, inward):
    a, b = np.asarray(a, dtype=float), np.asarray(b, dtype=float)
    d = b - a
    gap = float(np.linalg.norm(d))
    normal = np.array([-d[1], d[0]]) / gap
    if float(normal @ np.asarray(inward, dtype=float)) < 0.0:
        normal = -normal
    return EntryGate(tuple(a), tuple(b), tuple(normal))


def choose_entry_gate(points, boat_xy, params: StationParams):
    """한 변 간격 쌍 중 중점이 배에 가장 가까운 쌍 = 입구. 없으면 None."""
    boat = np.asarray(boat_xy, dtype=float)
    best = None
    for i in range(len(points)):
        for j in range(i + 1, len(points)):
            a, b = np.asarray(points[i], float), np.asarray(points[j], float)
            gap = float(np.linalg.norm(b - a))
            if not params.min_side <= gap < params.diagonal_threshold:
                continue
            mid = (a + b) / 2.0
            d = float(np.linalg.norm(mid - boat))
            if best is None or d < best[0]:
                best = (d, a, b, mid)
    if best is None:
        return None
    _, a, b, mid = best
    return _gate(a, b, mid - boat)


def track_gate(gate: EntryGate, points, params: StationParams) -> EntryGate:
    """고정한 입구의 두 부표를 이번 부표 목록에 다시 붙인다 (흔들림 추종). 못 찾으면 그대로."""
    pts = [np.asarray(pt, dtype=float) for pt in points]

    def nearest(ref, exclude):
        best = None
        for k, pt in enumerate(pts):
            if k == exclude:
                continue
            d = float(np.linalg.norm(pt - np.asarray(ref)))
            if d <= params.track_radius and (best is None or d < best[0]):
                best = (d, k)
        return best

    ma = nearest(gate.a, None)
    mb = nearest(gate.b, ma[1] if ma else None)
    a = pts[ma[1]] if ma else np.asarray(gate.a)
    b = pts[mb[1]] if mb else np.asarray(gate.b)
    if float(np.linalg.norm(b - a)) < params.min_side:
        return gate
    return _gate(a, b, gate.normal)


def entry_target(boat_xy, gate: EntryGate, center, params: StationParams):
    """입구 밖·수직이등분선에서 벗어나 있으면 입구 앞 정렬점, 아니면 중심 → (목표, 단계)."""
    n = np.asarray(gate.normal)
    t = np.array([-n[1], n[0]])
    mid = np.asarray(gate.midpoint)
    rel = np.asarray(boat_xy, dtype=float) - mid
    along, lateral = float(rel @ n), float(rel @ t)
    pre = mid - n * params.pre_gate_distance
    if (along >= -params.gate_lateral_tol or abs(lateral) <= params.gate_lateral_tol
            or float(np.linalg.norm(np.asarray(boat_xy, dtype=float) - pre))
            <= params.pos_deadband):
        return tuple(center), 'THROUGH'
    return tuple(pre), 'ALIGN_GATE'


# ── 5) 가상 앵커 ──────────────────────────────────────

def anchor_command(boat_xy, yaw, yaw_rate, target_xy, hold_yaw, params: StationParams):
    """→ (linear, angular, 단계). linear/angular 는 최대추력 비율."""
    p = params
    dx, dy = target_xy[0] - boat_xy[0], target_xy[1] - boat_xy[1]
    dist = math.hypot(dx, dy)
    rel = _wrap(math.atan2(dy, dx) - yaw)

    def turn(err):
        return _clamp(p.kp_yaw * err - p.kd_yaw * yaw_rate, p.max_turn)

    if dist <= p.pos_deadband:
        if hold_yaw is None:
            return 0.0, 0.0, 'HOLD'
        yaw_err = _wrap(hold_yaw - yaw)
        if abs(yaw_err) <= math.radians(p.yaw_deadband_deg):
            return 0.0, 0.0, 'HOLD'
        return 0.0, turn(yaw_err), 'ALIGN'
    if abs(rel) <= math.pi / 2.0:
        speed = min(p.max_forward, max(p.min_forward, p.kp_pos * dist))
        return speed * math.cos(rel), turn(rel), 'RETURN'
    if dist < p.pos_deadband + p.reverse_band and p.max_reverse > 0.0:
        rev = _wrap(rel - math.pi)
        speed = min(p.max_reverse, max(p.min_forward, p.kp_pos * dist))
        return -speed * math.cos(rev), turn(rev), 'BACK'
    return 0.0, turn(rel), 'TURN'


# ── 6) 상태기계 ──────────────────────────────────────

@dataclass
class StationDecision:
    phase: str                         # SEARCH / ENTER / HOLD
    kind: str                          # 'waypoint' (base waypoint()) | 'anchor'
    target: tuple | None = None        # 이번 주행 목표점 (HOLD 는 None — 중심)
    thrust_ratio: float = 1.0
    slow_radius: float | None = None
    linear: float = 0.0                # anchor 명령
    angular: float = 0.0
    stage: str = ''                    # 세부 단계 (로그·상태)
    center: tuple | None = None
    center_kind: str | None = None
    inside: bool = False
    hold_elapsed: float = 0.0
    completed_now: bool = False        # 이번 step 에서 완료 조건 처음 달성
    buoys: list = field(default_factory=list)
    gate: EntryGate | None = None


class StationFsm:
    """SEARCH(부표 2개 전, 목표로 접근) → ENTER(입구로 진입) → HOLD(중심 유지)."""

    def __init__(self, goal_xy, params: StationParams | None = None):
        self.p = params or StationParams()
        self.goal = (float(goal_xy[0]), float(goal_xy[1]))
        self.map = StationMap(self.goal, self.p)
        self.phase = 'SEARCH'
        self.gate: EntryGate | None = None
        self.center = None
        self.center_kind = None
        self.through = False
        self.hold_yaw = None
        self.inside_since = None
        self.completed = False

    def add_scan(self, ranges, angle_min, angle_increment, range_min, range_max,
                 lidar_xy, yaw):
        self.map.integrate_scan(ranges, angle_min, angle_increment, range_min, range_max,
                                lidar_xy, yaw)

    def step(self, boat_xy, yaw, yaw_rate, now) -> StationDecision:
        p = self.p
        boat = (float(boat_xy[0]), float(boat_xy[1]))
        buoys = find_buoys(self.map)
        pts = [(b.x, b.y) for b in buoys]

        if self.gate is not None:
            self.gate = track_gate(self.gate, pts, p)
        elif self.phase == 'SEARCH':
            self.gate = choose_entry_gate(pts, boat, p)
        estimate = estimate_center(pts, boat, p, hint_xy=self.center)
        if estimate is not None:
            self.center, self.center_kind = estimate

        def decision(**kw):
            return StationDecision(phase=self.phase, center=self.center,
                                   center_kind=self.center_kind, buoys=buoys,
                                   gate=self.gate, **kw)

        if self.phase == 'SEARCH':
            if self.gate is not None and self.center is not None:
                self.phase = 'ENTER'
            else:
                # 부표 2개 전(또는 대각선 2개만 보임): 추정 중심, 없으면 station_end 로 천천히.
                # 부표를 끝내 못 찾아도 fallback_to_goal 이면 station_end 에서 유지한다.
                target = self.center if self.center is not None else self.goal
                if ((self.center is not None or p.fallback_to_goal)
                        and math.dist(boat, target) <= p.hold_radius):
                    if self.center is None:
                        self.center, self.center_kind = self.goal, 'goal'
                    self._start_hold(yaw)
                else:
                    return decision(kind='waypoint', target=target,
                                    thrust_ratio=p.search_ratio,
                                    slow_radius=p.search_slow_radius, stage='TO_GOAL')

        if self.phase == 'ENTER':
            if math.dist(boat, self.center) <= p.hold_radius:
                self._start_hold(yaw)
            else:
                target, stage = self.center, 'DIRECT'
                if self.gate is not None:
                    if self.through:
                        stage = 'THROUGH'
                    else:
                        target, stage = entry_target(boat, self.gate, self.center, p)
                        self.through = stage == 'THROUGH'
                # 부표 사이를 지나므로 해류에 밀리지 않게 가상 앵커(전진 상한 max_forward)로 간다
                linear, angular, _ = anchor_command(boat, yaw, yaw_rate, target, None, p)
                return decision(kind='anchor', target=target, linear=linear,
                                angular=angular, stage=stage)

        # HOLD
        dist = math.dist(boat, self.center)
        inside = dist <= p.hold_radius
        completed_now = False
        elapsed = 0.0
        if inside:
            if self.inside_since is None or now < self.inside_since:
                self.inside_since = now
            elapsed = now - self.inside_since
            if not self.completed and elapsed >= p.hold_time:
                self.completed = True
                completed_now = True
        else:
            self.inside_since = None
        linear, angular, stage = anchor_command(boat, yaw, yaw_rate, self.center,
                                                self.hold_yaw, p)
        return decision(kind='anchor', linear=linear, angular=angular, stage=stage,
                        inside=inside, hold_elapsed=elapsed, completed_now=completed_now)

    def _start_hold(self, yaw):
        self.phase = 'HOLD'
        self.hold_yaw = float(yaw)
        self.inside_since = None
