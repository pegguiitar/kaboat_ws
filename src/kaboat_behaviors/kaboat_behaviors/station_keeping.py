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
2) find_buoys — 부표 크기 덩어리 중 강한 순 최대 4개. 후보는 기록 지점(지도 중심) 4 m 안,
   그리고 사각형을 찾은 뒤엔 지금 추정 중심 square_search_radius 안 (2026-11-13: 사각형이 기록
   위치에서 2 m 넘게 밀리면 먼 모서리 부표가 4 m 밖으로 빠져 출구를 엉뚱한 변으로 잡았다).
3) estimate_center — 부표 수별 사각형 중심.
     2개: 한 변이면 중점에서 안쪽으로 간격/2, 대각선이면 중점
     3개: 가장 먼 쌍 = 대각선 → 그 중점
     4개: 평균
   square_corners — 보이는 부표로 사각형 꼭짓점 4개 (3개면 평행사변형으로 4번째를 채움).
   부표는 매 틱 다시 찾으므로 흔들리거나 밀린 사각형을 그대로 따라간다.
4) 입구 — 처음 보이는 한 변(가장 가까운 쌍)을 입구로 고정하고, 입구 앞 정렬점을 거쳐
   중심으로 들어간다. 부표 사이를 지나므로 공용 waypoint()(전진 12 N)가 아니라 가상 앵커
   (전진 상한 max_forward 0.25 = 게이트 LOS 직진 상한)로 가서 해류에 밀리지 않게 한다.
5) anchor_command — 가상 앵커. 선수각 PD + 불감대는 수조 station_keeping_test 그대로.
   전진은 속도 제어 (2026-11-13): 목표까지 거리에 비례한 목표 속도(min(max_speed,
   k_approach·거리))를 /odom 전진 속도로 따라가고, 너무 빠르면 후진 추력으로 제동한다.
   목표를 지나쳐 뒤에 두고 제자리 회전할 때·불감대 안에서도 남은 속도를 제동한다.
   예전엔 거리 1 m 안까지 전진 0.25 를 내고 제동이 없어 관성(τ≈2.8 s)으로 사각형 변까지
   미끄러졌다(시뮬: 변까지 0.1 m, 4회 중 2회 부표 접촉).
6) StationFsm — SEARCH → ENTER → HOLD. 배 중심이 부표 사각형 안(각 변에서 inside_margin
   안쪽)에 hold_time 연속 있으면 완료 1회 보고(다음 미션 전환은 mission_manager 몫). 완료
   뒤에도 전환 전까지 계속 유지. (2026-11-13 경기 조건 변경: 예전엔 추정 중심 hold_radius
   반경 안이었다.) 부표를 끝내 못 찾아 station_end 를 중심으로 쓸 때만 반경 hold_radius 로 판정.
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
    buoy_search_radius: float = 4.0  # [m] 기록 지점(station_center, 없으면 station_end) 기준 후보 반경
    # [m] 사각형을 찾은 뒤 지금 추정 중심 기준 후보 반경 (반대각선 2.1 m + 흔들림·추정 오차).
    # 위 반경과 합집합 — 부표가 밀려도 추정 중심이 따라가며 네 개를 계속 잡는다.
    square_search_radius: float = 3.0
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
    # 완료 조건 (2026-11-13): 배 중심이 부표 사각형 각 변에서 inside_margin 이상 안쪽.
    # 0.6 m ≈ 선체 반길이 0.55 m — 선체가 어느 방향이어도 거의 다 사각형 안. 배 중심만
    # 안이면 되는 규정이면 0.
    inside_margin: float = 0.6
    hold_radius: float = 1.0        # [m] ENTER → HOLD 전환 반경, 부표를 못 찾았을 때 유지 판정
    hold_time: float = 5.0          # [s] 연속 유지 시간 → 완료 보고
    fallback_to_goal: bool = True   # 부표를 못 찾으면 station_end 를 중심으로 유지
    # ── 출구 (2026-11-12): 중심(station_center)과 station_end 가 다르면 유지 후 출구로 나간다 ──
    exit_outside: float = 1.5       # [m] 출구 쪽 변 중점에서 사각형 밖으로 이만큼 나간 경유점
    exit_reach: float = 0.8         # [m] 경유점·출구 도착 판정
    # ── 가상 앵커 (선수각·불감대는 수조 station_keeping_test 값, 상한은 LOS 직진·회전 상한) ──
    pos_deadband: float = 0.5
    yaw_deadband_deg: float = 8.0
    max_forward: float = 0.25
    max_reverse: float = 0.08       # 뒤쪽 가까운 목표로 천천히 후진 (BACK)
    max_turn: float = 0.31
    min_forward: float = 0.04
    kp_pos: float = 0.25            # BACK 후진 = kp_pos × 거리 (max_reverse 까지)
    kp_yaw: float = 1.5
    kd_yaw: float = 0.15
    reverse_band: float = 0.3       # [m] 불감대 + 이 거리 안의 뒤쪽 목표는 후진으로
    # ── 속도 제어·제동 (2026-11-13) ──
    max_speed: float = 0.5          # [m/s] 목표 속도 상한 (예전 전진 0.25 ≈ 0.7~0.85 m/s)
    k_approach: float = 0.4         # [1/s] 목표 속도 = k_approach × 거리 → 1 m 에서 0.4 m/s
    speed_ff: float = 0.25          # 목표 속도 1 m/s 당 전진 명령 (실측 0.4 ≈ 1.0 m/s, 저속은 덜 듦)
    k_speed: float = 0.4            # 속도 오차 1 m/s 당 명령 — 빠르면 음수 = 후진 제동
    max_brake: float = 0.15         # 제동 후진 명령 상한 (약 1419 µs)
    stop_speed: float = 0.08        # [m/s] 이보다 느리면 멈춘 것으로 보고 제동 안 함

    def __post_init__(self):
        floats = [v for v in vars(self).values() if not isinstance(v, bool)]
        if not all(math.isfinite(float(v)) for v in floats):
            raise ValueError('station parameters must be finite')
        positive = (self.resolution, self.map_half_size, self.max_range, self.l_occ,
                    self.occ_logodds, self.max_cluster_extent, self.buoy_search_radius,
                    self.side_length, self.min_side, self.pre_gate_distance,
                    self.gate_lateral_tol, self.track_radius, self.search_slow_radius,
                    self.hold_radius, self.pos_deadband, self.yaw_deadband_deg,
                    self.square_search_radius,
                    self.max_forward, self.max_turn, self.kp_pos, self.kp_yaw,
                    self.exit_outside, self.exit_reach, self.max_speed, self.k_approach,
                    self.k_speed)
        if (min(positive) <= 0 or self.l_free > 0 or not 0.0 <= self.free_ratio <= 1.0
                or self.l_min >= 0 or self.l_max <= self.occ_logodds
                or self.min_cluster_cells < 1 or self.big_min_cells < 1 or self.hold_time < 0
                or self.min_side >= self.side_length
                or not 0.0 <= self.inside_margin < self.side_length / 2.0
                or not 0.0 <= self.max_reverse <= 1.0 or self.max_forward > 1.0
                or not 0.0 <= self.max_brake <= 1.0 or self.speed_ff < 0
                or self.stop_speed < 0
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


def find_buoys(station_map: StationMap, near=None) -> list[Buoy]:
    """부표 크기 덩어리, 강한 순 최대 4개 (가까운 덩어리는 하나로).

    후보: 지도 중심(기록 지점) buoy_search_radius 안, 또는 near(지금 추정한 사각형 중심)가
    있으면 그 square_search_radius 안.
    """
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
        if (math.hypot(x - station_map.center[0], y - station_map.center[1]) > p.buoy_search_radius
                and (near is None
                     or math.hypot(x - near[0], y - near[1]) > p.square_search_radius)):
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


def square_corners(points, center, params: StationParams):
    """보이는 부표 + 추정 중심 → 사각형 꼭짓점 4개 (둘레 순서) 또는 None.

    4개: 그대로. 3개: 가장 먼 쌍이 대각선 → 나머지 꼭짓점의 대각선 대칭점이 4번째.
    2개 한 변: 중심 쪽 맞은편 변. 2개 대각선: 중점 둘레로 90° 돌린 대각선.
    """
    pts = [np.asarray(pt, dtype=float) for pt in points][:4]
    if center is None or len(pts) < 2:
        return None
    if len(pts) == 3:
        pairs = [(0, 1), (0, 2), (1, 2)]
        i, j = max(pairs, key=lambda ij: np.linalg.norm(pts[ij[0]] - pts[ij[1]]))
        pts.append(pts[i] + pts[j] - pts[3 - i - j])
    elif len(pts) == 2:
        a, b = pts
        mid = (a + b) / 2.0
        if float(np.linalg.norm(b - a)) >= params.diagonal_threshold:
            half = (b - a) / 2.0
            perp = np.array([-half[1], half[0]])
            pts += [mid + perp, mid - perp]
        else:
            shift = 2.0 * (np.asarray(center, dtype=float) - mid)
            pts += [a + shift, b + shift]
    c = np.mean(pts, axis=0)
    pts.sort(key=lambda q: math.atan2(q[1] - c[1], q[0] - c[0]))
    return [tuple(map(float, q)) for q in pts]


def square_clearance(boat_xy, corners) -> float:
    """배 중심에서 사각형 각 변(직선)까지의 안쪽 거리 중 최소 [m]. 음수면 사각형 밖."""
    pts = np.asarray(corners, dtype=float)
    boat = np.asarray(boat_xy, dtype=float)
    centroid = pts.mean(axis=0)
    best = math.inf
    for i in range(len(pts)):
        a, d = pts[i], pts[(i + 1) % len(pts)] - pts[i]
        length = float(np.linalg.norm(d))
        if length < 1e-6:
            continue
        inward = np.array([-d[1], d[0]]) / length
        if float(inward @ (centroid - a)) < 0.0:
            inward = -inward
        best = min(best, float(inward @ (boat - a)))
    return best


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

def brake_command(speed, params: StationParams) -> float:
    """남은 전진(후진) 속도 [m/s] 를 없애는 명령. 거의 멈췄으면 0."""
    p = params
    if abs(speed) <= p.stop_speed:
        return 0.0
    return max(-p.max_brake, min(p.max_forward, -p.k_speed * speed))


def anchor_command(boat_xy, yaw, yaw_rate, target_xy, hold_yaw, params: StationParams,
                   speed: float = 0.0):
    """→ (linear, angular, 단계). linear/angular 는 최대추력 비율, speed 는 /odom 전진 속도 [m/s].

    단계: RETURN(목표 쪽 전진) / BRAKE(목표 쪽인데 너무 빨라 후진 제동) / BACK(뒤 가까운
    목표로 후진) / TURN(뒤 먼 목표 — 제자리 회전 + 제동) / ALIGN·HOLD(불감대 안 + 제동).
    """
    p = params
    dx, dy = target_xy[0] - boat_xy[0], target_xy[1] - boat_xy[1]
    dist = math.hypot(dx, dy)
    rel = _wrap(math.atan2(dy, dx) - yaw)
    brake = brake_command(speed, p)

    def turn(err):
        return _clamp(p.kp_yaw * err - p.kd_yaw * yaw_rate, p.max_turn)

    if dist <= p.pos_deadband:
        stage = 'HOLD' if brake == 0.0 else 'BRAKE'
        if hold_yaw is None:
            return brake, 0.0, stage
        yaw_err = _wrap(hold_yaw - yaw)
        if abs(yaw_err) <= math.radians(p.yaw_deadband_deg):
            return brake, 0.0, stage
        return brake, turn(yaw_err), 'ALIGN'
    if abs(rel) <= math.pi / 2.0:
        # 목표 속도는 거리에 비례 — 다가갈수록 느려져 불감대에 약 k_approach·0.5 m/s 로 들어온다
        want = min(p.max_speed, p.k_approach * dist) * math.cos(rel)
        linear = p.speed_ff * want + p.k_speed * (want - speed)
        if 0.0 < linear < p.min_forward:
            linear = p.min_forward          # ESC 가 반응하는 최소 추력
        linear = max(-p.max_brake, min(p.max_forward, linear))
        return linear, turn(rel), 'RETURN' if linear >= 0.0 else 'BRAKE'
    if dist < p.pos_deadband + p.reverse_band and p.max_reverse > 0.0:
        rev = _wrap(rel - math.pi)
        back = -min(p.max_reverse, max(p.min_forward, p.kp_pos * dist)) * math.cos(rev)
        return min(back, brake), turn(rev), 'BACK'
    # 목표를 지나쳐 뒤에 둠 — 돌아서는 동안 관성으로 더 멀어지지 않게 제동
    return brake, turn(rel), 'TURN'


# ── 5b) 출구 ─────────────────────────────────────────

def exit_waypoints(buoys, center, exit_xy, params: StationParams):
    """유지가 끝난 중심 → 출구(station_end) 경유점 [사각형 밖 경유점, 출구].

    출구 방향을 가장 잘 바라보는 변(이웃한 두 부표, 간격 min_side~1.25·side_length — 대각선
    1.41·side 는 제외)의
    가운데를 수직으로 지나 exit_outside 만큼 밖으로 나간 뒤 출구로 간다 — 중심에서
    출구로 곧장 가면 모서리 부표에 닿을 수 있다. 쓸 만한 변이 없으면 [출구].
    """
    c = np.asarray(center, dtype=float)
    out_dir = np.asarray(exit_xy, dtype=float) - c
    if float(np.linalg.norm(out_dir)) < 1e-6:
        return [tuple(exit_xy)]
    out_dir /= np.linalg.norm(out_dir)
    pts = [np.asarray((b.x, b.y) if hasattr(b, 'x') else b, dtype=float) for b in buoys]
    pts.sort(key=lambda q: math.atan2(q[1] - c[1], q[0] - c[0]))
    best, best_dot = None, 0.0
    for i in range(len(pts)):
        a, b = pts[i], pts[(i + 1) % len(pts)]
        if len(pts) < 2 or (len(pts) == 2 and i == 1):
            break
        gap = float(np.linalg.norm(b - a))
        if not params.min_side <= gap <= 1.25 * params.side_length:
            continue
        mid = (a + b) / 2.0
        normal = np.array([-(b - a)[1], (b - a)[0]]) / gap
        if float(normal @ (mid - c)) < 0.0:
            normal = -normal                       # 사각형 밖을 향하게
        dot = float(normal @ out_dir)
        if dot > best_dot:
            best, best_dot = (mid, normal), dot
    if best is None:
        return [tuple(exit_xy)]
    mid, normal = best
    return [tuple(mid + normal * params.exit_outside), tuple(exit_xy)]


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
    inside: bool = False               # HOLD: 완료 조건(사각형 안)을 지금 만족하는지
    clearance: float | None = None     # HOLD: 사각형 변까지 최소 안쪽 거리 (사각형 없으면 None)
    hold_elapsed: float = 0.0
    completed_now: bool = False        # 이번 step 에서 완료 조건 처음 달성
    buoys: list = field(default_factory=list)
    gate: EntryGate | None = None
    square: list | None = None         # 사각형 꼭짓점 4개 (둘레 순서)


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
        self.square = None
        self.through = False
        self.hold_yaw = None
        self.inside_since = None
        self.completed = False

    def add_scan(self, ranges, angle_min, angle_increment, range_min, range_max,
                 lidar_xy, yaw):
        self.map.integrate_scan(ranges, angle_min, angle_increment, range_min, range_max,
                                lidar_xy, yaw)

    def step(self, boat_xy, yaw, yaw_rate, now, speed: float = 0.0) -> StationDecision:
        """speed: /odom 전진 속도 [m/s] (선체 기준 twist.linear.x) — 가상 앵커 제동에 쓴다."""
        p = self.p
        boat = (float(boat_xy[0]), float(boat_xy[1]))
        # 부표로 추정한 중심이 있으면 그 둘레도 찾는다 (기록 지점 대신 쓴 'goal' 은 제외)
        near = self.center if self.center_kind not in (None, 'goal') else None
        buoys = find_buoys(self.map, near)
        pts = [(b.x, b.y) for b in buoys]

        if self.gate is not None:
            self.gate = track_gate(self.gate, pts, p)
        elif self.phase == 'SEARCH':
            self.gate = choose_entry_gate(pts, boat, p)
        estimate = estimate_center(pts, boat, p, hint_xy=self.center)
        if estimate is not None:
            self.center, self.center_kind = estimate
            square = square_corners(pts, self.center, p)
            if square is not None:
                self.square = square

        def decision(**kw):
            return StationDecision(phase=self.phase, center=self.center,
                                   center_kind=self.center_kind, buoys=buoys,
                                   gate=self.gate, square=self.square, **kw)

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
                linear, angular, _ = anchor_command(boat, yaw, yaw_rate, target, None, p,
                                                    speed=speed)
                return decision(kind='anchor', target=target, linear=linear,
                                angular=angular, stage=stage)

        # HOLD — 완료 조건: 배 중심이 부표 사각형 안 (각 변에서 inside_margin 안쪽).
        # 부표를 못 찾아 station_end 로 유지 중이면(사각형 없음) 반경 hold_radius.
        clearance = None
        if self.square is not None:
            clearance = square_clearance(boat, self.square)
            inside = clearance >= p.inside_margin
        else:
            inside = math.dist(boat, self.center) <= p.hold_radius
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
                                                self.hold_yaw, p, speed=speed)
        return decision(kind='anchor', linear=linear, angular=angular, stage=stage,
                        inside=inside, clearance=clearance, hold_elapsed=elapsed,
                        completed_now=completed_now)

    def _start_hold(self, yaw):
        self.phase = 'HOLD'
        self.hold_yaw = float(yaw)
        self.inside_since = None
