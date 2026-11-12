"""LiDAR occupancy-grid gate pairing and DRI-checked centreline geometry.

The gate mission has ordered apertures, unlike free-form obstacle avoidance:
each detected pair supplies a mandatory midpoint. This module has no ROS node
state so synthetic grids can exercise the pairing and clearance rules.
"""

from dataclasses import dataclass
import math

import numpy as np
from scipy import ndimage

from .los_controller import bspline_path


@dataclass(frozen=True)
class GateParams:
    required_pairs: int = 7          # 대회 게이트 7쌍 (2026-11-12, 예전 수조 3쌍)
    occ_threshold: int = 95
    min_cluster_cells: int = 2
    max_cluster_extent: float = 1.2
    min_gate_width: float = 1.5
    max_gate_width: float = 7.0
    # 게이트선(두 기둥을 잇는 선)이 기준 진행 방향의 수직에서 이만큼까지 기울어도 쌍으로
    # 인정 [deg]. 기준 = 직전에 통과한 게이트의 진행 방향(통로를 따라 꺾여 감), 첫 게이트는
    # 배→목표 방향. 예전엔 배→목표 축 기준 기둥 앞뒤 차이(≤0.75 m)로 봐서 통로가 목표
    # 축에서 15° 넘게 휘면(폭 2.5 m: 2.5·sin θ > 0.75) 진짜 쌍을 놓쳤다. 35° (2026-11-12,
    # 현장 통로가 완전한 직선이 아님). 같은 줄의 두 기둥은 통로와 나란해 ~90° 라 걸리고,
    # 대각선 오쌍은 아래 gate_circle_margin 이 거른다.
    max_pair_skew_deg: float = 35.0
    # 대각선 오쌍 거르기: 두 기둥을 지름으로 하는 원(반지름 = 폭/2)의 이 배수 안에 다른
    # 기둥이 두 기둥을 잇는 선의 **양쪽에 모두** 있으면 게이트가 아니다. 직사각형 배치의
    # 대각선은 나머지 두 기둥이 그 원 위 양쪽에 하나씩 놓이고(원주각 90°), 진짜 게이트는
    # 가장 가까운 다른 기둥이 √(1+(2·간격/폭)²) 배 밖에 있다 (간격 2.5·폭 2.5 → 2.24,
    # 수조 간격 2.0·폭 3.0 → 1.67). 한쪽에만 있는 것(게이트 앞 장애물)은 경로 검사가 맡는다.
    gate_circle_margin: float = 1.2
    min_gate_forward: float = 0.3
    # 쌍 후보로 볼 부표의 앞 거리 상한 [m] (배→목표 축). 9.0 → 6.0 (2026-11-12): 8 m 밖 지도
    # 가장자리 부표로 엉뚱한 쌍을 골랐다. 게이트 간격 2.5 m 라 다음 게이트는 3~4 m 안,
    # gate_entry 는 첫 게이트 2~3 m 앞에 찍는다.
    max_gate_forward: float = 6.0
    max_midpoint_lateral: float = 2.5
    passed_pair_exclusion: float = 1.0
    min_pair_advance: float = 0.8
    # 연속 게이트 중점의 좌우 이동 허용치 [m] (배→목표 축 기준). 코스의 실제 최대 좌우
    # 어긋남보다 커야 한다 — 작으면 그 게이트를 건너뛰고 다음 게이트를 고른다.
    # 1.0 → 2.0 (2026-11-12): 게이트 간격 2.5 m 통로가 목표 축에서 35° 휘면 2.5·sin35°
    # = 1.43 m 어긋난다.
    max_midpoint_shift: float = 2.0
    exit_distance: float = 1.0
    sample_spacing: float = 0.05
    min_clearance: float = 0.5
    safe_threshold: float = 1.0
    max_goal_after_gates: float = 4.0
    scan_match_radius: float = 0.7

    def __post_init__(self):
        if (self.required_pairs < 1 or self.min_cluster_cells < 1
                or not 0 <= self.occ_threshold <= 100):
            raise ValueError('invalid gate count or occupancy thresholds')
        values = (self.max_cluster_extent, self.min_gate_width,
                  self.max_gate_width, self.max_pair_skew_deg, self.gate_circle_margin,
                  self.min_gate_forward, self.max_gate_forward,
                  self.max_midpoint_lateral, self.passed_pair_exclusion,
                  self.min_pair_advance, self.max_midpoint_shift,
                  self.exit_distance, self.sample_spacing,
                  self.min_clearance, self.safe_threshold,
                  self.max_goal_after_gates, self.scan_match_radius)
        if (not all(math.isfinite(v) for v in values)
                or min(values) <= 0
                or self.min_gate_width >= self.max_gate_width
                or self.min_gate_forward >= self.max_gate_forward
                or self.min_gate_width <= 2.0 * self.min_clearance
                or self.max_pair_skew_deg >= 90.0):
            raise ValueError('invalid gate geometry or clearance parameters')


@dataclass(frozen=True)
class GatePair:
    left: tuple[float, float]
    right: tuple[float, float]
    midpoint: tuple[float, float]
    width: float
    direction: tuple[float, float]


def _unit_toward(boat_xy, goal_xy):
    direction = np.asarray(goal_xy, dtype=float) - np.asarray(boat_xy, dtype=float)
    length = float(np.linalg.norm(direction))
    if not math.isfinite(length) or length < 1e-6:
        return None
    return direction / length


def _clusters(grid, p: GateParams):
    res = float(grid.info.resolution)
    width, height = int(grid.info.width), int(grid.info.height)
    if (not math.isfinite(res) or res <= 0 or width <= 0 or height <= 0
            or len(grid.data) != width * height):
        return []
    occupied = np.asarray(grid.data, dtype=np.int16).reshape(height, width)
    labels, count = ndimage.label(
        occupied >= p.occ_threshold,
        structure=np.ones((3, 3), dtype=np.uint8))
    ox = float(grid.info.origin.position.x)
    oy = float(grid.info.origin.position.y)
    clusters = []
    for label_id in range(1, count + 1):
        iy, ix = np.nonzero(labels == label_id)
        if len(ix) < p.min_cluster_cells:
            continue
        extent = res * max(ix.max() - ix.min() + 1,
                           iy.max() - iy.min() + 1)
        if extent > p.max_cluster_extent:
            continue  # walls and merged posts are not gate markers
        clusters.append((ox + (float(ix.mean()) + 0.5) * res,
                         oy + (float(iy.mean()) + 0.5) * res))
    return clusters


def scan_hits_world(scan, boat_xy, yaw, lidar_offset=(0.0, 0.0)):
    """Use the same scan geometry as occupancy_grid.py.

    lidar_offset 은 base_link(선체 중심) 기준 선체 라이다 위치 (선수, 좌현) [m].
    """
    c, s = math.cos(yaw), math.sin(yaw)
    ox, oy = float(lidar_offset[0]), float(lidar_offset[1])
    origin = (boat_xy[0] + ox * c - oy * s, boat_xy[1] + ox * s + oy * c)
    ranges = np.asarray(scan.ranges, dtype=float)
    angles = scan.angle_min + scan.angle_increment * np.arange(len(ranges))
    valid = (np.isfinite(ranges) & (ranges > scan.range_min)
             & (ranges < scan.range_max))
    if not np.any(valid):
        return np.empty((0, 2), dtype=float)
    world_angles = yaw + angles[valid]
    return np.column_stack((
        origin[0] + ranges[valid] * np.cos(world_angles),
        origin[1] + ranges[valid] * np.sin(world_angles)))


def pair_observed(pair: GatePair, scan_hits, p: GateParams):
    """Require current LiDAR hits near both posts; rolling grid has no decay."""
    hits = np.asarray(scan_hits, dtype=float)
    if hits.ndim != 2 or hits.shape[1] != 2 or len(hits) == 0:
        return False
    radius2 = p.scan_match_radius ** 2
    return all(np.any(np.sum((hits - post) ** 2, axis=1) <= radius2)
               for post in (pair.left, pair.right))


# select_gate_pair(stats=…) 가 채우는 탈락 이유 (로그 순서·한글 이름)
REJECT_REASONS = (
    ('width', '폭'), ('diagonal', '대각선'), ('skew', '기울기'), ('lateral', '좌우 멀다'),
    ('held', '통과·보류'), ('advance', '전진 부족'), ('shift', '좌우 어긋남'))


def select_gate_pair(grid, boat_xy, goal_xy, passed_midpoints, p: GateParams,
                     excluded_midpoints=(), last_direction=None, stats=None):
    """Find the nearest credible aperture ahead along the mission goal axis.

    excluded_midpoints: 잠시 후보에서 뺄 쌍의 중점 (경로가 계속 막혀 버린 쌍).
    last_direction: 직전에 통과한 게이트의 진행 방향 — 게이트선 기울기의 기준.
    없으면 배→목표 방향.
    stats: dict 를 주면 진단용 개수를 채운다 — posts(지도 부표 덩어리), ahead(앞 창 안),
    pairs(검사한 두 부표 조합), REJECT_REASONS 의 이유별 탈락 수, chosen_skew_deg.
    """
    if stats is not None:
        stats.clear()
        stats.update({'posts': 0, 'ahead': 0, 'pairs': 0, 'chosen_skew_deg': None,
                      **{key: 0 for key, _ in REJECT_REASONS}})

    def reject(reason):
        if stats is not None:
            stats[reason] += 1

    direction = _unit_toward(boat_xy, goal_xy)
    if direction is None:
        return None
    lateral = np.array([-direction[1], direction[0]])
    boat = np.asarray(boat_xy, dtype=float)
    axis = direction
    if last_direction is not None:
        last = np.asarray(last_direction, dtype=float)
        if np.all(np.isfinite(last)) and np.linalg.norm(last) > 1e-6:
            axis = last / np.linalg.norm(last)
    max_skew_sin = math.sin(math.radians(p.max_pair_skew_deg))
    candidates = []
    posts = _clusters(grid, p)
    all_posts = np.asarray(posts, dtype=float).reshape(-1, 2)
    for point in posts:
        delta = np.asarray(point) - boat
        along = float(delta @ direction)
        side = float(delta @ lateral)
        if p.min_gate_forward <= along <= p.max_gate_forward:
            candidates.append((point, along, side))
    if stats is not None:
        stats['posts'], stats['ahead'] = len(posts), len(candidates)

    pairs = []
    for i, (a, ta, la) in enumerate(candidates):
        for b, tb, lb in candidates[i + 1:]:
            if stats is not None:
                stats['pairs'] += 1
            a_xy, b_xy = np.asarray(a, dtype=float), np.asarray(b, dtype=float)
            post_line = b_xy - a_xy
            width = float(np.linalg.norm(post_line))
            mid = (a_xy + b_xy) / 2.0
            if not p.min_gate_width <= width <= p.max_gate_width:
                reject('width')
                continue
            # 지름 원 안 다른 기둥이 선 양쪽에 다 있으면 대각선 오쌍 (위 GateParams 주석)
            rel = all_posts - mid
            near = ((np.linalg.norm(rel, axis=1) < p.gate_circle_margin * width / 2.0)
                    & (np.linalg.norm(all_posts - a_xy, axis=1) > 1e-6)
                    & (np.linalg.norm(all_posts - b_xy, axis=1) > 1e-6))
            if near.any():
                side = post_line[0] * rel[near, 1] - post_line[1] * rel[near, 0]
                if np.any(side > 0.0) and np.any(side < 0.0):
                    reject('diagonal')
                    continue
            # 게이트선이 기준 진행 방향에 수직이면 0, 나란하면 1 (= sin 기울기)
            skew_sin = abs(float(post_line @ axis)) / width
            mid_side = float((mid - boat) @ lateral)
            if skew_sin > max_skew_sin:
                reject('skew')
                continue
            if abs(mid_side) > p.max_midpoint_lateral:
                reject('lateral')
                continue
            if any(np.linalg.norm(mid - old) < p.passed_pair_exclusion
                   for old in (*passed_midpoints, *excluded_midpoints)):
                reject('held')
                continue
            if passed_midpoints:
                from_last = mid - np.asarray(passed_midpoints[-1])
                if float(from_last @ direction) < p.min_pair_advance:
                    reject('advance')
                    continue
                if abs(float(from_last @ lateral)) > p.max_midpoint_shift:
                    reject('shift')
                    continue
            # 게이트를 지나는 방향 = 게이트선의 수직, 기준 진행 방향 쪽으로.
            # 통과 판정(crossed_gate)·경로 끝점(make_gate_path)이 이 방향을 쓴다.
            normal = np.array([-post_line[1], post_line[0]]) / width
            if float(normal @ axis) < 0.0:
                normal = -normal
            left_axis = np.array([-normal[1], normal[0]])
            left, right = (a, b) if float((a_xy - mid) @ left_axis) > 0 else (b, a)
            pair = GatePair(left, right, (float(mid[0]), float(mid[1])),
                            width, (float(normal[0]), float(normal[1])))
            pairs.append((0.5 * (ta + tb), skew_sin, abs(mid_side), pair))
    if not pairs:
        return None
    best = min(pairs, key=lambda row: row[:3])
    if stats is not None:
        stats['chosen_skew_deg'] = math.degrees(math.asin(min(1.0, best[1])))
    return best[-1]


def describe_reasons(stats) -> str:
    """select_gate_pair 의 stats → '부표 9 (앞 6), 조합 15 — 폭 3, 기울기 4 …' 한 줄."""
    if not stats:
        return '진단 없음'
    rejected = ', '.join(f'{name} {stats[key]}' for key, name in REJECT_REASONS if stats[key])
    return (f"부표 {stats['posts']} (앞 창 안 {stats['ahead']}), 조합 {stats['pairs']}"
            + (f' — 탈락: {rejected}' if rejected else ''))


def make_gate_path(boat_xy, pair: GatePair, p: GateParams):
    """Quadratic B-spline whose centre sample passes through the gate midpoint.

    A generic obstacle-avoidance B-spline may go around a pair. Solving the
    middle Bezier control point makes B(0.5) equal the mandatory midpoint.
    DRI clearance is checked separately before the path can be followed.
    """
    start = np.asarray(boat_xy, dtype=float)
    mid = np.asarray(pair.midpoint, dtype=float)
    exit_point = mid + p.exit_distance * np.asarray(pair.direction)
    control = 2.0 * mid - 0.5 * (start + exit_point)
    return bspline_path([start, control, exit_point],
                        spacing=p.sample_spacing, degree=2)


def points_safe(dri, points, p: GateParams):
    """Check both the existing DRI field and raw occupied-cell clearance."""
    points = np.asarray(points, dtype=float)
    if (points.ndim != 2 or points.shape[1] != 2 or len(points) == 0
            or not np.all(np.isfinite(points))):
        return False
    risks = dri.risk_at_many(points[:, 0], points[:, 1])
    if not np.all(np.isfinite(risks) & (risks <= p.safe_threshold)):
        return False
    if dri.occ_xy is not None and len(dri.occ_xy):
        occupied = np.asarray(dri.occ_xy, dtype=float)
        for chunk in np.array_split(points, max(1, math.ceil(len(points) / 100))):
            distances = np.linalg.norm(chunk[:, None, :] - occupied[None, :, :],
                                       axis=2)
            if np.any(distances < p.min_clearance):
                return False
    return True


def points_observed(grid, points):
    """Gate routes may not treat unknown occupancy cells as open water."""
    points = np.asarray(points, dtype=float)
    res = float(grid.info.resolution)
    width, height = int(grid.info.width), int(grid.info.height)
    if (points.ndim != 2 or points.shape[1] != 2
            or not np.all(np.isfinite(points)) or res <= 0
            or len(grid.data) != width * height):
        return False
    ox = float(grid.info.origin.position.x)
    oy = float(grid.info.origin.position.y)
    ix = np.floor((points[:, 0] - ox) / res).astype(np.int64)
    iy = np.floor((points[:, 1] - oy) / res).astype(np.int64)
    if not np.all((0 <= ix) & (ix < width) & (0 <= iy) & (iy < height)):
        return False
    data = np.asarray(grid.data, dtype=np.int16).reshape(height, width)
    return bool(np.all(data[iy, ix] >= 0))


def path_safe(dri, path, p: GateParams, start_idx=0):
    # The rolling DRI window may discard samples already behind the boat.
    return points_safe(dri, path.samples[max(0, start_idx - 2):], p)


def lookahead_chord_safe(dri, boat_xy, path, progress_idx, lookahead,
                         p: GateParams, grid=None):
    boat = np.asarray(boat_xy, dtype=float)
    goal = path.samples[-1]
    if np.linalg.norm(goal - boat) <= lookahead:
        target = goal
    else:
        idx = min(progress_idx + max(1, int(round(lookahead / path.spacing))),
                  len(path.samples) - 1)
        target = path.samples[idx]
        if np.linalg.norm(target - boat) < 0.3:
            target = goal
    length = float(np.linalg.norm(target - boat))
    samples = np.linspace(boat, target,
                          max(2, int(math.ceil(length / p.sample_spacing)) + 1))
    return (points_safe(dri, samples, p)
            and (grid is None or points_observed(grid, samples)))


def gate_crossing(previous_xy, boat_xy, pair: GatePair):
    """이번 이동이 게이트선을 진행 방향으로 가로질렀으면 (가로지른 점, 중점에서 옆 거리), 아니면 None."""
    previous = np.asarray(previous_xy, dtype=float)
    current = np.asarray(boat_xy, dtype=float)
    mid = np.asarray(pair.midpoint)
    direction = np.asarray(pair.direction)
    before = float((previous - mid) @ direction)
    after = float((current - mid) @ direction)
    if not before < 0 <= after:
        return None
    crossing = previous + (current - previous) * (-before / (after - before))
    gate_axis = np.asarray(pair.left) - np.asarray(pair.right)
    gate_axis /= np.linalg.norm(gate_axis)
    return crossing, abs(float((crossing - mid) @ gate_axis))


def crossed_gate(previous_xy, boat_xy, pair: GatePair, p: GateParams):
    """True only when the boat crosses the post line within the clear gap."""
    hit = gate_crossing(previous_xy, boat_xy, pair)
    return hit is not None and hit[1] <= pair.width / 2.0 - p.min_clearance
