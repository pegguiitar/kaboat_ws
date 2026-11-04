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
    required_pairs: int = 3
    occ_threshold: int = 95
    min_cluster_cells: int = 2
    max_cluster_extent: float = 1.2
    min_gate_width: float = 1.5
    max_gate_width: float = 7.0
    max_pair_along_error: float = 0.75
    min_gate_forward: float = 0.3
    max_gate_forward: float = 9.0
    max_midpoint_lateral: float = 2.5
    passed_pair_exclusion: float = 1.0
    min_pair_advance: float = 0.8
    max_midpoint_shift: float = 1.5
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
                  self.max_gate_width, self.max_pair_along_error,
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
                or self.min_gate_width <= 2.0 * self.min_clearance):
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

    lidar_offset 은 base_link(판 중점) 기준 선체 라이다 위치 (선수, 좌현) [m].
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


def select_gate_pair(grid, boat_xy, goal_xy, passed_midpoints, p: GateParams):
    """Find the nearest credible aperture ahead along the mission goal axis."""
    direction = _unit_toward(boat_xy, goal_xy)
    if direction is None:
        return None
    lateral = np.array([-direction[1], direction[0]])
    boat = np.asarray(boat_xy, dtype=float)
    candidates = []
    for point in _clusters(grid, p):
        delta = np.asarray(point) - boat
        along = float(delta @ direction)
        side = float(delta @ lateral)
        if p.min_gate_forward <= along <= p.max_gate_forward:
            candidates.append((point, along, side))

    pairs = []
    for i, (a, ta, la) in enumerate(candidates):
        for b, tb, lb in candidates[i + 1:]:
            width = abs(la - lb)
            mid = (np.asarray(a) + np.asarray(b)) / 2.0
            mid_side = float((mid - boat) @ lateral)
            if (not p.min_gate_width <= width <= p.max_gate_width
                    or abs(ta - tb) > p.max_pair_along_error
                    or abs(mid_side) > p.max_midpoint_lateral
                    or any(np.linalg.norm(mid - old) < p.passed_pair_exclusion
                           for old in passed_midpoints)):
                continue
            if passed_midpoints:
                from_last = mid - np.asarray(passed_midpoints[-1])
                if (float(from_last @ direction) < p.min_pair_advance
                        or abs(float(from_last @ lateral)) > p.max_midpoint_shift):
                    continue
            left, right = (a, b) if la > lb else (b, a)
            pair = GatePair(left, right, (float(mid[0]), float(mid[1])),
                            width, (float(direction[0]), float(direction[1])))
            pairs.append((0.5 * (ta + tb), abs(ta - tb), abs(mid_side), pair))
    return min(pairs, key=lambda row: row[:3])[-1] if pairs else None


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


def crossed_gate(previous_xy, boat_xy, pair: GatePair, p: GateParams):
    """True only when the boat crosses the post line within the clear gap."""
    previous = np.asarray(previous_xy, dtype=float)
    current = np.asarray(boat_xy, dtype=float)
    mid = np.asarray(pair.midpoint)
    direction = np.asarray(pair.direction)
    before = float((previous - mid) @ direction)
    after = float((current - mid) @ direction)
    if not before < 0 <= after:
        return False
    crossing = previous + (current - previous) * (-before / (after - before))
    gate_axis = np.asarray(pair.left) - np.asarray(pair.right)
    gate_axis /= np.linalg.norm(gate_axis)
    offset = abs(float((crossing - mid) @ gate_axis))
    return offset <= pair.width / 2.0 - p.min_clearance
