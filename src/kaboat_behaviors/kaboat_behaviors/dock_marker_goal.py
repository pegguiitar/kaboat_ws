"""Water-side goal from a selected marker and a shared 2D marker-wall line."""
from dataclasses import dataclass
import math

import numpy as np


@dataclass(frozen=True)
class MarkerGoal:
    marker_xy: tuple
    goal_xy: tuple
    outward: tuple
    wall_rms: float
    wall_span: float


def marker_goal(points, target, water_reference, *, offset=1.0,
                min_span=0.75, max_rms=0.25, tank_bounds=None,
                boundary_margin=0.5):
    """Move exactly offset metres towards water; never silently clamp the goal.

    At least two fresh class-labelled markers define a common line. With three
    points the RMS check rejects inconsistent depths. The pool centre is a
    stable sign reference; the boat can be used where no tank bounds exist.
    """
    if not all(math.isfinite(x) for x in (offset, min_span, max_rms, boundary_margin)):
        raise ValueError('goal geometry limits must be finite')
    if offset <= 0 or min_span <= 0 or max_rms < 0 or boundary_margin < 0:
        raise ValueError('invalid marker goal geometry limits')
    if target not in points or len(points) < 2:
        raise ValueError('selected marker and at least two fresh markers are required')
    xy = np.asarray(list(points.values()), dtype=float)
    reference = np.asarray(water_reference, dtype=float)
    if xy.shape != (len(points), 2) or reference.shape != (2,) or not np.isfinite(xy).all() or not np.isfinite(reference).all():
        raise ValueError('marker and reference coordinates must be finite XY pairs')
    center = xy.mean(axis=0)
    _, _, vh = np.linalg.svd(xy - center, full_matrices=False)
    tangent, normal = vh[0], vh[1]
    span = float(np.ptp((xy - center) @ tangent))
    rms = float(np.sqrt(np.mean(((xy - center) @ normal) ** 2)))
    if span < min_span or rms > max_rms:
        raise ValueError(f'marker wall fit rejected (span={span:.2f}m, RMS={rms:.2f}m)')
    sign = float(normal @ (reference - center))
    if abs(sign) < 1e-6:
        raise ValueError('water-side wall normal is ambiguous')
    if sign < 0:
        normal = -normal
    marker = np.asarray(points[target], dtype=float)
    goal = marker + offset * normal
    if tank_bounds is not None:
        require_interior(goal, tank_bounds, boundary_margin)
    return MarkerGoal(tuple(marker), tuple(goal), tuple(normal), rms, span)


def require_interior(goal, tank_bounds, boundary_margin):
    """Reject unreachable candidates without moving the operator's 1m offset."""
    if len(tank_bounds) != 4 or not all(math.isfinite(v) for v in tank_bounds):
        raise ValueError('invalid tank bounds')
    x0, y0, x1, y1 = tank_bounds
    if x1 <= x0 or y1 <= y0 or boundary_margin * 2 >= min(x1-x0, y1-y0):
        raise ValueError('invalid tank bounds/margin')
    if not (x0 + boundary_margin <= goal[0] <= x1 - boundary_margin
        and y0 + boundary_margin <= goal[1] <= y1 - boundary_margin):
        raise ValueError(f'offset goal ({goal[0]:.2f}, {goal[1]:.2f}) lies outside the permitted tank interior')


class TimedGoalAverage:
    """Time-window mean of distinct measured goals, with a scatter validity gate."""
    def __init__(self, window_s=1.0, min_samples=3, min_span_s=0.3, max_std_m=0.25):
        from collections import deque
        if (not all(math.isfinite(v) for v in (window_s, min_span_s, max_std_m))
                or window_s < 0 or min_samples < 1 or min_span_s < 0 or max_std_m <= 0
                or (window_s > 0 and min_span_s > window_s)):
            raise ValueError('invalid goal averaging limits')
        self.window = window_s
        self.min_samples = min_samples
        self.min_span = min_span_s
        self.max_std = max_std_m
        self.samples = deque()

    def reset(self):
        self.samples.clear()

    def update(self, xy, stamp):
        point = np.asarray(xy, dtype=float)
        if point.shape != (2,) or not np.isfinite(point).all() or not math.isfinite(stamp):
            raise ValueError('invalid goal sample')
        if self.samples and stamp < self.samples[-1][0]:
            self.reset()
        if not self.samples or stamp > self.samples[-1][0]:
            self.samples.append((stamp, point))
        while len(self.samples) > 1 and self.samples[0][0] < stamp - self.window:
            self.samples.popleft()
        values = np.array([p for _, p in self.samples])
        average = values.mean(axis=0)
        std = float(np.sqrt(np.mean(np.sum((values - average) ** 2, axis=1))))
        span = self.samples[-1][0] - self.samples[0][0]
        reason = 'ok'
        if self.window > 0 and (len(values) < self.min_samples or span + 1e-9 < self.min_span):
            reason = 'collecting goal averaging window'
        elif std > self.max_std:
            reason = 'goal averaging window is unstable'
        return average, dict(reason=reason, ready=reason == 'ok', samples=len(values),
                             span_s=span, std_m=std)
