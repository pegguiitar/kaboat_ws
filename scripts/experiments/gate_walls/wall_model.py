"""Experimental, colour-free two-wall fit. No ROS publishers or motor commands.

Coordinates and memory remain in odom. A fixed entry frame orders progress;
the distance metric can use current heading, waypoint, or the previous corridor
tangent. The two-wall constraint and seeded identities prevent a single-link
cluster from joining both banks. Future observations are never used.
"""
from dataclasses import dataclass
import math

import numpy as np
from scipy.sparse.csgraph import connected_components


def directional_distance(delta, angle, discount=3.0):
    d = np.asarray(delta, float)
    c, s = math.cos(angle), math.sin(angle)
    along = d[..., 0] * c + d[..., 1] * s
    across = -d[..., 0] * s + d[..., 1] * c
    return np.hypot(along / discount, across)


@dataclass
class Track:
    xy: np.ndarray
    hits: int
    last: float
    side: int = -1


class WallModel:
    def __init__(self, mode='adaptive', discount=3.0):
        if mode not in ('heading', 'waypoint', 'adaptive'):
            raise ValueError(mode)
        self.mode, self.discount = mode, discount
        self.tracks = []
        self.origin = self.axis = self.left_axis = None
        self.coeff = None
        self.frame_s = 0.0
        self.seeded = False
        self.last = None
        self.seed_station = None

    def _local(self, xy):
        d = np.asarray(xy, float) - self.origin
        return np.column_stack((d @ self.axis, d @ self.left_axis))

    def _world(self, s, y):
        return self.origin + np.asarray(s)[:, None] * self.axis + np.asarray(y)[:, None] * self.left_axis

    def curves(self, start, end, n=100):
        s = np.linspace(start, end, n)
        u = s - self.frame_s
        walls = [self._world(s, c[0] + c[1] * u + c[2] * u*u) for c in self.coeff]
        return walls[0], walls[1], (walls[0] + walls[1]) / 2

    def _remember(self, observations, now, boat):
        used = set()
        for xy in observations:
            xy = np.asarray(xy, float)
            if not np.all(np.isfinite(xy)) or np.linalg.norm(xy - boat) > 15:
                continue
            ds = [(np.linalg.norm(t.xy-xy), i) for i, t in enumerate(self.tracks) if i not in used]
            dist, idx = min(ds, default=(float('inf'), -1))
            if dist <= 0.65:
                t = self.tracks[idx]
                t.xy = 0.75*t.xy + 0.25*xy
                t.hits += 1
                t.last = now
            else:
                idx = len(self.tracks)
                self.tracks.append(Track(xy.copy(), 1, now))
            used.add(idx)
        self.tracks = [t for t in self.tracks if np.linalg.norm(t.xy-boat) <= 20
                       and now-t.last <= (45 if t.hits >= 3 else 1.0)]

    def update(self, observations, boat_xy, yaw, goal_xy, now, seed=None):
        boat, goal = np.asarray(boat_xy, float), np.asarray(goal_xy, float)
        if self.origin is None:
            self.origin = boat.copy()
            # Initial entry heading is causal; orientation never rotates with steering.
            self.axis = np.array([math.cos(yaw), math.sin(yaw)])
            if (goal-boat) @ self.axis < 0:
                self.axis *= -1
            self.left_axis = np.array([-self.axis[1], self.axis[0]])
        self._remember(observations, now, boat)
        confirmed = [t for t in self.tracks if t.hits >= 3]
        if len(confirmed) < 2:
            return self._result('WAIT_POSTS', boat)
        if self.coeff is None:
            # Only bootstrap from the first LiDAR-verified gate of the unchanged
            # competition selector. No later gate pairs or reference labels enter.
            if seed is None:
                return self._result('WAIT_SEED', boat)
            self.origin = np.asarray(seed.midpoint, float)
            self.axis = np.asarray(seed.direction, float)
            self.left_axis = np.array([-self.axis[1], self.axis[0]])
            self.seed_station = 0.
            initial = self._local([seed.left, seed.right])
            self.coeff = np.array([[initial[0, 1], 0., 0.], [initial[1, 1], 0., 0.]])
            self.frame_s = 0.
            for side, pt in enumerate((seed.left, seed.right)):
                # Seed observations may have just arrived, before this extra
                # memory's third observation. Preserve their bank identity now.
                idx = int(np.argmin([np.linalg.norm(t.xy-pt) for t in self.tracks]))
                if np.linalg.norm(self.tracks[idx].xy-pt) <= .7:
                    self.tracks[idx].side = side
            self.seeded = True
        # Unrelated objects behind the first verified gate cannot seed a bank.
        confirmed = [t for t in confirmed if self._local([t.xy])[0, 0] >= self.seed_station-1.]
        if len(confirmed) < 2:
            return self._result('WAIT_POSTS', boat)
        xy = np.array([t.xy for t in confirmed])
        local = self._local(xy)
        bs = self._local([boat])[0, 0]
        # Rebase polynomials before fitting to keep the solve well-conditioned.
        shift = bs-self.frame_s
        self.coeff[:, 0] += self.coeff[:, 1]*shift + self.coeff[:, 2]*shift*shift
        self.coeff[:, 1] += 2*self.coeff[:, 2]*shift
        self.frame_s = bs
        if self.mode == 'heading':
            angle = yaw
        elif self.mode == 'waypoint':
            angle = math.atan2(*(goal-boat)[::-1])
        else:
            v = self.axis + float(self.coeff[:, 1].mean()) * self.left_axis
            angle = math.atan2(v[1], v[0])
        delta = xy[:, None, :] - xy[None, :, :]
        raw = directional_distance(delta, angle, self.discount) <= 1.5
        np.fill_diagonal(raw, False)
        _, raw_labels = connected_components(raw, directed=False)
        # Alternate directional nearest-wall assignments and robust smooth fits.
        labels = np.full(len(xy), -1, int)
        for _ in range(3):
            walls = self.curves(bs-10, bs+10, 121)[:2]
            distances = np.column_stack([
                directional_distance(xy[:, None, :]-wall[None, :, :], angle, self.discount).min(1)
                for wall in walls])
            costs = distances.copy()
            for i, t in enumerate(confirmed):
                if t.side >= 0:
                    costs[i, 1-t.side] += 0.45
                else:
                    # New points must extend an existing bank, rather than be
                    # captured solely by a distant extrapolated polynomial.
                    for side in (0, 1):
                        anchors = [q.xy for q in confirmed if q.side == side]
                        if (not anchors or np.min(directional_distance(
                                t.xy-np.asarray(anchors), angle, self.discount)) > 1.8):
                            costs[i, side] = float('inf')
            labels = costs.argmin(1)
            labels[(distances[np.arange(len(xy)), labels] > 1.25)
                   | ~np.isfinite(costs.min(1))] = -1
            # Make accepted neighbours available in the next assignment pass.
            for tr, label in zip(confirmed, labels):
                if label >= 0:
                    tr.side = int(label)
            for side in (0, 1):
                sel = (labels == side) & (np.abs(local[:, 0]-bs) <= 10)
                if not sel.any():
                    continue
                u, y = local[sel, 0]-bs, local[sel, 1]
                weights = np.minimum(np.array([t.hits for t in confirmed])[sel], 12)/12
                # A single post constrains position, never an observed wall direction.
                degree = 2 if len(u) >= 4 and np.ptp(u) >= 5 else (1 if len(u) >= 2 and np.ptp(u) >= 1.5 else 0)
                A = np.column_stack((np.ones(len(u)), u, u*u))[:, :degree+1]
                prior = self.coeff[side, :degree+1].copy()
                beta = prior.copy()
                for _ in range(4):
                    residual = y-A@beta
                    w = weights*np.minimum(1., 0.35/np.maximum(abs(residual), 1e-9))
                    reg = np.diag([0.08, 0.5, 18.][:degree+1])
                    beta = np.linalg.solve(A.T@(w[:, None]*A)+reg, A.T@(w*y)+reg@prior)
                proposal = np.zeros(3)
                proposal[:degree+1] = beta
                # Bounded updates prevent one new outlier from rotating a whole wall.
                proposal[1] = np.clip(proposal[1], -1., 1.)
                proposal[2] = np.clip(proposal[2], -0.10, 0.10)
                self.coeff[side] = proposal
        for t, label in zip(confirmed, labels):
            if label >= 0:
                t.side = int(label)
        left, right, center = self.curves(bs-3, bs+6)
        # A prediction is publishable only inside common, recently supported extent.
        supports = []
        for side in (0, 1):
            sel = (labels == side) & (now-np.array([t.last for t in confirmed]) <= 10)
            supports.append(local[sel, 0])
        enough = all(len(v) >= 2 and np.ptp(v) >= 1.5 for v in supports)
        end = min(max(v) for v in supports) if all(len(v) for v in supports) else bs
        width = np.linalg.norm(left-right, axis=1)
        inside = (np.min(width) >= 1.5 and np.max(width) <= 7.)
        status = 'TRACK' if enough and inside and end >= bs+0.5 else 'WARMUP_OR_UNSUPPORTED'
        self.last = dict(status=status, angle=angle, xy=xy.tolist(), labels=labels.tolist(),
                         raw_labels=raw_labels.tolist(), left=left.tolist(), right=right.tolist(),
                         center=center.tolist(), supported_end=float(end), boat_s=float(bs),
                         counts=[int(np.sum(labels == s)) for s in (0, 1)],
                         width_min=float(width.min()), width_max=float(width.max()))
        return self.last

    def _result(self, status, boat):
        self.last = dict(status=status, xy=[], labels=[], raw_labels=[], left=[], right=[], center=[], counts=[0, 0])
        return self.last
