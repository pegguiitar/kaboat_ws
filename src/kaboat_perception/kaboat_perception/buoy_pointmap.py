"""buoy_pointmap — 원본 라이다 점마다 log-odds 를 붙여 남겨 두는 부표 지도 (ROS 무의존 순수 모듈).

스캔마다 점을 버리고 그 스캔만 군집화하던 방식(buoy_evidence)과 달리, 부표 덩어리 점을 전역(odom)
좌표 그대로 계속 남긴다. 점을 옮기거나 평균내지 않는다 — 남기거나 지우기만 한다.

점 하나의 존재 근거 (스캔마다):
  올림  새 스캔 점이 이 점 merge_radius 안에 옴 (= 다시 봄)      L += l_hit
        merge_radius 안에 기존 점이 없으면 새 점으로 추가 (그 자리 첫 측정 원본)
  깎음  빔이 이 점 자리를 지나 더 멀리서 맞음                     L += l_free_rate · dt
  유지  가림(앞에서 맞음)·반사 없음(검정 흡수)·거리 밖·스캔 없음
  지움  L ≤ l_delete, 굳지 않은 점(L < l_solid)이 tentative_timeout 동안 다시 안 보임(물보라),
        배에서 forget_range 넘게 멀어짐

덩어리: 굳은 점(L ≥ l_solid)만 cluster_eps 거리로 묶는다 (KD-tree, 단일 연결). 점 min_points 개 이상,
  폭 min_width ~ max_width 만 부표. 덩어리가 free·새 점에 따라 스캔마다 바뀐다.
부표 = 덩어리. 위치 = 덩어리 점 평균, 반폭 = 가장 먼 두 점 거리 / 2, 존재 L = 덩어리 점 L 최댓값.
  id 는 점에 붙은 부표 id 의 다수결로 이어받는다 (둘이 한 덩어리가 되면 많은 쪽이 남고 색을 합친다).
  확정 = L ≥ l_confirm · 덩어리에 히트가 있었던 스캔 confirm_min_hits 이상 · 처음 생긴 뒤 confirm_min_age.
  덩어리가 사라지면 track_grace 동안 기다렸다 지운다.

색은 buoy_evidence 와 같다 — 이번 스캔 덩어리(카메라 창)가 맞힌 점들의 부표 id 로 그 부표에 누적.
EvidenceTracker 와 같은 update/tracks/confirmed/last_assoc 를 내서 노드가 그대로 쓴다.
"""
import math
from dataclasses import dataclass
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import connected_components
from scipy.spatial import cKDTree

from kaboat_perception.buoy_evidence import TIME_JUMP_BACK, MAX_STEP, EvTrack, EvidenceTracker, LidarView


@dataclass(frozen=True)
class PointMapParams:
    merge_radius: float = 0.05      # 새 점이 기존 점 이 안이면 그 점을 다시 본 것 [m]
    l_hit: float = 0.8
    l_free_rate: float = -2.0       # 빔이 점 자리를 지나감 [/s]
    l_solid: float = 1.5            # 이 이상이면 굳은 점 — 덩어리에 들어감 (2 번 보임)
    l_delete: float = -0.5
    l_max: float = 4.0
    tentative_timeout: float = 3.0  # 굳지 않은 점이 이만큼 다시 안 보이면 지움 [s]
    pass_margin: float = 0.3        # 빔이 점보다 이만큼 멀리서 맞아야 지나감 [m]
    min_beam_half_angle: float = math.radians(0.3)
    point_half_width: float = 0.05  # 점 방향 빔 창 반폭 [m]
    max_range: float = 15.0         # 이보다 먼 점은 깎지 않음 [m]
    forget_range: float = 17.0
    cluster_eps: float = 0.35
    min_points: int = 3
    min_width: float = 0.1
    max_width: float = 1.5
    spawn_min_sep: float = 1.0      # 한 덩어리에 붙은 확정 부표 둘이 이보다 멀면 나누고, 가까우면 합친다
    merge_dist: float = 0.9         # 중심이 이 안인 덩어리 둘은 합친다 (합친 폭 ≤ max_width) — 번진 점이
                                    # 떨어져 나온 조각 (gate bag 초록 부표 옆 0.83 m). 탐색 붙은 쌍은 ≥ 1.26 m
    size_window: int = 30           # 반폭 = 최근 이만큼 스캔 덩어리 반폭의 size_pct 백분위
    size_pct: float = 80.0          # (쌓인 점은 번짐 0.15 m 가 더해져 실제보다 넓다)
    radius_min: float = 0.1
    radius_max: float = 0.75
    l_confirm: float = 2.0
    confirm_min_hits: int = 4
    confirm_min_age: float = 0.5
    track_grace: float = 1.0        # 덩어리가 사라진 부표를 지우기 전 기다림 [s]
    px_min_frac: float = 0.15
    px_dominance: float = 2.0
    px_min_visible: int = 150


class PointMapTracker:
    def __init__(self, params: PointMapParams = None):
        self.p = params or PointMapParams()
        self.reset()

    def reset(self):
        self.P = np.zeros((0, 2))
        self.L = np.zeros(0)
        self.seen = np.zeros(0)
        self.label = np.zeros(0, dtype=int)     # 점이 속한 부표 id (-1 없음)
        self.pcol = np.zeros((0, 3))            # 부표가 되기 전에 찍힌 색 픽셀 (빨강, 초록, 보임) — 점에 맡김
        self.tracks: List[EvTrack] = []
        self._next_id = 0
        self._last_now: Optional[float] = None
        self._last_blob: Dict[int, float] = {}
        self.last_assoc: Dict[int, int] = {}
        self.blob_points: Dict[int, np.ndarray] = {}   # 부표 id → 지금 덩어리 점 (시각화)

    def confirmed(self) -> List[EvTrack]:
        return [t for t in self.tracks if t.confirmed]

    # ── 스캔 하나 ──────────────────────────────────────
    def update(self, detections: Sequence[Tuple], now: float,
               view: Optional[LidarView] = None) -> List[EvTrack]:
        """detections = [(x, y, 색 표, can_spawn, 덩어리 점 N×2 (전역))] — 이번 스캔 덩어리."""
        p = self.p
        if self._last_now is not None and now < self._last_now - TIME_JUMP_BACK:
            self.reset()
        dt = 0.0 if self._last_now is None else min(max(now - self._last_now, 0.0), MAX_STEP)
        self._last_now = now
        detections = [(*d, True, None)[:5] if len(d) == 3 else (*d, None)[:5] for d in detections]

        # ① 올림 / 새 점
        n_old = len(self.P)
        tree = cKDTree(self.P) if n_old else None
        hit = np.zeros(n_old, dtype=bool)
        det_hits: List[np.ndarray] = []
        new_pts, new_det = [], []
        for i, (_x, _y, _c, can_spawn, pts) in enumerate(detections):
            if pts is None or len(pts) == 0:
                det_hits.append(np.zeros(0, dtype=int))
                continue
            pts = np.asarray(pts, dtype=float).reshape(-1, 2)
            if tree is not None:
                d, j = tree.query(pts, distance_upper_bound=p.merge_radius)
                old = d <= p.merge_radius
                hit[j[old]] = True
                det_hits.append(np.unique(j[old]))
                fresh = pts[~old]
            else:
                det_hits.append(np.zeros(0, dtype=int))
                fresh = pts
            if can_spawn and len(fresh):
                new_pts.append(fresh)
                new_det.extend([i] * len(fresh))
        self.L[hit] = np.minimum(p.l_max, self.L[hit] + p.l_hit)
        self.seen[hit] = now

        # ② 깎음 (이번에 안 맞은 기존 점만)
        if view is not None and n_old:
            free = self._free_mask(view, ~hit)
            self.L[free] += p.l_free_rate * dt

        # 새 점 추가 (같은 스캔 안에서 merge_radius 안에 겹친 점은 하나만)
        new_idx = {}
        if new_pts:
            q = np.vstack(new_pts)
            keys = np.floor(q / p.merge_radius).astype(np.int64)
            _, first = np.unique(keys, axis=0, return_index=True)
            first = np.sort(first)
            base = len(self.P)
            self.P = np.vstack((self.P, q[first]))
            self.L = np.concatenate((self.L, np.full(len(first), p.l_hit)))
            self.seen = np.concatenate((self.seen, np.full(len(first), now)))
            self.label = np.concatenate((self.label, np.full(len(first), -1)))
            self.pcol = np.vstack((self.pcol, np.zeros((len(first), 3))))
            for k, f in enumerate(first):
                new_idx.setdefault(new_det[f], []).append(base + k)

        # ③ 지움
        keep = self.L > p.l_delete
        keep &= ~((self.L < p.l_solid) & (now - self.seen > p.tentative_timeout))
        if view is not None:
            keep &= np.hypot(self.P[:, 0] - view.x, self.P[:, 1] - view.y) <= p.forget_range
        remap = np.full(len(self.P), -1)
        remap[keep] = np.arange(int(keep.sum()))
        self.P, self.L, self.seen, self.label = self.P[keep], self.L[keep], self.seen[keep], self.label[keep]
        self.pcol = self.pcol[keep]
        det_pts = []
        for i in range(len(detections)):
            idx = np.concatenate((det_hits[i], np.asarray(new_idx.get(i, []), dtype=int)))
            idx = remap[idx] if len(idx) else idx
            det_pts.append(idx[idx >= 0])

        # ④ 덩어리 → 부표
        self._cluster(now)

        # ⑤ 색·연결 (이번 스캔 덩어리가 맞힌 점들의 부표 id). 아직 부표가 아닌 점이면 색을 점에 맡겨
        #    두었다가 부표가 될 때 넘긴다 — 카메라가 처음 한 번만 본 빨강 부표 색이 없어지지 않게.
        by_id = {t.id: t for t in self.tracks}
        self.last_assoc = {}
        for i, idx in enumerate(det_pts):
            labs = self.label[idx]
            labs = labs[labs >= 0]
            t = by_id.get(int(np.bincount(labs).argmax())) if len(labs) else None
            v = detections[i][2]
            w = detections[i][4]
            if t is not None:
                self.last_assoc[i] = t.id
                EvidenceTracker._add_color(t, v)
                if w is not None and len(w) >= 2:
                    b = np.asarray(w, dtype=float).reshape(-1, 2)
                    half = 0.5 * float(np.sqrt(np.max(np.sum((b[:, None] - b[None]) ** 2, axis=2))))
                    t.widths = (t.widths + [half])[-self.p.size_window:]
                    t.radius = min(self.p.radius_max, max(self.p.radius_min,
                                                          float(np.percentile(t.widths, self.p.size_pct))))
            elif len(idx) and v is not None and not isinstance(v, str) and getattr(v, 'usable', False):
                self.pcol[idx] += np.array([v.red, v.green, v.visible], dtype=float) / len(idx)
        return self.confirmed()

    def _free_mask(self, view: LidarView, cand: np.ndarray) -> np.ndarray:
        p = self.p
        r = np.asarray(view.ranges, dtype=float)
        valid = np.isfinite(r) & (r > view.range_min) & (r < view.range_max)
        rv = np.where(valid, r, np.inf)
        dx, dy = self.P[:, 0] - view.x, self.P[:, 1] - view.y
        d = np.hypot(dx, dy)
        bear = np.arctan2(dy, dx) - view.yaw
        bear = (bear - view.angle_min) % (2.0 * math.pi)
        idx = np.round(bear / view.angle_increment).astype(int)
        half = np.maximum(p.min_beam_half_angle, np.arctan2(p.point_half_width, np.maximum(d, 1e-3)))
        k = np.ceil(half / view.angle_increment).astype(int)
        n = len(r)
        out = np.zeros(len(self.P), dtype=bool)
        ok = cand & (d > view.range_min + p.pass_margin) & (d < view.range_max - p.pass_margin) & (d <= p.max_range)
        ok &= (idx >= 0) & (idx < n + 0)
        for kk in np.unique(k[ok]):
            sel = np.where(ok & (k == kk))[0]
            offs = np.arange(-kk, kk + 1)
            win = idx[sel, None] + offs[None, :]
            inside = (win >= 0) & (win < n)
            vals = np.where(inside, rv[np.clip(win, 0, n - 1)], np.inf)
            mn = vals.min(axis=1)
            any_ret = np.isfinite(mn)
            out[sel] = any_ret & (mn > d[sel] + p.pass_margin)
        return out

    def _merge_close(self, groups):
        """중심이 merge_dist 안인 조각을 합친다 (합친 폭이 max_width 이하일 때만)."""
        p = self.p
        groups = [np.asarray(g) for g in groups]
        changed = True
        while changed and len(groups) > 1:
            changed = False
            cs = [self.P_solid[g].mean(axis=0) for g in groups]
            for i in range(len(groups)):
                for j in range(i + 1, len(groups)):
                    if (np.hypot(*(cs[i] - cs[j])) <= p.merge_dist
                            and not self._apart(self._owners(self._solid[groups[i]], True)
                                                + self._owners(self._solid[groups[j]], True))):
                        # 서로 1 m 넘게 떨어진 확정 부표 둘이면 안 합침 (gate bag 0.9 m 옆 물체)
                        g = np.concatenate((groups[i], groups[j]))
                        if _width(self.P_solid[g]) <= p.max_width:
                            groups[i] = g
                            groups.pop(j)
                            changed = True
                            break
                if changed:
                    break
        return groups

    def _owners(self, idx, confirmed_only=False):
        """점 묶음 안에서 min_points 개 이상 점을 가진 기존 부표 id."""
        ids = {t.id for t in self.tracks if t.confirmed or not confirmed_only}
        labs = self.label[idx]
        return [int(v) for v, n in zip(*np.unique(labs[labs >= 0], return_counts=True))
                if n >= self.p.min_points and int(v) in ids]

    def _apart(self, ids) -> bool:
        """확정 부표들 중 중심이 spawn_min_sep 보다 먼 쌍이 있나 — 있으면 서로 다른 부표."""
        by_id = {t.id: t for t in self.tracks}
        c = [(by_id[v].x, by_id[v].y) for v in set(ids) if v in by_id]
        return any(math.dist(a, b) > self.p.spawn_min_sep for i, a in enumerate(c) for b in c[i + 1:])

    def _split_wide(self, solid, grp, confirmed_only=False):
        """폭이 max_width 를 넘는 덩어리 — 배가 돌 때 번진 점이 부표에 붙어 넓어진 경우가 많다
        (gate bag 71 s: 통째로 버려져 부표 id 가 바뀌고 색을 잃었다). 안에 기존 부표가 있으면 점을
        가장 가까운 기존 부표 중심(radius_max + cluster_eps 안)으로 나누고, 없으면 그대로 버린다(벽·물가)."""
        p = self.p
        by_id = {t.id: t for t in self.tracks}
        ids = self._owners(solid[grp], confirmed_only)
        if not ids:
            return []
        c = np.array([[by_id[v].x, by_id[v].y] for v in ids])
        b = self.P_solid[grp]
        d = np.hypot(b[:, None, 0] - c[None, :, 0], b[:, None, 1] - c[None, :, 1])
        near = d.argmin(axis=1)
        ok = d.min(axis=1) <= p.radius_max      # 부표 중심 반폭 상한 안 — 나눈 조각 폭 ≤ max_width
        out = []
        for k in range(len(ids)):
            sub = grp[ok & (near == k)]
            if len(sub) >= p.min_points:
                out.append(sub)
        return out

    def _cluster(self, now):
        p = self.p
        solid = np.where(self.L >= p.l_solid)[0]
        blobs = []
        if len(solid) >= p.min_points:
            pts = self.P[solid]
            self.P_solid = pts
            self._solid = solid
            pr = cKDTree(pts).query_pairs(p.cluster_eps, output_type='ndarray')
            m = len(pts)
            g = coo_matrix((np.ones(len(pr)), (pr[:, 0], pr[:, 1])), shape=(m, m))
            _k, comp = connected_components(g, directed=False)
            order = np.argsort(comp, kind='stable')
            splits = np.flatnonzero(np.diff(comp[order])) + 1
            for grp in np.split(order, splits):
                if len(grp) < p.min_points:
                    continue
                if _width(pts[grp]) > p.max_width:
                    blobs.extend(self._split_wide(solid, grp))   # 넓음 → 기존 부표 중심으로 나눔
                elif self._apart(self._owners(solid[grp], True)):
                    blobs.extend(self._split_wide(solid, grp, True))   # 떨어진 확정 부표 둘이 붙음
                else:
                    blobs.append(grp)
            blobs = self._merge_close(blobs)
            out = []
            for grp in blobs:
                b = pts[grp]
                width = _width(b)
                if p.min_width <= width <= p.max_width:
                    out.append((solid[grp], b, width))
            blobs = out
        # id 다수결 — 점이 많은 덩어리부터 기존 id 를 가져감
        by_id = {t.id: t for t in self.tracks}
        taken = set()
        self.blob_points = {}
        self.label[:] = np.where(np.isin(self.label, list(by_id)), self.label, -1)
        for idx, b, width in sorted(blobs, key=lambda x: -len(x[0])):
            labs = self.label[idx]
            labs = labs[(labs >= 0)]
            cand = [int(v) for v in np.unique(labs) if int(v) not in taken]
            if cand:
                counts = {v: int(np.sum(labs == v)) for v in cand}
                tid = max(counts, key=counts.get)
                t = by_id[tid]
                for v in cand:            # 같은 덩어리에 다른 부표 점이 대부분 들어오면 합친다 (몇 점만
                    # 걸친 거면 합치지 않는다 — 옆 물체 덩어리에 점 몇 개가 걸려 부표 id 가 옆으로 넘어갔다)
                    if (v != tid and v in by_id
                            and counts[v] >= 0.5 * int(np.sum(self.label == v))):
                        o = by_id.pop(v)
                        t.px_red += o.px_red
                        t.px_green += o.px_green
                        t.px_visible += o.px_visible
                        for c, n in o.color_votes.items():
                            t.color_votes[c] = t.color_votes.get(c, 0) + n
                        t.born = min(t.born, o.born)
                        t.hits = max(t.hits, o.hits)
                        t.confirmed = t.confirmed or o.confirmed
                        self.label[self.label == v] = tid
            else:
                tid = self._next_id
                self._next_id += 1
                t = EvTrack(tid, 0.0, 0.0, 0.0, now, born=now)
                t.px_params = (p.px_min_frac, p.px_dominance, p.px_min_visible)
                by_id[tid] = t
            taken.add(tid)
            self.label[idx] = tid
            c = b.mean(axis=0)
            # 점에 맡겨 둔 색을 부표로 — 덩어리 점뿐 아니라 중심 radius_max 안의 아직 안 굳은 점까지 한 번에.
            # 탐색 bag 빨강 부표는 카메라에 한 프레임(187 px)만 찍혀, 굳은 점 몫만 넘기면 1 s 넘게 unseen 이었다.
            near = np.flatnonzero((np.hypot(self.P[:, 0] - c[0], self.P[:, 1] - c[1]) <= p.radius_max)
                                  & (self.label < 0))
            take = np.concatenate((idx, near))
            col = self.pcol[take].sum(axis=0)
            if col[2] > 0:
                t.px_red += int(round(col[0]))
                t.px_green += int(round(col[1]))
                t.px_visible += int(round(col[2]))
                self.pcol[take] = 0.0
            t.x, t.y = float(c[0]), float(c[1])
            if not t.widths:
                t.radius = min(p.radius_max, max(p.radius_min, 0.5 * width))
            t.L = float(self.L[idx].max())
            if np.any(self.seen[idx] == now):
                t.hits += 1
                t.last_seen = now
                t.last_evidence = 'hit'
            else:
                t.last_evidence = 'kept'
            if (not t.confirmed and t.L >= p.l_confirm and t.hits >= p.confirm_min_hits
                    and now - t.born >= p.confirm_min_age):
                t.confirmed = True
            self._last_blob[tid] = now
            self.blob_points[tid] = b
        keep = []
        for tid, t in by_id.items():
            if tid in taken or now - self._last_blob.get(tid, now) <= p.track_grace:
                keep.append(t)
            else:
                self.label[self.label == tid] = -1
                self._last_blob.pop(tid, None)
        self.tracks = sorted(keep, key=lambda t: t.id)


def _width(b) -> float:
    """가장 먼 두 점 거리."""
    if len(b) < 2:
        return 0.0
    return float(np.sqrt(np.max(np.sum((b[:, None, :] - b[None, :, :]) ** 2, axis=2))))
