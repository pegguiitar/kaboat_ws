"""횡단 판 + 선미 방향 세로판(┛ 모양)에서 수조 기준 선체 pose를 추정한다.

선체 좌표(x 선수, y 좌현, 원점 base_link)에서 판 모양은 다음과 같다.
  - 횡단 판: (0, +left) ~ (0, -right)
  - 세로판: 우현 끝 (0, -right) 에서 선미로 sail 길이만큼 (-sail, -right)
좌우 비대칭이라 두 판이 함께 보이면 yaw가 360° 범위에서 하나로 정해진다.

추정 순서:
  1. 클러스터에서 점이 가장 많이 올라가는 주 직선을 찾는다(RANSAC). 주 직선에
     안 맞는 나머지 점은 직선 맞춤에 쓰지 않고 해석 비교에만 쓴다.
  2. 주 직선 하나로 해석 4가지를 만든다: 횡단 판/세로판 × 꺾이는 지점이 어느
     끝인지. 꺾이는 지점은 항상 직선의 한쪽 끝이다. 해석끼리는 90°씩 다르다.
     남은 점으로 두 번째 직선이 잡히면 두 직선의 교차점 해석도 더한다.
  3. 각 해석을 L 모델 전체에 대해 ICP로 맞춘다.
  4. 모든 점이 L 모델 위에 있어야 비용이 낮다. 모델 밖(꺾이는 지점 너머 등)의
     점은 크게 감점하고, 판이 있어야 할 자리 뒤로 벽이 보이면(raycast) 감점한다.
"""

from dataclasses import dataclass
import math
from typing import List, Optional, Tuple

import numpy as np

from kaboat_hardware.pose_velocity import normalize_angle


@dataclass(frozen=True)
class SailGeometry:
    left: float = 0.45    # [m] base_link → 횡단 판 좌현 끝
    right: float = 0.30   # [m] base_link → 횡단 판 우현 끝(세로판 뿌리)
    sail: float = 0.45    # [m] 세로판 길이 (우현 끝에서 선미 방향)

    def segments(self, x, y, yaw):
        """수조 좌표의 판 선분 [(횡단 판), (세로판)] 을 반환한다."""
        c, s = math.cos(yaw), math.sin(yaw)

        def tf(px, py):
            return np.array([x + c * px - s * py, y + s * px + c * py])

        root = tf(0.0, -self.right)
        return [(tf(0.0, self.left), root), (root, tf(-self.sail, -self.right))]


@dataclass(frozen=True)
class Line:
    start: np.ndarray
    end: np.ndarray
    rms: float
    count: int

    @property
    def span(self):
        return float(np.linalg.norm(self.end - self.start))


@dataclass(frozen=True)
class SailPose:
    x: float
    y: float
    yaw: float
    cost: float          # 점당 평균 비용 (작을수록 좋음)
    matched: int         # L 모델 위에 놓인 점 수
    lines: int           # 2면 두 판이 모두 보인 확실한 L자
    yaw_unique: bool     # 확실한 L자로 yaw 를 스캔에서 직접 정했는지
    segments: Tuple[Tuple[np.ndarray, np.ndarray], ...]


def fit_line(points) -> Optional[Line]:
    pts = np.asarray(points, dtype=float)
    if len(pts) < 2:
        return None
    mean = pts.mean(axis=0)
    _, _, axes = np.linalg.svd(pts - mean, full_matrices=False)
    tangent, normal = axes
    along = (pts - mean) @ tangent
    across = (pts - mean) @ normal
    # 스캔 순서의 양 끝을 start/end 로 둔다.
    if along[0] > along[-1]:
        tangent = -tangent
        along = -along
    return Line(
        start=mean + tangent * along.min(), end=mean + tangent * along.max(),
        rms=float(np.sqrt(np.mean(across ** 2))), count=len(pts))


def _largest_piece(pts, idx, origin_pt, direction, max_gap):
    """직선 위 inlier 중 max_gap 보다 크게 끊기지 않은 가장 큰 조각."""
    along = (pts[idx] - origin_pt) @ direction
    order = np.argsort(along)
    best, cur = [], [order[0]]
    for a, b in zip(order[:-1], order[1:]):
        if along[b] - along[a] > max_gap:
            best = max(best, cur, key=len)
            cur = []
        cur.append(b)
    best = max(best, cur, key=len)
    return idx[best]


def ransac_line(points, inlier_tol=0.03, max_gap=0.45, min_points=3,
                max_pairs=400, seed=0):
    """끊김 없이 이어진 점이 가장 많이 올라가는 직선과 그 inlier 마스크."""
    pts = np.asarray(points, dtype=float)
    n = len(pts)
    if n < min_points:
        return None, None
    if n * (n - 1) // 2 <= max_pairs:
        pairs = [(i, j) for i in range(n) for j in range(i + 1, n)]
    else:
        rng = np.random.default_rng(seed)
        pairs = [tuple(rng.choice(n, 2, replace=False))
                 for _ in range(max_pairs)]
    best_keep = None
    for i, j in pairs:
        d = pts[j] - pts[i]
        norm = float(np.hypot(*d))
        if norm < 0.05:
            continue
        normal = np.array([-d[1], d[0]]) / norm
        idx = np.where(np.abs((pts - pts[i]) @ normal) <= inlier_tol)[0]
        if best_keep is not None and len(idx) <= len(best_keep):
            continue
        keep = _largest_piece(pts, idx, pts[i], d / norm, max_gap)
        if best_keep is None or len(keep) > len(best_keep):
            best_keep = keep
    if best_keep is None or len(best_keep) < min_points:
        return None, None
    mask = np.zeros(n, dtype=bool)
    mask[best_keep] = True
    return fit_line(pts[mask]), mask


def _unit(v):
    n = float(np.linalg.norm(v))
    return v / n if n > 1e-9 else None


def _rot90(v):
    return np.array([-v[1], v[0]])


def _pose_from_axes(base, x_axis):
    return (float(base[0]), float(base[1]),
            math.atan2(float(x_axis[1]), float(x_axis[0])))


def _line_intersection(l1: Line, l2: Line):
    d1, d2 = l1.end - l1.start, l2.end - l2.start
    den = d1[0] * d2[1] - d1[1] * d2[0]
    if abs(den) < 1e-9:
        return None
    w = l2.start - l1.start
    t = (w[0] * d2[1] - w[1] * d2[0]) / den
    return l1.start + t * d1


def single_line_hypotheses(line: Line, geom: SailGeometry):
    """주 직선 하나의 해석 8가지 [(pose, 고정한 끝의 바깥 방향)].

    직선은 횡단 판 또는 세로판의 일부다. 직선의 한쪽 끝을 판의 한쪽 끝에 맞춘다:
      - 보통은 꺾이는 지점(우현 끝 = 세로판 뿌리)이 직선 끝이다.
      - 세로판 점이 안 돌아오면 꺾이는 지점이 세로판 뒤에 가려지므로, 판의 자유단
        (횡단 판 좌현 끝, 세로판 꼬리)을 직선 끝에 맞춘 해석도 둔다.
    어느 해석이든 점은 모두 판 안쪽에 있다. 바깥 방향은 빔 증거로 끝점을
    보정할 때 쓴다(점이 성기면 실제 끝이 마지막 점보다 조금 더 바깥이다).
    """
    out = []
    for c_end, o_end in ((line.start, line.end), (line.end, line.start)):
        u = _unit(o_end - c_end)
        if u is None:
            continue
        # 횡단 판: c_end 쪽이 우현, o_end 쪽이 좌현(+y).
        x_axis = np.array([u[1], -u[0]])
        out.append((_pose_from_axes(c_end + geom.right * u, x_axis), -u))
        out.append((_pose_from_axes(o_end - geom.left * u, x_axis), u))
        # 세로판: c_end 쪽이 뿌리, o_end 쪽이 선미(-x).
        x_axis = -u
        y_axis = _rot90(x_axis)
        out.append((_pose_from_axes(c_end + geom.right * y_axis, x_axis), -u))
        root = o_end - geom.sail * u
        out.append((_pose_from_axes(root + geom.right * y_axis, x_axis), u))
    return out


def corner_hypotheses(l1: Line, l2: Line, geom: SailGeometry):
    """꺾인 두 직선의 교차점을 우현 끝으로 둔 해석."""
    corner = _line_intersection(l1, l2)
    if corner is None:
        return []
    out = []
    far = [max((ln.start, ln.end), key=lambda p: np.linalg.norm(p - corner))
           for ln in (l1, l2)]
    for cross_far, sail_far in ((far[0], far[1]), (far[1], far[0])):
        y_axis = _unit(cross_far - corner)
        x_axis = _unit(corner - sail_far)
        if y_axis is None or x_axis is None:
            continue
        if float(_rot90(x_axis) @ y_axis) < 0.5:
            continue  # 좌우가 뒤집힌 해석
        yaw = math.atan2(x_axis[1], x_axis[0])
        yaw_y = math.atan2(y_axis[1], y_axis[0]) - math.pi / 2.0
        yaw += 0.5 * normalize_angle(yaw_y - yaw)
        y_axis = np.array([-math.sin(yaw), math.cos(yaw)])
        base = corner + geom.right * y_axis
        out.append(((float(base[0]), float(base[1]), yaw), None))
    return out


def slide_to_evidence(pose, outward, points, geom, origin, angles, ranges,
                      valid, max_slide=0.30, step=0.02, see_through_margin=0.15,
                      point_tol=0.04):
    """꺾이는 지점을 바깥으로 밀 수 있는 구간의 가운데로 옮긴다.

    판 뒤 벽이 새로 비쳐 보이거나 점이 판 밖으로 떨어지기 직전까지가 가능한
    구간이다. 점이 성겨 실제 끝이 마지막 점과 다음 빔 사이 어딘가일 때 쓴다.
    """
    x, y, yaw = pose

    def state(shift):
        sx, sy = x + shift * outward[0], y + shift * outward[1]
        segments = geom.segments(sx, sy, yaw)
        expected = raycast(origin, angles, segments)
        see = int(np.count_nonzero(valid & np.isfinite(expected) & (
            ranges > expected + see_through_margin)))
        _, d, _, _ = closest_on_model(points, segments)
        return see, int(np.count_nonzero(d > point_tol))

    base_see, base_off = state(0.0)
    feasible = 0.0
    shift = step
    while shift <= max_slide + 1e-9:
        see, off = state(shift)
        if see > base_see or off > base_off:
            break
        feasible = shift
        shift += step
    half = 0.5 * feasible
    return (x + half * outward[0], y + half * outward[1], yaw), half


def closest_on_model(points, segments):
    """각 점의 L 모델 최근접점, 거리, 판 번호, 선분 위치(0~1)."""
    pts = np.asarray(points, dtype=float)
    best_d = np.full(len(pts), np.inf)
    best_q = np.zeros_like(pts)
    best_i = np.zeros(len(pts), dtype=int)
    best_t = np.zeros(len(pts))
    for i, (a, b) in enumerate(segments):
        e = b - a
        t = np.clip(((pts - a) @ e) / max(float(e @ e), 1e-12), 0.0, 1.0)
        q = a + t[:, None] * e
        d = np.hypot(*(pts - q).T)
        closer = d < best_d
        best_d = np.where(closer, d, best_d)
        best_q[closer] = q[closer]
        best_i = np.where(closer, i, best_i)
        best_t = np.where(closer, t, best_t)
    return best_q, best_d, best_i, best_t


def icp_refine(pose, points, geom, iterations=10, huber=0.05, reject=0.30,
               max_yaw_change=math.pi):
    """L 모델 전체를 점들에 맞춘다 (최근접점 ICP, 2D 강체).

    max_yaw_change: 시작 yaw 에서 벗어날 수 있는 최대 각도. 직선 해석은 직선
    방향이 믿을 만하므로 작게 묶어 성긴 점에 끌려 돌아가지 않게 한다.
    """
    x, y, yaw = pose
    yaw0 = yaw
    pts = np.asarray(points, dtype=float)
    for _ in range(iterations):
        q, d, _, _ = closest_on_model(pts, geom.segments(x, y, yaw))
        use = d < reject
        if np.count_nonzero(use) < 3:
            break
        w = np.where(d < huber, 1.0, huber / np.maximum(d, 1e-9))[use]
        e = (pts - q)[use]
        r = q[use] - np.array([x, y])
        r = np.column_stack((-r[:, 1], r[:, 0]))  # d q / d yaw
        # [dx, dy, dyaw] 최소제곱: e ≈ [dx, dy] + dyaw * r
        rows = np.zeros((2 * len(e), 3))
        rows[0::2, 0] = 1.0
        rows[1::2, 1] = 1.0
        rows[0::2, 2] = r[:, 0]
        rows[1::2, 2] = r[:, 1]
        sw = np.sqrt(np.repeat(w, 2))
        delta, *_ = np.linalg.lstsq(
            rows * sw[:, None], e.reshape(-1) * sw, rcond=None)
        x, y = x + delta[0], y + delta[1]
        change = normalize_angle(yaw + delta[2] - yaw0)
        change = max(-max_yaw_change, min(max_yaw_change, change))
        yaw = normalize_angle(yaw0 + change)
        if np.hypot(delta[0], delta[1]) < 1e-4 and abs(delta[2]) < 1e-4:
            break
    return (float(x), float(y), float(yaw))


def raycast(origin, angles, segments):
    """각 빔이 판 모델에 처음 닿는 거리 (안 닿으면 inf)."""
    return raycast_with_index(origin, angles, segments)[0]


def raycast_with_index(origin, angles, segments):
    """각 빔이 처음 닿는 거리와 그 판 번호 (안 닿으면 inf, -1)."""
    d = np.column_stack((np.cos(angles), np.sin(angles)))
    best = np.full(len(angles), np.inf)
    index = np.full(len(angles), -1)
    for i, (a, b) in enumerate(segments):
        e = b - a
        w = a - origin
        den = d[:, 0] * e[1] - d[:, 1] * e[0]
        with np.errstate(divide='ignore', invalid='ignore'):
            t = (w[0] * e[1] - w[1] * e[0]) / den
            u = (w[0] * d[:, 1] - w[1] * d[:, 0]) / den
        hit = (np.abs(den) > 1e-12) & (t > 0) & (u >= 0) & (u <= 1)
        closer = hit & (t < best)
        best = np.where(closer, t, best)
        index = np.where(closer, i, index)
    return best, index


@dataclass(frozen=True)
class _Scored:
    cost: float          # 총 비용
    mean: float          # 점당 비용
    pose: Tuple[float, float, float]
    arm_hits: Tuple[int, int]
    matched: int


def score_pose(pose, points, geom, origin, angles, ranges, valid,
               point_sigma=0.03, miss_cost=9.0, corner_zone=0.05,
               see_through_margin=0.15):
    """L 모델이 클러스터 점을 얼마나 설명하는지 + 벽이 비쳐 보이는 빔 감점."""
    segments = geom.segments(*pose)
    _, d, seg, t = closest_on_model(points, segments)
    cost = float(np.sum(np.minimum((d / point_sigma) ** 2, miss_cost)))
    on_model = d < 0.04
    # 꺾이는 지점 근처 점은 어느 판인지 애매하므로 판별 개수에서 뺀다.
    lengths = (geom.left + geom.right, geom.sail)
    dist_to_corner = np.where(seg == 0, (1.0 - t) * lengths[0], t * lengths[1])
    clear = on_model & (dist_to_corner > corner_zone)
    arm_hits = (int(np.count_nonzero(clear & (seg == 0))),
                int(np.count_nonzero(clear & (seg == 1))))
    # 판이 있어야 할 빔에서 그 뒤의 물체가 보이면 판이 거기 없다.
    expected = raycast(origin, angles, segments)
    see_through = valid & np.isfinite(expected) & (
        ranges > expected + see_through_margin)
    cost += miss_cost * float(np.count_nonzero(see_through))
    n = max(len(points), 1)
    return _Scored(cost=cost, mean=cost / n, pose=tuple(pose),
                   arm_hits=arm_hits, matched=int(np.count_nonzero(on_model)))


def estimate_sail_pose(
    clusters,
    origin,
    angles,
    ranges,
    valid,
    geom: SailGeometry = SailGeometry(),
    min_points: int = 3,
    max_extent: float = 1.2,
    line_rms_tol: float = 0.03,
    max_mean_cost: float = 3.0,
    previous_pose: Optional[Tuple[float, float, float]] = None,
    max_position_jump: float = math.inf,
    reference_yaw: Optional[float] = None,
    yaw_follow_window: float = math.radians(50.0),
    min_arm_hits: int = 2,
    default_yaw: Optional[float] = None,
    min_span: float = 0.15,
    tie_cost: float = 18.0,
) -> Optional[SailPose]:
    """클러스터들 중 L 모델과 가장 잘 맞는 선체 pose 를 고른다.

    angles 는 수조 좌표 기준 빔 방향, ranges/valid 는 같은 길이의 전체 빔이다.

    yaw 결정 규칙 (L자 판정: 두 판 모두 min_arm_hits 개 이상 점):
      - reference_yaw(기존 yaw)가 없으면: L자 해석이 있으면 그것, 없으면 비용이
        가장 낮은 해석. 비슷한 해석끼리는 default_yaw 쪽.
      - 기존 yaw 가 있으면 기본은 ±yaw_follow_window 안의 해석이다(직선 해석은
        90° 간격이라 보통 하나는 있다). 비용 + 기존 yaw 와의 차이로 고른다.
        두 판 모두 min_arm_hits+1 개 이상인 강한 L자만 범위 밖이어도 후보가
        되며, 이때도 yaw 차이를 비용에 더해 증거가 충분할 때만 넘어간다.
    """
    origin = np.asarray(origin, dtype=float)
    angles = np.asarray(angles, dtype=float)
    ranges = np.asarray(ranges, dtype=float)
    valid = np.asarray(valid, dtype=bool)
    scored: List[_Scored] = []
    for points in clusters:
        pts = np.asarray(points, dtype=float)
        if len(pts) < min_points:
            continue
        if float(np.max(np.ptp(pts, axis=0))) > max_extent:
            continue
        main, mask = ransac_line(pts, inlier_tol=line_rms_tol,
                                 min_points=min_points)
        if main is None:
            # 먼 거리에서 두 판에 점이 2개씩만 찍히면 3점 직선이 없다.
            # 스캔 순서로 두 조각을 나눠 각각 직선으로 본다.
            hypotheses = []
            if len(pts) >= 4:
                parts = min(((fit_line(pts[:k]), fit_line(pts[k:]))
                             for k in range(2, len(pts) - 1)),
                            key=lambda ab: ab[0].rms * ab[0].count
                            + ab[1].rms * ab[1].count)
                for part in parts:
                    if part.span >= 0.05:
                        hypotheses += single_line_hypotheses(part, geom)
                hypotheses += corner_hypotheses(parts[0], parts[1], geom)
            if not hypotheses:
                continue
            main = None
        elif main.span < min_span:
            continue
        else:
            hypotheses = single_line_hypotheses(main, geom)
        rest = pts[~mask] if main is not None else np.empty((0, 2))
        if len(rest) >= 2:
            second, _ = ransac_line(rest, inlier_tol=line_rms_tol, min_points=2)
            if second is not None:
                d1 = (main.end - main.start) / max(main.span, 1e-9)
                d2 = (second.end - second.start) / max(second.span, 1e-9)
                if abs(float(d1 @ d2)) < 0.6:
                    hypotheses += corner_hypotheses(main, second, geom)

        # 클러스터 방향 근처 빔만 raycast 비교에 쓴다.
        centroid = pts.mean(axis=0)
        dist = float(np.linalg.norm(centroid - origin))
        bearing = math.atan2(*(centroid - origin)[::-1])
        half = math.asin(min(1.0, 1.0 / max(dist, 1e-6))) + math.radians(2)
        offset = (angles - bearing + math.pi) % (2.0 * math.pi) - math.pi
        sel = np.abs(offset) <= half
        a_sel, r_sel, v_sel = angles[sel], ranges[sel], valid[sel]

        # 추적 중이면 직전 pose 도 후보로 둔다 (점이 아주 적을 때의 연속성).
        if previous_pose is not None and math.hypot(
                previous_pose[0] - centroid[0],
                previous_pose[1] - centroid[1]) < 1.0:
            hypotheses.append((tuple(previous_pose), 'previous'))
        for pose, outward in hypotheses:
            uncertain = 0.0
            if isinstance(outward, str):
                pose = icp_refine(pose, pts, geom,
                                  max_yaw_change=math.radians(15))
            elif outward is not None:
                # 끝점을 빔 증거로 먼저 옮긴 뒤, 직선 방향은 크게 바꾸지 않고 맞춘다.
                pose, uncertain = slide_to_evidence(
                    pose, outward, pts, geom, origin, a_sel, r_sel, v_sel)
                pose = icp_refine(pose, pts, geom,
                                  max_yaw_change=math.radians(15))
            else:
                pose = icp_refine(pose, pts, geom)
            s = score_pose(pose, pts, geom, origin, a_sel, r_sel, v_sel)
            if uncertain > 0.0:
                # 고정한 끝 바로 바깥이 비어 보이지 않으면(무효 빔) 끝이 확실치 않다.
                s = _Scored(cost=s.cost + 9.0 * uncertain / 0.10,
                            mean=(s.cost + 9.0 * uncertain / 0.10)
                            / max(len(pts), 1),
                            pose=s.pose, arm_hits=s.arm_hits, matched=s.matched)
            if s.matched >= min_points:
                scored.append(s)

    scored = [s for s in scored if s.mean <= max_mean_cost]
    if previous_pose is not None:
        scored = [s for s in scored
                  if math.hypot(s.pose[0] - previous_pose[0],
                                s.pose[1] - previous_pose[1])
                  <= max_position_jump]
    if not scored:
        return None
    scored.sort(key=lambda s: s.cost)

    def yaw_gap(s, ref):
        return abs(normalize_angle(s.pose[2] - ref))

    def is_l(s, extra=0):
        return min(s.arm_hits) >= min_arm_hits + extra

    if reference_yaw is None:
        confirmed = [s for s in scored
                     if is_l(s) and s.cost <= scored[0].cost + tie_cost]
        best = confirmed[0] if confirmed else scored[0]
        if not confirmed and default_yaw is not None:
            # 빔 2개 분량 이내로 비슷한 해석끼리만 기본 방향으로 고른다.
            rivals = [s for s in scored if s.cost <= best.cost + tie_cost]
            best = min(rivals, key=lambda s: yaw_gap(s, default_yaw))
    else:
        options = [s for s in scored
                   if yaw_gap(s, reference_yaw) <= yaw_follow_window
                   or is_l(s, extra=1)]
        if not options:
            return None

        def key(s):
            weight = 0.5 if is_l(s, extra=1) else 2.0
            return s.mean + weight * yaw_gap(s, reference_yaw) / yaw_follow_window

        best = min(options, key=key)
    confirmed = is_l(best)

    x, y, yaw = best.pose
    return SailPose(
        x=x, y=y, yaw=yaw, cost=best.mean, matched=best.matched,
        lines=2 if confirmed else 1, yaw_unique=confirmed,
        segments=tuple(geom.segments(x, y, yaw)))
