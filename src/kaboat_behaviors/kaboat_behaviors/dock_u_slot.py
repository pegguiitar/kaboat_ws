"""도킹 슬롯 ㄷ자 검출 — 고정 치수(dock_slot.yaml)로 뒷벽·양 안쪽 면·칸막이 끝을 한꺼번에 맞춘다 (ROS 비의존).

라이다 직선 하나(RANSAC)로 벽을 정하면, 뒷벽과 평행한 칸막이 끝면(뒷벽 1.94 m 앞)도 같은 조건을
통과해 정지점이 2 m 짧고 옆으로 어긋났다(2026-11-12 bag, 접근 중 첫 고정 8번 중 5번). 여기서는
슬롯 모양 전체를 찾는다:

    위에서 본 모습 (w = 도크 쪽 앞방향, u = 왼쪽)
        w_b ───────────── 뒷벽 ─────────────        ← back
            ┃왼쪽 안쪽 면           오른쪽 안쪽 면┃  ← left_face / right_face (깊이 depth)
    w_b-D ━━┛  (슬롯 안 = 물, 비어 있어야 함)    ┗━━  ← left_tip / right_tip (칸막이 끝, 바깥쪽 tip_length)
                       ⛴

칸막이 끝 직선을 뒷벽으로 보는 가설은 그 앞에 안쪽 면이 없어 떨어지고, 칸막이 가운데를 중심으로 보는
가설은 슬롯 안(interior)에 칸막이 점이 있어 떨어진다. 이웃 슬롯(간격 ~4 m)은 YOLO 목표 방위선에서
target_offset 안의 중심만 보므로 고르지 않는다.

입력 점은 같은 고정 좌표계(odom)에 쌓은 최근 스캔들. 반환 중심은 뒷벽 면 위 슬롯 중앙, outward 는
뒷벽에서 물 쪽 단위 법선 — 진입 방향은 -outward (법선과 평행).
"""
from dataclasses import dataclass
import math

import numpy as np


@dataclass(frozen=True)
class USlotParams:
    slot_width: float                # 칸막이 안쪽 면 사이 [m] — dock_slot.yaml slot_width
    depth: float                     # 뒷벽 면 → 칸막이 끝 면 [m] — dock_slot.yaml wall_to_mouth_m
    cell: float = .05                # 격자 [m]
    tip_length: float = .6           # 칸막이 끝면 중 슬롯 쪽 이만큼만 본다 (칸막이 폭 ~1.05 m)
    min_range: float = .8            # 이보다 가까운 점은 선체·물보라
    max_range: float = 12.0
    max_oblique_deg: float = 40.0    # 목표 방위선과 슬롯 법선 사이 최대각 (dock_lidar_markers 와 같음)
    target_offset: float = 1.6       # 슬롯 중심이 목표 방위선에서 이만큼 안 (슬롯 간격 ~4 m 의 절반 미만)
    min_wall_distance: float = 1.0   # 뒷벽이 배에서 이보다 멀어야 (앞방향)
    min_back: float = .35            # 뒷벽 선분 칸 중 점이 있는 비율
    min_face: float = .25            # 안쪽 면
    min_tip: float = .30             # 칸막이 끝 (안쪽 면이 가려졌을 때 대신)
    max_interior: float = .03        # 슬롯 안(물)에 점이 있는 칸 비율 상한
    min_points: int = 30

    def __post_init__(self):
        values = (self.slot_width, self.depth, self.cell, self.tip_length, self.min_range,
                  self.max_range, self.max_oblique_deg, self.target_offset, self.min_wall_distance)
        if not all(math.isfinite(v) and v > 0 for v in values):
            raise ValueError('U-slot parameters must be positive and finite')
        if not self.min_range < self.max_range or not 0 < self.max_oblique_deg < 45:
            raise ValueError('invalid U-slot range or obliqueness limit')
        if self.slot_width < 4 * self.cell or self.depth < 4 * self.cell:
            raise ValueError('slot dimensions too small for the grid')


@dataclass(frozen=True)
class USlot:
    center: tuple        # 뒷벽 면 위 슬롯 중앙 (입력 좌표계)
    outward: tuple       # 뒷벽 → 물 쪽 단위 법선
    coverage: dict       # back / left_face / right_face / left_tip / right_tip / interior (0~1)
    score: float
    wall_distance: float  # 배(boat_xy) → 뒷벽, 법선 방향 [m]


def _unit(v):
    v = np.asarray(v, float)
    n = float(np.linalg.norm(v))
    if v.shape != (2,) or not math.isfinite(n) or n < 1e-9:
        raise ValueError('direction must be a finite non-zero XY vector')
    return v / n


def _rotate(v, angle):
    c, s = math.cos(angle), math.sin(angle)
    return np.array([c * v[0] - s * v[1], s * v[0] + c * v[1]])


def _sharpness(values, cell):
    idx = np.floor((values - values.min()) / cell).astype(np.int64)
    counts = np.bincount(idx)
    return float(np.dot(counts, counts))


def dock_orientation(points, forward, max_oblique_deg):
    """점들의 직선 구조가 가장 날카로워지는 앞방향(도크 쪽) — forward 에서 ±max_oblique 안.

    뒷벽·칸막이 끝(앞방향에 수직)과 안쪽 면(앞방향에 평행)이 함께 정렬되는 각을 찾는다 — 90° 주기라
    탐색 범위를 ±45° 미만으로 두어 한쪽만 고른다.
    """
    pts = np.asarray(points, float)

    def score(angle):
        f = _rotate(forward, angle)
        t = np.array([-f[1], f[0]])
        return _sharpness(pts @ f, .05) + _sharpness(pts @ t, .05)

    limit = math.radians(max_oblique_deg)
    coarse = np.arange(-limit, limit + 1e-9, math.radians(1.0))
    best = max(coarse, key=score)
    fine = best + np.radians(np.arange(-1.0, 1.0001, .1))
    best = max((a for a in fine if abs(a) <= limit), key=score)
    return _rotate(forward, best)


def _box_sums(occ, ulo, uhi, wlo, whi):
    """각 칸(iu, iw)에서 occ[iu+ulo:iu+uhi, iw+wlo:iw+whi] 합 (범위 밖은 0)."""
    pu = max(abs(ulo), abs(uhi)) + 1
    pw = max(abs(wlo), abs(whi)) + 1
    padded = np.pad(occ, ((pu, pu), (pw, pw)))
    integral = np.zeros((padded.shape[0] + 1, padded.shape[1] + 1))
    integral[1:, 1:] = padded.cumsum(0).cumsum(1)
    iu = np.arange(occ.shape[0])[:, None] + pu
    iw = np.arange(occ.shape[1])[None, :] + pw
    return (integral[iu + uhi, iw + whi] - integral[iu + ulo, iw + whi]
            - integral[iu + uhi, iw + wlo] + integral[iu + ulo, iw + wlo])


def find_u_slot(points, boat_xy, line_point, line_dir, params):
    """목표 방위선(line_point + s·line_dir) 가까이에서 고정 치수 ㄷ자 슬롯을 찾는다.

    반환 (USlot 또는 None, 이유).
    """
    p = params
    boat = np.asarray(boat_xy, float)
    line_point = np.asarray(line_point, float)
    forward0 = _unit(line_dir)
    pts = np.asarray(points, float).reshape(-1, 2)
    pts = pts[np.isfinite(pts).all(axis=1)]
    rel = pts - boat
    ranges = np.linalg.norm(rel, axis=1)
    ahead = rel @ forward0 > 0
    # 방위선에서 슬롯 하나 + 칸막이 + 여유보다 먼 점은 필요 없다 (계산량)
    side = np.abs((pts - line_point) @ np.array([-forward0[1], forward0[0]]))
    reach = p.target_offset + p.slot_width / 2 + p.tip_length + 2.0
    pts = pts[(ranges >= p.min_range) & (ranges <= p.max_range) & ahead & (side <= reach)]
    if len(pts) < p.min_points:
        return None, f'few points ahead ({len(pts)})'

    forward = dock_orientation(pts - boat, forward0, p.max_oblique_deg)
    left = np.array([-forward[1], forward[0]])
    u = (pts - boat) @ left
    w = (pts - boat) @ forward

    cell = p.cell
    hw = int(round(p.slot_width / 2 / cell))
    depth = int(round(p.depth / cell))
    tip = int(round(p.tip_length / cell))
    margin = max(1, int(round(.1 / cell)))
    inner = max(2, int(round(.2 / cell)))

    u0 = u.min() - (hw + tip + 2) * cell
    w0 = min(0.0, w.min()) - cell
    nu = int(math.ceil((u.max() + (hw + tip + 2) * cell - u0) / cell)) + 1
    nw = int(math.ceil((w.max() + cell - w0) / cell)) + 1
    occ = np.zeros((nu, nw))
    occ[((u - u0) / cell).astype(int), ((w - w0) / cell).astype(int)] = 1.0
    # ±1 칸 번짐 허용 (RTK 위치·자세 오차, 측정 잡음)
    grown = occ.copy()
    for du in (-1, 0, 1):
        for dw in (-1, 0, 1):
            grown = np.maximum(grown, np.roll(np.roll(occ, du, 0), dw, 1))
    occ = grown

    boxes = {   # (u_lo, u_hi, w_lo, w_hi) 칸, 중심 = (슬롯 중앙, 뒷벽)
        'back': (-hw + margin, hw - margin + 1, 0, 1),
        'left_face': (hw, hw + 1, -depth + margin, -margin + 1),
        'right_face': (-hw, -hw + 1, -depth + margin, -margin + 1),
        'left_tip': (hw, hw + tip + 1, -depth, -depth + 1),
        'right_tip': (-hw - tip, -hw + 1, -depth, -depth + 1),
        'interior': (-hw + inner, hw - inner + 1, -depth - 2 * inner, -inner + 1),
    }
    cov = {}
    for name, (a, b, c, d) in boxes.items():
        cov[name] = _box_sums(occ, a, b, c, d) / float((b - a) * (d - c))
    score = (cov['back'] + cov['left_face'] + cov['right_face']
             + .5 * (cov['left_tip'] + cov['right_tip']) - 4.0 * cov['interior'])

    # 가설 중심: 목표 방위선에서 target_offset 안, 배보다 min_wall_distance 이상 앞
    cu = u0 + np.arange(nu) * cell + cell / 2
    cw = w0 + np.arange(nw) * cell + cell / 2
    lp = line_point - boat
    lu, lw = lp @ left, lp @ forward
    du, dw = forward0 @ left, forward0 @ forward
    offset = np.abs((cu[:, None] - lu) * dw - (cw[None, :] - lw) * du)
    allowed = (offset <= p.target_offset) & (cw[None, :] >= p.min_wall_distance)
    if not allowed.any():
        return None, 'no slot position near the target bearing'
    masked = np.where(allowed, score, -np.inf)
    iu, iw = np.unravel_index(int(np.argmax(masked)), masked.shape)
    c = {name: float(v[iu, iw]) for name, v in cov.items()}

    left_ok = c['left_face'] >= p.min_face or c['left_tip'] >= p.min_tip
    right_ok = c['right_face'] >= p.min_face or c['right_tip'] >= p.min_tip
    if c['back'] < p.min_back:
        return None, f'back wall not seen ({c["back"]:.2f})'
    if max(c['left_face'], c['right_face']) < p.min_face or not (left_ok and right_ok):
        return None, (f'slot sides not seen (faces {c["left_face"]:.2f}/{c["right_face"]:.2f}, '
                      f'tips {c["left_tip"]:.2f}/{c["right_tip"]:.2f})')
    if c['interior'] > p.max_interior:
        return None, f'slot interior not empty ({c["interior"]:.2f})'

    # 격자 위치를 실제 면 점으로 다듬는다 (5 cm 격자 → 면 중앙값)
    uc, wb = float(cu[iu]), float(cw[iw])
    half = p.slot_width / 2
    band = (w > wb - p.depth + .15) & (w < wb - .15)
    lpts = u[band & (np.abs(u - (uc + half)) < .12)]
    rpts = u[band & (np.abs(u - (uc - half)) < .12)]
    if len(lpts) >= 5 and len(rpts) >= 5:
        uc = .5 * (float(np.median(lpts)) + float(np.median(rpts)))
    elif len(lpts) >= 5:
        uc = float(np.median(lpts)) - half
    elif len(rpts) >= 5:
        uc = float(np.median(rpts)) + half
    bpts = w[(np.abs(w - wb) < .12) & (np.abs(u - uc) < half - .15)]
    if len(bpts) >= 5:
        wb = float(np.median(bpts))

    center = boat + uc * left + wb * forward
    return USlot(center=tuple(map(float, center)), outward=tuple(map(float, -forward)),
                 coverage=c, score=float(masked[iu, iw]), wall_distance=wb), 'ok'
