"""slot_axis — 라이다 스캔으로 도킹 슬롯 뒷벽의 직선(=슬롯 축)과 입구 점을 구하는 순수 라이브러리.

ROS 비의존. 슬롯 축·입구는 **라이다만**으로 정한다(깊이 법선은 쓰지 않는다 —
작은 표식 평면에서는 잡음이 커서). YOLO+depth 는 어느 슬롯인지와 표식의 대략
위치(base_link)만 준다. 그 표식 주변 반경 window_radius 안의 라이다 점들에
직선을 맞추면 그것이 뒷벽이다. 슬롯 3개가 벽을 공유하며 나란히 있어 뒷벽은
이웃까지 이어진 직선이지만, 창 반경(1.3m)이 슬롯 반폭(1.5m)보다 작아 모서리의
칸막이 벽 점은 들어오지 않는다.

직선이 믿을 수 있는지는 점 개수·길이·잔차·인라이어 비율, 직선이 표식 근처를
지나는지(max_marker_offset), 벽이 선체 쪽을 향하는지(max_view_angle)로 확인한다.
하나라도 어긋나면 None 이며, 호출자는 대체 추정 없이 계획을 만들지 않는다.

좌표계는 base_link(전방 x, 좌측 y)이며 스캔 각도 규약은 occupancy_grid 와 같다
(점 = (r·cosθ, r·sinθ)). 라이다 원점이 base_link 와 일치한다고 가정한다.
"""
from dataclasses import dataclass
import math

import numpy as np


@dataclass(frozen=True)
class WallFitParams:
    window_radius: float = 1.3     # 표식 중심 반경 [m] (슬롯 반폭 1.5m 보다 작게)
    inlier_dist: float = 0.04      # 직선 인라이어 허용 거리 [m]
    min_points: int = 20
    min_span: float = 1.2          # 인라이어가 벽 방향으로 퍼진 길이 [m]
    max_rms: float = 0.03          # 정제된 직선의 잔차 RMS [m]
    min_inlier_fraction: float = 0.5   # 창 안 점 중 인라이어 비율 (흩어진 점 거부)
    max_marker_offset: float = 0.35    # 직선~표식 거리 한계 [m] (depth 거리 오차 허용)
    max_view_angle: float = math.radians(75.0)   # 시선과 벽 법선 사이 각 한계
    min_pair_dist: float = 0.5     # RANSAC 두 점 최소 간격 [m]
    iterations: int = 150


@dataclass(frozen=True)
class WallLine:
    outward: tuple          # 벽에서 선체 쪽으로 향하는 단위 법선 (x, y)
    center: tuple           # 직선 위 인라이어 중심 (x, y)
    points: int
    span: float
    rms: float
    marker_offset: float    # 표식이 직선에서 떨어진 거리 [m]


@dataclass(frozen=True)
class SlotGeometry:
    marker_xy: tuple        # 표식을 뒷벽 직선에 투영한 점
    mouth_xy: tuple         # 그 점에서 wall_to_mouth 만큼 법선 방향
    wall: WallLine


def scan_points(ranges, angle_min: float, angle_increment: float,
                range_min: float, range_max: float) -> np.ndarray:
    """LaserScan 필드 → base_link 점 (N, 2). 무효 빔은 버린다."""
    r = np.asarray(ranges, dtype=np.float64)
    angles = angle_min + angle_increment * np.arange(r.shape[0])
    valid = np.isfinite(r) & (r > range_min) & (r < range_max)
    r, a = r[valid], angles[valid]
    return np.column_stack((r * np.cos(a), r * np.sin(a)))


def _refine(points: np.ndarray):
    """점들의 주성분 직선. (중심, 방향, 법선)."""
    center = points.mean(axis=0)
    _, _, vt = np.linalg.svd(points - center, full_matrices=False)
    direction = vt[0]
    return center, direction, np.array([-direction[1], direction[0]])


def fit_back_wall(points, marker_xy,
                  p: WallFitParams | None = None) -> WallLine | None:
    """표식 주변 뒷벽의 직선을 맞춘다. 믿을 수 없으면 None."""
    p = p or WallFitParams()
    points = np.asarray(points, dtype=np.float64).reshape(-1, 2)
    marker = np.asarray(marker_xy, dtype=np.float64)
    if points.shape[0] < p.min_points or not np.all(np.isfinite(marker)):
        return None
    cand = points[np.hypot(*(points - marker).T) <= p.window_radius]
    if cand.shape[0] < p.min_points:
        return None

    # RANSAC — 고정 시드라 같은 입력은 항상 같은 결과를 낸다.
    rng = np.random.default_rng(0)
    best_inliers, best_count = None, 0
    for _ in range(p.iterations):
        i, j = rng.integers(0, cand.shape[0], size=2)
        a, b = cand[i], cand[j]
        d = b - a
        length = float(np.hypot(d[0], d[1]))
        if length < p.min_pair_dist:
            continue
        normal = np.array([-d[1], d[0]]) / length
        inliers = np.abs((cand - a) @ normal) <= p.inlier_dist
        count = int(inliers.sum())
        if count > best_count:
            best_count, best_inliers = count, inliers
    if best_inliers is None or best_count < p.min_points:
        return None

    pts = cand[best_inliers]
    for _ in range(2):      # 주성분 직선으로 정제 후 인라이어 재선정
        center, direction, normal = _refine(pts)
        keep = np.abs((cand - center) @ normal) <= p.inlier_dist
        if keep.sum() < p.min_points:
            return None
        pts = cand[keep]
    center, direction, normal = _refine(pts)

    rms = float(np.sqrt(np.mean(((pts - center) @ normal) ** 2)))
    proj = (pts - center) @ direction
    span = float(proj.max() - proj.min())
    marker_offset = abs(float((marker - center) @ normal))
    if (span < p.min_span or rms > p.max_rms
            or pts.shape[0] < p.min_inlier_fraction * cand.shape[0]
            or marker_offset > p.max_marker_offset):
        return None

    if float(normal @ (-center)) < 0:      # 법선이 선체(원점) 쪽을 향하게
        normal = -normal
    # 벽 앞면이 선체를 향해야 한다 — 시선과 법선의 각이 너무 크면 벽 뒷면/옆면이다.
    distance = float(np.hypot(center[0], center[1]))
    if distance > 1e-6:
        cos_view = float(normal @ (-center)) / distance
        if cos_view < math.cos(p.max_view_angle):
            return None
    return WallLine((float(normal[0]), float(normal[1])),
                    (float(center[0]), float(center[1])),
                    int(pts.shape[0]), span, rms, marker_offset)


def locate_slot(points, marker_xy, wall_to_mouth: float,
                p: WallFitParams | None = None) -> SlotGeometry | None:
    """뒷벽 직선에서 슬롯 축과 입구 점을 계산한다.

    표식을 직선에 투영한 점이 벽 위의 기준점이고, 입구는 거기서 벽 법선
    방향으로 wall_to_mouth 만큼 떨어진 점이다. 직선이 없으면 None.
    """
    if not math.isfinite(wall_to_mouth) or wall_to_mouth <= 0.0:
        return None
    wall = fit_back_wall(points, marker_xy, p)
    if wall is None:
        return None
    marker = np.asarray(marker_xy, dtype=np.float64)
    center = np.asarray(wall.center)
    normal = np.asarray(wall.outward)
    base = marker - float((marker - center) @ normal) * normal
    mouth = base + wall_to_mouth * normal
    return SlotGeometry((float(base[0]), float(base[1])),
                        (float(mouth[0]), float(mouth[1])), wall)
