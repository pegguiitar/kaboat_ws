"""buoy_localize — 부표 위치 + 색 판정 빨강/초록/모름 (ROS 무의존 순수 모듈).

색 판정 (2026-11 경기규정): 탐색은 사전에 주어진 색의 부표를 돌며 빨강·초록은 우현,
  검정(노랑에서 바뀜)은 좌현 선회한다. 검정은 색으로 판정하지 않는다 — 밝기만으로는
  그늘진 빨강·초록, 역광 실루엣, 어두운 물과 갈리지 않고, D455 뎁스도 무광 검정에서 빈다.
  그래서 빨강·초록만 색으로 판정하고, 부표 크기 물체인데 빨강도 초록도 아니면 'unknown'
  으로 낸다 (소거법). 검정 목표는 탐색 behavior 가 unknown 부표에서 고른다.
  주황은 빨강과 구분이 어려워 따로 두지 않고 빨강 범위에 넣는다 (범위는 전체적으로 넉넉히).
  픽셀마다: 채도 있으면(S ≥ s_min) 색상(H)으로 빨강(주황 포함)·초록(연두 포함), 그 밖
  (노랑·파랑·자홍 등)은 기타. 어두운 픽셀(V ≤ v_dark_max)은 색 없이 물체 픽셀(기타)로만
  넣는다 — 검정 부표도 뎁스 경로에서 덩어리로 잡히게. 채도 낮은 밝은 픽셀(흰·회색)은 물체
  픽셀이 아니다. 물·하늘을 색으로 거르지는 않는다 — 뎁스 경로는 수면 높이(z)와 6 m clamp 가,
  라이다 경로는 라이다 군집(물을 못 봄)이 물체를 정한다.

두 경로:
  뎁스 (depth_max=6 m 이내) — camera_obstacles 와 같은 방식으로 뎁스가 물체를 딴다.
    1) 물체 픽셀을 정렬 뎁스로 3D 로 바꿔 수평 기준(수면 위 높이)으로 돌린다
       (장착 위치·숙임 + IMU 롤·피치, camera_obstacles.pixels_to_level).
    2) 수면 높이 이하(물 자체·물에 비친 반사상)·너무 높은 것(구조물)·6 m 너머(하늘·먼
       물가 포함)를 버린다 — 물·하늘은 여기서 빠지므로 색으로 거르지 않는다.
    3) 남은 점을 수평 격자 연결요소로 묶으면 한 덩어리 = 물체 1개 → 색 다수결
       (빨강·초록 중 1위가 min_color_frac 미만이면 unknown).
    4) 근처에 라이다 군집이 있으면 위치를 분산 가중평균으로 융합 (라이다가 훨씬 정확).
  라이다 (뎁스로 안 잡힌 것 — 6 m 너머, 또는 뎁스가 빈 경우) — 부표 크기 라이다 군집을
    영상에 투영해 그 자리(수면 ~ far_z_max 높이)의 픽셀로 색을 판정한다. 뎁스가 없어 창 뒤의
    물을 z 로 못 거르므로 빨강·초록 픽셀끼리만 투표한다 (파란 물은 기타라 빠진다).
    그런 픽셀이 far_min_pixels 보다 적으면(검정·흰·회색·노랑) unknown. 라이다는 물을 못
    보니 물결 오검출이 없다.

중심: 센서는 앞면만 본다. 반지름은 고정값이 아니라 매 관측에서 덩어리의 시선 수직
  폭 / 2 로 추정하고(max_radius 로 자름 — 넘어진 부표의 넓은 실루엣 대비), 앞면 점
  평균에서 시선 방향으로 (π/4)·r̂ 만큼 민다. 원 단면을 가로로 고르게 본 점들의 평균은
  중심보다 (π/4)·r 앞에 있기 때문이다 (∫√(1−x²)dx 평균).

좌표: 선체 기준 수평 프레임 — 원점 = base_link(선체 중심), x 앞, y 왼쪽.
라이다 점은 /scan/level(배가 수평일 때만)이라 회전 없이 옮긴다.
"""
import math
from dataclasses import dataclass
from typing import List

import cv2
import numpy as np

from kaboat_perception.camera_obstacles import (
    CameraObstacleParams, level_rotation, pixels_to_level)


@dataclass
class BuoyGeometry:
    # 카메라 장착 — competition 의 camera_mount.py 실측값 (2026-11-12 오후, 다시 수평 장착).
    # competition 에 merge 하면 camera_mount.CAMERA_* 를 import 해서 쓰도록 바꿀 것.
    cam_offset_x: float = -0.215      # 선체 중심 기준 (라이다 0.4 − 0.615)
    cam_offset_y: float = 0.0
    cam_height: float = 0.775         # 흘수선에서 렌즈까지 [m]
    cam_mount_pitch_deg: float = 0.0  # + = 아래 (수평)
    # 선체 라이다 — occupancy_grid·gate_follower 의 lidar_offset_x/y 와 같은 값.
    lidar_offset_x: float = 0.4
    lidar_offset_y: float = 0.0
    # 색 (OpenCV H 0~180 — 빨강 0, 주황 15, 노랑 30, 초록 60, 청록 90, 파랑 120, 자홍 150)
    # 조명·햇빛에 색이 틀어져도 놓치지 않게 넉넉히 잡았다 — 주황은 빨강, 연두는 초록으로
    # 들어온다. 초록 윗경계(89)만은 청록 앞에서 끊는다: 라이다 경로 색 창의 물 배경은 z 로
    # 못 거르니, 더 넓히면 청록빛 물이 초록 표가 된다. 현장 캘리브 필요 (셔터·AWB 고정 후).
    step: int = 2                     # 픽셀 건너뛰기 (640×480 → 320×240)
    v_dark_max: int = 70              # V ≤ 이 값 → 어두운 물체 픽셀 (색 없음 — 검정 부표 덩어리용)
    s_min: int = 60                   # 이보다 채도 낮은 밝은 픽셀(흰·회색)은 배경
    red_h_max: int = 25               # 빨강: H ≤ red_h_max 또는 H ≥ red_h_min (주황 포함)
    red_h_min: int = 155
    green_h_min: int = 30             # 초록: green_h_min ≤ H ≤ green_h_max (연두 포함)
    green_h_max: int = 89
    min_color_frac: float = 0.4       # 빨강·초록 1위가 물체 픽셀의 이 비율 미만이면 unknown
    # 뎁스 물체 (camera_obstacles 와 같은 기준)
    depth_min: float = 0.6            # D455 유효 거리 [m]
    depth_max: float = 6.0            # 이보다 먼 뎁스는 안 씀
    z_min: float = 0.02               # 수면 위 높이 하한 — 반사상(음수) 제거
    z_max: float = 1.5                # 이보다 높은 것(하늘·건물·부두 구조물)은 버림
    cluster_cell: float = 0.1         # 수평 격자 칸 [m] — 이웃 칸끼리 한 물체
    min_points: int = 15              # 한 물체 최소 점 수
    # 위치·융합
    max_radius: float = 0.5           # 추정 반지름 상한 [m]
    max_cluster_width: float = 1.5    # 이보다 넓은 덩어리·군집은 부표 아님 (벽·물가·배) [m]
    lidar_sigma: float = 0.05         # 라이다 중심 1σ [m] (반지름 추정 오차 포함)
    depth_sigma_k: float = 0.005      # 뎁스 중심 1σ = k·R² [m] (4 m ≈ 8 cm, 6 m ≈ 18 cm)
    assoc_radius: float = 0.6         # 뎁스 물체 ↔ 라이다 군집 같은 물체로 볼 최대 거리 [m]
    cluster_gap: float = 0.3          # 이웃 라이다 점 간격이 이보다 크면 다른 물체 [m]
    lidar_min_points: int = 2         # 라이다 군집 최소 빔 수 (1빔은 물보라·노이즈)
    # 라이다 경로 색 창 윗변 (수면 위 높이) [m] — 카메라 높이보다 낮아야 창 뒤 배경이 물이 된다
    far_z_max: float = 0.5
    far_min_pixels: int = 6           # 라이다 경로 색 창 안 빨강·초록 픽셀이 이보다 적으면 unknown (step 후)
    occlusion_margin: float = 0.5     # 색 창에서 뎁스가 군집보다 이만큼 가까운 픽셀 = 앞을 가린 물체 [m]

    def camera(self) -> CameraObstacleParams:
        return CameraObstacleParams(
            offset_x=self.cam_offset_x, offset_y=self.cam_offset_y,
            height=self.cam_height, mount_pitch_deg=self.cam_mount_pitch_deg)


@dataclass
class BuoyFix:
    """부표 1개 위치 (선체 수평 프레임)."""
    color: str          # 'red' | 'green' | 'unknown'
    x: float
    y: float
    radius: float       # 추정 반지름 [m]
    source: str         # 'fused' | 'depth' | 'lidar'
    points: int         # 뒷받침한 점 수 (뎁스 픽셀 또는 라이다 빔)


def effective_K(k, width, height, hfov):
    """camera_info K (fx, fy, cx, cy) 를 믿을 수 있을 때만 쓰고, 아니면 HFOV 로 만든다.

    이 sim 의 gz rgbd camera_info 는 width=1280 이라 하면서 K 는 320px 기준
    (cx≈160)으로 어긋나 있다 — 그대로 쓰면 모든 방위가 ~60° 쏠린다. 주점이
    이미지 중심 근처일 때만(실물 D455 는 정상) K 를 쓰고, 아니면 HFOV·실제
    이미지 크기로 초점거리를 유도한다(자기 일관적, 정사각 픽셀 가정).
    """
    if k is not None:
        fx, fy, cx, cy = k
        if (fx > 0.0 and fy > 0.0 and abs(cx - width / 2.0) < 0.1 * width
                and abs(cy - height / 2.0) < 0.1 * height):
            return tuple(k)
    f = (width / 2.0) / math.tan(hfov / 2.0)
    return (f, f, width / 2.0, height / 2.0)


def camera_xy(K, g: BuoyGeometry, roll=0.0, pitch=0.0):
    """카메라 위치 (x, y) — 수평 프레임 (depth 0 점 = 카메라 원점)."""
    return pixels_to_level([K[2]], [K[3]], [0.0], K, g.camera(), roll, pitch)[0, :2]


def surface_center(pts, origin, g: BuoyGeometry, robust=False, pad=0.0):
    """앞면 점들 (N×2) → (중심 x·y, 추정 반지름). origin = 그 점들을 본 센서 위치.

    반지름 = (시선 수직 폭 + pad) / 2 (robust 면 5~95 백분위 폭 — 뎁스 가장자리 튐 대비),
    중심 = 점 평균 + (π/4)·r̂ · 시선 방향. pad 는 표본 간격 보정 — 실제 가장자리는
    양 끝 표본에서 반 칸씩 바깥이라 라이다는 빔 한 칸(거리·각분해능)을 더한다.
    """
    mean = pts.mean(axis=0)
    los = mean - origin
    los = los / max(float(np.hypot(*los)), 1e-6)
    lateral = (pts - mean) @ np.array([-los[1], los[0]])
    if robust:
        lo, hi = np.percentile(lateral, [5.0, 95.0])
    else:
        lo, hi = lateral.min(), lateral.max()
    r = min(0.5 * (float(hi - lo) + pad), g.max_radius)
    return mean + (math.pi / 4.0) * r * los, r


COLORS = ('other', 'red', 'green')   # classify_pixels 라벨 번호 순
UNKNOWN = 'unknown'                   # 부표 크기 물체인데 빨강·초록이 아님 (검정 부표 포함)


def classify_pixels(bgr, g: BuoyGeometry):
    """BGR 영상 → step 간격 (물체 픽셀 마스크, 색 라벨 uint8 — COLORS 번호)."""
    s = max(1, int(g.step))
    hsv = cv2.cvtColor(np.ascontiguousarray(bgr[::s, ::s]), cv2.COLOR_BGR2HSV)
    h, sat, val = hsv[..., 0], hsv[..., 1], hsv[..., 2]
    dark = val <= g.v_dark_max
    colored = ~dark & (sat >= g.s_min)
    label = np.zeros(h.shape, dtype=np.uint8)                       # 기타 (어두운 픽셀 포함)
    label[colored & ((h <= g.red_h_max) | (h >= g.red_h_min))] = 1  # 빨강
    label[colored & (h >= g.green_h_min) & (h <= g.green_h_max)] = 2  # 초록
    return dark | colored, label


def _vote(labels, g: BuoyGeometry):
    """물체 픽셀 라벨들 → 'red' | 'green', 빨강·초록 1위가 min_color_frac 미만이면 unknown."""
    counts = np.bincount(labels, minlength=len(COLORS))
    best = int(np.argmax(counts[1:])) + 1
    if counts[best] == 0 or counts[best] < g.min_color_frac * labels.size:
        return UNKNOWN
    return COLORS[best]


def depth_buoys(obj, label, depth_m, K, g: BuoyGeometry, roll=0.0,
                pitch=0.0) -> List[BuoyFix]:
    """뎁스로 물체를 따고 색을 판정 — 6 m 이내 부표 목록 (source='depth').

    obj·label 은 classify_pixels() 결과 (step 간격), depth_m 은 원본 해상도 정렬 뎁스.
    """
    s = max(1, int(g.step))
    dep = depth_m[::s, ::s]
    if dep.shape != obj.shape:
        raise ValueError(f'color {obj.shape} and aligned depth {dep.shape} differ (step {s})')
    vs, us = np.nonzero(obj & np.isfinite(dep) & (dep > 0.0))
    if vs.size == 0:
        return []
    pts = pixels_to_level(us * s, vs * s, dep[vs, us], K, g.camera(), roll, pitch)
    cam = camera_xy(K, g, roll, pitch)
    horiz = np.hypot(pts[:, 0] - cam[0], pts[:, 1] - cam[1])
    keep = ((horiz >= g.depth_min) & (horiz <= g.depth_max)
            & (pts[:, 2] >= g.z_min) & (pts[:, 2] <= g.z_max))
    if keep.sum() < g.min_points:
        return []
    xy = pts[keep, :2]
    pt_label = label[vs[keep], us[keep]]

    # 수평 격자 연결요소 — 이웃(8방향) 칸끼리 한 물체
    ij = np.floor(xy / g.cluster_cell).astype(np.int64)
    ij -= ij.min(axis=0)
    occ = np.zeros(tuple(ij.max(axis=0) + 1), dtype=np.uint8)
    occ[ij[:, 0], ij[:, 1]] = 1
    _n, labels = cv2.connectedComponents(occ, connectivity=8)
    point_label = labels[ij[:, 0], ij[:, 1]]

    out = []
    for lab in np.unique(point_label):
        idx = point_label == lab
        n = int(idx.sum())
        if n < g.min_points:
            continue
        cl = xy[idx]
        if np.ptp(cl, axis=0).max() > g.max_cluster_width:
            continue
        color = _vote(pt_label[idx], g)               # 빨강·초록 아니면 unknown
        (cx, cy), r = surface_center(cl, cam, g, robust=True)
        out.append(BuoyFix(color, float(cx), float(cy), r, 'depth', n))
    return out


def lidar_points(ranges, angle_min, angle_inc, range_min, range_max, g: BuoyGeometry):
    """LaserScan → 수평 프레임 점 N×2 (빔 순서 유지)."""
    r = np.asarray(ranges, dtype=float)
    a = angle_min + angle_inc * np.arange(len(r))
    ok = np.isfinite(r) & (r > range_min) & (r < range_max)
    return np.column_stack((g.lidar_offset_x + r[ok] * np.cos(a[ok]),
                            g.lidar_offset_y + r[ok] * np.sin(a[ok])))


def lidar_clusters(points, g: BuoyGeometry, angle_inc=0.0):
    """라이다 점 → 부표 크기 군집들의 [(중심 x·y, 반지름, 빔 수)] (넓은 벽·물가 제외).

    angle_inc = 스캔 각분해능 [rad] — 먼 부표는 빔 몇 개뿐이라 폭을 빔 한 칸만큼 보정.
    """
    if points is None or len(points) == 0:
        return []
    origin = np.array([g.lidar_offset_x, g.lidar_offset_y])
    gaps = np.hypot(*np.diff(points, axis=0).T) > g.cluster_gap
    out = []
    for cl in np.split(points, np.nonzero(gaps)[0] + 1):
        if len(cl) < g.lidar_min_points or np.hypot(*(cl[-1] - cl[0])) > g.max_cluster_width:
            continue
        pad = float(np.hypot(*(cl.mean(axis=0) - origin))) * abs(angle_inc)
        center, r = surface_center(cl, origin, g, pad=pad)
        out.append((center, r, len(cl)))
    return out


def fuse_with_lidar(fix: BuoyFix, clusters, cam, g: BuoyGeometry) -> BuoyFix:
    """뎁스 부표 근처(assoc_radius)에 라이다 군집이 있으면 분산 가중평균으로 융합."""
    if not clusters:
        return fix
    p = np.array([fix.x, fix.y])
    center, r, _n = min(clusters, key=lambda c: np.hypot(*(c[0] - p)))
    if np.hypot(*(center - p)) > g.assoc_radius:
        return fix
    rng = float(np.hypot(*(p - cam)))
    wd = 1.0 / (g.depth_sigma_k * rng * rng) ** 2
    wl = 1.0 / g.lidar_sigma ** 2
    q = (wd * p + wl * center) / (wd + wl)
    return BuoyFix(fix.color, float(q[0]), float(q[1]),
                   (wd * fix.radius + wl * r) / (wd + wl), 'fused', fix.points)


def level_to_pixel(pts, K, g: BuoyGeometry, roll=0.0, pitch=0.0):
    """수평 프레임 점 N×3 → (u, v, 광학 깊이) — pixels_to_level 의 역."""
    a = math.radians(g.cam_mount_pitch_deg)
    mount = np.array([[math.cos(a), 0.0, math.sin(a)], [0.0, 1.0, 0.0],
                      [-math.sin(a), 0.0, math.cos(a)]])
    level = level_rotation(roll, pitch)
    t = level @ np.array([g.cam_offset_x, g.cam_offset_y, g.cam_height])
    body = (np.asarray(pts, dtype=float) - t) @ (level @ mount)   # 앞, 왼, 위
    fx, fy, cx, cy = K
    return cx - fx * body[:, 1] / body[:, 0], cy - fy * body[:, 2] / body[:, 0], body[:, 0]


def lidar_buoys(clusters, obj, label, K, g: BuoyGeometry, roll=0.0, pitch=0.0,
                skip=(), depth_m=None) -> List[BuoyFix]:
    """라이다 군집 → 영상 색 창으로 색 판정한 부표 목록 (source='lidar').

    창 = 군집 중심 좌우 ±r̂, 수면(z_min) ~ far_z_max 높이를 영상에 투영한 사각형.
    skip 의 부표(뎁스로 이미 잡은 것)와 assoc_radius 안의 군집은 건너뛴다.
    depth_m(정렬 뎁스)이 있으면 창에서 군집보다 occlusion_margin 넘게 가까운 픽셀 —
    앞을 가린 다른 물체 — 은 색 투표에서 뺀다 (뒤 부표가 앞 부표 색으로 잡히지 않게).
    """
    s = max(1, int(g.step))
    dep = None if depth_m is None else depth_m[::s, ::s]
    rows, cols = obj.shape
    cam = camera_xy(K, g, roll, pitch)
    out = []
    for center, r, n in clusters:
        if any(math.hypot(center[0] - f.x, center[1] - f.y) <= g.assoc_radius for f in skip):
            continue
        los = (center - cam) / max(float(np.hypot(*(center - cam))), 1e-6)
        side = np.array([-los[1], los[0]]) * max(r, 0.05)
        corners = np.array([[*(center + k * side), z]
                            for k in (-1.0, 1.0) for z in (g.z_min, g.far_z_max)])
        u, v, d = level_to_pixel(corners, K, g, roll, pitch)
        if np.any(d <= 0.0):
            continue                                   # 카메라 뒤
        c0, c1 = int(max(u.min() / s, 0)), int(min(math.ceil(u.max() / s) + 1, cols))
        r0, r1 = int(max(v.min() / s, 0)), int(min(math.ceil(v.max() / s) + 1, rows))
        if c0 >= c1 or r0 >= r1:
            continue                                   # 시야 밖
        win = obj[r0:r1, c0:c1]
        if dep is not None:
            near = dep[r0:r1, c0:c1]
            win = win & ~(np.isfinite(near) & (near > 0.0)
                          & (near < d.min() - g.occlusion_margin))
        voters = label[r0:r1, c0:c1][win]
        voters = voters[voters > 0]                    # 빨강·초록만 — 창 뒤 물(기타) 제외
        # 빨강·초록 픽셀이 없으면(검정·흰·회색·노랑) unknown — 라이다 군집이라 물체는 맞다
        color = _vote(voters, g) if voters.size >= g.far_min_pixels else UNKNOWN
        out.append(BuoyFix(color, float(center[0]), float(center[1]), r, 'lidar', n))
    return out
