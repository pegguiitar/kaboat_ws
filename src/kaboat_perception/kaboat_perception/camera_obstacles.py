"""컬러 + 정렬 뎁스로 수면 위 색 장애물을 찾는다 — occupancy_grid 의 카메라 층.

라이다 수평 한 면은 수면에 누운 낮은 주황 붐 위로 지나가 놓친다. 그래서
  1) 컬러 영상에서 장애물 색 픽셀만 고르고 (주황·빨강 기본, 흰색은 선택 — 수면
     반짝임·물거품·흰 구조물 오탐이 많고 흰 부표는 라이다가 이미 잡아서 기본 끔)
  2) 그 픽셀의 정렬 뎁스(/camera/depth/image_raw = aligned_depth_to_color)로 3D 위치를 재고
  3) 카메라 장착 위치·숙임 + IMU 롤·피치로 수평 기준(수면 위 높이)으로 돌려
  4) 수면보다 한참 아래(뎁스 오류)·너무 높은 것·너무 가깝거나 먼 것을 버린다.
색으로 "무엇"을, 뎁스로 "어디"를 정하므로 높이 기준은 느슨하게만 쓴다 — 주황은 수면 아래
0.3 m 까지 받는다(반사상은 붐 바로 아래 같은 x·y 라 지도에선 같은 칸, 장착 각도 오차 흡수).

좌표: 선체 기준 수평 프레임 — 원점 = base_link(선체 중심) 바로 아래 수면,
x 앞, y 왼쪽, z 수면 위 높이. 카메라 광학 프레임은 REP-103(x 오른쪽, y 아래, z 앞).
컬러 광학 중심과 camera_link 사이 몇 cm 차이는 무시한다.
"""

import math
from dataclasses import dataclass

import cv2
import numpy as np

from kaboat_perception.depth_utils import depth_to_meters


@dataclass
class CameraObstacleParams:
    # 카메라 장착 (선체 중심 기준). x·y 는 dock_yolo.launch.py 실측(라이다 +0.4 에서 -0.73).
    offset_x: float = -0.33
    offset_y: float = 0.0
    height: float = 0.775            # 흘수선에서 카메라 렌즈까지 [m] (2026-11-12 수평으로 다시 단 뒤 실측)
    mount_pitch_deg: float = 0.0     # 장착 숙임 (+ = 아래). 2026-11-12 10° 숙임 → 다시 수평
    step: int = 2                    # 픽셀 건너뛰기 (640×480 → 320×240)
    range_min: float = 0.6           # D455 유효 거리 [m] — 가까우면 배 자체, 멀면 부정확
    range_max: float = 5.0           # 멀수록 뎁스 오차가 거리² 로 커짐 (4 m ≈ 8 cm, 6 m ≈ 18 cm)
    z_max: float = 1.5               # 이보다 높은 것(하늘·건물·부두 구조물)은 버림
    # 주황·빨강 (OpenCV H 0~179 — 빨강은 0 근처와 179 근처로 나뉨)
    orange: bool = True
    orange_h_low_max: int = 20
    orange_h_high_min: int = 160
    # 채도·밝기 하한 120 → 90, 70 → 50 (2026-11-12 현장: 햇빛에 바랜 붐·그늘진 붐을 놓침)
    orange_s_min: int = 90
    orange_v_min: int = 50
    # 높이 하한 0.02 → -0.3 (2026-11-12 현장): 장착 각도가 1~2° 만 틀려도 3 m 의 붐(수면 위
    # 10 cm 남짓)이 수면 아래로 계산돼 통째로 버려졌다. 잔잔한 수면의 반사상은 붐 바로 아래
    # (같은 x·y)로 계산돼 지도에서 같은 칸에 쌓이므로 받아도 된다. -0.3 아래는 아래를 향한
    # 시선이 수면보다 깊이 간 것 = 뎁스 매칭 오류(실제보다 먼 가짜 점)라 버린다.
    orange_z_min: float = -0.3
    # 흰색 (선택)
    white: bool = False
    white_s_max: int = 40
    white_v_min: int = 190
    white_z_min: float = 0.10        # 수면 반짝임 때문에 더 높게


def color_image_rgb(msg) -> np.ndarray:
    """sensor_msgs/Image (rgb8·bgr8) → H×W×3 RGB uint8 (cv_bridge 없이)."""
    enc = msg.encoding.lower()
    if enc not in ('rgb8', 'bgr8'):
        raise ValueError(f'unsupported color encoding {msg.encoding!r}; expected rgb8/bgr8')
    rows = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.step)
    img = rows[:, :msg.width * 3].reshape(msg.height, msg.width, 3)
    return img if enc == 'rgb8' else img[:, :, ::-1]


def depth_image_m(msg) -> np.ndarray:
    """sensor_msgs/Image (16UC1 mm · 32FC1 m) → H×W float32 미터."""
    enc = msg.encoding.upper()
    if enc == '16UC1':
        dtype = np.dtype('>u2' if msg.is_bigendian else '<u2')
    elif enc == '32FC1':
        dtype = np.dtype('>f4' if msg.is_bigendian else '<f4')
    else:
        raise ValueError(f'unsupported depth encoding {msg.encoding!r}; expected 16UC1/32FC1')
    rows = np.frombuffer(msg.data, dtype).reshape(msg.height, msg.step // dtype.itemsize)
    return depth_to_meters(rows[:, :msg.width], enc)


def color_masks(rgb: np.ndarray, p: CameraObstacleParams) -> dict:
    """켜진 색 종류별 H×W bool 마스크."""
    hsv = cv2.cvtColor(np.ascontiguousarray(rgb), cv2.COLOR_RGB2HSV)
    h, s, v = hsv[..., 0], hsv[..., 1], hsv[..., 2]
    masks = {}
    if p.orange:
        masks['orange'] = (((h <= p.orange_h_low_max) | (h >= p.orange_h_high_min))
                           & (s >= p.orange_s_min) & (v >= p.orange_v_min))
    if p.white:
        masks['white'] = (s <= p.white_s_max) & (v >= p.white_v_min)
    return masks


def level_rotation(roll: float, pitch: float) -> np.ndarray:
    """선체 → 수평 기준 회전 R_y(pitch)·R_x(roll) (ROS 규약: pitch + = 선수 아래)."""
    cr, sr, cp, sp = math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch)
    rx = np.array([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]])
    ry = np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]])
    return ry @ rx


def pixels_to_level(u, v, depth, K, p: CameraObstacleParams, roll=0.0, pitch=0.0):
    """픽셀 (u, v) + 뎁스 [m] → 수평 프레임 점 N×3 (앞, 왼, 수면 위 높이)."""
    fx, fy, cx, cy = K
    d = np.asarray(depth, dtype=np.float64)
    xo = (np.asarray(u, dtype=np.float64) - cx) * d / fx
    yo = (np.asarray(v, dtype=np.float64) - cy) * d / fy
    body = np.stack([d, -xo, -yo], axis=1)               # 카메라 몸체: 앞, 왼, 위
    a = math.radians(p.mount_pitch_deg)
    mount = np.array([[math.cos(a), 0.0, math.sin(a)], [0.0, 1.0, 0.0],
                      [-math.sin(a), 0.0, math.cos(a)]])
    level = level_rotation(roll, pitch)
    rot = level @ mount
    t = level @ np.array([p.offset_x, p.offset_y, p.height])
    return body @ rot.T + t


def detect_points(rgb, depth_m, K, p: CameraObstacleParams, roll=0.0, pitch=0.0):
    """색 장애물 점 → (N×3 수평 프레임 점, 길이 N 색 이름 배열). rgb 와 depth 는 같은 크기."""
    if rgb.shape[:2] != depth_m.shape[:2]:
        raise ValueError(f'color {rgb.shape[:2]} and aligned depth {depth_m.shape[:2]} differ')
    s = max(1, int(p.step))
    rgb_s, dep = rgb[::s, ::s], depth_m[::s, ::s]
    good_depth = np.isfinite(dep) & (dep > 0.0)
    points, labels = [], []
    for name, mask in color_masks(rgb_s, p).items():
        vs, us = np.nonzero(mask & good_depth)
        if vs.size == 0:
            continue
        pts = pixels_to_level(us * s, vs * s, dep[vs, us], K, p, roll, pitch)
        horiz = np.hypot(pts[:, 0] - p.offset_x, pts[:, 1] - p.offset_y)
        z_min = p.orange_z_min if name == 'orange' else p.white_z_min
        keep = ((horiz >= p.range_min) & (horiz <= p.range_max)
                & (pts[:, 2] >= z_min) & (pts[:, 2] <= p.z_max))
        points.append(pts[keep])
        labels.append(np.full(int(keep.sum()), name))
    if not points:
        return np.empty((0, 3)), np.empty(0, dtype='<U6')
    return np.concatenate(points), np.concatenate(labels)


def visible_mask(cell_x, cell_y, boat_x, boat_y, yaw, K, width, p: CameraObstacleParams,
                 margin_deg=3.0):
    """월드 셀 중심 → 카메라가 지금 볼 수 있는 셀(수평 시야 안, range_min~range_max).

    이 칸들은 이번 프레임에 색 점이 없으면 '안 보임'이 아니라 '없음'으로 보고 감쇠한다.
    시야 밖 칸은 그대로 둔다 — 옆으로 지나간 붐을 잊지 않도록.
    """
    fx, cx = K[0], K[2]
    half = min(math.atan2(cx, fx), math.atan2(width - cx, fx)) - math.radians(margin_deg)
    c, s = math.cos(yaw), math.sin(yaw)
    dx, dy = cell_x - boat_x, cell_y - boat_y
    lx = c * dx + s * dy - p.offset_x
    ly = -s * dx + c * dy - p.offset_y
    dist = np.hypot(lx, ly)
    return (dist >= p.range_min) & (dist <= p.range_max) & (np.abs(np.arctan2(ly, lx)) <= half)
