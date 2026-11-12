"""buoy_color — 라이다 부표 후보에 카메라 RGB 로 색만 붙인다 (ROS 무의존 순수 모듈).

라이다 중심 부표 파이프라인(buoy_lidar_detector)의 색 단계. 부표가 있는지·어디 있는지는
라이다가 정하고, 여기서는 후보 자리를 영상에 투영한 창의 픽셀로 색 표 하나를 낸다.
뎁스는 쓰지 않는다 — 윤슬·반사에서 뎁스가 크게 튀어서 (2026-11-12 현장).

표 (ColorVote.color):
  'red' / 'green'  창 안 빨강·초록 픽셀이 min_color_px 이상이고 한쪽이 dominance 배 이상
  'unknown'        창이 충분히 보이는데(min_visible_px) 빨강·초록 픽셀이 모자람 — 검정 부표 등
  (트래커는 이 표보다 ColorVote.red/green/visible 픽셀 수를 부표마다 누적해 색을 정한다 — usable 일 때만)
  None             판정 불가 — 화면 밖(일부 포함), 카메라 뒤, 더 가까운 후보에 대부분 가림,
                   보이는 픽셀이 적음, 창 30 % 넘게 보라 번짐(D455 가장자리),
                   빨강·초록이 섞임. **표를 넣지 않는다.** 가려져서 잠깐씩만 보이는 초록
                   부표가 가려진 동안 unknown 표에 묻히지 않게 (2026-11-12 영상: 검정 뒤 초록이
                   1초 정도만 보임).

픽셀 색 (OpenCV H 0~180) — 2026-11-12 해 질 녘 bag 에서 부표 픽셀을 직접 재서 정했다:
  빨강  H ≤ 10 또는 ≥ 170, S ≥ 110, V ≥ 80.
        · 해 질 녘 초록 부표 옆면의 햇빛 받은 황갈색 띠(H 12~22, S≈75, V≈106)는 H 로 거른다.
        · 낮 gate 구간 빨강 부표는 그늘진 면이 어둡다 (빨강 픽셀 V 중앙값 111) — 예전 V ≥ 150 은
          창의 18 % 만 빨강으로 세서 빨강 부표가 unknown 으로 남았다. V ≥ 80 이면 52 %.
          그 빨강 픽셀은 H 170~180·0~5 에 있고 10~25 에는 없다 (2026-11-12 bag 실측).
        · H ≥ 170 은 D455 화면 오른쪽 아래 보라 번짐(H 145~165)을 피한다.
        · 주황(H 10~20)은 빨강으로 안 들어간다 — 경기 부표는 빨강·초록·검정뿐.
  초록  45 ≤ H ≤ 89, S ≥ 60, V ≥ 20. 초록 부표는 뚜껑과 짙은 초록 몸통(H 46~65, V≈34)인데,
        부표 옆 올리브색 물(H 26~45)이 예전 하한 30 으로는 초록 표가 됐다.
  검정 부표(V≈40, 보라 번짐 H≈148)는 둘 다 아니라 unknown.
"""
import math
from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple

import cv2
import numpy as np

from kaboat_perception.buoy_localize import BuoyGeometry, level_to_pixel


@dataclass(frozen=True)
class ColorParams:
    step: int = 2                 # 픽셀 건너뛰기 (640×480 → 320×240)
    red_h_max: int = 10           # 빨강: H ≤ red_h_max 또는 H ≥ red_h_min — 황갈색 띠(H 12~22) 제외
    red_h_min: int = 170
    red_s_min: int = 110          # 빨강은 진하고
    red_v_min: int = 80           # 그늘진 빨강(V 80~150)도 빨강 — 낮 gate bag 빨강 부표 V 중앙값 111
    green_h_min: int = 45         # 초록: green_h_min ≤ H ≤ green_h_max — 올리브 물(H < 45) 제외
    green_h_max: int = 89
    green_s_min: int = 40         # 이보다 채도 낮으면(흰·회색·윤슬) 초록 아님 — 초록 공 부표 S 50~70 (bag 실측)
    green_v_min: int = 20         # 짙은 초록 몸통(V≈34)도 초록
    # D455 화면 가장자리 보라 번짐 — 창의 max_tint_frac 넘게 이 색이면 그 프레임은 색을 못 믿음 (표 없음)
    tint_h_min: int = 140
    tint_h_max: int = 169
    tint_s_min: int = 40
    max_tint_frac: float = 0.3
    window_top: float = 0.6       # 창 윗변 — 수면 위 높이 [m]
    min_half_width: float = 0.15  # 창 반폭 하한 [m] (라이다 반지름이 작게 나올 때)
    min_visible_px: int = 20      # 가림·화면 밖을 빼고 이만큼 보여야 판정 (step 후 픽셀)
    min_visible_frac: float = 0.6  # 창의 이 비율 이상이 보여야 판정 — 앞 부표에 대부분 가리면 표 없음
    min_color_px: int = 6         # 빨강·초록 판정 최소 픽셀 (step 후)
    dominance: float = 2.0        # 1위 색이 2위의 이 배 이상이어야 판정 (섞이면 None)


@dataclass
class ColorVote:
    color: Optional[str]          # 'red' | 'green' | 'unknown' | None(판정 불가)
    rect: Optional[Tuple[int, int, int, int]]  # 원본 해상도 창 (u0, v0, u1, v1), 화면 밖이면 None
    red: int = 0
    green: int = 0
    visible: int = 0
    usable: bool = False          # 픽셀 수를 누적해도 되는 프레임 (보임·가림·번짐 조건 통과)


def classify_colors(bgr, p: ColorParams):
    """BGR → step 간격 라벨 (0 기타, 1 빨강, 2 초록, 3 보라 번짐)."""
    s = max(1, int(p.step))
    hsv = cv2.cvtColor(np.ascontiguousarray(bgr[::s, ::s]), cv2.COLOR_BGR2HSV)
    h, sat, val = hsv[..., 0], hsv[..., 1], hsv[..., 2]
    label = np.zeros(h.shape, dtype=np.uint8)
    label[(sat >= p.red_s_min) & (val >= p.red_v_min)
          & ((h <= p.red_h_max) | (h >= p.red_h_min))] = 1
    label[(sat >= p.green_s_min) & (val >= p.green_v_min)
          & (h >= p.green_h_min) & (h <= p.green_h_max)] = 2
    label[(sat >= p.tint_s_min) & (h >= p.tint_h_min) & (h <= p.tint_h_max)] = 3   # 보라 번짐
    return label


def candidate_rect(center, radius, K, g: BuoyGeometry, p: ColorParams, roll=0.0, pitch=0.0):
    """선체 수평 프레임 후보 (x, y, 반지름) → 원본 해상도 창 (u0, v0, u1, v1, 광학 깊이) 또는 None."""
    c = np.asarray(center, dtype=float)
    cam = np.array([g.cam_offset_x, g.cam_offset_y])
    rng = float(np.hypot(*(c - cam)))
    if rng < 1e-6:
        return None
    los = (c - cam) / rng
    side = np.array([-los[1], los[0]]) * max(float(radius), p.min_half_width)
    corners = np.array([[*(c + k * side), z] for k in (-1.0, 1.0) for z in (0.0, p.window_top)])
    u, v, d = level_to_pixel(corners, K, g, roll, pitch)
    if np.any(d <= 0.0):
        return None
    return float(u.min()), float(v.min()), float(u.max()), float(v.max()), float(d.min())


def vote_colors(label, candidates: Sequence[Tuple[Tuple[float, float], float]], K,
                g: BuoyGeometry, p: ColorParams, roll=0.0, pitch=0.0) -> List[ColorVote]:
    """후보들 [((x, y), 반지름), ...] (선체 수평 프레임) → 후보마다 ColorVote.

    더 가까운 후보의 창과 겹치는 픽셀은 가림으로 보고 뺀다 (뎁스 없이 라이다로 가림 판정).
    """
    s = max(1, int(p.step))
    rows, cols = label.shape
    rects = [candidate_rect(c, r, K, g, p, roll, pitch) for c, r in candidates]
    out = []
    for i, rect in enumerate(rects):
        if rect is None:
            out.append(ColorVote(None, None))
            continue
        u0, v0, u1, v1, depth = rect
        c0, c1 = int(max(math.floor(u0 / s), 0)), int(min(math.ceil(u1 / s) + 1, cols))
        r0, r1 = int(max(math.floor(v0 / s), 0)), int(min(math.ceil(v1 / s) + 1, rows))
        box = (int(u0), int(v0), int(u1), int(v1))
        if c0 >= c1 or r0 >= r1:
            out.append(ColorVote(None, None))
            continue
        keep = np.ones((r1 - r0, c1 - c0), dtype=bool)
        for j, other in enumerate(rects):
            if j == i or other is None or other[4] >= depth:
                continue
            a0 = int(max(math.floor(other[0] / s), c0)) - c0
            a1 = int(min(math.ceil(other[2] / s) + 1, c1)) - c0
            b0 = int(max(math.floor(other[1] / s), r0)) - r0
            b1 = int(min(math.ceil(other[3] / s) + 1, r1)) - r0
            if a0 < a1 and b0 < b1:
                keep[b0:b1, a0:a1] = False
        win = label[r0:r1, c0:c1][keep]
        visible = int(win.size)
        total = max((math.ceil(u1 / s) - math.floor(u0 / s) + 1)
                    * (math.ceil(v1 / s) - math.floor(v0 / s) + 1), 1)
        red, green = int(np.sum(win == 1)), int(np.sum(win == 2))
        vote = ColorVote(None, box, red, green, visible)
        tint = int(np.sum(win == 3))
        if (visible < p.min_visible_px or visible < p.min_visible_frac * total
                or tint > p.max_tint_frac * visible):
            out.append(vote)
            continue
        vote.usable = True
        best, other_n = (('red', red), green) if red >= green else (('green', green), red)
        if best[1] >= p.min_color_px:
            if best[1] >= p.dominance * other_n:
                vote.color = best[0]
        elif red + green < p.min_color_px:
            vote.color = 'unknown'
        out.append(vote)
    return out
