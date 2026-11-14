"""buoy_color — 라이다 부표 후보에 카메라 RGB 로 색만 붙인다 (ROS 무의존 순수 모듈).

라이다 중심 부표 파이프라인(buoy_lidar_detector)의 색 단계. 부표가 있는지·어디 있는지는
라이다가 정하고, 여기서는 후보 자리를 영상에 투영한 창의 픽셀로 색 표 하나를 낸다.
뎁스는 창 픽셀을 고르는 데만 쓴다 (refine_with_depth) — 창 픽셀마다 뎁스로 수평 좌표를 구해 그 부표 반경
안인 픽셀만 센다. 뎁스가 없거나 고른 픽셀이 적으면 창 전체 (vote_colors 결과 그대로).

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
  빨강  H ≤ 10 또는 ≥ 170, S ≥ 80, V ≥ 60 (2026-11-14 튜닝 — 아래 마지막 항목).
        · 해 질 녘 초록 부표 옆면의 햇빛 받은 황갈색 띠(H 12~22, S≈75, V≈106)는 H 로 거른다.
        · 낮 gate 구간 빨강 부표는 그늘진 면이 어둡다 (빨강 픽셀 V 중앙값 111) — 예전 V ≥ 150 은
          창의 18 % 만 빨강으로 세서 빨강 부표가 unknown 으로 남았다. V ≥ 80 이면 52 %.
          그 빨강 픽셀은 H 170~180·0~5 에 있고 10~25 에는 없다 (2026-11-12 bag 실측).
        · H ≥ 170 은 D455 화면 오른쪽 아래 보라 번짐(H 145~165)을 피한다.
        · 주황(H 10~20)은 빨강으로 안 들어간다 — 경기 부표는 빨강·초록·검정뿐.
  초록  64 ≤ H ≤ 89, S ≥ 40, V ≥ 50 (2026-11-14 튜닝 — 아래 마지막 항목). 부표 옆 올리브색 물(H 26~45)이 예전 하한 30 으로는 초록 표가 됐다.
        · 해 질 녘 검정 캡슐 몸통의 어두운 픽셀이 H 45~66(중앙 51)·V 26~32 라 하한 45 로는 초록이 돼,
          붙은 쌍의 검정이 초반 29 % 스캔 동안 초록이었다. 초록 공 부표 초록 픽셀은 H 60~79(중앙 73).
          하한 55 로 올리면 검정 오판 0 %, gate 초록 부표 초록 픽셀 비율은 86→82 % 정도로만 준다.
        · **어두운 픽셀(V < 50)은 H ≥ 68 일 때만 초록** (green_dark_v·green_dark_h_min, 2026-11-14).
          흐린 아침(비) 실주행 run1_20261114: 검정 캡슐 몸통이 H 56~71(중앙 64)·V 23~48(중앙 35)로 초록에
          들어가 누적 초록 비율 0.17~0.23 > 0.15 → 검출기가 검정을 green 으로 내 탐색(검정 목표) 실패.
          같은 날 초록 부표 초록 픽셀은 H 71~86(중앙 77~80)·V 중앙 59~72 — 이 규칙으로 검정 0.23 → 0.07
          (unknown), 초록 탐색 부표 2 개·gate 초록 7 개는 초록 픽셀 95~100 % 유지 (0.16~0.89 그대로),
          11-13 탐색 초록 100 %, 11-12 해 질 녘 초록 86 % 유지.
  검정 부표(V≈40, 보라 번짐 H≈148)는 둘 다 아니라 unknown.

2026-11-14 3 bag 튜닝 (11-12 오후·11-13 오전·11-14 흐린 아침, gate·탐색 6 구간 123 프레임, 부표 23 개 뎁스 창):
  물·다리가 초록으로 너무 많이, 빨강은 덜 잡혔다. 초록으로 잡힌 물·배경 픽셀은 H 63~87·S 중앙 75·V 중앙 49,
  초록 부표 픽셀은 H 67~79·S 중앙 155·V 중앙 81 — 흐린 날 초록 부표는 S 44~112 라 S 로는 못 가르고 V 로 가른다.
  빨강 부표에서 놓친 픽셀은 H 는 맞고 S 27~69.
                 지금 → 바뀐 값: 물·배경 초록 7.8 → 3.25 %, 검정 부표 최대 초록 비율 0.072 → 0.008,
                 11-14 탐색 초록 0.27 → 0.22 (기준 0.15 위), 부표 판정 20/23 → 20/23 (못 맞히는 3 개는 11-13
                 오전 gate 의 짙은 초록 — 물 색과 겹침). 초록 V ≥ 50 이라 위 어두운 픽셀 규칙(green_dark_v 50)은
                 지금 값에서는 쓰이지 않는다 (green_v_min 을 낮추면 다시 쓰임).
"""
import math
from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple

import cv2
import numpy as np

from kaboat_perception.buoy_localize import BuoyGeometry, level_to_pixel
from kaboat_perception.camera_obstacles import CameraObstacleParams, pixels_to_level


@dataclass(frozen=True)
class ColorParams:
    step: int = 2                 # 픽셀 건너뛰기 (640×480 → 320×240)
    red_h_max: int = 10           # 빨강: H ≤ red_h_max 또는 H ≥ red_h_min — 황갈색 띠(H 12~22) 제외
    red_h_min: int = 170
    red_s_min: int = 80           # 빨강 채도 하한 (110 → 80, 2026-11-14 3 bag 튜닝) — 흐린 날·오전 자주빛 빨강의 놓친 픽셀이 S 27~69
    red_v_min: int = 60           # 그늘진 빨강도 빨강 (80 → 60, 같은 튜닝)
    green_h_min: int = 64         # 초록: green_h_min ≤ H ≤ green_h_max (55 → 64, 2026-11-14 3 bag 튜닝)
    green_h_max: int = 89
    green_s_min: int = 40         # 채도는 그대로 — 흐린 날 초록 부표 S 44~112 라 올리면 진짜 부표가 먼저 깎인다 (60 이면 0.27 → 0.05)
    green_v_min: int = 50         # 밝기 하한 (20 → 50, 2026-11-14) — 물·다리 초록 픽셀 V 중앙 49, 초록 부표 V 중앙 81
    green_dark_v: int = 50        # 이보다 어두운 픽셀은 H ≥ green_dark_h_min 일 때만 초록 — 흐린 날 검정 캡슐
    green_dark_h_min: int = 68    # 몸통(H 56~71·V 23~48)이 초록이 됐다 (2026-11-14 bag). 0 이면 끔
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
    # 뎁스로 창 픽셀 고르기 (refine_with_depth) — 창 픽셀 중 뎁스로 수평 좌표를 구해 그 부표 반경 안인 것만
    use_depth: bool = True
    depth_tol: float = 0.5        # 부표 반폭 + 이만큼 안 [m]
    depth_max: float = 6.0        # 이보다 먼 뎁스 픽셀은 안 씀 [m, D455 뎁스 원시값 — 라이다로 ≈ 5.2 m (뎁스 ≈ 1.092 × 라이다 + 0.28)].
                                  # 8 → 6 (2026-11-14): D455 뎁스는 6 m 정도까지가 쓸 만하다(현장). 3 bag 123 프레임: 부표 6~8 m
                                  # (라이다)에서 뎁스로 고른 픽셀은 중앙 21 % 만 부표(창 마스크 안), 62 % 창은 절반도 안 됨 —
                                  # 0~6 m 는 58~62 %. 그 구간 프레임 판정도 창 전체가 더 맞았다 (28 장 중 22 vs 19, 틀린 색 0).
                                  # 이보다 먼 부표는 창 전체로 센다
    depth_scale: float = 1.092    # D455 뎁스 ≈ scale × 라이다 + bias (2026-11-12 gate bag 부표 픽셀 773 창 실측)
    depth_bias: float = 0.28
    depth_min_px: int = 20        # 고른 픽셀이 이보다 적으면 창 전체로 (지금 방식)
    depth_z_min: float = -0.3     # 수면 아래로 계산된 픽셀(뎁스 매칭 오류) 버림 [m]
    depth_z_max: float = 1.2


@dataclass
class ColorVote:
    color: Optional[str]          # 'red' | 'green' | 'unknown' | None(판정 불가)
    rect: Optional[Tuple[int, int, int, int]]  # 원본 해상도 창 (u0, v0, u1, v1), 화면 밖이면 None
    red: int = 0
    green: int = 0
    visible: int = 0
    usable: bool = False          # 픽셀 수를 누적해도 되는 프레임 (보임·가림·번짐 조건 통과)
    depth: bool = False           # 뎁스로 고른 픽셀로 셌나 (refine_with_depth)


def classify_colors(bgr, p: ColorParams):
    """BGR → step 간격 라벨 (0 기타, 1 빨강, 2 초록, 3 보라 번짐)."""
    s = max(1, int(p.step))
    hsv = cv2.cvtColor(np.ascontiguousarray(bgr[::s, ::s]), cv2.COLOR_BGR2HSV)
    h, sat, val = hsv[..., 0], hsv[..., 1], hsv[..., 2]
    label = np.zeros(h.shape, dtype=np.uint8)
    label[(sat >= p.red_s_min) & (val >= p.red_v_min)
          & ((h <= p.red_h_max) | (h >= p.red_h_min))] = 1
    label[(sat >= p.green_s_min) & (val >= p.green_v_min)
          & (h >= p.green_h_min) & (h <= p.green_h_max)
          & ((val >= p.green_dark_v) | (h >= p.green_dark_h_min))] = 2
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
        _decide(vote, p)
        out.append(vote)
    return out


def _decide(vote: ColorVote, p: ColorParams):
    best, other_n = (('red', vote.red), vote.green) if vote.red >= vote.green else (('green', vote.green), vote.red)
    vote.color = None
    if best[1] >= p.min_color_px:
        if best[1] >= p.dominance * other_n:
            vote.color = best[0]
    elif vote.red + vote.green < p.min_color_px:
        vote.color = 'unknown'


def refine_with_depth(label, votes: List[ColorVote], candidates, depth_m, K, g: BuoyGeometry,
                      p: ColorParams, roll=0.0, pitch=0.0) -> List[ColorVote]:
    """창 픽셀 중 정렬 뎁스로 수평 좌표를 구해(camera_obstacles 와 같은 역투영) 그 부표 반경
    (반폭 + depth_tol) 안에 떨어지는 픽셀만으로 색을 다시 센다. 픽셀은 가장 가까운 부표 하나에만 —
    앞 부표에 가린 뒤 부표 창의 픽셀은 앞 부표로 간다. 고른 픽셀이 depth_min_px 보다 적으면 그 창은
    지금 방식(창 전체) 그대로.

    라이다 창은 실제 부표에서 어긋난다 (gate bag: 부표 픽셀 중 창 안 비율 중앙 49~67 %) — 창 안 물·옆 부표
    픽셀을 뎁스로 걸러낸다. 2026-11-12 gate bag 5 m 안 1993 창: 프레임 색 맞음 79 → 88 %, 정답 색 비율
    57 → 71 %. 뎁스는 라이다보다 멀게 나와(≈ 1.092 × 라이다 + 0.28) 보정해서 쓴다.
    """
    if depth_m is None or not p.use_depth:
        return votes
    s = max(1, int(p.step))
    D = np.asarray(depth_m, dtype=np.float32)[::s, ::s][:label.shape[0], :label.shape[1]]
    cam = CameraObstacleParams(offset_x=g.cam_offset_x, offset_y=g.cam_offset_y, height=g.cam_height,
                               mount_pitch_deg=g.cam_mount_pitch_deg)
    C = np.array([c for c, _r in candidates], dtype=float).reshape(-1, 2)
    R = np.array([max(float(r), p.min_half_width) for _c, r in candidates])
    for k, v in enumerate(votes):
        if v.rect is None:
            continue
        u0, v0, u1, v1 = v.rect
        c0, c1 = max(u0 // s, 0), min(u1 // s + 1, label.shape[1])
        r0, r1 = max(v0 // s, 0), min(v1 // s + 1, label.shape[0])
        if c0 >= c1 or r0 >= r1:
            continue
        d = D[r0:r1, c0:c1]
        rr, cc = np.nonzero(np.isfinite(d) & (d > 0.3) & (d <= p.depth_max))
        if len(rr) < p.depth_min_px:
            continue
        dd = (d[rr, cc] - p.depth_bias) / p.depth_scale
        pts = pixels_to_level((cc + c0) * s, (rr + r0) * s, dd, K, cam, roll, pitch)
        dist = np.hypot(pts[:, None, 0] - C[None, :, 0], pts[:, None, 1] - C[None, :, 1]) - R[None, :]
        sel = ((dist.argmin(axis=1) == k) & (dist[:, k] <= p.depth_tol)
               & (pts[:, 2] >= p.depth_z_min) & (pts[:, 2] <= p.depth_z_max))
        if int(sel.sum()) < p.depth_min_px:
            continue
        lv = label[rr[sel] + r0, cc[sel] + c0]
        if int(np.sum(lv == 3)) > p.max_tint_frac * len(lv):
            continue
        v.red, v.green, v.visible = int(np.sum(lv == 1)), int(np.sum(lv == 2)), int(len(lv))
        v.usable = True
        v.depth = True
        _decide(v, p)
    return votes
