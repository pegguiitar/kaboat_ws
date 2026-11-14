"""buoy_color 단독 테스트 — 라이다 후보 창의 색 표 (판정 불가 = 표 없음)."""
import numpy as np
import pytest

from kaboat_perception.buoy_color import (
    ColorParams, candidate_rect, classify_colors, vote_colors)
from kaboat_perception.buoy_localize import BuoyGeometry

K = (380.0, 380.0, 320.0, 240.0)
G = BuoyGeometry()
P = ColorParams()
S = P.step


def blank():
    return np.zeros((480 // S, 640 // S), dtype=np.uint8)


def paint(label, center, r, value):
    u0, v0, u1, v1, _d = candidate_rect(center, r, K, G, P)
    label[int(v0) // S:int(v1) // S + 1, int(u0) // S:int(u1) // S + 1] = value
    return label


@pytest.mark.parametrize('value, color', [(1, 'red'), (2, 'green'), (0, 'unknown')])
def test_window_vote(value, color):
    lab = paint(blank(), (6.0, 0.0), 0.25, value)
    v = vote_colors(lab, [((6.0, 0.0), 0.25)], K, G, P)[0]
    assert v.color == color and v.rect is not None


def test_mixed_red_green_is_no_vote():
    lab = blank()
    u0, v0, u1, v1, _ = candidate_rect((6.0, 0.0), 0.25, K, G, P)
    mid = int((u0 + u1) / 2) // S
    lab[int(v0) // S:int(v1) // S + 1, int(u0) // S:mid] = 1
    lab[int(v0) // S:int(v1) // S + 1, mid:int(u1) // S + 1] = 2
    assert vote_colors(lab, [((6.0, 0.0), 0.25)], K, G, P)[0].color is None


def test_far_buoy_hidden_behind_near_one_gets_no_vote():
    # 검정(가까움) 뒤 초록(멀리) — 가려진 동안 unknown 표가 아니라 표 없음
    lab = paint(blank(), (4.0, 0.0), 0.3, 0)
    votes = vote_colors(lab, [((4.0, 0.0), 0.3), ((9.0, 0.0), 0.3)], K, G, P)
    assert votes[0].color == 'unknown'
    assert votes[1].color is None


def test_behind_camera_or_off_screen_is_no_vote():
    votes = vote_colors(blank(), [((-3.0, 0.0), 0.3), ((1.0, 6.0), 0.3)], K, G, P)
    assert [v.color for v in votes] == [None, None]


@pytest.mark.parametrize('hsv, label', [
    # 초록 — 2026-11-14 3 bag 튜닝: H 64~89 · S ≥ 40 · V ≥ 50
    ((77, 155, 81), 2),    # 초록 부표 중앙값 (3 bag 뎁스 픽셀)
    ((72, 55, 58), 2),     # 초록 공 부표 (채도 50~70, bag 실측)
    ((77, 50, 60), 2),     # 흐린 아침 초록 부표 저채도 픽셀 — S 는 40 그대로 (60 이면 진짜 부표가 0.27 → 0.05)
    ((66, 88, 80), 2),     # 밝으면 H 66 도 초록
    ((73, 75, 49), 0),     # 흐린 날 물·다리 초록 픽셀 중앙값 (V 49) — 이제 초록 아님
    ((72, 88, 34), 0),     # 짙은 몸통 V 34 — 물과 겹쳐 이제 초록 아님 (예전 V ≥ 20)
    ((60, 88, 80), 0),     # H 60 — 하한 64 아래
    ((55, 100, 120), 0),   # H 55 — 예전 하한 55 에선 초록이었다
    ((64, 66, 35), 0),     # 흐린 아침 검정 캡슐 몸통 (2026-11-14 bag 중앙) — 초록 아님
    ((51, 80, 32), 0),     # 해 질 녘 검정 캡슐 어두운 몸통 — 초록 아님
    ((36, 75, 50), 0),     # 부표 옆 올리브색 물 — 초록 아님
    # 빨강 — S ≥ 80 · V ≥ 60
    ((5, 133, 230), 1),    # 빨강 부표
    ((176, 188, 197), 1),  # 빨강 부표 (H 170+)
    ((175, 196, 111), 1),  # 낮 gate 빨강 부표 그늘진 면 (V 111)
    ((175, 90, 70), 1),    # 흐린 날·오전 자주빛 빨강 — 예전 S ≥ 110 · V ≥ 80 에선 놓쳤다
    ((175, 60, 100), 0),   # 채도 60 — 하한 80 아래
    ((17, 75, 106), 0),    # 초록 부표 옆면 황갈색 띠 — 빨강 아님
    ((12, 118, 140), 0),   # 황갈색 띠 중 채도·밝기 높은 픽셀 — H 로 거름
    # 보라 번짐
    ((148, 110, 40), 3),   # 검정 부표 위 보라 번짐 — 번짐 라벨
    ((156, 135, 99), 3),   # 렌즈 보라 번짐 물
])
def test_pixel_colours_measured_at_sunset(hsv, label):
    import cv2
    bgr = cv2.cvtColor(np.array([[hsv]], dtype=np.uint8), cv2.COLOR_HSV2BGR)
    assert classify_colors(np.repeat(np.repeat(bgr, 2, 0), 2, 1), P)[0, 0] == label


def test_window_mostly_magenta_tint_is_no_vote():
    lab = paint(blank(), (6.0, 0.0), 0.25, 3)
    v = vote_colors(lab, [((6.0, 0.0), 0.25)], K, G, P)[0]
    assert v.color is None and not v.usable


def test_usable_window_reports_pixel_counts():
    lab = paint(blank(), (6.0, 0.0), 0.25, 2)
    v = vote_colors(lab, [((6.0, 0.0), 0.25)], K, G, P)[0]
    assert v.usable and v.green > 0 and v.visible >= v.green


def _depth_for(center, rect_label_value, behind_depth=None):
    """창 안 픽셀에 부표 자리 뎁스를 준 640×480 뎁스 (뒤쪽 칸은 behind_depth)."""
    from kaboat_perception.buoy_color import refine_with_depth  # noqa: F401
    u0, v0, u1, v1, d = candidate_rect(center, 0.25, K, G, P)
    dep = np.zeros((480, 640), np.float32)
    # 부표 앞면 광학 깊이 ≈ d, 뎁스 보정 역 (scale·bias)
    dep[int(v0):int(v1) + 1, int(u0):int(u1) + 1] = d * P.depth_scale + P.depth_bias
    if behind_depth is not None:   # 창 왼쪽 절반은 멀리 떨어진 물체(옆 부표)
        mid = int((u0 + u1) / 2)
        dep[int(v0):int(v1) + 1, int(u0):mid] = behind_depth
    return dep, (u0, v0, u1, v1)


def test_depth_keeps_only_pixels_on_the_buoy():
    from kaboat_perception.buoy_color import refine_with_depth
    lab = blank()
    dep, (u0, v0, u1, v1) = _depth_for((4.5, 0.0), 1, behind_depth=12.0)
    mid = int((u0 + u1) / 2)
    lab[int(v0) // S:int(v1) // S + 1, int(u0) // S:mid // S] = 2       # 왼쪽 반 = 뒤의 초록 (12 m)
    lab[int(v0) // S:int(v1) // S + 1, mid // S + 1:int(u1) // S + 1] = 1   # 오른쪽 반 = 이 부표 빨강
    cands = [((4.5, 0.0), 0.25)]
    v = vote_colors(lab, cands, K, G, P)[0]
    assert v.color is None                                   # 창만 보면 섞여서 판정 불가
    r = refine_with_depth(lab, [v], cands, dep, K, G, P)[0]
    assert r.depth and r.color == 'red' and r.green == 0


def test_depth_not_used_beyond_six_metres_of_depth():
    # D455 뎁스는 원시값 6 m 정도까지만 쓸 만하다 — 라이다 6 m 부표(뎁스 ≈ 6.8)는 창 전체로 센다
    from kaboat_perception.buoy_color import refine_with_depth
    assert P.depth_max == 6.0
    lab = blank()
    dep, (u0, v0, u1, v1) = _depth_for((6.0, 0.0), 1)
    lab[int(v0) // S:int(v1) // S + 1, int(u0) // S:int(u1) // S + 1] = 1
    cands = [((6.0, 0.0), 0.25)]
    v = vote_colors(lab, cands, K, G, P)[0]
    r = refine_with_depth(lab, [v], cands, dep, K, G, P)[0]
    assert not r.depth and r.color == 'red'


def test_no_depth_keeps_window_vote():
    from kaboat_perception.buoy_color import refine_with_depth
    lab = paint(blank(), (6.0, 0.0), 0.25, 2)
    cands = [((6.0, 0.0), 0.25)]
    v = vote_colors(lab, cands, K, G, P)[0]
    r = refine_with_depth(lab, [v], cands, np.zeros((480, 640), np.float32), K, G, P)[0]
    assert not r.depth and r.color == 'green'


# ---------- 물 대비 마스크 (창 안에서 물과 색이 다른 덩어리만) ----------

def _water_scene(buoy_bgr, center=(7.0, 0.0), water_bgr=(95, 105, 70), frac=0.45):
    """잔잔한 청록 물 위 부표 — 창의 가운데 frac 폭만 부표, 나머지 창은 물. (영상 BGR 640×480, 창)"""
    rng = np.random.default_rng(0)
    img = (np.array(water_bgr, np.float32) + rng.normal(0, 3, (480, 640, 3))).clip(0, 255).astype(np.uint8)
    u0, v0, u1, v1, _ = candidate_rect(center, 0.25, K, G, P)
    cx, hw = (u0 + u1) / 2, (u1 - u0) * frac / 2
    img[int(v0):int(v1) + 1, int(cx - hw):int(cx + hw) + 1] = buoy_bgr
    return img, (u0, v0, u1, v1)


def test_water_mask_counts_only_the_buoy_not_the_water():
    # 물 색이 초록 범위(H≈75·S≈80·V≈105)라 창 전체로는 초록 표가 섞인다 — 마스크면 빨강 부표 픽셀만
    img, _ = _water_scene((40, 40, 200))
    lab = classify_colors(img, P)
    cands = [((7.0, 0.0), 0.25)]
    whole = vote_colors(lab, cands, K, G, P)[0]
    masked = vote_colors(lab, cands, K, G, P, image=img)[0]
    assert whole.green > 0 and whole.color is None          # 물이 초록으로 섞여 판정 불가
    assert masked.mask is not None and masked.color == 'red'
    assert masked.green / masked.visible < 0.5 * whole.green / whole.visible   # 경계 한 줄 정도만 남음
    assert masked.visible < whole.visible


def test_water_mask_off_or_no_contrast_falls_back_to_window():
    img, _ = _water_scene((95, 105, 70))                      # 부표가 물과 같은 색 — 못 땀
    lab = classify_colors(img, P)
    v = vote_colors(lab, [((7.0, 0.0), 0.25)], K, G, P, image=img)[0]
    assert v.mask is None
    from dataclasses import replace
    img2, _ = _water_scene((40, 40, 200))
    off = vote_colors(classify_colors(img2, P), [((7.0, 0.0), 0.25)], K, G, replace(P, water_mask=False), image=img2)[0]
    assert off.mask is None and off.green > 0


def test_depth_intersects_water_mask():
    # 4.5 m — 뎁스로 고른 창 전체(부표 자리 뎁스) ∩ 마스크 = 부표 픽셀만
    from kaboat_perception.buoy_color import refine_with_depth
    img, _ = _water_scene((40, 40, 200), center=(4.5, 0.0), frac=0.5)
    lab = classify_colors(img, P)
    cands = [((4.5, 0.0), 0.25)]
    v = vote_colors(lab, cands, K, G, P, image=img)[0]
    dep, _ = _depth_for((4.5, 0.0), 1)
    r = refine_with_depth(lab, [v], cands, dep, K, G, P)[0]
    assert r.depth and r.color == 'red' and r.visible <= v.visible and r.green <= v.green
