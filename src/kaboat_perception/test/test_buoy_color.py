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
    ((52, 88, 34), 2),     # 초록 부표 짙은 몸통 (2026-11-12 bag)
    ((55, 100, 120), 2),   # 초록 뚜껑
    ((36, 75, 50), 0),     # 부표 옆 올리브색 물 — 초록 아님
    ((17, 75, 106), 0),    # 초록 부표 옆면 황갈색 띠 — 빨강 아님
    ((148, 110, 40), 3),   # 검정 부표 위 보라 번짐 — 번짐 라벨
    ((156, 135, 99), 3),   # 렌즈 보라 번짐 물
    ((72, 55, 58), 2),     # 초록 공 부표 (채도 50~70, bag 실측)
    ((5, 133, 230), 1),    # 빨강 부표
    ((176, 188, 197), 1),  # 빨강 부표 (H 170+)
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
