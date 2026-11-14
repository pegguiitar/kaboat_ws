"""bag 에서 잰 값 → arena_20261114.json (odom 좌표, 2026-11-14 오전 run1_20261114_091623 실주행 배치).

출처 (모두 run1_20261114_091623 bag):
  게이트 7쌍·스테이션 4개·탐색 부표·회피 구역 안쪽 바·부표 = 미션 구간별 라이다 군집 (calib/build_arena.py 의
    points/clusters, 회피 구역은 7 m 안 점·min_hits 20), 부표 색 = 그 bag /detections/buoys
  도킹장 = 그 bag 에서 dock_ctrl_v2 가 고정한 칸(/dock/slot_markers, 서쪽 칸) + dock_slot.yaml 치수,
    표지 위치 = /dock/markers/<도형>/ground 중앙값, 오늘 목표 = triangle (/dock/status)
  미션 지점 = /mission_points/markers, 출발 자세 = AUTO 시작(78.8 s) odom, 탐색 목표 = black (/search/status)
  회피 구역 모서리·입구·출구·경계줄 = arena.json(11-13) 그대로 — 오늘 라이다 점과 겹치는 것 확인
  라이다 검출률 = arena.json(11-13) 그대로

탐색 구역 (171.6, −4.4) 은 검정 부표 하나: 라이다 트랙 하나가 처음엔 unseen (171.27, −4.46), 나중엔 green (171.82, −4.35) 로
나왔지만, 그 앞 1.4–1.7 m 카메라 사진에 짙은 캡슐 부표 하나만 보인다. 몸통 HSV 중앙 H 60–64·S 37–43·V 49–53 이 초록 기준
(H 55–89, S ≥ 40) 에 걸려 실물 검출기가 초록으로 판정했다. 시뮬 카메라는 검정을 검정으로 그리므로 이 오판은 재현하지 않는다.
"""
import json
import math
import os

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
BASE = json.load(open(f'{HERE}/arena.json'))      # 11-13 경기장 — 회피 경계·검출률·원점만 가져온다

A = {'frame': 'odom', 'source': 'run1_20261114_091623 (회피 경계·검출률은 arena.json 11-13)'}
A['origin'] = BASE['origin']
A['targets'] = {'search_color': 'black', 'dock_class': 'triangle'}
A['points'] = {
    'gate_entry': [107.31, -14.29, -6.4], 'gate_end': [131.57, -22.08, -4.8],
    'station_entry': [151.81, -24.63, -12.8], 'station_center': [155.16, -31.21, -90.4],
    'station_end': [163.56, -34.46, -21.2],
    'dock_entry': [169.44, -34.24, -95.5], 'dock_end': [168.76, -21.49, 75.9],
    'search_entry': [168.70, -8.73, 71.9], 'search_end': [147.10, -2.31, 171.0],
    'avoid_entry': [142.16, -1.53, 166.9], 'avoid_end': [95.00, 5.93, 139.5],
    'docking_entry': [166.86, -40.85, -116.3]}       # 미션 이름 규칙 밖 — mission_manager 는 안 씀 (bag 그대로)
A['start'] = [57.93, 8.24, 41.7]                     # AUTO 시작 (78.8 s) odom

circles, segs, polys = [], [], []


def C(name, x, y, r, cls, color):
    circles.append(dict(name=name, x=float(x), y=float(y), r=r, cls=cls, color=color))


def S(name, p0, p1, hw, cls, shrink=0.2):
    """군집 끝점(번짐 포함)을 양 끝 shrink 만큼 줄인 바."""
    p0, p1 = np.array(p0, float), np.array(p1, float)
    u = (p1 - p0) / np.linalg.norm(p1 - p0)
    a, b = p0 + u * shrink, p1 - u * shrink
    segs.append(dict(name=name, p0=a.round(3).tolist(), p1=b.round(3).tolist(), hw=hw, cls=cls))


# ── 게이트 7쌍 (북 = 초록, 남 = 빨강 — /detections/buoys 색) ──
gn = [(112.06, -14.31), (115.17, -15.38), (117.12, -16.51), (120.36, -16.92), (123.72, -17.61), (126.72, -18.22), (129.27, -18.64)]
gs = [(111.53, -17.94), (113.66, -18.41), (116.78, -18.86), (120.10, -19.65), (123.40, -20.02), (125.24, -20.29), (128.53, -22.00)]
A['gates'] = []
for i, (n_, s_) in enumerate(zip(gn, gs)):
    C(f'gate{i + 1}N', *n_, 0.25, 'buoy', 'green')
    C(f'gate{i + 1}S', *s_, 0.25, 'buoy', 'red')
    A['gates'].append([list(n_), list(s_)])

# ── 스테이션 부표 4개 (서·북·동·남, 군집이 게이트 부표보다 커 r 0.35) ──
st = [(152.06, -32.09), (156.09, -28.51), (160.63, -33.11), (155.84, -34.24)]
for i, p in enumerate(st):
    C(f'station{i + 1}', *p, 0.35, 'buoy', 'orange')
A['station_poly'] = [list(p) for p in st]

# ── 탐색 부표 ──
C('search_black', 171.6, -4.4, 0.3, 'buoy', 'black')          # 위 docstring — 카메라가 초록으로 본 검정
C('search_red', 177.13, 0.46, 0.3, 'buoy', 'red')
C('search_green', 168.19, 0.94, 0.3, 'buoy', 'green')
A['search'] = {'black': [171.6, -4.4], 'red': [177.13, 0.46], 'green': [168.19, 0.94]}

# ── 도킹장: 가운데 칸 뒷벽 중심 c0, 바깥 법선 n, 접선 t(동쪽) ──
# dock_ctrl_v2 가 고정한 서쪽(triangle) 칸: 뒷벽 (158.917,−43.867)–(161.792,−44.609), 입구 (159.402,−41.988)–(162.277,−42.731)
back0, back1 = np.array([158.917, -43.867]), np.array([161.792, -44.609])
open0 = np.array([159.402, -41.988])
t = (back1 - back0) / np.linalg.norm(back1 - back0)
n = (open0 - back0) / np.linalg.norm(open0 - back0)
n = n - (n @ t) * t
n /= np.linalg.norm(n)
W, D, P = 2.97, 1.94, 4.02
c0 = (back0 + back1) / 2 + P * t          # 서쪽 칸 중심 + 한 칸 = 가운데 칸
hwp = (P - W) / 2


def rect(name, a0, a1, b0, b1, cls='dock'):
    pts = [c0 + a * t + b * n for a, b in ((a0, b0), (a1, b0), (a1, b1), (a0, b1))]
    polys.append(dict(name=name, pts=[p.round(3).tolist() for p in pts], cls=cls))


for k, off in enumerate((-1.5 * P, -0.5 * P, 0.5 * P, 1.5 * P)):
    rect(f'dock_partition{k}', off - hwp, off + hwp, 0.0, D)
rect('dock_back', -1.5 * P - hwp, 1.5 * P + hwp, -1.0, 0.0)
order = {'triangle': -1, 'circle': 0, 'square': 1}      # 11-13 과 같은 순서 (서 → 동)
seen = {'triangle': (161.35, -44.43), 'circle': (164.51, -45.18), 'square': (168.11, -45.80)}   # /dock/markers/*/ground 중앙값
A['dock'] = {'c0': c0.round(3).tolist(), 'n': n.tolist(), 't': t.tolist(), 'slot_width': W, 'depth': D, 'pitch': P,
             'slots': {c: (c0 + k * P * t).round(3).tolist() for c, k in order.items()},
             'markers': {c: (c0 + ((np.array(p) - c0) @ t) * t - 0.05 * n).round(3).tolist() for c, p in seen.items()}}

# ── 회피 구역: 모서리·입구·출구·경계줄 = 11-13 그대로 ──
for c in BASE['circles']:
    if c['name'].startswith(('avoid_corner', 'avoid_entry', 'avoid_exit')) or c['cls'] == 'bound':
        circles.append(dict(c))
A['avoid'] = BASE['avoid']
# 안쪽: 오늘 군집 (calib/build_arena.points 7 m·min_hits 20, 끝점은 번짐 0.2 m 씩 줄임)
bars = [((116.072, 2.017), (117.487, 5.238)),     # 서쪽 대각선
        ((119.071, 0.281), (119.576, 4.399)),     # x≈119.3 남북
        ((120.868, 2.590), (120.420, 3.682)),     # 짧은 바
        ((122.031, 4.275), (124.090, 3.667)),     # 북쪽
        ((124.913, -3.718), (125.454, 0.860)),    # x≈125.2 남북
        ((126.685, 1.696), (125.349, 2.822)),     # 짧은 대각선
        ((126.899, -1.064), (126.060, 0.569)),    # 짧은 바
        ((131.737, -1.707), (127.542, 0.979)),    # 입구 쪽 긴 대각선
        ((131.315, 1.493), (130.600, 4.600))]     # x≈131 북쪽 (경계줄까지)
for i, (a, b) in enumerate(bars):
    S(f'avoid_bar{i}', a, b, 0.15, 'bar')
for i, p in enumerate([(116.20, -0.16), (119.10, -2.32), (122.04, -1.11), (122.51, 1.55), (127.89, -1.88),
                       (128.25, 3.40), (130.60, -3.95), (131.43, 1.00)]):
    C(f'avoid_buoy{i}', *p, 0.3, 'buoy', 'white')

A['circles'], A['segments'], A['polys'] = circles, segs, polys
A['detect'] = BASE['detect']
json.dump(A, open(f'{HERE}/arena_20261114.json', 'w'), indent=1)
print('circles', len(circles), 'segments', len(segs), 'polys', len(polys))

if __name__ == '__main__':
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(figsize=(30, 14))
    col = {'red': 'r', 'green': 'g', 'black': 'k', 'orange': 'orange', 'white': 'gray', 'yellow': 'y'}
    for c in circles:
        ax.add_patch(plt.Circle((c['x'], c['y']), c['r'], color=col[c['color']]))
    for s in segs:
        ax.plot([s['p0'][0], s['p1'][0]], [s['p0'][1], s['p1'][1]], 'm-', lw=3)
    for p in polys:
        ax.add_patch(plt.Polygon(p['pts'], color='b', alpha=0.5))
    for k, v in A['dock']['markers'].items():
        ax.annotate(k, v, fontsize=8, color='b')
    for k, (x, y, a) in A['points'].items():
        ax.plot(x, y, 'k+')
        ax.annotate(k, (x, y), fontsize=8)
        if k.endswith('entry'):
            ax.arrow(x, y, 2 * math.cos(math.radians(a)), 2 * math.sin(math.radians(a)), head_width=0.4)
    ax.plot(*A['start'][:2], 'b*', ms=12)
    ax.annotate('start', A['start'][:2])
    ax.set_aspect('equal')
    ax.set_xlim(52, 182)
    ax.set_ylim(-58, 14)
    ax.grid(alpha=0.3)
    fig.savefig(f'{HERE}/arena_20261114.png', dpi=50, bbox_inches='tight')
