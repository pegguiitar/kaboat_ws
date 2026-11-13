"""bag 에서 잰 값 → arena.json (odom 좌표, 2026-11-13 오전 dock_test1 배치 기준).

출처: gate/station/search/avoid 물체 = dock_test1 라이다 군집, dock = dock_ctrl_v2 가 그 bag 에서 고정한
ㄷ자(가운데 슬롯) + dock_slot.yaml 치수, 표지 순서 = 그 bag YOLO 방위, 회피 구역 서쪽 출구 = virtual_walls.yaml
+ run_20261112 군집, 미션 지점 = bag 의 /mission_points/markers, 검출률 = dock_test1·tournament1 스캔 통계.
회피 구역 연결 장애물은 사용자 지시(2026-11-14)로 부표 높이의 바 — 검출률은 tournament1 bag 의 바.
"""
import json, math
import numpy as np
A = {'frame': 'odom', 'source': 'dock_test1_20261113 (+run_20261112, dock2_20261113; bar detection from tournament1_20261113)'}
A['origin'] = {'lat': 35.1848790, 'lon': 128.5651015, 'alt': 0.0}
A['points'] = {
    'gate_entry': [108.80, -14.45, -6.4], 'gate_end': [131.57, -22.08, -4.8],
    'station_entry': [143.37, -26.65, -21.2], 'station_center': [155.16, -31.21, -90.4], 'station_end': [159.83, -33.02, -21.2],
    'dock_entry': [169.44, -34.24, -95.5], 'dock_end': [168.76, -21.49, 75.9],
    'search_entry': [168.70, -8.73, 71.9], 'search_end': [147.10, -2.31, 171.0],
    'avoid_entry': [142.16, -1.53, 166.9], 'avoid_end': [95.00, 5.93, 139.5]}
circles = []   # name, x, y, r, cls, color
segs = []      # name, [x0,y0,x1,y1], half_width, cls
def C(name, x, y, r, cls, color): circles.append(dict(name=name, x=x, y=y, r=r, cls=cls, color=color))
def S(name, p0, p1, hw, cls): segs.append(dict(name=name, p0=list(p0), p1=list(p1), hw=hw, cls=cls))
# ── 게이트 7쌍 (북 = 초록, 남 = 빨강) ──
gs = [(111.59,-17.92),(113.84,-19.58),(116.63,-20.08),(119.96,-19.70),(122.52,-20.84),(125.31,-22.09),(128.54,-22.05)]
gn = [(112.07,-14.37),(114.31,-16.21),(117.16,-17.13),(120.51,-17.02),(122.88,-18.46),(125.93,-19.06),(129.31,-18.66)]
A['gates'] = []
for i, (n_, s_) in enumerate(zip(gn, gs)):
    C(f'gate{i+1}N', *n_, 0.25, 'buoy', 'green'); C(f'gate{i+1}S', *s_, 0.25, 'buoy', 'red'); A['gates'].append([list(n_), list(s_)])
# ── 스테이션 부표 4개 ──
st = [(151.10,-32.20),(155.42,-29.22),(159.84,-33.54),(155.29,-35.03)]
for i, p in enumerate(st): C(f'station{i+1}', *p, 0.3, 'buoy', 'orange')
A['station_poly'] = [list(p) for p in st]
# ── 탐색 부표 ──
C('search_red', 175.64, -1.31, 0.3, 'buoy', 'red'); C('search_green', 167.27, -0.17, 0.3, 'buoy', 'green')
C('search_black', 170.14, -5.99, 0.3, 'buoy', 'black'); C('search_black2', 167.37, -1.77, 0.22, 'buoy', 'black')
A['search'] = {'red': [175.64, -1.31], 'green': [167.27, -0.17], 'black': [170.14, -5.99]}
# ── 도킹장: 가운데 슬롯 뒷벽 중심 c0, 바깥 법선 n, 접선 t(동쪽) ──
c0 = np.array([163.91, -51.795]); n = np.array([0.263, 0.964]); n /= np.linalg.norm(n); t = np.array([n[1], -n[0]])
W, D, P = 2.97, 1.94, 4.02
polys = []
def rect(name, a0, a1, b0, b1, cls='dock'):
    pts = [c0 + a * t + b * n for a, b in ((a0, b0), (a1, b0), (a1, b1), (a0, b1))]
    polys.append(dict(name=name, pts=[p.round(3).tolist() for p in pts], cls=cls))
hwp = (P - W) / 2
for k, off in enumerate((-1.5 * P, -0.5 * P, 0.5 * P, 1.5 * P)): rect(f'dock_partition{k}', off - hwp, off + hwp, 0.0, D)
rect('dock_back', -1.5 * P - hwp, 1.5 * P + hwp, -1.0, 0.0)
order = {'triangle': -1, 'circle': 0, 'square': 1}     # 접선(동쪽) 방향 슬롯 번호 — 배에서 보면 왼쪽부터 square, circle, triangle
moff = {'triangle': -3.7, 'circle': 0.2, 'square': 3.7}
A['dock'] = {'c0': c0.tolist(), 'n': n.tolist(), 't': t.tolist(), 'slot_width': W, 'depth': D, 'pitch': P,
             'slots': {c: (c0 + k * P * t).round(3).tolist() for c, k in order.items()},
             'markers': {c: (c0 + moff[c] * t - 0.05 * n).round(3).tolist() for c in order}}
# ── 회피 구역 ──
SW, SE, NE, NW = (109.87, -2.64), (136.53, -5.90), (137.19, 3.77), (110.80, 7.50)
gE = [(136.65, -2.80), (137.08, 0.58)]; gW = [(109.70, 0.90), (110.50, 4.60)]
for nm, p in (('avoid_cornerSW', SW), ('avoid_cornerSE', SE), ('avoid_cornerNE', NE), ('avoid_cornerNW', NW)): C(nm, *p, 0.5, 'buoy', 'orange')
for nm, p in (('avoid_entryS', gE[0]), ('avoid_entryN', gE[1]), ('avoid_exitS', gW[0]), ('avoid_exitN', gW[1])): C(nm, *p, 0.45, 'buoy', 'white')
def line(name, pts, step=0.8):
    k = 0
    for a, b in zip(pts[:-1], pts[1:]):
        a = np.array(a); b = np.array(b); L = np.linalg.norm(b - a); m = int(L // step)
        for i in range(1, m + 1):
            q = a + (b - a) * (i * step / L)
            if np.linalg.norm(q - b) < 0.5: continue
            C(f'{name}{k}', float(q[0]), float(q[1]), 0.08, 'bound', 'yellow'); k += 1
line('avoid_southline', [SW, (118.8, -4.92), (130.1, -6.14), SE])
line('avoid_northline', [NW, (116.3, 6.25), (122.0, 5.15), (131.0, 3.9), NE])
line('avoid_eastlineS', [SE, gE[0]]); line('avoid_eastlineN', [gE[1], NE]); line('avoid_westlineS', [SW, gW[0]]); line('avoid_westlineN', [gW[1], NW])
bars = [((116.07, -0.90), (116.30, 2.30)), ((119.09, -0.62), (119.18, 3.38)), ((121.50, -2.40), (122.20, 1.60)),
        ((125.00, -1.64), (125.32, 2.69)), ((130.38, -2.22), (131.17, 1.87)), ((127.00, -3.08), (130.88, -4.87)),
        ((122.10, 3.45), (125.00, 4.20))]
for i, (a, b) in enumerate(bars):
    S(f'avoid_bar{i}', a, b, 0.15, 'bar')
    C(f'avoid_bar{i}a', *a, 0.4, 'buoy', 'white'); C(f'avoid_bar{i}b', *b, 0.4, 'buoy', 'white')
for i, p in enumerate([(124.58, -3.46), (127.87, 0.20), (128.44, 2.89), (118.83, -2.92), (113.30, -1.70), (113.50, 3.30)]):
    C(f'avoid_buoy{i}', *p, 0.45, 'buoy', 'white')
A['avoid'] = {'corners': [list(SW), list(SE), list(NE), list(NW)], 'entry_gate': [list(p) for p in gE], 'exit_gate': [list(p) for p in gW]}
A['circles'] = circles; A['segments'] = segs; A['polys'] = polys
# ── 라이다 검출 (거리 구간별: 그 스캔에 보일 확률, 보일 때 빔 반환율) — cache/detcal.json 통계를 다듬은 값 ──
A['detect'] = {
    'bins': [0.5, 2.5, 4.0, 5.0, 6.0, 7.5, 10.0, 12.0, 15.0, 20.0],
    'buoy':  {'seen': [0.99, 0.98, 0.85, 0.60, 0.38, 0.28, 0.12, 0.05, 0.03], 'beam': 0.65},   # 게이트·스테이션·탐색·회피 부표 평균
    'bar':   {'seen': [1.00, 1.00, 0.50, 0.49, 0.36, 0.07, 0.01, 0.01, 0.00], 'beam': 0.70},   # tournament1 바 (부표 높이)
    'boom_low': {'seen': [0.77, 0.73, 0.66, 0.48, 0.44, 0.15, 0.06, 0.01, 0.00], 'beam': 0.55},  # 예전 낮은 연결물 (dock_test1) — 비교용
    'dock':  {'seen': [1.00, 1.00, 0.95, 0.85, 0.70, 0.40, 0.20, 0.05, 0.02], 'beam': 0.60},   # 약 6 m 안에서 안정 (2026-11-12 실측 메모)
    'bound': {'seen': [0.60, 0.60, 0.50, 0.35, 0.15, 0.08, 0.03, 0.00, 0.00], 'beam': 0.80}}   # 노란 작은 부이 (지름 0.15)
A['start'] = [99.86, -13.45, -6.4]      # 가정: gate_entry 선수 방향 9 m 뒤 (출발 슬롯 위치는 bag 에 없음)
json.dump(A, open('arena.json', 'w'), indent=1)
print('circles', len(circles), 'segments', len(segs), 'polys', len(polys))
# 그림
import matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
fig, ax = plt.subplots(figsize=(24, 14))
col = {'red': 'r', 'green': 'g', 'black': 'k', 'orange': 'orange', 'white': 'gray', 'yellow': 'y'}
for c in circles: ax.add_patch(plt.Circle((c['x'], c['y']), c['r'], color=col[c['color']]))
for s in segs: ax.plot([s['p0'][0], s['p1'][0]], [s['p0'][1], s['p1'][1]], 'm-', lw=3)
for p in polys: ax.add_patch(plt.Polygon(p['pts'], color='b', alpha=0.5))
for k, v in A['dock']['markers'].items(): ax.annotate(k, v, fontsize=8, color='b')
for k, (x, y, a) in A['points'].items():
    ax.plot(x, y, 'k+'); ax.annotate(k, (x, y), fontsize=8)
    if k.endswith('entry'): ax.arrow(x, y, 2 * math.cos(math.radians(a)), 2 * math.sin(math.radians(a)), head_width=0.4)
ax.plot(*A['start'][:2], 'b*', ms=12); ax.annotate('start', A['start'][:2])
ax.set_aspect('equal'); ax.set_xlim(92, 182); ax.set_ylim(-58, 12); ax.grid(alpha=0.3)
fig.savefig('arena.png', dpi=60, bbox_inches='tight')
