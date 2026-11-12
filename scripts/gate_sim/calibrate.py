"""rec_bag 으로 게이트 폐루프 시뮬레이션 값을 잰다 → calib.json.

부표 위치(지도 덩어리 누적), 부표별 라이다 검출률(거리별), 부표 크기, 배 근처 잡음 히트,
먼 구조물(맞은 빔이 free 를 칠하는 곳), 명령→속도·회전 응답.
"""
import datetime as dt
import json
import math
import sys

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

from kaboat_behaviors.gate_corridor import GateParams, grid_posts
from kaboat_behaviors.gate_memory import PostMemory

KST = dt.timezone(dt.timedelta(hours=9))
BAG, OUT = sys.argv[1], sys.argv[2]
T0 = dt.datetime(2026, 11, 12, 15, 24, 0, tzinfo=KST).timestamp()
T1 = dt.datetime(2026, 11, 12, 15, 37, 0, tzinfo=KST).timestamp()
REGION = (95.0, 140.0, -32.0, 0.0)          # 게이트 통로 둘레 (x0, x1, y0, y1)
LIDAR_X = 0.4


def reader(topics):
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=BAG, storage_id='sqlite3'), rosbag2_py.ConverterOptions('', ''))
    types = {t.name: t.type for t in r.get_all_topics_and_types()}
    r.set_filter(rosbag2_py.StorageFilter(topics=topics))
    while r.has_next():
        t, d, ns = r.read_next()
        s = ns * 1e-9
        if s < T0:
            continue
        if s > T1:
            break
        yield t, deserialize_message(d, get_message(types[t])), s


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


# 1) 부표: 지도 덩어리를 오래 쌓아 자주 잡힌 것만
p = GateParams()
mem = PostMemory(forget_sec=1e9, recheck_range=1e3, prune_range=1e4, confirm_hits=1)
odom = None
n_grid = 0
for t, m, s in reader(['/occupancy_grid', '/odom']):
    if t == '/odom':
        odom = m
        continue
    if odom is None:
        continue
    n_grid += 1
    if n_grid % 3:
        continue
    b = (odom.pose.pose.position.x, odom.pose.pose.position.y)
    mem.update(grid_posts(m, p), s, b)
x0, x1, y0, y1 = REGION
posts = [(float(q.xy[0]), float(q.xy[1]), q.hits) for q in mem.posts
         if q.hits >= 40 and x0 <= q.xy[0] <= x1 and y0 <= q.xy[1] <= y1]
P = np.array([(x, y) for x, y, _ in posts])
print(f'부표 {len(P)}개 (지도 {n_grid}장)')

# 2) 스캔: 부표별 검출률·크기, 배 근처 잡음, 먼 구조물
bins = [0.5, 2.5, 4.0, 5.0, 6.0, 7.5, 10.0, 12.0]
seen = np.zeros((len(P), len(bins) - 1))
tried = np.zeros((len(P), len(bins) - 1))
widths = []
clutter_counts, clutter_r = [], []
far_pts = []
pose = None
for t, m, s in reader(['/scan/level', '/odom']):
    if t == '/odom':
        q = m.pose.pose.orientation
        pose = (m.pose.pose.position.x, m.pose.pose.position.y, yaw_of(q))
        continue
    if pose is None:
        continue
    x, y, h = pose
    ox, oy = x + LIDAR_X * math.cos(h), y + LIDAR_X * math.sin(h)
    rg = np.asarray(m.ranges, float)
    a = m.angle_min + np.arange(len(rg)) * m.angle_increment
    ok = np.isfinite(rg) & (rg > m.range_min) & (rg < m.range_max)
    hx, hy = ox + rg[ok] * np.cos(h + a[ok]), oy + rg[ok] * np.sin(h + a[ok])
    H = np.stack([hx, hy], 1)
    d_post = np.sqrt(((H[:, None, :] - P[None, :, :]) ** 2).sum(2)) if len(H) else np.zeros((0, len(P)))
    near_post = d_post.min(1) < 0.6 if len(H) else np.zeros(0, bool)
    for j, (px, py) in enumerate(P):
        r = math.hypot(px - ox, py - oy)
        rel = math.atan2(py - oy, px - ox) - h
        rel = math.atan2(math.sin(rel), math.cos(rel))
        if abs(rel) > math.radians(110) or r < bins[0] or r >= bins[-1]:
            continue
        k = np.searchsorted(bins, r) - 1
        tried[j, k] += 1
        hit = d_post[:, j] < 0.5 if len(H) else np.zeros(0, bool)
        if hit.any():
            seen[j, k] += 1
            if 2.5 <= r < 5.0:
                widths.append((j, hit.sum() * m.angle_increment * r))
    rr = rg[ok]
    front = np.abs(np.arctan2(np.sin(a[ok]), np.cos(a[ok]))) < math.radians(110)
    clut = (~near_post) & (rr < 3.0) & front
    clutter_counts.append(int(clut.sum()))
    clutter_r.extend(rr[clut][::5].tolist())
    far = (~near_post) & (rr > 8.0)
    far_pts.extend(H[far][::3].tolist())
near = seen[:, :3].sum(1) / np.maximum(tried[:, :3].sum(1), 1)     # 5 m 안 검출률
real = (near >= 0.5) & (tried[:, :3].sum(1) >= 30)
print('부표별 5 m 안 검출률:', ' '.join(f'({x:.0f},{y:.0f}){r:.2f}' for (x, y), r in zip(P, near)))
print(f'진짜 부표(5 m 안 검출 ≥ 50 %) {real.sum()}개 / {len(P)}')
rate = (seen[real].sum(0) / np.maximum(tried[real].sum(0), 1)).tolist()
for k in range(len(rate)):
    print(f'  {bins[k]:4.1f}~{bins[k + 1]:4.1f} m: 검출 {rate[k]:.2f} ({int(tried[real, k].sum())}번 중)')
diam = float(np.median([w for j, w in widths if real[j]])) if widths else 0.4
ghosts = P[~real]
P = P[real]
print(f'부표 폭(히트 빔 각도 × 거리) 중앙 {diam:.2f} m, 배 근처(3 m 안) 잡음 히트 스캔당 중앙 '
      f'{np.median(clutter_counts):.0f}개, 평균 {np.mean(clutter_counts):.0f}')

# 먼 구조물 0.5 m 칸 (자주 맞은 곳)
F = np.asarray(far_pts)
cells = {}
for fx, fy in F:
    key = (int(math.floor(fx / 0.5)), int(math.floor(fy / 0.5)))
    cells[key] = cells.get(key, 0) + 1
far_cells = [k for k, c in cells.items() if c >= 5]
print(f'먼 구조물 칸 {len(far_cells)}개')

# 3) 명령 → 속도·회전 (자율 명령이 나간 구간)
cmd, od = [], []
for t, m, s in reader(['/cmd_vel', '/odom']):
    if t == '/cmd_vel':
        cmd.append((s, m.linear.x, m.angular.z))
    else:
        q = m.pose.pose.orientation
        od.append((s, m.pose.pose.position.x, m.pose.pose.position.y, yaw_of(q),
                   m.twist.twist.angular.z))
cmd, od = np.array(cmd), np.array(od)
dt_ = 0.1
ts = np.arange(max(cmd[0, 0], od[0, 0]), min(cmd[-1, 0], od[-1, 0]), dt_)
idx = np.searchsorted(cmd[:, 0], ts) - 1
fresh = (idx >= 0) & (ts - cmd[np.clip(idx, 0, None), 0] < 0.3)
cv = np.where(fresh, cmd[np.clip(idx, 0, None), 1], 0.0)
cw = np.where(fresh, cmd[np.clip(idx, 0, None), 2], 0.0)
X = np.interp(ts, od[:, 0], od[:, 1])
Y = np.interp(ts, od[:, 0], od[:, 2])
Hh = np.unwrap(np.interp(ts, od[:, 0], np.unwrap(od[:, 3])))
vx, vy = np.gradient(X, dt_), np.gradient(Y, dt_)
u = vx * np.cos(Hh) + vy * np.sin(Hh)                       # 선수 방향 속도
r = np.gradient(Hh, dt_)
# 명령이 나가던 구간: 1 초 안에 0 아닌 명령 (텔레옵은 /cmd_vel 에 0 으로 남아 빼야 함)
nz = fresh & ((np.abs(cv) > 1e-3) | (np.abs(cw) > 1e-3))
active = np.convolve(nz.astype(float), np.ones(10), 'full')[:len(nz)] > 0
lag = int(2.0 / dt_)
table = []
for lo, hi in ((0.0, 0.02), (0.02, 0.15), (0.15, 0.3), (0.3, 0.42)):
    hold = np.array([active[k - lag:k + 1].all() and np.all((cv[k - lag:k + 1] >= lo) & (cv[k - lag:k + 1] < hi))
                     for k in range(lag, len(cv))])
    vals = u[lag:][hold]
    table.append(((lo + hi) / 2, float(np.median(vals)) if len(vals) > 20 else float('nan'), int(len(vals))))
print('  정상 속도 표 (같은 명령 2 초 유지):', [(round(a, 2), round(b, 2), n) for a, b, n in table])
tv = [0.0] + [a for a, b, n in table if math.isfinite(b)]
tu = [0.0] + [max(b, 0.0) for a, b, n in table if math.isfinite(b)]
uss = np.interp(cv, tv, tu)
sel = active[:-1]
du = (u[1:] - u[:-1])[sel]
gap = (uss[:-1] - u[:-1])[sel]
k_u = float(np.dot(gap, du) / np.dot(gap, gap))
tau_u = dt_ / max(k_u, 1e-3)
Ar = np.column_stack([r[:-1], cw[:-1], cw[:-1] * u[:-1]])[sel]
coef_r, *_ = np.linalg.lstsq(Ar, r[1:][sel], rcond=None)
tau_r = -dt_ / math.log(max(min(coef_r[0], 0.999), 0.3))
gain_r = coef_r[1] / (1 - coef_r[0])
gain_ru = coef_r[2] / (1 - coef_r[0])
print(f'명령 구간 {sel.sum() * dt_:.0f} s: 속도 τ {tau_u:.2f} s; 회전 τ {tau_r:.2f} s, '
      f'정상 회전 = ({gain_r:.2f} + {gain_ru:.2f}·u)·w rad/s')
vs = list(zip(tv, tu))
gain_u = off_u = None
json.dump({
    'posts': P.tolist(), 'ghost_posts': ghosts.tolist(), 'detect_bins': bins, 'detect_rate': rate, 'post_diameter': diam,
    'clutter_counts': clutter_counts[::10], 'clutter_ranges': clutter_r[::10],
    'far_cells': far_cells, 'far_cell': 0.5,
    'tau_u': tau_u, 'gain_u': gain_u, 'off_u': off_u, 'speed_table': vs,
    'tau_r': tau_r, 'gain_r': gain_r, 'gain_ru': gain_ru,
}, open(OUT, 'w'))
print('→', OUT)
