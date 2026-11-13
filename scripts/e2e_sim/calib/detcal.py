"""물체(선분·원)별 스캔당 라이다 검출: 거리 구간별 (보인 스캔 비율, 맞은 빔 비율)."""
import pickle, numpy as np, math, sys, json
BINS = [0.5, 2.5, 4.0, 5.0, 6.0, 7.5, 10.0, 12.0, 15.0, 20.0]
_C = {}
def load(name):
    if name not in _C:
        D = pickle.load(open(f'cache/{name}.pkl', 'rb')); t0 = D['/odom'][0][0]
        od = np.array([(o[1], o[2], o[3], o[4]) for o in D['/odom']])
        S = np.load(f'cache/{name}_scan_level.npz')
        _C[name] = (t0, od, S['t'] - t0, S['ts'], S['r'], float(S['angle_min']), float(S['inc']))
    return _C[name]
def stats(name, t_lo, t_hi, p0, p1, tol=0.6, radius=None):
    t0, od, tr, ts, r, amin, inc = load(name)
    n = r.shape[1]; ang = amin + inc * np.arange(n)
    front = np.abs(np.degrees(ang)) < 118
    yaw_u = np.unwrap(od[:, 3])
    sel = np.nonzero((tr >= t_lo) & (tr <= t_hi))[0]
    px = np.interp(ts[sel], od[:, 0], od[:, 1]); py = np.interp(ts[sel], od[:, 0], od[:, 2]); pyaw = np.interp(ts[sel], od[:, 0], yaw_u)
    p0 = np.array(p0, float); p1 = np.array(p1, float)
    tried = np.zeros(len(BINS) - 1); seen = np.zeros(len(BINS) - 1); bexp = np.zeros(len(BINS) - 1); bhit = np.zeros(len(BINS) - 1)
    for k, i in enumerate(sel):
        lx = px[k] + 0.4 * math.cos(pyaw[k]); ly = py[k] + 0.4 * math.sin(pyaw[k])
        o = np.array([lx, ly]); a = pyaw[k] + ang; d = np.c_[np.cos(a), np.sin(a)]
        if radius is None:
            e = p1 - p0; den = d[:, 0] * e[1] - d[:, 1] * e[0]
            w = p0 - o
            with np.errstate(divide='ignore', invalid='ignore'):
                tt = (w[0] * e[1] - w[1] * e[0]) / den; s = (w[0] * d[:, 1] - w[1] * d[:, 0]) / den
            exp = front & (tt > 0.6) & (s >= 0) & (s <= 1)
        else:
            w = p0 - o; proj = d @ w; perp2 = w @ w - proj ** 2
            exp = front & (proj > 0.6) & (perp2 < radius ** 2); tt = proj
        if exp.sum() < 2: continue
        rng = float(np.median(tt[exp])); b = np.searchsorted(BINS, rng) - 1
        if b < 0 or b >= len(tried): continue
        rr = r[i]; hit = exp & np.isfinite(rr) & (np.abs(rr - tt) < tol)
        tried[b] += 1; seen[b] += hit.sum() >= 2; bexp[b] += exp.sum(); bhit[b] += hit.sum()
    return tried, seen, bexp, bhit
def show(label, items):
    T = S = E = Hh = 0
    for it in items:
        t, s, e, h = stats(*it[:5], **(it[5] if len(it) > 5 else {})); T = T + t; S = S + s; E = E + e; Hh = Hh + h
    with np.errstate(divide='ignore', invalid='ignore'):
        print(label); print('  bins   ', BINS); print('  scans  ', T.astype(int).tolist()); print('  seen   ', np.round(S / T, 2).tolist()); print('  beamfr ', np.round(Hh / E, 2).tolist())
    return dict(bins=BINS, scans=T.tolist(), seen=np.nan_to_num(S / T).round(3).tolist(), beam=np.nan_to_num(Hh / E).round(3).tolist())
out = {}
# 토너먼트 bag 의 바 (11-13 11:36) — 덜 번진 것 셋
out['bar_tour'] = show('tournament bars', [('tour', 34, 110, (19.889, -18.919), (22.455, -15.177)), ('tour', 34, 110, (9.512, -20.392), (11.677, -24.243)), ('tour', 34, 110, (15.347, -13.374), (19.287, -17.028))])
# 종합 회피 구역의 연결 장애물 (dock1 10:21, 바뀌기 전일 수 있음)
out['boom_dock1'] = show('avoid booms in dock1', [('dock1', 778, 901, (125.0, -1.64), (125.32, 2.69)), ('dock1', 778, 901, (121.5, -2.4), (122.2, 1.6)), ('dock1', 778, 901, (130.38, -2.2), (131.17, 1.87)), ('dock1', 778, 901, (119.09, -0.62), (119.18, 3.38))])
out['boom_run12'] = show('avoid booms in run12 (11-12)', [('run12', 0, 1553, (118.765, -1.565), (115.869, -1.7)), ('run12', 0, 1553, (115.03, 0.18), (113.97, 1.43))])
# 부표
out['buoy_gate'] = show('gate buoys dock1', [('dock1', 74, 146, c, None, dict(radius=0.3, tol=0.5)) for c in [(111.59,-17.92),(113.84,-19.58),(116.63,-20.08),(119.96,-19.70),(122.52,-20.84),(125.31,-22.09),(128.54,-22.05),(112.07,-14.37),(114.31,-16.21),(117.16,-17.13),(120.51,-17.02),(122.88,-18.46),(125.93,-19.06),(129.31,-18.66)]])
out['buoy_avoid'] = show('avoid white buoys dock1', [('dock1', 778, 901, c, None, dict(radius=0.45, tol=0.6)) for c in [(124.58,-3.46),(127.87,0.2),(128.44,2.89),(136.65,-2.8),(137.08,0.58)]])
out['buoy_tour'] = show('tournament buoys', [('tour', 34, 110, c, None, dict(radius=0.45, tol=0.6)) for c in [(11.39,-17.90),(14.74,-17.28),(21.08,-11.02),(19.98,-13.73)]])
out['buoy_station'] = show('station buoys dock1', [('dock1', 150, 300, c, None, dict(radius=0.35, tol=0.6)) for c in [(155.29,-35.03),(159.84,-33.54),(151.10,-32.20),(155.42,-29.22)]])
out['buoy_search'] = show('search buoys dock1', [('dock1', 674, 748, c, None, dict(radius=0.35, tol=0.6)) for c in [(170.14,-5.99),(175.44,-1.32),(167.26,-0.32)]])
# 도킹장 뒷벽 (가운데 슬롯)
out['dock_wall'] = show('dock back wall dock1', [('dock1', 459, 582, (162.48, -51.40), (165.34, -52.19), dict(tol=0.8))])
out['bound_avoid'] = show('avoid boundary south line dock1', [('dock1', 778, 901, (118.8, -4.92), (130.1, -6.14), dict(tol=0.5))])
json.dump(out, open('cache/detcal.json', 'w'), indent=1)
