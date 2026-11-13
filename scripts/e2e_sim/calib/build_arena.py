"""bag 라이다 점 → 구역별 물체(원·선분) 추출. 결과 cache/clusters.json + 겹쳐 그린 그림."""
import pickle, numpy as np, math, json, sys
from scipy import ndimage
import matplotlib; matplotlib.use('Agg')
import matplotlib.pyplot as plt
RES = 0.05
def load(name):
    D = pickle.load(open(f'cache/{name}.pkl', 'rb'))
    t0 = D['/odom'][0][0]
    od = np.array([(o[1], o[2], o[3], o[4], o[0] - t0) for o in D['/odom']])
    S = np.load(f'cache/{name}_scan.npz')
    return D, t0, od, S
CACHE = {}
def points(name, t_lo, t_hi, box, max_range=12.0):
    if name not in CACHE: CACHE[name] = load(name)
    D, t0, od, S = CACHE[name]
    ts = S['ts']; tr = S['t'] - t0
    r = S['r']; n = r.shape[1]
    ang = float(S['angle_min']) + float(S['inc']) * np.arange(n)
    yaw_u = np.unwrap(od[:, 3])
    sel = np.nonzero((tr >= t_lo) & (tr <= t_hi))[0]
    px = np.interp(ts[sel], od[:, 0], od[:, 1]); py = np.interp(ts[sel], od[:, 0], od[:, 2]); pyaw = np.interp(ts[sel], od[:, 0], yaw_u)
    out = []; poses = []
    for k, i in enumerate(sel):
        m = np.isfinite(r[i]) & (r[i] > 0.6) & (r[i] < max_range)
        if not m.any(): continue
        a = pyaw[k] + ang[m]
        lx = px[k] + 0.4 * math.cos(pyaw[k]); ly = py[k] + 0.4 * math.sin(pyaw[k])
        hx = lx + r[i][m] * np.cos(a); hy = ly + r[i][m] * np.sin(a)
        ok = (hx > box[0]) & (hx < box[1]) & (hy > box[2]) & (hy < box[3])
        out.append(np.c_[hx[ok], hy[ok]]); poses.append((lx, ly))
    return np.vstack(out), np.array(poses)
def clusters(pts, box, min_hits=4, min_cells=3, dilate=1):
    a, b, c, d = box
    W, H = int((b - a) / RES), int((d - c) / RES)
    ix = ((pts[:, 0] - a) / RES).astype(int).clip(0, W - 1); iy = ((pts[:, 1] - c) / RES).astype(int).clip(0, H - 1)
    g = np.zeros((H, W)); np.add.at(g, (iy, ix), 1)
    occ = g >= min_hits
    lab, n = ndimage.label(ndimage.binary_dilation(occ, iterations=dilate), structure=np.ones((3, 3)))
    lab = lab * occ
    res = []
    for i in range(1, n + 1):
        yy, xx = np.nonzero(lab == i)
        if len(xx) < min_cells: continue
        w = g[yy, xx]
        P = np.c_[a + (xx + .5) * RES, c + (yy + .5) * RES]
        mu = (P * w[:, None]).sum(0) / w.sum()
        C = np.cov((P - mu).T, aweights=w) if len(xx) > 2 else np.eye(2) * 1e-4
        ev, evec = np.linalg.eigh(C)
        major = evec[:, 1]; proj = (P - mu) @ major; perp = (P - mu) @ evec[:, 0]
        res.append(dict(c=mu.round(3).tolist(), hits=int(w.sum()), cells=len(xx), length=float(np.ptp(proj)) + RES, width=float(np.ptp(perp)) + RES,
                        p0=(mu + major * proj.min()).round(3).tolist(), p1=(mu + major * proj.max()).round(3).tolist(),
                        w95=float(np.percentile(np.abs(perp), 90)) * 2))
    return res, g
REG = {
  'tour':    dict(bag='tour', t=(34, 110),    box=(-2, 36, -36, 0),  rng=12, min_hits=5),
  'gate':    dict(bag='dock2', t=(0, 60),      box=(104, 136, -27, -9),  rng=10, min_hits=5),
  'station': dict(bag='dock1', t=(150, 300),    box=(146, 166, -40, -22), rng=12, min_hits=5),
  'gate1':   dict(bag='dock1', t=(74, 146),     box=(104, 136, -27, -9),  rng=10, min_hits=5),
  'search':  dict(bag='dock1', t=(674, 748),   box=(160, 182, -12, 6),   rng=12, min_hits=6),
  'dock':    dict(bag='dock1', t=(459, 582),   box=(154, 176, -58, -44), rng=10, min_hits=10),
  'avoid':   dict(bag='dock1', t=(778, 901),   box=(104, 144, -10, 12),  rng=12, min_hits=6),
  'avoid_b': dict(bag='dock1', t=(1012, 1076), box=(104, 144, -10, 12),  rng=12, min_hits=6),
  'avoid_w': dict(bag='dock1', t=(1012, 1113),    box=(104, 124, -10, 12),  rng=16, min_hits=4),
  'avoid_r': dict(bag='run12', t=(0, 1553),    box=(104, 124, -10, 12),  rng=16, min_hits=8),
}
out = {}
for rn, cfg in REG.items():
    if len(sys.argv) > 1 and rn not in sys.argv[1:]: continue
    pts, poses = points(cfg['bag'], *cfg['t'], cfg['box'], cfg['rng'])
    cl, g = clusters(pts, cfg['box'], cfg['min_hits'])
    out[rn] = cl
    a, b, c, d = cfg['box']
    fig, ax = plt.subplots(figsize=(20, 20 * (d - c) / (b - a)))
    ax.imshow(np.log1p(g), origin='lower', extent=cfg['box'], cmap='gray_r', vmax=4)
    ax.plot(poses[:, 0], poses[:, 1], 'b.', ms=1)
    for i, k in enumerate(cl):
        col = 'r' if k['length'] < 0.9 else 'm'
        ax.plot([k['p0'][0], k['p1'][0]], [k['p0'][1], k['p1'][1]], col + '-', lw=1)
        ax.annotate(f"{i}", k['c'], color=col, fontsize=8)
    ax.set_xticks(np.arange(a, b + 1e-6, 1)); ax.set_yticks(np.arange(c, d + 1e-6, 1)); ax.grid(alpha=0.3); ax.tick_params(labelsize=6)
    ax.set_title(f"{rn} {cfg['bag']} t={cfg['t']}")
    fig.savefig(f'cache/cl_{rn}.png', dpi=70, bbox_inches='tight'); plt.close(fig)
    print('==', rn, len(cl))
    for i, k in enumerate(cl):
        print('  %2d c=(%.2f,%.2f) hits=%d len=%.2f wid=%.2f p0=%s p1=%s' % (i, *k['c'], k['hits'], k['length'], k['width'], k['p0'], k['p1']))
json.dump(out, open('cache/clusters.json' if len(sys.argv) == 1 else 'cache/clusters_part.json', 'w'))
