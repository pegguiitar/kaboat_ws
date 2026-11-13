import json, numpy as np, sys
from scipy import ndimage
from matplotlib.path import Path
def bottleneck(A, verbose=True):
    RES = 0.05; X0, X1, Y0, Y1 = 104, 144, -9, 11
    W, H = int((X1 - X0) / RES), int((Y1 - Y0) / RES)
    occ = np.zeros((H, W), bool)
    yy, xx = np.mgrid[0:H, 0:W]; px = X0 + (xx + .5) * RES; py = Y0 + (yy + .5) * RES
    def capsule(a, b, r):
        a, b = np.array(a, float), np.array(b, float); d = b - a; L2 = max(float(d @ d), 1e-9)
        t = np.clip(((px - a[0]) * d[0] + (py - a[1]) * d[1]) / L2, 0, 1)
        return np.hypot(px - (a[0] + t * d[0]), py - (a[1] + t * d[1])) <= r
    for c in A['circles']:
        if c['name'].startswith('avoid'): occ |= np.hypot(px - c['x'], py - c['y']) <= c['r']
    for s in A['segments']: occ |= capsule(s['p0'], s['p1'], s['hw'])
    names = {c['name']: (c['x'], c['y']) for c in A['circles']}
    def chain(prefix, start, end):
        global_pts = [start] + [names[n] for n in sorted((n for n in names if n.startswith(prefix)), key=lambda n: int(n[len(prefix):]))] + [end]
        m = np.zeros_like(occ)
        for a, b in zip(global_pts[:-1], global_pts[1:]): m |= capsule(a, b, 0.08)
        return m
    SW, SE, NE, NW = A['avoid']['corners']; gE, gW = A['avoid']['entry_gate'], A['avoid']['exit_gate']
    for pre, a, b in (('avoid_southline', SW, SE), ('avoid_northline', NW, NE), ('avoid_eastlineS', SE, gE[0]), ('avoid_eastlineN', gE[1], NE), ('avoid_westlineS', SW, gW[0]), ('avoid_westlineN', gW[1], NW)):
        occ |= chain(pre, a, b)
    # 구역 밖은 막음 (입구 동쪽·출구 서쪽 통로만 열어 둠)
    poly = Path([(SW[0], SW[1] - 1.5), (SE[0], SE[1] - 1.5), (NE[0], NE[1] + 1.5), (NW[0], NW[1] + 1.5)])
    inside = poly.contains_points(np.c_[px.ravel(), py.ravel()]).reshape(H, W)
    east = (px > 136.0) & (py > gE[0][1] - 0.5) & (py < gE[1][1] + 0.5)
    west = (px < 111.5) & (py > gW[0][1] - 0.5) & (py < gW[1][1] + 0.5)
    occ |= ~(inside | east | west)
    dist = ndimage.distance_transform_edt(~occ) * RES
    cell = lambda x, y: (int((y - Y0) / RES), int((x - X0) / RES))
    s = cell(140.0, -1.1); g = cell(107.0, 2.7)
    lo, hi = 0.0, 3.0
    for _ in range(14):
        mid = (lo + hi) / 2
        lab, n = ndimage.label(dist >= mid)
        if lab[s] and lab[s] == lab[g]: lo = mid
        else: hi = mid
    lab, n = ndimage.label(dist >= lo + 0.03)
    reach = lab == lab[s] if lab[s] else np.zeros_like(occ)
    west_x = X0 + np.nonzero(reach.any(0))[0].min() * RES if reach.any() else None
    # 반경별 도달 가능한 가장 서쪽
    out = {'bottleneck_radius': round(lo, 2), 'passage_width': round(2 * lo, 2), 'westmost_if_slightly_wider': west_x}
    for r in (0.35, 0.5, 0.6, 0.75, 1.0):
        lab, n = ndimage.label(dist >= r)
        ok = bool(lab[s] and lab[s] == lab[g]); reach = lab == lab[s] if lab[s] else np.zeros_like(occ)
        out[f'r{r}'] = ('통과' if ok else '막힘 (서쪽 끝 x=%.1f)' % (X0 + np.nonzero(reach.any(0))[0].min() * RES)) if reach.any() else '입구부터 막힘'
    return out
if __name__ == '__main__':
    A = json.load(open(sys.argv[1] if len(sys.argv) > 1 else 'sim/arena.json'))
    print(json.dumps(bottleneck(A), ensure_ascii=False, indent=1))
