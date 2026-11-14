import numpy as np, json, math
from scipy.optimize import least_squares
DT = 0.02
def load(n):
    z = np.load(f'cache/{n}_dyn.npz'); return {k: z[k] for k in z.files}
# run14 = run1_20261114_091623 (2026-11-14 실주행, 자율 598 s) — 2026-11-14 추가. 예전 3 개 맞춤은 nomoto_20261113.json
data = {n: load(n) for n in ('dock1', 'dock2', 'run12', 'run14')}
def ratio(pwm):
    d = pwm - 1500.0
    return np.sign(d) * np.clip(np.abs(d) - 25.0, 0, None) / 375.0
def segments(d, min_len=8.0, pad=3.0):
    """구동(mode>0) 구간 + 뒤 pad 초 (감속 포함)."""
    act = d['mode'] > 0
    idx = np.nonzero(act)[0]
    if len(idx) == 0: return []
    cuts = np.nonzero(np.diff(idx) > int(2.0 / DT))[0]
    starts = np.r_[idx[0], idx[cuts + 1]]; ends = np.r_[idx[cuts], idx[-1]]
    out = []
    for s, e in zip(starts, ends):
        e2 = min(e + int(pad / DT), len(act) - 100)
        s = max(s, 100)
        if (e - s) * DT >= min_len: out.append((s, e2))
    return out
SEG = {n: segments(d) for n, d in data.items()}
print({n: (len(s), round(sum(e - b for b, e in s) * DT)) for n, s in SEG.items()})
def sim(p, fL, fR, u0, r0, nd):
    a, pw, krev, tau_u, K0, K1, T, q, tau_dn = p
    n = len(fL); u = np.empty(n); r = np.empty(n); uu, rr = u0, r0
    fb = (fL + fR) / 2; de = (fR - fL) / 2
    Uss = np.where(fb >= 0, a * np.abs(fb) ** pw, -a * krev * np.abs(fb) ** pw)
    Rin = np.sign(de) * np.abs(de) ** q
    for i in range(n):
        j = i - nd if i >= nd else 0
        uu += DT * (Uss[j] - uu) / (tau_u if abs(Uss[j]) >= abs(uu) else tau_dn)
        rr += DT * ((K0 + K1 * abs(uu)) * Rin[j] - rr) / T
        u[i] = uu; r[i] = rr
    return u, r
def resid(p, nd, sub=5, which='both'):
    out = []
    for n, d in data.items():
        fL, fR = ratio(d['L']), ratio(d['R'])
        for s, e in SEG[n]:
            u, r = sim(p, fL[s:e], fR[s:e], d['u'][s], d['r'][s], nd)
            if which in ('both', 'u'): out.append((u - d['u'][s:e])[::sub])
            if which in ('both', 'r'): out.append(((r - d['r'][s:e]) * 2.0)[::sub])
    return np.concatenate(out)
best = None
for delay in (0.1, 0.2):
    nd = int(delay / DT)
    p0 = [2.0, 0.6, 0.6, 1.0, 2.2, -1.0, 0.6, 1.0, 2.0]
    lo = [0.5, 0.2, 0.2, 0.3, 0.1, -2, 0.15, 0.3, 0.3]; hi = [5, 1.5, 1.5, 10, 10, 5, 6, 2.0, 10]
    res = least_squares(resid, p0, bounds=(lo, hi), args=(nd,), loss='soft_l1', f_scale=0.2, max_nfev=60)
    ru = resid(res.x, nd, which='u'); rr = resid(res.x, nd, which='r') / 2
    print('delay %.1f cost %.2f rms_u %.3f rms_r %.3f p=%s' % (delay, res.cost, np.sqrt((ru**2).mean()), np.sqrt((rr**2).mean()), np.round(res.x, 3)))
    if best is None or res.cost < best[0]: best = (res.cost, delay, res.x)
cost, delay, p = best
names = ['a', 'pw', 'krev', 'tau_u', 'K0', 'K1', 'T', 'q', 'tau_dn']
out = dict(zip(names, map(float, p))); out['delay'] = delay
# 표류: 모터 중립(idle) 3 s 뒤의 대지 속도 (dock1·dock2 만 — 모드 기록 있음). run14 는 뺀다: 주행 뒤(681 s~)
# 텔레옵 끊김 구간에 배를 끌거나 들어 0.6–1.2 m/s 로 움직였고, 출발 전 중립 표류는 0.01 m/s (물살 거의 없음)
drift = []
for n in ('dock1', 'dock2'):
    d = data[n]; idle = d['mode'] == 0
    run = np.zeros(len(idle)); c = 0
    for i, v in enumerate(idle):
        c = c + 1 if v else 0; run[i] = c
    m = run > int(8.0 / DT)
    vx = d['u'] * np.cos(d['yaw']) - d['v'] * np.sin(d['yaw']); vy = d['u'] * np.sin(d['yaw']) + d['v'] * np.cos(d['yaw'])
    sp = np.hypot(vx[m], vy[m]); drift.append(sp)
    print(n, 'idle>8s: n=%d s, drift speed p10/50/90/99 = %s, mean vec (%.3f, %.3f), |r| p50/p90 %.3f %.3f' % (m.sum() * DT, np.round(np.percentile(sp, [10, 50, 90, 99]), 3), vx[m].mean(), vy[m].mean(), *np.percentile(np.abs(d['r'][m]), [50, 90])))
out['drift_p50'] = float(np.median(np.concatenate(drift))); out['drift_p90'] = float(np.percentile(np.concatenate(drift), 90))
json.dump(out, open('cache/nomoto.json', 'w'), indent=1); print(out)
# 그림: 구간 몇 개
import matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
nd = int(delay / DT)
fig, axs = plt.subplots(4, 2, figsize=(18, 14))
k = 0
for n in ('dock1', 'dock2'):
    d = data[n]; fL, fR = ratio(d['L']), ratio(d['R'])
    segs = sorted(SEG[n], key=lambda s: s[0] - s[1])[:2]
    for s, e in segs:
        u, r = sim(p, fL[s:e], fR[s:e], d['u'][s], d['r'][s], nd); t = d['T'][s:e]
        axs[k, 0].plot(t, d['u'][s:e], 'k', lw=0.8, label='bag u'); axs[k, 0].plot(t, u, 'r', lw=0.8, label='model u'); axs[k, 0].plot(t, (fL[s:e] + fR[s:e]) / 2, 'b', lw=0.4, label='fbar'); axs[k, 0].legend(); axs[k, 0].set_title(f'{n} {t[0]:.0f}-{t[-1]:.0f}s surge')
        axs[k, 1].plot(t, d['r'][s:e], 'k', lw=0.8, label='bag r'); axs[k, 1].plot(t, r, 'r', lw=0.8, label='model r'); axs[k, 1].plot(t, (fR[s:e] - fL[s:e]) / 2, 'b', lw=0.4, label='delta'); axs[k, 1].legend(); axs[k, 1].set_title('yaw rate')
        k += 1
fig.savefig('cache/nomoto_fit.png', dpi=70, bbox_inches='tight')
