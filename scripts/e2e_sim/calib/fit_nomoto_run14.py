"""2026-11-14 야외 전체 실행(run1_20261114_091623) 으로 Nomoto 계수 다시 맞추기 → cache/nomoto_run14.json.

빈 작업 폴더에서:
  python3 calib/extract.py <run1 bag 폴더> cache/run14   (→ cache/run14.pkl)
  python3 calib/fit_prep.py run14                        (→ cache/run14_dyn.npz)
  python3 calib/fit_nomoto_run14.py [기존 nomoto.json]   (비교용 cache/{dock1,dock2,run12}_dyn.npz 가 있으면 같이 잼)

- 맞춤 구간 = 자동 미션 구간(gate 접근·gate·station·search 접근·search·avoid). 도킹(222–340 s)은 뺀다 —
  슬롯 벽에 닿고 밀려 배 자체 동특성이 아니다.
- 후진 계수 krev 는 오늘 자료에 거의 없어 11-13 도킹 bag 값 0.755 로 고정.
- 모델·잔차 정의는 fit_nomoto.py 와 같다 (선회 잔차 × 2 가중, soft_l1).
"""
import json, os, pickle, sys
import numpy as np
from scipy.optimize import least_squares

DT = 0.02
NAMES = ['a', 'pw', 'krev', 'tau_u', 'K0', 'K1', 'T', 'q', 'tau_dn']
KREV = 0.755
WINDOWS = {'gate접근': (81.8, 130.2), 'gate': (130.2, 160.8), 'station': (160.8, 222.2), 'dock': (222.2, 340.3),
           'search접근': (340.3, 371.6), 'search': (371.6, 521.7), 'avoid': (521.7, 660.7)}   # bag 시각 [s]


def ratio(pwm):
    d = pwm - 1500.0
    return np.sign(d) * np.clip(np.abs(d) - 25.0, 0, None) / 375.0


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


def err(p, d, w, nd=5):
    s, e = w
    u, r = sim(p, ratio(d['L'][s:e]), ratio(d['R'][s:e]), d['u'][s], d['r'][s], nd)
    return u - d['u'][s:e], r - d['r'][s:e]


def segments(d, min_len=8.0, pad=3.0):
    act = d['mode'] > 0
    idx = np.nonzero(act)[0]
    if len(idx) == 0:
        return []
    cuts = np.nonzero(np.diff(idx) > int(2.0 / DT))[0]
    out = []
    for s, e in zip(np.r_[idx[0], idx[cuts + 1]], np.r_[idx[cuts], idx[-1]]):
        e2 = min(e + int(pad / DT), len(act) - 100); s = max(s, 100)
        if (e - s) * DT >= min_len:
            out.append((s, e2))
    return out


def main():
    d = {k: v for k, v in np.load('cache/run14_dyn.npz').items()}
    t0 = pickle.load(open('cache/run14.pkl', 'rb'))['/odom'][0][0]
    # run14_dyn 의 T 는 첫 odom 기준 — bag 시작과 0.005 s 차이라 bag 시각 그대로 쓴다
    W = {k: (int(np.searchsorted(d['T'], a)), int(np.searchsorted(d['T'], b))) for k, (a, b) in WINDOWS.items()}
    fit_w = [w for k, w in W.items() if k != 'dock']
    old = json.load(open(sys.argv[1] if len(sys.argv) > 1 else os.path.join(os.path.dirname(__file__), '..', 'nomoto.json')))
    p_old = [old[k] for k in NAMES]

    def full(q):
        return [q[0], q[1], KREV] + list(q[2:])

    def resid(q):
        out = []
        for w in fit_w:
            du, dr = err(full(q), d, w)
            out += [du[::5], 2 * dr[::5]]
        return np.concatenate(out)

    q0 = [p_old[i] for i in range(9) if i != 2]
    res = least_squares(resid, q0, bounds=([0.5, 0.2, 0.3, 0.1, -2, 0.15, 0.3, 0.3], [5, 1.5, 10, 10, 5, 6, 2.0, 10]),
                        loss='soft_l1', f_scale=0.2, max_nfev=80)
    p_new = full(res.x)
    others = {n: {k: v for k, v in np.load(f'cache/{n}_dyn.npz').items()}
              for n in ('dock1', 'dock2', 'run12') if os.path.exists(f'cache/{n}_dyn.npz')}
    for tag, p in (('기존', p_old), ('오늘 맞춤', p_new)):
        print(tag, ' '.join(f'{k}={v:.3f}' for k, v in zip(NAMES, p)))
        print('  오늘 미션별 RMS 전진/선회:', ' | '.join(
            f'{k} {np.sqrt((err(p, d, w)[0] ** 2).mean()):.2f}/{np.sqrt((err(p, d, w)[1] ** 2).mean()):.2f}'
            for k, w in W.items()))
        for n, dd in others.items():
            es = [err(p, dd, w) for w in segments(dd)]
            print(f'  {n}: {np.sqrt((np.concatenate([e[0] for e in es]) ** 2).mean()):.3f} / '
                  f'{np.sqrt((np.concatenate([e[1] for e in es]) ** 2).mean()):.3f}')
    out = dict(old)
    out.update(dict(zip(NAMES, map(float, p_new))))
    out['delay'] = 0.1
    json.dump(out, open('cache/nomoto_run14.json', 'w'), indent=1)
    print('→ cache/nomoto_run14.json')


if __name__ == '__main__':
    main()
