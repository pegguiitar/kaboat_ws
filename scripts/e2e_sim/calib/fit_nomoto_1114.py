"""2026-11-14 오전·오후 두 번의 야외 전체 실행으로 Nomoto 계수 다시 맞추기 → cache/nomoto_1114.json.

빈 작업 폴더에서:
  python3 calib/extract.py <run1_20261114_091623 폴더> cache/run14
  python3 calib/extract.py <run2_20261114_145941 폴더> cache/run2
  python3 calib/fit_prep.py run14 run2
  python3 calib/fit_nomoto_1114.py [기존 nomoto.json]

- 맞춤 구간 = 각 bag /mission/state 로 나눈 자동 미션 구간(approach·gate·station·search·avoid).
  도킹은 슬롯 벽에 닿고 밀려 배 자체 동특성이 아니라서 뺀다 (undock·done 도 뺌).
- 모델·잔차·후진 계수(krev 0.755 고정)는 fit_nomoto_run14.py 와 같다.
"""
import json, os, pickle, sys
import numpy as np
from scipy.optimize import least_squares

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from fit_nomoto_run14 import NAMES, KREV, err   # noqa: E402

SKIP = ('dock', 'undock', 'done')
BAGS = ('run14', 'run2')


def windows(name, d):
    """bag 의 /mission/state 로 미션 구간 [(이름, s, e)] — d['T'] 는 첫 odom 기준 [s]."""
    D = pickle.load(open(f'cache/{name}.pkl', 'rb'))
    t0 = D['/odom'][0][0]
    st = [(r[0] - t0, r[1]) for r in D['/mission/state']]
    st.append((d['T'][-1] - 2.0, 'end'))
    out = []
    for (a, s), (b, _) in zip(st, st[1:]):
        if s in SKIP or b - a < 4.0:
            continue
        out.append((s, int(np.searchsorted(d['T'], a)), int(np.searchsorted(d['T'], b))))
    return out


def main():
    data = {n: {k: v for k, v in np.load(f'cache/{n}_dyn.npz').items()} for n in BAGS}
    W = {n: windows(n, data[n]) for n in BAGS}
    old = json.load(open(sys.argv[1] if len(sys.argv) > 1 else
                         os.path.join(os.path.dirname(__file__), '..', 'nomoto.json')))
    p_old = [old[k] for k in NAMES]

    def full(q):
        return [q[0], q[1], KREV] + list(q[2:])

    def resid(q):
        out = []
        for n in BAGS:
            for _, s, e in W[n]:
                du, dr = err(full(q), data[n], (s, e))
                out += [du[::5], 2 * dr[::5]]
        return np.concatenate(out)

    q0 = [p_old[i] for i in range(9) if i != 2]
    res = least_squares(resid, q0, bounds=([0.5, 0.2, 0.3, 0.1, -2, 0.15, 0.3, 0.3], [5, 1.5, 10, 10, 5, 6, 2.0, 10]),
                        loss='soft_l1', f_scale=0.2, max_nfev=80)
    p_new = full(res.x)
    for tag, p in (('기존', p_old), ('11-14 오전+오후 맞춤', p_new)):
        print(tag, ' '.join(f'{k}={v:.3f}' for k, v in zip(NAMES, p)))
        for n in BAGS:
            print(f'  {n} RMS 전진/선회:', ' | '.join(
                f'{k} {np.sqrt((err(p, data[n], (s, e))[0] ** 2).mean()):.2f}/'
                f'{np.sqrt((err(p, data[n], (s, e))[1] ** 2).mean()):.2f}' for k, s, e in W[n]))
    out = dict(old)
    out.update(dict(zip(NAMES, map(float, p_new))))
    out['delay'] = 0.1
    json.dump(out, open('cache/nomoto_1114.json', 'w'), indent=1)
    print('→ cache/nomoto_1114.json')


if __name__ == '__main__':
    main()
