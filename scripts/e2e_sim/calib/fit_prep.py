"""bag → 50 Hz 격자: 좌우 PWM(실제 믹서 재현), 모드, u, v, r."""
import pickle, numpy as np, math, sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[3] / 'src' / 'kaboat_hardware'))
from kaboat_hardware.thruster_backend import ThrusterConfig, ThrusterMixer
DT = 0.02
def prep(name):
    D = pickle.load(open(f'cache/{name}.pkl', 'rb'))
    od = D['/odom']; t0 = od[0][0]
    to = np.array([o[0] - t0 for o in od]); x = np.array([o[2] for o in od]); y = np.array([o[3] for o in od])
    yaw = np.unwrap(np.array([o[4] for o in od])); vx = np.array([o[5] for o in od]); vy = np.array([o[6] for o in od]); wz = np.array([o[7] for o in od])
    T = np.arange(0, to[-1], DT)
    # 위치 미분으로 본 속도 (세계) → 선체
    xs = np.interp(T, to, x); ys = np.interp(T, to, y); yw = np.interp(T, to, yaw)
    k = 25
    dvx = (np.roll(xs, -k) - np.roll(xs, k)) / (2 * k * DT); dvy = (np.roll(ys, -k) - np.roll(ys, k)) / (2 * k * DT)
    u_pos = dvx * np.cos(yw) + dvy * np.sin(yw); v_pos = -dvx * np.sin(yw) + dvy * np.cos(yw)
    r_pos = (np.roll(yw, -k) - np.roll(yw, k)) / (2 * k * DT)
    u_tw = np.interp(T, to, vx); v_tw = np.interp(T, to, vy); r_tw = np.interp(T, to, wz)
    print(name, 'twist vs pos-derived: corr u', np.corrcoef(u_tw[100:-100], u_pos[100:-100])[0, 1], 'r', np.corrcoef(r_tw[100:-100], r_pos[100:-100])[0, 1],
          'mean|u_tw|', np.abs(u_tw).mean(), 'mean|u_pos|', np.abs(u_pos).mean())
    cmd = D['/cmd_vel']; tc = np.array([c[0] - t0 for c in cmd]); cl = np.array([c[1] for c in cmd]); ca = np.array([c[2] for c in cmd])
    st = D.get('/teleop/status', []); tst = np.array([s[0] - t0 for s in st]); modes = [s[1] for s in st]
    lv = D.get('/teleop/levels', []); tl = np.array([l[0] - t0 for l in lv]); lvv = np.array([l[1] for l in lv]) if lv else np.zeros((0, 2))
    cfg = ThrusterConfig(); mixer = ThrusterMixer(cfg)
    L = np.full(len(T), 1500.0); R = np.full(len(T), 1500.0); mode = np.zeros(len(T), int)  # 0 idle 1 auto 2 teleop
    ic = np.searchsorted(tc, T, side='right') - 1
    is_ = np.searchsorted(tst, T, side='right') - 1 if len(tst) else np.full(len(T), -1)
    il = np.searchsorted(tl, T, side='right') - 1 if len(tl) else np.full(len(T), -1)
    for i, t in enumerate(T):
        m = modes[is_[i]] if is_[i] >= 0 else ('AUTO' if name == 'run12' else 'TIMEOUT')
        if m == 'TELEOP' and il[i] >= 0 and t - tl[il[i]] < 0.5:
            level, turn = int(lvv[il[i]][0]), int(lvv[il[i]][1])
            base = 1500 + (0 if level == 0 else [50, 100, 150, 200][abs(level) - 1] * (1 if level > 0 else -1))
            L[i] = base - turn * 150; R[i] = base + turn * 150; mode[i] = 2; mixer.reset()
        elif m in ('AUTO',) and ic[i] >= 0 and t - tc[ic[i]] <= 0.3:
            l, r, _, _ = mixer.mix(cl[ic[i]], ca[ic[i]], t); L[i] = l; R[i] = r; mode[i] = 1
        else:
            mixer.reset()
    np.savez(f'cache/{name}_dyn.npz', T=T, L=L, R=R, mode=mode, u=u_pos, v=v_pos, r=r_pos, u_tw=u_tw, r_tw=r_tw, yaw=yw, x=xs, y=ys)
    for mm, nm in ((0, 'idle'), (1, 'auto'), (2, 'teleop')):
        print('  ', nm, round((mode == mm).mean() * T[-1]), 's')
    a = mode == 1
    print('   auto PWM common range', np.percentile((L[a] + R[a]) / 2, [1, 50, 99]), 'diff', np.percentile((R[a] - L[a]) / 2, [1, 50, 99]))
for n in (sys.argv[1:] or ('run12', 'dock1', 'dock2')): prep(n)   # 예: python3 fit_prep.py run14
