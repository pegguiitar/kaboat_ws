#!/usr/bin/env python3
"""실주행 bag 캐시(calib/extract.py 의 pkl) → e2e_sim 판 폴더 형식 (trace.jsonl·events.jsonl·arena.json).

같은 analyze.py·plot_missions.py 로 실제 주행을 시뮬 판과 나란히 채점·그림.
rt = 첫 AUTO 시각부터. 위치는 bag /odom (GNSS·EKF, 정답이 아니라 측정값).

  python3 bag_to_run.py <pkl> <arena.json> <판 폴더>
"""
import bisect
import json
import math
import os
import pickle
import sys

pkl, arena, out = sys.argv[1:4]
D = pickle.load(open(pkl, 'rb'))
os.makedirs(out, exist_ok=True)
t0 = D['/odom'][0][0]
st = D['/teleop/status']
t_auto = next(t for t, s in st if s == 'AUTO')
states = D['/mission/state']
st_t = [t for t, _ in states]
cmd = D['/cmd_vel']
cmd_t = [c[0] for c in cmd]
mode_t = [t for t, _ in st]


def at(ts, seq, t):
    i = bisect.bisect_right(ts, t) - 1
    return seq[i] if i >= 0 else None


with open(f'{out}/trace.jsonl', 'w') as f:
    last = -1.0
    for o in D['/odom']:
        t = o[0]
        if t - last < 0.2:
            continue
        last = t
        s = at(st_t, states, t)
        c = at(cmd_t, cmd, t)
        fresh = c is not None and t - c[0] < 0.3
        f.write(json.dumps({'t': round(t - t0, 2), 'rt': round(t - t_auto, 2) if t >= t_auto else None,
                            'x': round(o[2], 3), 'y': round(o[3], 3), 'yaw': round(math.degrees(o[4]), 1),
                            'u': round(o[5], 3), 'r': round(o[7], 3), 'L': None, 'R': None,
                            'cl': round(c[1], 3) if fresh else 0.0, 'ca': round(c[2], 3) if fresh else 0.0,
                            'state': s[1] if s else '', 'mode': at(mode_t, st, t)[1] if at(mode_t, st, t) else ''}) + '\n')

with open(f'{out}/events.jsonl', 'w') as f:
    def ev(t, kind, **kw):
        f.write(json.dumps({'t': round(t - t0, 2), 'rt': round(t - t_auto, 2) if t >= t_auto else None,
                            'kind': kind, **kw}, ensure_ascii=False) + '\n')
    o0 = D['/odom'][0]
    ev(t0, 'start', pose=[o0[2], o0[3], math.degrees(o0[4])], scenario={'source': os.path.basename(pkl)}, seed=None)
    ev(t_auto, 'run_start')
    evs = [(t, 'state', {'value': s}) for t, s in states]
    evs += [(t, 'log', {'node': nm, 'level': lv, 'msg': msg}) for t, nm, lv, msg in D.get('/rosout', [])]
    for topic in ('/search/status', '/station/status', '/gate/status', '/mission/complete'):
        evs += [(t, 'status', {'topic': topic, 'value': s}) for t, s in D.get(topic, [])]
    for t, kind, kw in sorted(evs, key=lambda e: e[0]):
        ev(t, kind, **kw)
    done = next((t for t, s in states if s == 'done'), states[-1][0])
    ev(done + 5.0, 'end', reason='done', run_time=round(done - t_auto, 2))
json.dump(json.load(open(arena)), open(f'{out}/arena.json', 'w'))
print(out, 'AUTO 시작', round(t_auto - t0, 1), 's, done', round(done - t_auto, 1), 's (rt)')
