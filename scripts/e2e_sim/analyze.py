#!/usr/bin/env python3
"""한 판(runs/<이름>) 정답 채점 — 스택이 '완료'라고 한 것과 별개로 궤적으로 본다.

  gate    게이트 i 의 두 부표 사이 선분을 진행 방향으로 건넌 순서
  station 부표 사각형 안(또는 부표 5 m 안)에 연속으로 머문 최장 시간
  dock    목표 슬롯 안(뒷벽에서 깊이 안, 폭 안)에 선체 중심이 들어갔나, 다른 슬롯에 들어갔나
  search  목표 부표 둘레 누적 선회각 (시계 = 우현 선회 +)
  avoid   입구 → 출구를 지났나, 경계 밖으로 나갔나
출력: summary.json + 궤적 그림 (--plot).
"""
import json
import os
import math
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.environ.get('KABOAT_E2E_OUT', '/tmp/kaboat_e2e_runs')
MISSIONS = ['gate', 'station', 'dock', 'search', 'avoid']


def seg_cross(p, q, a, b):
    """선분 pq 가 선분 ab 를 건너면 True."""
    def o(u, v, w):
        return (v[0] - u[0]) * (w[1] - u[1]) - (v[1] - u[1]) * (w[0] - u[0])
    return o(p, q, a) * o(p, q, b) < 0 and o(a, b, p) * o(a, b, q) < 0


def in_poly(pt, poly):
    x, y = pt
    inside = False
    n = len(poly)
    for i in range(n):
        x1, y1 = poly[i]
        x2, y2 = poly[(i + 1) % n]
        if (y1 > y) != (y2 > y) and x < (x2 - x1) * (y - y1) / (y2 - y1) + x1:
            inside = not inside
    return inside


def analyze(name, plot=False, target_color=None, target_shape=None):
    run = f'{OUT}/{name}'
    # run_one.sh 가 판 폴더에 복사한 경기장 (예전 판은 없으므로 11-13 arena.json)
    arena = json.load(open(f'{run}/arena.json' if os.path.exists(f'{run}/arena.json') else f'{HERE}/arena.json'))
    target_color = target_color or arena.get('targets', {}).get('search_color', 'red')
    target_shape = target_shape or arena.get('targets', {}).get('dock_class', 'circle')
    tr = [json.loads(l) for l in open(f'{run}/trace.jsonl') if l.strip()]
    ev = [json.loads(l) for l in open(f'{run}/events.jsonl') if l.strip()]
    tr = [r for r in tr if r['rt'] is not None]
    T = np.array([r['rt'] for r in tr])
    X = np.array([(r['x'], r['y']) for r in tr])
    st = [r['state'] for r in tr]
    S = {'name': name, 'scenario': next((e.get('scenario') for e in ev if e['kind'] == 'start'), {})}

    # ── 미션 매니저가 본 진행 ──
    phases = []          # (rt, state)
    for e in ev:
        if e['kind'] == 'state':
            phases.append((e['rt'] if e['rt'] is not None else 0.0, e['value']))
    # 단독 미션 판은 모터가 돌기 전(rt None)에 접근 로그가 찍힌다 → 0 으로
    logs = [(e['rt'] if e['rt'] is not None else 0.0, e['msg']) for e in ev
            if e['kind'] == 'log' and e['node'] == 'mission_manager']
    mm = {m: {} for m in MISSIONS}
    cur = None
    for rt, msg in logs:
        for m in MISSIONS:
            if f"'{m}' 접근" in msg:
                mm[m]['approach_at'] = rt
                cur = m
            if f"'{m}' 시작 —" in msg:
                mm[m]['start_at'] = rt
            if msg.startswith('behavior 완료 +') and f"'{m}' 완료" in msg:
                mm[m]['done_at'] = rt              # 미션 매니저가 완료로 넘김
            if f"behavior '{m}' 완료 신호" in msg:
                mm[m]['behavior_done_at'] = rt     # behavior 는 끝냈다고 알림 (끝 지점 2 m 안은 아직)
            if msg.startswith(f"'{m}' ") and '건너뜀' in msg:
                mm[m]['skipped_at'] = rt
                mm[m]['skip_why'] = msg[:160]
        if '종합임무 전체 완료' in msg:
            S['all_done_at'] = rt
            S['done_msg'] = msg[:300]
        if '후진 끝' in msg:
            S['retreat_end'] = (rt, msg[:120])
    end = next((e for e in ev if e['kind'] == 'end'), None)
    S['end'] = {'reason': end['reason'], 'run_time': end['run_time']} if end else {'reason': 'killed', 'run_time': float(T[-1])}
    S['mission_manager'] = mm
    S['phases'] = phases

    # ── 정답: gate ──
    crossed = []
    for i in range(1, len(X)):
        for g, (a, b) in enumerate(arena['gates']):
            if seg_cross(X[i - 1], X[i], a, b):
                crossed.append((round(float(T[i]), 1), g + 1))
    S['gate_crossings'] = crossed
    order = [g for _, g in crossed]
    S['gate_ok'] = order[:7] == list(range(1, 8))
    # ── 정답: station ──
    poly = arena['station_poly']
    buoys = np.array(poly)
    inside = np.array([in_poly(p, poly) for p in X])
    near5 = np.array([np.min(np.hypot(*(buoys - p).T)) <= 5.0 for p in X])
    slow = np.array([abs(r['u']) < 0.3 for r in tr])
    def longest(mask):
        best = cur_ = 0.0
        t0 = None
        for i, m in enumerate(mask):
            if m:
                t0 = T[i] if t0 is None else t0
                cur_ = T[i] - t0
                best = max(best, cur_)
            else:
                t0 = None
        return round(float(best), 1)
    in_station = np.array([s == 'station' for s in st])
    S['station_hold_inside_s'] = longest(inside & in_station)
    S['station_hold_near5_slow_s'] = longest(near5 & slow & in_station)
    # ── 정답: dock ──
    d = arena['dock']
    n, t = np.array(d['n']), np.array(d['t'])
    S['dock_slots_entered'] = {}
    for shape, c in d['slots'].items():
        rel = X - np.array(c)
        a, b = rel @ t, rel @ n
        m = (np.abs(a) < d['slot_width'] / 2) & (b > 0) & (b < d['depth'])
        if m.any():
            S['dock_slots_entered'][shape] = {'first_at': round(float(T[m][0]), 1), 'seconds': round(float(m.sum() * 0.2), 1),
                                              'deepest_from_wall': round(float(b[m].min()), 2)}
    S['dock_ok'] = target_shape in S['dock_slots_entered'] and len(S['dock_slots_entered']) == 1
    # ── 정답: search ──
    S['search_orbit_deg'] = {}
    for col, c in arena['search'].items():
        rel = X - np.array(c)
        dist = np.hypot(rel[:, 0], rel[:, 1])
        ang = np.unwrap(np.arctan2(rel[:, 1], rel[:, 0]))
        m = (dist < 5.0) & np.array([s == 'search' for s in st])
        tot = 0.0
        for i in range(1, len(ang)):
            if m[i] and m[i - 1]:
                tot += ang[i] - ang[i - 1]
        S['search_orbit_deg'][col] = round(-math.degrees(tot), 0)     # + = 시계(우현 선회)
    S['search_ok'] = abs(S['search_orbit_deg'].get(target_color, 0)) >= 340   # 검정은 반시계(−)
    # ── 정답: avoid ──
    av = arena['avoid']
    t_in = [round(float(T[i]), 1) for i in range(1, len(X)) if seg_cross(X[i - 1], X[i], *av['entry_gate'])]
    t_out = [round(float(T[i]), 1) for i in range(1, len(X)) if seg_cross(X[i - 1], X[i], *av['exit_gate'])]
    c = av['corners']
    sides = {'south': (c[0], c[1]), 'north': (c[2], c[3]), 'east': (c[1], c[2]), 'west': (c[3], c[0])}
    side_cross = {k: [round(float(T[i]), 1) for i in range(1, len(X)) if seg_cross(X[i - 1], X[i], *v)] for k, v in sides.items()}
    S['avoid_entry_cross'] = t_in
    S['avoid_exit_cross'] = t_out
    S['avoid_side_cross'] = {k: v for k, v in side_cross.items() if v}
    S['avoid_ok'] = bool(t_in) and bool(t_out) and not side_cross['south'] and not side_cross['north']
    # ── 접촉 ──
    contacts = {}
    for e in ev:
        if e['kind'] == 'contact_start':
            contacts.setdefault(e['obj'], []).append((e['rt'], e.get('state')))
    S['contacts'] = {k: {'n': len(v), 'first_rt': v[0][0], 'state': v[0][1]} for k, v in contacts.items()}
    # ── 미션별 소요 ──
    S['times'] = {}
    for m in MISSIONS:
        q = mm[m]
        if 'approach_at' in q:
            endt = q.get('done_at', q.get('skipped_at'))
            S['times'][m] = {'approach_s': round(q['start_at'] - q['approach_at'], 1) if 'start_at' in q else None,
                             'mission_s': round(endt - q['start_at'], 1) if endt is not None and 'start_at' in q else None,
                             'result': 'done' if 'done_at' in q else (('skipped_after_behavior_done' if 'behavior_done_at' in q else 'skipped') if 'skipped_at' in q else 'unfinished')}
        elif 'skipped_at' in q:
            S['times'][m] = {'result': 'skipped', 'approach_s': None, 'mission_s': None}
    S['path_len_m'] = round(float(np.hypot(*np.diff(X, axis=0).T).sum()), 1)
    json.dump(S, open(f'{run}/summary.json', 'w'), ensure_ascii=False, indent=1)
    if plot:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(figsize=(22, 13))
        col = {'red': 'r', 'green': 'g', 'black': 'k', 'orange': 'orange', 'white': 'gray', 'yellow': 'y'}
        for c_ in arena['circles']:
            ax.add_patch(plt.Circle((c_['x'], c_['y']), c_['r'], color=col[c_['color']]))
        for s in arena['segments']:
            ax.plot([s['p0'][0], s['p1'][0]], [s['p0'][1], s['p1'][1]], 'm-', lw=3)
        for p in arena['polys']:
            ax.add_patch(plt.Polygon(p['pts'], color='b', alpha=0.5))
        cmap = {'undock': 'k', 'approach': 'gray', 'gate': 'tab:blue', 'station': 'tab:orange', 'dock': 'tab:purple',
                'retreat': 'brown', 'search': 'tab:red', 'avoid': 'tab:green', 'done': 'k'}
        i0 = 0
        for i in range(1, len(st) + 1):
            if i == len(st) or st[i] != st[i0]:
                ax.plot(X[i0:i + 1, 0], X[i0:i + 1, 1], '-', color=cmap.get(st[i0], 'k'), lw=1.3, label=st[i0])
                i0 = i
        h, l = ax.get_legend_handles_labels()
        uniq = dict(zip(l, h))
        ax.legend(uniq.values(), uniq.keys(), loc='upper left')
        for k, (x, y, a) in arena['points'].items():
            ax.plot(x, y, 'k+')
            ax.annotate(k, (x, y), fontsize=7)
        ax.set_aspect('equal')
        ax.set_xlim(min(92, X[:, 0].min() - 2), 182)
        ax.set_ylim(-58, 14)
        ax.grid(alpha=0.3)
        ax.set_title(f"{name}  end {S['end']}")
        fig.savefig(f'{run}/path.png', dpi=60, bbox_inches='tight')
        plt.close(fig)
    return S


def brief(S):
    t = S['times']
    row = [S['name'], f"{S['end']['reason']} {S['end']['run_time']:.0f}s" if S['end']['run_time'] else S['end']['reason']]
    for m in MISSIONS:
        q = t.get(m)
        row.append('-' if q is None else f"{q['result']}({q['approach_s']}/{q['mission_s']})")
    truth = (f"gate {'OK' if S['gate_ok'] else [g for _, g in S['gate_crossings']]} | station in {S['station_hold_inside_s']}s near5 {S['station_hold_near5_slow_s']}s | "
             f"dock {S['dock_slots_entered']} | orbit {S['search_orbit_deg']} | avoid in{S['avoid_entry_cross']} out{S['avoid_exit_cross']} side{S['avoid_side_cross']} | "
             f"contacts {S['contacts']}")
    return ' | '.join(row) + '\n    ' + truth


if __name__ == '__main__':
    for nm in sys.argv[1:]:
        try:
            print(brief(analyze(nm, plot=True)))
        except Exception as exc:       # 아직 도는 판
            print(nm, 'ERR', repr(exc))
