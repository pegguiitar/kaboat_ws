"""여러 조건 × 코드 변형 × 시드로 sim.py 를 병렬로 돌리고 표로 요약.

  python3 scripts/gate_sim/run_matrix.py calib.json gates.json out.jsonl [--old-root DIR] [--seeds 4]

변형: old(--old-root 의 kaboat_behaviors·gate_params.yaml), new(지금 코드), 그 밖 -p 덮어쓰기.
"""
import argparse
import collections
import concurrent.futures as cf
import json
import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
WS = os.path.abspath(os.path.join(HERE, '..', '..'))
NEW_PARAMS = os.path.join(WS, 'src', 'kaboat_bringup', 'config', 'gate_params.yaml')
SCENARIOS = ['base', 'wind10', 'wind20', 'wind20_head', 'weak_lidar', 'clutter3', 'start_off',
             'noisy_pose']


PRESETS = {
    'fixes': [
        ('cur', ['gate.post_visible_timeout_sec:=0.0', 'gate.hold_thrust_ratio:=0.0']),
        ('A:보임1s', ['gate.hold_thrust_ratio:=0.0']),
        ('A+B:자리지킴', []),
        ('A+B+C:덩어리2m', ['gate.max_cluster_extent:=2.0']),
    ],
    'now': [('지금', [])],
    'final': [
        ('커밋된 코드', ['gate.post_visible_timeout_sec:=0.0', 'gate.max_cluster_extent:=1.2']),
        ('지금(A+C)', []),
    ],
    'stops': [
        ('이전(10s,그대로)', ['gate.reject_hold_sec:=10.0', 'gate.replan_from_here:=false']),
        ('대기3s', ['gate.replan_from_here:=false']),
        ('대기3s+다시그림', []),
    ],
    'fixes2': [
        ('A:보임1s', ['gate.hold_thrust_ratio:=0.0']),
        ('A+B2:자리0.25', []),
        ('A+B2:자리0.15', ['gate.hold_thrust_ratio:=0.15']),
    ],
}


def variants(old_root, preset=None):
    out = []
    if preset:
        if old_root:
            out.append(('old', {'PYTHONPATH': os.path.join(old_root, 'src', 'kaboat_behaviors')},
                        os.path.join(old_root, 'src', 'kaboat_bringup', 'config',
                                     'gate_params.yaml'), []))
        return out + [(label, {}, NEW_PARAMS, ov) for label, ov in PRESETS[preset]]
    if old_root:
        out.append(('old', {'PYTHONPATH': os.path.join(old_root, 'src', 'kaboat_behaviors')},
                    os.path.join(old_root, 'src', 'kaboat_bringup', 'config', 'gate_params.yaml'),
                    []))
    out += [
        ('new', {}, NEW_PARAMS, []),
        ('new+ext2.0', {}, NEW_PARAMS, ['gate.max_cluster_extent:=2.0']),
        ('new+thrust0.4', {}, NEW_PARAMS, ['gate.los_thrust_ratio:=0.4']),
    ]
    return out


def run_one(job):
    label, env_add, params, overrides, scenario, seed, calib, gates = job
    env = dict(os.environ)
    for k, v in env_add.items():
        env[k] = v + os.pathsep + env.get(k, '')
    cmd = [sys.executable, os.path.join(HERE, 'sim.py'), calib, gates, '--params', params,
           '--scenario', scenario, '--seed', str(seed), '--label', label]
    for o in overrides:
        cmd += ['-p', o]
    p = subprocess.run(cmd, env=env, capture_output=True, text=True)
    line = [ln for ln in p.stdout.splitlines() if ln.startswith('{')]
    if not line:
        return {'label': label, 'scenario': scenario, 'seed': seed, 'error': p.stderr[-400:]}
    return json.loads(line[-1])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('calib')
    ap.add_argument('gates')
    ap.add_argument('out')
    ap.add_argument('--old-root')
    ap.add_argument('--seeds', type=int, default=4)
    ap.add_argument('--jobs', type=int, default=14)
    ap.add_argument('--scenarios', default=','.join(SCENARIOS))
    ap.add_argument('--preset', choices=sorted(PRESETS))
    args = ap.parse_args()
    jobs = [(label, env, params, ov, sc, seed, args.calib, args.gates)
            for label, env, params, ov in variants(args.old_root, args.preset)
            for sc in args.scenarios.split(',') for seed in range(args.seeds)]
    results = []
    with cf.ProcessPoolExecutor(args.jobs) as ex, open(args.out, 'w') as f:
        for r in ex.map(run_one, jobs):
            results.append(r)
            f.write(json.dumps(r, ensure_ascii=False) + '\n')
            f.flush()
    errors = [r for r in results if 'error' in r]
    for r in errors[:3]:
        print('오류', r['label'], r['scenario'], r['error'])
    table = collections.defaultdict(list)
    for r in results:
        if 'error' not in r:
            table[(r['scenario'], r['label'])].append(r)
    print(f"{'조건':12s} {'코드':14s} 완료  7개순서  시간(s)  정지(s)  최소거리(m)  게이트")
    for sc in args.scenarios.split(','):
        for label, *_ in variants(args.old_root, args.preset):
            rs = table.get((sc, label), [])
            if not rs:
                continue
            ok = sum(r['complete'] for r in rs)
            order = sum(r['in_order'] and r['passed'] == 7 for r in rs)
            t = sum(r['time'] for r in rs) / len(rs)
            st = sum(r['stop_time'] for r in rs) / len(rs)
            mc = min(r['min_clear'] for r in rs)
            g = ' '.join(''.join(str(x) for x in r['gates']) or '-' for r in rs)
            print(f'{sc:12s} {label:14s} {ok}/{len(rs)}   {order}/{len(rs)}    {t:6.1f}  {st:6.1f}    {mc:5.2f}     {g}')


if __name__ == '__main__':
    main()
