import json
import os, sys, re
def show(run, t0=0, t1=1e9, what='dock,search,mm,contact,station'):
    print('==', run)
    last = None; prev = None; cnt = 0
    RUNS = os.environ.get('KABOAT_E2E_OUT', '/tmp/kaboat_e2e_runs')
    for l in open(f'{RUNS}/{run}/events.jsonl'):
        e = json.loads(l)
        if e['rt'] is None or not (t0 <= e['rt'] <= t1): continue
        if e['kind'] == 'dock' and 'dock' in what:
            k = (e['phase'], re.sub(r'[-+]?\d+\.?\d*', '#', e['detail'])[:22])
            if k != last:
                print('  %.0f %s %s' % (e['rt'], e['phase'], e['detail'][:150])); last = k
        elif e['kind'] == 'status' and e['topic'] == '/search/status' and 'search' in what: print('  %.0f search: %s' % (e['rt'], e['value'][:150]))
        elif e['kind'] == 'status' and e['topic'] == '/station/status' and 'station' in what: print('  %.0f station: %s' % (e['rt'], e['value'][:110]))
        elif e['kind'] == 'status' and e['topic'] == '/mission/give_up': print('  %.0f GIVE_UP %s' % (e['rt'], e['value'][:120]))
        elif e['kind'] == 'log' and e['node'] == 'mission_manager' and 'mm' in what: print('  %.0f MM %s' % (e['rt'], e['msg'][:150]))
        elif e['kind'] == 'contact_start' and 'contact' in what: print('  %.0f CONTACT %s %s' % (e['rt'], e['obj'], e['pose']))
if __name__ == '__main__':
    show(sys.argv[1], float(sys.argv[2]) if len(sys.argv) > 2 else 0, float(sys.argv[3]) if len(sys.argv) > 3 else 1e9, sys.argv[4] if len(sys.argv) > 4 else 'dock,search,mm,contact,station')
