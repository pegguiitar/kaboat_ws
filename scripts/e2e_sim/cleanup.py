"""시뮬 판에 속한 프로세스만 정리 — 환경 변수 HOME 이 결과 폴더($KABOAT_E2E_OUT, 기본 /tmp/kaboat_e2e_runs) 아래인 것 (인자: 판 이름, 없으면 전부)."""
import os, signal, sys
mark = os.environ.get('KABOAT_E2E_OUT', '/tmp/kaboat_e2e_runs').rstrip('/') + '/' + (sys.argv[1] + '/' if len(sys.argv) > 1 else '')
n = 0
for pid in filter(str.isdigit, os.listdir('/proc')):
    try:
        env = open(f'/proc/{pid}/environ', 'rb').read().split(b'\0')
    except OSError:
        continue
    if any(e.startswith(b'HOME=') and mark.encode() in e for e in env) and int(pid) != os.getpid():
        try:
            os.kill(int(pid), signal.SIGKILL); n += 1
        except OSError:
            pass
print('killed', n)
