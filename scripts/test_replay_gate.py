"""replay_gate.py: 녹화 시각 시계·로그 (bag 없이 되는 부분)."""

import datetime as dt
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

import replay_gate as rg  # noqa: E402


def test_logger_stamps_bag_time_and_throttles_by_bag_time():
    clock = rg.BagClock()
    lines = []
    log = rg.ReplayLogger(clock, lines.append)
    t0 = dt.datetime(2026, 11, 12, 15, 31, 27, 400000, tzinfo=rg.KST).timestamp()
    for k in range(5):                                   # 0.3 초 간격으로 5번 (1.2 초)
        clock.now = t0 + 0.3 * k
        log.info('gate: WAIT_GATE_PAIR — 쌍 없음 …', throttle_duration_sec=1.0)
    log.warn('gate: PATH_UNSAFE for 2.0 s; dropping pair')
    assert lines[0] == '15:31:27.4 I gate: WAIT_GATE_PAIR — 쌍 없음 …'
    assert len(lines) == 3                               # 0 초, 1.2 초 + 경고
    assert lines[-1].startswith('15:31:28.6 W gate: PATH_UNSAFE')


def test_kst_seconds_uses_the_bag_day():
    start_ns = int(dt.datetime(2026, 11, 12, 15, 23, 47, tzinfo=rg.KST).timestamp() * 1e9)
    t = rg.kst_seconds(start_ns, '15:31:05')
    assert dt.datetime.fromtimestamp(t, rg.KST) == dt.datetime(2026, 11, 12, 15, 31, 5,
                                                               tzinfo=rg.KST)
