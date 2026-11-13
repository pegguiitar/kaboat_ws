#!/usr/bin/env python3
"""녹화한 bag 으로 gate_follower 판단을 다시 돌려 본다 (노트북, 오프라인).

  python3 scripts/replay_gate.py <bag 폴더> --start 15:31:05 --end 15:32:40 [-p gate.이름:=값 …]

rec_bag 에는 /gate/* (고른 쌍·경로·상태)가 없어서, 녹화된 입력(/odom, /scan/level,
/occupancy_grid, /mission/state, /mission/goal)을 지금 코드의 GateFollower 에 녹화 시각대로
넣고 10 Hz 로 compute_cmd 를 부른다. 시계(time.monotonic)는 녹화 시각으로 바꿔 끼운다.
로그·상태 변화를 녹화 시각(한국 시간)과 함께 찍는다.

⚠ 열린 고리: 배는 녹화 당시 실제로 움직인 대로 움직인다(여기서 낸 명령이 배를 바꾸지 않음).
   그 궤적 위에서 무엇을 골랐고 언제 멈추라고 했을지를 본다.
"""

import argparse
import datetime as dt
import os
import sys
import time

KST = dt.timezone(dt.timedelta(hours=9))
TOPICS = ('/mission/state', '/mission/goal', '/odom', '/scan/level', '/occupancy_grid')
HERE = os.path.dirname(os.path.abspath(__file__))
DEFAULT_PARAMS = os.path.join(HERE, '..', 'src', 'kaboat_bringup', 'config', 'gate_params.yaml')


class BagClock:
    """time.monotonic 대신 녹화 시각 [s]."""

    def __init__(self):
        self.now = 0.0

    def __call__(self):
        return self.now


class ReplayLogger:
    """노드 로그를 녹화 시각과 함께 찍는다. throttle 은 녹화 시각 기준."""

    def __init__(self, clock, out=print):
        self.clock, self.out, self._last = clock, out, {}

    def _log(self, level, msg, throttle_duration_sec=None, **_):
        key = (level, str(msg)[:30])
        if throttle_duration_sec and self.clock.now - self._last.get(key, -1e9) < throttle_duration_sec:
            return
        self._last[key] = self.clock.now
        stamp = dt.datetime.fromtimestamp(self.clock.now, KST).strftime('%H:%M:%S.%f')[:10]
        self.out(f'{stamp} {level} {msg}')

    def info(self, msg, **kw):
        self._log('I', msg, **kw)

    def warn(self, msg, **kw):
        self._log('W', msg, **kw)

    warning = warn

    def error(self, msg, **kw):
        self._log('E', msg, **kw)

    def debug(self, msg, **kw):
        pass


def _compressed(bag) -> bool:
    import yaml
    meta = os.path.join(bag, 'metadata.yaml')
    if not os.path.exists(meta):
        return False
    with open(meta, encoding='utf-8') as f:
        info = yaml.safe_load(f).get('rosbag2_bagfile_information', {})
    return bool(info.get('compression_mode'))


def kst_seconds(bag_start_ns, hhmmss):
    day = dt.datetime.fromtimestamp(bag_start_ns * 1e-9, KST)
    h, m, s = (int(v) for v in hhmmss.split(':'))
    return day.replace(hour=h, minute=m, second=s, microsecond=0).timestamp()


def replay(bag, start=None, end=None, params_file=DEFAULT_PARAMS, overrides=(), out=print,
           tick_hz=10.0):
    """bag 구간을 GateFollower 에 넣고 (상태 변화 목록, 노드) 를 돌려준다."""
    import rclpy
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    clock = BagClock()
    time.monotonic = clock                     # behavior_base·gate_follower 가 쓰는 시계
    from kaboat_behaviors.gate_follower import GateFollower

    # 런치와 같은 노드 이름이어야 gate_params.yaml(gate_follower:) 값이 들어간다
    args = ['--ros-args', '-r', '__node:=gate_follower', '--params-file', params_file,
            '-p', 'gate.motion_enabled:=true']
    for item in overrides:
        args += ['-p', item]
    rclpy.init(args=args)
    node = GateFollower()
    logger = ReplayLogger(clock, out)
    node.get_logger = lambda: logger

    # rec_bag 은 압축 영상 토픽이 없으면 메시지마다 zstd 로 녹화한다 — 그때는 압축 읽기
    reader = (rosbag2_py.SequentialCompressionReader() if _compressed(bag)
              else rosbag2_py.SequentialReader())
    reader.open(rosbag2_py.StorageOptions(uri=bag, storage_id='sqlite3'),
                rosbag2_py.ConverterOptions('', ''))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    t_start = kst_seconds(reader.get_metadata().starting_time.nanoseconds, start) if start else None
    t_end = kst_seconds(reader.get_metadata().starting_time.nanoseconds, end) if end else None
    reader.set_filter(rosbag2_py.StorageFilter(topics=[t for t in TOPICS if t in types]))
    handlers = {'/mission/state': node._on_state, '/mission/goal': node._on_goal,
                '/odom': node._on_odom, '/scan/level': node._on_scan,
                '/occupancy_grid': node._on_grid}
    latched = {}
    changes, next_tick, last_status = [], None, None
    try:
        while reader.has_next():
            topic, data, ns = reader.read_next()
            t = ns * 1e-9
            if t_end is not None and t > t_end:
                break
            msg = deserialize_message(data, get_message(types[topic]))
            if t_start is not None and t < t_start:
                if topic in ('/mission/state', '/mission/goal'):
                    latched[topic] = msg      # latched — 구간 시작 때 넣어 준다
                continue
            clock.now = t
            for name in ('/mission/state', '/mission/goal'):
                if name in latched:
                    handlers[name](latched.pop(name))
            handlers[topic](msg)
            if next_tick is None:
                next_tick = t
            while t >= next_tick:
                clock.now = next_tick
                node._tick() if node.active else None
                if node._status_text != last_status:
                    last_status = node._status_text
                    changes.append((next_tick, last_status))
                next_tick += 1.0 / tick_hz
    finally:
        rclpy.shutdown() if rclpy.ok() else None
    return changes, node


def main(argv=None):
    ap = argparse.ArgumentParser(description='bag 으로 gate_follower 판단 다시 돌리기')
    ap.add_argument('bag')
    ap.add_argument('--start', help='한국 시각 HH:MM:SS (그 전 /mission/state·goal 은 그대로 넣음)')
    ap.add_argument('--end', help='한국 시각 HH:MM:SS')
    ap.add_argument('--params', default=DEFAULT_PARAMS, help='gate_params.yaml')
    ap.add_argument('-p', dest='overrides', action='append', default=[],
                    help='파라미터 덮어쓰기 (예: -p gate.reject_hold_sec:=3.0)')
    args = ap.parse_args(argv)
    changes, node = replay(os.path.expanduser(args.bag), args.start, args.end,
                           args.params, args.overrides)
    print('\n== 상태 변화')
    for i, (t, status) in enumerate(changes):
        nxt = changes[i + 1][0] if i + 1 < len(changes) else None
        stamp = dt.datetime.fromtimestamp(t, KST).strftime('%H:%M:%S.%f')[:10]
        dur = f'{nxt - t:5.1f} s' if nxt else '   …'
        print(f'  {stamp} {dur}  {status}')
    print(f'== 통과 {len(node._passed_midpoints)}개: '
          + ', '.join(f'({x:.2f}, {y:.2f})' for x, y in node._passed_midpoints))


if __name__ == '__main__':
    sys.exit(main())
