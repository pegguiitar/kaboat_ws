#!/usr/bin/env python3
"""goto <미션>_entry 로 가다가 시작 조건(거리·선수각)을 만족하면 goto 를 끄고 그 미션 시험을 바로 켠다.

goto 를 손으로 끌(Ctrl+C) 필요 없이 한 명령으로 접근 → 미션 시작. 젯슨 단축어 (setup_jetson_bashrc.sh):

  go_tournament [시험 인자 …]   goto tournament_entry → test_tournament
  go_avoid      [시험 인자 …]   goto avoid_entry      → test_avoid_outdoor
  go_gate       [시험 인자 …]   goto gate_entry       → test_gate_outdoor
  go_dock       [시험 인자 …]   goto dock_entry       → test_dock_outdoor (YOLO venv, run_camera 필요)
  go_station    [시험 인자 …]   goto station_entry    → test_mission station (접근 단계 없이 바로)
  go_search     [시험 인자 …]   goto search_entry     → test_mission search (search_target_color:=… 필요)

시작 조건 (기본: 각 미션 시작 조건과 같음 — 회피·토너먼트 3 m·±45°, 나머지 2 m·±30°)
— 아래를 hold 초 동안 계속 만족하면 시험으로 넘긴다:
  거리   배 ↔ <미션>_entry 가 dist 안
  선수각 gate·dock·station·search: 기록 선수각(rec 한 방향) ± heading 안
         avoid·tournament: 배 → <미션>_end 방향 ± heading 안 (회피 시험의 출발 검사와 같은 기준).
         회피 시험 출발 검사는 넘기는 동안 흐를 여유로 dist + 0.5 m·heading + 5° 로 넘긴다
         (start_tolerance·start_heading_tolerance_deg 를 시험 인자로 직접 주면 그 값).

옵션은 시험 인자 앞에: --dist 2.0 --heading 30 --hold 0.5 --goto thrust_ratio:=0.3 (--goto 여러 번 가능)
시험 인자 start_point:=… goal_point:=… 를 주면 접근 지점·목표도 그 이름으로 바뀐다.

넘기는 순간: goto 를 Ctrl+C 처럼 끄고(SIGINT) 시험 런치를 켠다. 시험이 뜨는 몇 초(도킹은 YOLO 로딩이라
더 김)는 명령이 없어 배가 조금 흐를 수 있다. Ctrl+C 를 누르면 지금 켜진 쪽을 끄고 끝난다.
"""

import argparse
import math
import os
import signal
import subprocess
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path

WS = Path(__file__).resolve().parents[1]
ODOM_STALE_SEC = 1.0


@dataclass(frozen=True)
class Preset:
    entry: str                     # 접근할 rec 지점
    test: tuple                    # 시험 명령 (인자 앞부분)
    heading_ref: str = 'entry'     # 'entry' = 기록 선수각, 'goal' = 배 → goal 방향
    goal: str = ''                 # heading_ref == 'goal' 일 때 목표 지점
    dist: float = 2.0              # [m]   (미션 시작 조건 2 m·±30°, 2026-11-13)
    heading: float = 30.0          # [deg]
    avoid_test: bool = False       # 회피 시험 노드(출발 검사 start_tolerance·start_heading_tolerance_deg)
    notes: tuple = field(default_factory=tuple)


def _autonomy(state):
    return ('ros2', 'launch', 'kaboat_bringup', 'autonomy.launch.py', f'start_state:={state}',
            'single_mission:=true', 'entry_required:=false', 'gate_motion_enabled:=true',
            'approach_motion_enabled:=true', 'dock_motion_enabled:=true')


# 회피 시험 출발 검사 여유 — goto 를 끄고 시험이 뜨는 몇 초 동안 배가 흘러도 출발이 막히지 않게
AVOID_START_MARGIN_M = 0.5
AVOID_START_MARGIN_DEG = 5.0

PRESETS = {
    'tournament': Preset('tournament_entry',
                         ('ros2', 'launch', 'kaboat_hardware', 'outdoor_tournament_test.launch.py'),
                         heading_ref='goal', goal='tournament_end', dist=3.0, heading=45.0,
                         avoid_test=True),
    'avoid': Preset('avoid_entry',
                    ('ros2', 'launch', 'kaboat_hardware', 'outdoor_avoid_test.launch.py'),
                    heading_ref='goal', goal='avoid_end', dist=3.0, heading=45.0, avoid_test=True),
    'gate': Preset('gate_entry', ('ros2', 'launch', 'kaboat_hardware', 'outdoor_gate_test.launch.py')),
    'dock': Preset('dock_entry', ('bash', str(WS / 'scripts' / 'run_dock_mission.sh')),
                   notes=('도킹 시험은 YOLO 로딩에 몇 초~십수 초 걸려 그동안 배가 흐를 수 있음',)),
    'station': Preset('station_entry', _autonomy('station')),
    'search': Preset('search_entry', _autonomy('search')),
}


def wrap_deg(angle: float) -> float:
    return (angle + 180.0) % 360.0 - 180.0


def start_condition(boat_xy, yaw_deg, entry_xy, ref_yaw_deg, dist_tol, heading_tol):
    """(만족?, 거리 [m], 선수각 오차 [deg])."""
    dist = math.hypot(boat_xy[0] - entry_xy[0], boat_xy[1] - entry_xy[1])
    err = abs(wrap_deg(yaw_deg - ref_yaw_deg))
    return dist <= dist_tol and err <= heading_tol, dist, err


def reference_yaw(boat_xy, entry_yaw_deg, goal_xy=None):
    """기준 선수각 [deg] — goal 이 있으면 배 → goal 방향, 없으면 기록 선수각."""
    if goal_xy is None:
        return entry_yaw_deg
    return math.degrees(math.atan2(goal_xy[1] - boat_xy[1], goal_xy[0] - boat_xy[0]))


class HoldTimer:
    """조건이 hold 초 동안 끊기지 않고 이어지면 True."""

    def __init__(self, hold_sec: float):
        self.hold = float(hold_sec)
        self.since = None

    def update(self, ok: bool, now: float) -> bool:
        if not ok:
            self.since = None
            return False
        if self.since is None:
            self.since = now
        return now - self.since >= self.hold


def _arg_value(args, name):
    """launch 인자 목록에서 name:=값 (마지막 것)."""
    value = None
    for a in args:
        if a.startswith(f'{name}:='):
            value = a.split(':=', 1)[1]
    return value


@dataclass
class Plan:
    entry: str
    goal: str | None
    dist: float
    heading: float
    hold: float
    goto_cmd: list
    test_cmd: list
    warnings: list


def build_plan(mission, test_args=(), dist=None, heading=None, hold=0.5, goto_args=()):
    if mission not in PRESETS:
        raise ValueError(f'미션 이름은 {"/".join(PRESETS)} 중 하나 — 받은 값 {mission!r}')
    p = PRESETS[mission]
    test_args = list(test_args)
    entry = _arg_value(test_args, 'start_point') or p.entry
    goal = (_arg_value(test_args, 'goal_point') or p.goal) if p.heading_ref == 'goal' else None
    dist = p.dist if dist is None else float(dist)
    heading = p.heading if heading is None else float(heading)
    if not (dist > 0 and 0 < heading <= 180 and hold >= 0):
        raise ValueError('--dist > 0, 0 < --heading ≤ 180, --hold ≥ 0')
    warnings = list(p.notes)
    extra = []
    if p.avoid_test:
        # 시험 출발 검사가 넘길 때 조건과 같으면 시험이 뜨는 동안 조금만 흘러도 출발이 막히고
        # (시험은 출발 전 0 명령을 내서) 그대로 서 있게 된다 → 여유를 두고 넘긴다
        if _arg_value(test_args, 'start_tolerance') is None:
            extra.append(f'start_tolerance:={dist + AVOID_START_MARGIN_M:g}')
        if _arg_value(test_args, 'start_heading_tolerance_deg') is None:
            extra.append('start_heading_tolerance_deg:='
                         f'{min(heading + AVOID_START_MARGIN_DEG, 180.0):g}')
    if mission == 'search' and _arg_value(test_args, 'search_target_color') is None:
        warnings.append('search_target_color:=red|green|black 를 안 주면 탐색 미션에서 멈춤')
    goto_cmd = ['ros2', 'launch', 'kaboat_hardware', 'outdoor_waypoint.launch.py',
                f'point:={entry}', *goto_args]
    return Plan(entry, goal, dist, heading, float(hold), goto_cmd,
                [*p.test, *test_args, *extra], warnings)


def _spawn(cmd):
    print(f'\n▶ {" ".join(cmd)}\n', flush=True)
    return subprocess.Popen(cmd, start_new_session=True)   # 자식 그룹에 신호를 따로 보냄


def _stop(proc, label, timeout=8.0):
    if proc is None or proc.poll() is not None:
        return
    for sig, wait in ((signal.SIGINT, timeout), (signal.SIGTERM, 3.0), (signal.SIGKILL, 2.0)):
        try:
            os.killpg(proc.pid, sig)
        except ProcessLookupError:
            return
        try:
            proc.wait(wait)
            print(f'■ {label} 끔', flush=True)
            return
        except subprocess.TimeoutExpired:
            continue


def run(plan: Plan, points_file, origin_file):
    import rclpy
    from nav_msgs.msg import Odometry
    from rclpy.qos import qos_profile_sensor_data

    from kaboat_mission.mission_points import recorded_point

    entry = recorded_point(plan.entry, points_file, origin_file)
    goal = recorded_point(plan.goal, points_file, origin_file) if plan.goal else None
    for w in plan.warnings:
        print(f'⚠ {w}', flush=True)
    ref_text = (f'배 → {plan.goal} 방향' if goal else f'기록 선수각 {entry.yaw_deg:.0f}°')
    print(f'접근: {plan.entry} ({entry.x:.2f}, {entry.y:.2f}) — 시작 조건 {plan.dist:g} m 안 + '
          f'{ref_text} ±{plan.heading:g}° 안을 {plan.hold:g} s 유지하면 시험 시작', flush=True)

    try:   # Ctrl+C 는 아래 처리기가 받아 자식(goto·시험)을 차례로 끈다 — rclpy 처리기는 끔
        from rclpy.signals import SignalHandlerOptions
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    except ImportError:
        rclpy.init()
    node = rclpy.create_node('go_and_test')
    latest = {}

    def on_odom(msg):
        q = msg.pose.pose.orientation
        yaw = math.degrees(math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                      1.0 - 2.0 * (q.y * q.y + q.z * q.z)))
        latest.update(xy=(msg.pose.pose.position.x, msg.pose.pose.position.y), yaw=yaw,
                      t=time.monotonic())

    node.create_subscription(Odometry, '/odom', on_odom, qos_profile_sensor_data)
    stop = {'flag': False}
    signal.signal(signal.SIGINT, lambda *_: stop.update(flag=True))
    signal.signal(signal.SIGTERM, lambda *_: stop.update(flag=True))

    goto = _spawn(plan.goto_cmd)
    test = None
    hold = HoldTimer(plan.hold)
    last_print = 0.0
    code = 0
    try:
        while not stop['flag']:
            rclpy.spin_once(node, timeout_sec=0.1)
            now = time.monotonic()
            if goto.poll() is not None:
                print(f'✗ goto 가 먼저 끝남 (코드 {goto.returncode}) — 시험을 켜지 않고 끝냄', flush=True)
                code = goto.returncode or 1
                return code
            if 't' not in latest or now - latest['t'] > ODOM_STALE_SEC:
                hold.update(False, now)
                if now - last_print >= 2.0:
                    print('… /odom 기다리는 중', flush=True)
                    last_print = now
                continue
            ref = reference_yaw(latest['xy'], entry.yaw_deg, goal.xy() if goal else None)
            ok, dist, err = start_condition(latest['xy'], latest['yaw'], entry.xy(), ref,
                                            plan.dist, plan.heading)
            if hold.update(ok, now):
                print(f'✓ 시작 조건 만족 — 거리 {dist:.2f} m, 선수각 오차 {err:.0f}° → goto 끄고 시험 시작',
                      flush=True)
                break
            if now - last_print >= 1.0:
                print(f'  goto 중 — {plan.entry} 까지 {dist:.1f} m (≤ {plan.dist:g}), 선수각 오차 '
                      f'{err:.0f}° (≤ {plan.heading:g}){"  [유지 중]" if ok else ""}', flush=True)
                last_print = now
        if stop['flag']:
            return 130
        _stop(goto, 'goto')
        node.destroy_node()
        node = None
        test = _spawn(plan.test_cmd)
        while test.poll() is None and not stop['flag']:
            time.sleep(0.2)
        code = test.returncode if test.returncode is not None else 130
        return code
    finally:
        _stop(goto, 'goto')
        _stop(test, '시험')
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main(argv=None):
    ap = argparse.ArgumentParser(
        description='goto <미션>_entry → 시작 조건 만족하면 그 미션 시험 시작',
        usage='go_and_test.py 미션 [--dist M] [--heading DEG] [--hold S] [--goto 인자:=값 …] [시험 인자:=값 …]')
    ap.add_argument('mission', choices=sorted(PRESETS))
    ap.add_argument('--dist', type=float, help='시작 지점에서 이 거리 [m] 안 (기본: 미션별)')
    ap.add_argument('--heading', type=float, help='선수각 오차 [deg] 안 (기본: 미션별)')
    ap.add_argument('--hold', type=float, default=0.5, help='조건 유지 시간 [s] (기본 0.5)')
    ap.add_argument('--goto', action='append', default=[], metavar='인자:=값',
                    help='goto(outdoor_waypoint) 인자, 예 --goto thrust_ratio:=0.3')
    ap.add_argument('--points-file', default='~/kaboat_mission_points.json')
    ap.add_argument('--origin-file', default='~/kaboat_geo_origin.yaml')
    args, test_args = ap.parse_known_args(argv)
    bad = [a for a in test_args if ':=' not in a]
    if bad:
        ap.error(f'시험 인자는 이름:=값 형식 — {bad}')
    try:
        plan = build_plan(args.mission, test_args, args.dist, args.heading, args.hold, args.goto)
        return run(plan, args.points_file, args.origin_file)
    except ValueError as exc:
        print(f'✗ {exc}', file=sys.stderr)
        return 2


if __name__ == '__main__':
    sys.exit(main())
