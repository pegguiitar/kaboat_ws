#!/usr/bin/env python3
"""rec 로 기록한 지점 하나를 미터 단위로 옮긴다 — 배를 그 자리로 가져가 다시 rec 하지 않고.

  python3 scripts/shift_point.py station_end 4            # 기록 선수각 방향으로 4 m (미리보기)
  python3 scripts/shift_point.py station_end 4 --write    # 실제로 저장
  python3 scripts/shift_point.py station_end --dx 3 --dy -1 --write   # /odom x(동)·y(북) [m] 로

젯슨에서 워크스페이스를 source 한 뒤 실행한다 (kaboat_mission·kaboat_hardware 를 쓴다).
--write 없이는 바뀔 값만 보여 준다. 선수각(yaw_deg)은 그대로 두고, 위도·경도는 기록할 때와 같은
방식(원점 파일 기준 LocalFrame)으로 다시 계산한다.

⚠ point_recorder(run_points)는 켤 때 파일을 메모리로 읽고, rec 할 때마다 그 메모리 내용 전체를 다시
  쓴다. 켜 둔 채 이 스크립트로 고치면 다음 rec 때 되돌아간다 — 그래서 point_recorder 가 돌고 있으면
  저장하지 않는다. run_points 를 켠 터미널에서 Ctrl+C → 이 스크립트 --write → 그 터미널에서 run_points
  다시 켜기 → list_points 로 확인. mission_manager·station_keeper 는 켤 때 파일을 읽으므로 run_autonomy 는
  고친 뒤 켠다.
"""

import argparse
import math
import os
import subprocess
import sys
from dataclasses import replace
from datetime import datetime


def shifted(point, distance=None, dx=0.0, dy=0.0):
    """(새 x, 새 y). distance 를 주면 그 지점의 기록 선수각 방향, 아니면 dx·dy."""
    if distance is not None:
        a = math.radians(point.yaw_deg)
        return point.x + distance * math.cos(a), point.y + distance * math.sin(a)
    return point.x + dx, point.y + dy


def point_recorder_running() -> bool:
    try:
        out = subprocess.run(['pgrep', '-f', 'kaboat_hardware/point_recorder'],
                             capture_output=True, text=True, check=False)
    except FileNotFoundError:
        return False
    return bool(out.stdout.strip())


def main(argv=None):
    from kaboat_mission.mission_points import (
        DEFAULT_ORIGIN_FILE, DEFAULT_POINTS_FILE, PointStore, read_origin_file)

    ap = argparse.ArgumentParser(description='rec 기록 지점 하나를 미터 단위로 옮김')
    ap.add_argument('name', help='지점 이름 (예: station_end)')
    ap.add_argument('distance', nargs='?', type=float,
                    help='기록 선수각 방향으로 옮길 거리 [m] (음수면 반대 방향)')
    ap.add_argument('--dx', type=float, help='/odom x(동쪽) 로 [m] — distance 대신')
    ap.add_argument('--dy', type=float, help='/odom y(북쪽) 로 [m] — distance 대신')
    ap.add_argument('--write', action='store_true', help='실제로 저장 (없으면 미리보기만)')
    ap.add_argument('--points-file', default=DEFAULT_POINTS_FILE)
    ap.add_argument('--origin-file', default=DEFAULT_ORIGIN_FILE)
    ap.add_argument('--ignore-recorder', action='store_true',
                    help='point_recorder 가 떠 있어도 저장 (다음 rec 때 되돌아감을 안다면)')
    args = ap.parse_args(argv)
    by_dist = args.distance is not None
    by_dxy = args.dx is not None or args.dy is not None
    if by_dist == by_dxy:
        ap.error('거리(선수각 방향) 하나 또는 --dx/--dy 중 하나만 준다')
    if not all(math.isfinite(v) for v in (args.distance or 0.0, args.dx or 0.0, args.dy or 0.0)):
        ap.error('거리는 유한한 수')

    path = os.path.expanduser(args.points_file)
    try:
        store = PointStore.load(path)
        origin = read_origin_file(args.origin_file)
        store.check_origin(origin)          # 원점을 다시 잡았으면 x/y 가 다른 자리 — 옮기지 않음
        old = store.get(args.name)
    except (KeyError, ValueError, OSError) as exc:
        print(f'✗ {exc}', file=sys.stderr)
        return 1

    x, y = shifted(old, args.distance, args.dx or 0.0, args.dy or 0.0)
    x, y = round(x, 3), round(y, 3)
    lat = lon = None
    if origin is not None:
        from kaboat_hardware.geo_local import LocalFrame
        lat, lon, _ = LocalFrame(*origin).to_llh(x, y)
        lat, lon = round(lat, 9), round(lon, 9)
    new = replace(old, x=x, y=y, lat=lat, lon=lon,
                  recorded_at=datetime.now().isoformat(timespec='seconds'))
    how = (f'선수각 {old.yaw_deg:+.1f}° 방향 {args.distance:+.2f} m' if by_dist
           else f'dx {args.dx or 0.0:+.2f}, dy {args.dy or 0.0:+.2f} m')
    print(f'{args.name}: ({old.x:+.2f}, {old.y:+.2f}) → ({x:+.2f}, {y:+.2f})  [{how}, '
          f'선수각 {old.yaw_deg:+.1f}° 그대로]')
    others = [(n, math.dist((x, y), (p.x, p.y))) for n, p in store.points.items() if n != args.name]
    if others:
        near = sorted(others, key=lambda t: t[1])[:3]
        print('  가까운 기록 지점: ' + ', '.join(f'{n} {d:.1f} m' for n, d in near))
    if not args.write:
        print('  미리보기 — 저장하려면 --write')
        return 0
    if point_recorder_running() and not args.ignore_recorder:
        print('✗ point_recorder(run_points)가 돌고 있음 — 켜 둔 채 고치면 다음 rec 때 되돌아간다.\n'
              '  run_points 를 켠 터미널에서 Ctrl+C 로 끈 뒤 다시 실행하고, 끝나면 run_points 를 다시 켠다.',
              file=sys.stderr)
        return 2
    try:
        store, _ = store.with_point(args.name, new, origin)
        store.save(path)
    except (OSError, ValueError) as exc:
        print(f'✗ 저장 실패: {exc}', file=sys.stderr)
        return 1
    print(f'✓ 저장함 ({path}). 되돌리기: 같은 명령에 거리 부호를 바꿔서')
    return 0


if __name__ == '__main__':
    sys.exit(main())
