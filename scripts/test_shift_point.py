"""shift_point.py — 기록 지점 옮기기 (파일만, ROS 없이)."""

import json
import math
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))

import shift_point  # noqa: E402
from kaboat_mission.mission_points import MissionPoint, PointStore  # noqa: E402

ORIGIN = (35.1848790, 128.5651015, 0.0)


@pytest.fixture
def files(tmp_path, monkeypatch):
    monkeypatch.setattr(shift_point, 'point_recorder_running', lambda: False)
    origin = tmp_path / 'origin.yaml'
    origin.write_text(f'origin_lat: {ORIGIN[0]}\norigin_lon: {ORIGIN[1]}\norigin_alt: 0.0\n')
    points = tmp_path / 'points.json'
    store = PointStore(ORIGIN, {
        'station_end': MissionPoint(159.83, -33.02, -21.2, 1.0, 2.0, 'old'),
        'station_center': MissionPoint(155.16, -31.21, -90.4)})
    store.save(str(points))
    return ['--points-file', str(points), '--origin-file', str(origin)], points


def _end(points):
    return json.loads(points.read_text())['points']['station_end']


def test_preview_does_not_write(files, capsys):
    args, points = files
    assert shift_point.main(['station_end', '4', *args]) == 0
    assert _end(points)['x'] == 159.83
    assert '미리보기' in capsys.readouterr().out


def test_shift_along_recorded_heading(files):
    args, points = files
    assert shift_point.main(['station_end', '4', '--write', *args]) == 0
    end = _end(points)
    a = math.radians(-21.2)
    assert (end['x'], end['y']) == (round(159.83 + 4 * math.cos(a), 3), round(-33.02 + 4 * math.sin(a), 3))
    assert end['yaw_deg'] == -21.2                    # 선수각은 그대로
    assert end['lat'] != 1.0 and abs(end['lat'] - 35.1846) < 1e-3   # 위도·경도 다시 계산
    assert json.loads(points.read_text())['points']['station_center']['x'] == 155.16


def test_shift_by_dx_dy_and_undo(files):
    args, points = files
    assert shift_point.main(['station_end', '--dx', '3', '--dy', '-1', '--write', *args]) == 0
    assert (_end(points)['x'], _end(points)['y']) == (162.83, -34.02)
    assert shift_point.main(['station_end', '--dx', '-3', '--dy', '1', '--write', *args]) == 0
    assert (_end(points)['x'], _end(points)['y']) == (159.83, -33.02)


def test_refuses_while_point_recorder_runs(files, monkeypatch):
    args, points = files
    monkeypatch.setattr(shift_point, 'point_recorder_running', lambda: True)
    assert shift_point.main(['station_end', '4', '--write', *args]) == 2
    assert _end(points)['x'] == 159.83


def test_refuses_on_other_origin_and_unknown_name(files, tmp_path):
    args, points = files
    assert shift_point.main(['nope', '4', '--write', *args]) == 1
    other = tmp_path / 'other.yaml'
    other.write_text('origin_lat: 35.19\norigin_lon: 128.57\n')
    assert shift_point.main(['station_end', '4', '--write', '--points-file', str(points),
                             '--origin-file', str(other)]) == 1
    assert _end(points)['x'] == 159.83


def test_needs_exactly_one_way(files):
    args, _ = files
    with pytest.raises(SystemExit):
        shift_point.main(['station_end', *args])
    with pytest.raises(SystemExit):
        shift_point.main(['station_end', '4', '--dx', '1', *args])
