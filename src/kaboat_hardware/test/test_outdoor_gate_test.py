"""outdoor_gate_test.launch.py: 기록 지점 두 개로 게이트 미션 하나만, 노드 구성."""

import importlib.util
import os

import pytest

from kaboat_mission.mission_points import MissionPoint, PointStore

HERE = os.path.dirname(__file__)
LAUNCH = os.path.join(HERE, '..', 'launch', 'outdoor_gate_test.launch.py')
ORIGIN = (35.18487880, 128.56510100, 30.0)


@pytest.fixture
def mod():
    spec = importlib.util.spec_from_file_location('outdoor_gate_launch', LAUNCH)
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m


def _files(tmp_path, origin=ORIGIN, points=None):
    o = tmp_path / 'origin.yaml'
    o.write_text(f'origin_lat: {origin[0]}\norigin_lon: {origin[1]}\norigin_alt: {origin[2]}\n')
    p = tmp_path / 'points.json'
    PointStore(ORIGIN, points or {
        'gate_entry': MissionPoint(104.14, -15.69, 9.5),
        'gate_end': MissionPoint(144.38, -25.62, -2.0),
        'station_entry': MissionPoint(158.24, -35.60, -4.9),   # station_end 없어도 됨
    }).save(str(p))
    return str(p), str(o)


def _get(mod, **overrides):
    values = {name: default for name, default, _ in mod.ARGS}
    values.update(overrides)
    return values.__getitem__


def test_mission_runs_gate_only_from_the_two_recorded_points(mod, tmp_path):
    points, origin = _files(tmp_path)
    p = mod.mission_parameters(_get(mod, points_file=points, origin_file=origin))
    assert p['start_state'] == 'gate' and p['single_mission'] is True
    assert p['waypoints.gate'] == [144.38, -25.62]
    assert p['entry_waypoints.gate'] == [104.14, -15.69, 9.5] and p['entry_ready.gate'] is True
    assert p['entry_required'] is False and p['points_file'] == ''   # 전체 파일은 안 읽음
    assert mod.mission_parameters(_get(mod, points_file=points, origin_file=origin,
                                       approach='true'))['entry_required'] is True


def test_gate_parameters(mod):
    g = mod.gate_parameters(_get(mod))
    assert g == {'gate.motion_enabled': True, 'gate.required_pairs': 7, 'use_sim_time': False}
    assert mod.gate_parameters(_get(mod, motion_enabled='false'))['gate.motion_enabled'] is False
    with pytest.raises(RuntimeError, match='1 이상'):
        mod.gate_parameters(_get(mod, required_pairs='0'))


def test_refuses_missing_point_or_moved_origin(mod, tmp_path):
    points, origin = _files(tmp_path, points={'gate_entry': MissionPoint(1.0, 2.0, 0.0)})
    with pytest.raises(RuntimeError, match='goal_point:=gate_end'):
        mod.mission_parameters(_get(mod, points_file=points, origin_file=origin))
    moved_dir = tmp_path / 'moved'
    moved_dir.mkdir()
    points, _ = _files(tmp_path)
    _, moved = _files(moved_dir, (ORIGIN[0] + 0.0001, ORIGIN[1], ORIGIN[2]))
    with pytest.raises(RuntimeError, match='원점이 다름'):
        mod.mission_parameters(_get(mod, points_file=points, origin_file=moved))


def test_preview_starts_no_cmd_mux_and_motion_does(mod, tmp_path):
    from launch import LaunchContext
    points, origin = _files(tmp_path)

    def executables(**overrides):
        ctx = LaunchContext()
        values = {name: default for name, default, _ in mod.ARGS}
        values.update(points_file=points, origin_file=origin, gate_params_file='', **overrides)
        ctx.launch_configurations.update(values)
        return [a.node_executable for a in mod._actions(ctx)]

    assert executables(motion_enabled='false') == ['gate_follower', 'mission_manager']
    assert executables() == ['gate_follower', 'mission_manager', 'cmd_mux']   # 지도는 run_map
    assert 'mission_approach' in executables(approach='true')
    assert executables(start_occupancy_grid='true')[0] == 'occupancy_grid'
