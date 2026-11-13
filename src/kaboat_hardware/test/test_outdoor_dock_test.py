"""outdoor_dock_test.launch.py: 기록 지점 dock_entry 하나로 도킹 미션 하나만, 노드 구성·인자 검사."""

import importlib.util
import os

import pytest

from kaboat_mission.mission_points import MissionPoint, PointStore

HERE = os.path.dirname(__file__)
LAUNCH = os.path.join(HERE, '..', 'launch', 'outdoor_dock_test.launch.py')
ORIGIN = (35.18487880, 128.56510100, 30.0)


@pytest.fixture
def mod():
    spec = importlib.util.spec_from_file_location('outdoor_dock_launch', LAUNCH)
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m


def _files(tmp_path, points=None):
    o = tmp_path / 'origin.yaml'
    o.write_text(f'origin_lat: {ORIGIN[0]}\norigin_lon: {ORIGIN[1]}\norigin_alt: {ORIGIN[2]}\n')
    p = tmp_path / 'points.json'
    PointStore(ORIGIN, points or {
        'dock_entry': MissionPoint(160.5, 12.25, 90.0),        # dock_end 는 필요 없음
        'gate_entry': MissionPoint(104.14, -15.69, 9.5),
    }).save(str(p))
    return str(p), str(o)


def _configs(mod, tmp_path, **overrides):
    points, origin = _files(tmp_path)
    values = {name: default for name, default, _ in mod.ARGS}
    values.update(points_file=points, origin_file=origin, dock_weights='/models/best.pt')
    values.update(overrides)
    return values


def _get(mod, tmp_path, **overrides):
    return _configs(mod, tmp_path, **overrides).__getitem__


def test_mission_runs_dock_only_from_recorded_entry(mod, tmp_path):
    params = mod.mission_parameters(_get(mod, tmp_path))
    assert params['start_state'] == 'dock' and params['single_mission'] is True
    assert params['entry_waypoints.dock'] == pytest.approx([160.5, 12.25, 90.0])
    assert params['waypoints.dock'] == pytest.approx([160.5, 12.25])   # 경로 고정 전 자리표시
    assert params['entry_ready.dock'] is True and params['points_file'] == ''
    assert params['entry_required'] is False
    assert mod.mission_parameters(_get(mod, tmp_path, approach='true'))['entry_required'] is True


def test_missing_entry_point_is_refused(mod, tmp_path):
    with pytest.raises(RuntimeError, match='start_point:=dock_left'):
        mod.mission_parameters(_get(mod, tmp_path, start_point='dock_left'))


def test_dock_arguments_defaults_and_validation(mod, tmp_path):
    dock = mod.dock_arguments(_get(mod, tmp_path))
    assert dock == dict(target_class='triangle', motion_enabled=True, weights='/models/best.pt',
                        device='0', seek_cmd=0.07, approach_cmd=0.10, final_cmd=0.05,
                        retreat_cmd=0.20, marker_order='', position_source='odom')
    for bad in (dict(target_class='star'), dict(marker_order='square,triangle'),
                dict(position_source='filter'), dict(final_cmd='0'), dict(retreat_cmd='1.5'),
                dict(seek_cmd='0'),
                dict(motion_enabled='yes'), dict(dock_weights='')):
        with pytest.raises(RuntimeError):
            mod.dock_arguments(_get(mod, tmp_path, **bad))


def _executables(mod, tmp_path, **overrides):
    from types import SimpleNamespace
    context = SimpleNamespace(launch_configurations=_configs(mod, tmp_path, **overrides))
    from launch_ros.actions import Node
    return [a._Node__node_executable for a in mod._actions(context) if isinstance(a, Node)]


def test_nodes_motion_and_preview(mod, tmp_path):
    names = _executables(mod, tmp_path)
    assert names == ['mission_manager', 'static_transform_publisher', 'dock_mark_detector',
                     'dock_lidar_markers', 'dock_marker_goal', 'dock_ctrl_v2', 'cmd_mux']
    preview = _executables(mod, tmp_path, motion_enabled='false')
    assert 'cmd_mux' not in preview                            # 미리보기면 /cmd_vel 로 안 나감
    assert 'mission_approach' in _executables(mod, tmp_path, approach='true')
