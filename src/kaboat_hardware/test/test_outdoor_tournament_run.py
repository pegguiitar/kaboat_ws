"""토너먼트 한 번에 launch (go_tournament): cmd_mux·sequence·goto·회피 시험을 같이, 명령은 cmd_mux 로."""

import importlib.util
import os
from types import SimpleNamespace

import pytest
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

from kaboat_mission.mission_points import MissionPoint, PointStore

HERE = os.path.dirname(__file__)
LAUNCH = os.path.join(HERE, '..', 'launch', 'outdoor_tournament_run.launch.py')
ORIGIN = (35.18487880, 128.56510100, 30.0)
START = (40.0, 10.0)
GOAL = (58.0, 12.0)


@pytest.fixture
def mod():
    spec = importlib.util.spec_from_file_location('outdoor_tournament_run_launch', LAUNCH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _configs(mod, tmp_path, **overrides):
    o = tmp_path / 'origin.yaml'
    o.write_text(f'origin_lat: {ORIGIN[0]}\norigin_lon: {ORIGIN[1]}\norigin_alt: {ORIGIN[2]}\n')
    p = tmp_path / 'points.json'
    PointStore(ORIGIN, {'tournament_entry': MissionPoint(*START, 6.3),
                        'tournament_end': MissionPoint(*GOAL, 0.0)}).save(str(p))
    values = {name: default for name, default, _ in mod.ARGS}
    values.update(points_file=str(p), origin_file=str(o))
    values.update(overrides)
    return values


def _nodes(mod, tmp_path, **overrides):
    context = SimpleNamespace(launch_configurations=_configs(mod, tmp_path, **overrides))
    return {a._Node__node_executable: a for a in mod._actions(context) if isinstance(a, Node)}


def _get(mod, tmp_path, **overrides):
    return _configs(mod, tmp_path, **overrides).__getitem__


def _avoid(mod, tmp_path, **overrides):
    return mod._tournament.avoid_parameters(mod.avoid_getter(_get(mod, tmp_path, **overrides)))


def _remaps(node):
    return [(''.join(s.text for s in a) if not isinstance(a, str) else a,
             ''.join(s.text for s in b) if not isinstance(b, str) else b)
            for a, b in node._Node__remappings]


def test_arguments_are_sequence_plus_tournament_test_args(mod):
    names = [name for name, _, _ in mod.ARGS]
    assert names[:5] == ['undock_sec', 'start_dist', 'start_heading_deg', 'start_hold_sec',
                         'goto_thrust_ratio']
    assert 'wait_for_start' not in names                    # 서비스로 출발 — 고정
    assert 'start_occupancy_grid' not in names              # 지도는 run_map
    defaults = {name: default for name, default, _ in mod.ARGS}
    assert defaults['start_point'] == 'tournament_entry' and defaults['goal_point'] == 'tournament_end'
    assert defaults['undock_sec'] == '3.0' and defaults['drive_enabled'] == 'true'
    assert defaults['start_tolerance'] == 'auto'
    declared = {a.name for a in mod.generate_launch_description().entities
                if isinstance(a, DeclareLaunchArgument)}
    assert declared == set(names)


def test_four_nodes_and_only_cmd_mux_drives_cmd_vel(mod, tmp_path):
    nodes = _nodes(mod, tmp_path)
    assert set(nodes) == {'cmd_mux', 'tournament_sequence', 'indoor_waypoint',
                          'real_obstacle_avoid_test'}
    assert _remaps(nodes['indoor_waypoint']) == [('/cmd_vel', '/cmd/approach')]
    assert _remaps(nodes['real_obstacle_avoid_test']) == [('/cmd_vel', '/cmd/avoid')]
    assert _remaps(nodes['cmd_mux']) == [] and _remaps(nodes['tournament_sequence']) == []


def test_avoid_test_waits_for_the_sequence_with_start_margin(mod, tmp_path):
    p = _avoid(mod, tmp_path)
    assert p['wait_for_start'] is True                      # sequence 가 서비스로 출발시킴
    assert p['start_tolerance'] == 3.5 and p['start_heading_tolerance_deg'] == 50.0
    assert p['fixed_start_xy'] == list(START) and p['fixed_goal_xy'] == list(GOAL)
    assert p['drive_enabled'] is True
    q = _avoid(mod, tmp_path, start_dist='1.5', start_heading_deg='178', drive_enabled='false')
    assert q['start_tolerance'] == 2.0 and q['start_heading_tolerance_deg'] == 180.0
    assert q['drive_enabled'] is False
    assert _avoid(mod, tmp_path, start_tolerance='6')['start_tolerance'] == 6.0   # 직접 주면 그 값


def test_goto_goes_to_start_point_and_sequence_gets_conditions(mod, tmp_path):
    get = _get(mod, tmp_path, goto_thrust_ratio='0.3', undock_sec='0', start_hold_sec='1.0')
    g = mod._goto.node_parameters(mod.goto_getter(get))
    assert g['point'] == 'tournament_entry' and g['align_heading'] is True
    assert g['thrust_ratio'] == 0.3 and g['motion_enabled'] is True and g['outdoor'] is True
    s = mod.sequence_parameters(get)
    assert (s['start_point'], s['goal_point']) == ('tournament_entry', 'tournament_end')
    assert (s['undock_sec'], s['start_dist'], s['start_heading_deg'], s['start_hold_sec']) == (
        0.0, 3.0, 45.0, 1.0)


def test_refuses_missing_points_and_bad_numbers(mod, tmp_path):
    with pytest.raises(RuntimeError, match='start_point:=nope'):
        _nodes(mod, tmp_path, start_point='nope')
    with pytest.raises(RuntimeError, match='start_dist'):
        _nodes(mod, tmp_path, start_dist='far')
