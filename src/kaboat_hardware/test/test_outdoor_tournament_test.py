"""토너먼트 회피 시험 launch: 야외 회피 시험 launch 를 기본 지점만 바꿔 다시 쓴다."""

import importlib.util
import os

import pytest
from launch.actions import DeclareLaunchArgument

from kaboat_mission.mission_points import MissionPoint, PointStore, mission_slot

HERE = os.path.dirname(__file__)
LAUNCH = os.path.join(HERE, '..', 'launch', 'outdoor_tournament_test.launch.py')
ORIGIN = (35.18487880, 128.56510100, 30.0)
START = (40.0, 10.0)             # tournament_entry
GOAL = (58.0, 12.0)              # tournament_end (약 18 m)


@pytest.fixture
def mod():
    spec = importlib.util.spec_from_file_location('outdoor_tournament_launch', LAUNCH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _files(tmp_path, points):
    o = tmp_path / 'origin.yaml'
    o.write_text(f'origin_lat: {ORIGIN[0]}\norigin_lon: {ORIGIN[1]}\norigin_alt: {ORIGIN[2]}\n')
    p = tmp_path / 'points.json'
    PointStore(ORIGIN, points).save(str(p))
    return str(p), str(o)


def _get(mod, **overrides):
    values = {name: default for name, default, _ in mod.ARGS}
    values.update(overrides)
    return values.__getitem__


def test_defaults_are_tournament_points_and_rest_same_as_avoid(mod):
    mine = {name: default for name, default, _ in mod.ARGS}
    avoid = {name: default for name, default, _ in mod._avoid.ARGS}
    assert mine['start_point'] == 'tournament_entry'
    assert mine['goal_point'] == 'tournament_end'
    assert avoid['start_point'] == 'avoid_entry' and avoid['goal_point'] == 'avoid_end'
    assert list(mine) == list(avoid)                       # 인자 목록·순서 같음
    rest = lambda d: {k: v for k, v in d.items() if k not in mod.POINTS}  # noqa: E731
    assert rest(mine) == rest(avoid)                       # 지점 말고는 회피 시험과 같은 기본값


def test_launch_description_declares_tournament_defaults(mod):
    declared = {a.name: ''.join(s.text for s in a.default_value)
                for a in mod.generate_launch_description().entities
                if isinstance(a, DeclareLaunchArgument)}
    assert declared['start_point'] == 'tournament_entry'
    assert declared['goal_point'] == 'tournament_end'
    assert declared['drive_enabled'] == 'true'


def test_reads_recorded_tournament_pair(mod, tmp_path):
    points, origin = _files(tmp_path, {
        'tournament_entry': MissionPoint(*START, 6.3),
        'tournament_end': MissionPoint(*GOAL, 0.0),
        'avoid_entry': MissionPoint(0.0, 0.0, 0.0),        # 다른 지점이 있어도 tournament_* 만
        'avoid_end': MissionPoint(-30.0, 0.0, 0.0)})
    p = mod.avoid_parameters(_get(mod, points_file=points, origin_file=origin))
    assert p['fixed_start_xy'] == list(START) and p['fixed_goal_xy'] == list(GOAL)
    assert p['fixed_goal_enabled'] is True and p['drive_enabled'] is True
    assert p['start_tolerance'] == 3.0 and p['goal_tolerance'] == 2.0
    assert p['tank_bounds'] == [0.0] * 4 and p['exclude_tank_walls'] is False


def test_refuses_when_tournament_point_not_recorded(mod, tmp_path):
    points, origin = _files(tmp_path, {'avoid_entry': MissionPoint(*START, 0.0),
                                       'avoid_end': MissionPoint(*GOAL, 0.0)})
    with pytest.raises(RuntimeError, match='start_point:=tournament_entry'):
        mod.avoid_parameters(_get(mod, points_file=points, origin_file=origin))


def test_full_mission_does_not_read_tournament_points():
    assert mission_slot('tournament_entry') is None      # mission_manager 종합 미션과 별개
    assert mission_slot('tournament_end') is None
