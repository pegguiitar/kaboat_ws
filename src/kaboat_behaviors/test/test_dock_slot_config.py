"""config/dock_slot.yaml 이 노드가 실제로 읽는 파라미터와 맞는지 검사한다.

ROS 는 YAML 의 오타 키를 조용히 무시하므로(치수가 적용되지 않은 채 기본값으로
동작), 키 이름과 값의 일관성을 여기서 미리 잡는다.
"""
import dataclasses
from pathlib import Path

import pytest
import yaml

from kaboat_behaviors.docking_fsm import DockParams
from kaboat_behaviors.slot_dimensions import SlotDimensions

CONFIG = (Path(__file__).resolve().parents[2]
          / 'kaboat_bringup' / 'config' / 'dock_slot.yaml')
NODE_ONLY = {'dock.dimensions_measured', 'dock.wall_to_mouth_m',
             'dock.exit_clearance_m',
             'dock.slot_pitch_m'}   # dock_ctrl_v2.load_slot_pitch 가 파일에서 직접 읽음


def flatten(tree, prefix=''):
    flat = {}
    for key, value in tree.items():
        name = f'{prefix}{key}'
        if isinstance(value, dict):
            flat.update(flatten(value, name + '.'))
        else:
            flat[name] = value
    return flat


@pytest.fixture(scope='module')
def params():
    tree = yaml.safe_load(CONFIG.read_text(encoding='utf-8'))
    assert list(tree) == ['docking_ctrl']          # 노드 이름과 일치
    return flatten(tree['docking_ctrl']['ros__parameters'])


def test_every_key_is_a_real_node_parameter(params):
    allowed = NODE_ONLY | {f'dock.{f.name}' for f in dataclasses.fields(DockParams)}
    assert set(params) <= allowed, sorted(set(params) - allowed)


def test_every_slot_size_value_is_in_the_file(params):
    for key in ('dock.wall_to_mouth_m', 'dock.slot_width', 'dock.slot_width_tolerance',
                'dock.hull_width', 'dock.hull_length', 'dock.berth_distance',
                'dock.pre_entry_offset', 'dock.exit_clearance_m',
                'dock.dimensions_measured'):
        assert key in params, key


def test_shipped_values_pass_the_dimension_checks(params):
    dims = SlotDimensions(
        wall_to_mouth=params['dock.wall_to_mouth_m'],
        slot_width=params['dock.slot_width'],
        slot_width_tolerance=params['dock.slot_width_tolerance'],
        hull_width=params['dock.hull_width'],
        hull_length=params['dock.hull_length'],
        berth_distance=params['dock.berth_distance'],
        pre_entry_offset=params['dock.pre_entry_offset'],
        exit_clearance=params['dock.exit_clearance_m'])
    assert dims.reverse_min_distance > 0


def test_values_are_floats_so_ros_does_not_see_integer_parameters(params):
    # ROS 파라미터는 선언 시 타입이 고정된다: 3 (int) 은 double 파라미터에 못 들어간다.
    for key, value in params.items():
        if key != 'dock.dimensions_measured':
            assert isinstance(value, float), (key, value)


def test_files_are_unambiguous_single_source():
    """치수가 코드의 다른 곳에 중복으로 적혀 있지 않은지 launch 를 훑어 확인."""
    bringup = CONFIG.parents[1] / 'launch'
    for launch in (bringup / 'autonomy.launch.py', bringup / 'real_docking_test.launch.py'):
        text = launch.read_text(encoding='utf-8')
        for token in ("'wall_to_mouth_m'", "'hull_length_m'", "'hull_width_m'",
                      "'slot_width_m'", "'pre_entry_offset_m'", "'exit_clearance_m'",
                      "'dock.berth_distance'", "'dock.slot_width'"):
            assert token not in text, (launch.name, token)
