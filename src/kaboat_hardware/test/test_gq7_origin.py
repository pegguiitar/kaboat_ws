"""저장된 원점이 GQ7 드라이버 설정(고정 상대 원점)으로 들어가는지."""

import os

import pytest
import yaml

from kaboat_hardware.gq7_origin import (
    load_origin_file, params_with_origin, write_params_with_origin)

ORIGIN = (35.1234567, 129.0456789, 12.5)
OUTDOOR_CONFIG = os.path.join(
    os.path.dirname(__file__), '..', 'config', 'gq7_outdoor.yaml')


def test_load_origin_file(tmp_path):
    assert load_origin_file(str(tmp_path / 'none.yaml')) is None
    f = tmp_path / 'origin.yaml'
    f.write_text('origin_lat: 35.1234567\norigin_lon: 129.0456789\norigin_alt: 12.5\n')
    assert load_origin_file(str(f)) == pytest.approx(ORIGIN)
    f.write_text('origin_lat: 95.0\norigin_lon: 129.0\n')
    with pytest.raises(ValueError):
        load_origin_file(str(f))


def test_origin_overrides_relative_position_and_keeps_the_rest():
    base = {'microstrain_inertial_driver': {'ros__parameters': {
        'device_setup': True, 'map_frame_id': 'odom',
        'filter_relative_position_config': True,
        'filter_relative_position_source': 2}}}
    params = params_with_origin(base, ORIGIN)['microstrain_inertial_driver']['ros__parameters']
    assert params['filter_relative_position_source'] == 1        # 고정점 기준
    assert params['filter_relative_position_frame'] == 2         # 위도·경도
    assert params['filter_relative_position_ref'] == pytest.approx(list(ORIGIN))
    assert params['map_frame_id'] == 'odom' and params['device_setup'] is True
    assert base['microstrain_inertial_driver']['ros__parameters'][
        'filter_relative_position_source'] == 2                    # 원본은 그대로


def test_refuses_config_that_does_not_write_to_the_device():
    base = {'microstrain_inertial_driver': {'ros__parameters': {'device_setup': False}}}
    with pytest.raises(ValueError, match='device_setup'):
        params_with_origin(base, ORIGIN)


def test_outdoor_config_file_merges(tmp_path):
    path = write_params_with_origin(OUTDOOR_CONFIG, ORIGIN, str(tmp_path))
    data = yaml.safe_load(open(path))['microstrain_inertial_driver']['ros__parameters']
    assert data['filter_relative_position_ref'] == pytest.approx(list(ORIGIN))
    assert data['map_frame_id'] == 'odom' and data['use_enu_frame'] is True


def test_relaunch_overwrites_instead_of_piling_up_files(tmp_path):
    first = write_params_with_origin(OUTDOOR_CONFIG, ORIGIN, str(tmp_path))
    moved = (ORIGIN[0] + 0.001, ORIGIN[1], ORIGIN[2])
    second = write_params_with_origin(OUTDOOR_CONFIG, moved, str(tmp_path))
    assert first == second
    assert os.listdir(tmp_path) == [os.path.basename(first)]
    data = yaml.safe_load(open(second))['microstrain_inertial_driver']['ros__parameters']
    assert data['filter_relative_position_ref'][0] == pytest.approx(moved[0])
