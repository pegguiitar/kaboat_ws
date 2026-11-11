"""원점 파일 읽기와 야외 GQ7 설정 (드라이버 고정 원점 기능은 끄고 odometry_earth 를 씀)."""

import os

import pytest
import yaml

from kaboat_hardware.gq7_origin import load_origin_file

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


def test_outdoor_config_uses_earth_odometry_not_driver_origin():
    params = yaml.safe_load(open(OUTDOOR_CONFIG))['microstrain_inertial_driver']['ros__parameters']
    assert params['device_setup'] is True and params['use_enu_frame'] is True
    assert params['filter_relative_position_config'] is False      # 드라이버 원점 기능 끔
    assert 'filter_relative_position_ref' not in params
    assert params['filter_odometry_earth_data_rate'] > 0            # gq7_local_odom 입력
    assert params['gnss1_antenna_offset'] == [0.0, -0.4, 0.0]
    assert params['gnss2_antenna_offset'] == [0.0, 0.4, 0.0]
