"""real_sensors.launch.py rtk:=true — GQ7 NTRIP 입력 설정과 ntrip_client 실행."""

import importlib.util
import os

import pytest
import yaml

pytest.importorskip('launch_ros')
from launch import LaunchContext  # noqa: E402
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription  # noqa: E402
from launch_ros.actions import Node  # noqa: E402

HERE = os.path.dirname(__file__)
LAUNCH = os.path.join(HERE, '..', 'launch', 'real_sensors.launch.py')
CONFIG_DIR = os.path.join(HERE, '..', 'config')


@pytest.fixture
def rs(monkeypatch):
    spec = importlib.util.spec_from_file_location('real_sensors_launch', LAUNCH)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    # 설치 안 된 테스트 환경: kaboat_hardware share = 소스 폴더
    monkeypatch.setattr(mod, 'get_package_share_directory',
                        lambda name: os.path.join(HERE, '..'))
    return mod


def _context(mod, **overrides):
    ld = mod._launch(None, None, 'false', '', '', os.path.join(CONFIG_DIR, 'gq7.yaml'))
    ctx = LaunchContext()
    for e in ld.entities:
        if isinstance(e, DeclareLaunchArgument):
            ctx.launch_configurations[e.name] = ''.join(
                s.perform(ctx) for s in e.default_value) if e.default_value else ''
    ctx.launch_configurations.update(overrides)
    return ctx


def test_ntrip_params_copy_enables_aux_input(rs, tmp_path):
    path = rs.gq7_params_with_ntrip(os.path.join(CONFIG_DIR, 'gq7_outdoor.yaml'),
                                    '/dev/microstrain_aux', str(tmp_path))
    params = yaml.safe_load(open(path))['microstrain_inertial_driver']['ros__parameters']
    assert params['ntrip_interface_enable'] is True
    assert params['aux_port'] == '/dev/microstrain_aux'
    assert params['gnss1_antenna_offset'] == [0.0, -0.4, 0.0]       # 나머지 설정 유지
    assert os.listdir(tmp_path) == [os.path.basename(path)]


def test_rtk_off_by_default_and_on_adds_ntrip_client(rs, tmp_path, monkeypatch):
    monkeypatch.setattr(rs.tempfile, 'gettempdir', lambda: str(tmp_path))
    off = rs._gq7_actions(_context(rs, outdoor='true'))
    assert not any(isinstance(a, IncludeLaunchDescription) for a in off)
    assert [a.node_executable for a in off if isinstance(a, Node)] == ['gq7_local_odom']

    with pytest.raises(RuntimeError, match='NTRIP 계정'):
        rs._gq7_actions(_context(rs, outdoor='true', rtk='true'))   # 계정 없음
    on = rs._gq7_actions(_context(rs, outdoor='true', rtk='true', ntrip_mountpoint='YNDO_RTCM32',
                                  ntrip_username='user1', ntrip_password='pw1'))
    ntrip = [a for a in on if isinstance(a, IncludeLaunchDescription)]
    assert len(ntrip) == 1
    args = dict(ntrip[0].launch_arguments)
    assert args['host'] == 'ntrip.ndgps.go.kr' and args['mountpoint'] == 'YNDO_RTCM32'
    assert args['rtcm_message_package'] == 'rtcm_msgs'
    assert (args['username'], args['password']) == ('user1', 'pw1')
    written = yaml.safe_load(open(tmp_path / rs.RTK_PARAMS_FILE))
    assert written['microstrain_inertial_driver']['ros__parameters']['ntrip_interface_enable']
