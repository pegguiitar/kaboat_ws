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


def test_ntrip_launch_file_found_in_either_install_layout(rs, tmp_path, monkeypatch):
    monkeypatch.setattr(rs, 'get_package_share_directory', lambda name: str(tmp_path))
    with pytest.raises(FileNotFoundError):
        rs._ntrip_launch_file()
    (tmp_path / 'launch').mkdir()
    (tmp_path / 'launch' / 'ntrip_client_launch.py').write_text('')   # 소스 빌드
    assert rs._ntrip_launch_file().endswith('launch/ntrip_client_launch.py')
    (tmp_path / 'ntrip_client_launch.py').write_text('')              # Humble apt 패키지
    assert rs._ntrip_launch_file() == str(tmp_path / 'ntrip_client_launch.py')


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
    monkeypatch.setattr(rs, '_ntrip_launch_file', lambda: '/opt/x/ntrip_client_launch.py')
    off = rs._gq7_actions(_context(rs, outdoor='true'))
    assert not any(isinstance(a, IncludeLaunchDescription) for a in off)
    assert [a.node_executable for a in off if isinstance(a, Node)] == ['gq7_local_odom']

    monkeypatch.setattr(rs, 'check_aux_port', lambda path: None)   # 테스트 PC 엔 GQ7 AUX 없음
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


def _rtk_ctx(rs, **kw):
    return _context(rs, outdoor='true', rtk='true', ntrip_username='u', ntrip_password='p', **kw)


def test_rtk_relays_corrections_through_rtcm_guard(rs, tmp_path, monkeypatch):
    """드라이버는 /rtcm/gq7 을 받고(rtcm_guard 가 /rtcm 을 중계·감시), RTK 가 아니면 /rtcm 그대로."""
    from launch.actions import GroupAction
    from launch.utilities import perform_substitutions
    from launch_ros.actions import SetRemap
    monkeypatch.setattr(rs.tempfile, 'gettempdir', lambda: str(tmp_path))
    monkeypatch.setattr(rs, '_ntrip_launch_file', lambda: '/opt/x/ntrip_client_launch.py')
    monkeypatch.setattr(rs, 'check_aux_port', lambda path: None)

    def remaps(actions):
        ctx = LaunchContext()
        group = [a for a in actions if isinstance(a, GroupAction)][0]
        return {perform_substitutions(ctx, r.src): perform_substitutions(ctx, r.dst)
                for r in group.get_sub_entities() if isinstance(r, SetRemap)}
    on = rs._gq7_actions(_rtk_ctx(rs))
    assert 'rtcm_guard' in [a.node_executable for a in on if isinstance(a, Node)]
    assert remaps(on)['/rtcm'] == '/rtcm/gq7'
    off = rs._gq7_actions(_context(rs, outdoor='true'))
    assert '/rtcm' not in remaps(off)


def test_rtk_refuses_missing_aux_port_unless_check_off(rs, tmp_path, monkeypatch):
    monkeypatch.setattr(rs.tempfile, 'gettempdir', lambda: str(tmp_path))
    monkeypatch.setattr(rs, '_ntrip_launch_file', lambda: '/opt/x/ntrip_client_launch.py')
    missing = str(tmp_path / 'microstrain_aux')
    with pytest.raises(RuntimeError, match='AUX 포트 .* 가 없다'):
        rs._gq7_actions(_rtk_ctx(rs, gq7_aux_port=missing))
    rs._gq7_actions(_rtk_ctx(rs, gq7_aux_port=missing, gq7_aux_check='false'))   # 확인 끔


def test_check_aux_port_finds_a_process_holding_it(rs, tmp_path):
    dev = tmp_path / 'ttyACM1'
    dev.write_text('')
    link = tmp_path / 'microstrain_aux'
    link.symlink_to(dev)
    proc = tmp_path / 'proc'
    (proc / '4242' / 'fd').mkdir(parents=True)
    (proc / '4242' / 'cmdline').write_bytes(b'microstrain_inertial_driver_node\0--ros-args\0')
    (proc / '4243' / 'fd').mkdir(parents=True)
    (proc / '4243' / 'cmdline').write_bytes(b'bash\0')
    (proc / '4243' / 'fd' / '0').symlink_to(tmp_path / 'other')
    assert rs.check_aux_port(str(link), str(proc)) is None             # 아무도 안 씀
    (proc / '4242' / 'fd' / '7').symlink_to(dev)
    msg = rs.check_aux_port(str(link), str(proc))
    assert msg and '4242 microstrain_inertial_driver_node' in msg and 'kill' in msg
    assert '가 없다' in rs.check_aux_port(str(tmp_path / 'nope'), str(proc))
