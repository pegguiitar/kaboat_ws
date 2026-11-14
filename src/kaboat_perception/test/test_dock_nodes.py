"""dock_nodes·camera_mount: 도킹 launch 노드 묶음 — 카메라 TF 실측값, 검출기 remap, 녹화용 영상."""

import math

from kaboat_perception import camera_mount
from kaboat_perception.camera_obstacles import CameraObstacleParams
from kaboat_perception.dock_nodes import DETECTOR_REMAPS, docking_nodes, yolo_env


def test_camera_mount_is_the_single_source():
    # 2026-11-12 실측: 라이다에서 뒤 0.615 m·위 0.635 m, 수평, 흘수선 위 0.775 m
    assert camera_mount.CAMERA_FROM_LIDAR == (-0.615, 0.0, 0.635)
    assert camera_mount.CAMERA_X == -0.215
    p = CameraObstacleParams()
    assert (p.offset_x, p.offset_y, p.height, p.mount_pitch_deg) == (
        camera_mount.CAMERA_X, camera_mount.CAMERA_Y,
        camera_mount.CAMERA_HEIGHT_ABOVE_WATER, camera_mount.CAMERA_PITCH_DEG)
    args = camera_mount.camera_mount_tf_arguments()
    values = dict(zip(args[::2], args[1::2]))
    assert (float(values['--x']), float(values['--z'])) == (-0.615, 0.635)
    assert float(values['--pitch']) == math.radians(camera_mount.CAMERA_PITCH_DEG)
    assert (values['--frame-id'], values['--child-frame-id']) == ('onboard_laser_frame', 'camera_link')


def _nodes(**overrides):
    from launch_ros.actions import Node
    args = dict(target_class='circle', motion_enabled=True, weights='/w.pt',
                detector_env={'PYTHONPATH': '/venv/site'})
    args.update(overrides)
    return {n._Node__node_executable: n for n in docking_nodes(**args) if isinstance(n, Node)}


def _params(node):
    from launch import LaunchContext
    from launch_ros.utilities import evaluate_parameters
    flat = {}
    for item in evaluate_parameters(LaunchContext(), node._Node__parameters):
        if isinstance(item, dict):
            flat.update(item)
    return flat


def test_detector_outputs_are_kept_apart_from_lidar_markers():
    nodes = _nodes()
    assert list(nodes) == ['static_transform_publisher', 'dock_mark_detector',
                           'dock_lidar_markers', 'dock_marker_goal', 'dock_ctrl_v2']
    remapped = dict(DETECTOR_REMAPS)
    for c in ('triangle', 'circle', 'square'):
        assert remapped[f'/dock/markers/{c}/ground'] == f'/dock/markers/{c}/depth_ground'
    assert remapped['/dock/marker_ground'] == '/dock/marker_ground_depth'
    detector = _params(nodes['dock_mark_detector'])
    assert detector['publish_debug_image'] is True             # rec_bag 이 YOLO 결과 영상을 저장
    assert detector['device'] == '0'                           # 문자열 (정수 0 이면 타입 오류)
    ctrl = _params(nodes['dock_ctrl_v2'])
    assert (ctrl['position_source'], ctrl['marker_order'], ctrl['retreat_cmd']) == ('odom', '', 0.20)
    assert ctrl['seek_cmd'] == 0.07                            # 목표 방위 서행


def test_camera_tf_can_be_left_to_another_owner():
    assert 'static_transform_publisher' not in _nodes(camera_tf=False)


def _fake_venv(root, cusparselt=True):
    import sys
    site = root / 'lib' / f'python{sys.version_info.major}.{sys.version_info.minor}' / 'site-packages'
    (site / 'nvidia' / 'cusparselt' / 'lib').mkdir(parents=True) if cusparselt else site.mkdir(parents=True)
    return site


def test_yolo_env_finds_the_jetson_venv(tmp_path):
    # 젯슨: <ws>/.venv-dock-yolo 의 CUDA PyTorch + libcusparseLt (run_dock_yolo.sh 와 같은 값)
    site = _fake_venv(tmp_path / 'ws' / '.venv-dock-yolo')
    env = yolo_env({'KABOAT_WS': str(tmp_path / 'ws'), 'PYTHONPATH': '/opt/ros/lib',
                    'LD_LIBRARY_PATH': '/opt/ros/lib'}, workspace='')
    assert env['PYTHONPATH'] == f'{site}:/opt/ros/lib'                     # 가상환경이 앞
    assert env['LD_LIBRARY_PATH'] == f'{site}/nvidia/cusparselt/lib:/opt/ros/lib'
    assert env['YOLO_CONFIG_DIR'] == '/tmp/kaboat_yolo_config'


def test_yolo_env_override_and_missing(tmp_path):
    other = _fake_venv(tmp_path / 'other', cusparselt=False)
    _fake_venv(tmp_path / 'ws' / '.venv-dock-yolo')
    env = yolo_env({'KABOAT_YOLO_ENV': str(tmp_path / 'other'),
                    'KABOAT_WS': str(tmp_path / 'ws')}, workspace='')
    assert env['PYTHONPATH'] == str(other) and 'LD_LIBRARY_PATH' not in env
    assert yolo_env({'KABOAT_WS': str(tmp_path / 'none')}, workspace='') == {}   # 노트북 등


def test_venv_env_goes_to_the_detector_only():
    from launch_ros.actions import Node
    nodes = _nodes()
    detector_env = dict((k[0].text, v[0].text) for k, v in nodes['dock_mark_detector'].additional_env)
    assert detector_env == {'PYTHONPATH': '/venv/site'}
    for name, node in nodes.items():
        if name == 'dock_ctrl_v2':
            ctrl_env = dict((k[0].text, v[0].text) for k, v in node.additional_env)
            assert ctrl_env == {'OPENBLAS_NUM_THREADS': '1'}
        elif name != 'dock_mark_detector':
            assert not node.additional_env                       # 다른 노드는 시스템 Python 그대로
    without = docking_nodes(target_class='circle', motion_enabled=True, weights='/w.pt',
                            detector_env={})
    assert not isinstance(without[0], Node)                      # 못 찾으면 경고(LogInfo) 먼저
    assert 'YOLO' in without[0].msg[0].text
