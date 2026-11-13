"""도킹 파이프라인 launch 노드 묶음 — autonomy.launch.py 와 outdoor_dock_test.launch.py 가 같이 쓴다.

  카메라 TF(라이다 → camera_link, camera_mount.py 실측) → dock_mark_detector(YOLO, /detector/enable
  일 때만 추론) → dock_ctrl_v2(목표 도형 방위로 서행 → 고정 치수 ㄷ자 슬롯 고정 → one-shot 경로,
  /cmd/dock, /mission/complete=dock).
  dock_lidar_markers(마커별 RANSAC 벽)·dock_marker_goal 은 RViz·녹화용 비교 정보만 낸다 — 제어에는
  안 쓴다 (직선 하나로는 칸막이 끝면을 뒷벽으로 잡았다, 2026-11-12 bag).

값은 launch 치환식(LaunchConfiguration)도 되고 파이썬 값도 된다. kaboat_hardware 가 kaboat_bringup
에 의존해서 bringup 이 hardware 모듈을 import 할 수 없으므로 두 쪽이 다 의존하는 여기에 둔다.
"""

import os

from launch.actions import LogInfo
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from kaboat_perception.camera_mount import camera_mount_tf_arguments

CLASSES = ('triangle', 'circle', 'square')

# 검출기의 뎁스 기반 지면 추정은 dock_lidar_markers(라이다 기반)와 같은 이름을 쓴다 — depth_ 로 비켜
# 녹화·RViz 에서 둘이 섞이지 않게 (dock_yolo.launch.py 와 같음). 도킹 목표는 라이다 쪽만 쓴다.
DETECTOR_REMAPS = [
    *[(f'/dock/markers/{c}/ground', f'/dock/markers/{c}/depth_ground') for c in CLASSES],
    *[(f'/dock/markers/{c}/ground_base', f'/dock/markers/{c}/depth_ground_base') for c in CLASSES],
    ('/dock/markers_ground', '/dock/depth_markers_ground'),
    ('/dock/markers_base', '/dock/depth_markers_base'),
    ('/dock/marker_ground', '/dock/marker_ground_depth'),
]


def _workspace_from_install():
    """설치된 share 경로(<ws>/install/kaboat_perception/share/kaboat_perception) → <ws>."""
    try:
        from ament_index_python.packages import get_package_share_directory
        share = get_package_share_directory('kaboat_perception')
    except Exception:
        return ''
    return os.path.abspath(os.path.join(share, '..', '..', '..', '..'))


def yolo_env(environ=None, workspace=None):
    """젯슨 YOLO CUDA 가상환경을 검출기 노드에만 붙일 환경 변수. 못 찾으면 {}.

    젯슨의 CUDA PyTorch(nv24.08)·ultralytics 8.4.110 은 <ws>/.venv-dock-yolo 에만 있다. 노드는
    시스템 python3 로 돌므로 그 site-packages 를 PYTHONPATH 앞에, PyTorch 가 찾는 libcusparseLt 를
    LD_LIBRARY_PATH 앞에 둔다(시작할 때만 읽혀서 노드 안에서는 못 바꿈). run_dock_yolo.sh 와 같은 값.
    위치: KABOAT_YOLO_ENV → $KABOAT_WS/.venv-dock-yolo → 설치 경로에서 찾은 워크스페이스.
    """
    import sys
    environ = os.environ if environ is None else environ
    candidates = [environ.get('KABOAT_YOLO_ENV', '')]
    for ws in (environ.get('KABOAT_WS', ''), workspace if workspace is not None
               else _workspace_from_install()):
        if ws:
            candidates.append(os.path.join(ws, '.venv-dock-yolo'))
    python = f'python{sys.version_info.major}.{sys.version_info.minor}'
    for root in filter(None, candidates):
        site = os.path.join(root, 'lib', python, 'site-packages')
        if not os.path.isdir(site):
            continue
        env = {'PYTHONPATH': os.pathsep.join(filter(None, [site, environ.get('PYTHONPATH', '')])),
               'YOLO_CONFIG_DIR': environ.get('YOLO_CONFIG_DIR', '/tmp/kaboat_yolo_config')}
        cusparselt = os.path.join(site, 'nvidia', 'cusparselt', 'lib')
        if os.path.isdir(cusparselt):
            env['LD_LIBRARY_PATH'] = os.pathsep.join(
                filter(None, [cusparselt, environ.get('LD_LIBRARY_PATH', '')]))
        return env
    return {}


def default_weights():
    """kaboat_perception/models/best.engine (젯슨 TensorRT) 이 있으면 그것, 없으면 best.pt."""
    try:
        from ament_index_python.packages import get_package_share_directory
        models = os.path.join(get_package_share_directory('kaboat_perception'), 'models')
    except Exception:   # 설치 안 된 테스트 환경
        return ''
    engine = os.path.join(models, 'best.engine')
    return engine if os.path.isfile(engine) else os.path.join(models, 'best.pt')


def docking_nodes(*, target_class, motion_enabled, weights, device='0',
                  approach_cmd=0.10, final_cmd=0.05, retreat_cmd=0.05, seek_cmd=0.07,
                  marker_order='', position_source='odom', use_sim_time=False,
                  base_frame='base_link', camera_tf=True, camera_tf_condition=None,
                  debug_image=True, detector_env=None):
    """도킹 노드 목록. camera_tf=False 면 카메라 TF 를 안 냄(시뮬레이션·dock_yolo 가 내는 경우).

    detector_env: 검출기 노드에만 붙일 환경 변수. None 이면 yolo_env() — 젯슨 YOLO 가상환경을 찾아
    붙인다(ros2 launch 를 어느 터미널에서 켜도 CUDA YOLO). 못 찾으면 경고를 내고 시스템 Python 으로
    돈다(torch 가 없으면 검출기는 'Dock YOLO unavailable' 로 fail-closed).
    """
    common = {'use_sim_time': use_sim_time}
    env = yolo_env() if detector_env is None else detector_env
    nodes = []
    if not env:
        nodes.append(LogInfo(msg='⚠️ [dock] YOLO 가상환경(.venv-dock-yolo, KABOAT_YOLO_ENV) 못 찾음 — '
                                 '검출기는 시스템 Python 으로 돔 (CUDA PyTorch 없으면 검출 안 함)'))
    elif 'YOLO_CONFIG_DIR' in env:
        os.makedirs(env['YOLO_CONFIG_DIR'], exist_ok=True)
    if camera_tf:
        nodes.append(Node(
            package='tf2_ros', executable='static_transform_publisher',
            name='dock_camera_mount_tf', output='screen', condition=camera_tf_condition,
            arguments=camera_mount_tf_arguments()))
    nodes += [
        Node(package='kaboat_perception', executable='dock_mark_detector',
             name='dock_mark_detector', output='screen', remappings=DETECTOR_REMAPS,
             additional_env=env or None,
             parameters=[common, {
                 'weights': weights,
                 'target_class': target_class,
                 'device': ParameterValue(device, value_type=str),
                 'base_frame': base_frame,
                 # 녹화(rec_bag)·RViz 용 YOLO 결과 영상: 마스크·도형·신뢰도, 320 px 5 fps
                 'publish_debug_image': debug_image,
             }]),
        Node(package='kaboat_hardware', executable='dock_lidar_markers',
             name='dock_lidar_markers', output='screen', parameters=[common, {
                 'target_class': target_class,
                 'base_frame': base_frame,
                 'marker_behind_wall_m': 0.0,
                 'input_timeout_s': 2.0,
             }]),
        Node(package='kaboat_hardware', executable='dock_marker_goal',
             name='dock_marker_goal', output='screen', parameters=[common, {
                 'target_class': target_class,
                 'tank_bounds': [-1000000.0, -1000000.0, 1000000.0, 1000000.0],
                 'dock_stop_distance_m': 1.0,
                 'approach_extra_m': 2.0,
                 'marker_timeout_s': 2.0,
                 'adaptive_entry_enabled': False,
             }]),
        Node(package='kaboat_hardware', executable='dock_ctrl_v2',
             name='dock_ctrl_v2', output='screen', parameters=[common, {
                 'mission_mode': True,
                 'target_class': target_class,
                 'drive_enabled': ParameterValue(motion_enabled, value_type=bool),
                 'approach_cmd': ParameterValue(approach_cmd, value_type=float),
                 'final_cmd': ParameterValue(final_cmd, value_type=float),
                 'retreat_cmd': ParameterValue(retreat_cmd, value_type=float),
                 # 목표 도형 방위로 서행 — 라이다 벽(약 6 m 안)이 잡힐 때까지
                 'seek_cmd': ParameterValue(seek_cmd, value_type=float),
                 'marker_order': ParameterValue(marker_order, value_type=str),
                 'position_source': ParameterValue(position_source, value_type=str),
                 'gps_timeout_s': 2.0,
                 'odom_timeout_s': 2.0,
                 'perception_timeout_s': 2.0,
             }]),
    ]
    return nodes
