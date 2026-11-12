"""도킹 파이프라인 launch 노드 묶음 — autonomy.launch.py 와 outdoor_dock_test.launch.py 가 같이 쓴다.

  카메라 TF(라이다 → camera_link, camera_mount.py 실측) → dock_mark_detector(YOLO, /detector/enable
  일 때만 추론) → dock_lidar_markers(마커별 RANSAC 벽) → dock_marker_goal(경유·정지점)
  → dock_ctrl_v2(one-shot 경로, /cmd/dock, /mission/complete=dock)

값은 launch 치환식(LaunchConfiguration)도 되고 파이썬 값도 된다. kaboat_hardware 가 kaboat_bringup
에 의존해서 bringup 이 hardware 모듈을 import 할 수 없으므로 두 쪽이 다 의존하는 여기에 둔다.
"""

import os

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
                  approach_cmd=0.10, final_cmd=0.05, retreat_cmd=0.05,
                  marker_order='', position_source='odom', use_sim_time=False,
                  base_frame='base_link', camera_tf=True, camera_tf_condition=None,
                  debug_image=True):
    """도킹 노드 목록. camera_tf=False 면 카메라 TF 를 안 냄(시뮬레이션·dock_yolo 가 내는 경우)."""
    common = {'use_sim_time': use_sim_time}
    nodes = []
    if camera_tf:
        nodes.append(Node(
            package='tf2_ros', executable='static_transform_publisher',
            name='dock_camera_mount_tf', output='screen', condition=camera_tf_condition,
            arguments=camera_mount_tf_arguments()))
    nodes += [
        Node(package='kaboat_perception', executable='dock_mark_detector',
             name='dock_mark_detector', output='screen', remappings=DETECTOR_REMAPS,
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
                 'marker_order': ParameterValue(marker_order, value_type=str),
                 'position_source': ParameterValue(position_source, value_type=str),
                 'gps_timeout_s': 2.0,
                 'odom_timeout_s': 2.0,
                 'perception_timeout_s': 2.0,
             }]),
    ]
    return nodes
