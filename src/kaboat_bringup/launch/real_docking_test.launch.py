"""Single real-boat docking mission; requires separately running sensors/odom.

No motor driver is started. Dock motion is disabled unless explicitly enabled.
The staging waypoint, YOLO model and wall-to-mouth offset are required inputs.
"""

import math
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _nodes(context):
    bringup_share = get_package_share_directory('kaboat_bringup')
    mission_params = os.path.join(bringup_share, 'config', 'mission_params.yaml')
    staging = [float(LaunchConfiguration(name).perform(context))
               for name in ('staging_x', 'staging_y')]
    wall_to_mouth = float(LaunchConfiguration('wall_to_mouth_m').perform(context))
    pre_entry_offset = float(
        LaunchConfiguration('pre_entry_offset_m').perform(context))
    hull_length = float(LaunchConfiguration('hull_length_m').perform(context))
    exit_clearance = float(LaunchConfiguration('exit_clearance_m').perform(context))
    if not all(math.isfinite(value) for value in
               (*staging, wall_to_mouth, pre_entry_offset,
                hull_length, exit_clearance)):
        raise ValueError('docking dimensions and staging coordinates must be finite')
    if wall_to_mouth <= 0 or pre_entry_offset <= 0 or hull_length <= 0 or exit_clearance < 0:
        raise ValueError('dock lengths must be positive; '
                         'exit_clearance_m must be non-negative')
    reverse_distance = max(
        0.5, wall_to_mouth + hull_length / 2 + exit_clearance - 1.0)

    weights = LaunchConfiguration('dock_weights').perform(context)
    if not os.path.isfile(weights):
        raise FileNotFoundError(f'dock_weights file not found: {weights}')

    return [
        Node(package='kaboat_perception', executable='dock_mark_detector',
             name='dock_mark_detector', output='screen', parameters=[{
                 'use_sim_time': False,
                 'weights': weights,
                 'target_class': LaunchConfiguration('dock_target_class'),
                 'device': LaunchConfiguration('dock_device'),
                 'wall_to_mouth_m': wall_to_mouth,
             }]),
        Node(package='kaboat_behaviors', executable='docking_ctrl',
             name='docking_ctrl', output='screen', parameters=[{
                 'use_sim_time': False,
                 'dock.motion_enabled': ParameterValue(
                     LaunchConfiguration('dock_motion_enabled'), value_type=bool),
                 'dock.berth_distance': 1.0,
                 'dock.pre_entry_offset': ParameterValue(
                     LaunchConfiguration('pre_entry_offset_m'), value_type=float),
                 'dock.reverse_min_distance': reverse_distance,
             }]),
        Node(package='kaboat_control', executable='cmd_mux',
             name='cmd_mux', output='screen', parameters=[{'use_sim_time': False}]),
        Node(package='kaboat_mission', executable='mission_manager',
             name='mission_manager', output='screen', parameters=[
                 mission_params,
                 {'use_sim_time': False, 'start_state': 'dock',
                  'single_mission': True, 'auto_start': True,
                  'waypoints.dock': staging},
             ]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('staging_x', description='실측 odom 정렬점 X [m]'),
        DeclareLaunchArgument('staging_y', description='실측 odom 정렬점 Y [m]'),
        DeclareLaunchArgument('dock_weights', description='Jetson의 YOLO .pt 절대 경로'),
        DeclareLaunchArgument('dock_target_class',
                              description='red_triangle / green_circle / blue_square'),
        DeclareLaunchArgument('wall_to_mouth_m',
                              description='표식 뒷벽부터 슬롯 입구까지 실측 거리 [m]'),
        DeclareLaunchArgument('pre_entry_offset_m', default_value='1.5',
                              description='슬롯 입구 바깥 첫 목표점 거리 [m]'),
        DeclareLaunchArgument('hull_length_m', default_value='1.0',
                              description='선체 실제 전장 [m]; 기본 1m는 임시값'),
        DeclareLaunchArgument('exit_clearance_m', default_value='0.2',
                              description='후진 완료 시 선미-입구 추가 여유 [m]'),
        DeclareLaunchArgument('dock_device', default_value='cpu',
                              description='Ultralytics device: cpu 또는 0'),
        DeclareLaunchArgument('dock_motion_enabled', default_value='false',
                              description='기본 false: 센서 점검 뒤 명시적으로 true'),
        OpaqueFunction(function=_nodes),
    ])
