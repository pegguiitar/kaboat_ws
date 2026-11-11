"""Single real-boat docking mission; requires separately running sensors/odom.

No motor driver is started. Dock motion is disabled unless explicitly enabled.
The staging pose (position and heading) and YOLO model are required inputs.
Slot and hull dimensions (wall-to-mouth, slot width, hull size, ...) are NOT
launch arguments: they live in one file, config/dock_slot.yaml, chosen with
`slot_config`. Approach motion is disabled unless explicitly enabled.
An optional retry waypoint (retry_x/y/angle_deg) is visited once if the first
ACQUIRE search fails; without it a failed search aborts.
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
    heading_text = LaunchConfiguration('staging_angle_deg').perform(context).strip()
    staging_heading = float(heading_text)
    if not all(math.isfinite(value) for value in (*staging, staging_heading)):
        raise ValueError('staging coordinates must be finite')

    retry_text = [LaunchConfiguration(name).perform(context).strip()
                  for name in ('retry_x', 'retry_y', 'retry_angle_deg')]
    retry_enabled = any(retry_text)
    retry_params = {'dock.retry_enabled': False}
    if retry_enabled:
        if not all(retry_text):
            raise ValueError('retry_x, retry_y, retry_angle_deg must be '
                             'given together (or all left empty)')
        retry = [float(value) for value in retry_text]
        if not all(math.isfinite(value) for value in retry):
            raise ValueError('retry waypoint must be finite')
        retry_params = {
            'dock.retry_enabled': True, 'dock.retry_x': retry[0],
            'dock.retry_y': retry[1], 'dock.retry_angle_deg': retry[2],
        }

    slot_config = LaunchConfiguration('slot_config').perform(context)
    if not os.path.isfile(slot_config):
        raise FileNotFoundError(f'slot_config file not found: {slot_config}')

    weights = LaunchConfiguration('dock_weights').perform(context)
    if not os.path.isfile(weights):
        raise FileNotFoundError(f'dock_weights file not found: {weights}')

    return [
        Node(package='kaboat_behaviors', executable='mission_approach',
             name='mission_approach', output='screen', parameters=[{
                 'use_sim_time': False,
                 'approach.motion_enabled': ParameterValue(
                     LaunchConfiguration('approach_motion_enabled'),
                     value_type=bool),
             }]),
        Node(package='kaboat_perception', executable='dock_mark_detector',
             name='dock_mark_detector', output='screen', parameters=[{
                 'use_sim_time': False,
                 'weights': weights,
                 'target_class': LaunchConfiguration('dock_target_class'),
                 'device': LaunchConfiguration('dock_device'),
             }]),
        Node(package='kaboat_behaviors', executable='docking_ctrl',
             name='docking_ctrl', output='screen', parameters=[slot_config, {
                 'use_sim_time': False,
                 'dock.motion_enabled': ParameterValue(
                     LaunchConfiguration('dock_motion_enabled'), value_type=bool),
                 'dock.staging_heading_enabled': True,
                 'dock.staging_angle_deg': staging_heading,
                 **retry_params,
             }]),
        Node(package='kaboat_control', executable='cmd_mux',
             name='cmd_mux', output='screen', parameters=[{'use_sim_time': False}]),
        Node(package='kaboat_mission', executable='mission_manager',
             name='mission_manager', output='screen', parameters=[
                 mission_params,
                 {'use_sim_time': False, 'start_state': 'dock',
                  'single_mission': True, 'auto_start': True,
                  'entry_waypoints.dock': [*staging, staging_heading],
                  'entry_ready.dock': True,
                  'waypoints.dock': staging},
             ]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('staging_x', description='실측 odom 정렬점 X [m]'),
        DeclareLaunchArgument('staging_y', description='실측 odom 정렬점 Y [m]'),
        DeclareLaunchArgument('staging_angle_deg',
                              description='정렬점 도착 선수각 [deg]'),
        DeclareLaunchArgument('dock_weights', description='Jetson의 YOLO .pt 절대 경로'),
        DeclareLaunchArgument('dock_target_class',
                              description='triangle / circle / square'),
        DeclareLaunchArgument(
            'slot_config',
            default_value=os.path.join(
                get_package_share_directory('kaboat_bringup'),
                'config', 'dock_slot.yaml'),
            description='슬롯·선체 치수 YAML(wall_to_mouth, 슬롯 폭, 선체 크기 등). '
                        '슬롯 크기 값은 이 파일에서만 정한다'),
        DeclareLaunchArgument('retry_x', default_value='',
                              description='ACQUIRE 15초 실패 시 이동할 재시도 odom X [m]; '
                                          '비우면 재시도 없이 ABORT'),
        DeclareLaunchArgument('retry_y', default_value='',
                              description='재시도 waypoint odom Y [m]'),
        DeclareLaunchArgument('retry_angle_deg', default_value='',
                              description='재시도 waypoint 도착 선수각 [deg]'),
        DeclareLaunchArgument('dock_device', default_value='cpu',
                              description='Ultralytics device: cpu 또는 0'),
        DeclareLaunchArgument('dock_motion_enabled', default_value='false',
                              description='기본 false: 센서 점검 뒤 명시적으로 true'),
        DeclareLaunchArgument('approach_motion_enabled', default_value='false',
                              description='기본 false: 시작 자세 접근은 미리보기만 수행'),
        OpaqueFunction(function=_nodes),
    ])
