"""Single docking mission through the competition mission state machine.

Sensors and the real thruster driver run separately. After mission_approach
reaches the staging pose, /mission/state=dock starts measured-normal yaw
alignment, then a single GPS/LOS path. Motion is disabled by default.
"""

import math
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _nodes(context):
    bringup_share = get_package_share_directory('kaboat_bringup')
    mission_params = os.path.join(bringup_share, 'config', 'mission_params.yaml')
    common = {'use_sim_time': ParameterValue(
        LaunchConfiguration('use_sim_time'), value_type=bool)}
    staging = [float(LaunchConfiguration(name).perform(context))
               for name in ('staging_x', 'staging_y')]
    heading_text = LaunchConfiguration('staging_angle_deg').perform(context).strip()
    staging_heading = float(heading_text)
    if not all(math.isfinite(value) for value in (*staging, staging_heading)):
        raise ValueError('staging coordinates must be finite')

    if any(LaunchConfiguration(name).perform(context).strip()
           for name in ('retry_x', 'retry_y', 'retry_angle_deg')):
        raise ValueError('retry_* waypoints are unsupported by the one-shot docking controller')

    weights = LaunchConfiguration('dock_weights').perform(context)
    if not os.path.isfile(weights):
        raise FileNotFoundError(f'dock_weights file not found: {weights}')

    return [
        Node(package='kaboat_behaviors', executable='mission_approach',
             name='mission_approach', output='screen', parameters=[common, {
                 'approach.motion_enabled': ParameterValue(
                     LaunchConfiguration('approach_motion_enabled'),
                     value_type=bool),
             }]),
        Node(package='kaboat_perception', executable='dock_mark_detector',
             name='dock_mark_detector', output='screen', parameters=[common, {
                 'weights': weights,
                 'target_class': LaunchConfiguration('dock_target_class'),
                 'device': LaunchConfiguration('dock_device'),
                 'base_frame': LaunchConfiguration('dock_base_frame'),
             }]),
        Node(package='kaboat_hardware', executable='dock_lidar_markers',
             name='dock_lidar_markers', output='screen', parameters=[common, {
                 'target_class': LaunchConfiguration('dock_target_class'),
                 'base_frame': LaunchConfiguration('dock_base_frame'),
                 'marker_behind_wall_m': 0.0,
                 'input_timeout_s': 2.0,
             }]),
        Node(package='kaboat_hardware', executable='dock_marker_goal',
             name='dock_marker_goal', output='screen', parameters=[common, {
                 'target_class': LaunchConfiguration('dock_target_class'),
                 'tank_bounds': [-1000000.0, -1000000.0, 1000000.0, 1000000.0],
                 'dock_stop_distance_m': 1.0,
                 'approach_extra_m': 2.0,
                 'marker_timeout_s': 2.0,
                 'adaptive_entry_enabled': False,
             }]),
        Node(package='kaboat_hardware', executable='dock_ctrl_v2',
             name='dock_ctrl_v2', output='screen', parameters=[common, {
                 'mission_mode': True,
                 'target_class': LaunchConfiguration('dock_target_class'),
                 'drive_enabled': ParameterValue(
                     LaunchConfiguration('dock_motion_enabled'), value_type=bool),
                 'approach_cmd': ParameterValue(
                     LaunchConfiguration('dock_approach_cmd'), value_type=float),
                 'final_cmd': ParameterValue(
                     LaunchConfiguration('dock_final_cmd'), value_type=float),
                 'gps_timeout_s': 2.0,
                 'odom_timeout_s': 2.0,
                 'perception_timeout_s': 2.0,
             }]),
        Node(package='kaboat_control', executable='cmd_mux',
             name='cmd_mux', output='screen', parameters=[common]),
        Node(package='kaboat_sim', executable='twist2thrust.py',
             name='twist2thrust', output='screen', parameters=[common],
             condition=IfCondition(LaunchConfiguration('use_sim_actuator'))),
        Node(package='kaboat_mission', executable='mission_manager',
             name='mission_manager', output='screen', parameters=[
                 mission_params,
                 {**common, 'start_state': 'dock',
                  'single_mission': True, 'auto_start': True,
                  'entry_waypoints.dock': [*staging, staging_heading],
                  'entry_ready.dock': True,
                  'waypoints.dock': staging,
                  'points_file': ''},       # 수조 좌표 — 야외 rec 기록 지점 안 읽음
             ]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('use_sim_actuator', default_value='false'),
        DeclareLaunchArgument('staging_x', description='실측 odom 정렬점 X [m]'),
        DeclareLaunchArgument('staging_y', description='실측 odom 정렬점 Y [m]'),
        DeclareLaunchArgument('staging_angle_deg',
                              description='정렬점 도착 선수각 [deg]'),
        DeclareLaunchArgument('dock_weights',
                              default_value=os.path.join(
                                  get_package_share_directory('kaboat_perception'),
                                  'models', 'best.pt'),
                              description='YOLO .pt 절대 경로'),
        DeclareLaunchArgument('dock_target_class',
                              description='triangle / circle / square'),
        DeclareLaunchArgument(
            'slot_config',
            default_value=os.path.join(
                get_package_share_directory('kaboat_bringup'),
                'config', 'dock_slot.yaml'),
            description='이전 CLI 호환용. one-shot RANSAC 도킹에서는 사용하지 않음'),
        DeclareLaunchArgument('retry_x', default_value='',
                              description='이전 CLI 호환용. 값 설정 시 시작 거부'),
        DeclareLaunchArgument('retry_y', default_value='',
                              description='이전 CLI 호환용. 값 설정 시 시작 거부'),
        DeclareLaunchArgument('retry_angle_deg', default_value='',
                              description='이전 CLI 호환용. 값 설정 시 시작 거부'),
        DeclareLaunchArgument('dock_device', default_value='cpu',
                              description='Ultralytics device: cpu 또는 0'),
        DeclareLaunchArgument('dock_base_frame', default_value='base_link'),
        DeclareLaunchArgument('dock_approach_cmd', default_value='0.10'),
        DeclareLaunchArgument('dock_final_cmd', default_value='0.05'),
        DeclareLaunchArgument('dock_motion_enabled', default_value='false',
                              description='기본 false: 센서 점검 뒤 명시적으로 true'),
        DeclareLaunchArgument('approach_motion_enabled', default_value='false',
                              description='기본 false: 시작 자세 접근은 미리보기만 수행'),
        OpaqueFunction(function=_nodes),
    ])
