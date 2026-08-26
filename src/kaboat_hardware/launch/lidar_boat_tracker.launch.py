"""lidar_boat_tracker.launch.py — 실내 수조 고정 2D 라이다 배 위치 추적 런치."""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    hardware_share = get_package_share_directory('kaboat_hardware')
    bringup_share = get_package_share_directory('kaboat_bringup')

    default_config = os.path.join(hardware_share, 'config', 'lidar_tracker.yaml')
    default_rviz = os.path.join(bringup_share, 'rviz', 'tank_tracking.rviz')

    config_file = LaunchConfiguration('config_file')
    enable_rviz = LaunchConfiguration('enable_rviz')

    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file', default_value=default_config,
            description='라이다 배 위치 추적기 설정 파일 경로'),
        DeclareLaunchArgument(
            'enable_rviz', default_value='true',
            description='RViz2 시각화 자동 실행 여부'),

        LogInfo(msg='[LIDAR BOAT TRACKER] 수조 (5.0m, 0.0m) 고정 라이다 기반 배 위치 추적 시작...'),

        # 1. 배 위치 추적 노드
        Node(
            package='kaboat_hardware',
            executable='lidar_boat_tracker',
            name='lidar_boat_tracker',
            output='screen',
            parameters=[config_file, {'use_sim_time': False}],
        ),

        # 2. RViz2 시각화
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', default_rviz],
            parameters=[{'use_sim_time': False}],
            condition=IfCondition(enable_rviz),
        ),
    ])
