"""GQ7 위도·경도 → 고정 원점 기준 x/y 변환 노드 (geo_origin)."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_config = os.path.join(
        get_package_share_directory('kaboat_hardware'), 'config', 'geo_origin.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('config_file', default_value=default_config,
                              description='geo_origin 설정 YAML (원점·목표 위도/경도)'),
        Node(package='kaboat_hardware', executable='geo_origin', name='geo_origin',
             output='screen', parameters=[LaunchConfiguration('config_file')]),
    ])
