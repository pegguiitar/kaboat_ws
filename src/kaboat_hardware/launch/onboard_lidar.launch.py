"""Jetson-mounted TG-50 publishes /scan with returns within 0.40 m masked."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    hardware_share = get_package_share_directory('kaboat_hardware')
    default_config = os.path.join(
        hardware_share, 'config', 'tg50_onboard.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'port', default_value='/dev/ttyUSB0',
            description='Jetson TG-50 serial port; prefer /dev/serial/by-id/...'),
        DeclareLaunchArgument(
            'config_file', default_value=default_config,
            description='TG-50 settings with 0.40 m minimum range'),
        Node(
            package='ydlidar_ros2_driver',
            executable='ydlidar_ros2_driver_node',
            name='ydlidar_ros2_driver_node',
            output='screen',
            emulate_tty=True,
            parameters=[LaunchConfiguration('config_file'),
                        {'port': LaunchConfiguration('port')}],
            remappings=[('scan', '/scan')],
        ),
    ])
