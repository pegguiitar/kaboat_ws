"""Run only the shore RPLIDAR A3 scan and its RViz view."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('kaboat_hardware')
    config = os.path.join(share, 'config', 'sllidar_a3.yaml')
    rviz_config = os.path.join(share, 'rviz', 'shore_scan.rviz')
    default_port = '/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_344ff5135095724097887d9c6d4b91ce-if00-port0'

    port = LaunchConfiguration('port')
    enable_rviz = LaunchConfiguration('enable_rviz')

    return LaunchDescription([
        DeclareLaunchArgument('port', default_value=default_port,
                              description='Shore RPLIDAR serial device by-id path'),
        DeclareLaunchArgument('enable_rviz', default_value='true',
                              description='Show the scan in RViz'),
        Node(
            package='sllidar_ros2',
            executable='sllidar_node',
            name='sllidar_node',
            output='screen',
            emulate_tty=True,
            parameters=[config, {
                'serial_port': port,
                'frame_id': 'shore_laser_frame',
            }],
            remappings=[('scan', '/shore/scan')],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='shore_scan_rviz',
            output='screen',
            arguments=['-d', rviz_config],
            condition=IfCondition(enable_rviz),
        ),
    ])
