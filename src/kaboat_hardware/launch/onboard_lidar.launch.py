"""Jetson TG-50: /scan/raw -> IMU tilt & structure-sector filter -> public /scan."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


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
        DeclareLaunchArgument(
            'enable_sector_mask', default_value='true',
            description='Mask boat structure returns (default: true)'),
        DeclareLaunchArgument(
            'mask_start_deg', default_value='220.0',
            description='First boat-structure bearing to discard [deg, 0..360]'),
        DeclareLaunchArgument(
            'mask_end_deg', default_value='320.0',
            description='Last boat-structure bearing to discard [deg, 0..360]'),
        DeclareLaunchArgument(
            'enable_imu_filter', default_value='true',
            description='Filter scans when boat roll or pitch exceeds threshold (default: true)'),
        DeclareLaunchArgument(
            'tilt_threshold_deg', default_value='5.0',
            description='Boat tilt threshold for scan filtering [deg] (default: 5.0)'),
        DeclareLaunchArgument(
            'tilt_action', default_value='drop',
            description='Action when tilted: "drop" (skip message) or "invalidate" (all inf)'),
        Node(
            package='ydlidar_ros2_driver',
            executable='ydlidar_ros2_driver_node',
            name='ydlidar_ros2_driver_node',
            output='screen',
            emulate_tty=True,
            parameters=[LaunchConfiguration('config_file'),
                        {'port': LaunchConfiguration('port')}],
            remappings=[('scan', '/scan/raw')],
        ),
        Node(
            package='kaboat_hardware',
            executable='onboard_scan_filter',
            name='onboard_scan_filter',
            output='screen',
            parameters=[{
                'enable_sector_mask': ParameterValue(
                    LaunchConfiguration('enable_sector_mask'), value_type=bool),
                'mask_start_deg': ParameterValue(
                    LaunchConfiguration('mask_start_deg'), value_type=float),
                'mask_end_deg': ParameterValue(
                    LaunchConfiguration('mask_end_deg'), value_type=float),
                'enable_imu_filter': ParameterValue(
                    LaunchConfiguration('enable_imu_filter'), value_type=bool),
                'tilt_threshold_deg': ParameterValue(
                    LaunchConfiguration('tilt_threshold_deg'), value_type=float),
                'tilt_action': ParameterValue(
                    LaunchConfiguration('tilt_action'), value_type=str),
            }],
        ),
    ])
