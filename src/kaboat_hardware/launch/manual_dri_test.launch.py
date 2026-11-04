"""Jetson: sensor-only DRI monitoring for remote manual driving.

Provide /scan and /odom separately. This launch never starts autonomy,
cmd_mux, teleop, a thruster driver, or any /cmd_vel publisher.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    config = os.path.join(get_package_share_directory('kaboat_hardware'),
                          'config', 'manual_dri_test.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('start_occupancy_grid', default_value='true',
                              description='false if /occupancy_grid already has a publisher'),
        DeclareLaunchArgument('grid_size', default_value='15.0',
                              description='Rolling occupancy map side length in metres'),
        DeclareLaunchArgument('enable_tilt_gate', default_value='true',
                              description='Reject scans at large IMU roll/pitch'),
        DeclareLaunchArgument('publish_scan_tf', default_value='true',
                              description='Publish identity base_link to onboard_laser_frame TF'),
        DeclareLaunchArgument('params_file', default_value=config,
                              description='DRI visualization and input timeout parameters'),
        LogInfo(msg='Manual DRI test: sensors and risk map only; no motor command source.'),
        Node(
            package='kaboat_perception', executable='occupancy_grid',
            name='occupancy_grid', output='screen',
            parameters=[{
                'resolution': 0.2,
                'size': ParameterValue(LaunchConfiguration('grid_size'), value_type=float),
                'publish_rate': 10.0,
                'enable_tilt_gate': ParameterValue(
                    LaunchConfiguration('enable_tilt_gate'), value_type=bool),
                'use_sim_time': False,
            }],
            condition=IfCondition(LaunchConfiguration('start_occupancy_grid')),
        ),
        Node(
            package='kaboat_hardware', executable='manual_dri_monitor',
            name='manual_dri_monitor', output='screen',
            parameters=[LaunchConfiguration('params_file'),
                        {'use_sim_time': False}],
        ),
        Node(
            package='tf2_ros', executable='static_transform_publisher',
            name='manual_dri_onboard_laser_tf', output='screen',
            arguments=['--x', '0', '--y', '0', '--z', '0',
                       '--roll', '0', '--pitch', '0', '--yaw', '0',
                       '--frame-id', 'base_link',
                       '--child-frame-id', 'onboard_laser_frame'],
            condition=IfCondition(LaunchConfiguration('publish_scan_tf')),
        ),
    ])
