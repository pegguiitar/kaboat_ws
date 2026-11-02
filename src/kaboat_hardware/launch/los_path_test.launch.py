"""Test the shared B-spline or circle LOS follower in the indoor tank."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('path_kind', default_value='bspline',
                              description='bspline or circle'),
        DeclareLaunchArgument('thrust_ratio', default_value='0.5',
                              description='Ratio of existing safe thrust caps [0..1]'),
        DeclareLaunchArgument('motion_enabled', default_value='false',
                              description='false: preview only; true: publish /cmd_vel'),
        DeclareLaunchArgument('center_x', default_value='5.0'),
        DeclareLaunchArgument('center_y', default_value='2.5'),
        DeclareLaunchArgument('radius', default_value='1.2'),
        DeclareLaunchArgument('direction', default_value='ccw'),
        DeclareLaunchArgument('target_laps', default_value='2.0',
                              description='0: continuous orbit'),
        Node(
            package='kaboat_hardware', executable='los_path_test',
            name='los_path_test', output='screen',
            parameters=[{
                'path_kind': LaunchConfiguration('path_kind'),
                'thrust_ratio': ParameterValue(LaunchConfiguration('thrust_ratio'),
                                               value_type=float),
                'motion_enabled': ParameterValue(LaunchConfiguration('motion_enabled'),
                                                 value_type=bool),
                'center_x': ParameterValue(LaunchConfiguration('center_x'),
                                           value_type=float),
                'center_y': ParameterValue(LaunchConfiguration('center_y'),
                                           value_type=float),
                'radius': ParameterValue(LaunchConfiguration('radius'),
                                         value_type=float),
                'direction': LaunchConfiguration('direction'),
                'target_laps': ParameterValue(LaunchConfiguration('target_laps'),
                                              value_type=float),
                'use_sim_time': False,
            }],
        ),
    ])
