"""Five sequential tank waypoints with preview-first command output."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    default_config = PathJoinSubstitution([
        FindPackageShare('kaboat_hardware'), 'config',
        'waypoint_sequence_test.yaml'])
    return LaunchDescription([
        DeclareLaunchArgument('config_file', default_value=default_config,
                              description='YAML file with five tank waypoints'),
        DeclareLaunchArgument('thrust_ratio', default_value='0.5',
                              description='Ratio of existing safe thrust caps [0..1]'),
        DeclareLaunchArgument('motion_enabled', default_value='false',
                              description='false: preview only; true: publish /cmd_vel'),
        DeclareLaunchArgument('hold_duration_sec', default_value='3.0',
                              description='Continuous hold at each point before advancing [s]'),
        Node(
            package='kaboat_hardware', executable='waypoint_sequence_test',
            name='waypoint_sequence_test', output='screen',
            parameters=[
                LaunchConfiguration('config_file'),
                {
                    'thrust_ratio': ParameterValue(
                        LaunchConfiguration('thrust_ratio'), value_type=float),
                    'motion_enabled': ParameterValue(
                        LaunchConfiguration('motion_enabled'), value_type=bool),
                    'hold_duration_sec': ParameterValue(
                        LaunchConfiguration('hold_duration_sec'), value_type=float),
                    'use_sim_time': False,
                },
            ],
        ),
    ])
