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
        DeclareLaunchArgument('thrust_ratio', default_value='0.3',
                              description='Ratio of motor thrust output [0..1] (0.3 = 30%)'),
        DeclareLaunchArgument('motion_enabled', default_value='true',
                              description='false: preview only; true: publish /cmd_vel'),
        DeclareLaunchArgument('wait_for_start', default_value='false',
                              description='false: start immediately; true: wait for /waypoint_sequence/start service'),
        DeclareLaunchArgument('max_linear', default_value='1.0',
                              description='Max linear output normalization factor'),
        DeclareLaunchArgument('max_angular', default_value='1.0',
                              description='Max angular output normalization factor'),
        DeclareLaunchArgument('yaw_kp', default_value='1.2',
                              description='Heading P gain'),
        DeclareLaunchArgument('yaw_kd', default_value='0.15',
                              description='Heading D gain'),
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
                    'wait_for_start': ParameterValue(
                        LaunchConfiguration('wait_for_start'), value_type=bool),
                    'max_linear': ParameterValue(
                        LaunchConfiguration('max_linear'), value_type=float),
                    'max_angular': ParameterValue(
                        LaunchConfiguration('max_angular'), value_type=float),
                    'yaw_kp': ParameterValue(
                        LaunchConfiguration('yaw_kp'), value_type=float),
                    'yaw_kd': ParameterValue(
                        LaunchConfiguration('yaw_kd'), value_type=float),
                    'hold_duration_sec': ParameterValue(
                        LaunchConfiguration('hold_duration_sec'), value_type=float),
                    'use_sim_time': False,
                },
            ],
        ),
    ])
