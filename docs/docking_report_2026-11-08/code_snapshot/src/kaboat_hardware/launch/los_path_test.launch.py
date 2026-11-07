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
        DeclareLaunchArgument('thrust_ratio', default_value='0.3',
                              description='Ratio of motor thrust output [0..1] (0.3 = 30%)'),
        DeclareLaunchArgument('motion_enabled', default_value='true',
                              description='false: preview only; true: publish /cmd_vel'),
        DeclareLaunchArgument('wait_for_start', default_value='false',
                              description='false: start immediately; true: wait for /los_test/start'),
        DeclareLaunchArgument('max_linear', default_value='1.0',
                              description='Max linear output normalization factor'),
        DeclareLaunchArgument('max_angular', default_value='1.0',
                              description='Max angular output normalization factor'),
        DeclareLaunchArgument('yaw_kp', default_value='1.2',
                              description='Heading P gain'),
        DeclareLaunchArgument('yaw_kd', default_value='0.15',
                              description='Heading D gain'),
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
                'wait_for_start': ParameterValue(LaunchConfiguration('wait_for_start'),
                                                 value_type=bool),
                'max_linear': ParameterValue(LaunchConfiguration('max_linear'),
                                             value_type=float),
                'max_angular': ParameterValue(LaunchConfiguration('max_angular'),
                                              value_type=float),
                'yaw_kp': ParameterValue(LaunchConfiguration('yaw_kp'),
                                         value_type=float),
                'yaw_kd': ParameterValue(LaunchConfiguration('yaw_kd'),
                                         value_type=float),
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
