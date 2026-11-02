"""One indoor-tank (x, y, heading) waypoint; sensors/odom run separately."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('x', description='수조/odom 목표 X [m]'),
        DeclareLaunchArgument('y', description='수조/odom 목표 Y [m]'),
        DeclareLaunchArgument('angle', description='목표 선수각 [deg]; 0=+X, 90=+Y'),
        DeclareLaunchArgument('thrust_ratio', default_value='1.0',
                              description='기존 waypoint 추력 상한 대비 비율 [0..1]'),
        DeclareLaunchArgument('motion_enabled', default_value='false',
                              description='기본값 false: /cmd_vel 발행 없이 명령 미리보기'),
        Node(
            package='kaboat_hardware', executable='indoor_waypoint',
            name='indoor_waypoint', output='screen',
            parameters=[{
                'target_x': ParameterValue(LaunchConfiguration('x'), value_type=float),
                'target_y': ParameterValue(LaunchConfiguration('y'), value_type=float),
                'target_angle_deg': ParameterValue(
                    LaunchConfiguration('angle'), value_type=float),
                'thrust_ratio': ParameterValue(
                    LaunchConfiguration('thrust_ratio'), value_type=float),
                'motion_enabled': ParameterValue(
                    LaunchConfiguration('motion_enabled'), value_type=bool),
                'use_sim_time': False,
            }],
        ),
    ])
