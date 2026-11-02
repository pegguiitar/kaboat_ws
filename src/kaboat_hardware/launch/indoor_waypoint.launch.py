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
        DeclareLaunchArgument('thrust_ratio', default_value='0.3',
                              description='기존 waypoint 추력 상한 대비 비율 [0..1] (기본 0.3 = 30%)'),
        DeclareLaunchArgument('motion_enabled', default_value='true',
                              description='기본값 true: /cmd_vel 모터 구동 명령 발행'),
        DeclareLaunchArgument('max_linear', default_value='1.0',
                              description='실물 전진 정규화 상한 (1.0 = 100%)'),
        DeclareLaunchArgument('max_angular', default_value='1.0',
                              description='실물 회전 정규화 상한 (1.0 = 100%)'),
        DeclareLaunchArgument('yaw_kp', default_value='1.2',
                              description='헤딩 P 게인'),
        DeclareLaunchArgument('yaw_kd', default_value='0.15',
                              description='요레이트 D 게인'),
        DeclareLaunchArgument('position_tolerance', default_value='0.50',
                              description='목표점 위치 도달 판정 반경 [m] (기본: 0.50m)'),
        DeclareLaunchArgument('heading_tolerance_deg', default_value='15.0',
                              description='최종 헤딩 정렬 허용 오차 [deg] (기본: 15.0°)'),
        DeclareLaunchArgument('slow_radius', default_value='1.5',
                              description='목표점 접근 시 감속 시작 반경 [m] (기본: 1.5m)'),
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
                'max_linear': ParameterValue(
                    LaunchConfiguration('max_linear'), value_type=float),
                'max_angular': ParameterValue(
                    LaunchConfiguration('max_angular'), value_type=float),
                'yaw_kp': ParameterValue(
                    LaunchConfiguration('yaw_kp'), value_type=float),
                'yaw_kd': ParameterValue(
                    LaunchConfiguration('yaw_kd'), value_type=float),
                'position_tolerance': ParameterValue(
                    LaunchConfiguration('position_tolerance'), value_type=float),
                'heading_tolerance_deg': ParameterValue(
                    LaunchConfiguration('heading_tolerance_deg'), value_type=float),
                'slow_radius': ParameterValue(
                    LaunchConfiguration('slow_radius'), value_type=float),
                'use_sim_time': False,
            }],
        ),
    ])
