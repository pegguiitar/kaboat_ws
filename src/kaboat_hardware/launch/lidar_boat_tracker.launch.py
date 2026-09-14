"""lidar_boat_tracker.launch.py — 실내 수조 TG50 라이다 배 위치 추적 통합 런치."""

import glob
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    hardware_share = get_package_share_directory('kaboat_hardware')
    bringup_share = get_package_share_directory('kaboat_bringup')

    default_tracker_config = os.path.join(hardware_share, 'config', 'lidar_tracker.yaml')
    default_tg50_config = os.path.join(hardware_share, 'config', 'tg50.yaml')
    default_rviz = os.path.join(bringup_share, 'rviz', 'tank_tracking.rviz')

    # 연결된 USB 포트 자동 탐지 (/dev/ttyUSB* 중 존재하는 것 선택)
    available_ports = sorted(glob.glob('/dev/ttyUSB*'))
    default_port = available_ports[0] if available_ports else '/dev/ttyUSB0'

    tracker_config_file = LaunchConfiguration('tracker_config_file')
    tg50_config_file = LaunchConfiguration('tg50_config_file')
    port = LaunchConfiguration('port')
    launch_driver = LaunchConfiguration('launch_driver')
    enable_rviz = LaunchConfiguration('enable_rviz')

    return LaunchDescription([
        DeclareLaunchArgument(
            'port', default_value=default_port,
            description=f'라이다 시리얼 포트 경로 (자동 감지: {default_port})'),
        DeclareLaunchArgument(
            'launch_driver', default_value='true',
            description='TG50 라이다 드라이버 동시 실행 여부 (true/false)'),
        DeclareLaunchArgument(
            'tg50_config_file', default_value=default_tg50_config,
            description='TG50 라이다 드라이버 파라미터 파일 경로'),
        DeclareLaunchArgument(
            'tracker_config_file', default_value=default_tracker_config,
            description='라이다 배 위치 추적기 설정 파일 경로'),
        DeclareLaunchArgument(
            'enable_rviz', default_value='true',
            description='RViz2 시각화 자동 실행 여부 (true/false)'),

        LogInfo(msg=f'[LIDAR BOAT TRACKER] 수조 (5.0m, 0.0m) 고정 TG50 라이다 (포트: {default_port}) 배 위치 추적 시작...'),

        # 1. TG50 YDLIDAR 드라이버 노드 (/scan 발행)
        Node(
            package='ydlidar_ros2_driver',
            executable='ydlidar_ros2_driver_node',
            name='ydlidar_ros2_driver_node',
            output='screen',
            emulate_tty=True,
            parameters=[tg50_config_file, {'port': port}],
            condition=IfCondition(launch_driver),
        ),

        # 2. 배 위치 추적 노드 (/detections, /boat_position, /odom 발행)
        Node(
            package='kaboat_hardware',
            executable='lidar_boat_tracker',
            name='lidar_boat_tracker',
            output='screen',
            parameters=[tracker_config_file, {'use_sim_time': False}],
        ),

        # 3. RViz2 시각화
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', default_rviz],
            parameters=[{'use_sim_time': False}],
            condition=IfCondition(enable_rviz),
        ),
    ])
