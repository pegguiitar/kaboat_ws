"""실물 스러스터 드라이버 launch 파일."""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    hardware_share = get_package_share_directory('kaboat_hardware')
    default_config = os.path.join(hardware_share, 'config', 'thrusters.yaml')

    config_file = LaunchConfiguration('config_file')
    hardware_type = LaunchConfiguration('hardware_type')

    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file', default_value=default_config,
            description='스러스터/PWM 파라미터 YAML 경로'),
        DeclareLaunchArgument(
            'hardware_type', default_value='dummy',
            description="하드웨어 백엔드 타입 ('dummy' | 'serial' | 'pca9685')"),
        DeclareLaunchArgument(
            'port', default_value='/dev/ttyCH341USB0',
            description='아두이노/ESP32 시리얼 포트 경로 (CH340 아두이노: /dev/ttyCH341USB0, CP2102: /dev/ttyUSB*)'),
        DeclareLaunchArgument(
            'allow_port_scan', default_value='false',
            description='지정 포트 실패 시 다른 USB serial 포트 자동 탐색 여부'),
        DeclareLaunchArgument(
            'max_output_ratio', default_value='1.0',
            description='믹싱 후 좌/우 스러스터 최종 절대 출력 상한 [0.0~1.0]'),
        DeclareLaunchArgument(
            'gps_logger', default_value='true',
            description='노트북 Space 로 GPS 위치를 CSV 에 기록하는 gps_mark_logger 실행'),
        DeclareLaunchArgument(
            'gps_log_dir', default_value='~/kaboat_gps_logs',
            description='GPS 기록 CSV 저장 폴더 (젯슨)'),

        Node(
            package='kaboat_hardware',
            executable='thruster_driver',
            name='thruster_driver',
            output='screen',
            parameters=[
                config_file,
                {
                    'hardware_type': hardware_type,
                    'port': LaunchConfiguration('port'),
                    'allow_port_scan': LaunchConfiguration('allow_port_scan'),
                    'max_output_ratio': LaunchConfiguration('max_output_ratio'),
                    'use_sim_time': False,
                }
            ],
        ),
        Node(
            package='kaboat_hardware',
            executable='gps_mark_logger',
            name='gps_mark_logger',
            output='screen',
            condition=IfCondition(LaunchConfiguration('gps_logger')),
            parameters=[{'log_dir': LaunchConfiguration('gps_log_dir')}],
        ),
    ])
