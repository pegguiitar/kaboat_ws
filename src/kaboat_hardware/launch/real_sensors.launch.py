"""모터 출력을 전혀 실행하지 않는 실물 센서 배선 확인 launch."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    hardware_share = get_package_share_directory('kaboat_hardware')
    bringup_share = get_package_share_directory('kaboat_bringup')
    default_config = os.path.join(hardware_share, 'config', 'sensors.yaml')

    enable_d455 = LaunchConfiguration('enable_d455')
    config_file = LaunchConfiguration('config_file')

    d455 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, 'launch', 'd455.launch.py')),
        launch_arguments={'use_sim_time': 'false'}.items(),
        condition=IfCondition(enable_d455),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'enable_d455', default_value='false',
            description='realsense2_camera 설치 및 D455 연결 후 true'),
        DeclareLaunchArgument(
            'config_file', default_value=default_config,
            description='센서 토픽/최소 주기/timeout 설정'),
        LogInfo(
            msg='D455 driver disabled. 연결 후 enable_d455:=true 로 실행하세요.',
            condition=UnlessCondition(enable_d455)),
        d455,
        Node(
            package='kaboat_hardware',
            executable='sensor_health_monitor',
            name='sensor_health_monitor',
            output='screen',
            parameters=[config_file, {'use_sim_time': False}],
        ),
    ])
