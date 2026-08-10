"""모터 출력을 전혀 실행하지 않는 실물 센서 배선 확인 launch."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    LogInfo,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetRemap
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    hardware_share = get_package_share_directory('kaboat_hardware')
    bringup_share = get_package_share_directory('kaboat_bringup')
    default_config = os.path.join(hardware_share, 'config', 'sensors.yaml')
    default_gq7_config = os.path.join(hardware_share, 'config', 'gq7.yaml')

    enable_d455 = LaunchConfiguration('enable_d455')
    enable_gq7 = LaunchConfiguration('enable_gq7')
    config_file = LaunchConfiguration('config_file')
    gq7_config_file = LaunchConfiguration('gq7_config_file')

    d455 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, 'launch', 'd455.launch.py')),
        launch_arguments={'use_sim_time': 'false'}.items(),
        condition=IfCondition(enable_d455),
    )

    # 공식 MicroStrain launch를 사용하되, 나머지 자율주행 코드가 기대하는
    # KABOAT 표준 토픽으로 이름만 변환한다.
    gq7 = GroupAction(
        condition=IfCondition(enable_gq7),
        actions=[
            SetRemap(src='/gnss_1/llh_position', dst='/gps/fix'),
            SetRemap(src='/gnss_2/llh_position', dst='/gps/fix_secondary'),
            SetRemap(src='/ekf/odometry_map', dst='/odom'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([
                        FindPackageShare('microstrain_inertial_driver'),
                        'launch',
                        'microstrain_launch.py',
                    ])),
                launch_arguments={
                    'namespace': '/',
                    'node_name': 'microstrain_inertial_driver',
                    'params_file': gq7_config_file,
                }.items(),
            ),
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'enable_d455', default_value='false',
            description='realsense2_camera 설치 및 D455 연결 후 true'),
        DeclareLaunchArgument(
            'enable_gq7', default_value='true',
            description='GQ7 공식 드라이버와 KABOAT 토픽 remap 실행'),
        DeclareLaunchArgument(
            'config_file', default_value=default_config,
            description='센서 토픽/최소 주기/timeout 설정'),
        DeclareLaunchArgument(
            'gq7_config_file', default_value=default_gq7_config,
            description='GQ7 드라이버 override 설정'),
        LogInfo(
            msg='D455 driver disabled. 연결 후 enable_d455:=true 로 실행하세요.',
            condition=UnlessCondition(enable_d455)),
        LogInfo(
            msg='GQ7 driver disabled. enable_gq7:=true 로 실행하세요.',
            condition=UnlessCondition(enable_gq7)),
        d455,
        gq7,
        Node(
            package='kaboat_hardware',
            executable='sensor_health_monitor',
            name='sensor_health_monitor',
            output='screen',
            parameters=[config_file, {'use_sim_time': False}],
        ),
    ])
