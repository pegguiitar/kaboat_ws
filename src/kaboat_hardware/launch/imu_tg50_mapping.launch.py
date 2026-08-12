"""GPS/AprilTag 전 IMU dead-reckoning + TG-50 Occupancy Grid 단기 시험.

모터/행동/미션 노드는 실행하지 않는다. IMU 이중 적분 /odom은 센서 파이프라인
확인 전용이며 실제 항법에 사용하면 안 된다.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    hardware_share = get_package_share_directory('kaboat_hardware')
    bringup_share = get_package_share_directory('kaboat_bringup')
    gq7_config = os.path.join(hardware_share, 'config', 'gq7.yaml')
    tg50_config = os.path.join(hardware_share, 'config', 'tg50.yaml')
    odom_config = os.path.join(
        hardware_share, 'config', 'imu_dead_reckoning.yaml')
    rviz_config = os.path.join(
        bringup_share, 'rviz', 'occupancy_grid.rviz')

    gq7 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('microstrain_inertial_driver'),
            'launch', 'microstrain_launch.py',
        ])),
        launch_arguments={
            'namespace': '/',
            'node_name': 'microstrain_inertial_driver',
            'params_file': LaunchConfiguration('gq7_config_file'),
        }.items(),
        condition=IfCondition(LaunchConfiguration('enable_gq7')),
    )

    tg50 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('ydlidar_ros2_driver'),
            'launch', 'ydlidar_launch.py',
        ])),
        launch_arguments={
            'params_file': LaunchConfiguration('tg50_config_file'),
        }.items(),
        condition=IfCondition(LaunchConfiguration('enable_tg50')),
    )

    return LaunchDescription([
        DeclareLaunchArgument('enable_gq7', default_value='true'),
        DeclareLaunchArgument('enable_tg50', default_value='true'),
        DeclareLaunchArgument('enable_occupancy_grid', default_value='true'),
        DeclareLaunchArgument('enable_rviz', default_value='true'),
        DeclareLaunchArgument('gq7_config_file', default_value=gq7_config),
        DeclareLaunchArgument('tg50_config_file', default_value=tg50_config),
        DeclareLaunchArgument('odom_config_file', default_value=odom_config),
        DeclareLaunchArgument('rviz_config', default_value=rviz_config),

        LogInfo(msg='[TEST ONLY] IMU dead-reckoning + TG-50 mapping. '
                    '모터 주행 및 장기 항법에 사용하지 마세요.'),
        gq7,
        tg50,
        Node(
            package='kaboat_hardware',
            executable='imu_dead_reckoning_odom',
            name='imu_dead_reckoning_odom',
            output='screen',
            parameters=[LaunchConfiguration('odom_config_file'),
                        {'use_sim_time': False}],
        ),
        Node(
            package='kaboat_perception',
            executable='occupancy_grid',
            name='occupancy_grid',
            output='screen',
            parameters=[{'use_sim_time': False}],
            condition=IfCondition(LaunchConfiguration('enable_occupancy_grid')),
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', LaunchConfiguration('rviz_config')],
            parameters=[{'use_sim_time': False}],
            condition=IfCondition(LaunchConfiguration('enable_rviz')),
        ),
    ])
