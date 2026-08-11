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
            # ⚠️ 실내 수조에서는 apriltag_odom 이 /odom 을 발행하므로 이 remap 을
            # 꺼야 한다. 켠 채로 두면 EKF 가 수렴하는 순간 /odom 발행자가 둘이
            # 되어 두 좌표계가 섞인다 (GNSS 미수렴 중에는 조용해서 안 드러남).
            SetRemap(src='/ekf/odometry_map', dst='/odom',
                     condition=IfCondition(LaunchConfiguration('enable_odom_remap'))),
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
        DeclareLaunchArgument(
            'enable_odom_remap', default_value='true',
            description='GQ7 EKF → /odom remap. 실내 수조(apriltag_odom)에서는 false'),
        DeclareLaunchArgument(
            'publish_tf', default_value='true',
            description='RViz Fixed Frame 용 최소 TF. 스택 자체는 TF 를 쓰지 않는다'),
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
        # RViz 가 Fixed Frame 을 해석하려면 TF 트리가 있어야 한다. 실물에는
        # 발행자가 없어(gq7.yaml 의 tf_mode: 0) 여기서 최소 체인만 채운다 —
        # 제어 경로에는 관여하지 않는다.
        Node(
            package='kaboat_hardware',
            executable='odom_tf_broadcaster',
            name='odom_tf_broadcaster',
            output='screen',
            condition=IfCondition(LaunchConfiguration('publish_tf')),
            parameters=[{'use_sim_time': False}],
        ),
    ])
