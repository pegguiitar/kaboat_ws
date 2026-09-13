"""실내 수조 시험 bringup — 외부 라이다(실내 GPS) + 선체 GQ7 IMU로 /odom 생성.

실외 GNSS가 안 잡히는 실내에서 `real_sensors.launch.py`를 대신합니다.

두 운용 케이스 구분:
  1) 야외 실전 (Outdoor Competition):
     - `ros2 launch kaboat_hardware real_sensors.launch.py` 실행.
     - 배에 장착된 GQ7 통합 INS가 RTK-GNSS + IMU EKF로 /odom 및 TF를 직접 발행합니다.
     - 외부 PC나 수조 라이다가 필요 없습니다.
  2) 실내 수조 시험 (Indoor Tank Test):
     - `indoor_tank.launch.py`는 실내이므로 GQ7 EKF의 /odom remap을 비활성화합니다 (enable_odom_remap:=false).
     - 외부 라이다(수조 옆 PC, lidar_boat_tracker)가 실내 GPS 역할을 하여 배 위치(/boat_position)를 발행합니다.
     - 배 내부(Jetson)의 indoor_lidar_odom 노드가 라이다 위치와 선체 IMU(/imu/data)를 융합하여 /odom 및 TF를 발행합니다.

사용법:
  # 1. 수조 외벽 노트북 (외부 라이다)
  ros2 launch kaboat_hardware lidar_boat_tracker.launch.py

  # 2. 배 (Jetson)
  ros2 launch kaboat_hardware indoor_tank.launch.py
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, LogInfo)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    hardware_share = get_package_share_directory('kaboat_hardware')
    default_config = os.path.join(hardware_share, 'config', 'indoor_tank.yaml')
    sensors_config = os.path.join(hardware_share, 'config', 'sensors.yaml')

    config_file = LaunchConfiguration('config_file')

    # 배 센서 드라이버 (GQ7 IMU 등) 실행
    sensors = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(hardware_share, 'launch', 'real_sensors.launch.py')),
        launch_arguments={
            'enable_d455': LaunchConfiguration('enable_d455'),
            'enable_gq7': LaunchConfiguration('enable_gq7'),
            # 핵심 — 실내에서는 GQ7 EKF remap을 끄고, indoor_lidar_odom이 /odom을 소유
            'enable_odom_remap': 'false',
            'publish_tf': 'false',  # indoor_lidar_odom이 odom->base_link TF를 직접 발행
            'config_file': sensors_config,
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'enable_d455', default_value='false',
            description='D455 카메라 연결 후 true'),
        DeclareLaunchArgument(
            'enable_gq7', default_value='true',
            description='GQ7 드라이버 — 실내에서는 IMU(/imu/data)만 사용'),
        DeclareLaunchArgument(
            'config_file', default_value=default_config,
            description='indoor_lidar_odom 파라미터 (수조 실측 설정값)'),

        LogInfo(msg='[INDOOR TANK] 실내 수조 모드: 외부 라이다(/boat_position, 실내 GPS) + 선체 GQ7 IMU(/imu/data) → /odom 융합. '
                    'GQ7 EKF remap은 꺼져 있습니다.'),

        sensors,

        # 실내 오도메트리 융합 노드 (외부 라이다 위치 + 선체 IMU)
        Node(
            package='kaboat_hardware',
            executable='indoor_lidar_odom',
            name='indoor_lidar_odom',
            output='screen',
            parameters=[config_file, {'use_sim_time': False}],
        ),
    ])
