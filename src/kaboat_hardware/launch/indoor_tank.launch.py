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
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    hardware_share = get_package_share_directory('kaboat_hardware')
    default_config = os.path.join(hardware_share, 'config', 'indoor_tank.yaml')
    sensors_config = os.path.join(hardware_share, 'config', 'sensors.yaml')

    tank_config_file = LaunchConfiguration('tank_config_file')
    imu_yaw_offset_deg = LaunchConfiguration('imu_yaw_offset_deg')
    enable_thrusters = LaunchConfiguration('enable_thrusters')
    thruster_hw = LaunchConfiguration('thruster_hardware_type')
    thruster_port = LaunchConfiguration('thruster_port')

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

    # 모터 드라이버 실행 (기본값 serial)
    thrusters = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(hardware_share, 'launch', 'thrusters.launch.py')),
        launch_arguments={
            'hardware_type': thruster_hw,
            'port': thruster_port,
        }.items(),
        condition=IfCondition(enable_thrusters),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'enable_d455', default_value='false',
            description='D455 카메라 연결 후 true'),
        DeclareLaunchArgument(
            'enable_gq7', default_value='true',
            description='GQ7 드라이버 — 실내에서는 IMU(/imu/data)만 사용'),
        DeclareLaunchArgument(
            'enable_thrusters', default_value='false',
            description='스러스터 드라이버 동시 실행 여부 (기본값: false, 별도 실행 권장)'),
        DeclareLaunchArgument(
            'thruster_hardware_type', default_value='serial',
            description="스러스터 하드웨어 타입 ('serial' | 'dummy' | 'pca9685')"),
        DeclareLaunchArgument(
            'thruster_port', default_value='/dev/ttyUSB0',
            description='스러스터 시리얼 포트 경로 (기본값: /dev/ttyUSB0)'),
        DeclareLaunchArgument(
            'tank_config_file', default_value=default_config,
            description='indoor_lidar_odom 파라미터 (수조 실측 설정값)'),
        DeclareLaunchArgument(
            'imu_yaw_offset_deg', default_value='20.60',
            description='수조 +X축(0도) 기준 IMU 설치 편차 각도 [deg] (-X 방향 정렬 시 20.60도 보정)'),

        LogInfo(msg='[INDOOR TANK] 실내 수조 모드: 외부 라이다(/boat_position, 실내 GPS) + 선체 GQ7 IMU(/imu/data) → /odom 융합 + 모터 드라이버 실행.'),

        sensors,
        thrusters,

        # 실내 오도메트리 융합 노드 (외부 라이다 위치 + 선체 IMU)
        Node(
            package='kaboat_hardware',
            executable='indoor_lidar_odom',
            name='indoor_lidar_odom',
            output='screen',
            parameters=[
                tank_config_file,
                {
                    'use_sim_time': False,
                    'imu_yaw_offset_deg': imu_yaw_offset_deg,
                }
            ],
        ),
    ])
