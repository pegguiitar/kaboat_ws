"""실내 수조 시험 bringup — 천장 AprilTag + GQ7 자이로로 /odom 을 만든다.

실외 GNSS 가 안 잡히는 실내에서 `real_sensors.launch.py` 를 대신한다.
차이는 두 가지뿐이다:
  1) GQ7 은 **IMU 만** 쓴다 — EKF → /odom remap 을 끈다
     (안 끄면 EKF 가 수렴하는 순간 /odom 발행자가 둘이 된다)
  2) /odom 은 apriltag_odom 이 발행한다

모터는 실행하지 않는다 — real_sensors.launch.py 와 같은 규약.

AprilTag 검출기는 **이 launch 에 없다.** 천장 카메라는 별도 노트북에서
`ceiling_apriltag.launch.py`로 실행한다. 영상은 노트북에서 처리하고 /tf와
/apriltag/detections만 DDS로 공유한다. 자세한 설치·보정·Wi-Fi 설정은 저장소
루트의 APRILTAG_WIFI_SETUP.md를 따른다.

    # 노트북 — tag_size는 인쇄물 실측값[m]
    ros2 launch kaboat_hardware ceiling_apriltag.launch.py \
        tag_id:=0 tag_size:=0.162

    # 두 PC의 ROS_DOMAIN_ID를 맞추고 ROS_LOCALHOST_ONLY=0으로 둘 것

사용법:
    ros2 launch kaboat_hardware indoor_tank.launch.py
    ros2 launch kaboat_bringup rviz.launch.py publish_tf:=false   # TF 중복 방지
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

    sensors = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(hardware_share, 'launch', 'real_sensors.launch.py')),
        launch_arguments={
            'enable_d455': LaunchConfiguration('enable_d455'),
            'enable_gq7': LaunchConfiguration('enable_gq7'),
            # 핵심 — /odom 은 apriltag_odom 이 단독으로 소유한다
            'enable_odom_remap': 'false',
            'publish_tf': LaunchConfiguration('publish_odom_tf'),
            'config_file': sensors_config,
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'enable_d455', default_value='false',
            description='D455 연결 후 true'),
        DeclareLaunchArgument(
            'enable_gq7', default_value='true',
            description='GQ7 드라이버 — 실내에서는 IMU 만 쓴다'),
        DeclareLaunchArgument(
            'config_file', default_value=default_config,
            description='apriltag_odom 파라미터 (수조 실측값)'),
        DeclareLaunchArgument(
            'publish_odom_tf', default_value='true',
            description='RViz 용 odom→base_link TF (real_sensors 가 발행)'),

        LogInfo(msg='실내 수조 모드 — /odom 은 AprilTag, 각속도는 GQ7 자이로. '
                    'GQ7 EKF remap 은 꺼져 있다.'),

        sensors,
        # odom_tf_broadcaster 는 real_sensors 가 publish_tf 로 띄운다 (중복 방지)
        Node(
            package='kaboat_hardware', executable='apriltag_odom',
            name='apriltag_odom', output='screen',
            parameters=[config_file, {'use_sim_time': False}],
        ),
    ])
