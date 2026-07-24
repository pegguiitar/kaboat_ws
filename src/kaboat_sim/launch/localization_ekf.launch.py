"""localization_ekf — 실물용 위치추정 파이프라인 (GNSS + 외부 IMU → EKF → /odom).

⚠️ sim 기본 실행(simulation.launch.py)에는 포함되지 않는다. sim 은 대신
odom_gps_noise 노드로 위치오차를 직접 모델링한다(그 노드 docstring 참고 —
요약: EKF 튜닝은 실 센서 노이즈에 묶여 sim 튜닝이 실물로 이월되지 않음).

이 파일은 실물 전환 시 쓸 "진짜 융합 파이프라인"을 보존한 것이다. 실물은
순수 GNSS(F9P/GQ7 — GQ7 은 내부 IMU 고장으로 GPS 만 사용) + 외부 IMU
(EBIMU)를 이 EKF 로 융합한다. 실 센서 스펙이 나오면 ekf.yaml 공분산과
navsat datum(월드 원점 정렬)을 그 값으로 맞추고, 이 launch 를 실물 bringup
에 포함하면 된다.

구성(robot_localization 표준 이중 노드):
  navsat_transform_node: /gps/fix + /imu/data → /odometry/gps (로컬 ENU)
  ekf_filter_node: /imu/data + /odometry/gps 를 ekf.yaml 대로 융합 → /odom

미해결(실물 튜닝 시 처리): ① navsat datum 을 월드 spherical_coordinates
원점에 고정해 /odom 원점을 월드 프레임과 정렬(현재는 첫 GPS 픽스 기준이라
어긋남) ② 공분산/프로세스노이즈 튜닝으로 GPS 노이즈 평활.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ekf_config_path = os.path.join(
        get_package_share_directory('kaboat_bringup'), 'config', 'ekf.yaml')

    navsat_transform_node = Node(
        package='robot_localization',
        executable='navsat_transform_node',
        name='navsat_transform_node',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'yaw_offset': 0.0,
            'magnetic_declination_radians': 0.0,
            'zero_altitude': True,
            'use_odometry_yaw': False,
            'wait_for_datum': False,
            'publish_filtered_gps': False,
        }],
        remappings=[
            ('imu', '/imu/data'),
            ('gps/fix', '/gps/fix'),
            ('odometry/filtered', '/odom'),
            ('odometry/gps', '/odometry/gps'),
        ],
    )

    ekf_filter_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config_path, {'use_sim_time': True}],
        remappings=[('odometry/filtered', '/odom')],
    )

    return LaunchDescription([navsat_transform_node, ekf_filter_node])
