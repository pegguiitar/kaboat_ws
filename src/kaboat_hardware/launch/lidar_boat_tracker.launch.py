"""RPLIDAR A3 shore scan and indoor boat pose tracker."""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    hardware_share = get_package_share_directory('kaboat_hardware')
    bringup_share = get_package_share_directory('kaboat_bringup')

    default_tracker_config = os.path.join(hardware_share, 'config', 'lidar_tracker.yaml')
    default_sllidar_config = os.path.join(hardware_share, 'config', 'sllidar_a3.yaml')
    default_rviz = os.path.join(bringup_share, 'rviz', 'tank_tracking.rviz')

    # Measured CP2102 serial identity survives USB port/hub changes.
    default_port = '/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_344ff5135095724097887d9c6d4b91ce-if00-port0'

    tracker_config_file = LaunchConfiguration('tracker_config_file')
    sllidar_config_file = LaunchConfiguration('sllidar_config_file')
    port = LaunchConfiguration('port')
    launch_driver = LaunchConfiguration('launch_driver')
    enable_rviz = LaunchConfiguration('enable_rviz')

    def tracker_node(context):
        overrides = {
            'use_sim_time': False,
            'laser_frame': 'shore_laser_frame',
        }
        for name in ('lidar_pos_x', 'lidar_pos_y', 'lidar_yaw_deg'):
            value = LaunchConfiguration(name).perform(context).strip()
            if value:
                overrides[name] = float(value)
        return [Node(
            package='kaboat_hardware',
            executable='lidar_boat_tracker',
            name='lidar_boat_tracker',
            output='screen',
            parameters=[tracker_config_file, overrides],
            remappings=[('/scan', '/shore/scan')],
        )]

    return LaunchDescription([
        DeclareLaunchArgument(
            'port', default_value=default_port,
            description='RPLIDAR A3 serial port; override if the sensor changes'),
        DeclareLaunchArgument(
            'launch_driver', default_value='true',
            description='Run the shore RPLIDAR driver (true/false)'),
        DeclareLaunchArgument(
            'sllidar_config_file', default_value=default_sllidar_config,
            description='RPLIDAR A3 driver parameter file'),
        DeclareLaunchArgument(
            'tracker_config_file', default_value=default_tracker_config,
            description='라이다 배 위치 추적기 설정 파일 경로'),
        DeclareLaunchArgument(
            'lidar_pos_x', default_value='',
            description='수조 좌표계의 라이다 원점 X [m], 비우면 설정 파일 값 사용'),
        DeclareLaunchArgument(
            'lidar_pos_y', default_value='',
            description='수조 좌표계의 라이다 원점 Y [m], 비우면 설정 파일 값 사용'),
        DeclareLaunchArgument(
            'lidar_yaw_deg', default_value='',
            description='라이다 스캔 0°가 향하는 수조 방향 [deg], 비우면 설정 파일 값 사용'),
        DeclareLaunchArgument(
            'enable_rviz', default_value='true',
            description='RViz2 시각화 자동 실행 여부 (true/false)'),

        LogInfo(msg='[LIDAR BOAT TRACKER] Shore RPLIDAR A3 and boat pose tracker starting. Check installation position and yaw.'),

        # Shore scan stays separate from the Jetson onboard /scan.
        Node(
            package='sllidar_ros2',
            executable='sllidar_node',
            name='sllidar_node',
            output='screen',
            emulate_tty=True,
            parameters=[sllidar_config_file, {
                'serial_port': port,
                'frame_id': 'shore_laser_frame',
            }],
            remappings=[('scan', '/shore/scan')],
            condition=IfCondition(launch_driver),
        ),

        # 2. 횡단 판 선분으로 배 pose 추적 (/boat_pose, 호환용 /boat_position 발행)
        OpaqueFunction(function=tracker_node),

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
