"""circle_drive_test.launch.py — 실내 수조 원형 선회 주행 테스트 런치."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from kaboat_hardware.test_coordinates import CIRCLE_DRIVE


def generate_launch_description():
    center_x_arg = DeclareLaunchArgument(
        'center_x', default_value=str(CIRCLE_DRIVE['center_x']), description='원 중심 X 좌표 [m] (수조 중앙)')
    center_y_arg = DeclareLaunchArgument(
        'center_y', default_value=str(CIRCLE_DRIVE['center_y']), description='원 중심 Y 좌표 [m] (수조 중앙)')
    radius_arg = DeclareLaunchArgument(
        'radius', default_value=str(CIRCLE_DRIVE['radius']), description='선회 원 반경 [m]')
    direction_arg = DeclareLaunchArgument(
        'direction', default_value=str(CIRCLE_DRIVE['direction']), description='선회 방향 (ccw: 반시계, cw: 시계)')
    target_laps_arg = DeclareLaunchArgument(
        'target_laps', default_value=str(CIRCLE_DRIVE['target_laps']), description='목표 바퀴 수 (0.0: 무한 회전)')
    cruise_speed_arg = DeclareLaunchArgument(
        'cruise_speed', default_value='0.12', description='순항 전진 출력 비 (0.0 ~ 1.0)')
    max_angular_arg = DeclareLaunchArgument(
        'max_angular', default_value='0.80', description='최대 회전 출력 비 (0.0 ~ 1.0)')
    kp_yaw_arg = DeclareLaunchArgument(
        'kp_yaw', default_value='1.0', description='헤딩 P 게인')
    kd_yaw_arg = DeclareLaunchArgument(
        'kd_yaw', default_value='0.15', description='요레이트 D 게인')
    k_converge_arg = DeclareLaunchArgument(
        'k_converge', default_value='1.5', description='원 궤도 진입 수렴 게인')
    odom_timeout_arg = DeclareLaunchArgument(
        'odom_timeout_sec', default_value='0.5', description='오도메트리 미수신 안전 정지 타임아웃 [초]')
    wait_for_start_arg = DeclareLaunchArgument(
        'wait_for_start', default_value='true',
        description='외부 시작 신호(/start_mission) 대기 여부 (기본값 true: 대기 후 출발)')

    node = Node(
        package='kaboat_hardware',
        executable='circle_drive_test',
        name='circle_drive_test',
        output='screen',
        parameters=[{
            'center_x': LaunchConfiguration('center_x'),
            'center_y': LaunchConfiguration('center_y'),
            'radius': LaunchConfiguration('radius'),
            'direction': LaunchConfiguration('direction'),
            'target_laps': LaunchConfiguration('target_laps'),
            'cruise_speed': LaunchConfiguration('cruise_speed'),
            'max_angular': LaunchConfiguration('max_angular'),
            'kp_yaw': LaunchConfiguration('kp_yaw'),
            'kd_yaw': LaunchConfiguration('kd_yaw'),
            'k_converge': LaunchConfiguration('k_converge'),
            'odom_timeout_sec': LaunchConfiguration('odom_timeout_sec'),
            'wait_for_start': LaunchConfiguration('wait_for_start'),
            'use_sim_time': False,
        }],
    )

    return LaunchDescription([
        center_x_arg,
        center_y_arg,
        radius_arg,
        direction_arg,
        target_laps_arg,
        cruise_speed_arg,
        max_angular_arg,
        kp_yaw_arg,
        kd_yaw_arg,
        k_converge_arg,
        odom_timeout_arg,
        wait_for_start_arg,
        LogInfo(msg='[CircleDriveTest] 실내 수조 원형 선회 주행 테스트 노드 시작 중...'),
        node,
    ])

