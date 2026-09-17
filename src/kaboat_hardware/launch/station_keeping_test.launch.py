"""station_keeping_test.launch.py — 실내 수조 웨이포인트 정점 유지(DP) 테스트 런치."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from kaboat_hardware.test_coordinates import STATION_KEEPING


def generate_launch_description():
    target_x_arg = DeclareLaunchArgument(
        'target_x', default_value=str(STATION_KEEPING['target_x']), description='목표 X 좌표 [m] (수조 중앙)')
    target_y_arg = DeclareLaunchArgument(
        'target_y', default_value=str(STATION_KEEPING['target_y']), description='목표 Y 좌표 [m] (수조 중앙)')
    target_yaw_deg_arg = DeclareLaunchArgument(
        'target_yaw_deg', default_value=str(STATION_KEEPING['target_yaw_deg']), description='목표 선수각 [deg] (-999: 위치만 유지)')
    hold_duration_arg = DeclareLaunchArgument(
        'hold_duration_sec', default_value=str(STATION_KEEPING.get('hold_duration_sec', 0.0)), description='유지 시간 [초] (0.0: 무한 유지)')
    pos_deadband_arg = DeclareLaunchArgument(
        'pos_deadband', default_value=str(STATION_KEEPING.get('pos_deadband', 0.15)), description='위치 불감대 허용 반경 [m]')
    yaw_deadband_deg_arg = DeclareLaunchArgument(
        'yaw_deadband_deg', default_value=str(STATION_KEEPING.get('yaw_deadband_deg', 8.0)), description='헤딩 불감대 허용 각도 [deg]')
    max_fwd_speed_arg = DeclareLaunchArgument(
        'max_fwd_speed', default_value='0.50', description='최대 전진 출력 비 (기본 50%)')
    max_rev_speed_arg = DeclareLaunchArgument(
        'max_rev_speed', default_value='0.08', description='최대 후진 출력 비')
    max_angular_arg = DeclareLaunchArgument(
        'max_angular', default_value='0.60', description='최대 회전 출력 비 (기본 60%)')
    kp_pos_arg = DeclareLaunchArgument(
        'kp_pos', default_value='0.25', description='위치 오차 P 게인')
    kp_yaw_arg = DeclareLaunchArgument(
        'kp_yaw', default_value='1.0', description='헤딩 오차 P 게인')
    kd_yaw_arg = DeclareLaunchArgument(
        'kd_yaw', default_value='0.15', description='요레이트 D 게인')
    odom_timeout_arg = DeclareLaunchArgument(
        'odom_timeout_sec', default_value='0.5', description='오도메트리 미수신 안전 정지 타임아웃 [초]')
    wait_for_start_arg = DeclareLaunchArgument(
        'wait_for_start', default_value='true',
        description='외부 시작 신호(/start_mission) 대기 여부 (기본값 true: 대기 후 출발)')

    node = Node(
        package='kaboat_hardware',
        executable='station_keeping_test',
        name='station_keeping_test',
        output='screen',
        parameters=[{
            'target_x': LaunchConfiguration('target_x'),
            'target_y': LaunchConfiguration('target_y'),
            'target_yaw_deg': LaunchConfiguration('target_yaw_deg'),
            'hold_duration_sec': LaunchConfiguration('hold_duration_sec'),
            'pos_deadband': LaunchConfiguration('pos_deadband'),
            'yaw_deadband_deg': LaunchConfiguration('yaw_deadband_deg'),
            'max_fwd_speed': LaunchConfiguration('max_fwd_speed'),
            'max_rev_speed': LaunchConfiguration('max_rev_speed'),
            'max_angular': LaunchConfiguration('max_angular'),
            'kp_pos': LaunchConfiguration('kp_pos'),
            'kp_yaw': LaunchConfiguration('kp_yaw'),
            'kd_yaw': LaunchConfiguration('kd_yaw'),
            'odom_timeout_sec': LaunchConfiguration('odom_timeout_sec'),
            'wait_for_start': LaunchConfiguration('wait_for_start'),
            'use_sim_time': False,
        }],
    )

    return LaunchDescription([
        target_x_arg,
        target_y_arg,
        target_yaw_deg_arg,
        hold_duration_arg,
        pos_deadband_arg,
        yaw_deadband_deg_arg,
        max_fwd_speed_arg,
        max_rev_speed_arg,
        max_angular_arg,
        kp_pos_arg,
        kp_yaw_arg,
        kd_yaw_arg,
        odom_timeout_arg,
        wait_for_start_arg,
        LogInfo(msg='[StationKeepingTest] 실내 수조 웨이포인트 정점 유지(DP) 테스트 노드 시작 중...'),
        node,
    ])

