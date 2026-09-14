"""straight_line_test.launch.py — 실내 수조 (9.0, 3.0) -> (1.0, 3.0) 직선 주행 테스트 런처.

사용법:
  # 기본값으로 실행 (9.0, 3.0 -> 1.0, 3.0, 속도 12%):
  ros2 launch kaboat_hardware straight_line_test.launch.py

  # 파라미터 변경 예시 (속도 15%로 상향):
  ros2 launch kaboat_hardware straight_line_test.launch.py cruise_speed:=0.15
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from kaboat_hardware.test_coordinates import STRAIGHT_LINE


def generate_launch_description():
    # 런치 인자 선언 (test_coordinates.py 기본값 참조)
    start_x_arg = DeclareLaunchArgument(
        'start_x', default_value=str(STRAIGHT_LINE['start_x']), description='시작점 X 좌표 [m]')
    start_y_arg = DeclareLaunchArgument(
        'start_y', default_value=str(STRAIGHT_LINE['start_y']), description='시작점 Y 좌표 [m]')
    goal_x_arg = DeclareLaunchArgument(
        'goal_x', default_value=str(STRAIGHT_LINE['goal_x']), description='목표점 X 좌표 [m]')
    goal_y_arg = DeclareLaunchArgument(
        'goal_y', default_value=str(STRAIGHT_LINE['goal_y']), description='목표점 Y 좌표 [m]')

    cruise_speed_arg = DeclareLaunchArgument(
        'cruise_speed', default_value='0.12',
        description='직진 순항 모터 출력 비율 (0.0 ~ 1.0, 기본 0.12 = 12%)')
    max_angular_arg = DeclareLaunchArgument(
        'max_angular', default_value='0.25',
        description='최대 회전 모터 출력 비율 (0.0 ~ 1.0, 기본 0.25 = 25%)')
    lookahead_dist_arg = DeclareLaunchArgument(
        'lookahead_dist', default_value='1.2',
        description='LOS 경로 추종 전방 주시 거리 [m]')
    goal_tol_arg = DeclareLaunchArgument(
        'goal_tolerance', default_value='0.35',
        description='도착 판정 반경 [m]')
    slow_radius_arg = DeclareLaunchArgument(
        'slow_radius', default_value='1.5',
        description='목표점 접근 감속 반경 [m]')
    kp_yaw_arg = DeclareLaunchArgument(
        'kp_yaw', default_value='0.35', description='헤딩 비례 게인 P')
    kd_yaw_arg = DeclareLaunchArgument(
        'kd_yaw', default_value='0.12', description='요레이트 미분(감쇠) 게인 D')
    odom_timeout_arg = DeclareLaunchArgument(
        'odom_timeout_sec', default_value='0.5',
        description='오도메트리 미수신 시 안전 정지 타임아웃 [초]')
    wait_for_start_arg = DeclareLaunchArgument(
        'wait_for_start', default_value='true',
        description='외부 시작 신호(/start_mission) 대기 여부 (기본값 true: 대기 후 출발)')

    node = Node(
        package='kaboat_hardware',
        executable='straight_line_test',
        name='straight_line_test',
        output='screen',
        parameters=[{
            'start_x': LaunchConfiguration('start_x'),
            'start_y': LaunchConfiguration('start_y'),
            'goal_x': LaunchConfiguration('goal_x'),
            'goal_y': LaunchConfiguration('goal_y'),
            'cruise_speed': LaunchConfiguration('cruise_speed'),
            'max_angular': LaunchConfiguration('max_angular'),
            'lookahead_dist': LaunchConfiguration('lookahead_dist'),
            'goal_tolerance': LaunchConfiguration('goal_tolerance'),
            'slow_radius': LaunchConfiguration('slow_radius'),
            'kp_yaw': LaunchConfiguration('kp_yaw'),
            'kd_yaw': LaunchConfiguration('kd_yaw'),
            'odom_timeout_sec': LaunchConfiguration('odom_timeout_sec'),
            'wait_for_start': LaunchConfiguration('wait_for_start'),
            'use_sim_time': False,
        }],
    )

    return LaunchDescription([
        start_x_arg,
        start_y_arg,
        goal_x_arg,
        goal_y_arg,
        cruise_speed_arg,
        max_angular_arg,
        lookahead_dist_arg,
        goal_tol_arg,
        slow_radius_arg,
        kp_yaw_arg,
        kd_yaw_arg,
        odom_timeout_arg,
        wait_for_start_arg,
        LogInfo(msg='[StraightLineTest] 수조 직선 주행 테스트 노드 시작 중...'),
        node,
    ])

