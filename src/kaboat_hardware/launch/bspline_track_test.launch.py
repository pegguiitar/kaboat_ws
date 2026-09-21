"""bspline_track_test.launch.py — 실내 수조 (10m x 5m) B-Spline 곡선 경로 추종 테스트 런처.

사용법:
  # 기본 S자 슬라럼 코스 추종 (30% 출력):
  ros2 launch kaboat_hardware bspline_track_test.launch.py

  # 속도를 20%로 낮춰 실행:
  ros2 launch kaboat_hardware bspline_track_test.launch.py cruise_speed:=0.20

  # 전방 주시거리(lookahead)를 1.5m로 늘려 더 완만하게 회전:
  ros2 launch kaboat_hardware bspline_track_test.launch.py lookahead_dist:=1.5
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    cruise_speed_arg = DeclareLaunchArgument(
        'cruise_speed', default_value='0.30',
        description='직진 순항 모터 출력 비율 (0.0 ~ 1.0, 기본 0.30 = 30%)')
    max_angular_arg = DeclareLaunchArgument(
        'max_angular', default_value='0.60',
        description='최대 회전 모터 출력 비율 (0.0 ~ 1.0, 기본 0.60 = 60%)')
    lookahead_dist_arg = DeclareLaunchArgument(
        'lookahead_dist', default_value='0.6',
        description='Lookahead 전방 주시 거리 [m]')
    goal_tol_arg = DeclareLaunchArgument(
        'goal_tolerance', default_value='0.40',
        description='종점 도착 판정 반경 [m]')
    slow_radius_arg = DeclareLaunchArgument(
        'slow_radius', default_value='1.5',
        description='종점 접근 감속 시작 반경 [m]')
    kp_yaw_arg = DeclareLaunchArgument(
        'kp_yaw', default_value='1.5', description='헤딩 비례 게인 P')
    kd_yaw_arg = DeclareLaunchArgument(
        'kd_yaw', default_value='0.15', description='요레이트 감쇠 게인 D')
    curvature_slowdown_arg = DeclareLaunchArgument(
        'curvature_slowdown', default_value='0.25',
        description='곡률 기반 자동 감속 가중치')
    odom_timeout_arg = DeclareLaunchArgument(
        'odom_timeout_sec', default_value='0.5',
        description='오도메트리 미수신 시 안전 정지 타임아웃 [초]')
    wait_for_start_arg = DeclareLaunchArgument(
        'wait_for_start', default_value='true',
        description='외부 시작 신호(/start_mission) 대기 여부 (기본값 true: 대기 후 출발)')

    node = Node(
        package='kaboat_hardware',
        executable='bspline_track_test',
        name='bspline_track_test',
        output='screen',
        parameters=[{
            'cruise_speed': LaunchConfiguration('cruise_speed'),
            'max_angular': LaunchConfiguration('max_angular'),
            'lookahead_dist': LaunchConfiguration('lookahead_dist'),
            'goal_tolerance': LaunchConfiguration('goal_tolerance'),
            'slow_radius': LaunchConfiguration('slow_radius'),
            'kp_yaw': LaunchConfiguration('kp_yaw'),
            'kd_yaw': LaunchConfiguration('kd_yaw'),
            'curvature_slowdown': LaunchConfiguration('curvature_slowdown'),
            'odom_timeout_sec': LaunchConfiguration('odom_timeout_sec'),
            'wait_for_start': LaunchConfiguration('wait_for_start'),
            'use_sim_time': False,
        }],
    )

    return LaunchDescription([
        cruise_speed_arg,
        max_angular_arg,
        lookahead_dist_arg,
        goal_tol_arg,
        slow_radius_arg,
        kp_yaw_arg,
        kd_yaw_arg,
        curvature_slowdown_arg,
        odom_timeout_arg,
        wait_for_start_arg,
        LogInfo(msg='[BSplineTrackTest] 실내 수조 B-Spline 곡선 추종 테스트 시작 중...'),
        node,
    ])
