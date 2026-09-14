"""tank_tests.launch.py — 실내 수조 4대 자율운항 테스트 통합 실행 런치.

지원 테스트 모드 (test_type):
  1) straight        : 1자 직진 주행 (9.0m -> 1.0m, 방위각 180도)
  2) bspline         : B-Spline 곡선 추종 (S자 궤적 선회)
  3) circle          : 원형 선회 주행 (수조 중앙 반경 1.2m 선회)
  4) station_keeping : 정해진 웨이포인트 정점 유지 (가상 앵커/DP)

사용 예시:
  ros2 launch kaboat_hardware tank_tests.launch.py test_type:=straight
  ros2 launch kaboat_hardware tank_tests.launch.py test_type:=bspline
  ros2 launch kaboat_hardware tank_tests.launch.py test_type:=circle
  ros2 launch kaboat_hardware tank_tests.launch.py test_type:=station_keeping
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression


def generate_launch_description():
    pkg_share = get_package_share_directory('kaboat_hardware')
    launch_dir = os.path.join(pkg_share, 'launch')

    test_type = LaunchConfiguration('test_type')
    wait_for_start = LaunchConfiguration('wait_for_start')

    test_type_arg = DeclareLaunchArgument(
        'test_type',
        default_value='straight',
        description='실행할 수조 테스트 종류: straight | bspline | circle | station_keeping'
    )
    wait_for_start_arg = DeclareLaunchArgument(
        'wait_for_start',
        default_value='true',
        description='외부 시작 신호(/start_mission) 대기 여부 (true/false)'
    )

    # 1. 직진 주행 테스트
    straight_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'straight_line_test.launch.py')),
        launch_arguments={'wait_for_start': wait_for_start}.items(),
        condition=IfCondition(PythonExpression(["'", test_type, "' == 'straight'"]))
    )

    # 2. B-Spline 곡선 추종 테스트
    bspline_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'bspline_track_test.launch.py')),
        launch_arguments={'wait_for_start': wait_for_start}.items(),
        condition=IfCondition(PythonExpression(["'", test_type, "' == 'bspline'"]))
    )

    # 3. 원형 선회 주행 테스트
    circle_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'circle_drive_test.launch.py')),
        launch_arguments={'wait_for_start': wait_for_start}.items(),
        condition=IfCondition(PythonExpression(["'", test_type, "' == 'circle'"]))
    )

    # 4. 웨이포인트 정점 유지 테스트
    station_keeping_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'station_keeping_test.launch.py')),
        launch_arguments={'wait_for_start': wait_for_start}.items(),
        condition=IfCondition(PythonExpression(["'", test_type, "' == 'station_keeping'"]))
    )

    return LaunchDescription([
        test_type_arg,
        wait_for_start_arg,
        LogInfo(msg=['[TankTests] 수조 통합 테스트 실행: 모드 = ', test_type]),
        straight_launch,
        bspline_launch,
        circle_launch,
        station_keeping_launch,
    ])
