"""야외 장애물 회피 시험: rec 기록 지점 avoid_entry 에서 출발해 avoid_end 까지 (수조 test_avoid 의 야외판).

수조 시험과 같은 노드(real_obstacle_avoid_test: DRI → B-spline → FSM)를 쓰고,
시작점·목표점만 ~/kaboat_mission_points.json 에서 읽는다 (기록 당시 원점과 지금 원점이
다르면 시작 거부). 센서·모터는 따로: run_gq7_rtk, run_lidar, run_motor.

  goto avoid_entry                                  # 1) 시작점에 기록 선수각으로 세움 (도착 후 Ctrl+C)
  test_avoid_outdoor drive_enabled:=false           # 2) 미리보기: 경로만 (RViz /real_avoid_test/path)
  test_avoid_outdoor                                # 3) 출발 조건이 맞으면 바로 출발

출발 조건: 시작점 start_tolerance(3 m) 안, 선수가 목표 방향에서 45° 이내.
start_point:=gate_entry goal_point:=gate_end 처럼 다른 지점 쌍도 쓸 수 있다.
occupancy_grid 를 따로 켜 두었다면 start_occupancy_grid:=false (지도 발행자가 둘이 됨).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node

from kaboat_mission.mission_points import (
    DEFAULT_ORIGIN_FILE, DEFAULT_POINTS_FILE, recorded_point)

ARGS = [
    ('start_point', 'avoid_entry', '출발 지점 (rec 기록 이름)'),
    ('goal_point', 'avoid_end', '목표 지점 (rec 기록 이름)'),
    ('drive_enabled', 'true', 'false = 미리보기 (모터 명령 안 냄)'),
    ('wait_for_start', 'false', 'true = /real_avoid_test/start 서비스로 출발'),
    ('allow_escape', 'true', '경로가 없을 때 후진 탈출 허용 (false = 정지)'),
    ('start_tolerance', '3.0', '시작점에서 이 거리 안이어야 출발 [m]'),
    ('start_heading_tolerance_deg', '45.0', '선수가 목표 방향에서 이 각도 안이어야 출발 [deg]'),
    ('goal_tolerance', '2.0', '목표점 이 거리 안이면 종료 [m]'),
    ('max_cross_track_error', '1.5', '경로에서 이보다 벗어나면 정지 [m]'),
    ('start_occupancy_grid', 'true', 'false = 이미 켠 occupancy_grid 를 씀'),
    ('grid_size', '15.0', '장애물 지도 한 변 [m]'),
    ('cam_enabled', 'false', '카메라 층 (주황 붐 등 낮은 색 장애물) — d455.launch.py 필요'),
    ('cam_height', '0.85', '흘수선에서 카메라 렌즈까지 [m] (2026-11-12 실측)'),
    ('cam_mount_pitch_deg', '10.0', '카메라 숙임 [deg] (+ = 아래, 2026-11-12 10°)'),
    ('cam_white', 'false', '흰색(부표)도 카메라로 — 수면 반짝임 오탐 주의'),
    ('points_file', DEFAULT_POINTS_FILE, 'rec 기록 파일'),
    ('origin_file', DEFAULT_ORIGIN_FILE, 'GPS 원점 파일 (기록 당시 원점과 비교)'),
]
FLOAT_ARGS = ('start_tolerance', 'start_heading_tolerance_deg', 'goal_tolerance',
              'max_cross_track_error')
BOOL_ARGS = ('drive_enabled', 'wait_for_start', 'allow_escape')


def _bool(text: str, name: str) -> bool:
    value = text.strip().lower()
    if value not in ('true', 'false'):
        raise RuntimeError(f'{name}:= 는 true 또는 false — 받은 값 {text!r}')
    return value == 'true'


def _float(text: str, name: str) -> float:
    try:
        return float(text)
    except ValueError:
        raise RuntimeError(f'{name}:= 는 숫자여야 함 — 받은 값 {text!r}') from None


def avoid_parameters(get) -> dict:
    """launch 인자(get(name) → 문자열) → real_obstacle_avoid_test 덮어쓸 파라미터."""
    names = {}
    for role in ('start_point', 'goal_point'):
        name = get(role).strip()
        if not name:
            raise RuntimeError(f'{role}:= 가 비어 있음')
        try:
            names[role] = recorded_point(name, get('points_file'), get('origin_file'))
        except ValueError as exc:
            raise RuntimeError(f'{role}:={name} — {exc}') from None
    start, goal = names['start_point'], names['goal_point']
    params = {name: _bool(get(name), name) for name in BOOL_ARGS}
    params.update({name: _float(get(name), name) for name in FLOAT_ARGS})
    params.update({
        'fixed_goal_enabled': True,
        'fixed_start_xy': [start.x, start.y],
        'fixed_goal_xy': [goal.x, goal.y],
        'tank_bounds': [0.0, 0.0, 0.0, 0.0],
        'exclude_tank_walls': False,
        'use_sim_time': False,
    })
    return params


def _actions(context):
    get = lambda name: context.launch_configurations.get(name, '')
    share = get_package_share_directory('kaboat_hardware')
    actions = []
    if _bool(get('start_occupancy_grid'), 'start_occupancy_grid'):
        actions.append(Node(
            package='kaboat_perception', executable='occupancy_grid',
            name='occupancy_grid', output='screen',
            parameters=[{'resolution': 0.2,
                         'size': _float(get('grid_size'), 'grid_size'),
                         'publish_rate': 10.0, 'use_sim_time': False,
                         'cam_enabled': _bool(get('cam_enabled'), 'cam_enabled'),
                         'cam_height': _float(get('cam_height'), 'cam_height'),
                         'cam_mount_pitch_deg': _float(get('cam_mount_pitch_deg'),
                                                       'cam_mount_pitch_deg'),
                         'cam_white': _bool(get('cam_white'), 'cam_white')}]))
    actions.append(Node(
        package='kaboat_hardware', executable='real_obstacle_avoid_test',
        name='real_obstacle_avoid_test', output='screen',
        parameters=[os.path.join(share, 'config', 'outdoor_avoid_test.yaml'),
                    avoid_parameters(get)]))
    return actions


def generate_launch_description():
    return LaunchDescription(
        [DeclareLaunchArgument(name, default_value=default, description=text)
         for name, default, text in ARGS]
        + [OpaqueFunction(function=_actions)])
