"""야외 게이트 통과 시험: rec 기록 gate_entry → gate_end, 게이트 7쌍 (게이트 미션 하나만).

자율주행 스택과 같은 노드(occupancy_grid → gate_follower → cmd_mux, mission_manager)를
게이트 미션 하나만 돌리게 띄운다. mission_manager 는 기록 파일 전체를 읽지 않고
(station_end 등이 없어도 됨) 여기서 읽은 두 지점만 받는다 — 기록 당시 원점이 지금과
다르면 시작 거부. 센서·모터·지도는 따로: run_gq7_rtk, run_lidar, run_motor, run_map.

  goto gate_entry                       # 1) 첫 게이트 앞 통로 중심에 세움 (도착 후 Ctrl+C)
  test_gate_outdoor motion_enabled:=false   # 2) 미리보기: 노트북 RViz 에 쌍·경로 (/gate/*)
  test_gate_outdoor                     # 3) 실제 주행

gate_end 는 마지막 게이트 뒤 2~3 m 통로 중심에 기록한다 — 모든 쌍을 지난 뒤 4 m 안이어야
그리로 가고(max_goal_after_gates), 2 m 안이면 완료, 그다음 mission_manager 가 done(정지).
approach:=true 면 gate_entry 까지 미션 접근(직선, 장애물 회피 없음)부터 한다 — 1 m 안 +
선수각 ±15° 안을 0.5 s 유지하면(회전 속도 조건 없음, mission_approach 기본값) gate 시작.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node

from kaboat_mission.mission_points import (
    DEFAULT_ORIGIN_FILE, DEFAULT_POINTS_FILE, recorded_point)


def _default_gate_params():
    try:
        share = get_package_share_directory('kaboat_bringup')
    except Exception:  # 설치 안 된 테스트 환경
        return ''
    return os.path.join(share, 'config', 'gate_params.yaml')


ARGS = [
    ('start_point', 'gate_entry', '첫 게이트 앞 지점 (rec 기록 이름)'),
    ('goal_point', 'gate_end', '마지막 게이트 뒤 2~3 m 지점 (rec 기록 이름)'),
    ('motion_enabled', 'true', 'false = 미리보기 (모터 명령 안 냄)'),
    ('approach', 'false', 'true = gate_entry 까지 접근부터 (직선, 회피 없음)'),
    ('required_pairs', '7', '통과할 게이트 쌍 수'),
    ('los_thrust', '', "게이트 통과 직진 상한 0~1 ('' = gate_params.yaml 값 0.3)"),
    ('start_occupancy_grid', 'false', '지도는 run_map 으로 따로 (true 면 여기서 켬 — 따로 켠 게 없을 때만)'),
    ('cam_enabled', 'false', '카메라 층 (주황 등 낮은 색 장애물) — run_camera 필요'),
    ('cam_height', '0.775', '흘수선에서 카메라 렌즈까지 [m] (2026-11-12 수평으로 다시 단 뒤 실측)'),
    ('gate_params_file', _default_gate_params(), '게이트 파라미터 yaml'),
    ('points_file', DEFAULT_POINTS_FILE, 'rec 기록 파일'),
    ('origin_file', DEFAULT_ORIGIN_FILE, 'GPS 원점 파일 (기록 당시 원점과 비교)'),
]


def _bool(text: str, name: str) -> bool:
    value = text.strip().lower()
    if value not in ('true', 'false'):
        raise RuntimeError(f'{name}:= 는 true 또는 false — 받은 값 {text!r}')
    return value == 'true'


def _number(text: str, name: str, kind=float):
    try:
        return kind(text)
    except ValueError:
        raise RuntimeError(f'{name}:= 는 숫자여야 함 — 받은 값 {text!r}') from None


def _point(get, role):
    name = get(role).strip()
    if not name:
        raise RuntimeError(f'{role}:= 가 비어 있음')
    try:
        return recorded_point(name, get('points_file'), get('origin_file'))
    except ValueError as exc:
        raise RuntimeError(f'{role}:={name} — {exc}') from None


def mission_parameters(get) -> dict:
    """launch 인자 → mission_manager 파라미터 (게이트 하나만, 기록 지점 두 개)."""
    entry, end = _point(get, 'start_point'), _point(get, 'goal_point')
    return {
        'auto_start': True,
        'start_state': 'gate',
        'single_mission': True,
        'entry_required': _bool(get('approach'), 'approach'),
        'waypoints.gate': [end.x, end.y],
        'entry_waypoints.gate': [entry.x, entry.y, entry.yaw_deg],
        'entry_ready.gate': True,
        'points_file': '',            # 기록 파일 전체는 안 읽음 (다른 미션 지점 불필요)
        'use_sim_time': False,
    }


def gate_parameters(get) -> dict:
    pairs = _number(get('required_pairs'), 'required_pairs', int)
    if pairs < 1:
        raise RuntimeError('required_pairs:= 는 1 이상')
    params = {'gate.motion_enabled': _bool(get('motion_enabled'), 'motion_enabled'),
              'gate.required_pairs': pairs, 'use_sim_time': False}
    if get('los_thrust') != '':
        thrust = _number(get('los_thrust'), 'los_thrust')
        if not 0.0 <= thrust <= 1.0:
            raise RuntimeError('los_thrust:= 는 0~1')
        params['gate.los_thrust_ratio'] = thrust
    return params


def _actions(context):
    get = lambda name: context.launch_configurations.get(name, '')
    motion = _bool(get('motion_enabled'), 'motion_enabled')
    approach = _bool(get('approach'), 'approach')
    actions = []
    if _bool(get('start_occupancy_grid'), 'start_occupancy_grid'):
        actions.append(Node(
            package='kaboat_perception', executable='occupancy_grid',
            name='occupancy_grid', output='screen',
            parameters=[{'use_sim_time': False,
                         'cam_enabled': _bool(get('cam_enabled'), 'cam_enabled'),
                         'cam_height': _number(get('cam_height'), 'cam_height')}]))
    gate_file = get('gate_params_file')
    actions.append(Node(
        package='kaboat_behaviors', executable='gate_follower',
        name='gate_follower', output='screen',
        parameters=([gate_file] if gate_file else []) + [gate_parameters(get)]))
    actions.append(Node(
        package='kaboat_mission', executable='mission_manager',
        name='mission_manager', output='screen',
        parameters=[mission_parameters(get)]))
    if approach:
        actions.append(Node(
            package='kaboat_behaviors', executable='mission_approach',
            name='mission_approach', output='screen',
            parameters=[{'approach.motion_enabled': motion, 'use_sim_time': False}]))
    if motion:
        # 현재 미션 behavior 명령만 /cmd_vel 로 (끊기면 0, done 이면 정지). 미리보기면 안 켬
        actions.append(Node(package='kaboat_control', executable='cmd_mux',
                            name='cmd_mux', output='screen',
                            parameters=[{'use_sim_time': False}]))
    return actions


def generate_launch_description():
    return LaunchDescription(
        [DeclareLaunchArgument(name, default_value=default, description=text)
         for name, default, text in ARGS]
        + [OpaqueFunction(function=_actions)])
