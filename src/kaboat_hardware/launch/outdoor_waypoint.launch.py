"""야외 웨이포인트 1개 주행 시험 (센서·모터는 따로 실행: run_gq7_rtk, run_motor).

목표는 둘 중 하나:
  point:=gate_entry            rec 로 기록한 지점 (x, y, 기록 당시 선수각)
  x:=12.0 y:=-3.5 [angle:=90]  /odom 좌표 직접 (angle 생략 = 선수각 맞추지 않음)

거리 제한은 기본 없음. max_goal_distance:=… (> 0) 을 주면 그보다 먼 목표에서 GOAL_TOO_FAR 로 멈춘다.
motion_enabled:=false 면 /cmd_vel 없이 로그·/waypoint/cmd_preview 만 낸다.
"""

import math

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node

from kaboat_mission.mission_points import DEFAULT_ORIGIN_FILE, DEFAULT_POINTS_FILE

ARGS = [
    ('point', '', 'rec 로 기록한 지점 이름 (주면 x/y/angle 무시)'),
    ('heading', 'true', 'point 사용 시 기록 선수각까지 맞출지 (false = 위치만)'),
    ('x', '', '/odom 목표 X [m] (동쪽 +)'),
    ('y', '', '/odom 목표 Y [m] (북쪽 +)'),
    ('angle', '', '목표 선수각 [deg] (동쪽 0, 반시계 +); 생략 = 맞추지 않음'),
    ('thrust_ratio', '0.4', '추력 상한 대비 비율 [0..1] (기본 0.4 = 40%, 2026-11-12 현장: 0.2 는 바람에 못 돎)'),
    ('motion_enabled', 'true', 'false = 미리보기 (모터 명령 안 냄)'),
    ('max_goal_distance', '0', '목표가 이보다 멀면 정지 [m] (0 = 제한 없음)'),
    ('position_tolerance', '0.9', '도착 판정 반경 [m]'),
    ('heading_tolerance_deg', '10.0', '최종 선수각 허용 오차 [deg]'),
    ('slow_radius', '1.5', '감속 시작 반경 [m]'),
    ('max_linear', '1.0', '전진 정규화 상한'),
    ('max_angular', '1.0', '회전 정규화 상한'),
    ('yaw_kp', '1.2', '헤딩 P 게인'),
    ('yaw_kd', '0.15', '요레이트 D 게인'),
    ('points_file', DEFAULT_POINTS_FILE, 'rec 기록 파일'),
    ('origin_file', DEFAULT_ORIGIN_FILE, 'GPS 원점 파일 (기록 당시 원점과 비교)'),
]
FLOAT_ARGS = ('thrust_ratio', 'max_goal_distance', 'position_tolerance',
              'heading_tolerance_deg', 'slow_radius', 'max_linear', 'max_angular',
              'yaw_kp', 'yaw_kd')


def _bool(text: str, name: str) -> bool:
    value = text.strip().lower()
    if value not in ('true', 'false'):
        raise RuntimeError(f'{name}:= 는 true 또는 false — 받은 값 {text!r}')
    return value == 'true'


def _float(text: str, name: str) -> float:
    try:
        value = float(text)
    except ValueError:
        raise RuntimeError(f'{name}:= 는 숫자여야 함 — 받은 값 {text!r}') from None
    if not math.isfinite(value):
        raise RuntimeError(f'{name}:= 가 유한한 숫자가 아님: {text!r}')
    return value


def node_parameters(get) -> dict:
    """launch 인자(get(name) → 문자열) → indoor_waypoint 파라미터 (야외 모드)."""
    params = {name: _float(get(name), name) for name in FLOAT_ARGS}
    params.update({
        'outdoor': True,
        'motion_enabled': _bool(get('motion_enabled'), 'motion_enabled'),
        'points_file': get('points_file'),
        'origin_file': get('origin_file'),
        'use_sim_time': False,
    })
    point = get('point').strip()
    if point:
        params.update(point=point, align_heading=_bool(get('heading'), 'heading'))
        return params
    if not get('x').strip() or not get('y').strip():
        raise RuntimeError('목표 없음 — point:=지점이름 또는 x:=… y:=… 를 주세요')
    params['target_x'] = _float(get('x'), 'x')
    params['target_y'] = _float(get('y'), 'y')
    angle = get('angle').strip()
    params['align_heading'] = bool(angle)
    params['target_angle_deg'] = _float(angle, 'angle') if angle else 0.0
    return params


def _node(context):
    get = lambda name: context.launch_configurations.get(name, '')
    return [Node(
        package='kaboat_hardware', executable='indoor_waypoint',
        name='outdoor_waypoint', output='screen',
        parameters=[node_parameters(get)],
    )]


def generate_launch_description():
    return LaunchDescription(
        [DeclareLaunchArgument(name, default_value=default, description=text)
         for name, default, text in ARGS]
        + [OpaqueFunction(function=_node)])
