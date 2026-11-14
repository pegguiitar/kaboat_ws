"""토너먼트 한 번에 (go_tournament): 출발 직진 → goto tournament_entry → 토너먼트 회피 시험, 공백 없이.

토너먼트도 슬롯 안에서 출발한다. 노드를 처음에 전부 띄워 두고 명령만 넘겨주므로 단계 사이에 명령이
끊기지 않는다 (예전 go_and_test: 직진 뒤 goto 를 그때 켜 ~3 s, goto 를 끄고 시험을 그때 켜 몇 초 공백).

  cmd_mux               /mission/state 에 맞는 명령만 /cmd_vel 로 (전체 실행과 같은 노드, 워치독 0.3 s)
  tournament_sequence   state: undock → approach → avoid (출발 직진·시작 조건·시험 출발)
  goto (indoor_waypoint)       /cmd/approach — rec 지점 start_point 로 (로그는 파일로, 진행은 sequence 가 출력)
  real_obstacle_avoid_test     /cmd/avoid — test_tournament 와 같은 노드·인자, 출발은 sequence 가 서비스로

  go_tournament                                   # = ros2 launch kaboat_hardware outdoor_tournament_run.launch.py
  go_tournament drive_enabled:=false              # 회피 시험만 미리보기 (출발 직진·goto 는 실제로 움직임)
  go_tournament undock_sec:=0                     # 출발 직진 끄기
  go_tournament start_dist:=1.5 start_heading_deg:=20 goto_thrust_ratio:=0.3

센서·모터·지도는 따로(run_gq7_rtk, run_lidar, run_motor, run_map). /cmd_vel 을 내는 다른 노드
(run_autonomy, test_*, goto)는 끄고 켠다 — cmd_mux 와 같이 내면 명령이 섞인다.
회피 시험 출발 검사(start_tolerance·start_heading_tolerance_deg)는 기본 auto = 시작 조건 + 0.5 m·5°.
"""

import importlib.util
import math
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node

HERE = os.path.dirname(os.path.abspath(__file__))


def _load(filename, name):
    spec = importlib.util.spec_from_file_location(name, os.path.join(HERE, filename))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


_tournament = _load('outdoor_tournament_test.launch.py', 'outdoor_tournament_test_launch')
_goto = _load('outdoor_waypoint.launch.py', 'outdoor_waypoint_launch')

START_MARGIN_M = 0.5      # 시험 출발 검사 여유 (시작 조건보다 넓게 — 서비스를 부르는 사이 흐름)
START_MARGIN_DEG = 5.0
AUTO = 'auto'

SEQUENCE_ARGS = [
    ('undock_sec', '3.0', '출발 직진 시간 [s] (텔레옵 4단계, 모터 AUTO 인 시간만 셈). 0 = 끔'),
    ('start_dist', '3.0', '시작 조건: start_point 에서 이 거리 [m] 안'),
    ('start_heading_deg', '45.0', '시작 조건: 선수가 goal_point 방향에서 이 각도 [deg] 안'),
    ('start_hold_sec', '0.5', '시작 조건을 이만큼 [s] 유지하면 회피 시험 출발'),
    ('goto_thrust_ratio', '0.4', 'goto 추력 비율 (goto 기본과 같음)'),
]
# 시험 인자: test_tournament 와 같다. 빼는 것 — wait_for_start(이 launch 가 서비스로 출발시키므로 true 고정),
# 지도 인자(run_map 으로 따로). 출발 검사 두 개는 기본 auto (시작 조건 + 여유).
_SKIP = ('wait_for_start', 'start_occupancy_grid', 'grid_size', 'cam_enabled', 'cam_height',
         'cam_mount_pitch_deg', 'cam_white')
_AUTO_DEFAULTS = {'start_tolerance': AUTO, 'start_heading_tolerance_deg': AUTO}
TEST_ARGS = [(name, _AUTO_DEFAULTS.get(name, default), text)
             for name, default, text in _tournament.ARGS if name not in _SKIP]
ARGS = SEQUENCE_ARGS + TEST_ARGS


def _float(text, name):
    try:
        value = float(text)
    except ValueError:
        raise RuntimeError(f'{name}:= 는 숫자여야 함 — 받은 값 {text!r}') from None
    if not math.isfinite(value):
        raise RuntimeError(f'{name}:= 가 유한한 숫자가 아님: {text!r}')
    return value


def avoid_getter(get):
    """회피 시험 노드 인자 — wait_for_start true, 출발 검사 auto 는 시작 조건 + 여유."""
    dist = _float(get('start_dist'), 'start_dist')
    heading = _float(get('start_heading_deg'), 'start_heading_deg')
    auto = {'start_tolerance': f'{dist + START_MARGIN_M:g}',
            'start_heading_tolerance_deg': f'{min(heading + START_MARGIN_DEG, 180.0):g}'}

    def value(name):
        if name == 'wait_for_start':
            return 'true'
        text = get(name)
        if name in auto and text.strip().lower() in ('', AUTO):
            return auto[name]
        return text
    return value


def goto_getter(get):
    """goto(outdoor_waypoint) 인자 — 지점은 start_point, 추력은 goto_thrust_ratio, 나머지는 goto 기본."""
    defaults = {name: default for name, default, _ in _goto.ARGS}
    mine = {'point': get('start_point'), 'heading': 'true', 'motion_enabled': 'true',
            'thrust_ratio': get('goto_thrust_ratio'),
            'points_file': get('points_file'), 'origin_file': get('origin_file')}
    return lambda name: mine.get(name, defaults.get(name, ''))


def sequence_parameters(get) -> dict:
    return {
        'start_point': get('start_point'), 'goal_point': get('goal_point'),
        'points_file': get('points_file'), 'origin_file': get('origin_file'),
        'undock_sec': _float(get('undock_sec'), 'undock_sec'),
        'start_dist': _float(get('start_dist'), 'start_dist'),
        'start_heading_deg': _float(get('start_heading_deg'), 'start_heading_deg'),
        'start_hold_sec': _float(get('start_hold_sec'), 'start_hold_sec'),
        'use_sim_time': False,
    }


def _actions(context):
    get = lambda name: context.launch_configurations.get(name, '')  # noqa: E731
    share = get_package_share_directory('kaboat_hardware')
    avoid_params = _tournament.avoid_parameters(avoid_getter(get))   # 지점·원점 검사 (틀리면 여기서 멈춤)
    goto_params = _goto.node_parameters(goto_getter(get))
    # goto 는 state 가 approach 일 때만 몬다 — 직진 중에 미리 도착(REACHED)해 버리지 않게
    goto_params['active_state'] = 'approach'
    return [
        Node(package='kaboat_control', executable='cmd_mux', name='cmd_mux', output='screen'),
        Node(package='kaboat_hardware', executable='tournament_sequence',
             name='tournament_sequence', output='screen',
             parameters=[sequence_parameters(get)]),
        Node(package='kaboat_hardware', executable='indoor_waypoint',
             name='outdoor_waypoint', output='log',
             parameters=[goto_params], remappings=[('/cmd_vel', '/cmd/approach')]),
        Node(package='kaboat_hardware', executable='real_obstacle_avoid_test',
             name='real_obstacle_avoid_test', output='screen',
             parameters=[os.path.join(share, 'config', 'outdoor_avoid_test.yaml'), avoid_params],
             remappings=[('/cmd_vel', '/cmd/avoid')]),
    ]


def generate_launch_description():
    return LaunchDescription(
        [DeclareLaunchArgument(name, default_value=default, description=text)
         for name, default, text in ARGS]
        + [OpaqueFunction(function=_actions)])
