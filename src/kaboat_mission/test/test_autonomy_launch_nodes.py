"""autonomy.launch.py: 이번에 돌 미션의 노드만 띄운다 (start_state·single_mission·entry_required).

2026-11-13 dock bag: test_mission station 이 파이썬 노드 12개를 한꺼번에 띄워 배가 첫 명령을 받기까지
8 s 걸렸다. 미션 순서 규칙은 mission_manager 와 같은 missions_to_run 을 쓴다.
"""

import importlib.util
import os

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
import pytest

HERE = os.path.dirname(__file__)
LAUNCH = os.path.join(HERE, '..', '..', 'kaboat_bringup', 'launch', 'autonomy.launch.py')
ALWAYS = {'cmd_mux', 'mission_manager'}
DOCK = {'static_transform_publisher', 'dock_mark_detector', 'dock_lidar_markers',
        'dock_marker_goal', 'dock_ctrl_v2'}
EVERY = (ALWAYS | DOCK | {'gate_follower', 'station_keeper', 'buoy_lidar_detector',
                          'search_circler', 'obstacle_planner', 'mission_approach'})


@pytest.fixture(scope='module')
def mod():
    spec = importlib.util.spec_from_file_location('autonomy_launch', LAUNCH)
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m


def _launched(mod, **args):
    """launch 인자(기본값 + args) → 실제로 뜨는 노드 실행 파일 이름 집합 (조건 평가 포함)."""
    context = LaunchContext()
    for action in mod.generate_launch_description().entities:
        if isinstance(action, DeclareLaunchArgument):
            context.launch_configurations[action.name] = ''.join(
                s.perform(context) for s in action.default_value)
    context.launch_configurations.update(args)
    return {node.node_executable for node in mod._stack(context)
            if isinstance(node, Node)
            and (node.condition is None or node.condition.evaluate(context))}


def test_single_mission_launches_only_its_nodes(mod):
    # test_mission station: 접근(entry_required 기본 true) + 스테이션 + 부표 검출 (사각형 찾기 보완)
    assert _launched(mod, start_state='station', single_mission='true') == (
        ALWAYS | {'station_keeper', 'buoy_lidar_detector', 'mission_approach'})
    assert _launched(mod, start_state='gate', single_mission='true') == (
        ALWAYS | {'gate_follower', 'mission_approach'})
    assert _launched(mod, start_state='dock', single_mission='true') == (
        ALWAYS | DOCK | {'mission_approach'})
    # 부표 검출은 스테이션·탐색이 쓴다
    assert _launched(mod, start_state='search', single_mission='true') == (
        ALWAYS | {'buoy_lidar_detector', 'search_circler', 'mission_approach'})
    assert _launched(mod, start_state='avoid', single_mission='true') == (
        ALWAYS | {'obstacle_planner', 'mission_approach'})


def test_go_mission_without_entry_skips_the_approach_node(mod):
    # go_station·go_search: goto 로 이미 세웠으니 entry_required:=false — 접근 노드 필요 없음
    assert _launched(mod, start_state='station', single_mission='true',
                     entry_required='false') == ALWAYS | {'station_keeper', 'buoy_lidar_detector'}


def test_full_run_and_later_start_keep_the_remaining_missions(mod):
    assert _launched(mod) == EVERY                                       # run_autonomy
    assert _launched(mod, start_state='dock') == (
        ALWAYS | DOCK | {'buoy_lidar_detector', 'search_circler', 'obstacle_planner',
                         'mission_approach'})


def test_only_mission_nodes_false_launches_everything_as_before(mod):
    assert _launched(mod, start_state='station', single_mission='true',
                     only_mission_nodes='false') == EVERY


def test_unknown_start_state_and_bad_bool_are_refused(mod):
    with pytest.raises(RuntimeError, match='staton'):
        _launched(mod, start_state='staton', single_mission='true')
    with pytest.raises(RuntimeError, match='single_mission'):
        _launched(mod, start_state='station', single_mission='yes please')


def test_buoy_detector_is_launched_once_for_station_and_search(mod):
    context_nodes = [n.node_executable for n in mod._stack(_context(mod)) if hasattr(n, 'node_executable')]
    assert context_nodes.count('buoy_lidar_detector') == 1


def _context(mod, **args):
    context = LaunchContext()
    for action in mod.generate_launch_description().entities:
        if isinstance(action, DeclareLaunchArgument):
            context.launch_configurations[action.name] = ''.join(
                s.perform(context) for s in action.default_value)
    context.launch_configurations.update(args)
    return context
