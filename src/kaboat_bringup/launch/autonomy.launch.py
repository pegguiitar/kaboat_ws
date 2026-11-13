"""자율주행 알고리즘 스택 launch.

실물 안전 기본값:
  - use_sim_time=false
  - Gazebo용 twist2thrust는 실행하지 않음

센서 배선 확인은 먼저 ``real_sensors.launch.py``를 사용한다. 이 launch는
센서 드라이버나 실제 모터 드라이버를 자동으로 띄우지 않는다.
지도(occupancy_grid)도 기본으로 안 띄운다 — 젯슨 ``run_map``(라이다만)으로 따로
하나만 켜 두고 시험·미션이 같이 쓴다. 여기서 켜려면 start_occupancy_grid:=true.

이번 실행에서 돌 미션의 노드만 띄운다 (only_mission_nodes, 기본 true):
  start_state 부터 끝까지, single_mission:=true 면 그 미션 하나 (mission_manager 와 같은 규칙).
  미션 매니저·cmd_mux 는 항상, 접근(mission_approach)은 entry_required 일 때만.
  2026-11-13 dock bag: test_mission station 이 파이썬 노드 12개를 한꺼번에 띄워 배가 첫 명령을
  받기까지 8 s(도킹 시험 런치 약 7개는 3 s) — 그동안 0.1~0.2 m/s 로 밀렸다.
  only_mission_nodes:=false 면 예전처럼 전부 띄운다.

출발 직진 (undock_sec, 기본 3 s): 대회 출발은 슬롯 안에서 한다. 처음부터 도는 전체 실행
(start_state 없음, single_mission:=false)이고 approach_motion_enabled:=true(run_autonomy)면
mission_manager 가 텔레옵 4단계 직진(좌우 1700 µs)을 3 s 낸 뒤 gate_entry 로 접근한다.
test_mission·go_* 는 해당 없음. 끄려면 run_autonomy undock_sec:=0.

미션 제한 시간 (mission_timeouts, 기본 true): 경기 15 분을 gate 3·station 2·dock 4·search 3·avoid 3 분으로
나눠(mission_params.yaml time_limit.*), 넘거나 시작 자세가 없으면 그 미션을 건너뛴다 (mission_manager 설명).
"""

import math
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import yaml

from kaboat_mission.mission_points import MISSION_STATES, missions_to_run
from kaboat_perception.dock_nodes import default_weights, docking_nodes


def _bool(text: str, name: str) -> bool:
    """launch 인자 문자열 → bool. 파라미터(ParameterValue value_type=bool)와 같은 YAML 규칙."""
    try:
        value = yaml.safe_load(text)
    except yaml.YAMLError:
        value = None
    if not isinstance(value, bool):
        raise RuntimeError(f'{name}:= 는 true 또는 false — 받은 값 {text!r}')
    return value


def stack_plan(get) -> tuple:
    """launch 인자(get(name) → 문자열) → (노드를 띄울 미션, mission_approach 를 띄울지).

    알 수 없는 start_state 는 거부한다. only_mission_nodes:=false 면 예전처럼 전부.
    """
    try:
        missions = missions_to_run(get('start_state'),
                                   _bool(get('single_mission'), 'single_mission'))
    except ValueError as exc:
        raise RuntimeError(str(exc)) from None
    if not _bool(get('only_mission_nodes'), 'only_mission_nodes'):
        return MISSION_STATES, True
    return missions, _bool(get('entry_required'), 'entry_required')


# /detections/buoys 를 쓰는 미션 (USES_BUOYS) — 이 중 하나라도 돌면 buoy_lidar_detector 를 띄운다
BUOY_USERS = {'station', 'search'}


def mission_nodes(use_sim_time_value) -> dict:
    """미션 → 그 미션만 쓰는 노드 목록."""
    common_params = {'use_sim_time': use_sim_time_value}
    L = LaunchConfiguration
    return {
        'gate': [Node(
            package='kaboat_behaviors', executable='gate_follower',
            name='gate_follower', output='screen',
            parameters=[L('gate_params_file'), common_params, {
                'gate.motion_enabled': ParameterValue(
                    L('gate_motion_enabled'), value_type=bool),
            }])],
        # 스테이션은 자기 전용 지도(/scan/level) + 부표 검출 노드(아래 BUOY_USERS)로 부표 사각형을 찾는다
        'station': [Node(
            package='kaboat_behaviors', executable='station_keeper',
            name='station_keeper', output='screen', parameters=[common_params])],
        # 도킹(카메라 TF·YOLO·라이다 RANSAC·one-shot 경로) — outdoor_dock_test 와 같은 구성
        'dock': docking_nodes(
            target_class=L('dock_target_class'), motion_enabled=L('dock_motion_enabled'),
            weights=L('dock_weights'), device=L('dock_device'),
            approach_cmd=L('dock_approach_cmd'), final_cmd=L('dock_final_cmd'),
            retreat_cmd=L('dock_retreat_cmd'), seek_cmd=L('dock_seek_cmd'),
            marker_order=L('dock_marker_order'),
            position_source=L('dock_position_source'), use_sim_time=use_sim_time_value,
            base_frame=L('dock_base_frame'),
            camera_tf_condition=IfCondition(L('publish_camera_tf'))),
        'search': [
            # 탐색 목표 색은 대회 당일 공지 — 빨강·초록 우현(시계), 검정 좌현(반시계) 선회
            Node(package='kaboat_behaviors', executable='search_circler',
                 name='search_circler', output='screen', parameters=[common_params, {
                     'search.target_color': ParameterValue(
                         L('search_target_color'), value_type=str),
                 }]),
        ],
        'avoid': [Node(
            package='kaboat_behaviors', executable='obstacle_planner',
            name='obstacle_planner', output='screen', parameters=[common_params])],
    }


def undock_seconds(get) -> float:
    """출발 직진(슬롯 탈출) 시간 — 접근 주행이 켜진 실행(run_autonomy)만. 미리보기
    (approach_motion_enabled:=false, 기본)에서는 배를 움직이지 않으므로 0."""
    try:
        sec = float(get('undock_sec'))
    except ValueError:
        sec = float('nan')
    if not (math.isfinite(sec) and sec >= 0.0):
        raise RuntimeError(f"undock_sec:= 는 0 이상 초 — 받은 값 {get('undock_sec')!r}")
    if not _bool(get('approach_motion_enabled'), 'approach_motion_enabled'):
        return 0.0
    return sec


def _stack(context):
    get = lambda name: context.launch_configurations.get(name, '')  # noqa: E731
    use_sim_time_value = LaunchConfiguration('use_sim_time')
    common_params = {'use_sim_time': use_sim_time_value}
    missions, approach = stack_plan(get)
    undock_sec = undock_seconds(get)

    per_mission = mission_nodes(use_sim_time_value)
    behaviors = [node for mission in missions for node in per_mission[mission]]
    if BUOY_USERS & set(missions):
        # 부표 지도 — 라이다 = 있음·위치, 카메라 RGB = 색. 구독자(USES_BUOYS behavior 가 활성)가
        # 있을 때만 계산. 스테이션·탐색이 같이 쓰므로 한 번만 띄운다.
        behaviors.append(Node(package='kaboat_perception', executable='buoy_lidar_detector',
                              name='buoy_lidar_detector', output='screen',
                              parameters=[common_params]))
    if approach:
        behaviors.append(Node(
            package='kaboat_behaviors', executable='mission_approach',
            name='mission_approach', output='screen', parameters=[common_params, {
                'approach.motion_enabled': ParameterValue(
                    LaunchConfiguration('approach_motion_enabled'), value_type=bool),
            }]))

    perception = [
        Node(package='kaboat_perception', executable='occupancy_grid',
             name='occupancy_grid', output='screen', parameters=[common_params],
             condition=IfCondition(LaunchConfiguration('start_occupancy_grid'))),
    ]
    control = [
        Node(package='kaboat_control', executable='cmd_mux',
             name='cmd_mux', output='screen', parameters=[common_params]),
        # 시뮬레이터를 명시적으로 선택했을 때만 Gazebo 추력 토픽을 발행한다.
        Node(package='kaboat_sim', executable='twist2thrust.py',
             name='twist2thrust', output='screen', parameters=[common_params],
             condition=IfCondition(LaunchConfiguration('use_sim_actuator'))),
    ]
    mission = [
        Node(package='kaboat_mission', executable='mission_manager',
             name='mission_manager', output='screen',
             parameters=[LaunchConfiguration('mission_params_file'), common_params,
                         {'points_file': ParameterValue(
                             LaunchConfiguration('mission_points_file'), value_type=str),
                          # 미션 하나만 시험: start_state:=station single_mission:=true
                          'start_state': ParameterValue(
                              LaunchConfiguration('start_state'), value_type=str),
                          'single_mission': ParameterValue(
                              LaunchConfiguration('single_mission'), value_type=bool),
                          'entry_required': ParameterValue(
                              LaunchConfiguration('entry_required'), value_type=bool),
                          # 출발 직진 (슬롯 탈출) — 처음부터 도는 전체 실행에서만 mission_manager 가 씀
                          'undock_sec': undock_sec,
                          'undock_linear': ParameterValue(
                              LaunchConfiguration('undock_linear'), value_type=float),
                          # 미션 제한 시간 (값은 mission_params.yaml time_limit.*)
                          'mission_timeouts': ParameterValue(
                              LaunchConfiguration('mission_timeouts'), value_type=bool)}]),
    ]
    names = [str(getattr(a, 'node_executable', '?')) for a in behaviors if isinstance(a, Node)]
    full_run = not get('start_state') and not _bool(get('single_mission'), 'single_mission')
    summary = LogInfo(msg=(
        f"[autonomy] 미션 {' → '.join(missions)} — 띄우는 노드: {', '.join(names)}, "
        f"cmd_mux, mission_manager"
        + (f" | 출발 직진 {undock_sec:.1f} s 뒤 gate_entry 접근" if full_run and undock_sec > 0
           else '')))
    return [summary, *perception, *behaviors, *control, *mission]


def generate_launch_description():
    bringup_share = get_package_share_directory('kaboat_bringup')
    mission_params = os.path.join(bringup_share, 'config', 'mission_params.yaml')
    gate_params = os.path.join(bringup_share, 'config', 'gate_params.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Gazebo 실행 때만 true'),
        DeclareLaunchArgument(
            'start_occupancy_grid', default_value='false',
            description='false: 지도는 run_map 으로 따로 하나만 (발행자 둘이면 지도가 번갈아 들어옴)'),
        DeclareLaunchArgument(
            'use_sim_actuator', default_value='false',
            description='Gazebo용 twist2thrust 실행 여부; 실물에서는 false'),
        DeclareLaunchArgument(
            'mission_params_file', default_value=mission_params,
            description='미션 목표 좌표 YAML; 수조 시험에서는 실측 좌표 파일 지정'),
        DeclareLaunchArgument(
            'mission_points_file', default_value='~/kaboat_mission_points.json',
            description='현장 rec 기록 지점 JSON — entry/end 쌍, dock_entry 단독 허용. none 이면 안 읽음'),
        DeclareLaunchArgument(
            'start_state', default_value='',
            description="이 미션부터 시작 (gate / station / dock / search / avoid). '' = 처음(gate)부터"),
        DeclareLaunchArgument(
            'single_mission', default_value='false',
            description='true = start_state 하나만 하고 done(정지) — 미션 하나씩 시험'),
        DeclareLaunchArgument(
            'only_mission_nodes', default_value='true',
            description='true = 이번에 돌 미션(start_state 부터, single_mission 이면 하나)의 노드만 띄움. '
                        'false = 예전처럼 전부'),
        DeclareLaunchArgument(
            'entry_required', default_value='true',
            description='false = 미션 시작 자세 접근(approach) 없이 바로 시작 — go_station·go_search 처럼 '
                        'goto 로 이미 세운 뒤 켤 때'),
        DeclareLaunchArgument(
            'undock_sec', default_value='3.0',
            description='출발 직진 시간 [s] — 슬롯 안에서 출발하므로 처음부터 도는 전체 실행(run_autonomy)은 '
                        '이만큼 직진한 뒤 gate_entry 로 접근. 모터가 AUTO 인 시간만 셈. 0 = 끔. '
                        'approach_motion_enabled:=false(미리보기)면 안 함'),
        DeclareLaunchArgument(
            'undock_linear', default_value='0.4667',
            description='출발 직진 명령 — 0.4667 = 텔레옵 4단계(좌우 1700 µs)'),
        DeclareLaunchArgument(
            'mission_timeouts', default_value='true',
            description='true = 이어 도는 실행에서 미션별 제한 시간(15 분 배분, mission_params.yaml '
                        'time_limit.*)을 넘거나 시작 자세가 없으면 그 미션을 건너뜀. 단일 미션 시험은 해당 없음'),
        DeclareLaunchArgument(
            'gate_params_file', default_value=gate_params,
            description='게이트 쌍 수와 라이다/DRI 임계값 YAML'),
        DeclareLaunchArgument(
            'gate_motion_enabled', default_value='false',
            description='기본값 false: gate 경로/명령 미리보기만 수행'),
        DeclareLaunchArgument(
            'approach_motion_enabled', default_value='false',
            description='기본값 false: 미션 시작 위치/방향 접근은 미리보기만 수행'),
        DeclareLaunchArgument(
            'search_target_color', default_value='',
            description="탐색 목표 부표 색 red / green / black (대회 당일 공지). '' 이면 탐색 정지"),
        DeclareLaunchArgument(
            'dock_weights', default_value=default_weights(),
            description='YOLO26n-seg 가중치 (models/best.engine 이 있으면 그것, 없으면 best.pt)'),
        DeclareLaunchArgument(
            'dock_target_class', default_value='triangle',
            description='triangle / circle / square'),
        DeclareLaunchArgument(
            'dock_device', default_value='0',
            description="Ultralytics device: 젯슨 CUDA '0' (기본, YOLO venv 필요 — scripts/run_dock_mission.sh 참고) 또는 cpu"),
        DeclareLaunchArgument('dock_base_frame', default_value='base_link'),
        DeclareLaunchArgument('dock_seek_cmd', default_value='0.07',
                              description='목표 도형 방위로 서행 (라이다 벽이 잡힐 때까지)'),
        DeclareLaunchArgument('dock_approach_cmd', default_value='0.10',
                              description='경유점까지 LOS 직진 상한'),
        DeclareLaunchArgument('dock_final_cmd', default_value='0.05',
                              description='정지점까지 LOS 직진 상한'),
        DeclareLaunchArgument('dock_retreat_cmd', default_value='0.20',
                              description='도킹 후진 출력 (0.20 ≈ 1400 µs = 텔레옵 후진 2단계)'),
        DeclareLaunchArgument('dock_marker_order', default_value='',
                              description="화면 왼쪽부터 도형 순서 (예 square,triangle,circle). '' = 안 봄"),
        DeclareLaunchArgument('dock_position_source', default_value='odom',
                              description='odom (RTK 필터, 기본) 또는 gps (/gps/fix 원시)'),
        DeclareLaunchArgument('publish_camera_tf', default_value='true',
                              description='라이다 → camera_link TF (camera_mount.py). dock_yolo 를 같이 켜면 false'),
        DeclareLaunchArgument(
            'slot_config',
            default_value=os.path.join(bringup_share, 'config', 'dock_slot.yaml'),
            description='이전 CLI 호환용. one-shot RANSAC 도킹에서는 사용하지 않음'),
        DeclareLaunchArgument(
            'dock_motion_enabled', default_value='false',
            description='기본값 false; TF/오프셋/안전 검증 후에만 true'),
        OpaqueFunction(function=_stack),
    ])
