"""야외 도킹 시험: rec 기록 dock_entry 에서 one-shot 도킹 (도킹 미션 하나만).

자율주행 스택과 같은 도킹 노드(kaboat_perception/dock_nodes.py — 카메라 TF, YOLO, 라이다 RANSAC,
dock_marker_goal, dock_ctrl_v2)와 mission_manager 를 도킹 미션 하나만 돌리게 띄운다.
YOLO 는 젯슨 CUDA venv 가 필요하므로 scripts/run_dock_mission.sh (단축어 test_dock_outdoor)로 켠다.
센서·모터·지도·카메라는 따로: run_gq7_rtk, run_lidar, run_camera, run_map, run_motor.

  goto dock_entry                              # 1) 도크 정면, 세 도형이 다 보이는 곳에 세움
  test_dock_outdoor motion_enabled:=false      # 2) 미리보기: 경로(/dock/one_shot_path)만 고정·표시
  test_dock_outdoor target_class:=circle       # 3) 실제 도킹

순서 (dock_ctrl_v2): SCAN — 제자리에서 벽 법선 쪽으로만 돌며 YOLO 마커 3개(가운데가 화면 중앙, 양쪽
대칭)와 RANSAC 법선 3개가 안정될 때까지 기다림 → 경로 한 번 고정 → 경유점(목표 슬롯 뒷벽 앞 3 m)
→ 정지점(1 m) → 선수 유지 후진 3.25 m → /mission/complete=dock → mission_manager done(정지).
approach:=true 면 dock_entry 까지 미션 접근(직선, 회피 없음, 1 m·±15°)부터 한다.
YOLO 결과는 /dock/detection_status(JSON)·/dock/debug_image(마스크 영상) 등으로 나오고 rec_bag 이 저장한다.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node

from kaboat_hardware.dock_one_shot import parse_marker_order
from kaboat_mission.mission_points import (
    DEFAULT_ORIGIN_FILE, DEFAULT_POINTS_FILE, recorded_point)
from kaboat_perception.dock_nodes import CLASSES, default_weights, docking_nodes

ARGS = [
    ('start_point', 'dock_entry', '도크 정면 시작 지점 (rec 기록 이름) — 세 도형이 다 보이는 곳'),
    ('target_class', 'triangle', '목표 슬롯 도형: triangle / circle / square'),
    ('motion_enabled', 'true', 'false = 미리보기 (경로만 고정·표시, 모터 명령 안 냄)'),
    ('approach', 'false', 'true = dock_entry 까지 접근부터 (직선, 회피 없음)'),
    ('approach_cmd', '0.10', '경유점까지 LOS 직진 상한 (0.10 ≈ 1562 µs)'),
    ('final_cmd', '0.05', '정지점까지 LOS 직진 상한 (0.05 ≈ 1544 µs)'),
    ('retreat_cmd', '0.05', '후진 출력 (0.05 ≈ 1456 µs)'),
    ('marker_order', '', "화면 왼쪽부터 도형 순서를 알면 square,triangle,circle 처럼. '' = 순서 안 봄"),
    ('position_source', 'odom', 'odom (GQ7 RTK 필터, 기본) 또는 gps (/gps/fix 원시 2 Hz)'),
    ('dock_device', '0', "YOLO 장치: 젯슨 CUDA '0' 또는 cpu"),
    ('dock_weights', default_weights(), 'YOLO 가중치 (.engine 있으면 그것)'),
    ('points_file', DEFAULT_POINTS_FILE, 'rec 기록 파일'),
    ('origin_file', DEFAULT_ORIGIN_FILE, 'GPS 원점 파일 (기록 당시 원점과 비교)'),
]


def _bool(text: str, name: str) -> bool:
    value = text.strip().lower()
    if value not in ('true', 'false'):
        raise RuntimeError(f'{name}:= 는 true 또는 false — 받은 값 {text!r}')
    return value == 'true'


def _ratio(text: str, name: str) -> float:
    try:
        value = float(text)
    except ValueError:
        raise RuntimeError(f'{name}:= 는 숫자여야 함 — 받은 값 {text!r}') from None
    if not 0.0 < value <= 1.0:
        raise RuntimeError(f'{name}:= 는 0 초과 1 이하 — 받은 값 {value}')
    return value


def _point(get, role):
    name = get(role).strip()
    if not name:
        raise RuntimeError(f'{role}:= 가 비어 있음')
    try:
        return recorded_point(name, get('points_file'), get('origin_file'))
    except ValueError as exc:
        raise RuntimeError(f'{role}:={name} — {exc}') from None


def mission_parameters(get) -> dict:
    """launch 인자 → mission_manager 파라미터 (도킹 하나만, 기록 지점 하나).

    도킹 끝 지점은 기록하지 않는다 — dock_ctrl_v2 가 고정한 경로의 이탈점(/dock/one_shot_path
    마지막 점)을 mission_manager 가 목표로 쓴다. waypoints.dock 은 경로 고정 전 자리표시.
    """
    entry = _point(get, 'start_point')
    return {
        'auto_start': True,
        'start_state': 'dock',
        'single_mission': True,
        'entry_required': _bool(get('approach'), 'approach'),
        'waypoints.dock': [entry.x, entry.y],
        'entry_waypoints.dock': [entry.x, entry.y, entry.yaw_deg],
        'entry_ready.dock': True,
        'points_file': '',            # 기록 파일 전체는 안 읽음 (다른 미션 지점 불필요)
        'use_sim_time': False,
    }


def dock_arguments(get) -> dict:
    """launch 인자 검사 → docking_nodes 인자."""
    target = get('target_class').strip()
    if target not in CLASSES:
        raise RuntimeError(f'target_class:= 는 {"/".join(CLASSES)} 중 하나 — 받은 값 {target!r}')
    order = get('marker_order').strip()
    try:
        parse_marker_order(order)
    except ValueError as exc:
        raise RuntimeError(f'marker_order:= {exc}') from None
    source = get('position_source').strip().lower()
    if source not in ('odom', 'gps'):
        raise RuntimeError(f'position_source:= 는 odom 또는 gps — 받은 값 {source!r}')
    weights = get('dock_weights').strip()
    if not weights:
        raise RuntimeError('dock_weights:= 가 비어 있음 (kaboat_perception/models/best.pt)')
    return dict(target_class=target, motion_enabled=_bool(get('motion_enabled'), 'motion_enabled'),
                weights=weights, device=get('dock_device').strip() or '0',
                approach_cmd=_ratio(get('approach_cmd'), 'approach_cmd'),
                final_cmd=_ratio(get('final_cmd'), 'final_cmd'),
                retreat_cmd=_ratio(get('retreat_cmd'), 'retreat_cmd'),
                marker_order=order, position_source=source)


def _actions(context):
    get = lambda name: context.launch_configurations.get(name, '')
    dock = dock_arguments(get)
    motion, approach = dock['motion_enabled'], _bool(get('approach'), 'approach')
    actions = [Node(
        package='kaboat_mission', executable='mission_manager',
        name='mission_manager', output='screen',
        parameters=[mission_parameters(get)])]
    if approach:
        actions.append(Node(
            package='kaboat_behaviors', executable='mission_approach',
            name='mission_approach', output='screen',
            parameters=[{'approach.motion_enabled': motion, 'use_sim_time': False}]))
    actions += docking_nodes(**dock)
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
