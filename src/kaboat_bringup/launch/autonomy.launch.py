"""자율주행 스택 통합 launch — 시뮬레이터와 별도로 실행한다.

사용법 (sim 검증):
    # 터미널 1 — 시뮬레이터 (센서 발행 + /odom + 추력 입력)
    ros2 launch kaboat_sim simulation.launch.py            # 또는 headless:=True

    # 터미널 2 — 자율주행 스택 전체
    ros2 launch kaboat_bringup autonomy.launch.py

구성:
  ① 상시 인식     obstacle_detector, yolo_detector
  ② behavior ×5   gate_follower, station_keeper, docking_ctrl,
                   search_circler, obstacle_planner
  제어            cmd_mux(워치독), twist2thrust(kaboat_sim 재사용)
  관리            mission_manager (1m 전환 규칙, mission_params.yaml)

실물 전환 시 시뮬레이터 대신 실제 센서 드라이버 + robot_localization 을
띄우면 이 launch 는 그대로 재사용한다 (토픽 인터페이스 동일).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    bringup_share = get_package_share_directory('kaboat_bringup')
    mission_params = os.path.join(bringup_share, 'config', 'mission_params.yaml')

    use_sim_time = {'use_sim_time': True}

    # ── ① 상시 인식 레이어 ──────────────────────────────
    perception = [
        Node(package='kaboat_perception', executable='obstacle_detector',
             name='obstacle_detector', output='screen', parameters=[use_sim_time]),
        Node(package='kaboat_perception', executable='yolo_detector',
             name='yolo_detector', output='screen', parameters=[use_sim_time]),
    ]

    # ── ② behavior 레이어 (5개 전부 상주, cmd_mux 가 1개만 통과) ──
    behaviors = [
        Node(package='kaboat_behaviors', executable=exe,
             name=exe, output='screen', parameters=[use_sim_time])
        for exe in ['gate_follower', 'station_keeper', 'docking_ctrl',
                    'search_circler', 'obstacle_planner']
    ]

    # ── 제어 ───────────────────────────────────────────
    control = [
        Node(package='kaboat_control', executable='cmd_mux',
             name='cmd_mux', output='screen', parameters=[use_sim_time]),
        # /cmd_vel → 좌우 추력 (kaboat_sim 의 기존 노드 재사용)
        Node(package='kaboat_sim', executable='twist2thrust.py',
             name='twist2thrust', output='screen', parameters=[use_sim_time]),
    ]

    # ── 미션 관리 ───────────────────────────────────────
    mission = [
        Node(package='kaboat_mission', executable='mission_manager',
             name='mission_manager', output='screen',
             parameters=[mission_params, use_sim_time]),
    ]

    return LaunchDescription(perception + behaviors + control + mission)
