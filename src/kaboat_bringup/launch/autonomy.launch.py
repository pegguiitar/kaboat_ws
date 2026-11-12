"""자율주행 알고리즘 스택 launch.

실물 안전 기본값:
  - use_sim_time=false
  - Gazebo용 twist2thrust는 실행하지 않음

센서 배선 확인은 먼저 ``real_sensors.launch.py``를 사용한다. 이 launch는
센서 드라이버나 실제 모터 드라이버를 자동으로 띄우지 않는다.
지도(occupancy_grid)도 기본으로 안 띄운다 — 젯슨 ``run_map``(라이다만)으로 따로
하나만 켜 두고 시험·미션이 같이 쓴다. 여기서 켜려면 start_occupancy_grid:=true.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from kaboat_perception.dock_nodes import default_weights, docking_nodes


def generate_launch_description():
    bringup_share = get_package_share_directory('kaboat_bringup')
    mission_params = os.path.join(bringup_share, 'config', 'mission_params.yaml')
    gate_params = os.path.join(bringup_share, 'config', 'gate_params.yaml')
    use_sim_time_value = LaunchConfiguration('use_sim_time')
    common_params = {'use_sim_time': use_sim_time_value}
    L = LaunchConfiguration

    perception = [
        Node(package='kaboat_perception', executable='occupancy_grid',
             name='occupancy_grid', output='screen', parameters=[common_params],
             condition=IfCondition(LaunchConfiguration('start_occupancy_grid'))),
        # 부표 지도 — 라이다 = 있음·위치, 카메라 RGB = 색. 구독자(탐색 활성 중)가 있을 때만 계산.
        Node(package='kaboat_perception', executable='buoy_lidar_detector',
             name='buoy_lidar_detector', output='screen', parameters=[common_params]),
        # 도킹(YOLO·라이다 RANSAC·one-shot 경로)은 아래 docking_nodes — outdoor_dock_test 와 같은 구성
    ]

    behaviors = [
        Node(package='kaboat_behaviors', executable=exe,
             name=exe, output='screen', parameters=[common_params])
        for exe in ['station_keeper', 'obstacle_planner']
    ]
    # 탐색 목표 색은 대회 당일 공지 — 빨강·초록 우현(시계), 검정 좌현(반시계) 선회
    behaviors.append(Node(
        package='kaboat_behaviors', executable='search_circler',
        name='search_circler', output='screen', parameters=[common_params, {
            'search.target_color': ParameterValue(
                LaunchConfiguration('search_target_color'), value_type=str),
        }]))
    behaviors.append(Node(
        package='kaboat_behaviors', executable='mission_approach',
        name='mission_approach', output='screen', parameters=[common_params, {
            'approach.motion_enabled': ParameterValue(
                LaunchConfiguration('approach_motion_enabled'), value_type=bool),
        }]))
    behaviors.append(Node(
        package='kaboat_behaviors', executable='gate_follower',
        name='gate_follower', output='screen',
        parameters=[LaunchConfiguration('gate_params_file'), common_params, {
            'gate.motion_enabled': ParameterValue(
                LaunchConfiguration('gate_motion_enabled'), value_type=bool),
        }]))
    behaviors.extend(docking_nodes(
        target_class=L('dock_target_class'), motion_enabled=L('dock_motion_enabled'),
        weights=L('dock_weights'), device=L('dock_device'),
        approach_cmd=L('dock_approach_cmd'), final_cmd=L('dock_final_cmd'),
        retreat_cmd=L('dock_retreat_cmd'), seek_cmd=L('dock_seek_cmd'),
        marker_order=L('dock_marker_order'),
        position_source=L('dock_position_source'), use_sim_time=use_sim_time_value,
        base_frame=L('dock_base_frame'),
        camera_tf_condition=IfCondition(L('publish_camera_tf'))))

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
                             LaunchConfiguration('mission_points_file'), value_type=str)}]),
    ]

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
        DeclareLaunchArgument('dock_retreat_cmd', default_value='0.05',
                              description='정지점에서 후진 출력'),
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
        *perception,
        *behaviors,
        *control,
        *mission,
    ])
