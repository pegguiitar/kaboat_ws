"""자율주행 알고리즘 스택 launch.

실물 안전 기본값:
  - use_sim_time=false
  - Gazebo용 twist2thrust는 실행하지 않음

센서 배선 확인은 먼저 ``real_sensors.launch.py``를 사용한다. 이 launch는
센서 드라이버나 실제 모터 드라이버를 자동으로 띄우지 않는다.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    bringup_share = get_package_share_directory('kaboat_bringup')
    mission_params = os.path.join(bringup_share, 'config', 'mission_params.yaml')

    use_sim_time_value = LaunchConfiguration('use_sim_time')
    common_params = {'use_sim_time': use_sim_time_value}
    dock_detector_params = {
        **common_params,
        'weights': LaunchConfiguration('dock_weights'),
        'target_class': LaunchConfiguration('dock_target_class'),
        'device': LaunchConfiguration('dock_device'),
        'wall_to_mouth_m': ParameterValue(
            LaunchConfiguration('wall_to_mouth_m'), value_type=float),
    }

    perception = [
        Node(package='kaboat_perception', executable='occupancy_grid',
             name='occupancy_grid', output='screen', parameters=[common_params]),
        Node(package='kaboat_perception', executable='buoy_detector',
             name='buoy_detector', output='screen', parameters=[common_params]),
        Node(package='kaboat_perception', executable='dock_mark_detector',
             name='dock_mark_detector', output='screen', parameters=[dock_detector_params]),
    ]

    behaviors = [
        Node(package='kaboat_behaviors', executable=exe,
             name=exe, output='screen', parameters=[common_params])
        for exe in ['gate_follower', 'station_keeper',
                    'search_circler', 'obstacle_planner']
    ]
    behaviors.append(Node(
        package='kaboat_behaviors', executable='docking_ctrl',
        name='docking_ctrl', output='screen',
        parameters=[common_params, {
            'dock.motion_enabled': ParameterValue(
                LaunchConfiguration('dock_motion_enabled'), value_type=bool),
        }]))

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
             parameters=[mission_params, common_params]),
    ]

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Gazebo 실행 때만 true'),
        DeclareLaunchArgument(
            'use_sim_actuator', default_value='false',
            description='Gazebo용 twist2thrust 실행 여부; 실물에서는 false'),
        DeclareLaunchArgument(
            'dock_weights', default_value='',
            description='검증된 YOLO26n-seg .pt 절대 경로; 비워두면 검출 fail-closed'),
        DeclareLaunchArgument(
            'dock_target_class', default_value='red_triangle',
            description='red_triangle / green_circle / blue_square'),
        DeclareLaunchArgument(
            'dock_device', default_value='cpu',
            description='Ultralytics device: cpu 또는 Jetson CUDA 0'),
        DeclareLaunchArgument(
            'wall_to_mouth_m', default_value='2.5',
            description='마커 벽에서 입구까지의 수평 거리; 현장 실측 필요'),
        DeclareLaunchArgument(
            'dock_motion_enabled', default_value='false',
            description='기본값 false; TF/오프셋/안전 검증 후에만 true'),
        *perception,
        *behaviors,
        *control,
        *mission,
    ])
