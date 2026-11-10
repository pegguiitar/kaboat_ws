"""Preview LiDAR/DRI gate pairing and path without cmd_mux or a motor driver."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _nodes(context):
    goal_x = float(LaunchConfiguration('goal_x').perform(context))
    goal_y = float(LaunchConfiguration('goal_y').perform(context))
    pair_count = int(LaunchConfiguration('required_pairs').perform(context))
    if not 0.25 <= goal_x <= 9.75 or not 0.25 <= goal_y <= 4.75:
        raise ValueError('gate goal must lie inside the 10 x 5 m tank margin')
    if pair_count < 1:
        raise ValueError('required_pairs must be positive')

    gate_config = LaunchConfiguration('gate_params_file').perform(context)
    nodes = [
        Node(
            package='kaboat_behaviors', executable='gate_follower',
            name='gate_follower', output='screen',
            parameters=[gate_config, {
                'gate.required_pairs': pair_count,
                'gate.motion_enabled': False,
                'use_sim_time': False,
            }],
        ),
        Node(
            package='kaboat_mission', executable='mission_manager',
            name='mission_manager', output='screen',
            parameters=[{
                'auto_start': True,
                'start_state': 'gate',
                'single_mission': True,
                # Read-only gate preview has no measured entry pose or actuator.
                'entry_required': False,
                'waypoints.gate': [goal_x, goal_y],
                'points_file': '',          # 수조 좌표 — 야외 rec 기록 지점 안 읽음
                'use_sim_time': False,
            }],
        ),
    ]
    if LaunchConfiguration('start_occupancy_grid').perform(context).lower() == 'true':
        nodes.append(Node(
            package='kaboat_perception', executable='occupancy_grid',
            name='occupancy_grid', output='screen',
            # /scan 은 이제 기울어도 발행되므로(/scan/level 만 차단) 지도 쪽 게이트를 켠다
            parameters=[{'enable_tilt_gate': True, 'use_sim_time': False}],
        ))
    return nodes


def generate_launch_description():
    share = get_package_share_directory('kaboat_bringup')
    config = os.path.join(share, 'config', 'gate_params.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('goal_x', description='Measured tank/odom X beyond the last gate'),
        DeclareLaunchArgument('goal_y', description='Measured tank/odom Y beyond the last gate'),
        DeclareLaunchArgument('required_pairs', default_value='3',
                              description='Number of gate pairs to cross; tank trial uses 3'),
        DeclareLaunchArgument('gate_params_file', default_value=config),
        DeclareLaunchArgument('start_occupancy_grid', default_value='false',
                              description='true only if no /occupancy_grid publisher exists'),
        OpaqueFunction(function=_nodes),
    ])
