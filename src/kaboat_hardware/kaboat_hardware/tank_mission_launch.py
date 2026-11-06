"""Shared launch wiring for single gate/avoid missions in a 10 x 5 m tank."""

import math
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _float(context, name):
    value = float(LaunchConfiguration(name).perform(context))
    if not math.isfinite(value):
        raise ValueError(f'{name} must be finite')
    return value


def _nodes(context, mission):
    entry = [_float(context, key) for key in
             ('entry_x', 'entry_y', 'entry_angle_deg')]
    goal = [_float(context, key) for key in ('goal_x', 'goal_y')]
    tank_x = _float(context, 'tank_size_x')
    tank_y = _float(context, 'tank_size_y')
    margin = _float(context, 'tank_margin')
    linear = _float(context, 'max_linear_cmd')
    angular = _float(context, 'max_angular_cmd')
    approach_ratio = _float(context, 'approach_thrust_ratio')
    grid_size = _float(context, 'grid_size')
    drive = LaunchConfiguration('drive_enabled').perform(context).lower() in (
        'true', '1', 'yes')
    if (tank_x <= 0 or tank_y <= 0 or margin < 0
            or margin >= min(tank_x, tank_y) / 2):
        raise ValueError('invalid tank size or margin')
    if not all(margin <= x <= tank_x - margin and
               margin <= y <= tank_y - margin
               for x, y in (entry[:2], goal)):
        raise ValueError('entry and goal must lie inside the tank margin')
    if math.hypot(goal[0] - entry[0], goal[1] - entry[1]) <= 2.0:
        raise ValueError('entry and goal must be more than 2 m apart')
    if not 0 < linear <= 1 or not 0 < angular <= 1 or not 0 < approach_ratio <= 1:
        raise ValueError('command limits and approach ratio must be in (0, 1]')
    if grid_size <= 0:
        raise ValueError('grid_size must be positive')

    shared_limits = {'max_linear': linear, 'max_angular': angular,
                     'use_sim_time': False}
    behavior_params = [shared_limits]
    if mission == 'gate':
        pairs = int(LaunchConfiguration('required_pairs').perform(context))
        gate_ratio = _float(context, 'gate_thrust_ratio')
        if pairs < 1 or not 0 < gate_ratio <= 1:
            raise ValueError('required_pairs and gate_thrust_ratio are invalid')
        behavior_params = [
            LaunchConfiguration('gate_params_file'), shared_limits,
            {'gate.required_pairs': pairs,
             'gate.thrust_ratio': gate_ratio,
             # cmd_mux output is isolated; the guard controls /cmd_vel.
             'gate.motion_enabled': True},
        ]
    else:
        behavior_params = [
            LaunchConfiguration('avoid_params_file'), shared_limits]

    return [
        Node(package='kaboat_perception', executable='occupancy_grid',
             name='occupancy_grid', output='screen', parameters=[{
                 'resolution': 0.2,
                 'size': grid_size,
                 'publish_rate': 10.0,
                 'enable_tilt_gate': ParameterValue(
                     LaunchConfiguration('enable_tilt_gate'), value_type=bool),
                 'use_sim_time': False,
             }], condition=IfCondition(LaunchConfiguration('start_occupancy_grid'))),
        Node(package='kaboat_behaviors', executable='mission_approach',
             name='mission_approach', output='screen', parameters=[
                 shared_limits, {
                     'approach.motion_enabled': drive,
                     'approach.thrust_ratio': approach_ratio,
                 }]),
        Node(package='kaboat_behaviors',
             executable=('gate_follower' if mission == 'gate'
                         else 'obstacle_planner'),
             name=('gate_follower' if mission == 'gate'
                   else 'obstacle_planner'),
             output='screen', parameters=behavior_params),
        Node(package='kaboat_control', executable='cmd_mux',
             name='cmd_mux', output='screen',
             parameters=[{'use_sim_time': False}],
             remappings=[('/cmd_vel', '/tank_mission/cmd_raw')]),
        Node(package='kaboat_hardware', executable='tank_mission_guard',
             name='tank_mission_guard', output='screen', parameters=[{
                 'mission': mission,
                 'drive_enabled': drive,
                 'allow_reverse': ParameterValue(
                     LaunchConfiguration('allow_reverse'), value_type=bool),
                 'max_linear_cmd': linear,
                 'max_angular_cmd': angular,
                 'use_sim_time': False,
             }]),
        Node(package='kaboat_mission', executable='mission_manager',
             name='mission_manager', output='screen', parameters=[{
                 'auto_start': True,
                 'start_state': mission,
                 'single_mission': True,
                 # Preview starts the behavior immediately; drive first aligns
                 # at the measured entry pose and heading.
                 'entry_required': drive,
                 f'entry_waypoints.{mission}': entry,
                 f'entry_ready.{mission}': True,
                 f'waypoints.{mission}': goal,
                 'use_sim_time': False,
             }]),
    ]


def generate_tank_mission_launch(mission):
    if mission not in ('gate', 'avoid'):
        raise ValueError('mission must be gate or avoid')
    hardware_share = get_package_share_directory('kaboat_hardware')
    bringup_share = get_package_share_directory('kaboat_bringup')
    args = [
        DeclareLaunchArgument('entry_x', description='Measured entry X in odom [m]'),
        DeclareLaunchArgument('entry_y', description='Measured entry Y in odom [m]'),
        DeclareLaunchArgument('entry_angle_deg', description='Entry heading [deg]; 0=+X'),
        DeclareLaunchArgument('goal_x', description='Mission end goal X in odom [m]'),
        DeclareLaunchArgument('goal_y', description='Mission end goal Y in odom [m]'),
        DeclareLaunchArgument('drive_enabled', default_value='false',
                              description='false: preview only; true: arm service can enable /cmd_vel'),
        DeclareLaunchArgument('allow_reverse', default_value='false',
                              description='Allow reverse commands from avoid FSM'),
        DeclareLaunchArgument('tank_size_x', default_value='10.0'),
        DeclareLaunchArgument('tank_size_y', default_value='5.0'),
        DeclareLaunchArgument('tank_margin', default_value='0.5'),
        # 최종 /cmd_vel 상한. 게이트 LOS(0.3/0.3)와 회피(전진 0.23·회전 0.36)가
        # 잘리지 않도록 전진 0.3·회전 0.4 (이전 0.06/0.02 에선 회전이 0.02 로 막혔다).
        DeclareLaunchArgument('max_linear_cmd', default_value='0.30'),
        DeclareLaunchArgument('max_angular_cmd', default_value='0.40'),
        DeclareLaunchArgument('approach_thrust_ratio', default_value='0.5'),
        DeclareLaunchArgument('start_occupancy_grid', default_value='true',
                              description='false if one /occupancy_grid publisher already exists'),
        DeclareLaunchArgument('enable_tilt_gate', default_value='true'),
        DeclareLaunchArgument('grid_size', default_value='15.0'),
    ]
    if mission == 'gate':
        args.extend([
            DeclareLaunchArgument('gate_params_file', default_value=os.path.join(
                bringup_share, 'config', 'gate_params.yaml')),
            DeclareLaunchArgument('required_pairs', default_value='3'),
            DeclareLaunchArgument('gate_thrust_ratio', default_value='0.5'),
        ])
    else:
        args.append(DeclareLaunchArgument(
            'avoid_params_file', default_value=os.path.join(
                hardware_share, 'config', 'tank_avoid_mission.yaml')))
    return LaunchDescription([
        *args,
        LogInfo(msg=f'[Tank {mission}] preview default; drive needs /tank_mission/start'),
        OpaqueFunction(function=lambda context: _nodes(context, mission)),
    ])
