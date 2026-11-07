"""Three marker projections -> 1m inward selected goal -> DRI/B-spline mission.

Existing camera/YOLO, IMU, /scan and /odom may run separately. Starts in preview;
no motor commands until drive_enabled=true and /dock/avoid/start is requested.
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
    bringup = get_package_share_directory('kaboat_bringup')
    config = os.path.join(get_package_share_directory('kaboat_hardware'), 'config', 'dock_marker_avoid.yaml')
    shared_topics={'path':'/dock/bspline_path','dri':'/avoid/dri','dri_marks':'/avoid/dri_marks'}
    remaps=[(f'/real_avoid_test/{name}',shared_topics.get(name,f'/dock/avoid/{name}'))
            for name in ('path','dri','markers','dri_marks','status','cmd_preview','start')]
    remaps.append(('/start_mission', '/dock/avoid/start_mission'))
    return LaunchDescription([
        DeclareLaunchArgument('controller_executable',default_value='real_obstacle_avoid_test'),
        DeclareLaunchArgument('controller_name',default_value='dock_marker_avoid'),
        DeclareLaunchArgument('controller_config',default_value=config),
        DeclareLaunchArgument('target_class', default_value='triangle', choices=['triangle', 'circle', 'square']),
        DeclareLaunchArgument('goal_offset_m', default_value='1.0'),
        DeclareLaunchArgument('marker_behind_wall_m', default_value='1.0'),
        DeclareLaunchArgument('dock_stop_distance_m', default_value='1.0'),
        DeclareLaunchArgument('goal_average_window_s', default_value='0.0'),
        DeclareLaunchArgument('drive_enabled', default_value='false'),
        DeclareLaunchArgument('approach_only_entry_search', default_value='false'),
        DeclareLaunchArgument('start_occupancy_grid', default_value='true'),
        Node(package='kaboat_perception', executable='occupancy_grid', name='occupancy_grid',
             respawn=True, respawn_delay=2.0, output='screen', condition=IfCondition(LaunchConfiguration('start_occupancy_grid')),
             parameters=[{'use_sim_time': False, 'resolution': 0.2, 'size': 15.0,
                          'publish_rate': 10.0, 'enable_tilt_gate': True,
                          'pitch_min_deg': -1.2, 'pitch_max_deg': 4.0,
                          'roll_min_deg': -3.0, 'roll_max_deg': 3.0,
                          'lidar_offset_x': 0.73, 'lidar_offset_y': 0.0}]),
        Node(package='kaboat_hardware', executable='dock_lidar_markers', name='dock_lidar_markers',
             respawn=True, respawn_delay=2.0, output='screen', parameters=[{
                 'marker_behind_wall_m': ParameterValue(LaunchConfiguration('marker_behind_wall_m'), value_type=float),
                 'target_class': LaunchConfiguration('target_class'),
                 'station_normal_hint_odom': [1.0,0.0]}]),
        Node(package='kaboat_hardware', executable='dock_marker_goal', name='dock_marker_goal',
             respawn=True, respawn_delay=2.0, output='screen', parameters=[{
                 'target_class': LaunchConfiguration('target_class'),
                 'goal_average_window_s': ParameterValue(LaunchConfiguration('goal_average_window_s'), value_type=float),
                 'offset_m': ParameterValue(LaunchConfiguration('goal_offset_m'), value_type=float),
                 'dock_stop_distance_m': ParameterValue(LaunchConfiguration('dock_stop_distance_m'), value_type=float),
                 'adaptive_entry_enabled':True,'entry_step_m':0.15,'entry_search_radius_m':1.5,
                 'approach_only_entry_search':ParameterValue(LaunchConfiguration('approach_only_entry_search'),value_type=bool)}]),
        Node(package='kaboat_hardware', executable=LaunchConfiguration('controller_executable'),
             name=LaunchConfiguration('controller_name'), respawn=True, respawn_delay=2.0, output='screen', parameters=[LaunchConfiguration('controller_config'), {
                 'drive_enabled': ParameterValue(LaunchConfiguration('drive_enabled'), value_type=bool),
                 'wait_for_start': True}], remappings=remaps),
    ])
