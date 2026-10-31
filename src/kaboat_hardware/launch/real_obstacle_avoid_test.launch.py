"""Real tank avoidance test; launch indoor_tank separately for /scan and /odom.

Preview (no motor commands):
  ros2 launch kaboat_hardware real_obstacle_avoid_test.launch.py
Drive after inspecting the preview and starting the thruster driver:
  ros2 launch kaboat_hardware real_obstacle_avoid_test.launch.py drive_enabled:=true
  ros2 service call /real_avoid_test/start std_srvs/srv/Trigger '{}'
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    config = os.path.join(get_package_share_directory('kaboat_hardware'),
                          'config', 'real_obstacle_avoid_test.yaml')
    drive_enabled = LaunchConfiguration('drive_enabled')
    rviz_config = os.path.join(get_package_share_directory('kaboat_hardware'),
                               'rviz', 'real_obstacle_avoid_test.rviz')
    return LaunchDescription([
        DeclareLaunchArgument('drive_enabled', default_value='false',
                              description='Publish commands to /cmd_vel (default: preview only)'),
        DeclareLaunchArgument('allow_escape', default_value='false',
                              description='Permit FSM reverse ESCAPE command'),
        DeclareLaunchArgument('goal_distance', default_value='3.0',
                              description='Goal distance ahead of boat at start [m]'),
        DeclareLaunchArgument('start_occupancy_grid', default_value='true',
                              description='Launch mapping node; false if one already runs'),
        DeclareLaunchArgument('grid_size', default_value='10.0',
                              description='Rolling occupancy grid side length [m]'),
        DeclareLaunchArgument('enable_tilt_gate', default_value='true',
                              description='Use IMU roll/pitch to reject scans (default: true)'),
        DeclareLaunchArgument('tilt_threshold_deg', default_value='5.0',
                              description='Tilt gate threshold [deg] (default: 5.0)'),
        DeclareLaunchArgument('publish_scan_tf', default_value='true',
                              description='Publish identity base_link -> onboard_laser_frame for RViz'),
        DeclareLaunchArgument('enable_rviz', default_value='false',
                              description='Open RViz on this computer (usually use the shore PC instead)'),
        LogInfo(msg='[RealAvoidTest] Preview path first; motors require drive_enabled:=true and start service.'),
        Node(
            package='kaboat_perception', executable='occupancy_grid',
            name='occupancy_grid', output='screen',
            parameters=[{
                'resolution': 0.2,
                'size': ParameterValue(LaunchConfiguration('grid_size'), value_type=float),
                'publish_rate': 10.0,
                'enable_tilt_gate': ParameterValue(
                    LaunchConfiguration('enable_tilt_gate'), value_type=bool),
                'tilt_threshold_deg': ParameterValue(
                    LaunchConfiguration('tilt_threshold_deg'), value_type=float),
                'use_sim_time': False,
            }],
            condition=IfCondition(LaunchConfiguration('start_occupancy_grid')),
        ),
        Node(
            package='kaboat_hardware', executable='real_obstacle_avoid_test',
            name='real_obstacle_avoid_test', output='screen',
            parameters=[config, {
                'drive_enabled': ParameterValue(drive_enabled, value_type=bool),
                'allow_escape': ParameterValue(
                    LaunchConfiguration('allow_escape'), value_type=bool),
                'goal_distance': ParameterValue(
                    LaunchConfiguration('goal_distance'), value_type=float),
                'use_sim_time': False,
            }],
        ),
        # occupancy_grid assumes the onboard laser is at base_link. Match that
        # assumption in RViz so /scan can be transformed into the odom frame.
        Node(
            package='tf2_ros', executable='static_transform_publisher',
            name='real_avoid_onboard_laser_tf', output='screen',
            arguments=['--x', '0', '--y', '0', '--z', '0',
                       '--roll', '0', '--pitch', '0', '--yaw', '0',
                       '--frame-id', 'base_link',
                       '--child-frame-id', 'onboard_laser_frame'],
            condition=IfCondition(LaunchConfiguration('publish_scan_tf')),
        ),
        Node(
            package='rviz2', executable='rviz2', name='real_avoid_rviz',
            output='screen', arguments=['-d', rviz_config],
            condition=IfCondition(LaunchConfiguration('enable_rviz')),
        ),
    ])
