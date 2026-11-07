"""Docking sensors + existing B-spline controller + optional serial thrusters.

Explicit /dock/start is required, including after any process respawn or stop.
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    bringup=get_package_share_directory('kaboat_bringup')
    hardware=get_package_share_directory('kaboat_hardware')
    return LaunchDescription([
        DeclareLaunchArgument('drive_enabled',default_value='false'),
        DeclareLaunchArgument('start_thrusters',default_value='false'),
        DeclareLaunchArgument('thruster_port',default_value='/dev/ttyCH341USB0'),
        DeclareLaunchArgument('target_class',default_value='triangle'),
        DeclareLaunchArgument('dock_stop_distance_m',default_value='1.0'),
        DeclareLaunchArgument('goal_average_window_s',default_value='0.0'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(bringup,'launch','dock_marker_avoid.launch.py')),
            launch_arguments={
                'controller_executable':'dock_ctrl','controller_name':'dock_ctrl',
                'controller_config':os.path.join(hardware,'config','dock_ctrl.yaml'),
                'drive_enabled':LaunchConfiguration('drive_enabled'),
                'approach_only_entry_search':'true',
                'target_class':LaunchConfiguration('target_class'),
                'goal_average_window_s':LaunchConfiguration('goal_average_window_s'),
                'dock_stop_distance_m':LaunchConfiguration('dock_stop_distance_m')}.items()),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(hardware,'launch','thrusters.launch.py')),
            condition=IfCondition(LaunchConfiguration('start_thrusters')),
            launch_arguments={'hardware_type':'serial','port':LaunchConfiguration('thruster_port'),
                              'allow_port_scan':'false','max_output_ratio':'1.0','gps_logger':'false'}.items()),
    ])
