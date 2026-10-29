"""D455 + live dock detection, without mission or motor control nodes.

The camera mounting TF to base_link must be supplied by the robot.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    bringup_share = get_package_share_directory('kaboat_bringup')
    weights = os.path.join(
        get_package_share_directory('kaboat_perception'), 'models',
        'dock_sequence_v3_yolo26n_best.pt')
    return LaunchDescription([
        DeclareLaunchArgument('start_camera', default_value='true'),
        DeclareLaunchArgument('dock_weights', default_value=weights),
        DeclareLaunchArgument('dock_device', default_value='cpu'),
        DeclareLaunchArgument('dock_target_class', default_value='red_triangle'),
        DeclareLaunchArgument(
            'wall_to_mouth_m', default_value='2.5',
            description='마커 벽에서 입구까지 거리 [m]; 현장 실측값 필요'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(bringup_share, 'launch', 'd455.launch.py')),
            condition=IfCondition(LaunchConfiguration('start_camera'))),
        Node(
            package='kaboat_perception', executable='dock_mark_detector',
            name='dock_mark_detector', output='screen',
            parameters=[{
                'use_sim_time': False,
                'enabled': True,
                'weights': LaunchConfiguration('dock_weights'),
                'device': LaunchConfiguration('dock_device'),
                'target_class': LaunchConfiguration('dock_target_class'),
                'wall_to_mouth_m': ParameterValue(
                    LaunchConfiguration('wall_to_mouth_m'), value_type=float),
            }]),
    ])
