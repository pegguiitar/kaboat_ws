"""D455 + live dock detection, without mission or motor control nodes.

Base origin: lidar at (+0.73, 0, 0) m; camera_link at (0, 0, +0.63) m.
Camera relative to lidar: (-0.73, 0, +0.63) m.
Mount rotation is assumed parallel to the lidar; optical TF comes from D455.
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
        'best.pt')
    engine = os.path.splitext(weights)[0] + '.engine'
    if os.path.isfile(engine):
        weights = engine
    return LaunchDescription([
        DeclareLaunchArgument('start_camera', default_value='true'),
        DeclareLaunchArgument('publish_lidar_mount_tf', default_value='true',
                              description='base_link to onboard lidar (+0.73m X); disable if another node owns this TF'),
        DeclareLaunchArgument('camera_parent_frame', default_value='onboard_laser_frame'),
        DeclareLaunchArgument('dock_weights', default_value=weights),
        DeclareLaunchArgument('dock_device', default_value='0'),
        DeclareLaunchArgument('dock_target_class', default_value='triangle'),
        DeclareLaunchArgument('confidence', default_value='0.45'),
        DeclareLaunchArgument('mask_erosion_pixels', default_value='1'),
        DeclareLaunchArgument('max_depth_sigma_m', default_value='0.25'),
        DeclareLaunchArgument('depth_outlier_floor_m', default_value='0.06'),
        DeclareLaunchArgument('min_depth_inlier_fraction', default_value='0.7'),
        DeclareLaunchArgument('marker_filter_tau_s', default_value='0.5'),
        DeclareLaunchArgument('marker_filter_gate_m', default_value='0.35'),
        DeclareLaunchArgument('marker_filter_reset_s', default_value='1.5'),
        DeclareLaunchArgument('debug_max_fps', default_value='5.0'),
        DeclareLaunchArgument('debug_max_width', default_value='320'),
        DeclareLaunchArgument('half', default_value='true'),
        DeclareLaunchArgument('image_size', default_value='640'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(bringup_share, 'launch', 'd455.launch.py')),
            condition=IfCondition(LaunchConfiguration('start_camera'))),
        Node(
            package='tf2_ros', executable='static_transform_publisher',
            name='lidar_mount_tf', output='screen',
            condition=IfCondition(LaunchConfiguration('publish_lidar_mount_tf')),
            arguments=['--x', '0.73', '--y', '0', '--z', '0',
                       '--roll', '0', '--pitch', '0', '--yaw', '0',
                       '--frame-id', 'base_link',
                       '--child-frame-id', LaunchConfiguration('camera_parent_frame')]),
        Node(
            package='tf2_ros', executable='static_transform_publisher',
            name='camera_mount_tf', output='screen',
            arguments=['--x', '-0.73', '--y', '0', '--z', '0.63',
                       '--roll', '0', '--pitch', '0', '--yaw', '0',
                       '--frame-id', LaunchConfiguration('camera_parent_frame'),
                       '--child-frame-id', 'camera_link']),
        Node(
            package='kaboat_perception', executable='dock_mark_detector',
            name='dock_mark_detector', output='screen',
            remappings=[
                *[(f'/dock/markers/{c}/ground', f'/dock/markers/{c}/depth_ground') for c in ('triangle','circle','square')],
                *[(f'/dock/markers/{c}/ground_base', f'/dock/markers/{c}/depth_ground_base') for c in ('triangle','circle','square')],
                ('/dock/markers_ground', '/dock/depth_markers_ground'),
                ('/dock/markers_base', '/dock/depth_markers_base'),
                ('/dock/marker_ground', '/dock/marker_ground_depth'),
            ],
            respawn=True, respawn_delay=3.0,
            parameters=[{
                'use_sim_time': False,
                'enabled': True,
                'publish_debug_image': True,
                'mask_erosion_pixels': ParameterValue(LaunchConfiguration('mask_erosion_pixels'), value_type=int),
                'max_depth_sigma_m': ParameterValue(LaunchConfiguration('max_depth_sigma_m'), value_type=float),
                'depth_outlier_floor_m': ParameterValue(LaunchConfiguration('depth_outlier_floor_m'), value_type=float),
                'min_depth_inlier_fraction': ParameterValue(LaunchConfiguration('min_depth_inlier_fraction'), value_type=float),
                'marker_filter_tau_s': ParameterValue(LaunchConfiguration('marker_filter_tau_s'), value_type=float),
                'marker_filter_gate_m': ParameterValue(LaunchConfiguration('marker_filter_gate_m'), value_type=float),
                'marker_filter_reset_s': ParameterValue(LaunchConfiguration('marker_filter_reset_s'), value_type=float),
                'debug_max_fps': ParameterValue(LaunchConfiguration('debug_max_fps'), value_type=float),
                'debug_max_width': ParameterValue(LaunchConfiguration('debug_max_width'), value_type=int),
                'weights': LaunchConfiguration('dock_weights'),
                'device': ParameterValue(LaunchConfiguration('dock_device'), value_type=str),
                'half': ParameterValue(LaunchConfiguration('half'), value_type=bool),
                'target_class': LaunchConfiguration('dock_target_class'),
                'confidence': ParameterValue(LaunchConfiguration('confidence'), value_type=float),
                'image_size': ParameterValue(LaunchConfiguration('image_size'), value_type=int),
            }]),
    ])
