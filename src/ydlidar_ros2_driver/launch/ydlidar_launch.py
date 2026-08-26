#!/usr/bin/python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import LifecycleNode, Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    share_dir = get_package_share_directory('ydlidar_ros2_driver')
    hardware_share = get_package_share_directory('kaboat_hardware')
    default_param_file = os.path.join(hardware_share, 'config', 'tg50.yaml')
    parameter_file = LaunchConfiguration('params_file')

    params_declare = DeclareLaunchArgument(
        'params_file',
        default_value=default_param_file,
        description='Path to ROS 2 parameters file to use.'
    )

    driver_node = LifecycleNode(
        package='ydlidar_ros2_driver',
        executable='ydlidar_ros2_driver_node',
        name='ydlidar_ros2_driver_node',
        namespace='',
        output='screen',
        emulate_tty=True,
        parameters=[parameter_file],
    )

    tf2_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_pub_laser',
        arguments=['--x', '0', '--y', '0', '--z', '0.02', '--yaw', '0', '--pitch', '0', '--roll', '0', '--frame-id', 'base_link', '--child-frame-id', 'laser_frame'],
    )

    return LaunchDescription([
        params_declare,
        driver_node,
        tf2_node,
    ])
