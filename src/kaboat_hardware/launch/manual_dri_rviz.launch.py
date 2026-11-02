"""Laptop: RViz-only dashboard for Jetson manual driving and DRI monitoring."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(get_package_share_directory('kaboat_hardware'),
                          'rviz', 'manual_dri_test.rviz')
    return LaunchDescription([
        Node(package='rviz2', executable='rviz2', name='manual_dri_rviz',
             output='screen', arguments=['-d', config]),
    ])
