"""occupancy_grid 시각화용 RViz2 실행 — autonomy.launch.py 와 별도 터미널.

사용법 (sim 검증):
    # 터미널 1 — 시뮬레이터
    ros2 launch kaboat_sim simulation.launch.py

    # 터미널 2 — 자율주행 스택
    ros2 launch kaboat_bringup autonomy.launch.py

    # 터미널 3 — RViz (Map: /occupancy_grid, LaserScan: /scan, Odometry: /odom)
    ros2 launch kaboat_bringup rviz.launch.py
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    bringup_share = get_package_share_directory('kaboat_bringup')
    rviz_config = os.path.join(bringup_share, 'rviz', 'occupancy_grid.rviz')

    return LaunchDescription([
        Node(package='rviz2', executable='rviz2', name='rviz2',
             arguments=['-d', rviz_config], output='screen',
             parameters=[{'use_sim_time': True}]),
    ])
