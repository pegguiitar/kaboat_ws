"""Shore-laptop RViz with local target and measured boat trajectory overlays."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    kind = LaunchConfiguration('test_kind')
    rviz_config = LaunchConfiguration('rviz_config')
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
    )
    return LaunchDescription([
        DeclareLaunchArgument('test_kind', default_value='none'),
        DeclareLaunchArgument(
            'rviz_config',
            default_value='/workspace/kaboat_ws/src/kaboat_bringup/rviz/tank_tracking.rviz'),
        Node(
            package='kaboat_hardware',
            executable='tank_test_visualization',
            name='tank_test_visualization',
            output='screen',
            parameters=[{'test_kind': kind}],
        ),
        RegisterEventHandler(OnProcessExit(
            target_action=rviz,
            on_exit=[EmitEvent(event=Shutdown(reason='RViz closed'))],
        )),
        rviz,
    ])
