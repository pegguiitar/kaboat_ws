"""occupancy_grid 시각화용 RViz2 실행 — autonomy.launch.py 와 별도 터미널.

실물 안전 기본값 (autonomy.launch.py 와 동일 규약):
  use_sim_time:=false — Gazebo 를 쓸 때만 true 로 켠다. 실물에서 true 면
  RViz 가 절대 오지 않는 /clock 을 기다리며 화면이 빈 채로 멈춘다.

⚠️ **TF 는 이 launch 가 발행하지 않는다.** RViz 는 Fixed Frame 해석에 TF 가
필요한데, 실물에는 TF 발행자가 없다(gq7.yaml 의 tf_mode: 0,
robot_state_publisher 미실행). TF 는 로봇 bringup 의 책임이므로
``real_sensors.launch.py`` / ``indoor_tank.launch.py`` 가 발행한다
(kaboat_hardware 의 odom_tf_broadcaster). 시각화 패키지가 하드웨어 패키지를
띄우면 kaboat_hardware→kaboat_bringup 과 순환 의존이 된다.

``Fixed Frame [odom] does not exist`` 가 뜨면 센서 launch 가 안 떠 있거나
그쪽 publish_tf 가 꺼진 것이다.

사용법:
    # 실물 — 센서 bringup(TF 포함) + 격자 + RViz
    ros2 launch kaboat_hardware real_sensors.launch.py
    ros2 run kaboat_perception occupancy_grid
    ros2 launch kaboat_bringup rviz.launch.py

    # sim — Gazebo 가 clock/TF 를 모두 준다
    ros2 launch kaboat_bringup rviz.launch.py use_sim_time:=true
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_share = get_package_share_directory('kaboat_bringup')
    default_config = os.path.join(bringup_share, 'rviz', 'occupancy_grid.rviz')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Gazebo 를 실행 중일 때만 true'),
        DeclareLaunchArgument(
            'rviz_config', default_value=default_config,
            description='RViz2 설정 파일 경로'),

        Node(
            package='rviz2', executable='rviz2', name='rviz2',
            arguments=['-d', LaunchConfiguration('rviz_config')],
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }],
        ),
    ])
