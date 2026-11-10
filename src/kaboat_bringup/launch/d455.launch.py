"""Intel RealSense D455를 KABOAT 표준 카메라 토픽으로 연결한다.

이 launch는 카메라 드라이버만 담당한다.
RGB는 검출용 640x480, 깊이는 측정 정밀도를 위한 native HD를 사용한다.
정렬 깊이 출력은 RGB와 같은 크기이므로 네트워크 영상 크기는 증가하지 않는다.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import SetRemap


def generate_launch_description():
    realsense_share = get_package_share_directory('realsense2_camera')
    rs_launch = os.path.join(realsense_share, 'launch', 'rs_launch.py')

    use_sim_time = LaunchConfiguration('use_sim_time')
    color_profile = LaunchConfiguration('color_profile')
    depth_profile = LaunchConfiguration('depth_profile')
    enable_pointcloud = LaunchConfiguration('enable_pointcloud')

    # realsense-ros 기본 namespace/name이 둘 다 camera라 원본은
    # /camera/camera/... 이다. 아래 remap으로 sim과 같은 계약을 유지한다.
    camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rs_launch),
        launch_arguments={
            'config_file': os.path.join(
                get_package_share_directory('kaboat_bringup'), 'config', 'd455_streams.yaml'),
            'camera_namespace': 'camera',
            'camera_name': 'camera',
            'enable_color': 'true',
            'enable_depth': 'true',
            'enable_sync': 'true',
            'align_depth.enable': 'true',
            'pointcloud.enable': enable_pointcloud,
            'rgb_camera.color_profile': color_profile,
            'depth_module.depth_profile': depth_profile,
            'publish_tf': 'true',
            'use_sim_time': use_sim_time,
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='실물에서는 반드시 false'),
        DeclareLaunchArgument(
            'color_profile', default_value='640x480x15',
            description='Jetson 초기 검증용 RGB width x height x fps'),
        DeclareLaunchArgument(
            'depth_profile', default_value='1280x720x15',
            description='Native HD stereo depth for metric precision; aligned output follows RGB size'),
        DeclareLaunchArgument(
            'enable_pointcloud', default_value='false',
            description='초기 검증에서는 CPU/USB 부하를 줄이기 위해 비활성'),
        GroupAction([
            SetRemap(src='/camera/camera/color/image_raw',
                     dst='/camera/color/image_raw'),
            SetRemap(src='/camera/camera/color/camera_info',
                     dst='/camera/camera_info'),
            SetRemap(src='/camera/camera/aligned_depth_to_color/image_raw',
                     dst='/camera/depth/image_raw'),
            SetRemap(src='/camera/camera/depth/color/points',
                     dst='/camera/depth/points'),
            camera,
        ]),
    ])
