"""Intel RealSense D455 → KABOAT 표준 카메라 토픽.

RealSense의 depth를 RGB 좌표계로 정렬하고, 드라이버 기본 토픽을 sim과 같은
표준 이름으로 remap한다. buoy/dock detector와 occupancy_grid는 센서 종류를
모른 채 동일한 토픽을 계속 구독한다.

사용법:
    ros2 launch kaboat_bringup d455.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import SetRemap


def generate_launch_description():
    realsense_share = get_package_share_directory('realsense2_camera')
    rs_launch = os.path.join(realsense_share, 'launch', 'rs_launch.py')

    # realsense-ros 기본 namespace/name은 둘 다 camera → /camera/camera/...
    camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rs_launch),
        launch_arguments={
            'camera_namespace': 'camera',
            'camera_name': 'camera',
            'enable_color': 'true',
            'enable_depth': 'true',
            'enable_sync': 'true',
            'align_depth.enable': 'true',
            'pointcloud.enable': 'true',
        }.items(),
    )

    return LaunchDescription([
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
