"""외부 USB 웹캠으로 천장 AprilTag를 실시간 검출하고 TF를 발행한다.

카메라 PC에서 실행하며, /ceiling_cam/image_raw, /detections 및 ceiling_camera -> tag TF를 발행한다.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    video_device = LaunchConfiguration('video_device')
    tag_id = LaunchConfiguration('tag_id')
    tag_size = LaunchConfiguration('tag_size')

    detector_node = Node(
        package='kaboat_hardware',
        executable='ceiling_apriltag_node',
        name='ceiling_apriltag_node',
        output='screen',
        parameters=[{
            'video_device': video_device,
            'tag_id': tag_id,
            'tag_size': tag_size,
            'width': 1280,
            'height': 720,
            'camera_frame': 'ceiling_camera',
            'tag_frame': 'tag36h11:1',
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument('video_device', default_value='/dev/video2',
                              description='외부 UVC 웹캠 V4L2 장치'),
        DeclareLaunchArgument('tag_id', default_value='1',
                              description='검출할 AprilTag ID (tag36h11 / ArUco)'),
        DeclareLaunchArgument('tag_size', default_value='0.300',
                              description='실제 태그 한 변의 길이 [m] (30cm = 0.300)'),
        LogInfo(msg='천장 AprilTag 실시간 검출 및 TF 발행 노드 시작'),
        detector_node,
    ])
