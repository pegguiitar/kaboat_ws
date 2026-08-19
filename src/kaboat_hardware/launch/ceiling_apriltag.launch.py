"""노트북용 천장 웹캠 + AprilTag 검출 launch.

영상 처리는 노트북 안에서 composable node/intra-process로 끝내고, Jetson에는
작은 /tf와 /apriltag/detections만 ROS 2 DDS로 전달한다. 두 PC는 같은
ROS_DOMAIN_ID를 사용하고 ROS_LOCALHOST_ONLY=0이어야 한다.
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode


def _bool_arg(context, name):
    return LaunchConfiguration(name).perform(context).strip().lower() in (
        '1', 'true', 'yes', 'on')


def _launch_setup(context):
    camera = LaunchConfiguration('camera').perform(context)
    width = int(LaunchConfiguration('width').perform(context))
    height = int(LaunchConfiguration('height').perform(context))
    orientation = int(LaunchConfiguration('orientation').perform(context))
    camera_frame = LaunchConfiguration('camera_frame').perform(context)
    camera_info_url = LaunchConfiguration('camera_info_url').perform(context)
    camera_format = LaunchConfiguration('format').perform(context)
    tag_config = LaunchConfiguration('tag_config_file').perform(context)
    tag_family = LaunchConfiguration('tag_family').perform(context)
    tag_id = int(LaunchConfiguration('tag_id').perform(context))
    tag_size = float(LaunchConfiguration('tag_size').perform(context))
    tag_frame = LaunchConfiguration('tag_frame').perform(context)

    if width <= 0 or height <= 0:
        raise ValueError('width와 height는 1 이상이어야 합니다')
    if orientation not in (0, 90, 180, 270):
        raise ValueError('orientation은 0/90/180/270 중 하나여야 합니다')
    if tag_id < 0:
        raise ValueError('tag_id는 0 이상이어야 합니다')
    if tag_size <= 0.0:
        raise ValueError('tag_size는 실제 미터 단위 양수여야 합니다')

    camera_parameters = {
        'camera': camera,
        'role': 'viewfinder',
        'width': width,
        'height': height,
        'orientation': orientation,
        # 노트북 시스템 시각을 써야 Jetson의 tag_timeout 판정과 일치한다.
        'use_node_time': True,
        'camera_info_url': camera_info_url,
        'frame_id': camera_frame,
    }
    if camera_format:
        camera_parameters['format'] = camera_format

    components = [
        ComposableNode(
            package='camera_ros',
            plugin='camera::CameraNode',
            name='camera',
            namespace='ceiling_cam',
            parameters=[camera_parameters],
            extra_arguments=[{'use_intra_process_comms': True}],
        ),
        ComposableNode(
            package='image_proc',
            plugin='image_proc::RectifyNode',
            name='rectify',
            namespace='ceiling_cam',
            remappings=[
                ('image', '/ceiling_cam/camera/image_raw'),
                ('camera_info', '/ceiling_cam/camera/camera_info'),
            ],
            extra_arguments=[{'use_intra_process_comms': True}],
        ),
    ]

    if _bool_arg(context, 'enable_detector'):
        components.append(ComposableNode(
            package='apriltag_ros',
            plugin='AprilTagNode',
            name='apriltag',
            namespace='apriltag',
            parameters=[
                tag_config,
                {
                    'family': tag_family,
                    'size': tag_size,
                    'tag.ids': [tag_id],
                    'tag.frames': [tag_frame],
                    'tag.sizes': [tag_size],
                },
            ],
            remappings=[
                ('image_rect', '/ceiling_cam/image_rect'),
                ('camera_info', '/ceiling_cam/camera/camera_info'),
            ],
            extra_arguments=[{'use_intra_process_comms': True}],
        ))

    actions = [ComposableNodeContainer(
        package='rclcpp_components',
        executable='component_container_mt',
        name='ceiling_apriltag_container',
        namespace='',
        composable_node_descriptions=components,
        output='screen',
    )]

    if _bool_arg(context, 'enable_status'):
        actions.append(Node(
            package='kaboat_hardware',
            executable='ceiling_apriltag_status',
            name='ceiling_apriltag_status',
            output='screen',
            parameters=[{
                'camera_info_topic': '/ceiling_cam/camera/camera_info',
                'detections_topic': '/apriltag/detections',
                'detection_timeout_sec': 0.5,
                'capture_latency_tolerance_sec': 0.2,
            }],
        ))
    return actions


def generate_launch_description():
    share = Path(get_package_share_directory('kaboat_hardware'))
    default_tag_config = str(share / 'config' / 'ceiling_apriltag.yaml')
    default_camera_info = Path.home() / '.ros/camera_info/ceiling_camera.yaml'

    return LaunchDescription([
        DeclareLaunchArgument('camera', default_value='0',
                              description='libcamera 카메라 번호 또는 이름'),
        DeclareLaunchArgument('width', default_value='1280'),
        DeclareLaunchArgument('height', default_value='720'),
        DeclareLaunchArgument('format', default_value='',
                              description='비우면 카메라가 지원하는 형식을 자동 선택'),
        DeclareLaunchArgument('orientation', default_value='0',
                              description='웹캠 영상 회전: 0/90/180/270'),
        DeclareLaunchArgument('camera_frame', default_value='ceiling_camera'),
        DeclareLaunchArgument(
            'camera_info_url', default_value=default_camera_info.as_uri(),
            description='cameracalibrator가 만든 보정 YAML의 file:// URL'),
        DeclareLaunchArgument('tag_config_file', default_value=default_tag_config),
        DeclareLaunchArgument('tag_family', default_value='36h11'),
        DeclareLaunchArgument('tag_id', default_value='0'),
        DeclareLaunchArgument('tag_size', default_value='0.160',
                              description='인쇄된 태그 바깥 한 변 실제 길이[m]'),
        DeclareLaunchArgument('tag_frame', default_value='tag36h11:0'),
        DeclareLaunchArgument('enable_detector', default_value='true'),
        DeclareLaunchArgument('enable_status', default_value='true'),
        LogInfo(msg='천장 노트북 모드 — 원본 영상은 로컬 처리하고 '
                    '/tf와 /apriltag/detections만 Wi-Fi로 공유합니다.'),
        OpaqueFunction(function=_launch_setup),
    ])
