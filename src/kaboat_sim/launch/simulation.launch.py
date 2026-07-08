'''
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch.substitutions import Command

def generate_launch_description():
    # 1. 패키지 경로 탐색 (WAM-V 경로는 이제 뺐습니다)
    kaboat_sim_share = get_package_share_directory('kaboat_sim')
    vrx_gz_share = get_package_share_directory('vrx_gz')

    vrx_gz_prefix = get_package_prefix('vrx_gz')
    vrx_plugin_path = os.path.join(vrx_gz_prefix, 'lib')
    
    # 2. 리소스 경로 설정 (VRX 파도 모델과 커스텀 모델만 유지)
    gz_resource_paths = [
        os.path.dirname(vrx_gz_share),
        os.path.join(vrx_gz_share, 'models'),
        os.path.join(kaboat_sim_share, 'models')
    ]
    gz_models_path = ":".join(gz_resource_paths)

    if 'GZ_SIM_RESOURCE_PATH' in os.environ:
        gz_models_path = f"{os.environ['GZ_SIM_RESOURCE_PATH']}:{gz_models_path}"

    set_plugin_path = SetEnvironmentVariable(name='GZ_SIM_SYSTEM_PLUGIN_PATH', value=vrx_plugin_path)
    set_resource_path = SetEnvironmentVariable(name='GZ_SIM_RESOURCE_PATH', value=gz_models_path)

    current_ld_library_path = os.environ.get('LD_LIBRARY_PATH', '')
    new_ld_library_path = f"{vrx_plugin_path}:{current_ld_library_path}" if current_ld_library_path else vrx_plugin_path
    set_ld_library = SetEnvironmentVariable(name='LD_LIBRARY_PATH', value=new_ld_library_path)
    
    # 3. 가제보 실행
    world_file_path = os.path.join(kaboat_sim_share, 'worlds', 'perception_task.sdf')
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ]),
        launch_arguments={'gz_args': f'-r {world_file_path}'}.items(),
    )

    # 4. 💡 수정됨: 다시 원래 커스텀 배 XACRO 파일로 복구
    xacro_file_path = os.path.join(kaboat_sim_share, 'urdf', 'kaboat_msga_2.xacro')

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': Command(['xacro ', xacro_file_path]),
            'use_sim_time': True,
        }]
    )

    # 5. 💡 수정됨: 스폰 이름 'wamv' -> 'kaboat_msga'
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-world', 'kaboat_world',
            '-topic', 'robot_description',
            '-name', 'kaboat_msga', 
            '-x', '-528.0', '-y', '198.0', '-z', '0.2', '-Y', '-1.57'
        ],
        output='screen'
    )

    # 6. 💡 수정됨: Bridge 토픽을 커스텀 배에 맞게 복구
    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/lidar/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/camera@sensor_msgs/msg/Image[gz.msgs.Image',
            # 💡 가제보 스러스터의 공식 토픽 경로로 완벽하게 연결
            '/model/kaboat_msga/joint/left_engine_propeller_joint/cmd_thrust@std_msgs/msg/Float64]gz.msgs.Double',
            '/model/kaboat_msga/joint/right_engine_propeller_joint/cmd_thrust@std_msgs/msg/Float64]gz.msgs.Double'
        ],
        output='screen'
    )

    return LaunchDescription([
        set_plugin_path,
        set_resource_path,
        set_ld_library,
        gazebo_launch,
        robot_state_publisher,
        spawn_robot,
        ros_gz_bridge
    ])
'''
'''wam-v 를 위한 launch file'''
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch.substitutions import Command, LaunchConfiguration, PythonExpression

def generate_launch_description():
    # 1. 패키지 경로 탐색
    kaboat_sim_share = get_package_share_directory('kaboat_sim')
    vrx_gz_share = get_package_share_directory('vrx_gz')
    wamv_desc_share = get_package_share_directory('wamv_description')
    wamv_gz_share = get_package_share_directory('wamv_gazebo')

    vrx_gz_prefix = get_package_prefix('vrx_gz')
    vrx_plugin_path = os.path.join(vrx_gz_prefix, 'lib')
    
    # 2. 리소스 경로 종합 설정 (여기가 핵심입니다!)
    gz_resource_paths = [
        # [A] WAM-V 및 VRX 3D 메쉬를 찾기 위한 부모 디렉토리 (dirname 사용 - 인자 1개)
        os.path.dirname(wamv_desc_share),
        os.path.dirname(wamv_gz_share),
        os.path.dirname(vrx_gz_share),
        
        # [B] coast_waves 파도 모델 등을 찾기 위한 내부 models 디렉토리 (join 사용 - 인자 2개)
        os.path.join(vrx_gz_share, 'models'),
        os.path.join(wamv_gz_share, 'models'),
        os.path.join(kaboat_sim_share, 'models')
    ]
    
    # 리스트를 ':'로 연결하여 가제보에 전달
    gz_models_path = ":".join(gz_resource_paths)

    # 기존 환경 변수와 병합 (안전 장치)
    if 'GZ_SIM_RESOURCE_PATH' in os.environ:
        gz_models_path = f"{os.environ['GZ_SIM_RESOURCE_PATH']}:{gz_models_path}"

    # 3. 가제보 환경 변수 주입
    set_plugin_path = SetEnvironmentVariable(name='GZ_SIM_SYSTEM_PLUGIN_PATH', value=vrx_plugin_path)
    set_resource_path = SetEnvironmentVariable(name='GZ_SIM_RESOURCE_PATH', value=gz_models_path)

    current_ld_library_path = os.environ.get('LD_LIBRARY_PATH', '')
    new_ld_library_path = f"{vrx_plugin_path}:{current_ld_library_path}" if current_ld_library_path else vrx_plugin_path
    set_ld_library = SetEnvironmentVariable(name='LD_LIBRARY_PATH', value=new_ld_library_path)
    
    # 4. 가제보 Sim 서버 및 GUI 실행 (헤드리스 모드 지원)
    world_file_path = os.path.join(kaboat_sim_share, 'worlds', 'perception_task.sdf')
    
    headless_arg = DeclareLaunchArgument(
        'headless',
        default_value='False',
        description='Whether to run Gazebo in headless mode (server only)'
    )
    
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ]),
        launch_arguments={
            'gz_args': PythonExpression([
                "'-s -r ' + '", world_file_path, "' if '", LaunchConfiguration('headless'), "'.lower() == 'true' else '-r ' + '", world_file_path, "'"
            ])
        }.items(),
    )

    # 5. XACRO 파일 경로 지정 및 Robot State Publisher
    xacro_file_path = os.path.join(kaboat_sim_share, 'urdf', 'wamv_kaboat.xacro')

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': Command(['xacro ', xacro_file_path]),
            'use_sim_time': True,
        }]
    )

    

    # 6. 로봇 스폰
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-world', 'kaboat_world',
            '-topic', 'robot_description',
            '-name', 'wamv_kaboat', 
            '-x', '0.0', '-y', '0.0', '-z', '0.2',
        ],
        output='screen'
    )

    # 7. ROS-Gazebo Bridge 설정
    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            # [기존 설정] LiDAR 및 스러스터 제어
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/lidar/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/wamv/thrusters/left/thrust@std_msgs/msg/Float64]gz.msgs.Double',
            '/wamv/thrusters/right/thrust@std_msgs/msg/Float64]gz.msgs.Double',
            # 📷 [추가] 뎁스 카메라 설정 (Gazebo -> ROS2 방향이므로 '[' 사용)
            # 1. 일반 2D 컬러 이미지
            '/camera/depth/image@sensor_msgs/msg/Image[gz.msgs.Image',
            # 2. 깊이(Depth) 이미지
            '/camera/depth/depth_image@sensor_msgs/msg/Image[gz.msgs.Image',
            # 3. 3D 포인트 클라우드 (RViz에서 입체적으로 볼 때 아주 유용함)
            '/camera/depth/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
            # 4. 카메라 메타데이터 (FOV 등)
            '/camera/depth/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo'
        ],
        output='screen'
    )

    rviz_interactive_marker = Node(
        package='kaboat_sim',
        executable='rviz_interactive_marker.py',
        name='rviz_interactive_marker',
        output='screen'
    )

    return LaunchDescription([
        headless_arg,
        set_plugin_path,
        set_resource_path,
        set_ld_library,
        gazebo_launch,
        robot_state_publisher,
        spawn_robot,
        ros_gz_bridge,
        rviz_interactive_marker
    ])
