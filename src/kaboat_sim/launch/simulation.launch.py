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
            'robot_description': ParameterValue(Command(['xacro ', xacro_file_path]), value_type=str),
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
from launch_ros.parameter_descriptions import ParameterValue

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

    # 스폰 위치 — 기본값은 게이트 미션 시작점. 다른 미션을 검증할 때는
    # launch 파일을 복사하지 말고 이 인자를 넘긴다
    # (예: 회피 부표 밭 진입 = spawn_y:=75.0, scripts/demo/sim_avoid.sh).
    spawn_args = [
        DeclareLaunchArgument('spawn_x', default_value='2.0',
                              description='배 스폰 x [m]'),
        DeclareLaunchArgument('spawn_y', default_value='63.0',
                              description='배 스폰 y [m]'),
        DeclareLaunchArgument('spawn_yaw', default_value='0.0',
                              description='배 스폰 선수각 [rad]'),
    ]
    
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
            'robot_description': ParameterValue(Command(['xacro ', xacro_file_path]), value_type=str),
            'use_sim_time': True,
        }]
    )

    

    # 6. 로봇 스폰 — kaboat_2026_comprehensive_mission 경기장 하단 채널 왼쪽 끝.
    #    경기장은 world (0, 60) 오프셋에 배치되어 있고, 하단 채널은 빨강(y=0.5)/
    #    초록(y=5.5) 부표 5쌍(x/y 각 5m 간격)이 x=5~25 구간에 늘어선 구조 —
    #    항로추종은 이 두 열 사이(중심선 y=3.0)를 왼쪽→오른쪽(+x)으로 직진
    #    통과하는 미션이다 (경기장 도면 기준 왼쪽 아래에서 시작, 팀 확인).
    #    채널 왼쪽 끝(x=5 마커) 3m 앞(x=2)에서, 채널을 정면으로(+x, yaw=0°)
    #    바라보고 스폰한다.
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-world', 'kaboat_world',
            '-topic', 'robot_description',
            '-name', 'wamv_kaboat',
            '-x', LaunchConfiguration('spawn_x'),
            '-y', LaunchConfiguration('spawn_y'),
            '-z', '0.2',
            '-Y', LaunchConfiguration('spawn_yaw'),
        ],
        output='screen'
    )

    # 7. ROS-Gazebo Bridge 설정
    # 인라인 @ 문법은 gz 토픽명 == ros 토픽명 이라 표준 인터페이스로 이름을
    # 정규화할 수 없다. config 파일은 ros_topic_name != gz_topic_name 을 지원하므로
    # sensor_drivers 표준 인터페이스(/scan, /camera/color/image_raw,
    # /camera/depth/points, /imu/data, /gps/fix, /odom)로 리매핑하는 계약을
    # config/bridge_config.yaml 에 둔다.
    bridge_config_path = os.path.join(kaboat_sim_share, 'config', 'bridge_config.yaml')
    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{'config_file': bridge_config_path}],
        output='screen'
    )

    rviz_interactive_marker = Node(
        package='kaboat_sim',
        executable='rviz_interactive_marker.py',
        name='rviz_interactive_marker',
        output='screen'
    )

    # 8. 위치추정 노이즈 모델 — ground truth(/odom_ground_truth)에 GPS 수준
    #    위치오차(기본 ~50cm, OU 상관잡음)를 얹어 /odom 으로 재발행하고
    #    odom→base_link TF 도 같은 위치로 발행한다(odom_gps_noise.py).
    #    실물의 GNSS+IMU→EKF 파이프라인은 실물 전환 시 별도 launch 로 붙인다
    #    — sim 에서 EKF 를 튜닝해봐야 실 센서 노이즈에 묶인 튜닝이 실물로
    #    이월되지 않으므로, sim 은 "결과 오차"만 직접 모델링한다.
    #    (그래서 bridge_config.yaml 은 ground truth 를 /odom_ground_truth 로
    #    두고 /odom·TF 는 이 노드가 만든다.)
    # odom 위치노이즈 세기(sigma[m])를 런치 인자로 노출 — 기본은 노드 기본값과
    # 동일한 0.25(GPS 수준). 회피 데모처럼 노이즈 없이 순수 알고리즘만 보고
    # 싶을 때 odom_noise_sigma:=0.0 을 주면 /odom 이 ground truth 와 같아진다.
    # (TF(odom→base_link)도 이 노드가 같이 내므로 RViz 의 배·스캔 표시까지
    #  노이즈가 사라진다 — 단순 토픽 리매핑으로는 격자만 깨끗해지고 TF 는 튄다.)
    odom_noise_arg = DeclareLaunchArgument(
        'odom_noise_sigma', default_value='0.25',
        description='odom 위치오차 정상 stddev [m]. 0 이면 ground truth 와 동일')

    odom_gps_noise = Node(
        package='kaboat_sim',
        executable='odom_gps_noise.py',
        name='odom_gps_noise',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'sigma': ParameterValue(LaunchConfiguration('odom_noise_sigma'),
                                    value_type=float),
        }],
    )

    return LaunchDescription([
        headless_arg,
        *spawn_args,
        odom_noise_arg,
        set_plugin_path,
        set_resource_path,
        set_ld_library,
        gazebo_launch,
        robot_state_publisher,
        spawn_robot,
        ros_gz_bridge,
        rviz_interactive_marker,
        odom_gps_noise,
    ])
