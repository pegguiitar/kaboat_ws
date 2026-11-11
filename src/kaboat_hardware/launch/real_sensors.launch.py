"""모터 출력을 전혀 실행하지 않는 실물 센서 배선 확인 launch."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetRemap
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


# geo_origin 의 기본 원점 파일 (config/geo_origin.yaml 의 origin_file 과 같게)
DEFAULT_ORIGIN_FILE = '~/kaboat_geo_origin.yaml'


def generate_launch_description():
    hardware_share = get_package_share_directory('kaboat_hardware')
    bringup_share = get_package_share_directory('kaboat_bringup')
    default_config = os.path.join(hardware_share, 'config', 'sensors.yaml')
    default_gq7_config = os.path.join(hardware_share, 'config', 'gq7.yaml')

    enable_d455 = LaunchConfiguration('enable_d455')
    config_file = LaunchConfiguration('config_file')

    d455 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, 'launch', 'd455.launch.py')),
        launch_arguments={'use_sim_time': 'false'}.items(),
        condition=IfCondition(enable_d455),
    )

    # GQ7 은 원점 파일을 읽어 설정을 합쳐야 해서 실행 시점에 만든다
    gq7 = OpaqueFunction(function=_gq7_actions)

    return _launch(d455, gq7, enable_d455, config_file, default_config,
                   default_gq7_config)


def _truthy(context, name):
    return context.perform_substitution(
        LaunchConfiguration(name)).strip().lower() in ('true', '1', 'yes')


def _gq7_actions(context):
    """GQ7 드라이버 실행.

    야외(outdoor:=true): gq7_outdoor.yaml 로 켜고, /odom 은 gq7_local_odom 이 GQ7 필터의
    /ekf/odometry_earth(ECEF)를 원점 파일 기준 x=동·y=북으로 바꿔 낸다. 드라이버의 고정 원점
    기능(/ekf/odometry_map)은 쓰지 않는다 — 젯슨 드라이버에서 원점을 맞게 받고도 수백 km
    떨어진 값을 냈다(2026-11-11). 원점 파일이 바뀌면 gq7_local_odom 이 바로 다시 읽는다.
    실내·벤치(outdoor:=false): 예전처럼 /ekf/odometry_map → /odom remap(enable_odom_remap).
    """
    if not _truthy(context, 'enable_gq7'):
        return []
    params_file = context.perform_substitution(LaunchConfiguration('gq7_config_file'))
    outdoor = _truthy(context, 'outdoor')
    actions = []
    if outdoor:
        config_dir = os.path.join(get_package_share_directory('kaboat_hardware'), 'config')
        if os.path.basename(params_file) == 'gq7.yaml':
            params_file = os.path.join(config_dir, 'gq7_outdoor.yaml')
        origin_file = (context.perform_substitution(LaunchConfiguration('gq7_origin_file'))
                       or DEFAULT_ORIGIN_FILE)
        actions += [
            LogInfo(msg=(f'[GQ7] 야외: /odom = gq7_local_odom (/ekf/odometry_earth → 원점 '
                         f'{origin_file} 기준). 원점 파일이 바뀌면 GQ7 재시작 없이 바로 반영')),
            Node(
                package='kaboat_hardware',
                executable='gq7_local_odom',
                name='gq7_local_odom',
                output='screen',
                parameters=[{'origin_file': origin_file, 'use_sim_time': False}],
            ),
        ]
    remap_odom = not outdoor and _truthy(context, 'enable_odom_remap')

    # 공식 MicroStrain launch를 사용하되, 나머지 자율주행 코드가 기대하는
    # KABOAT 표준 토픽으로 이름만 변환한다.
    remaps = [
        SetRemap(src='/gnss_1/llh_position', dst='/gps/fix'),
        SetRemap(src='/gnss_2/llh_position', dst='/gps/fix_secondary'),
    ]
    if remap_odom:
        # ⚠️ 실내 수조에서는 indoor_lidar_odom 이, 야외에서는 gq7_local_odom 이 /odom 을
        # 발행하므로 이 remap 을 쓰지 않는다. 켠 채로 두면 /odom 발행자가 둘이 되어
        # 두 좌표계가 섞인다 (GNSS 미수렴 중에는 조용해서 안 드러남).
        remaps.append(SetRemap(src='/ekf/odometry_map', dst='/odom'))
    return actions + [GroupAction(
        actions=remaps + [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([
                        FindPackageShare('microstrain_inertial_driver'),
                        'launch',
                        'microstrain_launch.py',
                    ])),
                launch_arguments={
                    'namespace': '/',
                    'node_name': 'microstrain_inertial_driver',
                    'params_file': params_file,
                }.items(),
            ),
        ],
    )]


def _launch(d455, gq7, enable_d455, config_file, default_config, default_gq7_config):
    return LaunchDescription([
        DeclareLaunchArgument(
            'enable_d455', default_value='false',
            description='realsense2_camera 설치 및 D455 연결 후 true'),
        DeclareLaunchArgument(
            'enable_gq7', default_value='true',
            description='GQ7 공식 드라이버와 KABOAT 토픽 remap 실행'),
        DeclareLaunchArgument(
            'config_file', default_value=default_config,
            description='센서 토픽/최소 주기/timeout 설정'),
        DeclareLaunchArgument(
            'enable_scan_monitor', default_value='true',
            description='선체 /scan 진단 활성화 여부'),
        DeclareLaunchArgument(
            'enable_camera_monitor', default_value=enable_d455,
            description='D455 카메라 토픽 진단 활성화 여부'),
        DeclareLaunchArgument(
            'enable_gps_monitor', default_value='true',
            description='GNSS /gps/fix 진단 활성화 여부'),
        DeclareLaunchArgument(
            'gq7_config_file', default_value=default_gq7_config,
            description='GQ7 드라이버 override 설정 (야외: config/gq7_outdoor.yaml)'),
        DeclareLaunchArgument(
            'outdoor', default_value='false',
            description='true: 야외 — gq7_outdoor.yaml + gq7_local_odom 이 원점 '
                        '(~/kaboat_geo_origin.yaml) 기준 /odom 발행'),
        DeclareLaunchArgument(
            'gq7_origin_file', default_value='',
            description="야외 /odom 원점 파일 (geo_origin 이 저장). '' 이면 ~/kaboat_geo_origin.yaml"),
        DeclareLaunchArgument(
            'enable_odom_remap', default_value='true',
            description='GQ7 /ekf/odometry_map → /odom remap (outdoor:=false 일 때만). '
                        '실내 수조(indoor_lidar_odom)에서는 false'),
        DeclareLaunchArgument(
            'publish_tf', default_value='true',
            description='RViz Fixed Frame 용 최소 TF. 스택 자체는 TF 를 쓰지 않는다'),
        LogInfo(
            msg='D455 driver disabled. 연결 후 enable_d455:=true 로 실행하세요.',
            condition=UnlessCondition(enable_d455)),
        LogInfo(
            msg='GQ7 driver disabled. enable_gq7:=true 로 실행하세요.',
            condition=UnlessCondition(LaunchConfiguration('enable_gq7'))),
        d455,
        gq7,
        Node(
            package='kaboat_hardware',
            executable='sensor_health_monitor',
            name='sensor_health_monitor',
            output='screen',
            parameters=[
                config_file,
                {
                    'use_sim_time': False,
                    'scan.enabled': ParameterValue(
                        LaunchConfiguration('enable_scan_monitor'), value_type=bool),
                    'color.enabled': ParameterValue(
                        LaunchConfiguration('enable_camera_monitor'), value_type=bool),
                    'depth.enabled': ParameterValue(
                        LaunchConfiguration('enable_camera_monitor'), value_type=bool),
                    'camera_info.enabled': ParameterValue(
                        LaunchConfiguration('enable_camera_monitor'), value_type=bool),
                    'gps.enabled': ParameterValue(
                        LaunchConfiguration('enable_gps_monitor'), value_type=bool),
                },
            ],
        ),
        # RViz 가 Fixed Frame 을 해석하려면 TF 트리가 있어야 한다. 실물에는
        # 발행자가 없어(gq7.yaml 의 tf_mode: 0) 여기서 최소 체인만 채운다 —
        # 제어 경로에는 관여하지 않는다.
        Node(
            package='kaboat_hardware',
            executable='odom_tf_broadcaster',
            name='odom_tf_broadcaster',
            output='screen',
            condition=IfCondition(LaunchConfiguration('publish_tf')),
            parameters=[{'use_sim_time': False}],
        ),
    ])
