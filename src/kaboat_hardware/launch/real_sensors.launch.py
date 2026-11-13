"""모터 출력을 전혀 실행하지 않는 실물 센서 배선 확인 launch."""

import os
import tempfile

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

# RTK(rtk:=true) 때 GQ7 설정에 NTRIP 입력을 켠 사본 (임시 디렉터리에 매번 덮어씀)
RTK_PARAMS_FILE = 'kaboat_gq7_rtk.yaml'


def gq7_params_with_ntrip(base_path: str, aux_port: str, directory: str | None = None) -> str:
    """GQ7 설정에 NTRIP(RTCM) 입력을 켠 사본을 쓰고 그 경로를 돌려준다.

    GQ7 은 RTCM 보정을 AUX 포트로만 받는다. USB 하나로 연결해도 main·aux 두 포트가 잡히고
    드라이버 udev 규칙이 /dev/microstrain_aux 를 만든다. 드라이버는 ntrip_interface_enable 이면
    /rtcm(rtcm_msgs/Message) 를 AUX 로 넣고, AUX 의 NMEA 를 /nmea 로 낸다.
    """
    import yaml
    with open(os.path.expanduser(base_path), encoding='utf-8') as f:
        data = yaml.safe_load(f) or {}
    found = False
    for section in data.values():
        params = (section or {}).get('ros__parameters')
        if isinstance(params, dict):
            params.update({'ntrip_interface_enable': True, 'aux_port': aux_port})
            found = True
    if not found:
        raise ValueError(f'{base_path}: ros__parameters 가 없다')
    path = os.path.join(directory or tempfile.gettempdir(), RTK_PARAMS_FILE)
    tmp = path + '.tmp'
    with open(tmp, 'w', encoding='utf-8') as f:
        yaml.safe_dump(data, f, allow_unicode=True, sort_keys=False)
    os.replace(tmp, path)
    return path


def check_aux_port(path: str, proc_root: str = '/proc') -> str | None:
    """RTK 를 켜기 전 GQ7 AUX 포트 확인 — 문제가 있으면 이유·해결 문장, 없으면 None.

    AUX 가 없거나 다른 프로세스(남은 GQ7 드라이버 등)가 잡고 있으면 드라이버가 RTCM 을 못 써
    'Failed to write RTCM to aux port' 를 초당 ~10줄 찍고 RTK 가 SBAS 로 떨어진다
    (2026-11-13 토너먼트 시험 내내). 켤 때 바로 알린다.
    """
    if not os.path.exists(path):
        return (f'GQ7 AUX 포트 {path} 가 없다 — GQ7 USB 를 다시 꽂고 ls -l /dev/microstrain_* 로 '
                f'main·aux 둘 다 보이는지 확인. RTK 없이 쓰려면 run_gq7_outdoor')
    real = os.path.realpath(path)
    users = []
    try:
        pids = [d for d in os.listdir(proc_root) if d.isdigit() and int(d) != os.getpid()]
    except OSError:
        pids = []
    for pid in pids:
        fd_dir = os.path.join(proc_root, pid, 'fd')
        try:
            fds = os.listdir(fd_dir)
        except OSError:
            continue
        for fd in fds:
            try:
                if os.path.realpath(os.path.join(fd_dir, fd)) == real:
                    with open(os.path.join(proc_root, pid, 'cmdline'), 'rb') as f:
                        cmd = f.read().replace(b'\0', b' ').decode(errors='replace').strip()
                    users.append(f'{pid} {cmd[:80]}')
                    break
            except OSError:
                continue
    if users:
        return (f'GQ7 AUX 포트 {path}({real}) 를 이미 다른 프로세스가 쓰고 있다: {"; ".join(users)} — '
                f'남은 GQ7 드라이버·센서 런치를 끄고 다시 켤 것 (kill <PID>)')
    return None


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
    rtk = False
    actions = []
    if outdoor:
        config_dir = os.path.join(get_package_share_directory('kaboat_hardware'), 'config')
        if os.path.basename(params_file) == 'gq7.yaml':
            params_file = os.path.join(config_dir, 'gq7_outdoor.yaml')
        origin_file = (context.perform_substitution(LaunchConfiguration('gq7_origin_file'))
                       or DEFAULT_ORIGIN_FILE)
        source = context.perform_substitution(LaunchConfiguration('odom_position_source'))
        actions += [
            LogInfo(msg=(f'[GQ7] 야외: /odom = gq7_local_odom (위치 출처 {source}, 원점 '
                         f'{origin_file}). 원점 파일이 바뀌면 GQ7 재시작 없이 바로 반영')),
            Node(
                package='kaboat_hardware',
                executable='gq7_local_odom',
                name='gq7_local_odom',
                output='screen',
                parameters=[{'origin_file': origin_file, 'position_source': source,
                             'use_sim_time': False}],
            ),
        ]
        if _truthy(context, 'rtk'):
            aux_port = context.perform_substitution(LaunchConfiguration('gq7_aux_port'))
            if _truthy(context, 'gq7_aux_check'):
                problem = check_aux_port(aux_port)
                if problem:
                    raise RuntimeError(f'[GQ7] RTK 를 못 켬 — {problem}')
            actions += _rtk_actions(context)
            params_file = gq7_params_with_ntrip(params_file, aux_port)
            rtk = True
    remap_odom = not outdoor and _truthy(context, 'enable_odom_remap')

    # 공식 MicroStrain launch를 사용하되, 나머지 자율주행 코드가 기대하는
    # KABOAT 표준 토픽으로 이름만 변환한다.
    remaps = [
        SetRemap(src='/gnss_1/llh_position', dst='/gps/fix'),
        SetRemap(src='/gnss_2/llh_position', dst='/gps/fix_secondary'),
    ]
    if rtk:
        # 드라이버는 NTRIP 보정을 rtcm_guard 를 거쳐 받는다 (/rtcm → /rtcm/gq7, AUX 쓰기 실패 감시)
        remaps.append(SetRemap(src='/rtcm', dst='/rtcm/gq7'))
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


def _ntrip_launch_file() -> str:
    """ntrip_client 패키지의 런치 파일 경로.

    ROS 2 Humble apt 패키지(1.4.1)는 share/ntrip_client/ 바로 아래에, 소스(ros2 브랜치)
    빌드는 share/ntrip_client/launch/ 아래에 설치한다 — 둘 다 찾는다.
    """
    share = get_package_share_directory('ntrip_client')
    for path in (os.path.join(share, 'ntrip_client_launch.py'),
                 os.path.join(share, 'launch', 'ntrip_client_launch.py')):
        if os.path.isfile(path):
            return path
    raise FileNotFoundError(f'ntrip_client_launch.py 를 {share} 에서 못 찾음')


def _rtk_actions(context):
    """NTRIP 클라이언트(ntrip_client) — 인터넷의 RTK 보정(RTCM)을 /rtcm 으로 낸다.

    기본 캐스터는 해양수산부 국립해양측위정보원 NTRIP(ntrip.ndgps.go.kr:2101). 계정은
    서비스에서 발급받은 것을 ntrip_username / ntrip_password 로 넘긴다(레포에 넣지 않음).
    기준국은 대회장(창원, 35.18N 128.57E)에서 가장 가까운 RTCM 3 RTK 기준국
    SEOI_RTCM32(약 47 km, 공개 소스 테이블 기준). 단일 기준국 RTK 는 거리가 멀수록
    Fixed 가 늦거나 Float 에 머물 수 있다.
    """
    g = lambda name: context.perform_substitution(LaunchConfiguration(name))  # noqa: E731
    host, mount = g('ntrip_host'), g('ntrip_mountpoint')
    if not g('ntrip_username') or not g('ntrip_password'):
        raise RuntimeError(
            'rtk:=true 에는 NTRIP 계정이 필요하다 — '
            'ntrip_username:=<아이디> ntrip_password:=<비밀번호> 를 같이 넘긴다')
    return [
        LogInfo(msg=f'[GQ7] RTK: NTRIP {host}:{g("ntrip_port")}/{mount} → /rtcm → rtcm_guard → '
                    f'/rtcm/gq7 → GQ7 AUX ({g("gq7_aux_port")})'),
        # AUX 쓰기 실패('Failed to write RTCM to aux port' 폭주)·RTK 상태 감시, /gnss/rtk_status
        Node(package='kaboat_hardware', executable='rtcm_guard', name='rtcm_guard',
             output='screen', parameters=[{'use_sim_time': False}]),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(_ntrip_launch_file()),
            launch_arguments={
                'host': host,
                'port': g('ntrip_port'),
                'mountpoint': mount,
                'authenticate': 'True',
                'username': g('ntrip_username'),
                'password': g('ntrip_password'),
                'rtcm_message_package': 'rtcm_msgs',   # GQ7 드라이버가 받는 형식
            }.items(),
        ),
    ]


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
            'odom_position_source', default_value='filter',
            description="야외 /odom 위치 출처: filter(GQ7 필터 융합 위치 25 Hz, 기본 — RTK 와 함께 쓰면 "
                        "cm 수준) 또는 gnss(GNSS1 원시 + GQ7 선수각, 2 Hz — 필터가 흔들릴 때 대안)"),
        DeclareLaunchArgument(
            'rtk', default_value='false',
            description='true: 야외에서 NTRIP RTK 보정 받기 (ntrip_client 실행 + GQ7 AUX 로 RTCM). '
                        '젯슨 인터넷 필요, sudo apt install ros-humble-ntrip-client'),
        DeclareLaunchArgument('ntrip_host', default_value='ntrip.ndgps.go.kr',
                              description='NTRIP 캐스터 (국립해양측위정보원)'),
        DeclareLaunchArgument('ntrip_port', default_value='2101'),
        DeclareLaunchArgument('ntrip_mountpoint', default_value='SEOI_RTCM32',
                              description='기준국. 대회장 근처 RTCM3: SEOI_RTCM32(47 km), '
                                          'YNDO_RTCM32(48 km)'),
        DeclareLaunchArgument('ntrip_username', default_value='',
                              description='NTRIP 계정 아이디 (서비스에서 발급, rtk:=true 때 필수)'),
        DeclareLaunchArgument('ntrip_password', default_value='',
                              description='NTRIP 계정 비밀번호 (rtk:=true 때 필수)'),
        DeclareLaunchArgument('gq7_aux_port', default_value='/dev/microstrain_aux',
                              description='GQ7 AUX 포트 (RTCM 입력)'),
        DeclareLaunchArgument('gq7_aux_check', default_value='true',
                              description='rtk:=true 때 AUX 포트가 있고 다른 프로세스가 안 잡고 있는지 '
                                          '켤 때 확인 (false = 확인 안 함)'),
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
