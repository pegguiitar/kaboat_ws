# KABOAT ROS 2 통신 구조 및 원격 토픽 전달 — 보고서 작성 자료

> 기준: 2026-11-03, `kaboat_ws`의 `real` 브랜치 **현재 작업 트리**. 이 문서는 커밋된 `real` 코드뿐 아니라 로컬의 미커밋 Zenoh·키보드 조종·시각화 변경도 설명한다. `jiwoo` 브랜치에 이 Markdown 파일만 커밋하면 해당 코드 변경은 포함되지 않으므로, 문서의 현재 구현 목록을 원격 `jiwoo` 브랜치에 모두 구현된 기능으로 해석하지 않는다. 실제 배포본을 설명할 때는 해당 기기에 복사·빌드된 버전을 다시 확인해야 한다. 사용자가 적은 “taliscale”의 정식 명칭은 **Tailscale**이다.

## 1. 보고서용 핵심 요약

KABOAT는 ROS 2의 발행자·구독자(pub/sub) 구조로 센서, 위치 추정, 경로 계획, 미션 제어, 추진기 구동을 분리했다. 센서 드라이버가 `LaserScan`, `Imu`, `NavSatFix` 등을 발행하면 위치 추정 노드가 `Odometry`를 만들고, 제어 노드가 `Twist` 명령을 만들어 추진기 드라이버에 전달한다. 실내 수조에서는 외벽 노트북의 라이다가 `/boat_pose`를 계산해 선체 Jetson으로 보내고, Jetson의 `indoor_lidar_odom`이 `/odom`을 발행한다. 현재 기본 설정은 라이다 yaw를 직접 쓰고, 선택 설정에서 `/imu/data`의 gyro를 yaw 방향 선택에 사용한다. 실외에서는 GQ7의 GNSS/INS 결과를 `/odom`에 연결한다. 원격 모니터링은 두 장치의 ROS 도메인을 분리한 뒤, **Tailscale 네트워크 위에서 두 `zenoh-bridge-ros2dds` 프로세스를 TCP로 연결**하여 허용한 토픽만 양방향으로 중계하도록 구현했다.

이 설명은 **현재 소스 코드와 설정의 구현 내용**이다. 두 기기를 서로 다른 핫스팟에 연결한 현장 통신 성공을 이 자료의 작성 과정에서 확인한 것은 아니다.

### 1.1 장치별 역할과 통신 경계

| 위치 | 실행 환경과 ROS 도메인 | 주요 노드·장치 | 맡은 일 |
|---|---|---|---|
| 수조 외벽 노트북의 라이다 컨테이너 | ROS 2 Humble, `ROS_DOMAIN_ID=43`, Docker `network_mode: host` | 고정 RPLIDAR A3, `sllidar_node`, `lidar_boat_tracker`, 수조 RViz | `/shore/scan`으로 선체 표적을 측정하고 `/boat_pose` 발행 |
| 노트북 호스트 OS | ROS 2 Jazzy, `ROS_DOMAIN_ID=43` | Tailscale, `zenoh-bridge-ros2dds`, RViz, 선택적으로 `teleop_laptop.py` | Jetson 데이터 표시, `/boat_pose` 및 명시적인 키보드 조종 토픽 전송 |
| 선체 Jetson | ROS 2 Humble, `ROS_DOMAIN_ID=42` | GQ7, 선택적 선체 TG-50·D455, 위치 추정·미션·추진기 노드, Tailscale·Zenoh 브리지 | `/odom` 생성, 인식·제어 실행, 모터 출력 결정 |

**ROS 도메인**은 각 장치의 로컬 DDS 통신 범위이고, **Tailscale**은 두 호스트가 서로 접근할 IP 경로를 제공하며, **Zenoh ROS 2 DDS 브리지**는 허용한 ROS 토픽을 두 도메인 사이에서 중계한다. Docker의 호스트 네트워크 설정은 외벽 라이다 노드와 노트북 브리지가 같은 도메인 43의 토픽을 발견하기 위한 구성이다. 브리지 스크립트는 Jetson 프로세스에 `ROS_DISTRO=humble`, 노트북 호스트 프로세스에 `ROS_DISTRO=jazzy`를 지정한다. 이 값은 브리지가 각 호스트의 ROS 배포판과 맞도록 실행 환경을 설정하기 위한 것이다. [Docker Compose](docker-compose.lidar.yml), [브리지 스크립트](scripts/zenoh_monitor_link.py)

## 2. ROS 2 통신을 구현한 방법

1. **노드와 메시지 타입을 분리했다.** Python `rclpy` 노드에서 `create_publisher()`와 `create_subscription()`으로 토픽을 연결한다. 예를 들어 `lidar_boat_tracker`는 `LaserScan`을 구독하고 `PoseWithCovarianceStamped`인 `/boat_pose`를 발행한다. `indoor_lidar_odom`은 `/boat_pose`와 `Imu`를 구독하고 `Odometry`인 `/odom` 및 `odom → base_link` TF를 발행한다. 기본 설정에서 IMU는 yaw 결정에 개입하지 않는다. [배 위치 추적기](src/kaboat_hardware/kaboat_hardware/lidar_boat_tracker.py), [실내 오도메트리](src/kaboat_hardware/kaboat_hardware/indoor_lidar_odom.py)
2. **Launch와 파라미터로 실행 구성을 바꾼다.** 실내에서는 외벽 RPLIDAR A3의 스캔을 `/shore/scan`으로 이름을 바꾸고, 추적 노드의 입력도 그 토픽에 연결한다. 실외 GQ7 드라이버의 `/gnss_1/llh_position`은 `/gps/fix`로, `/ekf/odometry_map`은 필요할 때 `/odom`으로 remap한다. 실내 Launch는 GQ7의 `/odom` remap을 꺼서 `indoor_lidar_odom`이 `/odom`을 담당하게 한다. 토픽 **이름 remap은 좌표계 변환이 아니므로** `frame_id`, 원점, 축 방향은 별도로 맞춰야 한다. [외벽 라이다 Launch](src/kaboat_hardware/launch/lidar_boat_tracker.launch.py), [GQ7 Launch](src/kaboat_hardware/launch/real_sensors.launch.py), [실내 Launch](src/kaboat_hardware/launch/indoor_tank.launch.py)
3. **QoS를 센서 데이터에 맞춘다.** 라이다·IMU·카메라 입력에는 `qos_profile_sensor_data`를 쓰는 구독자가 있다. 센서 발행자가 Best Effort인데 구독자가 Reliable만 요구하면 토픽 이름이 보여도 메시지가 수신되지 않을 수 있다. 미션 상태와 목표는 `TRANSIENT_LOCAL`로 발행·구독하여 늦게 실행한 행동 노드도 최신 상태를 받는다. [부표 검출기](src/kaboat_perception/kaboat_perception/buoy_detector.py), [미션 관리자](src/kaboat_mission/kaboat_mission/mission_manager.py), [행동 공통부](src/kaboat_behaviors/kaboat_behaviors/behavior_base.py)
4. **구동 명령은 Jetson 안에서 결정한다.** 종합 미션의 행동 노드는 `/cmd/approach`, `/cmd/gate`, `/cmd/station`, `/cmd/dock`, `/cmd/search`, `/cmd/avoid` 중 해당 명령을 발행한다. `cmd_mux`가 `/mission/state`에 맞는 한 토픽만 `/cmd_vel`로 전달하고, `thruster_driver`가 이를 좌우 추진기 출력으로 변환한다. 단독 수조 시험 노드는 선택한 **한 노드가 `/cmd_vel`을 직접 발행**한다. 두 제어 경로를 동시에 실행하지 않는다. [명령 중재기](src/kaboat_control/kaboat_control/cmd_mux.py), [추진기 드라이버](src/kaboat_hardware/kaboat_hardware/thruster_driver.py), [자율운항 Launch](src/kaboat_bringup/launch/autonomy.launch.py)

### 2.1 ROS 2 토픽이 실제로 연결되는 순서

`ros2 launch`는 필요한 노드를 실행하고 YAML 파라미터와 remap 규칙을 전달한다. 노드는 시작하면서 메시지 **타입**, 토픽 **이름**, QoS를 지정해 발행자 또는 구독자를 만든다. 발행자가 메시지를 `publish()`하면 같은 도메인의 호환 구독자가 콜백에서 값을 처리한다. 예를 들어 수조 위치 경로는 다음과 같다.

1. `sllidar_node`가 USB 시리얼로 받은 스캔을 `sensor_msgs/msg/LaserScan`으로 만든다. Launch remap에 따라 최종 토픽은 `/shore/scan`이다.
2. `lidar_boat_tracker`는 그 스캔을 구독하고 수조 좌표계에서 선체 표적의 `x`, `y`, `yaw`와 공분산을 계산한다. 결과를 `geometry_msgs/msg/PoseWithCovarianceStamped`인 `/boat_pose`에 발행한다.
3. 노트북 Zenoh 브리지가 도메인 43의 `/boat_pose`를 읽어 Jetson 브리지로 전달한다. Jetson 브리지는 이를 도메인 42에서 다시 ROS 토픽으로 보이게 한다.
4. `indoor_lidar_odom`은 Jetson에서 `/boat_pose`를 구독하고 `nav_msgs/msg/Odometry`인 `/odom`을 발행한다. 기본 설정은 라이다가 계산한 yaw를 그대로 사용한다. 설정 파일의 `use_lidar_yaw_truth: false`를 선택한 경우에는 IMU gyro를 180° 방향 분기 선택에 쓴다.
5. Jetson의 제어 노드가 `/odom`을 구독한다. 같은 `/odom`을 Jetson Zenoh 브리지가 노트북 도메인 43으로 전달하면 RViz에서 위치와 경로를 표시한다.

메시지의 `header.stamp`는 측정 또는 발행 시각, `header.frame_id`는 좌표 기준을 담는다. 네트워크에서 토픽 이름과 메시지를 전달하는 것만으로 `map`, `odom`, `base_link`, `shore_laser_frame` 사이의 실제 좌표 변환이 생기지는 않는다. 실내 `/boat_pose`는 수조 절대좌표를 `odom` 프레임으로 사용하고, Jetson `/odom`의 `child_frame_id`는 `base_link`로 설정된다. 외벽 라이다 설치 좌표와 방향, 선체 표적의 기준점을 현장 실측으로 맞춰야 위치값이 물리적 의미를 가진다. [라이다 설정](src/kaboat_hardware/config/lidar_tracker.yaml), [실내 설정](src/kaboat_hardware/config/indoor_tank.yaml), [오도메트리 메시지 구성](src/kaboat_hardware/kaboat_hardware/indoor_lidar_odom.py)

### 2.2 메시지 타입과 QoS가 중요한 이유

같은 토픽 이름이 보여도 발행자와 구독자의 메시지 타입이 다르면 통신할 수 없다. `/boat_pose`는 `PoseWithCovarianceStamped`, `/odom`은 `Odometry`라서 서로 이름만 바꿔 대체할 수 없다. `/odom`에는 위치·자세 외에 선체 기준의 속도(`twist`)와 부모·자식 프레임이 들어간다. 반면 `/cmd_vel`과 `/teleop/cmd_vel`은 둘 다 `Twist`지만 **서로 다른 입력 경로**로 취급된다.

라이다·IMU·카메라 등 고주기 센서는 코드에서 `qos_profile_sensor_data`를 사용하는 구독자가 있다. 이 프로파일은 센서 데이터에 맞춘 Best Effort 계열이며, 기본 Reliable 구독 설정과 맞지 않으면 발견만 되고 실제 콜백은 실행되지 않을 수 있다. `ros2 topic info <토픽> --verbose`에서 발행자·구독자 QoS를 확인하고, `ros2 topic hz` 또는 `echo`로 **실제 메시지 수신**을 확인해야 한다. 미션의 `/mission/state`, `/mission/goal`은 `TRANSIENT_LOCAL` 내구성을 써서 늦게 시작한 행동 노드도 최신 값을 받을 수 있도록 했다. [부표 검출기 QoS](src/kaboat_perception/kaboat_perception/buoy_detector.py), [미션 관리자](src/kaboat_mission/kaboat_mission/mission_manager.py)

## 3. 토픽 흐름

### 3.1 실내 수조: 외벽 위치 측정 → 선체 제어 → 노트북 모니터링

```text
외벽 노트북 (ROS_DOMAIN_ID=43, 라이다 Docker는 host 네트워크)
RPLIDAR A3 ─USB→ sllidar_ros2 ─/shore/scan→ lidar_boat_tracker
                                      └─/boat_pose (위치·자세·공분산)
                                               │
                              Zenoh over Tailscale, 노트북 → Jetson
                                               ▼
선체 Jetson (ROS_DOMAIN_ID=42)
GQ7 ─/imu/data──────────────────────→ indoor_lidar_odom (선택 yaw 분기 모드에서 사용)
/boat_pose──────────────────────────→ indoor_lidar_odom ─/odom, /tf→ 제어 노드
                                                             │
       단독 시험 노드 ─/cmd_vel──────────────────────────────┤
       또는 미션 행동 ─/cmd/*→ cmd_mux ─/cmd_vel────────────┘
                                                             ▼
                                               thruster_driver ─시리얼/PWM→ 추진기
Jetson의 /odom·센서·경로·상태 ─Zenoh over Tailscale→ 노트북 RViz
```

외벽 라이다의 원본 `/shore/scan`은 노트북 로컬 데이터다. 브리지 허용 목록에는 `/shore/scan`이 없고, **계산된 `/boat_pose`만** Jetson으로 넘긴다. 현 설정에서는 외벽 추적기의 `/odom`·TF 발행을 끄고, Jetson의 `indoor_lidar_odom`이 `/odom`과 `odom → base_link` TF를 만든다. `indoor_tank.yaml` 기본값은 라이다 yaw를 직접 사용하며, IMU gyro와 bias를 쓰는 대체 모드는 파라미터로 선택한다. `/boat_pose`가 1초 넘게 끊기면 `/odom` 발행을 멈춘다. [라이다 설정](src/kaboat_hardware/config/lidar_tracker.yaml), [실내 오도메트리 설정](src/kaboat_hardware/config/indoor_tank.yaml), [오도메트리 코드](src/kaboat_hardware/kaboat_hardware/indoor_lidar_odom.py)

### 3.2 실외: 선체 센서 → 자율운항 → 원격 관찰

```text
선체 GQ7 GNSS/INS ─/gps/fix, /imu/data, /ekf/odometry_map
                               └─ remap /ekf/odometry_map → /odom (실외 구성)
선체 TG-50 ─/scan/raw→ onboard_scan_filter ─/scan→ 인식·점유 격자·회피
카메라(D455, 활성화 시) ─영상/깊이→ 부표·도킹 마커 검출
/odom + 검출 결과 + /occupancy_grid → mission_manager·behavior
→ /cmd/* → cmd_mux → /cmd_vel → thruster_driver → 추진기
관찰 허용 토픽 → Zenoh/Tailscale → 노트북 RViz·상태 확인
```

실외 `/odom`은 GQ7의 EKF 출력 이름을 연결한 것이며, 실제 `map`/`odom` 프레임·원점과 제어 노드의 기대 좌표계를 확인해야 한다. 선체 TG-50이나 D455를 실행하지 않으면 해당 센서 토픽도 나오지 않는다. [GQ7 Launch](src/kaboat_hardware/launch/real_sensors.launch.py), [선체 라이다 Launch](src/kaboat_hardware/launch/onboard_lidar.launch.py), [자율운항 Launch](src/kaboat_bringup/launch/autonomy.launch.py)

### 3.3 미션 제어 경로와 단독 시험 경로

**종합 미션 경로**에서 `mission_manager`는 `/odom`으로 현재 위치를 읽고 `/mission/state`와 `/mission/goal`을 발행한다. 행동 노드는 공통 `BehaviorBase`를 통해 위치·검출 결과·점유 격자를 받아 현재 미션에 맞는 `/cmd/<mission>`을 계산한다. `cmd_mux`는 `/mission/state`가 가리키는 명령 토픽만 `/cmd_vel`로 전달하고, 그 명령이 300ms 이상 갱신되지 않으면 0인 `Twist`를 출력한다. `cmd_mux`는 선택·전달·시간 감시를 맡는다. 행동 노드의 명령 상한은 `BehaviorBase`가 적용하고, 추진기 양쪽의 최종 출력 상한은 `thruster_driver`의 믹서가 별도로 적용한다. [미션 관리자](src/kaboat_mission/kaboat_mission/mission_manager.py), [행동 공통부](src/kaboat_behaviors/kaboat_behaviors/behavior_base.py), [명령 중재기](src/kaboat_control/kaboat_control/cmd_mux.py), [추진기 믹서](src/kaboat_hardware/kaboat_hardware/thruster_backend.py)

**단독 수조 시험 경로**는 B-spline 추종, 직선 주행, 원운동, 정점 유지 중 하나의 시험 노드가 `/odom`을 읽고 `/cmd_vel`을 직접 발행한다. 이때 `cmd_mux`를 거치지 않는다. 예를 들어 B-spline 시험은 `/bspline_test/path`와 `/bspline_test/markers`도 만들어 경로를 시각화한다. 단독 시험 노드의 `/odom` 수신 타임아웃과 `thruster_driver`의 `/cmd_vel` 타임아웃은 서로 다른 단계의 감시 장치다. `Twist.linear.x`와 `Twist.angular.z`는 이 프로젝트의 추진 제어에서 정규화된 전후진·회전 명령으로 사용하므로, `/cmd_vel`의 숫자를 곧바로 m/s 또는 rad/s의 실측 속도로 해석하면 안 된다. [B-spline 시험 노드](src/kaboat_hardware/kaboat_hardware/bspline_track_test.py), [추진기 드라이버](src/kaboat_hardware/kaboat_hardware/thruster_driver.py)

`thruster_driver`는 최종 명령을 좌우 추진기 비율로 섞고, 출력 제한과 변화율 제한을 적용한 뒤 선택한 백엔드로 보낸다. 시리얼 백엔드에서는 좌우 PWM 값을 각각 네 자리 정수로 만들어 `LLLLRRRR\n` 형식으로 전송한다. 기본 설정 파일의 백엔드는 `dummy`이며, 실물 시리얼 출력은 해당 하드웨어 타입과 포트를 명시해야 한다. 토픽 발행 또는 백엔드의 쓰기 성공만으로 실제 추진기 회전을 확인했다고 적을 수는 없다. [백엔드 코드](src/kaboat_hardware/kaboat_hardware/thruster_backend.py), [추진기 설정](src/kaboat_hardware/config/thrusters.yaml)

### 3.4 모니터링 화면으로 흐르는 데이터

Jetson의 `/odom`, `/scan`, `/occupancy_grid`, `/avoid/plan`, 시험 경로·마커와 도킹 상태 등은 허용 목록에 있을 때 노트북 도메인으로 들어간다. RViz는 위치·스캔·경로를 메시지의 프레임과 `/tf`에 맞춰 그린다. 따라서 토픽 데이터가 도착해도 필요한 TF 연결이 없으면 스캔이나 경로가 화면에 나오지 않을 수 있다. 선체 라이다만 단독으로 볼 때는 RViz의 고정 프레임을 `onboard_laser_frame`에 맞춰 확인할 수 있다. [원격 모니터링 가이드](REMOTE_MONITORING.md)

수조의 실제 주행 궤적은 노트북 컨테이너의 `tank_test_visualization`이 **노트북 로컬 `/boat_pose`**를 구독해 `/tank_test/actual_path`를 만든다. 목표 경로 `/tank_test/target_path`도 그 컨테이너에서 생성한다. 이 두 토픽은 현재 Zenoh 허용 목록에 없으며, Jetson의 `MarkerArray`가 원격으로 도착해야만 수조 경로를 볼 수 있는 구조가 아니다. 이 시각화 노드는 모터 명령을 발행하지 않는다. 근거: 로컬 미커밋 파일 `src/kaboat_hardware/kaboat_hardware/tank_test_visualization.py`, [RViz 실행 스크립트](scripts/rviz_tank.sh).

## 4. 주요 토픽 종류와 역할

`→`는 주요 소비처다. **원격 방향**의 `J→L`은 Jetson→노트북, `L→J`는 노트북→Jetson이며, `로컬`은 현재 브리지의 전달 허용 목록에 없다는 뜻이다. 허용 목록에 있어도 해당 노드가 실행 중이지 않으면 토픽 데이터는 생기지 않는다.

| 구분 | 토픽 / ROS 2 메시지 타입 | 발행 → 주요 소비처 | 원격 방향 |
|---|---|---|---|
| 외벽 라이다 | `/shore/scan` / `sensor_msgs/msg/LaserScan` | 외벽 RPLIDAR → `lidar_boat_tracker` | 로컬(L) |
| 외벽 위치 측정 | `/boat_pose` / `geometry_msgs/msg/PoseWithCovarianceStamped` | `lidar_boat_tracker` → `indoor_lidar_odom` | L→J |
| 선체 라이다 | `/scan/raw`, `/scan` / `sensor_msgs/msg/LaserScan` | TG-50 → `onboard_scan_filter` → 인식·격자·회피 | J→L |
| 관성 센서 | `/imu/data` / `sensor_msgs/msg/Imu` | GQ7 → 실내 위치 추정·센서 필터·인식 | J→L |
| GNSS/INS | `/gps/fix` / `sensor_msgs/msg/NavSatFix`; `/ekf/odometry_map` / `nav_msgs/msg/Odometry` | GQ7 → 위치 확인·실외 `/odom` remap | J→L |
| 대표 위치 추정 | `/odom` / `nav_msgs/msg/Odometry`; `/tf`, `/tf_static` / TF 메시지 | 실내 `indoor_lidar_odom` 또는 실외 GQ7 경로 → 제어·RViz | J→L |
| 주변 장애물 | `/occupancy_grid` / `nav_msgs/msg/OccupancyGrid` | `occupancy_grid` → 행동·회피 노드 | J→L |
| 부표·도킹 검출 | `/detections/buoys` / `kaboat_msgs/msg/BuoyArray`; `/detections/dock_marks` / `kaboat_msgs/msg/MarkArray` | 인식 노드 → 행동 노드 | 로컬(J) |
| 미션 상태·목표 | `/mission/state` / `std_msgs/msg/String`; `/mission/goal` / `geometry_msgs/msg/PoseStamped` | `mission_manager` → 행동 노드·`cmd_mux` | J→L |
| 미션별 명령 | `/cmd/approach`, `/cmd/gate`, `/cmd/station`, `/cmd/dock`, `/cmd/search`, `/cmd/avoid` / `geometry_msgs/msg/Twist` | 행동 노드 → `cmd_mux` | 로컬(J) |
| 최종 자율 명령 | `/cmd_vel` / `geometry_msgs/msg/Twist` | `cmd_mux` 또는 단독 수조 시험 노드 → `thruster_driver` | 로컬(J) |
| 노트북 키보드 조종 | `/teleop/enable` / `std_msgs/msg/Bool`; `/teleop/cmd_vel` / `geometry_msgs/msg/Twist` | 노트북 조종 창 → Jetson `thruster_driver` | L→J |
| 조종 응답·진단 | `/teleop/status` / `std_msgs/msg/String`; `/diagnostics` / `diagnostic_msgs/msg/DiagnosticArray` | `thruster_driver` → 노트북 창·로컬 진단 | `/teleop/status`: J→L; `/diagnostics`: 로컬(J) |
| 경로·시각화 | `/avoid/plan`, `/bspline_test/path`, `/dock/waypoints` / `nav_msgs/msg/Path`; 관련 `/.../markers` / `visualization_msgs/msg/MarkerArray` | 계획·시험 노드 → RViz | 허용된 항목만 J→L |

**표를 읽을 때의 주의점:** GQ7을 `real_sensors.launch.py`의 기본 실외 설정으로 실행하면 `/ekf/odometry_map`이 `/odom`으로 remap되므로 원래 이름의 발행자가 별도로 남는다고 가정하지 않는다. 외벽 추적기 코드에는 `/odom` 발행자 객체가 생성되지만 `lidar_tracker.yaml`의 `publish_odom: false`에서는 실제 `/odom` 메시지를 내보내지 않는다. 따라서 단순한 토픽 목록·발행자 수뿐 아니라 **메시지 수신 여부와 어느 노드가 발행하는지**를 함께 확인한다. [GQ7 Launch](src/kaboat_hardware/launch/real_sensors.launch.py), [라이다 설정](src/kaboat_hardware/config/lidar_tracker.yaml), [추적기 출력 코드](src/kaboat_hardware/kaboat_hardware/lidar_boat_tracker.py)

서비스(예: `/calibrate_imu_yaw`, `/clear_trajectory`)와 액션은 토픽과 구분해야 한다. 현재 Zenoh 설정의 서비스·액션 허용 목록은 비어 있다. 카메라 원본 영상, `/start_mission`, `/emergency_stop`, `/cmd_vel`, `/cmd/*`도 원격 전달 목록에 없다. [브리지 허용 목록](scripts/zenoh_monitor_link.py)

## 5. 서로 다른 네트워크에서 토픽을 전달한 구성

1. **Tailscale로 IP 연결 경로를 마련한다.** Jetson과 노트북 호스트 OS를 같은 tailnet에 가입시키고, 각 장치의 `tailscale ip -4`로 `100.64.0.0/10` 범위 주소를 확인한다. 예를 들어 서로 다른 휴대폰 핫스팟에 있어도 브리지의 접속 대상은 핫스팟의 가변 `192.168.x.x` 주소가 아니라 Jetson의 Tailscale 주소다. 이는 *구성 의도*이며, 실제 서로 다른 핫스팟에서의 성공 여부는 별도 측정 대상이다.
2. **ROS 2 도메인을 분리한다.** Jetson 노드는 `ROS_DOMAIN_ID=42`, 노트북 호스트와 외벽 라이다 Docker는 `ROS_DOMAIN_ID=43`을 쓴다. Docker Compose는 `network_mode: host`여서 노트북 호스트의 ROS 도메인 43에 참여한다. 같은 LAN에 있어도 두 도메인의 DDS 발견·전달이 직접 섞이지 않도록 분리한다. [Docker Compose](docker-compose.lidar.yml)
3. **각 호스트에서 Zenoh ROS 2 DDS 브리지를 실행한다.** `scripts/zenoh_monitor_link.py`는 실행 중인 Tailscale 주소와 ROS 도메인을 확인하고 임시 Zenoh 설정을 만든다. Jetson 브리지는 자신의 Tailscale IP의 `TCP 7447`에서 수신한다. 노트북 브리지는 로컬 루프백의 임의 포트에서 시작해 Jetson `TCP 7447`에 접속하고, 실패 시 재시도한다. 두 호스트 모두 `zenoh-bridge-ros2dds`를 사용한다. [브리지 스크립트](scripts/zenoh_monitor_link.py)
4. **토픽을 방향별로 제한한다.** Jetson 쪽 `publishers` 허용 목록은 센서·위치·경로·상태 등 모니터링 토픽이고, 노트북 쪽은 `/boat_pose`, `/teleop/enable`, `/teleop/cmd_vel`이다. 반대쪽 브리지의 `subscribers` 목록이 각각 이를 받는다. 토픽 이름은 `^...$` 정규식으로 정확히 일치시킨다. 서비스·액션은 허용하지 않는다. 그래서 노트북의 `/boat_pose`는 실내 위치 추정으로 들어가고 Jetson의 `/odom`은 노트북 RViz로 돌아올 수 있다. `/cmd_vel` 자체는 연결되지 않는다.

```bash
# Jetson 호스트: ROS 2와 해당 워크스페이스 환경을 먼저 로드한 터미널
export ROS_DOMAIN_ID=42
python3 scripts/zenoh_monitor_link.py jetson

# 노트북 호스트: 별도 터미널, Jetson의 실제 tailscale ip -4 값 사용
export ROS_DOMAIN_ID=43
export ROS_LOCALHOST_ONLY=0
python3 scripts/zenoh_monitor_link.py laptop --jetson-ip 100.x.y.z
```

노트북 키보드 조종을 켠 경우에도 `/teleop/cmd_vel`은 Jetson의 `thruster_driver`가 받는 **별도 수동 조종 입력**이다. 드라이버가 로컬 RC 입력, 수동 조종 활성 상태, 자율 `/cmd_vel`, 비상 정지 상태를 중재하고, 수동 입력이 끊기면 중립 출력을 선택한다. 노트북 창은 `/teleop/status` 응답이 오래되면 키보드 명령을 차단한다. 이 기능은 현재 작업 트리에 있으나, 기존 [원격 모니터링 가이드](REMOTE_MONITORING.md)의 “노트북→Jetson은 `/boat_pose`만”이라는 설명은 이 변경을 반영하지 못했다. 근거: 로컬 미커밋 파일 `scripts/teleop_laptop.py`, [추진기 드라이버](src/kaboat_hardware/kaboat_hardware/thruster_driver.py).

### 5.1 네트워크 계층을 따라 본 한 메시지의 이동

```text
노트북 라이다 노드 ── ROS 2 DDS (도메인 43) ── 노트북 zenoh-bridge-ros2dds
      ── Zenoh TCP 세션 ── Tailscale IP 경로 ── Jetson zenoh-bridge-ros2dds
      ── ROS 2 DDS (도메인 42) ── Jetson indoor_lidar_odom
```

Tailscale은 두 호스트 사이에 주소와 통신 경로를 제공하지만 ROS 토픽을 선택하거나 메시지 타입을 해석하지 않는다. Zenoh 브리지가 각 호스트의 **로컬 DDS 토픽**을 발견해 Zenoh 연결로 중계하고, 상대 브리지가 자기 ROS 도메인에 다시 발행한다. 이 프로젝트는 각 ROS 노드의 RMW를 `rmw_zenoh_cpp`로 교체하는 방식이 아니라, 기존 DDS 노드 옆에 **`zenoh-bridge-ros2dds`를 별도 프로세스로 실행**하는 방식이다. 실외의 Jetson 로컬 센서·제어 경로는 원격 모니터링 링크와 분리된다. 실내 수조의 `/boat_pose`는 이 링크에 의존하므로 연결이 끊겨 위치 측정이 멈추면 Jetson의 `/odom`도 타임아웃 후 중단된다. [브리지 실행 코드](scripts/zenoh_monitor_link.py), [실내 오도메트리](src/kaboat_hardware/kaboat_hardware/indoor_lidar_odom.py)

Jetson 브리지는 `mode: router`로 Tailscale IP의 `tcp/<Jetson-IP>:7447`만 수신한다. 노트북 브리지는 `tcp/127.0.0.1:0`에서 시작해 Jetson에 접속한다. 설정상 최초 접속에 실패해도 즉시 종료하지 않고 1초부터 최대 4초 간격으로 재시도한다. 토픽 허용 패턴은 `^/topic/name$` 형태로 고정되어 있어, 예를 들어 `/scan` 허용이 `/shore/scan`까지 포괄하지 않는다. [브리지 설정 생성 함수](scripts/zenoh_monitor_link.py)

### 5.2 현재 브리지의 허용 토픽 전체 목록

아래는 `scripts/zenoh_monitor_link.py`의 **이름 허용 목록**이다. 허용은 “전달할 수 있다”는 뜻이며, 각 토픽에 실제 발행자가 있다는 뜻은 아니다. Jetson→노트북 32개, 노트북→Jetson 3개다.

| 방향·용도 | 정확한 허용 토픽 |
|---|---|
| Jetson→노트북: 센서·위치·TF | `/scan`, `/scan/raw`, `/odom`, `/tf`, `/tf_static`, `/occupancy_grid`, `/imu/data`, `/gps/fix`, `/gnss_1/velocity`, `/ekf/status`, `/ekf/odometry_map` |
| Jetson→노트북: 회피·위험 지도 | `/avoid/plan`, `/avoid/cps`, `/avoid/dri`, `/real_avoid_test/path`, `/real_avoid_test/markers`, `/real_avoid_test/dri`, `/real_avoid_test/status`, `/manual_dri/dri`, `/manual_dri/status` |
| Jetson→노트북: 단독 시험 시각화 | `/straight_drive/markers`, `/bspline_test/path`, `/bspline_test/markers`, `/circle_drive/markers`, `/station_keeping/markers` |
| Jetson→노트북: 도킹·미션 | `/dock/waypoints`, `/dock/entrance_base`, `/dock/marker_base`, `/dock/detection_status`, `/mission/state`, `/mission/goal` |
| Jetson→노트북: 키보드 조종 응답 | `/teleop/status` |
| 노트북→Jetson: 위치 측정 | `/boat_pose` |
| 노트북→Jetson: 명시적 수동 조종 | `/teleop/enable`, `/teleop/cmd_vel` |

브리지 양쪽의 `publishers`와 `subscribers` 허용 목록은 이 두 방향에 맞춰 서로 반대로 설정된다. `service_servers`, `service_clients`, `action_servers`, `action_clients`는 빈 목록이다. `/cmd_vel`, `/cmd/*`, `/rc/cmd_vel`, `/emergency_stop`, `/start_mission`, `/diagnostics`, `/camera/color/image_raw`, `/camera/depth/image_raw`, `/shore/scan`은 이 브리지로 보내지 않는다. 특히 `/teleop/cmd_vel` 허용이 일반 `/cmd_vel`의 원격 허용을 뜻하지 않는다. [정확한 원본 목록](scripts/zenoh_monitor_link.py)

### 5.3 구성 반영과 실행 순서

**설치·배포 단계:** 두 호스트의 Tailscale 가입과 `zenoh-bridge-ros2dds` 설치는 [원격 모니터링 가이드](REMOTE_MONITORING.md#한-번-설치)에 있다. Jetson과 노트북의 브리지 스크립트가 같은 토픽 허용 목록을 갖고 있어야 한다. ROS 노드 코드가 바뀌었다면 해당 기기에서 `colcon build`한 뒤 새 터미널에서 워크스페이스 환경을 다시 로드하고, 이미 실행 중인 노드·브리지는 재시작해야 새 코드와 허용 목록을 적용한다. 단순 `source`만으로 기존 프로세스의 코드는 갱신되지 않는다. 키보드 조종의 상세 배포 안내는 로컬 미커밋 파일 `KEYBOARD_TELEOP.md`에 있다.

**실행 단계:** 아래는 통신 경로를 이해하기 위한 순서다. 외벽 라이다와 Jetson 센서를 먼저 구동하고, 두 브리지를 실행한 뒤 수신 토픽을 확인한다. 추진기 노드는 통신 확인만을 위해 실행할 필요가 없다.

```text
① 양쪽 호스트 Tailscale 연결 및 Jetson Tailscale IP 확인
② 노트북 Docker 라이다 컨테이너: 도메인 43에서 /shore/scan → /boat_pose 확인
③ Jetson 센서·실내 위치 추정: 도메인 42, /imu/data와 /odom 소유 확인
④ Jetson Zenoh 브리지: 도메인 42, Jetson Tailscale IP:7447 수신
⑤ 노트북 Zenoh 브리지: 도메인 43, Jetson IP:7447 접속
⑥ Jetson에서 /boat_pose 수신, 노트북에서 /odom 수신을 각각 확인
⑦ 필요할 때만 RViz·자율운항·추진기 관련 절차를 별도로 실행
```

노트북 컨테이너는 `ROS_DOMAIN_ID=43 docker compose -f docker-compose.lidar.yml up -d --build --force-recreate`로 설정을 반영할 수 있다. 외벽 추적기는 컨테이너 안의 `lidar_boat_tracker.launch.py`, Jetson 실내 센서는 `indoor_tank.launch.py enable_thrusters:=false`를 사용한다. 컨테이너의 ROS 2 Humble, Jetson의 Humble, 노트북 호스트의 Jazzy는 각각 **해당 터미널에서 맞는 ROS 기본 환경과 워크스페이스 `install/setup.bash`**를 로드해야 한다. 실제 포트·장치 연결, 빌드 명령과 RViz 실행은 [수조 시험 가이드](TANK_TESTS_GUIDE.md)에서 최신 하드웨어 상태에 맞춰 확인한다.

## 6. 실제 운용 확인에 필요한 증거

보고서에 “서로 다른 네트워크에서 실제 통신 성공”이라고 쓰려면 두 장치를 각각 다른 네트워크에 둔 상태에서 아래 결과를 기록해야 한다. 소스 코드 검사나 단일 PC 테스트만으로는 그 결론을 내릴 수 없다.

| 확인 항목 | 명령/관찰 | 통과 기준 |
|---|---|---|
| 두 장치의 가상 네트워크 연결 | 양쪽 `tailscale status`, `tailscale ip -4`; 노트북 `tailscale ping <Jetson-IP>` | 양쪽 Running, Jetson 주소 응답 |
| Zenoh 접속 | 양쪽 브리지 로그 | Jetson `TCP 7447` 수신, 노트북 연결 성공 및 재접속 상태 |
| 노트북→Jetson 위치 측정 | Jetson 도메인 42에서 `ros2 topic info /boat_pose --verbose`, `ros2 topic hz /boat_pose` | 발행자 발견과 실제 메시지 수신·주기 확인 |
| Jetson→노트북 오도메트리 | 양쪽에서 `ros2 topic hz /odom`; Jetson에서 `ros2 topic info /odom --verbose` | 실제 메시지가 흐르고 Jetson의 `/odom` 발행자가 한 개이며 프레임·원점이 운용 모드와 일치 |
| 원격 조종 사용 시 | `/teleop/status` 수신, Jetson의 선택 모드·중립 동작 관찰 | 상태 응답과 명령 타임아웃 동작 확인; 물리 출력은 별도 안전 절차로 검증 |

### 6.1 통신 단계별 읽기 전용 확인 명령

다음 명령은 **모터 노드를 실행하거나 움직임 명령을 발행하지 않는다.** 이미 실행 중인 센서·브리지의 상태를 읽는 데 사용한다. 각 터미널의 ROS 배포판과 워크스페이스 환경은 먼저 해당 장치에 맞게 로드해야 한다.

```bash
# 두 호스트 각각: 가상 네트워크 주소와 연결 상태
tailscale status
tailscale ip -4

# 노트북 호스트: Jetson이 출력한 실제 Tailscale 주소로 교체
tailscale ping <JETSON_TAILSCALE_IP>
```

```bash
# 노트북 라이다 Docker 컨테이너, ROS_DOMAIN_ID=43
ros2 topic info /shore/scan --verbose
timeout 8s ros2 topic hz /shore/scan
ros2 topic info /boat_pose --verbose
timeout 8s ros2 topic hz /boat_pose
ros2 topic echo /boat_pose --once --field header.frame_id
```

```bash
# Jetson 호스트, ROS_DOMAIN_ID=42, Zenoh 연결 후
ros2 topic info /boat_pose --verbose
timeout 8s ros2 topic hz /boat_pose
ros2 topic echo /boat_pose --once --field header.frame_id
ros2 topic info /odom --verbose
timeout 8s ros2 topic hz /odom
ros2 topic echo /odom --once --field header.frame_id
ros2 topic echo /odom --once --field child_frame_id
```

```bash
# 노트북 호스트, ROS_DOMAIN_ID=43, Zenoh 연결 후
ros2 topic info /odom --verbose
timeout 8s ros2 topic hz /odom
ros2 topic echo /odom --once --field header.frame_id
```

`ros2 topic list`는 발견된 이름만 보여준다. `info --verbose`는 발행·구독 노드와 QoS를, `hz`와 `echo`는 실제 메시지 수신을 확인한다. 센서가 표적을 못 찾으면 `/shore/scan`이 살아 있어도 `/boat_pose`는 멈출 수 있다. 반대로 `/boat_pose`가 노트북에 있지만 Jetson에 없다면 Tailscale·브리지·도메인·QoS 순으로 확인한다. `/odom`의 **실제 발행자 한 개** 조건은 Jetson 도메인 42에서 확인한다. 노트북의 외벽 추적기는 `publish_odom:false`여도 코드상 `/odom` 발행자 객체를 생성하므로, 노트북에서 단순 발행자 수만 세면 혼동할 수 있다. [라이다 추적기](src/kaboat_hardware/kaboat_hardware/lidar_boat_tracker.py), [라이다 설정](src/kaboat_hardware/config/lidar_tracker.yaml)

### 6.2 증상별 원인 구분

| 관찰 증상 | 먼저 확인할 위치 | 해석·후속 확인 |
|---|---|---|
| `tailscale ping`이 실패 | 두 호스트의 `tailscale status`, IP, 같은 tailnet 가입 여부 | ROS 토픽을 보기 전에 호스트 간 IP 연결부터 확인 |
| Jetson 브리지가 시작하지 못함 | Jetson 브리지 로그와 `tailscale ip -4`, `zenoh-bridge-ros2dds --version` | 지정 IP·포트·실행 파일·도메인 환경을 확인 |
| 노트북에는 `/boat_pose`, Jetson에는 메시지 없음 | 양쪽 브리지 로그, `ROS_DOMAIN_ID=43/42`, `/boat_pose` 발행·구독 QoS | 이름만 보이는 상태와 실제 수신을 구분 |
| `/boat_pose` 수신은 되지만 `/odom`이 멈춤 | Jetson `indoor_lidar_odom` 로그와 `/boat_pose` 주기·프레임 | 1초 이상 위치 측정이 끊기면 `/odom` 발행 중단; 유효하지 않은 pose도 거부 |
| 노트북에 `/odom`이 도착하지만 RViz 화면이 비어 있음 | `/odom`·`/scan`의 `header.frame_id`, `/tf`, RViz Fixed Frame | 토픽 수신과 좌표 변환 체인은 별개 |
| `/scan` 이름은 있으나 데이터가 없음 | Jetson `/scan/raw`, 스캔 필터·IMU 기울기 로그 | TG-50이 없거나 필터가 기울기 때문에 스캔 발행을 중단했을 수 있음 |
| 키보드 창이 명령을 차단 | `/teleop/status` 주기와 상태, 브리지 재시작 여부 | 젯슨 응답 부재·백엔드 오류·RC 우선 등 상태를 구분 |

### 6.3 최종 보고서에 채울 현장 측정 기록

실제 실험 후 아래 표를 복사해 수치와 로그 파일명을 채우면 “구현”과 “검증”을 명확히 구분할 수 있다. 실험하지 않은 칸은 **미측정**으로 둔다.

| 측정 날짜·장소 | 노트북 네트워크 / Jetson 네트워크 | Tailscale ping | `/shore/scan` | Jetson `/boat_pose` | 양쪽 `/odom` | 프레임·원점 확인 | 결과 로그·사진 |
|---|---|---|---|---|---|---|---|
| 미측정 | 미측정 | 미측정 | 미측정 | 미측정 | 미측정 | 미측정 | 미측정 |

현 저장소의 [원격 모니터링 가이드](REMOTE_MONITORING.md)도 두 기기·두 핫스팟 간 통신을 아직 검증하지 않았다고 적고 있다. 따라서 최종 DOCX에는 **“구현 구조”와 “현장 검증 결과”를 별도 항목**으로 두고, 검증 결과는 실제 측정 후 채워야 한다.

## 7. 보고서 본문에 옮길 수 있는 서술 초안

**통신 설계.** KABOAT의 센서·제어 기능은 ROS 2 노드로 나뉜다. 각 노드는 메시지 타입을 명시해 토픽을 발행하거나 구독한다. 선체 라이다의 스캔과 GQ7의 관성·GNSS 데이터가 위치 추정 및 환경 인식에 들어가고, 계산된 위치와 장애물 정보가 미션 행동 노드로 전달된다. 행동 노드는 미션별 명령 토픽을 발행하며, 명령 중재기가 현재 미션에 해당하는 한 명령을 최종 `/cmd_vel`로 선택한다. 추진기 드라이버는 이 값을 좌우 출력으로 변환한다. 실내 수조에서는 외벽 라이다가 계산한 `/boat_pose`가 위치 추정의 기준 입력이 되고, 실외에서는 GQ7 INS 결과가 `/odom` 경로에 연결된다.

**서로 다른 네트워크의 연결.** 선체 Jetson과 노트북은 서로 다른 ROS 도메인에서 동작한다. 노트북의 라이다 컨테이너와 노트북 호스트는 도메인 43, Jetson은 도메인 42를 사용한다. 두 호스트를 같은 Tailscale tailnet에 가입시키면 핫스팟의 로컬 주소가 달라져도 Jetson의 Tailscale IP를 접속 대상으로 사용할 수 있다. 각 호스트에 설치한 Zenoh ROS 2 DDS 브리지는 이 IP 경로 위에 TCP 연결을 만들고, 양쪽의 로컬 DDS에서 선택한 토픽을 상대 도메인으로 전달한다. 이 역할 분담으로 Tailscale은 호스트 간 연결을, Zenoh는 토픽의 선택·중계를 담당한다.

**양방향 데이터 흐름.** 실내 수조의 노트북에서는 고정 RPLIDAR A3가 `/shore/scan`을 발행하고 추적기가 선체 표적의 위치와 방향을 `/boat_pose`로 계산한다. Zenoh는 `/boat_pose`를 Jetson으로 전달한다. Jetson의 `indoor_lidar_odom`은 수신한 위치를 이용해 `/odom`과 TF를 발행하고, 제어 노드는 이 정보를 이용해 경로를 추종한다. 반대 방향으로는 Jetson의 `/odom`, 선체 센서, 회피 경로와 상태가 노트북에 전달되어 RViz에서 관찰된다. 노트북의 라이다 원본 `/shore/scan`과 Jetson의 최종 자율 명령 `/cmd_vel`은 브리지 전달 목록에 넣지 않았다.

**제어 입력과 검증 범위.** 현재 작업 트리에는 노트북 키보드 조종을 위한 `/teleop/enable`과 `/teleop/cmd_vel` 전달이 추가되어 있다. 이 입력은 Jetson의 추진기 드라이버가 자율 `/cmd_vel`, RC 입력 및 비상 정지와 함께 선택하는 별도 경로이며, Jetson에서 명령 상한과 타임아웃을 다시 적용한다. 소스 코드에 브리지와 토픽 경로가 구현된 사실은 확인했지만, 서로 다른 두 핫스팟에서 실제 메시지 주기·지연·끊김 복구를 측정한 결과는 아직 이 자료에 포함하지 않았다.

## 8. DOCX 작성 시 인용할 주요 소스

- 브리지의 토픽 허용 목록·TCP 연결·도메인 설정: [`scripts/zenoh_monitor_link.py`](scripts/zenoh_monitor_link.py)
- 양쪽 설치·실행 절차 및 기존 검증 범위: [`REMOTE_MONITORING.md`](REMOTE_MONITORING.md)
- 실내 수조 실행 절차: [`TANK_TESTS_GUIDE.md`](TANK_TESTS_GUIDE.md)
- 외벽 라이다 입력과 `/boat_pose` 출력: [`lidar_boat_tracker.launch.py`](src/kaboat_hardware/launch/lidar_boat_tracker.launch.py), [`lidar_boat_tracker.py`](src/kaboat_hardware/kaboat_hardware/lidar_boat_tracker.py)
- 실내 `/odom` 소유와 발행: [`indoor_tank.launch.py`](src/kaboat_hardware/launch/indoor_tank.launch.py), [`indoor_lidar_odom.py`](src/kaboat_hardware/kaboat_hardware/indoor_lidar_odom.py)
- 실외 GQ7 remap: [`real_sensors.launch.py`](src/kaboat_hardware/launch/real_sensors.launch.py)
- 미션 명령 흐름: [`mission_manager.py`](src/kaboat_mission/kaboat_mission/mission_manager.py), [`cmd_mux.py`](src/kaboat_control/kaboat_control/cmd_mux.py), [`thruster_driver.py`](src/kaboat_hardware/kaboat_hardware/thruster_driver.py)

> 주의: 기존 [`SYSTEM_DATAFLOW.md`](SYSTEM_DATAFLOW.md)의 외벽 센서명과 “Wi-Fi 직접 전송” 그림은 현재 RPLIDAR A3·Tailscale/Zenoh 구성보다 이전 설명을 포함한다. 새 보고서의 네트워크 구조에는 이 파일보다 위의 Launch, 브리지 스크립트, 현재 설정을 우선 적용한다.
