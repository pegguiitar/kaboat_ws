# 실내 수조 실물 테스트: 노트북·젯슨 실행 명령

이 문서에 실내 수조 시험의 실행 명령을 모았습니다. 수조 외벽 노트북에는 **고정 RPLIDAR A3M1**, 배 위 젯슨에는 **GQ7과 모터 제어기**를 연결합니다. 장애물 회피 시험에는 **선체 TG-50**도 연결합니다. 노트북은 `ROS_DOMAIN_ID=43`, 젯슨은 `ROS_DOMAIN_ID=42`를 사용합니다. 두 기기를 같은 Tailscale 네트워크에 연결하고 Zenoh 브리지를 각각 실행하면, 노트북의 `/boat_pose`가 젯슨에 전달되고 젯슨이 계산한 `/odom`이 노트북으로 돌아옵니다. 같은 와이파이에서도 두 ROS 도메인을 분리해 DDS 직접 전달과 Zenoh 전달이 중복되지 않게 합니다.

고정 RPLIDAR의 기본 위치는 수조 절대좌표 `(10.0, 2.5)m`(오른쪽 벽 중앙)이며,
스캔 0° 방향이 `-X`를 바라보도록 장착합니다. 새 센서의 실제 0° 방향을 먼저 확인하고 좌표와 각도는
[`lidar_tracker.yaml`](src/kaboat_hardware/config/lidar_tracker.yaml)에서 설정합니다.
배에는 좌우 두 봉을 잇는 길이 0.60m의 횡단 판을 부착하고, 그 중점이
`base_link` 원점과 일치해야 합니다. 추적기는 판의 선분 기울기와 중점으로
`/boat_pose`를 계산합니다.

외벽 노트북의 고정 RPLIDAR은 확인된 `/dev/serial/by-id/...` 경로를, 젯슨의 아두이노 CH340 모터 제어기는 최신 가이드의 `/dev/ttyCH341USB0`을 아래 명령에 직접 지정했습니다. 선체 TG-50 포트는 젯슨에서 따로 확인해 입력하세요. 장치나 드라이버가 바뀌면 해당 기기에서 `/dev/serial/by-id/`, `/dev/serial/by-path/`, `/dev/ttyUSB*`, `/dev/ttyCH341*`를 다시 확인해야 합니다.

## 수조 밖에서 외벽 라이다 스캔만 확인

이 런치는 RPLIDAR A3M1의 `/shore/scan`과 RViz만 실행하며 `/boat_pose` 추적기는 켜지 않습니다. 노트북의 그래픽 데스크톱 터미널에서 실행하세요.

```bash
cd ~/Projects/KABOAT/kaboat_ws
ROS_DOMAIN_ID=43 docker compose -f docker-compose.lidar.yml up -d --build
xhost +si:localuser:root
docker exec -it -e DISPLAY="$DISPLAY" kaboat_lidar bash
# Docker 내부
cd /workspace/kaboat_ws
source /opt/ros/humble/setup.bash
source /opt/sllidar_ws/install/setup.bash
colcon build --symlink-install --packages-select kaboat_hardware
source install/setup.bash
ros2 launch kaboat_hardware shore_lidar_scan.launch.py
```

RViz의 Fixed Frame은 `shore_laser_frame`, 표시 토픽은 `/shore/scan`입니다. 다른 Docker 터미널에서 `ros2 topic hz /shore/scan`으로 수신 주기를 확인합니다. 수조 위치 추적 런처를 시작하기 전 이 스캔 전용 런치를 `Ctrl+C`로 종료하세요. 한 장치를 두 드라이버가 동시에 열 수 없습니다.

## 1. 수조 외벽 노트북: 라이다와 배 위치 추적

**호스트 터미널 1** — Docker 시작. Docker 권한이 현재 터미널에만 반영되지 않았다면 `newgrp docker` 후 재실행합니다. Compose는 호스트 네트워크와 `/dev`를 공유하며 기본 Domain ID가 43입니다. 이전의 도메인 42 컨테이너를 사용했다면 `--force-recreate`로 새 환경을 적용합니다. Tailscale·Zenoh의 양쪽 기기 설치 명령은 [REMOTE_MONITORING.md](REMOTE_MONITORING.md#한-번-설치)를 따릅니다.

```bash
cd ~/Projects/KABOAT/kaboat_ws
ROS_DOMAIN_ID=43 docker compose -f docker-compose.lidar.yml up -d --build --force-recreate
docker exec -it kaboat_lidar bash
```

**같은 터미널, Docker 내부** — 코드 빌드 후 고정 라이다 실행. 이미 빌드했고 소스가 바뀌지 않았다면 `colcon build`를 생략할 수 있습니다.

```bash
cd /workspace/kaboat_ws
source /opt/ros/humble/setup.bash
source /opt/sllidar_ws/install/setup.bash
colcon build --symlink-install --packages-up-to kaboat_hardware
source install/setup.bash
ros2 launch kaboat_hardware lidar_boat_tracker.launch.py \
  port:=/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_344ff5135095724097887d9c6d4b91ce-if00-port0 \
  enable_rviz:=false
```

이 터미널은 켜 둡니다. 고정 라이다는 `/shore/scan`, 배 위치 추적기는 `/boat_pose`를 노트북 도메인 43에 발행합니다. `/boat_pose`만 Zenoh를 통해 젯슨 도메인 42로 전달됩니다. 외벽 라이다가 `/odom`을 젯슨에 직접 발행하지는 않습니다.
라이다를 옮긴 뒤에는 기존 추적기 프로세스를 종료하고 이 명령으로 다시 실행해야 새 설치 좌표가 적용됩니다.
별도 Docker 터미널에서 `ros2 topic hz /shore/scan`, `ros2 topic echo /shore/scan --once --field header`, `ros2 topic echo /boat_pose --once`로 확인할 수 있습니다. 스캔의 `header.frame_id`는 `shore_laser_frame`이어야 합니다. 수조 `(9.0, 2.5)m`에 놓은 물체가 RViz 스캔에서 그 부근에 보이는지도 확인합니다. 기본 설치값은 `(10.0, 2.5)m`, 스캔 0° 방향 `180°`입니다. 실제 장착값이 다르면 런치 명령에 `lidar_pos_x:=... lidar_pos_y:=... lidar_yaw_deg:=...`를 실측값으로 추가하세요. [`lidar_tracker.yaml`](src/kaboat_hardware/config/lidar_tracker.yaml)도 기본값으로 사용할 수 있습니다. `/boat_pose`의 위치·방향이 실물과 맞는지 확인하기 전에는 **배 추진 모터**를 켜지 않습니다.
배의 0.60m 횡단 판을 수조 안의 **실측한 위치**에 놓고 `/boat_pose`가 그 좌표 근처에서 연속으로 발행되는지 확인하세요. A3M1은 실측 720빔으로 이전 TG-50보다 스캔 점이 적으므로, 검출이 끊기면 RViz의 `/lidar_tracker/filtered_scan`에서 판에 최소 6점이 남는지 먼저 확인합니다. 판이 없는데도 `/boat_pose`가 나오면 다른 직선 물체를 잘못 고른 것이므로 그 상태로 젯슨의 IMU 보정·주행을 진행하지 마세요.
드라이버가 장치 정보와 `health status : OK`를 표시해도 `/shore/scan`이 나오지 않으면 회전부와 USB 연결을 확인하세요. 이 노트북의 A3M1은 공식 ROS 드라이버의 기존 `Standard` 경로에서 시간 초과됐지만, Docker 이미지의 표준 스캔 함수 패치 적용 후 `/shore/scan`을 약 12.7Hz로 발행했습니다. 이미지가 오래됐다면 `docker compose -f docker-compose.lidar.yml up -d --build --force-recreate`로 갱신하고, 같은 장치를 쓰는 스캔 전용 런치가 남아 있지 않은지 확인하세요. USB 재연결로 `/dev/ttyUSB*` 번호가 바뀌어도 같은 장치의 `by-id` 경로는 유지됩니다.

**호스트 터미널 2** — Docker 밖에서 Zenoh 브리지를 켭니다. 젯슨의 `tailscale ip -4` 출력값을 물음에 입력하세요. 라이다가 먼저 실행돼 있어도 브리지를 나중에 켜면 연결되며, 이 터미널은 계속 열어 둡니다.

```bash
cd ~/Projects/KABOAT/kaboat_ws
source /opt/ros/jazzy/setup.zsh
export ROS_DOMAIN_ID=43
export ROS_LOCALHOST_ONLY=0
printf '젯슨 Tailscale IP: '
read -r JETSON_TAILSCALE_IP
tailscale ping "$JETSON_TAILSCALE_IP"
python3 scripts/zenoh_monitor_link.py laptop --jetson-ip "$JETSON_TAILSCALE_IP"
```

Zenoh는 호스트의 도메인 43에서 Docker 라이다의 `/boat_pose`를 발견합니다. 이 브리지는 `/start_mission`, `/emergency_stop`, `/cmd_vel`을 젯슨에 전달하지 않습니다.

**호스트의 그래픽 데스크톱 터미널 3** — 고정 경로 추종용 RViz 창을 별도로 엽니다. 컨테이너의
root 사용자에게 X11 화면 접근을 허용해야 합니다. SSH 터미널이 아닌 노트북
화면에서 실행합니다.

```bash
xhost +si:localuser:root
docker exec -it -e DISPLAY="$DISPLAY" kaboat_lidar bash -lc \
  'source /workspace/kaboat_ws/install/setup.bash && ros2 run rviz2 rviz2 -d /workspace/kaboat_ws/install/kaboat_bringup/share/kaboat_bringup/rviz/tank_tracking.rviz'
```

장애물 회피 경로를 볼 때는 노트북의 **새 그래픽 데스크톱 터미널**에서 다음 전용 화면을 엽니다. 이 화면은 배 뒤에서 `base_link`를 따라 움직이는 3인칭 시점이며, 선체 `/scan`은 빨간 점, 생성한 경로는 청록색 선, 배 자세는 주황색 화살표로 표시합니다. 두 RViz 창은 동시에 실행할 수 있습니다.

```bash
docker exec -it -e DISPLAY="$DISPLAY" kaboat_lidar bash -lc \
  'source /workspace/kaboat_ws/install/setup.bash && ros2 launch kaboat_hardware real_obstacle_avoid_rviz.launch.py'
```

화면 연결 오류가 나면 **노트북 그래픽 데스크톱 터미널**에서 `echo "$DISPLAY"`가 비어 있지 않은지, `xhost`를 실행했는지 확인합니다. RViz를 닫은 뒤 접근 권한을 해제하려면 `xhost -si:localuser:root`를 실행합니다.

## 2. 젯슨: 센서와 오도메트리

젯슨에 코드를 새로 받았거나 수정했다면 **처음 한 번** 빌드합니다. 첫 줄의 경로는 젯슨에 클론한 워크스페이스 경로로 바꾸세요.

```bash
cd ~/Desktop/kaboat_ws/repo
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to kaboat_hardware
```

젯슨의 **새 터미널마다** 먼저 아래 명령을 실행합니다. (`install/setup.bash`를 빠뜨리면 `Package 'kaboat_hardware' not found` 오류가 납니다.)

```bash
cd ~/Desktop/kaboat_ws/repo
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0

# FastDDS UDP 버퍼 확대 (TG-50 라이다 역직렬화 에러 방지)
sudo sysctl -w net.core.rmem_max=8388608
sudo sysctl -w net.core.rmem_default=8388608
```

**젯슨 터미널 1** — 젯슨의 Tailscale 주소를 확인하고 Zenoh 브리지를 시작합니다. 설치와 로그인은 [REMOTE_MONITORING.md](REMOTE_MONITORING.md#젯슨에서-추가로-할-일)를 따릅니다. 나온 주소를 노트북 호스트 터미널 2에 입력합니다. 이 브리지 터미널은 계속 열어 둡니다.

```bash
tailscale ip -4
python3 scripts/zenoh_monitor_link.py jetson
```

**젯슨 터미널 2** — 위의 새 터미널 준비 명령을 다시 실행한 뒤 GQ7과 실내 오도메트리를 시작합니다. 선체 라이다와 모터는 아직 켜지 않습니다.

```bash
ros2 launch kaboat_hardware indoor_tank.launch.py enable_thrusters:=false
```

선체 TG-50을 장착했다면 **위 명령 대신** 실제 라이다 포트를 지정해 실행합니다.

```bash
ls -l /dev/serial/by-id/ /dev/serial/by-path/ /dev/ttyUSB* 2>/dev/null
printf '선체 TG-50 포트 경로: '
read -r ONBOARD_LIDAR_PORT
ros2 launch kaboat_hardware indoor_tank.launch.py enable_thrusters:=false enable_onboard_lidar:=true onboard_lidar_port:="$ONBOARD_LIDAR_PORT"
```

선체 라이다 드라이버 원본은 `/scan/raw`, 지도·회피·RViz가 사용하는 필터 결과는 `/scan`입니다. 배 구조물이 보이는 220°~320°(ROS -140°~-40°) 값은 `/scan`에서 `inf`로 무효화하고, **IMU roll/pitch 기울기 필터(기본 임계값: ±5.0°)**가 함께 적용됩니다. 배가 5° 이상 기울면 수면 반사나 하늘을 훑는 노이즈를 방지하기 위해 스캔 발행을 차단(`drop`)합니다. 젯슨 터미널에서 `python3 scripts/view_roll_pitch.py`를 실행하여 실시간 자세 각도 및 필터 통과 여부를 모니터링할 수 있습니다. 육상/벤치 테스트에서 기울기 필터를 끄려면 `enable_imu_filter:=false`를 지정합니다. /shore/scan은 그대로 둡니다. 장애물 회피 RViz에서 기본으로 꺼진 `/scan/raw` 레이어를 켜 실제 구조물 방향과 비교하세요. 필요하면 `indoor_tank.launch.py`에 `onboard_lidar_mask_start_deg:=... onboard_lidar_mask_end_deg:=...`를 지정합니다. 이전 구조물 셀이 점유 지도에 남아 있다면 장애물 시험 런처도 재시작합니다.
선체 TG-50의 0.40m 최소 거리 컷오프는 센서 원점 기준입니다. 구조물이 더 멀리 보이면 장착 위치를 실측하고 [`tg50_onboard.yaml`](src/kaboat_hardware/config/tg50_onboard.yaml)의 `range_min`과 `base_link → onboard_laser_frame` 변환을 확인하세요.

## 3. 젯슨: IMU 보정과 모터·테스트 시작

선수를 수조 좌표계의 **-X 방향(180°)**으로 두고 배를 움직이지 마세요.
외벽 라이다에 횡단 판이 충분한 길이의 한 선분으로 보여야 합니다. 판만으로는
선수 앞뒤를 구분할 수 없어, 이 자세에서 IMU 초기 보정으로 방향을 확정합니다.

**젯슨 터미널 3** — 위의 젯슨 새 터미널 준비 명령을 실행한 뒤, 젯슨 도메인 42에서 `/boat_pose`가 실제로 도착하는지 확인하고 IMU를 보정합니다:

```bash
ros2 topic echo /boat_pose --once
ros2 run kaboat_hardware calibrate_indoor_imu
ros2 topic hz /odom
```

보정 성공 메시지와 `/odom` 발행을 확인하고 `ros2 topic hz`는 `Ctrl+C`로 종료합니다. 보정 실패 시 모터를 시작하지 마세요. GQ7 또는 `indoor_tank.launch.py`를 재시작했다면 다시 보정합니다.
`ros2 topic info /odom --verbose`의 `Publisher count`도 **1**인지 확인합니다. 실내에서는 `real_sensors.launch.py`를 별도로 실행하지 않습니다.

**젯슨 터미널 4** — 새 터미널 준비 명령을 실행한 뒤 모터 제어기 시작. 아두이노(CH340) 모터 제어기 포트는 최신 가이드에 지정된 `/dev/ttyCH341USB0`입니다. 젯슨에서 해당 장치가 실제로 등록됐는지 확인하세요.

```bash
ls -l /dev/ttyCH341USB0
ros2 launch kaboat_hardware thrusters.launch.py hardware_type:=serial port:=/dev/ttyCH341USB0 allow_port_scan:=false
```

**젯슨 터미널 5** — 새 터미널 준비 명령을 실행한 뒤 B-Spline 주행 노드 시작. 출발 명령 전까지 대기합니다.

```bash
ros2 launch kaboat_hardware bspline_track_test.launch.py wait_for_start:=true
```

이 테스트는 미리 지정한 B-Spline 코스를 따라가며, 회피 플래너의
`follow_cmd()`와 같은 전방 곡률 감속·lookahead 조향을 사용합니다. 장애물을
보고 새 경로를 생성하지는 않습니다. `cruise_speed`는 출력 상한이자
`v_full` 속도에서의 기준 출력이며, 기본 `v_full`·`cmd_exp`는 시뮬레이션
교정값입니다. 실물 속도 로그로 재교정한 뒤 `v_full:=... cmd_exp:=...`를
지정하세요. 전방 곡률 판단 구간은 `kappa_window:=...`, 선회 한계는
`omega_max:=...`로 조정합니다.

다른 코스를 시험할 때는 젯슨 터미널 5에서 위 명령 대신 `ros2 launch kaboat_hardware tank_tests.launch.py test:=straight`처럼 실행합니다. `test` 값은 `straight`, `bspline`, `circle`, `station_keeping` 중 하나입니다. **주행 테스트 노드는 한 번에 하나만** 실행합니다.
[`tank_tests.yaml`](src/kaboat_hardware/config/tank_tests.yaml)은 현재 런처가 자동으로 읽지 않으므로 실제 노드 기본값과 런처 인자를 확인해 조정합니다.

## 4. 장애물 회피 경로 생성과 RViz 미리보기

**2절의 선체 TG-50 포함 명령**으로 `/scan`과 `/odom`을 먼저 준비합니다. 모터 드라이버와 다른 주행 테스트 노드는 켜지 않습니다. 젯슨의 새 터미널에서 준비 명령을 실행한 뒤 아래 런처를 시작합니다.

```bash
ros2 launch kaboat_hardware real_obstacle_avoid_test.launch.py
```

기본 `drive_enabled:=false`에서는 `/cmd_vel`을 발행하지 않습니다. `/scan` → 점유 지도 → DRI → B-Spline 생성·검사 → `AvoidFsm`을 실행하고 경로와 제어 미리보기를 발행합니다. 목표는 출발 순간 선수 방향 `goal_distance`(기본 3m) 앞에 고정되며, 지도 변화에 따라 경로를 갱신합니다. 노트북에서는 **1절의 장애물 회피 전용 RViz**를 엽니다. Fixed Frame은 `odom`, 3인칭 카메라의 Target Frame은 `base_link`입니다. 화면이 배를 따라가지 않거나 스캔이 보이지 않으면 `odom → base_link → onboard_laser_frame` TF가 연결됐는지 확인합니다.

이 시험 런처는 배의 수면 반사 및 이상 기울기 노이즈를 방지하기 위해 점유 지도 노드의 IMU roll·pitch 기반 스캔 차단이 기본 활성화(`enable_tilt_gate:=true`, 기본 임계값: `tilt_threshold_deg:=5.0`)되어 있습니다. 배가 ±5.0° 이상 기울면 해당 스캔은 격자 지도 갱신에서 제외됩니다. 만약 육상 테스트나 센서 장착 각도로 인해 스캔이 반영되지 않으면 `enable_tilt_gate:=false` 또는 `tilt_threshold_deg:=15.0`으로 인수를 주어 실행합니다. `/odom`을 만드는 GQ7·외벽 라이다 절차는 그대로 필요합니다. 이미 별도의 점유 지도 노드를 실행해 `start_occupancy_grid:=false`로 지정했다면, 그 노드에도 파라미터를 맞추어야 합니다.

| RViz 표시 | 토픽 | 확인할 내용 |
| --- | --- | --- |
| 선체 스캔 | `/scan` | 220°~320° 가림 적용 |
| 위치·방향 | `/odom` | 실제 배 위치와 선수 방향 |
| 점유 지도 | `/occupancy_grid` | 수조 벽과 장애물 |
| 위험도 지도 | `/real_avoid_test/dri` | 경로 생성기 입력 |
| 경로·표식 | `/real_avoid_test/path`, `/real_avoid_test/markers` | B-Spline, 목표, 제어점, 궤적 |

젯슨의 **새 준비된 터미널**에서 스캔과 시험 상태를 확인합니다. `ros2 topic hz`는 확인 후 `Ctrl+C`로 종료하세요.

```bash
ros2 topic hz /scan
ros2 topic echo /real_avoid_test/status
```

`PREVIEW`이고 경로가 장애물과 벽을 피하는지 확인합니다. `NO PLAN`이면 장애물 배치, 목표 거리, 점유 지도·DRI 설정을 점검합니다. 시작 조건에는 최근 0.5초 이내 센서 입력, 유효한 경로, 선수 전방 중심선 좌우 1m 통로의 점유 장애물이 포함됩니다. 이미 `/occupancy_grid`가 실행 중이면 `start_occupancy_grid:=false`를 지정합니다. 런처는 RViz용 `base_link → onboard_laser_frame` 원점 일치 정적 TF를 발행합니다. 실제 장착 변환을 별도로 발행한다면 `publish_scan_tf:=false`로 중복을 막으세요. 센서 입력이 끊기면 이전 경로·표식을 지웁니다.
수조 벽도 장애물로 잡힙니다. `d_panic`, `escape_radius`, `lookahead`, 목표 거리, 지도 크기 등 실외 기본값을 그대로 쓰지 말고 수조 실측 크기와 배 크기에 맞춰 조정합니다. 원점 일치 TF는 RViz 표시와 현재 점유 지도 계산의 가정일 뿐이므로, 선체 라이다가 `base_link`에서 떨어져 있으면 장착 변환과 지도 입력 처리도 실측에 맞게 보정해야 합니다.

## 5. 장애물 회피 저속 주행

미리보기를 확인한 뒤 **4절의 테스트 런처를 종료**합니다. 다른 `/cmd_vel` 발행 노드는 함께 켜지 않습니다. 젯슨 터미널 4에서 모터 제어기를 3절과 같이 확인해 드라이버를 시작하고, 젯슨 터미널 5에서 새 터미널 준비 명령 후 주행 모드를 켭니다.

```bash
# 젯슨 터미널 4: 아두이노(CH340) 모터 제어기 시작 (포트: /dev/ttyCH341USB0)
ros2 launch kaboat_hardware thrusters.launch.py hardware_type:=serial port:=/dev/ttyCH341USB0 allow_port_scan:=false
```

```bash
# 젯슨 터미널 5: 주행 모드
ros2 launch kaboat_hardware real_obstacle_avoid_test.launch.py drive_enabled:=true
```

젯슨의 새 터미널에서 준비 명령을 실행하고 상태와 물리적 여유를 확인한 후 출발합니다.

```bash
ros2 service call /real_avoid_test/start std_srvs/srv/Trigger '{}'
```

기본 주행 명령 상한은 전진 0.10, 회전 0.06이고 후진 탈출은 꺼져 있습니다. 센서 입력 끊김, 경로에서 0.7m 이상 이탈, 탈출 후진 요청은 시험을 중지합니다. [`real_obstacle_avoid_test.yaml`](src/kaboat_hardware/config/real_obstacle_avoid_test.yaml)의 `fsm.lookahead`, `fsm.cmd_full`, `fsm.v_full`, `fsm.cmd_exp` 등은 실물에서 조정해야 합니다. 속도→추력 기본 계수는 시뮬레이션 값입니다. 현재 DRI는 경로 점을 검사하며 선체 폭을 포함한 충돌 예측이나 미관측 셀 차단은 하지 않으므로 RViz에서 물리적 여유를 직접 확인합니다.

## 6. 공통 출발·정지·종료

**젯슨의 새 터미널** — 2절의 젯슨 새 터미널 준비 명령을 실행합니다. 출발·정지·비상 정지 명령은 브리지를 통과하지 않으므로 젯슨 도메인 42에서 실행합니다.

```bash
cd /path/to/kaboat_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=42
```

먼저 젯슨에서 `/boat_pose`와 `/odom`이 모두 수신되는지 확인합니다. 노트북의 RViz에서도 `/odom`을 확인합니다. 고정 경로 추종은 아래 출발 신호를 사용합니다. 장애물 회피 주행은 5절의 서비스 호출 또는 같은 출발 신호를 사용합니다.

```bash
ros2 topic echo /boat_pose --once
ros2 topic echo /odom --once
```

**출발:**

```bash
ros2 topic pub --once /start_mission std_msgs/msg/Bool '{data: true}'
```

**일시 정지:**

```bash
ros2 topic pub --once /start_mission std_msgs/msg/Bool '{data: false}'
```

**비상 정지:**

```bash
ros2 topic pub --once /emergency_stop std_msgs/msg/Bool '{data: true}'
```

원인을 해결한 뒤 비상 정지를 해제할 때만 `{data: false}`를 발행합니다. 장애물 회피 시험을 완료한 뒤 다시 시작할 때도 `/start_mission`에 `{data: false}`를 발행해 상태를 재설정합니다. 테스트 종료 후 젯슨의 주행 노드·모터 드라이버·센서 런처와 노트북 라이다·RViz를 각 터미널에서 `Ctrl+C`로 종료합니다. 노트북 호스트에서 다음을 실행합니다.

```bash
xhost -si:localuser:root
cd ~/Projects/KABOAT/kaboat_ws
docker compose -f docker-compose.lidar.yml down
```
