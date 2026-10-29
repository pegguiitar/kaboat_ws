# 실내 수조 실물 테스트: 노트북·젯슨 실행 명령

이 문서에 실내 수조 시험의 실행 명령을 모았습니다. 수조 외벽 노트북에는 **고정 TG-50**, 배 위 젯슨에는 **GQ7과 모터 제어기**를 연결합니다. 장애물 회피 시험에는 **선체 TG-50**도 연결합니다. 두 기기는 같은 네트워크에 연결하고 `ROS_DOMAIN_ID=42`를 사용합니다.

고정 TG-50의 기본 위치는 수조 절대좌표 `(10.0, 2.5)m`(오른쪽 벽 중앙)이며,
스캔 0° 방향이 `-X`를 바라보도록 장착합니다. 좌표와 각도는
[`lidar_tracker.yaml`](src/kaboat_hardware/config/lidar_tracker.yaml)에서 설정합니다.
배에는 좌우 두 봉을 잇는 길이 0.60m의 횡단 판을 부착하고, 그 중점이
`base_link` 원점과 일치해야 합니다. 추적기는 판의 선분 기울기와 중점으로
`/boat_pose`를 계산합니다.

외벽 노트북의 고정 TG-50은 확인된 `/dev/serial/by-id/...` 경로를, 젯슨의 아두이노 CH340 모터 제어기는 최신 가이드의 `/dev/ttyCH341USB0`을 아래 명령에 직접 지정했습니다. 선체 TG-50 포트는 젯슨에서 따로 확인해 입력하세요. 장치나 드라이버가 바뀌면 해당 기기에서 `/dev/serial/by-id/`, `/dev/serial/by-path/`, `/dev/ttyUSB*`, `/dev/ttyCH341*`를 다시 확인해야 합니다.

## 1. 수조 외벽 노트북: 라이다와 배 위치 추적

**호스트 터미널 1** — Docker 시작. Docker 권한이 현재 터미널에만 반영되지 않았다면 `newgrp docker` 후 재실행합니다. Compose는 호스트 네트워크와 `/dev`를 공유하며 기본 Domain ID가 42입니다. 다른 값을 쓴다면 `ROS_DOMAIN_ID=<값> docker compose -f docker-compose.lidar.yml up -d --build`로 생성하고 젯슨에도 같은 값을 설정합니다.

```bash
cd ~/Projects/KABOAT/kaboat_ws
docker compose -f docker-compose.lidar.yml up -d --build
docker exec -it kaboat_lidar bash
```

**같은 터미널, Docker 내부** — 코드 빌드 후 고정 라이다 실행. 이미 빌드했고 소스가 바뀌지 않았다면 `colcon build`를 생략할 수 있습니다.

```bash
cd /workspace/kaboat_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to kaboat_hardware
source install/setup.bash
ros2 launch kaboat_hardware lidar_boat_tracker.launch.py \
  port:=/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0 \
  enable_rviz:=false
```

이 터미널은 켜 둡니다. 고정 라이다는 `/shore/scan`, 배 위치 추적기는 `/boat_pose`를 발행합니다.
라이다를 옮긴 뒤에는 기존 추적기 프로세스를 종료하고 이 명령으로 다시 실행해야 새 설치 좌표가 적용됩니다.
`ros2 topic hz /shore/scan`과 `ros2 topic echo /boat_pose --once`로 확인할 수 있습니다. 수조 `(9.0, 2.5)m`에 놓은 물체가 RViz 스캔에서 그 부근에 보이는지도 확인합니다.

**호스트의 그래픽 데스크톱 터미널 2** — 고정 경로 추종용 RViz 창을 별도로 엽니다. 컨테이너의
root 사용자에게 X11 화면 접근을 허용해야 합니다. SSH 터미널이 아닌 노트북
화면에서 실행합니다.

```bash
xhost +si:localuser:root
docker exec -it -e DISPLAY="$DISPLAY" kaboat_lidar bash -lc \
  'source /workspace/kaboat_ws/install/setup.bash && ros2 run rviz2 rviz2 -d /workspace/kaboat_ws/install/kaboat_bringup/share/kaboat_bringup/rviz/tank_tracking.rviz'
```

장애물 회피 경로를 볼 때는 노트북의 **새 그래픽 데스크톱 터미널**에서 다음 전용 화면을 엽니다. 두 RViz 창은 동시에 실행할 수 있습니다.

```bash
docker exec -it -e DISPLAY="$DISPLAY" kaboat_lidar bash -lc \
  'source /workspace/kaboat_ws/install/setup.bash && ros2 launch kaboat_hardware real_obstacle_avoid_rviz.launch.py'
```

화면 연결 오류가 나면 **노트북 그래픽 데스크톱 터미널**에서 `echo "$DISPLAY"`가 비어 있지 않은지, `xhost`를 실행했는지 확인합니다. RViz를 닫은 뒤 접근 권한을 해제하려면 `xhost -si:localuser:root`를 실행합니다.

## 2. 젯슨: 센서와 오도메트리

젯슨에 코드를 새로 받았거나 수정했다면 **처음 한 번** 빌드합니다. 첫 줄의 경로는 젯슨에 클론한 워크스페이스 경로로 바꾸세요.

```bash
cd /path/to/kaboat_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to kaboat_hardware
```

젯슨의 **새 터미널마다** 먼저 아래 명령을 실행합니다. 빌드한 터미널에서도 실행하세요.

```bash
cd /path/to/kaboat_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0
```

**젯슨 터미널 1** — GQ7과 실내 오도메트리 시작. 선체 라이다와 모터는 아직 켜지 않습니다.

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

선체 라이다 드라이버 원본은 `/scan/raw`, 지도·회피·RViz가 사용하는 필터 결과는 `/scan`입니다. 배 구조물이 보이는 220°~320°(ROS -140°~-40°) 값은 `/scan`에서 `inf`로 무효화하고 `/shore/scan`은 그대로 둡니다. 장애물 회피 RViz에서 기본으로 꺼진 `/scan/raw` 레이어를 켜 실제 구조물 방향과 비교하세요. 필요하면 `indoor_tank.launch.py`에 `onboard_lidar_mask_start_deg:=... onboard_lidar_mask_end_deg:=...`를 지정합니다. 이전 구조물 셀이 점유 지도에 남아 있다면 장애물 시험 런처도 재시작합니다.
선체 TG-50의 0.40m 최소 거리 컷오프는 센서 원점 기준입니다. 구조물이 더 멀리 보이면 장착 위치를 실측하고 [`tg50_onboard.yaml`](src/kaboat_hardware/config/tg50_onboard.yaml)의 `range_min`과 `base_link → onboard_laser_frame` 변환을 확인하세요.

## 3. 젯슨: IMU 보정과 모터·테스트 시작

선수를 수조 좌표계의 **-X 방향(180°)**으로 두고 배를 움직이지 마세요.
외벽 라이다에 횡단 판이 충분한 길이의 한 선분으로 보여야 합니다. 판만으로는
선수 앞뒤를 구분할 수 없어, 이 자세에서 IMU 초기 보정으로 방향을 확정합니다.

**젯슨 터미널 2** — 위의 젯슨 새 터미널 준비 명령을 실행한 뒤 IMU 보정:

```bash
ros2 run kaboat_hardware calibrate_indoor_imu
ros2 topic hz /odom
```

보정 성공 메시지와 `/odom` 발행을 확인하고 `ros2 topic hz`는 `Ctrl+C`로 종료합니다. 보정 실패 시 모터를 시작하지 마세요. GQ7 또는 `indoor_tank.launch.py`를 재시작했다면 다시 보정합니다.
`ros2 topic info /odom --verbose`의 `Publisher count`도 **1**인지 확인합니다. 실내에서는 `real_sensors.launch.py`를 별도로 실행하지 않습니다.

**젯슨 터미널 3** — 새 터미널 준비 명령을 실행한 뒤 모터 제어기 시작. 아두이노(CH340) 모터 제어기 포트는 최신 가이드에 지정된 `/dev/ttyCH341USB0`입니다. 젯슨에서 해당 장치가 실제로 등록됐는지 확인하세요.

```bash
ls -l /dev/ttyCH341USB0
ros2 launch kaboat_hardware thrusters.launch.py hardware_type:=serial port:=/dev/ttyCH341USB0 allow_port_scan:=false
```

**젯슨 터미널 4** — 새 터미널 준비 명령을 실행한 뒤 B-Spline 주행 노드 시작. 출발 명령 전까지 대기합니다.

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

다른 코스를 시험할 때는 터미널 4에서 위 명령 대신 `ros2 launch kaboat_hardware tank_tests.launch.py test:=straight`처럼 실행합니다. `test` 값은 `straight`, `bspline`, `circle`, `station_keeping` 중 하나입니다. **주행 테스트 노드는 한 번에 하나만** 실행합니다.
[`tank_tests.yaml`](src/kaboat_hardware/config/tank_tests.yaml)은 현재 런처가 자동으로 읽지 않으므로 실제 노드 기본값과 런처 인자를 확인해 조정합니다.

## 4. 장애물 회피 경로 생성과 RViz 미리보기

**2절의 선체 TG-50 포함 명령**으로 `/scan`과 `/odom`을 먼저 준비합니다. 모터 드라이버와 다른 주행 테스트 노드는 켜지 않습니다. 젯슨 터미널 3에서 새 터미널 준비 명령을 실행한 뒤 아래 런처를 시작합니다.

```bash
ros2 launch kaboat_hardware real_obstacle_avoid_test.launch.py
```

기본 `drive_enabled:=false`에서는 `/cmd_vel`을 발행하지 않습니다. `/scan` → 점유 지도 → DRI → B-Spline 생성·검사 → `AvoidFsm`을 실행하고 경로와 제어 미리보기를 발행합니다. 목표는 출발 순간 선수 방향 `goal_distance`(기본 3m) 앞에 고정되며, 지도 변화에 따라 경로를 갱신합니다. 노트북에서는 **1절의 장애물 회피 전용 RViz**를 엽니다. Fixed Frame은 `odom`입니다.

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

미리보기를 확인한 뒤 **4절의 테스트 런처를 종료**합니다. 다른 `/cmd_vel` 발행 노드는 함께 켜지 않습니다. 젯슨 터미널 3에서 모터 제어기를 3절과 같이 확인해 드라이버를 시작하고, 젯슨 터미널 4에서 새 터미널 준비 명령 후 주행 모드를 켭니다.

```bash
# 젯슨 터미널 3: 아두이노(CH340) 모터 제어기 시작 (포트: /dev/ttyCH341USB0)
ros2 launch kaboat_hardware thrusters.launch.py hardware_type:=serial port:=/dev/ttyCH341USB0 allow_port_scan:=false
```

```bash
# 젯슨 터미널 4: 주행 모드
ros2 launch kaboat_hardware real_obstacle_avoid_test.launch.py drive_enabled:=true
```

젯슨 터미널 5에서 새 터미널 준비 명령을 실행하고 상태와 물리적 여유를 확인한 후 출발합니다.

```bash
ros2 service call /real_avoid_test/start std_srvs/srv/Trigger '{}'
```

기본 주행 명령 상한은 전진 0.10, 회전 0.06이고 후진 탈출은 꺼져 있습니다. 센서 입력 끊김, 경로에서 0.7m 이상 이탈, 탈출 후진 요청은 시험을 중지합니다. [`real_obstacle_avoid_test.yaml`](src/kaboat_hardware/config/real_obstacle_avoid_test.yaml)의 `fsm.lookahead`, `fsm.cmd_full`, `fsm.v_full`, `fsm.cmd_exp` 등은 실물에서 조정해야 합니다. 속도→추력 기본 계수는 시뮬레이션 값입니다. 현재 DRI는 경로 점을 검사하며 선체 폭을 포함한 충돌 예측이나 미관측 셀 차단은 하지 않으므로 RViz에서 물리적 여유를 직접 확인합니다.

## 6. 공통 출발·정지·종료

**노트북 호스트의 새 터미널** — Docker 셸에 들어갑니다.

```bash
docker exec -it kaboat_lidar bash
source /opt/ros/humble/setup.bash
source /workspace/kaboat_ws/install/setup.bash
```

먼저 두 기기가 통신하는지 확인합니다. `/boat_pose`와 `/odom`이 모두 수신되어야 출발합니다. 고정 경로 추종은 아래 출발 신호를 사용합니다. 장애물 회피 주행은 5절의 서비스 호출 또는 같은 출발 신호를 사용합니다.

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
