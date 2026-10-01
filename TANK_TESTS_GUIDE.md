# 실내 수조 실물 테스트: 터미널별 실행 명령

수조 외벽 노트북에는 **고정 TG-50**, 배 위 젯슨에는 **GQ7과 모터 제어기**를 연결합니다. 선체 TG-50을 장착하지 않으면 젯슨 라이다는 기본값으로 꺼집니다. 두 기기는 같은 네트워크에 연결하고 `ROS_DOMAIN_ID=42`를 사용합니다.

고정 TG-50의 기본 위치는 수조 절대좌표 `(10.0, 2.5)m`(오른쪽 벽 중앙)이며,
스캔 0° 방향이 `-X`를 바라보도록 장착합니다. 좌표와 각도는
[`lidar_tracker.yaml`](src/kaboat_hardware/config/lidar_tracker.yaml)에서 설정합니다.
배에는 좌우 두 봉을 잇는 길이 0.60m의 횡단 판을 부착하고, 그 중점이
`base_link` 원점과 일치해야 합니다. 추적기는 판의 선분 기울기와 중점으로
`/boat_pose`를 계산합니다.

`/dev/ttyUSB0` 같은 번호는 연결 순서에 따라 바뀔 수 있습니다. 각 기기에서 포트를 확인해 입력합니다. 장치가 고유한 USB 식별자를 제공한다면 `/dev/serial/by-id/...` 경로를 우선 사용하세요. 이 경로가 없으면 `/dev/serial/by-path/...` 또는 현재의 `/dev/ttyUSB*`를 확인해 입력합니다.

## 1. 수조 외벽 노트북: 라이다와 배 위치 추적

**호스트 터미널 1** — Docker 시작:

```bash
cd ~/Projects/KABOAT/kaboat_ws
docker compose -f docker-compose.lidar.yml up -d --build
docker exec -it kaboat_lidar bash
```

**같은 터미널, Docker 내부** — 코드 빌드 후 고정 라이다 실행:

```bash
cd /workspace/kaboat_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to kaboat_hardware
source install/setup.bash
ls -l /dev/serial/by-id/ /dev/serial/by-path/ /dev/ttyUSB* 2>/dev/null
printf '고정 TG-50 포트 경로: '
read -r LIDAR_PORT
ros2 launch kaboat_hardware lidar_boat_tracker.launch.py port:="$LIDAR_PORT" enable_rviz:=false
```

이 터미널은 켜 둡니다. 고정 라이다는 `/shore/scan`, 배 위치 추적기는 `/boat_pose`를 발행합니다.
라이다를 옮긴 뒤에는 기존 추적기 프로세스를 종료하고 이 명령으로 다시 실행해야 새 설치 좌표가 적용됩니다.

**호스트의 그래픽 데스크톱 터미널 2** — RViz 창을 별도로 엽니다. 컨테이너의
root 사용자에게 X11 화면 접근을 허용해야 합니다. SSH 터미널이 아닌 노트북
화면에서 실행합니다.

```bash
xhost +si:localuser:root
docker exec -it -e DISPLAY="$DISPLAY" kaboat_lidar bash -lc \
  'source /workspace/kaboat_ws/install/setup.bash && ros2 run rviz2 rviz2 -d /workspace/kaboat_ws/install/kaboat_bringup/share/kaboat_bringup/rviz/tank_tracking.rviz'
```

RViz를 닫은 뒤 화면 접근 권한을 해제하려면 호스트에서
`xhost -si:localuser:root`를 실행합니다. 자세한 GUI 오류 점검은
[LIDAR_DOCKER.md](LIDAR_DOCKER.md)의 "RViz 창 열기"를 참고하세요.

## 2. 젯슨: 센서와 오도메트리

젯슨에 코드를 새로 받았거나 수정했다면 **처음 한 번** 빌드합니다. 첫 줄의 경로는 젯슨에 클론한 워크스페이스 경로로 바꾸세요.

```bash
cd /path/to/kaboat_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select kaboat_hardware
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
ros2 launch kaboat_hardware indoor_tank.launch.py enable_thrusters:=false enable_onboard_lidar:=false onboard_lidar_port:="$ONBOARD_LIDAR_PORT"
```

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

**젯슨 터미널 3** — 새 터미널 준비 명령을 실행한 뒤 모터 제어기 시작. 포트를 **모터 제어기 장치**로 바꾸세요.

```bash
ros2 launch kaboat_hardware thrusters.launch.py hardware_type:=serial port:=/dev/ttyUSB0 allow_port_scan:=false
```

**젯슨 터미널 4** — 새 터미널 준비 명령을 실행한 뒤 B-Spline 주행 노드 시작. 출발 명령 전까지 대기합니다.

```bash
ros2 launch kaboat_hardware bspline_track_test.launch.py wait_for_start:=true
```

다른 코스를 시험할 때는 터미널 4에서 위 명령 대신 `ros2 launch kaboat_hardware tank_tests.launch.py test:=straight`처럼 실행합니다. `test` 값은 `straight`, `bspline`, `circle`, `station_keeping` 중 하나입니다. **주행 테스트 노드는 한 번에 하나만** 실행합니다.

## 4. 수조 외벽 노트북: 출발·정지

**호스트 터미널 2** — Docker 셸에 들어갑니다.

```bash
docker exec -it kaboat_lidar bash
source /opt/ros/humble/setup.bash
source /workspace/kaboat_ws/install/setup.bash
```

먼저 두 기기가 통신하는지 확인합니다. `/boat_pose`와 `/odom`이 모두 수신되어야 출발합니다.

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

원인을 해결한 뒤 비상 정지를 해제할 때만 `{data: false}`를 발행합니다. 테스트 종료 후 젯슨의 주행 노드와 모터 드라이버를 `Ctrl+C`로 종료하고, 노트북의 라이다 launch를 종료합니다.
