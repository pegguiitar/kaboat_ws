# 외부 TG-50 라이다 노트북: Docker 실행

이 절차는 수조 외벽 노트북에서 `ydlidar_ros2_driver`와
`kaboat_hardware/lidar_boat_tracker`를 실행할 때 사용한다. 배에 탑재하는
Jetson의 센서·모터 설정은 [REAL_HARDWARE.md](REAL_HARDWARE.md)를 따른다.
외벽 라이다는 `/shore/scan`, 선체 라이다는 `/scan`을 발행한다.

## 1. 컨테이너 준비

호스트에서 실행한다. `docker ps`가 권한 오류 없이 동작해야 한다. `docker`
그룹에 이미 등록됐는데 현재 터미널에서만 오류가 나면 `newgrp docker`를
실행한다. 새 로그인 세션에는 그룹이 자동 반영된다.

```bash
cd ~/Projects/KABOAT/kaboat_ws
docker compose -f docker-compose.lidar.yml build
docker compose -f docker-compose.lidar.yml up -d --no-build
docker exec -it kaboat_lidar bash
```

이미지는 ROS 2 Humble과 YDLidar SDK를 포함한다. Compose는 호스트 네트워크를
사용해 Jetson과 DDS 통신을 하고, USB 시리얼 장치가 연결 후에도 보이도록
`/dev`를 공유한다. 기본 `ROS_DOMAIN_ID`는 `42`이며 Jetson과 같아야 한다.
다른 값을 쓰려면 호스트에서 `ROS_DOMAIN_ID=<값> docker compose ... up -d`로
컨테이너를 생성한다.

## 2. 워크스페이스 빌드

아래는 **컨테이너 내부**에서 실행한다. 소스는 호스트와 공유하고 빌드 결과는
Docker 볼륨에 보관한다.

```bash
cd /workspace/kaboat_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to kaboat_hardware
source install/setup.bash
ros2 pkg prefix ydlidar_ros2_driver
ros2 pkg prefix kaboat_hardware
```

## 3. 라이다 연결 후 실행

호스트에서 `ls -l /dev/serial/by-id/`로 TG-50 포트를 확인한다. 모터 관련
명령은 이 노트북에서 실행하지 않는다.
라이다 원점은 수조 오른쪽 벽 중앙 `(10.0, 2.5)m`, 스캔의 0° 방향은
수조 왼쪽 `-X`를 향하도록 장착한다. 이 기준으로
[`lidar_tracker.yaml`](src/kaboat_hardware/config/lidar_tracker.yaml)의
`lidar_yaw_deg`는 `180.0`이다. 장착 방향이 다르면 이 값을 실측 방향에 맞춰
바꾼 뒤 추적기 launch를 재시작한다.

```bash
docker exec -it kaboat_lidar bash
source /workspace/kaboat_ws/install/setup.bash
ros2 launch kaboat_hardware lidar_boat_tracker.launch.py \
  port:=/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0 enable_rviz:=false
```

별도 컨테이너 터미널에서 스캔과 위치를 확인한다.

```bash
docker exec -it kaboat_lidar bash
source /workspace/kaboat_ws/install/setup.bash
ros2 topic hz /shore/scan
ros2 topic echo /boat_pose --once
```

좌표 확인용으로 수조 안쪽 `(9.0, 2.5)m`에 물체를 놓으면 RViz의 원본 스캔도
그 부근에 나타나야 한다. 선체 중심이 `(5.0, 2.5)m`이고 선수가 `+X`를
향하면 길이 0.60m의 횡단 판은 대략 `(5.0, 2.2)m`부터 `(5.0, 2.8)m`까지
보인다. `/boat_pose`의 위치는 판의 중점이다. **판만으로 선수의 앞뒤를 구분할
수 없으므로**, `/boat_pose` yaw는 180° 모호하다. Jetson의 IMU 초기 보정과
연속 추적이 실제 방향을 정한다.

## 4. RViz 창 열기

위 라이다 launch는 `enable_rviz:=false`로 실행했으므로 RViz 창을 따로 연다.
**노트북의 그래픽 데스크톱 터미널**에서 다음 명령을 실행한다. SSH 터미널이면
창이 노트북 화면에 나타나지 않을 수 있다.

```bash
# 호스트: 컨테이너의 root 사용자에게 현재 X11 화면 접근 허용
xhost +si:localuser:root

# 호스트: 현재 DISPLAY 값을 넘겨 기존 컨테이너에서 RViz만 실행
docker exec -it -e DISPLAY="$DISPLAY" kaboat_lidar bash -lc \
  'source /workspace/kaboat_ws/install/setup.bash && ros2 run rviz2 rviz2 -d /workspace/kaboat_ws/install/kaboat_bringup/share/kaboat_bringup/rviz/tank_tracking.rviz'
```

라이다 launch는 첫 번째 컨테이너 터미널에서 계속 실행한다. RViz를 종료할
때는 두 번째 터미널에서 `Ctrl+C`를 누른다. `Authorization required` 또는
`could not connect to display`가 보이면 `xhost`를 **호스트의 그래픽 데스크톱
터미널**에서 실행했는지와 `echo "$DISPLAY"`가 비어 있지 않은지 확인한다.
화면 접근 권한을 해제하려면 호스트에서 `xhost -si:localuser:root`를 실행한다.

## 종료

```bash
docker compose -f docker-compose.lidar.yml down
```

`down`은 빌드 결과가 담긴 이름 붙은 볼륨을 지우지 않는다.
