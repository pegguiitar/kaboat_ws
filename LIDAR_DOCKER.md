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

```bash
docker exec -it kaboat_lidar bash
source /workspace/kaboat_ws/install/setup.bash
ros2 launch kaboat_hardware lidar_boat_tracker.launch.py \
  port:=/dev/serial/by-id/<TG-50-장치명> enable_rviz:=false
```

별도 컨테이너 터미널에서 스캔과 위치를 확인한다.

```bash
docker exec -it kaboat_lidar bash
source /workspace/kaboat_ws/install/setup.bash
ros2 topic hz /shore/scan --qos-reliability best_effort
ros2 topic echo /boat_pose --once
```

RViz를 켜려면 호스트에서 컨테이너의 X11 접속을 허용한 뒤 launch 인자의
`enable_rviz:=true`를 사용한다. 기본 실행은 GUI 접속 문제와 분리하기 위해
RViz를 끈다.

## 종료

```bash
docker compose -f docker-compose.lidar.yml down
```

`down`은 빌드 결과가 담긴 이름 붙은 볼륨을 지우지 않는다.
