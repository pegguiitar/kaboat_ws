# competition 기반 도킹 통합 실행 가이드

기준: `origin/competition` → `feature/dock-real_test`. 아래 Docker 절차는 **Gazebo
시뮬레이션 컨테이너 전용**이다. 기존 Docker 이미지 태그
`ghcr.io/pegguiitar/kaboat_ws:latest`, 컨테이너 이름 `kaboat_sim_container`,
`docker compose` 명령은 유지한다. Dockerfile에 CPU PyTorch와 YOLO 의존성만 추가했다.

## 미션 흐름

`mission_manager`가 기록된 `dock_entry`에 접근해 `mission_approach`의 도착
신호를 받으면 `/mission/state=dock`을 발행한다. YOLO가 켜지고
`dock_ctrl_v2`가 이전 경로를 지우고 `SCAN`으로 시작한다. 이때 360도 회전하지
않는다. 카메라에서 세 마커가 안정적으로 검출되고 LiDAR의 세 독립 RANSAC
법선이 안정적으로 나올 때, 관측 법선에 맞춰 선수각을 미세 조정한다.
안정화된 순간 GPS와 LiDAR로 시작점→경유점→정지점 LOS 경로를 **한 번만**
고정한다. 목표 슬롯 뒷벽 앞 3 m가 경유점, 1 m가 정지점이다. 경유점에서
정지점 방향으로 선수를 정렬하고 진입한다. 정지 후 `/mission/complete=dock`을
보내고, 미션 매니저는 고정 경로의 최종점으로 완료를 판정한다.

센서와 제어 토픽은 실선과 시뮬레이션에서 같다:

```text
/camera/color/image_raw + /camera/camera_info + /camera/depth/image_raw
  → /dock/markers/{triangle,circle,square}/ray + /dock/detection_status
/scan/raw → onboard_scan_filter → /scan/level
  → /dock/lidar/status + /dock/lidar/{class}/normal + /dock/lidar/{class}/wall_point
/gps/fix + /odom + /dock/waypoints → dock_ctrl_v2
  → /dock/one_shot_path + /dock/one_shot_gps + /cmd/dock
/mission/state=dock → cmd_mux: /cmd/dock → /cmd_vel
```

실선에서는 기존 센서, GQ7 위치추정, 실제 추력 드라이버를 따로 실행한다.
시뮬레이션에서는 `simulation.launch.py`가 Gazebo 센서 브리지,
`onboard_scan_filter`, `/odom` 및 TF를 실행하며 도킹 런치에
`use_sim_actuator:=true`를 넘겨 `/cmd_vel`을 Gazebo 추력으로 바꾼다.

## 시뮬레이션 전용 이미지 및 워크스페이스 준비

다른 Gazebo/YOLO 컨테이너를 종료하고 메모리 여유를 확인한 뒤 **호스트에서**:

```bash
cd /home/eugene/Data/MAIN_KABOAT/kaboat_dock_real_test
docker compose build kaboat_sim_env
docker compose create kaboat_sim_env
docker update --cpus 5 --memory 4g --memory-swap 5g kaboat_sim_container
docker compose start kaboat_sim_env
docker exec -it kaboat_sim_container bash
```

이미지는 이 브랜치의 Dockerfile로 **한 번** 빌드한다. CPU PyTorch 휠을
다운로드하므로 네트워크와 디스크 공간이 필요하다. 기존 이미지가 있다면
`docker compose build` 없이도 컨테이너는 시작하지만 YOLO 의존성이 부족하다.
같은 이름의 기존 컨테이너가 있다면 중복 생성하지 말고 `docker update`만
적용한다. 빌드 과정에서 노트북의 다른 Gazebo/YOLO 작업을 함께 돌리지 않는다.
Gazebo 기반 환경은 `competition`에 원래 있던 구성을 그대로 사용한다.
이 브랜치의 Dockerfile 추가분은 YOLO와 PyTorch 의존성뿐이다.
VRX/ros_gz가 필요한 기존 시뮬레이션 초기 설치는 저장소 `README.MD` 절차를 따른다.

컨테이너 안에서 최초 1회, 또는 이 브랜치의 ROS 소스가 바뀐 뒤:

```bash
cd /workspace/kaboat_ws
./scripts/build_docking_competition_safe.sh --check-only
./scripts/build_docking_competition_safe.sh
source install/setup.bash
python3 -c 'import torch, ultralytics; print(torch.__version__, ultralytics.__version__)'
```

이후 새 컨테이너 터미널에서는 `source /opt/ros/humble/setup.bash`와
`source /workspace/kaboat_ws/install/setup.bash`만 하면 된다. `.bashrc`도 이를
자동 실행한다. 이미지와 `install/`은 다른 것이므로, 이미지 갱신 후에도
새 소스의 ROS 패키지는 다시 빌드해야 한다.

## Gazebo와 도킹 실험

호스트에서 `xhost +local:`를 실행해 X11 접근을 허용한 다음, 각 명령을
**별도 컨테이너 터미널**에서 실행한다. 설치 경로를 먼저 source한다.

터미널 1: 배를 도킹 스테이션보다 뒤쪽 `(58,66)`에 생성한다. GUI는 배가
처음 보이는 카메라 위치를 사용하며 파도는 꺼져 있다.

```bash
cd /workspace/kaboat_ws
./scripts/demo/sim_dock.sh
```

Gazebo 창이 뜬 뒤 왼쪽 아래 재생(▶) 버튼으로 물리를 시작한다.

터미널 2: YOLO + RANSAC + GPS 고정 경로 + competition 미션 FSM. 실제
시험과 같은 `real_docking_test.launch.py`를 사용한다. 아래 예는 가운데
삼각형 슬롯이다. 시뮬레이터의 선체 TF 이름만 `wamv/base_link`로 지정한다.

```bash
ros2 launch kaboat_bringup real_docking_test.launch.py \
  use_sim_time:=true use_sim_actuator:=true \
  staging_x:=58.0 staging_y:=66.0 staging_angle_deg:=0.0 \
  dock_target_class:=triangle dock_base_frame:=wamv/base_link \
  approach_motion_enabled:=true dock_motion_enabled:=true
```

목표 도형은 `dock_target_class:=circle` 또는 `:=square`로 바꾼다. 각
도형에서 `dock/lidar/status`의 세 마커 법선 상태와 `/dock/status`의
`route_fixed`를 확인한다. `dock_motion_enabled:=false`로 먼저 경로만 볼 수
있지만, 그 설정에서는 실제 이동이나 도킹 완료를 기대하면 안 된다.

터미널 3: RViz 표시:

```bash
cd /workspace/kaboat_ws
./scripts/demo/rviz_dock.sh
```

RViz에는 세 카메라 방위, 세 RANSAC 법선, 경유·정지점, 고정 LOS **선**이
표시된다. 상태와 측정치는 다른 터미널에서:

```bash
ros2 topic echo /mission/state
ros2 topic echo /dock/status
ros2 topic echo /dock/detection_status
ros2 topic echo /dock/lidar/status
ros2 topic echo /dock/avoid/goal_status
ros2 topic echo /dock/one_shot_gps
ros2 topic echo /dock/one_shot_path
```

`/dock/one_shot_path`는 안정 조건을 만족해 경로를 고정하기 전에는 비어
있다. 도킹 중 다시 탐색하려면 `ros2 service call /dock/start std_srvs/srv/Trigger
{}`를 호출한다. 이 명령은 저장된 경로와 안정화 카운트를 모두 초기화한다.
긴급 중단은 `ros2 service call /dock/stop std_srvs/srv/Trigger {}`이다.

## 실선 적용

실제 배의 Jetson 실행에는 VRX, `ros_gz`, Gazebo Garden, 시뮬레이션 Docker
이미지가 필요 없다. 저장소의 `Dockerfile.lidar`/`kaboat_lidar`는 **노트북 외벽
LiDAR 추적용**이며, 배의 YOLO 도킹 노드용 컨테이너가 아니다. 그러므로
그 이미지에도 VRX나 YOLO를 추가하지 않는다. Jetson에는 기존 ROS 2 Humble
환경에서 `kaboat_msgs`, `kaboat_behaviors`, `kaboat_perception`,
`kaboat_hardware`, `kaboat_control`, `kaboat_mission`, `kaboat_bringup`을
`./scripts/build_docking_real_safe.sh`로 순차 빌드한다. 이 스크립트는
`kaboat_sim`, VRX, ros_gz를 선택하지 않는다. Jetson에 맞는 PyTorch와
`requirements-yolo.txt`의 Ultralytics를 같은 Python 환경에 설치하고
`python3 -c 'import torch, ultralytics'`로 먼저 확인한다.

센서와 위치추정 bringup 후 `/gps/fix`, `/odom`, `/scan/level`, 카메라 토픽,
`odom→base_link` TF를 확인한다. 현장 기준 원점을 설정하고 `rec` 도구로
`dock_entry`를 기록한다. `dock_end`는 새 경로 계산에 필요하지 않다.
기존 전체 미션 명령 `ros2 launch kaboat_bringup autonomy.launch.py`를 쓰면
도킹 웨이포인트 접근 완료 후 같은 도킹 파이프라인으로 넘어간다. 실제
구동 전 `approach_motion_enabled`, `dock_motion_enabled`와 실제 추력 드라이버
연결을 현장 조건에 맞게 활성화한다. `use_sim_actuator`는 실선에서 `false`다.

단일 도킹 미션 시험은 같은 런치에서 `use_sim_time:=false`,
`use_sim_actuator:=false`, 실측 `staging_x/y/angle_deg`를 넘긴다. 실선 기본
`dock_base_frame:=base_link`를 사용한다. `retry_*` 인자는 one-shot 제어에서
지원하지 않으며 값이 설정되면 시작을 거부한다. `slot_config`는 이전 CLI
호환용으로만 남았다.

## 확인 상태

Python 순수 로직 및 mission manager ROS 테스트 20개 통과. 새 Docker 이미지
빌드, 전체 ROS 패키지 빌드, YOLO 실추론, Gazebo 종단 도킹 주행은 아직 검증되지
않았다. 이미지가 빌드되면 위 명령으로 세 도형을 각각 시험하고 토픽 및
최종 정지 오차를 기록한다.
