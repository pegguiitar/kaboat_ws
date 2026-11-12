# Gate 벽 추정 — 실제 rosbag 센서 + RViz 실행

## 이 실행에서 보이는 것

원본 rosbag의 센서를 재생하고 다음 노드를 **실시간으로 실행**한다.

```text
rosbag 센서 재생
  ├─ /scan, /odom, /imu/data → occupancy_grid → 점유지도
  ├─ /scan/level, /odom, RGB, camera_info, IMU → buoy_lidar_detector → 부표
  ├─ /scan, /odom → flat_scan_view → 지도 평면의 LiDAR·배 표시
  └─ 위 출력 → 기존 gate + 실험 벽 추정 → 세 경로 → 실제 RViz 창
```

| RViz 표시 | 토픽 | 의미 |
|---|---|---|
| Occupancy | `/occupancy_grid` | 최신 코드로 다시 계산한 점유지도 |
| LiDAR `/scan/level` | `/scan/level` | 기록된 원본 수평 스캔 |
| LiDAR scan on map | `/scan/flat` | 같은 스캔을 odom 바닥면에 표시 |
| Odometry | `/odom/flat` | 바닥면의 배 위치·선수 방향 |
| BuoyMap | `/buoy_lidar_detector/markers` | 실시간 검출·추적한 부표 |
| Camera | `/buoy_lidar_detector/debug_image` | 현재 RGB의 후보 창·색 관측 디버그 |
| Competition gate path, **흰색** | `/gate/path` | 기존 competition gate 경로 |
| NEW buoy-wall path, **주황색** | `/wall_corridor/detector/path` | 부표 검출기 입력의 벽 기반 중앙 경로 |
| NEW grid-wall path, **하늘색** | `/wall_corridor/grid/path` | 점유지도 입력의 벽 기반 중앙 경로 |
| NEW left-right walls | `/wall_corridor/markers` | 좌우 벽 가설과 상태 |

색상은 두 벽의 좌우 분류에 사용하지 않는다. 다만 현재 부표 검출기의 기본 출력은 카메라 관측 여부를 사용한다. 알고리즘 비교와 수치는 [분석 문서](GATE_WALL_EXPERIMENT.md)를 참조한다.

## 준비

- ROS 2 Humble, RViz, Python numpy/scipy/OpenCV, `rosbag2_py`, `kaboat_msgs`가 준비된 환경.
- 저장소의 일반 빌드·환경 설정은 [README](../README.MD) 참조. 기존 ROS 컨테이너 안에서 실행해도 된다.
- 원본 bag `run_20261112_152344/`의 `metadata.yaml`과 SQLite 파일 3개가 같은 폴더에 있어야 한다.
- GUI 환경에서 실행한다. 컨테이너라면 저장소의 일반 X11 설정이 필요하다.

## 실제 센서 재생 실행

저장소 루트에서 실행한다. 아래 첫 번째 경로는 장비의 실제 bag 위치로 바꾼다.

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash

GATE_WALL_BAG=/실제/경로/run_20261112_152344
GATE_WALL_RUN=$(mktemp -d /tmp/gate-wall-replay.XXXXXX)

python3 scripts/experiments/gate_walls/live_sensor_bag.py slice \
  "$GATE_WALL_BAG" --out "$GATE_WALL_RUN/bag"

bash scripts/experiments/gate_walls/live_view_session.sh "$GATE_WALL_RUN/bag"
```

`slice`는 원본을 읽기 전용으로 열어 **2026-11-12 15:31:05~15:33:10 KST** 구간의 센서·TF만 별도 bag으로 저장한다. 이번 기록에서는 약 234MiB, 25,025개 메시지다. 전체 5GB bag은 저장소에 포함하지 않는다.

`live_view_session.sh`는 이 구간을 1배속으로 반복 재생하고 **사용자 화면에 RViz 창을 띄운다.** 경로가 보이려면 시작 후 부표 관측과 최초 쌍 검증이 쌓일 시간이 필요하다. 노드 로그 경로는 시작할 때 출력한다.

이 실행은 해당 기록 전용이다. 구간 시각은 `live_sensor_bag.py`에, 목표점 `(131.569, -22.083)`은 `live_wall_path.py`에 들어 있다. 다른 bag을 시험할 때는 두 값을 함께 바꿔야 한다. 실시간 벽 거리의 방향 기준은 `waypoint`다.

다른 작업 폴더의 설치 결과를 사용하는 경우:

```bash
KABOAT_INSTALL_SETUP=/workspace/kaboat_ws/install/setup.bash \
  bash scripts/experiments/gate_walls/live_view_session.sh "$GATE_WALL_RUN/bag"
```

RViz 창을 닫거나 실행 터미널에서 Ctrl+C를 누르면 이 실행에서 시작한 자식 노드에 종료 신호를 보낸다. 기존에 별도로 실행한 노드는 종료하지 않는다.

## 경로와 상태 읽기

- 흰 경로는 기존 gate 알고리즘의 출력이다. `gate.motion_enabled=false`로 실행한다.
- 주황·하늘 경로는 새 벽 추정기의 **표시용 중앙 경로**다. 별도 주행 제어기에 연결하지 않았다.
- 최초 한 쌍만으로 생긴 직선이나 `WARMUP_OR_UNSUPPORTED` 상태에서도 짧은 가설 선을 표시할 수 있다. `TRACK`은 양측 벽 형상 조건을 만족했다는 뜻이며, 충돌 안전이나 주행 허가를 의미하지 않는다.
- 반복 재생으로 배가 시작 위치로 이동하면 벽 추정과 gate 진행 상태를 초기화한다.
- 반복마다 원본 메시지 바이트를 새로 역직렬화하고 timestamp를 앞으로 옮긴다. 시간이 거꾸로 흐르거나 같은 메시지에 시간 보정이 중복 누적되지 않게 했다.

재생 토픽은 스캔·odom·IMU·카메라·TF로 제한한다. 기록된 `/cmd_vel`·미션 제어·기존 경로는 재생하지 않는다. 비교 노드가 해당 기록의 목표와 `gate` 상태를 별도로 발행한다. `cmd_mux`·모터 드라이버는 실행하지 않는다. 기본 도메인은 **97**, 통신은 로컬로 제한한다.

## 표시가 없을 때

먼저 같은 도메인에서 입력과 출력이 있는지 확인한다.

```bash
ROS_DOMAIN_ID=97 ROS_LOCALHOST_ONLY=1 ros2 node list
ROS_DOMAIN_ID=97 ROS_LOCALHOST_ONLY=1 ros2 topic info /occupancy_grid
ROS_DOMAIN_ID=97 ROS_LOCALHOST_ONLY=1 ros2 topic info /buoy_lidar_detector/markers
ROS_DOMAIN_ID=97 ROS_LOCALHOST_ONLY=1 ros2 topic info /gate/path
ROS_DOMAIN_ID=97 ROS_LOCALHOST_ONLY=1 ros2 topic info /wall_corridor/detector/path
ROS_DOMAIN_ID=97 ROS_LOCALHOST_ONLY=1 ros2 topic info /wall_corridor/grid/path
```

- **센서만 보이고 경로가 없음:** RViz의 세 Path 표시가 켜져 있는지와 최초 쌍이 검증됐는지 확인한다. `gate.log`, `walls.log`의 오류·상태를 본다.
- **부표가 없음:** RGB·camera_info·odom·`/scan/level`이 함께 들어오는지 확인한다. 기본 검출기는 카메라에 관측되지 않은 트랙을 출력에서 제외한다.
- **스캔이 지도 위로 떠 보임:** `/scan/flat`, `/odom/flat` 표시를 사용한다. 원본 TF·스캔을 바꾸는 기능이 아니라 RViz용 평면 표시다.
- **원본 LaserScan의 TF extrapolation 로그:** 재생 중 수십 ms 단위의 TF 시각 차이 로그가 관측됐다. 평면 스캔 표시는 odom 이력을 사용하므로 별도로 확인할 수 있다.
- **Map의 `indexed_8bit_image` shader 로그:** 이번 환경에서 그래픽 shader 링크 오류가 기록됐다. 지도 토픽의 점유 셀 유입은 확인했지만, 최종 사용자 화면 캡처는 제한돼 저장하지 못했다. 그래픽 문제가 있으면 `rviz.log`와 Map 표시 상태를 확인한다.

## 저장된 벽 결과만 재생하는 별도 모드

다음 명령은 [저장된 비교 결과](../artifacts/gate_wall_review/preview.json.gz)를 마커로 표시한다. 위의 **센서 재생·실시간 검출 실행과 구분**한다.

```bash
source /opt/ros/humble/setup.bash
bash scripts/experiments/gate_walls/run_rviz_replay.sh
```

## 관련 파일

- [알고리즘 비교·장단점·실험 수치](GATE_WALL_EXPERIMENT.md)
- [실시간 실행 스크립트](../scripts/experiments/gate_walls/live_view_session.sh)
- [실시간 벽·경로 노드](../scripts/experiments/gate_walls/live_wall_path.py)
- [센서 구간 추출·반복 재생](../scripts/experiments/gate_walls/live_sensor_bag.py)
- [RViz 설정](../scripts/experiments/gate_walls/live_sensors.rviz)
- [실험 기하](../scripts/experiments/gate_walls/wall_model.py)
- [테스트](../scripts/experiments/gate_walls/test_wall_model.py)
