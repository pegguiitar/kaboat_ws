# 실내 수조 실물 테스트 가이드

수조 시험의 실행 명령 모음입니다. 환경 설정·단축어·Zenoh·텔레옵은
[COMMANDS_GUIDE.md](COMMANDS_GUIDE.md), 센서 판정 기준은
[SENSOR_CHECK.md](SENSOR_CHECK.md), 알고리즘·코드 API는
[SKELETON.MD](SKELETON.MD)·[AVOIDANCE.MD](AVOIDANCE.MD)를 봅니다.

## 단축어

이 문서의 명령 대부분은 아래 단축어로 줄일 수 있습니다. 정의와 설치는 [COMMANDS_GUIDE.md](COMMANDS_GUIDE.md)를 봅니다.

**노트북 호스트 (zsh, `~/.zshrc`)**

| 단축어 | 하는 일 | 절 |
|---|---|---|
| `set_kaboat` | Jazzy·DOMAIN 43 설정 + Zenoh 브리지 백그라운드 시작 | 2 |
| `stop_kaboat` | Zenoh 브리지 종료 | 2, 14 |
| `kaboat_lidar` | 외벽 라이다 컨테이너 셸 진입 | 2 |
| `rviz_tank [종류]` | 수조 RViz (`straight` `bspline` `circle` `los` `los_circle` `waypoint` `station` `gate` `avoid`) | 2 |
| `clear_traj` | `rviz_tank`의 실제 궤적(과 gate/avoid 이전 계획) 지우기 | 2 |
| `rviz_remote` | 선체 `/scan` 단독 RViz | 3, 9 |
| `view_pitch` | 롤/피치와 스캔 차단 상태 | 3 |

**노트북 컨테이너 (`kaboat_lidar`, 접속 시 환경 자동 로드)**

| 단축어 | 하는 일 | 절 |
|---|---|---|
| `cb` / `sb` | `colcon build --symlink-install` / `source install/setup.bash` | 1, 2 |
| `tracker enable_rviz:=false` | `lidar_boat_tracker.launch.py` (포트는 `port:=` 인자로 지정) | 2 |

**젯슨 (bash, `setup_jetson_bashrc.sh` 설치 후)**

| 단축어 | 하는 일 | 절 |
|---|---|---|
| `cb` / `sb` | 빌드 / 환경 리로드 | 3 |
| `run_zenoh` | Zenoh 브리지 (젯슨 쪽) | 3 |
| `run_boat_lidar` | GQ7 + 실내 `/odom` + 선체 TG-50 (**포트 `/dev/ttyUSB0` 고정**) | 3 |
| `run_motor` | 모터 드라이버 (`/dev/ttyCH341USB0`, `allow_port_scan:=false`) | 4 |
| `view_pitch` | 롤/피치 모니터링 | 3 |
| `cal_imu` | IMU 정지 보정 (`use_lidar_yaw_truth: false`일 때만 필요) | 3 |
| `test_bspline` `test_straight` `test_circle` `test_station` | 고정 경로 주행 (**즉시 출발**) | 5 |
| `test_waypoint` | 5개 웨이포인트 순차 주행 (`thrust_ratio:=0.3`, 즉시 출발) | 7 |
| `test_los` | LOS B-spline 추종 (SOTA 직진 0.25·회전 0.31, 즉시 출발) | 8 |
| `test_avoid` | 장애물 회피 주행 (`drive_enabled:=true`, 즉시 출발) | 10 |

- `test_*`는 모두 `wait_for_start:=false`라서 **실행하자마자 출발**합니다. 모터 드라이버(`run_motor`)가 켜져 있으면 바로 움직이니 배 주변을 먼저 확인하세요.
- 단일 waypoint, 11~13절(gate·avoid 미션, DRI), 선체 라이다 없는 `indoor_tank.launch.py`는 단축어가 없어 본문의 전체 명령을 사용합니다.

## 목차

- [단축어](#단축어)
- [0. 기본 규칙](#0-기본-규칙)
- **준비**
  - [1. 외벽 라이다 스캔만 확인](#1-외벽-라이다-스캔만-확인)
  - [2. 노트북 라이다와 배 위치 추적](#2-노트북-라이다와-배-위치-추적)
  - [3. 젯슨 센서와 오도메트리](#3-젯슨-센서와-오도메트리)
  - [4. 젯슨 모터 드라이버](#4-젯슨-모터-드라이버)
- **기본 주행 시험**
  - [5. 고정 경로 주행](#5-고정-경로-주행)
  - [6. 단일 waypoint](#6-단일-waypoint)
  - [7. 5개 웨이포인트 순차](#7-5개-웨이포인트-순차)
  - [8. LOS 경로](#8-los-경로)
- **장애물 회피와 미션 시험**
  - [9. 장애물 회피 미리보기](#9-장애물-회피-미리보기)
  - [10. 장애물 회피 저속 주행](#10-장애물-회피-저속-주행)
  - [11. 게이트 라이다 미리보기](#11-게이트-라이다-미리보기)
  - [12. gate와 avoid 미션 단독 시험](#12-gate와-avoid-미션-단독-시험)
  - [13. 무선 조종 중 DRI 모니터링](#13-무선-조종-중-dri-모니터링)
- [14. 출발 정지 종료](#14-출발-정지-종료)
- [15. 문제 해결](#15-문제-해결)

## 0. 기본 규칙

| 항목 | 값 |
|---|---|
| 노트북 | 외벽 RPLIDAR A3M1, `ROS_DOMAIN_ID=43` |
| 젯슨 | GQ7, 모터 제어기, (선택) 선체 TG-50, `ROS_DOMAIN_ID=42` |
| 수조 좌표 | `odom`, 10 × 5m, 0°=+X, 90°=+Y, 180°=-X |
| 외벽 라이다 | `(10.0, 2.5)m`, 스캔 0°가 -X 방향 ([`lidar_tracker.yaml`](src/kaboat_hardware/config/lidar_tracker.yaml)) |
| 모터 제어기 포트 | `/dev/ttyCH341USB0` (아두이노 CH340) |
| 선체 TG-50 포트 | 보통 `/dev/ttyUSB0` — **모터 포트로 쓰면 안 됨** |

- `/cmd_vel` 발행 노드는 **한 번에 하나만** 실행합니다(시험 노드, `cmd_mux`, 미션 가드).
- `/odom` 발행자는 **반드시 1개**입니다.
- 항상 미리보기(`motion_enabled:=false` / `drive_enabled:=false`)를 먼저 확인합니다.
- 출발·정지·비상 정지는 브리지를 통과하지 않으므로 **젯슨에서** 실행합니다.

## 1. 외벽 라이다 스캔만 확인

추적기 없이 `/shore/scan`과 RViz만 띄웁니다. 노트북 그래픽 터미널에서:

```bash
cd ~/Projects/KABOAT/kaboat_ws
ROS_DOMAIN_ID=43 docker compose -f docker-compose.lidar.yml up -d --build
xhost +si:localuser:root
docker exec -it -e DISPLAY="$DISPLAY" kaboat_lidar bash
# Docker 내부
colcon build --symlink-install --packages-select kaboat_hardware && source install/setup.bash
ros2 launch kaboat_hardware shore_lidar_scan.launch.py
```

- RViz Fixed Frame `shore_laser_frame`, 토픽 `/shore/scan`.
- 2절로 넘어가기 전에 `Ctrl+C`로 종료합니다(한 장치를 두 드라이버가 못 엶).

## 2. 노트북 라이다와 배 위치 추적

**호스트 터미널 1 → Docker 내부**

```bash
cd ~/Projects/KABOAT/kaboat_ws
ROS_DOMAIN_ID=43 docker compose -f docker-compose.lidar.yml up -d --build --force-recreate
docker exec -it kaboat_lidar bash
# Docker 내부 (kaboat_lidar 별칭 + tracker 별칭으로 대체 가능)
colcon build --symlink-install --packages-up-to kaboat_hardware && source install/setup.bash
ros2 launch kaboat_hardware lidar_boat_tracker.launch.py \
  port:=/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_344ff5135095724097887d9c6d4b91ce-if00-port0 \
  enable_rviz:=false
# 설치 위치가 다르면: lidar_pos_x:=... lidar_pos_y:=... lidar_yaw_deg:=...
```

확인(다른 Docker 터미널):

```bash
ros2 topic hz /shore/scan                            # 약 13 Hz
ros2 topic echo /shore/scan --once --field header    # frame_id: shore_laser_frame
ros2 topic echo /boat_pose --once
```

- 배의 ㄱ자 판을 실측 위치에 두고 `/boat_pose`가 그 좌표로 나오는지 확인하기 전에는 **모터를 켜지 않습니다**.

**호스트 터미널 2 — Zenoh 브리지** (`set_kaboat`로 대체 가능)

```bash
source /opt/ros/jazzy/setup.zsh
export ROS_DOMAIN_ID=43 ROS_LOCALHOST_ONLY=0
read -r "JETSON_TAILSCALE_IP?젯슨 Tailscale IP: "
tailscale ping "$JETSON_TAILSCALE_IP"
python3 ~/Projects/KABOAT/kaboat_ws/scripts/zenoh_monitor_link.py laptop --jetson-ip "$JETSON_TAILSCALE_IP"
```

**호스트 그래픽 터미널 3 — RViz**

```bash
rviz_tank                 # 위치 추적만
rviz_tank <시험이름>       # straight | bspline | circle | los | los_circle | waypoint | station | gate | avoid
# 장애물 회피 3인칭 화면 (동시 실행 가능)
docker exec -it -e DISPLAY="$DISPLAY" kaboat_lidar bash -lc \
  'source /workspace/kaboat_ws/install/setup.bash && ros2 launch kaboat_hardware real_obstacle_avoid_rviz.launch.py'
# 실제 궤적 지우기 (노트북 호스트 터미널에서 alias 입력)
clear_traj
```

- 젯슨 시험 좌표를 바꿨다면 노트북에도 같은 설정을 적용하고 RViz를 다시 엽니다.
- **`rviz_tank gate` / `rviz_tank avoid`**: 목표 경로(초록)를 젯슨이 계획한 `/gate/path` / `/avoid/plan`에서 받아 그립니다(Zenoh 경유). 주황색 실제 궤적은 외벽 라이다 `/boat_pose`입니다. 가장 최근의 유효한 계획이 화면에 남고, 계획이 바뀌면 갱신됩니다. 시도마다 `/tank_test/clear_trajectory`를 호출하면 궤적과 이전 계획이 함께 지워집니다. gate는 기둥 쌍·중앙(`/gate/pairs`)도 같이 보입니다. 10초 넘게 경로가 안 오면 노트북 로그에 경고가 나오니 12절 미리보기가 떠 있는지, 양쪽 브리지가 새 `zenoh_monitor_link.py`로 재시작됐는지 확인합니다.

## 3. 젯슨 센서와 오도메트리

**최초 1회 빌드**

```bash
cd ~/Desktop/kaboat_ws/repo
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to kaboat_hardware
# 11·12절까지 하려면: --packages-up-to kaboat_hardware kaboat_bringup
```

**새 터미널마다 준비 명령** (`setup_jetson_bashrc.sh` 설치 시 앞 4줄 자동)

```bash
cd ~/Desktop/kaboat_ws/repo
source /opt/ros/humble/setup.bash && source install/setup.bash
export ROS_DOMAIN_ID=42 ROS_LOCALHOST_ONLY=0
sudo sysctl -w net.core.rmem_max=8388608 net.core.rmem_default=8388608   # 재부팅 후 1회
```

**젯슨 터미널 1 — Zenoh 브리지** (`run_zenoh`)

```bash
tailscale ip -4            # 노트북 터미널 2에 입력
python3 scripts/zenoh_monitor_link.py jetson
```

**젯슨 터미널 2 — GQ7 + 실내 /odom** (둘 중 하나만)

```bash
# 선체 라이다 없이
ros2 launch kaboat_hardware indoor_tank.launch.py enable_thrusters:=false

# 선체 TG-50 포함 (포트가 /dev/ttyUSB0이면 run_boat_lidar)
ls -l /dev/serial/by-id/ /dev/ttyUSB* 2>/dev/null
ros2 launch kaboat_hardware indoor_tank.launch.py enable_thrusters:=false \
  enable_onboard_lidar:=true onboard_lidar_port:=/dev/ttyUSB0
# 육상/벤치에서 기울기 필터 끄기: enable_imu_filter:=false
```

**젯슨 터미널 3 — 확인**

```bash
ros2 topic echo /boat_pose --once
ros2 topic hz /odom                                          # 약 30 Hz
ros2 topic info /odom --verbose | grep "Publisher count"     # 반드시 1
ros2 run tf2_ros tf2_echo odom base_link
ros2 topic hz /scan                                          # 선체 라이다 장착 시
python3 scripts/view_roll_pitch.py                           # view_pitch, 스캔 차단 여부
```

- 기본 설정(`use_lidar_yaw_truth: true`)은 외벽 라이다 yaw를 그대로 쓰므로 **IMU 보정(`cal_imu`)이 필요 없습니다**.
  `use_lidar_yaw_truth: false`로 바꾼 경우에만 선수를 -X(180°)로 두고 정지한 채 `cal_imu`를 실행합니다.
- `/boat_pose`가 1초 넘게 끊기면 `/odom`이 멈춥니다.
- 실내에서 `real_sensors.launch.py`를 함께 실행하지 않습니다(`/odom` 발행자 중복).
- 선체 `/scan`은 후방 120°~240°를 가리고, Pitch [-4°, +2°]·Roll ±3°를 벗어나면 차단됩니다(`occupancy_grid`도 같은 범위). 원본은 `/scan/raw`.

## 4. 젯슨 모터 드라이버

**젯슨 터미널 4** (`run_motor`)

```bash
ls -l /dev/ttyCH341USB0
ros2 launch kaboat_hardware thrusters.launch.py hardware_type:=serial port:=/dev/ttyCH341USB0 allow_port_scan:=false

# 저출력 bring-up
ros2 launch kaboat_hardware thrusters.launch.py hardware_type:=serial port:=/dev/ttyCH341USB0 \
  allow_port_scan:=false max_output_ratio:=0.2
# 벤치(하드웨어 없음): hardware_type:=dummy / PCA9685: hardware_type:=pca9685
```

| 우선순위 | 입력 | 끊기면 |
|---|---|---|
| 1 | `/emergency_stop` true | 즉시 중립 |
| 2 | `/rc/cmd_vel` (RC 조종기) | 0.5초 후 다음 순위 |
| 3 | `/teleop/*` (노트북 WASD) | 0.5초 후 중립 (자율 복귀 안 함) |
| 4 | `/cmd_vel` (자율주행) | 0.3초 후 중립 |

```bash
ros2 topic echo /teleop/status   # AUTO | TIMEOUT | TELEOP | RC_OVERRIDE | ESTOP | BACKEND_ERROR
```

## 5. 고정 경로 주행

**젯슨 터미널 5** — 하나만 실행. 실행 즉시 출발하며, 대기하려면 `wait_for_start:=true`.

```bash
test_bspline     # bspline_track_test.launch.py wait_for_start:=false
test_straight
test_circle
test_station
# 또는
ros2 launch kaboat_hardware tank_tests.launch.py test:=straight   # straight | bspline | circle | station_keeping
```

- B-spline 튜닝 인자: `v_full:=... cmd_exp:=... kappa_window:=... omega_max:=...` (기본값은 시뮬레이션 교정값)
- 벽면 자동 정지가 없으므로 `/emergency_stop`을 준비해 둡니다.

## 6. 단일 waypoint

```bash
# 미리보기 (/cmd_vel 발행 안 함)
ros2 launch kaboat_hardware indoor_waypoint.launch.py \
  x:=5.0 y:=2.5 angle:=180.0 thrust_ratio:=0.5 motion_enabled:=false
ros2 topic echo /waypoint/status --once      # PREVIEW_APPROACH → PREVIEW_ALIGN → PREVIEW_REACHED
ros2 topic echo /waypoint/cmd_preview --once

# 실제 주행 (미리보기 종료, 4절 모터 드라이버 실행 후)
ros2 launch kaboat_hardware indoor_waypoint.launch.py \
  x:=5.0 y:=2.5 angle:=180.0 thrust_ratio:=0.5 motion_enabled:=true
```

- 도착 조건: 0.35m 이내, 각도 지정 시 8° 이내이면서 회전 속도 0.1 rad/s 이하를 0.5초 유지. 한 번 들어오면 반경의 2배(0.7m)를 벗어나야 다시 접근합니다(경계에서 접근↔정렬을 오가며 맴돌던 문제 수정, 2026-11-09). 목표는 벽에서 0.25m 이상 안쪽.
- 장애물 회피를 하지 않으므로 경로를 비워 둡니다.

## 7. 5개 웨이포인트 순차

목표는 [`waypoint_sequence_test.yaml`](src/kaboat_hardware/config/waypoint_sequence_test.yaml)의
`waypoints_x/y/angle_deg/use_heading`에 각각 정확히 5개씩 넣습니다.

```bash
# 미리보기
ros2 launch kaboat_hardware waypoint_sequence_test.launch.py hold_duration_sec:=3.0
# 실제 주행 (4절 모터 드라이버 실행 후; test_waypoint = 즉시 출발)
ros2 launch kaboat_hardware waypoint_sequence_test.launch.py \
  hold_duration_sec:=3.0 thrust_ratio:=0.3 motion_enabled:=true

# 다른 터미널
ros2 service call /waypoint_sequence/start std_srvs/srv/Trigger '{}'
ros2 service call /waypoint_sequence/pause std_srvs/srv/Trigger '{}'
ros2 topic echo /waypoint_sequence/status    # WAYPOINT_1/5 … WAYPOINT_5/5 → COMPLETE_HOLD
```

- 각 목표를 3초 연속 유지해야 다음으로 넘어갑니다. RViz Path: `/waypoint_sequence/path`.
- 실제 배 주행은 아직 검증하지 않았습니다.

## 8. LOS 경로

```bash
# 미리보기 (둘 중 하나)
ros2 launch kaboat_hardware los_path_test.launch.py path_kind:=bspline
ros2 launch kaboat_hardware los_path_test.launch.py path_kind:=circle \
  center_x:=5.0 center_y:=2.5 radius:=1.2 direction:=ccw target_laps:=2.0

# 실제 주행 (4절 모터 드라이버 실행 후; test_los = bspline, SOTA 출력, 즉시 출발)
ros2 launch kaboat_hardware los_path_test.launch.py \
  path_kind:=circle motion_enabled:=true

# 다른 터미널
ros2 service call /los_test/start std_srvs/srv/Trigger '{}'
ros2 topic echo /los_test/status
```

- RViz: `rviz_tank los` / `rviz_tank los_circle`, Path `/los_test/path`.
- **제어 상수:** LOS 시험과 게이트·탐색 미션의 공통 `los()`는 **수조 회피 SOTA**와 같은 값을 씁니다(2026-11-09): 직진 `thrust_ratio` 0.25(전진 기준·상한), 회전 상한 `turn_ratio` 0.31, 요 PD 1.4/0.16, lookahead 0.8 m, 20Hz. 값은 [`los_controller.py`](src/kaboat_behaviors/kaboat_behaviors/los_controller.py)의 `LOS_*`·`BSPLINE_TRACK_YAW_*` 상수 한 곳에서 정하고, 런치 기본값도 이 상수를 읽습니다(예전엔 런치에 kp 1.2가 따로 적혀 있었음). `bspline_track_test`(1.5/0.15, lookahead 1.0)와는 다릅니다.
- 수조 밖으로 나가는 경로는 시작 시 거부됩니다.

## 9. 장애물 회피 미리보기

3절의 **선체 TG-50 포함** 명령으로 `/scan`·`/odom`을 준비하고, 모터 드라이버는 켜지 않습니다.

```bash
ros2 launch kaboat_hardware real_obstacle_avoid_test.launch.py
# 이미 /occupancy_grid가 있으면: start_occupancy_grid:=false
# 실제 라이다 TF를 따로 발행 중이면: publish_scan_tf:=false
# 육상 시험이면: enable_tilt_gate:=false

ros2 topic hz /scan
ros2 topic echo /real_avoid_test/status     # PREVIEW 확인, NO PLAN이면 배치·목표·지도 점검
```

| RViz (2절 회피 화면) | 토픽 |
|---|---|
| 선체 스캔 | `/scan` (120°~240° 가림) |
| 점유 지도 / 위험도 | `/occupancy_grid` / `/real_avoid_test/dri` |
| 경로·표식 | `/real_avoid_test/path`, `/real_avoid_test/markers` |

- **고정 목표:** 목표 `(1.5, 2.5)`(수조 좌표, `config/real_obstacle_avoid_test.yaml`의 `fixed_goal_xy`)입니다. 시작 위치는 상관없고(벽에서 0.5m 이상), 배 선수를 **목표 방향 45° 이내**로 두면 출발합니다. 출발이 막히면 status와 실행 터미널에 `start_blocked=<이유>`가 나옵니다. 전방 장애물 확인 차선도 배→목표 방향입니다. 특정 시작점에서만 출발시키려면 YAML에 `fixed_start_xy`와 `start_tolerance`(>0)를 줍니다. 목표 0.5m 이내면 종료합니다. 런치 인자 `fixed_goal:=false`면 예전처럼 선수 앞 `goal_distance`(기본 3m)가 목표입니다. 목표가 DRI 창(±7.4m)보다 멀면 플래너가 창 안 6.9m 지평선으로 계획합니다.
- **수조 벽 처리:** `/odom`이 수조 좌표(좌하단 원점, 10 × 5m)이므로, 경계 띠(`tank_wall_band` 0.5m) 안의 점유 셀을 벽으로 봅니다. 벽은 버리지 않고 DRI를 `tank_wall_dri`(0.5)로 눌러서 지도에 약하게만 남깁니다. 후보 필터 문턱(1.116)보다 낮아서 경로를 막지 않고, 빨간 X와 "전방 장애물 확인"에는 들어가지 않습니다. `tank_wall_dri: 0`이면 벽 셀을 완전히 지우고, 런치 인자 `exclude_tank_walls:=false`면 벽도 일반 장애물입니다(`config/real_obstacle_avoid_test.yaml`). 야외에는 벽이 없으니 플래너는 그대로입니다. 먼 벽(약 7m)은 육상 yaw 오차로 0.25m쯤 밀려 보일 수 있어서, 먼 벽에 X가 남으면 띠를 키웁니다.
- **벽을 빼는 대신 경계 감시:** 플래너가 벽을 모르므로 선체 `/odom`의 x, y가 경계에서 `tank_stop_margin`(기본 0.5m) 안으로 들어오면 정지(`STOP: tank boundary`)합니다. 출발할 때도 배와 목표점이 이 안쪽이어야 합니다(아니면 `PREVIEW`에서 대기). 시작 y는 1.75~3.25m, 부표는 벽에서 1.6m 이상 떼어 두면 조사에서 안전했습니다.

## 10. 장애물 회피 저속 주행

9절 런처를 종료한 뒤:

```bash
# 젯슨 터미널 4: 4절 모터 드라이버
# 젯슨 터미널 5 (test_avoid = 즉시 출발)
ros2 launch kaboat_hardware real_obstacle_avoid_test.launch.py drive_enabled:=true
# 다른 터미널
ros2 service call /real_avoid_test/start std_srvs/srv/Trigger '{}'
```

- 상한 전진 0.25·회전 0.31(전진 0.3·회전 0.3이면 꺾이는 회피 경로에서 밀려 조정, 경로 추종 게인은 `test_los`와 같은 1.4/0.16). 경로를 5초 넘게 못 만들거나 위험이 안 풀리면 **후진 탈출(ESCAPE, 출력 0.1 — 지나온 길을 거꾸로 되짚음, 지나온 길이 없으면 정지)** 후 경로가 다시 나오면 복귀(`allow_escape:=false`면 대신 정지). 센서 끊김, 경로 0.7m 이탈, 벽 0.5m 접근(후진 중 포함) 시 중지.
- 경로 추종 값(`fsm.*`)은 `test_los`와 같은 코드 기본값을 씁니다. 바꿔 보려면 [`real_obstacle_avoid_test.yaml`](src/kaboat_hardware/config/real_obstacle_avoid_test.yaml)에 `fsm.*`를 추가해 덮어씁니다.
- 선체 폭 충돌 예측은 없으므로 RViz에서 여유를 직접 확인합니다.

## 11. 게이트 라이다 미리보기

3절로 `/odom`·선체 `/scan`을 준비합니다. `goal_x/y`는 **마지막 게이트 뒤쪽 실측 좌표**(출구에서 4m 이내).

```bash
ros2 launch kaboat_bringup gate_tank_preview.launch.py \
  goal_x:=9.0 goal_y:=2.5 required_pairs:=3 start_occupancy_grid:=true
# 이미 /occupancy_grid가 있으면 start_occupancy_grid:=true를 뺍니다

ros2 topic echo /gate/status
ros2 topic echo /gate/cmd_preview
```

- `cmd_mux`·모터는 시작하지 않으며 `/cmd/gate`는 0입니다.
- RViz: Fixed Frame `odom`, Path `/gate/path`, MarkerArray `/gate/pairs`(빨강=기둥, 초록=중앙). 노트북에서는 `rviz_tank gate`(두 토픽이 Zenoh로 전달됨).
- 설정: [`gate_params.yaml`](src/kaboat_bringup/config/gate_params.yaml) (`required_pairs`, 게이트 폭, `min_clearance` 등).
- 동작 원리는 [SKELETON.MD](SKELETON.MD)의 `gate_follower` 항목을 봅니다.

## 12. gate와 avoid 미션 단독 시험

생산용 `gate_follower`/`obstacle_planner`를 미션 가드(`tank_mission_guard`)와 함께 실행합니다.
좌표는 벽에서 0.5m 이상, 시작–목표 거리 2m 초과.

```bash
read -rp '시작 X Y 선수각(deg) 종료 X Y: ' ENTRY_X ENTRY_Y ENTRY_ANGLE GOAL_X GOAL_Y

# gate (미리보기)
ros2 launch kaboat_hardware tank_gate_mission.launch.py \
  entry_x:="$ENTRY_X" entry_y:="$ENTRY_Y" entry_angle_deg:="$ENTRY_ANGLE" \
  goal_x:="$GOAL_X" goal_y:="$GOAL_Y" required_pairs:=3

# avoid (미리보기, gate 런치 종료 후)
ros2 launch kaboat_hardware tank_avoid_mission.launch.py \
  entry_x:="$ENTRY_X" entry_y:="$ENTRY_Y" entry_angle_deg:="$ENTRY_ANGLE" \
  goal_x:="$GOAL_X" goal_y:="$GOAL_Y"

# 실제 주행: 4절 모터 드라이버 실행 후, 같은 명령 끝에 drive_enabled:=true
ros2 topic echo /tank_mission/status
ros2 service call /tank_mission/start std_srvs/srv/Trigger '{}'
ros2 service call /tank_mission/stop  std_srvs/srv/Trigger '{}'
```

- 확인 토픽: `/mission/state`, gate `/gate/status` `/gate/path`, avoid `/avoid/plan` `/avoid/dri`.
- 노트북에서 보기: `rviz_tank gate` 또는 `rviz_tank avoid`(목표=계획 경로, 실제=외벽 라이다). `/tank_mission/status`도 Zenoh로 넘어옵니다.
- 주행 시 `start` 서비스가 성공해야 움직입니다. `/odom`, `/occupancy_grid`, 원시 명령이 끊기거나 E-Stop이면 정지 후 재시작을 요구합니다.
- **가드는 `/scan`을 검사하지 않습니다.** 라이다가 끊기거나 기울기 필터가 스캔을 막아도 배는 마지막 지도로 계속 움직이므로(점유 셀 시간 감쇠 없음), 시험 중 노트북 RViz에서 `/scan`을 계속 보고 이상하면 바로 `/tank_mission/stop`이나 `/emergency_stop`을 보냅니다. gate는 자체 검사(`gate.scan_timeout_sec` 0.7초)가 있어 스캔이 끊기면 스스로 멈춥니다.
- 상한 전진 0.30·회전 0.40(`max_linear_cmd`·`max_angular_cmd`, 게이트 LOS·회피 0.25/0.31이 잘리지 않게), 후진 차단. 시작 자세까지의 직선 접근은 회피하지 않으므로 그 구간을 비웁니다.
- 이미 `/occupancy_grid`가 있으면 `start_occupancy_grid:=false`.

## 13. 무선 조종 중 DRI 모니터링

배는 무선 조종기로 움직이고 ROS는 DRI만 계산합니다(모터 명령 없음). 2~3절(선체 TG-50 포함)을 먼저 준비합니다.

```bash
# 젯슨
ros2 launch kaboat_hardware manual_dri_test.launch.py
# 옵션: start_occupancy_grid:=false / publish_scan_tf:=false
ros2 topic echo /manual_dri/status            # READY 확인
ros2 topic info /cmd_vel --verbose            # 발행자 0 확인

# 노트북: 컨테이너에 새 코드 반영 후 RViz
docker exec -it kaboat_lidar bash -lc \
  'source /opt/ros/humble/setup.bash && colcon build --symlink-install --packages-up-to kaboat_hardware'
docker exec -it -e DISPLAY="$DISPLAY" kaboat_lidar bash -lc \
  'source /workspace/kaboat_ws/install/setup.bash && ros2 launch kaboat_hardware manual_dri_rviz.launch.py'
```

- `WAIT_ODOM`/`WAIT_SCAN`/`WAIT_GRID`면 해당 입력을 확인합니다. 노트북에 DRI가 없으면 양쪽 Zenoh 스크립트가 최신인지 확인합니다.
- 설정: [`manual_dri_test.yaml`](src/kaboat_hardware/config/manual_dri_test.yaml).

## 14. 출발 정지 종료

젯슨(도메인 42) 터미널에서:

```bash
ros2 topic pub --once /start_mission  std_msgs/msg/Bool '{data: true}'    # 출발 (5절, 10절)
ros2 topic pub --once /start_mission  std_msgs/msg/Bool '{data: false}'   # 일시 정지 / 상태 재설정
ros2 topic pub --once /emergency_stop std_msgs/msg/Bool '{data: true}'    # 비상 정지
ros2 topic pub --once /emergency_stop std_msgs/msg/Bool '{data: false}'   # 원인 해결 후 해제
```

종료: 각 터미널 `Ctrl+C` 후 노트북 호스트에서

```bash
xhost -si:localuser:root
cd ~/Projects/KABOAT/kaboat_ws && docker compose -f docker-compose.lidar.yml down
```

남은 프로세스 정리는 [SENSOR_CHECK.md](SENSOR_CHECK.md#12-프로세스-정리)를 봅니다.

## 15. 문제 해결

| 증상 | 조치 |
|---|---|
| `/shore/scan` 토픽은 있는데 데이터 없음 | USB 재연결로 포트 번호가 바뀐 경우. 추적기를 `Ctrl+C` 후 재실행. 반복되면 허브·케이블 점검 |
| 드라이버는 `health OK`인데 스캔 없음 | 스캔 모드 로그와 `ros2 topic hz /shore/scan` 확인. 스캔 전용 런치가 남아 있는지 확인 |
| RViz에 격자만 보임 | `docker exec kaboat_lidar bash -lc 'source /opt/ros/humble/setup.bash; ROS_DOMAIN_ID=43 timeout 8s ros2 topic hz /shore/scan'` |
| 호스트 Jazzy RViz에 스캔 안 보임 | 컨테이너 Humble RViz(`rviz_tank`)를 사용 |
| RViz 화면 연결 오류 | 그래픽 터미널에서 `echo "$DISPLAY"`, `xhost +si:localuser:root` 확인 |
| `/boat_pose` 위치·yaw가 실물과 다름 | 다른 물체를 판으로 오인. RViz `/lidar_tracker/filtered_scan` 확인, 주행 중단 |
| `/odom` 없음 | 노트북 `/boat_pose`와 Zenoh 연결 확인 |
| 선체 `/scan` 0 Hz | 기울기 필터 차단 가능성. `view_pitch`로 확인, 벤치면 `enable_imu_filter:=false` |
| `Package 'kaboat_hardware' not found` | `source install/setup.bash` 누락 |

USB 진단:

```bash
ls -l /dev/serial/by-id/
journalctl -k --since '10 minutes ago' --no-pager | grep -Ei 'usb|cp210x|ttyUSB|disconnect'
```
