# KABOAT 현장 환경·원격 연결·명령어 가이드

**노트북 호스트**, **노트북 도커 컨테이너**, **선체 젯슨** 3개 환경의 설정,
단축어, 젯슨↔노트북 원격 연결(Tailscale + Zenoh), 노트북 키보드 조종을 정리한
문서입니다. 수조 시험 절차는 [TANK_TESTS_GUIDE.md](TANK_TESTS_GUIDE.md), 센서
bringup과 점검은 [SENSOR_CHECK.md](SENSOR_CHECK.md)를 봅니다. 야외 운용 때 치는
단축어·명령만 빠르게 보려면 [operation.md](operation.md)를 봅니다.

## 목차

- [1. 전체 통신 및 환경 구조](#1-전체-통신-및-환경-구조)
- [2. 한 번 설치](#2-한-번-설치)
- [3. 노트북 호스트 단축어](#3-노트북-호스트-단축어)
- [4. 노트북 도커 컨테이너](#4-노트북-도커-컨테이너)
- [5. 선체 젯슨 단축어](#5-선체-젯슨-단축어)
- [6. Zenoh 브리지 실행](#6-zenoh-브리지-실행)
- [7. 노트북 키보드 조종](#7-노트북-키보드-조종)
- [8. 실내 수조 표준 워크플로우](#8-실내-수조-표준-워크플로우)
- [9. 문제 해결](#9-문제-해결)

---

## 1. 전체 통신 및 환경 구조

```text
[ 선체 젯슨 (Jetson) ]                  [ 노트북 도커 (Docker) ]               [ 노트북 호스트 (Host) ]
- OS: Ubuntu (bash)                    - OS: Ubuntu (bash)                   - OS: Ubuntu (zsh)
- ROS: Humble                          - ROS: Humble                         - ROS: Jazzy
- DOMAIN: 42                           - DOMAIN: 43                          - DOMAIN: 43
- 역할: 센서 수집, 오도메트리,            - 역할: 수조 외벽 라이다 수신,           - 역할: Tailscale VPN, Zenoh 중계,
        모터 제어, 자율주행 알고리즘              배 위치(/boat_pose) 계산,            RViz 창 표시, 키보드 조종 창
                                               수조 RViz 실행
       │                                      │                                     ▲
       │                                      │ (Host-network DDS)                  │
       ▼                                      ▼                                     │
   Zenoh Bridge  ◀═════[ Tailscale 100.x.y.z ]═════▶  Zenoh Bridge (localhost:7447) ──┘
```

두 ROS 도메인을 42/43으로 달리해, 같은 와이파이에서 DDS 직통 경로가 생겨도
Zenoh 전달과 중복되지 않게 합니다. 서로 다른 휴대폰 핫스팟이어도 양쪽이
인터넷과 같은 Tailscale tailnet에 연결돼 있으면 같은 명령과 주소를 씁니다.
젯슨의 미션·센서·모터 제어는 젯슨에서 로컬로 실행하며 이 링크에 의존하지
않습니다.

Zenoh가 전달하는 토픽([`zenoh_monitor_link.py`](scripts/zenoh_monitor_link.py)):

| 방향 | 토픽 |
|---|---|
| 젯슨 → 노트북 | `/scan`, `/scan/raw`, `/odom`, `/tf`, `/tf_static`, `/occupancy_grid`, `/avoid/{plan,cps,dri}`, `/gate/{path,pairs,status,cmd_preview}`, `/tank_mission/status`, `/real_avoid_test/{path,markers,dri,status}`, `/manual_dri/{dri,status}`, `/straight_drive/markers`, `/bspline_test/{path,markers}`, `/circle_drive/markers`, `/station_keeping/markers`, `/dock/{waypoints,entrance_base,marker_base,detection_status}`, `/mission/{state,goal,entry_goal}`, `/imu/data`, `/gps/fix`, `/gnss_1/velocity`, `/ekf/status`, `/ekf/odometry_map`, `/teleop/{status,event}`, `/geo/{position,targets,target_markers,origin,status}`, `/mission_points/{result,markers}` |
| 노트북 → 젯슨 | `/boat_pose`, `/teleop/{enable,levels,gps_mark,esp_mode}`, `/mission_points/request` |
| 전달 안 함 | `/cmd_vel`, `/cmd/*`, `/start_mission`, `/emergency_stop`, 서비스, 액션, 카메라 원본 영상 |

출발·정지·비상 정지는 브리지를 통과하지 않으므로 **젯슨 도메인 42의 터미널**에서
실행합니다. 허용 목록은 브리지를 시작할 때 적용되므로, 새 토픽을 노트북에서 보려면 위 스크립트의
허용 목록에 추가하고 **양쪽 브리지를 모두 재시작**해야 합니다(젯슨 `run_zenoh`, 노트북 `stop_kaboat` → `set_kaboat`).

---

## 2. 한 번 설치

### Tailscale과 Zenoh (젯슨·노트북 호스트 각각)

Tailscale은 [공식 Linux 설치 안내](https://tailscale.com/docs/install/linux)대로
설치하고 같은 tailnet에 가입합니다. 화면이 없는 젯슨은 `tailscale up`이 출력한
인증 URL을 노트북 브라우저에서 열면 됩니다.

```bash
curl -fsSL https://tailscale.com/install.sh | sh
sudo systemctl enable --now tailscaled
sudo tailscale up
tailscale ip -4
```

Zenoh는 [공식 Debian 패키지 안내](https://github.com/eclipse-zenoh/zenoh-plugin-ros2dds#linux-debian)를
따릅니다. **젯슨과 노트북의 버전을 같게**(현재 1.10.2) 맞춥니다.

```bash
sudo install -d -m 0755 /etc/apt/keyrings
curl -L https://download.eclipse.org/zenoh/debian-repo/zenoh-public-key | \
  sudo gpg --dearmor --yes --output /etc/apt/keyrings/zenoh-public-key.gpg
echo 'deb [signed-by=/etc/apt/keyrings/zenoh-public-key.gpg] https://download.eclipse.org/zenoh/debian-repo/ /' | \
  sudo tee /etc/apt/sources.list.d/zenoh.list
sudo apt update
sudo apt install zenoh-bridge-ros2dds        # 버전이 다르면: zenoh-bridge-ros2dds=1.10.2
zenoh-bridge-ros2dds --version
```

### 젯슨 셸 환경

```bash
cd ~/Desktop/kaboat_ws/repo
bash scripts/setup_jetson_bashrc.sh
source ~/.bashrc
```

설치 후에는 새 터미널마다 워크스페이스 이동, Humble·`install/setup.bash` 로드,
`ROS_DOMAIN_ID=42`, `ROS_LOCALHOST_ONLY=0`이 자동 적용되고 5절의 단축어를 쓸 수
있습니다.

### 노트북 셸 환경

3절의 노트북 단축어는 레포가 아니라 **개인 `~/.zshrc`**에 정의돼 있습니다. 다른
노트북에서는 같은 함수·별칭을 추가하거나, 6절의 명령과
[`scripts/rviz_tank.sh`](scripts/rviz_tank.sh)를 직접 실행합니다.

---

## 3. 노트북 호스트 단축어

노트북 터미널(`zsh`)에서 실행합니다. 수조 RViz 창은 노트북 화면에 나타나지만
RViz 프로세스는 라이다와 같은 Humble 컨테이너에서 실행됩니다.

| 명령어 | 기능 |
|---|---|
| **`set_kaboat [젯슨IP]`** | Jazzy 로드, DOMAIN 43 설정, **Zenoh 브리지 백그라운드 시작** (기본 젯슨 IP 100.108.192.55, 로그 `/tmp/zenoh_bridge.log`) |
| **`stop_kaboat`** | 백그라운드 Zenoh 브리지 종료 |
| **`set_default`** | Jazzy 로드, DOMAIN 0으로 복귀 |
| **`kaboat_lidar`** | 외벽 라이다 컨테이너 셸 진입 (`DISPLAY` 전달) |
| **`kaboat_sim`** | 시뮬레이터 컨테이너 셸 진입 (`DISPLAY` 전달) |
| **`rviz_tank [시험종류]`** | 수조 추적·경로 비교 RViz. `straight`, `bspline`, `circle`, `los`, `los_circle`, `waypoint`, `station`은 고정 목표 경로, `gate`·`avoid`는 젯슨이 계획한 경로(`/gate/path`, `/avoid/plan`, Zenoh 경유)를 목표로 표시. 생략하면 추적 화면과 실제 궤적만 |
| **`clear_traj`** | `rviz_tank`의 실제 궤적(주황) 지우기. `gate`·`avoid`는 직전 계획 경로도 함께 지움. [`scripts/clear_tank_trajectory.sh`](scripts/clear_tank_trajectory.sh)가 `kaboat_lidar` 컨테이너의 `/tank_test/clear_trajectory`를 호출 |
| **`rviz_remote`** | 선체 `/scan`을 `onboard_laser_frame` 기준으로 표시 |
| **`view_pitch`** | 롤/피치 각도와 Pitch [-4.0°, +2.0°], Roll ±3.0° 스캔 차단 상태 출력 |
| **`python3 scripts/teleop_laptop.py`** | 노트북 키보드 조종 창 (7절) |
| **`rec [이름]`** / **`rec --list`** | 젯슨에 지금 위치 기록 요청 / 기록한 지점 보기 (`python3 scripts/rec_point.py`, 7-1절) |

전체 지도·경로 모니터링 화면은 다음으로 엽니다(Fixed Frame `odom`).

```bash
source /opt/ros/jazzy/setup.zsh && export ROS_DOMAIN_ID=43
rviz2 -d ~/Projects/KABOAT/kaboat_ws/scripts/remote_monitor.rviz
```

---

## 4. 노트북 도커 컨테이너

수조 외벽 라이다 드라이버 또는 Gazebo 시뮬레이터 등 **하드웨어 드라이버/시뮬레이션 격리 환경**입니다.
컨테이너에 접속(`kaboat_lidar` 등)하면 `/workspace/kaboat_ws` 이동 및 `setup.bash`가 **자동으로 source**됩니다.

```bash
# 수조 외벽 라이다 컨테이너 시작 (DOMAIN 43)
ROS_DOMAIN_ID=43 docker compose -f docker-compose.lidar.yml up -d

# 시뮬레이터 컨테이너 시작
docker compose up -d
```

| 단축어 | 실제 실행 명령어 | 설명 |
|---|---|---|
| **`cb`** | `colcon build --symlink-install` | 컨테이너 내부 워크스페이스 빌드 |
| **`sb`** | `source install/setup.bash` | 빌드 산출물 환경 즉시 반영 |
| **`tracker enable_rviz:=false`** | `ros2 launch kaboat_hardware lidar_boat_tracker.launch.py ...` | **수조 외벽 라이다 구동 및 배 위치(`/boat_pose`) 추적** (포트 인자는 [수조 가이드 2절](TANK_TESTS_GUIDE.md#2-노트북-라이다와-배-위치-추적)) |
| **`sim`** | `ros2 launch kaboat_sim simulation.launch.py` | 시뮬레이터 컨테이너 전용. `kaboat_lidar` 컨테이너에는 `kaboat_sim` 패키지가 없어 실행되지 않음 |
| **`autonomy`** | `ros2 launch kaboat_bringup autonomy.launch.py` | 자율주행 스택 전체 구동 |

---

## 5. 선체 젯슨 단축어

실제 보트 위에서 **센서 수집, 오도메트리 생성, 모터 제어**를 담당합니다.
`setup_jetson_bashrc.sh`(2절)로 설치됩니다.

| 단축어 | 실제 실행 명령어 | 설명 |
|---|---|---|
| **`cb`** / **`sb`** | `colcon build --symlink-install` / `source install/setup.bash` | 빌드 / 환경 리로드 |
| **`run_zenoh`** | `python3 scripts/zenoh_monitor_link.py jetson` | 노트북 모니터링 브리지 |
| **`run_boat_lidar`** | `ros2 launch kaboat_hardware indoor_tank.launch.py enable_thrusters:=false enable_onboard_lidar:=true onboard_lidar_port:=/dev/ttyUSB0` | GQ7·실내 `/odom`·선체 TG-50을 한 번에 실행. 외벽 라이다와 모터는 별도 |
| **`run_motor`** | `ros2 launch kaboat_hardware thrusters.launch.py hardware_type:=serial port:=/dev/ttyCH341USB0 allow_port_scan:=false` | 아두이노(CH340) 모터 드라이버 |
| **`run_points`** | `ros2 run kaboat_hardware point_recorder` | 미션 지점 기록기 (7-1절) |
| **`rec [이름]`** / **`list_points`** | `python3 scripts/rec_point.py [이름]` / `--list` | 지금 위치 기록 / 기록한 지점 보기 |
| **`cal_imu`** | `ros2 run kaboat_hardware calibrate_indoor_imu` | **[선택]** IMU 정지 보정. 기본 설정(외벽 라이다 yaw 직접 사용)에서는 불필요 |
| **`view_pitch`** | `python3 scripts/view_roll_pitch.py` | 실시간 롤/피치 모니터링 |
| **`test_bspline`** | `bspline_track_test.launch.py wait_for_start:=false` | B-Spline 곡선 주행 즉시 시작 |
| **`test_straight`** | `straight_line_test.launch.py wait_for_start:=false` | 직선 주행 즉시 시작 |
| **`test_circle`** | `circle_drive_test.launch.py wait_for_start:=false` | 원형 선회 즉시 시작 |
| **`test_station`** | `station_keeping_test.launch.py wait_for_start:=false` | 정점 유지 즉시 시작 |
| **`test_waypoint`** | `waypoint_sequence_test.launch.py motion_enabled:=true thrust_ratio:=0.3 wait_for_start:=false` | 5개 웨이포인트 순차 주행 즉시 시작 |
| **`test_los`** | `los_path_test.launch.py path_kind:=bspline motion_enabled:=true wait_for_start:=false` | LOS 경로 추종 즉시 시작 (SOTA 직진 0.25·회전 0.31) |
| **`test_avoid`** | `real_obstacle_avoid_test.launch.py drive_enabled:=true wait_for_start:=false` | 실시간 장애물 회피 즉시 시작 |

> `test_*` 단축어는 외부 시작 신호(`/start_mission`) 없이 **실행 즉시 출발**합니다. 출발 명령을 기다리게 하려면 `wait_for_start:=true`를 덧붙입니다.

`run_boat_lidar`의 TG-50 포트는 `/dev/ttyUSB0`으로 고정돼 있으므로 젯슨에서 실제
포트를 확인한 뒤 사용합니다. 선체 라이다 없이 시험할 때는
`ros2 launch kaboat_hardware indoor_tank.launch.py enable_thrusters:=false`를 직접
실행합니다. 센서 런처를 중복 실행하면 `/odom` 발행자가 둘이 되므로 한 번만 실행하세요.

---

## 6. Zenoh 브리지 실행

**젯슨** — 브리지는 젯슨의 **Tailscale IP에서만** TCP 7447을 듣습니다. 핫스팟의
`192.168.x.x` 주소는 설정에 넣지 않습니다.

```bash
cd ~/Desktop/kaboat_ws/repo
source /opt/ros/humble/setup.bash && source install/setup.bash
export ROS_DOMAIN_ID=42
tailscale ip -4                                 # 노트북에 입력할 주소
sudo sysctl -w net.core.rmem_max=8388608        # TG-50 대용량 패킷용, 재부팅 후 1회
sudo sysctl -w net.core.rmem_default=8388608
python3 scripts/zenoh_monitor_link.py jetson    # = run_zenoh
```

**노트북 호스트** — 젯슨의 Tailscale IP를 넣어 실행합니다(`set_kaboat`가 같은 일을
백그라운드로 합니다). 다음 날 핫스팟이 바뀌어도 이 IP는 그대로입니다.

```bash
cd ~/Projects/KABOAT/kaboat_ws
source /opt/ros/jazzy/setup.zsh
export ROS_DOMAIN_ID=43
export ROS_LOCALHOST_ONLY=0
python3 scripts/zenoh_monitor_link.py laptop --jetson-ip 100.x.y.z
```

스크립트는 Tailscale 상태와 주소를 확인하고 Zenoh 설정을 임시 파일로 만들어
브리지를 시작합니다. 노트북 브리지는 젯슨 연결을 계속 재시도합니다. 두 터미널을 켜 둡니다.

**노트북에서 수신 확인**

```bash
source /opt/ros/jazzy/setup.zsh && export ROS_DOMAIN_ID=43
tailscale ping 100.x.y.z
ros2 topic list | grep -E '^/(scan|odom|avoid/plan|real_avoid_test/path|dock/waypoints|ekf/status)$'
ros2 topic hz /odom
ros2 topic info /odom --verbose      # Publisher count 1 확인
```

실외에서 GQ7이 `/odom`을 발행하는 구성이라면 별도 창으로 자세 화면을 열 수 있습니다
(ROS 2 Jazzy, `matplotlib`, `microstrain_inertial_msgs` 필요). 수조의 외벽 라이다
기반 `/odom`에는 이 화면을 그대로 쓸 수 없습니다.

```bash
cd ~/Projects/KABOAT/sensor_check
python3 boat_attitude.py --gps-topic /gps/fix --odom-topic /odom
```

---

## 7. 노트북 키보드 조종

노트북 조종 창이 도메인 43에서 아래 토픽을 보내면 Zenoh가 젯슨 도메인 42로 전달합니다.
젯슨 `thruster_driver`가 받아 ESP에 시리얼로 보냅니다. 노트북이 `/cmd_vel`이나 모터 PWM을
직접 보내지는 않습니다.

| 토픽 | 내용 |
|---|---|
| `/teleop/enable` (Bool) | 노트북 조종 모드 진입·해제 |
| `/teleop/levels` (Int8MultiArray) | `[속도 단계 -4~+4, 선회 +1(A)/0/-1(D)]`, 20 Hz |
| `/teleop/gps_mark` (Empty) | Space — 젯슨 `gps_mark_logger`가 CSV에 기록 |
| `/teleop/esp_mode` (String) | `computer_control_mode` / `rc_control_mode` |
| `/teleop/status`, `/teleop/event` (젯슨 → 노트북) | 젯슨 제어 모드 / ESP 모드 전송·GPS 기록 결과 |

```bash
# 젯슨: 코드 갱신 후 빌드, 이전 run_zenoh·run_motor는 종료 후 재실행
cb && sb
run_zenoh
run_motor        # 다른 터미널 — thruster_driver 와 gps_mark_logger 를 같이 띄움
```

```zsh
# 노트북 그래픽 터미널: 브리지 재시작 후 조종 창
cd ~/Projects/KABOAT/kaboat_ws
stop_kaboat
set_kaboat
python3 scripts/teleop_laptop.py
```

젯슨이 `/teleop/status`를 보내기 전에는 조종 창이 속도를 올리지 않습니다.
조종 창을 클릭해 포커스를 준 상태에서 키를 누릅니다.

| 입력 | 동작 |
|---|---|
| `W` / `S` | 속도 단계 +1 / -1. -4 ~ +4 (0 중립, +는 전진, -는 후진), **키를 떼도 단계 유지**. 꾹 눌러도 한 번만 셈 |
| `A` / `D` (누르고 있는 동안) | 좌 / 우 회전. 좌우 추진기에 선회 폭(기본 150µs)만큼 차이 — 예) 전진 3단계(1650) + `A` → 좌 1500, 우 1800. 떼면 다시 같은 출력으로 직진 |
| 중립에서 `A` / `D` | 제자리 좌회전(좌 1350 후진, 우 1650 전진) / 제자리 우회전 |
| `X` | 즉시 0단계(중립) |
| `Space` | 그 시점 시각, GPS 위도·경도, 우리 원점 기준 x·y를 젯슨 CSV에 한 줄 기록 |
| `P` / `O` | ESP에 `computer_control_mode` / `rc_control_mode` 전송 (컴퓨터 조종 / RC 수신기 조종) |
| `Q`·`Esc` 또는 창 닫기 | 노트북 조종 종료, 젯슨 텔레옵 모드 유지하며 중립 |
| `R` | 노트북 조종 종료 후 자율주행 명령으로 명시적 복귀 |

- **단계 → PWM**: 1~4단계는 중립 1500µs에서 ±50/100/150/200µs입니다(전진 1550~1700, 후진 1450~1300). A/D 선회 폭은 `teleop_turn_pwm_offset`(기본 150µs)입니다. 둘 다 [`thrusters.yaml`](src/kaboat_hardware/config/thrusters.yaml)에서 바꾸고, `max_output_ratio` 상한은 그대로 적용됩니다.
- **ESP 시리얼**: 출력은 기존과 같은 8자리 `LLLLRRRR\n`(앞 4자리 좌, 뒤 4자리 우 PWM)이고, `P`/`O`는 문자열 한 줄(`computer_control_mode\n` / `rc_control_mode\n`)입니다.
- **GPS 기록**: `run_motor`가 시작될 때 젯슨 `~/kaboat_gps_logs/gps_log_날짜_시간.csv`를 만들고 첫 줄에 `timestamp,latitude,longitude,x,y`를 씁니다. x·y는 `~/kaboat_geo_origin.yaml` 원점 기준 [m](x = 동, y = 북 — `/odom`·`rec`와 같은 좌표계, GNSS 1 안테나 위치)이고, 원점 파일이 없으면 비워 둡니다. `Space`마다 한 줄씩 추가하고 곧바로 디스크에 반영하므로 비정상 종료돼도 그 전 기록은 남습니다. GPS가 없거나 2초 넘게 갱신되지 않으면 기록하지 않고 조종 창에 실패 이유를 표시합니다. 폴더는 `run_motor gps_log_dir:=...`로 바꿉니다.
- **안전**: 창이 포커스를 잃거나 젯슨 상태가 끊기면 즉시 0단계로 돌아갑니다(다시 포커스를 줘도 이전 속도로 출발하지 않음). 통신이 끊기면 젯슨이 0.5초 후 PWM 중립으로 전환하며 **자율주행으로 자동 복귀하지 않습니다**.
- `R`로 복귀하기 전에 자율주행 노드가 실행 중이고 현재 목표가 유효한지 확인합니다.
- 젯슨에서 `ros2 topic echo /teleop/status`로 `TELEOP`, `TELEOP_TIMEOUT`, `AUTO`, `RC_OVERRIDE`, `ESTOP`, `BACKEND_ERROR`를 확인합니다.

### 7-1. 미션 지점 기록 (노트북 → 젯슨)

배를 원하는 자리·방향에 둔 채 노트북에서 이름을 보내면, 젯슨 `point_recorder`가 그
순간의 `/odom` x, y, 선수각을 `~/kaboat_mission_points.json`에 저장합니다. 나중에
`mission_manager`가 이 파일을 미션 시작·목표 지점으로 읽습니다
([`mission_points.py`](src/kaboat_mission/kaboat_mission/mission_points.py)).
서비스는 브리지를 지나지 못하므로 짧은 String 토픽 두 개(`/mission_points/request`,
`/mission_points/result`)로 주고받습니다.

```bash
# 젯슨: 원점이 고정된 /odom 이 필요하다
run_gq7_outdoor
run_points            # 다른 터미널
```

```zsh
# 노트북: set_kaboat 로 브리지가 켜진 상태에서
rec gate_entry        # ✓ gate_entry: x=+5.50 m, y=-2.25 m, 선수각 +90.0°
rec                   # 이름 생략 → p1, p2 …
rec --list            # 저장된 지점 보기
```

`rec`는 `python3 ~/Projects/KABOAT/kaboat_ws/scripts/rec_point.py`의 단축어입니다. 노트북은
개인 `~/.zshrc`에 `alias rec="python3 <레포 경로>/scripts/rec_point.py"`를 추가하고, 젯슨은
`setup_jetson_bashrc.sh`가 같은 이름으로 만듭니다(젯슨에서도 `rec 이름`).

- 같은 이름으로 다시 보내면 덮어쓰고, 결과에 이전 값을 함께 보여 줍니다. 이름은 영문·숫자·한글·`_`·`-`만 씁니다.
- 결과가 안 오면 같은 요청 id로 1.5초마다 다시 보냅니다. 젯슨은 같은 id를 두 번 기록하지 않습니다.
- `✗ 받는 쪽이 없음`이면 젯슨 `run_points`·`run_zenoh`, 노트북 `set_kaboat`를 확인합니다.
- `/odom`이 0.5초 넘게 끊겼거나 `frame_id`가 `odom`이 아니면 기록하지 않습니다.
- 기록할 때의 GQ7 원점(`~/kaboat_geo_origin.yaml`)과 위도·경도를 같이 저장하고, 원점이 다른 지점은 한 파일에 섞지 않습니다. 원점을 새로 잡았으면 `run_gq7_outdoor`를 다시 켠 뒤 기록합니다.
- RViz에 `/mission_points/markers`(화살표 = 선수각)를 추가하면 기록한 지점이 보입니다.

---

## 8. 실내 수조 표준 워크플로우

자세한 명령과 확인 항목은 [TANK_TESTS_GUIDE.md](TANK_TESTS_GUIDE.md)를 봅니다.

| 순서 | 위치 | 명령 |
|---|---|---|
| 1 | 노트북 터미널 1 | `ROS_DOMAIN_ID=43 docker compose -f docker-compose.lidar.yml up -d` → `kaboat_lidar` → `tracker enable_rviz:=false` |
| 2 | 노트북 터미널 2 | `set_kaboat` → `rviz_tank los` (시험에 맞게 `straight`, `bspline`, `gate`, `avoid` 등) |
| 3 | 젯슨 터미널 1 | `run_zenoh` |
| 4 | 젯슨 터미널 2 | `run_boat_lidar` → `/odom` 30 Hz, 발행자 1개 확인 |
| 5 | 젯슨 터미널 4 | `run_motor` (주행할 때만, `ls -l /dev/ttyCH341USB0` 먼저) |
| 6 | 젯슨 터미널 5 | `test_los` 등 시험 노드 하나 |
| 7 | 젯슨 터미널 | 출발 `ros2 topic pub --once /start_mission std_msgs/msg/Bool '{data: true}'`, 비상 정지 `ros2 topic pub --once /emergency_stop std_msgs/msg/Bool '{data: true}'` |

---

## 9. 문제 해결

| 증상 | 원인 | 조치 |
|---|---|---|
| RViz `/scan` 안 보임, `queue is full` | Fixed Frame `odom`인데 `odom → base_link → onboard_laser_frame` TF 없음 | Fixed Frame을 `onboard_laser_frame`으로 바꾸거나 TF 확인 |
| `/scan` 0 Hz | IMU 기울기 필터가 스캔 차단 | `view_pitch`로 확인, 수평 맞추기, 벤치면 `enable_imu_filter:=false` |
| `sequence size exceeds remaining buffer` | TG-50 스캔 패킷이 기본 UDP 버퍼(208KB) 초과 | 젯슨에서 `sudo sysctl -w net.core.rmem_max=8388608 net.core.rmem_default=8388608` |
| `Package 'kaboat_hardware' not found` | `install/setup.bash` 미로드 | `source install/setup.bash` |
| `can't open file 'scripts/zenoh_monitor_link.py'` | 작업 디렉토리가 홈 | `cd ~/Desktop/kaboat_ws/repo` 후 실행 |
| 노트북에 특정 토픽 없음 | 브리지 허용 목록에 없음 또는 한쪽 스크립트가 구버전 | 1절 표 확인, 양쪽 스크립트 갱신 후 브리지 재시작 |

**연결이 끊겼을 때** — 핫스팟 변경 중에는 토픽이 잠시 멈춥니다. 두 기기의
`tailscale status`, 노트북의 `tailscale ping <젯슨 IP>`, 브리지 로그
(`tail -f /tmp/zenoh_bridge.log`), `ros2 topic hz /scan` 순서로 확인하고, 복구되지
않으면 노트북 브리지를 재시작합니다(`stop_kaboat` → `set_kaboat`).
