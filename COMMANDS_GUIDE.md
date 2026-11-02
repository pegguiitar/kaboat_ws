# 🚀 KABOAT 통합 환경 및 명령어 치트시트 (Commands Guide)

본 문서는 **노트북 호스트(Host OS)**, **노트북 도커 컨테이너(Docker)**, **선체 젯슨(Jetson)** 3개 환경의 자동화 설정 및 현장 실행 명령어를 한눈에 볼 수 있도록 정리한 가이드입니다.

---

## 🏗️ 1. 전체 통신 및 환경 구조

```text
[ 선체 젯슨 (Jetson) ]                  [ 노트북 도커 (Docker) ]               [ 노트북 호스트 (Host) ]
- OS: Ubuntu (bash)                    - OS: Ubuntu (bash)                   - OS: Ubuntu (zsh)
- ROS: Humble                          - ROS: Humble                         - ROS: Jazzy
- DOMAIN: 42                           - DOMAIN: 43                          - DOMAIN: 43
- 역할: 센서 수집, 오도메트리,            - 역할: 수조 외벽 라이다 수신,           - 역할: Tailscale VPN, Zenoh 중계,
        모터 제어, 자율주행 알고리즘              배 위치(/boat_pose) 계산,            RViz 창 표시
                                               수조 RViz 실행
       │                                      │                                     ▲
       │                                      │ (Host-network DDS)                  │
       ▼                                      ▼                                     │
   Zenoh Bridge  ◀═════[ Tailscale 100.x.y.z ]═════▶  Zenoh Bridge (localhost:7447) ──┘
```

---

## 💻 2. 노트북 호스트 (Laptop Host)

노트북 터미널(`zsh`)에서 명령을 실행합니다. 수조 RViz 창은 노트북 화면에 나타나지만, RViz 프로세스는 라이다와 같은 Humble 컨테이너에서 실행됩니다.

### 2-1. 환경 제어 명령어
| 명령어 | 기능 | 설명 |
|---|---|---|
| **`set_kaboat`** | **KABOAT 모드 활성화 & 브리지 가동** | ROS 2 Jazzy 로드, DOMAIN 43 설정, **Zenoh 브리지 백그라운드 자동 시작** (젯슨 100.108.192.55 연결 시도) |
| **`stop_kaboat`** | **KABOAT 브리지 종료** | 백그라운드에서 동작 중인 Zenoh 브리지 프로세스 종료 |
| **`set_default`** | **기본 ROS 모드 전환** | ROS 2 Jazzy 로드, DOMAIN 0(기본값)으로 복귀 |

### 2-2. 컨테이너 원클릭 접속
| 명령어 | 기능 | 실제 실행 명령어 |
|---|---|---|
| **`kaboat_lidar`** | 외벽 라이다 컨테이너 셸 진입 | 현재 화면 번호를 전달하는 `docker exec -it -e DISPLAY="$DISPLAY" kaboat_lidar bash` |
| **`kaboat_sim`** | 시뮬레이터 컨테이너 셸 진입 | 현재 화면 번호를 전달하는 `docker exec -it -e DISPLAY="$DISPLAY" kaboat_sim_container bash` |

### 2-3. 모니터링 & 시각화 단축어
| 명령어 | 기능 | 상세 내용 |
|---|---|---|
| **`rviz_tank`** | **수조 추적 RViz 실행** | [`scripts/rviz_tank.sh`](scripts/rviz_tank.sh)를 통해 Humble 컨테이너 RViz 실행. `/shore/scan`과 `/boat_pose` 표시 |
| **`rviz_remote`** | **원격 자율주행 RViz 실행** | 선체 라이다(`/scan`), 점유격자, 회피/LOS 경로, 오도메트리 시각화 |
| **`view_pitch`** | **실시간 롤/피치 각도 확인** | 배의 롤/피치 각도 및 ±5.0° 스캔 차단 임계값 상태 터미널 출력 |

---

## 🐳 3. 노트북 도커 컨테이너 (Laptop Docker)

수조 외벽 라이다 드라이버 또는 Gazebo 시뮬레이터 등 **하드웨어 드라이버/시뮬레이션 격리 환경**입니다.
컨테이너에 접속(`kaboat_lidar` 등)하면 `/workspace/kaboat_ws` 이동 및 `setup.bash`가 **자동으로 source**됩니다.

### 3-1. 컨테이너 구동 (노트북 호스트 터미널에서)
```bash
# 수조 외벽 라이다 컨테이너 시작
docker compose -f docker-compose.lidar.yml up -d

# 시뮬레이터 컨테이너 시작
docker compose up -d
```

### 3-2. 컨테이너 내부 단축어
| 단축어 | 실제 실행 명령어 | 설명 |
|---|---|---|
| **`cb`** | `colcon build --symlink-install` | 컨테이너 내부 워크스페이스 빌드 |
| **`sb`** | `source install/setup.bash` | 빌드 산출물 환경 즉시 반영 |
| **`tracker enable_rviz:=false`** | `ros2 launch kaboat_hardware lidar_boat_tracker.launch.py ...` | **수조 외벽 라이다 구동 및 배 위치(`/boat_pose`) 추적** (화면 없이 가볍게 실행) |
| **`sim`** | `ros2 launch kaboat_sim simulation.launch.py` | 시뮬레이터 컨테이너 전용. 현재 `kaboat_lidar` 컨테이너에는 `kaboat_sim` 패키지가 설치돼 있지 않아 실행되지 않음 |
| **`autonomy`** | `ros2 launch kaboat_bringup autonomy.launch.py` | 시뮬레이터 자율주행 스택 전체 구동 |

---

## ⚡ 4. 선체 젯슨 (Jetson)

실제 보트 위에서 **센서 수집, EKF 오도메트리 생성, 모터 제어**를 담당합니다.

### 4-1. 젯슨 1회 설치 (젯슨 터미널에서 최초 1회만)
```bash
cd ~/Desktop/kaboat_ws/repo
bash scripts/setup_jetson_bashrc.sh
source ~/.bashrc
```
*(설치 후에는 젯슨에서 새 터미널을 열 때마다 워크스페이스 이동, Humble 로드, `DOMAIN_ID=42`가 자동 적용됩니다.)*

### 4-2. 젯슨 현장 단축어
| 단축어 | 실제 실행 명령어 | 설명 |
|---|---|---|
| **`cb`** | `colcon build --symlink-install` | 젯슨 워크스페이스 빌드 |
| **`sb`** | `source install/setup.bash` | 빌드 환경 리로드 |
| **`run_tank`** | `ros2 launch kaboat_hardware indoor_tank.launch.py enable_thrusters:=false` | **실내 센서 및 오도메트리(`/odom`) 가동** (GQ7 IMU + 외벽 라이다 `/boat_pose` 융합) |
| **`run_boat_lidar`** | `... enable_onboard_lidar:=true onboard_lidar_port:=/dev/ttyUSB0` | 젯슨에 장착한 선체 TG-50 라이다까지 켜는 명령. 수조 외벽 라이다는 노트북에서 실행 |
| **`cal_imu`** | `ros2 run kaboat_hardware calibrate_indoor_imu` | **[필수] 정지 상태 IMU 방향 180° 분기 보정** |
| **`run_motor`** | 현재 `/dev/ttyUSB0`을 지정하는 모터 드라이버 실행 | 실제 젯슨의 모터 포트가 확인될 때까지 사용하지 말고 [수조 가이드](TANK_TESTS_GUIDE.md#3-젯슨-imu-보정과-모터테스트-시작)의 포트 확인 절차를 따르세요 |
| **`run_zenoh`** | `python3 scripts/zenoh_monitor_link.py jetson` | **노트북과의 원격 모니터링 통신 브리지 가동** |
| **`view_pitch`** | `python3 scripts/view_roll_pitch.py` | 실시간 롤/피치 자세 각도 모니터링 |
| **`test_los`** | `ros2 launch kaboat_hardware los_path_test.launch.py path_kind:=bspline` | LOS 경로 추종 자율주행 테스트 |
| **`test_bspline`**| `ros2 launch kaboat_hardware bspline_track_test.launch.py wait_for_start:=true` | B-Spline 곡선 자율주행 테스트 |

`run_boat_lidar`는 `run_tank` 전체 센서·오도메트리 스택에 **선체 TG-50**을 추가해 실행합니다. 두 명령을 동시에 실행하지 마세요. 현재 단축 명령의 TG-50 포트는 `/dev/ttyUSB0`으로 고정돼 있으므로 젯슨에서 실제 포트를 확인한 뒤 사용해야 합니다.

---

## 🎬 5. 실내 수조 실물 주행 표준 워크플로우

### 1단계: 수조 외벽 세팅 (내 노트북)
* **[노트북 터미널 1]** 수조 라이다 컨테이너 접속 후 트래커 구동:
  ```bash
  docker compose -f docker-compose.lidar.yml up -d
  kaboat_lidar
  tracker enable_rviz:=false
  ```
* **[노트북 터미널 2]** 노트북 화면에 컨테이너 수조 RViz 띄우기:
  ```zsh
  set_kaboat
  rviz_tank
  ```

---

### 2단계: 선체 세팅 (배 위 젯슨)
* **[젯슨 터미널 1]** 노트북 모니터링 통신 연결:
  ```bash
  run_zenoh
  ```
* **[젯슨 터미널 2]** 실내 오도메트리 가동:
  ```bash
  run_tank
  ```
* **[젯슨 터미널 3]** 선수를 -X(180°) 방향으로 정지한 상태에서 IMU 보정:
  ```bash
  cal_imu
  ```
  *(보정 성공 후 `/odom` 토픽이 정상 발행되는지 확인)*

---

### 3단계: 주행 테스트 및 제어
* **[젯슨 터미널 4]** 모터 드라이버 가동 (주행할 때만). 현재 `run_motor`의 `/dev/ttyUSB0`은 수조 가이드의 모터 포트와 다릅니다. 젯슨에서 포트를 확인하고 [수조 가이드](TANK_TESTS_GUIDE.md#3-젯슨-imu-보정과-모터테스트-시작)의 명시적 명령을 사용하세요:
  ```bash
  ls -l /dev/ttyCH341USB0
  ros2 launch kaboat_hardware thrusters.launch.py hardware_type:=serial port:=/dev/ttyCH341USB0 allow_port_scan:=false
  ```
* **[젯슨 터미널 5]** 자율주행 노드 가동 (예: LOS B-Spline 추종):
  ```bash
  test_los
  ```
* **[젯슨 터미널]** 출발 및 정지 명령 인가:
  ```bash
  # 출발
  ros2 topic pub --once /start_mission std_msgs/msg/Bool '{data: true}'

  # 비상 정지
  ros2 topic pub --once /emergency_stop std_msgs/msg/Bool '{data: true}'
  ```
  현재 Zenoh 모니터링 브리지는 노트북에서 젯슨으로 `/boat_pose`만 전달합니다. 위 제어 토픽은 노트북에서 발행해도 젯슨에 도착하지 않습니다.
