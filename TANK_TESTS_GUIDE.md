# 🌊 KABOAT 실내 수조 자율운항 테스트 종합 가이드

실내 수조(10m $\times$ 5m) 환경에서 자율운항 알고리즘 및 선체 추진/제어를 검증하기 위한 **4대 테스트 모드**와 **2대 기기(외벽 노트북 ↔ 배 Jetson)** 통합 운용 매뉴얼입니다.

### 💻 기기별 역할 분담 (Machine Ownership)

* **수조 외벽 노트북**: 고정 설치된 YDLIDAR TG-50 라이다 드라이버(`ydlidar_ros2_driver_node`), 수조 기반 배 위치 추적기(`lidar_boat_tracker`), RViz2 3D 종합 관제 모니터링
* **선체 탑재 젯슨 (Jetson)**: 선체 탑재 GQ7 센서 드라이버(`microstrain_inertial_driver`), 실내 오도메트리 융합기(`indoor_lidar_odom`), 스러스터 모터 드라이버(`thruster_driver`), 그리고 **단 1개의 선택된 수조 테스트 노드** (예: `bspline_track_test`)

---

## 1. 4대 테스트 모드 개요

| 모드 이름 | `test_type` | 주요 내용 | 적합한 검증 목적 |
| :--- | :--- | :--- | :--- |
| **직진 주행** | `straight` | (9.0, 3.0)m $\rightarrow$ (1.0, 3.0)m 구간 8m 직진 주행 | 선체 직진성, 차동 추력 균형, 도착 제동 |
| **B-Spline 곡선 추종** | `bspline` | 5개 제어점 기반 clamped B-spline 곡선 궤적(약 7.4m) 추종 | 선회 반응성, Pure Pursuit 경로 추종 안정성 |
| **원형 선회 주행** | `circle` | 수조 중앙(5.0, 2.5)m 기준 반경 1.2m 선회 주행 | 지속 선회 시 횡미끄러짐, 정상상태 원 궤도 유지 |
| **웨이포인트 정점 유지** | `station_keeping` | 지정 위치(5.0, 2.5)m 및 선수각(180°) 고정 (가상 앵커/DP) | 수류에 밀렸을 때 복귀, 제자리 방위각 정렬, 데드밴드 제동 |

> 📐 **B-Spline 곡선 특성 (Clamped B-Spline)**:
> 하드웨어 트랙 테스트의 B-spline 곡선은 시뮬레이션 플래너와 동일한 **open-uniform clamped** 스플라인(차수 3)입니다.
> * **시작 제어점(첫 점: (8.5, 2.0))과 끝 제어점(끝 점: (1.5, 2.5))만 정확히 통과(보간)**합니다.
> * **중간 제어점들((7.0, 3.5), (5.0, 1.5), (3.0, 3.5))은 곡선의 형상(볼록포/convex hull)을 유도하는 제어점(Control Points)이며, 배가 반드시 밟아야 하는 경유점(Waypoints)이 아닙니다.**
> * 기본 제어점 기준 생성 경로의 총 호길이는 코드(`generate_bspline_path()`) 산출 기준 **약 7.43m (약 7.4m)**입니다.

---

## 2. 📋 2대 기기 B-Spline 테스트 순차 실행 절차 (Copy-Pasteable Checklist)

노트북과 젯슨 간의 DDS 통신, 센서 bringup, 모터 안전 확인, 테스트 실행, 관제 명령의 표준 순서입니다.

### [Step 0] 공통 준비: ROS_DOMAIN_ID 일치 및 빌드/소스

노트북과 젯슨 양쪽 모두 같은 네트워크에서 동일한 `ROS_DOMAIN_ID`를 사용해야 합니다.
아래 `42`는 예시이므로 현장에서 선택한 동일한 값으로 맞춥니다. 새 터미널을
열 때마다 ROS 환경과 워크스페이스를 다시 source해야 합니다.

```bash
# 양쪽 기기 공통
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0
echo "ROS_DOMAIN_ID = $ROS_DOMAIN_ID"

# 각 장치에서 실제 저장소 경로로 이동한 뒤 최초 1회 또는 코드 변경 후 빌드
cd /path/to/2026KABOAT_REAL
colcon build --symlink-install
source install/setup.bash

# 두 장치에 배포된 코드 버전이 같은지도 비교
git branch --show-current
git rev-parse --short HEAD
```

### [Step 1] 명시적 시리얼 포트 탐색 (Serial Port Discovery)

하드웨어가 연결된 포트를 사전에 확인합니다.

```bash
# [수조 외벽 노트북] 고정 TG-50 라이다 포트 확인
ls -l /dev/serial/by-id/ /dev/ttyUSB* 2>/dev/null

# [선체 젯슨] 스러스터 아두이노/ESP32 및 GQ7 포트 확인
ls -l /dev/serial/by-id/ /dev/ttyUSB* /dev/ttyACM* /dev/microstrain* 2>/dev/null
```

`lidar_boat_tracker.launch.py`는 `port`를 생략하면 실행 시점의
`/dev/ttyUSB*` 중 정렬상 첫 장치를 기본값으로 고릅니다. USB 장치가 여러 개면
잘못 선택할 수 있으므로 LiDAR도 확인한 포트를 명시하는 것을 권장합니다.
스러스터의 대체 포트 자동 탐색은 `allow_port_scan:=false`가 기본이므로,
모터 제어기 포트를 반드시 명시합니다. 가능한 경우 재부팅해도 이름이 유지되는
`/dev/serial/by-id/...` 경로를 사용합니다.

### [Step 2] 센서 전용 Bringup (모터 미작동 안전 상태)

모터 드라이버를 켜지 않고 센서 및 위치/오도메트리 파이프라인부터 켭니다.

```bash
# ── [외벽 노트북: 터미널 1] 고정 TG-50 라이다 + 배 위치 추적기 + RViz2 실행 ──
# 아래 포트는 Step 1에서 확인한 실제 LiDAR 포트로 변경
ros2 launch kaboat_hardware lidar_boat_tracker.launch.py port:=/dev/ttyUSB0

# ── [선체 젯슨: 터미널 1] 선체 GQ7 IMU + 실내 오도메트리 융합 실행 ──
# 이 launch에서는 스러스터 드라이버를 시작하지 않음
ros2 launch kaboat_hardware indoor_tank.launch.py enable_thrusters:=false
```

### [Step 3] 센서 및 오도메트리 토픽 점검 (Topic Checks)

모터를 켜기 전에 토픽 수신율과 데이터가 정상인지 노트북에서 확인합니다.

```bash
# [외벽 노트북: 터미널 2]
# 1) 라이다 스캔 (≈10 Hz)
ros2 topic hz /scan --qos-reliability best_effort

# 2) 외부 라이다가 추적한 배 위치 (≈10 Hz)
ros2 topic hz /boat_position
ros2 topic echo /boat_position --once

# 3) 선체 GQ7 IMU 데이터 (≈50~100 Hz)
ros2 topic hz /imu/data --qos-reliability best_effort
ros2 topic echo /imu/data --once --field orientation --qos-reliability best_effort

# 4) indoor_lidar_odom이 융합 발행하는 오도메트리
# 검사 타이머는 30 Hz지만 새 위치 표본마다 한 번 발행하므로 현재는 약 10 Hz
ros2 topic hz /odom
ros2 topic echo /odom --once
ros2 topic info /odom --verbose | grep "Publisher count"   # 반드시 1

# 5) TF 브로드캐스트 확인 (odom -> base_link)
ros2 run tf2_ros tf2_echo odom base_link
```

### [Step 4] 스러스터 모터 드라이버 별도 실행 (Separate Thruster Launch)

오도메트리가 `/boat_position` 갱신률에 맞춰 정상 발행되는 것을 확인한 후,
젯슨에서 모터 드라이버를 단독 실행합니다.

```bash
# ── [선체 젯슨: 터미널 2] 실물 스러스터 드라이버 실행 ──
# 아래 port는 Step 1에서 확인한 모터 제어기 포트로 변경
ros2 launch kaboat_hardware thrusters.launch.py \
  hardware_type:=serial \
  port:=/dev/ttyUSB0 \
  allow_port_scan:=false
```
> 🛡️ **모터 안전**: 드라이버는 300ms 워치독이 내장되어 있어 `/cmd_vel` 명령이 들어오기 전까지 1500µs(중립/정지) 상태를 유지합니다.

### [Step 5] B-Spline 테스트 노드 실행 (Wait-for-start Launch)

젯슨에서 B-spline 테스트 노드를 실행합니다. `wait_for_start:=true` 기본 설정에 따라 출발 명령 전까지 대기합니다.

```bash
# ── [선체 젯슨: 터미널 3] B-Spline 곡선 추종 테스트 실행 ──
ros2 launch kaboat_hardware bspline_track_test.launch.py \
  cruise_speed:=0.50 \
  max_angular:=0.60 \
  wait_for_start:=true
```
> ⚠️ **토픽 경로 안내**: 수조 테스트 노드(`bspline_track_test` 등)는 `cmd_mux`를 거치지 않고 **`/cmd_vel`에 직접 속도/조향 명령을 발행**합니다. `autonomy.launch.py`, `cmd_mux`, 다른 수조 테스트 노드 또는 수동 `/cmd_vel` 발행 노드를 동시에 실행하지 마십시오. `/cmd_vel` 발행자가 경쟁하면 제어 명령이 섞입니다.

### [Step 6] 원격 제어 명령 (노트북에서 전송)

노트북에서 주행 시작, 일시 정지, 비상 정지, 궤적 리셋을 제어합니다.

```bash
# ── [외벽 노트북: 터미널 2] ──

# 🚀 1. 출발 명령 (Start)
ros2 topic pub --once /start_mission std_msgs/msg/Bool "{data: true}"
# 또는: ros2 service call /start_test std_srvs/srv/Trigger "{}"

# ⏸️ 2. 일시 정지 (Pause)
ros2 topic pub --once /start_mission std_msgs/msg/Bool "{data: false}"

# 🛑 3. 비상 정지 (Emergency Stop - 모터 즉시 차단)
ros2 topic pub --once /emergency_stop std_msgs/msg/Bool "{data: true}"

# 비상 정지 원인을 확인한 뒤 해제
ros2 topic pub --once /emergency_stop std_msgs/msg/Bool "{data: false}"

# 🧹 4. 주행 궤적 마커 초기화 (Clear Trajectory)
ros2 service call /clear_trajectory std_srvs/srv/Trigger "{}"
```

`/clear_trajectory`는 실제 주행 궤적 이력만 비웁니다. 목표 경로, 제어점,
도착 허용 반경과 미션 진행 상태는 유지됩니다. 테스트 노드를 `Ctrl+C`로
종료하면 0 명령을 한 번 발행하고, 이후에는 스러스터의 300ms 워치독이
중립 출력을 유지합니다.

### [Step 7] RViz2 모니터링 확인

노트북에서 실행된 RViz2(`tank_tracking.rviz`)를 통해 시각화 요소를 확인합니다:

* **B-Spline 계획 경로**: `/bspline_test/path` (`nav_msgs/msg/Path`, 청록색 라인)
* **B-Spline 마커 및 주행 이력**: `/bspline_test/markers` (`visualization_msgs/msg/MarkerArray`)
  * `bspline_control_points`: 노란색 구체 (5개 제어점)
  * `bspline_lookahead_target`: 청록색 구체 (현재 전방 주시점)
  * `goal_tolerance`: 노란색 원 (종점 허용오차 0.40m 반경)
  * `actual_trajectory`: 오렌지색 선 (배가 실제 지나온 주행 궤적)
* **수조 라이다 배 위치**: `/boat_position`, `/detections`, `/lidar_tracker/markers`
* **라이다 스캔**: `/scan`, `/lidar_tracker/filtered_scan`

---

## 3. 다른 테스트 모드 간편 실행 방법

배(Jetson) 내부 터미널에서 **`tank_tests.launch.py`**의 `test_type` 인자를 변경하여 다른 테스트도 실행할 수 있습니다:

```bash
# 1. 직진 주행 테스트
ros2 launch kaboat_hardware tank_tests.launch.py test_type:=straight

# 2. B-Spline 곡선 추종 테스트
ros2 launch kaboat_hardware tank_tests.launch.py test_type:=bspline

# 3. 원형 선회 주행 테스트
ros2 launch kaboat_hardware tank_tests.launch.py test_type:=circle

# 4. 웨이포인트 정점 유지 테스트
ros2 launch kaboat_hardware tank_tests.launch.py test_type:=station_keeping
```

> 💡 **개별 런치 파일로도 실행 가능**:
> * `ros2 launch kaboat_hardware straight_line_test.launch.py`
> * `ros2 launch kaboat_hardware bspline_track_test.launch.py`
> * `ros2 launch kaboat_hardware circle_drive_test.launch.py`
> * `ros2 launch kaboat_hardware station_keeping_test.launch.py`

---

## 4. 테스트 좌표 및 제어점 간편 수정 📍

테스트 경로와 좌표를 변경하고 싶을 때는 **전용 좌표 설정 파일**에서 즉시 수정할 수 있습니다:

* **현재 실행 기본값:** [`src/kaboat_hardware/kaboat_hardware/test_coordinates.py`](src/kaboat_hardware/kaboat_hardware/test_coordinates.py)
* **참고용 YAML:** [`src/kaboat_hardware/config/test_coordinates.yaml`](src/kaboat_hardware/config/test_coordinates.yaml)

현재 launch는 `test_coordinates.yaml`을 자동으로 읽지 않습니다. 좌표 기본값은
Python 파일에서 가져옵니다. `colcon build --symlink-install` 상태에서는 Python
파일을 저장한 뒤 노드를 다시 실행하면 보통 바로 반영되지만, 설치 상태가
확실하지 않다면 `colcon build --symlink-install --packages-select kaboat_hardware`를
다시 실행합니다.

### 수조 좌표계 다이어그램 (10m $\times$ 5m)
```text
      Y = 5.0m ┌────────────────────────────────────────────────────────────┐
               │                                                            │
               │                   (5.0, 2.5) [수조 정중앙]                 │
               │                                                            │
      Y = 0.0m └────────────────────────────────────────────────────────────┘
               X = 0.0m                                            X = 10.0m
```

### `test_coordinates.py` 편집 예시
```python
# 1. 직진 주행: 출발지와 목표지점 X, Y 수정
STRAIGHT_LINE = {
    'start_x': 9.0, 'start_y': 3.0,
    'goal_x':  1.0, 'goal_y':  3.0,
}

# 2. B-Spline 곡선: 제어점(Control Points) 나열 (첫 점과 끝 점만 보간 통과)
BSPLINE_TRACK = {
    'control_points': [
        (8.5, 2.0),   # 시작 제어점 (통과)
        (7.0, 3.5),   # 형상 제어점 (상단 선회 유도)
        (5.0, 1.5),   # 형상 제어점 (하단 선회 유도)
        (3.0, 3.5),   # 형상 제어점 (상단 선회 유도)
        (1.5, 2.5),   # 끝 제어점 (도착)
    ],
    'spline_degree': 3,     # B-spline 차수 (3: Cubic)
    'sample_spacing': 0.05, # [m] 경로 샘플링 간격 (5cm)
}
# (미리 준비된 's_curve', 'u_turn', 'diagonal', 'perimeter' 프리셋도 활용 가능)

# 3. 원형 선회: 중심점 및 반경
CIRCLE_DRIVE = {
    'center_x': 5.0, 'center_y': 2.5,
    'radius': 1.2, 'direction': 'ccw', 'target_laps': 2.0,
}

# 4. 정점 유지 (DP): 목표 유지 위치 및 선수각
STATION_KEEPING = {
    'target_x': 5.0, 'target_y': 2.5,
    'target_yaw_deg': 180.0,
}
```

---

## 5. 주행 파라미터 적용 방법

[`src/kaboat_hardware/config/tank_tests.yaml`](src/kaboat_hardware/config/tank_tests.yaml)은
현재 값들을 모아둔 참고용 파일이며, 현재 `tank_tests.launch.py`와 개별 테스트
launch가 자동으로 로드하지 않습니다. 실행 시 노출된 launch 인자를 넘기거나,
필요한 기본값을 각 테스트 노드/launch에서 변경해야 실제 동작에 반영됩니다.

### 주요 기본 파라미터 현황
* **전진 기본 순항 출력 (`cruise_speed`)**: 기본값 `0.50` (50% 모터 출력 비율)
* **최대 회전 출력 (`max_angular`)**: 기본값 `0.60` (60% 모터 출력 비율)
* **B-Spline Lookahead 거리 (`lookahead_dist`)**: 기본값 `1.2` (1.2m 전방 주시)
* **종점 도착 판정 반경 (`goal_tolerance`)**: 기본값 `0.40` (40cm 이내 도달 시 종료)
* **종점 접근 감속 반경 (`slow_radius`)**: 기본값 `1.5` (1.5m 전방부터 점진적 감속)
* **헤딩 P/D 제어 게인 (`kp_yaw`, `kd_yaw`)**: `kp_yaw: 1.0`, `kd_yaw: 0.15`
* **곡률 감속 가중치 (`curvature_slowdown`)**: `0.25` (급커브 구간 자동 감속)
* **오도메트리 타임아웃 (`odom_timeout_sec`)**: `0.5` (0.5초 동안 /odom 미수신 시 안전 정지)
* **외부 시작 신호 대기 (`wait_for_start`)**: `true` (기본값: 대기 모드 활성화)

---

## 6. RViz 시각화 모니터링 토픽 목록

각 테스트 실행 시 RViz2에서 아래 토픽을 구독하여 목표 경로, 마커, 주행 궤적을 확인합니다:

* **직선 주행**: `/straight_drive/markers` (`visualization_msgs/msg/MarkerArray`, 출발점-종점 라인 및 목표점 구체)
* **B-Spline 곡선**:
  * `/bspline_test/path` (`nav_msgs/msg/Path`, 계획된 B-Spline 곡선 경로)
  * `/bspline_test/markers` (`visualization_msgs/msg/MarkerArray`, 제어점 구체, Lookahead 타겟, 도착 반경 원, 실제 주행 궤적)
* **원형 선회**: `/circle_drive/markers` (`visualization_msgs/msg/MarkerArray`, 원형 궤적 링, 중심점 구체)
* **정점 유지**: `/station_keeping/markers` (`visualization_msgs/msg/MarkerArray`, 목표점 구체, 허용 불감대 링, 목표 방위 화살표)
