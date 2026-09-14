# 🌊 KABOAT 실내 수조 자율운항 테스트 종합 가이드

실내 수조(10m $\times$ 5m) 환경에서 자율운항 알고리즘 및 선체 추진/제어를 검증하기 위한 **4대 테스트 모드**와 통합 실행 매뉴얼입니다.

---

## 1. 4대 테스트 모드 개요

| 모드 이름 | `test_type` | 주요 내용 | 적합한 검증 목적 |
| :--- | :--- | :--- | :--- |
| **직진 주행** | `straight` | (9.0, 3.0)m $\rightarrow$ (1.0, 3.0)m 구간 8m 직진 주행 | 선체 직진성, 차동 추력 균형, 도착 제동 |
| **B-Spline 곡선 추종** | `bspline` | 5개 제어점 기반 S자 곡선 궤적(10.4m) 추종 | 선회 반응성, Pure Pursuit 경로 추종 안정성 |
| **원형 선회 주행** | `circle` | 수조 중앙(5.0, 2.5)m 기준 반경 1.2m 선회 주행 | 지속 선회 시 횡미끄러짐, 정상상태 원 궤도 유지 |
| **웨이포인트 정점 유지** | `station_keeping` | 지정 위치(5.0, 2.5)m 및 선수각(180°) 고정 (가상 앵커/DP) | 수류에 밀렸을 때 복귀, 제자리 방위각 정렬, 데드밴드 제동 |

---

## 2. 간편 실행 방법 (통합 런치 🚀)

배(Jetson) 내부 터미널에서 **`tank_tests.launch.py`** 하나로 원하는 테스트를 즉시 변경 실행할 수 있습니다:

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

## 3. 원격 관제 및 명령 (노트북에서 전송)

노드가 켜지면 기본적으로 **출발 신호 대기 모드(`wait_for_start:=true`)**로 진입합니다. 안전을 확인한 뒤 원격 노트북에서 명령을 전송합니다:

### 🚀 출발 신호 (Start)
```bash
ros2 topic pub --once /start_mission std_msgs/msg/Bool "{data: true}"
```
*(또는 서비스 호출: `ros2 service call /start_test std_srvs/srv/Trigger`)*

### ⏸️ 일시 정지 (Pause)
```bash
ros2 topic pub --once /start_mission std_msgs/msg/Bool "{data: false}"
```

### 🛑 비상 정지 (Emergency Stop)
```bash
ros2 topic pub --once /emergency_stop std_msgs/msg/Bool "{data: true}"
```

---

## 4. 파라미터 통합 설정 (`tank_tests.yaml`)

모든 테스트의 속도, 게인, 목표 좌표는 [`src/kaboat_hardware/config/tank_tests.yaml`](file:///home/jiwoo/Desktop/2026KABOAT_REAL/src/kaboat_hardware/config/tank_tests.yaml) 파일 한 곳에서 편리하게 수정할 수 있습니다.

### 주요 파라미터 튜닝 포인트
* **기본 전진 출력 (`cruise_speed`)**: 기본값 `0.12` (12% 출력). 수조 크기에 맞추어 안전한 저속으로 설정됨.
* **원 주행 반경/방향 (`radius`, `direction`)**:
  * `radius: 1.2` (기본 1.2m 반경, 지름 2.4m 원)
  * `direction: "ccw"` (반시계) 또는 `"cw"` (시계)
  * `target_laps: 2.0` (2바퀴 돌고 자동 종료. 0 설정 시 무한 회전)
* **웨이포인트 유지 불감대 (`pos_deadband`, `yaw_deadband_deg`)**:
  * `pos_deadband: 0.15` (15cm 이내로 안착 시 모터 휴지)
  * `yaw_deadband_deg: 8.0` (목표 각도 오차 8도 이내 시 회전 정지)

---

## 5. RViz 시각화 모니터링 토픽

각 테스트 실행 시 RViz2에서 `MarkerArray` 디스플레이를 추가하고 아래 토픽을 구독하면 목표 궤적이 3D 화면에 표시됩니다:

* **직진 주행**: `/straight_drive/markers` (출발점-종점 라인, 목표점 구)
* **B-Spline**: `/bspline_track/markers` (스플라인 곡선 경로 스트립, 제어점)
* **원형 선회**: `/circle_drive/markers` (원형 궤적 링, 중심점 구)
* **정점 유지**: `/station_keeping/markers` (목표점 구, 허용 불감대 링, 목표 방위 화살표)
