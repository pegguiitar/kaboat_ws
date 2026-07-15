# 🧭 KABOAT 2026 자율주행 스택 — 코드 구조

이 문서는 `src/` 아래 자율주행 패키지 6개(`kaboat_msgs`, `kaboat_perception`,
`kaboat_behaviors`, `kaboat_control`, `kaboat_mission`, `kaboat_bringup`)의
**구조·설계·실행법**을 다룹니다. 컨테이너 설치나 시뮬레이터(`kaboat_sim`)
사용법은 [README.MD](README.MD)를 참고하세요.

> **현재 상태**: 완성품이 아니라 **뼈대(skeleton)** 입니다. 센서→인식→behavior→
> 제어까지 데이터가 끝까지 흐르고 배가 전복 없이 미션을 순서대로 전환하며
> 이동하는 것까진 sim 으로 검증했지만, 각 behavior 의 실제 판단 로직(게이트
> 쌍 매칭, 정밀 도킹, 진짜 VFH 회피 등)은 대부분 `TODO(팀)` 자리만 잡아둔
> 상태입니다 — 아래 §4 참고.

---

## 1. 설계 원칙

**sim과 실물에서 동일하게 동작한다.** 자율주행 노드는 표준 토픽 이름만
구독/발행하고 데이터 출처(sim/실물)는 몰라도 됩니다. 다른 건
`sensor_drivers`(sim=`ros_gz_bridge`, 실물=UART/드라이버)와
`localization`(sim=Gazebo ground-truth, 실물=`robot_localization` EKF)뿐입니다.

**인식(perception)과 판단(behavior)을 분리한다.** "무엇이 보이는가"는 상시
레이어가, "지금 뭘 해야 하는가"는 현재 미션 하나가 담당합니다. 미션이 바뀌어도
인식 데이터 흐름은 그대로고, `cmd_mux`가 통과시키는 behavior 만 바뀝니다.

---

## 2. 노드 구조

```
① 인식 레이어 (v5: 프로세스는 전부 상시, YOLO 추론만 게이팅) ──
  sensor_drivers(=gz bridge)  →  /scan, /camera/color/image_raw,
                                  /camera/depth/points, /imu/data, /gps/fix
       │
       ├─▶ occupancy_grid      → /occupancy_grid          (장애물 격자 — v5 단일 장애물 표현)
       ├─▶ buoy_detector       → /detections/buoys        (HSV 상시 · 학습 불필요)
       └─▶ dock_mark_detector  → /detections/dock_marks   (YOLO 자리 · dock state 만 추론)
       localization(=gz odom / 실물은 robot_localization) → /odom, /tf

  ── 공유 버스 ──
  /occupancy_grid · /detections/buoys · /odom
  = behavior_base 가 일괄 구독하는 표준 토픽 묶음 (모든 behavior 공유).
  회피는 /occupancy_grid 를 소비 (apply_repulsion). /detections/dock_marks
  만 예외 — docking_ctrl 단독 구독.

② behavior (현재 미션 1개만 주도권) ────────────────────────
  gate_follower   → /cmd/gate      (항로추종 — buoys)
  station_keeper  → /cmd/station   (위치유지)
  docking_ctrl    → /cmd/dock      (도킹 — dock_marks)
  search_circler  → /cmd/search    (탐색 — buoys)
  obstacle_planner→ /cmd/avoid     (장애물 회피 — grid 기반 DRI/VFH 자리)
  (5개 전부 상주 — 비활성 상태에도 노드는 떠있고 compute_cmd() 만 안 돌림)

제어 ──────────────────────────────────────────────────────
  cmd_mux (현재 state 것만 통과 + 300ms 워치독) → /cmd_vel
  twist2thrust (kaboat_sim 기존 노드 재사용)     → 좌우 추력

관리 ──────────────────────────────────────────────────────
  mission_manager: /odom 과 목표좌표 거리 < 1m → 다음 미션 전환
                   gate → station → dock → search → avoid → done
                   /detector/enable (Bool, latched) — dock state 에서만 true
                   → dock_mark_detector 의 YOLO 추론 게이팅
```

이 구조는 **node_structure v5**(`node_structure.svg`) 기준입니다. 핵심 분리는
부표·게이트 색=**buoy_detector**(HSV·상시) / 도킹 표식=**dock_mark_detector**
(YOLO·dock state 만). `/detections/*` 인터페이스가 고정이라 백엔드(HSV→YOLO)를
갈아끼워도 behavior 는 변경 불필요합니다.

> **공유 버스**는 별도 노드가 아니라 `behavior_base.py` 가 일괄 구독하는 표준
> 토픽 묶음을 뜻합니다(ROS 2 는 토픽 자체가 버스 — rqt_graph 엔 토픽 화살표로 보임).

---

## 3. 센서 토픽 인터페이스 (sim ↔ 실물 공통 계약)

sim 은 Gazebo 센서 → `kaboat_sim/config/bridge_config.yaml` 로 아래 **표준 토픽
이름**에 맞춰 발행합니다. 실물 전환 시 각 센서 드라이버가 같은 토픽으로 발행하면
아래 모든 패키지는 코드 변경 없이 동작합니다.

| 센서 | 실물 기종 | sim 소스 | 표준 토픽 | 타입 |
|---|---|---|---|---|
| 2D LiDAR | TG 50 | gz `gpu_lidar` | `/scan` | `LaserScan` |
| Depth Cam (RGB) | Intel D455 | gz `rgbd_camera` | `/camera/color/image_raw` | `Image` |
| Depth Cam (Depth image) | Intel D455 | gz `rgbd_camera` | `/camera/depth/image_raw` | `Image` |
| Depth Cam (PointCloud) | Intel D455 | gz `rgbd_camera` | `/camera/depth/points` | `PointCloud2` |
| Depth Cam (CameraInfo) | Intel D455 | gz `rgbd_camera` | `/camera/camera_info` | `CameraInfo` |
| IMU | EBIMU-9DOFv5-R3 | gz `imu` | `/imu/data` | `Imu` |
| GPS | 미정 | gz `navsat` | `/gps/fix` | `NavSatFix` |
| Odometry | sim ground truth / 실물은 EKF | gz `odometry` | `/odom` | `Odometry` |
| TF | sim ground truth / robot_state_publisher | gz pose / robot description | `/tf` | `TFMessage` |

시뮬레이터 센서 스펙(FOV·해상도·노이즈)은 실물 데이터시트 기준 —
TG-50(2000점/회전, 50m), D455(87° depth FOV, 1280×720), EBIMU(100Hz),
GPS(수평 ~1.5m/수직 ~3m) 전부 `kaboat_sim/urdf/wamv_kaboat.xacro` 주석에
있습니다. ⚠️ gz `navsat` 는 수평 노이즈 stddev 를 미터가 아니라 위경도
**도(degree) 단위로 적용**합니다 — GPS 노이즈 재튜닝 시 xacro 주석의 미터→도
환산을 반드시 참고하세요(안 그러면 위도가 수백 km 로 튐).

---

## 4. 패키지별 상태

### `kaboat_msgs` — 완성
공용 메시지. `Obstacle`/`ObstacleArray`(위치+반지름), `Mark`/`MarkArray`
(색/모양/신뢰도/방위/거리). 필드가 더 필요하면 여기에 추가.

### `kaboat_perception` — 골격 + 일부 알고리즘 (v5 정렬)
- **`occupancy_grid.py`** (v5 · 장애물 표현 단독 담당, 상시): `/scan` Bresenham
  레이캐스팅으로 log-odds 누적(경로 셀=`L_FREE`, 히트 끝점=`L_OCC`) +
  `/camera/depth/points` z-band·거리(`CAM_RANGE`) 필터로 LiDAR 사각지대 보완
  (점유 증거만) + IMU 롤/피치 `TILT_MAX`(15°) 초과 스캔 게이팅. 셀 좌표는
  **월드 정수격자(`floor(x/res)`, global)** 이고, 그 격자를 **배 중심 20×20m 창으로
  잘라 담는 고정 크기 numpy 배열(rolling grid)** 을 쓴다 — 배열 인덱스는 좌표가
  아니라 창에서의 위치이고, 배가 움직이면 anchor(배열 (0,0)의 global 인덱스)를
  절대위치에서 다시 잡아 정수 셀 단위 `np.roll` 로 창만 민다(셀의 global 정체성
  불변, 새 가장자리만 unknown). 월드 축 정렬 유지라 heading 회전 오차 없고,
  발행 origin 을 격자에 스냅해 저장/발행 셀이 1:1(±1셀 aliasing 없음). 단독
  실행: `ros2 run kaboat_perception occupancy_grid`. TODO(팀): decay(노후 셀
  감쇠), 거리·빔각도 함수형 inverse sensor model(현재 `L_OCC`/`L_FREE` 상수).
- **`buoy_detector.py`** (v5, 상시·학습 불필요): HSV 색 블롭 → `/detections/buoys`.
  `buoy_detector_pipeline.svg` 5단계 중 **① HSV 마스크만 실동작**, ②블롭 기하 →
  ③수면 컨텍스트 → ④LiDAR 교차확인 → ⑤N-of-M 시간 추적은 훅만 자리. TODO(팀): ②~⑤.
- **`dock_mark_detector.py`** (v5, 도킹 state 만 추론): `/detector/enable`
  (latched) 이 true 인 동안만 이미지 처리 → `/detections/dock_marks`.
  **아직 YOLO 아님** — `hsv_blobs()` 재사용 placeholder, `shape="unknown"`.
  TODO(팀): YOLOv8 모양 3클래스 + bbox 내부 HSV 색 + depth 거리.

### `kaboat_behaviors` — 골격, 판단 로직은 대부분 placeholder
- **`behavior_base.py`**: 모든 behavior 공통 base. `/mission/state`로 active
  판단, PD 제어(`YAW_KP=0.3`, `YAW_KD=0.15`)로 목표점 추종하는 `seek_goal()`
  제공. 속도 상한 `max_linear=0.20`, `max_angular=0.05`. ⚠️ 이 값들은 실측
  안전값 — 함부로 올리지 마세요(이유는 §6).
- **`obstacle_avoidance_utils.py`**: 공통 회피-보정 함수. 5개 behavior 가 전부
  `apply_repulsion(cmd, occupancy_grid, odom)` 을 import(각자 구현 시 회피가
  제각각 되는 걸 방지). `/occupancy_grid`(점유 셀) 입력, 격자가 채워지므로 반발
  회피 실동작. TODO(팀): VFH(valley 탐색)로 교체.
- **`gate_follower.py`**: red+green 이 같이 보이면 중점 방위로 조향, 아니면 goal.
  TODO(팀): 게이트 쌍 매칭(제일 가까운 것끼리), 통과 판정.
- **`station_keeper.py`**: goal 접근 + 5m/5초 체류 판정. TODO(팀): 바람/파도 드리프트 보상.
- **`docking_ctrl.py`**: `/detections/dock_marks` **단독 구독**(v5). 지정 색
  (`target_color`) 마크 보이면 저속 접근. TODO(팀): YOLO 후 `target_shape` 매칭, 접안 정렬.
- **`search_circler.py`**: 구역 도착 후 마크 색 따라 좌/우 선회. TODO(팀): 규정에 맞는 선회 방향/횟수.
- **`obstacle_planner.py`**: goal 추종 + 회피 게인 강화. **VFH + B-spline 로컬
  플래너가 들어갈 자리** — 지금은 `obstacle_avoidance_utils` 단순 반발 조향으로 대체.

### `kaboat_control` — 완성
- **`cmd_mux.py`**: `/mission/state`에 맞는 `/cmd/*` 만 `/cmd_vel`로 통과.
  **300ms 워치독** — 활성 behavior 명령이 끊기거나 state 가 목록에 없으면
  (`done` 등) 즉시 정지(0). `done` 전이 시 페일세이프 작동 검증 완료.

### `kaboat_mission` — 완성 (규칙은 팀 합의로 확정)
- **`mission_manager.py`**: `/odom`과 목표좌표(`kaboat_bringup/config/mission_params.yaml`)
  거리가 `transition_radius`(기본 1.0m) 이내면 다음 미션 전환 — behavior 완료
  신호 없이 위치만으로 판정. 순서 `gate → station → dock → search → avoid → done`
  고정(`MISSION_ORDER`). 전이마다 `/detector/enable`(latched)도 발행 — `dock`
  진입 시 true, 이탈 시 false(YOLO 추론 게이팅).

### `kaboat_bringup` — 완성
- **`launch/autonomy.launch.py`**: 상시 인식 + behavior 5개 + `cmd_mux` +
  `twist2thrust`(kaboat_sim 재사용) + `mission_manager` 를 한 번에 실행.
- **`config/mission_params.yaml`**: 미션별 목표좌표. 기본값은 근거리 사각 경로
  (스모크 테스트용), 주석에 실제 `kaboat_2026_comprehensive_mission` 경기장
  좌표 예시 — 실전 검증 시 교체.
- **`config/ekf.yaml`**: **실물 전용 placeholder**. sim 은 Gazebo ground-truth
  `/odom` 을 쓰므로 미사용. 실물 전환 시 `robot_localization` 의
  `ekf_node`+`navsat_transform_node` 를 이 설정으로 띄우면 `/odom` 유지, 상위 노드 변경 불필요.

---

## 5. 실행법

```bash
# 터미널 1 — 시뮬레이터 (센서 발행 + /odom + 추력 입력)
docker exec -it kaboat_sim_container bash
source /opt/ros/humble/setup.bash && source /workspace/kaboat_ws/install/setup.bash
ros2 launch kaboat_sim simulation.launch.py     # 또는 headless:=True
# Gazebo 창에서 재생(▶) 버튼 누르기

# 터미널 2 — 자율주행 스택 전체
docker exec -it kaboat_sim_container bash
source /opt/ros/humble/setup.bash && source /workspace/kaboat_ws/install/setup.bash
ros2 launch kaboat_bringup autonomy.launch.py
```

미션 진행은 `mission_manager` 로그로 확인:
```
미션 [1/5] 'gate' 시작 — 목표 (8.0, 0.0)
목표 1.0m 이내 진입 — 'gate' 완료
...
종합임무 전체 완료 — done (cmd_mux 정지 상태)
```

### 개별 노드만 따로 확인
```bash
ros2 run kaboat_perception occupancy_grid
ros2 topic echo /occupancy_grid --once   # 장애물 격자 발행 확인
ros2 topic hz /imu/data                  # 센서 발행 주파수 확인
```

---

## 6. 실측으로 잡은 안전값 (함부로 바꾸지 말 것)

축소 선체(1.1m, `kaboat_hull.xacro`의 `target_length`)는 원본 4.9m WAM-V 대비
관성이 `s⁵`로 줄어 **훨씬 민첩하고 쉽게 전복**됩니다. 개발 중 겪은 두 크래시:

1. **추력 스케일 250 → 전복**: `twist2thrust.py`의 `scale`이 원본 기준 250이라
   축소 선체(최대추력 118.6N, 질량 ~2kg)엔 너무 커서 `linear.x=0.2`만 줘도
   전복. **`scale=60`으로 해결**(`linear.x=0.2` → 12N).
2. **P 제어만으로 조향 → 요 발진 → 전복**: `seek_goal()`이 처음엔
   `angular.z = 0.8 * heading_error`만 써서 발진. **PD(`YAW_KD=0.15`)로 해결**,
   `max_angular`도 0.15 → 0.05로 낮춤.

배 크기/무게를 바꾸면 이 값들(`scale`, `max_linear`, `max_angular`, `YAW_KP`,
`YAW_KD`)을 반드시 헤드리스로 재검증하세요 — 짧게 보면 안정적으로 오판하기
쉬우니 최소 몇 분 이상 5개 미션 완주로 판단(README §3 물리 스텝 경고와 같은 이유).

---

## 7. 알려진 이슈

- **게이트 마커(빨간 삼각뿔, arena (14,±18))가 LiDAR에 안 잡힘**: 범위(22.8m,
  30m 이내) 안인데도 2D LiDAR(`occupancy_grid` 레이캐스팅)가 검출 못함. 장착
  높이나 마커 형상 문제로 추정 — `gate_follower`가 LiDAR 융합을 쓰면 먼저 원인 규명.
- **`buoy_detector`/dock_mark placeholder 의 HSV 임계값은 sim 부표 원색 기준**.
  실제 카메라/조명에선 재캘리브레이션 필요(셔터·AWB 고정 후 현장 캘리브).

---

## 8. 수정 이력 (변경 로그)

> 스택 알고리즘·설정을 의미 있게 바꾸면 **최신 항목을 맨 위에** 추가:
> `### YYYY-MM-DD HH:MM — 요약` + 무엇을/왜 몇 문장. 파일 세부는 커밋 메시지로.

### 2026-07-15 16:55 — occupancy_grid 를 rolling grid 로 리팩터
저장을 월드좌표 `dict`+`prune` → global 셀은 그대로 두고 배 중심 20×20m 창으로
잘라 담는 고정 numpy 배열(정수 `np.roll`)로 교체해 `prune` 제거. 값·정확도는 동일, publish 는 벡터화로 경량화.
