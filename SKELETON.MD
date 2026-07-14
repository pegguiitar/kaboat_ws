# 🧭 KABOAT 2026 자율주행 스택 — 코드 구조

이 문서는 `src/` 아래 자율주행 패키지 6개(`kaboat_msgs`, `kaboat_perception`,
`kaboat_behaviors`, `kaboat_control`, `kaboat_mission`, `kaboat_bringup`)의
**구조·설계·실행법**을 다룹니다. 컨테이너 설치나 시뮬레이터 자체(`kaboat_sim`)
사용법은 [README.MD](README.MD)를 참고하세요.

> **지금 상태 — 정직하게**: 이건 "미션을 다 수행하는 완성품"이 아니라
> **뼈대(skeleton)** 입니다. 센서 → 인식 → behavior → 제어까지 데이터가
> 끝까지 흐르고, 배가 전복 없이 목표점으로 이동하고, 미션이 순서대로
> 전환되는 것까지는 시뮬레이션으로 검증했습니다. 하지만 각 behavior의
> 실제 판단 로직(게이트 쌍 매칭, 정밀 도킹 정렬, 진짜 VFH 회피 등)은
> 대부분 자리만 잡아둔 상태입니다. 파일마다 `TODO(팀)` 주석으로 뭐가
> 비어있는지 표시해뒀습니다 — 아래 "패키지별 상태" 참고.

---

## 1. 설계 원칙

**sim과 실물에서 동일하게 동작한다.** 자율주행 노드는 표준 토픽 이름만
구독/발행하고, 그 데이터가 시뮬레이터에서 오는지 실제 센서 드라이버에서
오는지는 몰라도 됩니다. 다르게 만드는 건 `sensor_drivers` 계층
(sim = `ros_gz_bridge`, 실물 = UART/드라이버 노드)과 `localization`
(sim = Gazebo ground-truth, 실물 = `robot_localization` EKF)뿐입니다.

**인식(perception)과 판단(behavior)을 분리한다.** "무엇이 보이는가"는
항상 켜져 있는 상시 레이어가 담당하고, "지금 뭘 해야 하는가"는 현재
미션 하나만 담당합니다. 미션이 바뀌어도 인식 레이어의 데이터 흐름은
전혀 바뀌지 않고, 바뀌는 건 `cmd_mux`가 어느 behavior의 명령을
통과시키느냐뿐입니다.

---

## 2. 노드 구조

```
① 상시 가동 (인식) ────────────────────────────────────────
  sensor_drivers(=gz bridge)  →  /scan, /camera/color/image_raw,
                                  /camera/depth/points, /imu/data, /gps/fix
       │
       ├─▶ obstacle_detector  → /obstacles          (LiDAR 클러스터링)
       └─▶ yolo_detector      → /detections/marks    (HSV 색 인식, YOLO 자리)
       localization(=gz odom / 실물은 robot_localization) → /odom, /tf

② behavior (현재 미션 1개만 주도권) ────────────────────────
  gate_follower   → /cmd/gate      (항로추종)
  station_keeper  → /cmd/station   (위치유지)
  docking_ctrl    → /cmd/dock      (도킹)
  search_circler  → /cmd/search    (탐색)
  obstacle_planner→ /cmd/avoid     (장애물 회피)
  (5개 전부 상주 — 비활성 상태에도 노드는 떠있고 compute_cmd() 만 안 돌림)

제어 ──────────────────────────────────────────────────────
  cmd_mux (현재 state 것만 통과 + 300ms 워치독) → /cmd_vel
  twist2thrust (kaboat_sim 기존 노드 재사용)     → 좌우 추력

관리 ──────────────────────────────────────────────────────
  mission_manager: /odom 과 목표좌표 거리 < 1m → 다음 미션 전환
                   gate → station → dock → search → avoid → done
```

이 구조는 세션 초반에 정리한 [노드 구조 다이어그램](https://claude.ai/code/artifact/2d88674d-c2d7-4c01-ba92-018b8bf7dc22)
그대로 구현했습니다 (그 다이어그램의 4가지 보완사항 — 거리 기반 전환,
cmd_mux 워치독, 공통 회피 라이브러리, robot_localization 재사용 — 전부 반영).

---

## 3. 센서 토픽 인터페이스 (sim ↔ 실물 공통 계약)

`sensor_drivers` 계층의 계약입니다. sim 은 Gazebo 센서 →
`kaboat_sim/config/bridge_config.yaml` 로 아래 **표준 토픽 이름**에 맞춰
발행하며, 실물 전환 시 각 센서 드라이버가 같은 토픽으로 발행하면 아래
모든 패키지는 코드 변경 없이 그대로 동작합니다.

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

시뮬레이터 센서 스펙(FOV·해상도·노이즈)은 실물 데이터시트 기준으로
맞춰뒀습니다 — TG-50(2000점/회전, 50m), D455(87° depth FOV, 1280×720),
EBIMU(100Hz), GPS(수평 ~1.5m/수직 ~3m 노이즈) 전부
`kaboat_sim/urdf/wamv_kaboat.xacro`에 주석과 함께 있습니다.
⚠️ gz `navsat` 센서는 수평 노이즈 stddev를 미터가 아니라 위경도
**도(degree) 단위로 그대로 적용**합니다 — 미터→도 환산이 xacro 주석에
있으니 GPS 노이즈를 다시 튜닝할 땐 반드시 참고하세요 (실측으로 확인한
내용, 처음엔 이걸 몰라서 위도가 수백 km 단위로 흔들리는 버그를
겪었습니다).

---

## 4. 패키지별 상태

### `kaboat_msgs` — 완성
공용 메시지. `Obstacle`/`ObstacleArray`(위치+반지름), `Mark`/`MarkArray`
(색/모양/신뢰도/방위/거리). 더 필요한 필드가 생기면 여기에 추가.

### `kaboat_perception` — 골격 + 일부 알고리즘
- **`obstacle_detector.py`**: 인접 빔 거리차 기반 클러스터링으로 `/scan` →
  `/obstacles`. **실제 부표 좌표와 교차검증 완료** (오차 없음, LiDAR가
  부표 표면을 맞히는 것까지 기하학적으로 정합). TODO(팀): DBSCAN 등
  제대로 된 클러스터링, `/camera/depth/points` 융합, 시간 필터링.
- **`yolo_detector.py`**: **아직 YOLO 아님** — HSV 색 블롭 검출
  placeholder. `shape`은 항상 `"unknown"`, `distance`는 항상 `-1.0`.
  TODO(팀): YOLO 추론으로 교체, depth 이미지로 거리 채우기.

### `kaboat_behaviors` — 골격, 판단 로직은 대부분 placeholder
- **`behavior_base.py`**: 모든 behavior의 공통 base. `/mission/state`로
  active 여부 판단, PD 제어(`YAW_KP=0.3`, `YAW_KD=0.15`)로 목표점 추종하는
  `seek_goal()` 제공. 속도 상한 `max_linear=0.20`, `max_angular=0.05`.
  ⚠️ **이 상한값은 실측으로 잡은 안전값입니다** — 축소 선체(1.1m)는
  관성이 작아 P만 쓰면 요 발진 → 전복하는 걸 직접 겪었습니다 (아래
  "실측으로 잡은 안전값" 참고). 함부로 올리지 마세요.
- **`obstacle_avoidance_utils.py`**: 공통 회피-보정 함수. 5개 behavior가
  전부 이 모듈의 `apply_repulsion()`을 import해서 씁니다 — 각자 따로
  구현하면 미묘하게 다른 회피 동작이 생기는 걸 막기 위함. TODO(팀):
  VFH(valley 탐색)로 교체.
- **`gate_follower.py`**: red+green 마크가 같이 보이면 중점 방위로 조향,
  아니면 goal로 이동. TODO(팀): 게이트 쌍 매칭(제일 가까운 것끼리),
  통과 판정.
- **`station_keeper.py`**: goal 접근 + 5m/5초 체류 판정 로그. TODO(팀):
  바람/파도 드리프트 보상.
- **`docking_ctrl.py`**: 파라미터로 지정한 색(`target_color`) 마크가
  보이면 저속 접근. TODO(팀): 표식 종류(십자/삼각) 매칭, 실제 접안 정렬.
- **`search_circler.py`**: 구역 도착 후 마크 색 따라 좌/우 선회. TODO(팀):
  규정에 맞는 정확한 선회 방향/횟수.
- **`obstacle_planner.py`**: goal 추종 + 회피 게인 강화. **여기가
  기존에 설계했던 VFH + B-spline 로컬 플래너가 들어갈 자리입니다** —
  지금은 `obstacle_avoidance_utils`의 단순 반발 조향으로 대체되어 있습니다.

### `kaboat_control` — 완성
- **`cmd_mux.py`**: `/mission/state`에 맞는 `/cmd/*` 토픽만 `/cmd_vel`로
  통과. **300ms 워치독**: 활성 behavior의 명령이 끊기거나 state가
  목록에 없으면(`done` 등) 즉시 정지(0) 발행. 시뮬레이션에서 전체
  미션 완주 후 `done` 전이 시 페일세이프가 정확히 작동하는 것까지
  확인했습니다.

### `kaboat_mission` — 완성 (규칙은 팀 합의로 확정)
- **`mission_manager.py`**: `/odom`과 현재 미션 목표좌표
  (`kaboat_bringup/config/mission_params.yaml`)의 거리가
  `transition_radius`(기본 1.0m) 이내면 다음 미션으로 전환. behavior가
  완료 신호를 발행할 필요가 없는 구조 — 위치만으로 판정합니다.
  순서는 `gate → station → dock → search → avoid → done` 고정
  (`MISSION_ORDER` 상수).

### `kaboat_bringup` — 완성
- **`launch/autonomy.launch.py`**: 상시 인식 2개 + behavior 5개 +
  `cmd_mux` + `twist2thrust`(kaboat_sim 재사용) + `mission_manager`를
  한 번에 띄웁니다.
- **`config/mission_params.yaml`**: 미션별 목표좌표. 기본값은 원점
  기준 근거리 사각 경로(스모크 테스트용)이고, 주석에 실제
  `kaboat_2026_comprehensive_mission` 경기장 좌표 예시가 있습니다 —
  실전 검증 시 이걸로 교체하세요.
- **`config/ekf.yaml`**: **실물 전용 placeholder**. sim은 Gazebo
  ground-truth `/odom`을 쓰므로 이 파일을 안 씁니다. 실물 전환 시
  `robot_localization`의 `ekf_node`+`navsat_transform_node`를 이
  설정으로 띄우면 `/odom`은 그대로 유지되고 상위 노드는 변경 불필요.

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

미션 진행 상황은 `mission_manager` 로그로 확인합니다:
```
미션 [1/5] 'gate' 시작 — 목표 (8.0, 0.0)
목표 1.0m 이내 진입 — 'gate' 완료
미션 [2/5] 'station' 시작 — 목표 (8.0, 8.0)
...
종합임무 전체 완료 — done (cmd_mux 정지 상태)
```

### 개별 노드만 따로 확인하고 싶을 때
```bash
ros2 run kaboat_perception obstacle_detector
ros2 topic echo /obstacles          # LiDAR 클러스터링 결과 확인
ros2 topic hz /imu/data             # 센서 발행 주파수 확인
```

---

## 6. 실측으로 잡은 안전값 (함부로 바꾸지 말 것)

축소 선체(1.1m, `kaboat_hull.xacro`의 `target_length`)는 원본 4.9m WAM-V
대비 관성이 `s⁵`로 줄어서 **원본보다 훨씬 민첩하고, 그만큼 쉽게
전복**됩니다. 개발 중 직접 겪은 두 크래시:

1. **추력 스케일 250 → 배 전복**: `twist2thrust.py`의 `scale`이 원본
   4.9m 배 기준 250이었는데, 축소 선체(최대추력 118.6N, 질량 ~2kg)엔
   너무 커서 `cmd_vel.linear.x=0.2`만 줘도 전복했습니다.
   **`scale=60`으로 낮춰서 해결** — `linear.x=0.2` → 12N (직진 주행
   테스트로 검증된 안전값).
2. **P 제어만으로 조향 → 요 발진 → 전복**: `behavior_base.seek_goal()`이
   처음엔 `angular.z = 0.8 * heading_error`만 썼는데, 민첩한 선체가
   목표 방향으로 홱홱 틀다가 발진해서 뒤집혔습니다.
   **PD 제어(`YAW_KD=0.15`로 요레이트 감쇠)로 해결**, `max_angular`도
   0.15 → 0.05로 낮췄습니다.

배 크기나 무게를 바꾸면 이 값들(`scale`, `max_linear`, `max_angular`,
`YAW_KP`, `YAW_KD`)을 반드시 헤드리스로 재검증하세요 — 최소 몇 분 이상
5개 미션을 완주시켜보고 판단하는 걸 권장합니다 (README.MD §3의 물리
스텝 경고와 같은 이유로, 짧게 보면 안정적으로 오판하기 쉽습니다).

---

## 7. 알려진 이슈

- **게이트 마커(빨간 삼각뿔, arena 좌표 (14,±18))가 LiDAR에 안 잡힘**:
  범위(30m 이내, 22.8m) 안인데도 `obstacle_detector`가 검출 못했습니다.
  2D LiDAR 장착 높이나 마커 형상 문제로 추정 — `gate_follower`가
  LiDAR 융합을 쓰게 되면 먼저 원인을 파야 합니다.
- **`yolo_detector`의 HSV 임계값은 시뮬레이션 부표 원색 기준**입니다.
  실제 카메라/조명에서는 재캘리브레이션이 필요합니다.
