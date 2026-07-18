# 🧭 KABOAT 2026 자율주행 스택 — 코드 구조

이 문서는 `src/` 아래 자율주행 패키지 6개(`kaboat_msgs`, `kaboat_perception`,
`kaboat_behaviors`, `kaboat_control`, `kaboat_mission`, `kaboat_bringup`)의
**구조·설계·실행법**을 다룹니다. 컨테이너 설치나 시뮬레이터(`kaboat_sim`)
사용법은 [README.MD](README.MD)를 참고하세요.

> **현재 상태**: 완성품이 아니라 **뼈대(skeleton)** 입니다. 센서→인식→behavior→
> 제어까지 데이터가 끝까지 흐르고 배가 전복 없이 미션을 순서대로 전환하며
> 이동하는 것까진 sim 으로 검증했지만, 각 behavior 의 실제 판단 로직(게이트
> 쌍 매칭, 정밀 도킹 등)은 대부분 `TODO(팀)` 자리만 잡아둔
> 상태입니다(장애물 회피만 완성 — [AVOIDANCE.MD](AVOIDANCE.MD)) — 아래 §4 참고.

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
  회피는 /occupancy_grid 로 위험도 지도를 만든다. /detections/dock_marks
  만 예외 — docking_ctrl 단독 구독.

② behavior (현재 미션 1개만 주도권) ────────────────────────
  gate_follower   → /cmd/gate      (항로추종 — buoys)
  station_keeper  → /cmd/station   (위치유지)
  docking_ctrl    → /cmd/dock      (도킹 — dock_marks)
  search_circler  → /cmd/search    (탐색 — buoys)
  obstacle_planner→ /cmd/avoid     (장애물 회피 — 위험도 지도 + 곡선 경로, 완성)
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
  제공. 명령 상한 `max_linear=0.20`, `max_angular=0.05`. ⚠️ 이 값들은 실측
  안전값 — 함부로 올리지 마세요(이유는 §6). **m/s 가 아니라 추력 명령**이라는
  점도 주의(0.20 → 실제 약 1.5 m/s, §6).
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
- **`dri.py`** — 완성. `/occupancy_grid` → 위험도(DRI) 격자. 점유 셀마다 등방
  가우시안 splat(가깝거나 정면일수록 크고 진하게), 셀 결합은 **max**. ROS 무의존
  순수 numpy(N3), `test/test_dri.py` 18개. 장애물 밭 552셀 9.4ms.
- **`bspline_planner.py`** — 완성. 위험도 격자 위에 제어점 6개짜리 곡선 경로를
  만들고(`generate`)·검사하고(`check`)·수리한다(`repair`). ROS 무의존 순수
  라이브러리, `test/test_bspline_planner.py` 23개.
- **`avoid_fsm.py`** — 완성. 회피 조종 정책(따라가기/후진 탈출 상태기계 +
  경로 추종 명령). 상태와 시간을 갖는 유일한 층이고, ROS 무의존이라 가짜
  시계로 전이 로직을 테스트한다(`test/test_avoid_fsm.py` 28개).
- **`obstacle_planner.py`** — 완성. 위 3개 라이브러리를 배선하는 얇은 ROS 노드
  (구독·발행·파라미터만, 판단 로직 없음). 알고리즘·파라미터 설명은
  [AVOIDANCE.MD](AVOIDANCE.MD).
  ⚠️ `build_dri` 는 매 tick(10Hz) 호출 — 위험도는 배 pose 의 함수라 격자
  주기(1Hz)에 묶으면 안 됨. 시간은 벽시계 말고 `/odom` 스탬프를 쓸 것.

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
   전복. `scale=60`(→ 12N)으로 해결 — 이 `scale` 상수 자체는 2026-07-17
   정규화로 폐기됐고(아래), 12N 이라는 검증 추력은 그대로 유지된다.
2. **P 제어만으로 조향 → 요 발진 → 전복**: `seek_goal()`이 처음엔
   `angular.z = 0.8 * heading_error`만 써서 발진. **PD(YAW_KD)로 해결**,
   차동 상한도 ±9N → ±3N 으로 낮춤.

⚠️ **명령 의미(2026-07-17 정규화): `linear.x`/`angular.z` = 최대 추력 대비
비율 [-1,1]** — 속도(m/s)가 아닙니다. 드라이버(`twist2thrust`, 실물은 ESC)가
자기 스펙 `max_thrust`(sim 118.6N)를 곱해 N 으로 변환하고 ±max_thrust 로
포화시킵니다. 구 의미(`scale=60` 임의상수 × cmd)는 폐기 — 안전값들은 추력
N 동등으로 환산돼 코드에 `N값/MAX_THRUST` 형태로 적혀 있습니다:

| 항목 | 구 (scale=60) | 신 (비율) | 물리 | sim 실측 거동 |
|---|---|---|---|---|
| `max_linear` | 0.20 | `12/118.6` ≈ 0.101 | 12N (최대의 ~10%) | ≈ 1.5 m/s |
| `max_angular` | 0.05 | `3/118.6` ≈ 0.025 | 차동 ±3N | ≈ 30°/s |
| `YAW_KP` | 0.3 | `18/118.6` | 18 N/rad | — |
| `YAW_KD` | 0.15 | `9/118.6` | 9 N/(rad/s) | — |

환산 등가는 pub/sub 테스트 4케이스(전진 12N·차동 9/15N·후진 −6N·포화 118.6N)
로 검증. ⚠️ 5미션 완주 회귀는 생략(사용자 결정) — 작업 3 단계 ① sim 주행이
사실상의 회귀가 되므로, 그때 이상 거동이 보이면 이 정규화 커밋부터 의심할 것.

배 크기/무게를 바꾸면 이 값들(`scale`, `max_linear`, `max_angular`, `YAW_KP`,
`YAW_KD`)을 반드시 헤드리스로 재검증하세요 — 짧게 보면 안정적으로 오판하기
쉬우니 최소 몇 분 이상 5개 미션 완주로 판단(README §3 물리 스텝 경고와 같은 이유).

### 실물 전환 시 재캘리브레이션 지도 (2026-07-17)

이 문서의 "실측"은 전부 **sim 실측**(1.1m 축소 선체)이다. sim 도 이미 최대
추력의 약 10%(12N)로 잘라 쓰고 있지만 이 10% 는 정한 값이 아니라 전복
실측으로 후퇴한 결과다. 실물 전환 때 파라미터는 세 층으로 나뉜다:

- **A. 물리 결합층 — 전부 실물 재실측**: `max_linear`/`max_angular`(추력
  컷 % — 정규화로 의미는 이식되지만 **효과는 배마다 다름**), `YAW_KP/KD`
  (N/rad 물리량 — 실물 관성으로 재튜닝), `ω_max`(0.52 rad/s), v↔cmd 매핑
  (12N→1.48 m/s 한 점짜리), `d_panic`(제동거리 의존), 드라이버의
  `max_thrust`(실물 스러스터 스펙으로 교체). 위 "배 바꾸면 재검증"의 극단 케이스.
- **B. 센서/환경 의존층 — 실물 재검토**: `occ_threshold`(실선 노이즈),
  HSV 임계값(§7).
- **C. 기하·위험 의미론층 — 그대로 이전**: `sigma`(berth 폭 — 경기장
  **실치수** 기준이라 sim=실물), `safe_threshold`, arc 규칙, `w_wp/w_prev`,
  `t_look`(시간 단위 — p1 은 실측 surge 와 곱이라 자동 적응). ×5 폐기·실크기
  경기장 결정의 배당금.

**추력 컷 %**(실물 최대 출력의 몇 %를 명령 상한으로 쓸지)는 avoid 플래너가
아니라 **실물 추력 드라이버(twist2thrust 등가물) bring-up 의 결정**이다 —
실물 스러스터 스펙 확정 후, sim 의 10% 를 출발 참조점으로 전복 안전 여유를
실측해 정할 것. 이를 위해 A층 상수는 코드에 하드코딩하지 않고 전부 ROS
param/설정으로 주입하며, avoid 의 v_cmd(m/s)→명령 변환은 함수 하나로
격리한다(실물에선 교정 곡선만 교체).

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
> `### YYYY-MM-DD HH:MM — [영역] 요약` + 무엇을/왜 몇 문장. 파일 세부는 커밋
> 메시지로. 영역 태그: `[occupancy_grid]` / `[obstacle]` + 세부 `[planner]`
> `[DRI]` `[state machine]` 등을 나란히 붙임.

### 2026-07-19 00:15 — [obstacle] [state machine] 작업 3 obstacle_planner 완성
avoid 미션이 이제 위험도 지도 + 곡선 경로 플래너로 돈다. 조종 정책은 순수 모듈
`avoid_fsm.py` 에 있고 노드는 배선만 한다. 경로를 따라가며 매 tick 안전을 검사해
막히면 수리 → 재계획 → 후진 탈출의 3단 사다리로 대응하고, 속도는 경로 굽이에
맞춰 자동 감속한다(v=ω_max/κ). 실측으로 정한 값: 후진 추력은 전진과 대칭
(-1.48 m/s), /odom twist = body frame, 속도↔명령 멱지수 1.64,
boundary_margin 0.5. 검증은 단위 테스트 41→69개 + sim 부표 밭 관통 8런 —
**접촉 0**(최소 이격 0.48~0.90m), 전복·워치독 0, 후진 탈출 자연 발동 2회.
그 검증 스택도 `scripts/demo/` 로 커밋했다 — 컨테이너 임시 폴더에만 있어서
`docker compose down` 하면 접촉 0 의 근거를 재현할 수 없는 상태였다. 회피용
launch 사본(한 줄 다른 9.6KB fork)은 `simulation.launch.py` 의 spawn 인자로
대체. 문서 AVOIDANCE_PLAN.MD → **AVOIDANCE.MD**(계획서가 아니라 알고리즘·
파라미터 설명서, 표는 §4). 함정: 노드 시간은 벽시계 말고 odom 스탬프 — sim 은
시계가 둘이라 벽시계를 쓰면 RViz 표시가 죽고 타이머 주기도 틀어진다.

### 2026-07-17 12:28 — [control] 명령 의미 정규화: scale 임의상수 → 최대추력 비율 (동작 불변)
사용자 설계 채택 — "teleop/scale 사슬의 끝값을 기준 삼지 말고 스러스터
스펙(노드) 기준으로". `/cmd_vel` 의 linear/angular 를 **최대 추력 대비 비율**
로 재정의: 드라이버(twist2thrust, 실물은 ESC)가 자기 `max_thrust`(param,
118.6N)를 곱해 N 변환 + ±max_thrust 포화. `scale=60` 폐기. 생산자 7곳
(behavior_base 상한·PD, gate/docking 원시 게인, repulsion GAIN, teleop,
rviz marker, 컨테이너 goto_point)을 추력 N 동등으로 환산 — 게인은 물리량
표현(`18.0/MAX_THRUST` = 18 N/rad)으로 적어 검산 가능. 등가는 pub/sub
4케이스 PASS 로 검증, 5미션 회귀는 생략(작업 3 단계 ① 주행이 대신). 이로써
실물 전환 시 컷 %·스펙이 드라이버 계층에 격리된다(§6 지도).

### 2026-07-17 11:58 — [obstacle] [state machine] 작업 3 설계·구현계획 확정 (코드 미착수)
obstacle_planner 노드의 미결 알고리즘과 구현 구조를 사용자와 전부 확정.
**알고리즘**: 상태기계 = FOLLOW/ESCAPE 2상태(재생성 트리거: r1 통과·r1 급락=
기준 장애물 교체·주기 백스톱 / 긴급도 = 위반까지 거리+시간상한 / ESCAPE 탈출 =
최소 이탈+연속 성공). follow_path = 기존 `seek_goal()` PD 조향 + 곡률 feedforward
감속 `v = ω_max/κ`(실측 전속 가용 κ 0.37 = 0.52/1.48 이 근거). ESCAPE 조향 =
후방 선분 DRI argmin 후진(포위 시만 전방위). waypoint 도착 판정은 안 만들고
lookahead 캡만. **구현 구조**: 조종 정책을 순수 라이브러리 `avoid_fsm.py` 로
신설하고 `obstacle_planner.py` 노드는 배선만(rclpy 무의존, YAW 게인은 노드 주입).
분리 근거 = 새 코드 중 테스트가 잡을 수 있는 유일한 버그 부류(전이·타이머)를
Node 메서드로 쓰면 rclpy.init 없이 생성이 안 돼 가짜 시계·합성 장면 루프(작업
1·2 의 설계 오류를 전부 잡은 검증 방식)에서 빠지기 때문. 개발 4단계: ①
follow_path+직선 stub → ② FOLLOW 사다리 → ③ ESCAPE → ④ 노드 통합·부표 밭
관통. 당시 검토·기각안은 이 커밋 시점의 AVOIDANCE.MD(git 히스토리) 참조.

### 2026-07-17 02:09 — [obstacle] [DRI] σ 게인을 확대 → 최대 정규화 감쇄로 (+ a_head 0.7)
"지금 퍼트림(σ 0.6)을 상한으로, 멀고 옆인 것만 덜 퍼뜨리자"는 사용자 지시.
구식(근접·정면을 위로 확대)은 σ 를 0.88m 로 불려 3m 격자 틈을 DRI 층에서
물리적으로 닫았다(pinch 1.12 실측). σ 게인은 진폭 게인과 중복 — 유효 반경 =
σ√(2ln(A/thr)) 라 진하기 경사만으로 반경 경사가 생긴다. 밭 한복판 실측으로
4 arc 전원 통로 관통 확인. 테스트 41개.

### 2026-07-17 01:54 — [obstacle] [planner] 후보 선택을 소프트 DRI 비용 → 선분 하드 필터로
사용자 설계 채택: 직전 CP→후보 선분 max DRI > safe_threshold 면 후보 제거,
생존자 중 방위 비용(w_wp·w_prev)만 argmin. threshold 이하 꼬리 위험이 벌점으로
작동해 CP 가 틈 대신 DRI=0 지대로 도망가던 문제(거품 크기·비용의 안전 이중
계산) 해소. w_dri·hard_max 삭제 — threshold 가 생성·검사 공용 유일 안전선.
gen_margin 0.85 신설 (chord 판정 vs 스플라인의 수 cm 편차 흡수). 테스트 40개.

### 2026-07-16 22:57 — [obstacle] [planner] CP 사다리를 r1 에서 분리 (a22d0dc)
radii 를 항상 ‖p1‖↔r_end 균등 4분할로 — r1(기준 장애물 실거리)은 재생성
트리거 전용으로 강등. 장애물이 지평선 근처면 구 규칙(첫 arc=r1)이 arc 를
[7.9,8.2,8.6,8.9] 로 뭉치게 해 경로가 장애물 밭 입구에서 끝나는 퇴화(sim
실측)가 동기. berth 는 arc 위치가 아니라 arc 후보의 DRI 비용이 정한다.

### 2026-07-16 21:31 — [occupancy_grid] 발행 10Hz + 스캔 벡터화 + 히트 dedup 기본 ON
`publish_rate` 1→10Hz(신규/유령 장애물의 소비자 노출 지연 최대 1s→0.1s),
`_on_scan` 파이썬 Bresenham 루프를 numpy+`cv2.polylines` 로 교체(최악
169→0.74ms/스캔). 같은 스캔의 이웃 빔은 독립 관측이 아니므로 히트를 스캔당
셀당 1회로 캡(`hit_dedup`, 베이즈 독립성 교정) — 수면반사 노이즈 피크≥90 비율
78→14%, 부표 가시성 손실 없음. `occ_threshold` 는 50 유지(75 재검토는 이동
조건 재실측 후 — 정적에선 부표 duty75 90%+).

### 2026-07-16 — [obstacle] [planner] B-spline 로컬 플래너 구현 (`bspline_planner.py`)
AVOIDANCE.MD 작업 2 완료 — generate/check/repair 순수 라이브러리 + 단독
테스트(컨테이너 scipy 1.8 검증). `p1 = p0 + max(surge·t_look, 1m)·ĥ`(배의 운동
구속은 yaw 실측 ~31°/s — 표류/저속 노이즈가 접선에서 빠짐), t_look 1.0s(5.0 은
첨점 재현), repair 는 4CP×±Δθ 8후보 전수(최근접-CP 교착 실측). 기본값은 회피
경기장(3m 격자 부표 밭) RViz 라이브 튜닝으로 실측 확정 — sigma 0.6·w_wp 1.0·
w_prev 0.5·k_bear 1.5·occ_threshold 50. generate 0.27ms / repair 0.70ms.
(같은 커밋에 AVOIDANCE.MD 을 현황 스냅샷 형식으로 전면 재작성 포함)

### 2026-07-16 02:30 — [obstacle] [DRI] 위험도 격자 구현 (`dri.py`)
AVOIDANCE.MD 작업 1 완료 — `/occupancy_grid` 점유 셀마다 가우시안을
splat(가깝고 정면일수록 크고 진하게)하고 셀 결합은 max 로 하는 ROS 무의존
순수 numpy 위험도 격자. sim 검증 중 초안 설계 4건 수정: 비등방 V자 → 등방
가우시안, 합산 → max 결합, 노이즈 필터(`occ_threshold` 미만 셀 제외), 격자
1Hz 가 아니라 배 pose 따라 매 tick(10Hz) 재계산. 장애물 밭 552셀 9.4ms.

### 2026-07-15 21:52 — [obstacle] 회피 로컬 플래너 작업계획 확정 (AVOIDANCE.MD)
`obstacle_planner` 를 단순 반발 조향 → DRI + B-spline 로컬 플래너로 교체하는
설계를 [AVOIDANCE.MD](AVOIDANCE.MD) 에 정리. 아직 구현 전(계획 문서만).
DRI 비등방 가우시안 수식, 제어점 6개 규칙, 생성/검사/수리 분리, 재생성 트리거
3종, ESCAPE(안전 경로 불가 시 후진), 모듈 인터페이스·작업 5분할 포함.

### 2026-07-15 16:55 — [occupancy_grid] dict+prune → rolling grid 리팩터
저장을 월드좌표 `dict`+`prune` → global 셀은 그대로 두고 배 중심 20×20m 창으로
잘라 담는 고정 numpy 배열(정수 `np.roll`)로 교체해 `prune` 제거. 값·정확도는 동일, publish 는 벡터화로 경량화.
