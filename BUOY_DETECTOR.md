# buoy_detector — 부표 위치 + 색(빨강/초록/검정) (feature/buoy-detector)

탐색(`search_circler`)·게이트에서 쓸 부표맵 `/detections/buoys`를 만드는 노드를 다시 짰다.
코드: `src/kaboat_perception/kaboat_perception/buoy_detector.py` (ROS 배선),
`buoy_localize.py` (계산, ROS 무의존), 테스트 `test/test_buoy_localize.py`.

## ⚠️ merge 전에 할 일 — 장착 파라미터를 competition 값으로 맞추기

이 브랜치의 카메라·라이다 장착값은 **브랜치를 딴 시점(2026-11-12 오전) 값 그대로 두었다.**
merge 할 때 competition 쪽 노드의 값이 정답이다 — 그걸 기준으로 `BuoyGeometry` 기본값
(`buoy_localize.py`)을 고친다. TF 는 읽지 않는다 (대회 스택 `autonomy.launch.py` 는
base_link→카메라 TF 를 안 띄우고, `cam_height` 는 흘수선 기준이라 TF 로 안 나온다).

| buoy_detector 파라미터 (`BuoyGeometry`) | 이 브랜치 값 | 기준 (competition) | 2026-11-12 오후 competition 값 |
|---|---|---|---|
| `cam_offset_x` | -0.33 | `camera_obstacles.py` `CameraObstacleParams.offset_x` (occupancy_grid 카메라 층 `cam_offset_x`) | -0.33 |
| `cam_offset_y` | 0.0 | 〃 `offset_y` | 0.0 |
| `cam_height` | **0.85** | 〃 `height` | **0.775** (다시 수평으로 단 뒤 실측) |
| `cam_mount_pitch_deg` | **10.0** | 〃 `mount_pitch_deg` | **0.0** (10° 숙임 → 다시 수평) |
| `lidar_offset_x` | 0.4 | `occupancy_grid.py` `lidar_offset_x` (= `gate_follower`·`station_keeper`·`odom_tf_broadcaster`·launch 의 base_link→라이다 TF) | 0.4 |
| `lidar_offset_y` | 0.0 | 〃 `lidar_offset_y` | 0.0 |

굵게 표시한 두 값은 이미 다르다. merge 때 위 표의 "기준" 위치에서 **그때의** 값을 다시
읽어 맞출 것 (오후 이후에도 바뀌었을 수 있음). 카메라 각도를 바꾸면 도킹 TF
(`dock_yolo.launch.py` `--pitch`)도 같이 바뀌어 있어야 한다.

확인 순서:
1. `git show origin/competition:src/kaboat_perception/kaboat_perception/camera_obstacles.py` 의
   `CameraObstacleParams` 와 `occupancy_grid.py` 의 `lidar_offset_x/y` 를 본다.
2. `buoy_localize.py` `BuoyGeometry` 의 `cam_*`·`lidar_offset_*` 기본값을 그 값으로 고친다.
3. 테스트(`test_buoy_localize.py`)는 `G = BuoyGeometry()` 기본값으로 장면을 렌더하므로
   값만 바꿔도 그대로 통과해야 한다 — 실패하면 기하 코드 문제다.
4. 현장 launch 가 `cam_height`·`cam_mount_pitch_deg` 를 인자로 넘기면 buoy_detector 에도
   같은 이름으로 넘긴다 (파라미터 이름이 occupancy_grid 의 `cam_*` 과 같다).

## 동작

```
① classify_pixels  물체 픽셀 = 검정, 또는 채도 있고 물·하늘색(파랑·청록) 아님 → 픽셀 색 라벨
② depth_buoys      6 m 이내: 물체 픽셀 → 정렬 뎁스로 3D → 수면 위만(반사상 제거)
                   → 수평 격자 덩어리 → 색 다수결 → 근처 라이다 군집과 위치 융합
③ lidar_buoys      ② 로 안 잡힌 라이다 군집(6 m 너머 등) → 영상에 투영한 창의 색 다수결
④ odom 으로 전역 투영 → ⑤ BuoyTracker (N-of-M 확정·prune)
```

- **색 (2026-11 경기규정 기준):** 빨강·초록·검정 세 가지만 받는다.
  규정 5.2(6) — 탐색은 사전에 주어진 색의 부표를 돌며 빨강·초록은 우현선회, 노랑은 좌현선회
  인데 이번에 노랑이 **검정**으로 바뀌었다 → 이진 분류는 쓰지 않는다.
  주황은 빨강과 구분이 어려워 따로 두지 않고 빨강에 넣는다. 조명·햇빛에 색이 틀어져도
  놓치지 않게 범위를 전체적으로 넉넉히 잡았다.
  픽셀마다 (OpenCV H 0~180): V ≤ 70 → 검정 / 채도 S ≥ 60 이면 H ≤ 25 또는 ≥ 155 빨강
  (주황 포함), 30~89 초록 (연두 포함), 그 밖(노랑 26~29, 청록·파랑 90~154)은 기타.
  채도 낮은 밝은 픽셀(흰·회색)은 물체 픽셀이 아니다.
  **물·하늘은 색으로 거르지 않는다.** 뎁스 경로는 수면 높이(z)와 6 m clamp 로 물·하늘이
  빠지고, 라이다 경로(뎁스 없음)는 빨강·초록·검정 픽셀끼리만 투표해서 창 뒤의 파란 물(기타)이
  빠진다. 그래서 초록 윗경계(89)만은 넓히지 않았다 — 더 넓히면 라이다 경로 창의 청록빛 물이
  초록 표가 된다.
  물체 색 = 다수결. 1위가 기타거나 40 %(`min_color_frac`) 미만이면 버린다.
  경계: `v_black_max`, `s_min`, `red_h_max/min`, `green_h_min/max`.
- **뎁스 (6 m 이내만):** `camera_obstacles`(점유지도 카메라 층)와 같은 방식 — 픽셀마다 3D,
  수면 위 높이 `z_min`(0.02 m) 미만(물 자체·물에 비친 반사상)·`z_max`(1.5 m) 초과·
  `depth_max`(6 m) 너머(하늘·먼 물가)를 버린다. D455 뎁스 오차는 거리² 로 커져서(4 m ≈ 8 cm, 6 m ≈ 18 cm) 6 m 너머는 안 쓴다.
- **라이다:** 색을 모르니 혼자 부표를 만들지는 않는다.
  6 m 이내는 뎁스 물체 `assoc_radius`(0.6 m) 안의 군집과 분산 가중평균 (라이다 1σ 5 cm,
  뎁스 1σ = 0.005·R²) — 4 m 만 돼도 위치는 거의 라이다로 정해진다.
  6 m 너머는 라이다 군집 위치 그대로, 색은 군집을 영상에 투영한 창(수면 ~ `far_z_max` 0.5 m)
  에서 본다. 라이다는 물을 못 보니 물결 오검출이 없고, 창에 물체 픽셀이 없으면(흰·회색 기둥)
  버린다. 창에서 뎁스가 군집보다 0.5 m(`occlusion_margin`) 넘게 가까운 픽셀은 앞을 가린 다른
  물체라 투표에서 뺀다 — 앞 부표 뒤의 먼 부표가 앞 부표 색으로 잡히지 않게.
  스캔은 `/scan/level`(배가 수평일 때만), `lidar_max_age_sec`(0.3 s)보다 오래되면 안 쓴다.
- **중심·반지름:** 반지름은 고정값이 아니다 (대회 부표 크기를 모르고, 넘어질 때도 있음).
  매 관측 덩어리의 시선 수직 폭 / 2 로 추정하고 (라이다는 빔 한 칸 간격 보정),
  앞면 점 평균에서 시선 방향으로 (π/4)·r̂ 뒤를 중심으로 잡는다. 넘어진 부표의 넓은
  실루엣 대비 `max_radius`(0.5 m) 상한만 둔다.

## 남은 일

- 장착 파라미터 맞추기 (위 표) — merge 때.
- HSV 경계 현장 캘리브 (셔터·AWB 고정 후). 지금 값은 sim 원색 기준.
- **`search_circler` 선회 방향이 규정과 다르다.** 지금 코드는 초록 → 반시계(좌), 빨강 → 시계(우).
  규정은 빨강·초록 → 우현(시계), 검정 → 좌현(반시계)이고, "사전에 주어진 색"의 부표만 돌아야
  한다 (대회 당일 공지). behavior 쪽에서 목표 색 파라미터 + 규정대로 방향을 고쳐야 한다.
- 검정은 어두움(V)만으로 판정한다. 그늘진 물·검은 기둥·타이어도 검정이 될 수 있다 — 뎁스 경로는
  수면 높이 필터가, 라이다 경로는 군집 크기 필터가 대부분 거르지만 현장 확인 필요.
  D455 는 무광 검정에서 뎁스가 비기 쉬워, 검정 부표는 라이다 경로로 잡히는 경우가 많을 것.
- `gate_follower` 는 아직 `/detections/buoys` 를 안 쓴다 (라이다 점유지도로 게이트를 찾음).
- `kaboat_msgs/Buoy.msg` 주석이 바뀌어 `kaboat_msgs` 재빌드 필요.
- sim/보트 실주행 미검증. 합성 영상 테스트(perception 95개) + 노드 스모크만 통과.
- 경기장 주황 부표(장애물·위치유지, 그림 1)는 **빨강으로 나온다.** 탐색 목표가 빨강이면
  behavior 가 주황 부표를 돌 수 있다 — 탐색 구역·위치로 거르거나, 현장에서 빨강/주황이
  갈리면 `red_h_max` 를 낮춘다.
- 넉넉한 범위의 대가: `v_black_max` 70 이라 어두운 빨강·초록(그늘)이 검정이 될 수 있고,
  `s_min` 60 이라 바랜 물체도 물체 픽셀이 된다. 현장 영상으로 확인.

테스트 (컨테이너, 빌드 없이 소스에서):

```bash
docker exec kaboat_sim_container bash -lc 'source /opt/ros/humble/setup.bash; cd /workspace/kaboat_ws/src/kaboat_perception && PYTHONPATH=$PWD:$PYTHONPATH python3 -m pytest -q test/'
```
