# pytest 시험 계획

본 문서는 KABOAT 자율운항 스택의 pytest 시험 구조와 기준을 정한다.
수조 시험과 Gazebo 시험은 현재 단계에서 제외하고, ROS 2 실행 없이 돌아가는 pytest만 다룬다.

## 1. 시험 구조 — 3단

시험은 기능 시험(L1), 미션 시험(L2), 전체 흐름 시험(L3)의 3단으로 나눈다.
각 단은 검증 대상이 다르기 때문에 서로를 대신하지 않는다.
L1은 부품 하나의 출력을 확인하고, L2는 부품을 이어 붙였을 때의 결과를 확인한다.
L3는 미션 관리와 명령 선택까지 포함한 전체 순서를 확인한다.

| 단 | 대상 | 단위 | 위치 | 실행 환경 |
|---|---|---|---|---|
| L1 기능 | 기능 하나의 출력 | 기능 | `src/<package>/test/` | ROS 없는 기능은 아무 Python, 노드는 Docker |
| L2 미션 | 한 미션의 기능 묶음 | 미션 5개 | `tests/missions/` | Docker |
| L3 전체 흐름 | `mission_manager` → behavior → `cmd_mux` | 1개 | `tests/system/` | Docker |

L1은 colcon 규칙에 맞게 각 패키지의 `test/`에 둔다.
L2와 L3는 여러 패키지를 함께 쓰기 때문에 저장소 루트의 `tests/`에 둔다.
노드 시험은 spin하지 않고 콜백과 `compute_cmd()`를 직접 호출한다 (`test_gate_follower.py`와 같은 방식).

## 2. 비중 원칙

비중은 미션이 아니라 기능 단위로 같게 둔다.
모든 기능은 같은 체크리스트를 통과해야 한다.
따라서 기능이 많은 미션은 시험 수가 자연스럽게 많아진다.
이때 시험 수가 많은 것은 기능 수가 많기 때문이며, 특정 미션을 따로 강조한 결과가 아니다.

기능 하나는 보고서에서 한 문단으로 설명하는 설계 요소 하나로 정의한다.
코드로는 대체로 순수 모듈 하나 또는 노드 하나의 역할에 해당한다.

## 3. L1 기능 시험 기준

### 3.1 체크리스트

모든 기능은 아래 항목을 각각 시험 1개 이상으로 확인한다.

- **A 정상 동작:** 대표 입력에서 기대 출력이 나오는지 확인한다.
- **B 경계·파라미터:** 경계값, 파라미터 변화, 잘못된 입력 처리를 확인한다.
- **C 실패 시 안전:** 입력 끊김, 판단 불가, 시간 초과에서 정지 명령 또는 실패 반환이 나오는지 확인한다.
- **D 노드 입출력:** 노드인 경우에만 해당한다. 메시지 입력에 대해 발행 명령과 완료 신호가 맞는지 확인한다.

### 3.2 횟수와 복잡도 기준

| 항목 | 기준 |
|---|---|
| 기능당 시험 수 | 최소는 체크리스트 항목 수(3~4개), 권장 4~10개 |
| 상한 | 한 기능이 15개를 넘으면 `pytest.mark.parametrize`로 묶거나 기능을 나눌 수 있는지 검토한다. 다만 기존 시험을 줄이지는 않는다 |
| 시험 1개 실행 시간 | 0.1 s 이하 (성능 시험 제외) |
| 입력 데이터 | 손으로 계산 가능한 작은 장면을 쓴다. 난수를 쓰면 seed를 고정한다 |
| 그림 | L1 기능마다 대표 출력 그림 1장 (DRI 지도, B-spline 경로 등) |

## 4. L2 미션 시험 기준

### 4.1 미션별 묶음

L2는 가짜 센서 입력부터 추진 명령까지 실제 코드를 이어 붙이고, 공통 선체 모델(`tests/sim/boat_model.py`)로 폐루프를 돌린다.

| 미션 | 이어 붙이는 기능 |
|---|---|
| Gate | scan·격자 → `gate_corridor` → `gate_follower` → 명령 |
| Station | odom → `station_keeper` → waypoint 제어 |
| Dock | 도킹 입구 점 → `dock_target` → `docking_fsm` → `docking_ctrl` |
| Search | odom → `search_circler` → LOS |
| Avoid | scan → 점유 격자 → DRI → B-spline → 회피 FSM → `obstacle_planner` |

### 4.2 미션별 공통 항목

모든 미션은 같은 4개 항목을 확인한다.

- **완료:** 시작부터 `/mission/complete` 발행까지 도달하는지 확인한다.
- **성능:** 제한 시간 안에 끝나는지, 목표 오차가 기준 이내인지 확인한다.
- **안전:** 장애물·벽과의 최소 거리가 기준 이상이고 수역을 벗어나지 않는지 확인한다.
- **실패 주입:** 도중에 센서를 끊으면 정지 명령이 나오는지 확인한다.

### 4.3 시나리오, 반복 횟수, 복잡도

미션마다 같은 3개 시나리오를 둔다.

| 시나리오 | 조건 | 반복 | 확인 항목 |
|---|---|---|---|
| S1 기본 | 잡음·외란 없음 | 1회 | 완료, 성능, 안전 |
| S2 외란 | odom 잡음, 일정 조류, 초기 자세 변화 | seed 5개 | 완료, 성능, 안전 → 성공률 |
| S3 실패 주입 | 진행 중 센서 입력 중단 | 1회 | 실패 주입 |

반복 횟수를 seed 5개로 정한 이유는 실행 시간 때문이다.
미션당 7회 × 5개 미션 = 35회이며, 1회당 시뮬레이션 시간 120 s(제어 1,200 step) 이하로 제한하면 L2 전체를 30 s 안에 끝낼 수 있다.
성공률을 더 정밀하게 보려면 `KABOAT_SEEDS` 환경 변수로 반복 횟수를 늘린다.

시뮬레이션 설정은 Table 1에 정리했다.

**Table 1. L2 시뮬레이션 설정**

| 항목 | 값 | 근거 |
|---|---|---|
| 제어 주기 | 0.1 s | behavior 10 Hz timer |
| 물리 적분 주기 | 0.02 s | 제어 1 step당 5회 |
| 선체 모델 | 3자유도 (surge, sway, yaw) | `kaboat_hull.xacro`의 질량·관성·감쇠 계수 |
| 추력 변환 | 좌 = (linear − angular) × 118.6 N, 우 = (linear + angular) × 118.6 N | `twist2thrust.py` |
| 시험 1회 상한 | 시뮬레이션 120 s | 수조 크기 기준 |

## 5. L3 전체 흐름 시험 기준

L3는 `mission_manager`, 5개 behavior, `cmd_mux`를 함께 띄우고 미션 순서를 확인한다.
각 behavior의 내부 판단은 L1·L2에서 이미 확인했기 때문에, L3에서는 완료 신호를 직접 넣어 전환만 확인한다.

- **순서:** gate → station → dock → search → avoid → done 순서로 전환되는지 확인한다.
- **전환 조건:** 완료 신호와 목표 4 m 이내 조건을 모두 만족할 때만 전환되는지 확인한다.
- **명령 선택:** `cmd_mux`가 현재 state의 behavior 명령만 통과시키는지 확인한다.
- **정지:** 명령이 300 ms 이상 끊기면 `cmd_mux`가 정지 명령을 내는지 확인한다.

## 6. 분류와 결과물

시험 파일의 분류(단, 기능, 묶음)는 `tests/testmap.py` 한 곳에서 관리한다.
루트 `conftest.py`가 이 표를 읽어 marker(`level_unit`, `group_avoid` 등)와 junit 속성을 붙인다.
분류표에 없는 시험 파일은 `unclassified`로 표시되어, 새 시험을 추가할 때 분류를 빠뜨리지 않게 한다.

| 결과물 | 생성 방법 |
|---|---|
| 기능 × 체크리스트 표 | junit XML → 집계 스크립트 |
| 미션 × 항목 표, S2 성공률 | junit XML → 집계 스크립트 |
| 그림 | `KABOAT_TEST_FIGURES=<dir>`를 지정했을 때만 저장한다. 평소 실행은 그림을 만들지 않는다 |

## 7. 실행

```bash
scripts/run_tests.sh            # L1~L3 전체, Docker
scripts/run_tests.sh -m unit    # L1만
```

ROS 없는 L1은 Windows Python에서도 `python -m pytest src/<package>/test`로 돌릴 수 있다.

## 8. 진행 순서

1. 공통 선체 모델과 가짜 센서 (`tests/sim/`), 분류표와 `conftest.py`
2. L1 빈 곳: 미션 관리(`mission_manager`, `mission_approach`, `cmd_mux`), 반발 조향(`apply_repulsion`), behavior 노드(`station_keeper`, `docking_ctrl`, `search_circler`, `obstacle_planner`), 인지(`buoy_detector` 색 검출 단계, `occupancy_grid`, `dock_segmentation`)
3. L2 미션 시험 5개
4. L3 전체 흐름 시험
5. 집계 스크립트와 그림

## 9. 범위에서 제외하는 것

수조 시험과 Gazebo 시험은 현재 단계에서 제외한다.
따라서 수조 주행 시험용 노드(`straight_line_test`, `bspline_track_test`, `circle_drive_test`, `los_path_test`, `waypoint_sequence_test`, `station_keeping_test`, `real_obstacle_avoid_test`, `manual_dri_monitor`, `tank_mission_guard`)와 그 보조 모듈(`test_coordinates`, `trajectory_visualization`)의 시험은 분류표에서 `excluded`로 둔다.
이 시험들은 회귀 확인을 위해 계속 실행하지만, 체크리스트 표와 보고서 집계에는 넣지 않는다.

YOLO 검출 노드(`dock_mark_detector`)도 제외한다.
이 노드는 실제 영상과 학습된 모델이 필요하기 때문이다.
`buoy_detector`는 학습 없는 HSV 색 검출이므로, 색 원을 그린 합성 영상과 합성 depth 영상으로 시험한다.
