# 장애물 회피 실험 기록

실물 수조에서 진행한 장애물 회피(`test_avoid`, `real_obstacle_avoid_test`) 실험의 설정값과 결과를 남깁니다.
알고리즘 설명은 [AVOIDANCE.MD](AVOIDANCE.MD), 실행 명령 전체는 [TANK_TESTS_GUIDE.md](TANK_TESTS_GUIDE.md) 9·10절을 봅니다.

## 현재 경로 추종 출력 (2026-11-06 기준)

| 항목 | 값 | 바꾸는 곳 |
|---|---|---|
| **직진(전진) 계수** | **0.23** (직선 구간 실제 전진 명령 0.210) | `FsmParams.cmd_full` ([avoid_fsm.py](src/kaboat_behaviors/kaboat_behaviors/avoid_fsm.py)), 수조 시험 상한 `max_linear_cmd` |
| **회전 계수** | **0.36** (회전 명령 상한) | 수조 시험 `max_angular_cmd`, 대회 미션 `avoid.max_angular_ratio` ([obstacle_planner.py](src/kaboat_behaviors/kaboat_behaviors/obstacle_planner.py)) |

수조 시험 값은 [real_obstacle_avoid_test.yaml](src/kaboat_hardware/config/real_obstacle_avoid_test.yaml), 수조 대회 미션 값은 [tank_avoid_mission.yaml](src/kaboat_hardware/config/tank_avoid_mission.yaml)에도 같은 값으로 적혀 있습니다. 바꿀 때는 코드 기본값과 두 YAML을 함께 바꿉니다.

같은 상황에서 나오는 명령(직선 경로, 선수 오차만 바꿈):

| 선수 오차 | 전진 / 회전 | 좌 / 우 추진기 |
|---|---|---|
| 0° | 0.210 / 0 | +0.21 / +0.21 |
| 10° | 0.210 / 0.26 | +0.47 / −0.05 |
| 20° 이상 | 0.210 / 0.36 (상한) | +0.57 / −0.15 |

## 저장해 둔 설정

실험하면서 다시 돌아갈 수 있게 남겨 둔 조합입니다. 되돌릴 때는 위 표의 "바꾸는 곳"과 두 YAML의 직진·회전 값을 이 표대로 바꿉니다(헤딩 게인은 `BSPLINE_TRACK_YAW_KP/KD`, [los_controller.py](src/kaboat_behaviors/kaboat_behaviors/los_controller.py)).

| 이름 | 직진 | 회전 | 헤딩 P / D | 커밋 | 비고 |
|---|---|---|---|---|---|
| 설정 1 | 0.20 | 0.38 | 1.5 / 0.15 | `0da9004` | 11-06 저장 |
| 설정 2 | 0.23 | 0.36 | 1.5 / 0.15 | `d895965` | 11-06 저장. D 0.18을 시험했다가 0.15로 되돌린 뒤의 값 |

## 나머지 설정값

**경로 추종** (`test_los`에서 검증한 값과 같음)

| 항목 | 값 |
|---|---|
| 헤딩 P / D 게인 | 1.5 / 0.15 (LOS·게이트와 공용 상수 `BSPLINE_TRACK_YAW_KD`) |
| 전방 주시 거리 `lookahead` | 1.0 m |
| 목표 근접 감속 반경 `slow_radius` | 1.5 m |
| 곡률 감속: `kappa_window` / `v_min` / `v_max` / `omega_max` | 2.0 m / 0.3 / 1.4 m/s / 0.52 rad/s |
| 속도→출력 변환 `v_full` / `cmd_exp` | 1.48 m/s / 1.64 |

**경로 계획** ([bspline_planner.py](src/kaboat_behaviors/kaboat_behaviors/bspline_planner.py))

| 항목 | 값 |
|---|---|
| 안전 기준 `safe_threshold` / 후보 마진 `gen_margin` | 1.2 / 0.93 (후보 필터 문턱 1.116, RViz 빨간 X 기준과 같음) |
| 후보 부채꼴 / 후보 수 | 목표 방위 ±45° / 원마다 31개 |
| 최소 계획 거리 `min_horizon` | 2.0 m |
| 후보 동점 처리 | 목표 방위에 가까운 후보 (좌우 대칭) |

**위험도 지도(DRI)**: `sigma` 0.4, 격자 15 m × 15 m (0.2 m 칸), 선체 라이다 위치 선수 0.5 m.

**수조 조건** ([real_obstacle_avoid_test.yaml](src/kaboat_hardware/config/real_obstacle_avoid_test.yaml))

| 항목 | 값 |
|---|---|
| 목표 | (1.5, 2.5) 고정, 시작 위치 무관 (벽에서 0.5 m 이상) |
| 출발 조건 | 선수가 목표 방향 45° 이내, 배→목표 직선 ±1 m 안에 장애물 |
| 벽 처리 | 경계 0.5 m 띠를 벽으로 보고 DRI 0.5로 낮춤 |
| 후진 탈출(ESCAPE) | 경로를 5초 넘게 못 만들거나 위험이 안 풀리면 후진(출력 0.1, `escape_thrust`). 최소 3초 뒤 경로가 5번 연속 나오면 복귀. `allow_escape: false`면 대신 정지 |
| 정지 조건 | 벽 0.5 m 접근(후진 중 포함), 경로 0.7 m 이탈, 센서 끊김, 목표 0.5 m 도착 |

## 실험 방법 요약

```bash
# 젯슨 (터미널마다 하나씩)
run_zenoh
run_boat_lidar
run_motor
test_avoid        # 조건이 맞으면 바로 출발
```
```zsh
# 노트북: 배 위치 추적 + 회피 화면
kaboat_lidar → tracker enable_rviz:=false
set_kaboat
rviz2 -d ~/Projects/KABOAT/kaboat_ws/src/kaboat_hardware/rviz/real_obstacle_avoid_test.rviz
```

- 노트북 키보드 조종 창은 닫고(`R`), ESP는 컴퓨터 조종 모드(`P`)여야 합니다.
- 모터 없이 경로만 볼 때는 `test_avoid` 대신 `ros2 launch kaboat_hardware real_obstacle_avoid_test.launch.py drive_enabled:=false wait_for_start:=true`.

## 튜닝 이력

| 날짜 | 바꾼 것 | 이유 / 관찰 |
|---|---|---|
| 11-04 | 후보 마진 `gen_margin` 0.85 → 0.93 | 2 m 간격 장애물 사이에 빨간 X가 너무 많아 후보가 적음 |
| 11-04 | `safe_threshold` 1.0 → 1.2, DRI `sigma` 0.5 → 0.4 | 장애물 사이 통과 후보를 늘림 (sim 값 → 실물 값) |
| 11-04 | 선체 라이다 위치 선수 0.5 m 반영 | 벽이 실제보다 0.5 m 밀려 보여 벽 필터가 안 먹음 |
| 11-04 | `min_horizon` 3.0 → 2.0, 벽 띠 0.3 → 0.5 m, 목표 (1.0, 2.5) → (1.5, 2.5) | 벽 앞에서 마지막 제어점이 벽 위에 놓여 NO PLAN |
| 11-04 | 경로 추종을 `test_los` 값으로 통일 (게인 0.152/0.076 → 1.5/0.15, 출력 0.101 → 0.3) | 예전 sim 값이라 전진 1.7%로 거의 안 움직이고 회전도 약함 |
| 11-04 | 시작 위치 무관 출발, 출발 막힘 이유 표시 | 고정 시작점 (9.0, 2.5)에 놓지 않으면 출발 안 함 |
| 11-05 | 후보 동점 시 목표 쪽 선택 | 오른쪽으로 피한 경로만 목표로 안 돌아와 목표 옆 2~3 m로 빠짐 |
| 11-06 | 직진 0.3 → 0.15, 회전 0.3 → 0.4 | 경로 생성은 정상인데 직진 0.3(실제 0.27)에서 꺾이는 경로를 관성으로 못 따라감 |
| 11-06 | 직진 0.15 → 0.2, 회전 0.4 → 0.38 | 실물 튜닝 |
| 11-06 | 직진 0.2 → 0.23 (회전 0.38 유지) | 실물 튜닝 |
| 11-06 | 회전 0.38 → 0.36, D 게인 0.15 → 0.18 (직진 0.23) | 선수가 경로 방향을 지나치는 것을 줄이려고 감쇠를 키움. D 게인은 `test_los`·게이트에도 적용 |
| 11-06 | D 게인 0.18 → 0.15 되돌림 (직진 0.23, 회전 0.36) | |
| 11-06 | D 게인 0.15 → 0.12, 수조 테스트 ESCAPE 켬(후진 출력 0.1) | ESCAPE 가 없어 경로를 못 만들면 정지만 했음. 예전 후진 출력은 0.04라 움직이지 않을 값이어서 0.1로 직접 지정 |
| 11-06 | **D 게인 0.12 → 0.15 되돌림** (현재: 직진 0.23, 회전 0.36, D 0.15, ESCAPE 켬·후진 0.1) | |

## 실험 결과

결과는 아래 표에 한 줄씩 추가합니다. 장애물 배치는 수조 좌표(좌하단 원점, x 0~10, y 0~5 m)로 적습니다.

| 날짜 | 직진 / 회전 | 장애물 배치 | 시작 위치·방향 | 결과 (도착 / 정지 이유) | 관찰 |
|---|---|---|---|---|---|
| 11-06 | 0.3 / 0.3 | | | 추종 실패 | 경로 생성은 문제없음. 추력이 세서 꺾이는 경로를 따라가지 못함 |
| 11-06 | 0.2 / 0.38 | | | | |
| 11-06 | 0.23 / 0.38 | | | | |
| 11-06 | 0.23 / 0.36, D 0.18 | | | | |
| 11-06 | 0.23 / 0.36, D 0.15 | | | | |
| 11-06 | 0.23 / 0.36, D 0.12, ESCAPE 켬 | | | | |
| 11-06 | 0.23 / 0.36, D 0.15, ESCAPE 켬 | | | | |
