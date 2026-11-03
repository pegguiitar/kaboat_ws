# pytest 결과 집계

## 단별 결과

| 단 | 시험 수 | 통과 | 실패 |
|---|---|---|---|
| L1 기능 | 399 | 399 | 0 |
| L2 미션 | 35 | 24 | 11 |
| L3 전체 흐름 | 6 | 6 | 0 |
| 시험 도구 검증 | 13 | 13 | 0 |
| 범위 밖 (수조 시험 도구) | 98 | 98 | 0 |

## L1 기능 × 체크리스트

| 묶음 | 기능 | A 정상 | B 경계 | C 실패 시 안전 | D 노드 입출력 | 실패 |
|---|---|---|---|---|---|---|
| 미션 관리 | 명령 선택·워치독 (cmd_mux) | 2 | 2 | 4 | 1 | 0 |
| 미션 관리 | 미션 시작 자세 접근 (mission_approach) | 2 | 3 | 3 | 1 | 0 |
| 미션 관리 | 미션 전환 (mission_manager) | 2 | 6 | 3 | 1 | 0 |
| behavior 공통 | LOS 경로 추종 | 3 | 4 | 1 | – | 0 |
| behavior 공통 | waypoint 제어 | 4 | 5 | 2 | – | 0 |
| behavior 공통 | 반발 조향 (apply_repulsion) | 3 | 5 | 2 | – | 0 |
| behavior 공통 | 부표맵 상대 기하 (behavior_base) | 6 | 3 | 1 | – | 0 |
| behavior 공통 | 속도 벡터 제어 | 3 | 2 | 2 | – | 0 |
| Gate (항로추종) | 게이트 노드 (gate_follower) | 1 | 5 | 2 | 4 | 0 |
| Gate (항로추종) | 게이트 통로 선택 | 3 | 1 | 3 | – | 0 |
| Station (위치유지) | 위치유지 노드 (station_keeper) | 3 | 3 | 5 | 1 | 0 |
| Dock (도킹) | 도킹 FSM | 4 | 4 | 5 | – | 0 |
| Dock (도킹) | 도킹 노드 (docking_ctrl) | 3 | 2 | 5 | 1 | 0 |
| Dock (도킹) | 도킹 진입 경로 계산 | 2 | 2 | 2 | – | 0 |
| Search (탐색) | 탐색 선회 노드 (search_circler) | 3 | 4 | 1 | 1 | 0 |
| Avoid (장애물 회피) | B-spline 경로 생성 | 11 | 11 | 4 | – | 0 |
| Avoid (장애물 회피) | DRI 위험도 | 9 | 8 | 2 | – | 0 |
| Avoid (장애물 회피) | 회피 FSM | 14 | 11 | 4 | – | 0 |
| Avoid (장애물 회피) | 회피 노드 (obstacle_planner) | 3 | 2 | 4 | 1 | 0 |
| 인지 | 깊이 영상 변환 | 2 | 4 | 3 | – | 0 |
| 인지 | 도킹 입구 추정 | 2 | 5 | 1 | – | 0 |
| 인지 | 도킹 표식 분할 뒤처리 | 1 | 2 | 2 | – | 0 |
| 인지 | 부표 맵 (buoy_tracker) | 10 | 8 | 4 | – | 0 |
| 인지 | 부표 색 검출 (buoy_detector) | 4 | 5 | 5 | 1 | 0 |
| 인지 | 점유 격자 (occupancy_grid) | 3 | 5 | 3 | 1 | 0 |
| 위치 추정 | IMU 추측 항법 | 3 | 1 | 1 | – | 0 |
| 위치 추정 | yaw bias EKF | 2 | 2 | 1 | – | 0 |
| 위치 추정 | ┛ 판 자세 추정 | 2 | 5 | 1 | – | 0 |
| 위치 추정 | 마커 자세 추정 | 4 | 1 | 2 | – | 0 |
| 위치 추정 | 선체 라이다 스캔 필터 | 2 | 3 | 1 | – | 0 |
| 위치 추정 | 실내 라이다 odom | 5 | 5 | 7 | 1 | 0 |
| 위치 추정 | 외벽 라이다 선체 추적 | 2 | 2 | 1 | – | 0 |
| 위치 추정 | 자세 미분 속도 | 5 | 4 | 2 | – | 0 |
| 위치 추정 | 판 자세 추정 | 1 | 3 | 1 | – | 0 |
| 제어 | 센서 상태 판정 | 1 | 2 | 1 | – | 0 |
| 제어 | 추진기 드라이버·원격 조종 | 3 | 6 | 1 | 1 | 0 |
| 제어 | 추진기 출력 변환 | 6 | 6 | 1 | – | 0 |

## L2 미션 시험

| 미션 | S1 | S2-0 | S2-1 | S2-2 | S2-3 | S2-4 | S3 | 통과 |
|---|---|---|---|---|---|---|---|---|
| Gate | 통과 | 통과 | 통과 | 실패 | 실패 | 실패 | 통과 | 4/7 |
| Station | 통과 | 통과 | 통과 | 통과 | 통과 | 통과 | 통과 | 7/7 |
| Dock | 실패 | 실패 | 실패 | 실패 | 실패 | 실패 | 통과 | 1/7 |
| Search | 통과 | 통과 | 통과 | 통과 | 통과 | 통과 | 통과 | 7/7 |
| Avoid | 통과 | 통과 | 통과 | 통과 | 실패 | 통과 | 실패 | 5/7 |

## L3 전체 흐름

| 시험 | 결과 |
|---|---|
| test_full_sequence_with_entry_approach_reaches_done | 통과 |
| test_only_active_behavior_commands_and_mux_passes_only_it | 통과 |
| test_completion_far_from_goal_waits_until_radius | 통과 |
| test_silent_behavior_triggers_mux_stop | 통과 |
| test_inactive_behaviors_cannot_complete_the_mission | 통과 |
| test_detector_enabled_only_during_dock_and_stop_after_done | 통과 |

## 실패 목록 (11개)

- `test_mission_avoid::test_avoid_disturbed[3]` — AssertionError: 부표와 0.30m 까지 접근
- `test_mission_avoid::test_avoid_odom_loss_stops` — AssertionError: odom 끊김 0.6초 뒤에도 추력 명령
- `test_mission_dock::test_dock_nominal` — AssertionError: 완료 신호 없음 — 상태 ['APPROACH', 'ACQUIRE', 'PRE_ENTRY', 'ABORT']
- `test_mission_dock::test_dock_disturbed[0]` — AssertionError: 완료 신호 없음 — 상태 ['APPROACH', 'ACQUIRE', 'PRE_ENTRY', 'ABORT']
- `test_mission_dock::test_dock_disturbed[1]` — AssertionError: 완료 신호 없음 — 상태 ['APPROACH', 'ACQUIRE', 'PRE_ENTRY', 'ABORT']
- `test_mission_dock::test_dock_disturbed[2]` — AssertionError: 완료 신호 없음 — 상태 ['APPROACH', 'ACQUIRE', 'PRE_ENTRY', 'ABORT']
- `test_mission_dock::test_dock_disturbed[3]` — AssertionError: 완료 신호 없음 — 상태 ['APPROACH', 'ACQUIRE', 'PRE_ENTRY', 'ABORT']
- `test_mission_dock::test_dock_disturbed[4]` — AssertionError: 완료 신호 없음 — 상태 ['APPROACH', 'ACQUIRE', 'PRE_ENTRY', 'ABORT']
- `test_mission_gate::test_gate_disturbed[2]` — AssertionError: 완료 신호 없음 (마지막 상태 FOLLOW_GATE_1/3)
- `test_mission_gate::test_gate_disturbed[3]` — AssertionError: 완료 신호 없음 (마지막 상태 FOLLOW_GATE_1/3)
- `test_mission_gate::test_gate_disturbed[4]` — AssertionError: 완료 신호 없음 (마지막 상태 FOLLOW_GATE_1/3)
