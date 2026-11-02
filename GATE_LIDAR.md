# 라이다 게이트 미션: 수조 3쌍 미리보기

게이트 미션은 YOLO의 빨강·초록 분류를 사용하지 않는다. 선체 라이다의
`/scan`을 점유 지도 노드가 `/occupancy_grid`로 바꾸고, gate behavior가
기존 `build_dri()`로 위험도를 계산한다. 지도에서 작은 점유 물체를 묶어
전진 방향에서 비슷한 거리에 있는 두 물체를 한 쌍으로 고른다. 한 쌍씩
중앙을 통과하고 출구까지 간 뒤 다음 쌍을 찾는다. 기본 완료 조건은
**3쌍 통과 + 최종 `/mission/goal` 2m 이내**다.

경로는 배 위치, 게이트 중앙, 출구를 연결하는 2차 B-spline이다. 중앙을
지나도록 제어점을 계산하고 공통 `los(path, thrust_ratio)`로 추종한다.
기존 장애물 회피의 자유 우회 경로 생성기는 사용하지 않는다. 생성한
경로와 실제 LOS 주시 선분을 DRI 및 점유 셀과의 최소 거리로 검사한다.
경로에 미관측(`-1`) 셀이 있으면 안전한 수면으로 간주하지 않고 정지한다.
현재 점유 지도에는 오래된 셀의 시간 감쇠가 없으므로 최신 `/scan`에서
선택한 쌍의 양쪽 기둥이 실제로 보이는지도 검사한다.
현재 점유 지도와 게이트 검출은 모두 라이다 스캔 원점을 `base_link`와
일치한다고 가정한다. 실물 장착 위치가 다르면 그 오프셋을 먼저 보정해야
중앙 경로의 좌표가 맞는다.

## 설정

[`gate_params.yaml`](src/kaboat_bringup/config/gate_params.yaml)의 기본
`gate.required_pairs: 3`은 수조 실험용이다. 4쌍 이상인 장소에서는 이
값을 실제 쌍 수로 바꾼다. 쌍은 한 번에 모두 보여야 하는 것이 아니라
가까운 것부터 하나씩 찾는다. `min_gate_width`, `max_gate_width`,
`min_clearance`, `max_pair_along_error`는 실물 장애물 간격과 선체 폭을
측정한 뒤 조정한다. `min_pair_advance`, `max_midpoint_shift`는 이전에
통과한 쌍과 다음 쌍 사이의 전진 거리 및 중앙선 이동 허용값이다.
기본 `gate.motion_enabled: false`,
`gate.thrust_ratio: 0.5`다. `gate.occ_threshold: 95`는 현재 점유 지도가
약 4회 히트로 확인한 셀을 사용한다.

미션 목표는 **마지막 게이트 뒤쪽**의 수조 `odom` 좌표로 설정한다.
마지막 게이트 출구에서 목표까지 기본 4m 이하여야 하며, 더 멀면
`GOAL_LEG_UNSAFE_OR_TOO_FAR`로 정지한다. 수조 밖 시뮬레이션 좌표가
들어 있는 기본 `mission_params.yaml`을 그대로 사용하지 않는다.

## 모터 없이 확인

젯슨에서 `/odom`과 선체 `/scan`을 먼저 준비한다. 이미
`/occupancy_grid`가 발행 중이면 아래 명령에서
`start_occupancy_grid:=true`를 빼야 한다. `goal_x`, `goal_y`는 **실측한
마지막 게이트 뒤쪽 좌표**로 바꾼다. 아래 값은 예시일 뿐이다.

```bash
cd ~/Desktop/kaboat_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to kaboat_bringup kaboat_behaviors
source install/setup.bash
export ROS_DOMAIN_ID=42
ros2 launch kaboat_bringup gate_tank_preview.launch.py \
  goal_x:=9.0 goal_y:=2.5 required_pairs:=3 start_occupancy_grid:=true
```

이 launch는 gate behavior와 단일 gate mission manager만 실행한다.
`cmd_mux`와 모터 드라이버는 시작하지 않으며 `/cmd/gate`는 0이다.
다른 mission manager나 점유 지도 노드가 실행 중이면 먼저 종료한다.

```bash
ros2 topic echo /gate/status
ros2 topic echo /gate/cmd_preview
ros2 topic info /scan
ros2 topic info /odom
ros2 topic info /occupancy_grid
```

RViz 고정 프레임은 `odom`으로 두고 `Path` 표시를 `/gate/path`,
`MarkerArray`를 `/gate/pairs`에 연결한다. 두 빨간 점은 선택한 기둥,
초록 점은 중앙이다. `WAIT_GATE_PAIR`, `GATE_POSTS_NOT_VISIBLE`,
`PATH_UNSAFE`, `PATH_UNOBSERVED`, `LOOKAHEAD_UNSAFE` 상태에서는 전진
명령을 내지 않는다.
실물에서 기둥 짝, 좌표계, 폭, 경로 안전 여유를 확인한 뒤에만 주행
설정을 조정한다. 현재 검증은 합성 지도와 ROS 노드 미리보기까지다.
