# 수조 5개 웨이포인트 순차 시험

[`waypoint_sequence_test.yaml`](src/kaboat_hardware/config/waypoint_sequence_test.yaml)의
`waypoints_x`, `waypoints_y`, `waypoints_angle_deg`, `waypoints_use_heading`에
각각 **정확히 5개** 값을 지정한다. 같은 순번의 네 값이 한 목표를 이룬다.
`waypoints_use_heading`이 `false`이면 해당 각도는 무시하고 위치만 맞춘다.
`true`이면 위치에 도착한 뒤 지정한 헤딩(도 단위)까지 맞춘다. 모든 좌표는
수조의 `odom` 좌표계이며, 기본 수조 크기는 10 × 5m다.

목표에서 0.35m 이내(헤딩을 켠 경우 ±8° 이내)로 들어오면 명령은 0이 된다.
기본 도착 안정화 0.5초 뒤 **연속 3초 동안 그 범위를 유지**해야 다음
웨이포인트로 진행한다. 범위를 벗어나면 같은 목표로 복귀하고 3초를 다시
센다. 다섯 번째 목표에서는 `COMPLETE_HOLD`를 알린 후에도 계속 위치를
감시하고, 밀려나면 같은 목표로 복귀한다. `hold_duration_sec:=3.0`이
기본값이다.

젯슨에서 `/odom`을 발행하는 ROS 환경을 준비하고 워크스페이스를 빌드한다.
아래의 워크스페이스 경로는 젯슨에 실제로 클론한 위치로 바꾼다.

```bash
cd ~/Desktop/kaboat_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to kaboat_hardware
source install/setup.bash
export ROS_DOMAIN_ID=42
ros2 topic info /odom
```

먼저 미리보기 모드로 시작한다. 기본 `thrust_ratio:=0.5`는 기존 안전
상한의 50%다. 이 모드에서는 `/cmd_vel` 발행자가 생기지 않는다.

```bash
ros2 launch kaboat_hardware waypoint_sequence_test.launch.py \
  hold_duration_sec:=3.0
```

같은 젯슨 ROS 환경의 다른 터미널에서 출발하고 상태를 확인한다.

```bash
ros2 service call /waypoint_sequence/start std_srvs/srv/Trigger '{}'
ros2 topic echo /waypoint_sequence/status
ros2 topic echo /waypoint_sequence/cmd_preview
```

RViz는 고정 프레임을 `odom`으로 설정하고, `Path` 표시를
`/waypoint_sequence/path`에 연결한다. 현재 목표는
`/waypoint_sequence/goal`에 나온다. 상태 메시지의 `WAYPOINT_1/5`부터
`WAYPOINT_5/5`까지 순서대로 진행하고, 마지막 유지 상태는
`COMPLETE_HOLD`로 표시된다.

실제 주행은 다른 `/cmd_vel` 발행 노드가 없는 상태에서 아래처럼 별도
실행한다. 이때도 시작 서비스를 호출하기 전에는 정지 명령을 낸다.
모터 드라이버와 `/odom`은 별도로 실행해야 한다.

```bash
ros2 launch kaboat_hardware waypoint_sequence_test.launch.py \
  hold_duration_sec:=3.0 thrust_ratio:=0.3 motion_enabled:=true
```

```bash
# 시작
ros2 service call /waypoint_sequence/start std_srvs/srv/Trigger '{}'
# 일시 정지
ros2 service call /waypoint_sequence/pause std_srvs/srv/Trigger '{}'
# 비상 정지
ros2 topic pub --once /emergency_stop std_msgs/msg/Bool '{data: true}'
```

`/odom`이 0.5초 넘게 끊기거나 발행자가 둘 이상이면 정지한다. 구동
모드에서는 `/cmd_vel` 발행자가 이 노드 하나가 아니어도 정지한다.
기본값은 미리보기이며 실제 배 주행은 아직 검증하지 않았다.
