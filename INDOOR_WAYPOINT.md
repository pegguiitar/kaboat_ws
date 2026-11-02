# 실내 수조 공통 waypoint 제어

`waypoint(x, y, angle=None, thrust_ratio=1.0)`은 수조와 일치하는
`odom` 좌표의 목표다.
`x`, `y`는 미터, 선택 사항인 `angle`은 도(deg)이며 **0°는 +X,
90°는 +Y, 180°는 -X**다. 각도를 주면 목표 위치에 접근한 뒤 지정
헤딩을 맞춘다. 기본 도착 조건은 위치 0.35m 이내, 각도를 준 경우
헤딩 8° 이내가 0.5초 동안 유지되는 것이다.
`thrust_ratio`는 0~1 범위이며, `1.0`은 현재 설정된 안전 추력 상한
(기본 전진 약 10.1%, 회전 약 2.5%)을 그대로 사용한다. `0.5`는 두
상한을 절반으로 낮추고, `0.0`은 추진·회전 명령을 0으로 만든다.

공통 계산은
[`waypoint_controller.py`](src/kaboat_behaviors/kaboat_behaviors/waypoint_controller.py)에
있다. 미션 behavior는 이미 가진 `/odom`과 `/cmd/<mission>` 배선을 통해
아래처럼 호출한다. `waypoint()`는 명령과 도착 여부를 **반환**하며 직접
`/cmd_vel`을 발행하지 않는다.

```python
def compute_cmd(self):
    cmd, reached = self.waypoint(5.0, 2.5, 180.0, thrust_ratio=0.5)
    if reached:
        self.report_complete()
    return cmd
```

최종 헤딩이 필요 없는 미션은
`self.waypoint(5.0, 2.5, thrust_ratio=0.5)`로 호출한다.
위치 유지 미션은 기존 감속 거리를 유지하기 위해
`self.waypoint(5.0, 2.5, slow_radius=5.0)`을 사용한다.

도킹 미션의 staging 지점도 `staging_angle_deg`를 주면 이 공통 제어기를
사용한다. 도착 위치와 헤딩이 모두 맞아야 `APPROACH → ACQUIRE`로 넘어간다.
카메라로 계산하는 W1/W2/W3 슬롯 진입 제어는 기존 도킹 FSM을 유지한다.
각도를 생략하면 기존 staging 위치 접근 동작을 유지한다.

```bash
ros2 launch kaboat_bringup real_docking_test.launch.py \
  staging_x:=5.0 staging_y:=2.5 staging_angle_deg:=180.0 \
  dock_weights:=/absolute/path/model.pt wall_to_mouth_m:=2.5 \
  dock_motion_enabled:=false
```

## 단일 목표점 수조 시험

젯슨에서 `indoor_tank.launch.py`로 **단일 `/odom` 발행자**를 준비한다.
센서·좌표를 확인하는 동안 추진 노드는 켜지 않는다. 이 런처는 센서나 모터
드라이버를 자동 실행하지 않는다. 목표는 기본적으로 수조 벽에서 0.25m
이상 안쪽이어야 한다. 새 목표를 주려면 이 런처를 종료하고 다시 실행한다.

```bash
cd ~/Desktop/kaboat_ws/repo
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=42
ros2 launch kaboat_hardware indoor_waypoint.launch.py \
  x:=5.0 y:=2.5 angle:=180.0 thrust_ratio:=0.5 motion_enabled:=false
```

`motion_enabled:=false`는 계산 결과만 `/waypoint/cmd_preview`로 내고
`/cmd_vel` 발행자는 만들지 않는다. 목표는 `/waypoint/goal`, 상태는
`/waypoint/status`에서 확인한다. 주요 상태는 `PREVIEW_APPROACH`,
`PREVIEW_ALIGN`, `PREVIEW_REACHED`다.

```bash
ros2 topic echo /waypoint/goal --once
ros2 topic echo /waypoint/status --once
ros2 topic echo /waypoint/cmd_preview --once
ros2 topic info /odom --verbose
ros2 topic info /cmd_vel --verbose
```

실제 이동을 시작하려면 preview 런처를 종료하고, 다른 `/cmd_vel` 발행 노드
(예: `cmd_mux`, 직진·B-Spline·정점 유지 테스트)가 없는 상태에서 모터
드라이버를 별도로 시작한 뒤 다음처럼 재실행한다. 이 제어기는 별도 장애물
회피 경로를 계산하지 않으므로 수조 경로를 비워 두고 저속으로 확인한다.

```bash
ros2 launch kaboat_hardware indoor_waypoint.launch.py \
  x:=5.0 y:=2.5 angle:=180.0 thrust_ratio:=0.5 motion_enabled:=true
```

`/odom`이 0.5초 넘게 끊기거나 발행자가 1개가 아니면, 비상 정지가 켜지면,
또는 다른 `/cmd_vel` 발행자가 생기면 즉시 0 명령을 낸다. 도착 후에도 0
명령을 유지한다. 실물에서 목표 좌표·헤딩과 실제 움직임을 대조하기 전에는
이 제어기의 물리적 정확도가 검증된 것으로 보지 않는다.
