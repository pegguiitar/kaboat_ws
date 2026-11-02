# 미션 공통 LOS 경로 추종

behavior에서는 `cmd, reached = self.los(path, thrust_ratio)`로 호출한다.
`path`는 `odom` 좌표계의 샘플 경로이며, **미션 시작 시 한 번 만들고** 매
제어 주기마다 같은 객체를 넘긴다. `los()`는 명령과 도착 여부를 반환하며
직접 `/cmd_vel`을 발행하지 않는다. `BehaviorBase`가 기존 미션 명령
토픽으로 발행한다.

```python
from kaboat_behaviors.behavior_base import BehaviorBase
from kaboat_behaviors.los_controller import bspline_path, circle_path

class MyBehavior(BehaviorBase):
    STATE_NAME = 'my_mission'
    CMD_TOPIC = '/cmd/my_mission'

    def on_activate(self):
        self.path = bspline_path([(8.5, 2.0), (7.0, 3.5),
                                  (5.0, 1.5), (1.5, 2.5)])

    def compute_cmd(self):
        cmd, reached = self.los(self.path, 0.5)
        if reached:
            self.report_complete()
        return cmd
```

새 미션 이름을 쓰는 경우 `mission_manager`의 미션 순서와 `cmd_mux`의
미션→명령 토픽 매핑에도 등록한다.
열린 경로의 첫 구간은 출발 위치 근처에 두어야 한다.

원 경로는 `self.path = circle_path(5.0, 2.5, 1.2,
direction='ccw', target_laps=2.0)`로 만든다. `direction='cw'`는 시계방향,
`target_laps=None`은 무한 순회다. 두 생성 함수는 모두
[`SampledPlan`](src/kaboat_behaviors/kaboat_behaviors/bspline_planner.py)을
반환한다. 이미 등간격으로 샘플링한 임의의 경로는
`sampled_path(points, spacing=0.05, curvatures=...)`로 전달할 수 있다.
곡률을 생략하면 샘플로 근사하므로 B-spline에는 생성 함수의 해석적 곡률을
사용하는 편이 안정적이다.

LOS는 현재 위치에서 경로의 최근접 진행점을 단조 갱신하고, 경로 위
앞쪽 지점을 향해 PD로 조향한다. 전방 창의 최대 절대 곡률 `κ`에 따라
`v = clamp(ω_max / κ, v_min, v_max)`로 감속한다. 열린 경로는 종점에서
감속·정지하고, 원 경로는 끝과 시작을 이어서 목표 바퀴 수를 센다.
이 계산은 [`bspline_track_test`](src/kaboat_hardware/kaboat_hardware/bspline_track_test.py)의
공통 `follow_cmd()`를 사용한다.

`thrust_ratio`는 0~1이며 `1.0`은 behavior에 설정된 기존 안전 상한의
100%, `0.5`는 그 절반, `0.0`은 정지 명령이다. 경로 좌표계가 `odom`이
아니거나 odom이 0.5초 이상 지연되면 명령은 0이다. 이 함수는 장애물
회피 경로를 만들지 않는다. 속도와 추력의 변환값은 기존 시험 코드의
교정값이므로 실물 로그로 확인해야 한다.

## 수조에서 두 경로 시험

`/odom` 발행 노드가 있는 컴퓨터의 ROS 환경에서 새 실행 파일을 빌드한다.
노트북 컨테이너라면 `/workspace/kaboat_ws`, 젯슨이면 실제로 클론한
워크스페이스 경로에서 실행한다. 두 장치의 `ROS_DOMAIN_ID`는 각각 현재
운용 중인 값을 유지한다.

```bash
cd /workspace/kaboat_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to kaboat_hardware
source install/setup.bash
ros2 topic info /odom
```

센서와 `/odom`을 먼저 실행한 뒤 아래 둘 중 하나만 실행한다. 두 명령은
기본적으로 `/cmd_vel`을 발행하지 않고 `/los_test/path`,
`/los_test/cmd_preview`, `/los_test/status`만 발행한다. RViz에서
고정 프레임 `odom`으로 설정하고 `Path` 표시를 `/los_test/path`에
연결하면 목표 경로를 볼 수 있다.

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch kaboat_hardware los_path_test.launch.py path_kind:=bspline
```

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch kaboat_hardware los_path_test.launch.py path_kind:=circle \
  center_x:=5.0 center_y:=2.5 radius:=1.2 direction:=ccw target_laps:=2.0
```

각 테스트는 시작 명령을 받기 전에는 정지 명령만 미리보기에 낸다. 다른
터미널에서 같은 컨테이너와 `ROS_DOMAIN_ID`로 다음을 실행한다.

```bash
ros2 service call /los_test/start std_srvs/srv/Trigger '{}'
ros2 topic echo /los_test/status
ros2 topic echo /los_test/cmd_preview
```

실제 구동 시험은 다른 `/cmd_vel` 발행 노드가 없는 상태에서
`motion_enabled:=true`를 명시한다. 이때도 시작 서비스 호출 전에는
정지 명령을 낸다. `thrust_ratio:=0.5`가 기본이며 이는 기존 안전
추력 상한의 50%다. 비상 정지는 `/emergency_stop`에 `true`를 발행한다.

```bash
ros2 launch kaboat_hardware los_path_test.launch.py \
  path_kind:=circle thrust_ratio:=0.3 motion_enabled:=true
# 다른 터미널에서 시작
ros2 service call /los_test/start std_srvs/srv/Trigger '{}'
# 중지
ros2 topic pub --once /emergency_stop std_msgs/msg/Bool '{data: true}'
```

`/odom`이 0.5초 넘게 멈추거나 발행자가 둘 이상이면 정지한다. B-spline
제어점은 `test_coordinates.py`의 `BSPLINE_TRACK`, 원의 기본값은
`CIRCLE_DRIVE`에서 가져온다. 테스트 노드는 수조 외부로 나가는 경로
설정을 시작 시 거부한다.

기존 탐색 behavior의 색상별 원형 선회는 `circle_path()`와 `self.los()`를
사용한다. 장애물 회피 behavior는 가변 B-spline 경로의 검사·수리·재생성을
`AvoidFsm`에서 처리하고, 추종 명령은 LOS와 같은 `follow_cmd()`로 계산한다.
기존 독립 `bspline_track_test`와 `circle_drive_test`는 별도 시험 노드이며,
이 새 테스트 파일은 공통 LOS 경로 생성·추종 코드를 시험한다.
