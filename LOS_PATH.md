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
