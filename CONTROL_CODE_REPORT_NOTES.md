# KABOAT 제어 코드와 추진기 구동 경로 — 보고서 작성 자료

> 작성 기준: 2026-11-03. 분석 대상은 `real` 브랜치의 커밋 `b6ca813e89fac66eb4e07145e0f91dde2c6bdb83`이다. 이 Markdown 파일은 `jiwoo` 브랜치에 보관한다. 아래의 소스 링크는 문서와 코드의 버전 차이를 피하도록 해당 `real` 커밋에 고정했다. 이 자료는 코드 구조와 설정을 설명하며, 현재 선체에서 실제 모터가 회전했다거나 물 위에서 제어 성능을 검증했다는 뜻은 아니다.

## 1. 보고서에 넣을 핵심 결론

KABOAT의 자율 제어는 `/odom`에서 위치·선수각·요레이트를 읽고 목표점 또는 경로의 전방 주시점을 정한다. `waypoint()`는 한 점과 선택적인 최종 선수각으로 접근하고, `los()`는 미리 생성한 곡선 또는 원의 샘플을 따라간다. 두 함수의 방위 제어는 **방위 오차에 비례하는 P항과 측정 요레이트를 감쇠하는 D항을 합한 PD형 제어**다. 적분항이 없으므로 현재 코드의 이 경로를 **PI 제어라고 쓰면 부정확하다**. 전진 출력은 거리 또는 경로 곡률에 따라 조절하고, 최종 명령은 실제 속도 단위가 아닌 정규화된 추력 비율로 발행한다. 미션 상태에 맞는 명령만 `cmd_mux`가 `/cmd_vel`로 전달하고, `thruster_driver`가 이를 좌우 추진기 비율과 ESC용 펄스폭 숫자로 변환해 선택한 하드웨어 백엔드에 보낸다.

보고서에서는 다음 세 층을 구분해 기술하는 것이 정확하다.

| 층 | 입력과 출력 | 실제 책임 |
|---|---|---|
| 유도·자세 제어 | 목표 좌표/경로 + `/odom` → `Twist` | 목표 방위, 전진 추력 비율, 회전 추력 비율 계산 |
| 명령 중재·보호 | 미션별 `/cmd/*` → `/cmd_vel` | 현재 미션만 통과, 명령 단절 감시. 수조 단독 미션은 추가 guard를 거칠 수 있음 |
| 구동·하드웨어 연결 | `/cmd_vel` → 좌우 펄스폭 숫자 → 시리얼/I²C | 좌우 차동 혼합, 출력 제한, 펄스폭 계산, 전송. 실제 전기 펄스 생성은 선택한 보드 또는 PWM 칩의 역할 |

## 2. 제어 신호의 전체 흐름

### 2.1 종합 미션에서의 신호 경로

```text
센서/위치 추정 → /odom (x, y, yaw, yaw_rate)
미션 관리자 → /mission/state + /mission/goal 또는 /mission/entry_goal
             ↓
활성 행동 노드의 waypoint()/los() 또는 미션별 제어기 → Twist(linear.x, angular.z)
             ↓
/cmd/approach | /cmd/gate | /cmd/station | /cmd/dock | /cmd/search | /cmd/avoid
             ↓
cmd_mux: /mission/state에 맞는 명령만 선택, 300 ms 감시
             ↓
/cmd_vel (정규화된 전진·회전 추력 명령)
             ↓
thruster_driver: E-stop/RC/노트북 조종/자율주행 중 하나 선택, 50 Hz 제어 루프
             ↓
ThrusterMixer: 좌 = 전진 − 회전, 우 = 전진 + 회전
             ↓
좌우 제한·변화율 제한 → 펄스폭 [µs] 계산
             ↓
serial: Jetson → USB 시리얼 → 아두이노/ESP32 → ESC → 좌·우 모터
또는 pca9685: Jetson → I²C PWM 칩 → ESC → 좌·우 모터
```

`mission_manager`는 미션 시작 자세가 설정돼 있으면 먼저 `approach` 상태를 켠다. `mission_approach`가 진입 좌표와 선수각에 도달했다고 알린 후에 해당 미션(`gate`, `station`, `dock`, `search`, `avoid`)을 시작한다. 미션 종료는 해당 행동 노드의 `/mission/complete`와 목표점 전이 반경 조건을 함께 확인한다. 각 행동 노드는 자신의 상태에서만 0.1초마다 `compute_cmd()`를 실행하고, `BehaviorBase`가 결과를 제한해 미션별 명령 토픽에 발행한다. [미션 관리자][mission-manager], [행동 공통부][behavior-base], [명령 중재기][cmd-mux]

### 2.2 수조 단독 시험과 다른 경로

`indoor_waypoint`, `los_path_test`, `bspline_track_test` 등의 단독 시험 노드는 `cmd_mux` 없이 `/cmd_vel`을 직접 발행할 수 있다. 따라서 종합 미션 스택의 `/cmd_vel` 발행자와 동시에 구동하면 명령이 충돌할 수 있다. `tank_gate_mission`/`tank_avoid_mission`의 단일 미션 구성은 다르다. 여기서는 `cmd_mux` 출력을 `/tank_mission/cmd_raw`로 remap하고 `tank_mission_guard`가 위치·스캔·격자·명령의 최신성과 시작 서비스를 확인한 뒤에만 `/cmd_vel`을 발행한다. Launch는 시작점·목표점이 지정한 수조 범위 안에 있는지 검사하지만, guard가 주행 중 선체의 벽 접근 거리를 감시하는 것은 아니다. 기본 `drive_enabled=false`이면 출력 토픽을 만들지 않는 미리보기 모드다. [수조 미션 배선][tank-launch], [수조 미션 보호 노드][tank-guard]

`autonomy.launch.py` 자체는 실제 `thruster_driver`를 실행하지 않는다. 실물 모터 출력이 필요하면 별도 `thrusters.launch.py`를 올리고 적절한 백엔드를 선택해야 한다. `thrusters.launch.py`의 기본 백엔드는 `dummy`이므로, 이 상태에서는 펄스폭 숫자를 계산해도 물리적인 PWM은 나오지 않는다. [자율운항 실행 구성][autonomy-launch], [추진기 실행 구성][thruster-launch]

## 3. 좌표·메시지·단위 계약

| 값 | 코드에서의 의미 | 단위 또는 범위 |
|---|---|---|
| `odom.pose.pose.position.x/y` | 현재 선체의 2차원 위치 | `odom` 좌표계의 m |
| `odom.pose.pose.orientation` | 쿼터니언에서 추출한 선수각 `yaw` | rad, +X 방향이 0°, +Y 방향이 90°, 반시계 + |
| `odom.twist.twist.angular.z` | 실제 측정된 요레이트 | rad/s |
| `waypoint(x, y, angle)`의 `angle` | 도착 후 맞출 선수각. 생략하면 위치만 판정 | **도(deg)**; 내부에서 rad로 변환 |
| `Twist.linear.x` | 전진/후진 구동 명령 | 이 스택에서는 **정규화 추력 비율**, m/s 아님 |
| `Twist.angular.z` | 좌우 차동 회전 명령 | 이 스택에서는 **정규화 추력 비율**, rad/s 아님 |
| `thrust_ratio` | 해당 호출의 선형·회전 허용 출력 배율 | 0.0~1.0 |
| `left_pwm/right_pwm` | ESC에 줄 펄스의 높은 레벨 유지 시간 | µs; 기본 중립 1500, 전진 상한 1900, 후진 상한 1100 |

`/odom`이 없거나 0.5초 넘게 오래됐거나, `header.frame_id`가 `odom`이 아니거나, 위치·쿼터니언·요레이트에 비정상 값이 있으면 `BehaviorBase.waypoint()`는 영 명령과 `reached=false`를 반환한다. `los()`도 동일한 오도메트리 검사와 경로의 `frame_id='odom'` 검사를 통과해야 명령을 만든다. 좌표계 이름만 맞아도 원점이나 축이 다른 경우에는 올바른 제어가 되지 않으므로 실물 적용 시 `/odom`의 기준을 별도로 확인해야 한다. [행동 공통부][behavior-base]

## 4. PI 요청과 실제 PD 제어의 관계

### 4.1 코드에 구현된 제어식

현재 `waypoint()`와 `los()`의 공통 방향 제어는 개념적으로 다음과 같다.

```text
목표 방위 ψ_ref = atan2(y_target − y_boat, x_target − x_boat)
방위 오차 e_ψ = wrap(ψ_ref − ψ_boat)             # −π ~ +π
회전 명령 u_turn = clamp(Kp · e_ψ − Kd · r_measured, ±u_turn,max)
```

`r_measured`는 `/odom`의 `twist.twist.angular.z`이고, `Kp` 기본값은 `18/118.6 ≈ 0.1518`, `Kd` 기본값은 `9/118.6 ≈ 0.0759`, 회전 명령 상한은 `3/118.6 ≈ 0.0253`이다. 추력 기준 `118.6 N`으로 나눠 정규화된 명령에 맞춘 숫자다. 이 숫자들이 실제 선체에서 18 N/rad 또는 9 N/(rad/s)로 작용한다는 것은 ESC·프로펠러 교정 없이는 단정할 수 없다. `waypoint()`는 `WaypointController.step()`에서, `los()`는 `follow_cmd()` → `control_velocity()`에서 이 형태를 쓴다. [행동 공통 게인][behavior-base], [웨이포인트 제어기][waypoint-controller], [공통 속도·방위 제어기][motion-controller]

P항은 목표 방위와 현재 선수각의 차이에 비례해 선체를 목표 쪽으로 돌린다. D항은 선체가 이미 회전 중일 때 그 요레이트에 반대로 작용해 과도한 회전을 감쇠한다. 엄밀히는 오차를 시간 미분한 값을 직접 계산하지 않고 **측정 요레이트를 감산**한다. 목표 방위가 고정돼 있을 때는 `de_ψ/dt ≈ −r_measured`이므로 PD형 감쇠로 해석할 수 있다. `los()`에서는 목표 주시점이 이동하므로 오차 미분과 측정 요레이트가 완전히 같지는 않다.

### 4.2 PI 제어로 기록하면 안 되는 이유

PI 제어는 보통 `u = Kp·e + Ki·∫e dt`처럼 오차 누적값을 가진다. 이 경로의 소스에는 `Ki`, 오차 적분 상태, 적분 상한(anti-windup)이 없다. `waypoint()`의 거리 비례 감속과 `los()`의 곡률 감속도 적분항이 아니다. 또한 목표 전진 속도와 실제 속도의 차이를 적분하는 속도 PI 루프가 이 경로에 구현돼 있지 않다. 따라서 보고서 문장은 **“방위 오차 P항과 측정 요레이트 감쇠항을 사용한 PD형 조향”**으로 쓰는 것이 코드와 일치한다. 추후 실제 PI 제어가 추가되면 적분 대상, `Ki`, 누적 주기, 포화·anti-windup, 리셋 조건을 별도로 기록해야 한다. [웨이포인트 제어기][waypoint-controller], [공통 속도·방위 제어기][motion-controller]

## 5. `waypoint()` 함수 상세

### 5.1 호출 계약과 처리 순서

`BehaviorBase.waypoint(x, y, angle=None, thrust_ratio=1.0, *, slow_radius=None)`은 목표 좌표와 선택적인 최종 방향으로 가기 위한 `(Twist 명령, 도착 여부)`를 반환한다. `x/y`는 `odom` 프레임의 m, `angle`은 도, `thrust_ratio`는 0~1이다. 별도의 순수 함수 `waypoint()`는 입력이 유한한지와 `thrust_ratio` 범위를 검사해 `Waypoint` 목표 객체를 만든다. 위치 경계나 수조 벽 검사는 이 공통 함수 자체가 담당하지 않는다. [행동 공통부][behavior-base], [웨이포인트 제어기][waypoint-controller]

1. 목표 좌표와 `thrust_ratio`를 검증하고, 오도메트리의 최신성과 프레임을 확인한다.
2. `dx = x_target − x_boat`, `dy = y_target − y_boat`, `d = hypot(dx, dy)`를 계산한다.
3. 목표 위치 밖(`d > 0.35 m`)이면 목표 방위와 현재 선수각의 차이를 계산해 PD형 회전 명령을 낸다. 기본 전진 명령은 `max_linear × min(d / slow_radius, 1)`이고, `slow_radius` 기본값은 `1.5 m`이다.
4. 목표 각도를 **생략**한 경우에는 방위 오차가 60° 이상일 때 전진을 `max_linear × 0.3`으로 낮춘다. 제자리 회전으로 바꾸지 않는 위치 전용 접근 규칙이다.
5. 목표 각도를 **지정**한 경우에는 접근 중 방위 오차가 90° 이상이면 전진 0, 60° 이상이면 전진을 20%로 줄인다.
6. 위치 오차가 허용 범위에 들어오고 최종 각도가 지정돼 있다면 `ALIGN` 단계에서 전진 0으로 방향을 맞춘다. 각도 허용 오차 기본값은 8°다.
7. 위치와 필요한 경우 최종 각도가 모두 조건을 만족하면 0.5초간 `SETTLING`을 유지한다. 그 후 `REACHED`와 `reached=true`를 반환한다. 접근·정렬·정착 중 `thrust_ratio`는 전진·회전 명령 모두에 곱해진다.

오도메트리가 끊기면 정착 시간 기록도 지워져 재수신 뒤 허용 시간부터 다시 센다. 호출자는 반환된 명령을 자신의 `/cmd/<미션>`으로 보내고, `reached=true`일 때 필요한 완료 신호를 낸다. 예를 들어 `mission_approach`는 시작 위치·선수각에 도달하면 `/mission/complete`를 발행한다. [웨이포인트 제어기][waypoint-controller], [진입 제어 노드][mission-approach]

### 5.2 수식과 예시

```text
u_fwd = max_linear × min(d / slow_radius, 1) × thrust_ratio
u_turn = clamp(Kp × wrap(atan2(dy, dx) − yaw) − Kd × yaw_rate,
               −max_angular, +max_angular) × thrust_ratio
```

위 식은 일반적인 접근 단계의 기본형이다. 실제로는 앞 절의 큰 방위 오차 감속, 최종 방향 정렬, 도착 후 영 명령이 추가된다. 예를 들어 기본 `max_linear=12/118.6≈0.1012`, `slow_radius=1.5 m`, 목표까지 `d=0.75 m`, `thrust_ratio=0.5`이며 큰 방위 오차가 없다면 전진 명령은 약 `0.1012 × 0.5 × 0.5 = 0.0253`이다. 이 값은 **2.53 cm/s가 아니라 정규화된 전진 명령**이다.

## 6. `los()` 함수 상세

### 6.1 경로 생성과 전방 주시점

`BehaviorBase.los(path, thrust_ratio=1.0)` 역시 `(Twist 명령, 도착 여부)`를 반환하지만, 입력은 단일 좌표가 아니라 `SampledPlan`이다. 경로는 제어 주기마다 새로 만들기보다 `bspline_path()`, `circle_path()`, `sampled_path()`로 한 번 만들고 **동일한 객체**를 반복 전달한다. 그래야 `LosFollower`가 어느 샘플까지 진행했는지 유지할 수 있다. `bspline_path()`는 기본 3차 B-spline을 약 `0.05 m` 호길이 간격으로 샘플링하고 미분값으로 곡률을 계산한다. `circle_path()`는 닫힌 원을 샘플링하고 곡률을 `±1/radius`로 둔다. [LOS 추종기][los-controller]

추종기는 선체 근처의 샘플을 찾아 `progress_idx`를 갱신한다. 처음에는 개방 경로의 시작 부분에서 가까운 점을 찾고, 이후에는 직전 진행점 주변의 제한된 창에서 찾으며 인덱스를 뒤로 되돌리지 않는다. 개방 경로에서 전방 주시점은 현재 진행 샘플에서 `lookahead` 호길이만큼 앞선 샘플이다. 기본 미션용 `LosFollower`의 `lookahead`는 `1.0 m`이고, 경로 끝에 가까우면 종점으로 제한한다. 닫힌 원 경로는 샘플 인덱스를 순환시켜 주시점을 고른다. [LOS 추종기][los-controller], [경로 추종 명령][avoid-fsm]

여기서 `los()`라는 이름은 **경로상의 전방 주시점으로 향하는 시선(Line of Sight) 유도**를 뜻한다. 직선 경로에 대한 별도 횡오차 공식이나 “곡선과 일정 거리를 유지하는 평행 궤적”을 구현한 것은 아니다. 선체가 현재 위치에서 앞선 샘플을 향하는 직선 방향을 목표 방위로 삼기 때문에 곡선이 급하면 선체의 실제 궤적은 원래 곡선의 안쪽으로 잘릴 수 있다. 이 설명은 실제 수상 추종 오차의 측정 결과가 아니라 구현식의 기하학적 해석이다.

### 6.2 곡률에 따른 전진 감속과 PD형 조향

`follow_cmd()`는 앞쪽 `kappa_window` 구간의 **최대 절대 곡률**을 확인한다. 기본 `kappa_window=2.0 m`, `omega_max=0.52 rad/s`, `v_min=0.3 m/s`, `v_max=1.4 m/s`이며 다음 식으로 경로의 목표 속도 `v`를 만든다. 값은 제어 파라미터이며 실물 선체의 실측 한계라고 이 문서에서 주장하지 않는다. [경로 추종 명령][avoid-fsm], [곡률 계산][bspline-planner]

```text
κ ≈ 0이면 v = v_max
그 외에는 v = clamp(omega_max / |κ|, v_min, v_max)
개방 경로의 종점에 가까우면 v ← v × min(종점까지 거리 / slow_radius, 1)
큰 방위 오차에서는 별도의 저속 상한 적용
u_fwd = cmd_full × (v / v_full) ^ cmd_exp
u_turn = Kp × wrap(주시점 방위 − yaw) − Kd × yaw_rate
```

마지막 `u_fwd` 변환의 기본 `cmd_full=12/118.6`, `v_full=1.48 m/s`, `cmd_exp=1.64`는 **시뮬레이션 보정값**이 포함된 속도→명령 근사다. 이를 실물 선체의 속도 폐루프 제어 또는 실물에서 검증된 속도 곡선으로 쓰면 안 된다. `LosFollower`는 결과를 `max_linear`, `max_angular`로 각각 제한하고 `thrust_ratio`를 곱한다. 이후 `BehaviorBase._tick()`도 설정 상한을 다시 적용한다. [경로 추종 명령][avoid-fsm], [공통 속도·방위 제어기][motion-controller], [LOS 추종기][los-controller]

개방 경로는 종점에서 `0.4 m` 이내이고 남은 경로 길이가 `lookahead` 이하일 때 도착으로 판정한다. 닫힌 원은 지정된 바퀴 수와 종점 근접 조건을 만족하면 도착하고, `target_laps=None`인 원은 계속 추종한다. `gate_follower`는 안전한 게이트 경로를 만든 뒤 `los()`를 사용하고, 종점 쪽 목표 접근에 `waypoint()`를 사용한다. `search_circler`는 원 경로에 `los()`를 사용한다. [LOS 추종기][los-controller], [게이트 행동 노드][gate-follower], [탐색 행동 노드][search-circler]

## 7. 명령 제한·선택·안전 정지

### 7.1 행동 출력과 `cmd_mux`

`BehaviorBase`의 10 Hz 루프는 행동 노드가 반환한 `Twist`의 `linear.x`와 `angular.z`를 각각 `max_linear`, `max_angular`로 제한한다. 기본값은 `12/118.6≈0.1012`와 `3/118.6≈0.0253`이다. `waypoint()` 또는 `los()`에서 `thrust_ratio`를 사용한 경우에는 뒤따르는 회피 보정이 그 낮은 상한을 넘어가지 못하게 발행 직전에 한 번 더 제한한다. 이는 **전진·회전 성분의 상한**이며, 두 성분을 합친 각 모터 출력의 최종 상한과는 다르다. [행동 공통부][behavior-base]

`cmd_mux`는 `/mission/state`와 미션별 `/cmd/*`를 구독한다. 예를 들어 상태가 `gate`일 때 `/cmd/gate`만 `/cmd_vel`로 그대로 전달하고 다른 행동의 메시지는 버린다. `cmd_mux`는 `max_linear`를 계산하거나 모터 명령을 좌우로 나누지 않는다. 활성 토픽 명령이 기본 300 ms 넘게 끊기면 영 `Twist`를 내고, `idle`/`done` 등 매핑되지 않은 상태에서도 영 명령을 낸다. [명령 중재기][cmd-mux]

### 7.2 `thruster_driver`의 제어권 우선순위

`thruster_driver`는 기본 50 Hz 루프에서 다음 순서로 입력을 결정한다. 이는 `cmd_mux`와 별개의 **최종 구동 선택**이다. [추진기 드라이버][thruster-driver]

1. `/emergency_stop=true`: 즉시 중립 목표, 변화율 제한 상태 리셋.
2. 유효한 `/rc/cmd_vel`: 물리 RC 조종 입력을 우선 적용. 기본 유효 시간 0.5초.
3. `/teleop/enable=true`: `/teleop/cmd_vel`을 적용. 0.5초 동안 새 명령이 없으면 자율주행으로 자동 복귀하지 않고 중립 목표와 리셋을 적용한다. Jetson에서 전진·후진·회전 상한을 각각 `12/118.6`, `6/118.6`, `3/118.6`으로 제한한다.
4. 최근 0.3초 안에 수신한 `/cmd_vel`: 자율주행 명령을 적용한다.
5. 그 외: 워치독 타임아웃으로 중립 목표와 리셋을 적용한다.

선택된 입력의 `linear.x`/`angular.z`를 믹서에 보내고, 전송 결과와 펄스폭은 `/diagnostics`, 현재 제어 모드는 `/teleop/status`에 발행한다. 영 명령이 되어도 백엔드가 미연결이면 실제 ESC에 중립 펄스가 전달됐다고 단정할 수 없다. [추진기 드라이버][thruster-driver]

## 8. `/cmd_vel`에서 아두이노·ESC·모터까지

### 8.1 차동 혼합과 펄스폭 숫자 계산

`ThrusterMixer.mix()`가 아래 순서로 처리한다. 설정값은 [추진기 설정][thruster-config]의 기본값을 사용한다. [추진기 백엔드][thruster-backend]

1. `left_raw = linear.x − angular.z`, `right_raw = linear.x + angular.z`로 좌우 차동 명령을 계산한다. 양의 회전 명령은 이 수식상 오른쪽 추진기 비율을 높이고 왼쪽을 낮춘다. **실제 선체의 좌·우 회전 방향은 배선과 모터 반전 설정 확인이 필요하다.**
2. `invert_left/right`와 `left_trim/right_trim`을 적용한다. 기본은 반전 없음, 트림 1.0이다.
3. 각 모터 비율을 `±max_output_ratio`로 제한한다. 기본 1.0이며 행동 노드의 `max_linear/max_angular`와 별도이다.
4. 각 모터에 `max_slew_rate=2.0/s` 변화율 제한을 적용한다. 단, 제한기 최초 호출은 목표값을 바로 넣으며, 이후 업데이트 간의 변화를 제한한다. E-stop·워치독에서는 제한기를 리셋해 중립 목표가 즉시 계산되게 한다.
5. 정규화된 각 모터 비율을 ESC 펄스폭 **숫자**로 바꾼다. 0은 1500 µs, 양수는 `1525 + ratio × 375` µs, 음수는 `1475 + ratio × 375` µs로 계산한다. 정수 µs로 반올림한다. 후진 식의 `ratio`는 음수이므로 1100 µs 쪽으로 내려간다. 아주 작은 `|ratio|<10⁻⁴`는 1500 µs로 처리한다.

예를 들어 변화율·반전·트림·출력 제한의 영향이 없는 시점에 `/cmd_vel`의 `linear.x=0.05`, `angular.z=0.01`이면 좌우 비율은 `0.04`, `0.06`이다. 기본 설정에서 좌측은 `1525+0.04×375=1540 µs`, 우측은 `1525+0.06×375=1547.5→1548 µs`가 된다. 이는 **코드 계산 예시**이고 실제 회전수나 추력[N] 측정값은 아니다. [추진기 백엔드][thruster-backend]

### 8.2 시리얼 백엔드: Jetson → 아두이노/ESP32

`hardware_type=serial`일 때 `SerialBackend`가 지정 포트를 기본 `115200 baud`로 연다. [추진기 설정][thruster-config]과 [추진기 실행 구성][thruster-launch]에 적힌 포트 기본값은 `/dev/ttyCH341USB0`이며, 자동 포트 탐색은 기본으로 꺼져 있다. 다만 `thruster_driver.py` 클래스 자체의 내부 기본 포트는 `/dev/ttyUSB0`이므로 **어떤 설정 파일·launch 인자를 실제로 넣었는지**가 실행 결과를 결정한다. [추진기 드라이버][thruster-driver], [추진기 백엔드][thruster-backend]

Python 쪽은 좌우 펄스폭을 각각 1000~2000 µs로 한 번 더 제한하고 ASCII `LLLLRRRR\n`으로 보낸다. 앞 네 자리와 뒤 네 자리가 각각 좌측·우측 펄스폭이다. 위 계산 예시의 바이트열은 `15401548\n`, 중립은 `15001500\n`이다. 노드 종료 시에도 중립 문자열 전송을 시도한다. `serial.write()` 성공은 호스트가 바이트를 썼다는 뜻이며 보드가 이를 올바르게 해석했다는 확인 응답은 이 코드에 없다. `thruster_driver`의 50 Hz 루프는 **펄스폭 값의 전송/갱신 주기**이며, 아두이노의 펌웨어가 ESC 핀에 실제로 몇 Hz 펄스를 생성하는지는 이 저장소에서 확인할 수 없다. [추진기 백엔드][thruster-backend]

저장소에는 이 문자열을 수신·파싱하여 두 출력 핀에 펄스를 생성하는 **아두이노/ESP32 펌웨어 소스가 없다**. 따라서 보고서에는 “Jetson 코드가 좌우 ESC용 펄스폭 값을 시리얼로 보낸다”까지를 확인된 구현으로 적고, “아두이노가 이를 해석해 ESC 신호를 생성한다”는 부분은 **시스템 설계상 요구되는 다음 단계**로 기술해야 한다. 펌웨어의 핀 번호, 펄스 반복 주기, 수신 타임아웃 시 중립 처리, 잘못된 패킷 폐기, 실제 배선은 해당 보드의 코드와 현장 측정으로 확인해야 한다. ESC는 제어 펄스를 받아 모터 전력을 조절하므로, 펄스폭 숫자가 모터에 직접 전력 펄스로 인가된다고 표현하면 안 된다.

### 8.3 대체 백엔드와 물리 펄스의 의미

`hardware_type=pca9685`이면 Jetson이 I²C로 PCA9685의 좌우 채널을 설정한다. 이 백엔드는 기본 50 Hz, 즉 한 주기 약 20,000 µs를 기준으로 펄스폭을 4096분할 카운터 값으로 변환한다. 이 경우 실제 ESC 제어 핀의 신호는 PWM 칩이 만든다. `hardware_type=dummy`이면 값을 기억할 뿐 물리 신호가 없다. 어느 백엔드든 소프트웨어의 `1500 µs`는 **ESC 제어 신호의 한 주기에서 높은 레벨을 유지할 시간**을 뜻하며, 모터 전원선에 1500 µs 동안 전압을 직접 주는 명령이 아니다. [추진기 백엔드][thruster-backend]

## 9. 보고서에 옮겨 쓸 수 있는 본문 초안

### 9.1 제어 구조

> 본 시스템은 미션 상태, 위치 추정, 유도 제어, 추진기 출력을 ROS 2 토픽으로 분리하였다. 미션 관리자는 수행할 미션과 목표를 발행하고, 활성 행동 노드는 오도메트리에서 선체 위치·선수각·요레이트를 읽어 목표점 또는 경로 전방 주시점을 향하는 정규화 추력 명령을 계산한다. 미션별 명령은 `cmd_mux`를 통과해 `/cmd_vel`로 전달되며, 추진기 드라이버에서 좌우 차동 명령으로 변환된다.

### 9.2 PD 조향과 `waypoint()`·`los()`

> `waypoint()`는 목표점까지의 거리와 방위 오차를 이용해 접근·최종 방향 정렬·도착 판정을 수행한다. `los()`는 B-spline 또는 원 경로의 현재 진행점보다 앞선 샘플을 주시점으로 골라 추종하고, 전방 경로의 곡률이 클수록 목표 전진 속도를 낮춘다. 두 방식의 방향 제어는 방위 오차 비례항과 측정 요레이트 감쇠항으로 구성된 PD형 제어다. 현재 구현에는 오차 적분항이 없어 PI 제어라고 표기하지 않는다.

### 9.3 구동 신호

> 추진기 드라이버는 `/cmd_vel`의 전진·회전 명령을 각각 좌우 추진기 비율로 혼합하고, 반전·트림·출력 상한·변화율 제한을 적용한 뒤 ESC 제어 펄스폭 숫자로 변환한다. 시리얼 백엔드에서는 좌우 값을 `LLLLRRRR\n` 형식으로 보드에 전송한다. 저장소에서 확인되는 구간은 이 숫자의 계산과 시리얼 전송까지다. 보드가 실제 PWM을 생성하는 펌웨어와 ESC 배선은 별도 자료와 현장 계측으로 확인해야 한다.

## 10. 그림·표 작성 제안 및 검증 범위

- **그림 1:** `2.1`의 종합 미션 신호 경로를 “센서/위치 추정 → 유도 제어 → 명령 중재 → 추진기 드라이버 → 시리얼 보드 → ESC/모터” 블록도로 옮긴다. 수조 단독 미션은 guard를 거치는 분기, 단독 시험은 `/cmd_vel` 직결 분기로 표시한다.
- **그림 2:** `waypoint()`의 현재 위치, 목표점, 거리 `d`, 현재 선수각 `yaw`, 목표 방위 `ψ_ref`, 방위 오차 `e_ψ`를 평면도에 표시한다.
- **그림 3:** `los()`의 샘플 경로, 현재 진행점, 1 m 앞 주시점, 선체와 주시점을 잇는 선분, 앞쪽 2 m 곡률 평가 구간을 표시한다. 경로 곡선 자체와 선체가 실제로 가는 궤적을 구분한다.
- **표 1:** `3`절의 좌표·메시지·단위 계약. **표 2:** `7.2`절의 명령 선택 우선순위. **표 3:** `8.1`절의 정규화 비율→펄스폭 계산 예시.

이 문서의 검증 범위는 저장소의 Python 제어식·ROS 토픽 배선·설정 파일·시리얼 전송 형식의 **정적 코드 확인**이다. `real` 브랜치의 앞선 변경은 ROS 2 빌드와 관련 테스트를 통과했지만, 이 자료 작성 중 아두이노 펌웨어·시리얼 실선·오실로스코프 펄스·ESC·모터의 실제 동작은 검증하지 않았다. Word 본문에서 “구현하였다”와 “실물에서 확인하였다”를 구분해야 한다.

## 11. 소스 위치

아래 링크는 모두 문서 분석 기준 `real` 커밋의 파일이다. Word 작성 시 구현 근거 각주 또는 부록에 사용할 수 있다.

[behavior-base]: https://github.com/pegguiitar/kaboat_ws/blob/b6ca813e89fac66eb4e07145e0f91dde2c6bdb83/src/kaboat_behaviors/kaboat_behaviors/behavior_base.py
[waypoint-controller]: https://github.com/pegguiitar/kaboat_ws/blob/b6ca813e89fac66eb4e07145e0f91dde2c6bdb83/src/kaboat_behaviors/kaboat_behaviors/waypoint_controller.py
[los-controller]: https://github.com/pegguiitar/kaboat_ws/blob/b6ca813e89fac66eb4e07145e0f91dde2c6bdb83/src/kaboat_behaviors/kaboat_behaviors/los_controller.py
[motion-controller]: https://github.com/pegguiitar/kaboat_ws/blob/b6ca813e89fac66eb4e07145e0f91dde2c6bdb83/src/kaboat_behaviors/kaboat_behaviors/motion_controller.py
[avoid-fsm]: https://github.com/pegguiitar/kaboat_ws/blob/b6ca813e89fac66eb4e07145e0f91dde2c6bdb83/src/kaboat_behaviors/kaboat_behaviors/avoid_fsm.py
[bspline-planner]: https://github.com/pegguiitar/kaboat_ws/blob/b6ca813e89fac66eb4e07145e0f91dde2c6bdb83/src/kaboat_behaviors/kaboat_behaviors/bspline_planner.py
[mission-approach]: https://github.com/pegguiitar/kaboat_ws/blob/b6ca813e89fac66eb4e07145e0f91dde2c6bdb83/src/kaboat_behaviors/kaboat_behaviors/mission_approach.py
[gate-follower]: https://github.com/pegguiitar/kaboat_ws/blob/b6ca813e89fac66eb4e07145e0f91dde2c6bdb83/src/kaboat_behaviors/kaboat_behaviors/gate_follower.py
[search-circler]: https://github.com/pegguiitar/kaboat_ws/blob/b6ca813e89fac66eb4e07145e0f91dde2c6bdb83/src/kaboat_behaviors/kaboat_behaviors/search_circler.py
[mission-manager]: https://github.com/pegguiitar/kaboat_ws/blob/b6ca813e89fac66eb4e07145e0f91dde2c6bdb83/src/kaboat_mission/kaboat_mission/mission_manager.py
[cmd-mux]: https://github.com/pegguiitar/kaboat_ws/blob/b6ca813e89fac66eb4e07145e0f91dde2c6bdb83/src/kaboat_control/kaboat_control/cmd_mux.py
[thruster-driver]: https://github.com/pegguiitar/kaboat_ws/blob/b6ca813e89fac66eb4e07145e0f91dde2c6bdb83/src/kaboat_hardware/kaboat_hardware/thruster_driver.py
[thruster-backend]: https://github.com/pegguiitar/kaboat_ws/blob/b6ca813e89fac66eb4e07145e0f91dde2c6bdb83/src/kaboat_hardware/kaboat_hardware/thruster_backend.py
[thruster-config]: https://github.com/pegguiitar/kaboat_ws/blob/b6ca813e89fac66eb4e07145e0f91dde2c6bdb83/src/kaboat_hardware/config/thrusters.yaml
[thruster-launch]: https://github.com/pegguiitar/kaboat_ws/blob/b6ca813e89fac66eb4e07145e0f91dde2c6bdb83/src/kaboat_hardware/launch/thrusters.launch.py
[autonomy-launch]: https://github.com/pegguiitar/kaboat_ws/blob/b6ca813e89fac66eb4e07145e0f91dde2c6bdb83/src/kaboat_bringup/launch/autonomy.launch.py
[tank-launch]: https://github.com/pegguiitar/kaboat_ws/blob/b6ca813e89fac66eb4e07145e0f91dde2c6bdb83/src/kaboat_hardware/kaboat_hardware/tank_mission_launch.py
[tank-guard]: https://github.com/pegguiitar/kaboat_ws/blob/b6ca813e89fac66eb4e07145e0f91dde2c6bdb83/src/kaboat_hardware/kaboat_hardware/tank_mission_guard.py
