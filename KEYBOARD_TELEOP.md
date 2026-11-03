# 노트북 WASD 텔레옵 (실물 젯슨)

노트북의 ROS 도메인 43에서 `/teleop/enable`(Bool)과
`/teleop/cmd_vel`(Twist)을 발행합니다. Zenoh가 두 토픽을 젯슨 도메인
42로 전달하고, 젯슨 `thruster_driver`가 자율주행 `/cmd_vel`과 선택합니다.
노트북이 `/cmd_vel`이나 모터 PWM을 직접 발행하지 않습니다.

## 설치 반영

젯슨과 노트북에 이 변경 코드가 모두 있어야 합니다. 젯슨에서 코드를 갱신한
뒤 아래를 실행하고, 이전 `run_zenoh`와 `run_motor` 프로세스는 종료 후
새로 실행하세요. 기존 실행 프로세스는 변경 코드를 자동으로 읽지 않습니다.

```bash
# 젯슨 터미널
cd ~/Desktop/kaboat_ws/repo
colcon build --symlink-install --packages-select kaboat_hardware
source install/setup.bash
run_zenoh
```

```bash
# 젯슨의 다른 터미널: 자율주행 스택과 별도로 모터 드라이버 실행
run_motor
```

노트북 브리지는 변경된 허용 목록을 적용하도록 재시작합니다. 이후
노트북 그래픽 터미널에서 조종 창을 엽니다. 노트북 호스트의 Jazzy
노드에서 발행한 명령을 젯슨의 Humble 노드로 전달합니다.

```zsh
cd ~/Projects/KABOAT/kaboat_ws
stop_kaboat
set_kaboat
python3 scripts/teleop_laptop.py
```

젯슨 Zenoh 브리지도 새 코드를 사용하도록 재시작해야 합니다. 젯슨이
`/teleop/status`를 보내기 전에는 조종 창이 움직이는 명령을 차단합니다.

## 조작

조종 창을 클릭해 포커스를 준 상태에서 키를 누릅니다.

| 입력 | 동작 |
|---|---|
| `W` / `S` | 전진 / 후진 |
| `A` / `D` | 제자리 좌회전 / 우회전 |
| `W+A` / `W+D` | 전진하며 좌회전 / 우회전 |
| `S+A` / `S+D` | 후진하며 좌회전 / 우회전 |
| `Space` | 즉시 중립 명령 |
| `Q` 또는 창 닫기 | 노트북 조종 종료, 젯슨 텔레옵 모드 유지하며 중립 |
| `R` | 노트북 조종 종료 후 자율주행 명령으로 명시적 복귀 |

키를 놓으면 목표 명령이 0으로 부드럽게 내려갑니다. 창이 포커스를 잃거나
`Space`를 누르면 즉시 0을 발행합니다. 노트북 통신이 끊기면 젯슨이
0.5초 후 PWM 중립으로 전환하며, 자율주행으로 자동 복귀하지 않습니다.
물리 RC 모드는 별도로 운영하며 기존 RC 경로는 변경하지 않았습니다.

기본 텔레옵 명령 상한은 전진 `12/118.6`, 후진 `6/118.6`, 회전
`3/118.6` 정규화 비율입니다. 젯슨이 이 상한을 다시 검사합니다.
자율주행 복귀(`R`) 전에 자율주행 노드가 계속 실행 중이며 현재 목표가
유효한지 확인하세요.

젯슨에서 모드만 확인하려면 `ros2 topic echo /teleop/status`를 실행합니다.
`TELEOP`, `TELEOP_TIMEOUT`, `AUTO`, `RC_OVERRIDE`, `ESTOP`,
`BACKEND_ERROR` 중 현재 상태가 표시됩니다.
