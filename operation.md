# 🚤 야외 운용 — 단축어·실행 명령 모음

`competition` 브랜치 기준, 야외(GQ7 GNSS) 운용 때 치는 명령을 한곳에 모았습니다.
통신 구조와 설치는 [COMMANDS_GUIDE.md](COMMANDS_GUIDE.md), 실내 수조 시험은
[TANK_TESTS_GUIDE.md](TANK_TESTS_GUIDE.md)를 봅니다.

- 젯슨 = 선체, ROS 도메인 **42**, bash. 단축어는 `scripts/setup_jetson_bashrc.sh`가 만듭니다.
- 노트북 = 원격 모니터링, 도메인 **43**, zsh. 단축어는 각자의 `~/.zshrc`에 둡니다(4절).
- 둘은 Tailscale 위의 Zenoh 브리지로 이어집니다. 같은 Wi-Fi일 필요는 없습니다.

---

## 1. 처음 한 번

```bash
# 젯슨
cd ~/Desktop/kaboat_ws/repo
git fetch && git checkout competition && git pull
bash scripts/setup_jetson_bashrc.sh && source ~/.bashrc
cb && sb
```

```zsh
# 노트북: 레포 갱신 + 4절의 단축어를 ~/.zshrc 에 넣고
cd ~/Projects/KABOAT/kaboat_ws && git checkout competition && git pull
source ~/.zshrc
```

코드를 받은 뒤에는 젯슨에서 `git pull && cb && sb`, Zenoh 허용 목록이 바뀌었으면
양쪽 브리지를 재시작합니다(젯슨 `run_zenoh`, 노트북 `stop_kaboat` → `set_kaboat`).

---

## 2. 현장 순서

| 순서 | 어디서 | 명령 | 확인 |
|---|---|---|---|
| 1 | 젯슨 터미널 1 | `run_zenoh` | 노트북과 연결 |
| 2 | 노트북 | `set_kaboat` | `🚀 [Zenoh] … 연결 시도 중` |
| 3 | 젯슨 터미널 2 | `run_gq7_outdoor` | `[gq7_local_odom] /odom 원점 = (…) [file …]` 로그 |
| 4 | 젯슨 터미널 3 | `run_lidar` (= `ros2 launch kaboat_hardware onboard_lidar.launch.py port:=<CP2102 고정 경로>`) | `/scan` (선체 TG-50) |
| 5 | 젯슨 터미널 4 | `run_motor` | 모터 드라이버 + GPS CSV 기록기 |
| 6 | 젯슨 터미널 5 | `run_points` | 미션 지점 기록기 |
| 7 | 노트북 | `view_lidar` (= `rviz2 -d …/scripts/remote_monitor.rviz`) | Fixed Frame `odom`, 장애물 지도·`/scan`·배 위치 |
| 8 | 노트북 | 배를 옮겨 가며 `rec 이름` (아래 이름 규칙) | `✓ 이름: x=… y=… 선수각 …` |
| 9 | 젯슨 터미널 6 | `ros2 launch kaboat_bringup autonomy.launch.py` | 자율주행 스택. 로그에 `[기록 지점] gate: 시작 … → 끝 …` |

**웨이포인트 이름 규칙** — `mission_manager`가 `~/kaboat_mission_points.json`에서 이 이름만
자동으로 읽어 `mission_params.yaml` 좌표를 덮어씁니다.

| 이름 | 쓰이는 곳 | 기록할 때 |
|---|---|---|
| `<미션>_entry` | 미션 시작 자세 `[x, y, 선수각]` (`entry_waypoints`, `entry_ready=true`) | 배를 **출발할 방향으로** 세우고 기록 |
| `<미션>_end` | 미션 끝 지점 `[x, y]` (`waypoints`, 전환 반경 **2 m** 판정) | 위치만 쓰임. 미션이 실제로 끝나는 자리에 (도킹은 정박 위치, 게이트는 마지막 게이트 2~3 m 뒤) |

`<미션>` = `gate`, `station`, `dock`, `search`, `avoid`. 예) `rec gate_entry` → `rec gate_end`.

- 한 미션은 `_entry`와 `_end`를 **둘 다** 기록합니다. 하나만 있으면 `mission_manager`가 시작을 거부합니다.
- 기록이 없는 미션은 `mission_params.yaml` 값을 씁니다. 지금은 `entry_ready: false`라서 그 미션에 이르면 멈춥니다.
- 다른 이름(`p1`, `start` 등)은 저장만 되고 미션에는 안 쓰입니다. `rec` 결과에 표시됩니다.
- 기록 당시 원점이 지금 `~/kaboat_geo_origin.yaml`과 다르면 `mission_manager`가 `시작 거부: … 원점이 다름`으로 종료합니다.

**원점을 처음 잡는 날** (경기장마다 한 번) 3번 전에:

```bash
run_geo                                                      # 젯슨 터미널 A
ros2 service call /geo/set_origin std_srvs/srv/Trigger '{}'  # 젯슨 터미널 B, 배를 원점에 멈춘 채
```

GNSS 20개(약 10초)를 평균해 `~/kaboat_geo_origin.yaml`에 저장합니다. `run_gq7_outdoor`가
켜져 있으면 `gq7_local_odom`이 **1초 안에 새 원점을 읽어** `/odom`에 반영합니다. GQ7을 다시 켤
필요가 없습니다(다시 켜면 필터가 다시 수렴하느라 몇 분 걸림). 원점을 바꾸면 그 전에 기록한
지점 파일은 쓸 수 없습니다.

`/odom`은 GQ7 드라이버의 원점 기능(`/ekf/odometry_map`)이 아니라 `gq7_local_odom`이
`/ekf/odometry_earth`를 원점 파일 기준으로 바꿔 만듭니다. 젯슨 드라이버의 원점 기능은 원점을
맞게 받고도 수백 km 떨어진 값을 냈습니다(2026-11-11).

**`/odom` 위치 출처** (`run_gq7_outdoor odom_position_source:=…`, RTK 는 `run_gq7_rtk`)

| 값 | 위치 | 선수각·회전속도 | 선속도 | 비고 |
|---|---|---|---|---|
| `filter` (기본) | GQ7 필터 (25 Hz, 매끄러움) | GQ7 필터 | GQ7 필터 | RTK Fixed 와 함께 쓰면 멈춘 배에서 5 cm 안. 켠 직후 1~2분은 수렴하며 조금 흔들림 |
| `gnss` | GNSS1 원시 `/gps/fix` (2 Hz), 안테나 좌현 0.4 m 보정 | GQ7 필터 (듀얼 안테나) | GNSS 속도 | 필터가 흔들릴 때 대안. GNSS 1초 끊기면 `/odom` 멈춤. 위치가 0.5초마다 계단처럼 바뀜 |

필터가 GNSS 보정을 실제로 쓰는지는 `ros2 topic echo /mip/ekf/aiding_measurement_summary`
(보정원별 `used`, `residual_high`, `sample_time_warning`)로 확인합니다.

**RTK 보정 (선택)** — 위치 정확도 ±1~2 m(SBAS) → 수 cm. 젯슨 인터넷과 NTRIP 계정이 필요합니다.

```bash
sudo apt install ros-humble-ntrip-client          # 젯슨에 한 번
ls -l /dev/microstrain_aux                        # GQ7 AUX 포트(RTCM 입력)가 보이는지
ros2 launch kaboat_hardware real_sensors.launch.py outdoor:=true rtk:=true \
    ntrip_username:=<아이디> ntrip_password:=<비밀번호>
```

- 기본 캐스터 `ntrip.ndgps.go.kr:2101`(국립해양측위정보원), 기준국 `SEOI_RTCM32`(대회장에서 약 47 km).
  바꾸려면 `ntrip_mountpoint:=YNDO_RTCM32` 등. 계정은 서비스에서 발급받은 것을 씁니다(레포에 넣지 않음).
- 확인: `ros2 topic echo /ekf/status --once --field gnss_state` 가 `RTK Float` → `RTK Fixed`,
  `ros2 topic echo /gps/fix --once --field status` 의 `status: 2`. 기준국이 멀어 Fixed 까지 오래 걸리거나
  Float 에 머물 수 있습니다.

⚠️ 실내 수조 런치(`run_boat_lidar`)와 `run_gq7_outdoor`를 같이 켜면 `/odom` 발행자가
둘이 됩니다. 하나만 켭니다.

---

## 3. 젯슨 단축어

| 단축어 | 실제 명령 | 설명 |
|---|---|---|
| `cb` / `sb` | `colcon build --symlink-install` / `source install/setup.bash` | 빌드 / 환경 리로드 |
| `run_zenoh` | `python3 scripts/zenoh_monitor_link.py jetson` | 노트북 모니터링 브리지 |
| `run_gq7_outdoor` | `ros2 launch kaboat_hardware real_sensors.launch.py outdoor:=true` | 야외 GQ7. `gq7_outdoor.yaml` + `gq7_local_odom`(`/ekf/odometry_earth` → 원점 파일 기준 `/odom`) |
| `run_gq7_rtk` | `real_sensors.launch.py outdoor:=true rtk:=true ntrip_username:=… ntrip_password:=…` | 야외 GQ7 + RTK 보정 + 필터 위치. 계정은 `~/.ntrip_account` |
| `run_lidar` | `ros2 launch kaboat_hardware onboard_lidar.launch.py port:=$(_tg50_port)` | 야외 선체 TG-50 (`/scan`, `/scan/level`). GQ7 먼저. 포트는 `/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0`(없으면 `/dev/ttyUSB0`) |
| `run_camera` | `ros2 launch kaboat_bringup d455.launch.py` | D455 카메라 (컬러 640×480 + 정렬 뎁스, 15 fps). 카메라 층·도킹용. USB 3 포트 |
| `run_geo` | `ros2 launch kaboat_hardware geo_origin.launch.py` | GPS 원점 설정, 위도·경도 → x/y (`/geo/*`) |
| `run_points` | `ros2 run kaboat_hardware point_recorder` | 미션 지점 기록기 |
| `rec [이름]` | `python3 scripts/rec_point.py [이름]` | 지금 위치(x, y, 선수각) 기록. 이름 생략 시 `p1`, `p2` … |
| `list_points` | `python3 scripts/rec_point.py --list` | 기록한 지점 보기 |
| `goto 지점` / `goto x y [각도]` | `ros2 launch kaboat_hardware outdoor_waypoint.launch.py point:=지점` (또는 `x:= y:= angle:=`) | 야외 웨이포인트 1개 주행, **실행 즉시 출발**(추력 40%). 5절 참고 |
| `run_motor` | `ros2 launch kaboat_hardware thrusters.launch.py hardware_type:=serial port:=/dev/ttyCH341USB0 allow_port_scan:=false` | 아두이노(CH340) 모터 드라이버 + `gps_mark_logger` |
| `view_pitch` | `python3 scripts/view_roll_pitch.py` | 롤/피치와 스캔 차단 상태 |
| `view_position` | `python3 scripts/view_position.py` | `/odom` x·y·선수각(동쪽 0°)·나침반 방위·속도·GNSS 상태(RTK Fixed 등) 실시간 한 줄 |
| `run_boat_lidar` | `ros2 launch kaboat_hardware indoor_tank.launch.py enable_thrusters:=false enable_onboard_lidar:=true onboard_lidar_port:=$(_tg50_port)` | **실내 수조 전용** (야외에서 쓰지 않음) |
| `cal_imu` | `ros2 run kaboat_hardware calibrate_indoor_imu` | 실내 수조 IMU 정지 보정 |
| `test_gate_outdoor` | `ros2 launch kaboat_hardware outdoor_gate_test.launch.py` | 야외 게이트 7쌍 `gate_entry` → `gate_end`, **즉시 출발**. 5절 참고 |
| `test_avoid_outdoor` | `ros2 launch kaboat_hardware outdoor_avoid_test.launch.py` | 야외 회피 시험 `avoid_entry` → `avoid_end`, **조건이 맞으면 즉시 출발**. 5절 참고 |
| `test_straight` / `test_circle` / `test_bspline` / `test_station` / `test_waypoint` / `test_los` / `test_avoid` | 각 `*_test.launch.py … wait_for_start:=false` | 단독 주행 시험, **실행 즉시 출발** ([TANK_TESTS_GUIDE.md](TANK_TESTS_GUIDE.md)) |

---

## 4. 노트북 단축어

레포가 아니라 각자의 `~/.zshrc`에 있습니다. 새 노트북이면 아래를 붙여 넣고 경로·젯슨
IP만 맞춥니다(지금 설정은 `/home/jiwoo/Projects/KABOAT/kaboat_ws`, 젯슨 `100.108.192.55`).

| 단축어 | 설명 |
|---|---|
| `set_kaboat [젯슨IP]` | Jazzy 로드, 도메인 43, Zenoh 브리지 백그라운드 시작 (로그 `/tmp/zenoh_bridge.log`) |
| `stop_kaboat` | Zenoh 브리지 종료 |
| `set_default` | Jazzy 로드, 도메인 0으로 복귀 |
| `rec [이름]` / `rec --list` | 젯슨에 지금 위치 기록 요청 / 기록한 지점 보기 (`set_kaboat` 후) |
| `teleop` | 노트북 키보드 조종 창 (`scripts/teleop_laptop.py`, 도메인 43). `set_kaboat` 후, 젯슨 `run_motor` 필요 |
| `view_position` | `/odom` x·y·선수각·나침반 방위·속도 실시간 한 줄 (`set_kaboat` 후) |
| `view_lidar` | 야외 모니터링 RViz (`remote_monitor.rviz`, Fixed Frame `odom`: 장애물 지도·스캔·배 위치·경로). `set_kaboat` 후. 스캔·배 위치는 `scripts/flat_scan_view.py`가 바닥면에 납작하게 다시 그린 `/scan/flat`·`/odom/flat` (GNSS 고도·롤·피치 때문에 원본 `/scan`이 허공에 비스듬히 뜸). 원본·TF는 RViz 패널에서 체크하면 보임 |
| `rviz_tank [시험]` / `clear_traj` | 수조 추적 RViz / 궤적 지우기 |
| `rviz_remote` | 선체 `/scan`을 라이다 좌표계로 표시 |
| `view_pitch` | 롤/피치 모니터 |
| `kaboat_sim` / `kaboat_lidar` | 시뮬레이터 / 외벽 라이다 컨테이너 셸 |

```zsh
KABOAT_WS=/home/jiwoo/Projects/KABOAT/kaboat_ws    # 레포 경로

set_kaboat() {
    [ -f /opt/ros/jazzy/setup.zsh ] && source /opt/ros/jazzy/setup.zsh
    export ROS_DOMAIN_ID=43 ROS_LOCALHOST_ONLY=0
    local jetson_ip="${1:-100.108.192.55}"
    if pgrep -f "zenoh_monitor_link.py|zenoh-bridge-ros2dds" > /dev/null 2>&1; then
        echo "📡 [Zenoh] 브리지가 이미 동작 중"
    else
        nohup python3 $KABOAT_WS/scripts/zenoh_monitor_link.py laptop --jetson-ip "$jetson_ip" \
            > /tmp/zenoh_bridge.log 2>&1 &
        echo "🚀 [Zenoh] 브리지 시작 → 젯슨($jetson_ip)"
    fi
}
stop_kaboat() { pkill -f zenoh_monitor_link.py; pkill -f zenoh-bridge-ros2dds; }
set_default() { source /opt/ros/jazzy/setup.zsh; export ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0; }

alias rec="python3 $KABOAT_WS/scripts/rec_point.py"
view_lidar() {    # 스캔·배 위치를 바닥면에 납작하게 그리는 flat_scan_view.py 를 같이 켜고 끔
    local flat_pid=
    if ! pgrep -f "python3 .*scripts/flat_scan_view[.]py" > /dev/null; then
        ROS_DOMAIN_ID=43 python3 $KABOAT_WS/scripts/flat_scan_view.py > /tmp/flat_scan_view.log 2>&1 &!
        flat_pid=$!
    fi
    { ROS_DOMAIN_ID=43 rviz2 -d $KABOAT_WS/scripts/remote_monitor.rviz "$@"; } \
        always { [ -n "$flat_pid" ] && kill $flat_pid 2>/dev/null; }
}
teleop() { ROS_DOMAIN_ID=43 python3 $KABOAT_WS/scripts/teleop_laptop.py "$@"; }
view_position() { ROS_DOMAIN_ID=43 python3 $KABOAT_WS/scripts/view_position.py "$@"; }
alias rviz_tank="$KABOAT_WS/scripts/rviz_tank.sh"
alias rviz_remote="$KABOAT_WS/scripts/rviz_remote.sh"
alias clear_traj="$KABOAT_WS/scripts/clear_tank_trajectory.sh"
alias view_pitch="python3 $KABOAT_WS/scripts/view_roll_pitch.py"
alias kaboat_sim='docker exec -it -e DISPLAY="$DISPLAY" kaboat_sim_container bash'
alias kaboat_lidar='docker exec -it -e DISPLAY="$DISPLAY" kaboat_lidar bash'
```

---

## 5. 직접 치는 명령

**안전** (젯슨 도메인 42 — 브리지를 지나지 않으므로 젯슨 터미널에서)

```bash
ros2 topic pub --once /emergency_stop std_msgs/msg/Bool '{data: true}'    # 비상 정지
ros2 topic pub --once /emergency_stop std_msgs/msg/Bool '{data: false}'   # 원인 해결 후 해제
ros2 topic pub --once /start_mission  std_msgs/msg/Bool '{data: true}'    # 출발 신호 (wait_for_start:=true 시험)
```

**자율주행 스택**

```bash
ros2 launch kaboat_bringup autonomy.launch.py                          # 기본: gate·approach 는 미리보기만
ros2 launch kaboat_bringup autonomy.launch.py \
    gate_motion_enabled:=true approach_motion_enabled:=true             # 실제로 움직임
```

`dock_weights:=<.pt 절대경로>`, `dock_device:=0`(젯슨 CUDA), `mission_params_file:=<yaml>`로 바꿉니다.
기록 지점 파일은 `mission_points_file:=<json>`으로 바꾸고, `mission_points_file:=none`이면 읽지 않습니다.

**웨이포인트 1개 시험** (`goto`, 미션 스택 없이 `waypoint()` 제어기만)

준비: `run_gq7_rtk`(RTK Fixed), `run_motor`, ESP 컴퓨터 모드. 다른 `/cmd_vel` 노드
(`test_*`, 회피 미리보기 `real_obstacle_avoid_test`, `autonomy.launch.py`)는 끕니다.

```bash
goto gate_entry motion_enabled:=false   # 미리보기: 로그의 출력 [전진 %, 회전 %] 방향이 맞는지
goto gate_entry                         # 기록 지점으로 가서 기록 선수각까지 맞춤
goto gate_end heading:=false            # 위치만
goto 10.0 -5.0 90                       # /odom 좌표 (x 동쪽, y 북쪽, 선수각 동쪽 0° 반시계 +)
goto gate_entry thrust_ratio:=0.5 position_tolerance:=1.5   # 기본 0.4 보다 세게
```

- 로그 1초마다 `[단계] 현재: (x, y) | 목표거리 | 선수각(오차) | 출력`. 도착하면 상태 `REACHED`, 이후 정지 명령만 냅니다. Ctrl+C로 끕니다.
- 상태는 `ros2 topic echo /waypoint/status`, 목표는 `/waypoint/goal`(RViz).
- 노트북 `teleop`으로 수동 모드를 켜면 언제든 넘겨받습니다(수동을 끄면 아직 도착 전이면 다시 이어서 감). 비상 정지는 아래 안전 명령.
- 목표 거리 제한은 기본으로 없습니다. 좌표 실수가 걱정되면 `max_goal_distance:=50`처럼 걸면 그보다 먼 목표에서 `GOAL_TOO_FAR`로 멈춥니다.
- 목표까지 **직선으로** 갑니다(장애물 회피 없음). 멀리 보낼 때는 지켜보다가 `teleop`으로 넘겨받으세요.
- 기록 지점은 기록 당시 원점과 지금 원점이 다르면 시작을 거부합니다(`mission_manager`와 같은 규칙).

**야외 회피 시험** (`test_avoid_outdoor`, 수조 `test_avoid`와 같은 DRI·B-spline·FSM)

준비는 `goto`와 같고 `run_lidar`도 필요합니다. 지도(`occupancy_grid`)는 이 런치가 켭니다 —
따로 켜 둔 것이 있으면 끄거나 `start_occupancy_grid:=false`.

```bash
goto avoid_entry                                 # 1) 시작점에 기록 선수각으로 세움 → REACHED 후 Ctrl+C
test_avoid_outdoor drive_enabled:=false          # 2) 미리보기: RViz /real_avoid_test/path 에 경로
test_avoid_outdoor                               # 3) 실제 주행: 출발 조건이 맞으면 바로 출발
test_avoid_outdoor start_point:=gate_entry goal_point:=gate_end   # 다른 지점 쌍
```

- 출발 조건: 시작점에서 3 m(`start_tolerance`) 안, 선수가 목표 방향에서 45° 안. 안 맞으면
  `ros2 topic echo /real_avoid_test/status`에 `start_blocked=…` 이유가 나옵니다.
- 목표 2 m(`goal_tolerance`) 안이면 `FINISHED`로 정지. 경로에서 1.5 m(`max_cross_track_error`) 넘게
  벗어나면 정지, 경로가 5초 넘게 없으면 후진 탈출(`allow_escape:=false`면 정지).
- 계획은 지도(15 m) 안에서 목표 쪽으로 짧게 이어 가므로 목표가 멀어도 됩니다.
- 기록 당시 원점과 지금 원점이 다르면 시작을 거부합니다.

**야외 게이트 시험** (`test_gate_outdoor`, 자율주행과 같은 `gate_follower` + `cmd_mux`, 게이트 미션 하나만)

`mission_manager`가 기록 파일 전체가 아니라 `gate_entry`·`gate_end` 두 지점만 받으므로 다른 미션
지점(`station_end` 등)이 없어도 됩니다. 준비는 `goto`와 같고 `run_lidar`도 필요합니다. 지도는 이 런치가 켭니다.

```bash
goto gate_entry                              # 1) 첫 게이트 앞 통로 중심에 세움 → REACHED 후 Ctrl+C
test_gate_outdoor motion_enabled:=false      # 2) 미리보기: 노트북 RViz 에 게이트 쌍(/gate/pairs)·경로(/gate/path)
test_gate_outdoor                            # 3) 실제 주행 (7쌍 통과 → gate_end 2 m 안 → 정지)
test_gate_outdoor approach:=true             # gate_entry 접근(직선, 회피 없음)부터
```

- 기록: `gate_entry` = 첫 게이트 앞 통로 중심(진행 방향으로 세워서), `gate_end` = 마지막 게이트 뒤
  **2~3 m** 통로 중심. 모든 쌍을 지난 뒤 `gate_end`가 4 m 안이어야 그리로 가고 2 m 안이면 완료.
- 추력: 게이트 통과(LOS) 직진 상한 **0.4**(약 1675 µs, `gate.los_thrust_ratio`)·회전 0.31.
- `approach:=true` → 게이트 시작 조건: `gate_entry` **1 m 안 + 선수각 ±15° 안**을 0.5초 유지(회전 속도
  조건 없음). 모든 미션의 시작 자세 접근(`mission_approach`)이 같은 조건. 미리보기
  (`motion_enabled:=false`)에서는 도착 신호를 안 내서 `gate`로 넘어가지 않습니다.
- 쌍 고르기: 두 부표를 잇는 선이 진행 방향(직전 게이트 방향, 첫 게이트는 배→`gate_end`)의 수직에서
  35° 안(`gate.max_pair_skew_deg`)이면 쌍. 통로가 휘어도 됨. 폭 1.5~7 m.
- 상태: `ros2 topic echo /gate/status` (`FOLLOW_GATE_3/7`, `GATE_3/7_PASSED`, `WAIT_GATE_PAIR`, `COMPLETE` …).

**카메라 층 — 라이다가 놓치는 낮은 색 장애물** (주황 붐 등, `occupancy_grid` 의 `cam_enabled`, 기본 끔)

라이다 수평 한 면은 수면에 누운 주황 붐 위로 지나갑니다. 카메라 층은 컬러 영상에서
주황·빨강 픽셀을 고르고 정렬 뎁스로 위치를 재(0.6~5 m), IMU 롤·피치로 수평 보정한 뒤
수면 위(반사상 제외) 점만 지도에 따로 쌓습니다. 라이다가 지우지 못하고, 카메라가 그
자리를 다시 보는데 없을 때만 사라집니다. 같은 칸이 4프레임(약 0.8초) 이어져야 회피
대상(95)이 됩니다. 흰색은 수면 반짝임 오탐이 많아 기본 끔(`cam_white:=true`).

```bash
run_camera                                                         # 젯슨: D455 (= ros2 launch kaboat_bringup d455.launch.py)
ros2 run kaboat_perception occupancy_grid --ros-args \
    -p cam_enabled:=true                                           # 카메라 높이 기본 0.85 m (실측)
test_avoid_outdoor cam_enabled:=true                              # 회피 시험에서 같이 켜기
```

- `cam_height` = 배가 물에 뜬 상태에서 흘수선부터 카메라 렌즈 중심까지 [m], 실측 **0.85**가 기본
  (바꾸면 `-p cam_height:=0.9`처럼 소수점 포함). 숙여 달았으면
  `cam_mount_pitch_deg`(+ = 아래). 선체 중심 기준 위치(뒤 0.33 m)는 도킹 실측값이 기본.
- 노트북 `view_lidar`의 주황 점(`/occupancy_grid/cam_points`)이 카메라가 고른 점입니다. 붐 위에
  모이는지, 수면에 점이 흩어지지 않는지 봅니다.
- 색 기준·높이 하한은 `cam_orange_*`, `cam_white_*`, `cam_range_*` 파라미터.
- 튜닝용 녹화 (영상이 커서 30초 안팎으로): `ros2 bag record -o boom_$(date +%H%M%S)
  /camera/color/image_raw /camera/depth/image_raw /camera/camera_info /imu/data /odom /scan`

**원점·위치 확인**

```bash
ros2 topic echo /geo/status                     # WAIT_ORIGIN / WAIT_FIX / OK: … boat x= y=
ros2 topic echo /odom --field pose.pose.position
ros2 topic echo /gps/fix --once
cat ~/kaboat_geo_origin.yaml
```

**미션 지점** (`rec` 없이 보낼 때 — 결과는 `/mission_points/result`)

```bash
ros2 topic pub --once /mission_points/request std_msgs/msg/String "{data: gate_entry}"
cat ~/kaboat_mission_points.json
```

---

## 6. 파일 위치 (젯슨)

| 파일 | 내용 | 만드는 것 |
|---|---|---|
| `~/kaboat_geo_origin.yaml` | 고정 원점 위도·경도·고도 | `/geo/set_origin` |
| `~/kaboat_mission_points.json` | 미션 지점 x, y, 선수각 (+원점, 위도·경도) | `rec` |
| `~/kaboat_gps_logs/gps_log_*.csv` | 노트북 Space로 찍은 GPS 위치 (위도·경도 + 원점 기준 x·y) | `gps_mark_logger` (`run_motor`) |
| `src/kaboat_hardware/config/gq7_outdoor.yaml` | GQ7 야외 설정 (안테나 오프셋 ±0.4 m, `map_frame_id: odom`) | — |

---

## 7. 막힐 때

| 증상 | 확인 |
|---|---|
| `rec` → `✗ 받는 쪽이 없음` | 젯슨 `run_points`·`run_zenoh`, 노트북 `set_kaboat`. 양쪽 스크립트가 최신인지 |
| `rec` → `/odom 을 아직 못 받음` / `안 옴` | `run_gq7_outdoor` 켜짐, GNSS Full Nav 수렴 (`ros2 topic echo /ekf/status`) |
| `rec` → `원점이 다름` | 원점을 새로 잡았음. 지점 파일을 옮기고(`mv ~/kaboat_mission_points.json …`) 다시 기록 |
| `/odom` 원점이 켤 때마다 바뀜 | `~/kaboat_geo_origin.yaml` 없음 → 2절 원점 잡기 |
| `mission_manager` → `시작 거부: … 원점이 다름` | 지점을 기록한 뒤 원점을 다시 잡았음. 지점을 다시 기록하거나 원점 파일을 되돌림 |
| `mission_manager` → `시작 거부: gate_end 이 없음` | 그 미션의 `_entry`/`_end` 중 하나만 기록됨. 나머지를 `rec`로 기록 |
| `goto` → `CMD_VEL_PUBLISHER_CONFLICT` | 다른 `/cmd_vel` 노드가 켜져 있음 (`ros2 topic info /cmd_vel -v`). 끄고 다시 |
| `goto` → `ODOM_PUBLISHER_COUNT_NOT_ONE` / `ODOM_TIMEOUT` | `/odom` 발행자가 0개 또는 2개 이상 (`ros2 topic info /odom -v`), GQ7 켜짐 확인 |
| `goto` → `GOAL_TOO_FAR` | `max_goal_distance:=…`를 직접 준 경우만 — 목표 좌표·원점 확인 또는 값 올리기 |
| behavior 가 안 움직임 | `/odom` `frame_id`가 `odom`인지, `autonomy.launch.py`의 `*_motion_enabled` |
| 노트북에 토픽이 안 보임 | 브리지 허용 목록([`zenoh_monitor_link.py`](scripts/zenoh_monitor_link.py)), 양쪽 브리지 재시작 |
