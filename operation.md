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
| 3 | 젯슨 터미널 2 | `run_gq7_outdoor` | `[GQ7] /ekf/odometry_map(/odom) 원점 = …` 로그 |
| 4 | 젯슨 터미널 3 | `ros2 launch kaboat_hardware onboard_lidar.launch.py port:=/dev/ttyUSB0` | `/scan` (선체 TG-50, 실제 포트 확인) |
| 5 | 젯슨 터미널 4 | `run_motor` | 모터 드라이버 + GPS CSV 기록기 |
| 6 | 젯슨 터미널 5 | `run_points` | 미션 지점 기록기 |
| 7 | 노트북 | `rviz2 -d ~/Projects/KABOAT/kaboat_ws/scripts/remote_monitor.rviz` | Fixed Frame `odom` |
| 8 | 노트북 | 배를 옮겨 가며 `rec 이름` (아래 이름 규칙) | `✓ 이름: x=… y=… 선수각 …` |
| 9 | 젯슨 터미널 6 | `ros2 launch kaboat_bringup autonomy.launch.py` | 자율주행 스택. 로그에 `[기록 지점] gate: 시작 … → 끝 …` |

**웨이포인트 이름 규칙** — `mission_manager`가 `~/kaboat_mission_points.json`에서 이 이름만
자동으로 읽어 `mission_params.yaml` 좌표를 덮어씁니다.

| 이름 | 쓰이는 곳 | 기록할 때 |
|---|---|---|
| `<미션>_entry` | 미션 시작 자세 `[x, y, 선수각]` (`entry_waypoints`, `entry_ready=true`) | 배를 **출발할 방향으로** 세우고 기록 |
| `<미션>_end` | 미션 끝 지점 `[x, y]` (`waypoints`, 전환 반경 5 m 판정) | 위치만 쓰임 |

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

GNSS 20개(약 10초)를 평균해 `~/kaboat_geo_origin.yaml`에 저장합니다. 그 뒤
`run_gq7_outdoor`를 **다시 켜야** GQ7 `/odom` 원점에 반영됩니다(GQ7은 드라이버 시작 때만
원점을 장치에 씀). 원점을 바꾸면 그 전에 기록한 지점 파일은 쓸 수 없습니다.

⚠️ 실내 수조 런치(`run_boat_lidar`)와 `run_gq7_outdoor`를 같이 켜면 `/odom` 발행자가
둘이 됩니다. 하나만 켭니다.

---

## 3. 젯슨 단축어

| 단축어 | 실제 명령 | 설명 |
|---|---|---|
| `cb` / `sb` | `colcon build --symlink-install` / `source install/setup.bash` | 빌드 / 환경 리로드 |
| `run_zenoh` | `python3 scripts/zenoh_monitor_link.py jetson` | 노트북 모니터링 브리지 |
| `run_gq7_outdoor` | `ros2 launch kaboat_hardware real_sensors.launch.py outdoor:=true` | 야외 GQ7. `gq7_outdoor.yaml` + 원점 파일, `/ekf/odometry_map` → `/odom` |
| `run_geo` | `ros2 launch kaboat_hardware geo_origin.launch.py` | GPS 원점 설정, 위도·경도 → x/y (`/geo/*`) |
| `run_points` | `ros2 run kaboat_hardware point_recorder` | 미션 지점 기록기 |
| `rec [이름]` | `python3 scripts/rec_point.py [이름]` | 지금 위치(x, y, 선수각) 기록. 이름 생략 시 `p1`, `p2` … |
| `list_points` | `python3 scripts/rec_point.py --list` | 기록한 지점 보기 |
| `run_motor` | `ros2 launch kaboat_hardware thrusters.launch.py hardware_type:=serial port:=/dev/ttyCH341USB0 allow_port_scan:=false` | 아두이노(CH340) 모터 드라이버 + `gps_mark_logger` |
| `view_pitch` | `python3 scripts/view_roll_pitch.py` | 롤/피치와 스캔 차단 상태 |
| `run_boat_lidar` | `ros2 launch kaboat_hardware indoor_tank.launch.py enable_thrusters:=false enable_onboard_lidar:=true onboard_lidar_port:=/dev/ttyUSB0` | **실내 수조 전용** (야외에서 쓰지 않음) |
| `cal_imu` | `ros2 run kaboat_hardware calibrate_indoor_imu` | 실내 수조 IMU 정지 보정 |
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
| behavior 가 안 움직임 | `/odom` `frame_id`가 `odom`인지, `autonomy.launch.py`의 `*_motion_enabled` |
| 노트북에 토픽이 안 보임 | 브리지 허용 목록([`zenoh_monitor_link.py`](scripts/zenoh_monitor_link.py)), 양쪽 브리지 재시작 |
