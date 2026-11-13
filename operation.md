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
| 9 | 젯슨 터미널 6 | `run_map` | 장애물 지도 (라이다만) — 하나만. 로그 `가상 벽 2줄 …` |
| 10 | 젯슨 터미널 7 | `ros2 launch kaboat_bringup autonomy.launch.py` | 자율주행 스택 (지도는 9번 것을 씀). 로그에 `[기록 지점] gate: 시작 … → 끝 …` |

**한 번에 켜기:** 3~6·9번(GQ7은 RTK)과 `run_camera`는 `run_sensors_motors` 하나로 켤 수 있습니다 — tmux 세션
`boat`에 창 하나씩(0 gq7, 1 lidar, 2 motor, 3 points, 4 map, 5 camera), 라이다는 `/imu/data`가 나온 뒤.
`Ctrl+B` 숫자로 창 이동, `Ctrl+B d`로 빠져나와도 계속 돌고 다시 `run_sensors_motors`로 붙습니다. 한 창만
Ctrl+C로 끄고 ↑ Enter로 다시 켤 수 있습니다. 전부 끄기 `stop_sensors_motors`. `run_zenoh`는 따로 켭니다.
tmux가 없으면 `sudo apt install tmux`.

**웨이포인트 이름 규칙** — `mission_manager`가 `~/kaboat_mission_points.json`에서 이 이름만
자동으로 읽어 `mission_params.yaml` 좌표를 덮어씁니다.

| 이름 | 쓰이는 곳 | 기록할 때 |
|---|---|---|
| `<미션>_entry` | 미션 시작 자세 `[x, y, 선수각]` (`entry_waypoints`, `entry_ready=true`) | 배를 **출발할 방향으로** 세우고 기록 |
| `<미션>_end` | 미션 끝 지점 `[x, y]` (`waypoints`, 전환 반경 **2 m** 판정) | 위치만 쓰임. 미션이 실제로 끝나는 자리에 (도킹은 정박 위치, 게이트는 마지막 게이트 2~3 m 뒤, 탐색은 선회 뒤 빠져나갈 출구) |
| `search_center` (선택) | 탐색 영역 중심 `[x, y]` — 목표 부표를 이 점 **12 m 안**에서만 고름 | 탐색 부표 셋(빨강·초록·검정) 가운데쯤. 부표가 바람에 흔들려도 대략이면 됨. 안 찍으면 `search_entry` 선수 방향 7 m 앞 |

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
- **보정이 GQ7 에 실제로 들어가는지 (2026-11-13):** 보정은 `/rtcm` → `rtcm_guard` → `/rtcm/gq7` → GQ7 AUX 로 갑니다.
  `ros2 topic echo /gnss/rtk_status` 가 `RTK Fixed | 보정 중계 중` 이면 정상입니다.
  - 켤 때 AUX 포트가 없거나 다른 프로세스(남은 GQ7 드라이버 등)가 잡고 있으면 `[GQ7] RTK 를 못 켬 — …` 로
    멈춥니다. GQ7 USB 를 다시 꽂거나 그 프로세스를 끄고 다시 켭니다(`gq7_aux_check:=false` 로 끌 수 있음).
  - 주행 중 드라이버가 `Failed to write RTCM to aux port` 를 쏟아내면 `rtcm_guard` 가 중계를 멈춰 로그 폭주를
    끊고, `GQ7 AUX 포트에 RTK 보정을 못 씀 — …` 을 30초마다 한 번 알립니다(그동안 **RTK 아님**). 10초마다 하나씩
    다시 넣어 보고 되면 자동으로 이어 갑니다. 안 되면 GQ7 USB 를 다시 꽂고 `run_gq7_rtk` 재시작.
    2026-11-13 토너먼트 시험은 11:36 부터 이 상태(RTK Fixed → SBAS)로 7번을 다 했는데 아무도 몰랐습니다.
  - 보정이 10초 넘게 안 오거나, 들어가는데 2분 넘게 RTK 가 아니면 그것도 경고합니다.

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
| `run_map` | `ros2 run kaboat_perception occupancy_grid --ros-args` | 장애물 지도(라이다 + 가상 벽, **카메라 안 씀**). **하나만** 켜 두고 `test_*`·`autonomy.launch.py`가 같이 씀 (`ros2 topic info /occupancy_grid` 발행자 1). 뒤에 `-p 이름:=값` 붙임 — 카메라 층은 `run_map -p cam_enabled:=true` |
| `run_camera` | `ros2 launch kaboat_bringup d455.launch.py` | D455 카메라 (컬러 640×480 + 정렬 뎁스, 15 fps). 도킹용 (지도 카메라 층은 기본 끔). USB 3 포트 |
| `run_geo` | `ros2 launch kaboat_hardware geo_origin.launch.py` | GPS 원점 설정, 위도·경도 → x/y (`/geo/*`) |
| `run_points` | `ros2 run kaboat_hardware point_recorder` | 미션 지점 기록기 |
| `run_sensors_motors` | `bash scripts/run_sensors_motors.sh` | `run_gq7_rtk`·`run_lidar`(`/imu/data` 뒤)·`run_motor`·`run_points`·`run_map`·`run_camera`를 tmux 세션 `boat`에 창 하나씩 (0~5, `Ctrl+B` 숫자로 이동). 이미 켜져 있으면 붙기만. `run_zenoh`는 따로. tmux 필요 (`sudo apt install tmux`) |
| `stop_sensors_motors` | `bash scripts/run_sensors_motors.sh stop` | 위 세션 모든 창에 Ctrl+C(카메라 → GQ7 순) → 다 꺼지면 세션 닫기 |
| `rec [이름]` | `python3 scripts/rec_point.py [이름]` | 지금 위치(x, y, 선수각) 기록. 이름 생략 시 `p1`, `p2` … |
| `list_points` | `python3 scripts/rec_point.py --list` | 기록한 지점 보기 |
| `rec_bag [이름]` | `bash scripts/record_run.sh [이름]` | 주행 센서 rosbag 녹화 → `~/kaboat_bags/이름_날짜_시각/`. **센서를 다 켠 뒤** 실행, **Ctrl+C로 종료**(창을 그냥 닫지 말 것). 카메라는 압축 토픽(약 4~5 MB/s, 20분 ≈ 6 GB), 압축 토픽이 없으면 원본 + zstd(20분 ≈ 15~18 GB). 남은 용량 10 GB 미만이면 시작 안 함. 2 GB마다 파일 분할. **카메라가 필요 없는 시험(회피·토너먼트·게이트)은 `rec_bag --no-camera [이름]`** — 영상·압축 없이(원본 영상 zstd 는 젯슨이 못 따라가 메시지를 버림, 2026-11-13). 자세한 사용법 [ROSBAG_GUIDE.md](ROSBAG_GUIDE.md) |
| `rec_rgb [이름]` | `bash scripts/record_run.sh --rgb [이름]` | 카메라 컬러(RGB) 영상만 녹화 → `~/kaboat_bags/이름_날짜_시각/` (이름 생략 시 `rgb`). `run_camera`만 켜면 됨, **Ctrl+C로 종료**. 압축 토픽(JPEG) 약 1~1.5 MB/s, 없으면 원본 + zstd. 영상은 `bag_to_mp4 <폴더>` |
| `bag_to_mp4 <bag 폴더>` | `python3 scripts/bag_to_mp4.py <bag 폴더>` | bag 의 카메라 영상 → mp4 (bag 폴더 안 `<폴더 이름>.mp4`, 동영상 플레이어로 열림). `rec_bag` 폴더(압축 토픽)·`ros2 bag record` 폴더 둘 다. 토픽 기본 `/camera/color/image_raw`(또는 그 `/compressed`), 바꾸려면 `--topic /dock/debug_image`. OpenCV 가 mp4 를 못 쓰면 `.avi` |
| `goto 지점` / `goto x y [각도]` | `ros2 launch kaboat_hardware outdoor_waypoint.launch.py point:=지점` (또는 `x:= y:= angle:=`) | 야외 웨이포인트 1개 주행, **실행 즉시 출발**(추력 40%). 5절 참고 |
| `run_motor` | `ros2 launch kaboat_hardware thrusters.launch.py hardware_type:=serial port:=/dev/ttyCH341USB0 allow_port_scan:=false` | 아두이노(CH340) 모터 드라이버 + `gps_mark_logger` |
| `view_pitch` | `python3 scripts/view_roll_pitch.py` | 롤/피치와 스캔 차단 상태 |
| `view_position` | `python3 scripts/view_position.py` | `/odom` x·y·선수각(동쪽 0°)·나침반 방위·속도·GNSS 상태(RTK Fixed 등) 실시간 한 줄 |
| `run_boat_lidar` | `ros2 launch kaboat_hardware indoor_tank.launch.py enable_thrusters:=false enable_onboard_lidar:=true onboard_lidar_port:=$(_tg50_port)` | **실내 수조 전용** (야외에서 쓰지 않음) |
| `cal_imu` | `ros2 run kaboat_hardware calibrate_indoor_imu` | 실내 수조 IMU 정지 보정 |
| `test_gate_outdoor` | `ros2 launch kaboat_hardware outdoor_gate_test.launch.py` | 야외 게이트 7쌍 `gate_entry` → `gate_end`, **즉시 출발**. 5절 참고 |
| `test_dock_outdoor` | `bash scripts/run_dock_mission.sh` (= YOLO CUDA venv + `outdoor_dock_test.launch.py`) | 야외 도킹 `dock_entry` → 목표 슬롯 → 후진 3.25 m, **즉시 시작**. `run_camera` 필요. 5절 참고 |
| `check_yolo` | `python3 scripts/check_yolo_masks.py` | 도킹 YOLO **마스크만** 확인 (뎁스·라이다 무관). `bash scripts/run_dock_yolo.sh`(D455 + YOLO, 모터 없음)를 먼저 켜 둔다. 1초마다 도형별 ✓신뢰도·화면 위치·방위각·최근 검출률, 추론 ms. 마스크 그린 영상을 5초마다 `~/kaboat_yolo_checks/날짜_시각/`에 PNG (`--save-every 0` 끔). Ctrl+C로 요약 |
| `test_avoid_outdoor` | `ros2 launch kaboat_hardware outdoor_avoid_test.launch.py` | 야외 회피 시험 `avoid_entry` → `avoid_end`, **조건이 맞으면 즉시 출발**. 5절 참고 |
| `test_tournament` | `ros2 launch kaboat_hardware outdoor_tournament_test.launch.py` | 토너먼트 회피(종합 미션과 별개 경기장) `tournament_entry` → `tournament_end`. `test_avoid_outdoor`와 같은 노드·조건·인자, **조건이 맞으면 즉시 출발**. 5절 참고 |
| `go_tournament` / `go_avoid` / `go_gate` / `go_dock` / `go_station` / `go_search` | `python3 scripts/go_and_test.py 미션` | **`goto 미션_entry` → 시작 조건을 0.5 s 유지하면 goto 를 끄고 그 미션 시험을 바로 켬** (goto 를 손으로 안 끔). 시험은 각각 `test_tournament`·`test_avoid_outdoor`·`test_gate_outdoor`·`test_dock_outdoor`·`test_mission station`·`test_mission search`(접근 단계 없이 `entry_required:=false`). 기본 조건(각 미션 시작 조건과 같음): 게이트·도킹·스테이션·탐색 **2 m·기록 선수각 ±30°**, 회피·토너먼트 **3 m·`_end` 방향 ±45°**. 옵션 → 시험 인자 순: `go_tournament --dist 1.5 --heading 20 --goto thrust_ratio:=0.3 drive_enabled:=false`. 회피·토너먼트는 넘기는 동안 흐를 여유로 시험 출발 검사를 0.5 m·5° 넓혀 넘김(`start_tolerance`·`start_heading_tolerance_deg` 를 직접 주면 그 값). 넘기는 몇 초(도킹은 YOLO 로딩이라 더 김)는 명령이 없어 배가 조금 흐를 수 있음. Ctrl+C = 켜진 쪽 끄고 끝 |
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
ros2 launch kaboat_bringup autonomy.launch.py                          # 기본: gate·approach·dock 미리보기만
run_autonomy search_target_color:=red dock_target_class:=circle        # 전체 실제 주행 (게이트→…→회피)
test_mission station                                                   # 미션 하나만, 끝나면 done(정지)
test_mission search search_target_color:=red
test_mission dock dock_target_class:=circle
```

- `run_autonomy` = `autonomy.launch.py gate_motion_enabled:=true approach_motion_enabled:=true dock_motion_enabled:=true`.
  뒤에 붙인 인자가 이깁니다.
- `test_mission 미션` = 같은 스택에 `start_state:=미션 single_mission:=true`를 더한 것입니다.
  - 그 미션의 `_entry`로 접근(`mission_approach`)한 뒤 미션을 하고 `done`(정지)합니다.
  - 미리 `goto 미션_entry`로 세워 두면 접근이 바로 끝납니다.
- **이번에 돌 미션의 노드만 띄웁니다** (`only_mission_nodes:=true` 기본). `start_state`부터 끝까지,
  `single_mission:=true`면 그 미션 하나만. 미션 매니저·`cmd_mux`는 항상, `mission_approach`는
  `entry_required:=true`(기본)일 때만 뜹니다. 런치를 켜면 `[autonomy] 미션 … — 띄우는 노드: …` 한 줄로 보입니다.

  | 미션 | 띄우는 노드 |
  |---|---|
  | gate | `gate_follower` |
  | station | `station_keeper` (부표는 전용 지도로 찾음 — 부표 검출 노드 안 씀) |
  | dock | 카메라 TF·`dock_mark_detector`·`dock_lidar_markers`·`dock_marker_goal`·`dock_ctrl_v2` |
  | search | `buoy_lidar_detector`·`search_circler` |
  | avoid | `obstacle_planner` |

  2026-11-13 dock bag: `test_mission station`이 파이썬 노드 12개를 한꺼번에 띄워 배가 첫 명령을 받기까지
  **8 s**(노드 약 7개인 도킹 시험 런치는 **3 s**) 걸렸고 그동안 0.1~0.2 m/s로 밀렸습니다.
  예전처럼 전부 띄우려면 `only_mission_nodes:=false`. 알 수 없는 `start_state`(오타)는 런치가 거부합니다
  (예전엔 미션 매니저가 오류를 내고 gate부터 시작).
- `approach_motion_enabled:=false`면 접근이 끝나지 않아 미션이 시작되지 않습니다(접근 명령은 미리보기만 나감).

`dock_weights:=<.pt 절대경로>`, `dock_device:=0`(젯슨 CUDA), `mission_params_file:=<yaml>`로 바꿉니다.
탐색 목표 부표 색(대회 당일 공지)은 **`search_target_color:=red|green|black`** 로 넣습니다 — 안 넣으면
탐색 미션에서 멈춥니다. 빨강·초록은 우현(시계), 검정은 좌현(반시계)으로 부표 둘레 반경 2 m 를
1.1 바퀴 돕니다. 목표 부표는 **탐색 영역 중심(`search_center`, 없으면 `search_entry` 선수 방향 7 m 앞)
12 m 안**의 목표 색 부표 중 중심에 가장 가까운 것입니다 (주황 장애물 부표도 빨강으로 잡히므로 영역 밖은 안 봄).
`search_end`는 선회를 마치고 빠져나갈 **출구**로만 씁니다 — 2026-11-13 처럼 탐색 부표에서 22 m
떨어진 곳에 찍어도 됩니다 (예전엔 `search_end` 10 m 안에서 찾아서 후보가 0개였음).
탐색이 켜지면 먼저 **15 초 동안 그 자리를 지키며(전진 상한 0.25) 영역 쪽을 둘러봅니다**(LOOK) —
부표 트래커는 탐색이 켜져야 돌기 시작하고, 시작 자세에서 셋이 들어오는 데 약 15 초 걸렸습니다.
빨강·초록은 보이는 즉시 잡고, 못 찾으면 영역 중심으로 가서 제자리 둘러보기 → 90 초 뒤 포기.
고른 부표가 원 밖이면 규정 방향 **접선으로 진입**하고, 진입 직선이 다른 부표 1.5 m 안을 지나면 비켜 갑니다.
탐색은 게이트처럼 **점유지도(`run_map`)가 있어야 움직입니다** — `/occupancy_grid`가 0.7 초 넘게 없으면 정지
(지도 반발 조향: 영역 이동·접선 진입·선회·출구, LOOK 은 밀려서 되돌아갈 때만). 지도 없는 시험은
`search.require_grid:=false`. `goto`(go_search 앞부분)는 지도를 안 보고 직선으로 갑니다.
검정은 색으로 판정하지 않고 "빨강·초록이 아닌 부표 크기 물체(unknown)"에서 고릅니다 — 빨강·초록도
색이 붙기 전 잠깐 unknown 이라, 검정 목표는 LOOK 15 초가 끝난 뒤에 고릅니다. 흰 기둥·심판 보트도
unknown 이 되니 `search_center`를 탐색 부표 가운데에 찍어 두세요.
검정 후보는 두 조건을 더 통과해야 합니다 — ① **부표 모양**: 점유지도 덩어리 긴 축 ≤ 2.0 m·칸 ≤ 25·
주변 1–2 m 고리 ≤ 12 칸 (배 6 m 안에서 지도마다 재고 최근 60 % 이상 통과. 2026-11-13 bag 실측: 누운 부표
포함 부표 최대 1.28 m·19 칸, 도크 벽·줄부표 2.6–9.5 m·20–155 칸) ② **탐색 부표 무리 안**: 영역 안에서 색이
확인된 빨강·초록 중 가장 가까운 것까지 10 m 안, 가까운 순 (같은 bag 탐색 부표 사이 6.2–8.1 m). 빨강·초록을
하나도 못 봤으면 검정을 고르지 않습니다 — 부표만 한 도크 조각(1.35 m·14 칸)은 모양으로 못 거르기 때문
(`search.black_needs_field:=false`로 끔). 빨강·초록 목표는 구조물로 판정된 것만 뺍니다.
끄기: `search.shape_check:=false`, 값: `search.shape_max_extent`·`search.field_gap` 등.
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

준비는 `goto`와 같고 `run_lidar`와 지도 `run_map`도 필요합니다(지도는 따로 하나만 — 이 런치는 기본으로
안 켬. 따로 켠 게 없을 때만 `start_occupancy_grid:=true`).

```bash
goto avoid_entry                                 # 1) 시작점에 기록 선수각으로 세움 → REACHED 후 Ctrl+C
test_avoid_outdoor drive_enabled:=false          # 2) 미리보기: RViz /real_avoid_test/path 에 경로
test_avoid_outdoor                               # 3) 실제 주행: 출발 조건이 맞으면 바로 출발
test_avoid_outdoor start_point:=gate_entry goal_point:=gate_end   # 다른 지점 쌍
```

- 출발 조건: 시작점에서 3 m(`start_tolerance`) 안, 선수가 목표 방향에서 ±45° 안(다른 미션 2 m·±30° 보다 널널하게). 안 맞으면
  `ros2 topic echo /real_avoid_test/status`에 `start_blocked=…` 이유가 나옵니다.
- 목표 2 m(`goal_tolerance`) 안이면 `FINISHED`로 정지. 경로에서 1.5 m(`max_cross_track_error`) 넘게
  벗어나면 정지, 경로가 5초 넘게 없으면 후진 탈출(`allow_escape:=false`면 정지).
- 계획은 지도(15 m) 안에서 목표 쪽으로 짧게 이어 가므로 목표가 멀어도 됩니다.
- 목표 방향 ±45° 안에 안전한 경로가 없으면 같은 순간 ±90°로 한 번 더 찾습니다(`planner.search_fov_fallback`,
  막힐 때만). 넓혀 만든 경로는 수리 후 곡선 전체가 안전할 때만 씁니다.
- **가상 벽:** 회피 경기장 둘레를 `occupancy_grid`가 지도에 장애물(100)로 박습니다 — 긴 두 변
  (노란 작은 부이 로프)은 전부, 짧은 두 변(동쪽 입구 #2–#3·서쪽 출구 #4–#5)은 **가운데 3 m만 비우고**.
  `src/kaboat_perception/config/virtual_walls.yaml`의 `arenas.avoid`(모서리 위도·경도, 2026-11-12 Space
  기록 #2~#5, 비우는 폭 `openings`). 켤 때와 원점을 다시 잡을 때 지금 원점 기준으로 바꿉니다.
  로그 `가상 벽 2줄 — avoid_0 … 비움: avoid 변 0 가운데 3.0 m, avoid 변 2 가운데 3.0 m`.
  끄려면 `-p virtual_walls_file:=none`. 원점 파일이 없으면 벽을 쓰지 않습니다.
  기록 지점이 부표 안쪽이라 벽이 안쪽에 잡혀서, 둘레를 경기장 바깥쪽으로 **0.6 m** 넓히고
  (`virtual_wall_outward_offset`, 0 = 기록 위치 그대로, 모서리는 이어짐) 두께 **0.3 m**
  (`virtual_wall_thickness`, 칸 0.2 m 라 1칸)로 칠합니다. 예) `run_map -p virtual_wall_outward_offset:=0.8`.
  3 m 입구에서 회피 계획이 지날 수 있는 폭은 약 2 m (DRI 1.116 이하).
- 기록 당시 원점과 지금 원점이 다르면 시작을 거부합니다.

**토너먼트 회피 시험** (`test_tournament`, 종합 자율 미션과 별개로 따로 하는 회피 경기장)

위 야외 회피 시험과 같은 노드·출발 조건·인자이고, 시작·목표 지점 이름만 `tournament_entry` →
`tournament_end`입니다. 종합 미션(`mission_manager`)은 이 두 이름을 읽지 않습니다 — 그래서 `rec` 할 때
`(미션 이름 규칙 아님 …)`이 붙어 나오는 게 정상입니다.

**가상 벽:** 토너먼트 경기장 둘레도 `virtual_walls.yaml`의 `arenas.tournament`에 들어 있어 `run_map`이
회피 경기장과 함께 칠합니다.
- 경기장 크기: 약 10.7 × 28.1 m 직사각형이고, 긴 변은 남서–북동입니다. 2026-11-12 Space 기록 #1~#4입니다.
- 벽 모양: **긴 두 변만 벽**이고 짧은 두 변(남서 끝·북동 끝)에는 벽이 없습니다 (`openings: {0: all, 2: all}`).
  2026-11-13에 가운데 3 m만 비우던 짧은 변 벽을 뺐습니다. `run_map` 로그: `비움: … tournament 변 0 전체, tournament 변 2 전체`.
- 바깥 오프셋: 0.6 m(`virtual_wall_outward_offset`)는 회피 경기장과 같습니다.
- 반영: `virtual_walls.yaml`을 바꾸면 젯슨에서 `cb` 후 `run_map`을 다시 켭니다(설치 폴더의 설정은 복사본).

```bash
rec tournament_entry                             # 0) 노트북: 경기장 입구에 진행 방향으로 세우고 기록
rec tournament_end                               #    도착(목표) 지점에서 기록
goto tournament_entry                            # 1) 시작점에 기록 선수각으로 세움 → REACHED 후 Ctrl+C
test_tournament drive_enabled:=false             # 2) 미리보기: RViz /real_avoid_test/path 에 경로
test_tournament                                  # 3) 실제 주행: 출발 조건이 맞으면 바로 출발
```

- 출발 조건(3 m·±45°), 목표 2 m, 경로 이탈 1.5 m 정지, 후진 탈출은 `test_avoid_outdoor`와 같고 같은 인자로
  바꿉니다. 예) `test_tournament max_cross_track_error:=2.5`.
- 상태: `ros2 topic echo /real_avoid_test/status`.
- 가상 벽을 다 끄려면 `run_map -p virtual_walls_file:=none` (회피·토너먼트 경기장 벽 모두 꺼짐).

**야외 게이트 시험** (`test_gate_outdoor`, 자율주행과 같은 `gate_follower` + `cmd_mux`, 게이트 미션 하나만)

`mission_manager`가 기록 파일 전체가 아니라 `gate_entry`·`gate_end` 두 지점만 받으므로 다른 미션
지점(`station_end` 등)이 없어도 됩니다. 준비는 `goto`와 같고 `run_lidar`와 지도 `run_map`도 필요합니다
(지도는 따로 하나만 — 이 런치는 기본으로 안 켬).

```bash
goto gate_entry                              # 1) 첫 게이트 앞 통로 중심에 세움 → REACHED 후 Ctrl+C
test_gate_outdoor motion_enabled:=false      # 2) 미리보기: 노트북 RViz 에 게이트 쌍(/gate/pairs)·경로(/gate/path)
test_gate_outdoor                            # 3) 실제 주행 (7쌍 통과 → gate_end 2 m 안 → 정지)
test_gate_outdoor approach:=true             # gate_entry 접근(직선, 회피 없음)부터
```

- 기록: `gate_entry` = 첫 게이트 앞 통로 중심(진행 방향으로 세워서), `gate_end` = 마지막 게이트 뒤
  **2~3 m** 통로 중심. 모든 쌍을 지난 뒤 `gate_end`가 4 m 안이어야 그리로 가고 2 m 안이면 완료.
- 추력: 게이트 통과(LOS) 직진 상한 **0.35**(약 1656 µs, `gate.los_thrust_ratio`)·회전 0.31. 0.4(1675 µs)는
  실측 약 1.0 m/s 로 간격 3~4 m 게이트에 빨라 0.3(1638 µs)으로 내렸다가 조금 올림(2026-11-13).
  현장에서 바꿔 보기: `test_gate_outdoor los_thrust:=0.3`.
- `approach:=true` → 게이트 시작 조건: `gate_entry` **2 m 안 + 선수각 ±30° 안**을 0.5초 유지(회전 속도
  조건 없음). 모든 미션의 시작 자세 접근(`mission_approach`)이 같은 조건. 미리보기
  (`motion_enabled:=false`)에서는 도착 신호를 안 내서 `gate`로 넘어가지 않습니다.
- 쌍 고르기: 두 부표를 잇는 선이 진행 방향(직전 게이트 방향, 첫 게이트는 배→`gate_end`)의 수직에서
  35° 안(`gate.max_pair_skew_deg`)이면 쌍. 통로가 휘어도 됨. 폭 1.5~7 m.
- 상태: `ros2 topic echo /gate/status` (`FOLLOW_GATE_3/7`, `GATE_3/7_PASSED`, `WAIT_GATE_PAIR`, `COMPLETE` …).
- **게이트 사슬**(2026-11-12): 지도에서 3번 잡힌 부표는 기억해 두고, 지금 게이트 **다음 게이트를 미리**
  골라 경로를 중점들로 이어 둡니다(최대 3개, `gate.max_chain_gates`). 지금 게이트를 기둥 사이로 지나면
  경로를 새로 만들지 않고 바로 다음 게이트로 넘어갑니다. 로그 `게이트 4/7 미리 잡음 — … 앞 게이트에서
  앞 3.5 m·옆 -1.3 m`, `기둥 사이 통과 — 이어 둔 다음 게이트로`. 예전처럼 하나씩: `-p gate.max_chain_gates:=1`.
  - RViz `/gate/pairs`: 지금 게이트 주황 공(중점 초록), **미리 이은 게이트 노란 공**, 기억한 부표 흰 기둥
    (확정 전 회색). 기억은 4 m 안에 들어온 뒤 1초 넘게 안 잡히면 지웁니다(물보라·반사).
  - 안전·관측 검사는 경로 앞 6 m(`gate.check_horizon`)까지만 — 사슬 끝은 지도 창 밖일 수 있어서.
    이은 부분이 검사에 걸리면 지금 게이트를 버리지 않고 지금 게이트만 가는 경로로 줄입니다.
  - **빔이 지나 본 물은 관측으로 침**: 지도는 맞은 빔만 free 로 칠해서, 뒤에 맞을 게 없는 게이트
    두 부표 사이 물은 unknown(회색)으로 남습니다(앞 빔의 약 80 %가 무반사 inf). 예전엔 경로가 바로
    그 사이를 지나 `PATH_UNOBSERVED` 로 쌍을 버리고 10초 섰습니다. 이제 지금 스캔 빔이 그 점을
    지나 봤으면(더 멀리서 맞음·무반사) 관측으로 칩니다 — 6 m 안, 선수 ±110° 안만(`gate.see_through_*`).
  - **관측 검사는 서지 않음**: 안 본 칸은 경로 앞 3 m(`gate.observed_horizon`)·LOS 앞 직선만 보고, 걸리면
    직진을 절반(`gate.unobserved_thrust_scale`)으로 줄여 계속 갑니다(상태 `FOLLOW_GATE_n/7_SLOW`).
    서는 건 안전 검사(경로 앞 6 m 장애물 0.5 m·DRI, `PATH_UNSAFE`/`LOOKAHEAD_UNSAFE`)와 기둥 안 보임뿐.
  - **멈춤 줄이기**(2026-11-12, 녹화 기반 시뮬로 확인 — 아래 `scripts/gate_sim`):
    - 기둥 보임: 두 기둥이 **각각 1초 안에 한 번** 잡히면 보임(`gate.post_visible_timeout_sec`). 예전엔
      지금 스캔에 둘 다 — 검출이 스캔당 83~92 % 라 4번에 1번 추진 0.
    - 막혀서 버린 쌍 보류 10 → **3초**(`gate.reject_hold_sec`).
    - 바람에 밀려 막히면 서기 전에 **지금 자리에서 경로를 다시 그림**, 안전 검사는 이미 부표 0.5 m
      안에 밀려 들어온 배가 "지금보다 더 가까워지지 않는" 쪽으로는 가게 풂(`gate.replan_from_here`).
    - 고른 뒤 새로 잡힌 부표로 **대각선 오쌍**이 드러나면 바로 버리고 다시 고름(게이트 건너뜀 방지).
    - 부표 덩어리 크기 상한 1.2 → **2.0 m**(`gate.max_cluster_extent`) — 실제 부표가 1.4~2.0 m 로 잡혀
      x≈114 게이트를 건너뛰고 6개만 셌다. float32 해상도로 6칸(1.2 m)이 빠지던 것도 고침.
    - 서서 기다릴 때 자리·선수각 지키기(`gate.hold_thrust_ratio`)는 넣어 두었지만 **기본 끔**
      (시뮬에서 바람엔 좋아졌으나 잔잔할 때 첫 게이트를 놓침).
- 녹화로 다시 돌려보기(노트북): `rec_bag` bag 의 지도·위치·스캔을 지금 코드의 `gate_follower` 에 넣어
  무엇을 골랐고 어디서 멈췄을지 봅니다(배는 녹화 궤적 그대로 — 명령은 배를 안 움직임).
  `python3 scripts/replay_gate.py <bag 폴더> --start 15:31:05 --end 15:32:40 [-p gate.max_chain_gates:=1]`
- 녹화 기반 폐루프 시뮬(노트북, `scripts/gate_sim/`): bag 에서 잰 부표 위치·거리별 검출률·잡음·배 응답으로
  가상 라이다·배를 만들고 실제 `occupancy_grid`·`gate_follower` 를 돌립니다 (배가 명령대로 움직임).
  ```bash
  python3 scripts/gate_sim/sim.py scripts/gate_sim/calib_20261112.json scripts/gate_sim/gates_20261112.json \
      --params src/kaboat_bringup/config/gate_params.yaml --scenario wind10 --seed 0 --verbose
  python3 scripts/gate_sim/run_matrix.py scripts/gate_sim/calib_20261112.json \
      scripts/gate_sim/gates_20261112.json /tmp/m.jsonl --preset final --seeds 6   # 조건 8개 × 시드, 표
  ```
  조건: `base` `wind10`(북 0.1 m/s) `wind20` `wind20_head`(맞바람) `weak_lidar`(검출 70 %) `clutter3`
  `start_off`(선수 25°·옆 1 m) `noisy_pose`. 새 녹화로 다시 재기: `scripts/gate_sim/calibrate.py <bag> calib.json`.

**야외 도킹 시험** (`test_dock_outdoor`, 자율주행과 같은 도킹 노드 + `cmd_mux`, 도킹 미션 하나만)

`dock_entry` 하나만 기록하면 됩니다(`dock_end` 불필요 — 끝 지점은 도킹 중 고정한 경로의 이탈점).
YOLO가 젯슨 CUDA venv를 써야 해서 `ros2 launch`가 아니라 `scripts/run_dock_mission.sh`로 켭니다
(`run_dock_yolo.sh`와 같은 환경·잠금 — 둘을 같이 켜지 않습니다). 센서·모터·지도·카메라는 따로:
`run_gq7_rtk`, `run_lidar`, `run_camera`, `run_map`, `run_motor`. 녹화는 `rec_bag dock_test`.

```bash
goto dock_entry                              # 1) 도크 정면, 세 도형이 다 보이는 곳(뒷벽에서 8~12 m)에 세움 → Ctrl+C
test_dock_outdoor motion_enabled:=false      # 2) 미리보기: 배가 안 도니 목표가 화면 가운데 ±8°면 방위 고정까지 (/dock/status)
test_dock_outdoor target_class:=circle       # 3) 실제 도킹 (기본 triangle)
test_dock_outdoor approach:=true             # dock_entry 접근(직선, 회피 없음)부터
```

- 순서 (`dock_ctrl_v2`, 2026-11-13):
  1. **SCAN**: 제자리에서 목표 도형만 보고, 그 세계 방위로 선수를 돌립니다(odom PD 0.8/0.4, 데드밴드 없음).
     화면 가운데 ±8° 프레임 5개(최근 2초)의 세계 방위가 3° 안이면 고정합니다. 세계 방위는 **영상 촬영 시각의
     선수각**으로 계산하므로(카메라 지연 약 0.2 s 보상) 회전 중이어도 되고, 선회율 제한은 없습니다.
     세 슬롯 중 어디로 갈지는 이것으로 정합니다.
     - 목표 도형이 안 보이면 `search_turn_cmd`(기본 0.06, 약 8°/s)로 선회각 제한 없이 돌며 찾습니다. 방향은:
       - 다른 도형 **2개**: 슬롯은 같은 간격이라 목표는 두 도형 화면 간격만큼 바깥 어느 한쪽. 화면 안 후보는
         빼고(보였을 것) 가운데에 가까운 쪽. 가까우면 도형이 양쪽에 걸쳐 보여 평균 쪽은 틀립니다(시뮬 5.5 m).
         화면 안/밖은 검출기가 `camera_info`로 계산해 함께 보내는 좌우 반화각(`image_half_fov_deg`, D455
         640x480 실측 왼쪽 40.0°·오른쪽 40.6°)으로 판단하고, 없으면 `camera_half_fov_deg`(40.0)를 씁니다.
       - 다른 도형 **1개**가 가운데 ±10° 밖: 그 도형 쪽(목표는 그 너머).
       - 다른 도형 1개가 **가운데 근처(±10°)**: 뒷벽 약 4.2 m 안에서만 생깁니다(표지 간격 3.55 m, 반화각 40°).
         **치우친 반대쪽**으로 제자리 선회합니다 — 가려진 옆 도형 중 화면 가장자리에 더 가까운 쪽이라 가장 적게
         돌아 단서가 나옵니다. 정확히 가운데면 시계방향. 후진은 하지 않습니다(라이다가 120~240°를 가려 뒤를 못 봄).
       - 정한 방향으로 돌아도 새 도형이 안 나오면 **한 번** 반대로 돕니다: 도형 1개로 정했으면 **75°**(시뮬에서 맞는 쪽이면 최대 68° 안에 보임), 도형 2개
         (간격 규칙)로 정했으면 **예측 위치가 광축 가운데에 올 때까지**(예측 각도, 최소 20°). 간격 규칙은 화면 간격이
         같다고 보고 가까운 후보를 고르는데, 보이던 도형이 끝 슬롯이면 틀립니다(시뮬: 끝 슬롯 앞에서 330° 돎).
         같은 간격 가정의 위치 오차(시뮬 −30~+40°)가 반화각 40° 안이라 맞는 쪽이면 그 전에 보입니다.
         반전한 쪽은 비어 있다고 보고, 그 뒤 간격 규칙도 그쪽 후보를 뺍니다.
       - **아무것도 없음**: 시계방향.
       - 정한 방향은 목표를 볼 때까지 유지(돌면 보이던 도형이 반대로 밀려나므로)하고, 더 좋은 근거(1개 → 2개)가
         처음 생길 때만 다시 정합니다. YOLO 상태 자체가 끊기면(`YOLO stale`) 멈춥니다.
       - 누적 `search_max_turns`(기본 2)바퀴를 돌아도 목표가 없으면 멈추고 `/dock/status`에
         `target … not found after 2 turns — holding`을 남깁니다(한 바퀴 약 45 s). 목표가 보이면 다시 진행합니다.
       - 시뮬(11/13 bag 식별 모델) 확인: 멀리서 목표가 좌/우 화면 밖, 가까이(5.5 m)서 양 끝이 화면 밖, 도크
         반대 방향으로 시작, 바람 편향 ±4°/s — 모두 맞는 슬롯에 COMPLETE. 단 옆 슬롯 정면 5.5 m에서 두 칸 떨어진
         슬롯이 목표면(축과 49°) SEEK에서 ㄷ자 한쪽 면이 안 보여 앞 부교 2 m 앞에서 멈춥니다(아래 40° 주의).
     - 2026-11-13 bag: 예전 방식(받은 시각 선수각, ±3° 데드밴드, ±6°·4°/s 동시 조건)은 ±18°·25°/s로 계속
       흔들려 한 번도 고정하지 못했습니다(텔레옵 아님, `analysis_20261113/REPORT.md`).
  2. **SEEK**: 목표 도형을 선수 ±8° 안에 둘 때만 `seek_cmd`(기본 0.07)로 서행하고, 벗어나면 제자리에서 선수만
     맞춥니다. 보이는 동안 지연 보상한 방위로 목표 방위를 갱신하고(한 번에 10° 넘게 튀면 버림), 가까워져
     안 보이면 마지막 방위로 갑니다. 라이다 스캔면이 수면 위 0.14 m라 슬롯 벽은
     **약 6 m 안에서만** 잡힙니다(2026-11-12 bag: 6 m 안 91~100%, 8 m 밖 거의 0).
     - 최근 1초의 `/scan/level`을 odom에 쌓아 **고정 치수 ㄷ자 슬롯**을 찾습니다(`dock_slot.yaml`:
       폭 2.97 m, 깊이 1.94 m). 뒷벽, 양쪽 안쪽 면(가려지면 칸막이 끝), 비어 있는 슬롯 안을 한꺼번에 맞춥니다.
     - 목표 방위선에서 1.6 m 안의 ㄷ자만 봅니다. YOLO는 세 슬롯 중 어디인지만 정하고, 중심은 라이다로 잡습니다.
     - 뒷벽 6 m 안에서 같은 ㄷ자가 1초 넘게 같은 자리(0.3 m)·법선(5°)이면 경로를 **한 번** 고정합니다
       (2026-11-13 bag 재생: 7.7 m 고정은 슬롯 중심이 0.89 m 틀림, 6 m 안은 ±0.15 m).
     - 앞 ±25°·2 m 안에 점이 있거나, 10 m를 가도 못 잡으면 멈춰 기다립니다.
     - **우회(REPOSITION, 한 번만):** 가까이서 목표 슬롯 축과 비스듬하면 목표 표지로 가는 시선이 칸막이에
       걸려 목표 ㄷ자 한쪽 면이 안 보이고, 그대로 기어가면 칸막이 끝을 옆으로 스친다(시뮬 선체 중심 0.72 m).
       - 들어가는 조건: SEEK 중 6 m 안에 양쪽 면까지 보이는 ㄷ자가 있고 그 법선과 목표 방위가
         **37° 이상**(atan(슬롯 반폭 1.485/깊이 1.94))이면 다가가기 전에 바로. 도크 위치는 몰라도 된다(보이는
         ㄷ자 법선 = 세 슬롯 공통). 보이는 ㄷ자가 바로 목표 슬롯이면 우회 안 함(SEEK 가 그대로 잠금).
         예비로 앞이 3초 넘게 막힌 채 ㄷ자를 못 잡을 때도.
       - 보이는 ㄷ자를 슬롯 간격 `slot_pitch_m`(3.55 m, bag 추정·실측 아님)만큼 옮겨 임시 목표 ㄷ자를 잡고, 그 축 위
         뒷벽 5 m 앞 경유점으로 간다(경유점 쪽 선회 → 전진(앞 막힘 검사) → 임시 ㄷ자 정면 선회 → SCAN 다시).
         5 m 는 ㄷ자 고정 거리(6 m) 안·BACKOFF 기준(3.94 m) 밖.
       - 우회 중에도 임시 목표 축으로 목표 ㄷ자를 찾아, 중심이 임시 목표에서 1 m 안이고 뒷벽 3.94 m 이상에서
         안정되면 경유점 전에 바로 TO_VIA.
       - 최종 경로·정지점은 목표 ㄷ자를 다시 잠가 정한다. 보이는 ㄷ자가 없거나 우회 뒤 또 막히면 멈추고
         `detail`에 이유를 남긴다.
     - 예전 방식(마커 시선 + 직선 하나)은 뒷벽과 평행한 칸막이 끝면(1.94 m 앞)을 벽으로 잡았습니다.
       2026-11-12 bag 기준으로 접근 첫 고정 8번 중 5번이 정지점 2 m 짧음, 옆으로 0.8~2.3 m 어긋남이었습니다.
       ㄷ자 방식은 같은 bag에서 첫 고정 13번 중 12번이 깊이 0.24 m·좌우 0.38 m·법선 2.8° 안이었습니다.
       나머지 1번은 화면 끝 YOLO 검출로 옆 슬롯을 고른 경우인데, SCAN은 화면 가운데(±8°)에서만 방위를 고정합니다.
  3. 고정할 때 이미 경유점 + 0.5 m보다 가까우면 먼저 선수 유지 후진(**BACKOFF**, 최대 3 m)합니다.
  4. 경유점(입구 밖 `pre_entry_offset` 1.5 m = 뒷벽 앞 3.44 m) → ㄷ자 중심선과 평행하게 선수 정렬 →
     정지점(뒷벽 앞 `berth_distance` 1.0 m, 선체 중심) → 선수 유지 후진 3.25 m
     (3 m 이상·좌우 0.75 m 안) → `/mission/complete=dock` → 이탈점 2 m 안이면 `done`(정지).
- ⚠️ 양쪽 면이 보이려면 슬롯을 **40° 안쪽**에서 바라봐야 합니다(`dock_entry`는 가운데 슬롯 정면, 8~12 m).
  RViz에서 찾은 ㄷ자는 `/dock/slot_markers`(초록 선), 상태는 `/dock/status`의 `slot`(찾았는지, 이유, 면별 점 비율).
- ⚠️ 목표 도형이 안 보이면 전진 0으로 제자리 탐색 선회합니다(`/dock/status`의 `detail`: `searching left/right …
  (other shapes …; turned …/720 deg)`). 누적 2바퀴면 멈추고 `not found after 2 turns — holding`을 남깁니다
  (`dock_target_class`·조명 확인). YOLO가 꺼져 있으면 멈춰 기다립니다. 미션 매니저에는 도킹 시간 제한이 없습니다.
- RViz(`scripts/demo/rviz_dock.sh --real` → `dock_real.rviz`): `/dock/ctrl_markers` — 배 위에 단계·상세·명령, 그 위에 단계 전이 기록. 초록/노랑 선 = 지연 보상한 목표
  시선(초록이면 방위 고정 표본 ±8° 안), 주황 = SCAN 목표 방위, 빨강 = 고정 방위선, 하늘색 = SEEK 선수 ±8°.
- 도킹 전에 마스크만 따로 확인: `bash scripts/run_dock_yolo.sh`(카메라 이미 켰으면 `start_camera:=false`) →
  다른 창에서 `check_yolo`. 목표 도형이 신뢰도 0.55 이상으로 꾸준히 잡혀야 합니다.
- 도형 배치: 기본은 순서를 안 봄. 대회 배치를 알면 `marker_order:=square,triangle,circle`(화면 왼쪽부터)로
  켭니다. 그러면 같이 보이는 마커들의 좌우 순서가 어긋난 프레임은 목표 방위에서 뺍니다. 화면 끝의 엉뚱한
  도형 오검출을 거르는 용도입니다.
- 위치: 기본 `/odom`(GQ7 RTK 필터 25 Hz, 선체 중심). `position_source:=gps`는 `/gps/fix` 원시(2 Hz, 늦음).
- 추력 (`seek_cmd` / `approach_cmd` / `final_cmd` / `retreat_cmd`, 기본 0.07 / 0.10 / 0.05 / 0.05
  ≈ 1551 / 1562 / 1544 / 1456 µs)은
  teleop 1단계(1550 µs)와 비슷해 바람에 약합니다. 미리보기 뒤 바람을 보고 올립니다
  (참고: 0.2 ≈ 1600 µs, goto 기본 0.4 ≈ 1675 µs).
- 카메라 TF(라이다 → `camera_link`, 뒤 0.615·위 0.635 m, 수평)는 이 런치가 냅니다. 값은
  `src/kaboat_perception/kaboat_perception/camera_mount.py` 한곳(카메라 층·`dock_yolo`와 공유).
- 상태: `ros2 topic echo /dock/status` (`phase`: `SCAN` → `SEEK` → (`BACKOFF`) → `TO_VIA` → `ALIGN_VIA` → `TO_GOAL` → `REVERSE` →
  `COMPLETE`), YOLO `/dock/detection_status`, 라이다 `/dock/lidar/status`. RViz `bash scripts/demo/rviz_dock.sh --real`.
- 전체 미션(`autonomy.launch.py`)의 도킹도 같은 노드(`kaboat_perception/dock_nodes.py`)라 같은 인자
  (`dock_target_class`, `dock_marker_order`, `dock_retreat_cmd` …)를 받습니다. 그냥 `ros2 launch`로 켜도
  됩니다 — 런치가 젯슨 YOLO 가상환경(`.venv-dock-yolo`)을 찾아 **검출기 노드에만** `PYTHONPATH`·
  `LD_LIBRARY_PATH`(libcusparseLt)를 붙입니다. 못 찾으면 시작할 때 `⚠️ [dock] YOLO 가상환경 … 못 찾음`이
  나오고 검출을 안 합니다. 가상환경 확인은 [DOCKING_YOLO.md](DOCKING_YOLO.md) "젯슨 YOLO 가상환경".

**카메라 층 — 라이다가 놓치는 낮은 색 장애물** (주황 붐 등, `occupancy_grid` 의 `cam_enabled`, 기본 끔)

라이다 수평 한 면은 수면에 누운 주황 붐 위로 지나갑니다. 카메라 층은 컬러 영상에서
주황·빨강 픽셀을 고르고 정렬 뎁스로 위치를 재(0.6~5 m), IMU 롤·피치로 수평 보정한 뒤
수면 아래 0.3 m 위쪽 점(반사상은 붐과 같은 자리라 받음)만 지도에 따로 쌓습니다. 라이다가 지우지
못하고, 카메라가 그 자리를 다시 보는데 없을 때만 사라집니다. 처리 10 Hz(`cam_rate_hz`), 같은 칸이
3프레임(약 0.3초) 이어져야 회피 대상(95)이 됩니다(`cam_l_occ` 1.0). 보이는데 없는 프레임은 −0.4
(`cam_l_free`)라 처리 프레임의 약 29% 이상에서 잡혀야 칸이 유지됩니다. 흰색은 수면 반짝임 오탐이 많아 기본 끔(`cam_white:=true`).

```bash
run_camera                                                         # 젯슨: D455 (= ros2 launch kaboat_bringup d455.launch.py)
run_map -p cam_enabled:=true                                       # 지도에 카메라 층 켜기 (run_map 기본은 라이다만)
                                                                   # 카메라 높이 기본 0.775 m (실측)
```

- `cam_height` = 배가 물에 뜬 상태에서 흘수선부터 카메라 렌즈 중심까지 [m], 실측 **0.775**가 기본
  (바꾸면 `-p cam_height:=0.8`처럼 소수점 포함). 카메라는 **수평**으로 달았고(2026-11-12 10° 숙임에서
  되돌림) 그게 기본 (`cam_mount_pitch_deg` 0, + = 아래). 선체 중심 기준 위치는 뒤 **0.215 m**(`cam_offset_x`, 라이다에서 뒤 0.615 m·위 0.635 m, 2026-11-12 실측).
  카메라를 옮기면 `src/kaboat_perception/kaboat_perception/camera_mount.py` 한곳만 고칩니다 — 카메라 층 기본값과
  도킹 TF(`dock_yolo`, `test_dock_outdoor`, `autonomy.launch.py`)가 같이 읽습니다.
- 노트북 `view_lidar`의 주황 점(`/occupancy_grid/cam_points`)이 카메라가 고른 점입니다. 붐 위에
  모이는지, 수면에 점이 흩어지지 않는지 봅니다.
- 색 기준·높이 하한은 `cam_orange_*`, `cam_white_*`, `cam_range_*` 파라미터. 주황 기본값
  (2026-11-12 현장 완화): 채도 ≥ **90**(`cam_orange_s_min`), 밝기 ≥ **50**(`cam_orange_v_min`),
  높이 ≥ **−0.3 m**(`cam_orange_z_min`) — 장착 각도 오차로 붐이 수면 아래로 계산돼도 받고, 반사상은
  붐과 같은 자리라 받아도 됨. 붐 앞 수면에 점이 번지면 `-p cam_orange_z_min:=-0.1`, 갈색·석양 반사가
  잡히면 `-p cam_orange_s_min:=110`.
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
