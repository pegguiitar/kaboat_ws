# 🎥 주행 센서 녹화 (rosbag)

실제 주행 때 센서 토픽을 젯슨에 저장해 두고, 노트북에서 그대로 재생하며 알고리즘을 시험하기 위한
문서입니다. 명령 모음은 [operation.md](operation.md) 3절을 봅니다.

- 쓰는 프로그램: ROS 2 기본 도구 **`ros2 bag`** (따로 설치할 것 없음).
- 우리는 이를 감싼 **`scripts/record_run.sh`** (단축어 **`rec_bag`**)를 씁니다. 토픽 목록·압축·
  용량 확인·파일 분할을 알아서 해 줍니다.
- 녹화는 **젯슨에서** 합니다. 무선으로 노트북에 보내며 녹화하면 영상 프레임이 빠집니다.

---

## 1. 처음 한 번 (젯슨)

```bash
cd ~/Desktop/kaboat_ws/repo
git fetch && git checkout competition && git pull   # rec_bag 은 competition 에 합쳐짐
bash scripts/setup_jetson_bashrc.sh && source ~/.bashrc     # rec_bag 단축어 등록
sudo apt install ros-humble-image-transport-plugins tmux    # 카메라 압축 토픽 + 끊김 방지
```

`image-transport-plugins`가 없어도 녹화는 됩니다. 다만 원본 영상을 저장해 용량이 약 3배가 됩니다.

---

## 2. 현장 순서

```bash
tmux new -s bag            # ① SSH가 끊겨도 녹화가 이어지도록 tmux 안에서
# ② 다른 창에서 센서를 먼저 켠다: run_gq7_rtk, run_lidar, run_camera (필요하면 run_geo)
rec_bag gate_test          # ③ 녹화 시작. 이름은 자유 (생략 시 run)
# ④ 미션·주행 실행 (test_gate_outdoor 등)
# ⑤ 끝나면 녹화 창에서 Ctrl+C
```

- 저장 위치: `~/kaboat_bags/gate_test_20261112_143000/`
- **반드시 Ctrl+C로 끕니다.** 창을 그냥 닫으면 마무리(`metadata.yaml`)가 안 됩니다.
- tmux에서 빠져나오기 `Ctrl+B` → `D`, 다시 들어가기 `tmux attach -t bag`.

시작할 때 화면에 이렇게 나오면 정상입니다.

```
📷 카메라: 압축 토픽 (…/compressed, …/compressedDepth) — 약 4~5 MB/s
💾 녹화 시작 → /home/…/kaboat_bags/gate_test_…  (남은 용량 57 GB)
```

`⚠️ 압축 토픽 없음`이 나오면 원본 영상으로 녹화 중입니다(1절 apt 설치 필요).
`⚠️ 카메라 토픽이 아직 없음`이면 `run_camera`를 안 켠 것입니다. 나중에 켜도 녹화에 들어갑니다.

### 카메라 빼고 녹화 (`rec_bag --no-camera`)

카메라가 필요 없는 시험(회피·토너먼트·게이트)은 영상을 빼고 녹화합니다.

```bash
rec_bag --no-camera tournament     # 카메라 토픽·도킹 YOLO 그림 없이, 압축도 안 함
```

화면에 `📷 카메라 빼고 녹화 (영상·압축 없음 — 젯슨 부하 최소)`가 나오면 정상입니다.

- 압축 토픽이 없을 때 원본 영상을 zstd 로 압축하며 녹화하면 젯슨이 따라가지 못해 **메시지를 버립니다**
  (2026-11-13 토너먼트: `Dropping message … compression queue … full`, 컬러 영상 1/3·로그 일부 누락,
  `/rosout` 10초 지연). 영상이 꼭 필요하면 1절 플러그인을 설치해 압축 토픽으로 녹화하세요.
- `--no-camera`(줄여서 `--nocam`)는 `--rgb`와 같이 쓸 수 없습니다.

### 카메라 RGB 영상만 녹화 (`rec_rgb`)

센서·판단 토픽 없이 컬러 영상 하나만 남깁니다(뎁스·`camera_info`도 없음). `run_camera`만 켜 두면 됩니다.

```bash
rec_rgb                    # ~/kaboat_bags/rgb_20261112_153828/  (이름 생략 시 rgb)
rec_rgb dock_view          # ~/kaboat_bags/dock_view_…/
# 끝나면 Ctrl+C → 영상으로 보기
bag_to_mp4 ~/kaboat_bags/rgb_20261112_153828   # 폴더 안 rgb_20261112_153828.mp4
```

- 압축 토픽(JPEG)이 있으면 약 1~1.5 MB/s, 없으면 원본 + zstd로 약 7~14 MB/s입니다(640×480 15 fps 기준).
- 시작 화면에 `📷 RGB 영상만: /camera/color/image_raw/compressed`가 나오면 정상입니다.
- `rec_bag`과 동시에 켤 필요는 없습니다. `rec_bag`에도 같은 컬러 영상이 들어 있습니다.

---

## 3. 무엇이 저장되나

| 구분 | 토픽 |
|---|---|
| LiDAR | `/scan`, `/scan/raw`, `/scan/level` |
| 자세·위치 | `/imu/data`, `/gps/fix`, `/gps/fix_secondary`, `/odom`, `/tf`, `/tf_static`, `/geo/origin`, `/geo/status`, `/geo/position`, `/geo/targets`·`target_local`·`target_markers` |
| GQ7 진단 | `/ekf/status`(**`gnss_state` = RTK Fixed/Float**), `/ekf/odometry_earth`(필터 원본 25 Hz), `/ekf/dual_antenna_heading`, `/gnss_1/velocity`, `/mip/ekf/gnss_dual_antenna_status`·`aiding_measurement_summary`·`gnss_position_aiding_status`, `/rtcm`(NTRIP 보정 입력), `/nmea` |
| 카메라 | 컬러 JPEG, 정렬 뎁스 무손실 PNG(`compressedDepth`), `/camera/camera_info`, `/occupancy_grid/cam_points`(카메라 장애물 점, 최대 3000점) |
| 수동·자율 구분 | `/teleop/enable`·`levels`·`esp_mode`·`status`·`event`, `/rc/cmd_vel`, `/emergency_stop`, `/diagnostics` |
| 노드 로그 | `/rosout` — 회피 ESCAPE 진입 이유처럼 토픽으로 안 나오는 판단 |
| 미션 전환 | `/mission/state`, `/mission/goal`, `/mission/entry_goal`, `/mission/complete`, `/cmd/approach`, `/mission/approach_cmd_preview`, `/mission_points/markers`·`result`, `/waypoint/status` |
| 장애물 회피 | `/avoid/dri`(위험도 지도), `/avoid/plan`(B-spline 경로), `/avoid/cps`(제어점), `/avoid/dri_marks`(고위험 칸), `/cmd/avoid` |
| 스테이션·수색 | `/station/map`·`markers`·`status`·`cmd_preview`, `/cmd/station`, `/search/markers`·`status`, `/cmd/search` |
| 게이트 | `/gate/status`(WAIT·PASSED…), `/gate/pairs`, `/gate/path`, `/gate/cmd_preview`, `/cmd/gate` |
| 도킹 YOLO 결과 | `/dock/detection_status`(마커별 신뢰도·픽셀·시선, JSON), `/dock/debug_image`(마스크·도형 그린 영상 320 px 5 fps ≈ 1.2 MB/s), `/detections/dock_marks`, `/dock/bearing_markers`, `/dock/markers/<도형>/ray`·`depth_ground`·`ground`·`base`·`ground_base`, `/dock/marker_base`·`marker_ground(_raw)`, `/dock/markers_base`·`markers_ground`, `/detector/enable` |
| 도킹 라이다·경로 | `/dock/lidar/status`·`markers`·`direct_markers`·`<도형>/normal(_base)`·`wall_point(_base)`·`marker_ground`·`marker_base`, `/dock/waypoints`, `/dock/approach`·`alignment`·`stop_point`, `/dock/entrance_base`, `/dock/avoid/goal(_status)`·`goal_candidate`·`goal_markers`, `/dock/bspline_path`, `/dock/slot_markers`(제어가 고른 ㄷ자 슬롯), `/dock/status`(`slot`: 찾았는지·이유·면별 점 비율), `/dock/one_shot_path`·`gps`·`markers`, `/dock/cmd_preview`, `/cmd/dock` |
| 당시 판단 (비교용) | `/cmd_vel`, `/occupancy_grid`, `/detections/buoys`·`buoys_viz` |
| 야외 단독 시험 | `/real_avoid_test/path`, `/manual_dri/dri`·`status`, `/los_test/path`·`status`·`cmd_preview`, `/waypoint/goal` |

- 그 순간 돌고 있는 노드의 토픽만 저장됩니다. 미션은 하나씩 돌아서 회피·스테이션 지도가 동시에
  쌓이지 않습니다. 판단 토픽을 다 합쳐도 늘어나는 양은 많아야 약 0.3 MB/s입니다(카메라의 5% 안팎).
- `/avoid/dri`·`/avoid/plan`·`/station/map`·`/occupancy_grid/cam_points`는 **구독자가 있을 때만**
  만들어지는 디버그 토픽입니다. 녹화를 켜면 젯슨이 10 Hz로 만들기 시작하므로(메시지당 몇 ms),
  처음 한 번은 `jtop`으로 CPU 여유를 봅니다.
- 압축 안 된 카메라 원본(`image_raw`)은 일부러 뺐습니다. `ros2 bag record -a`(전부 녹화)를 쓰면 이것까지 들어가
  용량이 몇 배가 되므로 쓰지 않습니다.

토픽을 더 넣고 싶으면 `scripts/record_run.sh`의 `topics=( … )`에 추가합니다.

---

## 4. 용량과 부하

| 카메라 저장 방식 | 초당 | 20분 |
|---|---|---|
| 압축 토픽 (기본) | 약 4~5 MB | 약 6 GB |
| 압축 토픽 없음 → 원본 + zstd | 약 12~15 MB | 약 15~18 GB |

- 남은 용량이 **10 GB 미만이면 시작하지 않습니다**(`KABOAT_BAG_MIN_FREE_GB`로 변경).
- **2 GB마다 파일을 나눕니다.** 중간에 문제가 생겨도 앞부분은 남습니다.
- 녹화 자체는 디스크에 쓰기만 해서 가볍습니다. 압축은 카메라 노드가 하며 대략 CPU 코어 1개의
  30~40% 정도입니다(추정). 처음 녹화 때 `jtop`으로 YOLO와 같이 돌려도 여유가 있는지 봅니다.
- 남은 용량 확인: `df -h ~`, 지난 녹화 크기: `du -sh ~/kaboat_bags/*`

---

## 5. 녹화 확인과 노트북으로 옮기기

```bash
ros2 bag info ~/kaboat_bags/gate_test_20261112_143000     # 젯슨: 길이·토픽별 메시지 수
```

토픽별 메시지 수가 0이면 그 센서가 꺼져 있었던 것입니다.

```zsh
# 노트북: 젯슨(Tailscale 100.108.192.55)에서 받아오기
rsync -avP <젯슨계정>@100.108.192.55:~/kaboat_bags/gate_test_20261112_143000 ~/kaboat_bags/
```

옮긴 뒤 젯슨 쪽은 지워서 용량을 확보합니다: `rm -r ~/kaboat_bags/gate_test_…`

---

## 6. 노트북에서 재생 (간단히)

```zsh
ros2 bag info ~/kaboat_bags/gate_test_…                   # 노트북(Jazzy)에서도 그대로 읽힘
ros2 bag play ~/kaboat_bags/gate_test_… --clock           # 녹화 시각 그대로 재생
ros2 bag play … --clock --topics /scan /odom /imu/data    # 일부 토픽만
```

- 시험할 노드는 `use_sim_time:=true`로 띄웁니다. 그래야 노드 시계가 bag 시각을 따릅니다.
- 시험하는 노드가 내는 토픽(예: `/occupancy_grid`)은 재생에서 빼야 새 결과와 섞이지 않습니다.
  예: 회피를 다시 돌릴 때 `ros2 bag play … --clock -x '^/(avoid/|cmd/avoid|cmd_vel$)'`
- 재생은 **보트 움직임이 녹화 그대로**입니다. 인지(LiDAR·카메라·격자)와 판단(FSM 상태 전이)
  시험에는 좋지만, 제어 게인(LOS·웨이포인트) 튜닝에는 쓸 수 없습니다.

---

## 7. 막힐 때

| 증상 | 해결 |
|---|---|
| `metadata.yaml`이 없음 (창을 닫았거나 SSH가 끊김) | `ros2 bag reindex ~/kaboat_bags/<폴더>` |
| `❌ 남은 용량 … GB` | 지난 녹화를 노트북으로 옮기고 지움 |
| `⚠️ 압축 토픽 없음` | `sudo apt install ros-humble-image-transport-plugins` 후 `run_camera` 재시작 |
| 녹화 창에 `Dropping message … compression queue … full` | 원본 영상 압축을 젯슨이 못 따라감 — 카메라가 필요 없으면 `rec_bag --no-camera`, 필요하면 위 플러그인 설치 |
| 영상 메시지 수가 기대보다 훨씬 적음 | 저장 장치가 느림(SD카드). NVMe/USB SSD로 `KABOAT_BAG_DIR=/경로 rec_bag …` |
| `rec_bag: command not found` | 1절 `setup_jetson_bashrc.sh` 다시 실행 후 `source ~/.bashrc` |
