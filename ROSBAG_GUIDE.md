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

---

## 3. 무엇이 저장되나

| 구분 | 토픽 |
|---|---|
| LiDAR | `/scan`, `/scan/raw`, `/scan/level` |
| 자세·위치 | `/imu/data`, `/gps/fix`, `/gps/fix_secondary`, `/odom`, `/tf`, `/tf_static`, `/geo/origin`, `/geo/status` |
| 카메라 | 컬러 JPEG, 정렬 뎁스 무손실 PNG(`compressedDepth`), `/camera/camera_info` |
| 도킹 YOLO 결과 | `/dock/detection_status`(마커별 신뢰도·픽셀·시선, JSON), `/dock/debug_image`(마스크·도형 그린 영상 320 px 5 fps ≈ 1.2 MB/s), `/detections/dock_marks`, `/dock/bearing_markers`, `/dock/markers/<도형>/ray`·`depth_ground`, `/detector/enable` |
| 도킹 라이다·경로 | `/dock/lidar/status`·`markers`·`direct_markers`·`<도형>/normal`·`wall_point`, `/dock/waypoints`, `/dock/avoid/goal(_status)`, `/dock/status`, `/dock/one_shot_path`·`gps`·`markers`, `/dock/cmd_preview`, `/cmd/dock`, `/mission/complete` |
| 당시 판단 (비교용) | `/cmd_vel`, `/occupancy_grid`, `/mission/state`, `/mission/goal`, `/detections/buoys`, `/dock/lidar/status`, `/waypoint/status` |

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
- 재생은 **보트 움직임이 녹화 그대로**입니다. 인지(LiDAR·카메라·격자)와 판단(FSM 상태 전이)
  시험에는 좋지만, 제어 게인(LOS·웨이포인트) 튜닝에는 쓸 수 없습니다.

---

## 7. 막힐 때

| 증상 | 해결 |
|---|---|
| `metadata.yaml`이 없음 (창을 닫았거나 SSH가 끊김) | `ros2 bag reindex ~/kaboat_bags/<폴더>` |
| `❌ 남은 용량 … GB` | 지난 녹화를 노트북으로 옮기고 지움 |
| `⚠️ 압축 토픽 없음` | `sudo apt install ros-humble-image-transport-plugins` 후 `run_camera` 재시작 |
| 영상 메시지 수가 기대보다 훨씬 적음 | 저장 장치가 느림(SD카드). NVMe/USB SSD로 `KABOAT_BAG_DIR=/경로 rec_bag …` |
| `rec_bag: command not found` | 1절 `setup_jetson_bashrc.sh` 다시 실행 후 `source ~/.bashrc` |
