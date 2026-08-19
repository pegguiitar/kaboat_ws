# 천장 AprilTag → Wi-Fi → Jetson `/odom`

천장 웹캠 영상은 노트북 안에서 처리하고, 계산된 AprilTag TF와 검출 상태만
ROS 2 DDS로 Jetson에 보낸다. 원본 영상은 Jetson이 구독하지 않으므로 Wi-Fi
대역폭을 계속 사용하지 않는다.

```text
노트북: camera_ros → image_proc → apriltag_ros → /tf, /apriltag/detections
                                               │ 같은 Wi-Fi / ROS 2 DDS
Jetson: GQ7 /imu/data ─────────────────────────┼→ apriltag_odom → /odom
```

최종 `/odom`의 위치·yaw는 AprilTag, 선속도는 태그 위치 미분, 각속도는
Jetson의 GQ7 자이로를 사용한다.

## 1. 공통 Wi-Fi·ROS 설정

두 장비를 같은 공유기의 **일반 SSID**에 연결한다. 게스트 Wi-Fi와 AP/client
isolation은 장치끼리 통신을 차단하므로 사용하지 않는다. 양쪽에서 상대 IP로
`ping`이 되어야 한다.

노트북과 Jetson의 모든 관련 터미널에서 동일하게 설정한다.

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0
```

부팅 후에도 유지하려면 두 줄을 각 장비의 `~/.bashrc`에 추가한다. ROS 배포판과
RMW 구현도 양쪽에서 같게 유지하는 것이 좋다.

카메라 검출 stamp는 노트북 시각이고 Jetson이 `0.3s` timeout을 판정하므로
시각 동기화가 필수다.

```bash
sudo apt update
sudo apt install chrony
sudo systemctl enable --now chrony
chronyc tracking
```

먼저 노트북에서 시험 토픽을 발행한다.

```bash
ros2 topic pub -r 1 /network_test std_msgs/msg/String \
  "{data: 'laptop connected'}"
```

Jetson에서 다음 문자열이 보여야 한다.

```bash
ros2 topic echo /network_test
```

ping은 되는데 토픽이 안 보이면 `ROS_DOMAIN_ID`, `ROS_LOCALHOST_ONLY`, Ubuntu
방화벽, 공유기 client isolation 순서로 확인한다.

## 2. 노트북 설치와 빌드

노트북은 Ubuntu 22.04 + ROS 2 Humble 기준이다.

```bash
sudo apt update
sudo apt install \
  ros-humble-camera-ros \
  ros-humble-image-proc \
  ros-humble-apriltag-ros \
  ros-humble-camera-calibration \
  ros-humble-rqt-image-view
```

노트북에도 이 저장소의 `real` 브랜치를 clone한 뒤 빌드한다.

```bash
cd ~/kaboat_ws/repo
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select kaboat_hardware
source install/setup.bash
```

웹캠 인식 여부를 먼저 확인한다.

```bash
lsusb
cam -l
```

## 3. 웹캠 내부 파라미터 보정

AprilTag PnP는 `CameraInfo`의 초점거리와 주점을 사용한다. 미보정 카메라는
좌표를 발행하더라도 실제 위치로 사용하면 안 된다.

검출기를 끄고 카메라만 실행한다.

```bash
source /opt/ros/humble/setup.bash
source ~/kaboat_ws/repo/install/setup.bash
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0

ros2 launch kaboat_hardware ceiling_apriltag.launch.py \
  camera:=0 width:=1280 height:=720 enable_detector:=false
```

다른 터미널에서 체스보드 내부 코너 수와 한 칸 실측 길이를 넣는다. 아래
`8x6`, `0.030`은 예시다.

```bash
source /opt/ros/humble/setup.bash
ros2 run camera_calibration cameracalibrator \
  --size 8x6 --square 0.030 \
  image:=/ceiling_cam/camera/image_raw \
  camera:=/ceiling_cam/camera
```

GUI에서 여러 위치·각도로 샘플을 채운 뒤 `CALIBRATE` → `SAVE` → `COMMIT`한다.
기본 보정 파일 경로는 다음이다.

```text
~/.ros/camera_info/ceiling_camera.yaml
```

실제 파일 경로가 다르면 launch에 URL을 직접 지정한다.

```bash
camera_info_url:=file:///home/USER/.ros/camera_info/ceiling_camera.yaml
```

## 4. 배 태그와 노트북 검출 실행

- 태그 패밀리: `tag36h11`
- 기본 ID: `0`
- `tag_size`: 검은 바깥 테두리 한 변을 자로 잰 실제 길이[m]
- 태그는 배의 수평면과 평행하게 단단히 고정
- 태그 정면과 선수 방향 차이도 기록

예를 들어 ID 0, 실제 한 변 162mm이면 노트북에서 실행한다.

```bash
source /opt/ros/humble/setup.bash
source ~/kaboat_ws/repo/install/setup.bash
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0

ros2 launch kaboat_hardware ceiling_apriltag.launch.py \
  camera:=0 width:=1280 height:=720 \
  tag_id:=0 tag_size:=0.162 tag_frame:=tag36h11:0
```

정상이면 상태 노드에 다음과 비슷한 로그가 나온다.

```text
정상 — tag IDs=[0], 검출 지연=0.0xxs
```

화면을 확인하려면 노트북의 다른 터미널에서 실행한다.

```bash
ros2 run rqt_image_view rqt_image_view /ceiling_cam/image_rect
```

토픽과 TF를 확인한다.

```bash
ros2 topic hz /ceiling_cam/image_rect
ros2 topic echo /apriltag/detections --once
ros2 run tf2_ros tf2_echo ceiling_camera tag36h11:0
```

## 5. Jetson에서 `/odom` 생성

노트북 검출 launch를 켠 상태에서 Jetson에서 실행한다.

```bash
cd /home/msga2026/Desktop/kaboat_ws/repo
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0

ros2 launch kaboat_hardware indoor_tank.launch.py
```

이 launch는 GQ7의 GNSS/EKF `/odom` remap을 끄고 `apriltag_odom`만 `/odom`을
발행하게 한다. 임시 IMU dead-reckoning launch와 동시에 실행하지 않는다.

Jetson에서 다음을 확인한다.

```bash
ros2 topic echo /apriltag/detections --once
ros2 run tf2_ros tf2_echo odom tag36h11:0
ros2 topic hz /odom
ros2 topic echo /odom --once
ros2 topic info /odom --verbose | grep "Publisher count"
```

`Publisher count: 1`이어야 한다. 태그를 가리면 `0.3s` 뒤 `/odom` 발행을
멈추는 것이 정상 안전 동작이다.

## 6. 수조 좌표계 실측

[`indoor_tank.yaml`](src/kaboat_hardware/config/indoor_tank.yaml)에 실제값을
반영한다.

- `camera_xyz`: 선택한 수조 원점에서 천장 카메라까지 위치[m]
- `camera_rpy`: 카메라 설치 자세[rad]
- `tag_yaw_offset`: 태그 정면과 선수 방향의 차이[rad]
- `tag_offset_xy`: 선체 중심에서 태그 중심까지 오프셋[m]

단순히 카메라가 아래를 본다는 이유로 `[π, 0, 0]`만 넣고 주행하면 안 된다.
천장 브래킷 기울기와 카메라 optical frame 축을 포함해 고정 기준점으로 검증한다.
수조 모서리처럼 좌표를 아는 지점에 배를 차례로 놓고 `/odom` 오차를 기록한 뒤
카메라 외부 파라미터를 보정한다.

## 7. 장애별 판정

| 현상 | 확인할 것 |
|---|---|
| 노트북 영상 없음 | `cam -l`, camera 번호, 다른 앱의 카메라 점유 |
| `카메라 미보정` | 보정 YAML 경로와 `CameraInfo.k` |
| 검출 메시지는 있으나 태그 없음 | ID, 실제 tag size, 조명, 이미지 내 태그 40~60px 이상 |
| 노트북에서는 검출되나 Jetson에서 안 보임 | domain ID, localhost-only, ping, 방화벽, AP isolation |
| `노트북 시각이 미래` | 양쪽 `chronyc tracking` |
| `태그 TF 없음` | `camera_frame`, `tag_frame` 이름과 TF 트리 |
| `/odom` 발행자 2개 | IMU 임시 odom/GQ7 EKF remap 종료 |
