# 노트북 AprilTag 검출 단독 확인 절차

오늘 목표는 배를 움직이거나 Jetson에서 `/odom`을 만들지 않고, 다른 노트북의
천장 웹캠으로 AprilTag가 정상 검출되고 카메라 기준 3차원 위치가 계산되는지만
확인하는 것이다.

```text
웹캠 영상 확인
  → 카메라 내부 파라미터 보정
  → tag36h11 ID 0 검출
  → camera → tag TF 위치 변화 확인
```

Jetson, GQ7, TG-50, 임시 IMU odom, `indoor_tank.launch.py`는 오늘 실행하지
않는다. 아래 명령은 Ubuntu 22.04 + ROS 2 Humble 기준이다.

## 1. 필요한 패키지 설치

```bash
source /opt/ros/humble/setup.bash

sudo apt update
sudo apt install -y \
  git \
  v4l-utils \
  ros-humble-camera-ros \
  ros-humble-image-proc \
  ros-humble-apriltag-ros \
  ros-humble-camera-calibration \
  ros-humble-rqt-image-view
```

## 2. `real` 브랜치 clone 및 빌드

처음 받는 노트북에서는 다음을 실행한다.

```bash
mkdir -p ~/kaboat_ws
cd ~/kaboat_ws

git clone --branch real \
  https://github.com/pegguiitar/kaboat_ws.git repo

cd repo
source /opt/ros/humble/setup.bash

colcon build --symlink-install \
  --packages-select kaboat_hardware

source install/setup.bash
```

이미 clone했다면 최신 `real`을 받고 다시 빌드한다.

```bash
cd ~/kaboat_ws/repo

git switch real
git pull origin real

source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select kaboat_hardware
source install/setup.bash
```

## 3. 웹캠과 태그 준비

웹캠을 연결하고 장치 목록을 확인한다.

```bash
v4l2-ctl --list-devices

source /opt/ros/humble/setup.bash
cam -l
```

오늘 기준 태그는 다음과 같다.

```text
family: tag36h11
ID: 0
```

태그의 흰 여백을 제외한 **바깥 검은 사각형 한 변**을 자로 잰다. 예를 들어
실측 길이가 162mm라면 launch 인자는 `tag_size:=0.162`다. 태그 크기 오차는
카메라가 계산하는 거리의 비율 오차로 그대로 이어진다.

## 4. 웹캠 화면 확인

첫 번째 터미널에서 검출기를 끄고 카메라만 실행한다.

```bash
cd ~/kaboat_ws/repo

source /opt/ros/humble/setup.bash
source install/setup.bash

export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0

ros2 launch kaboat_hardware ceiling_apriltag.launch.py \
  camera:=0 \
  width:=1280 \
  height:=720 \
  enable_detector:=false \
  enable_status:=false
```

두 번째 터미널에서 원본 화면을 띄운다.

```bash
source /opt/ros/humble/setup.bash

ros2 run rqt_image_view rqt_image_view \
  /ceiling_cam/camera/image_raw
```

영상 주기도 확인한다.

```bash
ros2 topic hz /ceiling_cam/camera/image_raw
```

영상이 안 나오면 첫 번째 launch의 `camera:=0`을 `camera:=1`로 바꿔 다시
확인한다. Zoom, 브라우저 등 다른 프로그램이 웹캠을 점유하고 있으면 종료한다.

## 5. 카메라 내부 파라미터 보정

기존 보정 파일을 확인한다.

```bash
ls ~/.ros/camera_info/ceiling_camera.yaml
```

파일이 없으면 4절의 카메라 launch를 실행한 상태에서 다른 터미널에 다음을
입력한다. 아래 `8x6`은 체스보드 내부 코너 수, `0.030`은 한 칸 길이 3cm의
예시다. 실제 체스보드 규격으로 변경해야 한다.

```bash
source /opt/ros/humble/setup.bash

ros2 run camera_calibration cameracalibrator \
  --size 8x6 \
  --square 0.030 \
  image:=/ceiling_cam/camera/image_raw \
  camera:=/ceiling_cam/camera
```

체스보드를 화면 중앙·가장자리·여러 거리와 각도에서 천천히 움직여 샘플을
채운 뒤 GUI에서 다음 순서로 누른다.

```text
CALIBRATE → SAVE → COMMIT
```

완료 후 카메라 launch를 `Ctrl+C`로 종료한다.

> 체스보드 `--square`는 카메라 렌즈 보정용 크기이고, 아래의 `tag_size`는
> AprilTag 거리 계산용 크기다. 서로 다른 값이다.

## 6. AprilTag 검출 실행

첫 번째 터미널에서 실행한다. `tag_size`는 반드시 인쇄물을 재서 바꾼다.

```bash
cd ~/kaboat_ws/repo

source /opt/ros/humble/setup.bash
source install/setup.bash

export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0

ros2 launch kaboat_hardware ceiling_apriltag.launch.py \
  camera:=0 \
  width:=1280 \
  height:=720 \
  tag_id:=0 \
  tag_size:=0.162 \
  tag_frame:=tag36h11:0
```

정상 검출 시 launch 터미널에 다음과 비슷한 상태 로그가 나온다.

```text
정상 — tag IDs=[0], 검출 지연=0.0xxs
```

## 7. 영상·검출·TF 확인

두 번째 터미널에서 왜곡 보정 영상을 띄운다.

```bash
source /opt/ros/humble/setup.bash

ros2 run rqt_image_view rqt_image_view \
  /ceiling_cam/image_rect
```

이 화면은 보정된 원본 영상이며 검출 사각형을 덧그리지 않는다. 실제 검출
여부는 다음 명령으로 확인한다.

```bash
ros2 topic echo /apriltag/detections
```

태그가 보일 때 다음 필드가 나와야 한다.

```text
family: 36h11
id: 0
hamming: 0
decision_margin: ...
```

카메라 기준 태그의 3차원 위치와 자세를 확인한다.

```bash
ros2 run tf2_ros tf2_echo \
  ceiling_camera tag36h11:0
```

태그를 좌우·앞뒤로 천천히 움직였을 때 `Translation` 값이 연속적으로 변하면
미터 단위 위치 계산까지 정상이다. 태그를 가렸다가 다시 보여 상태 로그가
`태그 미검출`에서 `정상`으로 복귀하는지도 확인한다.

## 8. 오늘의 통과 기준

- `/ceiling_cam/camera/image_raw` 영상이 지속적으로 보인다.
- `/ceiling_cam/image_rect`가 정상 발행된다.
- 상태 로그에 `tag IDs=[0]`이 나온다.
- `/apriltag/detections`에 `family: 36h11`, `id: 0`, `hamming: 0`이 나온다.
- `ceiling_camera → tag36h11:0` TF가 나오고 태그 이동에 따라 변한다.
- 태그를 가리면 미검출, 다시 보여주면 정상으로 복귀한다.

여기까지 확인되면 노트북 단독 AprilTag 검출 시험은 완료다. Jetson Wi-Fi 연결,
수조 좌표계 외부 보정, GQ7 자이로 결합과 `/odom` 생성은 다음 단계에서 진행한다.

전체 노트북→Jetson 연동 절차는
[`APRILTAG_WIFI_SETUP.md`](APRILTAG_WIFI_SETUP.md)를 참고한다.
