# KABOAT Jetson 실물 센서 bringup

이 브랜치의 첫 단계는 **모터를 구동하지 않고 센서 입력만 확인**하는 것이다.
`real_sensors.launch.py`는 ESC/추력 노드를 실행하지 않는다.

> 현장 점검 명령과 판정 기준은 **[SENSOR_CHECK.md](SENSOR_CHECK.md)** 에 모아뒀다.
> 실내 수조(천장 AprilTag) 모드는 아래 §"실내 수조 시험" 참고.

## 센서 토픽 계약

| 장치 | ROS 2 타입 | KABOAT 표준 토픽 | 필수 frame_id 예시 |
|---|---|---|---|
| D455 RGB | `sensor_msgs/Image` | `/camera/color/image_raw` | `camera_color_optical_frame` |
| D455 aligned depth | `sensor_msgs/Image` | `/camera/depth/image_raw` | `camera_color_optical_frame` |
| D455 calibration | `sensor_msgs/CameraInfo` | `/camera/camera_info` | `camera_color_optical_frame` |
| 2D LiDAR | `sensor_msgs/LaserScan` | `/scan` | `laser_link` |
| GQ7 IMU | `sensor_msgs/Imu` | `/imu/data` | `imu_link` |
| GQ7 GNSS 1 | `sensor_msgs/NavSatFix` | `/gps/fix` | `gnss_1_antenna_link` |
| GQ7 GNSS 2 | `sensor_msgs/NavSatFix` | `/gps/fix_secondary` | `gnss_2_antenna_link` |
| GQ7 onboard INS | `nav_msgs/Odometry` | `/odom` | `map` |

TG-50 등 제조사 드라이버가 다른 토픽을 발행하면
[`sensors.yaml`](src/kaboat_hardware/config/sensors.yaml)의 `topic`을 바꾸거나,
드라이버 launch에서 위 표준 이름으로 remap한다. GQ7은 공식 드라이버의
`/gnss_1/llh_position`과 `/ekf/odometry_map`을 각각 `/gps/fix`, `/odom`으로
자동 remap한다. IMU는 원래 `/imu/data`여서 그대로 사용한다. IMU는 토픽명뿐 아니라
축 방향(ROS REP-103 ENU), 단위, orientation covariance를 반드시 확인해야 한다.

## 2026-08-10 Jetson 연결 확인 결과

| 장치 | USB 식별자/장치 노드 | 확인 결과 |
|---|---|---|
| Intel RealSense D455 | `8086:0b5c` | USB 3.x, `5000M`로 연결 확인 |
| TG-50 2D LiDAR | `10c4:ea60`, `/dev/ttyUSB0` | CP210x 시리얼, `512000` baud에서 TG50 모델 응답 및 health 정상 확인 |
| HBK MicroStrain 3DM-GQ7-GNSS/INS | `0483:5740`, `/dev/ttyACM0` | `cdc_acm` 드라이버, 모델 `3DM-GQ7`, serial `6284.176046`, firmware `1.1.04` 확인 |

GQ7 공식 드라이버 실측 결과 `/imu/data`는 약 `100 Hz`, `/gps/fix`는
약 `2 Hz`로 수신됐다. 실내 시험에서는 GNSS 메시지의 status가 no-fix일 수
있으며, 이는 USB/드라이버 인식 실패와는 다르다. `/odom`은 ROS 토픽 이름과
타입(`nav_msgs/Odometry`)까지 생성되는 것을 확인했지만, 현재 GQ7 장치에 저장된
출력 구성에서는 EKF 스트림이 나오지 않아 메시지 수신은 0건이었다. 안테나 위치와
장착 자세를 실측한 뒤 EKF 출력 설정을 안전하게 적용하는 작업이 남아 있다.

USB와 장치 노드는 다음처럼 다시 확인한다.

```bash
lsusb
lsusb -t
ls -l /dev/ttyUSB* /dev/ttyACM* /dev/microstrain* 2>/dev/null
udevadm info --query=property --name=/dev/ttyACM0
```

## 빌드

```bash
cd /home/msga2026/Desktop/kaboat_ws/repo
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

`realsense2_camera`가 아직 설치되지 않았다면 D455 드라이버를 끈 기본 실행은
가능하다. 단, 전체 `rosdep` 단계에서는 해당 패키지를 설치하려고 시도한다.

## 센서 연결 전 코드 확인

```bash
ros2 launch kaboat_hardware real_sensors.launch.py
```

10초의 시작 유예시간 뒤 연결되지 않은 센서는 `no messages`로 표시된다. 이는
현재 단계에서 정상이다. 같은 상태가 `/diagnostics`에도 발행된다.

```bash
ros2 topic echo /diagnostics
```

## D455 연결 후

```bash
ros2 launch kaboat_hardware real_sensors.launch.py enable_d455:=true
```

초기값은 Jetson의 USB/CPU 부하를 낮추기 위해 RGB/depth `640x480@15Hz`,
point cloud 비활성이다. 필요하면 다음처럼 바꾼다.

```bash
ros2 launch kaboat_bringup d455.launch.py \
  color_profile:=640x480x30 depth_profile:=640x480x30 \
  enable_pointcloud:=true
```

## GQ7 연결 후

공식 ROS 2 드라이버를 설치한다.

```bash
sudo apt-get update
sudo apt-get install -y ros-humble-microstrain-inertial-driver
```

재연결하면 udev rule이 GQ7 main port를 `/dev/microstrain_main`으로 만든다.
드라이버를 설치할 때 GQ7이 이미 연결돼 있었다면 규칙 적용 이벤트가 발생하지
않아 별칭이 아직 없을 수 있다. 이때는 GQ7 USB 케이블을 한 번 뺐다가 다시 꽂고
아래 명령으로 확인한다.

```bash
ls -l /dev/ttyACM0 /dev/microstrain_main
```

실물 센서 launch는 GQ7 드라이버를 기본으로 실행하며 모터는 실행하지 않는다.

```bash
ros2 launch kaboat_hardware real_sensors.launch.py
```

[`gq7.yaml`](src/kaboat_hardware/config/gq7.yaml)은 초기 검증 중 장치의 기존
보정값을 보호하기 위해 `device_setup: false`, `save_settings: false`로 두었다.
GQ7 설치 방향과 두 GNSS 안테나의 lever arm을 실측하기 전에는 이 값을
`true`로 바꾸거나 예시 오프셋을 입력하지 않는다.

토픽 매핑과 실제 수신은 다음 명령으로 확인한다.

```bash
ros2 topic list
ros2 topic hz /imu/data
ros2 topic hz /gps/fix
ros2 topic hz /odom
ros2 topic echo /imu/data --once
ros2 topic echo /gps/fix --once
ros2 topic echo /odom --once
```

현재 적용한 remap은 다음과 같다.

| GQ7 공식 드라이버 출력 | KABOAT 입력 |
|---|---|
| `/imu/data` | `/imu/data` (변경 없음) |
| `/gnss_1/llh_position` | `/gps/fix` |
| `/gnss_2/llh_position` | `/gps/fix_secondary` |
| `/ekf/odometry_map` | `/odom` |

정상 판정은 단순히 토픽이 존재하는지가 아니라 다음을 모두 본다.

- 설정한 최소 수신률 이상인지
- 마지막 메시지가 timeout보다 오래되지 않았는지
- `header.frame_id`가 비어 있지 않은지
- 이미지/스캔이 비어 있지 않은지
- IMU quaternion norm이 유효하고 orientation이 제공되는지
- GNSS fix 상태와 좌표가 유효한지
- GQ7 INS odometry의 위치/속도가 유한하고 child frame이 존재하는지

## 실내 수조 시험 (천장 AprilTag)

실내는 GNSS가 물리적으로 안 잡혀 GQ7 EKF `/odom`을 쓸 수 없다. 대신 천장에
고정한 카메라로 배 위의 AprilTag를 추적해 `/odom`을 만든다.
같은 Wi-Fi를 통한 노트북→Jetson 연결과 웹캠 보정 절차는
[`APRILTAG_WIFI_SETUP.md`](APRILTAG_WIFI_SETUP.md)에 정리돼 있다.

AprilTag 설치 전 TG-50와 Occupancy Grid 파이프라인만 짧게 시험할 때는 다음
임시 IMU dead-reckoning launch를 쓴다. 가속도 이중 적분이라 빠르게 드리프트하며
모터 주행용이 아니다. 시작 후 2초간 센서를 움직이지 말아야 한다.

```bash
ros2 launch kaboat_hardware imu_tg50_mapping.launch.py
```

세부 판정 및 리셋 명령은 [`SENSOR_CHECK.md`](SENSOR_CHECK.md)의
`GPS/AprilTag 전 임시 IMU dead-reckoning 시험` 절을 따른다.

```bash
# 천장 카메라 노트북 — tag_size는 인쇄한 태그 실측값으로 변경
ros2 launch kaboat_hardware ceiling_apriltag.launch.py \
  tag_id:=0 tag_size:=0.162

# 배 (Jetson)
ros2 launch kaboat_hardware indoor_tank.launch.py
```

필드별로 출처가 다르다 — 각 센서가 제일 잘하는 것만 취한다.

| `/odom` 필드 | 출처 | 이유 |
|---|---|---|
| `pose.position` / `orientation` | AprilTag | 절대 측정, 드리프트 없음 |
| `twist.angular.z` | **GQ7 자이로** | 직접 측정. yaw 미분은 각도 오차 1°가 0.52 rad/s(=실측 ω_max)로 증폭돼 D항을 포화시킨다 |
| `twist.linear.x/y` | AprilTag 위치 미분 | 가속도 적분은 자세 오차 1°만 있어도 10초에 1.7 m/s 발산 |

속도 노이즈는 5mm 검출 오차 기준 실측(`test_pose_velocity.py`)으로
window 1프레임 0.205 m/s → window 0.15s 0.042 → +EMA **0.011 m/s**.

`ceiling camera → tag` TF만 apriltag_ros가 발행하고, `odom → camera` 설치
자세는 `apriltag_odom`이 정적 TF로 낸다 — tf2가 합성을 대신하므로 노드는
행렬 계산을 하지 않는다.

### 준비물

- **카메라 캘리브레이션 필수** — pose는 `camera_info` 내부 파라미터로 계산된다.
  `ros2 run camera_calibration cameracalibrator ...`
- **왜곡 보정 이미지(`image_rect`)** — 광각 렌즈면 수조 가장자리 오차가 크다.
- 태그 패밀리 `tag36h11`, 이미지에서 한 변 최소 40~60 px.
  **인쇄물을 자로 재서 그 값을 `size`에 넣는다** (프린터 배율 오차가 거리
  오차에 그대로 비례).

### 외부 USB 웹캠(천장 카메라 PC) 빠른 시작

레포에는 UVC 외부 웹캠 → 왜곡 보정 → AprilTag 검출을 한 번에 올리는 launch가
있다. USB A 포트에 연결한 뒤 실제 장치 번호를 확인한다. 내장 웹캠이 있으면
외부 카메라는 보통 `/dev/video2`부터 잡히지만, **반드시 아래 명령 결과를 쓴다.**

```bash
cd ~/kaboat_ws                 # 이 레포의 실제 경로로 바꾼다
./scripts/install_apriltag_dependencies.sh
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select kaboat_hardware
source install/setup.bash

v4l2-ctl --list-devices
```

먼저 외부 카메라를 보정한다. 보정 중에는 체커보드를 여러 거리·각도에서 화면에
채우고, 결과 yaml 파일을 보관한다.

```bash
ros2 run v4l2_camera v4l2_camera_node --ros-args \
  -r __ns:=/ceiling_cam -p video_device:=/dev/video2
ros2 run camera_calibration cameracalibrator --size 8x6 --square 0.024 \
  image:=/ceiling_cam/image_raw camera:=/ceiling_cam
```

`--size`와 `--square`는 **사용한 체커보드의 내부 코너 수와 실제 한 칸 길이[m]**로
바꾼다. 저장된 보정 yaml의 경로와 태그의 실제 검은 테두리 한 변[m]을
`src/kaboat_hardware/config/ceiling_apriltag.yaml`의 `size`·`tag.sizes`에 같은
값으로 반영한 뒤 다시 빌드한다.

```bash
ros2 launch kaboat_hardware ceiling_apriltag.launch.py \
  video_device:=/dev/video2 \
  camera_info_url:=file:///home/$USER/.ros/camera_info/ceiling.yaml
```

정상이라면 `/ceiling_cam/image_rect`, `/detections`, 그리고
`ceiling camera optical frame → tag36h11:0` TF가 나온다. 이 PC와 배 Jetson은
같은 네트워크·`ROS_DOMAIN_ID`여야 하며, 두 시스템의 시간이 동기화돼야 한다.

### ⚠️ `/odom` 발행자 중복

`indoor_tank.launch.py`는 GQ7의 EKF→`/odom` remap을 자동으로 끈다
(`enable_odom_remap:=false`). `real_sensors.launch.py`를 직접 쓸 때는 수동으로
꺼야 한다 — 안 끄면 EKF가 수렴하는 순간 발행자가 둘이 되어 두 좌표계가 섞인다.
GNSS 미수렴 중에는 조용해서 드러나지 않으니 주의.

### ⚠️ 파라미터 스케일

현재 값들은 실제 경기장(수십 m, 전속 1.48 m/s) 기준이라 실내 수조에서는
전부 과대하다. `d_panic`(3.0m)·`escape_radius`(2.5m)·`lookahead`(2.0m)·
`min_horizon`(3.0m)·`transition_radius`(2.0m)·occupancy_grid `size`(20m)와
경기장 좌표 waypoint를 수조 실측값으로 재산출해야 한다. 목록은
[`indoor_tank.yaml`](src/kaboat_hardware/config/indoor_tank.yaml) 하단에 있다.

또 **수조 벽이 LiDAR에 전부 장애물로 잡혀** 격자가 사방으로 막힌다. avoid
플래너는 "전진 반평면에 답 없음" → ESCAPE(후진)로 갈 것이다. 회피 시험은
이걸 감안해 파라미터를 잡은 뒤에 한다.

---

## 실물 스러스터(모터/ESC) 제어 드라이버

실물 모터 구동 노드는 `kaboat_hardware/thruster_driver`를 사용한다.
`/cmd_vel`(`[-1.0, 1.0]`)을 구독하여 차동 구동 좌/우 PWM(1000~2000µs)을 생성하고 하드웨어로 전달한다.

### 1) 모터 드라이버 실행

```bash
# 1-1. 하드웨어 미연결 벤치/더미 테스트
ros2 launch kaboat_hardware thrusters.launch.py hardware_type:=dummy

# 1-2. 아두이노/ESP32 USB 시리얼 연결 (<PWM_L,PWM_R>\n 형식)
ros2 launch kaboat_hardware thrusters.launch.py hardware_type:=serial

# 1-3. Jetson I2C 버스 직결 PCA9685 16채널 PWM 모듈
ros2 launch kaboat_hardware thrusters.launch.py hardware_type:=pca9685
```

### 2) 안전 기능 및 수동 조종 오버라이드
- **300ms 워치독**: `/cmd_vel` 수신이 0.3초 이상 끊기면 자동으로 1500µs(중립/정지) 전송.
- **RC Manual Override**: RC 수신기 수동 조종 토픽(`/rc/cmd_vel`) 수신 시 자율주행 명령을 즉시 무시하고 수동 조종 우선 적용.
- **비상 정지(E-Stop)**: `/emergency_stop`(`std_msgs/Bool`, `data: true`) 수신 시 즉시 PWM 중립 차단.
- **가속도 제한(Slew Rate)**: `max_slew_rate`(기본 2.0/s)로 급가속에 의한 전압 강하 및 요 발진/전복 방지.
- **불감대(Deadband)**: `deadband_us`(기본 ±25µs)로 ESC 불감대를 건너뛰어 저속 제어성 확보.

---

## 자율주행 스택 안전 기본값

`autonomy.launch.py`는 `use_sim_time:=false`, `use_sim_actuator:=false`가 기본이다.
따라서 실물에서 실행해도 Gazebo용 `twist2thrust.py`는 뜨지 않는다. 실물 주행 시에는
위 `thrusters.launch.py`를 함께 실행하여 모터를 제어한다.

시뮬레이터에서 기존 동작을 재현할 때만 명시적으로 켠다.

```bash
ros2 launch kaboat_bringup autonomy.launch.py \
  use_sim_time:=true use_sim_actuator:=true
```

