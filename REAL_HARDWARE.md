# KABOAT Jetson 실물 센서 bringup

이 브랜치의 첫 단계는 **모터를 구동하지 않고 센서 입력만 확인**하는 것이다.
`real_sensors.launch.py`는 ESC/추력 노드를 실행하지 않는다.

> 현장 점검 명령과 판정 기준은 **[SENSOR_CHECK.md](SENSOR_CHECK.md)** 에 모아뒀다.
> 실내 수조(외부 TG-50 라이다 + 선체 GQ7 IMU) 모드는 아래 §"실내 수조 시험" 참고.

## 센서 토픽 계약

| 장치 | ROS 2 타입 | KABOAT 표준 토픽 | 필수 frame_id 예시 |
|---|---|---|---|
| D455 RGB | `sensor_msgs/Image` | `/camera/color/image_raw` | `camera_color_optical_frame` |
| D455 aligned depth | `sensor_msgs/Image` | `/camera/depth/image_raw` | `camera_color_optical_frame` |
| D455 calibration | `sensor_msgs/CameraInfo` | `/camera/camera_info` | `camera_color_optical_frame` |
| 2D LiDAR | `sensor_msgs/LaserScan` | `/scan` | `laser_frame` |
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

새 터미널에서는 반드시 ROS 2 기본 환경과 워크스페이스 `install` 환경을 모두 source해야 한다. (`install/setup.bash` 누락 시 `Package 'kaboat_hardware' not found` 오류 발생)

```bash
cd /home/msga2026/Desktop/kaboat_ws/repo
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=42
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

## 실내 수조 시험 (외벽 TG-50 + 선체 GQ7·TG-50)

실내에서는 GNSS를 사용할 수 없어 외벽 TG-50의 `/boat_pose`와 선체 GQ7 IMU로 `/odom`을 만듭니다. `indoor_tank.launch.py`가 실외용 GQ7 EKF `/odom` remap과 TF broadcaster를 끄므로 실내에서 `real_sensors.launch.py`를 따로 실행하면 안 됩니다. 선체 TG-50을 쓰면 `/scan/raw`에서 배 구조물이 보이는 220°~320°를 가린 `/scan`을 지도·회피에 사용합니다. 고정 라이다 `/shore/scan`에는 이 필터가 적용되지 않습니다.

**노트북·젯슨 빌드, 포트 선택, IMU 보정, 고정 B-Spline 추종, 장애물 회피 미리보기·주행, RViz, 출발·정지는 [TANK_TESTS_GUIDE.md](TANK_TESTS_GUIDE.md)에 모았습니다.** 이 절차의 설치 좌표와 선체 폭·라이다 원점 변환은 현장 실측값으로 확인해야 합니다. 선체 TG-50의 최소 거리 설정은 [`tg50_onboard.yaml`](src/kaboat_hardware/config/tg50_onboard.yaml), 실내 오도메트리 설정은 [`indoor_tank.yaml`](src/kaboat_hardware/config/indoor_tank.yaml)에 있습니다.

외벽 라이다 추적 전 센서만 짧게 확인할 때의 임시 IMU dead reckoning 절차는 [SENSOR_CHECK.md](SENSOR_CHECK.md) §11을 참조하세요. 가속도 이중 적분으로 빠르게 드리프트하므로 모터 주행용 오도메트리로 사용하지 않습니다.

---

## 실물 스러스터(모터/ESC) 제어 드라이버

실물 모터 구동 노드는 `kaboat_hardware/thruster_driver`를 사용한다.
`/cmd_vel`(`[-1.0, 1.0]`)을 구독하여 현재 설정 기준 차동 구동 좌/우
PWM(1100~1900µs, 중립 1500µs)을 생성하고 하드웨어로 전달한다.

### 1) 모터 드라이버 실행

```bash
# 1-1. 하드웨어 미연결 벤치/더미 테스트
ros2 launch kaboat_hardware thrusters.launch.py hardware_type:=dummy

# 1-2. 아두이노/ESP32 USB 시리얼 연결 — 실제 포트를 명시
ls -l /dev/serial/by-id/ /dev/ttyUSB* /dev/ttyACM* 2>/dev/null
ros2 launch kaboat_hardware thrusters.launch.py \
  hardware_type:=serial port:=/dev/ttyUSB0 allow_port_scan:=false

# 1-3. Jetson I2C 버스 직결 PCA9685 16채널 PWM 모듈
ros2 launch kaboat_hardware thrusters.launch.py hardware_type:=pca9685
```

### 2) 안전 기능 및 수동 조종 오버라이드

- **300ms 워치독**: `/cmd_vel` 수신이 0.3초 이상 끊기면 자동으로 1500µs(중립/정지) 전송.
- **RC Manual Override**: RC 수신기 수동 조종 토픽(`/rc/cmd_vel`) 수신 시 자율주행 명령을 즉시 무시하고 수동 조종 우선 적용.
- **비상 정지(E-Stop)**: `/emergency_stop`(`std_msgs/msg/Bool`, `data: true`) 수신 시 즉시 PWM 중립 차단.
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
