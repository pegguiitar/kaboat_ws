# KABOAT Jetson 실물 센서 bringup

이 브랜치의 첫 단계는 **모터를 구동하지 않고 센서 입력만 확인**하는 것이다.
`real_sensors.launch.py`는 ESC/추력 노드를 실행하지 않는다.

## 센서 토픽 계약

| 장치 | ROS 2 타입 | KABOAT 표준 토픽 | 필수 frame_id 예시 |
|---|---|---|---|
| D455 RGB | `sensor_msgs/Image` | `/camera/color/image_raw` | `camera_color_optical_frame` |
| D455 aligned depth | `sensor_msgs/Image` | `/camera/depth/image_raw` | `camera_color_optical_frame` |
| D455 calibration | `sensor_msgs/CameraInfo` | `/camera/camera_info` | `camera_color_optical_frame` |
| 2D LiDAR | `sensor_msgs/LaserScan` | `/scan` | `laser_link` |
| EBIMU | `sensor_msgs/Imu` | `/imu/data` | `imu_link` |
| GNSS | `sensor_msgs/NavSatFix` | `/gps/fix` | `gps_link` |

TG-50, EBIMU, GNSS 제조사 드라이버가 다른 토픽을 발행하면
[`sensors.yaml`](src/kaboat_hardware/config/sensors.yaml)의 `topic`을 바꾸거나,
드라이버 launch에서 위 표준 이름으로 remap한다. IMU는 단순 토픽명뿐 아니라
축 방향(ROS REP-103 ENU), 단위, orientation covariance를 반드시 확인해야 한다.

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

## LiDAR·IMU·GNSS 연결 후

각 제조사 드라이버를 먼저 실행한 다음 진단 launch를 실행한다. 현재 저장소에는
장치 모델/프로토콜이 확정되지 않은 드라이버를 임의로 포함하지 않았다.

```bash
ros2 topic list
ros2 topic hz /scan
ros2 topic hz /imu/data
ros2 topic hz /gps/fix
ros2 topic echo /scan --once
ros2 topic echo /imu/data --once
ros2 topic echo /gps/fix --once
```

정상 판정은 단순히 토픽이 존재하는지가 아니라 다음을 모두 본다.

- 설정한 최소 수신률 이상인지
- 마지막 메시지가 timeout보다 오래되지 않았는지
- `header.frame_id`가 비어 있지 않은지
- 이미지/스캔이 비어 있지 않은지
- IMU quaternion norm이 유효하고 orientation이 제공되는지
- GNSS fix 상태와 좌표가 유효한지

## 자율주행 스택 안전 기본값

`autonomy.launch.py`는 `use_sim_time:=false`, `use_sim_actuator:=false`가 기본이다.
따라서 실물에서 실행해도 Gazebo용 `twist2thrust.py`는 뜨지 않는다. 아직 실제
ESC 드라이버는 연결하지 않았으므로 센서 검증이 끝나기 전에는 별도 모터 노드를
추가하지 않는다.

시뮬레이터에서 기존 동작을 재현할 때만 명시적으로 켠다.

```bash
ros2 launch kaboat_bringup autonomy.launch.py \
  use_sim_time:=true use_sim_actuator:=true
```
