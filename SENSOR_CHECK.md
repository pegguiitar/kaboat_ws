# 센서 토픽 점검 명령 모음

현장에서 센서가 제대로 들어오는지 확인하는 명령과 판정 기준. 배선·bringup
절차는 [REAL_HARDWARE.md](REAL_HARDWARE.md), 스택 구조는
[SKELETON.MD](SKELETON.MD) 를 본다.

> **판정 원칙**: "토픽이 있다" 는 합격이 아니다. ① 발행률 ② 최신성
> ③ frame_id ④ 데이터 유효성 ⑤ **부호·방향** 까지 봐야 한다.
> 특히 ⑤ 는 모터를 붙이기 전에 확인해야 한다 — 부호가 틀리면 전복한다.

---

## 0. 매 터미널 준비

```bash
source /opt/ros/humble/setup.bash
source ~/kaboat_ws/install/setup.bash      # 실제 워크스페이스 경로로
```

여러 PC(천장 카메라 PC ↔ Jetson)를 쓰면 양쪽이 같아야 한다:

```bash
echo $ROS_DOMAIN_ID          # 두 기기가 같은 값이어야 서로 보인다
ros2 node list               # 상대편 노드가 보이는지로 확인
```

---

## 1. ⚠️ 가장 먼저 — QoS 함정

**`echo` 나 `hz` 가 조용한데 드라이버는 멀쩡한 경우가 제일 흔하다.**

센서 드라이버는 보통 `BEST_EFFORT` 로 발행하는데 `ros2 topic echo` 기본값은
`RELIABLE` 이라 서로 매칭되지 않는다. 이때는 아무 에러 없이 그냥 조용하다.

```bash
# 1) 발행자가 있는지부터 (QoS 무관하게 보인다)
ros2 topic info /scan --verbose

# 2) Publisher count 가 1 이상인데 echo 가 조용하면 QoS 미스매치
ros2 topic echo /scan --once --no-arr --qos-reliability best_effort
ros2 topic hz /scan --qos-reliability best_effort
```

`Publisher count: 0` 이면 드라이버가 안 뜬 것이고, `1` 이상인데 조용하면
QoS 문제다. 이 둘을 먼저 가른다.

---

## 2. 전체 한눈에 보기

```bash
ros2 topic list
ros2 node list
```

진단 노드를 띄우면 8개 센서를 2초마다 한 줄로 요약해준다. **현장에서는
이걸 한 터미널에 계속 띄워두는 게 가장 편하다.**

```bash
ros2 launch kaboat_hardware real_sensors.launch.py
```

```
[sensor_health_monitor]: color:no messages(0.0Hz) | depth:no messages(0.0Hz) |
camera_info:no messages(0.0Hz) | scan:receiving(9.9Hz) | imu:receiving(99.8Hz) |
gps:receiving with warning(2.0Hz) | odom:no messages(0.0Hz) | pointcloud:disabled(0.0Hz)
```

상세는 `/diagnostics`:

```bash
ros2 topic echo /diagnostics
```

`level` 은 바이트로 나온다 — **`"\0"`=OK, `"\x01"`=WARN, `"\x02"`=ERROR**.

| message | 뜻 | 조치 |
|---|---|---|
| `receiving` | 정상 | — |
| `waiting for first message` | 시작 후 10초 유예 중 | 기다린다 |
| `no messages` | 유예 지나도 0건 | **드라이버/배선 확인** |
| `stale` | 오다가 끊김 | USB 접촉, 대역폭 |
| `rate below minimum` | 느림 | 드라이버 설정 or `sensors.yaml` 기준 조정 |
| `empty frame_id` | frame 미설정 | 드라이버 파라미터 |
| `receiving with warning` | 데이터 이상 | 아래 센서별 항목 참조 |

판정 기준값은 [`sensors.yaml`](src/kaboat_hardware/config/sensors.yaml) 에서
바꾼다.

---

## 3. `/scan` — 2D LiDAR (TG-50)

### 3.1 형식

`sensor_msgs/msg/LaserScan`. **각 원소가 그 방향으로 쏜 레이저의 거리[m]** 이고,
**방향은 배열 인덱스에 들어있다**:

```
angle_i = angle_min + angle_increment × i
x = r·cos(angle_i)      # 배 기준 전방
y = r·sin(angle_i)      # 배 기준 좌측
```

| 인덱스 (2000개 기준) | 각도 | 방향 |
|---|---|---|
| `ranges[0]` | −180° | 정후방 |
| `ranges[500]` | −90° | 우현 |
| **`ranges[1000]`** | **0°** | **정면** |
| `ranges[1500]` | +90° | 좌현 |

`.inf` 는 "무한히 멀다" 가 아니라 **"반사가 안 돌아왔다"** — 하늘, 잔잔한
수면, `range_max` 초과가 전부 여기다. 정상 스캔에도 섞여 있는 게 당연하다.

### 3.2 명령

```bash
# 헤더만 (2000개 배열 접기) — ★ 제일 먼저 이걸 본다
ros2 topic echo /scan --once --no-arr

# 거리값
ros2 topic echo /scan --once --field ranges | head -20

# 정면(인덱스 1000) 근처만 — sed 는 1-based
ros2 topic echo /scan --once --field ranges | sed -n '995,1005p'

ros2 topic hz /scan
```

### 3.3 기대값 (TG-50, `wamv_kaboat.xacro` 의 데이터시트 기준)

| 필드 | 기대값 |
|---|---|
| `ranges` 길이 | 2000 |
| `angle_min` / `angle_max` | −3.14159 / +3.14159 |
| `angle_increment` | ≈ 0.00314 (0.18°) |
| `range_min` / `range_max` | 0.05 / 50.0 |
| `frame_id` | `laser_link` |
| 주파수 | 10 Hz (`min/max` 가 0.099~0.101 로 붙어야 정상) |

### 3.4 정상 신호의 특징

- **인접 빔끼리 값이 연속적** — 0.18° 간격이라 같은 물체는 여러 빔에 잡힌다.
  매 빔 무작위로 튀면 노이즈/통신 오류.
- 실내면 벽까지 거리(수 m)가 나온다.
- **전부 `.inf` 거나 전부 `0.0`** → 회전만 하고 측정 실패.
  `/diagnostics` 에 `valid_ranges: 0`, `data_check: no valid returns` 로 뜬다.

### 3.5 ⚠️ 각도 부호 확인 (거울상 방지)

코드는 REP-103(x 전방, **반시계 +**)을 전제로 `wa = yaw + angle` 을 쓴다.
드라이버 규약이 반대면 **점유격자가 좌우 거울상**이 된다.

> 정면에 물체를 두고 인덱스 1000 근처 값이 실제 거리와 맞는지 확인 →
> 물체를 **왼쪽으로 옮겼을 때 1000보다 큰 인덱스로 이동**하면 정상.
> 작은 인덱스로 가면 좌우 반전 상태다.

### 3.6 알아둘 것

- **2D 라 높이 정보가 없다.** 라이다 장착 높이의 수평면 하나만 훑는다.
  그 위/아래 물체는 배열에 아예 안 나타난다.
- **장착 오프셋 미보정** — `occupancy_grid` 는 TF 를 안 쓰고
  `lidar_link == base_link` 로 가정한다. 실물 오프셋을 반영하려면 코드를
  같이 고쳐야 한다.

---

## 4. `/imu/data` — IMU (GQ7 자이로)

### 4.1 명령

```bash
ros2 topic hz /imu/data                                    # ≈100 Hz
ros2 topic echo /imu/data --once --field angular_velocity
ros2 topic echo /imu/data --once --field orientation
```

### 4.2 ⚠️ 부호 확인 — **모터 붙이기 전 필수**

```bash
ros2 topic echo /imu/data --field angular_velocity.z
```

> **반시계(좌회전)로 돌렸을 때 값이 양수여야 한다.**

부호가 반대면 `seek_goal()` 의 D항이 감쇠가 아니라 **발진 방향으로 작용**한다
— SKELETON §6 에 기록된 전복 사고 2건 중 하나가 정확히 요 발진이다.
반대면 `apriltag_odom` 의 `yaw_rate_sign: -1.0` 으로 뒤집는다.

### 4.3 바이어스 확인

가만히 두고 `angular_velocity.z` 를 본다. **0.01 rad/s 이하면 무시해도 된다**
(D항 기여가 `max_angular` 의 1.5% 수준). 크면 `gyro_bias_z` 에 넣어 뺀다.

### 4.4 `orientation unavailable` WARN 은 실내에서 정상

GQ7 은 자력계가 없고 절대 방위를 듀얼 안테나 GNSS 로 얻는다. **실내에서는
orientation 이 없거나 못 믿는 게 정상**이고 `/diagnostics` 에 WARN 이 뜬다.

실내 수조 모드에서는 방위를 AprilTag 가 주므로 이 WARN 은 무시해도 된다.
**IMU 에서 쓰는 건 `angular_velocity.z` 하나뿐이다.**

---

## 5. `/gps/fix` — GNSS

### 5.1 명령

```bash
ros2 topic hz /gps/fix                              # ≈2 Hz
ros2 topic echo /gps/fix --once --field status
ros2 topic echo /gps/fix --once
```

### 5.2 ⚠️ 실내에서는 `status: -1` 이 **합격**이다

GNSS 신호는 지표면 도달 세기가 잡음 수준이라(≈−130 dBm), 콘크리트가
20~30 dB 를 깎으면 추적 한계 아래로 내려간다. **실내는 정확도가 나쁜 게
아니라 아예 fix 가 안 잡힌다.**

| `/diagnostics` 출력 | 뜻 |
|---|---|
| `receiving with warning` + `status: -1` + `no GNSS fix` | ✅ **배선 정상, 하늘만 없음** |
| `no messages` | ❌ USB/드라이버 문제 |

이 둘을 헷갈리지 않는 게 핵심이다.

### 5.3 창가는 오히려 위험

하늘이 일부만 보이면 **다중경로(multipath)** 로 반사 신호를 직접파로 착각해
수십 m 틀린 좌표를 "fix 있음" 으로 보고한다. no-fix 보다 나쁘다 — EKF 가
그 값을 믿고 수렴하기 때문이다. **실내에서 그럴듯한 좌표가 찍혀도 믿지 않는다.**

### 5.4 실외 검증 시

- 건물 벽에서 몇 m 떨어진 개활지 (주차장 한가운데, 운동장)
- 콜드 스타트는 30초~수 분 — 짧게 보고 실패 판정하지 않는다
- 두 안테나는 실제 선체 장착 간격 그대로

---

## 6. `/odom` — 위치·자세 (스택의 좌표 기준)

### 6.1 스택이 실제로 쓰는 필드

| 필드 | 쓰는 곳 |
|---|---|
| `pose.pose.position.x/y` | occupancy_grid 좌표변환, mission_manager 전환 판정 |
| `pose.pose.orientation` | occupancy_grid yaw, behavior 방위 계산 |
| **`twist.twist.angular.z`** | `seek_goal()` D항 — **전복 방지의 핵심** |
| `twist.twist.linear.x/y` | avoid FSM 속도벡터 (**body frame**) |
| **`header.stamp`** | **avoid FSM·buoy_tracker 의 시계** |
| `child_frame_id` | health monitor (비면 WARN) |

`header.frame_id` 는 **아무도 안 읽는다** — 프레임 이름은 자유롭다.

### 6.2 명령

```bash
ros2 topic hz /odom
ros2 topic echo /odom --once --field pose.pose
ros2 topic echo /odom --once --field twist.twist
ros2 topic echo /odom --once --field child_frame_id
ros2 topic echo /odom --field pose.pose.position          # 연속 관찰
```

### 6.3 현장 sanity check

1. **정지 시 위치가 흐르지 않는가** — 가만히 두고 30초. 한 방향으로 계속
   밀리면 추정 미수렴.
2. **직진하면 값이 맞게 늘어나는가** — 10 m 걸어가서 변화량 대조.
3. **`twist.angular.z` 부호** — 반시계에서 양수 (§4.2 와 동일).

### 6.4 ⚠️ `/odom` 이 없으면 `/occupancy_grid` 도 안 나온다

`occupancy_grid._on_scan` 첫 줄이 `if self.odom is None: return` 이다.
**`/odom` 이 0건이면 `/scan` 이 아무리 잘 와도 격자는 영원히 비어 있다.**
정상 동작이니 라이다 판정은 `/scan` 직접 확인으로 한다.

### 6.5 GQ7 EKF 가 0건일 때

```bash
ros2 topic list | grep ekf
ros2 topic echo /ekf/status --once           # filter state 확인
ros2 param list /microstrain_inertial_driver | grep -i "filter\|ekf\|odom"
ros2 param get /microstrain_inertial_driver device_setup
```

원인은 두 층이고 **둘 다 풀어야** 값이 나온다:

| 층 | 상태 | 성격 |
|---|---|---|
| ① 장치가 EKF 스트림 미출력 | `gq7.yaml` 의 `device_setup: false` | 설정 |
| ② EKF 미수렴 | GNSS fix 없음 + 정지 | **물리 — 실내에서는 못 품** |

> ⚠️ **`device_setup: true` 로 바꾸기 전에**: 켜면 드라이버가 장치에 저장된
> 보정값과 안테나 설정을 덮어쓴다. **두 GNSS 안테나 lever arm 과 장착 자세를
> 실측하기 전에는 켜지 않는다.** 대안으로 SensorConnect(제조사 GUI)에서 EKF
> 스트림만 켜 장치에 저장하고 `device_setup: false` 를 유지하는 방법이 있다.

---

## 7. D455 — RGB / Depth / CameraInfo

```bash
ros2 launch kaboat_hardware real_sensors.launch.py enable_d455:=true

ros2 topic hz /camera/color/image_raw
ros2 topic hz /camera/depth/image_raw
ros2 topic echo /camera/color/image_raw --once --no-arr | grep -E "encoding|width|height|frame_id"
ros2 topic echo /camera/depth/image_raw --once --no-arr | grep -E "encoding|width|height"
ros2 topic echo /camera/camera_info --once
```

### 확인 포인트

- **RGB 와 depth 해상도가 같아야 한다.** 다르면 `buoy_detector` 가 같은 픽셀의
  거리로 오인하지 않으려고 해당 프레임을 버리고 오류를 낸다.
- **encoding**: RealSense depth 는 `16UC1`[mm], Gazebo 는 `32FC1`[m]. 코드가
  `depth_utils` 에서 float32 미터로 통일하므로 둘 다 정상이다.
- **`camera_info` 의 K 가 해상도와 맞는지** — `width` 와 `k[0]`(fx),
  `k[2]`(cx) 를 대조한다. cx 가 width/2 근처가 아니면 방위 계산이 크게
  쏠린다(sim 에서 ~60° 쏠린 전례, SKELETON §7).

이미지를 눈으로 보려면:

```bash
ros2 run rqt_image_view rqt_image_view
```

---

## 8. TF 확인

스택 자체는 TF 를 안 쓰지만 **RViz 는 Fixed Frame 해석에 TF 가 필요하다.**
실물에는 TF 발행자가 없어(`gq7.yaml` 의 `tf_mode: 0`) 별도로 띄운다.

```bash
ros2 run tf2_tools view_frames          # frames.pdf 생성
ros2 topic echo /tf_static --once
ros2 run tf2_ros tf2_echo odom base_link
```

TF 는 **센서 launch** 가 발행한다 — `real_sensors.launch.py` /
`indoor_tank.launch.py` 의 `publish_tf:=true`(기본)가 `odom_tf_broadcaster` 를
띄운다. RViz 는 시각화만 하므로 TF 를 만들지 않는다.
`Fixed Frame [odom] does not exist` 가 뜨면 센서 launch 가 안 떠 있거나
`publish_tf` 가 꺼진 것이다.

---

## 9. RViz 로 보기

```bash
# 실물 — 센서 launch 가 TF 를 발행하므로 먼저 떠 있어야 한다
ros2 launch kaboat_hardware real_sensors.launch.py
ros2 launch kaboat_bringup rviz.launch.py

# sim (Gazebo 가 clock/TF 를 모두 준다)
ros2 launch kaboat_bringup rviz.launch.py use_sim_time:=true
```

Map `/occupancy_grid` · LaserScan `/scan` · Odometry `/odom` 이 뜬다.
화면이 비면 이 순서로 좁힌다:

1. `ros2 run tf2_tools view_frames` — TF 트리가 있는가
2. RViz 좌하단 Displays 의 빨간 에러 메시지
3. `ros2 topic hz /occupancy_grid` — 격자가 나오고는 있는가
4. `/odom` 이 있는가 (§6.4 — 없으면 격자가 안 채워진다)

---

## 10. 실내 수조 모드 (AprilTag)

GNSS 대신 천장 AprilTag + GQ7 자이로로 `/odom` 을 만든다.
노트북 웹캠 설치·보정·Wi-Fi DDS 연결의 전체 절차는
[`APRILTAG_WIFI_SETUP.md`](APRILTAG_WIFI_SETUP.md)를 따른다.

```bash
# 천장 카메라 노트북 (tag_size는 인쇄물 실측값으로 변경)
ros2 launch kaboat_hardware ceiling_apriltag.launch.py \
  tag_id:=0 tag_size:=0.162

# 배 (Jetson)
ros2 launch kaboat_hardware indoor_tank.launch.py
```

### 확인

```bash
ros2 run tf2_tools view_frames                    # 태그 TF 이름 확인 (tag36h11:0 등)
ros2 topic echo /detections --once                # 태그가 보이는가
ros2 topic hz /odom                               # ≈30 Hz
ros2 topic echo /odom --once --field twist.twist  # angular.z 가 자이로 값인가
```

노드 로그로 상태를 읽는다:

| 로그 | 뜻 |
|---|---|
| `태그 획득 — /odom 발행 시작` | 정상 |
| `태그 TF 없음` | apriltag_ros 미실행 / `tag_frame` 이름 불일치 |
| `태그 유실 — /odom 발행 중단` | 카메라가 태그를 놓침 (**의도된 안전 동작**) |
| `IMU 각속도 없음/지연 — 태그 미분으로 폴백` | 자이로 끊김. 노이즈 증가 |

태그 유실 시 발행을 멈추는 것은 의도적이다 — 마지막 위치를 재발행하면 배가
옛 좌표를 믿고 달린다. 끊으면 `cmd_mux` 의 300ms 워치독이 정지시킨다.

### ⚠️ `/odom` 발행자 중복 주의

`indoor_tank.launch.py` 는 GQ7 의 EKF→`/odom` remap 을 자동으로 끈다
(`enable_odom_remap:=false`). `real_sensors.launch.py` 를 직접 쓸 때는 수동으로
꺼야 한다 — 안 끄면 EKF 가 수렴하는 순간 발행자가 둘이 되어 두 좌표계가 섞인다.

```bash
ros2 topic info /odom --verbose | grep "Publisher count"   # 반드시 1
```

---

## 11. GPS/AprilTag 전 임시 IMU dead-reckoning 시험

TG-50과 GQ7을 손으로 같이 움직이며 `/occupancy_grid`가 이동·회전에 따라
갱신되는지만 확인하는 **단기 시험 전용** 모드다. GQ7 자세로 중력을 제거한
평면 가속도를 두 번 적분해 `/odom`을 만들므로, 시간이 갈수록 위치가 반드시
드리프트한다. 항법·미션·모터 주행에는 사용하지 않는다.

```bash
source /opt/ros/humble/setup.bash
source /home/msga2026/Desktop/kaboat_ws/ydlidar_ws/install/setup.bash
source /home/msga2026/Desktop/kaboat_ws/repo/install/setup.bash

ros2 launch kaboat_hardware imu_tg50_mapping.launch.py
```

시작 직후 `2초` 동안 GQ7과 TG-50을 바닥에 놓고 움직이지 않는다. 로그에
`보정 완료 — /odom 발행 시작`이 나온 뒤 두 센서를 장착 상태 그대로 천천히
같이 움직인다. launch에는 GQ7, TG-50, 임시 odom, occupancy-grid, RViz만
포함되며 모터·미션·행동 노드는 실행하지 않는다.

```bash
ros2 topic hz /imu/data          # 약 100 Hz
ros2 topic hz /scan              # 약 10 Hz
ros2 topic hz /odom              # 약 100 Hz
ros2 topic hz /occupancy_grid    # 약 10 Hz

# 현재 위치·속도·상대 yaw만 원점으로 재설정 (바이어스 보정은 유지)
ros2 service call /imu_dead_reckoning_odom/reset std_srvs/srv/Trigger '{}'
```

파라미터는
[`imu_dead_reckoning.yaml`](src/kaboat_hardware/config/imu_dead_reckoning.yaml)에
있다. 정지 감지(`zupt_enabled`)는 기본적으로 끈다. 켜면 멈춘 뒤 속도
드리프트는 줄지만, IMU만으로는 등속 이동과 정지를 구분할 수 없어 부드럽게
운반하는 구간을 정지로 잘못 판단할 수 있다.

시험이 끝나면 launch를 종료한다. 이후 AprilTag/GQ7 EKF `/odom`과 동시에
실행하면 발행자가 둘이 되므로 다음 값은 반드시 `1`이어야 한다.

```bash
ros2 topic info /odom --verbose | grep "Publisher count"
```

---

## 12. 프로세스 정리

이전 프로세스가 남아 토픽이 꼬이는 경우가 흔하다.

```bash
ps -ef | grep -E "ros2 launch|apriltag|microstrain|rviz2|parameter_bridge" | grep -v grep

ps -ef | grep -E "gz sim|ros2 launch|parameter_bridge|robot_state_publisher" \
  | grep -v grep | awk '{print $2}' | xargs -r kill -9
```

`kill -9` 직후 바로 재실행하지 말고 **1~2초 텀**을 둔다.

---

## 13. 요약 — 실내에서 확인 가능한 범위

| 항목 | 실내 | 합격 기준 |
|---|---|---|
| `/imu/data` | ✅ 완전 검증 | 100 Hz, 반시계=양수, 바이어스 <0.01 rad/s |
| `/scan` | ✅ **오히려 좋음** | 10 Hz, 2000빔, 벽까지 거리 정상 |
| D455 | ✅ 검증 가능 | 해상도·encoding·프레임률 |
| `/gps/fix` **배선** | ✅ 검증 가능 | 2 Hz + `status: -1` ← **정상 결과** |
| `/gps/fix` **좌표** | ❌ | 실외 필요 |
| `/odom` (GQ7 EKF) | ❌ | 실외 + 이동 필요 |
| `/odom` (AprilTag) | ✅ | 30 Hz, §10 |
