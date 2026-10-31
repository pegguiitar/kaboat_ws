# Jetson → 노트북 실물 모니터링 (Tailscale + Zenoh)

`real` 브랜치의 `scripts/zenoh_monitor_link.py`는 젯슨의 ROS 2 데이터를 노트북
RViz와 `sensor_check/boat_attitude.py`에서 읽도록 연결합니다. 서로 다른 휴대폰
핫스팟이어도 양쪽이 인터넷과 같은 Tailscale tailnet에 연결돼 있으면 같은
명령과 주소를 사용합니다. 이 파일은 모터 노드를 실행하지 않습니다.

## 연결 구조

```text
Jetson ROS_DOMAIN_ID=42 ── Zenoh bridge ── Tailscale ── Zenoh bridge ── 노트북 ROS_DOMAIN_ID=43
     /scan, /odom, 회피·도킹 경로, GQ7 상태  ──────────────────────────────→  RViz / 자세 화면
     indoor_lidar_odom 이 쓰는 /boat_pose       ←──────────────────────────────  외벽 RPLIDAR A3M1
```

두 ROS 도메인을 달리해 같은 와이파이에서 DDS 직통 경로가 생겨도 Zenoh와
중복되지 않게 합니다. 노트북에서 젯슨으로 가는 유일한 허용 ROS 토픽은
실내 수조의 `/boat_pose`입니다. `/cmd_vel`, `/cmd/*`, 서비스, 액션, 카메라
원본 영상은 전달하지 않습니다. 실외에서는 `/boat_pose` 발행자가 없으므로
반대 방향으로 실제 데이터가 흐르지 않습니다.

젯슨 → 노트북 허용 토픽: `/scan`, `/scan/raw`, `/odom`, `/tf`, `/tf_static`,
`/occupancy_grid`, `/avoid/plan`, `/avoid/cps`, `/avoid/dri`,
`/real_avoid_test/path`, `/real_avoid_test/markers`, `/real_avoid_test/dri`,
`/real_avoid_test/status`, `/straight_drive/markers`, `/bspline_test/path`,
`/bspline_test/markers`, `/circle_drive/markers`, `/station_keeping/markers`,
`/dock/waypoints`, `/dock/entrance_base`,
`/dock/marker_base`, `/dock/detection_status`, `/mission/state`,
`/mission/goal`, `/imu/data`, `/gps/fix`, `/gnss_1/velocity`,
`/ekf/status`, `/ekf/odometry_map`.

`/dock/waypoints`는 현재 작업 트리의 `docking_ctrl.py`가 발행하는
`nav_msgs/Path`입니다. 해당 변경을 젯슨에 복사·빌드하기 전에는 토픽이
나오지 않습니다. 선체 라이다를 장착하지 않으면 `/scan`도 나오지 않습니다.

## 한 번 설치

젯슨과 노트북의 **호스트 OS 각각**에 Tailscale과 `zenoh-bridge-ros2dds`를
설치합니다. Tailscale은 [공식 Linux 설치 안내](https://tailscale.com/docs/install/linux)
대로 설치한 뒤 각 기기에서 `sudo tailscale up`을 실행하고 같은 tailnet에
가입합니다. 두 기기의 Tailscale IP는 각각 `tailscale ip -4`로 확인합니다.

Zenoh의 [공식 Debian 패키지 안내](https://github.com/eclipse-zenoh/zenoh-plugin-ros2dds#linux-debian)
에 따라 각 기기에서 다음을 한 번 실행합니다.

```bash
sudo install -d -m 0755 /etc/apt/keyrings
curl -L https://download.eclipse.org/zenoh/debian-repo/zenoh-public-key | \
  sudo gpg --dearmor --yes --output /etc/apt/keyrings/zenoh-public-key.gpg
echo 'deb [signed-by=/etc/apt/keyrings/zenoh-public-key.gpg] https://download.eclipse.org/zenoh/debian-repo/ /' | \
  sudo tee /etc/apt/sources.list.d/zenoh.list
sudo apt update
sudo apt install zenoh-bridge-ros2dds
zenoh-bridge-ros2dds --version
```

젯슨과 노트북에 같은 Zenoh 브리지 버전을 설치합니다. 노트북에서
`boat_attitude.py`도 실행하려면 ROS 2 Jazzy, `matplotlib`,
`microstrain_inertial_msgs`가 필요합니다. 이 노트북의 기존 `sensor_check`
환경을 사용합니다.

### 젯슨에서 추가로 할 일

젯슨의 Ubuntu 버전과 CPU를 먼저 확인합니다. 이 노트북에 설치된 Zenoh
브리지는 **1.10.2**이며, 공식 저장소에 ARM64용 같은 버전도 있습니다.

```bash
cat /etc/os-release
uname -m
curl -fsSL https://tailscale.com/install.sh | sh
sudo systemctl enable --now tailscaled
sudo tailscale up
tailscale ip -4
```

화면이 없는 젯슨이라도 `tailscale up`이 출력한 인증 URL을 노트북
브라우저에서 열어 같은 tailnet에 가입시킬 수 있습니다. 위의 Zenoh APT
설치 블록도 젯슨에서 실행한 뒤 `zenoh-bridge-ros2dds --version`이
노트북과 같은지 확인합니다. `1.10.2`가 아닌 버전이 설치되었다면
`apt-cache policy zenoh-bridge-ros2dds`에서 1.10.2 제공 여부를 확인하고
`sudo apt install zenoh-bridge-ros2dds=1.10.2`로 맞춥니다. 마지막으로 이
레포의 `scripts/zenoh_monitor_link.py`가 젯슨의 `kaboat_ws/scripts/`에도
복사되어 있어야 합니다.

## 젯슨에서 실행

젯슨의 터미널에서는 워크스페이스 디렉토리로 이동한 후 **반드시 ROS 2 기본 환경과 워크스페이스 install 환경을 모두 source**합니다. (`install/setup.bash`를 생략하면 `Package 'kaboat_hardware' not found` 오류가 발생합니다.)

```bash
cd ~/Desktop/kaboat_ws/repo
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=42

# 1) Tailscale IP 확인 (예: 100.108.192.55)
tailscale ip -4

# 2) FastDDS 대용량 라이다(TG-50) 패킷 버퍼 설정 (재부팅 후 첫 실행 시 1회 권장)
# 미설정 시 'sequence size exceeds remaining buffer' 에러가 발생할 수 있습니다.
sudo sysctl -w net.core.rmem_max=8388608
sudo sysctl -w net.core.rmem_default=8388608

# 3) Zenoh 브리지 시작
python3 scripts/zenoh_monitor_link.py jetson
```

브리지는 젯슨의 **Tailscale IP에서만** TCP 7447을 듣습니다. 핫스팟의
`192.168.x.x` 주소는 설정에 넣지 않습니다.

### 젯슨에서 센서 및 모니터링 실행 명령

별도 터미널에서 다음 명령들을 유용하게 실행할 수 있습니다:

```bash
cd ~/Desktop/kaboat_ws/repo
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=42

# 실시간 롤/피치 기울기 및 스캔 차단 상태 확인 (임계값: ±5.0°)
python3 scripts/view_roll_pitch.py

# 선체 TG-50 라이다 단독 실행 (IMU ±5.0° 기울기 필터 + 선체 220°~320° 가림 적용)
ros2 launch kaboat_hardware onboard_lidar.launch.py port:=/dev/ttyUSB0

# 육상/책상 테스트 등 센서가 기울어진 상태에서 필터를 끄고 스캔만 볼 때:
ros2 launch kaboat_hardware onboard_lidar.launch.py port:=/dev/ttyUSB0 enable_imu_filter:=false
```

## 노트북에서 실행

외벽 RPLIDAR A3M1 Docker 컨테이너를 쓰는 수조 시험에서는 컨테이너를 노트북
도메인 43으로 생성합니다. `docker-compose.lidar.yml`은 호스트 네트워크를
사용하므로 별도 Docker 포트 게시가 필요하지 않습니다.

```bash
cd /home/jiwoo/Projects/KABOAT/kaboat_ws
ROS_DOMAIN_ID=43 docker compose -f docker-compose.lidar.yml up -d --build --force-recreate
```

수조 테스트 가이드의 외벽 RPLIDAR A3M1 실행 명령을 이어서 사용합니다.
`--force-recreate`는 기존 컨테이너를 재시작하므로 센서 시험 시작 전에
실행합니다. 실외에서 외벽 라이다를 쓰지 않으면 Docker 단계는 생략합니다.

노트북 호스트의 **다른 터미널**에서 젯슨의 Tailscale IP를 한 번 입력해
브리지를 실행합니다. 다음 날 핫스팟이 바뀌어도 이 IP는 그대로 사용합니다.

```bash
cd /home/jiwoo/Projects/KABOAT/kaboat_ws
source /opt/ros/jazzy/setup.zsh
export ROS_DOMAIN_ID=43
export ROS_LOCALHOST_ONLY=0
python3 scripts/zenoh_monitor_link.py laptop --jetson-ip 100.x.y.z
```

`100.x.y.z`를 젯슨에서 확인한 실제 `tailscale ip -4` 값으로 바꿉니다.
스크립트는 Tailscale 실행 상태와 주소를 확인하고, Zenoh 설정을 임시
파일로 만들어 브리지를 시작합니다. 노트북 브리지는 젯슨에 연결을
재시도하도록 설정합니다. 두 터미널을 켜 둡니다.

실내 수조 시험에서는 [TANK_TESTS_GUIDE.md](TANK_TESTS_GUIDE.md)를 따라
노트북 라이다 Docker를 도메인 43으로, 젯슨을 도메인 42로 실행합니다.
노트북의 `/boat_pose`가 Zenoh를 통해 젯슨에 들어가고,
젯슨의 `indoor_lidar_odom`이 만든 `/odom`이 노트북으로 돌아옵니다.
출발·정지·비상 정지 명령은 브리지에서 제외했으므로 젯슨 도메인 42의
터미널에서 실행합니다.

## 노트북에서 수신 확인

새 터미널에서 같은 노트북 ROS 도메인을 사용합니다.

```bash
source /opt/ros/jazzy/setup.zsh
export ROS_DOMAIN_ID=43
tailscale ping 100.x.y.z
ros2 topic list | grep -E '^/(scan|odom|avoid/plan|real_avoid_test/path|dock/waypoints|ekf/status)$'
ros2 topic hz /odom
ros2 topic info /odom --verbose      # Publisher count 1 확인
```

실외 GQ7이 `/odom`을 발행하는 구성이라면 같은 노트북에서
`sensor_check` 화면을 별도 창으로 열 수 있습니다.

```bash
cd /home/jiwoo/Projects/KABOAT/sensor_check
python3 boat_attitude.py --gps-topic /gps/fix --odom-topic /odom
```

이 화면은 `/ekf/status`가 `Full Nav`이고 GQ7 INS 오도메트리가 살아 있을 때
위치·속도를 표시합니다. 수조에서 외벽 라이다 기반 `/odom`을 쓰는 경우에는
기존 화면을 그 데이터에 그대로 연결할 수 없습니다. 실내용 자세/위치
표시 모드는 별도로 필요합니다. RViz도 노트북 호스트에서 별도 창으로 엽니다.

```bash
source /opt/ros/jazzy/setup.zsh
export ROS_DOMAIN_ID=43
rviz2 -d /home/jiwoo/Projects/KABOAT/kaboat_ws/scripts/remote_monitor.rviz
```

이 설정에는 `/scan`, `/odom`, `/avoid/plan`, `/real_avoid_test/path`,
`/dock/waypoints`, 회피 마커가 들어 있습니다. 기본 `Fixed Frame`은 `odom`입니다.
실외 GQ7 출력의 실제 프레임이 `map`이면 RViz에서 그에 맞게 바꾸고,
`base_link`까지 필요한 TF가 이어지는지 확인합니다. 프레임 이름만 바꾸는 것은
실제 좌표 변환을 만들지 않습니다.

### 자주 발생하는 오류 및 현장 조치 (Troubleshooting)

1. **RViz에서 `/scan`이 안 보이고 `queue is full` 에러가 발생할 때**
   * 증상: `Message Filter dropping message: frame 'onboard_laser_frame' at time ... for reason 'discarding message because the queue is full'`
   * 원인: RViz의 `Fixed Frame`이 `odom`으로 되어 있으나, 현재 TF 트리에 `odom -> base_link -> onboard_laser_frame` 변환이 없기 때문입니다.
   * 조치: RViz 좌측 패널 상단 **`Global Options` → `Fixed Frame`을 `onboard_laser_frame`으로 변경**하면 라이다 스캔을 단독으로 즉시 확인할 수 있습니다.

2. **`/scan` 토픽이 발행되지 않거나 데이터가 안 들어올 때**
   * 증상: `ros2 topic hz /scan`이 0Hz이거나 메시지가 수신되지 않음.
   * 원인: 젯슨의 IMU 기울기 필터가 활성화된 상태에서 보트나 IMU 센서가 $\pm 5.0^\circ$ 이상 기울어져 있어 스캔을 차단(`drop`)하는 중일 가능성이 높습니다.
   * 확인: 젯슨에서 `python3 scripts/view_roll_pitch.py`를 실행하여 Roll / Pitch 각도와 `[ ⚠️ TILTED > 5° (스캔 차단됨) ]` 경고가 뜨는지 확인합니다.
   * 조치: 센서의 수평을 맞추거나, 육상/벤치 테스트인 경우 `enable_imu_filter:=false` 옵션을 주어 실행합니다.

3. **`sequence size exceeds remaining buffer` 에러**
   * 증상: FastDDS CDR 역직렬화 실패로 스캔이 깨지거나 노드가 중단됨.
   * 원인: TG-50 라이다 1회 스캔(약 2030포인트) 패킷 크기가 Linux 기본 UDP 수신 소켓 버퍼(208KB)를 초과함.
   * 조치: 젯슨에서 소켓 버퍼 크기를 8MB로 확대합니다:
     ```bash
     sudo sysctl -w net.core.rmem_max=8388608
     sudo sysctl -w net.core.rmem_default=8388608
     ```

4. **`Package 'kaboat_hardware' not found` 에러**
   * 원인: ROS 2 환경 source 시 워크스페이스 `install/setup.bash`가 누락됨.
   * 조치: `source /opt/ros/humble/setup.bash`와 함께 반드시 `source install/setup.bash`를 실행합니다.

5. **`python3: can't open file 'scripts/zenoh_monitor_link.py': No such file or directory`**
   * 원인: 터미널 작업 디렉토리가 홈(`~`)에 위치함.
   * 조치: `cd ~/Desktop/kaboat_ws/repo`로 이동 후 실행합니다.

## 연결이 끊겼을 때

핫스팟 변경 중에는 토픽이 일시적으로 멈춥니다. 두 기기에서
`tailscale status`, 노트북에서 `tailscale ping <젯슨 IP>`, 브리지 로그,
`ros2 topic hz /scan` 순서로 확인합니다. 데이터가 복구되지 않으면
노트북 브리지를 재시작합니다. 젯슨의 미션·센서·모터 제어는 젯슨에서
로컬로 실행하며 이 모니터링 링크에 의존하지 않게 유지합니다.

실제 두 기기와 두 핫스팟 간 통신은 이 레포에서 아직 검증하지 않았습니다.
