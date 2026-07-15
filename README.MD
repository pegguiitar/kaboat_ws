# 🚢 kaboat_ws (2026)

OSRF VRX 오픈소스 기반 WAM-V 선박의 통합 자율주행 알고리즘 시뮬레이션 테스트 환경입니다.
로컬 PC에 ROS 2를 직접 설치하지 않고 **Docker 컨테이너**로 격리된 우분투 환경에서 동일하게 구동합니다.

> 이 문서는 **컨테이너 설치와 시뮬레이터 환경 구축**을 다룹니다.
> 자율주행 스택(perception/behavior/control/mission)의 **코드 구조·설계**는
> **[SKELETON.MD](SKELETON.MD)** 를 참고하세요.

---

## 📋 1. 개발 환경 요구사항
* **Host OS:** Ubuntu 22.04 LTS
* **필수 툴:** Docker, Docker Compose, VS Code (Dev Containers 확장)
* **ROS 2 설치:** **불필요** (도커 내부에서 Humble Desktop Full 구동)

---

## 🏗️ 2. 빠른 시작 (Quick Start)

### 1) GUI 출력 사전 세팅 (★ PC 켤 때마다 1회)
컨테이너 안 Gazebo 화면을 호스트 모니터에 띄우려면 디스플레이 접근을 허용합니다.

```bash
xhost +local:docker
```
*(성공 시 `non-network local connections being added to access control list` 출력)*

### 2) 레포지토리 클론
```bash
git clone https://github.com/pegguiitar/kaboat_ws.git
cd kaboat_ws
```

> ⚠️ **Gazebo Garden 필수**: VRX 는 Gazebo Garden(`gz-sim7`)을 요구하는데 ROS 2 Humble
> 공식 apt 패키지는 Fortress 로 고정돼 VRX 플러그인(부력·파도)이 안 뜹니다. 그래서 이
> Dockerfile 은 Garden 런타임을 설치하고 `ros_gz` 를 소스로 `GZ_VERSION=garden` 빌드합니다
> (이미 설정됨 — 아래 순서대로만 하면 됩니다).

### 3) 도커 컨테이너 가동
```bash
docker compose up -d
```
* `ghcr.io/pegguiitar/kaboat_ws` 이미지를 Pull 해 백그라운드로 띄우고, 호스트 `src/` 를 컨테이너에 볼륨마운트합니다.
* **Dockerfile 을 직접 고쳤다면** `docker compose up -d --build` (검증 후 §6 규칙대로 ghcr.io 이미지도 갱신).
* 이미지가 **Private** 면 먼저 `docker login ghcr.io -u <github-id>` (비밀번호 칸에 `read:packages` 권한 PAT).

### 4) VS Code 접속 및 빌드
1. VS Code 로 `kaboat_ws` 폴더를 엽니다.
2. 좌하단 원격 아이콘(`><`) → **"Attach to Running Container..."** → **`kaboat_sim_container`**.
   (터미널만 쓰면 `docker exec -it kaboat_sim_container bash` 로 대체 가능.)
3. 컨테이너 터미널에서 **순서대로** 실행:

```bash
# 4-1) 외부 소스 가져오기 (★ 최초 1회) — vrx, ros_gz 를 vrx.repos 기준으로 받고
#      깨진 심링크(ball_shooter 텍스처)를 자동 제거. 반드시 컨테이너 안에서 실행
#      (vcstool 은 이 이미지 안에만 있음).
cd /workspace/kaboat_ws
./scripts/setup_workspace.sh

# 4-2) 의존성 설치 및 빌드
rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install   # GZ_VERSION=garden 은 Dockerfile 에서 설정됨

# 4-3) 환경 변수 반영
source install/setup.bash
```

### 5) 시뮬레이션 구동 테스트
```bash
ros2 launch kaboat_sim simulation.launch.py
```
* Gazebo 창이 뜨면 **좌하단 재생(▶) 버튼**을 눌러야 물리가 시작됩니다. world 가
  `start_paused: true` 라 안 누르면 배가 멈춰있는 게 정상(버그 아님).
* GUI 없이 가볍게: `ros2 launch kaboat_sim simulation.launch.py headless:=True`

배가 물 위에 안정적으로 떠 있으면 세팅 완료입니다.

> 자율주행 스택(`autonomy.launch.py`) 실행법은 **[SKELETON.MD](SKELETON.MD)** 참고.
> 아래 6)/7)은 자율주행 코드 없이 시뮬레이터/보트 자체를 검증하는 수동 도구입니다.

### 6) WASD 원격 조종
`/cmd_vel` 에 직접 조종 명령을 발행합니다. `autonomy.launch.py` 의 `cmd_mux` 도 `/cmd_vel`
을 발행하므로, 수동 조종 시엔 자율주행 스택을 끄고 쓰세요.

터미널 2 — `/cmd_vel` → 좌우 추력 변환:
```bash
source /opt/ros/humble/setup.bash
source /workspace/kaboat_ws/install/setup.bash
ros2 run kaboat_sim twist2thrust.py
```

터미널 3 — 키보드 조종:
```bash
source /opt/ros/humble/setup.bash
source /workspace/kaboat_ws/install/setup.bash
ros2 run kaboat_sim keyboard_teleop.py
```

키: `w` 전진 · `s` 후진 · `a` 좌회전 · `d` 우회전 · `space`/`x` 정지 · `q` 종료.
입력이 잠시 끊기면 자동 정지합니다.

### 7) 배 직진 주행 테스트
시뮬레이션이 **재생 중인 상태**에서 새 터미널(`docker exec -it kaboat_sim_container bash`):

```bash
source /opt/ros/humble/setup.bash
source /workspace/kaboat_ws/install/setup.bash
ros2 run kaboat_sim straight_drive_test.py
```
* 좌/우 추력에 같은 값을 발행해 요 토크를 상쇄하고 직진하는 최소 테스트 노드.
* 기본 `thrust=12.0N`(축소 선체 최대추력 118.6N의 ~10%), `duration=10.0s`. 조절:
  ```bash
  ros2 run kaboat_sim straight_drive_test.py --ros-args -p thrust:=30.0 -p duration:=15.0
  ```
* ⚠️ `thrust` 를 `max_thrust`(기본 118.6, 배 크기 바꾸면 갱신 — §3) 의 80% 넘게 주면 경고.
  질량이 매우 작아(~2kg) 최대치 근처 추력은 급가속 물리 발산 크래시(§7 "ODE INTERNAL ERROR")를 유발할 수 있습니다.

---

## 🚤 3. 선체 크기 조정

배 크기는 [`kaboat_hull.xacro`](src/kaboat_sim/urdf/kaboat_hull.xacro) 맨 위 **`target_length` 한 줄**로 조정합니다.

```xml
<xacro:property name="target_length" value="1.1"/>   <!-- 원하는 선체 세로 길이 [m] -->
```

이 값 하나로 원본 VRX WAM-V(4.9m) 대비 스케일 비율(`s`)이 계산되어 선체·엔진·배터리·부력·유체동역학·추력·센서 장착부가 **전부 비례 조정**됩니다. 질량은 `s³`, 관성모멘트는 `s⁵`에 비례해 줄어듭니다.

### ⚠️ 크기를 바꿀 때 반드시 같이 확인할 것

1. **물리 스텝 크기** — [`perception_task.sdf`](src/kaboat_sim/worlds/perception_task.sdf)의 `<max_step_size>`.
   배를 크게 줄이면(4.9m→1.1m) 가벼워진 선체가 기존 4ms 스텝을 못 따라가 물리 엔진이
   `"ODE INTERNAL ERROR ... aabbBound"` 로 죽습니다 — 지금은 **1ms**로 낮춰뒀습니다. 원본
   크기 가까이 키우면 4ms 로 되돌려 프레임레이트를 회복해도 되고, 더 줄이면 1ms보다 더
   낮춰야 할 수 있습니다. 바꾼 뒤 **최소 60초 이상** 헤드리스로 크래시 확인(짧게 보면 오판하기 쉬움).

2. **`mass_factor` 는 1.0 고정** — 흘수선을 낮추려고 이 값을 낮추면 부력-질량 진동이
   불안정해져 물리 엔진이 크래시합니다(파일 주석에 상세). 흘수선 조정은 감쇠 계수
   (`SimpleHydrodynamics` 의 `zW` 등)나 물리 스텝을 낮추는 쪽으로 접근하세요.

3. **재빌드 불필요**: `--symlink-install` 로 빌드했다면 `urdf/`·`worlds/` 는 install 에
   심링크돼 있고 xacro 는 런치 때 매번 전개되므로, 소스 고치고 바로 `simulation.launch.py` 재실행하면 됩니다.

> ℹ️ **배 크기는 이미지가 아니라 소스(git)에 있습니다.** `target_length` 는 `src/` 안
> 파일이고 `src/` 는 볼륨마운트라 이미지에 안 굽습니다 — 크기를 바꾼 뒤 **git push 만 하면**
> 팀원에게 전파되고 ghcr.io 이미지 재빌드는 불필요합니다(이미지 갱신은 `Dockerfile` 자체를 고쳤을 때만, §6).

---

## 🌊 4. 파도(흘수선 움직임) 설정

기본값(`perception_task.sdf`)은 **잔잔한 물**(`gain: 0.0`) — 인지 태스크용입니다. 배가 파도에
흔들리는 걸 보려면 하단 `/vrx/wavefield/parameters` 블록의 `gain` 을 0보다 크게 바꾸세요.

```xml
key: "gain"
value { type: DOUBLE double_value: 0.3 }   <!-- 0.0 = 잔잔함, 클수록 파도가 커짐 -->
```

> ⚠️ `gain: 0.0` 이어도 물 표면이 반짝이며 움직이는 건 **반사광 애니메이션(시각 효과)** 이라
> 정상입니다 — 실제 파도 높이(물리)는 고정입니다(실측 검증). 배가 실제로 상하로 흔들리는지는
> `gain` 을 키워서 대조하세요.

### 🌊 파도 조건에서 occupancy_grid 기울기 게이팅(TILT_MAX) 검증

`occupancy_grid` 는 IMU 롤/피치가 `TILT_MAX`(15°)를 넘으면 그 `/scan` 을 통째로 버립니다
(배가 기울면 2D LiDAR 가 수면/하늘을 훑어 유령 장애물이 생김). 파도를 켜서(`gain=0.5`) 확인:

```bash
# 1) kaboat_sim 재빌드 (world gain 변경 반영)
colcon build --symlink-install --packages-select kaboat_sim

# 2) 시뮬레이터 재실행 — Gazebo 뜨면 재생(▶)
source /opt/ros/humble/setup.bash
source /workspace/kaboat_ws/install/setup.bash
ros2 launch kaboat_sim simulation.launch.py

# 3) occupancy_grid 실행 (새 터미널)
ros2 run kaboat_perception occupancy_grid

# 4) RViz 확인 (Map: /occupancy_grid, LaserScan: /scan, Odometry: /odom)
ros2 launch kaboat_bringup rviz.launch.py
```

`gain=0.5` 면 `TILT_MAX` 초과 순간마다 스캔이 스킵됩니다 — 잔잔한 물(`gain=0.0`)보다 격자에 유령 장애물이 덜 생기는지 대조하세요.

---

## 📂 5. 프로젝트 구조
```text
kaboat_ws/              # 통합 워크스페이스 루트
├── Dockerfile          # 로컬 빌드용 도커 명세서 (Gazebo Garden 설치 포함)
├── docker-compose.yml  # 컨테이너 실행 및 볼륨 마운트 설정
├── .gitignore          # build, install, log 등 빌드 결과물 제외
├── vrx.repos           # 외부 소스 참조 (vcstool): vrx, ros_gz
├── scripts/
│   └── setup_workspace.sh  # vcs import + 알려진 심링크 문제 자동 수정
└── src/                # 소스코드 폴더
    ├── kaboat_sim/     # 커스텀 VRX 시뮬레이션 패키지 (Git 관리 대상)
    │   ├── urdf/kaboat_hull.xacro     # 배 크기(target_length) 조정 지점
    │   ├── urdf/wamv_kaboat.xacro     # 센서(라이다·카메라·IMU·GPS) 장착부, 위 hull include
    │   ├── config/bridge_config.yaml  # ros_gz 브리지 계약: gz → 표준 ROS 토픽 정규화
    │   └── worlds/perception_task.sdf # world: 물리 스텝, 파도(gain), 아레나·부표 배치
    ├── kaboat_msgs/        # ┐
    ├── kaboat_perception/  # │
    ├── kaboat_behaviors/   # │ 자율주행 스택 — 구조/설명은 SKELETON.MD 참고
    ├── kaboat_mission/     # │
    ├── kaboat_control/     # │
    ├── kaboat_bringup/     # ┘
    ├── vrx/            # 공식 OSRF VRX (vcstool, Git 미추적)
    └── ros_gz/         # ROS-Gazebo 브릿지, Garden 소스 빌드 (Git 미추적)
```

시뮬레이터 실행 절차는 이 README 를, 자율주행 스택 구조·센서 토픽·실행법은 **[SKELETON.MD](SKELETON.MD)** 를 기준으로 관리합니다.

---

## 📂 6. 브랜치 관리 및 협업 전략 (필독)

* **main:** 검증된 배포용 브랜치.
* **feature / bugfix:** 알고리즘 추가·센서 세팅 변경·버그 수정은 개별 브랜치에서 작업 후 PR 로 병합.
* **자율주행 스택:** 센서 표준 토픽(§5) 기준으로 perception/behavior/mission/control 을 분리 개발, 기능별 브랜치 + PR.
* **`feature/autonomy-skeleton-v5` 진행 상황:** 센서 계층 위에 스택 골격을 올리고 인식 레이어를
  `node_structure` v5 로 정렬 — 검출기를 `buoy_detector`(HSV·상시)/`dock_mark_detector`(YOLO·도킹
  게이팅)로 분리, 장애물 표현을 `occupancy_grid` 단독으로 통일. 상세·TODO 는 **[SKELETON.MD](SKELETON.MD)**.
  * *상세 변경 이력은 [SKELETON.MD](SKELETON.MD) §8 수정 이력에 정리합니다.* ⚠️ behavior 는 각자
    활성 state 에서만 `compute_cmd` 가 돌므로 검증은 **미션 state 별로** 하세요(초기 `gate` 만 보면 후속 크래시를 놓침).

### 💡 깃 협업 흐름
1. 소스 수정·브랜치 변경(`git checkout`)은 **호스트 터미널**에서.
2. 볼륨 마운트라 컨테이너 내부에도 실시간 반영됩니다.
3. 수정 후 **컨테이너 터미널**에서 `colcon build --symlink-install` 만 다시 돌려 테스트.
4. ⚠️ `build/`·`install/`·`log/` 와 `src/vrx/`·`src/ros_gz/` 는 `.gitignore` 로 제외 — 커밋에 포함되지 않게 주의.

### 🐳 Dockerfile / 개발 이미지 관리 규칙 (필독)
`docker compose up -d` 는 `ghcr.io/pegguiitar/kaboat_ws` 이미지를 그대로 Pull 합니다.
**Dockerfile 을 고쳐 push 하는 것만으로는 레지스트리 이미지가 갱신되지 않습니다** — 아래를 함께 하세요.

**최초 1회 — ghcr.io 로그인** (GitHub 계정으로 인증)
```bash
# GitHub → Settings → Developer settings → Personal access tokens 에서
# write:packages 권한 토큰(classic) 발급 후:
docker login ghcr.io -u <github-id>
# 비밀번호 칸에 발급한 토큰 붙여넣기 (GitHub 비밀번호 아님)
```

**Dockerfile 수정할 때마다**
```bash
docker compose up -d --build                              # 로컬 Dockerfile 로 빌드/검증
docker tag  peg0525/kaboat_2026_sim:latest \
            ghcr.io/pegguiitar/kaboat_ws:latest           # 이미지 태깅
docker push ghcr.io/pegguiitar/kaboat_ws:latest           # 레지스트리 갱신
```

> 안 하면 "Dockerfile 은 최신인데 팀원 이미지는 옛날 버전" 상태가 에러 없이 조용히 생깁니다.
>
> - ghcr.io 패키지는 기본 **Private** 라, 팀원 pull 을 허용하려면 Package settings 에서 저장소 연결로 접근을 열거나 Public 전환.
> - **개인 계정 네임스페이스 주의**: `ghcr.io/pegguiitar/...` 는 **계정 본인만 최초 1회 push(패키지 생성)** 가능합니다. 다른 팀원이 먼저 push 하면 `permission_denied: create_package`. 최초 push 는 소유자(`pegguiitar`)가 하고, 이후 Package settings 에서 collaborator 로 추가하면 누구나 갱신 가능.

---

## 🛠️ 7. 문제 해결 (Troubleshooting)

### "재생 버튼을 눌렀는데도 배가 안 움직여요"
- world 의 `<start_paused>true</start_paused>` 때문에 처음엔 항상 멈춰있습니다 — 좌하단 **재생(▶)** 을 누르세요.
- 그래도 안 움직이면 `gain=0`(§4) 이라 잔잔한 수면에선 원래 안 흔들리는 것(버그 아님). `gain` 을 키워 대조하세요.

### "물 표면이 계속 반짝이며 움직이는데, 파도가 켜진 건가요?"
아닙니다 — `gain=0` 이어도 물 셰이더 반사광 애니메이션(노멀맵 스크롤)은 항상 재생됩니다. 파도 높이(물리)와 무관한 시각 효과. §4 참고.

### "시뮬레이션이 자주 죽거나, 예전엔 됐던 게 갑자기 안 돼요"
십중팔구 **이전 시뮬 프로세스가 안 죽고 남아있는 것**입니다. `Ctrl+C` 나 `pkill -f "gz sim"`
만으로는 하위 프로세스(`ros2 launch`, `parameter_bridge`, `robot_state_publisher`,
`rviz_interactive_marker.py`)가 남아, 여러 브릿지가 같은 토픽을 발행하며 꼬입니다. 재실행 전 전부 정리:

```bash
ps -ef | grep -E "gz sim|ros2 launch|parameter_bridge|robot_state_publisher|rviz_interactive" \
  | grep -v grep | awk '{print $2}' | xargs -r kill -9

# 확인 (아무것도 안 나와야 정상)
ps -ef | grep -E "gz sim|ros2 launch|parameter_bridge" | grep -v grep
```

**주의**: `kill -9` 직후 바로 재실행하면 이전 `gz sim` 렌더링 스레드가 정리되기 전에 새
프로세스가 떠서 `vrx::WaveVisual` 소멸자에서 `Segmentation fault` 로 죽을 수 있습니다 — **1~2초 텀**을 두세요.

### "물리 엔진이 `ODE INTERNAL ERROR ... aabbBound` 로 죽어요"
배 크기(`target_length`)를 줄인 뒤 물리 스텝(`max_step_size`)을 같이 안 줄였을 가능성이 높습니다
(§3 "크기를 바꿀 때 반드시 같이 확인할 것"). **최소 60초 이상 헤드리스로** 확인하세요 — 경계 상태는 처음 몇십 초는 멀쩡하다 나중에 죽습니다.

### "ghcr.io pull/push 시 권한 에러가 나요"
- **Pull 실패**: 패키지가 Private 인지 확인(§6), 필요하면 `docker login ghcr.io`.
- **Push 시 `permission_denied: create_package`**: §6 "개인 계정 네임스페이스 주의" — 최초 push 는 계정 소유자만 가능.
