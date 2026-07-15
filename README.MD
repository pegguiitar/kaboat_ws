# 🚢 kaboat_ws (2026)

OSRF VRX 오픈소스를 기반으로 구축된 WAM-V 선박의 통합 자율주행 알고리즘 시뮬레이션 테스트 환경입니다.  
로컬 PC에 ROS 2를 직접 설치하지 않고, **Docker 컨테이너**를 활용하여 격리된 우분투 환경에서 안전하고 동일하게 시뮬레이션을 구동합니다.

> 이 문서는 **컨테이너 설치와 시뮬레이터 환경 구축**을 다룹니다.
> 자율주행 스택(perception/behavior/control/mission)의 **코드 구조와 설계**는
> **[SKELETON.MD](SKELETON.MD)** 를 참고하세요.

---

## 📋 1. 개발 환경 요구사항
본 패키지는 팀원 간 환경 일치를 위해 다음 호스트 환경을 전제로 구성되었습니다.
* **Host OS:** Ubuntu 22.04 LTS (듀얼부팅 환경)
* **필수 설치 툴:** Docker, Docker Compose, VS Code (Dev Containers 확장 프로그램 필수)
* **ROS 2 설치 여부:** **필요 없음** (도커 내부에서 자동으로 Humble Desktop Full 버전 구동)

---

## 🏗️ 2. 빠른 시작 (Quick Start)

### 1) GUI 화면 출력을 위한 사전 세팅 (★ PC 켤 때마다 최초 1회)
도커 컨테이너 내부의 Gazebo 시뮬레이터 화면을 호스트 PC(우분투) 모니터에 띄우기 위해 디스플레이 접근 권한을 허용해야 합니다. 호스트 터미널에 아래 명령어를 입력하세요.

```bash
xhost +local:docker
```
*(성공 시 `non-network local connections being added to access control list` 문구가 출력됩니다.)*

### 2) 레포지토리 클론 및 폴더 이동
호스트 터미널에서 아래 명령어를 실행하여 통합 레포지토리를 클론합니다.

```bash
git clone https://github.com/pegguiitar/kaboat_ws.git
cd kaboat_ws
```

> ⚠️ **Gazebo Garden 필수**: VRX 는 Gazebo Garden(`gz-sim7`)을 요구하는데,
> ROS 2 Humble 공식 apt 패키지(`ros-humble-ros-gz-sim` 등)는 Gazebo
> Fortress 로 고정 빌드되어 있어 VRX 플러그인(부력·파도 등)이 로드되지
> 않습니다. 그래서 이 Dockerfile은 Gazebo Garden 런타임(`gz-garden`)을 설치하고,
> `ros_gz` 는 apt 대신 소스로 받아 `GZ_VERSION=garden`(Dockerfile에 이미 설정됨)
> 으로 빌드합니다. 아래 순서대로만 하면 별도 조치 불필요합니다.

### 3) 도커 컨테이너 가동
```bash
docker compose up -d
```
* 이 명령어를 실행하면 GitHub Container Registry(`ghcr.io/pegguiitar/kaboat_ws`)에서 공용 개발 환경 이미지를 자동으로 Pull 받아와 컨테이너를 백그라운드에 띄웁니다.
* 호스트 PC의 `src/` 폴더가 컨테이너 내부 워크스페이스와 실시간 연동(Volume Mount)됩니다.
* **Dockerfile 을 직접 고쳤다면** `docker compose up -d --build` 로 실행하면 레지스트리 이미지 대신 이 저장소의 `Dockerfile` 로 로컬에서 빌드합니다. (검증 후에는 아래 §6 규칙에 따라 ghcr.io 이미지도 갱신해 주세요.)
* `ghcr.io/pegguiitar/kaboat_ws` 이미지가 **Private** 로 설정되어 있다면, Pull 전에 한 번 로그인해야 합니다: `docker login ghcr.io -u <github-id>` (비밀번호 칸에 GitHub Personal Access Token 입력, `read:packages` 권한 필요).

### 4) VS Code를 통한 컨테이너 접속 및 빌드
1. **VS Code**를 실행한 뒤 `kaboat_ws` 폴더를 열어줍니다.
2. 왼쪽 아래 초록색 원격 아이콘(`><`)을 클릭합니다.
3. **"Attach to Running Container..."**를 선택한 후, **`kaboat_sim_container`**를 클릭하여 컨테이너 내부로 진입합니다.
   (VS Code 없이 터미널만 쓸 거면 호스트에서 `docker exec -it kaboat_sim_container bash` 로 대신 접속해도 됩니다.)
4. **컨테이너 내부 터미널**(`Ctrl + ~`)에서 아래를 **순서대로** 입력합니다.

```bash
# 4-1) 외부 소스 가져오기 (★ 최초 1회) — vrx, ros_gz 를 vrx.repos 기준으로 받고
#      vrx 저장소의 알려진 깨진 심링크(ball_shooter 텍스처)를 자동 제거합니다.
#      반드시 컨테이너 안에서 실행하세요 — vcstool 은 호스트가 아니라
#      이 Docker 이미지 안에만 설치되어 있습니다 (호스트엔 ROS2/vcstool 설치 불필요).
cd /workspace/kaboat_ws
./scripts/setup_workspace.sh

# 4-2) 워크스페이스 의존성 설치 및 빌드
rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install   # GZ_VERSION=garden 은 Dockerfile에서 이미 설정됨

# 4-3) 빌드 환경 변수 반영
source install/setup.bash
```

### 5) 시뮬레이션 구동 테스트
환경 변수가 반영된 VS Code 내부 터미널(또는 `docker exec -it kaboat_sim_container bash`)에서 실행합니다.

```bash
ros2 launch kaboat_sim simulation.launch.py
```
* Gazebo 창이 뜨면 **좌하단의 재생(▶) 버튼을 눌러야 시뮬레이션이 움직입니다.**
  world 설정이 `start_paused: true` 라서, 재생을 안 누르면 배가 계속 멈춰있는 게 정상입니다
  (버그 아님 — 처음 보면 "안 움직인다"고 헷갈리기 쉬우니 꼭 확인하세요).
* GUI 없이 가볍게 돌리고 싶으면(원격/저사양 환경): `ros2 launch kaboat_sim simulation.launch.py headless:=True`

호스트 PC 화면에 Gazebo 가상 해상 환경과 WAM-V 선박이 정상적으로 팝업되고, 배가 물 위에 안정적으로 떠 있으면 모든 세팅이 완료된 것입니다.

> 자율주행 스택(`kaboat_bringup autonomy.launch.py` 등) 실행법은
> **[SKELETON.MD](SKELETON.MD)** 에 있습니다. 아래 6)/7)은 자율주행 코드 없이
> 시뮬레이터/보트 자체를 검증하는 수동 도구입니다.

### 6) WASD 원격 조종
시뮬레이터와 `twist2thrust`가 켜진 상태에서 `/cmd_vel`에 직접 조종 명령을 발행합니다.
`autonomy.launch.py`를 함께 켜면 `cmd_mux`도 `/cmd_vel`을 발행하므로, 수동 조종할 때는
자율주행 스택을 끄고 사용하세요.

터미널 2에서 `/cmd_vel` → 좌우 추력 변환 노드를 먼저 실행합니다.

```bash
source /opt/ros/humble/setup.bash
source /workspace/kaboat_ws/install/setup.bash
ros2 run kaboat_sim twist2thrust.py
```

터미널 3에서 키보드 조종 노드를 실행합니다.

```bash
source /opt/ros/humble/setup.bash
source /workspace/kaboat_ws/install/setup.bash
ros2 run kaboat_sim keyboard_teleop.py
```

키 입력은 `w` 전진, `s` 후진, `a` 좌회전, `d` 우회전, `space`/`x` 정지, `q` 종료입니다.
키 입력이 잠시 끊기면 자동으로 정지 명령을 발행합니다.

### 7) 배 직진 주행 테스트
시뮬레이션이 **재생(▶) 중인 상태**에서, 컨테이너 안에 새 터미널을 하나 더 열어서
(`docker exec -it kaboat_sim_container bash`) 실행합니다.

```bash
source /opt/ros/humble/setup.bash
source /workspace/kaboat_ws/install/setup.bash
ros2 run kaboat_sim straight_drive_test.py
```
* 좌/우 추력 토픽에 동일한 값을 발행해서 요(yaw) 토크를 상쇄시켜 직진하는 최소 테스트 노드입니다.
* 기본값은 `thrust=12.0N`(현재 축소된 선체 기준 최대추력 118.6N의 약 10%), `duration=10.0s` 입니다.
  더 세게/오래 밀어보고 싶으면 파라미터로 조절하세요:
  ```bash
  ros2 run kaboat_sim straight_drive_test.py --ros-args -p thrust:=30.0 -p duration:=15.0
  ```
* ⚠️ `thrust` 를 `max_thrust`(기본 118.6, 배 크기 바꾸면 같이 갱신 필요 — §3 참고)의 80% 넘게
  주면 경고 로그가 뜹니다. 배가 축소된 뒤로 질량이 매우 작아서(~2kg), 최대치에 가까운 추력을
  주면 급가속으로 물리 발산 크래시(§7 "ODE INTERNAL ERROR" 참고)가 재현될 수 있습니다.

---

## 🚤 3. 선체 크기 조정

배 크기는 [`src/kaboat_sim/urdf/kaboat_hull.xacro`](src/kaboat_sim/urdf/kaboat_hull.xacro) 맨 위 **`target_length` 한 줄**로 조정합니다.

```xml
<xacro:property name="target_length" value="1.1"/>   <!-- 원하는 선체 세로 길이 [m] -->
```

이 값 하나로 원본 VRX WAM-V(4.9m) 대비 스케일 비율(`s`)이 계산되고, 선체·엔진·배터리·부력(Surface)·유체동역학(SimpleHydrodynamics)·추력·센서 장착부 치수가 **전부 자동으로 비례 조정**됩니다. 질량은 부피(`s³`)에 비례해서, 관성모멘트는 `s⁵`에 비례해서 같이 줄어듭니다.

### ⚠️ 크기를 바꿀 때 반드시 같이 확인할 것

1. **물리 스텝 크기** — [`src/kaboat_sim/worlds/perception_task.sdf`](src/kaboat_sim/worlds/perception_task.sdf)의 `<max_step_size>`.
   배를 크게 줄이면(원본 4.9m → 1.1m, 약 1/4.5) 작고 가벼워진 선체가 기존 4ms 스텝으로는
   부력·회전 응답을 안정적으로 못 따라가서 물리 엔진이 `"ODE INTERNAL ERROR ... aabbBound"`
   로 죽습니다(직접 겪은 문제). 지금은 1ms로 낮춰서 안정화해뒀습니다.
   **배를 다시 원본 크기 가까이로 키운다면 4ms로 되돌려 성능(프레임레이트)을 회복해도 됩니다.**
   반대로 더 작게 줄인다면 1ms보다 더 낮춰야 할 수 있습니다 — 바꾼 뒤 **최소 60초 이상**
   헤드리스로 돌려서 크래시 없는지 확인하세요 (짧게 보면 안정적으로 오판하기 쉽습니다,
   저희도 9초만 보고 안정적이라 판단했다가 나중에 크래시를 겪었습니다).

2. **`mass_factor`를 흘수선 조정 용도로 낮추지 마세요** — [`kaboat_hull.xacro`](src/kaboat_sim/urdf/kaboat_hull.xacro)에
   `mass_factor`라는 파라미터가 있는데, 이건 **1.0으로 고정해서 쓰세요.** 배가 덜 잠기게
   하려고 이 값을 낮추면(질량만 가벼워짐) 부력-질량 진동이 불안정해지면서 물리 엔진이
   크래시합니다(직접 겪고 검증한 문제, 파일 안 주석에 상세 기록). 흘수선을 조정하고
   싶으면 감쇠 계수(`zW` 등, `SimpleHydrodynamics` 플러그인) 조정이나 world의 물리 스텝을
   더 낮추는 방향으로 접근하세요.

3. **재빌드 불필요**: `colcon build --symlink-install`로 빌드했다면 `urdf/`, `worlds/`
   파일들은 install 폴더에 **심링크**되어 있습니다. 소스를 고치고 바로
   `ros2 launch kaboat_sim simulation.launch.py`만 다시 실행하면 됩니다
   (xacro 는 런치 시점에 매번 새로 전개되므로).

> ℹ️ **배 크기는 Docker 이미지가 아니라 소스코드(git)에 들어있습니다.**
> `target_length` 같은 값은 `src/` 안 파일이고, `src/`는 `docker-compose.yml`이
> 호스트에서 볼륨마운트하는 것이지 이미지에 구워 넣는 게 아닙니다. 즉 배 크기를
> 바꾼 뒤 **git push만 하면** 팀원에게 전파되고, ghcr.io 이미지를 다시 빌드/push할
> 필요가 없습니다. (이미지에는 OS·ROS2·Gazebo Garden 툴체인만 들어있고, `kaboat_sim`
> 소스·world 설정은 항상 git 저장소에서 옵니다 — 이미지를 갱신해야 하는 건 오직
> `Dockerfile` 자체를 고쳤을 때뿐입니다, §6 참고.)

---

## 🌊 4. 파도(흘수선 움직임) 설정

기본값(`perception_task.sdf`)은 **파도 없는 잔잔한 물**(`gain: 0.0`)로 설정되어 있습니다 — 인지(perception) 태스크용으로 의도된 설정입니다.

실제로 배가 파도에 반응해서 흔들리는 걸 보고 싶다면, [`perception_task.sdf`](src/kaboat_sim/worlds/perception_task.sdf) 하단의 `/vrx/wavefield/parameters` 블록에서 `gain` 값을 0보다 크게(예: `0.3`) 바꾸세요.

```xml
key: "gain"
value { type: DOUBLE double_value: 0.3 }   <!-- 0.0 = 잔잔함, 클수록 파도가 커짐 -->
```

> ⚠️ **참고**: `gain: 0.0`이어도 화면상 물 표면이 반짝이며 계속 움직이는 것처럼 보입니다 —
> 이건 실제 파도 높이(물리)가 아니라 **반사광 애니메이션(시각 효과)**이라 정상입니다.
> 물의 정점 높이는 `gain=0`일 때 완전히 고정되어 있고(실측 검증됨), 화면에 보이는 "출렁임"은
> 물 셰이더의 노멀맵이 항상 스크롤되면서 하늘 반사 패턴이 바뀌는 것뿐입니다. 배가 실제로
> 위아래로 움직이는지 확인하려면 `gain`을 0보다 크게 바꿔서 대조해 보세요 — 그러면 배가
> 뚜렷하게 상하로 흔들리는 걸 확인할 수 있습니다.

### 🌊 파도 조건에서 occupancy_grid 기울기 게이팅(TILT_MAX) 검증

`occupancy_grid`([occupancy_grid.py](src/kaboat_perception/kaboat_perception/occupancy_grid.py))는 IMU 롤/피치가
`TILT_MAX`(15도)를 넘으면 그 순간의 `/scan`을 통째로 버립니다 — 배가 기울면 2D LiDAR가 수면/하늘을 훑어
유령 장애물이 생기기 때문입니다. 파도를 켜서(`gain`을 0보다 크게, 예: `0.5`) 이 게이팅이 실제로 동작하는지
확인할 수 있습니다.

```bash
# 1) kaboat_sim 재빌드 (world 파일의 gain 변경 반영)
colcon build --symlink-install --packages-select kaboat_sim

# 2) 시뮬레이터 재실행 — Gazebo 뜨면 재생(▶) 버튼 누르기
source /opt/ros/humble/setup.bash
source /workspace/kaboat_ws/install/setup.bash
ros2 launch kaboat_sim simulation.launch.py

# 3) occupancy_grid 실행 (새 터미널)
ros2 run kaboat_perception occupancy_grid

# 4) RViz 로 확인 (Map: /occupancy_grid, LaserScan: /scan, Odometry: /odom)
ros2 launch kaboat_bringup rviz.launch.py
```

`gain=0.5`면 배가 롤/피치로 흔들리고, `TILT_MAX`를 넘는 순간마다 해당 스캔이 스킵됩니다 — 잔잔한 물
(`gain=0.0`)일 때보다 격자에 노이즈(유령 장애물)가 덜 생기는지 대조해서 확인하세요.

---

## 📂 5. 프로젝트 구조
```text
kaboat_ws/              # 통합 워크스페이스 루트
├── Dockerfile          # 로컬 빌드용 도커 명세서 (Gazebo Garden 설치 포함)
├── docker-compose.yml  # 컨테이너 실행 및 볼륨 마운트 설정 파일
├── .gitignore          # build, install, log 등 빌드 결과물 제외 설정
├── vrx.repos           # 외부 소스 참조 (vcstool): vrx, ros_gz
├── scripts/
│   └── setup_workspace.sh  # vcs import + 알려진 심링크 문제 자동 수정
└── src/                # 소스코드 폴더
    ├── kaboat_sim/     # 우리 팀 커스텀 VRX 시뮬레이션 패키지 (Git 관리 대상)
    │   ├── urdf/kaboat_hull.xacro     # 배 크기(target_length) 조정 지점
    │   ├── urdf/wamv_kaboat.xacro     # 센서(라이다·카메라·IMU·GPS) 장착부, 위 hull 을 include
    │   ├── config/bridge_config.yaml  # ros_gz 브리지 계약: gz 토픽 → 표준 ROS 토픽 이름 정규화
    │   └── worlds/perception_task.sdf # world: 물리 스텝, 파도(gain), 아레나·부표 배치
    ├── kaboat_msgs/        # ┐
    ├── kaboat_perception/  # │
    ├── kaboat_behaviors/   # │ 자율주행 스택 — 구조/설명은 SKELETON.MD 참고
    ├── kaboat_mission/     # │
    ├── kaboat_control/     # │
    ├── kaboat_bringup/     # ┘
    ├── vrx/            # 공식 OSRF VRX (vcstool 로 내려받음, Git 미추적)
    └── ros_gz/         # ROS-Gazebo 브릿지, Garden 용으로 소스 빌드 (Git 미추적)
```

`src/kaboat_sim/README.md` 처럼 패키지 내부에 따로 있던 예전 단독 시뮬레이터 가이드는
현재 통합 워크스페이스 구조와 맞지 않아 제거했습니다. 시뮬레이터 실행 절차는 이 루트
README 를, 자율주행 스택 구조·센서 토픽 인터페이스·실행법은 **[SKELETON.MD](SKELETON.MD)**
를 기준으로 관리합니다.

---

## 📂 6. 브랜치 관리 및 협업 전략 (필독)

* **main:** 안정성이 검증된 배포용 최종 코드 브랜치입니다.
* **feature / bugfix:** 모든 알고리즘 추가, 센서 세팅 변경, 버그 수정은 개별 브랜치에서 작업 후 Pull Request를 거쳐 병합합니다.
* **자율주행 스택 작업:** 센서 표준 토픽(§5)을 기준으로 perception, behavior, mission, control 패키지를 분리해 개발합니다. 각 기능 추가는 개별 브랜치에서 작업 후 Pull Request를 통해 병합합니다.
* **`feature/autonomy-skeleton-v5` 진행 상황:** 센서 계층(§5) 위에 자율주행 스택 골격을 올린 뒤 인식 레이어를 `node_structure` v5 로 정렬했습니다 — 검출기를 `buoy_detector`(HSV·상시)와 `dock_mark_detector`(YOLO·도킹 state 게이팅)로 분리, 장애물 표현을 `occupancy_grid` 단독으로 통일(회피가 격자 소비, `obstacle_detector` 는 참고용으로 launch 제외). 노드별 상세·TODO 는 **[SKELETON.MD](SKELETON.MD)** 참고.
  * *수정 이력:* 격자 단일화 시 behavior 5개의 `apply_repulsion` 입력을 `/obstacles`→`/occupancy_grid` 로 교체했는데 `station_keeper` 만 누락돼 `station` 미션 활성 시 크래시하던 것을 수정(전 behavior 격자 방식 통일). ⚠️ behavior 는 각자 활성 state 에서만 `compute_cmd` 가 돌므로, 검증은 **미션 state 별로** 확인해야 합니다(초기 `gate` 만 보면 후속 state 크래시를 놓칩니다).

### 💡 깃 협업 시 실제 개발 흐름
1. 소스코드 수정이나 브랜치 변경(`git checkout`)은 **호스트 PC 터미널**에서 평소처럼 수행합니다.
2. 볼륨 마운트 덕분에 도커 컨테이너 내부에도 실시간으로 코드가 변경됩니다.
3. 코드 수정 후에는 **VS Code 컨테이너 터미널**로 돌아와 `colcon build --symlink-install`만 다시 실행하고 시뮬레이션을 돌려 테스트하면 됩니다.
4. **주의:** 도커 내부에서 생성되는 `build/`, `install/`, `log/` 폴더와 `src/vrx/`, `src/ros_gz/` 는 `.gitignore`에 의해 자동으로 제외되므로 커밋 시 포함되지 않도록 주의하십시오.

### 🐳 Dockerfile / 개발 이미지 관리 규칙 (필독)
`docker compose up -d` 는 GitHub Container Registry 의 `ghcr.io/pegguiitar/kaboat_ws`
이미지를 그대로 Pull 합니다.
**저장소의 `Dockerfile` 을 고쳐서 push 하는 것만으로는 이 레지스트리 이미지가 갱신되지 않습니다.**
따라서 Dockerfile 을 수정했다면 **반드시 아래를 함께 수행**해 저장소와 이미지가 어긋나지 않게 하세요.

**최초 1회 — ghcr.io 로그인 (Docker Hub 계정 불필요, GitHub 계정으로 인증)**
```bash
# GitHub → Settings → Developer settings → Personal access tokens 에서
# write:packages 권한을 가진 토큰(classic) 발급 후:
docker login ghcr.io -u <github-id>
# 비밀번호 입력창에 방금 발급한 토큰을 붙여넣기 (GitHub 비밀번호 아님)
```

**Dockerfile 수정할 때마다**
```bash
docker compose up -d --build                              # 로컬 Dockerfile 로 빌드/검증
docker tag  peg0525/kaboat_2026_sim:latest \
            ghcr.io/pegguiitar/kaboat_ws:latest             # 이미지 태깅
docker push ghcr.io/pegguiitar/kaboat_ws:latest             # 레지스트리 갱신
```

> 이렇게 하지 않으면 "Dockerfile 은 최신인데 팀원이 받는 이미지는 옛날 버전" 인 상태가
> 에러 없이 조용히 발생하여 원인 파악이 어려워집니다.
>
> ghcr.io 패키지가 기본적으로 **Private** 로 생성되므로, 팀원들이 pull 할 수 있으려면
> GitHub 저장소 → Packages → `kaboat_ws` → Package settings 에서 이 저장소에
> "Manage Actions access" 또는 저장소 연결(Link repository)로 접근 권한을 열어주거나,
> 패키지를 Public 으로 전환하세요.

> ℹ️ **개인 계정 네임스페이스 주의**: `ghcr.io/pegguiitar/...` 처럼 **개인(organization
> 아닌) 계정** 이름으로 이미지를 올릴 경우, **그 계정 본인만 최초 1회 push(=패키지 생성)를
> 할 수 있습니다.** 저장소에 push 권한이 있는 다른 팀원이 먼저 push 하면
> `permission_denied: create_package` 로 실패합니다. 최초 push는 반드시 계정
> 소유자(`pegguiitar`)가 하고, 그 이후 Package settings 에서 다른 팀원을
> collaborator로 추가하면 그때부터는 누구나 갱신할 수 있습니다.

---

## 🛠️ 7. 문제 해결 (Troubleshooting)

### "재생 버튼을 눌렀는데도 배가 안 움직여요"
- world의 `<start_paused>true</start_paused>` 설정 때문에 시뮬레이션이 처음엔 항상 멈춰있습니다.
  Gazebo 창 좌하단의 **재생(▶) 버튼**을 눌러야 물리 연산이 시작됩니다.
- 그래도 안 움직이면 `gain=0`(§4 참고)이라 파도가 없어서 정지 수면에선 배가 원래 안 흔들리는
  것일 수 있습니다 — 버그 아님. 대조 확인하려면 `gain`을 0보다 크게 바꿔보세요.

### "물 표면이 계속 반짝이며 움직이는데, 파도가 켜진 건가요?"
아닙니다. `gain=0`이어도 물 셰이더의 반사광 애니메이션(노멀맵 스크롤)은 항상 재생됩니다.
실제 파도 높이(물리적으로 배를 흔드는 것)와는 무관한 순수 시각 효과입니다. §4 참고.

### "시뮬레이션이 이상하게 자주 죽거나(크래시), 예전엔 됐던 게 갑자기 안 돼요"
십중팔구 **이전에 띄운 시뮬레이션 프로세스가 안 죽고 남아있는 것**입니다.
`ros2 launch`를 Ctrl+C 로 끄거나 `pkill -f "gz sim"` 만으로는 하위 프로세스
(`ros2 launch`, `parameter_bridge`, `robot_state_publisher`, `rviz_interactive_marker.py`)가
안 죽고 계속 쌓일 수 있습니다. 같은 토픽(`/clock`, `/lidar/scan`, 추력 등)을 여러 개의
브릿지가 동시에 발행하면서 상태가 꼬입니다. 재실행 전에 **아래 명령으로 확실하게 전부
정리**하세요:

```bash
ps -ef | grep -E "gz sim|ros2 launch|parameter_bridge|robot_state_publisher|rviz_interactive" \
  | grep -v grep | awk '{print $2}' | xargs -r kill -9

# 확인 (아무것도 안 나와야 정상)
ps -ef | grep -E "gz sim|ros2 launch|parameter_bridge" | grep -v grep
```

**주의**: 위 `kill -9` 직후 바로 `ros2 launch`를 다시 실행하면, 이전 `gz sim` 프로세스의
렌더링 스레드가 완전히 정리되기 전에 새 프로세스가 떠서 검은 화면 후
`vrx::WaveVisual` 소멸자에서 `Segmentation fault`가 나며 죽는 경우가 있습니다.
kill 후 **1~2초 정도 텀을 두고** 재실행하세요.

### "물리 엔진이 `ODE INTERNAL ERROR ... aabbBound` 로 죽어요"
배 크기(`target_length`)를 줄인 뒤 world의 물리 스텝(`max_step_size`)을 같이 안 줄였을
가능성이 높습니다. §3의 "크기를 바꿀 때 반드시 같이 확인할 것" 참고. 짧게(10초 이하) 관찰해서
"안정적이다"라고 판단하지 말고, **최소 60초 이상 헤드리스로 크래시 없는지 확인**하세요 —
경계 상태에서는 처음 몇십 초는 멀쩡하다가 나중에 죽는 경우가 있습니다.

### "ghcr.io pull/push 시 권한 에러가 나요"
- **Pull 이 안 될 때**: 패키지가 Private 인지 확인 (§6 마지막 참고), 필요하면
  `docker login ghcr.io` 먼저 실행.
- **Push 시 `permission_denied: create_package`**: 위 "개인 계정 네임스페이스 주의" 참고 —
  최초 push는 계정 소유자만 가능합니다.
