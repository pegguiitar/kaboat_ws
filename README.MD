# 🚢 kaboat_ws (2026)

OSRF VRX 오픈소스를 기반으로 구축된 WAM-V 선박의 통합 자율주행 알고리즘 시뮬레이션 테스트 환경입니다.  
로컬 PC에 ROS 2를 직접 설치하지 않고, **Docker 컨테이너**를 활용하여 격리된 우분투 환경에서 안전하고 동일하게 시뮬레이션을 구동합니다.

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

### 2-1) 외부 소스 가져오기 (★ 최초 1회)
VRX 와 ros_gz 는 저장소에 직접 커밋하지 않고 `vrx.repos` 에 명시된 커밋을 내려받습니다.
(왜 ros_gz 까지 소스로 받는지는 아래 "Gazebo Garden 필수" 참고.)

```bash
./scripts/setup_workspace.sh
```
*(컨테이너 접속 후 컨테이너 내부 터미널에서 실행해도 됩니다. 도커 이미지에는 vcstool 이 이미 포함되어 있습니다.)*

이 스크립트는 `vcs import` 로 소스를 받은 뒤, vrx 저장소에 들어있는 **알려진 깨진 심링크**
(`ball_shooter` 텍스처)를 자동으로 제거합니다 — 손으로 지우는 걸 깜빡하면
`colcon build --symlink-install` 이 실패하니 꼭 스크립트로 실행하세요.

> ⚠️ **Gazebo Garden 필수**: VRX 는 Gazebo Garden(`gz-sim7`)을 요구하는데,
> ROS 2 Humble 공식 apt 패키지(`ros-humble-ros-gz-sim` 등)는 Gazebo
> Fortress 로 고정 빌드되어 있어 VRX 플러그인(부력·파도 등)이 로드되지
> 않습니다. 그래서 이 Dockerfile은 Gazebo Garden 런타임(`gz-garden`)을 설치하고,
> `ros_gz` 는 apt 대신 위 스크립트로 받은 소스를 `GZ_VERSION=garden`
> (Dockerfile에 이미 설정됨)으로 빌드합니다. 별도 조치 불필요 — 그냥
> `colcon build` 하면 됩니다.

### 3) 도커 컨테이너 가동
```bash
docker compose up -d
```
* 이 명령어를 실행하면 도커허브(`peg0525/kaboat_2026_sim`)에서 공용 개발 환경 이미지를 자동으로 Pull 받아와 컨테이너를 백그라운드에 띄웁니다.
* 호스트 PC의 `src/` 폴더가 컨테이너 내부 워크스페이스와 실시간 연동(Volume Mount)됩니다.
* **Dockerfile 을 직접 고쳤다면** `docker compose up -d --build` 로 실행하면 Hub 이미지 대신 이 저장소의 `Dockerfile` 로 로컬에서 빌드합니다. (검증 후에는 아래 §4 규칙에 따라 Hub 이미지도 갱신해 주세요.)

### 4) VS Code를 통한 컨테이너 접속 및 빌드
1. **VS Code**를 실행한 뒤 `kaboat_ws` 폴더를 열어줍니다.
2. 왼쪽 아래 초록색 원격 아이콘(`><`)을 클릭합니다.
3. **"Attach to Running Container..."**를 선택한 후, **`kaboat_sim_container`**를 클릭하여 컨테이너 내부로 진입합니다.
4. VS Code 내부 터미널(`Ctrl + ~`)을 열고 아래 명령어를 순서대로 입력하여 최초 빌드를 수행합니다.

```bash
# 워크스페이스 의존성 설치 및 빌드
rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install   # GZ_VERSION=garden 은 Dockerfile에서 이미 설정됨

# 빌드 환경 변수 반영
source install/setup.bash
```

### 5) 시뮬레이션 구동 테스트
환경 변수가 반영된 VS Code 내부 터미널에서 준비된 커스텀 시뮬레이터 런치 파일을 테스트 실행합니다.

```bash
ros2 launch kaboat_sim [실행할_런치파일_이름].launch.py
```
호스트 PC 화면에 Gazebo 가상 해상 환경과 WAM-V 선박이 정상적으로 팝업되면 모든 세팅이 완료된 것입니다.

---

## 📂 3. 프로젝트 구조
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
    ├── vrx/            # 공식 OSRF VRX (vcstool 로 내려받음, Git 미추적)
    └── ros_gz/         # ROS-Gazebo 브릿지, Garden 용으로 소스 빌드 (Git 미추적)
```

---

## 📂 4. 브랜치 관리 및 협업 전략 (필독)

* **main:** 안정성이 검증된 배포용 최종 코드 브랜치입니다.
* **feature / bugfix:** 모든 알고리즘 추가, 센서 세팅 변경, 버그 수정은 개별 브랜치에서 작업 후 Pull Request를 거쳐 병합합니다.

### 💡 깃 협업 시 실제 개발 흐름
1. 소스코드 수정이나 브랜치 변경(`git checkout`)은 **호스트 PC 터미널**에서 평소처럼 수행합니다.
2. 볼륨 마운트 덕분에 도커 컨테이너 내부에도 실시간으로 코드가 변경됩니다.
3. 코드 수정 후에는 **VS Code 컨테이너 터미널**로 돌아와 `colcon build --symlink-install`만 다시 실행하고 시뮬레이션을 돌려 테스트하면 됩니다.
4. **주의:** 도커 내부에서 생성되는 `build/`, `install/`, `log/` 폴더와 `src/vrx/`, `src/ros_gz/` 는 `.gitignore`에 의해 자동으로 제외되므로 커밋 시 포함되지 않도록 주의하십시오.

### 🐳 Dockerfile / 개발 이미지 관리 규칙 (필독)
`docker compose up -d` 는 Docker Hub 의 `peg0525/kaboat_2026_sim` 이미지를 그대로 Pull 합니다.
**저장소의 `Dockerfile` 을 고쳐서 push 하는 것만으로는 이 Hub 이미지가 갱신되지 않습니다.**
따라서 Dockerfile 을 수정했다면 **반드시 아래를 함께 수행**해 저장소와 이미지가 어긋나지 않게 하세요.

```bash
# Dockerfile 수정 → 로컬 빌드로 검증 → Hub 이미지 갱신
docker compose up -d --build                       # 로컬 Dockerfile 로 빌드/검증
docker build -t peg0525/kaboat_2026_sim:latest .   # 이미지 태깅
docker push peg0525/kaboat_2026_sim:latest         # Hub 갱신 (권한 필요)
```

> 이렇게 하지 않으면 "Dockerfile 은 최신인데 팀원이 받는 이미지는 옛날 버전" 인 상태가
> 에러 없이 조용히 발생하여 원인 파악이 어려워집니다.