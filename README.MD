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
git clone [https://github.com/pegguiitar/kaboat_ws.git](https://github.com/pegguiitar/kaboat_ws.git)
cd kaboat_ws
```

### 3) 도커 컨테이너 가동
```bash
docker compose up -d
```
* 이 명령어를 실행하면 도커허브(`peg0525/kaboat_2026_sim`)에서 공용 개발 환경 이미지를 자동으로 Pull 받아와 컨테이너를 백그라운드에 띄웁니다.
* 호스트 PC의 `src/` 폴더가 컨테이너 내부 워크스페이스와 실시간 연동(Volume Mount)됩니다.

### 4) VS Code를 통한 컨테이너 접속 및 빌드
1. **VS Code**를 실행한 뒤 `kaboat_ws` 폴더를 열어줍니다.
2. 왼쪽 아래 초록색 원격 아이콘(`><`)을 클릭합니다.
3. **"Attach to Running Container..."**를 선택한 후, **`kaboat_sim_container`**를 클릭하여 컨테이너 내부로 진입합니다.
4. VS Code 내부 터미널(`Ctrl + ~`)을 열고 아래 명령어를 순서대로 입력하여 최초 빌드를 수행합니다.

```bash
# 워크스페이스 의존성 설치 및 빌드
rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install

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
├── Dockerfile          # 로컬 빌드용 도커 명세서
├── docker-compose.yml  # 컨테이너 실행 및 볼륨 마운트 설정 파일
├── .gitignore          # build, install, log 등 빌드 결과물 제외 설정
└── src/                # 소스코드 폴더 (Git 관리 대상)
    ├── kaboat_sim/     # 우리 팀 커스텀 VRX 시뮬레이션 패키지
    └── vrx/            # 공식 OSRF VRX 시뮬레이터 오픈소스 패키지
```

---

## 📂 4. 브랜치 관리 및 협업 전략 (필독)

* **main:** 안정성이 검증된 배포용 최종 코드 브랜치입니다.
* **feature / bugfix:** 모든 알고리즘 추가, 센서 세팅 변경, 버그 수정은 개별 브랜치에서 작업 후 Pull Request를 거쳐 병합합니다.

### 💡 깃 협업 시 실제 개발 흐름
1. 소스코드 수정이나 브랜치 변경(`git checkout`)은 **호스트 PC 터미널**에서 평소처럼 수행합니다.
2. 볼륨 마운트 덕분에 도커 컨테이너 내부에도 실시간으로 코드가 변경됩니다.
3. 코드 수정 후에는 **VS Code 컨테이너 터미널**로 돌아와 `colcon build --symlink-install`만 다시 실행하고 시뮬레이션을 돌려 테스트하면 됩니다.
4. **주의:** 도커 내부에서 생성되는 `build/`, `install/`, `log/` 폴더는 `.gitignore`에 의해 자동으로 제외되므로 커밋 시 포함되지 않도록 주의하십시오.