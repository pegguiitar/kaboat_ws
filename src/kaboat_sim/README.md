# 🚢 kaboat_2026_vrx_simulator

OSRF VRX 오픈소스를 기반으로 구축된 WAM-V 선박의 **통합 자율주행 알고리즘 시뮬레이션 테스트 환경**입니다. 
로컬 PC에 클론하여 센서 데이터 수집 및 조종 알고리즘 테스트 셋업을 진행할 수 있습니다.

> 📝 **가이드라인 정보**: 상세한 조작법 및 추가 세팅법은 공유해 드린 **노션(Notion) 안내 페이지**의 가이드를 준수하여 주시기 바랍니다.

---

## 📋 1. 사전 요구사항 (Prerequisites)
이 패키지는 다음 개발 환경에 최적화되어 작동하며, 버전 불일치 시 일부 토픽 브릿지 및 물리 플러그인 에러가 발생할 수 있습니다.

* **OS**: **Ubuntu 22.04 LTS (Jammy Jellyfish)**
* **ROS 2**: **ROS 2 Humble Hawksbill** (`desktop` 버전 권장)

---

## 📂 2. 브랜치 관리 및 협업 전략
* **main**: 안정성이 검증된 배포용 최종 코드 브랜치입니다.
* **feature / bugfix**: 모든 버그 패치, 시뮬레이션 맵/센서 추가 및 코드 수정 사항은 개별 **`feature`** 또는 **`bugfix`** 브랜치에서 작업 후 Pull Request를 통해 통합 관리합니다.

---

## 🏗️ 3. 빠른 시작 (Quick Start)

### 1) 워크스페이스 구성 및 클론
```bash
# 워크스페이스 폴더 생성 및 이동
mkdir -p kaboat_ws/src
cd kaboat_ws

# 1. 우리 시뮬레이션 패키지 클론
git clone https://github.com/pegguiitar/kaboat_2026_vrx_simulator.git src/kaboat_sim

# 2. 의존성 VRX 패키지 클론
git clone -b humble https://github.com/osrf/vrx.git src/vrx
```

### 2) 깨진 리소스 제거 및 빌드
```bash
# 깨진 텍스처 심링크 파일 사전 제거 (필수)
rm -f src/vrx/vrx_urdf/vrx_gazebo/models/ball_shooter/meshes/Glass_BaseColor.png

# ROS 환경 소싱 및 빌드
source /opt/ros/humble/setup.bash
colcon build --symlink-install --cmake-args -DCMAKE_POLICY_VERSION_MINIMUM=3.5
```

### 3) 런치 파일 실행
```bash
source install/setup.bash

# A. 일반 시뮬레이터 실행 (가제보 GUI 화면 포함)
ros2 launch kaboat_sim simulation.launch.py

# B. 헤드리스 시뮬레이터 실행 (가제보 GUI 미포함 - 리소스 절약 및 RViz 제어용)
ros2 launch kaboat_sim simulation.launch.py headless:=True
```

---

## 📝 4. 업데이트 및 패치 로그 (Modification Log)

* **`[6/29-01:40]`** : 가제보 `/clock` 브릿징 연동을 통한 RViz 뎁스/포인트클라우드 토픽 깜빡임(Flickering) 문제 완전히 해결
* **`[6/29-01:20]`** : LiDAR 센서 장착 높이 정상화 (갑판 아래 ➡️ 센서 폴 상단) 및 백색 부이(Buoy) 종횡 비율 및 안정성 개선
* **`[6/29-01:00]`** : RViz 마우스 드래그형 가상 조이스틱 (Interactive Marker) 제어 노드 추가 및 스프링-백 기능 연동
* **`[6/29-00:40]`** : 경기장(Arena) 지그재그 채널 간격 대폭 확장(6m) 및 후반부 슬라럼 게이트 트랙 연장 적용 (`model.sdf`)
* **`[6/29-00:20]`** : 레포지토리 초기 생성 및 최초 커밋