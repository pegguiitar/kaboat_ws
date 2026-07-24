FROM osrf/ros:humble-desktop-full

# 1. 시뮬레이션 및 GUI 구동을 위한 필수 패키지 설치
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y \
    tmux \
    vim \
    git \
    wget \
    gnupg \
    lsb-release \
    python3-pip \
    python3-vcstool \
    ros-humble-robot-localization \
    && rm -rf /var/lib/apt/lists/*

# 2. Gazebo (OSRF) apt 저장소 추가 + Gazebo Garden 설치
#
#    VRX(https://github.com/osrf/vrx) 는 Gazebo Garden(gz-sim7 / gz-msgs9)을
#    요구한다. ROS 2 Humble의 기본 apt 패키지(ros-humble-ros-gz-sim 등)는
#    Gazebo Fortress(ignition-gazebo6)로 고정 빌드되어 있어 VRX 플러그인과
#    ABI 가 맞지 않는다 (증상: "does not export any plugins. The symbol
#    [IgnitionPluginHook] is missing" 로그와 함께 부력/파도 플러그인 로드 실패).
#
#    그래서 ros-humble-ros-gz* apt 패키지 대신, Gazebo Garden 런타임을 여기서
#    설치하고 ros_gz 는 vcstool 로 받아 GZ_VERSION=garden 으로 소스에서 빌드한다
#    (ros_gz.repos 참고, entrypoint 에서 vcs import 로 받음).
RUN wget -q https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
       > /etc/apt/sources.list.d/gazebo-stable.list \
    && apt-get update \
    && apt-get install -y gz-garden \
    && rm -rf /var/lib/apt/lists/*

# GZ_VERSION=garden: ros_gz 를 src/ 에서 빌드할 때 이 값을 보고 gz-sim7/gz-msgs9
# 를 찾도록 분기한다 (ros_gz_sim/CMakeLists.txt 참고). colcon build 시 반드시
# 필요하므로 이미지 레벨에서 고정해 둔다 — 매번 export 하는 걸 잊지 않도록.
ENV GZ_VERSION=garden

# 3. 작업 디렉토리 설정 (컨테이너 내부 워크스페이스)
WORKDIR /workspace/kaboat_ws

# 4. 기본 쉘을 bash로 설정
SHELL ["/bin/bash", "-c"]

# 5. 컨테이너 접속 시 ROS2 및 워크스페이스 환경 변수 자동 세팅
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc \
    && echo "if [ -f /workspace/kaboat_ws/install/setup.bash ]; then source /workspace/kaboat_ws/install/setup.bash; fi" >> ~/.bashrc

CMD ["bash"]