FROM osrf/ros:humble-desktop-full

# 1. 시뮬레이션 및 GUI 구동을 위한 필수 패키지 설치
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y \
    tmux \
    vim \
    git \
    python3-pip \
    ros-humble-ros-gz \
    ros-humble-ros-gz-sim \
    && rm -rf /var/lib/apt/lists/*

# 2. 작업 디렉토리 설정 (컨테이너 내부 워크스페이스)
WORKDIR /workspace/kaboat_ws

# 3. 기본 쉘을 bash로 설정
SHELL ["/bin/bash", "-c"]

# 4. 컨테이너 접속 시 ROS2 및 워크스페이스 환경 변수 자동 세팅
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc \
    && echo "if [ -f /workspace/kaboat_ws/install/setup.bash ]; then source /workspace/kaboat_ws/install/setup.bash; fi" >> ~/.bashrc

CMD ["bash"]