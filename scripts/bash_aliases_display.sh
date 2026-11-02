# DISPLAY는 Compose 또는 호스트의 docker exec -e DISPLAY=...에서 전달받는다.
# X11 소켓 중 가장 큰 번호를 고르면 로그인 화면(gdm)의 :1025를 선택할 수 있다.
# 전달받은 DISPLAY를 임의로 덮어쓰지 않는다.

# 1. 워크스페이스 디렉토리로 자동 이동
cd /workspace/kaboat_ws 2>/dev/null

# 2. ROS 2 기본 및 워크스페이스 빌드 환경 자동 로드
if [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
fi
if [ -f /opt/sllidar_ws/install/setup.bash ]; then
    source /opt/sllidar_ws/install/setup.bash
fi
if [ -f /workspace/kaboat_ws/install/setup.bash ]; then
    source /workspace/kaboat_ws/install/setup.bash
fi

# 3. 컨테이너 내부 편의 단축어 (Aliases)
alias cb="colcon build --symlink-install"
alias sb="source /workspace/kaboat_ws/install/setup.bash"
alias sim="ros2 launch kaboat_sim simulation.launch.py"
alias autonomy="ros2 launch kaboat_bringup autonomy.launch.py"
alias tracker="ros2 launch kaboat_hardware lidar_boat_tracker.launch.py"

echo "🤖 [KABOAT Docker] 워크스페이스 및 ROS 2 Humble 환경 자동 로드 완료"
echo "   (단축어: cb = 빌드, sb = 환경 리로드, sim = 시뮬레이터, autonomy = 자율주행, tracker = 외벽라이다)"
