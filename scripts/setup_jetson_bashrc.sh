#!/usr/bin/env bash
# ==============================================================================
# KABOAT 젯슨(Jetson) 기본 환경 및 단축어 자동 설정 스크립트
#
# 젯슨 터미널에서 1회 실행:
#   bash scripts/setup_jetson_bashrc.sh
# ==============================================================================

set -e

BASHRC="$HOME/.bashrc"
MARKER="# >>> KABOAT JETSON ENVIRONMENT >>>"

echo "🔧 젯슨 ~/.bashrc 자동 설정을 진행합니다..."

# 이미 등록되어 있는지 확인
if grep -q "$MARKER" "$BASHRC" 2>/dev/null; then
    echo "⚠️  기존 KABOAT 설정 블록이 발견되었습니다. 최신 버전으로 갱신합니다."
    # 이전 설정 블록 삭제
    sed -i "/$MARKER/,/# <<< KABOAT JETSON ENVIRONMENT <<</d" "$BASHRC"
fi

cat << 'EOF' >> "$BASHRC"
# >>> KABOAT JETSON ENVIRONMENT >>>
# 1. 워크스페이스 위치 탐색 및 디렉토리 자동 이동
for _ws_dir in "$HOME/Desktop/kaboat_ws/repo" "$HOME/Desktop/kaboat_ws" "$HOME/kaboat_ws"; do
    if [ -d "$_ws_dir" ]; then
        export KABOAT_WS="$_ws_dir"
        cd "$KABOAT_WS" 2>/dev/null
        break
    fi
done

# 2. ROS 2 기본 환경 로드
if [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
fi

# 3. 워크스페이스 install 환경 로드
if [ -n "$KABOAT_WS" ] && [ -f "$KABOAT_WS/install/setup.bash" ]; then
    source "$KABOAT_WS/install/setup.bash"
fi

# 4. ROS 통신 도메인 설정 (선체 젯슨: 42)
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0

# 5. 자주 쓰는 현장 단축어 (Aliases)
alias cb="colcon build --symlink-install"
alias sb="[ -n \"$KABOAT_WS\" ] && source \"$KABOAT_WS/install/setup.bash\" && echo '🔄 install 환경 리로드 완료'"

# 센서 / 오도메트리 실행
alias run_boat_lidar="ros2 launch kaboat_hardware indoor_tank.launch.py enable_thrusters:=false enable_onboard_lidar:=true onboard_lidar_port:=/dev/ttyUSB0"

# IMU 정지 보정
alias cal_imu="ros2 run kaboat_hardware calibrate_indoor_imu"

# 모터 제어기 실행 (아두이노 CH340: /dev/ttyCH341USB0)
alias run_gq7_outdoor="ros2 launch kaboat_hardware real_sensors.launch.py outdoor:=true"
alias run_geo="ros2 launch kaboat_hardware geo_origin.launch.py"
alias run_motor="ros2 launch kaboat_hardware thrusters.launch.py hardware_type:=serial port:=/dev/ttyCH341USB0 allow_port_scan:=false"

# 노트북 원격 모니터링 통신 (Zenoh 브리지)
alias run_zenoh="python3 scripts/zenoh_monitor_link.py jetson"

# 롤/피치 자세 각도 모니터링
alias view_pitch="python3 scripts/view_roll_pitch.py"

# 주행 테스트 실행 (노트북 시작 신호 없이 30% 기본 출력으로 즉시 시작)
alias test_bspline="ros2 launch kaboat_hardware bspline_track_test.launch.py wait_for_start:=false"
alias test_straight="ros2 launch kaboat_hardware straight_line_test.launch.py wait_for_start:=false"
alias test_circle="ros2 launch kaboat_hardware circle_drive_test.launch.py wait_for_start:=false"
alias test_station="ros2 launch kaboat_hardware station_keeping_test.launch.py wait_for_start:=false"
alias test_waypoint="ros2 launch kaboat_hardware waypoint_sequence_test.launch.py motion_enabled:=true thrust_ratio:=0.3 wait_for_start:=false"
alias test_los="ros2 launch kaboat_hardware los_path_test.launch.py path_kind:=bspline motion_enabled:=true wait_for_start:=false"
alias test_avoid="ros2 launch kaboat_hardware real_obstacle_avoid_test.launch.py drive_enabled:=true wait_for_start:=false"

echo "🤖 [KABOAT Jetson] 환경 세팅 완료 (DOMAIN: $ROS_DOMAIN_ID | WS: ${KABOAT_WS:-미탐색})"
echo "   - run_boat_lidar: 선체 TG-50 포함 센서/오도메트리 가동 (cal_imu 불필요, 즉시 /odom 생성)"
echo "   - run_gq7_outdoor : 야외 GQ7 (/odom 원점 = ~/kaboat_geo_origin.yaml)"
echo "   - run_geo       : GPS 원점 설정·위도경도→x/y 변환 (geo_origin)"
echo "   - run_motor     : 모터 제어기 가동"
echo "   - cal_imu       : [선택] IMU 정지 수동 보정 (수조 라이다 Truth 모드에서는 생략 가능)"
echo "   - test_bspline  : B-Spline 곡선 자율주행 (출력 30%, 즉시 시작)"
echo "   - test_straight : 직선 주행 테스트 (출력 30%, 즉시 시작)"
echo "   - test_circle   : 원형 선회 주행 테스트 (출력 30%, 즉시 시작)"
echo "   - test_station  : 정점 유지 DP 테스트 (출력 30%, 즉시 시작)"
echo "   - test_waypoint : 5개 웨이포인트 순차 주행 (출력 30%, 즉시 시작)"
echo "   - test_los      : LOS 경로 추종 테스트 (SOTA 직진 0.25·회전 0.31, 즉시 시작)"
echo "   - test_avoid    : 실시간 장애물 회피 주행 (즉시 시작)"
echo "   - run_zenoh     : 노트북 모니터링 브리지 가동"
echo "   - cb / sb       : 빌드 / 환경 리로드"
# <<< KABOAT JETSON ENVIRONMENT <<<
EOF

echo "✅ 젯슨 ~/.bashrc 설정이 성공적으로 등록되었습니다!"
echo "👉 바로 적용하려면 'source ~/.bashrc'를 실행하세요."
