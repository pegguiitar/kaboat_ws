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

# 선체 TG-50 포트: CP2102 고정 경로 (다시 꽂아 ttyUSB 번호가 바뀌어도 그대로). 없으면 /dev/ttyUSB0
_tg50_port() {
    local p=/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0
    if [ -e "$p" ]; then echo "$p"; else echo /dev/ttyUSB0; fi
}

# 센서 / 오도메트리 실행
alias run_boat_lidar='ros2 launch kaboat_hardware indoor_tank.launch.py enable_thrusters:=false enable_onboard_lidar:=true onboard_lidar_port:=$(_tg50_port)'

# IMU 정지 보정
alias cal_imu="ros2 run kaboat_hardware calibrate_indoor_imu"

# 모터 제어기 실행 (아두이노 CH340: /dev/ttyCH341USB0)
alias run_gq7_outdoor="ros2 launch kaboat_hardware real_sensors.launch.py outdoor:=true"
# 야외 선체 TG-50 (/scan 항상, /scan/level 수평일 때만). GQ7 을 먼저 켤 것 (기울기 판단에 IMU 사용)
alias run_lidar='ros2 launch kaboat_hardware onboard_lidar.launch.py port:=$(_tg50_port)'
# 야외 GQ7 + RTK. NTRIP 계정은 젯슨에만 둔다 (~/.ntrip_account 에 export NTRIP_USER=… NTRIP_PASS=…)
[ -f ~/.ntrip_account ] && source ~/.ntrip_account
run_gq7_rtk() {
    ros2 launch kaboat_hardware real_sensors.launch.py outdoor:=true rtk:=true \
        ntrip_username:="${NTRIP_USER:?~/.ntrip_account 에 NTRIP_USER 가 없음}" \
        ntrip_password:="${NTRIP_PASS:?~/.ntrip_account 에 NTRIP_PASS 가 없음}" "$@"
}
alias run_geo="ros2 launch kaboat_hardware geo_origin.launch.py"
# D455 카메라 (컬러 640×480 + 정렬 뎁스, 15 fps). 도킹용 (지도 카메라 층은 기본 끔)
alias run_camera="ros2 launch kaboat_bringup d455.launch.py"
# 장애물 지도 (라이다 + 가상 벽, 카메라 안 씀). 하나만 켜 두고 test_*·autonomy 가 같이 쓴다
#   뒤에 -p 이름:=값 을 붙일 수 있게 --ros-args 로 끝낸다 (카메라 층: run_map -p cam_enabled:=true)
alias run_map="ros2 run kaboat_perception occupancy_grid --ros-args"

# 미션 지점 기록 (지금 /odom 위치 → ~/kaboat_mission_points.json)
alias run_points="ros2 run kaboat_hardware point_recorder"
# 한 번에: run_gq7_rtk → (/imu/data 뒤) run_lidar, run_motor, run_points, run_map, run_camera 를
#   tmux 세션 boat 에 창 하나씩 (Ctrl+B 숫자로 이동, Ctrl+B d 빠져나오기). run_zenoh 는 따로.
#   이미 켜져 있으면 붙기만. 끄기 stop_sensors_motors (모든 창 Ctrl+C → 세션 닫기)
alias run_sensors_motors='bash "$KABOAT_WS/scripts/run_sensors_motors.sh"'
alias stop_sensors_motors='bash "$KABOAT_WS/scripts/run_sensors_motors.sh" stop'
alias rec='python3 "$KABOAT_WS/scripts/rec_point.py"'
alias list_points='python3 "$KABOAT_WS/scripts/rec_point.py" --list'
# 주행 센서 녹화 (~/kaboat_bags/이름_날짜_시각/, Ctrl+C 로 종료). 센서를 다 켠 뒤 실행
#   카메라가 필요 없는 시험(회피·토너먼트·게이트)은 rec_bag --no-camera [이름] — 영상·압축 없이
alias rec_bag='bash "$KABOAT_WS/scripts/record_run.sh"'
# 카메라 컬러(RGB) 영상만 녹화 (~/kaboat_bags/이름_날짜_시각/, 이름 기본 rgb, Ctrl+C 로 종료)
alias rec_rgb='bash "$KABOAT_WS/scripts/record_run.sh" --rgb'
# bag 의 카메라 영상 → mp4 (bag 폴더 안 <폴더 이름>.mp4). bag_to_mp4 rgb_153828 [--topic …] [--fps N]
alias bag_to_mp4='python3 "$KABOAT_WS/scripts/bag_to_mp4.py"'
# 야외 웨이포인트 1개 주행 (실행 즉시 출발, 추력 40%):
#   goto gate_entry          rec 기록 지점 (위치 + 기록 선수각)
#   goto 12.0 -3.5 [90]      /odom x y [선수각, 생략 = 위치만]
#   뒤에 motion_enabled:=false (미리보기)·thrust_ratio:=0.3·heading:=false 등을 붙일 수 있다
goto() {
    local num='^-?[0-9]*[.]?[0-9]+$'
    if [ $# -eq 0 ]; then
        echo "사용: goto 지점이름 [옵션…]  또는  goto x y [선수각] [옵션…]"; return 1
    fi
    if [[ "$1" =~ $num ]]; then
        if ! [[ "$2" =~ $num ]]; then echo "y 좌표가 숫자가 아님: '$2'"; return 1; fi
        local target=(x:="$1" y:="$2"); shift 2
        if [[ "$1" =~ $num ]]; then target+=(angle:="$1"); shift; fi
        ros2 launch kaboat_hardware outdoor_waypoint.launch.py "${target[@]}" "$@"
    else
        local name="$1"; shift
        ros2 launch kaboat_hardware outdoor_waypoint.launch.py point:="$name" "$@"
    fi
}
alias run_motor="ros2 launch kaboat_hardware thrusters.launch.py hardware_type:=serial port:=/dev/ttyCH341USB0 allow_port_scan:=false"

# 노트북 원격 모니터링 통신 (Zenoh 브리지)
alias run_zenoh="python3 scripts/zenoh_monitor_link.py jetson"

# 롤/피치 자세 각도 모니터링
alias view_pitch="python3 scripts/view_roll_pitch.py"
# /odom x·y·선수각·나침반 방위·속도·GNSS(RTK) 상태 한 줄 표시
alias view_position='python3 "$KABOAT_WS/scripts/view_position.py"'

# 주행 테스트 실행 (노트북 시작 신호 없이 30% 기본 출력으로 즉시 시작)
alias test_bspline="ros2 launch kaboat_hardware bspline_track_test.launch.py wait_for_start:=false"
alias test_straight="ros2 launch kaboat_hardware straight_line_test.launch.py wait_for_start:=false"
alias test_circle="ros2 launch kaboat_hardware circle_drive_test.launch.py wait_for_start:=false"
alias test_station="ros2 launch kaboat_hardware station_keeping_test.launch.py wait_for_start:=false"
alias test_waypoint="ros2 launch kaboat_hardware waypoint_sequence_test.launch.py motion_enabled:=true thrust_ratio:=0.3 wait_for_start:=false"
alias test_los="ros2 launch kaboat_hardware los_path_test.launch.py path_kind:=bspline motion_enabled:=true wait_for_start:=false"
alias test_avoid="ros2 launch kaboat_hardware real_obstacle_avoid_test.launch.py drive_enabled:=true wait_for_start:=false"
# 야외 회피 시험: rec 기록 avoid_entry → avoid_end (goto avoid_entry 로 세운 뒤. drive_enabled:=false 면 미리보기)
alias test_avoid_outdoor="ros2 launch kaboat_hardware outdoor_avoid_test.launch.py"
# 토너먼트 회피 (종합 미션과 별개 경기장): rec 기록 tournament_entry → tournament_end. 인자는 test_avoid_outdoor 와 같음
alias test_tournament="ros2 launch kaboat_hardware outdoor_tournament_test.launch.py"
# 야외 게이트 시험: rec 기록 gate_entry → gate_end, 7쌍 (goto gate_entry 로 세운 뒤. motion_enabled:=false 면 미리보기)
alias test_gate_outdoor="ros2 launch kaboat_hardware outdoor_gate_test.launch.py"
# 야외 도킹 시험: rec 기록 dock_entry 에서 one-shot 도킹 (goto dock_entry 로 세운 뒤, run_camera 필요.
# YOLO CUDA venv 로 켬. target_class:=circle, motion_enabled:=false 면 경로만 미리보기)
alias test_dock_outdoor='bash "$KABOAT_WS/scripts/run_dock_mission.sh"'
# 도킹 YOLO 마스크만 확인 (run_dock_yolo.sh 를 먼저 켜 둠): 도형별 신뢰도·화면 위치·검출률,
# 마스크 그린 영상 PNG 저장 (~/kaboat_yolo_checks). Ctrl+C 로 요약
alias check_yolo='python3 "$KABOAT_WS/scripts/check_yolo_masks.py"'
# 종합 자율주행: 게이트 → 스테이션 → 도킹 → 탐색 → 회피, 전부 실제로 움직임 (센서·모터·run_map·카메라는 따로).
#   run_autonomy search_target_color:=red dock_target_class:=circle   (뒤에 붙인 인자가 이김)
alias run_autonomy='ros2 launch kaboat_bringup autonomy.launch.py gate_motion_enabled:=true approach_motion_enabled:=true dock_motion_enabled:=true'
# 미션 하나만: 그 미션 노드만 띄워(접근·미션 매니저·cmd_mux 포함) _entry 로 접근 → 미션 → done(정지)
#   test_mission station | test_mission search search_target_color:=red | test_mission dock dock_target_class:=circle
test_mission() {
    if [ -z "$1" ]; then
        echo "사용: test_mission gate|station|dock|search|avoid [인자:=값 …]"
        return 1
    fi
    local mission="$1"
    shift
    ros2 launch kaboat_bringup autonomy.launch.py start_state:="$mission" single_mission:=true \
        gate_motion_enabled:=true approach_motion_enabled:=true dock_motion_enabled:=true "$@"
}
# goto <미션>_entry → 시작 조건(거리·선수각)을 0.5 s 유지하면 goto 를 끄고 그 미션 시험을 바로 켬 (goto 손으로 안 꿈).
#   기본 조건: 게이트·도킹·스테이션·탐색 2 m·기록 선수각 ±30°, 회피 3 m·목표(_end) 방향 ±45°.
#   go_avoid --dist 1.5 --heading 20 --goto thrust_ratio:=0.3 drive_enabled:=false  (옵션 → 시험 인자 순)
# 토너먼트 한 번에 (슬롯 출발): 출발 직진 3 s → goto tournament_entry → 토너먼트 회피 시험, 노드를 처음에 다 띄우고
#   cmd_mux 로 명령만 넘김 — 단계 사이 명령 공백 없음 (2026-11-14). 인자는 이름:=값:
#   go_tournament drive_enabled:=false | undock_sec:=0 | start_dist:=1.5 start_heading_deg:=20 goto_thrust_ratio:=0.3
alias go_tournament='ros2 launch kaboat_hardware outdoor_tournament_run.launch.py'
alias go_avoid='python3 "$KABOAT_WS/scripts/go_and_test.py" avoid'
alias go_gate='python3 "$KABOAT_WS/scripts/go_and_test.py" gate'
alias go_dock='python3 "$KABOAT_WS/scripts/go_and_test.py" dock'
alias go_station='python3 "$KABOAT_WS/scripts/go_and_test.py" station'
alias go_search='python3 "$KABOAT_WS/scripts/go_and_test.py" search'

echo "🤖 [KABOAT Jetson] 환경 세팅 완료 (DOMAIN: $ROS_DOMAIN_ID | WS: ${KABOAT_WS:-미탐색})"
echo "   - run_boat_lidar: 선체 TG-50 포함 센서/오도메트리 가동 (cal_imu 불필요, 즉시 /odom 생성)"
echo "   - run_gq7_outdoor : 야외 GQ7 (/odom 원점 = ~/kaboat_geo_origin.yaml, 필터 위치)"
echo "   - run_gq7_rtk   : 야외 GQ7 + RTK 보정 (계정 ~/.ntrip_account)"
echo "   - run_lidar     : 야외 선체 TG-50 라이다 (CP2102 고정 경로, GQ7 먼저)"
echo "   - run_geo       : GPS 원점 설정·위도경도→x/y 변환 (geo_origin)"
echo "   - run_camera    : D455 카메라 (컬러 + 정렬 뎁스)"
echo "   - run_map       : 장애물 지도 (라이다 + 가상 벽, 카메라 안 씀) — 하나만 켜 둠"
echo "   - run_points    : 미션 지점 기록기 (~/kaboat_mission_points.json)"
echo "   - run_sensors_motors : GQ7(RTK)·라이다·모터·지점·지도·카메라 한 번에 (tmux 창 0~5, zenoh 는 따로)"
echo "   - stop_sensors_motors: 위 세션 전부 Ctrl+C 후 닫기"
echo "   - rec 이름      : 지금 위치(x, y, 선수각) 기록 (이름 생략 시 p1, p2 …)"
echo "   - list_points   : 기록한 지점 보기"
echo "   - rec_bag [이름] : 주행 센서 rosbag 녹화 (~/kaboat_bags, Ctrl+C 로 종료). 카메라 빼기: rec_bag --no-camera [이름]"
echo "   - rec_rgb [이름] : 카메라 RGB 영상만 녹화 (~/kaboat_bags, Ctrl+C 로 종료, mp4 는 bag_to_mp4)"
echo "   - goto 지점|x y [각도] : 야외 웨이포인트 1개 주행 (즉시 출발 40%, motion_enabled:=false 면 미리보기)"
echo "   - run_motor     : 모터 제어기 가동"
echo "   - cal_imu       : [선택] IMU 정지 수동 보정 (수조 라이다 Truth 모드에서는 생략 가능)"
echo "   - test_bspline  : B-Spline 곡선 자율주행 (출력 30%, 즉시 시작)"
echo "   - test_straight : 직선 주행 테스트 (출력 30%, 즉시 시작)"
echo "   - test_circle   : 원형 선회 주행 테스트 (출력 30%, 즉시 시작)"
echo "   - test_station  : 정점 유지 DP 테스트 (출력 30%, 즉시 시작)"
echo "   - test_waypoint : 5개 웨이포인트 순차 주행 (출력 30%, 즉시 시작)"
echo "   - test_los      : LOS 경로 추종 테스트 (SOTA 직진 0.25·회전 0.31, 즉시 시작)"
echo "   - test_avoid    : 실시간 장애물 회피 주행 (즉시 시작)"
echo "   - test_avoid_outdoor : 야외 회피 avoid_entry → avoid_end (시작점 3 m·목표 방향 ±45° 안이면 즉시 출발)"
echo "   - test_tournament    : 토너먼트 회피 tournament_entry → tournament_end (test_avoid_outdoor 와 같은 조건)"
echo "   - test_gate_outdoor  : 야외 게이트 7쌍 gate_entry → gate_end (즉시 출발)"
echo "   - test_dock_outdoor  : 야외 도킹 dock_entry → 목표 슬롯 → 후진 (YOLO 결과는 rec_bag 이 저장)"
echo "   - check_yolo    : 도킹 YOLO 마스크 확인 (run_dock_yolo.sh 먼저, 영상 PNG ~/kaboat_yolo_checks)"
echo "   - run_autonomy  : 종합 자율주행 전체 (게이트→스테이션→도킹→탐색→회피, 실제 주행). 예) search_target_color:=red"
echo "   - test_mission 미션 : 미션 하나만, 그 미션 노드만 띄움 (예: test_mission station, 끝나면 정지)"
echo "   - go_tournament : 토너먼트 한 번에 — 출발 직진 3 s → goto tournament_entry → 토너먼트 회피 시험 (공백 없이)"
echo "   - go_avoid / go_gate / go_dock / go_station / go_search : goto 그 미션 _entry → 조건 만족하면 바로 그 시험 시작"
echo "   - view_position : /odom x·y·선수각·나침반 방위·GNSS 상태 실시간 표시"
echo "   - run_zenoh     : 노트북 모니터링 브리지 가동"
echo "   - cb / sb       : 빌드 / 환경 리로드"
# <<< KABOAT JETSON ENVIRONMENT <<<
EOF

echo "✅ 젯슨 ~/.bashrc 설정이 성공적으로 등록되었습니다!"
echo "👉 바로 적용하려면 'source ~/.bashrc'를 실행하세요."
