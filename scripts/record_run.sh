#!/usr/bin/env bash
# ==============================================================================
# 실제 주행 센서 녹화 (rosbag) — 노트북에서 알고리즘을 재생 시험하기 위한 기록
#
# 젯슨에서 센서(run_gq7_rtk, run_lidar, run_camera …)를 켠 뒤 실행:
#   bash scripts/record_run.sh [이름]        # 단축어: rec_bag [이름]
#   bash scripts/record_run.sh --rgb [이름]  # 단축어: rec_rgb [이름] — 카메라 컬러 영상만 (이름 기본 rgb)
#   bash scripts/record_run.sh --no-camera [이름]  # rec_bag --no-camera — 카메라 빼고 (회피·토너먼트·게이트)
# Ctrl+C 로 멈추면 녹화가 정리되고 끝난다. 터미널 창을 그냥 닫지 말 것.
# 영상으로 보기: bag_to_mp4 ~/kaboat_bags/<폴더>  → 폴더 안 <폴더 이름>.mp4
#
# 카메라는 압축 토픽(컬러 JPEG, 뎁스 무손실 PNG)으로 저장한다. 압축 토픽이 없으면
# (image_transport 플러그인 미설치) 원본 영상을 zstd 로 압축해 저장한다 — 젯슨이 압축을 못 따라가
# 메시지를 버린다 (2026-11-13 토너먼트: 'Dropping message … compression queue is full', 컬러 영상
# 1/3·로그 일부 누락, /rosout 10 s 지연). 카메라가 필요 없는 시험은 --no-camera: 영상·압축 없이.
# 저장 위치: ~/kaboat_bags/<이름>_<날짜_시각>/  (KABOAT_BAG_DIR 로 변경)
# ==============================================================================

set -eo pipefail

BAG_DIR="${KABOAT_BAG_DIR:-$HOME/kaboat_bags}"
MIN_FREE_GB="${KABOAT_BAG_MIN_FREE_GB:-10}"
SPLIT_BYTES=$((2 * 1024 * 1024 * 1024))   # 2 GB 마다 파일을 나눈다

rgb_only=false
no_camera=false
while [ $# -gt 0 ]; do
    case "$1" in
        --rgb) rgb_only=true; shift ;;
        --no-camera|--nocam) no_camera=true; shift ;;
        --) shift; break ;;
        -*) echo "❌ 모르는 옵션: $1 (쓸 수 있는 것: --rgb, --no-camera)" >&2; exit 2 ;;
        *) break ;;
    esac
done
if $rgb_only && $no_camera; then
    echo "❌ --rgb(영상만)와 --no-camera(영상 빼기)는 같이 못 씀" >&2
    exit 2
fi
if $rgb_only; then name="${1:-rgb}"; else name="${1:-run}"; fi
out="$BAG_DIR/${name}_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$BAG_DIR"

free_gb=$(df -BG --output=avail "$BAG_DIR" | tail -1 | tr -dc '0-9')
if [ "$free_gb" -lt "$MIN_FREE_GB" ]; then
    echo "❌ 남은 용량 ${free_gb} GB < ${MIN_FREE_GB} GB. ~/kaboat_bags 의 지난 녹화를 옮기거나 지울 것." >&2
    exit 1
fi

# 센서 원본 + 재생 시 비교할 당시 판단 결과
topics=(
    /scan /scan/raw /scan/level
    /imu/data /gps/fix /gps/fix_secondary /odom
    /tf /tf_static
    /camera/camera_info
    /cmd_vel /occupancy_grid
    /mission/state /mission/goal
    /detections/buoys /dock/lidar/status
    /geo/origin /geo/status /waypoint/status
    /mission/complete /mission/entry_goal
    # 노드 로그 — 회피 ESCAPE 진입 이유처럼 토픽으로는 안 나오는 판단이 여기에만 남는다
    /rosout
    # GQ7: RTK 상태(/ekf/status 의 gnss_state), 필터 원본, 듀얼 안테나·GNSS 보정 진단,
    # NTRIP 보정 입력(/rtcm)·AUX NMEA 출력 — 'RTK 가 그때 풀렸나'를 나중에 확인
    /ekf/status /ekf/odometry_earth /ekf/dual_antenna_heading /gnss_1/velocity
    /mip/ekf/gnss_dual_antenna_status /mip/ekf/aiding_measurement_summary
    /mip/ekf/gnss_position_aiding_status /rtcm /nmea
    # rtcm_guard: GQ7 로 넘긴 보정(/rtcm/gq7)과 RTK·AUX 상태 한 줄 (AUX 쓰기 실패면 중계 멈춤)
    /rtcm/gq7 /gnss/rtk_status
    # 수동·자율 구분: 노트북 텔레옵·RC·비상정지 입력, 모터 드라이버 상태·이벤트
    /teleop/enable /teleop/levels /teleop/esp_mode /teleop/status /teleop/event
    /rc/cmd_vel /emergency_stop /diagnostics
    # 미션 사이 이동·지점·인지 시각화
    /cmd/approach /mission/approach_cmd_preview
    /geo/position /geo/targets /geo/target_local /geo/target_markers
    /mission_points/markers /mission_points/result
    /occupancy_grid/cam_points
    # 부표 지도 마커 — /detections/buoys 와 함께 녹화하면 buoy_lidar_detector 가 구독자를 보고
    # 미션과 상관없이 계속 계산한다 (평소엔 탐색 동안만)
    /buoy_lidar_detector/markers
    # 장애물 회피 판단: 위험도 지도(DRI)·B-spline 경로·제어점·고위험 칸·명령
    # (구독자가 있을 때만 만드는 토픽이라 녹화를 켜면 젯슨이 10 Hz 로 만들기 시작한다)
    /avoid/dri /avoid/plan /avoid/cps /avoid/dri_marks /cmd/avoid
    # 스테이션 키핑(전용 지도 140×140, 구독자 있을 때만)·수색
    /station/map /station/markers /station/status /station/cmd_preview /cmd/station
    /search/markers /search/status /cmd/search
    # 도킹 YOLO 결과: 마커별 신뢰도·픽셀·시선(JSON), 마스크 그린 영상(320 px 5 fps ≈ 1.2 MB/s), 시선 방향
    /detector/enable /dock/detection_status /detections/dock_marks /dock/debug_image
    /dock/bearing_markers /dock/markers/{triangle,circle,square}/ray
    /dock/markers/{triangle,circle,square}/depth_ground
    /dock/markers/{triangle,circle,square}/{ground,base,ground_base}
    /dock/marker_base /dock/marker_ground /dock/marker_ground_raw
    /dock/markers_base /dock/markers_ground
    # 도킹 라이다 RANSAC 벽·목표·one-shot 경로·명령
    /dock/lidar/markers /dock/lidar/direct_markers
    /dock/lidar/{triangle,circle,square}/normal /dock/lidar/{triangle,circle,square}/wall_point
    /dock/lidar/{triangle,circle,square}/{marker_ground,wall_point_base,marker_base,normal_base}
    /dock/waypoints /dock/avoid/goal /dock/avoid/goal_status
    /dock/avoid/goal_candidate /dock/avoid/goal_markers
    /dock/{approach,alignment,stop}_point /dock/entrance_base /dock/bspline_path
    /dock/status /dock/one_shot_path /dock/one_shot_gps /dock/one_shot_markers
    /dock/cmd_preview /cmd/dock /dock/slot_markers
    # 게이트 판단: 상태(WAIT·PASSED…)·고른 쌍·미리 이은 쌍·기억한 부표(MarkerArray)·경로·명령
    # (2026-11-12 4번 시험은 이게 없어 멈춘 이유를 영상 속 터미널로 읽었다)
    /gate/status /gate/pairs /gate/path /gate/cmd_preview /cmd/gate
    # 야외 단독 시험 노드 (회피·DRI·LOS 시험 런치)
    /real_avoid_test/path /real_avoid_test/status /manual_dri/dri /manual_dri/status
    /los_test/path /los_test/status /los_test/cmd_preview /waypoint/goal
)

available="$(ros2 topic list 2>/dev/null || true)"
has() { grep -qx "$1" <<< "$available"; }

# d455.launch.py 의 SetRemap 은 image_transport 하위 토픽까지 바꾸지 않을 수 있어 두 이름을 다 본다
color_c=""; depth_c=""
for base in /camera/color/image_raw /camera/camera/color/image_raw; do
    has "$base/compressed" && { color_c="$base/compressed"; break; }
done
for base in /camera/depth/image_raw /camera/camera/aligned_depth_to_color/image_raw; do
    has "$base/compressedDepth" && { depth_c="$base/compressedDepth"; break; }
done

extra=()
if $no_camera; then
    # 카메라 토픽(camera_info·도킹 YOLO 그림 포함)을 빼고, 영상이 없으니 전체 압축도 안 한다
    kept=()
    for t in "${topics[@]}"; do
        case "$t" in
            /camera/*|/dock/debug_image) ;;
            *) kept+=("$t") ;;
        esac
    done
    topics=("${kept[@]}")
    echo "📷 카메라 빼고 녹화 (영상·압축 없음 — 젯슨 부하 최소)"
elif $rgb_only; then
    # 컬러 영상 하나만 (센서·판단 토픽, 뎁스, camera_info 없음)
    if [ -n "$color_c" ]; then
        topics=("$color_c")
        echo "📷 RGB 영상만: $color_c (JPEG, 약 1~1.5 MB/s)"
    else
        color_raw=/camera/color/image_raw
        if ! has "$color_raw" && has /camera/camera/color/image_raw; then
            color_raw=/camera/camera/color/image_raw
        fi
        topics=("$color_raw")
        extra+=(--compression-mode message --compression-format zstd)
        if has "$color_raw"; then
            echo "⚠️  압축 토픽 없음 → RGB 원본 + zstd ($color_raw, 약 7~14 MB/s)"
            echo "   다음부터 압축 토픽을 쓰려면: sudo apt install ros-humble-image-transport-plugins"
        else
            echo "⚠️  카메라 토픽이 아직 없음 (run_camera 안 켬?) — 나중에 켜지면 $color_raw 원본 + zstd 로 녹화된다"
        fi
    fi
elif [ -n "$color_c" ] && [ -n "$depth_c" ]; then
    topics+=("$color_c" "$depth_c")
    echo "📷 카메라: 압축 토픽 ($color_c, $depth_c) — 약 4~5 MB/s"
else
    topics+=(/camera/color/image_raw /camera/depth/image_raw)
    extra+=(--compression-mode message --compression-format zstd)
    if has /camera/color/image_raw; then
        echo "⚠️  압축 토픽 없음 → 원본 영상 + zstd (약 12~15 MB/s, CPU 부하 조금 더)"
        echo "   다음부터 압축 토픽을 쓰려면: sudo apt install ros-humble-image-transport-plugins"
    else
        echo "⚠️  카메라 토픽이 아직 없음 (run_camera 안 켬?) — 나중에 켜지면 원본 + zstd 로 녹화된다"
    fi
fi

echo "💾 녹화 시작 → $out  (남은 용량 ${free_gb} GB)"
echo "   멈추려면 Ctrl+C"
exec ros2 bag record -o "$out" --max-bag-size "$SPLIT_BYTES" "${extra[@]}" "${topics[@]}"
