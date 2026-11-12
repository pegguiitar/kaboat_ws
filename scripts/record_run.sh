#!/usr/bin/env bash
# ==============================================================================
# 실제 주행 센서 녹화 (rosbag) — 노트북에서 알고리즘을 재생 시험하기 위한 기록
#
# 젯슨에서 센서(run_gq7_rtk, run_lidar, run_camera …)를 켠 뒤 실행:
#   bash scripts/record_run.sh [이름]      # 단축어: rec_bag [이름]
# Ctrl+C 로 멈추면 녹화가 정리되고 끝난다. 터미널 창을 그냥 닫지 말 것.
#
# 카메라는 압축 토픽(컬러 JPEG, 뎁스 무손실 PNG)으로 저장한다. 압축 토픽이 없으면
# (image_transport 플러그인 미설치) 원본 영상을 zstd 로 압축해 저장한다.
# 저장 위치: ~/kaboat_bags/<이름>_<날짜_시각>/  (KABOAT_BAG_DIR 로 변경)
# ==============================================================================

set -eo pipefail

BAG_DIR="${KABOAT_BAG_DIR:-$HOME/kaboat_bags}"
MIN_FREE_GB="${KABOAT_BAG_MIN_FREE_GB:-10}"
SPLIT_BYTES=$((2 * 1024 * 1024 * 1024))   # 2 GB 마다 파일을 나눈다

name="${1:-run}"
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
if [ -n "$color_c" ] && [ -n "$depth_c" ]; then
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
