#!/bin/bash
# 라이다 중심 부표 검출(buoy_lidar_detector)을 bag 으로 반복 재생해 RViz 로 본다.
# 사용: replay_buoy_lidar.sh <bag 폴더> [재생 배속, 기본 1.0]
#   RViz: 라이다 점(/scan/level) + 점유지도 + 부표 마커(존재 확률) + 카메라 디버그 화면(창·색).
#   bag 의 카메라 토픽은 D455 드라이버 기본 이름(/camera/camera/color/image_raw/compressed).
#   반복은 loop_bag_play.py — 반복마다 시각을 bag 길이만큼 앞으로 민다 (ros2 bag play --loop 는
#   시간이 거꾸로 가서 Humble RViz 가 죽는다). 끝내려면 Ctrl+C.
source "$(dirname "${BASH_SOURCE[0]}")/_env.sh"
BAG=${1:?bag 폴더를 주세요}; RATE=${2:-1.0}
ros2 run kaboat_perception buoy_lidar_detector --ros-args -p use_sim_time:=true \
  -p color_topic:=/camera/camera/color/image_raw/compressed -p color_compressed:=true \
  -p depth_topic:=/camera/camera/aligned_depth_to_color/image_raw/compressedDepth -p depth_compressed:=true &
rviz2 -d "$HERE/buoy_lidar.rviz" --ros-args -p use_sim_time:=true &
sleep 3
exec python3 "$HERE/loop_bag_play.py" "$BAG" "$RATE"
