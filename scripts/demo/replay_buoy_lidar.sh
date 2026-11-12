#!/bin/bash
# 라이다 중심 부표 검출(buoy_lidar_detector)을 bag 으로 재생해 RViz 로 본다.
# 사용: replay_buoy_lidar.sh <bag 폴더> [재생 배속, 기본 1.0]
#   한 번만 재생한다 — --loop 로 처음으로 돌아가면 시뮬 시간이 거꾸로 가서 Humble RViz 가
#   "Resetting RViz" 하다 죽는다. 다시 보려면 스크립트를 다시 실행.
#   RViz: 라이다 점(/scan/level) + 부표 마커(확률·색·근거) + 카메라 디버그 화면(창·색 표).
#   bag 의 카메라 토픽은 D455 드라이버 기본 이름(/camera/camera/color/image_raw/compressed).
source "$(dirname "${BASH_SOURCE[0]}")/_env.sh"
BAG=${1:?bag 폴더를 주세요}; RATE=${2:-1.0}
ros2 run kaboat_perception buoy_lidar_detector --ros-args -p use_sim_time:=true \
  -p color_topic:=/camera/camera/color/image_raw/compressed -p color_compressed:=true &
rviz2 -d "$HERE/buoy_lidar.rviz" --ros-args -p use_sim_time:=true &
sleep 3
ros2 bag play "$BAG" --clock -r "$RATE"
echo "재생 끝 — RViz 는 마지막 상태로 남아 있다 (Ctrl+C 로 종료)"
wait
