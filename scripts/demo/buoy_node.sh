#!/bin/bash
# 부표 검출기 — /detections/buoys(BuoyArray) + ~/markers(RViz) + ~/debug_image (라이다 = 위치, 카메라 = 색)
# 구독자가 있을 때만 계산한다 — RViz(rviz_buoy.sh)를 같이 켜거나 /detections/buoys 를 구독할 것.
source "$(dirname "${BASH_SOURCE[0]}")/_env.sh"
exec ros2 run kaboat_perception buoy_lidar_detector "$@"
