#!/bin/bash
# RViz — 점유 지도 + 라이다 + odom + 부표맵(buoy_lidar_detector 마커) + 카메라 디버그 영상
source "$(dirname "${BASH_SOURCE[0]}")/_env.sh"
exec rviz2 -d "$HERE/buoy_lidar.rviz"
