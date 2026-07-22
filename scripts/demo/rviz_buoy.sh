#!/bin/bash
# RViz — 점유 지도 + 라이다 + odom + 부표맵(색 구·id) + 카메라 영상
source "$(dirname "${BASH_SOURCE[0]}")/_env.sh"
exec rviz2 -d "$HERE/buoy.rviz"
