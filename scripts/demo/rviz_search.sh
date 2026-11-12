#!/bin/bash
# RViz — 부표맵 + 탐색 목표 부표·선회 원(/search/markers) + 지도 + 카메라
source "$(dirname "${BASH_SOURCE[0]}")/_env.sh"
exec rviz2 -d "$HERE/search.rviz"
