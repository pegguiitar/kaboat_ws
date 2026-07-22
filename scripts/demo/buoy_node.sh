#!/bin/bash
# 부표 검출기 — /detections/buoys(BuoyArray) + /detections/buoys_viz(RViz 마커)
source "$(dirname "${BASH_SOURCE[0]}")/_env.sh"
exec ros2 run kaboat_perception buoy_detector
