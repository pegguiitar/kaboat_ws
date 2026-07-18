#!/bin/bash
# RViz — 점유 지도 + 위험도 + 경로/제어점/목표점
source "$(dirname "${BASH_SOURCE[0]}")/_env.sh"
exec rviz2 -d "$HERE/avoid.rviz"
