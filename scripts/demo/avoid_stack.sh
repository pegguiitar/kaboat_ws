#!/bin/bash
# 회피 자율운항 스택 — 점유 지도 + 명령 먹스 + 회피 플래너
# (sim 과 드라이버는 sim_avoid.sh / t2t.sh 로 따로 띄운다)
source "$(dirname "${BASH_SOURCE[0]}")/_env.sh"
ros2 run kaboat_perception occupancy_grid &
ros2 run kaboat_control cmd_mux &
exec ros2 run kaboat_behaviors obstacle_planner
