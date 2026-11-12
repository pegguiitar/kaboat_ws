#!/bin/bash
# 탐색 미션 단독 스택 — 지도 + 부표 검출 + search_circler + mission_manager(search 하나만) + 드라이버.
# 사용: search_stack.sh [색 red|green|black] [search_end x] [search_end y]
#   기본 = red, mission_params.yaml 의 search (57, 76) — 빨강 부표 위에 찍힌 값이라
#   완료 뒤 부표에서 1.5 m 비킨 점으로 가는 것(exit_point)까지 같이 확인된다.
# sim 은 sim_search.sh 로 먼저 띄운다. RViz 는 rviz_search.sh.
source "$(dirname "${BASH_SOURCE[0]}")/_env.sh"
COLOR=${1:-red}; GX=${2:-57.0}; GY=${3:-76.0}
SIM=(--ros-args -p use_sim_time:=true)
ros2 run kaboat_perception occupancy_grid "${SIM[@]}" &
ros2 run kaboat_perception buoy_detector "${SIM[@]}" &
ros2 run kaboat_control cmd_mux "${SIM[@]}" &
python3 "$WS/src/kaboat_sim/launch/twist2thrust.py" "${SIM[@]}" &
ros2 run kaboat_behaviors search_circler "${SIM[@]}" -p search.target_color:="$COLOR" &
sleep 3   # behavior 들이 뜬 뒤 미션 시작
exec ros2 run kaboat_mission mission_manager "${SIM[@]}" \
  -p start_state:=search -p single_mission:=true -p entry_required:=false \
  -p points_file:=none -p "waypoints.search:=[$GX, $GY]"
