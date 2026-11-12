#!/bin/bash
# 탐색 검증용 sim — 우측 탐색 구역 서쪽 (44, 76)에서 동쪽을 보고 스폰.
# 정면 13 m 에 빨강(57,76), 왼쪽 앞 주황(52,80)·초록(62,80), 오른쪽 앞 주황(55,71).
# GUI 로 띄운다 (headless:=True 를 넘기면 서버만). 탐색 스택은 search_stack.sh.
source "$(dirname "${BASH_SOURCE[0]}")/_env.sh"

# 이전 sim이 비정상 종료되면서 남긴 odom publisher를 제거한다 (sim_avoid.sh 와 같음).
pkill -TERM -f '[o]dom_gps_noise.py' 2>/dev/null || true
for _ in {1..20}; do
  pgrep -f '[o]dom_gps_noise.py' >/dev/null || break
  sleep 0.1
done
pkill -KILL -f '[o]dom_gps_noise.py' 2>/dev/null || true

exec ros2 launch kaboat_sim simulation.launch.py spawn_x:=44.0 spawn_y:=76.0 spawn_yaw:=0.0 "$@"
