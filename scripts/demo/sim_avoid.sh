#!/bin/bash
# 회피 검증용 sim — 부표 밭 서쪽(2, 75)에 스폰. 부표 행 라인 정면이라
# 출발하자마자 회피 기동이 나온다(통로 정중앙 76.5 는 직진만 함).
source "$(dirname "${BASH_SOURCE[0]}")/_env.sh"
exec ros2 launch kaboat_sim simulation.launch.py headless:=True spawn_y:=75.0
