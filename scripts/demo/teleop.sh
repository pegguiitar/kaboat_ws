#!/bin/bash
# 키보드 teleop — WASD 로 /cmd_vel 발행 (twist2thrust 가 추력으로 변환).
# ⚠️ 반드시 대화형 터미널에서 실행 (키 입력 포커스 필요):
#   docker exec -it kaboat_sim_container bash -lc 'scripts/demo/teleop.sh'
source "$(dirname "${BASH_SOURCE[0]}")/_env.sh"
exec python3 "$WS/src/kaboat_sim/launch/keyboard_teleop.py"
