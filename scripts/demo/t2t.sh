#!/bin/bash
# 드라이버 — /cmd_vel(최대추력 비율) → 좌우 스러스터 N
source "$(dirname "${BASH_SOURCE[0]}")/_env.sh"
exec python3 "$WS/src/kaboat_sim/launch/twist2thrust.py"
