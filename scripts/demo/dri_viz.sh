#!/bin/bash
# 위험도 지도를 RViz 용 격자(/avoid/dri)로 발행 — 파라미터 튜닝용.
# `ros2 param set /dri_viz k_head 1.5` 처럼 런타임에 즉시 반영된다.
source "$(dirname "${BASH_SOURCE[0]}")/_env.sh"
export PYTHONPATH="$WS/src/kaboat_behaviors:$PYTHONPATH"
exec python3 "$HERE/dri_viz.py" --ros-args -p use_sim_time:=true
