#!/bin/bash
# 부표 데모용 sim — 기본 spawn(게이트 채널 (2,63) 정면). avoid 스택을 안 켜서
# 배가 빨강-초록 게이트 부표를 마주 보고 멈춰 있어 부표맵이 안정적으로 잡힌다.
source "$(dirname "${BASH_SOURCE[0]}")/_env.sh"
exec ros2 launch kaboat_sim simulation.launch.py headless:=True
