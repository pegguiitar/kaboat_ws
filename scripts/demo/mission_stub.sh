#!/bin/bash
# mission_manager 대역 — avoid 상태와 목표점을 latched 로 계속 발행한다.
# 회피만 따로 검증할 때 미션 전체를 띄우지 않으려고 쓴다.
# 사용: mission_stub.sh <goal_x> <goal_y>   (기본 = 부표 밭 관통 목표)
source "$(dirname "${BASH_SOURCE[0]}")/_env.sh"
GX=${1:-29.0}; GY=${2:-76.5}
ros2 topic pub --qos-durability transient_local --qos-depth 1 -r 1 \
  /mission/state std_msgs/msg/String "{data: avoid}" &
exec ros2 topic pub --qos-durability transient_local --qos-depth 1 -r 1 \
  /mission/goal geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: odom}, pose: {position: {x: $GX, y: $GY}, orientation: {w: 1.0}}}"
