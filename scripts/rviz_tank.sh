#!/usr/bin/env bash
set -euo pipefail

if [[ -z "${DISPLAY:-}" ]]; then
    echo '그래픽 데스크톱 터미널에서 실행하세요: DISPLAY가 비어 있습니다.' >&2
    exit 1
fi

# Use the same ROS environment as the shore LiDAR tracker.
xhost +si:localuser:root >/dev/null
exec docker exec -it -e DISPLAY="$DISPLAY" kaboat_lidar bash -lc \
    'source /opt/ros/humble/setup.bash && source /workspace/kaboat_ws/install/setup.bash && export ROS_DOMAIN_ID=43 ROS_LOCALHOST_ONLY=0 && exec rviz2 -d /workspace/kaboat_ws/src/kaboat_bringup/rviz/tank_tracking.rviz'
