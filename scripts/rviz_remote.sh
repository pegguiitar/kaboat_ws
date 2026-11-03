#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/jazzy/setup.bash
set -u
export ROS_DOMAIN_ID=43

workspace_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
scan_config="$workspace_dir/src/ydlidar_ros2_driver/config/ydlidar.rviz"
output_config=/tmp/boat_scan.rviz

if ! grep -q 'Fixed Frame: laser_frame' "$scan_config"; then
    echo "RViz 설정에서 laser_frame을 찾지 못했습니다: $scan_config" >&2
    exit 1
fi

sed 's/Fixed Frame: laser_frame/Fixed Frame: onboard_laser_frame/' \
    "$scan_config" > "$output_config"
exec rviz2 -d "$output_config"
