#!/usr/bin/env bash
# Existing scripts/run_dock_yolo.sh supplies the camera + three marker projections.
set -eo pipefail
exec 9>/tmp/kaboat_dock_marker_avoid.lock
if ! flock -n 9; then
    echo 'The marker avoidance mission is already running.' >&2
    exit 1
fi
repo_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/humble/setup.bash
source "$repo_dir/install/setup.bash"
export ROS_LOG_DIR="${ROS_LOG_DIR:-/tmp/kaboat_dock_marker_avoid_logs}"
exec ros2 launch kaboat_bringup dock_marker_avoid.launch.py "$@"
