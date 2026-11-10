#!/usr/bin/env bash
set -eo pipefail
exec 9>/tmp/kaboat_dock_marker_avoid.lock
if ! flock -n 9; then
    echo 'Docking controller already running; stop the existing launch first.' >&2
    exit 1
fi
repo_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/humble/setup.bash
source "$repo_dir/install/setup.bash"
export ROS_LOG_DIR="${ROS_LOG_DIR:-/tmp/kaboat_dock_ctrl_logs}"
exec ros2 launch kaboat_bringup dock_ctrl.launch.py drive_enabled:=true start_thrusters:=true "$@"
