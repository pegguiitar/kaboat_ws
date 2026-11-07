#!/usr/bin/env bash
set -eo pipefail
repo_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/humble/setup.bash
source "$repo_dir/install/setup.bash"
export ROS_LOG_DIR=/tmp/kaboat_topic_monitors
exec "$repo_dir/.venv-dock-yolo/bin/python" "$repo_dir/scripts/monitor_dock_topic.py" "$@"
