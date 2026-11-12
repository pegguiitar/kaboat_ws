#!/usr/bin/env bash
# Build only the competition docking runtime on the boat. No VRX/Gazebo source.
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/.."
source /opt/ros/humble/setup.bash
export MAKEFLAGS='-j1 -l1'
export CMAKE_BUILD_PARALLEL_LEVEL=1
exec nice -n 10 colcon build --symlink-install \
  --executor sequential --parallel-workers 1 \
  --packages-select kaboat_msgs kaboat_behaviors kaboat_perception \
                    kaboat_hardware kaboat_control kaboat_mission kaboat_bringup \
  --cmake-args -DBUILD_TESTING=OFF
