#!/usr/bin/env bash
# Container-only build with one CMake job and one colcon package at a time.
set -eo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/.."
source /opt/ros/humble/setup.bash
set -u

# Keep the unchanged Compose command; cap the existing named container with
# `docker update --cpus 5 --memory 4g --memory-swap 5g kaboat_sim_container`.
if [[ -r /sys/fs/cgroup/memory.max ]]; then
  memory_limit=$(cat /sys/fs/cgroup/memory.max)
  if [[ "$memory_limit" == max || "$memory_limit" -gt 4294967296 ]]; then
    echo 'Refusing unbounded build: cap kaboat_sim_container at 4 GiB.' >&2
    exit 1
  fi
fi
if [[ -r /sys/fs/cgroup/cpu.max ]]; then
  read -r cpu_quota cpu_period < /sys/fs/cgroup/cpu.max
  if [[ "$cpu_quota" == max || "$cpu_quota" -gt $((5 * cpu_period)) ]]; then
    echo 'Refusing unbounded build: cap kaboat_sim_container at 5 CPUs.' >&2
    exit 1
  fi
fi

if [[ "${1:-}" == '--check-only' ]]; then
  echo 'Docker CPU/memory limits are active; build was not started.'
  exit 0
fi

export MAKEFLAGS='-j1 -l1'
export CMAKE_BUILD_PARALLEL_LEVEL=1
exec nice -n 10 colcon build --symlink-install \
  --executor sequential --parallel-workers 1 \
  --packages-ignore ydlidar_ros2_driver \
  --packages-select kaboat_msgs kaboat_sim kaboat_behaviors kaboat_perception \
                    kaboat_hardware kaboat_control kaboat_mission kaboat_bringup \
  --cmake-args -DBUILD_TESTING=OFF
