#!/usr/bin/env bash
# Middle (red triangle) dock slot: world centre (70, 66), boat faces +X.
set -eo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/_env.sh"
set -u
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1
# On this Intel Iris Xe host, Mesa's auto selection fell back to swrast and
# consumed three CPU cores in the Gazebo GUI. Keep user overrides intact.
if [[ -z "${MESA_LOADER_DRIVER_OVERRIDE:-}" ]] && \
   [[ -r /sys/class/drm/card1/device/vendor ]] && \
   [[ "$(cat /sys/class/drm/card1/device/vendor)" == '0x8086' ]]; then
  unset LIBGL_ALWAYS_SOFTWARE
  export MESA_LOADER_DRIVER_OVERRIDE=iris
fi
setsid ros2 launch kaboat_sim simulation.launch.py \
  spawn_x:=58.0 spawn_y:=66.0 spawn_yaw:=0.0 \
  odom_noise_sigma:=0.0 sim_camera_enabled:=true "$@" &
sim_pid=$!

# ros_gz_sim starts Gazebo through a Ruby/sh wrapper. Its server and GUI create
# their own process groups, but stay in the isolated launch session.
cleanup() {
  trap - EXIT INT TERM
  pkill -TERM -s "$sim_pid" 2>/dev/null || true
  wait "$sim_pid" 2>/dev/null || true
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
wait "$sim_pid"
