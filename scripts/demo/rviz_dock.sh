#!/usr/bin/env bash
set -eo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/_env.sh"
if [[ -r /sys/class/drm/card1/device/vendor ]] && \
   [[ "$(cat /sys/class/drm/card1/device/vendor)" == '0x8086' ]]; then
  unset LIBGL_ALWAYS_SOFTWARE
  export MESA_LOADER_DRIVER_OVERRIDE="${MESA_LOADER_DRIVER_OVERRIDE:-iris}"
fi
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1
exec rviz2 -d "$WS/scripts/demo/dock.rviz" --ros-args -p use_sim_time:=true "$@"
