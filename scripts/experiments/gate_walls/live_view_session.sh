#!/usr/bin/env bash
# Sensor-only replay + live perception + three path previews. No cmd_mux/motors.
set -e
HERE=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
ROOT=${KABOAT_WS:-$(cd "$HERE/../../.." && pwd)}
BAG=${1:?사용법: bash live_view_session.sh <센서 구간 bag 폴더>}
BAG=$(realpath "$BAG")
if [[ ! -f "$BAG/metadata.yaml" ]]; then
  echo "rosbag metadata.yaml이 없습니다: $BAG" >&2
  exit 1
fi
source /opt/ros/humble/setup.bash
SETUP=${KABOAT_INSTALL_SETUP:-$ROOT/install/setup.bash}
if [[ -f "$SETUP" ]]; then source "$SETUP"; fi
export PYTHONPATH="$ROOT/src/kaboat_perception:$ROOT/src/kaboat_behaviors:$ROOT/src/kaboat_mission:$HERE:${PYTHONPATH:-}"
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-97} ROS_LOCALHOST_ONLY=1
export DISPLAY=${DISPLAY:-:0} QT_X11_NO_MITSHM=1
export LIBGL_ALWAYS_SOFTWARE=${LIBGL_ALWAYS_SOFTWARE:-1}
RUN_DIR=$(mktemp -d /tmp/kaboat-wall-live.XXXXXX)
PIDS=()
cleanup() {
  for pid in "${PIDS[@]}"; do kill -INT "$pid" 2>/dev/null || true; done
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
start() {
  local name=$1
  shift
  "$@" >"$RUN_DIR/$name.log" 2>&1 &
  PIDS+=("$!")
  echo "$!" >"$RUN_DIR/$name.pid"
}
echo "실제 센서 재생 / ROS_DOMAIN_ID=$ROS_DOMAIN_ID / 로그: $RUN_DIR"
start occupancy python3 -m kaboat_perception.occupancy_grid --ros-args \
  -p use_sim_time:=true -p virtual_walls_file:=none
start detector python3 -m kaboat_perception.buoy_lidar_detector --ros-args \
  -p use_sim_time:=true \
  -p color_topic:=/camera/camera/color/image_raw/compressed \
  -p color_compressed:=true
start flat python3 "$ROOT/scripts/flat_scan_view.py" --ros-args -p use_sim_time:=true
start gate python3 -m kaboat_behaviors.gate_follower --ros-args -r __node:=gate_follower \
  --params-file "$ROOT/src/kaboat_bringup/config/gate_params.yaml" \
  -p use_sim_time:=true -p gate.motion_enabled:=false
start walls python3 "$HERE/live_wall_path.py" --ros-args -p use_sim_time:=true
start bag python3 "$HERE/live_sensor_bag.py" play "$BAG" --rate 1
start rviz rviz2 -d "$HERE/live_sensors.rviz" --ros-args -p use_sim_time:=true
# Closing this RViz stops only the child processes started by this session.
wait "${PIDS[-1]}"
