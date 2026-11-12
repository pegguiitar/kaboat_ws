#!/usr/bin/env bash
# Standard ROS messages only. Publishes visual markers and a static TF, no commands.
set -euo pipefail
HERE=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
ROOT=$(cd "$HERE/../../.." && pwd)
REPLAY=${1:-$ROOT/artifacts/gate_wall_review/preview.json.gz}
if [[ $# -gt 0 ]]; then shift; fi
if ! command -v rviz2 >/dev/null; then
  echo 'ROS 환경을 먼저 불러오세요: source /opt/ros/humble/setup.bash' >&2
  exit 1
fi
if [[ ! -f "$REPLAY" ]]; then
  echo "재생 결과가 없습니다: $REPLAY" >&2
  exit 1
fi
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-97}
export ROS_LOCALHOST_ONLY=1
python3 "$HERE/preview_rviz.py" "$REPLAY" --trial 4 --loop "$@" &
PREVIEW_PID=$!
trap 'kill "$PREVIEW_PID" 2>/dev/null || true' EXIT
rviz2 -d "$HERE/walls.rviz"
