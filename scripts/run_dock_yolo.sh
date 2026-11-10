#!/usr/bin/env bash
# GPU runtime prepared for JetPack 6 / CUDA 12.6. Override with KABOAT_YOLO_ENV.
set -eo pipefail
# One camera/detector owner; the lock is released when the launch exits.
exec 9>/tmp/kaboat_dock_yolo.lock
if ! flock -n 9; then
    echo 'D455/YOLO is already running. Use the existing terminal.' >&2
    exit 1
fi
repo_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
yolo_env="${KABOAT_YOLO_ENV:-$repo_dir/.venv-dock-yolo}"
source /opt/ros/humble/setup.bash
source "$repo_dir/install/setup.bash"
export PYTHONPATH="$yolo_env/lib/python3.10/site-packages:${PYTHONPATH:-}"
export LD_LIBRARY_PATH="$yolo_env/lib/python3.10/site-packages/nvidia/cusparselt/lib:${LD_LIBRARY_PATH:-}"
export YOLO_CONFIG_DIR="${YOLO_CONFIG_DIR:-/tmp/kaboat_yolo_config}"
mkdir -p "$YOLO_CONFIG_DIR"
"$yolo_env/bin/python" -c 'import torch; assert torch.cuda.is_available(), "CUDA PyTorch required; CPU-only build detected"'
exec ros2 launch kaboat_bringup dock_yolo.launch.py "$@"
