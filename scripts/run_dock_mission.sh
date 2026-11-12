#!/usr/bin/env bash
# 야외 도킹 시험 (outdoor_dock_test.launch.py) — YOLO 가 젯슨 CUDA venv 를 쓰게 환경을 맞춰 켠다.
# 단축어: test_dock_outdoor [target_class:=circle] [motion_enabled:=false] …
# 환경은 run_dock_yolo.sh 와 같다 (JetPack 6 / CUDA 12.6, KABOAT_YOLO_ENV 로 venv 변경).
# 카메라는 run_camera 로 따로 켠다. dock_yolo(run_dock_yolo.sh)와 같은 잠금을 써서 검출기가 둘 뜨지 않게 한다.
# 런치(dock_nodes.py)도 검출기 노드에 가상환경을 붙이지만, 여기서는 시작 전에 CUDA 를 확인하고
# 잠금을 잡는다 (DOCKING_YOLO.md "젯슨 YOLO 가상환경").
set -eo pipefail
exec 9>/tmp/kaboat_dock_yolo.lock
if ! flock -n 9; then
    echo 'D455/YOLO 검출기가 이미 돌고 있음 (run_dock_yolo.sh 또는 다른 test_dock_outdoor). 그쪽을 먼저 끌 것.' >&2
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
exec ros2 launch kaboat_hardware outdoor_dock_test.launch.py "$@"
