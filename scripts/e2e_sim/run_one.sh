#!/usr/bin/env bash
# 한 판: scripts/e2e_sim/run_one.sh <이름> <도메인 ID> <seed> [시나리오.json] [autonomy 런치 추가 인자…]
# 실제 스택 = thruster_driver(dummy) + occupancy_grid(run_map 과 같음) + autonomy.launch.py (run_autonomy 인자).
# HOME 을 판 폴더로 바꿔 ~/kaboat_mission_points.json·~/kaboat_geo_origin.yaml 이 시뮬 것을 가리키게 한다
# (사용자 홈은 건드리지 않음, 런치 인자는 실물과 같게 기본값 그대로). 결과: $KABOAT_E2E_OUT(기본 /tmp/kaboat_e2e_runs)/<이름>/
set -o pipefail
HERE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WS="$(cd -- "$HERE/../.." && pwd)"
OUT="${KABOAT_E2E_OUT:-/tmp/kaboat_e2e_runs}"
NAME=$1; DOMAIN=$2; SEED=$3; SCEN=${4:-}; shift 4 || shift $#
EXTRA=("$@")
RUN=$OUT/$NAME
rm -rf "$RUN"; mkdir -p "$RUN/home"
# 같은 ROS_DOMAIN_ID 를 쓰는 다른 프로세스(다른 시험·세션이 남긴 mission_manager 등)가 있으면 명령이 섞인다
# (2026-11-14: 도메인 86·88 에 남은 노드 때문에 두 판이 무효). 있으면 시작하지 않는다.
BUSY=$(python3 - "$DOMAIN" <<'PY'
import os, sys
want = f'ROS_DOMAIN_ID={sys.argv[1]}'.encode()
for pid in filter(str.isdigit, os.listdir('/proc')):
    try:
        env = open(f'/proc/{pid}/environ', 'rb').read().split(b'\0')
    except OSError:
        continue
    if want in env and int(pid) != os.getppid():
        print(pid, end=' ')
PY
)
if [ -n "$BUSY" ]; then
    echo "✗ ROS_DOMAIN_ID=$DOMAIN 을 이미 쓰는 프로세스: $BUSY — 다른 도메인으로" >&2
    exit 3
fi
source /opt/ros/jazzy/setup.bash
source $WS/install/setup.bash
# install/kaboat_hardware 는 옛 복사본(2026-11-13 00:20 빌드)이라 지금 소스가 먼저 잡히게 한다
export PYTHONPATH=$WS/src/kaboat_hardware:$PYTHONPATH
export ROS_DOMAIN_ID=$DOMAIN ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export HOME=$RUN/home ROS_LOG_DIR=$RUN/roslog PYTHONUNBUFFERED=1
python3 "$HERE/make_home.py" "$HERE/arena.json" "$HOME" "$SCEN"
cd "$RUN"
python3 "$HERE/sim_node.py" --arena "$HERE/arena.json" --nomoto "$HERE/nomoto.json" \
    ${SCEN:+--scenario "$SCEN"} --out "$RUN" --seed "$SEED" > sim.log 2>&1 &
SIM=$!
setsid ros2 run kaboat_hardware thruster_driver --ros-args \
    --params-file $WS/src/kaboat_hardware/config/thrusters.yaml \
    -p hardware_type:=dummy > motor.log 2>&1 &
MOTOR=$!
setsid ros2 run kaboat_perception occupancy_grid --ros-args > map.log 2>&1 &
MAP=$!
sleep 4
setsid ros2 launch kaboat_bringup autonomy.launch.py gate_motion_enabled:=true approach_motion_enabled:=true \
    dock_motion_enabled:=true search_target_color:=red dock_target_class:=circle dock_device:=cpu "${EXTRA[@]}" \
    > launch.log 2>&1 &
LAUNCH=$!
wait $SIM
for p in $LAUNCH $MAP $MOTOR; do kill -INT -- -$p 2>/dev/null; done
sleep 4
for p in $LAUNCH $MAP $MOTOR; do kill -KILL -- -$p 2>/dev/null; done
echo "run $NAME finished"
